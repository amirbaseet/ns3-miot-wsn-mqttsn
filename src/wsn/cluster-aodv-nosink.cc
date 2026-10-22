/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * ns3-miot-simulation — Cluster-Based MIoT Comparison
 *
 * File:         cluster-aodv-nosink.cc
 * Description:  Traditional WSN — cluster-based AODV + UDP + UdpForwarder + --hetero.
 *
 * Authors:      Amro Baseet, İsmail Bütün
 * Institution:  Sakarya University, Türkiye
 * Companion code for: Performance Evaluation of Cluster-Based WSN-AODV
 *               and MQTT-SN Architectures for Medical IoT Using ns-3,
 *               A. Baseet and İ. Bütün.
 * Repository:   https://github.com/amirbaseet/ns3-miot-simulation
 * Requires:     ns-3 v3.40 (v3.41+ has a known PDR=0% regression)
 * License:      MIT (see LICENSE)
 */
/*
 * Traditional WSN: Cluster-Based AODV + UDP (Scalability Version)
 * ================================================================
 * Supports command-line node count:
 *   ./ns3 run "cluster-aodv-nosink --numRegular=50 --numCH=5"
 *   ./ns3 run "cluster-aodv-nosink --numRegular=200 --numCH=20"
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/energy-module.h"
#include "ns3/wifi-radio-energy-model-helper.h"

#include <cmath>
#include <vector>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <limits>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("ClusterAodvNoSink");

static const double   AREA       = 1000.0;
static const double   SIM_TIME   = 300.0;
static const uint32_t PKT_SIZE   = 512;
static const std::string DATA_RATE = "4kbps";
static const uint16_t APP_PORT   = 9;
static const uint16_t SINK_PORT  = 10;
static const double   START_MIN  = 30.0;   // Warm-up: 0-30s for AODV convergence
static const double   START_MAX  = 45.0;   // All sensors start within 30-45s
static const double   STOP_TIME  = 290.0;  // Stop 10s before end

// Dynamic (sized at runtime)
std::vector<uint32_t> clusterAssign;
std::vector<uint32_t> clusterSizes;

// ============================================================
// UdpForwarder: Receives packets and forwards them to sink
// This is REAL forwarding — CH only sends to sink what it receives
// ============================================================
class UdpForwarder : public Application
{
public:
  static TypeId GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::UdpForwarder")
      .SetParent<Application> ()
      .AddConstructor<UdpForwarder> ();
    return tid;
  }

  UdpForwarder () : m_rxSocket (nullptr), m_txSocket (nullptr),
    m_rxPort (0), m_rxCount (0), m_fwdCount (0), m_fwdEnabled (false) {}

  virtual ~UdpForwarder () {}

  void Setup (uint16_t rxPort, Ipv4Address sinkAddr, uint16_t sinkPort)
  {
    m_rxPort = rxPort;
    m_sinkAddr = sinkAddr;
    m_sinkPort = sinkPort;
    m_fwdEnabled = true;
  }

  void SetupNoForward (uint16_t rxPort)
  {
    m_rxPort = rxPort;
    m_fwdEnabled = false;
  }

  uint32_t GetRxCount () const { return m_rxCount; }
  uint32_t GetFwdCount () const { return m_fwdCount; }

private:
  virtual void StartApplication (void)
  {
    // Receive socket
    m_rxSocket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
    m_rxSocket->Bind (InetSocketAddress (Ipv4Address::GetAny (), m_rxPort));
    m_rxSocket->SetRecvCallback (MakeCallback (&UdpForwarder::HandleRead, this));

    // Forward socket (if enabled)
    if (m_fwdEnabled)
      {
        m_txSocket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
        m_txSocket->Bind ();
      }
  }

  virtual void StopApplication (void)
  {
    if (m_rxSocket) { m_rxSocket->Close (); }
    if (m_txSocket) { m_txSocket->Close (); }
  }

  void HandleRead (Ptr<Socket> socket)
  {
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom (from)))
      {
        m_rxCount++;

        // Forward to sink (real forwarding — same packet)
        if (m_fwdEnabled && m_txSocket)
          {
            Ptr<Packet> fwdPkt = packet->Copy ();
            m_txSocket->SendTo (fwdPkt, 0,
              InetSocketAddress (m_sinkAddr, m_sinkPort));
            m_fwdCount++;
          }
      }
  }

  Ptr<Socket>  m_rxSocket;
  Ptr<Socket>  m_txSocket;
  uint16_t     m_rxPort;
  Ipv4Address  m_sinkAddr;
  uint16_t     m_sinkPort;
  uint32_t     m_rxCount;
  uint32_t     m_fwdCount;
  bool         m_fwdEnabled;
};

NS_OBJECT_ENSURE_REGISTERED (UdpForwarder);

static double CalcDist (const Vector &a, const Vector &b)
{ return std::sqrt ((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y)); }

// ============================================================
NetDeviceContainer
ConfigWifi (NodeContainer &all, YansWifiPhyHelper &phy)
{
  WifiHelper w; w.SetStandard (WIFI_STANDARD_80211b);
  w.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
    "DataMode", StringValue ("DsssRate11Mbps"),
    "ControlMode", StringValue ("DsssRate1Mbps"));
  YansWifiChannelHelper ch;
  ch.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  ch.AddPropagationLoss ("ns3::LogDistancePropagationLossModel",
    "Exponent", DoubleValue (2.5), "ReferenceDistance", DoubleValue (1.0),
    "ReferenceLoss", DoubleValue (46.6777));
  phy.SetChannel (ch.Create ());
  phy.Set ("TxPowerStart", DoubleValue (20.0));
  phy.Set ("TxPowerEnd", DoubleValue (20.0));
  WifiMacHelper mac; mac.SetType ("ns3::AdhocWifiMac");
  return w.Install (phy, mac, all);
}

Ipv4InterfaceContainer
InstallStack (NodeContainer &all, NetDeviceContainer &dev)
{
  AodvHelper aodv;
  aodv.Set ("HelloInterval", TimeValue (Seconds (2)));
  aodv.Set ("AllowedHelloLoss", UintegerValue (3));
  aodv.Set ("ActiveRouteTimeout", TimeValue (Seconds (15)));
  aodv.Set ("RreqRetries", UintegerValue (5));
  aodv.Set ("NetDiameter", UintegerValue (20));
  aodv.Set ("NodeTraversalTime", TimeValue (MilliSeconds (100)));
  InternetStackHelper inet; inet.SetRoutingHelper (aodv); inet.Install (all);
  Ipv4AddressHelper addr; addr.SetBase ("10.1.0.0", "255.255.0.0");
  return addr.Assign (dev);
}

void PositionNodes (NodeContainer &sens, NodeContainer &chs, NodeContainer &sinkNode,
                    uint32_t nCH)
{
  MobilityHelper m; m.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  uint32_t cols = (uint32_t)std::ceil (std::sqrt ((double)nCH));
  if (cols < 1) cols = 1;
  uint32_t rows = (nCH + cols - 1) / cols;

  Ptr<ListPositionAllocator> cp = CreateObject<ListPositionAllocator> ();
  double cW = AREA / cols, cH = AREA / rows;
  for (uint32_t i = 0; i < nCH; i++)
    cp->Add (Vector ((i%cols+0.5)*cW, (i/cols+0.5)*cH, 0));
  m.SetPositionAllocator (cp); m.Install (chs);

  uint32_t nSens = sens.GetN ();
  Ptr<ListPositionAllocator> sp = CreateObject<ListPositionAllocator> ();
  Ptr<UniformRandomVariable> rx = CreateObject<UniformRandomVariable> ();
  rx->SetAttribute ("Min", DoubleValue (0)); rx->SetAttribute ("Max", DoubleValue (AREA));
  Ptr<UniformRandomVariable> ry = CreateObject<UniformRandomVariable> ();
  ry->SetAttribute ("Min", DoubleValue (0)); ry->SetAttribute ("Max", DoubleValue (AREA));
  for (uint32_t i = 0; i < nSens; i++)
    sp->Add (Vector (rx->GetValue (), ry->GetValue (), 0));
  m.SetPositionAllocator (sp); m.Install (sens);

  // Sink at center
  Ptr<ListPositionAllocator> skp = CreateObject<ListPositionAllocator> ();
  skp->Add (Vector (AREA/2.0, AREA/2.0, 0));
  m.SetPositionAllocator (skp); m.Install (sinkNode);
}

void AssignClusters (NodeContainer &sens, NodeContainer &chs, uint32_t nCH)
{
  uint32_t nSens = sens.GetN ();
  clusterAssign.resize (nSens, 0);
  clusterSizes.resize (nCH, 0);

  std::vector<Vector> chP (nCH);
  for (uint32_t i = 0; i < nCH; i++)
    chP[i] = chs.Get (i)->GetObject<MobilityModel> ()->GetPosition ();
  std::fill (clusterSizes.begin (), clusterSizes.end (), 0);

  for (uint32_t n = 0; n < nSens; n++)
    {
      Vector np = sens.Get (n)->GetObject<MobilityModel> ()->GetPosition ();
      double md = 1e9; uint32_t ne = 0;
      for (uint32_t c = 0; c < nCH; c++)
        { double d = CalcDist (np, chP[c]); if (d < md) { md = d; ne = c; } }
      clusterAssign[n] = ne; clusterSizes[ne]++;
    }

  std::cout << "  Clusters: min=" << *std::min_element (clusterSizes.begin (), clusterSizes.end ())
            << " max=" << *std::max_element (clusterSizes.begin (), clusterSizes.end ())
            << " avg=" << std::fixed << std::setprecision (1)
            << (double)nSens/nCH << "\n";
}

void InstallTraffic (NodeContainer &sens, NodeContainer &chs,
                     NodeContainer &sinkNode, Ipv4InterfaceContainer &ifaces,
                     uint32_t nSens, bool useSink, bool hetero)
{
  Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable> ();
  rng->SetAttribute ("Min", DoubleValue (START_MIN));
  rng->SetAttribute ("Max", DoubleValue (START_MAX));

  uint32_t nCH = chs.GetN ();
  uint32_t sinkIfIdx = nSens + nCH;
  uint32_t nECG = nSens / 3;
  uint32_t nHR  = nSens / 3;

  // Sensor -> CH traffic (OnOff UDP)
  for (uint32_t n = 0; n < nSens; n++)
    {
      uint32_t chIf = nSens + clusterAssign[n];

      std::string rate;
      uint32_t pktSize;

      if (hetero)
        {
          // Match MQTT-SN traffic profile: ECG=4kbps/128B, HR=0.5kbps/64B, Temp=0.05kbps/32B
          if (n < nECG) { rate = "4kbps"; pktSize = 128; }
          else if (n < nECG + nHR) { rate = "512bps"; pktSize = 64; }
          else { rate = "51bps"; pktSize = 32; }
        }
      else
        {
          // Original constant rate
          rate = DATA_RATE;
          pktSize = PKT_SIZE;
        }

      OnOffHelper onoff ("ns3::UdpSocketFactory",
        InetSocketAddress (ifaces.GetAddress (chIf), APP_PORT));
      onoff.SetAttribute ("DataRate", StringValue (rate));
      onoff.SetAttribute ("PacketSize", UintegerValue (pktSize));
      onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
      onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
      ApplicationContainer app = onoff.Install (sens.Get (n));
      app.Start (Seconds (rng->GetValue ()));
      app.Stop (Seconds (STOP_TIME));
    }

  // CH: UdpForwarder
  for (uint32_t c = 0; c < nCH; c++)
    {
      Ptr<UdpForwarder> fwd = CreateObject<UdpForwarder> ();
      if (useSink)
        fwd->Setup (APP_PORT, ifaces.GetAddress (sinkIfIdx), SINK_PORT);
      else
        fwd->SetupNoForward (APP_PORT);
      chs.Get (c)->AddApplication (fwd);
      fwd->SetStartTime (Seconds (0.0));
      fwd->SetStopTime (Seconds (SIM_TIME));
    }

  // Sink
  if (useSink)
    {
      PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory",
        InetSocketAddress (Ipv4Address::GetAny (), SINK_PORT));
      sinkHelper.Install (sinkNode.Get (0)).Start (Seconds (0.0));
    }

  if (hetero)
    std::cout << "  Traffic: HETEROGENEOUS (" << nECG << " ECG@4kbps + "
              << nHR << " HR@512bps + " << (nSens-nECG-nHR) << " Temp@51bps)\n";
  else
    std::cout << "  Traffic: CONSTANT (" << nSens << " x " << DATA_RATE << ", " << PKT_SIZE << "B)\n";

  if (useSink)
    std::cout << "  Flow: Sensors -> " << nCH << " CHs (forward) -> 1 Sink\n\n";
  else
    std::cout << "  Flow: Sensors -> " << nCH << " CHs (receive only)\n\n";
}

void PrintResults (Ptr<FlowMonitor> fm, FlowMonitorHelper &fh,
                   std::string csvName)
{
  fm->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> cls = DynamicCast<Ipv4FlowClassifier> (fh.GetClassifier ());
  auto stats = fm->GetFlowStats ();

  uint64_t tTx = 0, tRx = 0, tRxB = 0;
  double tDel = 0; uint32_t fl = 0, dead = 0;

  std::ofstream csv (csvName);
  csv << "FlowID,SrcAddr,DstAddr,TxPackets,RxPackets,LostPackets,"
      << "PDR_pct,Throughput_kbps,AvgDelay_ms,AvgJitter_ms\n";

  for (auto &kv : stats)
    {
      auto ft = cls->FindFlow (kv.first); auto &s = kv.second;
      double pdr = s.txPackets > 0 ? (double)s.rxPackets/s.txPackets*100 : 0;
      double dur = s.rxPackets > 0 ? (s.timeLastRxPacket-s.timeFirstTxPacket).GetSeconds () : 0;
      double tp = dur > 0 ? s.rxBytes*8.0/dur/1000 : 0;
      double dl = s.rxPackets > 0 ? s.delaySum.GetMilliSeconds ()/(double)s.rxPackets : 0;
      double jt = s.rxPackets > 1 ? s.jitterSum.GetMilliSeconds ()/(double)(s.rxPackets-1) : 0;
      tTx += s.txPackets; tRx += s.rxPackets; tRxB += s.rxBytes;
      tDel += s.rxPackets > 0 ? s.delaySum.GetMilliSeconds () : 0;
      fl++; if (s.rxPackets == 0) dead++;
      csv << kv.first << "," << ft.sourceAddress << "," << ft.destinationAddress
          << "," << s.txPackets << "," << s.rxPackets << "," << s.lostPackets
          << "," << std::fixed << std::setprecision (2)
          << pdr << "," << tp << "," << dl << "," << jt << "\n";
    }
  csv.close ();

  double pdr = tTx > 0 ? (double)tRx/tTx*100 : 0;
  double avgD = tRx > 0 ? tDel/tRx : 0;

  std::cout << "============================================================\n"
    << "  WSN RESULTS\n"
    << "============================================================\n"
    << "  Flows: " << fl << " (dead: " << dead << ")\n"
    << "  Tx: " << tTx << " | Rx: " << tRx << "\n"
    << "  PDR: " << std::fixed << std::setprecision (2) << pdr << "%\n"
    << "  Avg Delay: " << avgD << " ms\n"
    << "  Throughput: " << (tRxB*8.0/SIM_TIME/1000) << " kbps\n"
    << "  Output: " << csvName << "\n"
    << "============================================================\n\n";

  fm->SerializeToXmlFile (csvName + ".flowmon.xml", true, true);
}

// ============================================================
int main (int argc, char *argv[])
{
  uint32_t numRegular = 180;
  uint32_t numCH      = 20;
  uint32_t runNum     = 1;
  double   simTime    = 300.0;
  bool     useSink    = false;
  bool     hetero     = false;  // Heterogeneous traffic (match MQTT-SN load)
  bool     enableAnim = false;
  bool     verbose    = false;
  std::string csvName = "";

  CommandLine cmd (__FILE__);
  cmd.AddValue ("numRegular", "Number of regular sensor nodes", numRegular);
  cmd.AddValue ("numCH", "Number of cluster heads", numCH);
  cmd.AddValue ("run", "Run number for seed variation", runNum);
  cmd.AddValue ("simTime", "Simulation time (seconds)", simTime);
  cmd.AddValue ("useSink", "Enable sink/broker node", useSink);
  cmd.AddValue ("hetero", "Heterogeneous traffic (match MQTT-SN rates)", hetero);
  cmd.AddValue ("anim", "Enable NetAnim output", enableAnim);
  cmd.AddValue ("verbose", "Enable verbose logging", verbose);
  cmd.AddValue ("csv", "Output CSV filename", csvName);
  cmd.Parse (argc, argv);

  if (verbose)
    LogComponentEnable ("ClusterAodvNoSink", LOG_LEVEL_INFO);

  if (csvName.empty ())
    {
      std::ostringstream oss;
      oss << "wsn-" << numRegular << "nodes";
      if (useSink) oss << "-sink";
      oss << "-results.csv";
      csvName = oss.str ();
    }

  SeedManager::SetSeed (42);
  SeedManager::SetRun (runNum);

  uint32_t totalNodes = numRegular + numCH + 1; // +1 sink always created

  std::cout << "\n============================================================\n"
    << "  Traditional WSN (AODV + UDP)\n"
    << "  Sensors: " << numRegular << " | CHs: " << numCH
    << " | Sink: " << (useSink ? "yes" : "no")
    << " | Total: " << totalNodes << "\n"
    << "  Area: " << AREA << "x" << AREA << "m | AODV | " << simTime << "s\n"
    << "  CSV: " << csvName << "\n"
    << "============================================================\n\n";

  NodeContainer sens, chs, sinkNode, all;
  sens.Create (numRegular);
  chs.Create (numCH);
  sinkNode.Create (1);
  all.Add (sens); all.Add (chs); all.Add (sinkNode);

  YansWifiPhyHelper phy;
  NetDeviceContainer dev = ConfigWifi (all, phy);
  Ipv4InterfaceContainer ifaces = InstallStack (all, dev);

  PositionNodes (sens, chs, sinkNode, numCH);
  AssignClusters (sens, chs, numCH);
  InstallTraffic (sens, chs, sinkNode, ifaces, numRegular, useSink, hetero);

  // ---- Energy Model Setup ----
  BasicEnergySourceHelper energySourceHelper;
  // BasicEnergySourceHelper uses BasicEnergySource by default
  energySourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (2.0));
  EnergySourceContainer sensSources;
  for (uint32_t i = 0; i < numRegular; i++)
    {
      NodeContainer tmp;
      tmp.Add (sens.Get (i));
      sensSources.Add (energySourceHelper.Install (tmp));
    }

  BasicEnergySourceHelper chEnergyHelper;
  // BasicEnergySourceHelper uses BasicEnergySource by default
  chEnergyHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (5.0));
  EnergySourceContainer chSources;
  for (uint32_t i = 0; i < numCH; i++)
    {
      NodeContainer tmp;
      tmp.Add (chs.Get (i));
      chSources.Add (chEnergyHelper.Install (tmp));
    }

  WifiRadioEnergyModelHelper radioEnergyHelper;
  radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.346));
  radioEnergyHelper.Set ("RxCurrentA", DoubleValue (0.285));
  radioEnergyHelper.Set ("IdleCurrentA", DoubleValue (0.248));
  radioEnergyHelper.Set ("SleepCurrentA", DoubleValue (0.030));
  for (uint32_t i = 0; i < numRegular; i++)
    {
      NetDeviceContainer d; d.Add (dev.Get (i));
      radioEnergyHelper.Install (d, sensSources.Get (i));
    }
  for (uint32_t i = 0; i < numCH; i++)
    {
      NetDeviceContainer d; d.Add (dev.Get (numRegular + i));
      radioEnergyHelper.Install (d, chSources.Get (i));
    }
  std::cout << "  Energy: Sensors=2.0J, CHs=5.0J\n\n";

  FlowMonitorHelper fh;
  Ptr<FlowMonitor> fm = fh.InstallAll ();

  std::cout << "[RUN] " << simTime << "s...\n";
  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  PrintResults (fm, fh, csvName);

  // ---- Energy Report ----
  std::cout << "  ============================================\n"
            << "  ENERGY REPORT\n"
            << "  ============================================\n";
  double totalSensInit = numRegular * 2.0;
  double totalCHInit   = numCH * 5.0;
  double totalSensRemain = 0, totalCHRemain = 0;
  uint32_t deadSensors = 0, deadCHs = 0;
  double minSE = 999, maxSE = 0, sumSE = 0;
  for (uint32_t i = 0; i < numRegular; i++)
    {
      double r = sensSources.Get (i)->GetRemainingEnergy ();
      totalSensRemain += r; sumSE += r;
      if (r < minSE) minSE = r; if (r > maxSE) maxSE = r;
      if (r <= 0.0) deadSensors++;
    }
  double minCE = 999, maxCE = 0, sumCE = 0;
  for (uint32_t i = 0; i < numCH; i++)
    {
      double r = chSources.Get (i)->GetRemainingEnergy ();
      totalCHRemain += r; sumCE += r;
      if (r < minCE) minCE = r; if (r > maxCE) maxCE = r;
      if (r <= 0.0) deadCHs++;
    }
  double sensConsumed = totalSensInit - totalSensRemain;
  double chConsumed   = totalCHInit   - totalCHRemain;
  std::cout << std::fixed << std::setprecision (4)
            << "  Sensor Energy:\n"
            << "    Initial: "   << totalSensInit   << " J (" << numRegular << " x 2.0 J)\n"
            << "    Remaining: " << totalSensRemain << " J\n"
            << "    Consumed: "  << sensConsumed    << " J ("
            << std::setprecision (1) << (sensConsumed/totalSensInit*100) << "%)\n"
            << std::setprecision (4)
            << "    Per sensor: avg=" << (numRegular>0?sumSE/numRegular:0)
            << "J  min=" << minSE << "J  max=" << maxSE << "J\n"
            << "    Dead sensors (0 energy): " << deadSensors << "\n"
            << "  CH Energy:\n"
            << "    Initial: "   << totalCHInit   << " J (" << numCH << " x 5.0 J)\n"
            << "    Remaining: " << totalCHRemain << " J\n"
            << "    Consumed: "  << chConsumed    << " J ("
            << std::setprecision (1) << (chConsumed/totalCHInit*100) << "%)\n"
            << std::setprecision (4)
            << "    Per CH: avg=" << (numCH>0?sumCE/numCH:0)
            << "J  min=" << minCE << "J  max=" << maxCE << "J\n"
            << "    Dead CHs (0 energy): " << deadCHs << "\n"
            << "  ============================================\n\n";
  std::string energyCsv = csvName + ".energy.csv";
  std::ofstream ecsv (energyCsv);
  ecsv << "NodeID,Type,InitialEnergy_J,RemainingEnergy_J,ConsumedEnergy_J,ConsumedPct\n";
  uint32_t nECG_e = numRegular/3, nHR_e = numRegular/3;
  for (uint32_t i = 0; i < numRegular; i++)
    {
      double rem = sensSources.Get (i)->GetRemainingEnergy ();
      double cons = 2.0 - rem;
      std::string t = (i < nECG_e) ? "ECG" : (i < nECG_e+nHR_e) ? "HR" : "Temp";
      ecsv << i << "," << t << ",2.0," << std::fixed << std::setprecision (4)
           << rem << "," << cons << "," << std::setprecision (2) << (cons/2.0*100) << "\n";
    }
  for (uint32_t i = 0; i < numCH; i++)
    {
      double rem = chSources.Get (i)->GetRemainingEnergy ();
      double cons = 5.0 - rem;
      ecsv << (numRegular+i) << ",CH,5.0," << std::fixed << std::setprecision (4)
           << rem << "," << cons << "," << std::setprecision (2) << (cons/5.0*100) << "\n";
    }
  ecsv.close ();
  std::cout << "  Energy CSV: " << energyCsv << "\n\n";

  // Print CH forwarding stats
  if (useSink)
    {
      std::cout << "  Per-CH Forwarding Stats:\n";
      uint32_t totalRx = 0, totalFwd = 0;
      for (uint32_t c = 0; c < numCH; c++)
        {
          Ptr<UdpForwarder> fwd = DynamicCast<UdpForwarder> (chs.Get (c)->GetApplication (0));
          if (fwd)
            {
              std::cout << "    CH" << (numRegular + c) << " | Rx:" << fwd->GetRxCount ()
                        << " Fwd:" << fwd->GetFwdCount () << "\n";
              totalRx += fwd->GetRxCount ();
              totalFwd += fwd->GetFwdCount ();
            }
        }
      std::cout << "  Total: Rx=" << totalRx << " Fwd=" << totalFwd << "\n\n";
    }

  Simulator::Destroy ();
  std::cout << "[DONE]\n\n";
  return 0;
}
