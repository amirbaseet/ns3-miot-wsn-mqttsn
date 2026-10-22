/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * ns3-miot-simulation — Cluster-Based MIoT Comparison
 *
 * File:         cluster-aodv-mqtt.cc
 * Description:  Main simulation driver for MQTT-SN publish-subscribe architecture.
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
 * Priority-Aware MQTT-SN for MIoT — Multi-Broker Hierarchy Extension
 * ====================================================================
 * Topology modes (--numLocalBrokers):
 *   0  = no broker           (original Set 1 / Set 3)
 *   1  = single root broker  (original Set 2 / Set 3b — baseline)
 *  >1  = two-tier hierarchy  (new: N local tier-1 brokers + 1 root broker)
 *
 * Usage examples:
 *   ./ns3 run "cluster-aodv-mqtt --nSensors=200 --nCH=20 --numLocalBrokers=0"
 *   ./ns3 run "cluster-aodv-mqtt --nSensors=200 --nCH=20 --numLocalBrokers=1"
 *   ./ns3 run "cluster-aodv-mqtt --nSensors=200 --nCH=20 --numLocalBrokers=4"
 *
 * NOTE: --broker is a legacy flag (maps to numLocalBrokers=0 or 1).
 *       Do NOT combine --broker with --numLocalBrokers in the same command.
 */

#include "mqtt-sn-header.h"
#include "mqtt-sn-publisher.h"
#include "mqtt-sn-gateway.h"
#include "mqtt-sn-broker.h"

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/energy-module.h"
#include "ns3/wifi-radio-energy-model-helper.h"

#include <cmath>
#include <vector>
#include <set>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("ClusterAodvMqtt");

static const double   AREA          = 1000.0;
static const uint16_t MQTT_PORT     = 1883;
static const uint16_t BROKER_PORT   = 1884;  // tier-1 local brokers (and single broker)
static const uint16_t ROOT_PORT     = 1885;  // tier-2 root broker
static const double   EMERGENCY_PROB = 0.005;

std::vector<uint32_t> cAssign;
std::vector<uint32_t> cSizes;

static double CalcDist (const Vector &a, const Vector &b)
{ return std::sqrt ((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y)); }

// ============================================================
NetDeviceContainer ConfigWifi (NodeContainer &all, YansWifiPhyHelper &phy)
{
  WifiHelper w; w.SetStandard (WIFI_STANDARD_80211b);
  w.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
    "DataMode",    StringValue ("DsssRate11Mbps"),
    "ControlMode", StringValue ("DsssRate1Mbps"));
  YansWifiChannelHelper ch;
  ch.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  ch.AddPropagationLoss  ("ns3::LogDistancePropagationLossModel",
    "Exponent",          DoubleValue (2.5),
    "ReferenceDistance", DoubleValue (1.0),
    "ReferenceLoss",     DoubleValue (46.6777));
  phy.SetChannel (ch.Create ());
  phy.Set ("TxPowerStart", DoubleValue (20.0));
  phy.Set ("TxPowerEnd",   DoubleValue (20.0));
  WifiMacHelper mac; mac.SetType ("ns3::AdhocWifiMac");
  return w.Install (phy, mac, all);
}

Ipv4InterfaceContainer InstallStack (NodeContainer &all, NetDeviceContainer &dev)
{
  AodvHelper aodv;
  aodv.Set ("HelloInterval",      TimeValue    (Seconds (2)));
  aodv.Set ("AllowedHelloLoss",   UintegerValue (3));
  aodv.Set ("ActiveRouteTimeout", TimeValue    (Seconds (15)));
  aodv.Set ("RreqRetries",        UintegerValue (5));
  aodv.Set ("NetDiameter",        UintegerValue (20));
  aodv.Set ("NodeTraversalTime",  TimeValue    (MilliSeconds (100)));
  InternetStackHelper inet; inet.SetRoutingHelper (aodv); inet.Install (all);
  Ipv4AddressHelper addr; addr.SetBase ("10.1.0.0", "255.255.0.0");
  return addr.Assign (dev);
}

// ============================================================
void PositionNodes (NodeContainer &sens, NodeContainer &chs,
                    NodeContainer &localBrokers, NodeContainer &rootSink,
                    uint32_t nCH, uint32_t nLBnodes,
                    bool mobility, double speed)
{
  // CHs: static grid
  MobilityHelper chMob;
  chMob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  uint32_t cols = (uint32_t)std::ceil (std::sqrt ((double)nCH));
  if (cols < 1) cols = 1;
  uint32_t rows = (nCH + cols - 1) / cols;
  Ptr<ListPositionAllocator> cp = CreateObject<ListPositionAllocator> ();
  double cW = AREA / cols, cH = AREA / rows;
  for (uint32_t i = 0; i < nCH; i++)
    cp->Add (Vector ((i%cols+0.5)*cW, (i/cols+0.5)*cH, 0));
  chMob.SetPositionAllocator (cp); chMob.Install (chs);

  // Sensors: static or mobile
  uint32_t nSens = sens.GetN ();
  if (mobility)
    {
      MobilityHelper sensMob;
      Ptr<RandomRectanglePositionAllocator> wpAlloc =
        CreateObject<RandomRectanglePositionAllocator> ();
      Ptr<UniformRandomVariable> xVar = CreateObject<UniformRandomVariable> ();
      xVar->SetAttribute ("Min", DoubleValue (0.0));
      xVar->SetAttribute ("Max", DoubleValue (AREA));
      Ptr<UniformRandomVariable> yVar = CreateObject<UniformRandomVariable> ();
      yVar->SetAttribute ("Min", DoubleValue (0.0));
      yVar->SetAttribute ("Max", DoubleValue (AREA));
      wpAlloc->SetX (xVar); wpAlloc->SetY (yVar);
      sensMob.SetPositionAllocator (
        "ns3::RandomRectanglePositionAllocator",
        "X", StringValue ("ns3::UniformRandomVariable[Min=0|Max=" + std::to_string((int)AREA) + "]"),
        "Y", StringValue ("ns3::UniformRandomVariable[Min=0|Max=" + std::to_string((int)AREA) + "]"));
      std::string speedStr = "ns3::ConstantRandomVariable[Constant=" + std::to_string(speed) + "]";
      sensMob.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
        "Speed",             StringValue (speedStr),
        "Pause",             StringValue ("ns3::ConstantRandomVariable[Constant=2.0]"),
        "PositionAllocator", PointerValue (wpAlloc));
      sensMob.Install (sens);
      std::cout << "  Mobility: RandomWaypoint (speed=" << speed << " m/s, pause=2s)\n";
    }
  else
    {
      MobilityHelper sensMob;
      sensMob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
      Ptr<ListPositionAllocator> sp = CreateObject<ListPositionAllocator> ();
      Ptr<UniformRandomVariable> rx = CreateObject<UniformRandomVariable> ();
      rx->SetAttribute ("Min", DoubleValue (0)); rx->SetAttribute ("Max", DoubleValue (AREA));
      Ptr<UniformRandomVariable> ry = CreateObject<UniformRandomVariable> ();
      ry->SetAttribute ("Min", DoubleValue (0)); ry->SetAttribute ("Max", DoubleValue (AREA));
      for (uint32_t i = 0; i < nSens; i++)
        sp->Add (Vector (rx->GetValue (), ry->GetValue (), 0));
      sensMob.SetPositionAllocator (sp); sensMob.Install (sens);
      std::cout << "  Mobility: Static\n";
    }

  // Local brokers: grid placement (only if they exist)
  if (nLBnodes > 0)
    {
      MobilityHelper lbMob;
      lbMob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
      uint32_t lbCols = (uint32_t)std::ceil (std::sqrt ((double)nLBnodes));
      if (lbCols < 1) lbCols = 1;
      Ptr<ListPositionAllocator> lbp = CreateObject<ListPositionAllocator> ();
      double lW = AREA / lbCols;
      uint32_t lbRows = (nLBnodes + lbCols - 1) / lbCols;
      double lH = AREA / lbRows;
      for (uint32_t i = 0; i < nLBnodes; i++)
        lbp->Add (Vector ((i%lbCols+0.5)*lW, (i/lbCols+0.5)*lH, 0));
      lbMob.SetPositionAllocator (lbp); lbMob.Install (localBrokers);
      std::cout << "  Local Brokers: " << nLBnodes << " (grid placement)\n";
    }

  // Root sink: always at center
  MobilityHelper sinkMob;
  sinkMob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  Ptr<ListPositionAllocator> skp = CreateObject<ListPositionAllocator> ();
  skp->Add (Vector (AREA/2.0, AREA/2.0, 0));
  sinkMob.SetPositionAllocator (skp); sinkMob.Install (rootSink);
}

// ============================================================
void AssignClusters (NodeContainer &sens, NodeContainer &chs, uint32_t nCH)
{
  uint32_t nSens = sens.GetN ();
  cAssign.resize (nSens, 0);
  cSizes.resize  (nCH,   0);
  std::vector<Vector> chP (nCH);
  for (uint32_t i = 0; i < nCH; i++)
    chP[i] = chs.Get (i)->GetObject<MobilityModel> ()->GetPosition ();
  std::fill (cSizes.begin (), cSizes.end (), 0);
  for (uint32_t n = 0; n < nSens; n++)
    {
      Vector np = sens.Get (n)->GetObject<MobilityModel> ()->GetPosition ();
      double md = 1e9; uint32_t ne = 0;
      for (uint32_t c = 0; c < nCH; c++)
        { double d = CalcDist (np, chP[c]); if (d < md) { md = d; ne = c; } }
      cAssign[n] = ne; cSizes[ne]++;
    }
  std::cout << "  Clusters: min=" << *std::min_element (cSizes.begin (), cSizes.end ())
            << " max=" << *std::max_element (cSizes.begin (), cSizes.end ())
            << " avg=" << std::fixed << std::setprecision (1)
            << (double)nSens/nCH << "\n";
}

// ============================================================
//  Assign each CH to its nearest local broker
// ============================================================
std::vector<uint32_t> AssignCHsToLocalBrokers (NodeContainer &chs,
                                                NodeContainer &localBrokers,
                                                uint32_t nCH,
                                                uint32_t nLB)
{
  std::vector<uint32_t> chToBroker (nCH, 0);
  std::vector<Vector> lbPos (nLB);
  for (uint32_t i = 0; i < nLB; i++)
    lbPos[i] = localBrokers.Get (i)->GetObject<MobilityModel> ()->GetPosition ();
  for (uint32_t c = 0; c < nCH; c++)
    {
      Vector cp = chs.Get (c)->GetObject<MobilityModel> ()->GetPosition ();
      double md = 1e9; uint32_t best = 0;
      for (uint32_t b = 0; b < nLB; b++)
        { double d = CalcDist (cp, lbPos[b]); if (d < md) { md = d; best = b; } }
      chToBroker[c] = best;
    }
  return chToBroker;
}

// ============================================================
void InstallMqtt (NodeContainer &sens, NodeContainer &chs,
                  NodeContainer &localBrokers, NodeContainer &rootSink,
                  Ipv4InterfaceContainer &ifaces,
                  uint32_t nSens, uint32_t nCH, uint32_t numLocalBrokers,
                  uint32_t nECGin, uint32_t nHRin, double simTime)
{
  // ── ifaces index layout ──────────────────────────────────────
  // [0 .. nSens-1]                   sensors
  // [nSens .. nSens+nCH-1]           cluster heads
  // [nSens+nCH .. nSens+nCH+nLB-1]  local brokers  (if numLocalBrokers > 0)
  // [nSens+nCH+nLB]                  root sink      (always last)
  // ─────────────────────────────────────────────────────────────
  uint32_t nLB       = (numLocalBrokers > 1) ? numLocalBrokers : 0;
  uint32_t nLBnodes  = (numLocalBrokers == 1) ? 1 : nLB;
  uint32_t rootIfIdx = nSens + nCH + nLBnodes;
  Ipv4Address rootAddr = ifaces.GetAddress (rootIfIdx);  // always rootSink

  // ── MODE 0: no broker ─────────────────────────────────────────
  if (numLocalBrokers == 0)
    {
      std::cout << "  Topology: NO BROKER\n";
      for (uint32_t c = 0; c < nCH; c++)
        {
          Ptr<MqttSnGateway> gw = CreateObject<MqttSnGateway> ();
          gw->Setup (MQTT_PORT);
          chs.Get (c)->AddApplication (gw);
          gw->SetStartTime (Seconds (0));
          gw->SetStopTime  (Seconds (simTime));
        }
    }

  // ── MODE 1: single broker (baseline) ──────────────────────────
  else if (numLocalBrokers == 1)
    {
      std::cout << "  Topology: SINGLE BROKER (baseline)\n";
      Ipv4Address singleBrokerAddr = ifaces.GetAddress (nSens + nCH);

      Ptr<MqttSnBroker> broker = CreateObject<MqttSnBroker> ();
      broker->Setup (BROKER_PORT);
      // No SetRootBroker — acts as final sink, no forwarding
      localBrokers.Get (0)->AddApplication (broker);
      broker->SetStartTime (Seconds (0));
      broker->SetStopTime  (Seconds (simTime));

      for (uint32_t c = 0; c < nCH; c++)
        {
          Ptr<MqttSnGateway> gw = CreateObject<MqttSnGateway> ();
          gw->Setup (MQTT_PORT);
          gw->SetBroker (singleBrokerAddr, BROKER_PORT);
          chs.Get (c)->AddApplication (gw);
          gw->SetStartTime (Seconds (0));
          gw->SetStopTime  (Seconds (simTime));
        }
    }

  // ── MODE 2+: two-tier hierarchy ───────────────────────────────
  else
    {
      std::cout << "  Topology: TWO-TIER HIERARCHY ("
                << nLB << " local brokers + 1 root)\n";

      // Tier-2: root broker on rootSink
      Ptr<MqttSnBroker> rootBroker = CreateObject<MqttSnBroker> ();
      rootBroker->Setup (ROOT_PORT);
      rootSink.Get (0)->AddApplication (rootBroker);
      rootBroker->SetStartTime (Seconds (0));
      rootBroker->SetStopTime  (Seconds (simTime));

      // Tier-1: local brokers, each forwarding to root
      for (uint32_t b = 0; b < nLB; b++)
        {
          Ptr<MqttSnBroker> lb = CreateObject<MqttSnBroker> ();
          lb->Setup (BROKER_PORT);
          lb->SetRootBroker (rootAddr, ROOT_PORT);
          localBrokers.Get (b)->AddApplication (lb);
          lb->SetStartTime (Seconds (0));
          lb->SetStopTime  (Seconds (simTime));
        }

      // Assign each CH to nearest local broker and install gateways
      std::vector<uint32_t> chToBroker =
        AssignCHsToLocalBrokers (chs, localBrokers, nCH, nLB);

      std::cout << "  CH-to-LocalBroker assignment:\n";
      for (uint32_t b = 0; b < nLB; b++)
        {
          uint32_t cnt = 0;
          for (uint32_t c = 0; c < nCH; c++) if (chToBroker[c] == b) cnt++;
          std::cout << "    LocalBroker[" << b << "]: " << cnt << " CHs\n";
        }

      for (uint32_t c = 0; c < nCH; c++)
        {
          uint32_t bIdx  = chToBroker[c];
          Ipv4Address lbAddr = ifaces.GetAddress (nSens + nCH + bIdx);
          Ptr<MqttSnGateway> gw = CreateObject<MqttSnGateway> ();
          gw->Setup (MQTT_PORT);
          gw->SetBroker (lbAddr, BROKER_PORT);
          chs.Get (c)->AddApplication (gw);
          gw->SetStartTime (Seconds (0));
          gw->SetStopTime  (Seconds (simTime));
        }
    }

  // ── Publishers on sensors (identical for all topology modes) ──
  Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable> ();
  rng->SetAttribute ("Min", DoubleValue (30.0));
  rng->SetAttribute ("Max", DoubleValue (45.0));

  uint32_t nECG = (nECGin > 0) ? nECGin : nSens / 3;
  uint32_t nHR  = (nHRin  > 0) ? nHRin  : nSens / 3;
  if (nECG + nHR > nSens) { nECG = nSens / 3; nHR = nSens / 3; }

  for (uint32_t n = 0; n < nSens; n++)
    {
      Ipv4Address gwAddr = ifaces.GetAddress (nSens + cAssign[n]);

      uint16_t topic; Time interval; uint32_t payload; uint8_t qos;
      if (n < nECG) {
        topic = TOPIC_ECG;         interval = MilliSeconds (250);
        payload = 128;             qos = PRIORITY_HIGH;
      } else if (n < nECG + nHR) {
        topic = TOPIC_HEART_RATE;  interval = Seconds (1.0);
        payload = 64;              qos = PRIORITY_MEDIUM;
      } else {
        topic = TOPIC_TEMPERATURE; interval = Seconds (5.0);
        payload = 32;              qos = PRIORITY_LOW;
      }

      Ptr<MqttSnPublisher> pub = CreateObject<MqttSnPublisher> ();
      pub->Setup (gwAddr, MQTT_PORT, topic, interval, payload, qos);
      pub->EnableEmergencyDetection (EMERGENCY_PROB);
      sens.Get (n)->AddApplication (pub);
      pub->SetStartTime (Seconds (rng->GetValue ()));
      pub->SetStopTime  (Seconds (simTime - 10.0));
    }

  std::cout << "  Sensors: " << nECG << " ECG + " << nHR << " HR + "
            << (nSens - nECG - nHR) << " Temp\n\n";
}

// ============================================================
//  PrintResults
//  KEY FIX: in 2-tier mode FlowMonitor sees 3 hop-types:
//    sensor→CH, CH→local-broker, local-broker→root-broker
//  We must count ONLY sensor-origin flows (src in sensor IP set)
//  to get true end-to-end PDR comparable to single-broker mode.
// ============================================================
void PrintResults (Ptr<FlowMonitor> fm, FlowMonitorHelper &fh,
                   NodeContainer &chs, NodeContainer &localBrokers,
                   NodeContainer &rootSink,
                   Ipv4InterfaceContainer &ifaces,
                   uint32_t nSens, uint32_t nCH, uint32_t numLocalBrokers,
                   std::string csvName, double simTime)
{
  fm->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> cls = DynamicCast<Ipv4FlowClassifier> (fh.GetClassifier ());
  auto stats = fm->GetFlowStats ();

  // Build sensor IP set — sensors are ifaces[0..nSens-1]
  std::set<Ipv4Address> sensorIPs;
  for (uint32_t i = 0; i < nSens; i++)
    sensorIPs.insert (ifaces.GetAddress (i));

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
      double tp  = dur > 0 ? s.rxBytes*8.0/dur/1000 : 0;
      double dl  = s.rxPackets > 0 ? s.delaySum.GetMilliSeconds ()/(double)s.rxPackets : 0;
      double jt  = s.rxPackets > 1 ? s.jitterSum.GetMilliSeconds ()/(double)(s.rxPackets-1) : 0;

      csv << kv.first << "," << ft.sourceAddress << "," << ft.destinationAddress
          << "," << s.txPackets << "," << s.rxPackets << "," << s.lostPackets
          << "," << std::fixed << std::setprecision (2)
          << pdr << "," << tp << "," << dl << "," << jt << "\n";

      // Aggregate ONLY sensor-origin flows for PDR/delay headline metrics.
      // In single-broker mode all flows are sensor-origin so behaviour is identical.
      // In two-tier mode this filters out CH→broker and broker→root flows.
      if (sensorIPs.count (ft.sourceAddress))
        {
          tTx += s.txPackets; tRx += s.rxPackets; tRxB += s.rxBytes;
          tDel += s.rxPackets > 0 ? s.delaySum.GetMilliSeconds () : 0;
          fl++; if (s.rxPackets == 0) dead++;
        }
    }
  csv.close ();

  double pdr  = tTx > 0 ? (double)tRx/tTx*100 : 0;
  double avgD = tRx > 0 ? tDel/tRx : 0;

  std::cout << "============================================================\n"
    << "  RESULTS  (sensor-origin flows only)\n"
    << "============================================================\n"
    << "  Flows: " << fl << " (dead: " << dead << ")\n"
    << "  Tx: " << tTx << " | Rx: " << tRx << "\n"
    << "  PDR: " << std::fixed << std::setprecision (2) << pdr << "%\n"
    << "  Avg Delay: " << avgD << " ms\n"
    << "  Throughput: " << (tRxB*8.0/simTime/1000) << " kbps\n"
    << "  Output: " << csvName << "\n"
    << "============================================================\n\n";

  // Per-Gateway stats
  uint32_t totalEmerg = 0, totalFwd = 0;
  for (uint32_t c = 0; c < nCH; c++)
    {
      Ptr<MqttSnGateway> gw = DynamicCast<MqttSnGateway> (chs.Get (c)->GetApplication (0));
      if (gw) {
        gw->PrintStats ();
        totalEmerg += gw->GetStats ().emergencies;
        totalFwd   += gw->GetStats ().forwardedToBroker;
      }
    }

  // Broker stats by topology mode
  if (numLocalBrokers == 1)
    {
      Ptr<MqttSnBroker> br = DynamicCast<MqttSnBroker> (localBrokers.Get (0)->GetApplication (0));
      if (br) br->PrintStats ();
    }
  else if (numLocalBrokers > 1)
    {
      uint32_t nLB = numLocalBrokers;
      uint32_t totalToRoot = 0;
      for (uint32_t b = 0; b < nLB; b++)
        {
          Ptr<MqttSnBroker> lb = DynamicCast<MqttSnBroker> (localBrokers.Get (b)->GetApplication (0));
          if (lb) {
            lb->PrintStats ();
            totalToRoot += lb->GetStats ().forwardedToRoot;
          }
        }
      Ptr<MqttSnBroker> root = DynamicCast<MqttSnBroker> (rootSink.Get (0)->GetApplication (0));
      if (root) root->PrintStats ();
      std::cout << "  Total forwarded to root: " << totalToRoot << "\n\n";
    }

  std::cout << "  Emergencies: " << totalEmerg
            << " | Forwarded: " << totalFwd << "\n\n";

  fm->SerializeToXmlFile (csvName + ".flowmon.xml", true, true);
}

// ============================================================
//  MAIN
// ============================================================
int main (int argc, char *argv[])
{
  uint32_t nSensors        = 180;
  uint32_t nCH             = 20;
  uint32_t nECGcmd         = 0;
  uint32_t nHRcmd          = 0;
  uint32_t runNum          = 1;
  double   simTime         = 300.0;
  uint32_t numLocalBrokers = 1;  // 0=no broker, 1=single(baseline), N>=2=two-tier
  bool     enableMobility  = false;
  double   speed           = 1.5;
  bool     enableAnim      = false;
  bool     verbose         = false;
  std::string csvName      = "";

  // Legacy --broker flag: --broker=true → numLocalBrokers=1, --broker=false → 0
  // NOTE: Do NOT combine --broker with --numLocalBrokers in the same command.
  bool legacyBroker        = true;
  bool legacyBrokerSet     = false;  // detect if user actually passed --broker

  CommandLine cmd (__FILE__);
  cmd.AddValue ("nSensors",        "Number of sensor nodes",                   nSensors);
  cmd.AddValue ("nCH",             "Number of cluster heads",                  nCH);
  cmd.AddValue ("nECG",            "Number of ECG sensors (0=auto)",           nECGcmd);
  cmd.AddValue ("nHR",             "Number of HR sensors (0=auto)",            nHRcmd);
  cmd.AddValue ("run",             "Run number for seed variation",             runNum);
  cmd.AddValue ("simTime",         "Simulation time (s)",                      simTime);
  cmd.AddValue ("numLocalBrokers", "0=no broker, 1=single, N>=2=two-tier",     numLocalBrokers);
  cmd.AddValue ("broker",          "Legacy flag: true=single broker, false=none", legacyBroker);
  cmd.AddValue ("mobility",        "Enable sensor mobility",                   enableMobility);
  cmd.AddValue ("speed",           "Mobility speed (m/s)",                    speed);
  cmd.AddValue ("anim",            "Enable NetAnim output",                    enableAnim);
  cmd.AddValue ("verbose",         "Enable verbose logging",                   verbose);
  cmd.AddValue ("csv",             "Output CSV filename",                      csvName);
  cmd.Parse (argc, argv);

  // Apply legacy --broker only if --numLocalBrokers was not explicitly set
  // (detect by checking if numLocalBrokers is still at its default of 1
  //  and legacyBroker is false, meaning user passed --broker=false)
  if (!legacyBroker && numLocalBrokers == 1)
    numLocalBrokers = 0;

  if (verbose) {
    LogComponentEnable ("ClusterAodvMqtt", LOG_LEVEL_INFO);
    LogComponentEnable ("MqttSnPublisher", LOG_LEVEL_INFO);
    LogComponentEnable ("MqttSnGateway",   LOG_LEVEL_INFO);
    LogComponentEnable ("MqttSnBroker",    LOG_LEVEL_INFO);
  }

  // Auto CSV name encodes topology
  if (csvName.empty ())
    {
      std::ostringstream oss;
      oss << "mqtt-sn-" << nSensors << "nodes";
      if      (numLocalBrokers == 0) oss << "-nobroker";
      else if (numLocalBrokers == 1) oss << "-broker";
      else                           oss << "-hier" << numLocalBrokers;
      oss << "-results.csv";
      csvName = oss.str ();
    }

  // nLBnodes: physical nodes to create for the local broker tier
  //   mode 0 → 0 local broker nodes
  //   mode 1 → 1 local broker node (IS the single broker)
  //   mode N → N local broker nodes + 1 rootSink
  uint32_t nLBnodes   = (numLocalBrokers >= 1) ? numLocalBrokers : 0;
  uint32_t totalNodes = nSensors + nCH + nLBnodes + 1;  // +1 for rootSink always

  SeedManager::SetSeed (42); SeedManager::SetRun (runNum);

  std::cout << "\n============================================================\n"
    << "  MQTT-SN MIoT Simulation\n"
    << "  Sensors: " << nSensors << " | CHs: " << nCH
    << " | LocalBrokers: " << nLBnodes
    << " | numLocalBrokers=" << numLocalBrokers
    << " | Total: " << totalNodes << "\n"
    << "  Area: " << AREA << "x" << AREA << "m | AODV | " << simTime << "s\n"
    << "  CSV: " << csvName << "\n"
    << "============================================================\n\n";

  // Build node containers
  NodeContainer sens, chs, localBrokers, rootSink, all;
  sens.Create  (nSensors);
  chs.Create   (nCH);
  if (nLBnodes > 0) localBrokers.Create (nLBnodes);
  rootSink.Create (1);  // always created for positioning / root broker

  all.Add (sens); all.Add (chs);
  if (nLBnodes > 0) all.Add (localBrokers);
  all.Add (rootSink);

  YansWifiPhyHelper phy;
  NetDeviceContainer     dev    = ConfigWifi  (all, phy);
  Ipv4InterfaceContainer ifaces = InstallStack (all, dev);

  PositionNodes  (sens, chs, localBrokers, rootSink, nCH, nLBnodes, enableMobility, speed);
  AssignClusters (sens, chs, nCH);

  InstallMqtt (sens, chs, localBrokers, rootSink, ifaces,
               nSensors, nCH, numLocalBrokers, nECGcmd, nHRcmd, simTime);

  // ---- Energy Model ----
  BasicEnergySourceHelper sensEnergyHelper;
  sensEnergyHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (2.0));
  sensEnergyHelper.Set ("BasicEnergySupplyVoltageV",        DoubleValue (3.3));
  EnergySourceContainer sensSources = sensEnergyHelper.Install (sens);

  BasicEnergySourceHelper chEnergyHelper;
  chEnergyHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (5.0));
  chEnergyHelper.Set ("BasicEnergySupplyVoltageV",        DoubleValue (3.3));
  EnergySourceContainer chSources = chEnergyHelper.Install (chs);

  WifiRadioEnergyModelHelper radioEnergyHelper;
  radioEnergyHelper.Set ("TxCurrentA",    DoubleValue (0.346));
  radioEnergyHelper.Set ("RxCurrentA",    DoubleValue (0.285));
  radioEnergyHelper.Set ("IdleCurrentA",  DoubleValue (0.248));
  radioEnergyHelper.Set ("SleepCurrentA", DoubleValue (0.030));

  for (uint32_t i = 0; i < nSensors; i++)
    { NetDeviceContainer d; d.Add (dev.Get (i)); radioEnergyHelper.Install (d, sensSources.Get (i)); }
  for (uint32_t i = 0; i < nCH; i++)
    { NetDeviceContainer d; d.Add (dev.Get (nSensors+i)); radioEnergyHelper.Install (d, chSources.Get (i)); }

  std::cout << "  Energy: Sensors=2.0J, CHs=5.0J\n\n";

  FlowMonitorHelper fh;
  Ptr<FlowMonitor> fm = fh.InstallAll ();

  // NetAnim
  AnimationInterface *anim = nullptr;
  if (enableAnim)
    {
      anim = new AnimationInterface ("mqtt-sn-animation.xml");
      anim->SetMaxPktsPerTraceFile (500000);
      uint32_t nECG = nSensors/3, nHR = nSensors/3;
      for (uint32_t i = 0; i < nECG; i++)
        { anim->UpdateNodeColor (sens.Get(i), 255,59,48); anim->UpdateNodeSize(sens.Get(i)->GetId(),8,8); }
      for (uint32_t i = nECG; i < nECG+nHR; i++)
        { anim->UpdateNodeColor (sens.Get(i), 0,122,255); anim->UpdateNodeSize(sens.Get(i)->GetId(),8,8); }
      for (uint32_t i = nECG+nHR; i < nSensors; i++)
        { anim->UpdateNodeColor (sens.Get(i), 52,199,89); anim->UpdateNodeSize(sens.Get(i)->GetId(),8,8); }
      for (uint32_t i = 0; i < nCH; i++)
        { anim->UpdateNodeColor (chs.Get(i), 175,82,222); anim->UpdateNodeSize(chs.Get(i)->GetId(),25,25); }
      for (uint32_t i = 0; i < nLBnodes; i++)
        { anim->UpdateNodeColor (localBrokers.Get(i), 255,149,0); anim->UpdateNodeSize(localBrokers.Get(i)->GetId(),30,30); }
      anim->UpdateNodeColor (rootSink.Get(0), 255,204,0);
      anim->UpdateNodeSize  (rootSink.Get(0)->GetId(), 40, 40);
    }

  std::cout << "[RUN] " << simTime << "s...\n";
  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  PrintResults (fm, fh, chs, localBrokers, rootSink,
                ifaces, nSensors, nCH, numLocalBrokers, csvName, simTime);

  // ---- Energy Report ----
  std::cout << "  ============================================\n"
            << "  ENERGY REPORT\n"
            << "  ============================================\n";

  double totalSensInit = nSensors * 2.0;
  double totalCHInit   = nCH * 5.0;
  double totalSensRemain = 0, totalCHRemain = 0;
  uint32_t deadSensors = 0, deadCHs = 0;
  double minSensE = 999, maxSensE = 0, sumSensE = 0;
  for (uint32_t i = 0; i < nSensors; i++)
    {
      double r = sensSources.Get (i)->GetRemainingEnergy ();
      totalSensRemain += r; sumSensE += r;
      if (r < minSensE) minSensE = r; if (r > maxSensE) maxSensE = r;
      if (r <= 0.0) deadSensors++;
    }
  double minCHE = 999, maxCHE = 0, sumCHE = 0;
  for (uint32_t i = 0; i < nCH; i++)
    {
      double r = chSources.Get (i)->GetRemainingEnergy ();
      totalCHRemain += r; sumCHE += r;
      if (r < minCHE) minCHE = r; if (r > maxCHE) maxCHE = r;
      if (r <= 0.0) deadCHs++;
    }
  double sensConsumed = totalSensInit - totalSensRemain;
  double chConsumed   = totalCHInit   - totalCHRemain;
  double avgSensE     = nSensors > 0 ? sumSensE / nSensors : 0;
  double avgCHE       = nCH      > 0 ? sumCHE   / nCH     : 0;

  std::cout << std::fixed << std::setprecision (4)
    << "  Sensor Energy:\n"
    << "    Initial: "    << totalSensInit   << " J (" << nSensors << " x 2.0 J)\n"
    << "    Remaining: "  << totalSensRemain << " J\n"
    << "    Consumed: "   << sensConsumed    << " J ("
    << std::setprecision(1) << (sensConsumed/totalSensInit*100) << "%)\n"
    << std::setprecision(4)
    << "    Per sensor: avg=" << avgSensE << "J  min=" << minSensE << "J  max=" << maxSensE << "J\n"
    << "    Dead sensors: "   << deadSensors << "\n"
    << "  CH Energy:\n"
    << "    Initial: "    << totalCHInit   << " J (" << nCH << " x 5.0 J)\n"
    << "    Remaining: "  << totalCHRemain << " J\n"
    << "    Consumed: "   << chConsumed    << " J ("
    << std::setprecision(1) << (chConsumed/totalCHInit*100) << "%)\n"
    << std::setprecision(4)
    << "    Per CH: avg=" << avgCHE << "J  min=" << minCHE << "J  max=" << maxCHE << "J\n"
    << "    Dead CHs: "   << deadCHs << "\n"
    << "  ============================================\n\n";

  std::string energyCsv = csvName + ".energy.csv";
  std::ofstream ecsv (energyCsv);
  ecsv << "NodeID,Type,InitialEnergy_J,RemainingEnergy_J,ConsumedEnergy_J,ConsumedPct\n";
  for (uint32_t i = 0; i < nSensors; i++)
    {
      double rem = sensSources.Get (i)->GetRemainingEnergy ();
      double cons = 2.0 - rem;
      std::string t = (i < nSensors/3) ? "ECG" : (i < 2*nSensors/3) ? "HR" : "Temp";
      ecsv << i << "," << t << ",2.0," << std::fixed << std::setprecision(4)
           << rem << "," << cons << "," << std::setprecision(2) << (cons/2.0*100) << "\n";
    }
  for (uint32_t i = 0; i < nCH; i++)
    {
      double rem = chSources.Get (i)->GetRemainingEnergy ();
      double cons = 5.0 - rem;
      ecsv << (nSensors+i) << ",CH,5.0," << std::fixed << std::setprecision(4)
           << rem << "," << cons << "," << std::setprecision(2) << (cons/5.0*100) << "\n";
    }
  ecsv.close ();
  std::cout << "  Energy CSV: " << energyCsv << "\n\n";

  Simulator::Destroy ();
  if (anim) delete anim;
  std::cout << "[DONE]\n\n";
  return 0;
}
