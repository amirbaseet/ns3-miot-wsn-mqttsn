/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * ns3-miot-simulation — Cluster-Based MIoT Comparison
 *
 * File:         mqtt-sn-broker.cc
 * Description:  MQTT-SN broker implementation with hierarchical aggregation.
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
#include "mqtt-sn-broker.h"
#include "mqtt-sn-header.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/inet-socket-address.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/packet.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MqttSnBroker");
NS_OBJECT_ENSURE_REGISTERED (MqttSnBroker);

TypeId MqttSnBroker::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MqttSnBroker")
    .SetParent<Application> ()
    .AddConstructor<MqttSnBroker> ();
  return tid;
}

MqttSnBroker::MqttSnBroker ()
  : m_socket (nullptr), m_rootSocket (nullptr),
    m_port (1884), m_rootPort (1885), m_hasRoot (false) {}

MqttSnBroker::~MqttSnBroker ()
{
  m_socket     = nullptr;
  m_rootSocket = nullptr;
}

void MqttSnBroker::Setup (uint16_t port)
{
  m_port = port;
}

void MqttSnBroker::SetRootBroker (Ipv4Address rootAddr, uint16_t rootPort)
{
  m_rootAddr = rootAddr;
  m_rootPort = rootPort;
  m_hasRoot  = true;
}

BrokerStats MqttSnBroker::GetStats () const { return m_stats; }

void MqttSnBroker::PrintStats () const
{
  std::string role     = m_hasRoot ? "LOCAL BROKER (tier-1)" : "ROOT BROKER (tier-2)";
  std::string upstreamLabel = m_hasRoot ? "From Gateways     " : "From Local Brokers";

  std::cout << "\n  ============================================\n"
            << "  " << role << " STATISTICS\n"
            << "  ============================================\n"
            << "  Total Rx Packets  : " << m_stats.rxPackets << "\n"
            << "  Total Rx Bytes    : " << m_stats.rxBytes << "\n"
            << "  " << upstreamLabel << ": " << m_stats.fromUpstream << "\n"
            << "  Emergencies       : " << m_stats.emergencies << "\n";
  if (m_hasRoot)
    std::cout << "  Forwarded to Root : " << m_stats.forwardedToRoot << "\n";
  std::cout << "  ---\n"
            << "  Per-Topic Breakdown:\n";
  for (auto &kv : m_stats.topicCounts)
    {
      MqttSnHeader tmp;
      tmp.SetTopicId (kv.first);
      std::cout << "    " << tmp.GetTopicName () << " (ID=" << kv.first
                << "): " << kv.second << " packets\n";
    }
  std::cout << "  Per-Priority Breakdown:\n";
  for (auto &kv : m_stats.priorityCounts)
    {
      std::string name;
      switch (kv.first) {
        case 3:  name = "CRITICAL"; break;
        case 2:  name = "HIGH";     break;
        case 1:  name = "MEDIUM";   break;
        default: name = "LOW";      break;
      }
      std::cout << "    " << name << " (" << (int)kv.first
                << "): " << kv.second << " packets\n";
    }
  std::cout << "  ============================================\n\n";
}

void MqttSnBroker::StartApplication (void)
{
  // Receive socket — accepts from gateways (tier-1) or local brokers (root)
  if (!m_socket)
    {
      m_socket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      m_socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), m_port));
      m_socket->SetRecvCallback (MakeCallback (&MqttSnBroker::HandleRead, this));
    }

  // Forward socket — only created for tier-1 brokers
  if (m_hasRoot && !m_rootSocket)
    {
      m_rootSocket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      m_rootSocket->Bind ();
    }

  NS_LOG_INFO ("[BROKER] Listening on port " << m_port
               << (m_hasRoot ? " (tier-1, forwards to root)" : " (root)"));
}

void MqttSnBroker::StopApplication (void)
{
  if (m_socket)
    {
      m_socket->Close ();
      m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket>> ());
    }
  if (m_rootSocket)
    m_rootSocket->Close ();

  PrintStats ();
}

void MqttSnBroker::HandleRead (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address from;

  while ((packet = socket->RecvFrom (from)))
    {
      MqttSnHeader header;
      packet->RemoveHeader (header);

      m_stats.rxPackets++;
      m_stats.rxBytes += packet->GetSize () + header.GetSerializedSize ();
      m_stats.fromUpstream++;
      m_stats.topicCounts[header.GetTopicId ()]++;
      m_stats.priorityCounts[header.GetPriority ()]++;

      if (header.IsEmergency ())
        {
          m_stats.emergencies++;
          NS_LOG_INFO ("[BROKER] *** EMERGENCY *** from "
                       << InetSocketAddress::ConvertFrom (from).GetIpv4 ()
                       << " Topic=" << header.GetTopicName ());
        }
      else
        {
          NS_LOG_DEBUG ("[BROKER] Rx from "
                        << InetSocketAddress::ConvertFrom (from).GetIpv4 ()
                        << " Topic=" << header.GetTopicName ()
                        << " Priority=" << header.GetPriorityName ());
        }

      // Tier-1 broker: forward every received packet up to the root
      if (m_hasRoot)
        ForwardToRoot (header);
    }
}

void MqttSnBroker::ForwardToRoot (MqttSnHeader &hdr)
{
  // Build a fresh packet from the received header fields and forward to root.
  // Pattern mirrors MqttSnGateway::ForwardToBroker exactly.
  Ptr<Packet> fwdPkt = Create<Packet> (hdr.GetPayloadSize ());
  MqttSnHeader fwdHdr;
  fwdHdr.SetMsgType    (MQTTSN_PUBLISH);
  fwdHdr.SetTopicId    (hdr.GetTopicId ());
  fwdHdr.SetMsgId      (hdr.GetMsgId ());
  fwdHdr.SetPayloadSize(hdr.GetPayloadSize ());
  fwdHdr.SetQos        (hdr.GetQos ());
  fwdHdr.SetEmergency  (hdr.IsEmergency ());
  fwdPkt->AddHeader (fwdHdr);

  m_rootSocket->SendTo (fwdPkt, 0,
    InetSocketAddress (m_rootAddr, m_rootPort));

  m_stats.forwardedToRoot++;

  NS_LOG_DEBUG ("[LOCAL-BROKER] Forwarded to root: Topic=" << hdr.GetTopicName ()
                << " Priority=" << hdr.GetPriorityName ());
}

} // namespace ns3
