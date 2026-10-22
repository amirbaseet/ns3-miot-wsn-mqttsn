/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * ns3-miot-simulation — Cluster-Based MIoT Comparison
 *
 * File:         mqtt-sn-gateway.cc
 * Description:  MQTT-SN gateway implementation with 4-level priority queue.
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
#include "mqtt-sn-gateway.h"
#include "mqtt-sn-header.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/inet-socket-address.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/packet.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MqttSnGateway");
NS_OBJECT_ENSURE_REGISTERED (MqttSnGateway);

TypeId MqttSnGateway::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MqttSnGateway")
    .SetParent<Application> ()
    .AddConstructor<MqttSnGateway> ();
  return tid;
}

MqttSnGateway::MqttSnGateway ()
  : m_socket (nullptr), m_brokerSocket (nullptr), m_port (9),
    m_brokerPort (1884), m_hasBroker (false) {}

MqttSnGateway::~MqttSnGateway () { m_socket = nullptr; m_brokerSocket = nullptr; }

void MqttSnGateway::Setup (uint16_t port) { m_port = port; }

void MqttSnGateway::SetBroker (Ipv4Address brokerAddr, uint16_t brokerPort)
{
  m_brokerAddr = brokerAddr;
  m_brokerPort = brokerPort;
  m_hasBroker = true;
}

GatewayStats MqttSnGateway::GetStats () const { return m_stats; }

void MqttSnGateway::PrintStats () const
{
  std::cout << "    CH" << GetNode ()->GetId ()
            << " | Rx:" << m_stats.rxPackets
            << " Emerg:" << m_stats.emergencies
            << " Fwd:" << m_stats.forwardedToBroker
            << " QDrop:" << m_stats.queueDrops
            << " | Pri[H:" << m_stats.processedHigh
            << " M:" << m_stats.processedMed
            << " L:" << m_stats.processedLow << "]"
            << " | Topics:";
  for (auto &kv : m_stats.topicCounts)
    {
      MqttSnHeader tmp;
      tmp.SetTopicId (kv.first);
      std::cout << " " << tmp.GetTopicName () << "=" << kv.second;
    }
  std::cout << std::endl;
}

void MqttSnGateway::StartApplication (void)
{
  // Socket for receiving from sensors
  if (!m_socket)
    {
      m_socket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      m_socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), m_port));
      m_socket->SetRecvCallback (MakeCallback (&MqttSnGateway::HandleRead, this));
    }

  // Socket for forwarding to broker
  if (m_hasBroker && !m_brokerSocket)
    {
      m_brokerSocket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      m_brokerSocket->Bind ();
    }

  NS_LOG_INFO ("[CH " << GetNode ()->GetId () << "] Gateway on port " << m_port
               << (m_hasBroker ? " (broker forwarding enabled)" : ""));
}

void MqttSnGateway::StopApplication (void)
{
  ProcessQueue ();
  if (m_socket)
    {
      m_socket->Close ();
      m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket>> ());
    }
  if (m_brokerSocket)
    {
      m_brokerSocket->Close ();
    }
  PrintStats ();
}

void MqttSnGateway::HandleRead (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address from;

  while ((packet = socket->RecvFrom (from)))
    {
      MqttSnHeader header;
      packet->RemoveHeader (header);

      m_stats.rxPackets++;
      m_stats.rxBytes += packet->GetSize () + header.GetSerializedSize ();

      switch (header.GetMsgType ())
        {
        case MQTTSN_CONNECT:
          m_stats.connects++;
          NS_LOG_INFO ("[CH " << GetNode ()->GetId ()
                       << "] CONNECT from " << InetSocketAddress::ConvertFrom (from).GetIpv4 ()
                       << " Topic=" << header.GetTopicName ()
                       << " Priority=" << header.GetPriorityName ());
          SendConnAck (socket, from);
          break;

        case MQTTSN_PUBLISH:
          {
            m_stats.topicCounts[header.GetTopicId ()]++;
            m_stats.priorityCounts[header.GetPriority ()]++;

            if (header.IsEmergency ())
              {
                m_stats.emergencies++;
                NS_LOG_INFO ("[CH " << GetNode ()->GetId ()
                             << "] *** EMERGENCY *** from "
                             << InetSocketAddress::ConvertFrom (from).GetIpv4 ());
              }

            // Add to priority queue
            PriorityPacket pp;
            pp.priority  = header.GetPriority ();
            pp.topicId   = header.GetTopicId ();
            pp.msgId     = header.GetMsgId ();
            pp.size      = header.GetPayloadSize ();
            pp.rxTime    = Simulator::Now ().GetSeconds ();
            pp.qos       = header.GetQos ();
            pp.emergency = header.IsEmergency ();
            if (m_pQueue.size () >= 10000)
              {
                m_stats.queueDrops++;
                NS_LOG_WARN ("[CH " << GetNode ()->GetId () << "] Queue full — DROP");
              }
            else
              {
                m_pQueue.push (pp);
              }

            if (header.GetQos () > 0)
              SendPubAck (socket, from, header.GetMsgId ());

            ProcessQueue ();
            break;
          }

        case MQTTSN_DISCONNECT:
          m_stats.disconnects++;
          NS_LOG_INFO ("[CH " << GetNode ()->GetId ()
                       << "] DISCONNECT from "
                       << InetSocketAddress::ConvertFrom (from).GetIpv4 ());
          break;

        default:
          NS_LOG_WARN ("[CH " << GetNode ()->GetId ()
                       << "] Unknown MsgType: " << (int)header.GetMsgType ());
          break;
        }
    }
}

void MqttSnGateway::ProcessQueue (void)
{
  // Process ONE packet at a time with 1ms delay between each
  // This makes priority queue effect visible in delay measurements
  if (!m_pQueue.empty ())
    {
      PriorityPacket pp = m_pQueue.top ();
      m_pQueue.pop ();

      if (pp.priority >= 2)      m_stats.processedHigh++;
      else if (pp.priority == 1) m_stats.processedMed++;
      else                       m_stats.processedLow++;

      // Forward to broker (sink)
      if (m_hasBroker)
        ForwardToBroker (pp);

      // Schedule next packet processing after 1ms
      if (!m_pQueue.empty ())
        Simulator::Schedule (MilliSeconds (1), &MqttSnGateway::ProcessQueue, this);
    }
}

void MqttSnGateway::ForwardToBroker (PriorityPacket &pp)
{
  // Create aggregated packet to forward to broker
  Ptr<Packet> pkt = Create<Packet> (pp.size);
  MqttSnHeader header;
  header.SetMsgType (MQTTSN_PUBLISH);
  header.SetTopicId (pp.topicId);
  header.SetMsgId (pp.msgId);
  header.SetPayloadSize (pp.size);
  header.SetQos (pp.qos);
  header.SetEmergency (pp.emergency);
  pkt->AddHeader (header);

  m_brokerSocket->SendTo (pkt, 0,
    InetSocketAddress (m_brokerAddr, m_brokerPort));

  m_stats.forwardedToBroker++;

  NS_LOG_DEBUG ("[CH " << GetNode ()->GetId ()
                << "] FORWARD to Broker: Topic=" << header.GetTopicName ()
                << " Priority=" << header.GetPriorityName ());
}

void MqttSnGateway::SendConnAck (Ptr<Socket> socket, Address from)
{
  Ptr<Packet> pkt = Create<Packet> ();
  MqttSnHeader h;
  h.SetMsgType (MQTTSN_CONNACK);
  h.SetQos (0); h.SetEmergency (false);
  pkt->AddHeader (h);
  socket->SendTo (pkt, 0, from);
}

void MqttSnGateway::SendPubAck (Ptr<Socket> socket, Address from, uint16_t msgId)
{
  Ptr<Packet> pkt = Create<Packet> ();
  MqttSnHeader h;
  h.SetMsgType (MQTTSN_PUBACK);
  h.SetMsgId (msgId);
  h.SetQos (0); h.SetEmergency (false);
  pkt->AddHeader (h);
  socket->SendTo (pkt, 0, from);
}

} // namespace ns3
