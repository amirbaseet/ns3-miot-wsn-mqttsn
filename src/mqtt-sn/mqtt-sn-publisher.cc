/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * ns3-miot-simulation — Cluster-Based MIoT Comparison
 *
 * File:         mqtt-sn-publisher.cc
 * Description:  MQTT-SN publisher implementation with per-sensor-type intervals.
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
#include "mqtt-sn-publisher.h"
#include "mqtt-sn-header.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/inet-socket-address.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/packet.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MqttSnPublisher");
NS_OBJECT_ENSURE_REGISTERED (MqttSnPublisher);

TypeId MqttSnPublisher::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MqttSnPublisher")
    .SetParent<Application> ()
    .AddConstructor<MqttSnPublisher> ();
  return tid;
}

MqttSnPublisher::MqttSnPublisher ()
  : m_socket (nullptr), m_port (9), m_topicId (TOPIC_HEART_RATE),
    m_interval (Seconds (1.0)), m_payloadSize (64), m_qos (0),
    m_msgCounter (0), m_running (false),
    m_emergencyEnabled (false), m_emergencyProb (0.01),
    m_txPackets (0), m_txBytes (0), m_emergencyCount (0)
{
  m_emergencyRng = CreateObject<UniformRandomVariable> ();
}

MqttSnPublisher::~MqttSnPublisher () { m_socket = nullptr; }

void MqttSnPublisher::Setup (Address ga, uint16_t port, uint16_t topic,
                             Time interval, uint32_t payload, uint8_t qos)
{
  m_gatewayAddr = ga; m_port = port; m_topicId = topic;
  m_interval = interval; m_payloadSize = payload; m_qos = qos;
}

void MqttSnPublisher::EnableEmergencyDetection (double prob)
{
  m_emergencyEnabled = true;
  m_emergencyProb = prob;
}

void MqttSnPublisher::StartApplication (void)
{
  m_running = true;
  m_msgCounter = 0;
  m_txPackets = 0; m_txBytes = 0; m_emergencyCount = 0;

  if (!m_socket)
    {
      m_socket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
      m_socket->Bind ();
      m_socket->SetRecvCallback (MakeCallback (&MqttSnPublisher::HandleRead, this));
    }

  SendConnect ();
  m_sendEvent = Simulator::Schedule (Seconds (0.5), &MqttSnPublisher::SendPublish, this);
}

void MqttSnPublisher::StopApplication (void)
{
  m_running = false;
  Simulator::Cancel (m_sendEvent);

  if (m_socket)
    {
      Ptr<Packet> pkt = Create<Packet> ();
      MqttSnHeader h;
      h.SetMsgType (MQTTSN_DISCONNECT);
      h.SetTopicId (m_topicId);
      h.SetMsgId (m_msgCounter++);
      h.SetPayloadSize (0);
      h.SetQos (0);
      h.SetEmergency (false);
      pkt->AddHeader (h);
      m_socket->SendTo (pkt, 0, InetSocketAddress (Ipv4Address::ConvertFrom (m_gatewayAddr), m_port));

      NS_LOG_INFO ("[Node " << GetNode ()->GetId ()
                   << "] DISCONNECT. Published: " << m_txPackets
                   << " pkts, Emergencies: " << m_emergencyCount);
    }
}

void MqttSnPublisher::SendConnect (void)
{
  Ptr<Packet> pkt = Create<Packet> ();
  MqttSnHeader h;
  h.SetMsgType (MQTTSN_CONNECT);
  h.SetTopicId (m_topicId);
  h.SetMsgId (0);
  h.SetPayloadSize (0);
  h.SetQos (m_qos);
  h.SetEmergency (false);
  pkt->AddHeader (h);
  m_socket->SendTo (pkt, 0, InetSocketAddress (Ipv4Address::ConvertFrom (m_gatewayAddr), m_port));

  NS_LOG_INFO ("[Node " << GetNode ()->GetId ()
               << "] CONNECT Topic=" << h.GetTopicName ()
               << " QoS=" << (int)m_qos
               << " Priority=" << h.GetPriorityName ());
}

void MqttSnPublisher::SendPublish (void)
{
  if (!m_running) return;

  // Check for emergency (abnormal sensor value)
  if (m_emergencyEnabled && m_emergencyRng->GetValue () < m_emergencyProb)
    {
      SendEmergency ();
    }

  // Normal PUBLISH
  Ptr<Packet> pkt = Create<Packet> (m_payloadSize);
  MqttSnHeader h;
  h.SetMsgType (MQTTSN_PUBLISH);
  h.SetTopicId (m_topicId);
  h.SetMsgId (m_msgCounter++);
  h.SetPayloadSize (m_payloadSize);
  h.SetQos (m_qos);
  h.SetEmergency (false);
  pkt->AddHeader (h);

  int sent = m_socket->SendTo (pkt, 0,
    InetSocketAddress (Ipv4Address::ConvertFrom (m_gatewayAddr), m_port));

  if (sent > 0) { m_txPackets++; m_txBytes += sent; }

  NS_LOG_DEBUG ("[Node " << GetNode ()->GetId ()
                << "] PUBLISH #" << (m_msgCounter - 1)
                << " Topic=" << h.GetTopicName ()
                << " Priority=" << h.GetPriorityName ());

  m_sendEvent = Simulator::Schedule (m_interval, &MqttSnPublisher::SendPublish, this);
}

void MqttSnPublisher::SendEmergency (void)
{
  // Emergency: abnormal value detected! Send immediately with highest priority
  Ptr<Packet> pkt = Create<Packet> (m_payloadSize);
  MqttSnHeader h;
  h.SetMsgType (MQTTSN_PUBLISH);
  h.SetTopicId (TOPIC_EMERGENCY);
  h.SetMsgId (m_msgCounter++);
  h.SetPayloadSize (m_payloadSize);
  h.SetQos (PRIORITY_CRITICAL);
  h.SetEmergency (true);
  pkt->AddHeader (h);

  m_socket->SendTo (pkt, 0,
    InetSocketAddress (Ipv4Address::ConvertFrom (m_gatewayAddr), m_port));

  m_emergencyCount++;
  m_txPackets++;

  NS_LOG_INFO ("[Node " << GetNode ()->GetId ()
               << "] *** EMERGENCY ALERT *** #" << m_emergencyCount
               << " from " << h.GetTopicName ());
}

void MqttSnPublisher::HandleRead (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address from;
  while ((packet = socket->RecvFrom (from)))
    {
      MqttSnHeader h;
      packet->RemoveHeader (h);
      if (h.GetMsgType () == MQTTSN_CONNACK)
        NS_LOG_INFO ("[Node " << GetNode ()->GetId () << "] CONNACK received");
      else if (h.GetMsgType () == MQTTSN_PUBACK)
        NS_LOG_DEBUG ("[Node " << GetNode ()->GetId () << "] PUBACK MsgID=" << h.GetMsgId ());
    }
}

} // namespace ns3
