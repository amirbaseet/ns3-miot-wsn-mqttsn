/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * ns3-miot-simulation — Cluster-Based MIoT Comparison
 *
 * File:         mqtt-sn-header.cc
 * Description:  MQTT-SN header serialization/deserialization implementation.
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
#include "mqtt-sn-header.h"
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MqttSnHeader");
NS_OBJECT_ENSURE_REGISTERED (MqttSnHeader);

TypeId MqttSnHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MqttSnHeader")
    .SetParent<Header> ()
    .AddConstructor<MqttSnHeader> ();
  return tid;
}

TypeId MqttSnHeader::GetInstanceTypeId () const { return GetTypeId (); }

MqttSnHeader::MqttSnHeader ()
  : m_msgType (0), m_topicId (0), m_msgId (0),
    m_payloadSize (0), m_qos (0), m_emergency (0) {}

MqttSnHeader::~MqttSnHeader () {}

void MqttSnHeader::SetMsgType (uint8_t t) { m_msgType = t; }
void MqttSnHeader::SetTopicId (uint16_t t) { m_topicId = t; }
void MqttSnHeader::SetMsgId (uint16_t m) { m_msgId = m; }
void MqttSnHeader::SetPayloadSize (uint16_t s) { m_payloadSize = s; }
void MqttSnHeader::SetQos (uint8_t q) { m_qos = q; }
void MqttSnHeader::SetEmergency (bool e) { m_emergency = e ? 1 : 0; }

uint8_t  MqttSnHeader::GetMsgType () const { return m_msgType; }
uint16_t MqttSnHeader::GetTopicId () const { return m_topicId; }
uint16_t MqttSnHeader::GetMsgId () const { return m_msgId; }
uint16_t MqttSnHeader::GetPayloadSize () const { return m_payloadSize; }
uint8_t  MqttSnHeader::GetQos () const { return m_qos; }
bool     MqttSnHeader::IsEmergency () const { return m_emergency != 0; }

uint8_t
MqttSnHeader::GetPriority () const
{
  // Emergency always highest priority (3)
  if (m_emergency) return 3;
  // Otherwise priority = QoS level (0=low, 1=medium, 2=high)
  return m_qos;
}

std::string MqttSnHeader::GetMsgTypeName () const
{
  switch (m_msgType) {
    case MQTTSN_CONNECT:    return "CONNECT";
    case MQTTSN_CONNACK:    return "CONNACK";
    case MQTTSN_PUBLISH:    return "PUBLISH";
    case MQTTSN_PUBACK:     return "PUBACK";
    case MQTTSN_SUBSCRIBE:  return "SUBSCRIBE";
    case MQTTSN_SUBACK:     return "SUBACK";
    case MQTTSN_PINGREQ:    return "PINGREQ";
    case MQTTSN_PINGRESP:   return "PINGRESP";
    case MQTTSN_DISCONNECT: return "DISCONNECT";
    default:                return "UNKNOWN";
  }
}

std::string MqttSnHeader::GetTopicName () const
{
  switch (m_topicId) {
    case TOPIC_ECG:         return "ECG";
    case TOPIC_HEART_RATE:  return "HeartRate";
    case TOPIC_TEMPERATURE: return "Temperature";
    case TOPIC_BLOOD_OX:    return "BloodOxygen";
    case TOPIC_BLOOD_PRESS: return "BloodPressure";
    case TOPIC_EMERGENCY:   return "EMERGENCY";
    default:                return "Topic_" + std::to_string (m_topicId);
  }
}

std::string MqttSnHeader::GetPriorityName () const
{
  switch (GetPriority ()) {
    case 3:  return "CRITICAL";
    case 2:  return "HIGH";
    case 1:  return "MEDIUM";
    case 0:  return "LOW";
    default: return "UNKNOWN";
  }
}

// Serialized: [Len:1][MsgType:1][QoS:1][Emergency:1][TopicID:2][MsgID:2][PayloadSize:2] = 10 bytes
uint32_t MqttSnHeader::GetSerializedSize () const { return 10; }

void MqttSnHeader::Serialize (Buffer::Iterator start) const
{
  start.WriteU8 (GetSerializedSize ());
  start.WriteU8 (m_msgType);
  start.WriteU8 (m_qos);
  start.WriteU8 (m_emergency);
  start.WriteHtonU16 (m_topicId);
  start.WriteHtonU16 (m_msgId);
  start.WriteHtonU16 (m_payloadSize);
}

uint32_t MqttSnHeader::Deserialize (Buffer::Iterator start)
{
  start.ReadU8 ();
  m_msgType     = start.ReadU8 ();
  m_qos         = start.ReadU8 ();
  m_emergency   = start.ReadU8 ();
  m_topicId     = start.ReadNtohU16 ();
  m_msgId       = start.ReadNtohU16 ();
  m_payloadSize = start.ReadNtohU16 ();
  return GetSerializedSize ();
}

void MqttSnHeader::Print (std::ostream &os) const
{
  os << "MQTT-SN [" << GetMsgTypeName ()
     << " Topic=" << GetTopicName ()
     << " MsgID=" << m_msgId
     << " QoS=" << (uint16_t)m_qos
     << " Priority=" << GetPriorityName ()
     << (m_emergency ? " EMERGENCY!" : "")
     << " Payload=" << m_payloadSize << "B]";
}

} // namespace ns3
