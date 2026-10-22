/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * ns3-miot-simulation — Cluster-Based MIoT Comparison
 *
 * File:         mqtt-sn-header.h
 * Description:  MQTT-SN packet header with 4-level priority enum and QoS support.
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
#ifndef MQTT_SN_HEADER_H
#define MQTT_SN_HEADER_H

#include "ns3/header.h"
#include "ns3/buffer.h"

namespace ns3 {

// Message Types
enum MqttSnMsgType : uint8_t
{
  MQTTSN_CONNECT    = 0x04,
  MQTTSN_CONNACK    = 0x05,
  MQTTSN_PUBLISH    = 0x0C,
  MQTTSN_PUBACK     = 0x0D,
  MQTTSN_SUBSCRIBE  = 0x12,
  MQTTSN_SUBACK     = 0x13,
  MQTTSN_PINGREQ    = 0x16,
  MQTTSN_PINGRESP   = 0x17,
  MQTTSN_DISCONNECT = 0x18
};

// Topic IDs
enum MqttSnTopicId : uint16_t
{
  TOPIC_ECG          = 10,
  TOPIC_HEART_RATE   = 20,
  TOPIC_TEMPERATURE  = 30,
  TOPIC_BLOOD_OX     = 40,
  TOPIC_BLOOD_PRESS  = 50,
  TOPIC_EMERGENCY    = 99
};

// Priority Levels (mapped to QoS)
enum MqttSnPriority : uint8_t
{
  PRIORITY_HIGH   = 2,   // ECG — must deliver, retransmit if lost
  PRIORITY_CRITICAL = 3,  // Emergency — highest priority
  PRIORITY_MEDIUM = 1,   // Heart Rate — important but tolerates some loss
  PRIORITY_LOW    = 0    // Temperature — best effort
};

class MqttSnHeader : public Header
{
public:
  MqttSnHeader ();
  virtual ~MqttSnHeader ();

  void SetMsgType (uint8_t type);
  void SetTopicId (uint16_t topicId);
  void SetMsgId (uint16_t msgId);
  void SetPayloadSize (uint16_t size);
  void SetQos (uint8_t qos);
  void SetEmergency (bool isEmergency);

  uint8_t GetMsgType () const;
  uint16_t GetTopicId () const;
  uint16_t GetMsgId () const;
  uint16_t GetPayloadSize () const;
  uint8_t GetQos () const;
  bool IsEmergency () const;

  // Priority based on QoS + Emergency flag
  uint8_t GetPriority () const;

  std::string GetMsgTypeName () const;
  std::string GetTopicName () const;
  std::string GetPriorityName () const;

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;

private:
  uint8_t  m_msgType;
  uint16_t m_topicId;
  uint16_t m_msgId;
  uint16_t m_payloadSize;
  uint8_t  m_qos;
  uint8_t  m_emergency;  // 0=normal, 1=emergency
};

} // namespace ns3
#endif
