/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * ns3-miot-simulation — Cluster-Based MIoT Comparison
 *
 * File:         mqtt-sn-publisher.h
 * Description:  MQTT-SN sensor app — heterogeneous traffic (ECG/HR/Temp) + emergency.
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
#ifndef MQTT_SN_PUBLISHER_H
#define MQTT_SN_PUBLISHER_H

#include "ns3/application.h"
#include "ns3/socket.h"
#include "ns3/address.h"
#include "ns3/event-id.h"
#include "ns3/random-variable-stream.h"

namespace ns3 {

class MqttSnPublisher : public Application
{
public:
  static TypeId GetTypeId (void);
  MqttSnPublisher ();
  virtual ~MqttSnPublisher ();

  void Setup (Address gatewayAddr, uint16_t port,
              uint16_t topicId, Time interval,
              uint32_t payloadSize, uint8_t qos);

  // Enable emergency detection (random abnormal values trigger emergency)
  void EnableEmergencyDetection (double probability);

private:
  virtual void StartApplication (void);
  virtual void StopApplication (void);

  void SendConnect (void);
  void SendPublish (void);
  void SendEmergency (void);
  void HandleRead (Ptr<Socket> socket);

  Ptr<Socket> m_socket;
  Address     m_gatewayAddr;
  uint16_t    m_port;
  uint16_t    m_topicId;
  Time        m_interval;
  uint32_t    m_payloadSize;
  uint8_t     m_qos;
  uint16_t    m_msgCounter;
  bool        m_running;
  EventId     m_sendEvent;

  // Emergency
  bool        m_emergencyEnabled;
  double      m_emergencyProb;
  Ptr<UniformRandomVariable> m_emergencyRng;

  // Stats
  uint32_t    m_txPackets;
  uint32_t    m_txBytes;
  uint32_t    m_emergencyCount;
};

} // namespace ns3
#endif
