/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * ns3-miot-simulation — Cluster-Based MIoT Comparison
 *
 * File:         mqtt-sn-broker.h
 * Description:  MQTT-SN broker — single-broker and two-tier hierarchy support.
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
#ifndef MQTT_SN_BROKER_H
#define MQTT_SN_BROKER_H

#include "ns3/application.h"
#include "ns3/socket.h"
#include "ns3/ipv4-address.h"
#include "mqtt-sn-header.h"
#include <map>

namespace ns3 {

struct BrokerStats
{
  uint32_t rxPackets       = 0;
  uint32_t rxBytes         = 0;
  uint32_t emergencies     = 0;
  uint32_t fromUpstream    = 0;  // from gateways (tier-1) or local brokers (root)
  uint32_t forwardedToRoot = 0;  // tier-1 only: packets forwarded up to root
  std::map<uint16_t, uint32_t> topicCounts;
  std::map<uint8_t,  uint32_t> priorityCounts;
};

class MqttSnBroker : public Application
{
public:
  static TypeId GetTypeId (void);
  MqttSnBroker ();
  virtual ~MqttSnBroker ();

  void Setup (uint16_t port);

  // Configure this as a tier-1 local broker that forwards to a root broker
  void SetRootBroker (Ipv4Address rootAddr, uint16_t rootPort);

  BrokerStats GetStats () const;
  void PrintStats () const;

private:
  virtual void StartApplication (void);
  virtual void StopApplication  (void);
  void HandleRead    (Ptr<Socket> socket);
  void ForwardToRoot (MqttSnHeader &hdr);  // no pkt param — rebuilds from hdr

  Ptr<Socket>  m_socket;
  Ptr<Socket>  m_rootSocket;  // send socket to root broker (tier-1 only)
  uint16_t     m_port;
  Ipv4Address  m_rootAddr;
  uint16_t     m_rootPort;
  bool         m_hasRoot;
  BrokerStats  m_stats;
};

} // namespace ns3
#endif
