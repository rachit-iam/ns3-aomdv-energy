/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 University of Washington
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

//
// This program configures a grid (default 5x5) of nodes on an
// 802.11b physical layer, with
// 802.11b NICs in adhoc mode, and by default, sends one packet of 1000
// (application) bytes to node 1.
//
// The default layout is like this, on a 2-D grid.
//
// n20  n21  n22  n23  n24
// n15  n16  n17  n18  n19
// n10  n11  n12  n13  n14
// n5   n6   n7   n8   n9
// n0   n1   n2   n3   n4
//
// the layout is affected by the parameters given to GridPositionAllocator;
// by default, GridWidth is 5 and numNodes is 25..
//
// There are a number of command-line options available to control
// the default behavior.  The list of available command-line options
// can be listed with the following command:
// ./waf --run "wifi-simple-adhoc-grid --help"
//
// Note that all ns-3 attributes (not just the ones exposed in the below
// script) can be changed at command line; see the ns-3 documentation.
//
// For instance, for this configuration, the physical layer will
// stop successfully receiving packets when distance increases beyond
// the default of 500m.
// To see this effect, try running:
//
// ./waf --run "wifi-simple-adhoc --distance=500"
// ./waf --run "wifi-simple-adhoc --distance=1000"
// ./waf --run "wifi-simple-adhoc --distance=1500"
//
// The source node and sink node can be changed like this:
//
// ./waf --run "wifi-simple-adhoc --sourceNode=20 --sinkNode=10"
//
// This script can also be helpful to put the Wifi layer into verbose
// logging mode; this command will turn on all wifi logging:
//
// ./waf --run "wifi-simple-adhoc-grid --verbose=1"
//
// By default, trace file writing is off-- to enable it, try:
// ./waf --run "wifi-simple-adhoc-grid --tracing=1"
//
// When you are done tracing, you will notice many pcap trace files
// in your directory.  If you have tcpdump installed, you can try this:
//
// tcpdump -r wifi-simple-adhoc-grid-0-0.pcap -nn -tt
//

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/aomdv-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/energy-module.h"
#include "ns3/wifi-radio-energy-model-helper.h"

using namespace ns3;

uint32_t receivedPacketCount = 0;
uint32_t sentPacketCount = 0;
NS_LOG_COMPONENT_DEFINE ("WifiSimpleAdhocGrid");
std::map <Ptr<Node> , uint32_t > nodes; 
std::vector <bool> active;

void ReceivePacket (Ptr<Socket> socket)
{
  while (socket->Recv ())
    {
      NS_LOG_UNCOND ("Received one packet! by "<<nodes[socket->GetNode()]<<" at "<< Simulator::Now());
      receivedPacketCount++;
    }
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      NS_LOG_UNCOND("Sending one packet from "<<nodes[socket->GetNode()]<<" at "<< Simulator::Now().GetSeconds());
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
      active[nodes[socket->GetNode()]] = 0;
    }
}

static void SendPackets(uint32_t sourceNode, uint32_t sinkNode, uint32_t pktSize,  uint32_t pktCount, NodeContainer c, Ipv4InterfaceContainer i){
        if(active[sinkNode] || active[sourceNode]) return ;
        TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
        Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (sinkNode), tid);
        Ipv4Address anyAd = Ipv4Address::GetAny();
        InetSocketAddress local = InetSocketAddress (anyAd, 80);
        recvSink->Bind (local);
        recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));  
        Ptr<Socket> source = Socket::CreateSocket (c.Get (sourceNode), tid);
        InetSocketAddress remote = InetSocketAddress (i.GetAddress (sinkNode, 0), 80);
        source->Connect (remote);
        active[sourceNode] = 1;
        Time pktInterval = Seconds(1.0);  
        sentPacketCount += pktCount;      
        Simulator::Schedule(pktInterval, &GenerateTraffic, source, pktSize, pktCount, pktInterval);
}

int randomInRange(int start =0, int end = 1000){
        int range = end-start+1;
        int result = rand()%range + start;
        return result;
}

int
main (int argc, char *argv[])
{
  std::string phyMode ("DsssRate1Mbps");
  double distance = 500; // m
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 50;
  uint32_t numNodes = 25; // by default, 5x5
  uint32_t sinkNode = 0;
  uint32_t sourceNode = 24;
  double interval = 1; // seconds
  bool verbose = false;
  bool tracing = true;

  uint32_t simulationDuration = 200;
  uint32_t numberOfPings = 2;

  CommandLine cmd;
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("distance", "distance (m)", distance);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("tracing", "turn on ascii and pcap tracing", tracing);
  cmd.AddValue ("numNodes", "number of nodes", numNodes);
  cmd.AddValue ("sinkNode", "Receiver node number", sinkNode);
  cmd.AddValue ("sourceNode", "Sender node number", sourceNode);
  cmd.Parse (argc, argv);
  // Convert to time object
  Time interPacketInterval = Seconds (interval);

  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (phyMode));

  NodeContainer c;
  c.Create (numNodes);

  for(uint32_t n=0; n<numNodes; n++){
        nodes[c.Get(n)] = n;
  }

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents (); // Turn on all Wifi logging
    }

  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  // set it to zero; otherwise, gain will be added
  wifiPhy.Set ("RxGain", DoubleValue (-10));
  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add an upper mac and disable rate control
  WifiMacHelper wifiMac;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (phyMode),
                                "ControlMode", StringValue (phyMode));
  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, c);

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator", "MinX", DoubleValue (0.0), "MinY",
                                 DoubleValue (0.0), "DeltaX", DoubleValue (distance), "DeltaY",
                                 DoubleValue (distance), "GridWidth", UintegerValue (5),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c);

  // Enable OLSR
  AomdvHelper aomdv;
  Ipv4StaticRoutingHelper staticRouting;

  /** Energy Model **/
  /***************************************************************************/
  /* energy source */
  BasicEnergySourceHelper basicSourceHelper;
  // configure energy source
  basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (200));
  // install source
  EnergySourceContainer sources = basicSourceHelper.Install (c);
  /* device energy model */
  WifiRadioEnergyModelHelper radioEnergyHelper;
  // configure radio energy model
  radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
  // install device model
  DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (devices, sources);
  /***************************************************************************/
  Ipv4ListRoutingHelper list;
  list.Add (aomdv, 10);

  InternetStackHelper internet;
  internet.SetRoutingHelper (list); // has effect on the next Install ()
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  // Start Application
 Ptr<UniformRandomVariable> xx = CreateObject<UniformRandomVariable> ();
 std::vector <Time> lastUsed(numNodes, Seconds(5));
 active.assign(numNodes,false);
 
  for(uint32_t loops=0; loops<numberOfPings;loops++){
        sinkNode = randomInRange(0,numNodes-1);
        sourceNode = randomInRange(0,numNodes-1);
        if(sinkNode==sourceNode) {
                loops--;
                continue;
        }

        Time sch = Max(lastUsed[sourceNode], lastUsed[sinkNode]) + Seconds(xx->GetValue(50.0, 70.0));
        lastUsed[sinkNode] =sch;
        lastUsed[sourceNode] = sch;
        if(sch > Seconds(simulationDuration  -2)) continue;
        NS_LOG_UNCOND("Sending "<<numPackets<<" packets from "<<sourceNode <<" to "<<sinkNode<<" at time "<<sch);
        Simulator::Schedule(sch, &SendPackets, sourceNode, sinkNode, packetSize, numPackets,  c, i);
        //Simulator::Schedule (Seconds (time), &GenerateTraffic,
                       //source, packetSize, numPackets, interPacketInterval);
  }

  if (tracing == true)
    {
      AsciiTraceHelper ascii;
      wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("wifi-simple-adhoc-grid.tr"));
      wifiPhy.EnablePcap ("wifi-simple-adhoc-grid", devices);
      // Trace routing tables
      Ptr<OutputStreamWrapper> routingStream =
          Create<OutputStreamWrapper> ("wifi-simple-adhoc-grid.routes", std::ios::out);
      aomdv.PrintRoutingTableAllEvery (Seconds (2), routingStream);
      Ptr<OutputStreamWrapper> neighborStream =
          Create<OutputStreamWrapper> ("wifi-simple-adhoc-grid.neighbors", std::ios::out);
      aomdv.PrintNeighborCacheAllEvery (Seconds (2), neighborStream);

      // To do-- enable an IP-level trace that shows forwarding events only
    }

  NS_LOG_UNCOND ("Testing from node " << sourceNode << " to " << sinkNode << " with grid distance "
                                      << distance);

  AnimationInterface anim ("animation.xml");
  Simulator::Stop (Seconds (simulationDuration));
  Simulator::Run ();
  Simulator::Destroy ();

  NS_LOG_UNCOND("sent:"<<sentPacketCount<<" , received:"<<receivedPacketCount);

  return 0;
}
