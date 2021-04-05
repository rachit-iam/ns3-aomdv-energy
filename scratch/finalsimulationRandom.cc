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
#include "ns3/mobility-module.h"
#include "ns3/aomdv-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/energy-module.h"
#include "ns3/wifi-radio-energy-model-helper.h"
#include "ns3/seq-ts-header.h"
#include <fstream>
using namespace ns3;
uint32_t currentSequenceNumber;
int64_t senttime ;
double bytesTotal ;
int64_t packetsReceived ;
int64_t rcv;
int64_t sqhd;
int64_t delay;
double xx;
double throughput ;
double remainingEnergy =0.0;
uint32_t numPackets = 2000;
uint32_t seed = 0; 
double Emin =1000.0;

NS_LOG_COMPONENT_DEFINE ("WifiSimpleAdhocGrid");

// Generate a random number in the range given inclusive
int randomInRange(int start =0, int end = 1000){
        int range = end-start+1;
        int result = rand()%range + start;
        return result;
}


void
ReceivePacket (Ptr<Socket> socket)
{
   //NS_LOG_UNCOND ("Received One packet!");
   Ptr<Packet> packet ;
   while(packet = socket->Recv())
    {
        rcv = Simulator::Now().GetNanoSeconds(); // time at the receive 
        SeqTsHeader seqTsx;
        packet->RemoveHeader (seqTsx);
        sqhd = seqTsx.GetTs().GetNanoSeconds();
        currentSequenceNumber = seqTsx.GetSeq ();
        bytesTotal += packet->GetSize ();
        packetsReceived += 1;
        // NS_LOG_UNCOND("For the received packet, Seq No = " << currentSequenceNumber << " ,timestamp on the packet = " << sqhd);//Just to check seq number and Tx time
        delay = delay +  (rcv - sqhd); //delay calculation
    }
    xx = rcv - senttime;    
    throughput = (bytesTotal * 8 * 1000000000) / (1024 * xx); 
}

static void
GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval)
{
  if (pktCount > 0)
    {
      SeqTsHeader seqTs;
      seqTs.SetSeq (pktCount);
      Ptr<Packet> p = Create<Packet> (pktSize-(8+4)); // 8+4 : the size of the seqTs header
      p->AddHeader (seqTs);
      socket->Send (p);
      //NS_LOG_UNCOND ("Sending "<< pktCount  << " packet! \n");   //pktCount is the sequence number 
      if(pktCount == numPackets)
      {
        senttime = Simulator::Now().GetNanoSeconds();
      }
      Simulator::Schedule (pktInterval, &GenerateTraffic, socket, pktSize, pktCount - 1,
                           pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

void 
EnergyCheck(NodeContainer c) {
    for(NodeContainer::Iterator i = c.Begin() ; i < c.End() ; i++){
        double e = ((*i)->GetObject<EnergySourceContainer>()->Get(0)->GetRemainingEnergy());
        Emin = std::min(Emin, e);
        remainingEnergy += e;
    }
    // NS_LOG_UNCOND("Total energy remaining " << Esum);
}

void PrintEnergyOfNode(NodeContainer c){
    for(NodeContainer::Iterator i = c.Begin() ; i < c.End() ; i++){
        double e = ((*i)->GetObject<EnergySourceContainer>()->Get(0)->GetRemainingEnergy());
        NS_LOG_UNCOND("Energy of node "<< *i<< " = " << e);
    }
    return ;
}

int
main (int argc, char *argv[])
{
  std::string phyMode ("DsssRate1Mbps");
  double distance = 10; // m
  uint32_t packetSize = 1000; // bytes
  //uint32_t numPackets = 1000; now defined as global variable for throughput calculations
  uint32_t numNodes = 25; // by default, 5x5
  
  // sourceNode and sinkNode are set after the commandline arguments are set
  uint32_t sourceNode=0;
  uint32_t sinkNode =numNodes-1;
  double interval = 0.1; // seconds
  bool verbose = false;
  bool tracing = true;

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
  cmd.AddValue ("seed", "Seed Value", seed);
  cmd.Parse (argc, argv);
  
  srand(seed); // setting the seed for rand() function in randomInRange function
  /* // to set random sourceNode and sinkNode
  sinkNode = randomInRange(0,numNodes-1);
  sourceNode = sinkNode;
  while(sourceNode==sinkNode){
        sourceNode = randomInRange(0,numNodes-1);
  }
  */
        

  // Convert to time object
  Time interPacketInterval = Seconds (interval);

  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (phyMode));

  NodeContainer c;
  c.Create (numNodes);

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents (); // Turn on all Wifi logging
    }

  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  // set it to zero; otherwise, gain will be added
  wifiPhy.Set ("RxGain", DoubleValue (-41));
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
   mobility.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
                                  "X", StringValue ("ns3::UniformRandomVariable[Min=0|Max=50]"),
                                  "Y", StringValue ("ns3::UniformRandomVariable[Min=0|Max=50]"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c);

  // Enable OLSR
  AomdvHelper aomdv;
  Ipv4StaticRoutingHelper staticRouting;

  /** Energy Model **/
  /***************************************************************************/
  /* energy source */
  // Setting different initial energy
  
  
  uint32_t minInitialEnergy = 300;
  uint32_t maxInitialEnergy = 500;  
  BasicEnergySourceHelper basicSourceHelper;
  std::map <Ptr<Node>, int> NodeInitialEnergyMap ; 
  for(NetDeviceContainer :: Iterator it = devices.Begin(); it!=devices.End(); it++){
        Ptr <Node> currentNode = (*it)->GetNode();
        int NodeEnergy = randomInRange(minInitialEnergy,maxInitialEnergy);
        NodeInitialEnergyMap.insert(std::make_pair(currentNode, NodeEnergy));
        basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (NodeEnergy));
        EnergySourceContainer sources = basicSourceHelper.Install (currentNode);
        WifiRadioEnergyModelHelper radioEnergyHelper;
        // configure radio energy model
        radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
        // install device model
        DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (*it, sources);
  }
  
  PrintEnergyOfNode(c);
  //NS_LOG_UNCOND("Printing NodeInitialEnergyMap");
  double totalInitialEnergy = 0.0;
  for(auto pp : NodeInitialEnergyMap){
        //NS_LOG_UNCOND("energy of node "<< pp.first << " = " << pp.second);
        totalInitialEnergy += pp.second;
  }

  Ipv4ListRoutingHelper list;
  list.Add (aomdv, 10);

  InternetStackHelper internet;
  internet.SetRoutingHelper (list); // has effect on the next Install ()
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (sinkNode), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  Ptr<Socket> source = Socket::CreateSocket (c.Get (sourceNode), tid);
  InetSocketAddress remote = InetSocketAddress (i.GetAddress (sinkNode, 0), 80);
  source->Connect (remote);

  if (tracing == true)
    {
      AsciiTraceHelper ascii;
      wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("wifi-simple-adhoc-grid.tr"));
      wifiPhy.EnablePcap ("wifi-simple-adhoc-grid", devices);
      // Trace routing tables
      Ptr<OutputStreamWrapper> routingStream =
          Create<OutputStreamWrapper> ("wifi-simple-adhoc-grid.routes", std::ios::out);
      aomdv.PrintRoutingTableAllEvery (Seconds (1), routingStream);
      Ptr<OutputStreamWrapper> neighborStream =
          Create<OutputStreamWrapper> ("wifi-simple-adhoc-grid.neighbors", std::ios::out);
      aomdv.PrintNeighborCacheAllEvery (Seconds (1), neighborStream);

      // To do-- enable an IP-level trace that shows forwarding events only
    }

  double stopTime = 250.0;
  Simulator::Schedule (Seconds (5.0), &GenerateTraffic, source, packetSize, numPackets,
                       interPacketInterval);
  Simulator::Schedule(Seconds(stopTime-20), &EnergyCheck, c);
  //Simulator::Schedule(Seconds (5.0) , &EnergyCheck, c);
  // Output what we are doing
  AnimationInterface anim ("animation.xml");
  Simulator::Stop (Seconds (stopTime));
  Simulator::Run ();
  Simulator::Destroy ();
  NS_LOG_UNCOND("\n\n\nSimulation Report");
  NS_LOG_UNCOND("Seed = "<<seed);
  NS_LOG_UNCOND("Source = " << sourceNode <<"\nSink = "<<sinkNode);
  NS_LOG_UNCOND("NumPackets = "<<numPackets);
  NS_LOG_UNCOND("Packet Loss = "<< numPackets-packetsReceived);
  NS_LOG_UNCOND("Total delay =  "<< delay/1000000 <<"ms ");
  NS_LOG_UNCOND("Average Delay = "<< ((double)delay/(double)packetsReceived)/(double)1000000.0 <<"ms");
  NS_LOG_UNCOND("Throughput = "<< throughput << "kbps");
  NS_LOG_UNCOND("Total initial energy provided = "<< totalInitialEnergy << "\nEnergy Remaining = "<< remainingEnergy);
  NS_LOG_UNCOND("Minimum of energy of all nodes = " << Emin);
  NS_LOG_UNCOND("Energy Consumption = "<< totalInitialEnergy-remainingEnergy);
  
  std::fstream fout;
  fout.open("outputSimulationRandom.csv", std::ios::out | std::ios::app);
  
  fout << sourceNode << ", "
       << sinkNode << ", "
       << numPackets << ", "
       << numPackets-packetsReceived << ", "
       << delay/1000000 << ", "
       << ((double)delay/(double)packetsReceived)/(double)1000000.0 << ", "
       << throughput << ", "
       << totalInitialEnergy << ", "
       << totalInitialEnergy-remainingEnergy << ", "
       << Emin 
       << "\n";
       
       
       
  return 0;
}
