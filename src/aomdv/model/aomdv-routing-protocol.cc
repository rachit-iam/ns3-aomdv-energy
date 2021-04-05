/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
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
 * Based on 
 *      NS-2 AOMDV model developed by the CMU/MONARCH group and optimized and
 *      tuned by Samir Das and Mahesh Marina, University of Cincinnati;
 * 
 *      AOMDV-UU implementation by Erik Nordström of Uppsala University
 *      http://core.it.uu.se/core/index.php/AOMDV-UU
 *
 * Authors: Elena Buchatskaia <borovkovaes@iitp.ru>
 *          Pavel Boyko <boyko@iitp.ru>
 */
#define NS_LOG_APPEND_CONTEXT                                   \
  if (m_ipv4) { std::clog << "[node " << m_ipv4->GetObject<Node> ()->GetId () << "] "; } 

#include "aomdv-routing-protocol.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/random-variable-stream.h"
#include "ns3/inet-socket-address.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/wifi-net-device.h"
#include "ns3/adhoc-wifi-mac.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include <algorithm>
#include <limits>
#include "ns3/udp-socket-factory.h"
#include "ns3/udp-l4-protocol.h"
#include "ns3/udp-header.h"
#include "ns3/energy-module.h"
#include "ns3/mobility-model.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE ("AomdvRoutingProtocol");

namespace aomdv
{
NS_OBJECT_ENSURE_REGISTERED (RoutingProtocol);

/// UDP Port for AOMDV control traffic
const uint32_t RoutingProtocol::AOMDV_PORT = 654;

//-----------------------------------------------------------------------------
/// Tag used by AOMDV implementation

class DeferredRouteOutputTag : public Tag
{

public:
  DeferredRouteOutputTag (int32_t o = -1) : Tag (), m_oif (o) {}

  static TypeId GetTypeId ()
  {
    static TypeId tid = TypeId ("ns3::aomdv::DeferredRouteOutputTag")
      .SetParent<Tag> ()
      .SetGroupName("Aomdv")
      .AddConstructor<DeferredRouteOutputTag> ()
    ;
    return tid;
  }

  TypeId  GetInstanceTypeId () const 
  {
    return GetTypeId ();
  }

  int32_t GetInterface() const
  {
    return m_oif;
  }

  void SetInterface(int32_t oif)
  {
    m_oif = oif;
  }

  uint32_t GetSerializedSize () const
  {
    return sizeof(int32_t);
  }

  void  Serialize (TagBuffer i) const
  {
    i.WriteU32 (m_oif);
  }

  void  Deserialize (TagBuffer i)
  {
    m_oif = i.ReadU32 ();
  }

  void  Print (std::ostream &os) const
  {
    os << "DeferredRouteOutputTag: output interface = " << m_oif;
  }

private:
  /// Positive if output device is fixed in RouteOutput
  int32_t m_oif;
};

NS_OBJECT_ENSURE_REGISTERED (DeferredRouteOutputTag);


//-----------------------------------------------------------------------------
RoutingProtocol::RoutingProtocol () :
  m_rreqRetries (1),
  m_ttlStart (2),
  m_ttlIncrement (2),
  m_ttlThreshold (8),
  m_timeoutBuffer (2),
  m_rreqRateLimit (10),
  m_rerrRateLimit (10),
  m_activeRouteTimeout (Time(Seconds (50))),
  m_netDiameter (10),
  m_nodeTraversalTime (MilliSeconds (40)),
  m_netTraversalTime (Time ((2 * m_netDiameter) * m_nodeTraversalTime)),
  m_pathDiscoveryTime ( Time (2 * m_netTraversalTime)),
  m_myRouteTimeout (Time (2 * std::max (m_pathDiscoveryTime, m_activeRouteTimeout))),
  m_helloInterval (Seconds (1)),
  m_allowedHelloLoss (2),
  m_deletePeriod (Time (5 * std::max (m_activeRouteTimeout, m_helloInterval))),
  m_nextHopWait (m_nodeTraversalTime + MilliSeconds (10)),
  m_blackListTimeout (Time (m_rreqRetries * m_netTraversalTime)),
  m_maxQueueLen (64),
  m_maxQueueTime (Seconds (30)),
  m_destinationOnly (true),
  m_gratuitousReply (true),
  m_enableHello (false),
  m_routingTable (m_deletePeriod),
  m_queue (m_maxQueueLen, m_maxQueueTime),
  m_requestId (0),
  m_seqNo (2),
  m_rreqIdCache (m_pathDiscoveryTime),
  m_dpd (m_pathDiscoveryTime),
  m_nb (m_helloInterval),
  m_rreqCount (0),
  m_rerrCount (0),
  m_htimer (Timer::CANCEL_ON_DESTROY),
  m_rreqRateLimitTimer (Timer::CANCEL_ON_DESTROY),
  m_rerrRateLimitTimer (Timer::CANCEL_ON_DESTROY),
  m_lastBcastTime (Seconds (0))
{
  //m_nb.SetCallback (MakeCallback (&RoutingProtocol::SendRerrWhenBreaksLinkToNextHop, this));
}

TypeId
RoutingProtocol::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::aomdv::RoutingProtocol")
    .SetParent<Ipv4RoutingProtocol> ()
    .SetGroupName("Aomdv")
    .AddConstructor<RoutingProtocol> ()
    .AddAttribute ("HelloInterval", "HELLO messages emission interval.",
                   TimeValue (Seconds (1)),
                   MakeTimeAccessor (&RoutingProtocol::m_helloInterval),
                   MakeTimeChecker ())
    .AddAttribute ("RreqRetries", "Maximum number of retransmissions of RREQ to discover a route",
                   UintegerValue (1),
                   MakeUintegerAccessor (&RoutingProtocol::m_rreqRetries),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("RreqRateLimit", "Maximum number of RREQ per second.",
                   UintegerValue (10),
                   MakeUintegerAccessor (&RoutingProtocol::m_rreqRateLimit),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("RerrRateLimit", "Maximum number of RERR per second.",
                   UintegerValue (10),
                   MakeUintegerAccessor (&RoutingProtocol::m_rerrRateLimit),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("NodeTraversalTime", "Conservative estimate of the average one hop traversal time for packets and should include "
                   "queuing delays, interrupt processing times and transfer times.",
                   TimeValue (MilliSeconds (40)),
                   MakeTimeAccessor (&RoutingProtocol::m_nodeTraversalTime),
                   MakeTimeChecker ())
    .AddAttribute ("NextHopWait", "Period of our waiting for the neighbour's RREP_ACK = 10 ms + NodeTraversalTime",
                   TimeValue (MilliSeconds (50)),
                   MakeTimeAccessor (&RoutingProtocol::m_nextHopWait),
                   MakeTimeChecker ())
    .AddAttribute ("ActiveRouteTimeout", "Period of time during which the route is considered to be valid",
                   TimeValue (Seconds (50)),
                   MakeTimeAccessor (&RoutingProtocol::m_activeRouteTimeout),
                   MakeTimeChecker ())
    .AddAttribute ("MyRouteTimeout", "Value of lifetime field in RREP generating by this node = 2 * max(ActiveRouteTimeout, PathDiscoveryTime)",
                   TimeValue (Seconds (11.2)),
                   MakeTimeAccessor (&RoutingProtocol::m_myRouteTimeout),
                   MakeTimeChecker ())
    .AddAttribute ("BlackListTimeout", "Time for which the node is put into the blacklist = RreqRetries * NetTraversalTime",
                   TimeValue (Seconds (5.6)),
                   MakeTimeAccessor (&RoutingProtocol::m_blackListTimeout),
                   MakeTimeChecker ())
    .AddAttribute ("DeletePeriod", "DeletePeriod is intended to provide an upper bound on the time for which an upstream node A "
                   "can have a neighbor B as an active next hop for destination D, while B has invalidated the route to D."
                   " = 5 * max (HelloInterval, ActiveRouteTimeout)",
                   TimeValue (Seconds (15)),
                   MakeTimeAccessor (&RoutingProtocol::m_deletePeriod),
                   MakeTimeChecker ())
    .AddAttribute ("NetDiameter", "Net diameter measures the maximum possible number of hops between two nodes in the network",
                   UintegerValue (10),
                   MakeUintegerAccessor (&RoutingProtocol::m_netDiameter),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("NetTraversalTime", "Estimate of the average net traversal time = 2 * NodeTraversalTime * NetDiameter",
                   TimeValue (Seconds (2.8)),
                   MakeTimeAccessor (&RoutingProtocol::m_netTraversalTime),
                   MakeTimeChecker ())
    .AddAttribute ("PathDiscoveryTime", "Estimate of maximum time needed to find route in network = 2 * NetTraversalTime",
                   TimeValue (Seconds (5.6)),
                   MakeTimeAccessor (&RoutingProtocol::m_pathDiscoveryTime),
                   MakeTimeChecker ())
    .AddAttribute ("MaxQueueLen", "Maximum number of packets that we allow a routing protocol to buffer.",
                   UintegerValue (64),
                   MakeUintegerAccessor (&RoutingProtocol::SetMaxQueueLen,
                                         &RoutingProtocol::GetMaxQueueLen),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("MaxQueueTime", "Maximum time packets can be queued (in seconds)",
                   TimeValue (Seconds (30)),
                   MakeTimeAccessor (&RoutingProtocol::SetMaxQueueTime,
                                     &RoutingProtocol::GetMaxQueueTime),
                   MakeTimeChecker ())
    .AddAttribute ("AllowedHelloLoss", "Number of hello messages which may be loss for valid link.",
                   UintegerValue (2),
                   MakeUintegerAccessor (&RoutingProtocol::m_allowedHelloLoss),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("GratuitousReply", "Indicates whether a gratuitous RREP should be unicast to the node originated route discovery.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RoutingProtocol::SetGratuitousReplyFlag,
                                        &RoutingProtocol::GetGratuitousReplyFlag),
                   MakeBooleanChecker ())
    .AddAttribute ("DestinationOnly", "Indicates only the destination may respond to this RREQ.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&RoutingProtocol::SetDesinationOnlyFlag,
                                        &RoutingProtocol::GetDesinationOnlyFlag),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableHello", "Indicates whether a hello messages enable.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RoutingProtocol::SetHelloEnable,
                                        &RoutingProtocol::GetHelloEnable),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableBroadcast", "Indicates whether a broadcast data packets forwarding enable.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RoutingProtocol::SetBroadcastEnable,
                                        &RoutingProtocol::GetBroadcastEnable),
                   MakeBooleanChecker ())
    .AddAttribute ("UniformRv",
                   "Access to the underlying UniformRandomVariable",
                   StringValue ("ns3::UniformRandomVariable"),
                   MakePointerAccessor (&RoutingProtocol::m_uniformRandomVariable),
                   MakePointerChecker<UniformRandomVariable> ())
  ;
  return tid;
}

void
RoutingProtocol::SetMaxQueueLen (uint32_t len)
{
  m_maxQueueLen = len;
  m_activeRouteTimeout = Seconds(50);// i dont kno why(as the values are set once again in this AddAttribute) i ve to cheange this here, but when i was changing in the original position it was not working
  m_rreqRetries = 1;
  m_netDiameter = 10;
  m_ttlThreshold = 8;
  m_queue.SetMaxQueueLen (len);
}
void
RoutingProtocol::SetMaxQueueTime (Time t)
{
  m_maxQueueTime = t;
  m_queue.SetQueueTimeout (t);
}

RoutingProtocol::~RoutingProtocol ()
{
}

void
RoutingProtocol::DoDispose ()
{
  m_ipv4 = 0;
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::iterator iter =
         m_socketAddresses.begin (); iter != m_socketAddresses.end (); iter++)
    {
      iter->first->Close ();
    }
  m_socketAddresses.clear ();
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::iterator iter =
         m_socketSubnetBroadcastAddresses.begin (); iter != m_socketSubnetBroadcastAddresses.end (); iter++)
    {
      iter->first->Close ();
    }
  m_socketSubnetBroadcastAddresses.clear ();
  Ipv4RoutingProtocol::DoDispose ();
}

void
RoutingProtocol::PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit) const
{
  *stream->GetStream () << "Node: " << m_ipv4->GetObject<Node> ()->GetId () << " Time: " << Simulator::Now ().GetSeconds () << "s ";
  m_routingTable.Print (stream);
}

int64_t
RoutingProtocol::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream (stream);
  return 1;
}

void
RoutingProtocol::Start ()
{
  NS_LOG_FUNCTION (this);
  if (m_enableHello)
    {
      m_nb.ScheduleTimer ();
    }
  m_rreqRateLimitTimer.SetFunction (&RoutingProtocol::RreqRateLimitTimerExpire,
                                    this);
  m_rreqRateLimitTimer.Schedule (Seconds (1));

  m_rerrRateLimitTimer.SetFunction (&RoutingProtocol::RerrRateLimitTimerExpire,
                                    this);
  m_rerrRateLimitTimer.Schedule (Seconds (1));

}

Ptr<Ipv4Route>
RoutingProtocol::RouteOutput (Ptr<Packet> p, const Ipv4Header &header,
                              Ptr<NetDevice> oif, Socket::SocketErrno &sockerr)
{
  //NS_LOG_UNCOND (this << header << (oif ? oif->GetIfIndex () : 0));
  //dev-rachit
  if (!p)
    {
      NS_LOG_DEBUG("Packet is == 0");
      return LoopbackRoute (header, oif); // later
    }
  if (m_socketAddresses.empty ())
    {
      sockerr = Socket::ERROR_NOROUTETOHOST;
      NS_LOG_LOGIC ("No aomdv interfaces");
      Ptr<Ipv4Route> route;
      return route;
    }
  
  //NS_LOG_UNCOND("CHECK");
  sockerr = Socket::ERROR_NOTERROR;
  Ptr<Ipv4Route> route;
  Ipv4Address dst = header.GetDestination ();
  RoutingTableEntry rt;
  if (m_routingTable.LookupValidRoute (dst, rt))//todo check path availabilty or just do smth abt this
    {
      RoutingTableEntry::Path *path = rt.PathLoadBalancedFind ();//todo
      // NS_LOG_UNCOND("GET ROUTE BUG desination = " << dst);
      // NS_LOG_UNCOND("GET ROUTE BUG route destination = " << rt.GetNumberofPaths());
      // NS_LOG_UNCOND("GET ROUTE BUG path next hop = " << path->GetNextHop());
      route = path->GetRoute ();
      //NS_LOG_UNCOND("GET ROUTE BUG - destination = " << route->GetDestination() << " gateway = " << route->GetGateway());
      NS_ASSERT (route != 0);
      NS_LOG_DEBUG ("Exist route to " << route->GetDestination () << " from interface " << route->GetSource ());
      if (oif != 0 && route->GetOutputDevice () != oif)
        {
          NS_LOG_DEBUG ("Output device doesn't match. Dropped.");
          sockerr = Socket::ERROR_NOROUTETOHOST;
          return Ptr<Ipv4Route> ();
        }
      
      UpdatePathsLifeTime (route->GetGateway (), m_activeRouteTimeout);
      path->SetExpire (std::max (m_activeRouteTimeout, path->GetExpire ()));
      m_routingTable.Update(rt);
      UpdateRouteLifeTime (dst, m_activeRouteTimeout);
      //NS_LOG_UNCOND("PATH EXPIRY expected = " << path->GetExpire().GetSeconds() << " real = " << rt.PathLoadBalancedFind ()->GetExpire().GetSeconds());
      //m_routingTable.Update(rt);//change 3/1/2021
      return route;
    }

  // Valid route not found, in this case we return loopback. 
  // Actual route request will be deferred until packet will be fully formed, 
  // routed to loopback, received from loopback and passed to RouteInput (see below)
  uint32_t iif = (oif ? m_ipv4->GetInterfaceForDevice (oif) : -1);
  DeferredRouteOutputTag tag (iif);
  //NS_LOG_UNCOND ("Valid Route not found");
  if (!p->PeekPacketTag (tag))
    {
      p->AddPacketTag (tag);
    }
  return LoopbackRoute (header, oif);
}

void
RoutingProtocol::DeferredRouteOutput (Ptr<const Packet> p, const Ipv4Header & header, 
                                      UnicastForwardCallback ucb, ErrorCallback ecb)
{
  NS_LOG_FUNCTION (this << p << header);
  NS_ASSERT (p != 0 && p != Ptr<Packet> ());

  QueueEntry newEntry (p, header, ucb, ecb);
  bool result = m_queue.Enqueue (newEntry);
  if (result)
    {
      NS_LOG_LOGIC ("Add packet " << p->GetUid () << " to queue. Protocol " << (uint16_t) header.GetProtocol ());
      RoutingTableEntry rt;
      bool result = m_routingTable.LookupRoute (header.GetDestination (), rt);
      if(!result || ((rt.GetFlag () != IN_SEARCH) && result))
        {
          NS_LOG_LOGIC ("Send new RREQ for outbound packet to " <<header.GetDestination ());
          SendRequest (header.GetDestination ());
        }
    }
}

bool
RoutingProtocol::RouteInput (Ptr<const Packet> p, const Ipv4Header &header,
                             Ptr<const NetDevice> idev, UnicastForwardCallback ucb,
                             MulticastForwardCallback mcb, LocalDeliverCallback lcb, ErrorCallback ecb)
{
  //NS_LOG_UNCOND (this << p->GetUid () << header.GetDestination () << idev->GetAddress ());
  if (m_socketAddresses.empty ())
    {
      NS_LOG_LOGIC ("No aomdv interfaces");
      return false;
    }
  NS_ASSERT (m_ipv4 != 0);
  NS_ASSERT (p != 0);
  // Check if input device supports IP
  NS_ASSERT (m_ipv4->GetInterfaceForDevice (idev) >= 0);
  int32_t iif = m_ipv4->GetInterfaceForDevice (idev);

  Ipv4Address dst = header.GetDestination ();
  Ipv4Address origin = header.GetSource ();

  // Deferred route request
  if (idev == m_lo)
    {
      DeferredRouteOutputTag tag;
      if (p->PeekPacketTag (tag))
        {
          DeferredRouteOutput (p, header, ucb, ecb);
          return true;
        }
    }

  // Duplicate of own packet
  if (IsMyOwnAddress (origin))
    return true;

  // AOMDV is not a multicast routing protocol
  if (dst.IsMulticast ())
    {
      return false; 
    }
  // Broadcast local delivery/forwarding
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin ();
       j != m_socketAddresses.end (); ++j)
    {
      Ipv4InterfaceAddress iface = j->second;
      if (m_ipv4->GetInterfaceForAddress (iface.GetLocal ()) == iif)
        {
          if (dst == iface.GetBroadcast () || dst.IsBroadcast ())
            {
              if (m_dpd.IsDuplicate (p, header))
                {
                  NS_LOG_DEBUG ("Duplicated packet " << p->GetUid () << " from " << origin
                                                     << ". Drop.");
                  return true;
                }
              UpdateRouteLifeTime (origin, m_activeRouteTimeout);
              UpdatePathsLifeTime (origin, m_activeRouteTimeout);
              Ptr<Packet> packet = p->Copy ();
              if (lcb.IsNull () == false)
                {
                  NS_LOG_LOGIC ("Broadcast local delivery to " << iface.GetLocal ());
                  lcb (p, header, iif);
                  // Fall through to additional processing
                }
              else
                {
                  NS_LOG_ERROR ("Unable to deliver packet locally due to null callback "
                                << p->GetUid () << " from " << origin);
                  ecb (p, header, Socket::ERROR_NOROUTETOHOST);
                }
              if (!m_enableBroadcast)
                {
                  return true;
                }
              if (header.GetProtocol () == UdpL4Protocol::PROT_NUMBER)
                {
                  UdpHeader udpHeader;
                  p->PeekHeader(udpHeader);
                  if (udpHeader.GetDestinationPort () == AOMDV_PORT)
                    {
                      return true;
                    }
                }
              if (header.GetTtl () > 1)
                {
                  NS_LOG_LOGIC ("Forward broadcast. TTL " << (uint16_t) header.GetTtl ());
                  RoutingTableEntry toBroadcast;
                  if (m_routingTable.LookupRoute (dst, toBroadcast) && toBroadcast.GetNumberofPaths ())
                    {
                      RoutingTableEntry::Path *path = toBroadcast.PathFind ();
                      Ptr<Ipv4Route> route = path->GetRoute ();
                      ucb (route, packet, header);
                    }
                  else
                    {
                      NS_LOG_DEBUG ("No route to forward broadcast. Drop packet " << p->GetUid ());
                    }
                }
              else
                {
                  NS_LOG_DEBUG ("TTL exceeded. Drop packet " << p->GetUid ());
                }
              return true;
            }
        }
    }

  // Unicast local delivery
  if (m_ipv4->IsDestinationAddress (dst, iif))
    {
      UpdatePathsLifeTime (origin, m_activeRouteTimeout);
      UpdateRouteLifeTime (origin, m_activeRouteTimeout);
      RoutingTableEntry toOrigin;
      if (m_routingTable.LookupValidRoute (origin, toOrigin))
        {
          UpdatePathsLifeTime (toOrigin.PathFind ()->GetNextHop (), m_activeRouteTimeout);
          UpdateRouteLifeTime (origin, m_activeRouteTimeout);
          m_nb.Update (toOrigin.PathFind ()->GetNextHop (), m_activeRouteTimeout);//todo only pathfins are updated
        }
      if (lcb.IsNull () == false)
        {
          NS_LOG_LOGIC ("Unicast local delivery to " << dst);
          
          lcb (p, header, iif);
        }
      else
        {
          NS_LOG_ERROR ("Unable to deliver packet locally due to null callback " << p->GetUid () << " from " << origin);
          ecb (p, header, Socket::ERROR_NOROUTETOHOST);
        }
      return true;
    }

  // Forwarding
  //energy example
  //NS_LOG_UNCOND("Node " << m_ipv4->GetObject<Node> ()->GetId () << "has energy remaining = " << m_ipv4->GetObject<Node> ()->GetObject<EnergySourceContainer>()->Get(0)->GetRemainingEnergy());
  //NS_LOG_UNCOND("check route input packet id = " << p->GetUid() << "current node = " << m_ipv4->GetAddress (1, 0).GetLocal ()<< "at " << Simulator::Now().GetSeconds());
  return Forwarding (p, header, ucb, ecb);
}

bool
RoutingProtocol::Forwarding (Ptr<const Packet> p, const Ipv4Header & header,
                             UnicastForwardCallback ucb, ErrorCallback ecb)
{
  NS_LOG_FUNCTION (this);
  Ipv4Address dst = header.GetDestination ();
  Ipv4Address origin = header.GetSource ();
  m_routingTable.Purge ();
  //NS_LOG_UNCOND("YO SOURCE = " << origin << ", dest = " << dst << ", mynode = " << m_ipv4->GetAddress(1,0).GetLocal());
  RoutingTableEntry toDst;
  if (m_routingTable.LookupRoute (dst, toDst))
    {
      if (toDst.GetFlag () == VALID && toDst.GetNumberofPaths ())
        {
          RoutingTableEntry::Path *path = toDst.PathFind ();
          Ptr<Ipv4Route> route = path->GetRoute ();
          NS_LOG_LOGIC (route->GetSource ()<<" forwarding to " << dst << " from " << origin << " packet " << p->GetUid ());

          /*
           *  Each time a route is used to forward a data packet, its Active Route
           *  Lifetime field of the source, destination and the next hop on the
           *  path to the destination is updated to be no less than the current
           *  time plus ActiveRouteTimeout.
           */
          UpdatePathsLifeTime (origin, m_activeRouteTimeout);
          UpdateRouteLifeTime (dst, m_activeRouteTimeout);
          path->SetExpire (std::max (m_activeRouteTimeout, path->GetExpire ()));
          UpdatePathsLifeTime (route->GetGateway (), m_activeRouteTimeout);
          /*
           *  Since the route between each originator and destination pair is expected to be symmetric, the
           *  Active Route Lifetime for the previous hop, along the reverse path back to the IP source, is also updated
           *  to be no less than the current time plus ActiveRouteTimeout
           */
          RoutingTableEntry toOrigin;
          m_routingTable.LookupRoute (origin, toOrigin);
          //NS_LOG_UNCOND(toOrigin.GetLifeTime() << " " << );
          if(toOrigin.GetNumberofPaths())UpdatePathsLifeTime (toOrigin.PathFind ()->GetNextHop (), m_activeRouteTimeout);
          //todo change if(toOrigin.GetNumberofPaths()) dont know why 
          m_nb.Update (route->GetGateway (), m_activeRouteTimeout);
          if(toOrigin.GetNumberofPaths())m_nb.Update (toOrigin.PathFind ()->GetNextHop (), m_activeRouteTimeout);//todo only one is updating

          ucb (route, p, header);
          return true;
        }
      else
        {
          if (toDst.GetValidSeqNo ())
            {
              SendRerrWhenNoRouteToForward (dst, toDst.GetSeqNo (), origin);
              NS_LOG_UNCOND ("Drop packet " << p->GetUid () << " because no route to forward it.");
              return false;
            }
        }
    }
  NS_LOG_UNCOND ("route not found to "<< dst << ". Send RERR message.");
  NS_LOG_UNCOND ("Drop packet " << p->GetUid () << " because no route to forward it 2.");
  SendRerrWhenNoRouteToForward (dst, 0, origin);
  return false;
}

void
RoutingProtocol::SetIpv4 (Ptr<Ipv4> ipv4)
{
  NS_ASSERT (ipv4 != 0);
  NS_ASSERT (m_ipv4 == 0);

  m_ipv4 = ipv4;

  // Create lo route. It is asserted that the only one interface up for now is loopback
  NS_ASSERT (m_ipv4->GetNInterfaces () == 1 && m_ipv4->GetAddress (0, 0).GetLocal () == Ipv4Address ("127.0.0.1"));
  m_lo = m_ipv4->GetNetDevice (0);
  NS_ASSERT (m_lo != 0);
  // Remember lo route
  RoutingTableEntry rt (/*dst=*/ Ipv4Address::GetLoopback (), /*validSeqNo=*/ true, /*seqno=*/ 0, 
                              /*lifeTime=*/ Simulator::GetMaximumSimulationTime ());
  rt.PathInsert (/*device=*/ m_lo, /*nextHop=*/ Ipv4Address::GetLoopback (), /*hop=*/ 1, 
                 /*expireTime=*/ Simulator::GetMaximumSimulationTime (), 
                 /*lastHop=*/ Ipv4Address::GetLoopback (), 
                 /*iface=*/ Ipv4InterfaceAddress (Ipv4Address::GetLoopback (), Ipv4Mask ("255.0.0.0")),
                 /*MRE=*/INFINITY3, /*distance squared*/ 0, 0);
  m_routingTable.AddRoute (rt);
  Simulator::ScheduleNow (&RoutingProtocol::Start, this);
}

void
RoutingProtocol::NotifyInterfaceUp (uint32_t i)
{
  NS_LOG_FUNCTION (this << m_ipv4->GetAddress (i, 0).GetLocal ());
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
  
  if (l3->GetNAddresses (i) > 1)
    {
      NS_LOG_WARN ("AOMDV does not work with more then one address per each interface.");
    }
  Ipv4InterfaceAddress iface = l3->GetAddress (i, 0);
  if (iface.GetLocal () == Ipv4Address ("127.0.0.1"))
    return;
 
  // Create a socket to listen only on this interface
  Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (),
                                             UdpSocketFactory::GetTypeId ());
  NS_ASSERT (socket != 0);
  socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvAomdv, this));
  socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), AOMDV_PORT));

  socket->BindToNetDevice (l3->GetNetDevice (i));
  socket->SetAllowBroadcast (true);
  socket->SetIpRecvTtl (true);
  m_socketAddresses.insert (std::make_pair (socket, iface));

  // create also a subnet broadcast socket
  socket = Socket::CreateSocket (GetObject<Node> (),
                                 UdpSocketFactory::GetTypeId ());
  NS_ASSERT (socket != 0);
  socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvAomdv, this));
  socket->Bind (InetSocketAddress (iface.GetBroadcast (), AOMDV_PORT));
  socket->BindToNetDevice (l3->GetNetDevice (i));
  socket->SetAllowBroadcast (true);
  socket->SetIpRecvTtl (true);
  m_socketSubnetBroadcastAddresses.insert (std::make_pair (socket, iface));

  // Add local broadcast record to the routing table
  Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (iface.GetLocal ()));
  RoutingTableEntry rt (/*dst=*/ iface.GetBroadcast (), /*validSeqNo=*/ true, /*seqno=*/ 0, 
                              /*lifeTime=*/ Simulator::GetMaximumSimulationTime ());
  
  rt.PathInsert (/*device=*/ dev, /*nextHop=*/ iface.GetBroadcast (), /*hop=*/ 1, 
                 /*expireTime=*/ Simulator::GetMaximumSimulationTime (), 
                 /*lastHop=*/ iface.GetBroadcast (), 
                 /*iface=*/ iface, /*MRE=*/INFINITY3, /*squaredDistance*/0, /*delay*/0);
  m_routingTable.AddRoute (rt);
  //RoutingTableEntry rt1;
  // m_routingTable.LookupValidRoute(iface.GetBroadcast (), rt1);
  // NS_LOG_UNCOND("IMP CHECK PATH hopcount = " << (int)(rt1.PathFind())->GetHopCount());
  // NS_LOG_UNCOND("imp CHECK" << rt.GetDestination());
  // NS_LOG_UNCOND("imp CHECK1" << rt1.GetDestination()); debug
  if (l3->GetInterface (i)->GetArpCache ())
    {
      m_nb.AddArpCache (l3->GetInterface (i)->GetArpCache ());
    }

  // Allow neighbor manager use this interface for layer 2 feedback if possible
  Ptr<WifiNetDevice> wifi = dev->GetObject<WifiNetDevice> ();
  if (wifi == 0)
    return;
  Ptr<WifiMac> mac = wifi->GetMac ();
  if (mac == 0)
    return;

  mac->TraceConnectWithoutContext ("TxErrHeader", m_nb.GetTxErrorCallback ());
}

void
RoutingProtocol::NotifyInterfaceDown (uint32_t i)
{
  NS_LOG_FUNCTION (this << m_ipv4->GetAddress (i, 0).GetLocal ());

  // Disable layer 2 link state monitoring (if possible)
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
  Ptr<NetDevice> dev = l3->GetNetDevice (i);
  Ptr<WifiNetDevice> wifi = dev->GetObject<WifiNetDevice> ();
  if (wifi != 0)
    {
      Ptr<WifiMac> mac = wifi->GetMac ()->GetObject<AdhocWifiMac> ();
      if (mac != 0)
        {
          mac->TraceDisconnectWithoutContext ("TxErrHeader",
                                              m_nb.GetTxErrorCallback ());
          m_nb.DelArpCache (l3->GetInterface (i)->GetArpCache ());
        }
    }

  // Close socket 
  Ptr<Socket> socket = FindSocketWithInterfaceAddress (m_ipv4->GetAddress (i, 0));
  NS_ASSERT (socket);
  socket->Close ();
  m_socketAddresses.erase (socket);

  // Close socket
  socket = FindSubnetBroadcastSocketWithInterfaceAddress (m_ipv4->GetAddress (i, 0));
  NS_ASSERT (socket);
  socket->Close ();
  m_socketSubnetBroadcastAddresses.erase (socket);

  if (m_socketAddresses.empty ())
    {
      NS_LOG_LOGIC ("No aomdv interfaces");
      m_htimer.Cancel ();
      m_nb.Clear ();
      m_routingTable.Clear ();
      return;
    }
  m_routingTable.DeleteAllRoutesFromInterface (m_ipv4->GetAddress (i, 0));
}

void
RoutingProtocol::NotifyAddAddress (uint32_t i, Ipv4InterfaceAddress address)
{
  NS_LOG_FUNCTION (this << " interface " << i << " address " << address);
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
  if (!l3->IsUp (i))
    return;
  if (l3->GetNAddresses (i) == 1)
    {
      Ipv4InterfaceAddress iface = l3->GetAddress (i, 0);
      Ptr<Socket> socket = FindSocketWithInterfaceAddress (iface);
      if (!socket)
        {
          if (iface.GetLocal () == Ipv4Address ("127.0.0.1"))
            return;
          // Create a socket to listen only on this interface
          Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (),
                                                     UdpSocketFactory::GetTypeId ());
          NS_ASSERT (socket != 0);
          socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvAomdv,this));
          socket->Bind (InetSocketAddress (iface.GetLocal (), AOMDV_PORT));
          socket->BindToNetDevice (l3->GetNetDevice (i));
          socket->SetAllowBroadcast (true);
          socket->SetIpRecvTtl (true);
          m_socketAddresses.insert (std::make_pair (socket, iface));

          // create also a subnet directed broadcast socket
          socket = Socket::CreateSocket (GetObject<Node> (),
                                                       UdpSocketFactory::GetTypeId ());
          NS_ASSERT (socket != 0);
          socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvAomdv, this));
          socket->Bind (InetSocketAddress (iface.GetBroadcast (), AOMDV_PORT));
          socket->BindToNetDevice (l3->GetNetDevice (i));
          socket->SetAllowBroadcast (true);
          socket->SetIpRecvTtl (true);
          m_socketSubnetBroadcastAddresses.insert (std::make_pair (socket, iface));

          // Add local broadcast record to the routing table
          Ptr<NetDevice> dev = m_ipv4->GetNetDevice (
              m_ipv4->GetInterfaceForAddress (iface.GetLocal ()));
          RoutingTableEntry rt (/*dst=*/ iface.GetBroadcast (), /*validSeqNo=*/ true, /*seqno=*/ 0, 
                              /*lifeTime=*/ Simulator::GetMaximumSimulationTime ());
          rt.PathInsert (/*device=*/ dev, /*nextHop=*/ iface.GetBroadcast (), /*hop=*/ 1, 
                         /*expireTime=*/ Simulator::GetMaximumSimulationTime (), 
                         /*lastHop=*/ iface.GetBroadcast (), 
                         /*iface=*/ iface, /*MRE=*/INFINITY3, /*distanceSquared*/0, 0);
          m_routingTable.AddRoute (rt);
        }
    }
  else
    {
      NS_LOG_LOGIC ("AOMDV does not work with more then one address per each interface. Ignore added address");
    }
}

void
RoutingProtocol::NotifyRemoveAddress (uint32_t i, Ipv4InterfaceAddress address)
{
  NS_LOG_FUNCTION (this);
  Ptr<Socket> socket = FindSocketWithInterfaceAddress (address);
  if (socket)
    {
      m_routingTable.DeleteAllRoutesFromInterface (address);
      socket->Close ();
      m_socketAddresses.erase (socket);

      Ptr<Socket> unicastSocket = FindSubnetBroadcastSocketWithInterfaceAddress (address);
      if (unicastSocket)
        {
          unicastSocket->Close ();
          m_socketAddresses.erase (unicastSocket);
        }

      Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
      if (l3->GetNAddresses (i))
        {
          Ipv4InterfaceAddress iface = l3->GetAddress (i, 0);
          // Create a socket to listen only on this interface
          Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (),
                                                     UdpSocketFactory::GetTypeId ());
          NS_ASSERT (socket != 0);
          socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvAomdv, this));
          // Bind to any IP address so that broadcasts can be received
          socket->Bind (InetSocketAddress (iface.GetLocal (), AOMDV_PORT));
          socket->BindToNetDevice (l3->GetNetDevice (i));
          socket->SetAllowBroadcast (true);
          socket->SetIpRecvTtl (true);
          m_socketAddresses.insert (std::make_pair (socket, iface));

          // create also a unicast socket
          socket = Socket::CreateSocket (GetObject<Node> (),
                                                       UdpSocketFactory::GetTypeId ());
          NS_ASSERT (socket != 0);
          socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvAomdv, this));
          socket->Bind (InetSocketAddress (iface.GetBroadcast (), AOMDV_PORT));
          socket->BindToNetDevice (l3->GetNetDevice (i));
          socket->SetAllowBroadcast (true);
          socket->SetIpRecvTtl (true);
          m_socketSubnetBroadcastAddresses.insert (std::make_pair (socket, iface));

          // Add local broadcast record to the routing table
          Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (iface.GetLocal ()));
          RoutingTableEntry rt (/*dst=*/ iface.GetBroadcast (), /*validSeqNo=*/ true, /*seqno=*/ 0, 
                              /*lifeTime=*/ Simulator::GetMaximumSimulationTime ());
          rt.PathInsert (/*device=*/ dev, /*nextHop=*/ iface.GetBroadcast (), /*hop=*/ 1, 
                         /*expireTime=*/ Simulator::GetMaximumSimulationTime (), 
                         /*lastHop=*/ iface.GetBroadcast (), 
                         /*iface=*/ iface, /*MRE=*/INFINITY3, /*squaredDistance*/ 0, /*delay*/ 0);
          m_routingTable.AddRoute (rt);
        }
      if (m_socketAddresses.empty ())
        {
          NS_LOG_LOGIC ("No aomdv interfaces");
          m_htimer.Cancel ();
          m_nb.Clear ();
          m_routingTable.Clear ();
          return;
        }
    }
  else
    {
      NS_LOG_LOGIC ("Remove address not participating in AOMDV operation");
    }
}

uint32_t
RoutingProtocol::GetRemainingEnergy(void) 
{
  return (uint32_t)((m_ipv4->GetObject<Node> ()->GetObject<EnergySourceContainer>()->Get(0)->GetRemainingEnergy())*10000);
}

std::pair<uint32_t, uint32_t>
RoutingProtocol::GetPosition(void) 
{
  Vector v = m_ipv4->GetObject<Node> ()->GetObject<MobilityModel> () ->GetPosition ();
  return std::pair<u_int32_t, u_int32_t>( {v.x, v.y});
}


bool
RoutingProtocol::IsMyOwnAddress (Ipv4Address src)
{
  NS_LOG_FUNCTION (this << src);
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
         m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
    {
      Ipv4InterfaceAddress iface = j->second;
      if (src == iface.GetLocal ())
        {
          return true;
        }
    }
  return false;
}

Ptr<Ipv4Route> 
RoutingProtocol::LoopbackRoute (const Ipv4Header & hdr, Ptr<NetDevice> oif) const
{
  NS_LOG_FUNCTION (this << hdr);
  NS_ASSERT (m_lo != 0);
  Ptr<Ipv4Route> rt = Create<Ipv4Route> ();
  rt->SetDestination (hdr.GetDestination ());
  //
  // Source address selection here is tricky.  The loopback route is
  // returned when AOMDV does not have a route; this causes the packet
  // to be looped back and handled (cached) in RouteInput() method
  // while a route is found. However, connection-oriented protocols
  // like TCP need to create an endpoint four-tuple (src, src port,
  // dst, dst port) and create a pseudo-header for checksumming.  So,
  // AOMDV needs to guess correctly what the eventual source address
  // will be.
  //
  // For single interface, single address nodes, this is not a problem.
  // When there are possibly multiple outgoing interfaces, the policy
  // implemented here is to pick the first available AOMDV interface.
  // If RouteOutput() caller specified an outgoing interface, that 
  // further constrains the selection of source address
  //
  std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin ();
  if (oif)
    {
      // Iterate to find an address on the oif device
      for (j = m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
        {
          Ipv4Address addr = j->second.GetLocal ();
          int32_t interface = m_ipv4->GetInterfaceForAddress (addr);
          if (oif == m_ipv4->GetNetDevice (static_cast<uint32_t> (interface)))
            {
              rt->SetSource (addr);
              break;
            }
        }
    }
  else
    {
      rt->SetSource (j->second.GetLocal ());
    }
  NS_ASSERT_MSG (rt->GetSource () != Ipv4Address (), "Valid AOMDV source address not found");
  rt->SetGateway (Ipv4Address ("127.0.0.1"));
  rt->SetOutputDevice (m_lo);
  return rt;
}

void
RoutingProtocol::SendRequest (Ipv4Address dst)
{
  NS_LOG_FUNCTION ( this << dst);
  // A node SHOULD NOT originate more than RREQ_RATELIMIT RREQ messages per second.
  if (m_rreqCount == m_rreqRateLimit)
    {
      Simulator::Schedule (m_rreqRateLimitTimer.GetDelayLeft () + MicroSeconds (100),
                           &RoutingProtocol::SendRequest, this, dst);
      return;
    }
  else
    m_rreqCount++;
  // Create RREQ header
  RreqHeader rreqHeader;
  rreqHeader.SetDst (dst);
  RoutingTableEntry rt;
  // Using the Hop field in Routing Table to manage the expanding ring search
  uint16_t ttl = m_ttlStart;
  if (m_routingTable.LookupRoute (dst, rt))
    {
      if (rt.GetFlag () != IN_SEARCH)
        {
          ttl = std::min<uint16_t> (rt.GetLastHopCount () + m_ttlIncrement, m_netDiameter);
        }
      else
        {
          ttl = rt.GetLastHopCount () + m_ttlIncrement;
          if (ttl > m_ttlThreshold)
            ttl = m_netDiameter;
        }
      if (ttl == m_netDiameter)
        rt.IncrementRreqCnt ();
      if (rt.GetValidSeqNo ())
        rreqHeader.SetDstSeqno (rt.GetSeqNo ());
      else
        rreqHeader.SetUnknownSeqno (true);
      rt.SetLastHopCount (ttl);
      rt.SetFlag (IN_SEARCH);
      rt.SetLifeTime (std::max(m_pathDiscoveryTime, rt.GetLifeTime ()));
      if(rt.GetFlag () == IN_SEARCH && rt.GetNumberofPaths ()){
        rreqHeader.SetRReq (true);
      }
      m_routingTable.Update (rt);
    }
  else
    {
      rreqHeader.SetUnknownSeqno (true);
      Ptr<NetDevice> dev = 0;
      RoutingTableEntry newEntry (/*dst=*/ dst, /*validSeqNo=*/ false, /*seqno=*/ 0, /*lifeTime=*/ m_pathDiscoveryTime);
      // Check if TtlStart == NetDiameter
      newEntry.SetLastHopCount (ttl);
      if (ttl == m_netDiameter)
        newEntry.IncrementRreqCnt ();
      newEntry.SetFlag (IN_SEARCH);
      m_routingTable.AddRoute (newEntry);
    }
  if (m_gratuitousReply)
    rreqHeader.SetGratiousRrep (true);
  if (m_destinationOnly)
    rreqHeader.SetDestinationOnly (true);

  m_seqNo += 2;
  rreqHeader.SetOriginSeqno (m_seqNo);
  m_requestId++;
  rreqHeader.SetId (m_requestId);
  rreqHeader.SetHopCount (0);
  rreqHeader.SetMRE(INFINITY3);
  std::pair<uint32_t, uint32_t> pos = GetPosition();
  rreqHeader.SetX(pos.first);
  rreqHeader.SetY(pos.second);
  rreqHeader.SetSquaredDistance(0);
  rreqHeader.SetTimeStamp(Simulator::Now().GetNanoSeconds());
  //NS_LOG_UNCOND("RREQ SENT FROM ORIGIN WITH TTL = " << ttl<< " at time = " << Simulator::Now().GetSeconds());
  //now workingNS_LOG_UNCOND((int)rreqHeader.GetHopCount() << "yo");//bug not working
  // Send RREQ as subnet directed broadcast from each interface used by aomdv
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
         m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
    {
      Ptr<Socket> socket = j->first;
      Ipv4InterfaceAddress iface = j->second;

      rreqHeader.SetOrigin (iface.GetLocal ());
      m_rreqIdCache.IsDuplicate (iface.GetLocal (), m_requestId);

      Ptr<Packet> packet = Create<Packet> ();
      SocketIpTtlTag tag;
      tag.SetTtl (ttl);
      packet->AddPacketTag (tag);
      packet->AddHeader (rreqHeader);
      TypeHeader tHeader (AOMDVTYPE_RREQ);
      packet->AddHeader (tHeader);
      // Send to all-hosts broadcast if on /32 addr, subnet-directed otherwise
      Ipv4Address destination;
      if (iface.GetMask () == Ipv4Mask::GetOnes ())
        {
          destination = Ipv4Address ("255.255.255.255");
        }
      else
        { 
          destination = iface.GetBroadcast ();
        }
      //NS_LOG_UNCOND(destination);
      NS_LOG_DEBUG ("Send RREQ with id " << rreqHeader.GetId () << " to socket");
      m_lastBcastTime = Simulator::Now ();
      Simulator::Schedule (Time (MilliSeconds (m_uniformRandomVariable->GetInteger (0, 10))), &RoutingProtocol::SendTo, this, socket, packet, destination); 
    }
  ScheduleRreqRetry (dst);
}

void
RoutingProtocol::SendTo (Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address destination)
{
    socket->SendTo (packet, 0, InetSocketAddress (destination, AOMDV_PORT));

}
void
RoutingProtocol::ScheduleRreqRetry (Ipv4Address dst)
{
  NS_LOG_FUNCTION (this << dst);
  if (m_addressReqTimer.find (dst) == m_addressReqTimer.end ())
    {
      Timer timer (Timer::CANCEL_ON_DESTROY);
      m_addressReqTimer[dst] = timer;
    }
  m_addressReqTimer[dst].SetFunction (&RoutingProtocol::RouteRequestTimerExpire, this);
  m_addressReqTimer[dst].Cancel ();
  m_addressReqTimer[dst].SetArguments (dst);
  RoutingTableEntry rt;
  m_routingTable.LookupRoute (dst, rt);
  Time retry;
  retry = 2 * m_nodeTraversalTime * (rt.GetLastHopCount () + m_timeoutBuffer);
  m_addressReqTimer[dst].Schedule (retry);
  NS_LOG_LOGIC ("Scheduled RREQ retry in " << retry.GetSeconds () << " seconds");
}

void
RoutingProtocol::RecvAomdv (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  Address sourceAddress;
  Ptr<Packet> packet = socket->RecvFrom (sourceAddress);
  InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom (sourceAddress);
  Ipv4Address sender = inetSourceAddr.GetIpv4 ();
  Ipv4Address receiver;
  if (m_socketAddresses.find (socket) != m_socketAddresses.end ())
    {
      receiver = m_socketAddresses[socket].GetLocal ();
    }
  else if(m_socketSubnetBroadcastAddresses.find (socket) != m_socketSubnetBroadcastAddresses.end ())
    {
      receiver = m_socketSubnetBroadcastAddresses[socket].GetLocal ();
    }
  else
    {
      NS_ASSERT_MSG (false, "Received a packet from an unknown socket");
    }
  NS_LOG_DEBUG ("AOMDV node " << this << " received a AOMDV packet from " << sender << " to " << receiver);
  //NS_LOG_UNCOND("RECV AOMDV CHECK");
  UpdateRouteToNeighbor (sender, receiver);
  TypeHeader tHeader (AOMDVTYPE_RREQ);
  packet->RemoveHeader (tHeader);
  if (!tHeader.IsValid ())
    {
      NS_LOG_DEBUG ("AOMDV message " << packet->GetUid () << " with unknown type received: " << tHeader.Get () << ". Drop");
      return; // drop
    }
  //NS_LOG_UNCOND(tHeader.Get ());
  switch (tHeader.Get ())
    {
    case AOMDVTYPE_RREQ:
      {
        //NS_LOG_UNCOND("CHECK RCV");// WAS RECEIVED HERE
        RecvRequest (packet, receiver, sender);
        //S_LOG_UNCOND("CHECK RCV"); NOT PRINTED
        break;
      }
    case AOMDVTYPE_RREP:
      {
        RecvReply (packet, receiver, sender);
        break;
      }
    case AOMDVTYPE_RERR:
      {
        RecvError (packet, sender);
        break;
      }
    case AOMDVTYPE_RREP_ACK:
      {
        RecvReplyAck (sender);
        break;
      }
    }
}

bool
RoutingProtocol::UpdateRouteLifeTime (Ipv4Address addr, Time lifetime)
{
  NS_LOG_FUNCTION (this << addr << lifetime);
  RoutingTableEntry rt;
  if (m_routingTable.LookupRoute (addr, rt))
    {
      if (rt.GetFlag () == VALID)
        {
          NS_LOG_DEBUG ("Updating VALID route");
          rt.SetRreqCnt (0);
          rt.SetLifeTime (std::max (lifetime, rt.GetLifeTime ()));
          m_routingTable.Update (rt);
          return true;
        }
    }
  return false;
}

bool
RoutingProtocol::UpdatePathsLifeTime (Ipv4Address addr, Time lifetime)//todo thias is not woking
{
  NS_LOG_FUNCTION (this << addr << lifetime);
  RoutingTableEntry rt;
  if (m_routingTable.LookupRoute (addr, rt))
    {
      if (rt.GetFlag () == VALID)
        {
          NS_LOG_DEBUG ("Updating VALID route");
          rt.SetRreqCnt (0);
          rt.SetLifeTime (std::max (lifetime, rt.GetLifeTime ()));
          std::vector<RoutingTableEntry::Path> paths;
          rt.GetPaths (paths);
          for (std::vector<RoutingTableEntry::Path>::iterator i = paths.begin (); i!= paths.end (); ++i)
            {
               i->SetExpire (std::max (lifetime, i->GetExpire ()));
            }
          m_routingTable.Update (rt);//change 3/1/2021
          return true;
        }
    }
  return false;
}

void
RoutingProtocol::UpdateRouteToNeighbor (Ipv4Address sender, Ipv4Address receiver)
{
  NS_LOG_FUNCTION (this << "sender " << sender << " receiver " << receiver);
  RoutingTableEntry toNeighbor;
  if (!m_routingTable.LookupRoute (sender, toNeighbor))
    {
      Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (receiver));
      RoutingTableEntry newEntry (/*dst=*/ sender, /*validSeqNo=*/ false, /*seqno=*/ 0, 
                                  /*lifeTime=*/ m_activeRouteTimeout);
      newEntry.PathInsert (/*device=*/ dev, /*nextHop=*/ sender, /*hop=*/ 1, 
                           /*expireTime=*/ m_activeRouteTimeout, 
                           /*lastHop=*/ sender, 
                           /*iface=*/ m_ipv4->GetAddress (m_ipv4->GetInterfaceForAddress (receiver), 0),
                           /*MRE*/ INFINITY3 , 10000, 0);//wrong
      m_routingTable.AddRoute (newEntry);
    }
  else
    {
      if(toNeighbor.GetFlag () == IN_SEARCH) return;
      Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (receiver));
      std::vector<RoutingTableEntry::Path> paths;
      bool validPath = false;
      toNeighbor.GetPaths (paths);
      for (std::vector<RoutingTableEntry::Path>::const_iterator i = paths.begin (); i!= paths.end (); ++i)
        {
          if ((i->GetHopCount () == 1) && (i->GetOutputDevice () == dev))
            {
              validPath = true;
            }
        }
      if (toNeighbor.GetValidSeqNo () && validPath)
        {
          toNeighbor.SetLifeTime (std::max (m_activeRouteTimeout, toNeighbor.GetLifeTime ()));
        }
      else
        {
          RoutingTableEntry newEntry (/*dst=*/ sender, /*validSeqNo=*/ false, /*seqno=*/ 0, 
                                      /*lifeTime=*/ std::max (m_activeRouteTimeout, toNeighbor.GetLifeTime ()));
          newEntry.PathInsert (/*device=*/ dev, /*nextHop=*/ sender, /*hop=*/ 1, 
                               /*expireTime=*/ std::max (m_activeRouteTimeout, toNeighbor.GetLifeTime ()), 
                               /*lastHop=*/ sender, 
                               /*iface=*/ m_ipv4->GetAddress (m_ipv4->GetInterfaceForAddress (receiver), 0),
                               /*MRE=*/ INFINITY3, 10000, 0);//wrong
          m_routingTable.Update (newEntry);
        }
    }

}

void
RoutingProtocol::RecvRequest (Ptr<Packet> p, Ipv4Address receiver, Ipv4Address src)
{
  NS_LOG_FUNCTION (this);
  bool killRequestPropagation = false;
  RreqHeader rreqHeader;
  p->RemoveHeader (rreqHeader);
  IdCache::UniqueId* b = NULL;
  RoutingTableEntry::Path* reversePath = NULL;
  RoutingTableEntry::Path* erp = NULL;

  // A node ignores all RREQs received from any node in its blacklist
  RoutingTableEntry toPrev;
  if (m_routingTable.LookupRoute (src, toPrev))
    {
      if (toPrev.IsUnidirectional ())
        {
          NS_LOG_DEBUG ("Ignoring RREQ from node in blacklist");
          return;
        }
    }
  uint32_t id = rreqHeader.GetId ();
  Ipv4Address origin = rreqHeader.GetOrigin ();
  // NS_LOG_UNCOND("CHECK THIS");
  // NS_LOG_UNCOND(origin)
  // NS_LOG_UNCOND("rrec reveived at " << receiver << " from " << src);
  // NS_LOG_UNCOND(receiver);debug ok
  /*
   *  Node checks to determine whether it has received a RREQ with the same Originator IP Address and RREQ ID.
   *  If such a RREQ has been received, the node silently discards the newly received RREQ.
   */
  //NS_LOG_UNCOND("CHECK RREQ HEADER ID = " << id);
  if(IsMyOwnAddress(origin)) return;

  //NS_LOG_UNCOND("START");
  //NS_LOG_UNCOND(receiver);
  if ((b = m_rreqIdCache.GetId (origin, id)) == NULL)
    {
      m_rreqIdCache.InsertId (origin, id);
      b = m_rreqIdCache.GetId (origin, id);
    }
  else
    {
      killRequestPropagation = true;
      //NS_LOG_UNCOND("SEEN");
    }
  
   if (rreqHeader.GetHopCount () == 0) 
    {
      rreqHeader.SetFirstHop (receiver);
      //rq->rq_first_hop = index;
    }
  // Increment RREQ hop count
  uint8_t hop = rreqHeader.GetHopCount () + 1;
  std::pair<uint32_t, uint32_t> pos = GetPosition();
  uint32_t edgeDistance = (rreqHeader.GetX () - pos.first)*(rreqHeader.GetX () - pos.first) + (rreqHeader.GetY () - pos.second)*(rreqHeader.GetY () - pos.second); 
  uint32_t squaredDistance = rreqHeader.GetSquaredDistance() + edgeDistance;
  uint64_t delay = Simulator::Now().GetNanoSeconds () - rreqHeader.GetTimeStamp ();
  Ipv4Address dst = rreqHeader.GetDst ();
  //rreqHeader.SetHopCount (hop);

  /*
   *  When the reverse route is created or updated, the following actions on the route are also carried out:
   *  1. the Originator Sequence Number from the RREQ is compared to the corresponding destination sequence number
   *     in the route table entry and copied if greater than the existing value there
   *  2. the valid sequence number field is set to true;
   *  3. the next hop in the routing table becomes the node from which the  RREQ was received
   *  4. the hop count is copied from the Hop Count in the RREQ message;
   *  5. the Lifetime is set to be the maximum of (ExistingLifetime, MinimalLifetime), where
   *     MinimalLifetime = current time + 2*NetTraversalTime - 2*HopCount*NodeTraversalTime
   */
  RoutingTableEntry toOrigin;
  //NS_LOG_UNCOND("check");
  Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (receiver));
  if (!m_routingTable.LookupRoute (origin, toOrigin))
    {
      RoutingTableEntry newEntry (
          /*dst=*/origin, /*validSeno=*/true, /*seqNo=*/rreqHeader.GetOriginSeqno (),
          /*timeLife=*/Time (m_activeRouteTimeout));
      m_routingTable.AddRoute (newEntry);
      m_routingTable.LookupRoute (origin, toOrigin);
    }
  RoutingTableEntry toDstRReq;
  if(rreqHeader.GetRReq () && (IsMyOwnAddress(dst) || m_routingTable.LookupRoute (dst, toDstRReq)))
  {
    if (IsMyOwnAddress (dst))
    {
      //NS_LOG_UNCOND("rrec reveived at " << receiver << " from " << src);
      if (toOrigin.GetFlag () == VALID)
      {
        if ((reversePath = toOrigin.PathLookupDisjoint (src, rreqHeader.GetFirstHop ())))
          {
            if (reversePath->GetHopCount () == (rreqHeader.GetHopCount () + 1))
              {
                toOrigin.SetSeqNo (rreqHeader.GetOriginSeqno ());
                reversePath->SetExpire (std::max (
                    reversePath->GetExpire (),
                    Time ((2 * m_netTraversalTime - 2 * hop * m_nodeTraversalTime))));//todo update mre and pos and this is not working
              }
            NS_ASSERT(reversePath->GetHopCount () == (rreqHeader.GetHopCount () + 1));
          }
        else if (toOrigin.PathNewDisjoint (src, rreqHeader.GetFirstHop ()))
          {
            if ((((rreqHeader.GetHopCount () + 1) - toOrigin.PathGetMinHopCount ()) <=
                  AOMDV_PRIM_ALT_PATH_LENGTH_DIFF))
              {
                toOrigin.SetSeqNo (rreqHeader.GetOriginSeqno ());
                reversePath = toOrigin.PathInsert (
                    dev, src, rreqHeader.GetHopCount () + 1,
                    Time ((2 * m_netTraversalTime - 2 * hop * m_nodeTraversalTime)),
                    rreqHeader.GetFirstHop (),
                    m_ipv4->GetAddress (m_ipv4->GetInterfaceForAddress (receiver), 0), rreqHeader.GetMRE(), squaredDistance, delay);
                toOrigin.SetLastHopCount (toOrigin.PathGetMaxHopCount ());
              }
            if (((rreqHeader.GetHopCount () + 1) - toOrigin.PathGetMinHopCount ()) >
                AOMDV_PRIM_ALT_PATH_LENGTH_DIFF)
              {
                return;
              }
          }
        else if ((((erp = toOrigin.PathLookupLastHop (rreqHeader.GetFirstHop ())) ==
                    NULL) ||
                  ((rreqHeader.GetHopCount () + 1) > erp->GetHopCount ())))
          {
            return; //todo can have some changes
          }
      }
    else //fully chanes from return;
      {
        return;
      }
    }
    else if( m_routingTable.LookupRoute (dst, toDstRReq) )
      return;
  }
  else if (int32_t (rreqHeader.GetOriginSeqno ()) - int32_t (toOrigin.GetSeqNo ()) > 0)
    {
      toOrigin.SetSeqNo (rreqHeader.GetOriginSeqno ());
      toOrigin.SetAdvertisedHopCount (INFINITY2);
      toOrigin.PathAllDelete ();
      toOrigin.SetFlag (VALID);
      Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (receiver));
      reversePath = toOrigin.PathInsert (
          dev, src, rreqHeader.GetHopCount () + 1,
          Time ((2 * m_netTraversalTime - 2 * hop * m_nodeTraversalTime)),
          rreqHeader.GetFirstHop (),
          m_ipv4->GetAddress (m_ipv4->GetInterfaceForAddress (receiver), 0), rreqHeader.GetMRE(), squaredDistance, delay);
      toOrigin.SetLastHopCount (toOrigin.PathGetMaxHopCount ());
    }
  else if ((int32_t (rreqHeader.GetOriginSeqno ()) == int32_t (toOrigin.GetSeqNo ())) 
          && (((int)toOrigin.GetAdvertisedHopCount ()) > (rreqHeader.GetHopCount ())))
    {
    if (toOrigin.GetFlag () == VALID)
      {
        if ((reversePath = toOrigin.PathLookupDisjoint (src, rreqHeader.GetFirstHop ())))
          {
            if (reversePath->GetHopCount () == (rreqHeader.GetHopCount () + 1))
              {
                reversePath->SetExpire (std::max (
                    reversePath->GetExpire (),
                    Time ((2 * m_netTraversalTime - 2 * hop * m_nodeTraversalTime))));//todo update mre and pos and this is not working
              }
            NS_ASSERT(reversePath->GetHopCount () == (rreqHeader.GetHopCount () + 1));
          }
        else if (toOrigin.PathNewDisjoint (src, rreqHeader.GetFirstHop ()))
          {
            if ((((rreqHeader.GetHopCount () + 1) - toOrigin.PathGetMinHopCount ()) <=
                  AOMDV_PRIM_ALT_PATH_LENGTH_DIFF))
              {
                reversePath = toOrigin.PathInsert (
                    dev, src, rreqHeader.GetHopCount () + 1,
                    Time ((2 * m_netTraversalTime - 2 * hop * m_nodeTraversalTime)),
                    rreqHeader.GetFirstHop (),
                    m_ipv4->GetAddress (m_ipv4->GetInterfaceForAddress (receiver), 0), rreqHeader.GetMRE(), squaredDistance, delay);
                toOrigin.SetLastHopCount (toOrigin.PathGetMaxHopCount ());
              }
            if (((rreqHeader.GetHopCount () + 1) - toOrigin.PathGetMinHopCount ()) >
                AOMDV_PRIM_ALT_PATH_LENGTH_DIFF)
              {
                return;
              }
          }
        else if ((IsMyOwnAddress (rreqHeader.GetDst ())) &&
                  (((erp = toOrigin.PathLookupLastHop (rreqHeader.GetFirstHop ())) ==
                    NULL) ||
                  ((rreqHeader.GetHopCount () + 1) > erp->GetHopCount ())))
          {
            return; //todo can have some changes
          }
      }
    else //fully chanes from return;
      {
        return;
      }
    }
  else {
    return;
  }
  m_routingTable.Update (toOrigin);
  // else
  // {

  //   toOrigin.SetSeqNo (rreqHeader.GetOriginSeqno ());
  //   toOrigin.SetValidSeqNo (true);
  //   toOrigin.SetAdvertisedHopCount (INFINITY2);
  //   toOrigin.PathAllDelete ();
  //   toOrigin.SetFlag (VALID);
  //   Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (receiver));
  //   reversePath = toOrigin.PathInsert (
  //       dev, src, rreqHeader.GetHopCount () + 1,
  //       Time ((2 * m_netTraversalTime - 2 * hop * m_nodeTraversalTime)), rreqHeader.GetFirstHop (),
  //       m_ipv4->GetAddress (m_ipv4->GetInterfaceForAddress (receiver), 0));
  //   toOrigin.SetLastHopCount (toOrigin.PathGetMaxHopCount ());
  // }



  // RoutingTableEntry toNeighbor;
  // if (!m_routingTable.LookupRoute (src, toNeighbor))
  //   {
  //     NS_LOG_DEBUG ("Neighbor:" << src << " not found in routing table. Creating an entry"); 
  //     Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (receiver));
  //     RoutingTableEntry newEntry (/*dst=*/ src, /*validSeno=*/ false, /*seqNo=*/ rreqHeader.GetOriginSeqno (),
  //                                 /*timeLife=*/ m_activeRouteTimeout);
  //     newEntry.PathInsert (dev, src, 1, 
  //                          m_activeRouteTimeout, src, 
  //                          m_ipv4->GetAddress (m_ipv4->GetInterfaceForAddress (receiver), 0), rreqHeader.GetMRE(), edgeDistance);
  //     m_routingTable.AddRoute (newEntry);
  //   }
  // else
  //   {
  //     toNeighbor.SetLifeTime (m_activeRouteTimeout);
  //     toNeighbor.SetValidSeqNo (false);
  //     toNeighbor.SetSeqNo (rreqHeader.GetOriginSeqno ()); 
  //     toNeighbor.SetFlag (VALID);
  //     RoutingTableEntry::Path *p = toNeighbor.PathLookup (src);
  //     if(!p){
  //       toNeighbor.PathInsert (dev, src, 1, 
  //                          m_activeRouteTimeout, src, 
  //                          m_ipv4->GetAddress (m_ipv4->GetInterfaceForAddress (receiver), 0), rreqHeader.GetMRE(), edgeDistance);
  //     }
  //     else {
  //       p->SetOutputDevice (m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (receiver)));
  //       p->SetInterface (m_ipv4->GetAddress (m_ipv4->GetInterfaceForAddress (receiver), 0));
  //       p->SetHopCount (1);
  //       p->SetNextHop (src);
  //       p->SetSquaredDistance(edgeDistance);
  //     }
  //     //NS_LOG_UNCOND("PATH CHECK NOW FOR");
  //     //NS_LOG_UNCOND(src);
      
  //     m_routingTable.Update (toNeighbor);
  //   }
  m_nb.Update (src, Time (m_allowedHelloLoss * m_helloInterval));

  NS_LOG_LOGIC (receiver << " receive RREQ with hop count " << static_cast<uint32_t>(rreqHeader.GetHopCount ()) 
                         << " ID " << rreqHeader.GetId ()
                         << " to destination " << rreqHeader.GetDst ());

  //  A node generates a RREP if either:
  //  (i)  it is itself the destination,
  rreqHeader.SetX(pos.first);
  rreqHeader.SetY(pos.second);
  rreqHeader.SetSquaredDistance(squaredDistance);
  if (IsMyOwnAddress (rreqHeader.GetDst ()))
    {
      m_routingTable.LookupRoute (origin, toOrigin);
      //NS_LOG_UNCOND ("Send reply since I am the destination" << src << " " << toOrigin.GetNumberofPaths()<< "at time : "<< Simulator::Now().GetSeconds());
      // NS_LOG_UNCOND ("LAST HOP - " << src << " firstHop = " << rreqHeader.GetFirstHop ());
      //NS_LOG_UNCOND ("NEXT HOP IN current PATH = " << toOrigin.PathFind()->GetNextHop());
      //NS_LOG_UNCOND ("rreq id = " << rreqHeader.GetId());
      if(toOrigin.PathLookup(src) != NULL) SendReply (rreqHeader, toOrigin, src);//todo reversepath can be used
      return;
    }
  
  /*
   * (ii) or it has an active route to the destination, the destination sequence number in the node's existing route table entry for the destination
   *      is valid and greater than or equal to the Destination Sequence Number of the RREQ, and the "destination only" flag is NOT set.
   */
  RoutingTableEntry toDst;
  if (m_routingTable.LookupRoute (dst, toDst) )//&& false ) TODO GOOD THINGS HAPPEN EXCEPT ONLY SOURCE IS NOT RECEIVING RREP 
    {
      /*
       * Drop RREQ, This node RREP wil make a loop.
       */
      //NS_LOG_UNCOND("DESTINATION INTERMEDIATE CHECK");
      //NS_LOG_UNCOND(receiver);
      //NS_LOG_UNCOND(origin);

      if (toDst.PathFind ()->GetNextHop () == src || toDst.PathFind ()->GetNextHop () == toOrigin.PathFind ()->GetNextHop())//toDst.PathFind ()->GetNextHop ()
        {
          //NS_LOG_UNCOND ("Drop RREQ from " << src << ", dest next hop " << toDst.PathFind()->GetNextHop ());
          return;
        }
        else{
          // NS_LOG_UNCOND("CHECK INTERMEDIATE REPLY");
          // NS_LOG_UNCOND("sender = " << src << " current node - "<< receiver << " origin = " << origin << " destination = " << dst);
          // NS_LOG_UNCOND("HOPS TILL NOW = " << (int)rreqHeader.GetHopCount() << " next hop origin = " << toOrigin.PathFind()->GetNextHop());
          // NS_LOG_UNCOND("HOPS FOR DESTINATION = " << toDst.PathFind()->GetHopCount () << " next hop for destination = " << toDst.PathFind()->GetLastHop()<< '\n');
          // NS_LOG_UNCOND("END CHECK");
        }
      /*
       * The Destination Sequence number for the requested destination is set to the maximum of the corresponding value
       * received in the RREQ message, and the destination sequence value currently maintained by the node for the requested destination.
       * However, the forwarding node MUST NOT modify its maintained value for the destination sequence number, even if the value
       * received in the incoming RREQ is larger than the value currently maintained by the forwarding node.
       */
      if ((rreqHeader.GetUnknownSeqno () || (int32_t (toDst.GetSeqNo ()) - int32_t (rreqHeader.GetDstSeqno ()) >= 0))
          && toDst.GetValidSeqNo () )
        {
          if (!rreqHeader.GetDestinationOnly () && toDst.GetFlag () == VALID)
            {
              //NS_LOG_UNCOND("reverse path  == nul  " << (reversePath == NULL));
              if (reversePath)
                {
                  #ifdef AOMDV_NODE_DISJOINT_PATHS
                  if (b->count == 0)
                    {
                      b->count = 1;
                      if (toDst.GetAdvertisedHopCount () == INFINITY2)
                        {
                          toDst.SetAdvertisedHopCount (toDst.PathGetMaxHopCount ());
                        }
                      RoutingTableEntry::Path *forwardPath = toDst.PathFind ();
                      toDst.SetError (true);
                      m_routingTable.Update(toDst);
                      SendReplyByIntermediateNode (toDst, toOrigin, rreqHeader.GetGratiousRrep (),forwardPath->GetExpire (), 
                                                   forwardPath->GetLastHop (),rreqHeader.GetId ());
                      return;
                    }
                  #endif
                  #ifdef AOMDV_LINK_DISJOINT_PATHS
                  Ipv4Address forNextHop = Ipv4Address::GetAny(), forLastHop = Ipv4Address::GetAny();
                  Time forExpire;
                  //RoutingTableEntry::Path *firstPath = toDst.PathFind (); // Get first path for RREQ destination
			/* Make sure we don't answer with the same forward path twice in response 
				to a certain RREQ (received more than once). E.g. "middle node" 
				in "double diamond". */
                  std::vector<RoutingTableEntry::Path> paths;
                  toDst.GetPaths (paths);
                  //AOMDVRoute rt;
                  for (std::vector<RoutingTableEntry::Path>::const_iterator i = paths.begin (); i!= paths.end (); ++i)
                    {
                      if (!(b->ForwardPathLookup (i->GetNextHop (), i->GetLastHop ())))
                        {
                          forNextHop = i->GetNextHop ();
                          forLastHop = i->GetLastHop ();
                          forExpire = i->GetExpire ();
                          break;
                        }
                    }
                  /* If an unused forward path is found and we have not answered
				along this reverse path (for this RREQ) - send a RREP back. */
                  if ((forNextHop != Ipv4Address::GetAny()) && !(b->ReversePathLookup (reversePath->GetNextHop (), reversePath->GetLastHop ())))
                    {
                      /* Mark the reverse and forward path as used (for this RREQ). */
				// Cache the broadcast ID
                      b->ReversePathInsert (reversePath->GetNextHop(),reversePath->GetLastHop());
                      b->ForwardPathInsert (forNextHop, forLastHop);
                      if (toDst.GetAdvertisedHopCount () == INFINITY2)
                        {
                          toDst.SetAdvertisedHopCount (toDst.PathGetMaxHopCount ());
                        }
                      toDst.SetError (true);
                      SendReplyByIntermediateNode (toDst, toOrigin, rreqHeader.GetGratiousRrep (), forExpire , 
                                                   forLastHop , rreqHeader.GetId ());
                      return;
		    }
                  #endif // AOMDV_LINK_DISJOINT_PATHS
                }
            }
            else{
              //NS_LOG_UNCOND("ULULULULYLULULULULLULULULULULULULLLULLLLLUU");
            }
          rreqHeader.SetDstSeqno (toDst.GetSeqNo ());
          rreqHeader.SetUnknownSeqno (false);
        }
    }   
  if (killRequestPropagation || IsMyOwnAddress(dst))
    {
      return;
    }
  else
    {
      if (toOrigin.GetAdvertisedHopCount () == INFINITY2)
        {
          toOrigin.SetAdvertisedHopCount ( toOrigin.PathGetMaxHopCount ());
        }
        m_routingTable.Update(toOrigin);
      rreqHeader.SetHopCount (toOrigin.GetAdvertisedHopCount ());
      rreqHeader.SetMRE(std::min(rreqHeader.GetMRE () , GetRemainingEnergy ()));
      #ifdef AOMDV_NODE_DISJOINT_PATHS
      rreqHeader.SetFirstHop ((toOrigin.PathFind ())->GetLastHop ());// add if(toOrigin.PathFind())
      #endif // AOMDV_NODE_DISJOINT_PATHS
      SocketIpTtlTag tag;
      p->RemovePacketTag (tag);
      if (tag.GetTtl () < 2)
        {
          NS_LOG_DEBUG ("TTL exceeded. Drop RREQ origin " << src << " destination " << dst );
          return;
        }
      for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
           m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
        {
          Ptr<Socket> socket = j->first;
          Ipv4InterfaceAddress iface = j->second;
          Ptr<Packet> packet = Create<Packet> ();
          SocketIpTtlTag ttl;
          ttl.SetTtl (tag.GetTtl () - 1);
          packet->AddPacketTag (ttl);
          packet->AddHeader (rreqHeader);
          TypeHeader tHeader (AOMDVTYPE_RREQ);
          packet->AddHeader (tHeader);
          // Send to all-hosts broadcast if on /32 addr, subnet-directed otherwise
          Ipv4Address destination;
          if (iface.GetMask () == Ipv4Mask::GetOnes ())
            {
              destination = Ipv4Address ("255.255.255.255");
            }
          else
            { 
              destination = iface.GetBroadcast ();
            }
          m_lastBcastTime = Simulator::Now ();
          Simulator::Schedule (Time (MilliSeconds (m_uniformRandomVariable->GetInteger (0, 10))), 
                               &RoutingProtocol::SendTo, this, socket, packet, destination); 
        }
    }
}

void
RoutingProtocol::SendReply (RreqHeader const & rreqHeader, RoutingTableEntry & toOrigin,
                            Ipv4Address firstHop)
{
  NS_LOG_FUNCTION (this << toOrigin.GetDestination ());
  // NS_LOG_UNCOND ("REALLY IT IS GOING TO DEST");
  // NS_LOG_UNCOND (firstHop);
  // NS_LOG_UNCOND (toOrigin.PathFind ()->GetNextHop ());
  /*
   * Destination node MUST increment its own sequence number by one if the sequence number in the RREQ packet is equal to that
   * incremented value. Otherwise, the destination does not change its sequence number before generating the  RREP message.
   */
  if ( m_seqNo <= rreqHeader.GetDstSeqno())//change from real ns
    m_seqNo = rreqHeader.GetDstSeqno()+1;
  if (m_seqNo%2) m_seqNo++;
  std::pair<uint32_t, uint32_t> pos = GetPosition();
  //NS_LOG_UNCOND("destination seq no = " << m_seqNo);
  //NS_LOG_UNCOND("RREQ HEADER SEQ NO = " << rreqHeader.GetDstSeqno());
  RrepHeader rrepHeader ( /*prefixSize=*/ 0, /*hops=*/ 0, /*dst=*/ rreqHeader.GetDst (),
                          /*dstSeqNo=*/ m_seqNo, /*origin=*/ toOrigin.GetDestination (), 
                          /*requestId=*/ rreqHeader.GetId (), /*firstHop=*/firstHop,
                          /*lifeTime=*/ m_activeRouteTimeout, /*MRE*/ INFINITY3, pos.first , pos.second, 0, Simulator::Now().GetNanoSeconds());
  if(rreqHeader.GetRReq ()) rrepHeader.SetDstSeqno(rreqHeader.GetDstSeqno ());
  Ptr<Packet> packet = Create<Packet> ();
  SocketIpTtlTag tag;
  //NS_LOG_UNCOND("TTL AT DESTINATION FOR RREP = " << toOrigin.GetAdvertisedHopCount ()+1);
  tag.SetTtl (toOrigin.PathGetMaxHopCount ()+1);// TODO toOrigin.GetAdvertisedHopCount ()+1 CHANGE BAD
  //NS_LOG_UNCOND("original tag tll = " << (int)tag.GetTtl());
  packet->AddPacketTag (tag);
  packet->AddHeader (rrepHeader);
  TypeHeader tHeader (AOMDVTYPE_RREP);
  packet->AddHeader (tHeader);

  Ptr<Socket> socket = FindSocketWithInterfaceAddress (toOrigin.PathFind()->GetInterface ());//wrong assumed same interface
  NS_ASSERT (socket);

  socket->SendTo (packet, 0, InetSocketAddress (firstHop, AOMDV_PORT));//TODO changed
}

void
RoutingProtocol::SendReplyByIntermediateNode (RoutingTableEntry & toDst, RoutingTableEntry & toOrigin,
                                              bool gratRep, Time expire, 
                                              Ipv4Address firstHop, uint32_t requestId)
{
  NS_LOG_FUNCTION (this);
  //NS_LOG_UNCOND("WHAT WHAT");
  std::pair<uint32_t, uint32_t> pos = GetPosition();
  RrepHeader rrepHeader (/*prefix size=*/ 0, /*hops=*/ toDst.PathFind ()->GetHopCount (), /*dst=*/ toDst.GetDestination (), 
                         /*dst seqno=*/ toDst.GetSeqNo (), /*origin=*/ toOrigin.GetDestination (),
                         /*requestId=*/requestId, /*firstHop=*/ firstHop, /*lifetime=*/ toDst.GetLifeTime (), /*MRE*/ std::min(toDst.PathFind ()->GetMRE (), GetRemainingEnergy ()),
                         pos.first, pos.second, toDst.PathFind ()->GetSquaredDistance(), Simulator::Now().GetNanoSeconds() - toDst.PathFind()->GetDelay());

  /* If the node we received a RREQ for is a neighbor we are
   * probably facing a unidirectional link... Better request a RREP-ack
   */
  if (toDst.PathFind ()->GetHopCount () == 1)
    {
      rrepHeader.SetAckRequired (true);
      RoutingTableEntry toNextHop;
      m_routingTable.LookupRoute (toOrigin.PathFind ()->GetNextHop (), toNextHop);
      toNextHop.m_ackTimer.SetFunction (&RoutingProtocol::AckTimerExpire, this);
      toNextHop.m_ackTimer.SetArguments (toNextHop.GetDestination (), m_blackListTimeout);
      toNextHop.m_ackTimer.SetDelay (m_nextHopWait);
    }
  toDst.InsertPrecursor (toOrigin.PathFind ()->GetNextHop ());
  toOrigin.InsertPrecursor (toDst.PathFind ()->GetNextHop ());
  m_routingTable.Update (toDst);
  m_routingTable.Update (toOrigin);

  Ptr<Packet> packet = Create<Packet> ();
  SocketIpTtlTag tag;
  tag.SetTtl (toOrigin.PathGetMaxHopCount ()+1);// TODO toOrigin.GetAdvertisedHopCount ()+1 CHANGE BAD
  packet->AddPacketTag (tag);
  packet->AddHeader (rrepHeader);
  TypeHeader tHeader (AOMDVTYPE_RREP);
  packet->AddHeader (tHeader);
  Ptr<Socket> socket = FindSocketWithInterfaceAddress (toOrigin.PathFind ()->GetInterface ());
  NS_ASSERT (socket);
  socket->SendTo (packet, 0, InetSocketAddress (toOrigin.PathFind ()->GetNextHop (), AOMDV_PORT));

  // Generating gratuitous RREPs
  if (gratRep)
    {
      RrepHeader gratRepHeader (/*prefix size=*/ 0, /*hops=*/ toOrigin.PathFind ()->GetHopCount (), /*dst=*/ toOrigin.GetDestination (),
                                /*dst seqno=*/ toOrigin.GetSeqNo (), /*origin=*/ toDst.GetDestination (), /*requestId=*/
                                requestId, /*firstHop=*/ firstHop, /*lifetime=*/ toOrigin.GetLifeTime (), /*MRE*/ toOrigin.PathFind ()->GetMRE ());
      Ptr<Packet> packetToDst = Create<Packet> ();
      SocketIpTtlTag gratTag;
      gratTag.SetTtl (toDst.GetAdvertisedHopCount ());
      packetToDst->AddPacketTag(gratTag);
      packetToDst->AddHeader (gratRepHeader);
      TypeHeader type (AOMDVTYPE_RREP);
      packetToDst->AddHeader (type);
      Ptr<Socket> socket = FindSocketWithInterfaceAddress (toDst.PathFind ()->GetInterface ());
      NS_ASSERT (socket);
      NS_LOG_LOGIC ("Send gratuitous RREP " << packet->GetUid ());
      socket->SendTo (packetToDst, 0, InetSocketAddress (toDst.PathFind ()->GetNextHop (), AOMDV_PORT));
    }
}

void
RoutingProtocol::SendReplyAck (Ipv4Address neighbor)
{
  NS_LOG_FUNCTION (this << " to " << neighbor);
  RrepAckHeader h;
  TypeHeader typeHeader (AOMDVTYPE_RREP_ACK);
  Ptr<Packet> packet = Create<Packet> ();
  SocketIpTtlTag tag;
  tag.SetTtl (1);
  packet->AddPacketTag (tag);
  packet->AddHeader (h);
  packet->AddHeader (typeHeader);
  RoutingTableEntry toNeighbor;
  m_routingTable.LookupRoute (neighbor, toNeighbor);
  Ptr<Socket> socket = FindSocketWithInterfaceAddress (toNeighbor.PathFind ()->GetInterface ());
  NS_ASSERT (socket);
  socket->SendTo (packet, 0, InetSocketAddress (neighbor, AOMDV_PORT));
}

void
RoutingProtocol::RecvReply (Ptr<Packet> p, Ipv4Address receiver, Ipv4Address sender)
{
  NS_LOG_FUNCTION (this << " src " << sender);
  //NS_LOG_UNCOND("START RECVREPLY at " << receiver);
  RrepHeader rrepHeader;
  p->RemoveHeader (rrepHeader);
  IdCache::UniqueId* b = NULL;
  RoutingTableEntry::Path* forwardPath = NULL;
  Ipv4Address dst = rrepHeader.GetDst ();
  NS_LOG_LOGIC ("RREP destination " << dst << " RREP origin " << rrepHeader.GetOrigin ());
  if (IsMyOwnAddress (dst))
    {

      return;
    }
  if (rrepHeader.GetHopCount () == 0) 
    {
      rrepHeader.SetFirstHop (receiver);
      //rq->rq_first_hop = index;
    }

  uint8_t hop = rrepHeader.GetHopCount () + 1;
  std::pair<uint32_t, uint32_t> pos = GetPosition();
  uint32_t edgeDistance = (rrepHeader.GetX () - pos.first)*(rrepHeader.GetX () - pos.first) + (rrepHeader.GetY () - pos.second)*(rrepHeader.GetY () - pos.second); 
  uint32_t squaredDistance = rrepHeader.GetSquaredDistance() + edgeDistance;
  uint64_t delay = Simulator::Now().GetNanoSeconds () - rrepHeader.GetTimeStamp ();
  //rrepHeader.SetHopCount (hop);
  //NS_LOG_UNCOND((int)hop);
  // If RREP is Hello message
  if (dst == rrepHeader.GetOrigin ())
    {
      ProcessHello (rrepHeader, receiver);
      return;
    }
  /*
   * If the route table entry to the destination is created or updated, then the following actions occur:
   * -  the route is marked as active,
   * -  the destination sequence number is marked as valid,
   * -  the next hop in the route entry is assigned to be the node from which the RREP is received,
   *    which is indicated by the source IP address field in the IP header,
   * -  the hop count is set to the value of the hop count from RREP message + 1
   * -  the expiry time is set to the current time plus the value of the Lifetime in the RREP message,
   * -  and the destination sequence number is the Destination Sequence Number in the RREP message.
   */
  Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (receiver));
  //RoutingTableEntry newEntry (/*device=*/ dev, /*dst=*/ dst, /*validSeqNo=*/ true, /*seqno=*/ rrepHeader.GetDstSeqno (),
  //                                        /*iface=*/ m_ipv4->GetAddress (m_ipv4->GetInterfaceForAddress (receiver), 0),/*hop=*/ hop,
  //                                        /*nextHop=*/ sender, /*lifeTime=*/ rrepHeader.GetLifeTime ());

  RoutingTableEntry newEntry (/*dst=*/ dst, /*validSeqNo=*/ true, /*seqno=*/ rrepHeader.GetDstSeqno (),//todo can change to zero 
                              /*lifeTime=*/ rrepHeader.GetLifeTime ());
  if(IsMyOwnAddress(rrepHeader.GetOrigin ())) newEntry.SetFlag(IN_SEARCH);
  RoutingTableEntry toDst;
  
  if (!m_routingTable.LookupRoute (dst, toDst))
    {
    // The forward route for this destination is created if it does not already exist.
      //NS_LOG_UNCOND ("add new route");
      m_routingTable.AddRoute (newEntry);
      m_routingTable.LookupRoute(dst, toDst);
    }
    /*
      * The existing entry is updated only in the following circumstances:
      * (i) the sequence number in the routing table is marked as invalid in route table entry.
      */
    // if (!toDst.GetValidSeqNo ())
    //   {
    //     m_routingTable.Update (newEntry);
    //   }
    // (ii)the Destination Sequence Number in the RREP is greater than the node's copy of the destination sequence number
    // and the known value is valid,
    //NS_LOG_UNCOND("number of paths before= " << toDst.GetNumberofPaths() );
    //TODO WHY NS_LOG_UNCOND()
    //NS_LOG_UNCOND("Check middle of recv reply , rrep seq no = " << rrepHeader.GetDstSeqno () << " dst seq no = " << toDst.GetSeqNo ());
    //NS_LOG_UNCOND(rrepHeader.GetDstSeqno () << " " << toDst.GetSeqNo ());
    if ( (int32_t (rrepHeader.GetDstSeqno ()) - int32_t (toDst.GetSeqNo ())) > 0)
      {//TODO GETNUMBER OF PATHS = 0 DOESNT MEAN WE CAN ADD THIS
        toDst.SetValidSeqNo(rrepHeader.GetDstSeqno() != 0);
        toDst.SetSeqNo (rrepHeader.GetDstSeqno ());
        toDst.SetAdvertisedHopCount (INFINITY2);
        toDst.PathAllDelete ();
        if( !IsMyOwnAddress (rrepHeader.GetOrigin ()) )toDst.SetFlag (VALID);
        /* Insert forward path to RREQ destination. *///NS_LOG_UNCOND(toDst.GetNumberofPaths());
        forwardPath = toDst.PathInsert (dev, sender, hop, 
                                        Simulator::Now() + rrepHeader.GetLifeTime (), rrepHeader.GetFirstHop (),
                                        m_ipv4->GetAddress (m_ipv4->GetInterfaceForAddress (receiver), 0), rrepHeader.GetMRE (),
                                        squaredDistance, delay);
  // CHANGE
        //NS_LOG_UNCOND("ath insert next hop = " << forwardPath->GetNextHop() << "last hop = " << forwardPath->GetLastHop());
        toDst.SetLastHopCount (toDst.PathGetMaxHopCount ());
        
      }

    /* If the sequence number in the RREP is the same as for route entry but 
    with a smaller hop count - try to insert new forward path to (RREQ) dest. */
    // (iii) the sequence numbers are the same, but the route is marked as inactive.
    else if ((rrepHeader.GetDstSeqno () == toDst.GetSeqNo ()) && (toDst.GetFlag () == VALID || (toDst.GetFlag () ==IN_SEARCH && IsMyOwnAddress (rrepHeader.GetOrigin ()) ))
              && (toDst.GetAdvertisedHopCount () > rrepHeader.GetHopCount () || IsMyOwnAddress (rrepHeader.GetOrigin ())))
      {
        // NS_LOG_UNCOND("WHART AHAT");
        // NS_LOG_UNCOND(rrepHeader.GetDstSeqno ());
        // NS_LOG_UNCOND(toDst.GetNumberofPaths ());
        // NS_LOG_UNCOND((int)rrepHeader.GetHopCount ());
        if ((forwardPath = toDst.PathLookupDisjoint (sender,rrepHeader.GetFirstHop ())))//change rrepHeader.getorigin to sender
          {
            if (forwardPath->GetHopCount () == hop)
              {
                forwardPath->SetExpire (std::max (forwardPath->GetExpire (), Simulator::Now() + rrepHeader.GetLifeTime ()));
                forwardPath->SetMRE(rrepHeader.GetMRE ());//todo not working
                forwardPath->SetSquaredDistance(squaredDistance);
                forwardPath->SetDelay(delay);
              }
          }
        else if ((toDst.PathNewDisjoint (sender ,rrepHeader.GetFirstHop ()))//change imp
                  && (hop - toDst.PathGetMinHopCount () <= AOMDV_PRIM_ALT_PATH_LENGTH_DIFF))
          {
            /* Insert forward path to RREQ destination. */
            forwardPath = toDst.PathInsert (dev, sender, hop, 
                                            Simulator::Now() + rrepHeader.GetLifeTime (), 
                                            rrepHeader.GetFirstHop (),
                                            m_ipv4->GetAddress (m_ipv4->GetInterfaceForAddress (receiver), 0), rrepHeader.GetMRE (), squaredDistance, delay);
      // CHANGE
            toDst.SetLastHopCount (toDst.PathGetMaxHopCount ());
          }
        else
          {
            return;
          }
      
      }
    else          
      {
        //NS_LOG_UNCOND("IS THIS THE ERROR " << rrepHeader.GetDstSeqno () << " " << toDst.GetSeqNo ());
        return;
      }
    toDst.SetLifeTime(m_activeRouteTimeout);
    m_routingTable.Update(toDst);

    
  m_routingTable.LookupRoute (dst, toDst);
  // Acknowledge receipt of the RREP by sending a RREP-ACK message back
  if (rrepHeader.GetAckRequired ())
    {
      SendReplyAck (sender);
      rrepHeader.SetAckRequired (false);
    }
  //NS_LOG_UNCOND ("receiver " << receiver << " destination = " << dst << "next hop = "  << (toDst.PathFind())->GetNextHop());
  if (IsMyOwnAddress (rrepHeader.GetOrigin ()))
    {
      toDst.SetLifeTime(m_activeRouteTimeout);
      //NS_LOG_UNCOND("YO NI");
      // if (toDst.GetFlag () == IN_SEARCH)
      //   {
      //     m_routingTable.Update (newEntry);
      //   }
      // m_routingTable.LookupRoute (dst, toDst);
      // NS_LOG_UNCOND("RREP has been received at origin AT " << Simulator::Now().GetSeconds() << " to " << toDst.PathFind()->GetNextHop ());
      //NS_LOG_UNCOND(rrepHeader.GetOrigin());
      //NS_LOG_UNCOND(toDst.GetNumberofPaths());
      //SendPacketFromQueue (dst, toDst.PathLoadBalancedFind()->GetRoute ());
      return;
    }
  RoutingTableEntry toOrigin;
  uint32_t id = rrepHeader.GetRequestID ();
  b = m_rreqIdCache.GetId (rrepHeader.GetOrigin(), id);///CHANGED FROM DST TO ORIGIN
  Ipv4Address nextHop;
  //NS_LOG_UNCOND("Check middle of recv reply");
  #ifdef AOMDV_NODE_DISJOINT_PATHS
  if (!m_routingTable.LookupRoute (rrepHeader.GetOrigin (), toOrigin) || (toOrigin.GetFlag () != VALID)
      || (b == NULL) || (b->count))
    {
      //todo b is null but it should have seen these things
      return; // Impossible! drop.mk
      //EVEN THE FIRST RREP IS HERE SO . . . . .. . TODO
    }

  b->count = 1;
  RoutingTableEntry::Path *reversePath = toOrigin.PathFind ();
  //NS_LOG_UNCOND(reversePath->GetNextHop ());
  if (toDst.GetAdvertisedHopCount () == INFINITY2)
    {
      toDst.SetAdvertisedHopCount (toDst.PathGetMaxHopCount ());
    }
  
  rrepHeader.SetHopCount (toDst.GetAdvertisedHopCount ());
  rrepHeader.SetMRE( std::min(rrepHeader.GetMRE(), GetRemainingEnergy()));
  //NS_LOG_UNCOND("rEMAINING ENERGY= " << rrepHeader.GetMRE());
  //NS_LOG_UNCOND( "Position = " << GetPosition().first << " , "  << GetPosition().second);
  //NS_LOG_UNCOND("NO OF PATHS FOR DEST " << receiver << " to" << dst << " = " << toDst.GetNumberofPaths());
  rrepHeader.SetFirstHop (toDst.PathFind ()->GetLastHop ());//todo start from here
  reversePath->SetExpire (Simulator::Now() + m_activeRouteTimeout);
  toDst.SetError (true);
  m_routingTable.Update(toDst);
  nextHop = toOrigin.PathFind ()->GetNextHop ();
  #endif // AOMDV_NODE_DISJOINT_PATHS

  #ifdef AOMDV_LINK_DISJOINT_PATHS
  /* Drop the RREP packet if we do not have a path back to the source, 
      or the route is marked as down, or if we never received the original RREQ. */
  if (!m_routingTable.LookupRoute (rrepHeader.GetOrigin (), toOrigin) || (toOrigin.GetFlag () != VALID)
      || (b == NULL))
    {
      return; // Impossible! drop.
    }
  /* Make sure we don't answer along the same path twice in response 
      to a certain RREQ. Try to find an unused (reverse) path to forward the RREP. */
  //RoutingTableEntry::Path* reversePath = NULL;
  Ipv4Address revNextHop =Ipv4Address::GetAny(), revLastHop = Ipv4Address::GetAny();
  //RoutingTableEntry::Path *firstPath = toDst.PathFind (); // Get first path for RREQ destination
  /* Make sure we don't answer with the same forward path twice in response 
  to a certain RREQ (received more than once). E.g. "middle node" in "double diamond". */
  std::vector<RoutingTableEntry::Path> paths;
  toOrigin.GetPaths (paths);//change from todst
  for (std::vector<RoutingTableEntry::Path>::const_iterator i = paths.begin (); i!= paths.end (); ++i)
    {
      if (!(b->ReversePathLookup (i->GetNextHop (), i->GetLastHop ())))
        {
          revNextHop = i->GetNextHop();
          revLastHop = i->GetLastHop();
          break;
        }
    }

  /* If an unused reverse path is found and the forward path (for 
     this RREP) has not already been replied - forward the RREP. */
  if ((revNextHop != Ipv4Address::GetAny()) && !(b->ForwardPathLookup (forwardPath->GetNextHop (),  (forwardPath->GetLastHop ()))))
    {
      if(forwardPath->GetNextHop () == sender && forwardPath->GetLastHop () == rrepHeader.GetFirstHop ())
        {
          b->ReversePathInsert (revNextHop , revLastHop );
          b->ForwardPathInsert (forwardPath->GetNextHop (), forwardPath->GetLastHop ());
          // route advertisement
          if (toDst.GetAdvertisedHopCount () == INFINITY2)
            {
              toDst.SetAdvertisedHopCount (toDst.PathGetMaxHopCount ());
            }
          //NS_LOG_UNCOND("CHECK DEEP HOP COUNT = " << (int)hop);
          rrepHeader.SetHopCount (hop);
          rrepHeader.SetMRE( std::min(rrepHeader.GetMRE(), GetRemainingEnergy()));//todo make advertised for energy also
          //reversePath->SetExpire (Simulator::Now() + m_activeRouteTimeout);  
          // CHANGE
          toDst.SetError (true);
          m_routingTable.Update(toDst);
	      }
    }
  else
    {
      return;
    }
  nextHop = revNextHop;
  #endif // AOMDV_LINK_DISJOINT_PATHS
  //NS_LOG_UNCOND("SENDING RREP FROM " << receiver << " to " << toOrigin.PathFind ()->GetNextHop() << " ,id = "<< 
  //              rrepHeader.GetRequestID() << " ,at = " << Simulator::Now().GetSeconds());
  
  rrepHeader.SetX(pos.first);
  rrepHeader.SetY(pos.second);
  rrepHeader.SetSquaredDistance(squaredDistance);
  SocketIpTtlTag tag;
  p->RemovePacketTag (tag);
  if (tag.GetTtl () < 2)
    {
      //NS_LOG_UNCOND ("TTL exceeded. Drop RREP destination " << dst << " origin " << rrepHeader.GetOrigin () << " TTL  = " << (int)tag.GetTtl());
      return;//TODO THIS IS HAPPENING IN THE FIRST HOP ITSELF
    }
  Ptr<Packet> packet = Create<Packet> ();
  SocketIpTtlTag ttl;
  ttl.SetTtl (tag.GetTtl () - 1);
  packet->AddPacketTag (ttl);
  packet->AddHeader (rrepHeader);
  TypeHeader tHeader (AOMDVTYPE_RREP);
  packet->AddHeader (tHeader);
  Ptr<Socket> socket = FindSocketWithInterfaceAddress (toOrigin.PathFind ()->GetInterface ());
  NS_ASSERT (socket);
  
  socket->SendTo (packet, 0, InetSocketAddress (nextHop, AOMDV_PORT));
}

void
RoutingProtocol::RecvReplyAck (Ipv4Address neighbor)
{
  NS_LOG_FUNCTION (this);
  RoutingTableEntry rt;
  if(m_routingTable.LookupRoute (neighbor, rt))
    {
      rt.m_ackTimer.Cancel ();
      rt.SetFlag (VALID);
      m_routingTable.Update (rt);
    }
}

void
RoutingProtocol::ProcessHello (RrepHeader const & rrepHeader, Ipv4Address receiver )
{
  //NS_LOG_UNCOND (this << "from " << rrepHeader.GetDst ());
  /*
   *  Whenever a node receives a Hello message from a neighbor, the node
   * SHOULD make sure that it has an active route to the neighbor, and
   * create one if necessary.
   */
  RoutingTableEntry toNeighbor;
  if (!m_routingTable.LookupRoute (rrepHeader.GetDst (), toNeighbor) || (toNeighbor.GetNumberofPaths() == 0))
    {
      Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (receiver));
      RoutingTableEntry newEntry (/*dst=*/ rrepHeader.GetDst (), /*validSeqNo=*/ true, /*seqno=*/ rrepHeader.GetDstSeqno (), 
                              /*lifeTime=*/ rrepHeader.GetLifeTime ());
      newEntry.PathInsert (/*device=*/ dev, /*nextHop=*/ rrepHeader.GetDst (), /*hop=*/ 1, 
                           /*expireTime=*/ rrepHeader.GetLifeTime (), 
                           /*lastHop=*/ rrepHeader.GetFirstHop (), 
                           /*iface=*/ m_ipv4->GetAddress (m_ipv4->GetInterfaceForAddress (receiver), 0), INFINITY3, 100000, 0);//wrong
      m_routingTable.AddRoute (newEntry);
    }
  else
    {
      RoutingTableEntry::Path *neighborPath = toNeighbor.PathFind ();
      toNeighbor.SetLifeTime (std::max (Time (m_allowedHelloLoss * m_helloInterval), toNeighbor.GetLifeTime ()));
      toNeighbor.SetSeqNo (rrepHeader.GetDstSeqno ());
      toNeighbor.SetValidSeqNo (true);
      toNeighbor.SetFlag (VALID);
      neighborPath->SetOutputDevice (m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (receiver)));
      neighborPath->SetInterface (m_ipv4->GetAddress (m_ipv4->GetInterfaceForAddress (receiver), 0));
      neighborPath->SetHopCount (1);
      neighborPath->SetNextHop (rrepHeader.GetDst ());
      neighborPath->SetExpire (std::max (Time (m_allowedHelloLoss * m_helloInterval), toNeighbor.GetLifeTime ()));
      neighborPath->SetMRE (INFINITY3);
      m_routingTable.Update (toNeighbor);
    }
  if (m_enableHello)
    {
      m_nb.Update (rrepHeader.GetDst (), Time (m_allowedHelloLoss * m_helloInterval));
    }
}

void
RoutingProtocol::RecvError (Ptr<Packet> p, Ipv4Address src )
{
  NS_LOG_UNCOND (this << " from " << src << " received at " << m_ipv4->GetAddress(1,0).GetLocal() << " at " << Simulator::Now().GetSeconds());
  RerrHeader rerrHeader;
  p->RemoveHeader (rerrHeader);
  std::map<Ipv4Address, uint32_t> dstWithNextHopSrc;
  std::map<Ipv4Address, uint32_t> unreachable;
  m_routingTable.GetListOfDestinationWithNextHop (src, dstWithNextHopSrc);
  std::pair<Ipv4Address, uint32_t> un;
  while (rerrHeader.RemoveUnDestination (un))
    { 

      for (std::map<Ipv4Address, uint32_t>::const_iterator i =
           dstWithNextHopSrc.begin (); i != dstWithNextHopSrc.end (); ++i)
      {
        if (i->first == un.first)
          {
            unreachable.insert (un);
          }
      }
    }

  std::vector<Ipv4Address> precursors;
  for (std::map<Ipv4Address, uint32_t>::const_iterator i = unreachable.begin ();
       i != unreachable.end ();)
    {
      if (!rerrHeader.AddUnDestination (i->first, i->second))
        {
          TypeHeader typeHeader (AOMDVTYPE_RERR);
          Ptr<Packet> packet = Create<Packet> ();
          SocketIpTtlTag tag;
          tag.SetTtl (1);
          packet->AddPacketTag (tag);
          packet->AddHeader (rerrHeader);
          packet->AddHeader (typeHeader);
          SendRerrMessage (packet, precursors);
          rerrHeader.Clear ();
        }
      else
        {
          RoutingTableEntry toDst;
          m_routingTable.LookupRoute (i->first, toDst);
          toDst.GetPrecursors (precursors);
          //NS_LOG_UNCOND(precursors.size());
          ++i;
        }
    }
  if (rerrHeader.GetDestCount () != 0)
    {
      TypeHeader typeHeader (AOMDVTYPE_RERR);
      Ptr<Packet> packet = Create<Packet> ();
      SocketIpTtlTag tag;
      tag.SetTtl (1);
      packet->AddPacketTag (tag);
      packet->AddHeader (rerrHeader);
      packet->AddHeader (typeHeader);
      SendRerrMessage (packet, precursors);
    }
  m_routingTable.InvalidateRoutesWithDst (unreachable);
}

void
RoutingProtocol::RouteRequestTimerExpire (Ipv4Address dst)
{
  NS_LOG_LOGIC (this);
  RoutingTableEntry toDst;
  //NS_LOG_UNCOND(dst);
  if (m_routingTable.LookupValidRoute (dst, toDst))
    {
      m_addressReqTimer[dst].Remove ();
      m_addressReqTimer.erase (dst);
      SendPacketFromQueue (dst, toDst.PathLoadBalancedFind()->GetRoute ());
      NS_LOG_LOGIC ("route to " << dst << " found");
      return;
    }
  /*
   *  If a route discovery has been attempted RreqRetries times at the maximum TTL without
   *  receiving any RREP, all data packets destined for the corresponding destination SHOULD be
   *  dropped from the buffer and a Destination Unreachable message SHOULD be delivered to the application.
   */
  if (toDst.GetRreqCnt () == m_rreqRetries)
    {
      toDst.PathDeleteLongestUnnecessary();
      if(toDst.GetFlag () == IN_SEARCH && toDst.GetNumberofPaths ())
      {
        m_addressReqTimer[dst].Remove ();
        m_addressReqTimer.erase (dst);
        toDst.SetFlag(VALID);
        m_routingTable.Update(toDst);
        SendPacketFromQueue (dst, toDst.PathLoadBalancedFind()->GetRoute ());//the second argument holds no value now
        NS_LOG_LOGIC ("route to " << dst << " found");
        return;
      }
      NS_LOG_LOGIC ("route discovery to " << dst << " has been attempted RreqRetries (" << m_rreqRetries << ") times");
      m_addressReqTimer.erase (dst);
      m_routingTable.DeleteRoute (dst);
      NS_LOG_DEBUG ("Route not found. Drop all packets with dst " << dst);
      m_queue.DropPacketWithDst (dst);
      return;
    }

  if (toDst.GetFlag () == IN_SEARCH)
    {
      // if(toDst.GetNumberofPaths () >= AOMDV_MAX_PATHS) {
      //   m_addressReqTimer[dst].Remove ();
      //   m_addressReqTimer.erase (dst);
      //   toDst.SetFlag(VALID);
      //   m_routingTable.Update(toDst);
      //   SendPacketFromQueue (dst, toDst.PathLoadBalancedFind()->GetRoute ());
      //   NS_LOG_LOGIC ("route to " << dst << " found");//todo has to mark the routes and something else as valid
      //   return;
      // } 
      NS_LOG_LOGIC ("Resend RREQ to " << dst << " ttl " << m_netDiameter);
      SendRequest (dst);
    }
  else
    {
      NS_LOG_DEBUG ("Route down. Stop search. Drop packet with destination " << dst);
      m_addressReqTimer.erase (dst);
      m_routingTable.DeleteRoute (dst);
      m_queue.DropPacketWithDst (dst);
    }
}

void
RoutingProtocol::HelloTimerExpire ()
{
  NS_LOG_FUNCTION (this);
  Time offset = Time (Seconds (0));
  if (m_lastBcastTime > Time (Seconds (0)))
    {
      offset = Simulator::Now () - m_lastBcastTime;
      NS_LOG_DEBUG ("Hello deferred due to last bcast at:" << m_lastBcastTime);
    }
  else
    {
      //SendHello ();//DEBUG
    }
  m_htimer.Cancel ();
  Time diff = m_helloInterval - offset;
  m_htimer.Schedule (std::max (Time (Seconds (0)), diff));
  m_lastBcastTime = Time (Seconds (0));
}

void
RoutingProtocol::RreqRateLimitTimerExpire ()
{
  NS_LOG_FUNCTION (this);
  m_rreqCount = 0;
  m_rreqRateLimitTimer.Schedule (Seconds (1));
}

void
RoutingProtocol::RerrRateLimitTimerExpire ()
{
  NS_LOG_FUNCTION (this);
  m_rerrCount = 0;
  m_rerrRateLimitTimer.Schedule (Seconds (1));
}

void
RoutingProtocol::AckTimerExpire (Ipv4Address neighbor, Time blacklistTimeout)
{
  NS_LOG_FUNCTION (this);
  m_routingTable.MarkLinkAsUnidirectional (neighbor, blacklistTimeout);
}

void
RoutingProtocol::SendHello ()
{
  NS_LOG_FUNCTION (this);
  /* Broadcast a RREP with TTL = 1 with the RREP message fields set as follows:
   *   Destination IP Address         The node's IP address.
   *   Destination Sequence Number    The node's latest sequence number.
   *   Hop Count                      0
   *   Lifetime                       AllowedHelloLoss * HelloInterval
   */
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
    {
      Ptr<Socket> socket = j->first;
      Ipv4InterfaceAddress iface = j->second;
      RrepHeader helloHeader (/*prefix size=*/ 0, /*hops=*/ 1, /*dst=*/ iface.GetLocal (), /*dst seqno=*/ m_seqNo,
                              /*origin=*/ iface.GetLocal (), /*requestId=*/ 0, /*firstHop=*/ Ipv4Address (),
                              /*lifetime=*/ Time (m_allowedHelloLoss * m_helloInterval), /*mre*/ INFINITY3);
      Ptr<Packet> packet = Create<Packet> ();
      SocketIpTtlTag tag;
      tag.SetTtl (1);
      packet->AddPacketTag (tag);
      packet->AddHeader (helloHeader);
      TypeHeader tHeader (AOMDVTYPE_RREP);
      packet->AddHeader (tHeader);
      // Send to all-hosts broadcast if on /32 addr, subnet-directed otherwise
      Ipv4Address destination;
      if (iface.GetMask () == Ipv4Mask::GetOnes ())
        {
          destination = Ipv4Address ("255.255.255.255");
        }
      else
        { 
          destination = iface.GetBroadcast ();
        }
      Time jitter = Time (MilliSeconds (m_uniformRandomVariable->GetInteger (0, 10)));
      Simulator::Schedule (jitter, &RoutingProtocol::SendTo, this , socket, packet, destination);
    }
}

void
RoutingProtocol::SendPacketFromQueue (Ipv4Address dst, Ptr<Ipv4Route> route)
{
  NS_LOG_FUNCTION (this);
  QueueEntry queueEntry;
  RoutingTableEntry toDst;
  m_routingTable.LookupRoute(dst, toDst);
  while (m_queue.Dequeue (dst, queueEntry))
    {
      DeferredRouteOutputTag tag;
      Ptr<Packet> p = ConstCast<Packet> (queueEntry.GetPacket ());
      if (p->RemovePacketTag (tag) && 
          tag.GetInterface() != -1 &&
          tag.GetInterface() != m_ipv4->GetInterfaceForDevice (route->GetOutputDevice ()))
        {
          //NS_LOG_UNCOND ("Output device doesn't match. Dropped.");
          return;
        }
      route = toDst.PathLoadBalancedFind()->GetRoute ();
      UnicastForwardCallback ucb = queueEntry.GetUnicastForwardCallback ();
      Ipv4Header header = queueEntry.GetIpv4Header ();
      header.SetSource (route->GetSource ());
      header.SetTtl (header.GetTtl () + 1); // compensate extra TTL decrement by fake loopback routing
      //NS_LOG_UNCOND("SENDING FROM QUEUE" << route->GetGateway() << " TIME = " << Simulator::Now().GetSeconds());
      ucb (route, p, header);
    }
}

void
RoutingProtocol::SendRerrWhenBreaksLinkToNextHop (Ipv4Address nextHop)
{
  NS_LOG_FUNCTION (this << nextHop);
  RerrHeader rerrHeader;
  std::vector<Ipv4Address> precursors;
  std::map<Ipv4Address, uint32_t> unreachable;

  RoutingTableEntry toNextHop;
  if (!m_routingTable.LookupRoute (nextHop, toNextHop))
    return;
  toNextHop.GetPrecursors (precursors);
  rerrHeader.AddUnDestination (nextHop, toNextHop.GetSeqNo ());
  m_routingTable.GetListOfDestinationWithNextHop (nextHop, unreachable);
  for (std::map<Ipv4Address, uint32_t>::const_iterator i = unreachable.begin (); i
       != unreachable.end ();)
    {
      if (!rerrHeader.AddUnDestination (i->first, i->second))
        {
          NS_LOG_LOGIC ("Send RERR message with maximum size.");
          TypeHeader typeHeader (AOMDVTYPE_RERR);
          Ptr<Packet> packet = Create<Packet> ();
          SocketIpTtlTag tag;
          tag.SetTtl (1);
          packet->AddPacketTag (tag);
          packet->AddHeader (rerrHeader);
          packet->AddHeader (typeHeader);
          SendRerrMessage (packet, precursors);
          rerrHeader.Clear ();
        }
      else
        {
          RoutingTableEntry toDst;
          m_routingTable.LookupRoute (i->first, toDst);
          toDst.GetPrecursors (precursors);
          ++i;
        }
    }
  if (rerrHeader.GetDestCount () != 0)
    {
      TypeHeader typeHeader (AOMDVTYPE_RERR);
      Ptr<Packet> packet = Create<Packet> ();
      SocketIpTtlTag tag;
      tag.SetTtl (1);
      packet->AddPacketTag (tag);
      packet->AddHeader (rerrHeader);
      packet->AddHeader (typeHeader);
      SendRerrMessage (packet, precursors);
    }
  unreachable.insert (std::make_pair (nextHop, toNextHop.GetSeqNo ()));
  m_routingTable.InvalidateRoutesWithDst (unreachable);
}

void
RoutingProtocol::SendRerrWhenNoRouteToForward (Ipv4Address dst,
                                               uint32_t dstSeqNo, Ipv4Address origin)
{
  NS_LOG_FUNCTION (this);
  // A node SHOULD NOT originate more than RERR_RATELIMIT RERR messages per second.
  if (m_rerrCount == m_rerrRateLimit)
    {
      // Just make sure that the RerrRateLimit timer is running and will expire
      NS_ASSERT (m_rerrRateLimitTimer.IsRunning ());
      // discard the packet and return
      NS_LOG_LOGIC ("RerrRateLimit reached at " << Simulator::Now ().GetSeconds () << " with timer delay left " 
                                                << m_rerrRateLimitTimer.GetDelayLeft ().GetSeconds ()
                                                << "; suppressing RERR");
      return;
    }
  RerrHeader rerrHeader;
  rerrHeader.AddUnDestination (dst, dstSeqNo);
  RoutingTableEntry toOrigin;
  Ptr<Packet> packet = Create<Packet> ();
  SocketIpTtlTag tag;
  tag.SetTtl (1);
  packet->AddPacketTag (tag);
  packet->AddHeader (rerrHeader);
  packet->AddHeader (TypeHeader (AOMDVTYPE_RERR));
  if (m_routingTable.LookupValidRoute (origin, toOrigin))
    {
      Ptr<Socket> socket = FindSocketWithInterfaceAddress (
          toOrigin.PathFind()->GetInterface ());
      NS_ASSERT (socket);
      NS_LOG_LOGIC ("Unicast RERR to the source of the data transmission");
      socket->SendTo (packet, 0, InetSocketAddress (toOrigin.PathFind()->GetNextHop (), AOMDV_PORT));
    }
  else
    {
      for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator i =
             m_socketAddresses.begin (); i != m_socketAddresses.end (); ++i)
        {
          Ptr<Socket> socket = i->first;
          Ipv4InterfaceAddress iface = i->second;
          NS_ASSERT (socket);
          NS_LOG_LOGIC ("Broadcast RERR message from interface " << iface.GetLocal ());
          // Send to all-hosts broadcast if on /32 addr, subnet-directed otherwise
          Ipv4Address destination;
          if (iface.GetMask () == Ipv4Mask::GetOnes ())
            {
              destination = Ipv4Address ("255.255.255.255");
            }
          else
            { 
              destination = iface.GetBroadcast ();
            }
          socket->SendTo (packet->Copy (), 0, InetSocketAddress (destination, AOMDV_PORT));
        }
    }
}

void
RoutingProtocol::SendRerrMessage (Ptr<Packet> packet, std::vector<Ipv4Address> precursors)
{
  NS_LOG_FUNCTION (this);
  //NS_ASSERT(1 == 0);
  if (precursors.empty ())
    {
      NS_LOG_LOGIC ("No precursors");
      return;
    }
  // A node SHOULD NOT originate more than RERR_RATELIMIT RERR messages per second.
  if (m_rerrCount == m_rerrRateLimit)
    {
      // Just make sure that the RerrRateLimit timer is running and will expire
      NS_ASSERT (m_rerrRateLimitTimer.IsRunning ());
      // discard the packet and return
      NS_LOG_LOGIC ("RerrRateLimit reached at " << Simulator::Now ().GetSeconds () << " with timer delay left " 
                                                << m_rerrRateLimitTimer.GetDelayLeft ().GetSeconds ()
                                                << "; suppressing RERR");
      return;
    }
  // If there is only one precursor, RERR SHOULD be unicast toward that precursor
  if (precursors.size () == 1)
    {
      RoutingTableEntry toPrecursor;
      if (m_routingTable.LookupValidRoute (precursors.front (), toPrecursor))
        {
          Ptr<Socket> socket = FindSocketWithInterfaceAddress (toPrecursor.PathFind ()->GetInterface ());
          NS_ASSERT (socket);
          NS_LOG_LOGIC ("one precursor => unicast RERR to " << toPrecursor.GetDestination () << " from " << 
                         toPrecursor.PathFind ()->GetInterface ().GetLocal ());
          Simulator::Schedule (Time (MilliSeconds (m_uniformRandomVariable->GetInteger (0, 10))), &RoutingProtocol::SendTo, this, socket, packet, precursors.front ());
          m_rerrCount++;
        }
      return;
    }

  //  Should only transmit RERR on those interfaces which have precursor nodes for the broken route
  std::vector<Ipv4InterfaceAddress> ifaces;
  RoutingTableEntry toPrecursor;
  for (std::vector<Ipv4Address>::const_iterator i = precursors.begin (); i != precursors.end (); ++i)
    {
      if (m_routingTable.LookupValidRoute (*i, toPrecursor) && 
          std::find (ifaces.begin (), ifaces.end (), toPrecursor.PathFind ()->GetInterface ()) == ifaces.end ())
        {
          ifaces.push_back (toPrecursor.PathFind ()->GetInterface ());
        }
    }

  for (std::vector<Ipv4InterfaceAddress>::const_iterator i = ifaces.begin (); i != ifaces.end (); ++i)
    {
      Ptr<Socket> socket = FindSocketWithInterfaceAddress (*i);
      NS_ASSERT (socket);
      NS_LOG_LOGIC ("Broadcast RERR message from interface " << i->GetLocal ());
      // std::cout << "Broadcast RERR message from interface " << i->GetLocal () << std::endl;
      // Send to all-hosts broadcast if on /32 addr, subnet-directed otherwise
      Ptr<Packet> p = packet->Copy ();
      Ipv4Address destination;
      if (i->GetMask () == Ipv4Mask::GetOnes ())
        {
          destination = Ipv4Address ("255.255.255.255");
        }
      else
        { 
          destination = i->GetBroadcast ();
        }
      Simulator::Schedule (Time (MilliSeconds (m_uniformRandomVariable->GetInteger (0, 10))), &RoutingProtocol::SendTo, this, socket, p, destination);
    }
}

Ptr<Socket>
RoutingProtocol::FindSocketWithInterfaceAddress (Ipv4InterfaceAddress addr ) const
{
  NS_LOG_FUNCTION (this << addr);
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
         m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
    {
      Ptr<Socket> socket = j->first;
      Ipv4InterfaceAddress iface = j->second;
      if (iface == addr)
        return socket;
    }
  Ptr<Socket> socket;
  return socket;
}

Ptr<Socket>
RoutingProtocol::FindSubnetBroadcastSocketWithInterfaceAddress (Ipv4InterfaceAddress addr ) const
{
  NS_LOG_FUNCTION (this << addr);
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
         m_socketSubnetBroadcastAddresses.begin (); j != m_socketSubnetBroadcastAddresses.end (); ++j)
    {
      Ptr<Socket> socket = j->first;
      Ipv4InterfaceAddress iface = j->second;
      if (iface == addr)
        return socket;
    }
  Ptr<Socket> socket;
  return socket;
}

void
RoutingProtocol::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  uint32_t startTime;
  if (m_enableHello)
    {
      m_htimer.SetFunction (&RoutingProtocol::HelloTimerExpire, this);
      startTime = m_uniformRandomVariable->GetInteger (0, 100);
      NS_LOG_DEBUG ("Starting at time " << startTime << "ms");
      m_htimer.Schedule (MilliSeconds (startTime));
    }
  Ipv4RoutingProtocol::DoInitialize ();
}

} //namespace aomdv
} //namespace ns3
