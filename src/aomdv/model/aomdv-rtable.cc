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

#include "aomdv-rtable.h"
#include <algorithm>
#include <iomanip>
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <iostream>

NS_LOG_COMPONENT_DEFINE ("AomdvRoutingTable");

namespace ns3
{
namespace aomdv
{

RoutingTableEntry::Path::Path (Ptr<NetDevice> dev, Ipv4Address dst, Ipv4Address nextHop, uint16_t hopCount, 
                               Time expireTime, Ipv4Address lastHop, Ipv4InterfaceAddress iface,
                                uint32_t MRE, uint32_t squaredDistance, uint64_t delay) :
  m_hopCount (hopCount), m_expire (expireTime + Simulator::Now ()), m_lastHop (lastHop), 
  m_iface (iface), m_ts(Simulator::Now ()), m_pathError(false), m_MRE(MRE), m_squaredDistance(squaredDistance), m_delay (delay)
{
  m_ipv4Route = Create<Ipv4Route> ();
  m_ipv4Route->SetDestination (dst);
  m_ipv4Route->SetGateway (nextHop);
  m_ipv4Route->SetSource (m_iface.GetLocal ());
  m_ipv4Route->SetOutputDevice (dev);
}

void
RoutingTableEntry::Path::Print (Ptr<OutputStreamWrapper> stream) const
{
  std::ostream* os = stream->GetStream ();
  *os << "\n\t\t\t\t\t";
  *os << m_ipv4Route->GetGateway ()
      << "\t" << m_iface.GetLocal () ;
  *os << "\t";
  *os << std::setiosflags (std::ios::fixed) << 
  std::setiosflags (std::ios::left) << std::setprecision (2) <<
  std::setw (14) << (m_expire - Simulator::Now ()).GetSeconds ();
  *os << "\t" << m_hopCount;
  *os << "\t" << m_MRE << "\t\t" << m_squaredDistance << "\t\t" << m_delay << "\n";
}

  /*
 The Routing Table Entry
 */

RoutingTableEntry::RoutingTableEntry (Ipv4Address dst, bool vSeqNo, uint32_t seqNo, Time lifetime) :
  m_ackTimer (Timer::CANCEL_ON_DESTROY), m_dst(dst) ,
  m_validSeqNo (vSeqNo), m_seqNo (seqNo), 
  m_lifeTime (lifetime + Simulator::Now ()), m_flag (VALID), m_reqCount (0), 
  m_blackListState (false), m_blackListTimeout (Simulator::Now ()),
  m_advertisedHopCount (INFINITY2), m_highestSeqnoHeard (0),
  m_lastHopCount (0),//changed this from infinity2 to 0 
  m_numPaths (0), m_error (false)
{
}

RoutingTableEntry::~RoutingTableEntry ()
{
}

/* 
Our contribution 
*/
void 
RoutingTableEntry::PrintPaths() 
{
  for (std::vector<Path>::const_iterator i = m_pathList.begin(); i != m_pathList.end(); ++i) 
    {
      std::cout<<i->GetNextHop ()<<" "<<i->m_hopCount<<" "<<i->m_lastHop;  
    }
}

struct RoutingTableEntry::Path* 
RoutingTableEntry::PathInsert (Ptr<NetDevice> dev, Ipv4Address nextHop, uint16_t hopCount, 
                               Time expireTime, Ipv4Address lastHop, Ipv4InterfaceAddress iface,
                                uint32_t MRE, uint32_t squaredDistance, uint64_t delay)
{
  Path *p = new Path(dev, m_dst, nextHop, hopCount, expireTime, lastHop, iface, MRE, squaredDistance, delay);
  m_pathList.push_back (*p);
  m_numPaths += 1;     //TODO
  //RoutingTableEntry::Path *p = (struct RoutingTableEntry::Path*)malloc(sizeof(struct RoutingTableEntry::Path));
  return p;
}

struct RoutingTableEntry::Path* 
RoutingTableEntry::PathLookup (Ipv4Address id) 
{
  NS_LOG_FUNCTION (this << id);
  for (std::vector<Path>::iterator i = m_pathList.begin (); i!= m_pathList.end (); ++i)
    {
      if (i->m_ipv4Route->GetGateway () == id)
        {
          //NS_LOG_UNCOND ("Path " << id << " found");
          Path *path = &(*i);
          return path;
        }
    }
  NS_LOG_LOGIC ("Path " << id << " not found");
  return NULL;
}


struct RoutingTableEntry::Path* 
RoutingTableEntry::PathLookupDisjoint (Ipv4Address nh, Ipv4Address lh) 
{
  NS_LOG_FUNCTION (this << nh << lh);
  for (std::vector<Path>::iterator i = m_pathList.begin (); i!= m_pathList.end (); ++i)
    {
      if (i->m_ipv4Route->GetGateway () == nh && i->m_lastHop == lh)
        {
          NS_LOG_LOGIC ("Disjoint Path " << nh << lh << " found");
          Path *path = &(*i);
          return path;
        }
    }
  NS_LOG_LOGIC ("Disjoint Path " << nh << lh <<" not found");
  return NULL;
}

bool 
RoutingTableEntry::PathNewDisjoint (Ipv4Address nh, Ipv4Address lh) 
{
  NS_LOG_FUNCTION (this << nh << lh);
  for (std::vector<Path>::iterator i = m_pathList.begin (); i!= m_pathList.end (); ++i)
  {
      if (i->m_ipv4Route->GetGateway () == nh || i->m_lastHop == lh)
      {
        NS_LOG_LOGIC ("Disjoint New Path " << nh << lh << " found");
        return false;
      }
  }
  NS_LOG_LOGIC ("Disjoint New Path " << nh << lh <<" not found");
  return true;
}

struct RoutingTableEntry::Path* 
RoutingTableEntry::PathLookupLastHop (Ipv4Address id) 
{
  NS_LOG_FUNCTION (this << id);
  for (std::vector<Path>::iterator i = m_pathList.begin (); i!= m_pathList.end (); ++i)
  {
      if (i->m_lastHop == id)
      {
        NS_LOG_LOGIC ("Path " << id << " found");
        Path *path = &(*i);
        return path;
      }
  }
  NS_LOG_LOGIC ("Path " << id << " not found");
  return NULL;
}

//TODO
void 
RoutingTableEntry::PathDelete (Ipv4Address id)
{
  NS_LOG_FUNCTION (this << id);
  for (std::vector<Path>::iterator i = m_pathList.begin (); i!= m_pathList.end (); ++i)
  {
    if (i->m_ipv4Route->GetGateway () == id)
    {
      NS_LOG_LOGIC ("Path " << id << " found");
      m_pathList.erase (i);
      m_numPaths -= 1;   //TODO
    }
  }
}

void 
RoutingTableEntry::DeletePathFromInterface (Ipv4InterfaceAddress iface)
{
  for (std::vector<Path>::iterator j = m_pathList.begin (); j!= m_pathList.end (); ++j)
    {
      if (j->GetInterface () == iface)
        {
          std::vector<Path>::iterator tmp = j;
          m_pathList.erase (tmp);
        }
    }
}

void 
RoutingTableEntry::PathAllDelete (void)
{
  NS_LOG_FUNCTION (this);
  m_pathList.clear (); 
  m_numPaths = 0;
}

void
RoutingTableEntry::PathDeleteLongestUnnecessary (void) {
  while(m_pathList.size() > AOMDV_MAX_PATHS) {
    this->PathDeleteLongest();
  }
}

void 
RoutingTableEntry::PathDeleteLongest (void)
{
  Path *path = NULL;
  std::vector<Path>::iterator j;
  uint64_t comMetric = AOMDV_LOAD_BALANCING_STRATEGY == 1 ? 10000000000: 0, metric;
  for (std::vector<Path>::iterator i = m_pathList.begin (); i!= m_pathList.end (); ++i)
  {
    switch (AOMDV_LOAD_BALANCING_STRATEGY) {
      case 0:
        metric = i->m_squaredDistance;
        break;
      case 1:
        metric = i->m_MRE;
        break;
      case 2:
        metric = i->m_delay;
        break;
      default:
        NS_LOG_UNCOND("LOAD BALANCING STRATEGY NOT SET CORRECTLY");
    }
    if ((AOMDV_LOAD_BALANCING_STRATEGY != 1 && metric > comMetric) || (AOMDV_LOAD_BALANCING_STRATEGY == 1 && metric < comMetric))
    {
      //assert (i->hopcount != INFINITY2); //TODO
      path = &(*i);
      j = i;
      comMetric = metric;
    }
  }
  if (path)
  {
    m_pathList.erase(j);
    m_numPaths -= 1;
  }
}

bool 
RoutingTableEntry::PathEmpty (void) const
{
  return m_pathList.empty ();
}

struct RoutingTableEntry::Path * 
RoutingTableEntry::PathFind (void) 
{
  Path *path = NULL;
  //NS_LOG_UNCOND("PATH FIND CHECK" << m_pathList.size());
  std::vector<Path>::iterator i = m_pathList.begin ();
  path = &(*i);
  return path;
}

struct RoutingTableEntry::Path * 
RoutingTableEntry::PathLoadBalancedFind (void) //todo if more than one path then load balance
{
  Path *path = NULL;
  //NS_LOG_UNCOND("PATH FIND CHECK" << m_pathList.size());
  std::vector<Path>::iterator i = m_pathList.begin ();
  if(m_pathList.size() < 2) {
    path = &(*i);
    return path;
  }
  double sum = 0.0 , midSum = 0.0;
  Ptr<UniformRandomVariable> rr = CreateObject<UniformRandomVariable> ();
  for(; i != m_pathList.end () ; i++) {
    double metric = 0;
    switch (AOMDV_LOAD_BALANCING_STRATEGY) {
      case 0:
        metric = 10000.0 / i->m_squaredDistance;
        break;
      case 1:
        metric = i->m_MRE;
        break;
      case 2:
        metric = 1000000000.0 / i->m_delay;
        break;
      default:
        NS_LOG_UNCOND("LOAD BALANCING STRATEGY NOT SET CORRECTLY");
    }
    sum += metric;
  }
  double z = rr->GetValue(0.0 , 1.0)*sum;
  //NS_LOG_UNCOND("Random value = " << z << " Sum = " << sum);
  i = m_pathList.begin ();
  for( ; i != m_pathList.end() ; i++) {
    double metric = 0;
    switch (AOMDV_LOAD_BALANCING_STRATEGY) {
      case 0:
        metric = 10000.0 / i->m_squaredDistance;
        break;
      case 1:
        metric = i->m_MRE;
        break;
      case 2:
        metric = 1000000000.0 / i->m_delay;
        break;
      default:
        NS_LOG_UNCOND("LOAD BALANCING STRATEGY NOT SET CORRECTLY");
    }
    midSum += metric;
    if(z <= midSum) {
      path = &(*i);
      break;
    }
    //NS_LOG_UNCOND("Midsum = " << midSum);
  }
  if(path == NULL) {
    i = m_pathList.begin ();
    path = &(*i);
  }
  //NS_LOG_UNCOND("Next hop is " << path->GetNextHop ());
  return path;
}

struct RoutingTableEntry::Path* 
RoutingTableEntry::PathFindMinHop (void)
{
  Path *path = NULL;
  uint16_t minHopCount = 0xffff;
  for (std::vector<Path>::iterator i = m_pathList.begin (); i!= m_pathList.end (); ++i)
  {
    if (i->m_hopCount < minHopCount)
    {
      path = &(*i);
      minHopCount = i->m_hopCount;
    }
  }
  return path;
}

uint16_t 
RoutingTableEntry::PathGetMaxHopCount (void)
{
  uint16_t maxHopCount = 0;
  for (std::vector<Path>::iterator i = m_pathList.begin (); i!= m_pathList.end (); ++i)
  {
    if (i->m_hopCount > maxHopCount)
    {
      maxHopCount = i->m_hopCount;
    }
  }
  if(maxHopCount == 0) 
    {  
      return INFINITY2;
    }
  else 
    {
      return maxHopCount;
    }
} 

uint16_t 
RoutingTableEntry::PathGetMinHopCount (void)
{
  uint16_t minHopCount = INFINITY2;
  for (std::vector<Path>::iterator i = m_pathList.begin (); i!= m_pathList.end (); ++i)
  {
    if (i->m_hopCount < minHopCount)
    {
      minHopCount = i->m_hopCount;
    }
  }
  return minHopCount;
}  

Time 
RoutingTableEntry::PathGetMaxExpirationTime (void)
{
  Time maxExpireTime = Time ();
  for (std::vector<Path>::iterator i = m_pathList.begin (); i!= m_pathList.end (); ++i)
  {
    if (i->m_expire > maxExpireTime)
    {
      maxExpireTime = i->m_expire;
    }
  }
  return maxExpireTime;
} 

void 
RoutingTableEntry::PathPurge (void)
{
  Time now = Simulator::Now ();
  return;//todo changed but BAD
  bool cond;
  do
    {
      cond = false;
      for (std::vector<Path>::iterator i = m_pathList.begin (); i!= m_pathList.end (); ++i)
        {
          if (i->m_expire < now)
            {
              cond = true;
              m_pathList.erase(i);
              m_numPaths -= 1;
              break;
            }
        }
    } while(cond);
}

bool
RoutingTableEntry::InsertPrecursor (Ipv4Address id)
{
  NS_LOG_FUNCTION (this << id);
  if (!LookupPrecursor (id))
    {
      m_precursorList.push_back (id);
      return true;
    }
  else
    return false;
}

bool
RoutingTableEntry::LookupPrecursor (Ipv4Address id)
{
  NS_LOG_FUNCTION (this << id);
  for (std::vector<Ipv4Address>::const_iterator i = m_precursorList.begin (); i
       != m_precursorList.end (); ++i)
    {
      if (*i == id)
        {
          NS_LOG_LOGIC ("Precursor " << id << " found");
          return true;
        }
    }
  NS_LOG_LOGIC ("Precursor " << id << " not found");
  return false;
}

bool
RoutingTableEntry::DeletePrecursor (Ipv4Address id)
{
  NS_LOG_FUNCTION (this << id);
  std::vector<Ipv4Address>::iterator i = std::remove (m_precursorList.begin (),
                                                      m_precursorList.end (), id);
  if (i == m_precursorList.end ())
    {
      NS_LOG_LOGIC ("Precursor " << id << " not found");
      return false;
    }
  else
    {
      NS_LOG_LOGIC ("Precursor " << id << " found");
      m_precursorList.erase (i, m_precursorList.end ());
    }
  return true;
}

void
RoutingTableEntry::DeleteAllPrecursors ()
{
  NS_LOG_FUNCTION (this);
  m_precursorList.clear ();
}

bool
RoutingTableEntry::IsPrecursorListEmpty () const
{
  return m_precursorList.empty ();
}

void
RoutingTableEntry::GetPrecursors (std::vector<Ipv4Address> & prec) const
{
  NS_LOG_FUNCTION (this);
  if (IsPrecursorListEmpty ())
    return;
  for (std::vector<Ipv4Address>::const_iterator i = m_precursorList.begin (); i
       != m_precursorList.end (); ++i)
    {
      bool result = true;
      for (std::vector<Ipv4Address>::const_iterator j = prec.begin (); j
           != prec.end (); ++j)
        {
          if (*j == *i)
            result = false;
        }
      if (result){
        prec.push_back (*i);
      }
    }
}

void
RoutingTableEntry::GetPaths (std::vector<Path> & paths) const
{
  NS_LOG_FUNCTION (this);
  if (PathEmpty ())
    return;
  for (std::vector<Path>::const_iterator i = m_pathList.begin (); i
       != m_pathList.end (); ++i)
    {
      bool result = true;
      for (std::vector<Path>::const_iterator j = paths.begin (); j
           != paths.end (); ++j)
        {
          if (*j == *i)
            result = false;
        }
      if (result)
        paths.push_back (*i);
    }
}

void
RoutingTableEntry::Invalidate (Time badLinkLifetime)
{
  NS_LOG_FUNCTION (this << badLinkLifetime.GetSeconds ());
  if (m_flag == INVALID)
    return;
  m_flag = INVALID;
  m_reqCount = 0;
  m_lifeTime = badLinkLifetime + Simulator::Now ();
}

void
RoutingTableEntry::Print (Ptr<OutputStreamWrapper> stream) const
{
  std::ostream* os = stream->GetStream ();
  *os << m_dst << "\t";
  switch (m_flag)
    {
    case VALID:
      {
        *os << "UP";
        break;
      }
    case INVALID:
      {
        *os << "DOWN";
        break;
      }
    case IN_SEARCH:
      {
        *os << "IN_SEARCH";
        break;
      }
    }
  *os << "  " << (m_lifeTime - Simulator::Now()).GetSeconds();
   for (std::vector<Path>::const_iterator i = m_pathList.begin (); i!= m_pathList.end (); ++i)
     {
       i->Print (stream);
       *os << "\n\t\t\t\t";
     }
  *stream->GetStream () << "\n";
}

/*
 The Routing Table
 */

RoutingTable::RoutingTable (Time t) : 
  m_badLinkLifetime (t)
{
}

bool
RoutingTable::LookupRoute (Ipv4Address id, RoutingTableEntry & rt)
{
  NS_LOG_FUNCTION (this << id);
  Purge ();
  if (m_ipv4AddressEntry.empty ())
    {
      NS_LOG_LOGIC ("Route to " << id << " not found; m_ipv4AddressEntry is empty");
      return false;
    }
  std::map<Ipv4Address, RoutingTableEntry>::const_iterator i =
    m_ipv4AddressEntry.find (id);
  if (i == m_ipv4AddressEntry.end ())
    {
      NS_LOG_LOGIC ("Route to " << id << " not found");
      return false;
    }
  rt = i->second;
  NS_LOG_LOGIC ("Route to " << id << " found");
  return true;
}

bool
RoutingTable::LookupValidRoute (Ipv4Address id, RoutingTableEntry & rt)
{
  NS_LOG_FUNCTION (this << id);
  if (!LookupRoute (id, rt))
    {
      NS_LOG_LOGIC ("Route to " << id << " not found");
      return false;
    }
  NS_LOG_LOGIC ("Route to " << id << " flag is " << ((rt.GetFlag () == VALID) ? "valid" : "not valid"));
  return (rt.GetFlag () == VALID);
}

bool
RoutingTable::DeleteRoute (Ipv4Address dst)
{
  NS_LOG_FUNCTION (this << dst);
  Purge ();
  if (m_ipv4AddressEntry.erase (dst) != 0)
    {
      NS_LOG_LOGIC ("Route deletion to " << dst << " successful");
      return true;
    }
  NS_LOG_LOGIC ("Route deletion to " << dst << " not successful");
  return false;
}

bool
RoutingTable::AddRoute (RoutingTableEntry & rt)
{
  NS_LOG_FUNCTION (this);
  Purge ();
  if (rt.GetFlag () != IN_SEARCH)
    {
      rt.SetRreqCnt (0);
    }
  std::pair<std::map<Ipv4Address, RoutingTableEntry>::iterator, bool> result =
    m_ipv4AddressEntry.insert (std::make_pair (rt.GetDestination (), rt));
  return result.second;
}

bool
RoutingTable::Update (RoutingTableEntry & rt)
{
  NS_LOG_FUNCTION (this);
  std::map<Ipv4Address, RoutingTableEntry>::iterator i =
    m_ipv4AddressEntry.find (rt.GetDestination ());
  if (i == m_ipv4AddressEntry.end ())
    {
      NS_LOG_LOGIC ("Route update to " << rt.GetDestination () << " fails; not found");
      return false;
    }
  i->second = rt;
  if (i->second.GetFlag () != IN_SEARCH)
    {
      NS_LOG_LOGIC ("Route update to " << rt.GetDestination () << " set RreqCnt to 0");
      i->second.SetRreqCnt (0);
    }
  return true;
}

bool
RoutingTable::SetEntryState (Ipv4Address id, RouteFlags state)
{
  NS_LOG_FUNCTION (this);
  std::map<Ipv4Address, RoutingTableEntry>::iterator i =
    m_ipv4AddressEntry.find (id);
  if (i == m_ipv4AddressEntry.end ())
    {
      NS_LOG_LOGIC ("Route set entry state to " << id << " fails; not found");
      return false;
    }
  i->second.SetFlag (state);
  i->second.SetRreqCnt (0);
  NS_LOG_LOGIC ("Route set entry state to " << id << ": new state is " << state);
  return true;
}

void
RoutingTable::GetListOfDestinationWithNextHop (Ipv4Address nextHop, std::map<Ipv4Address, uint32_t> & unreachable )
{
  NS_LOG_FUNCTION (this);
  Purge ();
  unreachable.clear ();
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i =
         m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end (); ++i)
    {
      RoutingTableEntry::Path *p = i->second.PathLookup (nextHop);
      if (p != NULL)
        {  
          NS_LOG_LOGIC ("Unreachable insert " << i->first << " " << i->second.GetSeqNo ());
          unreachable.insert (std::make_pair (i->first, i->second.GetSeqNo ()));
        }
    }
}

void
RoutingTable::InvalidateRoutesWithDst (const std::map<Ipv4Address, uint32_t> & unreachable)
{
  NS_LOG_FUNCTION (this);
  Purge ();
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i =
         m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end (); ++i)
    {
      for (std::map<Ipv4Address, uint32_t>::const_iterator j =
             unreachable.begin (); j != unreachable.end (); ++j)
        {
          if ((i->first == j->first) && (i->second.GetFlag () == VALID))
            {
              NS_LOG_LOGIC ("Invalidate route with destination address " << i->first);
              //NS_LOG_UNCOND("UNREACHABLE = " << i->first);
              i->second.Invalidate (m_badLinkLifetime);
            }
        }
    }
}

void
RoutingTable::DeleteAllRoutesFromInterface (Ipv4InterfaceAddress iface)
{
  NS_LOG_FUNCTION (this);
  if (m_ipv4AddressEntry.empty ())
    return;
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i =
         m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end (); ++i)
    {
      i->second.DeletePathFromInterface (iface);
    }
}

void
RoutingTable::Purge ()
{
  NS_LOG_FUNCTION (this);
  if (m_ipv4AddressEntry.empty ())
    return;
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i =
         m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end ();)
    {
      if (i->second.GetLifeTime () < Seconds (0))
        {
          if (i->second.GetFlag () == INVALID)
            {
              std::map<Ipv4Address, RoutingTableEntry>::iterator tmp = i;
              ++i;
              m_ipv4AddressEntry.erase (tmp);
            }
          else if (i->second.GetFlag () == VALID)
            {
              NS_LOG_LOGIC ("Invalidate route with destination address " << i->first);
              i->second.Invalidate (m_badLinkLifetime);
              ++i;
            }
          else
            ++i;
        }
      else 
        {
          ++i;
        }
    }
}

void
RoutingTable::Purge (std::map<Ipv4Address, RoutingTableEntry> &table) const
{
  NS_LOG_FUNCTION (this);
  if (table.empty ())
    return;
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i =
         table.begin (); i != table.end ();)
    {
      if (i->second.GetLifeTime () < Seconds (0))
        {
          if (i->second.GetFlag () == INVALID)
            {
              std::map<Ipv4Address, RoutingTableEntry>::iterator tmp = i;
              ++i;
              table.erase (tmp);
            }
          else if (i->second.GetFlag () == VALID)
            {
              NS_LOG_LOGIC ("Invalidate route with destination address " << i->first);
              i->second.Invalidate (m_badLinkLifetime);
              ++i;
            }
          else
            ++i;
        }
      else 
        {
          ++i;
        }
    }
}

//contribution
bool 
RoutingTable::HasActiveRoutes () 
{
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator i =
         m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end (); ++i)
    {
      if(i->second.GetFlag () == VALID)
        {
          return true;
        }
    }
    return false;
}

bool
RoutingTable::MarkLinkAsUnidirectional (Ipv4Address neighbor, Time blacklistTimeout)
{
  NS_LOG_FUNCTION (this << neighbor << blacklistTimeout.GetSeconds ());
  std::map<Ipv4Address, RoutingTableEntry>::iterator i =
    m_ipv4AddressEntry.find (neighbor);
  if (i == m_ipv4AddressEntry.end ())
    {
      NS_LOG_LOGIC ("Mark link unidirectional to  " << neighbor << " fails; not found");
      return false;
    }
  i->second.SetUnidirectional (true);
  i->second.SetBlacklistTimeout (blacklistTimeout);
  i->second.SetRreqCnt (0);
  NS_LOG_LOGIC ("Set link to " << neighbor << " to unidirectional");
  return true;
}

void
RoutingTable::Print (Ptr<OutputStreamWrapper> stream) const
{
  std::map<Ipv4Address, RoutingTableEntry> table = m_ipv4AddressEntry;
  Purge (table);
  *stream->GetStream () << "\nAOMDV Routing table\n"
                        << "Destination\tFlag\tGateway\t\tInterface\tExpire\t\tHops\tR Energy\tSquared Distance\n";
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator i =
         table.begin (); i != table.end (); ++i)
    {
      i->second.Print (stream);
    }
  *stream->GetStream () << "\n";
}

}
}
