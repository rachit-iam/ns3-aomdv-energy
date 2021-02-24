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

#ifndef AOMDV_ID_CACHE_H
#define AOMDV_ID_CACHE_H

#include "ns3/ipv4-address.h"
#include "ns3/simulator.h"
#include <vector>

namespace ns3
{
namespace aomdv
{
// AOMDV code
/*
  Route List
*/
class AOMDVRoute 
{
public:
  AOMDVRoute(Ipv4Address nextHop = 0, Ipv4Address lastHop = 0) : m_nextHop (nextHop) , m_lastHop (lastHop) {}
  void SetNextHop (Ipv4Address nextHop) { m_nextHop = nextHop; }
  Ipv4Address GetNextHop () const { return m_nextHop; }
  void SetLastHop (Ipv4Address lastHop) { m_lastHop = lastHop; }
  Ipv4Address GetLastHop () const { return m_lastHop; }
private:
  Ipv4Address m_nextHop;
  Ipv4Address m_lastHop;
};

/**
 * \ingroup aomdv
 * 
 * \brief Unique packets identification cache used for simple duplicate detection.
 */
class IdCache
{
public:
  /// c-tor
  IdCache (Time lifetime) : m_lifetime (lifetime) {}
  /// Unique packet ID
  struct UniqueId
  {
    /// ID is supposed to be unique in single address context (e.g. sender address)
    Ipv4Address m_context;
    /// The id
    uint32_t m_id;
    /// When record will expire
    Time m_expire;
    int	count;
    std::vector<AOMDVRoute> m_reversePathList;     // List of reverse paths used for forwarding replies
    std::vector<AOMDVRoute> m_forwardPathList;     // List of forward paths advertised already
    void ReversePathInsert (Ipv4Address nextHop, Ipv4Address lastHop = 0);
    bool ReversePathLookup (Ipv4Address nextHop, Ipv4Address lastHop = 0);
    void ForwardPathInsert (Ipv4Address nextHop, Ipv4Address lastHop = 0);
    bool ForwardPathLookup (Ipv4Address nextHop, Ipv4Address lastHop = 0);
  };

  void InsertId (Ipv4Address addr, uint32_t id);
  struct UniqueId* GetId (Ipv4Address addr, uint32_t id);
  /// Check that entry (addr, id) exists in cache. Add entry, if it doesn't exist.
  bool IsDuplicate (Ipv4Address addr, uint32_t id);
  /// Remove all expired entries
  void Purge ();
  /// Return number of entries in cache
  uint32_t GetSize ();
  /// Set lifetime for future added entries.
  void SetLifetime (Time lifetime) { m_lifetime = lifetime; }
  /// Return lifetime for existing entries in cache
  Time GetLifeTime () const { return m_lifetime; }

private:
  
  struct IsExpired
  {
    bool operator() (const struct UniqueId & u) const
    {
      return (u.m_expire < Simulator::Now ());
    }
  };
  /// Already seen IDs
  std::vector<UniqueId> m_idCache;
  /// Default lifetime for ID records
  Time m_lifetime;
};
}
}
#endif /* AOMDV_ID_CACHE_H */
