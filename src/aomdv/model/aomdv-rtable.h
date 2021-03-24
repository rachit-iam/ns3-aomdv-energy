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
#ifndef AOMDV_RTABLE_H
#define AOMDV_RTABLE_H

#include <stdint.h>
#include <cassert>
#include <map>
#include <sys/types.h>
#include "ns3/ipv4.h"
#include "ns3/ipv4-route.h"
#include "ns3/timer.h"
#include "ns3/net-device.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/random-variable-stream.h"
#include <vector>

#define INFINITY2 0xff
#define INFINITY3 9999999
#define AOMDV_MAX_PATHS 5
#define AOMDV_LOAD_BALANCING_STRATEGY 2 //0 - SQUARED DISTANCE , 1 - MRE, 2 - DELAY 

namespace ns3 {
namespace aomdv {

/**
 * \ingroup aomdv
 * \brief Route record states
 */
enum RouteFlags
{
  VALID = 0,          //!< VALID
  INVALID = 1,        //!< INVALID
  IN_SEARCH = 2,      //!< IN_SEARCH
};


/**
 * \ingroup aomdv
 * \brief Routing table entry
 */
class RoutingTableEntry
{
public:
  /// c-to
  RoutingTableEntry (Ipv4Address dst = Ipv4Address (), bool vSeqNo = false, uint32_t m_seqNo = 0,
                     Time lifetime = Simulator::Now());	//TODO

  ~RoutingTableEntry ();

  /// Path - contribution


  struct Path
  {  
    /** Ip route, include
     *   - destination address
     *   - source address
     *   - next hop address (gateway)
     *   - output device
     */
    Ptr<Ipv4Route> m_ipv4Route;
    uint16_t m_hopCount;   // hopcount through this nexthop
    Time m_expire;     // expiration timeout
    Ipv4Address m_lastHop;    // lasthop address
    /// Output interface address
    Ipv4InterfaceAddress m_iface;
    Time m_ts;         // time when we saw this nexthop
    // CHANG
    bool m_pathError;
    uint32_t m_MRE;
    uint32_t m_squaredDistance;
    uint64_t m_delay;
    
    Path (Ptr<NetDevice> dev, Ipv4Address dst, Ipv4Address nextHop, uint16_t hopCount, Time expireTime, 
          Ipv4Address lastHop, Ipv4InterfaceAddress iface, uint32_t MRE, uint32_t squaredDistance, uint64_t delay);

    Ptr<Ipv4Route> GetRoute () const { return m_ipv4Route; }
    void SetRoute (Ptr<Ipv4Route> r) { m_ipv4Route = r; }
    void SetNextHop (Ipv4Address nextHop) { m_ipv4Route->SetGateway (nextHop); }
    Ipv4Address GetNextHop () const { return m_ipv4Route->GetGateway (); }
    void SetLastHop (Ipv4Address lastHop) { m_lastHop = lastHop; }
    Ipv4Address GetLastHop () const { return m_lastHop; }
    void SetMRE (uint32_t MRE) { m_MRE = MRE; }
    uint32_t GetMRE () const { return m_MRE; }
    void SetSquaredDistance (uint32_t squaredDistance) { m_squaredDistance = squaredDistance; }
    uint32_t GetSquaredDistance () const { return m_squaredDistance; }
    void SetDelay (uint64_t delay) { m_delay = delay; }
    uint64_t GetDelay () { return m_delay; }
    void SetOutputDevice (Ptr<NetDevice> dev) { m_ipv4Route->SetOutputDevice (dev); }
    Ptr<NetDevice> GetOutputDevice () const { return m_ipv4Route->GetOutputDevice (); }
    void SetHopCount (uint16_t hop) { m_hopCount = hop; }
    uint16_t GetHopCount () const { return m_hopCount; }
    Ipv4InterfaceAddress GetInterface () const { return m_iface; }
    void SetInterface (Ipv4InterfaceAddress iface) { m_iface = iface; }
    void SetExpire (Time et) { m_expire = et + Simulator::Now (); }
    Time GetExpire () const { return m_expire - Simulator::Now (); }
  
    void Print (Ptr<OutputStreamWrapper> stream) const;

    friend bool operator == (Path const &a, Path const &b);
  };

  /// Path functions - contribution
  void PrintPaths ();
  struct Path* PathInsert (Ptr<NetDevice> dev, Ipv4Address nextHop, uint16_t hopCount, 
                           Time expireTime, Ipv4Address lastHop, Ipv4InterfaceAddress iface,
                           uint32_t MRE, uint32_t squaredDistance, uint64_t delay);
  struct Path* PathLookup (Ipv4Address id);
  struct Path* PathLookupDisjoint (Ipv4Address nh, Ipv4Address lh);
  bool PathNewDisjoint (Ipv4Address nh, Ipv4Address lh);
  struct Path* PathLookupLastHop (Ipv4Address id);
  void PathDelete (Ipv4Address id);
  void DeletePathFromInterface (Ipv4InterfaceAddress iface);
  void PathAllDelete (void);                  // delete all paths
  void PathDeleteLongestUnnecessary (void); 
  void PathDeleteLongest (void);          // delete longest path
  bool PathEmpty (void) const;                   // is the path list empty?
  struct Path * PathFind (void);            // find the path that we got first
  struct Path * PathLoadBalancedFind (void);
  struct Path* PathFindMinHop (void);            // find the shortest path
  uint16_t PathGetMaxHopCount (void);  
  uint16_t PathGetMinHopCount (void);  
  Time PathGetMaxExpirationTime (void); 
  void PathPurge (void);



  ///\name Precursors management
  //\{
  /**
   * Insert precursor in precursor list if it doesn't yet exist in the list
   * \param id precursor address
   * \return true on success
   */
  bool InsertPrecursor (Ipv4Address id);
  /**
   * Lookup precursor by address
   * \param id precursor address
   * \return true on success
   */
  bool LookupPrecursor (Ipv4Address id);
  /**
   * \brief Delete precursor
   * \param id precursor address
   * \return true on success
   */
  bool DeletePrecursor (Ipv4Address id);
  /// Delete all precursors
  void DeleteAllPrecursors ();
  /**
   * Check that precursor list empty
   * \return true if precursor list empty
   */
  bool IsPrecursorListEmpty () const;
  /**
   * Inserts precursors in vector prec if they does not yet exist in vector
   */
  void GetPrecursors (std::vector<Ipv4Address> & prec) const;
  //\}

  /// Mark entry as "down" (i.e. disable it)
  void Invalidate (Time badLinkLifetime);

  //AOMDV Code
  void GetPaths (std::vector<Path> & paths) const; 
  ///\name Fields
  //\{
  Ipv4Address GetDestination () const { return m_dst; }
  void SetValidSeqNo (bool s) { m_validSeqNo = s; }
  bool GetValidSeqNo () const { return m_validSeqNo; }
  void SetSeqNo (uint32_t sn) { m_seqNo = sn; }
  uint32_t GetSeqNo () const { return m_seqNo; }
  void SetLifeTime (Time lt) { m_lifeTime = lt + Simulator::Now (); }
  Time GetLifeTime () const { return m_lifeTime - Simulator::Now (); }
  void SetFlag (RouteFlags flag) { m_flag = flag; }
  RouteFlags GetFlag () const { return m_flag; }
  void SetRreqCnt (uint8_t n) { m_reqCount = n; }
  uint8_t GetRreqCnt () const { return m_reqCount; }
  void IncrementRreqCnt () { m_reqCount++; }
  void SetUnidirectional (bool u) { m_blackListState = u; }
  bool IsUnidirectional () const { return m_blackListState; }
  void SetBlacklistTimeout (Time t) { m_blackListTimeout = t; }
  Time GetBlacklistTimeout () const { return m_blackListTimeout; }
  void SetAdvertisedHopCount (uint32_t ahc ) { m_advertisedHopCount = ahc; }
  uint32_t GetAdvertisedHopCount () const { return m_advertisedHopCount; }
  void SetHighestSequenceNumberHeard (uint32_t hsh ) { m_highestSeqnoHeard = hsh; }
  uint32_t GetHighestSequenceNumberHeard () const { return m_highestSeqnoHeard; }
  void SetLastHopCount (uint32_t lhc ) { m_lastHopCount = lhc; }
  uint32_t GetLastHopCount () const { return m_lastHopCount; }
  void SetNumberofPaths (uint32_t np ) { m_numPaths = np; }
  uint32_t GetNumberofPaths () const { return m_numPaths; }
  void SetError (bool e) { m_error = e; }
  bool IsError () const { return m_error; }

  /// RREP_ACK timer
  Timer m_ackTimer;
  //\}

  /**
   * \brief Compare destination address
   * \return true if equal
   */
  bool operator== (Ipv4Address const  dst) const
  {
    return (m_dst == dst);
  }
  void Print (Ptr<OutputStreamWrapper> stream) const;

private:
  //Destination IP Address
  Ipv4Address m_dst;
  /// Valid Destination Sequence Number flag
  bool m_validSeqNo;
  /// Destination Sequence Number, if m_validSeqNo = true
  uint32_t m_seqNo;
  /// Hop Count (number of hops needed to reach destination)
  //uint16_t m_hops;
  /**
  * \brief Expiration or deletion time of the route
  *	Lifetime field in the routing table plays dual role --
  *	for an active route it is the expiration time, and for an invalid route
  *	it is the deletion time.
  */
  Time m_lifeTime;
  /// Routing flags: valid, invalid or in search
  RouteFlags m_flag;

  /// List of precursors
  std::vector<Ipv4Address> m_precursorList;
  /// List of Paths - contribution
  std::vector<Path> m_pathList;
  /// When I can send another request
  Time m_routeRequestTimout;
  /// Number of route requests
  uint8_t m_reqCount;
  /// Indicate if this entry is in "blacklist"
  bool m_blackListState;
  /// Time for which the node is put into the blacklist
  Time m_blackListTimeout;

  //AOMDV
  uint16_t  m_advertisedHopCount;
  uint32_t  m_highestSeqnoHeard;
  uint32_t  m_lastHopCount;  
  int  m_numPaths;
  bool  m_error;
};


/**
 * \ingroup aomdv
 * \brief The Routing table used by AOMDV protocol
 */
class RoutingTable
{
public:
  /// c-tor
  RoutingTable (Time t);
  ///\name Handle life time of invalid route
  //\{
  Time GetBadLinkLifetime () const { return m_badLinkLifetime; }
  void SetBadLinkLifetime (Time t) { m_badLinkLifetime = t; }
  //\}
  /**
   * Add routing table entry if it doesn't yet exist in routing table
   * \param r routing table entry
   * \return true in success
   */
  bool AddRoute (RoutingTableEntry & r);
  /**
   * Delete routing table entry with destination address dst, if it exists.
   * \param dst destination address
   * \return true on success
   */
  bool DeleteRoute (Ipv4Address dst);
  /**
   * Lookup routing table entry with destination address dst
   * \param dst destination address
   * \param rt entry with destination address dst, if exists
   * \return true on success
   */
  //Contribution
  void                 rt_dumptable();
  bool                 rt_has_active_route();

  bool LookupRoute (Ipv4Address dst, RoutingTableEntry & rt);
  /// Lookup route in VALID state
  bool LookupValidRoute (Ipv4Address dst, RoutingTableEntry & rt);
  /// Update routing table
  bool Update (RoutingTableEntry & rt);
  /// Set routing table entry flags
  bool SetEntryState (Ipv4Address dst, RouteFlags state);
  /// Lookup routing entries with next hop Address dst and not empty list of precursors.
  void GetListOfDestinationWithNextHop (Ipv4Address nextHop, std::map<Ipv4Address, uint32_t> & unreachable);
  /**
   *   Update routing entries with this destinations as follows:
   *  1. The destination sequence number of this routing entry, if it
   *     exists and is valid, is incremented.
   *  2. The entry is invalidated by marking the route entry as invalid
   *  3. The Lifetime field is updated to current time plus DELETE_PERIOD.
   */
  void InvalidateRoutesWithDst (std::map<Ipv4Address, uint32_t> const & unreachable);
  /// Delete all route from interface with address iface
  void DeleteAllRoutesFromInterface (Ipv4InterfaceAddress iface);
  /// Delete all entries from routing table
  void Clear () { m_ipv4AddressEntry.clear (); }
  /// Delete all outdated entries and invalidate valid entry if Lifetime is expired
  void Purge ();

  bool HasActiveRoutes();
  /** Mark entry as unidirectional (e.g. add this neighbor to "blacklist" for blacklistTimeout period)
   * \param neighbor - neighbor address link to which assumed to be unidirectional
   * \param blacklistTimeout - time for which the neighboring node is put into the blacklist
   * \return true on success
   */
  bool MarkLinkAsUnidirectional (Ipv4Address neighbor, Time blacklistTimeout);
  /// Print routing table
  void Print (Ptr<OutputStreamWrapper> stream) const;

private:
  std::map<Ipv4Address, RoutingTableEntry> m_ipv4AddressEntry;
  /// Deletion time for invalid routes
  Time m_badLinkLifetime;
  /// const version of Purge, for use by Print() method
  void Purge (std::map<Ipv4Address, RoutingTableEntry> &table) const;
};

inline bool operator == (const RoutingTableEntry::Path &a, const RoutingTableEntry::Path &b)
  {
    return (a.m_ipv4Route == b.m_ipv4Route && a.m_hopCount == b.m_hopCount && 
            a.m_expire == b.m_expire && a.m_lastHop == b.m_lastHop && a.m_iface == b.m_iface 
            && a.m_ts == b.m_ts && a.m_pathError == b.m_pathError);
  }
}
}

#endif /* AOMDV_RTABLE_H */
