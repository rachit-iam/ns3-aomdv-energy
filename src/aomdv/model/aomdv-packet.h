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
#ifndef AOMDVPACKET_H
#define AOMDVPACKET_H

#include <iostream>
#include "ns3/header.h"
#include "ns3/enum.h"
#include "ns3/ipv4-address.h"
#include <map>
#include "ns3/nstime.h"

#define INFINITY3 9999999

namespace ns3 {
namespace aomdv {

enum MessageType
{
  AOMDVTYPE_RREQ  = 1,   //!< AOMDVTYPE_RREQ
  AOMDVTYPE_RREP  = 2,   //!< AOMDVTYPE_RREP
  AOMDVTYPE_RERR  = 3,   //!< AOMDVTYPE_RERR
  AOMDVTYPE_RREP_ACK = 4 //!< AOMDVTYPE_RREP_ACK
};

/**
* \ingroup aomdv
* \brief AOMDV types
*/
class TypeHeader : public Header
{
public:
  /// c-tor
  TypeHeader (MessageType t = AOMDVTYPE_RREQ);

  ///\name Header serialization/deserialization
  //\{
  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const;
  uint32_t GetSerializedSize () const;
  void Serialize (Buffer::Iterator start) const;
  uint32_t Deserialize (Buffer::Iterator start);
  void Print (std::ostream &os) const;
  //\}

  /// Return type
  MessageType Get () const { return m_type; }
  /// Check that type if valid
  bool IsValid () const { return m_valid; }
  bool operator== (TypeHeader const & o) const;
private:
  MessageType m_type;
  bool m_valid;
};

std::ostream & operator<< (std::ostream & os, TypeHeader const & h);

/**
* \ingroup aomdv
* \brief   Route Request (RREQ) Message Format
  \verbatim
  0                   1                   2                   3
  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |     Type      |J|R|G|D|U|   Reserved          |   Hop Count   |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                            RREQ ID                            |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                    Destination IP Address                     |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                  Destination Sequence Number                  |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                    Originator IP Address                      |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                  Originator Sequence Number                   |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  \endverbatim
*/
class RreqHeader : public Header 
{
public:
  /// c-tor
  RreqHeader (uint8_t flags = 0, uint8_t reserved = 0, uint8_t hopCount = 0,
              uint32_t requestID = 0, Ipv4Address dst = Ipv4Address (),
              uint32_t dstSeqNo = 0, Ipv4Address origin = Ipv4Address (),
              uint32_t originSeqNo = 0, Ipv4Address firstHop = Ipv4Address(), uint32_t MRE = INFINITY3,
              uint32_t x = 0, uint32_t y = 0, uint32_t squaredDistance = 0, uint64_t timeStamp = 0);

  ///\name Header serialization/deserialization
  //\{
  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const;
  uint32_t GetSerializedSize () const;
  void Serialize (Buffer::Iterator start) const;
  uint32_t Deserialize (Buffer::Iterator start);
  void Print (std::ostream &os) const;
  //\}

  ///\name Fields
  //\{
  void SetHopCount (uint8_t count) { m_hopCount = count; }
  uint8_t GetHopCount () const { return m_hopCount; }
  void SetId (uint32_t id) { m_requestID = id; }
  uint32_t GetId () const { return m_requestID; }
  void SetDst (Ipv4Address a) { m_dst = a; }
  Ipv4Address GetDst () const { return m_dst; }
  void SetDstSeqno (uint32_t s) { m_dstSeqNo = s; }
  uint32_t GetDstSeqno () const { return m_dstSeqNo; }
  void SetOrigin (Ipv4Address a) { m_origin = a; }
  Ipv4Address GetOrigin () const { return m_origin; }
  void SetOriginSeqno (uint32_t s) { m_originSeqNo = s; }
  uint32_t GetOriginSeqno () const { return m_originSeqNo; }
  void SetFirstHop (Ipv4Address a) { m_firstHop = a; }
  Ipv4Address GetFirstHop () const { return m_firstHop; }
  void SetMRE (uint32_t MRE) { m_MRE = MRE; }
  uint32_t GetMRE () const { return m_MRE; }
  void SetX (uint32_t x) { m_x = x; }
  uint32_t GetX () { return m_x; }
  void SetY (uint32_t y) { m_y = y; }
  uint32_t GetY () { return m_y; }
  void SetSquaredDistance (uint32_t squaredDistance) { m_squaredDistance = squaredDistance; }
  uint32_t GetSquaredDistance () { return m_squaredDistance; }
  void SetTimeStamp (uint64_t timeStamp) { m_timeStamp = timeStamp; }
  uint64_t GetTimeStamp () { return m_timeStamp; }
  //\}

  ///\name Flags
  //\{
  void SetGratiousRrep (bool f);
  bool GetGratiousRrep () const;
  void SetDestinationOnly (bool f);
  bool GetDestinationOnly () const;
  void SetUnknownSeqno (bool f);
  bool GetUnknownSeqno () const;
  void SetRReq (bool f);
  bool GetRReq () const;
  //\}

  bool operator== (RreqHeader const & o) const;
private:
  uint8_t        m_flags;          ///< |J|R|G|D|U| bit flags, see RFC
  uint8_t        m_reserved;       ///< Not used
  uint8_t        m_hopCount;       ///< Hop Count
  uint32_t       m_requestID;      ///< RREQ ID
  Ipv4Address    m_dst;            ///< Destination IP Address
  uint32_t       m_dstSeqNo;       ///< Destination Sequence Number
  Ipv4Address    m_origin;         ///< Originator IP Address
  uint32_t       m_originSeqNo;    ///< Source Sequence Number
  Ipv4Address    m_firstHop;
  uint32_t       m_MRE;
  uint32_t       m_x;
  uint32_t       m_y;
  uint32_t       m_squaredDistance;
  uint64_t       m_timeStamp;
};

std::ostream & operator<< (std::ostream & os, RreqHeader const &);

/**
* \ingroup aomdv
* \brief Route Reply (RREP) Message Format
  \verbatim
  0                   1                   2                   3
  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |     Type      |R|A|    Reserved     |Prefix Sz|   Hop Count   |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                     Destination IP address                    |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                  Destination Sequence Number                  |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                    Originator IP address                      |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                           Lifetime                            |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  \endverbatim
*/
class RrepHeader : public Header
{
public:
  /// c-tor
  RrepHeader (uint8_t prefixSize = 0, uint8_t hopCount = 0, Ipv4Address dst =
                Ipv4Address (), uint32_t dstSeqNo = 0, Ipv4Address origin =
                Ipv4Address (), uint32_t requestID = 0,  Ipv4Address firstHop =
                Ipv4Address (), Time lifetime = MilliSeconds (0), uint32_t MRE = INFINITY3,
                uint32_t x = 0, uint32_t y = 0, uint32_t squaredDistance = 0, uint64_t timeStamp = 0);

  ///\name Header serialization/deserialization
  //\{
  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const;
  uint32_t GetSerializedSize () const;
  void Serialize (Buffer::Iterator start) const;
  uint32_t Deserialize (Buffer::Iterator start);
  void Print (std::ostream &os) const;
  //\}

  ///\name Fields
  //\{
  void SetHopCount (uint8_t count) { m_hopCount = count; }
  uint8_t GetHopCount () const { return m_hopCount; }
  void SetDst (Ipv4Address a) { m_dst = a; }
  Ipv4Address GetDst () const { return m_dst; }
  void SetDstSeqno (uint32_t s) { m_dstSeqNo = s; }
  uint32_t GetDstSeqno () const { return m_dstSeqNo; }
  void SetOrigin (Ipv4Address a) { m_origin = a; }
  Ipv4Address GetOrigin () const { return m_origin; }
  void SetRequestID (uint32_t s) { m_requestID = s; }
  uint32_t GetRequestID () const { return m_requestID; }
  void SetFirstHop (Ipv4Address a) { m_firstHop = a; }
  Ipv4Address GetFirstHop () const { return m_firstHop; }
  void SetLifeTime (Time t);
  Time GetLifeTime () const;
  void SetMRE (uint32_t MRE) { m_MRE = MRE; }
  uint32_t GetMRE () const { return m_MRE; }
  void SetX (uint32_t x) { m_x = x; }
  uint32_t GetX () { return m_x; }
  void SetY (uint32_t y) { m_y = y; }
  uint32_t GetY () { return m_y; }
  void SetSquaredDistance (uint32_t squaredDistance) { m_squaredDistance = squaredDistance; }
  uint32_t GetSquaredDistance () { return m_squaredDistance; }
  void SetTimeStamp (uint64_t timeStamp) { m_timeStamp = timeStamp; }
  uint64_t GetTimeStamp () { return m_timeStamp; }
  //\}

  ///\name Flags
  //\{
  void SetAckRequired (bool f);
  bool GetAckRequired () const;
  void SetPrefixSize (uint8_t sz);
  uint8_t GetPrefixSize () const;
  //\}

  /// Configure RREP to be a Hello message
  void SetHello (Ipv4Address src, uint32_t srcSeqNo, Time lifetime);

  bool operator== (RrepHeader const & o) const;
private:
  uint8_t       m_flags;                  ///< A - acknowledgment required flag
  uint8_t       m_prefixSize;         ///< Prefix Size
  uint8_t       m_hopCount;         ///< Hop Count
  Ipv4Address   m_dst;              ///< Destination IP Address
  uint32_t      m_dstSeqNo;         ///< Destination Sequence Number
  Ipv4Address   m_origin;           ///< Source IP Address
  uint32_t      m_requestID;
  Ipv4Address   m_firstHop;
  uint32_t      m_lifeTime;         ///< Lifetime (in milliseconds)
  uint32_t      m_MRE;
  uint32_t      m_x;
  uint32_t      m_y;
  uint32_t      m_squaredDistance;
  uint64_t      m_timeStamp;
};

std::ostream & operator<< (std::ostream & os, RrepHeader const &);

/**
* \ingroup aomdv
* \brief Route Reply Acknowledgment (RREP-ACK) Message Format
  \verbatim
  0                   1
  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |     Type      |   Reserved    |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  \endverbatim
*/
class RrepAckHeader : public Header
{
public:
  /// c-tor
  RrepAckHeader ();

  ///\name Header serialization/deserialization
  //\{
  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const;
  uint32_t GetSerializedSize () const;
  void Serialize (Buffer::Iterator start) const;
  uint32_t Deserialize (Buffer::Iterator start);
  void Print (std::ostream &os) const;
  //\}

  bool operator== (RrepAckHeader const & o) const;
private:
  uint8_t       m_reserved;
};
std::ostream & operator<< (std::ostream & os, RrepAckHeader const &);


/**
* \ingroup aomdv
* \brief Route Error (RERR) Message Format
  \verbatim
  0                   1                   2                   3
  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |     Type      |N|          Reserved           |   DestCount   |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |            Unreachable Destination IP Address (1)             |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |         Unreachable Destination Sequence Number (1)           |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-|
  |  Additional Unreachable Destination IP Addresses (if needed)  |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |Additional Unreachable Destination Sequence Numbers (if needed)|
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  \endverbatim
*/
class RerrHeader : public Header
{
public:
  /// c-tor
  RerrHeader ();

  // Header serialization/deserialization
  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const;
  uint32_t GetSerializedSize () const;
  void Serialize (Buffer::Iterator i) const;
  uint32_t Deserialize (Buffer::Iterator start);
  void Print (std::ostream &os) const;

  // No delete flag
  void SetNoDelete (bool f);
  bool GetNoDelete () const;

  /**
   * Add unreachable node address and its sequence number in RERR header
   *\return false if we already added maximum possible number of unreachable destinations
   */
  bool AddUnDestination (Ipv4Address dst, uint32_t seqNo);
  /** Delete pair (address + sequence number) from REER header, if the number of unreachable destinations > 0
   * \return true on success
   */
  bool RemoveUnDestination (std::pair<Ipv4Address, uint32_t> & un);
  /// Clear header
  void Clear ();
  /// Return number of unreachable destinations in RERR message
  uint8_t GetDestCount () const { return (uint8_t)m_unreachableDstSeqNo.size (); }
  bool operator== (RerrHeader const & o) const;
private:
  uint8_t m_flag;            ///< No delete flag
  uint8_t m_reserved;        ///< Not used

  /// List of Unreachable destination: IP addresses and sequence numbers
  std::map<Ipv4Address, uint32_t> m_unreachableDstSeqNo;
};

std::ostream & operator<< (std::ostream & os, RerrHeader const &);
}
}
#endif /* AOMDVPACKET_H */
