/*
 * Copyright (c) 2019 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __MEM_RUBY_STRUCTURES_SNOOPFILTER_HH__
#define __MEM_RUBY_STRUCTURES_SNOOPFILTER_HH__

#include <unordered_map>

#include "base/compiler.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/protocol/AccessPermission.hh"

namespace gem5
{

namespace ruby
{

template<class ENTRY>
struct SnoopFilterCacheLineState
{
    SnoopFilterCacheLineState() { m_permission = AccessPermission_NUM; }
    AccessPermission m_permission;
    ENTRY m_entry;
};

template<class ENTRY>
inline std::ostream&
operator<<(std::ostream& out, const SnoopFilterCacheLineState<ENTRY>& obj)
{
    return out;
}

template<class ENTRY>
class SnoopFilter
{
  public:
    SnoopFilter();

    // tests to see if an address is present in the cache
    bool isTagPresent(Addr address) const;

    // Returns true if there is:
    //   a) a tag match on this address or there is
    //   b) an Invalid line in the same cache "way"
    bool cacheAvail(Addr address) const;

    // find an Invalid entry and sets the tag appropriate for the address
    void allocate(Addr address);

    void deallocate(Addr address);

    // Returns with the physical address of the conflicting cache line
    Addr cacheProbe(Addr newAddress) const;

    // looks an address up in the cache
    ENTRY* lookup(Addr address);
    const ENTRY* lookup(Addr address) const;

    // Get/Set permission of cache block
    AccessPermission getPermission(Addr address) const;
    void changePermission(Addr address, AccessPermission new_perm);

    // Print cache contents
    void print(std::ostream& out) const;

  private:
    // Private copy constructor and assignment operator
    SnoopFilter(const SnoopFilter& obj);
    SnoopFilter& operator=(const SnoopFilter& obj);

    // Data Members (m_prefix)
    std::unordered_map<Addr, SnoopFilterCacheLineState<ENTRY> > m_map;
};

template<class ENTRY>
inline std::ostream&
operator<<(std::ostream& out, const SnoopFilter<ENTRY>& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

template<class ENTRY>
inline
SnoopFilter<ENTRY>::SnoopFilter()
{
}

// tests to see if an address is present in the cache
template<class ENTRY>
inline bool
SnoopFilter<ENTRY>::isTagPresent(Addr address) const
{
    return m_map.count(makeLineAddress(address)) > 0;
}

template<class ENTRY>
inline bool
SnoopFilter<ENTRY>::cacheAvail(Addr address) const
{
    return true;
}

// find an Invalid or already allocated entry and sets the tag
// appropriate for the address
template<class ENTRY>
inline void
SnoopFilter<ENTRY>::allocate(Addr address)
{
    SnoopFilterCacheLineState<ENTRY> line_state;
    line_state.m_permission = AccessPermission_Invalid;
    line_state.m_entry = ENTRY();
    m_map[makeLineAddress(address)] = line_state;
}

// deallocate entry
template<class ENTRY>
inline void
SnoopFilter<ENTRY>::deallocate(Addr address)
{
    [[maybe_unused]] auto num_erased = m_map.erase(makeLineAddress(address));
    assert(num_erased == 1);
}

// Returns with the physical address of the conflicting cache line
template<class ENTRY>
inline Addr
SnoopFilter<ENTRY>::cacheProbe(Addr newAddress) const
{
    panic("cacheProbe called in perfect cache");
    return newAddress;
}

// looks an address up in the cache
template<class ENTRY>
inline ENTRY*
SnoopFilter<ENTRY>::lookup(Addr address)
{
    return &m_map[makeLineAddress(address)].m_entry;
}

// looks an address up in the cache
template<class ENTRY>
inline const ENTRY*
SnoopFilter<ENTRY>::lookup(Addr address) const
{
    return &m_map[makeLineAddress(address)].m_entry;
}

template<class ENTRY>
inline AccessPermission
SnoopFilter<ENTRY>::getPermission(Addr address) const
{
    return m_map[makeLineAddress(address)].m_permission;
}

template<class ENTRY>
inline void
SnoopFilter<ENTRY>::changePermission(Addr address,
                                            AccessPermission new_perm)
{
    Addr line_address = makeLineAddress(address);
    SnoopFilterCacheLineState<ENTRY>& line_state = m_map[line_address];
    line_state.m_permission = new_perm;
}

template<class ENTRY>
inline void
SnoopFilter<ENTRY>::print(std::ostream& out) const
{
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_STRUCTURES_SNOOPFILTER_HH__
