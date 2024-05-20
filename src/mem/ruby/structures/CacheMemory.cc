/*
 * Copyright (c) 2020-2021 ARM Limited
 * All rights reserved
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
 * Copyright (c) 1999-2012 Mark D. Hill and David A. Wood
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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

#include "mem/ruby/structures/CacheMemory.hh"

#include "base/compiler.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "debug/HtmMem.hh"
#include "debug/RubyCache.hh"
#include "debug/RubyCacheTrace.hh"
#include "debug/RubyResourceStalls.hh"
#include "debug/RubyStats.hh"
#include "mem/cache/replacement_policies/weighted_lru_rp.hh"
#include "mem/ruby/protocol/AccessPermission.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "debug/ProtocolTrace.hh"

namespace gem5
{

namespace ruby
{

std::ostream&
operator<<(std::ostream& out, const CacheMemory& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

CacheMemory::CacheMemory(const Params &p)
    : SimObject(p),
    dataArray(p.dataArrayBanks, p.dataAccessLatency,
              p.start_index_bit, p.ruby_system),
    tagArray(p.tagArrayBanks, p.tagAccessLatency,
             p.start_index_bit, p.ruby_system),
    cacheMemoryStats(this)
{
    m_cache_size = p.size;
    m_cache_assoc = p.assoc;
    m_replacementPolicy_ptr = p.replacement_policy;
    m_start_index_bit = p.start_index_bit;
    m_is_instruction_only_cache = p.is_icache;
    m_resource_stalls = p.resourceStalls;
    m_block_size = p.block_size;  // may be 0 at this point. Updated in init()
    m_use_occupancy = dynamic_cast<replacement_policy::WeightedLRU*>(
                                    m_replacementPolicy_ptr) ? true : false;
}

void
CacheMemory::init()
{
    if (m_block_size == 0) {
        m_block_size = RubySystem::getBlockSizeBytes();
    }
    m_cache_num_sets = (m_cache_size / m_cache_assoc) / m_block_size;
    assert(m_cache_num_sets > 1);
    m_cache_num_set_bits = floorLog2(m_cache_num_sets);
    assert(m_cache_num_set_bits > 0);

    m_cache.resize(m_cache_num_sets,
                    std::vector<AbstractCacheEntry*>(m_cache_assoc, nullptr));
    replacement_data.resize(m_cache_num_sets,
                               std::vector<ReplData>(m_cache_assoc, nullptr));
    // instantiate all the replacement_data here
    for (int i = 0; i < m_cache_num_sets; i++) {
        for ( int j = 0; j < m_cache_assoc; j++) {
            replacement_data[i][j] =
                                m_replacementPolicy_ptr->instantiateEntry();
        }
    }
}

CacheMemory::~CacheMemory()
{
    if (m_replacementPolicy_ptr)
        delete m_replacementPolicy_ptr;
    for (int i = 0; i < m_cache_num_sets; i++) {
        for (int j = 0; j < m_cache_assoc; j++) {
            delete m_cache[i][j];
        }
    }
}

// convert a Address to its location in the cache
int64_t
CacheMemory::addressToCacheSet(Addr address) const
{
    assert(address == makeLineAddress(address));
    return bitSelect(address, m_start_index_bit,
                     m_start_index_bit + m_cache_num_set_bits - 1);
}

// Given a cache index: returns the index of the tag in a set.
// returns -1 if the tag is not found.
int
CacheMemory::findTagInSet(int64_t cacheSet, Addr tag) const
{
    assert(tag == makeLineAddress(tag));
    // search the set for the tags
    auto it = m_tag_index.find(tag);
    if (it != m_tag_index.end())
        if (m_cache[cacheSet][it->second]->m_Permission !=
            AccessPermission_NotPresent)
            return it->second;
    return -1; // Not found
}

// Given a cache index: returns the index of the tag in a set.
// returns -1 if the tag is not found.
int
CacheMemory::findTagInSetIgnorePermissions(int64_t cacheSet,
                                           Addr tag) const
{
    assert(tag == makeLineAddress(tag));
    // search the set for the tags
    auto it = m_tag_index.find(tag);
    if (it != m_tag_index.end())
        return it->second;
    return -1; // Not found
}

// Given an unique cache block identifier (idx): return the valid address
// stored by the cache block.  If the block is invalid/notpresent, the
// function returns the 0 address
Addr
CacheMemory::getAddressAtIdx(int idx) const
{
    Addr tmp(0);

    int set = idx / m_cache_assoc;
    assert(set < m_cache_num_sets);

    int way = idx - set * m_cache_assoc;
    assert (way < m_cache_assoc);

    AbstractCacheEntry* entry = m_cache[set][way];
    if (entry == NULL ||
        entry->m_Permission == AccessPermission_Invalid ||
        entry->m_Permission == AccessPermission_NotPresent) {
        return tmp;
    }
    return entry->m_Address;
}

bool
CacheMemory::tryCacheAccess(Addr address, RubyRequestType type,
                            DataBlock*& data_ptr)
{
    DPRINTF(RubyCache, "address: %#x\n", address);
    AbstractCacheEntry* entry = lookup(address);
    if (entry != nullptr) {
        // Do we even have a tag match?
        m_replacementPolicy_ptr->touch(entry->replacementData);
        entry->setLastAccess(curTick());
        data_ptr = &(entry->getDataBlk());

        if (entry->m_Permission == AccessPermission_Read_Write) {
            return true;
        }
        if ((entry->m_Permission == AccessPermission_Read_Only) &&
            (type == RubyRequestType_LD || type == RubyRequestType_IFETCH)) {
            return true;
        }
        // The line must not be accessible
    }
    data_ptr = NULL;
    return false;
}

bool
CacheMemory::testCacheAccess(Addr address, RubyRequestType type,
                             DataBlock*& data_ptr)
{
    DPRINTF(RubyCache, "address: %#x\n", address);
    AbstractCacheEntry* entry = lookup(address);
    if (entry != nullptr) {
        // Do we even have a tag match?
        m_replacementPolicy_ptr->touch(entry->replacementData);
        entry->setLastAccess(curTick());
        data_ptr = &(entry->getDataBlk());

        return entry->m_Permission != AccessPermission_NotPresent;
    }

    data_ptr = NULL;
    return false;
}

// tests to see if an address is present in the cache
bool
CacheMemory::isTagPresent(Addr address) const
{
    const AbstractCacheEntry* const entry = lookup(address);
    if (entry == nullptr) {
        // We didn't find the tag
        DPRINTF(RubyCache, "No tag match for address: %#x\n", address);
        return false;
    }
    DPRINTF(RubyCache, "address: %#x found\n", address);
    return true;
}

// Returns true if there is:
//   a) a tag match on this address or there is
//   b) an unused line in the same cache "way"
bool
CacheMemory::cacheAvail(Addr address) const
{
    assert(address == makeLineAddress(address));

    int64_t cacheSet = addressToCacheSet(address);

    for (int i = 0; i < m_cache_assoc; i++) {
        AbstractCacheEntry* entry = m_cache[cacheSet][i];
        if (entry != NULL) {
            if (entry->m_Address == address ||
                entry->m_Permission == AccessPermission_NotPresent) {
                // Already in the cache or we found an empty entry
                return true;
            }
        } else {
            return true;
        }
    }
    return false;
}

bool
CacheMemory::cacheAvailForECI(Addr address) const
{
    assert(address == makeLineAddress(address));

    int64_t cacheSet = addressToCacheSet(address);

    for (int i = 0; i < m_cache_assoc; i++) {
        AbstractCacheEntry* entry = m_cache[cacheSet][i];
        if (entry != NULL) {
            if ( entry->m_Permission == AccessPermission_NotPresent) {
                // Already in the cache or we found an empty entry
                return true;
            }
        } else {
            return true;
        }
    }
    return false;
}

AbstractCacheEntry*
CacheMemory::allocate(Addr address, AbstractCacheEntry *entry)
{
    assert(address == makeLineAddress(address));
    assert(!isTagPresent(address));
    assert(cacheAvail(address));
    DPRINTF(RubyCache, "address: %#x\n", address);

    // Find the first open slot
    int64_t cacheSet = addressToCacheSet(address);
    std::vector<AbstractCacheEntry*> &set = m_cache[cacheSet];
    for (int i = 0; i < m_cache_assoc; i++) {
        if (!set[i] || set[i]->m_Permission == AccessPermission_NotPresent) {
            if (set[i] && (set[i] != entry)) {
                warn_once("This protocol contains a cache entry handling bug: "
                    "Entries in the cache should never be NotPresent! If\n"
                    "this entry (%#x) is not tracked elsewhere, it will memory "
                    "leak here. Fix your protocol to eliminate these!",
                    address);
            }
            set[i] = entry;  // Init entry
            set[i]->m_Address = address;
            set[i]->m_Permission = AccessPermission_Invalid;
            DPRINTF(RubyCache, "Allocate clearing lock for addr: %x\n",
                    address);
            set[i]->m_locked = -1;
            m_tag_index[address] = i;
            set[i]->setPosition(cacheSet, i);
            set[i]->replacementData = replacement_data[cacheSet][i];
            set[i]->setLastAccess(curTick());

            // Call reset function here to set initial value for different
            // replacement policies.
            m_replacementPolicy_ptr->reset(entry->replacementData);

            return entry;
        }
    }
    DPRINTF(ProtocolTrace, "address: %#x\n", address);
    panic("Allocate didn't find an available entry");
}

void
CacheMemory::deallocate(Addr address)
{
    DPRINTF(RubyCache, "address: %#x\n", address);
    AbstractCacheEntry* entry = lookup(address);
    assert(entry != nullptr);
    m_replacementPolicy_ptr->invalidate(entry->replacementData);
    uint32_t cache_set = entry->getSet();
    uint32_t way = entry->getWay();
    delete entry;
    m_cache[cache_set][way] = NULL;
    m_tag_index.erase(address);
}

// Returns with the physical address of the conflicting cache line
Addr
CacheMemory::cacheProbe(Addr address) const
{
    assert(address == makeLineAddress(address));
    assert(!cacheAvail(address));

    int64_t cacheSet = addressToCacheSet(address);
    std::vector<ReplaceableEntry*> candidates;
    for (int i = 0; i < m_cache_assoc; i++) {
        candidates.push_back(static_cast<ReplaceableEntry*>(
                                                       m_cache[cacheSet][i]));
    }
    return m_cache[cacheSet][m_replacementPolicy_ptr->
                        getVictim(candidates)->getWay()]->m_Address;
}

Addr
CacheMemory::cacheProbeForECI(Addr address) const
{
    assert(address == makeLineAddress(address));
    assert(!cacheAvailForECI(address));

    int64_t cacheSet = addressToCacheSet(address);
    std::vector<ReplaceableEntry*> candidates;
    for (int i = 0; i < m_cache_assoc; i++) {
        candidates.push_back(static_cast<ReplaceableEntry*>(
                                                       m_cache[cacheSet][i]));
    }
    return m_cache[cacheSet][m_replacementPolicy_ptr->
                        getVictim(candidates)->getWay()]->m_Address;
}

Addr
CacheMemory::cacheProbeForCRAR(Addr address, int cnt) const
{
    assert(address == makeLineAddress(address));
    assert(!cacheAvail(address));

    int64_t cacheSet = addressToCacheSet(address);
    std::vector<ReplaceableEntry*> candidates;
    for (int i = 0; i < m_cache_assoc; i++) {
        candidates.push_back(static_cast<ReplaceableEntry*>(
                                                       m_cache[cacheSet][i]));
    }
    return m_cache[cacheSet][m_replacementPolicy_ptr->
                        getVictimCRAR(candidates, cnt)->getWay()]->m_Address;
}

// looks an address up in the cache
int
CacheMemory::GetDcache_Mstate_LruPosition(Addr address) 
{
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    int Lru_Position = 17;
    int loc = findTagInSet(cacheSet, address);  //way
    DPRINTF(ProtocolTrace, "loc: %d\n", loc);
    if (loc == -1) {return NULL;}
    else {
        //Addr temp_cache_entry = m_cache[cacheSet][loc]->m_Address;
        std::vector<ReplaceableEntry*> candidates;
        for (int i = 0; i < m_cache_assoc; i++) {
            if (m_cache[cacheSet][i]){
                candidates.push_back(static_cast<ReplaceableEntry*>( m_cache[cacheSet][i] ));
            }
        }
        DPRINTF(ProtocolTrace, "loc: candidates\n");
        for (int i = 0; i < m_cache_assoc; i++){
            Addr address_temp = m_cache[cacheSet][m_replacementPolicy_ptr->getVictimCRAR(candidates, i)->getWay()]->m_Address;
            Lru_Position = i;
            DPRINTF(ProtocolTrace, "Lru_Position: %d\n", Lru_Position);
            if (address_temp == address) break;
        }
    }
    profileDcache_Mstate_LruPosition(Lru_Position);
    return Lru_Position;
}

int
CacheMemory::GetDcache_Sstate_LruPosition(Addr address) 
{
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    int Lru_Position = 17;
    int loc = findTagInSet(cacheSet, address);  //way
    DPRINTF(ProtocolTrace, "loc: %d\n", loc);
    if (loc == -1) {return NULL;}
    else {
        //Addr temp_cache_entry = m_cache[cacheSet][loc]->m_Address;
        std::vector<ReplaceableEntry*> candidates;
        for (int i = 0; i < m_cache_assoc; i++) {
            if (m_cache[cacheSet][i]){
                candidates.push_back(static_cast<ReplaceableEntry*>( m_cache[cacheSet][i] ));
            }
        }
        DPRINTF(ProtocolTrace, "loc: candidates\n");
        for (int i = 0; i < m_cache_assoc; i++){
            Addr address_temp = m_cache[cacheSet][m_replacementPolicy_ptr->getVictimCRAR(candidates, i)->getWay()]->m_Address;
            Lru_Position = i;
            DPRINTF(ProtocolTrace, "Lru_Position: %d\n", Lru_Position);
            if (address_temp == address) break;
        }
    }
    profileDcache_Sstate_LruPosition(Lru_Position);
    return Lru_Position;
}

int
CacheMemory::GetIcache_Mstate_LruPosition(Addr address) 
{
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    int Lru_Position = 17;
    int loc = findTagInSet(cacheSet, address);  //way
    DPRINTF(ProtocolTrace, "loc: %d\n", loc);
    if (loc == -1) {return NULL;}
    else {
        //Addr temp_cache_entry = m_cache[cacheSet][loc]->m_Address;
        std::vector<ReplaceableEntry*> candidates;
        for (int i = 0; i < m_cache_assoc; i++) {
            if (m_cache[cacheSet][i]){
                candidates.push_back(static_cast<ReplaceableEntry*>( m_cache[cacheSet][i] ));
            }
        }
        DPRINTF(ProtocolTrace, "loc: candidates\n");
        for (int i = 0; i < m_cache_assoc; i++){
            Addr address_temp = m_cache[cacheSet][m_replacementPolicy_ptr->getVictimCRAR(candidates, i)->getWay()]->m_Address;
            Lru_Position = i;
            DPRINTF(ProtocolTrace, "Lru_Position: %d\n", Lru_Position);
            if (address_temp == address) break;
        }
    }
    profileIcache_Mstate_LruPosition(Lru_Position);
    return Lru_Position;
}

int
CacheMemory::GetIcache_Sstate_LruPosition(Addr address) 
{
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    int Lru_Position = 17;
    int loc = findTagInSet(cacheSet, address);  //way
    DPRINTF(ProtocolTrace, "loc: %d\n", loc);
    if (loc == -1) {return NULL;}
    else {
        //Addr temp_cache_entry = m_cache[cacheSet][loc]->m_Address;
        std::vector<ReplaceableEntry*> candidates;
        for (int i = 0; i < m_cache_assoc; i++) {
            if (m_cache[cacheSet][i]){
                candidates.push_back(static_cast<ReplaceableEntry*>( m_cache[cacheSet][i] ));
            }
        }
        DPRINTF(ProtocolTrace, "loc: candidates\n");
        for (int i = 0; i < m_cache_assoc; i++){
            Addr address_temp = m_cache[cacheSet][m_replacementPolicy_ptr->getVictimCRAR(candidates, i)->getWay()]->m_Address;
            Lru_Position = i;
            DPRINTF(ProtocolTrace, "Lru_Position: %d\n", Lru_Position);
            if (address_temp == address) break;
        }
    }
    profileIcache_Sstate_LruPosition(Lru_Position);
    return Lru_Position;
}

// looks an address up in the cache
AbstractCacheEntry*
CacheMemory::lookup(Addr address)
{
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    int loc = findTagInSet(cacheSet, address);
    if (loc == -1) return NULL;
    return m_cache[cacheSet][loc];
}

// looks an address up in the cache
const AbstractCacheEntry*
CacheMemory::lookup(Addr address) const
{
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    int loc = findTagInSet(cacheSet, address);
    if (loc == -1) return NULL;
    return m_cache[cacheSet][loc];
}

// Sets the most recently used bit for a cache block
void
CacheMemory::setMRU(Addr address)
{
    AbstractCacheEntry* entry = lookup(makeLineAddress(address));
    if (entry != nullptr) {
        m_replacementPolicy_ptr->touch(entry->replacementData);
        entry->setLastAccess(curTick());
    }
}

void
CacheMemory::setMRU(AbstractCacheEntry *entry)
{
    assert(entry != nullptr);
    m_replacementPolicy_ptr->touch(entry->replacementData);
    entry->setLastAccess(curTick());
}

void
CacheMemory::setMRU(Addr address, int occupancy)
{
    AbstractCacheEntry* entry = lookup(makeLineAddress(address));
    if (entry != nullptr) {
        // m_use_occupancy can decide whether we are using WeightedLRU
        // replacement policy. Depending on different replacement policies,
        // use different touch() function.
        if (m_use_occupancy) {
            static_cast<replacement_policy::WeightedLRU*>(
                m_replacementPolicy_ptr)->touch(
                entry->replacementData, occupancy);
        } else {
            m_replacementPolicy_ptr->touch(entry->replacementData);
        }
        entry->setLastAccess(curTick());
    }
}

int
CacheMemory::getReplacementWeight(int64_t set, int64_t loc)
{
    assert(set < m_cache_num_sets);
    assert(loc < m_cache_assoc);
    int ret = 0;
    if (m_cache[set][loc] != NULL) {
        ret = m_cache[set][loc]->getNumValidBlocks();
        assert(ret >= 0);
    }

    return ret;
}

void
CacheMemory::recordCacheContents(int cntrl, CacheRecorder* tr) const
{
    uint64_t warmedUpBlocks = 0;
    [[maybe_unused]] uint64_t totalBlocks = (uint64_t)m_cache_num_sets *
                                         (uint64_t)m_cache_assoc;

    for (int i = 0; i < m_cache_num_sets; i++) {
        for (int j = 0; j < m_cache_assoc; j++) {
            if (m_cache[i][j] != NULL) {
                AccessPermission perm = m_cache[i][j]->m_Permission;
                RubyRequestType request_type = RubyRequestType_NULL;
                if (perm == AccessPermission_Read_Only) {
                    if (m_is_instruction_only_cache) {
                        request_type = RubyRequestType_IFETCH;
                    } else {
                        request_type = RubyRequestType_LD;
                    }
                } else if (perm == AccessPermission_Read_Write) {
                    request_type = RubyRequestType_ST;
                }

                if (request_type != RubyRequestType_NULL) {
                    Tick lastAccessTick;
                    lastAccessTick = m_cache[i][j]->getLastAccess();
                    tr->addRecord(cntrl, m_cache[i][j]->m_Address,
                                  0, request_type, lastAccessTick,
                                  m_cache[i][j]->getDataBlk());
                    warmedUpBlocks++;
                }
            }
        }
    }

    DPRINTF(RubyCacheTrace, "%s: %lli blocks of %lli total blocks"
            "recorded %.2f%% \n", name().c_str(), warmedUpBlocks,
            totalBlocks, (float(warmedUpBlocks) / float(totalBlocks)) * 100.0);
}

void
CacheMemory::print(std::ostream& out) const
{
    out << "Cache dump: " << name() << std::endl;
    for (int i = 0; i < m_cache_num_sets; i++) {
        for (int j = 0; j < m_cache_assoc; j++) {
            if (m_cache[i][j] != NULL) {
                out << "  Index: " << i
                    << " way: " << j
                    << " entry: " << *m_cache[i][j] << std::endl;
            } else {
                out << "  Index: " << i
                    << " way: " << j
                    << " entry: NULL" << std::endl;
            }
        }
    }
}

void
CacheMemory::printData(std::ostream& out) const
{
    out << "printData() not supported" << std::endl;
}

void
CacheMemory::setLocked(Addr address, int context)
{
    DPRINTF(RubyCache, "Setting Lock for addr: %#x to %d\n", address, context);
    AbstractCacheEntry* entry = lookup(address);
    assert(entry != nullptr);
    entry->setLocked(context);
}

void
CacheMemory::clearLocked(Addr address)
{
    DPRINTF(RubyCache, "Clear Lock for addr: %#x\n", address);
    AbstractCacheEntry* entry = lookup(address);
    assert(entry != nullptr);
    entry->clearLocked();
}

void
CacheMemory::clearLockedAll(int context)
{
    // iterate through every set and way to get a cache line
    for (auto i = m_cache.begin(); i != m_cache.end(); ++i) {
        std::vector<AbstractCacheEntry*> set = *i;
        for (auto j = set.begin(); j != set.end(); ++j) {
            AbstractCacheEntry *line = *j;
            if (line && line->isLocked(context)) {
                DPRINTF(RubyCache, "Clear Lock for addr: %#x\n",
                    line->m_Address);
                line->clearLocked();
            }
        }
    }
}

bool
CacheMemory::isLocked(Addr address, int context)
{
    AbstractCacheEntry* entry = lookup(address);
    assert(entry != nullptr);
    DPRINTF(RubyCache, "Testing Lock for addr: %#llx cur %d con %d\n",
            address, entry->m_locked, context);
    return entry->isLocked(context);
}

CacheMemory::
CacheMemoryStats::CacheMemoryStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(numDataArrayReads, "Number of data array reads"),
      ADD_STAT(numDataArrayWrites, "Number of data array writes"),
      ADD_STAT(numTagArrayReads, "Number of tag array reads"),
      ADD_STAT(numTagArrayWrites, "Number of tag array writes"),
      ADD_STAT(numTagArrayStalls, "Number of stalls caused by tag array"),
      ADD_STAT(numDataArrayStalls, "Number of stalls caused by data array"),
      ADD_STAT(htmTransCommitReadSet, "Read set size of a committed "
                                      "transaction"),
      ADD_STAT(htmTransCommitWriteSet, "Write set size of a committed "
                                       "transaction"),
      ADD_STAT(htmTransAbortReadSet, "Read set size of a aborted transaction"),
      ADD_STAT(htmTransAbortWriteSet, "Write set size of a aborted "
                                      "transaction"),
    //-----------------------------------------
      ADD_STAT(Mstate_DcachePosition0, "Number of evict lru position 0"),
      ADD_STAT(Mstate_DcachePosition1, "Number of evict lru position 1"),
      ADD_STAT(Mstate_DcachePosition2, "Number of evict lru position 2"),
      ADD_STAT(Mstate_DcachePosition3, "Number of evict lru position 3"),  
      ADD_STAT(Mstate_DcachePosition4, "Number of evict lru position 4"),
      ADD_STAT(Mstate_DcachePosition5, "Number of evict lru position 5"),
      ADD_STAT(Mstate_DcachePosition6, "Number of evict lru position 6"),
      ADD_STAT(Mstate_DcachePosition7, "Number of evict lru position 7"),    
      ADD_STAT(Mstate_DcachePosition8, "Number of evict lru position 8"),
      ADD_STAT(Mstate_DcachePosition9, "Number of evict lru position 9"),
      ADD_STAT(Mstate_DcachePosition10, "Number of evict lru position 10"),
      ADD_STAT(Mstate_DcachePosition11, "Number of evict lru position 11"),  
      ADD_STAT(Mstate_DcachePosition12, "Number of evict lru position 12"),
      ADD_STAT(Mstate_DcachePosition13, "Number of evict lru position 13"),
      ADD_STAT(Mstate_DcachePosition14, "Number of evict lru position 14"),
      ADD_STAT(Mstate_DcachePosition15, "Number of evict lru position 15"),         
    //-------------------------------------------
    //-----------------------------------------
      ADD_STAT(Sstate_DcachePosition0, "Number of evict lru position 0"),
      ADD_STAT(Sstate_DcachePosition1, "Number of evict lru position 1"),
      ADD_STAT(Sstate_DcachePosition2, "Number of evict lru position 2"),
      ADD_STAT(Sstate_DcachePosition3, "Number of evict lru position 3"),  
      ADD_STAT(Sstate_DcachePosition4, "Number of evict lru position 4"),
      ADD_STAT(Sstate_DcachePosition5, "Number of evict lru position 5"),
      ADD_STAT(Sstate_DcachePosition6, "Number of evict lru position 6"),
      ADD_STAT(Sstate_DcachePosition7, "Number of evict lru position 7"),   
      ADD_STAT(Sstate_DcachePosition8, "Number of evict lru position 8"),
      ADD_STAT(Sstate_DcachePosition9, "Number of evict lru position 9"),
      ADD_STAT(Sstate_DcachePosition10, "Number of evict lru position 10"),
      ADD_STAT(Sstate_DcachePosition11, "Number of evict lru position 11"),  
      ADD_STAT(Sstate_DcachePosition12, "Number of evict lru position 12"),
      ADD_STAT(Sstate_DcachePosition13, "Number of evict lru position 13"),
      ADD_STAT(Sstate_DcachePosition14, "Number of evict lru position 14"),
      ADD_STAT(Sstate_DcachePosition15, "Number of evict lru position 15"),          
    //-------------------------------------------
    //-----------------------------------------
      ADD_STAT(Mstate_IcachePosition0, "Number of evict lru position 0"),
      ADD_STAT(Mstate_IcachePosition1, "Number of evict lru position 1"),
      ADD_STAT(Mstate_IcachePosition2, "Number of evict lru position 2"),
      ADD_STAT(Mstate_IcachePosition3, "Number of evict lru position 3"),  
      ADD_STAT(Mstate_IcachePosition4, "Number of evict lru position 4"),
      ADD_STAT(Mstate_IcachePosition5, "Number of evict lru position 5"),
      ADD_STAT(Mstate_IcachePosition6, "Number of evict lru position 6"),
      ADD_STAT(Mstate_IcachePosition7, "Number of evict lru position 7"),  
      ADD_STAT(Mstate_IcachePosition8, "Number of evict lru position 8"),
      ADD_STAT(Mstate_IcachePosition9, "Number of evict lru position 9"),
      ADD_STAT(Mstate_IcachePosition10, "Number of evict lru position 10"),
      ADD_STAT(Mstate_IcachePosition11, "Number of evict lru position 11"),  
      ADD_STAT(Mstate_IcachePosition12, "Number of evict lru position 12"),
      ADD_STAT(Mstate_IcachePosition13, "Number of evict lru position 13"),
      ADD_STAT(Mstate_IcachePosition14, "Number of evict lru position 14"),
      ADD_STAT(Mstate_IcachePosition15, "Number of evict lru position 15"),            
    //-------------------------------------------
    //-----------------------------------------
      ADD_STAT(Sstate_IcachePosition0, "Number of evict lru position 0"),
      ADD_STAT(Sstate_IcachePosition1, "Number of evict lru position 1"),
      ADD_STAT(Sstate_IcachePosition2, "Number of evict lru position 2"),
      ADD_STAT(Sstate_IcachePosition3, "Number of evict lru position 3"),  
      ADD_STAT(Sstate_IcachePosition4, "Number of evict lru position 4"),
      ADD_STAT(Sstate_IcachePosition5, "Number of evict lru position 5"),
      ADD_STAT(Sstate_IcachePosition6, "Number of evict lru position 6"),
      ADD_STAT(Sstate_IcachePosition7, "Number of evict lru position 7"),    
      ADD_STAT(Sstate_IcachePosition8, "Number of evict lru position 8"),
      ADD_STAT(Sstate_IcachePosition9, "Number of evict lru position 9"),
      ADD_STAT(Sstate_IcachePosition10, "Number of evict lru position 10"),
      ADD_STAT(Sstate_IcachePosition11, "Number of evict lru position 11"),  
      ADD_STAT(Sstate_IcachePosition12, "Number of evict lru position 12"),
      ADD_STAT(Sstate_IcachePosition13, "Number of evict lru position 13"),
      ADD_STAT(Sstate_IcachePosition14, "Number of evict lru position 14"),
      ADD_STAT(Sstate_IcachePosition15, "Number of evict lru position 15"),          
    //-------------------------------------------
      ADD_STAT(m_demand_hits, "Number of cache demand hits"),
      ADD_STAT(m_demand_misses, "Number of cache demand misses"),
      ADD_STAT(m_demand_accesses, "Number of cache demand accesses",
               m_demand_hits + m_demand_misses),
      ADD_STAT(m_demand_missrate, "cache demand accesses miss rate",
               m_demand_misses/(m_demand_hits + m_demand_misses) ),
      ADD_STAT(m_prefetch_hits, "Number of cache prefetch hits"),
      ADD_STAT(m_prefetch_misses, "Number of cache prefetch misses"),
      ADD_STAT(m_prefetch_accesses, "Number of cache prefetch accesses",
               m_prefetch_hits + m_prefetch_misses),
      ADD_STAT(m_accessModeType, "")
{
    numDataArrayReads
        .flags(statistics::nozero);

    numDataArrayWrites
        .flags(statistics::nozero);

    numTagArrayReads
        .flags(statistics::nozero);

    numTagArrayWrites
        .flags(statistics::nozero);

    numTagArrayStalls
        .flags(statistics::nozero);

    numDataArrayStalls
        .flags(statistics::nozero);

    htmTransCommitReadSet
        .init(8)
        .flags(statistics::pdf | statistics::dist | statistics::nozero |
            statistics::nonan);

    htmTransCommitWriteSet
        .init(8)
        .flags(statistics::pdf | statistics::dist | statistics::nozero |
            statistics::nonan);

    htmTransAbortReadSet
        .init(8)
        .flags(statistics::pdf | statistics::dist | statistics::nozero |
            statistics::nonan);

    htmTransAbortWriteSet
        .init(8)
        .flags(statistics::pdf | statistics::dist | statistics::nozero |
            statistics::nonan);

    m_prefetch_hits
        .flags(statistics::nozero);

    m_prefetch_misses
        .flags(statistics::nozero);

    m_prefetch_accesses
        .flags(statistics::nozero);

    m_accessModeType
        .init(RubyRequestType_NUM)
        .flags(statistics::pdf | statistics::total);

    for (int i = 0; i < RubyAccessMode_NUM; i++) {
        m_accessModeType
            .subname(i, RubyAccessMode_to_string(RubyAccessMode(i)))
            .flags(statistics::nozero)
            ;
    }
}

// assumption: SLICC generated files will only call this function
// once **all** resources are granted
void
CacheMemory::recordRequestType(CacheRequestType requestType, Addr addr)
{
    DPRINTF(RubyStats, "Recorded statistic: %s\n",
            CacheRequestType_to_string(requestType));
    switch(requestType) {
    case CacheRequestType_DataArrayRead:
        if (m_resource_stalls)
            dataArray.reserve(addressToCacheSet(addr));
        cacheMemoryStats.numDataArrayReads++;
        return;
    case CacheRequestType_DataArrayWrite:
        if (m_resource_stalls)
            dataArray.reserve(addressToCacheSet(addr));
        cacheMemoryStats.numDataArrayWrites++;
        return;
    case CacheRequestType_TagArrayRead:
        if (m_resource_stalls)
            tagArray.reserve(addressToCacheSet(addr));
        cacheMemoryStats.numTagArrayReads++;
        return;
    case CacheRequestType_TagArrayWrite:
        if (m_resource_stalls)
            tagArray.reserve(addressToCacheSet(addr));
        cacheMemoryStats.numTagArrayWrites++;
        return;
    default:
        warn("CacheMemory access_type not found: %s",
             CacheRequestType_to_string(requestType));
    }
}

bool
CacheMemory::checkResourceAvailable(CacheResourceType res, Addr addr)
{
    if (!m_resource_stalls) {
        return true;
    }

    if (res == CacheResourceType_TagArray) {
        if (tagArray.tryAccess(addressToCacheSet(addr))) return true;
        else {
            DPRINTF(RubyResourceStalls,
                    "Tag array stall on addr %#x in set %d\n",
                    addr, addressToCacheSet(addr));
            cacheMemoryStats.numTagArrayStalls++;
            return false;
        }
    } else if (res == CacheResourceType_DataArray) {
        if (dataArray.tryAccess(addressToCacheSet(addr))) return true;
        else {
            DPRINTF(RubyResourceStalls,
                    "Data array stall on addr %#x in set %d\n",
                    addr, addressToCacheSet(addr));
            cacheMemoryStats.numDataArrayStalls++;
            return false;
        }
    } else {
        panic("Unrecognized cache resource type.");
    }
}

bool
CacheMemory::isBlockInvalid(int64_t cache_set, int64_t loc)
{
  return (m_cache[cache_set][loc]->m_Permission == AccessPermission_Invalid);
}

bool
CacheMemory::isBlockNotBusy(int64_t cache_set, int64_t loc)
{
  return (m_cache[cache_set][loc]->m_Permission != AccessPermission_Busy);
}

/* hardware transactional memory */

void
CacheMemory::htmAbortTransaction()
{
    uint64_t htmReadSetSize = 0;
    uint64_t htmWriteSetSize = 0;

    // iterate through every set and way to get a cache line
    for (auto i = m_cache.begin(); i != m_cache.end(); ++i)
    {
        std::vector<AbstractCacheEntry*> set = *i;

        for (auto j = set.begin(); j != set.end(); ++j)
        {
            AbstractCacheEntry *line = *j;

            if (line != nullptr) {
                htmReadSetSize += (line->getInHtmReadSet() ? 1 : 0);
                htmWriteSetSize += (line->getInHtmWriteSet() ? 1 : 0);
                if (line->getInHtmWriteSet()) {
                    line->invalidateEntry();
                }
                line->setInHtmWriteSet(false);
                line->setInHtmReadSet(false);
                line->clearLocked();
            }
        }
    }

    cacheMemoryStats.htmTransAbortReadSet.sample(htmReadSetSize);
    cacheMemoryStats.htmTransAbortWriteSet.sample(htmWriteSetSize);
    DPRINTF(HtmMem, "htmAbortTransaction: read set=%u write set=%u\n",
        htmReadSetSize, htmWriteSetSize);
}

void
CacheMemory::htmCommitTransaction()
{
    uint64_t htmReadSetSize = 0;
    uint64_t htmWriteSetSize = 0;

    // iterate through every set and way to get a cache line
    for (auto i = m_cache.begin(); i != m_cache.end(); ++i)
    {
        std::vector<AbstractCacheEntry*> set = *i;

        for (auto j = set.begin(); j != set.end(); ++j)
        {
            AbstractCacheEntry *line = *j;
            if (line != nullptr) {
                htmReadSetSize += (line->getInHtmReadSet() ? 1 : 0);
                htmWriteSetSize += (line->getInHtmWriteSet() ? 1 : 0);
                line->setInHtmWriteSet(false);
                line->setInHtmReadSet(false);
                line->clearLocked();
             }
        }
    }

    cacheMemoryStats.htmTransCommitReadSet.sample(htmReadSetSize);
    cacheMemoryStats.htmTransCommitWriteSet.sample(htmWriteSetSize);
    DPRINTF(HtmMem, "htmCommitTransaction: read set=%u write set=%u\n",
        htmReadSetSize, htmWriteSetSize);
}

void
CacheMemory::profileDcache_Mstate_LruPosition(int pos)
{
    if (pos == 0) cacheMemoryStats.Mstate_DcachePosition0++;
    else if ( pos == 1) cacheMemoryStats.Mstate_DcachePosition1++;
    else if ( pos == 2) cacheMemoryStats.Mstate_DcachePosition2++;
    else if ( pos == 3) cacheMemoryStats.Mstate_DcachePosition3++;
    else if ( pos == 4) cacheMemoryStats.Mstate_DcachePosition4++;
    else if ( pos == 5) cacheMemoryStats.Mstate_DcachePosition5++;
    else if ( pos == 6) cacheMemoryStats.Mstate_DcachePosition6++;
    else if ( pos == 7) cacheMemoryStats.Mstate_DcachePosition7++;
    else if ( pos == 8) cacheMemoryStats.Mstate_DcachePosition8++;
    else if ( pos == 9) cacheMemoryStats.Mstate_DcachePosition9++;
    else if ( pos == 10) cacheMemoryStats.Mstate_DcachePosition10++;
    else if ( pos == 11) cacheMemoryStats.Mstate_DcachePosition11++;
    else if ( pos == 12) cacheMemoryStats.Mstate_DcachePosition12++;
    else if ( pos == 13) cacheMemoryStats.Mstate_DcachePosition13++;
    else if ( pos == 14) cacheMemoryStats.Mstate_DcachePosition14++;
    else if ( pos == 15) cacheMemoryStats.Mstate_DcachePosition15++;
}

void
CacheMemory::profileDcache_Sstate_LruPosition(int pos)
{
    if (pos == 0) cacheMemoryStats.Sstate_DcachePosition0++;
    else if ( pos == 1) cacheMemoryStats.Sstate_DcachePosition1++;
    else if ( pos == 2) cacheMemoryStats.Sstate_DcachePosition2++;
    else if ( pos == 3) cacheMemoryStats.Sstate_DcachePosition3++;
    else if ( pos == 4) cacheMemoryStats.Sstate_DcachePosition4++;
    else if ( pos == 5) cacheMemoryStats.Sstate_DcachePosition5++;
    else if ( pos == 6) cacheMemoryStats.Sstate_DcachePosition6++;
    else if ( pos == 7) cacheMemoryStats.Sstate_DcachePosition7++;
    else if ( pos == 8) cacheMemoryStats.Sstate_DcachePosition8++;
    else if ( pos == 9) cacheMemoryStats.Sstate_DcachePosition9++;
    else if ( pos == 10) cacheMemoryStats.Sstate_DcachePosition10++;
    else if ( pos == 11) cacheMemoryStats.Sstate_DcachePosition11++;
    else if ( pos == 12) cacheMemoryStats.Sstate_DcachePosition12++;
    else if ( pos == 13) cacheMemoryStats.Sstate_DcachePosition13++;
    else if ( pos == 14) cacheMemoryStats.Sstate_DcachePosition14++;
    else if ( pos == 15) cacheMemoryStats.Sstate_DcachePosition15++;
}

void
CacheMemory::profileIcache_Mstate_LruPosition(int pos)
{
    if (pos == 0) cacheMemoryStats.Mstate_IcachePosition0++;
    else if ( pos == 1) cacheMemoryStats.Mstate_IcachePosition1++;
    else if ( pos == 2) cacheMemoryStats.Mstate_IcachePosition2++;
    else if ( pos == 3) cacheMemoryStats.Mstate_IcachePosition3++;
    else if ( pos == 4) cacheMemoryStats.Mstate_IcachePosition4++;
    else if ( pos == 5) cacheMemoryStats.Mstate_IcachePosition5++;
    else if ( pos == 6) cacheMemoryStats.Mstate_IcachePosition6++;
    else if ( pos == 7) cacheMemoryStats.Mstate_IcachePosition7++;
    else if ( pos == 8) cacheMemoryStats.Mstate_IcachePosition8++;
    else if ( pos == 9) cacheMemoryStats.Mstate_IcachePosition9++;
    else if ( pos == 10) cacheMemoryStats.Mstate_IcachePosition10++;
    else if ( pos == 11) cacheMemoryStats.Mstate_IcachePosition11++;
    else if ( pos == 12) cacheMemoryStats.Mstate_IcachePosition12++;
    else if ( pos == 13) cacheMemoryStats.Mstate_IcachePosition13++;
    else if ( pos == 14) cacheMemoryStats.Mstate_IcachePosition14++;
    else if ( pos == 15) cacheMemoryStats.Mstate_IcachePosition15++;
}

void
CacheMemory::profileIcache_Sstate_LruPosition(int pos)
{
    if (pos == 0) cacheMemoryStats.Sstate_IcachePosition0++;
    else if ( pos == 1) cacheMemoryStats.Sstate_IcachePosition1++;
    else if ( pos == 2) cacheMemoryStats.Sstate_IcachePosition2++;
    else if ( pos == 3) cacheMemoryStats.Sstate_IcachePosition3++;
    else if ( pos == 4) cacheMemoryStats.Sstate_IcachePosition4++;
    else if ( pos == 5) cacheMemoryStats.Sstate_IcachePosition5++;
    else if ( pos == 6) cacheMemoryStats.Sstate_IcachePosition6++;
    else if ( pos == 7) cacheMemoryStats.Sstate_IcachePosition7++;
    else if ( pos == 8) cacheMemoryStats.Sstate_IcachePosition8++;
    else if ( pos == 9) cacheMemoryStats.Sstate_IcachePosition9++;
    else if ( pos == 10) cacheMemoryStats.Sstate_IcachePosition10++;
    else if ( pos == 11) cacheMemoryStats.Sstate_IcachePosition11++;
    else if ( pos == 12) cacheMemoryStats.Sstate_IcachePosition12++;
    else if ( pos == 13) cacheMemoryStats.Sstate_IcachePosition13++;
    else if ( pos == 14) cacheMemoryStats.Sstate_IcachePosition14++;
    else if ( pos == 15) cacheMemoryStats.Sstate_IcachePosition15++;
}

void
CacheMemory::profileDemandHit()
{
    cacheMemoryStats.m_demand_hits++;
}

void
CacheMemory::profileDemandMiss()
{
    cacheMemoryStats.m_demand_misses++;
}

void
CacheMemory::profilePrefetchHit()
{
    cacheMemoryStats.m_prefetch_hits++;
}

void
CacheMemory::profilePrefetchMiss()
{
    cacheMemoryStats.m_prefetch_misses++;
}

} // namespace ruby
} // namespace gem5
