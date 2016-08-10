/*
 * Copyright (c) 2010-2014 ARM Limited
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
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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
 *
 * Authors: Kevin Lim
 *          Korey Sewell
 */

#ifndef __CPU_O3_FETCH_IMPL_HH__
#define __CPU_O3_FETCH_IMPL_HH__

#include <algorithm>
#include <cstring>
#include <list>
#include <map>
#include <queue>

#include "arch/isa_traits.hh"
#include "arch/tlb.hh"
#include "arch/utility.hh"
#include "arch/vtophys.hh"
#include "base/random.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
//#include "cpu/checker/cpu.hh"
#include "cpu/o3/fetch.hh"
#include "cpu/exetrace.hh"
#include "debug/Activity.hh"
#include "debug/Drain.hh"
#include "debug/Fetch.hh"
#include "debug/O3PipeView.hh"
#include "mem/packet.hh"
#include "params/DerivO3CPU.hh"
#include "sim/byteswap.hh"
#include "sim/core.hh"
#include "sim/eventq.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

using namespace std;

template<class Impl>
DefaultFetch<Impl>::DefaultFetch(O3CPU *_cpu, DerivO3CPUParams *params)
    : cpu(_cpu),
      decodeToFetchDelay(params->decodeToFetchDelay),
      renameToFetchDelay(params->renameToFetchDelay),
      iewToFetchDelay(params->iewToFetchDelay),
      commitToFetchDelay(params->commitToFetchDelay),
      fetchWidth(params->fetchWidth),
      decodeWidth(params->decodeWidth),
      retryPkt(NULL),
      retryTid(InvalidThreadID),
      cacheBlkSize(cpu->cacheLineSize()),
      fetchBufferSize(params->fetchBufferSize),
      fetchBufferMask(fetchBufferSize - 1),
      fetchQueueSize(params->fetchQueueSize),
      numThreads(params->numThreads),
      numFetchingThreads(params->smtNumFetchingThreads),
      finishTranslationEvent(this)
{
    if (numThreads > Impl::MaxThreads)
        fatal("numThreads (%d) is larger than compiled limit (%d),\n"
              "\tincrease MaxThreads in src/cpu/o3/impl.hh\n",
              numThreads, static_cast<int>(Impl::MaxThreads));
    if (fetchWidth > Impl::MaxWidth)
        fatal("fetchWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/impl.hh\n",
             fetchWidth, static_cast<int>(Impl::MaxWidth));
    if (fetchBufferSize > cacheBlkSize)
        fatal("fetch buffer size (%u bytes) is greater than the cache "
              "block size (%u bytes)\n", fetchBufferSize, cacheBlkSize);
    if (cacheBlkSize % fetchBufferSize)
        fatal("cache block (%u bytes) is not a multiple of the "
              "fetch buffer (%u bytes)\n", cacheBlkSize, fetchBufferSize);

    std::string policy = params->smtFetchPolicy;

    // Convert string to lowercase
    std::transform(policy.begin(), policy.end(), policy.begin(),
                   (int(*)(int)) tolower);

    // Figure out fetch policy
    if (policy == "singlethread") {
        fetchPolicy = SingleThread;
        if (numThreads > 1)
            panic("Invalid Fetch Policy for a SMT workload.");
    } else if (policy == "roundrobin") {
        fetchPolicy = RoundRobin;
        DPRINTF(Fetch, "Fetch policy set to Round Robin\n");
    } else if (policy == "branch") {
        fetchPolicy = Branch;
        DPRINTF(Fetch, "Fetch policy set to Branch Count\n");
    } else if (policy == "iqcount") {
        fetchPolicy = IQ;
        DPRINTF(Fetch, "Fetch policy set to IQ count\n");
    } else if (policy == "lsqcount") {
        fetchPolicy = LSQ;
        DPRINTF(Fetch, "Fetch policy set to LSQ count\n");
    } else {
        fatal("Invalid Fetch Policy. Options Are: {SingleThread,"
              " RoundRobin,LSQcount,IQcount}\n");
    }

    // Get the size of an instruction.
    instSize = sizeof(TheISA::MachInst);

    for (int i = 0; i < Impl::MaxThreads; i++) {
        decoder[i] = NULL;
        fetchBuffer[i] = NULL;
        fetchBufferPC[i] = 0;
        fetchBufferValid[i] = false;
    }

    branchPred = params->branchPred;

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        decoder[tid] = new TheISA::Decoder;
        // Create space to buffer the cache line data,
        // which may not hold the entire cache line.
        fetchBuffer[tid] = new uint8_t[fetchBufferSize];
    }
}

template <class Impl>
std::string
DefaultFetch<Impl>::name() const
{
    return cpu->name() + ".fetch";
}

template <class Impl>
void
DefaultFetch<Impl>::regProbePoints()
{
    ppFetch = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "Fetch");
}

template <class Impl>
void
DefaultFetch<Impl>::regStats()
{
    icacheStallCycles
        .name(name() + ".icacheStallCycles")
        .desc("Number of cycles fetch is stalled on an Icache miss")
        .prereq(icacheStallCycles);

    fetchedInsts
        .name(name() + ".Insts")
        .desc("Number of instructions fetch has processed")
        .prereq(fetchedInsts);

    fetchedBranches
        .name(name() + ".Branches")
        .desc("Number of branches that fetch encountered")
        .prereq(fetchedBranches);

    predictedBranches
        .name(name() + ".predictedBranches")
        .desc("Number of branches that fetch has predicted taken")
        .prereq(predictedBranches);

    fetchCycles
        .name(name() + ".Cycles")
        .desc("Number of cycles fetch has run and was not squashing or"
              " blocked")
        .prereq(fetchCycles);

    fetchSquashCycles
        .name(name() + ".SquashCycles")
        .desc("Number of cycles fetch has spent squashing")
        .prereq(fetchSquashCycles);

    fetchTlbCycles
        .name(name() + ".TlbCycles")
        .desc("Number of cycles fetch has spent waiting for tlb")
        .prereq(fetchTlbCycles);

    fetchIdleCycles
        .name(name() + ".IdleCycles")
        .desc("Number of cycles fetch was idle")
        .prereq(fetchIdleCycles);

    fetchBlockedCycles
        .name(name() + ".BlockedCycles")
        .desc("Number of cycles fetch has spent blocked")
        .prereq(fetchBlockedCycles);

    fetchedCacheLines
        .name(name() + ".CacheLines")
        .desc("Number of cache lines fetched")
        .prereq(fetchedCacheLines);

    fetchMiscStallCycles
        .name(name() + ".MiscStallCycles")
        .desc("Number of cycles fetch has spent waiting on interrupts, or "
              "bad addresses, or out of MSHRs")
        .prereq(fetchMiscStallCycles);

    fetchPendingDrainCycles
        .name(name() + ".PendingDrainCycles")
        .desc("Number of cycles fetch has spent waiting on pipes to drain")
        .prereq(fetchPendingDrainCycles);

    fetchNoActiveThreadStallCycles
        .name(name() + ".NoActiveThreadStallCycles")
        .desc("Number of stall cycles due to no active thread to fetch from")
        .prereq(fetchNoActiveThreadStallCycles);

    fetchPendingTrapStallCycles
        .name(name() + ".PendingTrapStallCycles")
        .desc("Number of stall cycles due to pending traps")
        .prereq(fetchPendingTrapStallCycles);

    fetchPendingQuiesceStallCycles
        .name(name() + ".PendingQuiesceStallCycles")
        .desc("Number of stall cycles due to pending quiesce instructions")
        .prereq(fetchPendingQuiesceStallCycles);

    fetchIcacheWaitRetryStallCycles
        .name(name() + ".IcacheWaitRetryStallCycles")
        .desc("Number of stall cycles due to full MSHR")
        .prereq(fetchIcacheWaitRetryStallCycles);

    fetchIcacheSquashes
        .name(name() + ".IcacheSquashes")
        .desc("Number of outstanding Icache misses that were squashed")
        .prereq(fetchIcacheSquashes);

    fetchTlbSquashes
        .name(name() + ".ItlbSquashes")
        .desc("Number of outstanding ITLB misses that were squashed")
        .prereq(fetchTlbSquashes);

    fetchNisnDist
        .init(/* base value */ 0,
              /* last value */ fetchWidth,
              /* bucket size */ 1)
        .name(name() + ".rateDist")
        .desc("Number of instructions fetched each cycle (Total)")
        .flags(Stats::pdf);

    idleRate
        .name(name() + ".idleRate")
        .desc("Percent of cycles fetch was idle")
        .prereq(idleRate);
    idleRate = fetchIdleCycles * 100 / cpu->numCycles;

    branchRate
        .name(name() + ".branchRate")
        .desc("Number of branch fetches per cycle")
        .flags(Stats::total);
    branchRate = fetchedBranches / cpu->numCycles;

    fetchRate
        .name(name() + ".rate")
        .desc("Number of inst fetches per cycle")
        .flags(Stats::total);
    fetchRate = fetchedInsts / cpu->numCycles;
}

template<class Impl>
void
DefaultFetch<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer)
{
    timeBuffer = time_buffer;

    // Create wires to get information from proper places in time buffer.
    fromDecode = timeBuffer->getWire(-decodeToFetchDelay);
    fromRename = timeBuffer->getWire(-renameToFetchDelay);
    fromIEW = timeBuffer->getWire(-iewToFetchDelay);
    fromCommit = timeBuffer->getWire(-commitToFetchDelay);
}

template<class Impl>
void
DefaultFetch<Impl>::setActiveThreads(std::list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
}

template<class Impl>
void
DefaultFetch<Impl>::setFetchQueue(TimeBuffer<FetchStruct> *ftb_ptr)
{
    // Create wire to write information to proper place in fetch time buf.
    toDecode = ftb_ptr->getWire(0);
}

template<class Impl>
void
DefaultFetch<Impl>::startupStage()
{
    assert(priorityList.empty());
    resetStage();

    // Fetch needs to start fetching instructions at the very beginning,
    // so it must start up in active state.
    switchToActive();
}

template<class Impl>
void
DefaultFetch<Impl>::resetStage()
{
    numInst = 0;
    interruptPending = false;
    cacheBlocked = false;

    priorityList.clear();

    // Setup PC and nextPC with initial state.
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        fetchStatus[tid] = Running;
        pc[tid] = cpu->pcState(tid);
        fetchOffset[tid] = 0;
        macroop[tid] = NULL;

        delayedCommit[tid] = false;
        memReq[tid] = NULL;

        stalls[tid].decode = false;
        stalls[tid].drain = false;

        fetchBufferPC[tid] = 0;
        fetchBufferValid[tid] = false;

        fetchQueue[tid].clear();

        priorityList.push_back(tid);
    }

    wroteToTimeBuffer = false;
    _status = Inactive;
}

template<class Impl>
void
DefaultFetch<Impl>::processCacheCompletion(PacketPtr pkt)
{
    ThreadID tid = pkt->req->threadId();

    DPRINTF(Fetch, "[tid:%u] Waking up from cache miss.\n", tid);
    assert(!cpu->switchedOut());

    // Only change the status if it's still waiting on the icache access
    // to return.
    if (fetchStatus[tid] != IcacheWaitResponse ||
        pkt->req != memReq[tid]) {
        ++fetchIcacheSquashes;
        delete pkt->req;
        delete pkt;
        return;
    }

    memcpy(fetchBuffer[tid], pkt->getConstPtr<uint8_t>(), fetchBufferSize);
    fetchBufferValid[tid] = true;

    // Wake up the CPU (if it went to sleep and was waiting on
    // this completion event).
    cpu->wakeCPU();

    DPRINTF(Activity, "[tid:%u] Activating fetch due to cache completion\n",
            tid);

    switchToActive();

    // Only switch to IcacheAccessComplete if we're not stalled as well.
    if (checkStall(tid)) {
        fetchStatus[tid] = Blocked;
    } else {
        fetchStatus[tid] = IcacheAccessComplete;
    }

    pkt->req->setAccessLatency();
    cpu->ppInstAccessComplete->notify(pkt);
    // Reset the mem req to NULL.
    delete pkt->req;
    delete pkt;
    memReq[tid] = NULL;
}

template <class Impl>
void
DefaultFetch<Impl>::drainResume()
{
    for (ThreadID i = 0; i < numThreads; ++i)
        stalls[i].drain = false;
}

template <class Impl>
void
DefaultFetch<Impl>::drainSanityCheck() const
{
    assert(isDrained());
    assert(retryPkt == NULL);
    assert(retryTid == InvalidThreadID);
    assert(!cacheBlocked);
    assert(!interruptPending);

    for (ThreadID i = 0; i < numThreads; ++i) {
        assert(!memReq[i]);
        assert(fetchStatus[i] == Idle || stalls[i].drain);
    }

    branchPred->drainSanityCheck();
}

template <class Impl>
bool
DefaultFetch<Impl>::isDrained() const
{
    /* Make sure that threads are either idle of that the commit stage
     * has signaled that draining has completed by setting the drain
     * stall flag. This effectively forces the pipeline to be disabled
     * until the whole system is drained (simulation may continue to
     * drain other components).
     */
    for (ThreadID i = 0; i < numThreads; ++i) {
        // Verify fetch queues are drained
        if (!fetchQueue[i].empty())
            return false;

        // Return false if not idle or drain stalled
        if (fetchStatus[i] != Idle) {
            if (fetchStatus[i] == Blocked && stalls[i].drain)
                continue;
            else
                return false;
        }
    }

    /* The pipeline might start up again in the middle of the drain
     * cycle if the finish translation event is scheduled, so make
     * sure that's not the case.
     */
    return !finishTranslationEvent.scheduled();
}

template <class Impl>
void
DefaultFetch<Impl>::takeOverFrom()
{
    assert(cpu->getInstPort().isConnected());
    resetStage();

}

template <class Impl>
void
DefaultFetch<Impl>::drainStall(ThreadID tid)
{
    assert(cpu->isDraining());
    assert(!stalls[tid].drain);
    DPRINTF(Drain, "%i: Thread drained.\n", tid);
    stalls[tid].drain = true;
}

template <class Impl>
void
DefaultFetch<Impl>::wakeFromQuiesce()
{
    DPRINTF(Fetch, "Waking up from quiesce\n");
    // Hopefully this is safe
    // @todo: Allow other threads to wake from quiesce.
    fetchStatus[0] = Running;
}

template <class Impl>
inline void
DefaultFetch<Impl>::switchToActive()
{
    if (_status == Inactive) {
        DPRINTF(Activity, "Activating stage.\n");

        cpu->activateStage(O3CPU::FetchIdx);

        _status = Active;
    }
}

template <class Impl>
inline void
DefaultFetch<Impl>::switchToInactive()
{
    if (_status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");

        cpu->deactivateStage(O3CPU::FetchIdx);

        _status = Inactive;
    }
}

template <class Impl>
void
DefaultFetch<Impl>::deactivateThread(ThreadID tid)
{
    // Update priority list
    auto thread_it = std::find(priorityList.begin(), priorityList.end(), tid);
    if (thread_it != priorityList.end()) {
        priorityList.erase(thread_it);
    }
}

template <class Impl>
bool
DefaultFetch<Impl>::lookupAndUpdateNextPC(
        DynInstPtr &inst, TheISA::PCState &nextPC)
{
    // Do branch prediction check here.
    // A bit of a misnomer...next_PC is actually the current PC until
    // this function updates it.
    bool predict_taken;

    if (!inst->isControl()) {
        TheISA::advancePC(nextPC, inst->staticInst);
        inst->setPredTarg(nextPC);
        inst->setPredTaken(false);
        return false;
    }

    ThreadID tid = inst->threadNumber;
    predict_taken = branchPred->predict(inst->staticInst, inst->seqNum,
                                        nextPC, tid);

    if (predict_taken) {
        DPRINTF(Fetch, "[tid:%i]: [sn:%i]:  Branch predicted to be taken to %s.\n",
                tid, inst->seqNum, nextPC);
    } else {
        DPRINTF(Fetch, "[tid:%i]: [sn:%i]:Branch predicted to be not taken.\n",
                tid, inst->seqNum);
    }

    DPRINTF(Fetch, "[tid:%i]: [sn:%i] Branch predicted to go to %s.\n",
            tid, inst->seqNum, nextPC);
    inst->setPredTarg(nextPC);
    inst->setPredTaken(predict_taken);

    ++fetchedBranches;

    if (predict_taken) {
        ++predictedBranches;
    }

    return predict_taken;
}
/*
 * fetchCacheLine()
 * 初始化fault为NoFault
 * 如果cacheBlocked为true，即cache阻塞
 * 	  直接return false；
 * 如果是中断并且没有延迟提交(!delayedCommit[tid]==true)
 *    直接return false；
 * fetchBufferBlockPC为参数vaddr进行align(重置为0)后的地址
 * RequestPtr mem_re//传参，初始化mem_re变量
 * 用cpu->taskId()初始化mem_re的_taskId的值
 * 令memReq[tid] = mem_re;
 * 将fetchStatus[tid]的状态改为ItlbWait
 * new FetchTranslation(this)实际上初始化的是指向DefaultFetch<Impl>的指针
 * 声明指向FetchTranslation的指针变量trans，并定义
 * 调用cpu->itb->translateTiming()函数
 * 参数为(mem_req, cpu->thread[tid]->getTC(),trans, BaseTLB::Execute);
 * getTC()函数全局查找有三个，作用大致都是返回指向ThreadContext的指针
 * return true;
 */
template <class Impl>
bool
DefaultFetch<Impl>::fetchCacheLine(Addr vaddr, ThreadID tid, Addr pc)
{
    Fault fault = NoFault;

    assert(!cpu->switchedOut());

    // @todo: not sure if these should block translation.
    //AlphaDep
    if (cacheBlocked) {
        DPRINTF(Fetch, "[tid:%i] Can't fetch cache line, cache blocked\n",
                tid);
        return false;
    } else if (checkInterrupt(pc) && !delayedCommit[tid]) {
        // Hold off fetch from getting new instructions when:
        // Cache is blocked, or
        // while an interrupt is pending and we're not in PAL mode, or
        // fetch is switched out.
        DPRINTF(Fetch, "[tid:%i] Can't fetch cache line, interrupt pending\n",
                tid);
        return false;
    }

    // Align the fetch address to the start of a fetch buffer segment.
    Addr fetchBufferBlockPC = fetchBufferAlignPC(vaddr);

    DPRINTF(Fetch, "[tid:%i] Fetching cache line %#x for addr %#x\n",
            tid, fetchBufferBlockPC, vaddr);

    // Setup the memReq to do a read of the first instruction's address.
    // Set the appropriate read size and flags as well.
    // Build request here.
    RequestPtr mem_req =
        new Request(tid, fetchBufferBlockPC, fetchBufferSize,
                    Request::INST_FETCH, cpu->instMasterId(), pc,
                    cpu->thread[tid]->contextId(), tid);

    mem_req->taskId(cpu->taskId());

    memReq[tid] = mem_req;

    // Initiate translation of the icache block
    fetchStatus[tid] = ItlbWait;
    FetchTranslation *trans = new FetchTranslation(this);
    cpu->itb->translateTiming(mem_req, cpu->thread[tid]->getTC(),
                              trans, BaseTLB::Execute);
    return true;
}


template <class Impl>
void
DefaultFetch<Impl>::finishTranslation(const Fault &fault, RequestPtr mem_req)
{
	//
    ThreadID tid = mem_req->threadId();
    Addr fetchBufferBlockPC = mem_req->getVaddr();

    assert(!cpu->switchedOut());

    // Wake up CPU if it was idle
    cpu->wakeCPU();

    if (fetchStatus[tid] != ItlbWait || mem_req != memReq[tid] ||
        mem_req->getVaddr() != memReq[tid]->getVaddr()) {
        DPRINTF(Fetch, "[tid:%i] Ignoring itlb completed after squash\n",
                tid);
        ++fetchTlbSquashes;
        delete mem_req;
        return;
    }


    // If translation was successful, attempt to read the icache block.
    if (fault == NoFault) {
        // Check that we're not going off into random memory
        // If we have, just wait around for commit to squash something and put
        // us on the right track
        if (!cpu->system->isMemAddr(mem_req->getPaddr())) {
            warn("Address %#x is outside of physical memory, stopping fetch\n",
                    mem_req->getPaddr());
            fetchStatus[tid] = NoGoodAddr;
            delete mem_req;
            memReq[tid] = NULL;
            return;
        }

        // Build packet here.
        PacketPtr data_pkt = new Packet(mem_req, MemCmd::ReadReq);
        data_pkt->dataDynamic(new uint8_t[fetchBufferSize]);

        fetchBufferPC[tid] = fetchBufferBlockPC;
        fetchBufferValid[tid] = false;
        DPRINTF(Fetch, "Fetch: Doing instruction read.\n");

        fetchedCacheLines++;

        // Access the cache.
        if (!cpu->getInstPort().sendTimingReq(data_pkt)) {
            assert(retryPkt == NULL);
            assert(retryTid == InvalidThreadID);
            DPRINTF(Fetch, "[tid:%i] Out of MSHRs!\n", tid);

            fetchStatus[tid] = IcacheWaitRetry;
            retryPkt = data_pkt;
            retryTid = tid;
            cacheBlocked = true;
        } else {
            DPRINTF(Fetch, "[tid:%i]: Doing Icache access.\n", tid);
            DPRINTF(Activity, "[tid:%i]: Activity: Waiting on I-cache "
                    "response.\n", tid);
            lastIcacheStall[tid] = curTick();
            fetchStatus[tid] = IcacheWaitResponse;
        }
    } else {
        // Don't send an instruction to decode if we can't handle it.
        if (!(numInst < fetchWidth) || !(fetchQueue[tid].size() < fetchQueueSize)) {
            assert(!finishTranslationEvent.scheduled());
            finishTranslationEvent.setFault(fault);
            finishTranslationEvent.setReq(mem_req);
            cpu->schedule(finishTranslationEvent,
                          cpu->clockEdge(Cycles(1)));
            return;
        }
        DPRINTF(Fetch, "[tid:%i] Got back req with addr %#x but expected %#x\n",
                tid, mem_req->getVaddr(), memReq[tid]->getVaddr());
        // Translation faulted, icache request won't be sent.
        delete mem_req;
        memReq[tid] = NULL;

        // Send the fault to commit.  This thread will not do anything
        // until commit handles the fault.  The only other way it can
        // wake up is if a squash comes along and changes the PC.
        TheISA::PCState fetchPC = pc[tid];

        DPRINTF(Fetch, "[tid:%i]: Translation faulted, building noop.\n", tid);
        // We will use a nop in ordier to carry the fault.
        DynInstPtr instruction = buildInst(tid,
                decoder[tid]->decode(TheISA::NoopMachInst, fetchPC.instAddr()),
                NULL, fetchPC, fetchPC, false);

        instruction->setPredTarg(fetchPC);
        instruction->fault = fault;
        wroteToTimeBuffer = true;

        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();

        fetchStatus[tid] = TrapPending;

        DPRINTF(Fetch, "[tid:%i]: Blocked, need to handle the trap.\n", tid);
        DPRINTF(Fetch, "[tid:%i]: fault (%s) detected @ PC %s.\n",
                tid, fault->name(), pc[tid]);
    }
    _status = updateFetchStatus();
}

template <class Impl>
inline void
DefaultFetch<Impl>::doSquash(const TheISA::PCState &newPC,
                             const DynInstPtr squashInst, ThreadID tid)
{
    DPRINTF(Fetch, "[tid:%i]: Squashing, setting PC to: %s.\n",
            tid, newPC);

    pc[tid] = newPC;
    fetchOffset[tid] = 0;
    if (squashInst && squashInst->pcState().instAddr() == newPC.instAddr())
        macroop[tid] = squashInst->macroop;
    else
        macroop[tid] = NULL;
    decoder[tid]->reset();

    // Clear the icache miss if it's outstanding.
    if (fetchStatus[tid] == IcacheWaitResponse) {
        DPRINTF(Fetch, "[tid:%i]: Squashing outstanding Icache miss.\n",
                tid);
        memReq[tid] = NULL;
    } else if (fetchStatus[tid] == ItlbWait) {
        DPRINTF(Fetch, "[tid:%i]: Squashing outstanding ITLB miss.\n",
                tid);
        memReq[tid] = NULL;
    }

    // Get rid of the retrying packet if it was from this thread.
    if (retryTid == tid) {
        assert(cacheBlocked);
        if (retryPkt) {
            delete retryPkt->req;
            delete retryPkt;
        }
        retryPkt = NULL;
        retryTid = InvalidThreadID;
    }

    fetchStatus[tid] = Squashing;

    // Empty fetch queue
    fetchQueue[tid].clear();

    // microops are being squashed, it is not known wheather the
    // youngest non-squashed microop was  marked delayed commit
    // or not. Setting the flag to true ensures that the
    // interrupts are not handled when they cannot be, though
    // some opportunities to handle interrupts may be missed.
    delayedCommit[tid] = true;

    ++fetchSquashCycles;
}

template<class Impl>
void
DefaultFetch<Impl>::squashFromDecode(const TheISA::PCState &newPC,
                                     const DynInstPtr squashInst,
                                     const InstSeqNum seq_num, ThreadID tid)
{
    DPRINTF(Fetch, "[tid:%i]: Squashing from decode.\n", tid);

    doSquash(newPC, squashInst, tid);

    // Tell the CPU to remove any instructions that are in flight between
    // fetch and decode.
    cpu->removeInstsUntil(seq_num, tid);
}

template<class Impl>
bool
DefaultFetch<Impl>::checkStall(ThreadID tid) const
{
    bool ret_val = false;

    if (stalls[tid].drain) {
        assert(cpu->isDraining());
        DPRINTF(Fetch,"[tid:%i]: Drain stall detected.\n",tid);
        ret_val = true;
    }

    return ret_val;
}

template<class Impl>
typename DefaultFetch<Impl>::FetchStatus
DefaultFetch<Impl>::updateFetchStatus()
{
    //Check Running
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        //循环判断，每个线程判断一次
        //如果fetchStatus(其实是ThreadStatus)为Running、Squashing、IcacheAccessComplete之一
        	//如果FetchStatus(真正的stage status)为Inactive
        		//那么调用activateStage(更新stageActive[stageIdx]为true)
        	//返回Active
        //FetchStatus为Active，那么调用deactivateStage(更新stageActive[stageIdx]为false)
        //返回Inactive
        if (fetchStatus[tid] == Running ||
            fetchStatus[tid] == Squashing ||
            fetchStatus[tid] == IcacheAccessComplete) {

            if (_status == Inactive) {
                DPRINTF(Activity, "[tid:%i]: Activating stage.\n",tid);

                if (fetchStatus[tid] == IcacheAccessComplete) {
                    DPRINTF(Activity, "[tid:%i]: Activating fetch due to cache"
                            "completion\n",tid);
                }

                cpu->activateStage(O3CPU::FetchIdx);
            }

            return Active;
        }
    }

    // Stage is switching from active to inactive, notify CPU of it.
    if (_status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");

        cpu->deactivateStage(O3CPU::FetchIdx);
    }

    return Inactive;
}

template <class Impl>
void
DefaultFetch<Impl>::squash(const TheISA::PCState &newPC,
                           const InstSeqNum seq_num, DynInstPtr squashInst,
                           ThreadID tid)
{
    DPRINTF(Fetch, "[tid:%u]: Squash from commit.\n", tid);

    doSquash(newPC, squashInst, tid);

    // Tell the CPU to remove any instructions that are not in the ROB.
    cpu->removeInstsNotInROB(tid);
}

template <class Impl>
void
DefaultFetch<Impl>::tick()
{
	//active thread的begin位置
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();//active thread的end位置
    bool status_change = false;//用于标记fetch thread的状态改变与否

    wroteToTimeBuffer = false;//用于标记此cycle内的fetch是否被写入time buffer
    //将issuePipelinedIfetch的数组值全设为false
    //issuePipelinedIfetch数组的索引值为fetch thread id
    //issuePipelinedIfetch为true时，表示该线程的I-cache request应该被issued
    for (ThreadID i = 0; i < numThreads; ++i) {
        issuePipelinedIfetch[i] = false;
    }
    //updated_status//
    while (threads != end) {
        ThreadID tid = *threads++;

        // Check the signals for each thread to determine the proper status
        // for each thread.
        //根据checkSignalsAndUpdate来更新status_change的状态
        bool updated_status = checkSignalsAndUpdate(tid);
        status_change =  status_change || updated_status;
    }
    //记录gem5的track
    DPRINTF(Fetch, "Running stage.\n");
    //如果是FullSystem模式
    	//如果从commit stage那里得到中断信息，则设置interruptPending为true
    	//如果从commit stage那里得到清除中断信息，则设置interruptPending为false
    if (FullSystem) {
        if (fromCommit->commitInfo[0].interruptPending) {
            interruptPending = true;
        }

        if (fromCommit->commitInfo[0].clearInterrupt) {
            interruptPending = false;
        }
    }
    //for loop
    //对每个线程都调用fetch(status_change)函数
    //fetch(status_change)
    for (threadFetched = 0; threadFetched < numFetchingThreads;
         threadFetched++) {
        // Fetch each of the actively fetching threads.
        fetch(status_change);
    }

    // Record number of instructions fetched this cycle for distribution.
    //记录每cycle fetch的instructions数量
    fetchNisnDist.sample(numInst);
    //如果status_change为true(上面的checkSignalsAndUpdate函数)
    //那么更新FetchStatus的状态，通过updateFetchStatus()获取
    if (status_change) {
        // Change the fetch stage status if there was a status change.
        _status = updateFetchStatus();
    }

    // Issue the next I-cache request if possible.
	//for loop
	//遍历所有线程，根据issuePipelinedIfetch判断是否执行pipelineIcacheAccesses
	//pipelineIcacheAccesses函数内部判断，如果buffer没有blocked，则会调用fetchCacheLine()
	//issuePipelinedIfetch,即issue instructions to fetch
	//pipelineIcacheAccesses作用是issue next i-cache
    for (ThreadID i = 0; i < numThreads; ++i) {
        if (issuePipelinedIfetch[i]) {
            pipelineIcacheAccesses(i);
        }
    }

    // Send instructions enqueued into the fetch queue to decode.
    // Limit rate by fetchWidth.  Stall if decode is stalled.
    //to decode的instructions数量
    //可用的instructions数量
    unsigned insts_to_decode = 0;
    unsigned available_insts = 0;
    //for loop
    //遍历所有active thread
    //(实际功能与英文注释不一)统计fetchQueue的所有可用instructions数量
    for (auto tid : *activeThreads) {
        if (!stalls[tid].decode) {
            available_insts += fetchQueue[tid].size();
        }
    }

    // Pick a random thread to start trying to grab instructions from
    auto tid_itr = activeThreads->begin();
    std::advance(tid_itr, random_mt.random<uint8_t>(0, activeThreads->size() - 1));
	//while循环，随机选取一个线程，遍历activeThreads
		//将fetchQueue的指令pop出来，交给toDecode，在decode stage中会进行decode
		//对应更新insts_to_decode、available_insts的值
	//将wroteToTimeBuffer设置为true，告诉cpu fetch is active in this cycle
	//cpu->activityThisCycle()的具体实现是activity.cc的activity方法
	//将instructions的number重置为0
    while (available_insts != 0 && insts_to_decode < decodeWidth) {
        ThreadID tid = *tid_itr;
        if (!stalls[tid].decode && !fetchQueue[tid].empty()) {
            auto inst = fetchQueue[tid].front();
            toDecode->insts[toDecode->size++] = inst;
            DPRINTF(Fetch, "[tid:%i][sn:%i]: Sending instruction to decode from "
                    "fetch queue. Fetch queue size: %i.\n",
                    tid, inst->seqNum, fetchQueue[tid].size());

            wroteToTimeBuffer = true;
            fetchQueue[tid].pop_front();
            insts_to_decode++;
            available_insts--;
        }

        tid_itr++;
        // Wrap around if at end of active threads list
        if (tid_itr == activeThreads->end())
            tid_itr = activeThreads->begin();
    }

    // If there was activity this cycle, inform the CPU of it.
    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();
    }

    // Reset the number of the instruction we've fetched.
    numInst = 0;
}

template <class Impl>
bool
DefaultFetch<Impl>::checkSignalsAndUpdate(ThreadID tid)
{
    // Update the per thread stall statuses.
	//从后面的time buffer里面获取decode的状态信息
	//如果某线程的decode阻塞，stalls[tid].decode = true
    if (fromDecode->decodeBlock[tid]) {
        stalls[tid].decode = true;
    }
    //如果某线程的decode非阻塞，stalls[tid].decode = false
    if (fromDecode->decodeUnblock[tid]) {
        assert(stalls[tid].decode);
        assert(!fromDecode->decodeBlock[tid]);
        stalls[tid].decode = false;
    }

    // Check squash signals from commit.
	//如果从commit stage那里得到squash signal
		//那么squash the thread and resets the PC
		//如果是branch mispredict，那么update branch predictor(squashes)
		//如果不是，那么squash sequence number
	//如果从commit stage那里得到sequence number done的signal
		//update branch predictor
	//如果从decode stage得到squash的signal
		//如果得到的decode info是branch mispredict，那么squash branch predictor
		//否则，只squash sequence number
		//如果该线程的fetch status不是squash
			//那么squash the thread，reset the PC
			//告诉cpu去squash应该被squash的instructions between fetch and decode

	//如果fetch status满足5个条件之一
		//那么设置fetch status为Blocked(这里的操作应该是为下一个判断做准备的)
	//如果fetch status的现在状态为Blocked或Squashing，那么马上切换为Running
	//最后，如果没有signal导致fetch status状态切换，那么返回false
    //PS：这里的squash操作就相当于我们流水线设计中进行的flush操作，可以参考进行设计
    if (fromCommit->commitInfo[tid].squash) {

        DPRINTF(Fetch, "[tid:%u]: Squashing instructions due to squash "
                "from commit.\n",tid);
        // In any case, squash.
        squash(fromCommit->commitInfo[tid].pc,
               fromCommit->commitInfo[tid].doneSeqNum,
               fromCommit->commitInfo[tid].squashInst, tid);

        // If it was a branch mispredict on a control instruction, update the
        // branch predictor with that instruction, otherwise just kill the
        // invalid state we generated in after sequence number
        if (fromCommit->commitInfo[tid].mispredictInst &&
            fromCommit->commitInfo[tid].mispredictInst->isControl()) {
            branchPred->squash(fromCommit->commitInfo[tid].doneSeqNum,
                              fromCommit->commitInfo[tid].pc,
                              fromCommit->commitInfo[tid].branchTaken,
                              tid);
        } else {
            branchPred->squash(fromCommit->commitInfo[tid].doneSeqNum,
                              tid);
        }

        return true;
    } else if (fromCommit->commitInfo[tid].doneSeqNum) {
        // Update the branch predictor if it wasn't a squashed instruction
        // that was broadcasted.
        branchPred->update(fromCommit->commitInfo[tid].doneSeqNum, tid);
    }

    // Check squash signals from decode.
    if (fromDecode->decodeInfo[tid].squash) {
        DPRINTF(Fetch, "[tid:%u]: Squashing instructions due to squash "
                "from decode.\n",tid);

        // Update the branch predictor.
        if (fromDecode->decodeInfo[tid].branchMispredict) {
            branchPred->squash(fromDecode->decodeInfo[tid].doneSeqNum,
                              fromDecode->decodeInfo[tid].nextPC,
                              fromDecode->decodeInfo[tid].branchTaken,
                              tid);
        } else {
            branchPred->squash(fromDecode->decodeInfo[tid].doneSeqNum,
                              tid);
        }

        if (fetchStatus[tid] != Squashing) {

            DPRINTF(Fetch, "Squashing from decode with PC = %s\n",
                fromDecode->decodeInfo[tid].nextPC);
            // Squash unless we're already squashing
            squashFromDecode(fromDecode->decodeInfo[tid].nextPC,
                             fromDecode->decodeInfo[tid].squashInst,
                             fromDecode->decodeInfo[tid].doneSeqNum,
                             tid);

            return true;
        }
    }

    if (checkStall(tid) &&
        fetchStatus[tid] != IcacheWaitResponse &&
        fetchStatus[tid] != IcacheWaitRetry &&
        fetchStatus[tid] != ItlbWait &&
        fetchStatus[tid] != QuiescePending) {
        DPRINTF(Fetch, "[tid:%i]: Setting to blocked\n",tid);

        fetchStatus[tid] = Blocked;

        return true;
    }

    if (fetchStatus[tid] == Blocked ||
        fetchStatus[tid] == Squashing) {
        // Switch status to running if fetch isn't being told to block or
        // squash this cycle.
        DPRINTF(Fetch, "[tid:%i]: Done squashing, switching to running.\n",
                tid);

        fetchStatus[tid] = Running;

        return true;
    }

    // If we've reached this point, we have not gotten any signals that
    // cause fetch to change its status.  Fetch remains the same as before.
    return false;
}

template<class Impl>
typename Impl::DynInstPtr
DefaultFetch<Impl>::buildInst(ThreadID tid, StaticInstPtr staticInst,
                              StaticInstPtr curMacroop, TheISA::PCState thisPC,
                              TheISA::PCState nextPC, bool trace)
{
    // Get a sequence number.
    InstSeqNum seq = cpu->getAndIncrementInstSeq();

    // Create a new DynInst from the instruction fetched.
    DynInstPtr instruction =
        new DynInst(staticInst, curMacroop, thisPC, nextPC, seq, cpu);
    instruction->setTid(tid);

    instruction->setASID(tid);

    instruction->setThreadState(cpu->thread[tid]);

    DPRINTF(Fetch, "[tid:%i]: Instruction PC %#x (%d) created "
            "[sn:%lli].\n", tid, thisPC.instAddr(),
            thisPC.microPC(), seq);

    DPRINTF(Fetch, "[tid:%i]: Instruction is: %s\n", tid,
            instruction->staticInst->
            disassemble(thisPC.instAddr()));

#if TRACING_ON
    if (trace) {
        instruction->traceData =
            cpu->getTracer()->getInstRecord(curTick(), cpu->tcBase(tid),
                    instruction->staticInst, thisPC, curMacroop);
    }
#else
    instruction->traceData = NULL;
#endif

    // Add instruction to the CPU's list of instructions.
    instruction->setInstListIt(cpu->addInst(instruction));

    // Write the instruction to the first slot in the queue
    // that heads to decode.
    assert(numInst < fetchWidth);
    fetchQueue[tid].push_back(instruction);
    assert(fetchQueue[tid].size() <= fetchQueueSize);
    DPRINTF(Fetch, "[tid:%i]: Fetch queue entry created (%i/%i).\n",
            tid, fetchQueue[tid].size(), fetchQueueSize);
    //toDecode->insts[toDecode->size++] = instruction;

    // Keep track of if we can take an interrupt at this boundary
    delayedCommit[tid] = instruction->isDelayedCommit();

    return instruction;
}

template<class Impl>
void
DefaultFetch<Impl>::fetch(bool &status_change)
{
    //////////////////////////////////////////
    // Start actual fetch
    //////////////////////////////////////////
	//ThreadID tid = getFetchingThread(fetchPolicy);
	//调用getFetchingThread()函数获取thread id
	//getFetchingThread()简略说作用是获取优先级最高的线程tid
	//详细说明参考具体函数注释
	//如果tid是InvalidThreadID，总线程数为1是stall
	//否则直接return，让下一个线程进行调用
    ThreadID tid = getFetchingThread(fetchPolicy);

    assert(!cpu->switchedOut());

    if (tid == InvalidThreadID) {
        // Breaks looping condition in tick()
        threadFetched = numFetchingThreads;

        if (numThreads == 1) {  // @todo Per-thread stats
            profileStall(0);
        }

        return;
    }

    DPRINTF(Fetch, "Attempting to fetch from [tid:%i]\n", tid);

    // The current PC.
    //TheISA::PCState//thisPC:tid线程对应的pc值
    //Addr:pcOffset//tid线程对应的fetch offset
    //Addr:fetchAddr//通过计算得到的fetch Address
    //bool:inRom//判断是否是rom里的micro PC
    TheISA::PCState thisPC = pc[tid];

    Addr pcOffset = fetchOffset[tid];
    Addr fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;

    bool inRom = isRomMicroPC(thisPC.microPC());

    // If returning from the delay of a cache miss, then update the status
    // to running, otherwise do the cache access.  Possibly move this up
    // to tick() function.
    //对tid的fetchStatus进行判断
    	//IcacheAccessComplete：
    		//fetchStatus切换为Running
    		//status_change设置为true
    	//Running：
    		//Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr)
    		//fetchBufferAlignPC:将fetch PC排列到fetch buffer的start位置
    		//如果fetch Buffer is invalid或者fetchAddr已经移到下一个cache block
    		//并且没有在macroop中没有剩余的ucode时
    			//执行fetchCacheLine函数，从icache获取
    			//下面是一些状态数值的计算
    		//另外如果有终端或commit instructions存在delay(actually means faults)
    	//else:idle and so on, do nothing
    if (fetchStatus[tid] == IcacheAccessComplete) {
        DPRINTF(Fetch, "[tid:%i]: Icache miss is complete.\n", tid);

        fetchStatus[tid] = Running;
        status_change = true;
    } else if (fetchStatus[tid] == Running) {
        // Align the fetch PC so its at the start of a fetch buffer segment.
        Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);

        // If buffer is no longer valid or fetchAddr has moved to point
        // to the next cache block, AND we have no remaining ucode
        // from a macro-op, then start fetch from icache.
        if (!(fetchBufferValid[tid] && fetchBufferBlockPC == fetchBufferPC[tid])
            && !inRom && !macroop[tid]) {
            DPRINTF(Fetch, "[tid:%i]: Attempting to translate and read "
                    "instruction, starting at PC %s.\n", tid, thisPC);

            fetchCacheLine(fetchAddr, tid, thisPC.instAddr());

            if (fetchStatus[tid] == IcacheWaitResponse)
                ++icacheStallCycles;
            else if (fetchStatus[tid] == ItlbWait)
                ++fetchTlbCycles;
            else
                ++fetchMiscStallCycles;
            return;
        } else if ((checkInterrupt(thisPC.instAddr()) && !delayedCommit[tid])) {
            // Stall CPU if an interrupt is posted and we're not issuing
            // an delayed commit micro-op currently (delayed commit instructions
            // are not interruptable by interrupts, only faults)
            ++fetchMiscStallCycles;
            DPRINTF(Fetch, "[tid:%i]: Fetch is stalled!\n", tid);
            return;
        }
    } else {
        if (fetchStatus[tid] == Idle) {
            ++fetchIdleCycles;
            DPRINTF(Fetch, "[tid:%i]: Fetch is idle!\n", tid);
        }

        // Status is Idle, so fetch should do nothing.
        return;
    }
    //TheISA::PCState：nextPC//值为上一个thisPC，即tid线程对应的pc值
    //StaticInstPtr：staticInst//null
    //StaticInstPtr：curMacroop//tid线程对应的macroop值
    //bool：predictedBranch//初始化false
    //bool：quiesce//被用来stop fetch，初始化为false
    //TheISA::MachInst *cacheInsts//指令从fetchBuffer[tid]中获取
    //const unsigned numInsts//通过计算得到fetchBuffer中的指令数
    //unsigned blkOffset//当前block的偏移量，用于后面判断指令执行有没有超出边界
    //
    ++fetchCycles;

    TheISA::PCState nextPC = thisPC;

    StaticInstPtr staticInst = NULL;
    StaticInstPtr curMacroop = macroop[tid];

    // If the read of the first instruction was successful, then grab the
    // instructions from the rest of the cache line and put them into the
    // queue heading to decode.

    DPRINTF(Fetch, "[tid:%i]: Adding instructions to queue to "
            "decode.\n", tid);

    // Need to keep track of whether or not a predicted branch
    // ended this fetch block.
    bool predictedBranch = false;

    // Need to halt fetch if quiesce instruction detected
    bool quiesce = false;

    TheISA::MachInst *cacheInsts =
        reinterpret_cast<TheISA::MachInst *>(fetchBuffer[tid]);

    const unsigned numInsts = fetchBufferSize / instSize;
    unsigned blkOffset = (fetchAddr - fetchBufferPC[tid]) / instSize;

    // Loop through instruction memory from the cache.
    // Keep issuing while fetchWidth is available and branch is not
    // predicted taken
    //while循环，当指令数小于fetchWidth+fetchQueue的size小于fetchQueueSize+
    //不是predicted branch+没有静默指令，执行
    	//bool：needMem//当StaticInst不在rom/不是当前macroop/ready for decoder时true
    	//重新获取fetchAddr
    	//Addr fetchBufferBlockPC//通过调用fetchBufferAlignPC()获得
    	//如果needMem，即需要内存
    		//如果fetch buffer is invalid/fetchAddr移到下一个cache block，return
    		//如果判断访问超出current block边界，则return
    		//如果ISA存在delay并且pcOffset=0
    			//当buffer里的fetchAddr和pcAddr对不上时，
    			//fetchAddr += instSize进行叠加
    		//MachInst inst//
    		//decoder[tid]->moreBytes//
    		//如果发现decoder needMoreBytes
    			//blkOffset++;
                //fetchAddr += instSize;
                //pcOffset += instSize;
    	//

    while (numInst < fetchWidth && fetchQueue[tid].size() < fetchQueueSize
           && !predictedBranch && !quiesce) {
        // We need to process more memory if we aren't going to get a
        // StaticInst from the rom, the current macroop, or what's already
        // in the decoder.
        bool needMem = !inRom && !curMacroop &&
            !decoder[tid]->instReady();
        fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;
        Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);

        if (needMem) {
            // If buffer is no longer valid or fetchAddr has moved to point
            // to the next cache block then start fetch from icache.
            if (!fetchBufferValid[tid] ||
                fetchBufferBlockPC != fetchBufferPC[tid])
                break;

            if (blkOffset >= numInsts) {
                // We need to process more memory, but we've run out of the
                // current block.
                break;
            }

            if (ISA_HAS_DELAY_SLOT && pcOffset == 0) {
                // Walk past any annulled delay slot instructions.
                Addr pcAddr = thisPC.instAddr() & BaseCPU::PCMask;
                while (fetchAddr != pcAddr && blkOffset < numInsts) {
                    blkOffset++;
                    fetchAddr += instSize;
                }
                if (blkOffset >= numInsts)
                    break;
            }

            MachInst inst = TheISA::gtoh(cacheInsts[blkOffset]);
            decoder[tid]->moreBytes(thisPC, fetchAddr, inst);

            if (decoder[tid]->needMoreBytes()) {
                blkOffset++;
                fetchAddr += instSize;
                pcOffset += instSize;
            }
        }

        // Extract as many instructions and/or microops as we can from
        // the memory we've processed so far.
        //do{}
			//如果不是current macroop且不是inRom
				//如果decoder的instructions is ready
					//staticInst//将thisPC在decoder那里解码
					//如果staticInst是宏操作(macroop)，那么curMacroop = staticInst;
					//否则，pcOffset = 0;
				//否则，直接break；
        		//bool newMacro//初始化为false
        		//如果是宏操作/inRom
        			//如果是inRom：通过调用cpu->microcodeRom.fetchMicroop获取staticInst
        			//如果是宏操作：通过curMacroop->fetchMicroop获取staticInst
        			//如果staticInst是最后一个微操作，则newMacro为true
        		//DynInstPtr instruction = buildInst()
        		//ppFetch，fetch指针对instruction进行notify
        		//numInst++
        		//调试部分，用于输出O3pipeView(然而X86版并未完全兼容，暂时未能调试出)
        		//最新的dev版似乎解决了一些问题，但是还是中途发生错误无法输出
        		//令nextPC = thisPC;
        do {
            if (!(curMacroop || inRom)) {
                if (decoder[tid]->instReady()) {
                    staticInst = decoder[tid]->decode(thisPC);

                    // Increment stat of fetched instructions.
                    ++fetchedInsts;

                    if (staticInst->isMacroop()) {
                        curMacroop = staticInst;
                    } else {
                        pcOffset = 0;
                    }
                } else {
                    // We need more bytes for this instruction so blkOffset and
                    // pcOffset will be updated
                    break;
                }
            }
            // Whether we're moving to a new macroop because we're at the
            // end of the current one, or the branch predictor incorrectly
            // thinks we are...
            bool newMacro = false;
            if (curMacroop || inRom) {
                if (inRom) {
                    staticInst = cpu->microcodeRom.fetchMicroop(
                            thisPC.microPC(), curMacroop);
                } else {
                    staticInst = curMacroop->fetchMicroop(thisPC.microPC());
                }
                newMacro |= staticInst->isLastMicroop();
            }

            DynInstPtr instruction =
                buildInst(tid, staticInst, curMacroop,
                          thisPC, nextPC, true);

            ppFetch->notify(instruction);
            numInst++;

#if TRACING_ON
            if (DTRACE(O3PipeView)) {
                instruction->fetchTick = curTick();
            }
#endif

            nextPC = thisPC;

            // If we're branching after this instruction, quit fetching
            // from the same block.
            //predictBranch根据thisPC的branching()和lookupAndUpdateNextPC()确定
            //其bool值，true，则是predict branch；false，则不是。
            //newMacro,结果取决于newMacro前面的值，另一个条件似乎没有其作用(!)
            //thisPC = nextPC;
            //inRom = isRomMicroPC(thisPC.microPC());这个两行还没有弄懂
            //如果newMacrotrue，重新计算fetchAddr、blkOffset
            	//并将pcOffset和currMacroop重置为0和NULL
            //如果instruction->isQuiesce()true，
            	//更新fetchStatus[tid]为QuiescePending
            	//更新status_change为true
            	//更新quiesce为true
        //do{}while()的while条件：
            //(curMacroop不为NULL或decoder[tid]的instruction是true)+
            //(numInst小于fetchWidth)+(fetchQueue[tid].size()小于fetchQueueSize)
            predictedBranch |= thisPC.branching();
            predictedBranch |=
                lookupAndUpdateNextPC(instruction, nextPC);
            if (predictedBranch) {
                DPRINTF(Fetch, "Branch detected with PC = %s\n", thisPC);
            }

            newMacro |= thisPC.instAddr() != nextPC.instAddr();

            // Move to the next instruction, unless we have a branch.
            thisPC = nextPC;
            inRom = isRomMicroPC(thisPC.microPC());

            if (newMacro) {
                fetchAddr = thisPC.instAddr() & BaseCPU::PCMask;
                blkOffset = (fetchAddr - fetchBufferPC[tid]) / instSize;
                pcOffset = 0;
                curMacroop = NULL;
            }

            if (instruction->isQuiesce()) {
                DPRINTF(Fetch,
                        "Quiesce instruction encountered, halting fetch!\n");
                fetchStatus[tid] = QuiescePending;
                status_change = true;
                quiesce = true;
                break;
            }
        } while ((curMacroop || decoder[tid]->instReady()) &&
                 numInst < fetchWidth &&
                 fetchQueue[tid].size() < fetchQueueSize);
    }

    if (predictedBranch) {
        DPRINTF(Fetch, "[tid:%i]: Done fetching, predicted branch "
                "instruction encountered.\n", tid);
    } else if (numInst >= fetchWidth) {
        DPRINTF(Fetch, "[tid:%i]: Done fetching, reached fetch bandwidth "
                "for this cycle.\n", tid);
    } else if (blkOffset >= fetchBufferSize) {
        DPRINTF(Fetch, "[tid:%i]: Done fetching, reached the end of the"
                "fetch buffer.\n", tid);
    }
    //macroop[tid] = curMacroop;
    //fetchOffset[tid] = pcOffset;
    //如果此时numInst>0的话，令wroteToTimeBuffer为true
    //即告诉cpu这个cycle fetch是active的
    //pc[tid] = thisPC;
    //fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;
    //Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);
    //计算得到fetchAddr和fetchBufferBlockPC
    //计算issuePipelinedIfetch[tid]的值
    //如果fetchStatus[tid]！=IcacheWaitResponse/ItlbWait/IcacheWaitRetry/QuiescePending
    //且fetchBufferBlockPC != fetchBufferPC[tid]，且curMacroop为NULL
    //那么issuePipelinedIfetch[tid]为true，否则false
    macroop[tid] = curMacroop;
    fetchOffset[tid] = pcOffset;

    if (numInst > 0) {
        wroteToTimeBuffer = true;
    }

    pc[tid] = thisPC;

    // pipeline a fetch if we're crossing a fetch buffer boundary and not in
    // a state that would preclude fetching
    fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;
    Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);
    issuePipelinedIfetch[tid] = fetchBufferBlockPC != fetchBufferPC[tid] &&
        fetchStatus[tid] != IcacheWaitResponse &&
        fetchStatus[tid] != ItlbWait &&
        fetchStatus[tid] != IcacheWaitRetry &&
        fetchStatus[tid] != QuiescePending &&
        !curMacroop;
}

template<class Impl>
void
DefaultFetch<Impl>::recvReqRetry()
{
    if (retryPkt != NULL) {
        assert(cacheBlocked);
        assert(retryTid != InvalidThreadID);
        assert(fetchStatus[retryTid] == IcacheWaitRetry);

        if (cpu->getInstPort().sendTimingReq(retryPkt)) {
            fetchStatus[retryTid] = IcacheWaitResponse;
            retryPkt = NULL;
            retryTid = InvalidThreadID;
            cacheBlocked = false;
        }
    } else {
        assert(retryTid == InvalidThreadID);
        // Access has been squashed since it was sent out.  Just clear
        // the cache being blocked.
        cacheBlocked = false;
    }
}

///////////////////////////////////////
//                                   //
//  SMT FETCH POLICY MAINTAINED HERE //
//                                   //
///////////////////////////////////////
/*
 * getFetchingThread()
 * 线程数多于1时,根据fetch_priority判断
 *    SingleThread:0
 *    RoundRobin:roundRobin()
 *    IQ:iqCount()
 *    LSQ:lsqCount()
 *    Branch:branchCount()
 *    default:InvalidThreadID
 * 线程数为1时
 * 	  活动线程数为1，且状态是Running/IcacheAccessComplete/Idle之一，返回tid
 * 	  否则，返回InvalidThreadID
 */
template<class Impl>
ThreadID
DefaultFetch<Impl>::getFetchingThread(FetchPriority &fetch_priority)
{
    if (numThreads > 1) {
        switch (fetch_priority) {

          case SingleThread:
            return 0;

          case RoundRobin:
            return roundRobin();

          case IQ:
            return iqCount();

          case LSQ:
            return lsqCount();

          case Branch:
            return branchCount();

          default:
            return InvalidThreadID;
        }
    } else {
        list<ThreadID>::iterator thread = activeThreads->begin();
        if (thread == activeThreads->end()) {
            return InvalidThreadID;
        }

        ThreadID tid = *thread;

        if (fetchStatus[tid] == Running ||
            fetchStatus[tid] == IcacheAccessComplete ||
            fetchStatus[tid] == Idle) {
            return tid;
        } else {
            return InvalidThreadID;
        }
    }
}

/*
 * roundRobin()
 * 返回优先线程队列中第一个状态为Running/IcacheAccessComplete/Idle的线程ID
 * 将返回的线程ID重新加入队尾
 * 若没有找到，则返回InvalidThreadID
 */
template<class Impl>
ThreadID
DefaultFetch<Impl>::roundRobin()
{
    list<ThreadID>::iterator pri_iter = priorityList.begin();
    list<ThreadID>::iterator end      = priorityList.end();

    ThreadID high_pri;

    while (pri_iter != end) {
        high_pri = *pri_iter;

        assert(high_pri <= numThreads);

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle) {

            priorityList.erase(pri_iter);
            priorityList.push_back(high_pri);

            return high_pri;
        }

        pri_iter++;
    }

    return InvalidThreadID;
}
/*
 *iqCount()
 *PQ//priority_queue,从小到大排列，数字(iqCount)越小，优先级越高
 *threadMap//map<unsigned,threadID>
 *while循环中，遍历线程
 *	获取iew stage中的iqCount，并存入PQ中；iqCount和tid存入threadMap中
 *下面功能同roundRobin()，即优先队列的获取过程不同
 */
template<class Impl>
ThreadID
DefaultFetch<Impl>::iqCount()
{
    //sorted from lowest->highest
    std::priority_queue<unsigned,vector<unsigned>,
                        std::greater<unsigned> > PQ;
    std::map<unsigned, ThreadID> threadMap;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        unsigned iqCount = fromIEW->iewInfo[tid].iqCount;

        //we can potentially get tid collisions if two threads
        //have the same iqCount, but this should be rare.
        PQ.push(iqCount);
        threadMap[iqCount] = tid;
    }

    while (!PQ.empty()) {
        ThreadID high_pri = threadMap[PQ.top()];

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle)
            return high_pri;
        else
            PQ.pop();

    }

    return InvalidThreadID;
}
/*
 * lsqCount()
 * 参考上方iqCount()函数，不过优先级处理为ldstqCount越小，优先级越高
 */
template<class Impl>
ThreadID
DefaultFetch<Impl>::lsqCount()
{
    //sorted from lowest->highest
    std::priority_queue<unsigned,vector<unsigned>,
                        std::greater<unsigned> > PQ;
    std::map<unsigned, ThreadID> threadMap;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        unsigned ldstqCount = fromIEW->iewInfo[tid].ldstqCount;

        //we can potentially get tid collisions if two threads
        //have the same iqCount, but this should be rare.
        PQ.push(ldstqCount);
        threadMap[ldstqCount] = tid;
    }

    while (!PQ.empty()) {
        ThreadID high_pri = threadMap[PQ.top()];

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle)
            return high_pri;
        else
            PQ.pop();
    }

    return InvalidThreadID;
}
/*
 * branchCount()
 * 存在调试部分，功能是获取第一个active thread的ID，暂时没有发现有其他作用
 * 似乎功能并没有完善，存在留白
 * 返回InvalidThreadID
 */
template<class Impl>
ThreadID
DefaultFetch<Impl>::branchCount()
{
#if 0
    list<ThreadID>::iterator thread = activeThreads->begin();
    assert(thread != activeThreads->end());
    ThreadID tid = *thread;
#endif

    panic("Branch Count Fetch policy unimplemented\n");
    return InvalidThreadID;
}

template<class Impl>
void
DefaultFetch<Impl>::pipelineIcacheAccesses(ThreadID tid)
{
    if (!issuePipelinedIfetch[tid]) {
        return;
    }

    // The next PC to access.
    TheISA::PCState thisPC = pc[tid];

    if (isRomMicroPC(thisPC.microPC())) {
        return;
    }

    Addr pcOffset = fetchOffset[tid];
    Addr fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;

    // Align the fetch PC so its at the start of a fetch buffer segment.
    Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);

    // Unless buffer already got the block, fetch it from icache.
    if (!(fetchBufferValid[tid] && fetchBufferBlockPC == fetchBufferPC[tid])) {
        DPRINTF(Fetch, "[tid:%i]: Issuing a pipelined I-cache access, "
                "starting at PC %s.\n", tid, thisPC);

        fetchCacheLine(fetchAddr, tid, thisPC.instAddr());
    }
}

template<class Impl>
void
DefaultFetch<Impl>::profileStall(ThreadID tid) {
    DPRINTF(Fetch,"There are no more threads available to fetch from.\n");

    // @todo Per-thread stats

    if (stalls[tid].drain) {
        ++fetchPendingDrainCycles;
        DPRINTF(Fetch, "Fetch is waiting for a drain!\n");
    } else if (activeThreads->empty()) {
        ++fetchNoActiveThreadStallCycles;
        DPRINTF(Fetch, "Fetch has no active thread!\n");
    } else if (fetchStatus[tid] == Blocked) {
        ++fetchBlockedCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is blocked!\n", tid);
    } else if (fetchStatus[tid] == Squashing) {
        ++fetchSquashCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is squashing!\n", tid);
    } else if (fetchStatus[tid] == IcacheWaitResponse) {
        ++icacheStallCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is waiting cache response!\n",
                tid);
    } else if (fetchStatus[tid] == ItlbWait) {
        ++fetchTlbCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is waiting ITLB walk to "
                "finish!\n", tid);
    } else if (fetchStatus[tid] == TrapPending) {
        ++fetchPendingTrapStallCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is waiting for a pending trap!\n",
                tid);
    } else if (fetchStatus[tid] == QuiescePending) {
        ++fetchPendingQuiesceStallCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is waiting for a pending quiesce "
                "instruction!\n", tid);
    } else if (fetchStatus[tid] == IcacheWaitRetry) {
        ++fetchIcacheWaitRetryStallCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is waiting for an I-cache retry!\n",
                tid);
    } else if (fetchStatus[tid] == NoGoodAddr) {
            DPRINTF(Fetch, "[tid:%i]: Fetch predicted non-executable address\n",
                    tid);
    } else {
        DPRINTF(Fetch, "[tid:%i]: Unexpected fetch stall reason (Status: %i).\n",
             tid, fetchStatus[tid]);
    }
}

#endif//__CPU_O3_FETCH_IMPL_HH__
