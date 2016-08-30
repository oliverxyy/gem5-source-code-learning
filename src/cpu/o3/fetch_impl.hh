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
/*
 * DefaultFetch<Impl>::DefaultFetch()
 * DefaultFetch的构造函数，初始化成员变量
 *
 */
template<class Impl>
DefaultFetch<Impl>::DefaultFetch(O3CPU *_cpu, DerivO3CPUParams *params)
	  /*
	   * 根据配置初始化cpu成员变量
	   *
	   * 初始化cpu,引用FullO3CPU
	   * 参数类定义于params/DerivO3CPU.hh
	   */
    : cpu(_cpu),
	  /*
	   * 根据配置初始化各stage之间的时延
	   *
	   * 根据配置初始化decodeToFetchDelay(decode到fetch的时延，下义同)
	   * 根据配置初始化renameToFetchDelay
	   * 根据配置初始化iewToFetchDelay
	   * 根据配置初始化commitToFetchDelay
	   */
      decodeToFetchDelay(params->decodeToFetchDelay),
      renameToFetchDelay(params->renameToFetchDelay),
      iewToFetchDelay(params->iewToFetchDelay),
      commitToFetchDelay(params->commitToFetchDelay),
	  /*
	   * 根据配置初始化fetch和decode的最大指令数
	   *
	   * 根据配置初始化fetchWidth(fetch的最大指令数，下义同)
	   * 根据配置初始化decodeWidth
	   */
      fetchWidth(params->fetchWidth),
      decodeWidth(params->decodeWidth),
	  /*
	   * 初始化retryPkt(要被重新发送的数据包)
	   *
	   * 初始化retryPkt，值为NULL
	   * 类型为Packet指针
	   * Packet是用于存储系统(e.g.L1,L2 cache)之间交换被封装好的数据包
	   * Packet类位于mem/packet.hh中
	   */
      retryPkt(NULL),
	  /*
	   * 初始化retryTid(cache中等待通知fetch retry的线程ID)
	   *
	   * 初始化retryTid，InvalidThreadID值为-1
	   * 类型为int16_t，定义于fetch.hh和base/types.hh
	   * cache中等待告知fetch retry的线程ID
	   */
      retryTid(InvalidThreadID),
	  /*
	   * 根据配置初始化cacheBlkSize
	   *
	   * 初始化cacheBlkSize，值是System类的cache line size，位于sim/system.hh|system.cc
	   * 结合FullO3CPU类的构造函数来看，System初始化时会从参数中获取cache_line_size
	   */
      cacheBlkSize(cpu->cacheLineSize()),
	  /*
	   * 根据配置初始化fetchBufferSize、fetchBufferMask、fetchQueueSize、
	   * numThreads和numFetchingThreads
	   *
	   * 根据配置初始化fetchBufferSize(fetch buffer的大小，可能会比cache line小,单位byte)
	   * 根据配置初始化fetchBufferMask(将fetch address修正到fetch buffer边界内)
	   * 根据配置初始化fetchQueueSize(存储微操作的fetch queue大小)
	   * 根据配置初始化numThreads(fetch线程的数量)
	   * 根据配置初始化numFetchingThreads(正在进行fetching操作的线程)
	   */
      fetchBufferSize(params->fetchBufferSize),
      fetchBufferMask(fetchBufferSize - 1),
      fetchQueueSize(params->fetchQueueSize),
      numThreads(params->numThreads),
      numFetchingThreads(params->smtNumFetchingThreads),
	  /*
	   * 初始化finishTranslationEvent
	   *
	   * 用来延迟translation faults生成的事件
	   * 定义和英文注释参见cpu/o3/fetch.hh的FinishTranslationEvent类
	   */
      finishTranslationEvent(this)
{
	/*
	 * 检测运行配置参数，输出错误并退出
	 *
	 * MaxThreads为4，配置位于cpu/o3/impl.hh
	 * 如果numThreads>MaxThreads,输出错误信息并退出(定义位于base/misc.hh)
	 * 如果fetchWidth>MaxWidth,输出错误信息并退出
	 * 如果fetchBufferSize>cacheBlkSize,输出错误信息并退出
	 * 如果cacheBlkSize>fetchBufferSize,输出错误信息并退出
	 */
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
    /*
     * 根据配置初始化fetch的fetchPolicy
     *
     * 获取配置中的smtFetchPolicy
     * 将smtFetchPolicy中的大写字母全部转化为小写字母
     * 根据字符串的值初始化fetchPolicy
     * (SingleThread/roundrobin/Branch/iqcount/lsqcount)
     */
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
    /*
     * 根据配置初始化instSize
     * 初始化所有线程中decoder、fetchBuffer、fetchBufferPC、fetchBufferValid
     *
     * instSize是fetch中的instruction的大小
     * decoder定义于arch/X86/decoder.hh
     * fetchBufferPC的值是fetch buffer中第一个指令的address
     * fetchBufferValid用来标记fetch buffer中值是否有效
     */
    instSize = sizeof(TheISA::MachInst);

    for (int i = 0; i < Impl::MaxThreads; i++) {
        decoder[i] = NULL;
        fetchBuffer[i] = NULL;
        fetchBufferPC[i] = 0;
        fetchBufferValid[i] = false;
    }
    /*
     * 根据配置初始化branchPred(branch predictor)
     * 实例化单核线程中所有的decoder
     * 为fetchBuffer按照配置分配空间
     *
     * BPredUnit类定义于cpu/pred/bpred_unit.hh
     */
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

template <class Impl>
bool
DefaultFetch<Impl>::fetchCacheLine(Addr vaddr, ThreadID tid, Addr pc)
{
	/*
	 * 初始化fault为NoFault
	 * 断言cpu不是switch out的状态
	 * 如果cacheBlocked为true，即cache阻塞
	 * 	  直接return false；
	 * 如果是中断并且没有延迟提交(!delayedCommit[tid]==true)
	 *    直接return false；
	 */
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
    /*
     * fetchBufferBlockPC为vaddr对齐掩码之后的地址
     * 实例化一个指向Request的mem_req指针
     * 并给mem_req的赋值taskId
     * 令memReq[tid]=mem_req
     * 令fetchStatus[tid]=ItlbWait
     * 实例化FetchTranslation类
     * 调用translateTiming方法
     * return true
     *
     */
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
	/*
	 * 从mem_req中取出之前的thread id
	 * 获取fetchBufferBlockPC
	 * 断言cpu不是switch out状态
	 * 唤醒cpu
	 *
	 */
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
	//获取activeThreads的起始和结尾地址
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        //遍历activeThreads中的线程
        //如果fetchStatus(thread的FetchStatus)为Running、Squashing、IcacheAccessComplete之一
        	//如果_status(真正的stage status)为Inactive
        		//那么调用activateStage(更新stageActive[stageIdx]为true)
        	//return Active
        //如果_status为Active，那么调用cpu的deactivateStage方法
        //方法定义于cpu/activity.hh,更新stageActive[stageIdx]为false
        //return Inactive
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

/*
 * fetch stage的时间点计时函数
 */
template <class Impl>
void
DefaultFetch<Impl>::tick()
{
	/*
	 * 获取activeThreads中线程的起始位置
	 * 声明并初始化status_change为false
	 * 初始化wroteToTimeBuffer为false
	 *
	 * activeThreads：活动线程
	 * status_change：用于标记fetch thread的状态改变与否
	 * wroteToTimeBuffer：标记此cycle内的fetch是否将指令输出到toDecode中
	 * toDecode为fetch和decode交换数据的timeBuffer
	 * 用来告知CPU这个cycle内是否activity
	 */
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();
    bool status_change = false;
    wroteToTimeBuffer = false;
    /*
     * 初始化所有单核线程的issuePipelinedIfetch
     *
     * 将issuePipelinedIfetch的值全部初始化为false
     * 标记流水线中的I-cache request是否应该被issued(true:issued)
     */
    for (ThreadID i = 0; i < numThreads; ++i) {
        issuePipelinedIfetch[i] = false;
    }
    //updated_status//
    /*
     * 通过检测所有活动线程中的各TimeBuffer类型的变量状态来更新status_change
     *
     * TimeBuffer：记录了和其他stage的交换信息(signal)
     * 通过checkSignalsAndUpdate函数来更新status_change的状态
     * TimeBuffer存储TimeBufStruct的变量，TimeBufStruct类定义于cpu/o3/comm.hh
     */
    while (threads != end) {
        ThreadID tid = *threads++;

        // Check the signals for each thread to determine the proper status
        // for each thread.
        bool updated_status = checkSignalsAndUpdate(tid);
        status_change =  status_change || updated_status;
    }
    //记录fetch的运行信息
    DPRINTF(Fetch, "Running stage.\n");
    /*
     * 初始化FS模式下的interruptPending变量
     *
     * 从fromCommit这个timeBuffer中获取interruptPending值
     * interruptPending:用于标记是否需要中断挂起一次
     */
    if (FullSystem) {
        if (fromCommit->commitInfo[0].interruptPending) {
        	interruptPending = true;
        }

        if (fromCommit->commitInfo[0].clearInterrupt) {
            interruptPending = false;
        }
    }
    /*
     * 让所有活跃的fetch线程都开始调用fetch(bool)方法
     *
     * numFetchingThreads:活跃的fetching线程数
     *
     */
    for (threadFetched = 0; threadFetched < numFetchingThreads;
         threadFetched++) {
        // Fetch each of the actively fetching threads.
        fetch(status_change);
    }

    // Record number of instructions fetched this cycle for distribution.
    //记录每cycle fetched的instructions数量
    fetchNisnDist.sample(numInst);
    /*
     * 判断并更新fetch stage的状态
     *
     * 如果status_change为true(通过上面checkSignalsAndUpdate函数更新)
     * 那么通过updateFetchStatus()更新_status的值
     */
    if (status_change) {
        // Change the fetch stage status if there was a status change.
        _status = updateFetchStatus();
    }

    // Issue the next I-cache request if possible.
    /*
     * 遍历所有fetch线程，判断需不需要访问I-cache
     *
     * issuePipelinedIfetch,用来标记fetch线程需要不需要被分配I-cache request
     * pipelineIcacheAccesses作用是
     * pipelineIcacheAccesses函数：如果buffer没有blocked，则会调用fetchCacheLine()
     */
    for (ThreadID i = 0; i < numThreads; ++i) {
        if (issuePipelinedIfetch[i]) {
            pipelineIcacheAccesses(i);
        }
    }

    // Send instructions enqueued into the fetch queue to decode.
    // Limit rate by fetchWidth.  Stall if decode is stalled.
    /*
     * 初始化insts_to_decode和available_insts为0
     * 遍历所有活动线程，统计未decode的指令数量
     *
     * 初始化fetchQueue中需要decode的instructions数量为0
     * 初始化fetchQueue中还未分派去decode的instructions数量为0
     * 遍历所有active threads，统计fetchQueue的所有未decode的instructions数量
     */
    unsigned insts_to_decode = 0;
    unsigned available_insts = 0;
    for (auto tid : *activeThreads) {
        if (!stalls[tid].decode) {
            available_insts += fetchQueue[tid].size();
        }
    }

    // Pick a random thread to start trying to grab instructions from
    /*
     * 随机选取一个线程，按照次序遍历整个activeThreads
     * 如果没有fetch线程发现decode出现stall且线程中的fetchQueue不为空
     * 那么将fetchQueue一条条pop出来，加入到toDecode这个timeBuffer中去decode
     *
     * 随机选取一个线程地址，开始通过while循环遍历所有activeThreads
     * advance方法是STL迭代器方法，作用是将迭代器iterator移动n位(这里的处理n随机)
     *    如果没有fetch线程发现decode出现stall且线程中的fetchQueue不为空
     *       获取此线程中的fetchQueue的第一个指令inst
     *       记录运行信息(用到了inst，所以先用临时变量存储，后面再pop出来)
     *       将wroteToTimeBuffer设置为true，
     *       即表示此cycle的fetch已经将指令输出到toDecode这个timeBuffer中了
     *       将此线程的第一个指令pop出来
     *       令insts_to_decode+1，available_insts-1
     */
    auto tid_itr = activeThreads->begin();
    std::advance(tid_itr, random_mt.random<uint8_t>(0, activeThreads->size() - 1));
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
    //如果wroteToTimeBuffer为true
    //	 记录运行信息(这个cycle有activity)
    //   通知cpu调用activityThisCycle
    //activityThisCycle位于cpu/activity.cc
    //主要做了一件事情：++activityCount并记录运行信息下来/cycle
    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();
    }

    // Reset the number of the instruction we've fetched.
    //重设numInst(标记本cycle内fetched的指令数)为0
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
/*
 * DefaultFetch<Impl>::fetch
 * 开始fetch指令
 *
 */
template<class Impl>
void
DefaultFetch<Impl>::fetch(bool &status_change)
{
    //////////////////////////////////////////
    // Start actual fetch
    //////////////////////////////////////////
	/*
	 * 通过getFetchingThread选取优先级最高的线程ID tid
	 * 断言cpu状态不是switchedOut
	 * 如果tid==InvalidThreadID
	 * 	  令threadFetched=活跃的fetching线程数
	 * 	  如果配置的numThreads==1，stall
	 * 	  然后直接执行return操作，fetch(bool)方法结束
	 *
	 * fetchPolicy从构造函数处已经初始化
	 * 通过调用getFetchingThread获取优先级最高的线程ID
	 * 如果是意外情况会返回InvalidThreadID，
	 * 具体参见getFetchingThread方法的注释
	 *
	 */
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
	/*
	 * 记录fetch运行信息
	 * 获取指令的在PC中的地址
	 * 计算得到fetchAddr
	 * 定义并初始化inRom
	 *
	 *
	 * 令thisPC=当前线程pc寄存器中的pc地址
	 * 令pcOffset=当前线程的fetchOffset,默认值为0
	 * PCMask为64位地址，除后三位为0，其余位上全为1
	 * 通过计算得到的fetchAddr,前面61位不变，最后三位修改为0
	 * 通过isRomMicroPC方法获取inRom的值，标记指令是否在Rom里面
	 */
    DPRINTF(Fetch, "Attempting to fetch from [tid:%i]\n", tid);
    // The current PC.
    TheISA::PCState thisPC = pc[tid];

    Addr pcOffset = fetchOffset[tid];
    Addr fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;

    bool inRom = isRomMicroPC(thisPC.microPC());

    // If returning from the delay of a cache miss, then update the status
    // to running, otherwise do the cache access.  Possibly move this up
    // to tick() function.
    /*
     * ThreadStatus定义于cpu/o3/fetch.hh中
     * 如果当前线程的状态为IcacheAccessComplete
     * 	  记录Icache已经可以访问的运行信息
     * 	  将当前线程状态修正为Running
     * 	  将status_change设置为true(前面提到过，标记当前线程状态改变与否的变量)
     * 如果当前线程状态为Running
     *    计算得到fetchBufferBlockPC
     *    如果fetchBufferValid不valid或fetchBufferBlockPC!=fetchBufferPC
     *    且指令不在Rom里且不是宏操作
     *       那么记录要对instruction进行translate和read的操作的运行信息
     *       执行fetchCacheLine函数
     *       如果当前线程的状态为IcacheWaitResponse，那么++icacheStallCycles
     *       如果当前线程的状态为ItlbWait，那么++fetchTlbCycles
     *       否则++fetchMiscStallCycles
     *       之后直接return，结束fetch(bool)方法
     *    如果检查出中断信号，且当前线程的delayedCommitfalse
     *    	 ++fetchMiscStallCycles;
     *    	 记录当前线程的fetch进程stalled的运行信息
     *    	 直接return，结束fetch(bool)方法
     * 否则(剩余几种情况，如Idle、Squashing and so on)
     * 	  如果当前线程的fetchStatus为Idle
     * 	  	 ++fetchIdleCycles;
     * 	  	 记录当前线程处于空闲状态的运行信息
     * 	  然后直接return，结束fetch(bool)方法
     *
     */
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
    /*
     * ++fetchCycles
     * 令nextPC = thisPC
     * 定义并初始化staticInst为NULL
     * 定义并初始化curMacroop为当前线程的宏操作
     * 记录fetch将指令丢入队列中去decode的运行信息
     * 定义并初始化predictedBranch为false
     * 定义并初始化quiesce为false
     * 定义并初始化cacheInsts指向本线程的fetchBuffer
     * 通过计算得到numInsts和blkOffset
     *
     * staticInst：记录了宏指令的信息
     * curMacroop：x86宏指令
     * predictedBranch：标记指令是否是predicted Branch
     * quiesce：静默指令标记，用于暂停fetch操作
     * cacheInsts:从fetchBuffer中读出的数据(uint64_t)
     * numInsts：fetchBuffer中的指令数
     * blkOffset：几个block的偏移量，一个block理论上对应一条指令内容
     * Haswell中每次传输16Bytes数据到fetchBuffer
     * fetchBuffer每次最多输出6条instructions
     * gem5这里设计理念相同，具体数值待探究
     */
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
    /*
     * 当指令数小于fetchWidth且fetchQueue的size小于fetchQueueSize且
     * 不是predicted branch且没有静默标志，执行while循环
     */
    while (numInst < fetchWidth && fetchQueue[tid].size() < fetchQueueSize
           && !predictedBranch && !quiesce) {
        // We need to process more memory if we aren't going to get a
        // StaticInst from the rom, the current macroop, or what's already
        // in the decoder.
    	/*
    	 * 定义并初始化needMem
    	 * 计算对齐后的fetchAddr
    	 * 定义并初始化fetchBufferBlockPC
    	 *
    	 * 当inRom为false且curMacroop为NULL
    	 * 且decoder[tid]没有生成可执行机器指令时needMem为true
    	 * fetchBufferBlockPC
    	 * (当已生成可执行的机器指令时，instReady返回true)
    	 */
        bool needMem = !inRom && !curMacroop &&
            !decoder[tid]->instReady();
        fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;
        Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);
        /*
         * 如果needMem为true
         * 	  如果fetchBufferValid[tid]为false
         * 	  或fetchBufferBlockPC != fetchBufferPC[tid]
         * 	  	 执行break操作，跳出while循环
         * 	  如果blkOffset>=numInsts
         * 	     执行break操作，跳出while循环
         * 	  X86中没有ISA_HAS_DELAY_SLOT，略过此if、
         *
         * 	  获取cacheInsts中的指令inst的内容
         * 	  执行decoder[tid]->moreBytes对inst进行predecode
         *    如果decoder[tid]中的predecode完成
         *       将各种变量的标志位对应移至到下一指令(blkOffset等)
         *
         * 当fetchBufferPC[tid]位于fetchBuffer的边界之外
         * 即fetchBufferPC地址越界时，计算得到的指令范围变大，
         * 从而会使blkOffset >= numInsts
         * 此时，指令地址不正确，跳出while循环
         *
         * inst是cacheInsts中的内容，8Bytes
         * fetchBufferPC初始化为0
         * 根据推理只有当fetchAddr同样初始为0，即pc地址从0开始计数才能满足逻辑
         * moreBytes方法中会对inst进行predecode
         *
         * blkOffset++:cacheInsts中下一条指令
         * fetchAddr：将fetchAddr地址移至下一条指令的位置
         * pcOffset：记录pc中的offset偏移量
         */
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
            //X86略过(X86推理下去逻辑会有问题！)
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
        do {
        	/*
        	 * 如果curMacroop为NULL且inRom为false时
        	 *    如果decoder[tid]已经生成可执行机器指令
        	 *    	 获取staticInst和curMacroop，更新fetchedInsts、pcOffset
        	 *       staticInst为decoder进行decode操作后返回的StaticInstPtr
        	 *       ++fetchedInsts(记录fetched指令总数)
        	 *       如果staticInst是宏指令操作
        	 *          那么令curMacroop=staticInst
        	 *       否则令pcOffset = 0
        	 *    否则，直接return，跳出while循环
        	 *
        	 * staticInst为译码得到的指令分类信息
        	 * 参照cpu/static_inst.hh分析
        	 */
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
            /*
             * 定义并初始化newMacro为false
             * 如果获取到了curMacroop或inRom为true
             *    如果inRom为true
             *    	 那么通过microcodeRom的fetchMicroop获取staticInst
             *    否则
             *       staticInst直接通过curMacroop的fetchMicroop取出
             *    如果通过staticInst判断是last microop，那么newMacro为true
             *
             *
             */
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
            /*
             * 定义并初始化instruction
             * ppFetch用于监听探测instruction的状态
             * numInst++
             * 下面是O3PipeView的调试代码
             * 令nextPC = thisPC
             *
             * DynInst定义于cpu/o3/dyn_inst_impl.hh中
             * 比较复杂
             * ProbePointArg定义于sim/probe/probe.hh中
             */
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
            /*
             * 判断当前指令是否是预测分支(predictedBranch true or not)
             * 同时更新nextPC的值
             * 如果是预测分支，则记录下运行信息
             *
             * predictBranch根据thisPC的branching()和lookupAndUpdateNextPC()确定
             * 其bool值，true，则是predict branch；false，则不是。
             * lookupAndUpdateNextPC函数中调用太多了，待研究
             */
            predictedBranch |= thisPC.branching();
            predictedBranch |=
                lookupAndUpdateNextPC(instruction, nextPC);
            if (predictedBranch) {
                DPRINTF(Fetch, "Branch detected with PC = %s\n", thisPC);
            }
            /*
             * 如果还存在nextPC，那么newMacro为true
             * 即还有新的宏指令
             * 令thisPC = nextPC(nextPC已经是下一条指令了)
             * 更新inRom的值(判断指令在不在Rom)
             */
            newMacro |= thisPC.instAddr() != nextPC.instAddr();

            // Move to the next instruction, unless we have a branch.
            thisPC = nextPC;
            inRom = isRomMicroPC(thisPC.microPC());
            /*
             * 如果newMacro为true
             * 更新fetchAddr、blkOffset、pcOffset、curMacroop
             * 为下一循环译码做准备
             *
             * 如果newMacro为true，重新计算fetchAddr、blkOffset
             * 并将pcOffset和currMacroop重置为0和NULL
             */
            if (newMacro) {
                fetchAddr = thisPC.instAddr() & BaseCPU::PCMask;
                blkOffset = (fetchAddr - fetchBufferPC[tid]) / instSize;
                pcOffset = 0;
                curMacroop = NULL;
            }
            /*
             * 如果instruction被判断是静默指令
             * 暂停fetch操作，更新fetchStatus、status_change、quiesce
             * 然后直接break出此do while循环
             *
             * 如果instruction->isQuiesce()true
             * 更新fetchStatus[tid]为QuiescePending
             * 更新status_change为true
             * 更新quiesce为true
             */
            if (instruction->isQuiesce()) {
                DPRINTF(Fetch,
                        "Quiesce instruction encountered, halting fetch!\n");
                fetchStatus[tid] = QuiescePending;
                status_change = true;
                quiesce = true;
                break;
            }
            //do while 循环条件
        } while ((curMacroop || decoder[tid]->instReady()) &&
                 numInst < fetchWidth &&
                 fetchQueue[tid].size() < fetchQueueSize);
    }
    //判断predictedBranch、numInst、blkOffset情况
    //记录运行信息
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
    /*
     * 令macroop[tid]=curMacroop
     * 更新macroop[tid]、fetchOffset[tid]、
     * wroteToTimeBuffer、fetchAddr、fetchBufferBlockPC
     * 以及issuePipelinedIfetch[tid]，为下一cycle fetch做准备
     *
     */
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
