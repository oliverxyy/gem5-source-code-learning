/*
Copyright (c) 2011-2012, 2014 ARM Limited
Copyright (c) 2013 Advanced Micro Devices, Inc.
All rights reserved
 *
The license below extends only to copyright in the software and shall
not be construed as granting a license to any other intellectual
property including but not limited to intellectual property relating
to a hardware implementation of the functionality of the software
licensed hereunder.  You may use the software subject to the license
terms below provided that you ensure that this notice is replicated
unmodified and in its entirety in all distributions of the software,
modified or unmodified, in source code or in binary form.
 *
Copyright (c) 2004-2006 The Regents of The University of Michigan
Copyright (c) 2011 Regents of the University of California
All rights reserved.
 *
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met: redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer;
redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution;
neither the name of the copyright holders nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.
 *
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
Authors: Kevin Lim
         Korey Sewell
         Rick Strong
 */

#include "arch/kernel_stats.hh"
#include "config/the_isa.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/checker/thread_context.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/isa_specific.hh"
#include "cpu/o3/thread_context.hh"
#include "cpu/activity.hh"
#include "cpu/quiesce_event.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "debug/Activity.hh"
#include "debug/Drain.hh"
#include "debug/O3CPU.hh"
#include "debug/Quiesce.hh"
#include "enums/MemoryMode.hh"
#include "sim/core.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/stat_control.hh"
#include "sim/system.hh"

#if THE_ISA == ALPHA_ISA
#include "arch/alpha/osfpal.hh"
#include "debug/Activity.hh"
#endif

struct BaseCPUParams;

using namespace TheISA;
using namespace std;

BaseO3CPU::BaseO3CPU(BaseCPUParams *params)
    : BaseCPU(params)
{
}

void
BaseO3CPU::regStats()
{
    BaseCPU::regStats();
}

template<class Impl>
bool
FullO3CPU<Impl>::IcachePort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(O3CPU, "Fetch unit received timing\n");
    // We shouldn't ever get a cacheable block in ownership state
    assert(pkt->req->isUncacheable() ||
           !(pkt->memInhibitAsserted() && !pkt->sharedAsserted()));
    fetch->processCacheCompletion(pkt);

    return true;
}

template<class Impl>
void
FullO3CPU<Impl>::IcachePort::recvReqRetry()
{
    fetch->recvReqRetry();
}

template <class Impl>
bool
FullO3CPU<Impl>::DcachePort::recvTimingResp(PacketPtr pkt)
{
    return lsq->recvTimingResp(pkt);
}

template <class Impl>
void
FullO3CPU<Impl>::DcachePort::recvTimingSnoopReq(PacketPtr pkt)
{
    // X86 ISA: Snooping an invalidation for monitor/mwait
    if(cpu->getCpuAddrMonitor()->doMonitor(pkt)) {
        cpu->wakeup();
    }
    lsq->recvTimingSnoopReq(pkt);
}

template <class Impl>
void
FullO3CPU<Impl>::DcachePort::recvReqRetry()
{
    lsq->recvReqRetry();
}

template <class Impl>
FullO3CPU<Impl>::TickEvent::TickEvent(FullO3CPU<Impl> *c)
    : Event(CPU_Tick_Pri), cpu(c)
{
}

template <class Impl>
void
FullO3CPU<Impl>::TickEvent::process()
{
    cpu->tick();
}

template <class Impl>
const char *
FullO3CPU<Impl>::TickEvent::description() const
{
    return "FullO3CPU tick";
}
/*
 * FullO3CPU()
 * FullO3CPU(FullSystem out of order CPU core simulation)
 * FS模式的乱序CPU core
 *
 * 模板类Impl实际类型为O3CPUImpl，定义于cpu/o3/impl.hh，
 * 里面配置了maxThreads=4，maxWidth=8，应用于整个o3大模块
 *
 */
template <class Impl>
FullO3CPU<Impl>::FullO3CPU(DerivO3CPUParams *params)
/*
 * 根据配置初始化BaseO3CPU、itb、dtb、tickEvent
 * debug模式会执行的调试代码
 * 初始化removeInstsThisCycle为false
 *
 * params是模拟器的配置参数列表
 * DerivO3CPUParams定义位于params/DerivO3CPUParams.hh
 * 根据配置初始化BaseO3CPU
 * 根据配置初始化itb(定义与实现位于arch/X86/tlb.hh|tlb.cc)
 * 根据配置初始化dtb(同上，都是TLB类)
 * 初始化tickEvent,参数为this
 * 如果是非debug模式(即release)，则不声明定义instcount；
 * 如果是debug模式，则声明定义instcount，初始化为0
 * removeInstsThisCycle用于标记这个cycle的指令需不需要被remove
 * (如果是retired和squashed两种状态的话)参考自cpu/o3/cpu.hh英文注解
 *
 */
    : BaseO3CPU(params),
      itb(params->itb),
      dtb(params->dtb),
      tickEvent(this),
#ifndef NDEBUG
      instcount(0),
#endif
      removeInstsThisCycle(false),
	  /*
	   * 初始化fetch、decode、rename、iew、commit并加载基本配置信息
	   *
	   * 初始化fetch，并加载基本配置信息
	   * 初始化decode，并加载基本配置信息
	   * 初始化rename，并加载基本配置信息
	   * 初始化iew，并加载基本配置信息
	   * 初始化commit，并加载基本配置信息
	   */
      fetch(this, params),
      decode(this, params),
      rename(this, params),
      iew(this, params),
      commit(this, params),
	  /*
	   * 初始化regFile、reg的freeList、rob和scoreboard
	   *
	   * 根据配置初始化regFile(初始化register file成员变量),定义于cpu/o3/regfile.cc
	   * 根据配置初始化freeList,定义于cpu/o3/free_list.hh|free_list.cc
	   * 实际上调用的是cpu/o3/regfile.cc的initFreeList，初始化register free list
	   * 根据配置初始化rob(re-order buffer)，定义于cpu/o3/rob.hh|rob_impl.hh
	   * 根据配置初始化scoreboard，定义于cpu/o3/scoreboard.hh|scoreboard.cc
	   * 记录各种register的状态参数，比如数量，ready，总量，index等等
	   */
      regFile(params->numPhysIntRegs,
              params->numPhysFloatRegs,
              params->numPhysCCRegs),
      freeList(name() + ".freelist", &regFile),
      rob(this, params),
      scoreboard(name() + ".scoreboard",
                 regFile.totalNumPhysRegs(), TheISA::NumMiscRegs,
                 TheISA::ZeroReg, TheISA::ZeroReg),
	  /*
	   * 初始化ISA类、icachePort和dcachePort
	   *
	   * numThreads为单核可运行线程数
	   * 根据配置初始化numThreads个ISA为NULL，定义于arch/X86/isa.hh|isa.cc
	   * 根据配置初始化IcachePort(for fetching instruction)，定义于cpu/o3/cpu.hh
	   * 根据配置初始化dcachePort(for load/store queue),定义于cpu/o3/cpu.hh
	   */
      isa(numThreads, NULL),
      icachePort(&fetch, this),
      dcachePort(&iew.ldstQueue, this),
	  /*
	   * 根据配置初始化timeBuffer、fetchQueue、decodeQueue、renameQueue、iewueue
	   *
	   * 根据配置初始化timeBuffer(前后stage交流用的),分配空间，定义于cpu/timebuf.hh
	   * 根据配置初始化fetchQueue(fetch stage的IQ),分配空间，类型是TimeBuffer
	   * 根据配置初始化decodeQueue(fetch stage的IQ),分配空间，类型是TimeBuffer
	   * 根据配置初始化renameQueue(fetch stage的IQ),分配空间，类型是TimeBuffer
	   * 根据配置初始化iewueue(fetch stage的IQ),分配空间，类型是TimeBuffer
	   */
      timeBuffer(params->backComSize, params->forwardComSize),
      fetchQueue(params->backComSize, params->forwardComSize),
      decodeQueue(params->backComSize, params->forwardComSize),
      renameQueue(params->backComSize, params->forwardComSize),
      iewQueue(params->backComSize, params->forwardComSize),
	  /*
	   * 根据配置初始化activityRec、globalSeqNum、system、drainManager、lastRunningCycle
	   * 根据配置初始化activityRec，定义于cpu/activity.hh|activity.cc
	   * 根据配置初始化activityRec，定义于cpu/activity.hh|activity.cc功能主要是通知cpu activities的状态
	   * 初始化globalSeqNum为1，globalSeqNum是instruction全局计数器
	   * 根据配置初始化system(模拟的系统)，传入基本配置参数，定义位于sim/system.hh|system.cc
	   * 初始化drainManager(track所有currently draining的simObject)为NULL
	   * 初始化lastRunningCycle(记录上一个运行的Cycle数)为当前Cycle数
	   */
      activityRec(name(), NumStages,
                  params->backComSize + params->forwardComSize,
                  params->activity),
      globalSeqNum(1),
      system(params->system),
      drainManager(NULL),
      lastRunningCycle(curCycle())
{
	/*
	 * 根据配置初始化_status、checker、thread和tid内线程数
	 *
	 * 如果params参数的switched_out为false，那么_status(CPU的状态标识符)为Running
	 * 否则，_status=SwitchedOut
	 * 如果params参数中checker不为NULL，那么实例化Checker(作用类比Oracle/Check)
	 * 传入IcachePort和System参数
	 * 否则，checker为NULL
	 * 如果FullSystem为false，即非FullSystem模式运行，则改变thread和tids的大小为numThreads
	 * FS模式暂时不支持超线程，SE模式支持，所以这里resize cpu中线程的数量
	 */
    if (!params->switched_out) {
        _status = Running;
    } else {
        _status = SwitchedOut;
    }

    if (params->checker) {
        BaseCPU *temp_checker = params->checker;
        checker = dynamic_cast<Checker<Impl> *>(temp_checker);
        checker->setIcachePort(&icachePort);
        checker->setSystem(params->system);
    } else {
        checker = NULL;
    }

    if (!FullSystem) {
        thread.resize(numThreads);
        tids.resize(numThreads);
    }

    // The stages also need their CPU pointer setup.  However this
    // must be done at the upper level CPU because they have pointers
    // to the upper level CPU, and not this FullO3CPU.

    // Set up Pointers to the activeThreads list for each stage
    /*
     * 根据配置初始化fetch等stage的activeThreads成员变量
     *
     * 给fetch stage传入activeThreads list引用
     * 给decode stage传入activeThreads list引用
     * 给rename stage传入activeThreads list引用
     * 给iew stage传入activeThreads list引用
     * 给commit stage传入activeThreads list引用
     */
    fetch.setActiveThreads(&activeThreads);
    decode.setActiveThreads(&activeThreads);
    rename.setActiveThreads(&activeThreads);
    iew.setActiveThreads(&activeThreads);
    commit.setActiveThreads(&activeThreads);

    // Give each of the stages the time buffer they will use.
    /*
     * 初始化fetch等stage的timeBuffer
     *
     * 给fetch stage传入timeBuffer引用
     * 给decode stage传入timeBuffer引用
     * 给rename stage传入timeBuffer引用
     * 给iew stage传入timeBuffer引用
     * 给commit stage传入timeBuffer引用
     */
    fetch.setTimeBuffer(&timeBuffer);
    decode.setTimeBuffer(&timeBuffer);
    rename.setTimeBuffer(&timeBuffer);
    iew.setTimeBuffer(&timeBuffer);
    commit.setTimeBuffer(&timeBuffer);

    // Also setup each of the stages' queues.
    /*
     * 初始化fetch等stage的各种Queue
     *
     * 给fetch stage传入fetchQueue引用
     * 给decode stage传入fetchQueue引用
     * 给commit stage传入fetchQueue引用
     * 给decode stage传入decodeQueue引用
     * 给rename stage传入decodeQueue引用
     * 给rename stage传入renameQueue引用
     * 给iew stage传入renameQueue引用
     * 给iew stage传入iewQueue引用
     * 给commit stage传入iewQueue引用
     * 给commit stage传入renameQueue引用
     */
    fetch.setFetchQueue(&fetchQueue);
    decode.setFetchQueue(&fetchQueue);
    commit.setFetchQueue(&fetchQueue);
    decode.setDecodeQueue(&decodeQueue);
    rename.setDecodeQueue(&decodeQueue);
    rename.setRenameQueue(&renameQueue);
    iew.setRenameQueue(&renameQueue);
    iew.setIEWQueue(&iewQueue);
    commit.setIEWQueue(&iewQueue);
    commit.setRenameQueue(&renameQueue);
    /*
     * 初始化commit和rename的iew和commit成员变量
     *
     * 给commit stage传入iew引用
     * 给rename stage传入iew引用
     * 给rename stage传入commit引用
     */
    commit.setIEWStage(&iew);
    rename.setIEWStage(&iew);
    rename.setCommitStage(&commit);
    /*
     * 根据模拟模式初始化active_threads
     *
     * 定义类型为ThreadID的active_threads
     * 如果是FullSystem,那么active_threads=1
     * 否则active_threads = params->workload.size()
     *    且如果active_threads>MaxThreads时，输出错误信息并退出
     */
    ThreadID active_threads;
    if (FullSystem) {
        active_threads = 1;
    } else {
        active_threads = params->workload.size();

        if (active_threads > Impl::MaxThreads) {
            panic("Workload Size too large. Increase the 'MaxThreads' "
                  "constant in your O3CPU impl. file (e.g. o3/alpha/impl.hh) "
                  "or edit your workload size.");
        }
    }

    //Make Sure That this a Valid Architeture
    /*
     * 断言模拟器参数配置
     *
     * 断言如果传入参数的物理int型寄存器配置小于单核线程数*ISA中NumIntRegs的配置，则退出
     * 断言如果传入参数的物理Float型寄存器配置小于单核线程数*ISA中NumFloatRegs的配置，则退出
     * 断言如果传入参数的物理CC(condition code)型寄存器配置小于单核线程数*ISA中NumCCRegs的配置，则退出
     */
    assert(params->numPhysIntRegs   >= numThreadsTheISA::NumIntRegs);
    assert(params->numPhysFloatRegs >= numThreadsTheISA::NumFloatRegs);
    assert(params->numPhysCCRegs >= numThreadsTheISA::NumCCRegs);
    /*
     * 初始化rename和iew的scoreboard
     * 初始化所有线程的isa和commitRenameMap、renameMap
     *
     * 传入rename stage scoreboard的引用
     * 传入iew stage scoreboard的引用
     * 根据配置初始化线程中的isa
     * 对单核中所有线程中的commitRenameMap和renameMap执行init方法进行初始化
     * commitRenameMap和renameMap都是RenameMap类型(架构寄存器到物理寄存器的映射)
     */
    rename.setScoreboard(&scoreboard);
    iew.setScoreboard(&scoreboard);

    // Setup the rename map for whichever stages need it.
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        isa[tid] = params->isa[tid];

        // Only Alpha has an FP zero register, so for other ISAs we
        // use an invalid FP register index to avoid special treatment
        // of any valid FP reg.
        RegIndex invalidFPReg = TheISA::NumFloatRegs + 1;
        RegIndex fpZeroReg =
            (THE_ISA == ALPHA_ISA) ? TheISA::ZeroReg : invalidFPReg;

        commitRenameMap[tid].init(&regFile, TheISA::ZeroReg, fpZeroReg,
                                  &freeList);

        renameMap[tid].init(&regFile, TheISA::ZeroReg, fpZeroReg,
                            &freeList);
    }

    // Initialize rename map to assign physical registers to the
    // architectural registers for active threads only.
    /*
     * 初始化rename、commit的renameMap和commitRenameMap
     *
     * 对每个活动线程的renameMap和commitRenameMap绑定Entry
     * 按int,float,cc三种类型分别进行Entry的绑定
     *
     * 给rename stage传入配置好映射的renameMap
     * 给commit stage传入配置好映射的commitRenameMap
     * 给rename stage传入freeList(内有空闲的register list)引用
     *
     */
    for (ThreadID tid = 0; tid < active_threads; tid++) {
        for (RegIndex ridx = 0; ridx < TheISA::NumIntRegs; ++ridx) {
            // Note that we can't use the rename() method because we don't
            // want special treatment for the zero register at this point
            PhysRegIndex phys_reg = freeList.getIntReg();
            renameMap[tid].setIntEntry(ridx, phys_reg);
            commitRenameMap[tid].setIntEntry(ridx, phys_reg);
        }

        for (RegIndex ridx = 0; ridx < TheISA::NumFloatRegs; ++ridx) {
            PhysRegIndex phys_reg = freeList.getFloatReg();
            renameMap[tid].setFloatEntry(ridx, phys_reg);
            commitRenameMap[tid].setFloatEntry(ridx, phys_reg);
        }

        for (RegIndex ridx = 0; ridx < TheISA::NumCCRegs; ++ridx) {
            PhysRegIndex phys_reg = freeList.getCCReg();
            renameMap[tid].setCCEntry(ridx, phys_reg);
            commitRenameMap[tid].setCCEntry(ridx, phys_reg);
        }
    }

    rename.setRenameMap(renameMap);
    commit.setRenameMap(commitRenameMap);
    rename.setFreeList(&freeList);

    // Setup the ROB for whichever stages need it.
    /*
     * 初始化lastActivatedCycle和commit的ROB
     * Debug模式执行的调试代码
     * 记录运行信息
     * 分配thread的线程队列大小
     *
     * 给commit stage传入ROB
     * 初始化lastActivatedCycle(标记上一个活动状态的Cycle)为0
     * 调试代码，已被弃用(if 0)
     * 记录运行信息
     * 初始化thread(vector容器，存储线程)并分配numThreads大小的空间
     */
    commit.setROB(&rob);
    lastActivatedCycle = 0;
#if 0
    // Give renameMap & rename stage access to the freeList;
    for (ThreadID tid = 0; tid < numThreads; tid++)
        globalSeqNum[tid] = 1;
#endif

    DPRINTF(O3CPU, "Creating O3CPU object.\n");

    // Setup any thread state.
    this->thread.resize(this->numThreads);

    for (ThreadID tid = 0; tid < this->numThreads; ++tid) {
    	/*
    	 * 根据模拟模式实例化所有单核中可运行的thread(FS:1,SE:2)
    	 *
    	 * 如果是FullSystem(FullSystem模式暂时不支持超线程，所以还是一核一线程)
    	 *    断言numThreads==1，否则输出信息退出
    	 *    给每个线程实例化一个Thread，第三个参数process为NULL
    	 * 如果是非FullSystem模式(SE模式，支持超线程，所以numThreads>=1)
    	 * 	  如果tid<params->workload.size()(传入的配置参数)
    	 * 	  	 记录运行信息
    	 * 	  	 对每个线程进行实例化
    	 * 	  否则
    	 * 	     对每个线程进行实例化时，将第三个参数process设置为NULL
    	 */
        if (FullSystem) {
            // SMT is not supported in FS mode yet.
            assert(this->numThreads == 1);
            this->thread[tid] = new Thread(this, 0, NULL);
        } else {
            if (tid < params->workload.size()) {
                DPRINTF(O3CPU, "Workload[%i] process is %#x",
                        tid, this->thread[tid]);
                this->thread[tid] = new typename FullO3CPU<Impl>::Thread(
                        (typename Impl::O3CPU *)(this),
                        tid, params->workload[tid]);

                //usedTids[tid] = true;
                //threadMap[tid] = tid;
            } else {
                //Allocate Empty thread so M5 can use later
                //when scheduling threads to CPU
                Process* dummy_proc = NULL;

                this->thread[tid] = new typename FullO3CPU<Impl>::Thread(
                        (typename Impl::O3CPU *)(this),
                        tid, dummy_proc);
                //usedTids[tid] = false;
            }
        }
        /*
         * 实例化tc、o3_tc，并给成员变量cpu、thread进行初始化
         * 对当前对象(FullO3CPU)[的quiesceEvent]、thread的tc、threadContexts赋值
         *
         * [遍历所有单核线程:]
         * tc是指向ThreadContext的指针，ThreadContext提供所有thread state的公共接口
         * 声明并实例化O3ThreadContext<Impl> *类型的o3_tc
         * 令tc = o3_tc，会让tc的指针指向O3ThreadContext<Impl>
         * 这样tc可以就访问O3ThreadContext<Impl>的所有方法(多态)
         * 如果传入的参数中checker不为NULL
         *    则令tc指向CheckerThreadContext<O3ThreadContext<Impl> >的实例
         * 令o3_tc的cpu成员变量等于Impl::O3CPU *的this(原类型为FullO3CPU)
         * 断言o3_tc的成员变量cpu不为NULL，否则输出信息退出
         * 令o3_tc的thread等于当前线程
         * 如果是FullSystem
         *    那么当前线程的quiesceEvent赋值为EndQuiesceEvent(tc)
         *    EndQuiesceEvent的定义位于cpu/quiesce_event.hh|quiesce_event.cc
         *    EndQuiesceEvent作用是timing out静默指令
         * 给当前线程的成员变量tc赋值，赋值tc(前面刚实例化的)
         * 将tc添加到当前cpu的成员变量threadContexts的队列中去
         */
        ThreadContext *tc;

        // Setup the TC that will serve as the interface to the threads/CPU.
        O3ThreadContext<Impl> *o3_tc = new O3ThreadContext<Impl>;

        tc = o3_tc;

        // If we're using a checker, then the TC should be the
        // CheckerThreadContext.
        if (params->checker) {
            tc = new CheckerThreadContext<O3ThreadContext<Impl> >(
                o3_tc, this->checker);
        }

        o3_tc->cpu = (typename Impl::O3CPU *)(this);
        assert(o3_tc->cpu);
        o3_tc->thread = this->thread[tid];

        if (FullSystem) {
            // Setup quiesce event.
            this->thread[tid]->quiesceEvent = new EndQuiesceEvent(tc);
        }
        // Give the thread the TC.
        this->thread[tid]->tc = tc;

        // Add the TC to the CPU's list of TC's.
        this->threadContexts.push_back(tc);
    }

    // FullO3CPU always requires an interrupt controller.
    /*
     * 根据配置参数进行运行前的判断以及错误信息输出
     * 初始化单核中所有线程中当前对象的thread的funcExeInst为0
     *
     * 如果传入参数中switched_out为false且ISA配置中interrupts为false，
     *    那么说明没有载入中断控制，输出错误信息并退出
     * 遍历单核所有线程
     *    设置当前线程的funcExeInst(记录已经执行的指令数)为0
     *    setFuncExeInst定义于cpu/thread_state.hh
     */
    if (!params->switched_out && !interrupts) {
        fatal("FullO3CPU %s has no interrupt controller.\n"
              "Ensure createInterruptController() is called.\n", name());
    }

    for (ThreadID tid = 0; tid < this->numThreads; tid++)
        this->thread[tid]->setFuncExeInst(0);
}

template <class Impl>
FullO3CPU<Impl>::~FullO3CPU()
{
}

template <class Impl>
void
FullO3CPU<Impl>::regProbePoints()
{
    BaseCPU::regProbePoints();

    ppInstAccessComplete = new ProbePointArg<PacketPtr>(getProbeManager(), "InstAccessComplete");
    ppDataAccessComplete = new ProbePointArg<std::pair<DynInstPtr, PacketPtr> >(getProbeManager(), "DataAccessComplete");

    fetch.regProbePoints();
    iew.regProbePoints();
    commit.regProbePoints();
}

template <class Impl>
void
FullO3CPU<Impl>::regStats()
{
    BaseO3CPU::regStats();

    // Register any of the O3CPU's stats here.
    timesIdled
        .name(name() + ".timesIdled")
        .desc("Number of times that the entire CPU went into an idle state and"
              " unscheduled itself")
        .prereq(timesIdled);

    idleCycles
        .name(name() + ".idleCycles")
        .desc("Total number of cycles that the CPU has spent unscheduled due "
              "to idling")
        .prereq(idleCycles);

    quiesceCycles
        .name(name() + ".quiesceCycles")
        .desc("Total number of cycles that CPU has spent quiesced or waiting "
              "for an interrupt")
        .prereq(quiesceCycles);

    // Number of Instructions simulated
    // --------------------------------
    // Should probably be in Base CPU but need templated
    // MaxThreads so put in here instead
    committedInsts
        .init(numThreads)
        .name(name() + ".committedInsts")
        .desc("Number of Instructions Simulated")
        .flags(Stats::total);

    committedOps
        .init(numThreads)
        .name(name() + ".committedOps")
        .desc("Number of Ops (including micro ops) Simulated")
        .flags(Stats::total);

    cpi
        .name(name() + ".cpi")
        .desc("CPI: Cycles Per Instruction")
        .precision(6);
    cpi = numCycles / committedInsts;

    totalCpi
        .name(name() + ".cpi_total")
        .desc("CPI: Total CPI of All Threads")
        .precision(6);
    totalCpi = numCycles / sum(committedInsts);

    ipc
        .name(name() + ".ipc")
        .desc("IPC: Instructions Per Cycle")
        .precision(6);
    ipc =  committedInsts / numCycles;

    totalIpc
        .name(name() + ".ipc_total")
        .desc("IPC: Total IPC of All Threads")
        .precision(6);
    totalIpc =  sum(committedInsts) / numCycles;

    this->fetch.regStats();
    this->decode.regStats();
    this->rename.regStats();
    this->iew.regStats();
    this->commit.regStats();
    this->rob.regStats();

    intRegfileReads
        .name(name() + ".int_regfile_reads")
        .desc("number of integer regfile reads")
        .prereq(intRegfileReads);

    intRegfileWrites
        .name(name() + ".int_regfile_writes")
        .desc("number of integer regfile writes")
        .prereq(intRegfileWrites);

    fpRegfileReads
        .name(name() + ".fp_regfile_reads")
        .desc("number of floating regfile reads")
        .prereq(fpRegfileReads);

    fpRegfileWrites
        .name(name() + ".fp_regfile_writes")
        .desc("number of floating regfile writes")
        .prereq(fpRegfileWrites);

    ccRegfileReads
        .name(name() + ".cc_regfile_reads")
        .desc("number of cc regfile reads")
        .prereq(ccRegfileReads);

    ccRegfileWrites
        .name(name() + ".cc_regfile_writes")
        .desc("number of cc regfile writes")
        .prereq(ccRegfileWrites);

    miscRegfileReads
        .name(name() + ".misc_regfile_reads")
        .desc("number of misc regfile reads")
        .prereq(miscRegfileReads);

    miscRegfileWrites
        .name(name() + ".misc_regfile_writes")
        .desc("number of misc regfile writes")
        .prereq(miscRegfileWrites);
}


/*
 *
void:tick()
FullO3CPU时间点计时函数
 *
 */
template <class Impl>
void
FullO3CPU<Impl>::tick()
{
	/*
	 * 记录trace信息
	 * DPRINTF()函数位于base/trace.hh，作用是记录运行信息
	 *
	 * assert是宏，定义位于/usr/include/assert.h
	 * 用于debug模式，断言失败时输出错误信息并且终止程序运行
	 * 使用两条断言，断言只在debug模式下起作用
	 * 第一条断言条件是!switchedOut()，函数位于cpu/base.hh
	 * 当CPU是switch out状态时为true，!switchedOut()==false
	 * 当CPU是active状态时为false，!switchedOut()==true
	 * 第二条断言条件是getDrainState() != Drainable::Drained
	 * getDrainState()函数位于sim/drain.hh
	 * Drainable类的成员变量State为enum类型：Running/Draining/Drained
	 * 当_drainState为Drained时，条件为false，输出错误信息并终止程序运行
	 * 当_drainState为Running/Draining时，条件为true
	 *
	 */
    DPRINTF(O3CPU, "\n\nFullO3CPU: Ticking main, FullO3CPU.\n");
    assert(!switchedOut());
    assert(getDrainState() != Drainable::Drained);
    /*
     * Stats::Scalar numCycles
     * numCycles是标记CPU simulated cycle的number
     * 每调用一次FullO3CPU<Impl>::tick()的方法就++numCycles
     *
     * ppCycles的定义位于cpu/base.hh，为PMUUPtr类型
     * PMUUPtr实际上是unique_ptr<PMU>，定义在sim/probe/pmu.hh
     * PMU的类型是ProbePointArg<uint64_t>，也定义在sim/probe/pmu.hh
     * ProbePointArg定义在sim/probe/probe.hh
     * ppCycles->notify(1)实际上调用ProbePointArg的notify方法
     * 方法作用是遍历vector<ProbeListenerArgBase<Arg> *> listeners，
     * 并调用listener的ProbeListenerArgBase的notify()方法
     * 然而ProbeListenerArgBase的notify方法是虚函数
     * 因而看其子类ProbeListenerArgBase的实现
     * 子类的notify方法里面比较复杂，需要找到实例进行分析
     * 最后总结推测，ppCycles->notify(1)的作用是唤醒PMU的listener
     *
     */
    ++numCycles;
    ppCycles->notify(1);

//    activity = false;

    //Tick each of the stages
    /*
     * fetch等变量的定义位于cpu/o3/cpu.hh
     * Fetch等变量的定义位于cpu/o3/cpu_policy.hh
     *
     * 调用fetch_impl.hh中DefaultFetch的tick()方法
     * 调用decode_impl.hh中DefaultDecode的tick()方法
     * 调用rename_impl.hh中DefaultRename的tick()方法
     * 调用iew_impl.hh中DefaultIEW的tick()方法
     * 调用commit_impl.hh中DefaultCommit的tick()方法
     *
	 * 调用fetch stage的时间点计时函数(即开始功能模块的调用)
	 * 调用decode stage时间点计时函数
	 * 调用rename stage时间点计时函数
	 * 调用iew stage时间点计时函数
	 * 调用commit stage时间点计时函数
     *
     */
    fetch.tick();
    
    decode.tick();

    rename.tick();

    iew.tick();

    commit.tick();

    // Now advance the time buffers
    /*
     * 为timeBuffer、fetchQueue、decodeQueue、
     * renameQueue、iewQueue、activityRec分配空间
     *
     * timeBuffer的定义位于cpu/o3/cpu.hh
     * timeBuffer的定义位于cpu/o3/cpu.hhTimeBuffer类的定义位于cpu/timebuf.hh
     */
    timeBuffer.advance();

    fetchQueue.advance();
    decodeQueue.advance();
    renameQueue.advance();
    iewQueue.advance();

    activityRec.advance();
    /*
     * 如果removeInstsThisCycle为true
     * 那么清除instList
     */
    if (removeInstsThisCycle) {
        cleanUpRemovedInsts();
    }
    /*
     * 如果tickEvent没有调度
     *    当_status为SwitchedOut时
     *       记录cpu SwitchedOut的运行信息
     *    	 令lastRunningCycle = curCycle()
     *    当activityRec是inactive时或_status为Idle
     *    	 记录cpu Idle的运行信息
     *    	 令lastRunningCycle = curCycle()，
     *    	 同时timesIdled++
     *    其他情况：
     *       开始执行schedule方法对tickEvent进行调度
     *       记录cpu Scheduling new tick的运行信息
     */
    if (!tickEvent.scheduled()) {
        if (_status == SwitchedOut) {
            DPRINTF(O3CPU, "Switched out!\n");
            // increment stat
            lastRunningCycle = curCycle();
        } else if (!activityRec.active() || _status == Idle) {
            DPRINTF(O3CPU, "Idle!\n");
            lastRunningCycle = curCycle();
            timesIdled++;
        } else {
            schedule(tickEvent, clockEdge(Cycles(1)));
            DPRINTF(O3CPU, "Scheduling next tick!\n");
        }
    }
    /*
     * 如果不是FS模式，马上调用updateThreadPriority
     * 执行tryDrain()，对tickEvent重新进行调度
     *
     * updateThreadPriority作用是将activeThreads队列中
     * 的第一个线程取出，丢入最后一个
     */
    if (!FullSystem)
        updateThreadPriority();

    tryDrain();
}

template <class Impl>
void
FullO3CPU<Impl>::init()
{
    BaseCPU::init();

    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        // Set noSquashFromTC so that the CPU doesn't squash when initially
        // setting up registers.
        thread[tid]->noSquashFromTC = true;
        // Initialise the ThreadContext's memory proxies
        thread[tid]->initMemProxies(thread[tid]->getTC());
    }

    if (FullSystem && !params()->switched_out) {
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            ThreadContext *src_tc = threadContexts[tid];
            TheISA::initCPU(src_tc, src_tc->contextId());
        }
    }

    // Clear noSquashFromTC.
    for (int tid = 0; tid < numThreads; ++tid)
        thread[tid]->noSquashFromTC = false;

    commit.setThreads(thread);
}

template <class Impl>
void
FullO3CPU<Impl>::startup()
{
    BaseCPU::startup();
    for (int tid = 0; tid < numThreads; ++tid)
        isa[tid]->startup(threadContexts[tid]);

    fetch.startupStage();
    decode.startupStage();
    iew.startupStage();
    rename.startupStage();
    commit.startupStage();
}

template <class Impl>
void
FullO3CPU<Impl>::activateThread(ThreadID tid)
{
    list<ThreadID>::iterator isActive =
        std::find(activeThreads.begin(), activeThreads.end(), tid);

    DPRINTF(O3CPU, "[tid:%i]: Calling activate thread.\n", tid);
    assert(!switchedOut());

    if (isActive == activeThreads.end()) {
        DPRINTF(O3CPU, "[tid:%i]: Adding to active threads list\n",
                tid);

        activeThreads.push_back(tid);
    }
}

template <class Impl>
void
FullO3CPU<Impl>::deactivateThread(ThreadID tid)
{
    //Remove From Active List, if Active
    list<ThreadID>::iterator thread_it =
        std::find(activeThreads.begin(), activeThreads.end(), tid);

    DPRINTF(O3CPU, "[tid:%i]: Calling deactivate thread.\n", tid);
    assert(!switchedOut());

    if (thread_it != activeThreads.end()) {
        DPRINTF(O3CPU,"[tid:%i]: Removing from active threads list\n",
                tid);
        activeThreads.erase(thread_it);
    }

    fetch.deactivateThread(tid);
    commit.deactivateThread(tid);
}

template <class Impl>
Counter
FullO3CPU<Impl>::totalInsts() const
{
    Counter total(0);

    ThreadID size = thread.size();
    for (ThreadID i = 0; i < size; i++)
        total += thread[i]->numInst;

    return total;
}

template <class Impl>
Counter
FullO3CPU<Impl>::totalOps() const
{
    Counter total(0);

    ThreadID size = thread.size();
    for (ThreadID i = 0; i < size; i++)
        total += thread[i]->numOp;

    return total;
}

template <class Impl>
void
FullO3CPU<Impl>::activateContext(ThreadID tid)
{
    assert(!switchedOut());

    // Needs to set each stage to running as well.
    activateThread(tid);

    // We don't want to wake the CPU if it is drained. In that case,
    // we just want to flag the thread as active and schedule the tick
    // event from drainResume() instead.
    if (getDrainState() == Drainable::Drained)
        return;

    // If we are time 0 or if the last activation time is in the past,
    // schedule the next tick and wake up the fetch unit
    if (lastActivatedCycle == 0 || lastActivatedCycle < curTick()) {
        scheduleTickEvent(Cycles(0));

        // Be sure to signal that there's some activity so the CPU doesn't
        // deschedule itself.
        activityRec.activity();
        fetch.wakeFromQuiesce();

        Cycles cycles(curCycle() - lastRunningCycle);
        // @todo: This is an oddity that is only here to match the stats
        if (cycles != 0)
            --cycles;
        quiesceCycles += cycles;

        lastActivatedCycle = curTick();

        _status = Running;
    }
}

template <class Impl>
void
FullO3CPU<Impl>::suspendContext(ThreadID tid)
{
    DPRINTF(O3CPU,"[tid: %i]: Suspending Thread Context.\n", tid);
    assert(!switchedOut());

    deactivateThread(tid);

    // If this was the last thread then unschedule the tick event.
    if (activeThreads.size() == 0) {
        unscheduleTickEvent();
        lastRunningCycle = curCycle();
        _status = Idle;
    }

    DPRINTF(Quiesce, "Suspending Context\n");
}

template <class Impl>
void
FullO3CPU<Impl>::haltContext(ThreadID tid)
{
    //For now, this is the same as deallocate
    DPRINTF(O3CPU,"[tid:%i]: Halt Context called. Deallocating", tid);
    assert(!switchedOut());

    deactivateThread(tid);
    removeThread(tid);
}

template <class Impl>
void
FullO3CPU<Impl>::insertThread(ThreadID tid)
{
    DPRINTF(O3CPU,"[tid:%i] Initializing thread into CPU");
    // Will change now that the PC and thread state is internal to the CPU
    // and not in the ThreadContext.
    ThreadContext *src_tc;
    if (FullSystem)
        src_tc = system->threadContexts[tid];
    else
        src_tc = tcBase(tid);

    //Bind Int Regs to Rename Map
    for (int ireg = 0; ireg < TheISA::NumIntRegs; ireg++) {
        PhysRegIndex phys_reg = freeList.getIntReg();

        renameMap[tid].setEntry(ireg,phys_reg);
        scoreboard.setReg(phys_reg);
    }

    //Bind Float Regs to Rename Map
    int max_reg = TheISA::NumIntRegs + TheISA::NumFloatRegs;
    for (int freg = TheISA::NumIntRegs; freg < max_reg; freg++) {
        PhysRegIndex phys_reg = freeList.getFloatReg();

        renameMap[tid].setEntry(freg,phys_reg);
        scoreboard.setReg(phys_reg);
    }

    //Bind condition-code Regs to Rename Map
    max_reg = TheISA::NumIntRegs + TheISA::NumFloatRegs + TheISA::NumCCRegs;
    for (int creg = TheISA::NumIntRegs + TheISA::NumFloatRegs;
         creg < max_reg; creg++) {
        PhysRegIndex phys_reg = freeList.getCCReg();

        renameMap[tid].setEntry(creg,phys_reg);
        scoreboard.setReg(phys_reg);
    }

    //Copy Thread Data Into RegFile
    //this->copyFromTC(tid);

    //Set PC/NPC/NNPC
    pcState(src_tc->pcState(), tid);

    src_tc->setStatus(ThreadContext::Active);

    activateContext(tid);

    //Reset ROB/IQ/LSQ Entries
    commit.rob->resetEntries();
    iew.resetEntries();
}

template <class Impl>
void
FullO3CPU<Impl>::removeThread(ThreadID tid)
{
    DPRINTF(O3CPU,"[tid:%i] Removing thread context from CPU.\n", tid);

    // Copy Thread Data From RegFile
    // If thread is suspended, it might be re-allocated
    // this->copyToTC(tid);


    // @todo: 2-27-2008: Fix how we free up rename mappings
    // here to alleviate the case for double-freeing registers
    // in SMT workloads.

    // Unbind Int Regs from Rename Map
    for (int ireg = 0; ireg < TheISA::NumIntRegs; ireg++) {
        PhysRegIndex phys_reg = renameMap[tid].lookup(ireg);
        scoreboard.unsetReg(phys_reg);
        freeList.addReg(phys_reg);
    }

    // Unbind Float Regs from Rename Map
    int max_reg = TheISA::FP_Reg_Base + TheISA::NumFloatRegs;
    for (int freg = TheISA::FP_Reg_Base; freg < max_reg; freg++) {
        PhysRegIndex phys_reg = renameMap[tid].lookup(freg);
        scoreboard.unsetReg(phys_reg);
        freeList.addReg(phys_reg);
    }

    // Unbind condition-code Regs from Rename Map
    max_reg = TheISA::CC_Reg_Base + TheISA::NumCCRegs;
    for (int creg = TheISA::CC_Reg_Base; creg < max_reg; creg++) {
        PhysRegIndex phys_reg = renameMap[tid].lookup(creg);
        scoreboard.unsetReg(phys_reg);
        freeList.addReg(phys_reg);
    }

    // Squash Throughout Pipeline
    DynInstPtr inst = commit.rob->readHeadInst(tid);
    InstSeqNum squash_seq_num = inst->seqNum;
    fetch.squash(0, squash_seq_num, inst, tid);
    decode.squash(tid);
    rename.squash(squash_seq_num, tid);
    iew.squash(tid);
    iew.ldstQueue.squash(squash_seq_num, tid);
    commit.rob->squash(squash_seq_num, tid);


    assert(iew.instQueue.getCount(tid) == 0);
    assert(iew.ldstQueue.getCount(tid) == 0);

    // Reset ROB/IQ/LSQ Entries

    // Commented out for now.  This should be possible to do by
    // telling all the pipeline stages to drain first, and then
    // checking until the drain completes.  Once the pipeline is
    // drained, call resetEntries(). - 10-09-06 ktlim
/*
    if (activeThreads.size() >= 1) {
        commit.rob->resetEntries();
        iew.resetEntries();
    }
*/
}

template <class Impl>
Fault
FullO3CPU<Impl>::hwrei(ThreadID tid)
{
#if THE_ISA == ALPHA_ISA
    // Need to clear the lock flag upon returning from an interrupt.
    this->setMiscRegNoEffect(AlphaISA::MISCREG_LOCKFLAG, false, tid);

    this->thread[tid]->kernelStats->hwrei();

    // FIXME: XXX check for interrupts? XXX
#endif
    return NoFault;
}

template <class Impl>
bool
FullO3CPU<Impl>::simPalCheck(int palFunc, ThreadID tid)
{
#if THE_ISA == ALPHA_ISA
    if (this->thread[tid]->kernelStats)
        this->thread[tid]->kernelStats->callpal(palFunc,
                                                this->threadContexts[tid]);

    switch (palFunc) {
      case PAL::halt:
        halt();
        if (--System::numSystemsRunning == 0)
            exitSimLoop("all cpus halted");
        break;

      case PAL::bpt:
      case PAL::bugchk:
        if (this->system->breakpoint())
            return false;
        break;
    }
#endif
    return true;
}

template <class Impl>
Fault
FullO3CPU<Impl>::getInterrupts()
{
    // Check if there are any outstanding interrupts
    return this->interrupts->getInterrupt(this->threadContexts[0]);
}

template <class Impl>
void
FullO3CPU<Impl>::processInterrupts(const Fault &interrupt)
{
    // Check for interrupts here.  For now can copy the code that
    // exists within isa_fullsys_traits.hh.  Also assume that thread 0
    // is the one that handles the interrupts.
    // @todo: Possibly consolidate the interrupt checking code.
    // @todo: Allow other threads to handle interrupts.

    assert(interrupt != NoFault);
    this->interrupts->updateIntrInfo(this->threadContexts[0]);

    DPRINTF(O3CPU, "Interrupt %s being handled\n", interrupt->name());
    this->trap(interrupt, 0, nullptr);
}

template <class Impl>
void
FullO3CPU<Impl>::trap(const Fault &fault, ThreadID tid,
                      const StaticInstPtr &inst)
{
    // Pass the thread's TC into the invoke method.
    fault->invoke(this->threadContexts[tid], inst);
}

template <class Impl>
void
FullO3CPU<Impl>::syscall(int64_t callnum, ThreadID tid)
{
    DPRINTF(O3CPU, "[tid:%i] Executing syscall().\n\n", tid);

    DPRINTF(Activity,"Activity: syscall() called.\n");

    // Temporarily increase this by one to account for the syscall
    // instruction.
    ++(this->thread[tid]->funcExeInst);

    // Execute the actual syscall.
    this->thread[tid]->syscall(callnum);

    // Decrease funcExeInst by one as the normal commit will handle
    // incrementing it.
    --(this->thread[tid]->funcExeInst);
}

template <class Impl>
void
FullO3CPU<Impl>::serializeThread(std::ostream &os, ThreadID tid)
{
    thread[tid]->serialize(os);
}

template <class Impl>
void
FullO3CPU<Impl>::unserializeThread(Checkpoint *cp, const std::string &section,
                                   ThreadID tid)
{
    thread[tid]->unserialize(cp, section);
}

template <class Impl>
unsigned int
FullO3CPU<Impl>::drain(DrainManager *drain_manager)
{
    // If the CPU isn't doing anything, then return immediately.
    if (switchedOut()) {
        setDrainState(Drainable::Drained);
        return 0;
    }

    DPRINTF(Drain, "Draining...\n");
    setDrainState(Drainable::Draining);

    // We only need to signal a drain to the commit stage as this
    // initiates squashing controls the draining. Once the commit
    // stage commits an instruction where it is safe to stop, it'll
    // squash the rest of the instructions in the pipeline and force
    // the fetch stage to stall. The pipeline will be drained once all
    // in-flight instructions have retired.
    commit.drain();

    // Wake the CPU and record activity so everything can drain out if
    // the CPU was not able to immediately drain.
    if (!isDrained())  {
        drainManager = drain_manager;

        wakeCPU();
        activityRec.activity();

        DPRINTF(Drain, "CPU not drained\n");

        return 1;
    } else {
        setDrainState(Drainable::Drained);
        DPRINTF(Drain, "CPU is already drained\n");
        if (tickEvent.scheduled())
            deschedule(tickEvent);

        // Flush out any old data from the time buffers.  In
        // particular, there might be some data in flight from the
        // fetch stage that isn't visible in any of the CPU buffers we
        // test in isDrained().
        for (int i = 0; i < timeBuffer.getSize(); ++i) {
            timeBuffer.advance();
            fetchQueue.advance();
            decodeQueue.advance();
            renameQueue.advance();
            iewQueue.advance();
        }

        drainSanityCheck();
        return 0;
    }
}

template <class Impl>
bool
FullO3CPU<Impl>::tryDrain()
{
    if (!drainManager || !isDrained())
        return false;

    if (tickEvent.scheduled())
        deschedule(tickEvent);

    DPRINTF(Drain, "CPU done draining, processing drain event\n");
    drainManager->signalDrainDone();
    drainManager = NULL;

    return true;
}

template <class Impl>
void
FullO3CPU<Impl>::drainSanityCheck() const
{
    assert(isDrained());
    fetch.drainSanityCheck();
    decode.drainSanityCheck();
    rename.drainSanityCheck();
    iew.drainSanityCheck();
    commit.drainSanityCheck();
}

template <class Impl>
bool
FullO3CPU<Impl>::isDrained() const
{
    bool drained(true);

    if (!instList.empty() || !removeList.empty()) {
        DPRINTF(Drain, "Main CPU structures not drained.\n");
        drained = false;
    }

    if (!fetch.isDrained()) {
        DPRINTF(Drain, "Fetch not drained.\n");
        drained = false;
    }

    if (!decode.isDrained()) {
        DPRINTF(Drain, "Decode not drained.\n");
        drained = false;
    }

    if (!rename.isDrained()) {
        DPRINTF(Drain, "Rename not drained.\n");
        drained = false;
    }

    if (!iew.isDrained()) {
        DPRINTF(Drain, "IEW not drained.\n");
        drained = false;
    }

    if (!commit.isDrained()) {
        DPRINTF(Drain, "Commit not drained.\n");
        drained = false;
    }

    return drained;
}

template <class Impl>
void
FullO3CPU<Impl>::commitDrained(ThreadID tid)
{
    fetch.drainStall(tid);
}

template <class Impl>
void
FullO3CPU<Impl>::drainResume()
{
    setDrainState(Drainable::Running);
    if (switchedOut())
        return;

    DPRINTF(Drain, "Resuming...\n");
    verifyMemoryMode();

    fetch.drainResume();
    commit.drainResume();

    _status = Idle;
    for (ThreadID i = 0; i < thread.size(); i++) {
        if (thread[i]->status() == ThreadContext::Active) {
            DPRINTF(Drain, "Activating thread: %i\n", i);
            activateThread(i);
            _status = Running;
        }
    }

    assert(!tickEvent.scheduled());
    if (_status == Running)
        schedule(tickEvent, nextCycle());
}

template <class Impl>
void
FullO3CPU<Impl>::switchOut()
{
    DPRINTF(O3CPU, "Switching out\n");
    BaseCPU::switchOut();

    activityRec.reset();

    _status = SwitchedOut;

    if (checker)
        checker->switchOut();
}

template <class Impl>
void
FullO3CPU<Impl>::takeOverFrom(BaseCPU *oldCPU)
{
    BaseCPU::takeOverFrom(oldCPU);

    fetch.takeOverFrom();
    decode.takeOverFrom();
    rename.takeOverFrom();
    iew.takeOverFrom();
    commit.takeOverFrom();

    assert(!tickEvent.scheduled());

    FullO3CPU<Impl> *oldO3CPU = dynamic_cast<FullO3CPU<Impl>*>(oldCPU);
    if (oldO3CPU)
        globalSeqNum = oldO3CPU->globalSeqNum;

    lastRunningCycle = curCycle();
    _status = Idle;
}

template <class Impl>
void
FullO3CPU<Impl>::verifyMemoryMode() const
{
    if (!system->isTimingMode()) {
        fatal("The O3 CPU requires the memory system to be in "
              "'timing' mode.\n");
    }
}

template <class Impl>
TheISA::MiscReg
FullO3CPU<Impl>::readMiscRegNoEffect(int misc_reg, ThreadID tid) const
{
    return this->isa[tid]->readMiscRegNoEffect(misc_reg);
}

template <class Impl>
TheISA::MiscReg
FullO3CPU<Impl>::readMiscReg(int misc_reg, ThreadID tid)
{
    miscRegfileReads++;
    return this->isa[tid]->readMiscReg(misc_reg, tcBase(tid));
}

template <class Impl>
void
FullO3CPU<Impl>::setMiscRegNoEffect(int misc_reg,
        const TheISA::MiscReg &val, ThreadID tid)
{
    this->isa[tid]->setMiscRegNoEffect(misc_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setMiscReg(int misc_reg,
        const TheISA::MiscReg &val, ThreadID tid)
{
    miscRegfileWrites++;
    this->isa[tid]->setMiscReg(misc_reg, val, tcBase(tid));
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readIntReg(int reg_idx)
{
    intRegfileReads++;
    return regFile.readIntReg(reg_idx);
}

template <class Impl>
FloatReg
FullO3CPU<Impl>::readFloatReg(int reg_idx)
{
    fpRegfileReads++;
    return regFile.readFloatReg(reg_idx);
}

template <class Impl>
FloatRegBits
FullO3CPU<Impl>::readFloatRegBits(int reg_idx)
{
    fpRegfileReads++;
    return regFile.readFloatRegBits(reg_idx);
}

template <class Impl>
CCReg
FullO3CPU<Impl>::readCCReg(int reg_idx)
{
    ccRegfileReads++;
    return regFile.readCCReg(reg_idx);
}

template <class Impl>
void
FullO3CPU<Impl>::setIntReg(int reg_idx, uint64_t val)
{
    intRegfileWrites++;
    regFile.setIntReg(reg_idx, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatReg(int reg_idx, FloatReg val)
{
    fpRegfileWrites++;
    regFile.setFloatReg(reg_idx, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatRegBits(int reg_idx, FloatRegBits val)
{
    fpRegfileWrites++;
    regFile.setFloatRegBits(reg_idx, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setCCReg(int reg_idx, CCReg val)
{
    ccRegfileWrites++;
    regFile.setCCReg(reg_idx, val);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readArchIntReg(int reg_idx, ThreadID tid)
{
    intRegfileReads++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupInt(reg_idx);

    return regFile.readIntReg(phys_reg);
}

template <class Impl>
float
FullO3CPU<Impl>::readArchFloatReg(int reg_idx, ThreadID tid)
{
    fpRegfileReads++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupFloat(reg_idx);

    return regFile.readFloatReg(phys_reg);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readArchFloatRegInt(int reg_idx, ThreadID tid)
{
    fpRegfileReads++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupFloat(reg_idx);

    return regFile.readFloatRegBits(phys_reg);
}

template <class Impl>
CCReg
FullO3CPU<Impl>::readArchCCReg(int reg_idx, ThreadID tid)
{
    ccRegfileReads++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupCC(reg_idx);

    return regFile.readCCReg(phys_reg);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchIntReg(int reg_idx, uint64_t val, ThreadID tid)
{
    intRegfileWrites++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupInt(reg_idx);

    regFile.setIntReg(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchFloatReg(int reg_idx, float val, ThreadID tid)
{
    fpRegfileWrites++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupFloat(reg_idx);

    regFile.setFloatReg(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchFloatRegInt(int reg_idx, uint64_t val, ThreadID tid)
{
    fpRegfileWrites++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupFloat(reg_idx);

    regFile.setFloatRegBits(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchCCReg(int reg_idx, CCReg val, ThreadID tid)
{
    ccRegfileWrites++;
    PhysRegIndex phys_reg = commitRenameMap[tid].lookupCC(reg_idx);

    regFile.setCCReg(phys_reg, val);
}

template <class Impl>
TheISA::PCState
FullO3CPU<Impl>::pcState(ThreadID tid)
{
    return commit.pcState(tid);
}

template <class Impl>
void
FullO3CPU<Impl>::pcState(const TheISA::PCState &val, ThreadID tid)
{
    commit.pcState(val, tid);
}

template <class Impl>
Addr
FullO3CPU<Impl>::instAddr(ThreadID tid)
{
    return commit.instAddr(tid);
}

template <class Impl>
Addr
FullO3CPU<Impl>::nextInstAddr(ThreadID tid)
{
    return commit.nextInstAddr(tid);
}

template <class Impl>
MicroPC
FullO3CPU<Impl>::microPC(ThreadID tid)
{
    return commit.microPC(tid);
}

template <class Impl>
void
FullO3CPU<Impl>::squashFromTC(ThreadID tid)
{
    this->thread[tid]->noSquashFromTC = true;
    this->commit.generateTCEvent(tid);
}

template <class Impl>
typename FullO3CPU<Impl>::ListIt
FullO3CPU<Impl>::addInst(DynInstPtr &inst)
{
    instList.push_back(inst);

    return --(instList.end());
}

template <class Impl>
void
FullO3CPU<Impl>::instDone(ThreadID tid, DynInstPtr &inst)
{
    // Keep an instruction count.
    if (!inst->isMicroop() || inst->isLastMicroop()) {
        thread[tid]->numInst++;
        thread[tid]->numInsts++;
        committedInsts[tid]++;
        system->totalNumInsts++;

        // Check for instruction-count-based events.
        comInstEventQueue[tid]->serviceEvents(thread[tid]->numInst);
        system->instEventQueue.serviceEvents(system->totalNumInsts);
    }
    thread[tid]->numOp++;
    thread[tid]->numOps++;
    committedOps[tid]++;

    probeInstCommit(inst->staticInst);
}

template <class Impl>
void
FullO3CPU<Impl>::removeFrontInst(DynInstPtr &inst)
{
    DPRINTF(O3CPU, "Removing committed instruction [tid:%i] PC %s "
            "[sn:%lli]\n",
            inst->threadNumber, inst->pcState(), inst->seqNum);

    removeInstsThisCycle = true;

    // Remove the front instruction.
    removeList.push(inst->getInstListIt());
}

template <class Impl>
void
FullO3CPU<Impl>::removeInstsNotInROB(ThreadID tid)
{
    DPRINTF(O3CPU, "Thread %i: Deleting instructions from instruction"
            " list.\n", tid);

    ListIt end_it;

    bool rob_empty = false;

    if (instList.empty()) {
        return;
    } else if (rob.isEmpty(tid)) {
        DPRINTF(O3CPU, "ROB is empty, squashing all insts.\n");
        end_it = instList.begin();
        rob_empty = true;
    } else {
        end_it = (rob.readTailInst(tid))->getInstListIt();
        DPRINTF(O3CPU, "ROB is not empty, squashing insts not in ROB.\n");
    }

    removeInstsThisCycle = true;

    ListIt inst_it = instList.end();

    inst_it--;

    // Walk through the instruction list, removing any instructions
    // that were inserted after the given instruction iterator, end_it.
    while (inst_it != end_it) {
        assert(!instList.empty());

        squashInstIt(inst_it, tid);

        inst_it--;
    }

    // If the ROB was empty, then we actually need to remove the first
    // instruction as well.
    if (rob_empty) {
        squashInstIt(inst_it, tid);
    }
}

template <class Impl>
void
FullO3CPU<Impl>::removeInstsUntil(const InstSeqNum &seq_num, ThreadID tid)
{
    assert(!instList.empty());

    removeInstsThisCycle = true;

    ListIt inst_iter = instList.end();

    inst_iter--;

    DPRINTF(O3CPU, "Deleting instructions from instruction "
            "list that are from [tid:%i] and above [sn:%lli] (end=%lli).\n",
            tid, seq_num, (*inst_iter)->seqNum);

    while ((*inst_iter)->seqNum > seq_num) {

        bool break_loop = (inst_iter == instList.begin());

        squashInstIt(inst_iter, tid);

        inst_iter--;

        if (break_loop)
            break;
    }
}

template <class Impl>
inline void
FullO3CPU<Impl>::squashInstIt(const ListIt &instIt, ThreadID tid)
{
    if ((*instIt)->threadNumber == tid) {
        DPRINTF(O3CPU, "Squashing instruction, "
                "[tid:%i] [sn:%lli] PC %s\n",
                (*instIt)->threadNumber,
                (*instIt)->seqNum,
                (*instIt)->pcState());

        // Mark it as squashed.
        (*instIt)->setSquashed();

        // @todo: Formulate a consistent method for deleting
        // instructions from the instruction list
        // Remove the instruction from the list.
        removeList.push(instIt);
    }
}

template <class Impl>
void
FullO3CPU<Impl>::cleanUpRemovedInsts()
{
    while (!removeList.empty()) {
        DPRINTF(O3CPU, "Removing instruction, "
                "[tid:%i] [sn:%lli] PC %s\n",
                (*removeList.front())->threadNumber,
                (*removeList.front())->seqNum,
                (*removeList.front())->pcState());

        instList.erase(removeList.front());

        removeList.pop();
    }

    removeInstsThisCycle = false;
}
/*
template <class Impl>
void
FullO3CPU<Impl>::removeAllInsts()
{
    instList.clear();
}
*/
template <class Impl>
void
FullO3CPU<Impl>::dumpInsts()
{
    int num = 0;

    ListIt inst_list_it = instList.begin();

    cprintf("Dumping Instruction List\n");

    while (inst_list_it != instList.end()) {
        cprintf("Instruction:%i\nPC:%#x\n[tid:%i]\n[sn:%lli]\nIssued:%i\n"
                "Squashed:%i\n\n",
                num, (*inst_list_it)->instAddr(), (*inst_list_it)->threadNumber,
                (*inst_list_it)->seqNum, (*inst_list_it)->isIssued(),
                (*inst_list_it)->isSquashed());
        inst_list_it++;
        ++num;
    }
}
/*
template <class Impl>
void
FullO3CPU<Impl>::wakeDependents(DynInstPtr &inst)
{
    iew.wakeDependents(inst);
}
*/
template <class Impl>
void
FullO3CPU<Impl>::wakeCPU()
{
    if (activityRec.active() || tickEvent.scheduled()) {
        DPRINTF(Activity, "CPU already running.\n");
        return;
    }

    DPRINTF(Activity, "Waking up CPU\n");

    Cycles cycles(curCycle() - lastRunningCycle);
    // @todo: This is an oddity that is only here to match the stats
    if (cycles > 1) {
        --cycles;
        idleCycles += cycles;
        numCycles += cycles;
        ppCycles->notify(cycles);
    }

    schedule(tickEvent, clockEdge());
}

template <class Impl>
void
FullO3CPU<Impl>::wakeup()
{
    if (this->thread[0]->status() != ThreadContext::Suspended)
        return;

    this->wakeCPU();

    DPRINTF(Quiesce, "Suspended Processor woken\n");
    this->threadContexts[0]->activate();
}

template <class Impl>
ThreadID
FullO3CPU<Impl>::getFreeTid()
{
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (!tids[tid]) {
            tids[tid] = true;
            return tid;
        }
    }

    return InvalidThreadID;
}

template <class Impl>
void
FullO3CPU<Impl>::updateThreadPriority()
{
    if (activeThreads.size() > 1) {
        //DEFAULT TO ROUND ROBIN SCHEME
        //e.g. Move highest priority to end of thread list
        list<ThreadID>::iterator list_begin = activeThreads.begin();

        unsigned high_thread = *list_begin;

        activeThreads.erase(list_begin);

        activeThreads.push_back(high_thread);
    }
}

// Forward declaration of FullO3CPU.
template class FullO3CPU<O3CPUImpl>;
