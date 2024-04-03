/* 046267 Computer Architecture - HW #4 */

#include "core_api.h"
#include "sim_api.h"

#include <stdio.h>
#include "vector"


class threadClass{
public:
    int tid;
    int latency;
    tcontext regs;
    unsigned inst_num;
    explicit threadClass(int tid=0): tid(tid), latency(0),inst_num(0){
        for (int i = 0; i < REGS_COUNT; ++i) {
            regs.reg[i] =0;
        }
    }

};

class coreClass {
public:
    int threads_num;
    threadClass *threads;
    int MRUThread;
    int cycles_num;
    int inst_num;
    int load_latency;
    int store_latency;
    int switch_cycles;
    bool is_fineGrained;

    coreClass(int threadNum, bool is_fineGrained, int switch_latency = 0) : threads_num(threadNum),
    threads(new threadClass[threadNum]),MRUThread(0), cycles_num(0), inst_num(0),
    load_latency(SIM_GetLoadLat()), store_latency(SIM_GetStoreLat()), switch_cycles(switch_latency),
    is_fineGrained(is_fineGrained) {
        for (int i = 0; i < threadNum; ++i) {
            threads[i] = threadClass(i);
        }
    }
    ~coreClass() {
        delete[] threads;
    }

    void initializeQueues(bool* running_queue, int threads_num,
                          int& num_of_working_threads, int& num_of_running_threads) {
        num_of_working_threads = threads_num;
        num_of_running_threads = threads_num;
        // Initialize running_queue to indicate all threads are initially running
        for (int i = 0; i < threads_num; ++i) {
            running_queue[i] = true;
        }
    }

    int findNextRunningThread(int MRUThread, int threads_num, bool* running_queue) {
        for (int i = 1; i < threads_num; ++i) {
            if (running_queue[(MRUThread + i) % threads_num]) {
                return (MRUThread + i) % threads_num;
            }
        }
        return MRUThread; // Fallback to current MRUThread if no other threads are running
    }

// Fetch and execute instructions for the most recently used thread
    void fetchAndExecuteInstruction(threadClass& thread, bool is_fineGrained, int& MRUThread, int switch_cycles,
        int threads_num, int& cycles_num, int& num_of_running_threads, int& num_of_working_threads, std::vector<int>& waiting_queue, bool* running_queue) {
        if (!running_queue[MRUThread]) {
            // Find the next available running thread
            MRUThread = findNextRunningThread(MRUThread, threads_num, running_queue);
        }

        // If the thread is running, execute the next instruction
        if (running_queue[MRUThread]) {
            Instruction inst;
            inst_num++;
            SIM_MemInstRead(thread.inst_num++, &inst, MRUThread);

            switch (inst.opcode) {
                case CMD_NOP:
                    break;
                case CMD_ADD:
                    thread.regs.reg[inst.dst_index] = thread.regs.reg[inst.src1_index] + thread.regs.reg[inst.src2_index_imm];
                    break;
                case CMD_ADDI:
                    thread.regs.reg[inst.dst_index] = thread.regs.reg[inst.src1_index] + inst.src2_index_imm;
                    break;
                case CMD_SUB:
                    thread.regs.reg[inst.dst_index] = thread.regs.reg[inst.src1_index] - thread.regs.reg[inst.src2_index_imm];
                    break;
                case CMD_SUBI:
                    thread.regs.reg[inst.dst_index] = thread.regs.reg[inst.src1_index] - inst.src2_index_imm;
                    break;
                case CMD_HALT:
                    running_queue[MRUThread] = false;
                    num_of_running_threads--;
                    num_of_working_threads--;
                    break;
                case CMD_LOAD: {
                    unsigned addr = thread.regs.reg[inst.src1_index] +
                                    (inst.isSrc2Imm ? inst.src2_index_imm : thread.regs.reg[inst.src2_index_imm]);
                    SIM_MemDataRead(addr, &(thread.regs.reg[inst.dst_index]));
                    thread.latency = load_latency; // Set thread latency for memory load
                    running_queue[MRUThread] = false;
                    num_of_running_threads--;
                    //waiting_queue.push_back(MRUThread);
                    break;
                }
                case CMD_STORE: {
                    unsigned addr = thread.regs.reg[inst.dst_index] +
                                    (inst.isSrc2Imm ? inst.src2_index_imm : thread.regs.reg[inst.src2_index_imm]);
                    SIM_MemDataWrite(addr, thread.regs.reg[inst.src1_index]);
                    thread.latency = store_latency; // Set thread latency for memory store
                    running_queue[MRUThread] = false;
                    num_of_running_threads--;
                    //waiting_queue.push_back(MRUThread);
                    break;
                }
            }

            // Update MRUThread for fine-grained MT
            if (is_fineGrained) {
                MRUThread = (MRUThread + 1) % threads_num;
            }

            // If thread switching is needed and not fine-grained MT, add switch cycles
            if (!is_fineGrained && num_of_running_threads > 0 && !running_queue[MRUThread]) {
                MRUThread = findNextRunningThread(MRUThread, threads_num, running_queue);
                cycles_num += switch_cycles;
            }
        }
    }

// Handle memory accesses and update thread states
    void handleMemoryAccess(bool* running_queue, int& num_of_running_threads, std::vector<int>& waiting_queue) {
        int removed=-1;
        if (threads[MRUThread].latency > 0){
            removed=MRUThread;
        }
        auto it = waiting_queue.begin();
        while (it != waiting_queue.end()) {
            --threads[*it].latency;
            if (threads[*it].latency <= 0) {
                running_queue[*it] = true;
                it = waiting_queue.erase(it); /// check may have error
                num_of_running_threads++;
            }
            else
                ++it;
        }
        if (removed >= 0) {
            waiting_queue.push_back(removed);
        }
    }

// Decrement latency of waiting threads and handle thread switching if necessary
    void decrementLatencyAndHandleSwitching(std::vector<int>& waiting_queue, bool* running_queue,
        int& num_of_running_threads, int& MRUThread, int threads_num, int switch_cycles, int& cycles_num) {

        // Handle thread switching
        if (num_of_running_threads > 0 && !running_queue[MRUThread]) {
            MRUThread = findNextRunningThread(MRUThread, threads_num, running_queue);
            if (!running_queue[MRUThread]) {
                cycles_num += switch_cycles;
                auto it = waiting_queue.begin();
                while (it != waiting_queue.end()) {
                    threads[*it].latency -= switch_cycles;
                    if (threads[*it].latency <= 0) {
                        running_queue[*it] = true;
                        it = waiting_queue.erase(it);/// check may have error
                        num_of_running_threads++;
                    }
                    else
                        ++it;
                }
            }
        }
    }

    void executeAllInstructions(threadClass* threads, int threads_num, bool is_fineGrained, int switch_cycles) {
        bool* running_queue = new bool[threads_num];
        std::vector<int> waiting_queue;
        int num_of_working_threads;
        int num_of_running_threads;


        initializeQueues(running_queue, threads_num, num_of_working_threads, num_of_running_threads);

        // Main execution loop
        while (num_of_working_threads > 0) {
            fetchAndExecuteInstruction(threads[MRUThread], is_fineGrained, MRUThread, switch_cycles,
                                       threads_num, cycles_num, num_of_running_threads, num_of_working_threads, waiting_queue, running_queue);
            handleMemoryAccess(running_queue, num_of_running_threads, waiting_queue);
            decrementLatencyAndHandleSwitching(waiting_queue, running_queue, num_of_running_threads,
                                               MRUThread, threads_num, switch_cycles, cycles_num);
            cycles_num++;
        }

        delete[] running_queue;
    }

};

coreClass* block_core;
coreClass* finegrained_core;

void CORE_BlockedMT() {
    block_core = new coreClass(SIM_GetThreadsNum(), false,SIM_GetSwitchCycles());
    block_core->executeAllInstructions(block_core->threads,block_core->threads_num,block_core->is_fineGrained,block_core->switch_cycles);
}

void CORE_FinegrainedMT() {
    finegrained_core = new coreClass(SIM_GetThreadsNum(),true);
    finegrained_core->executeAllInstructions(finegrained_core->threads,finegrained_core->threads_num,finegrained_core->is_fineGrained,finegrained_core->switch_cycles);
}

double CORE_BlockedMT_CPI(){
    double tmp =0;
    if (block_core->inst_num > 0)
        tmp= double (block_core->cycles_num)/ double (block_core->inst_num);
    delete block_core;
    return tmp;
}

double CORE_FinegrainedMT_CPI(){
    double tmp=0;
    if (finegrained_core->inst_num >0)
        tmp = double (finegrained_core->cycles_num)/ double (finegrained_core->inst_num);
    delete finegrained_core;
    return tmp;
}

void CORE_BlockedMT_CTX(tcontext* context, int threadid) {
    for (int i = 0; i < REGS_COUNT; ++i) {
        context[threadid].reg[i]= block_core->threads[threadid].regs.reg[i];
    }
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid) {
    for (int i = 0; i < REGS_COUNT; ++i) {
        context[threadid].reg[i]= finegrained_core->threads[threadid].regs.reg[i];
    }
}
