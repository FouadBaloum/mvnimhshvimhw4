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

    void initializeQueues(bool* running_queue, std::vector<int>& waiting_queue, int threads_num,
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
                                    int threads_num, int& cycles_num, int& num_of_running_threads, std::vector<int>& waiting_queue, bool* running_queue) {
        if (!running_queue[MRUThread]) {
            // Find the next available running thread
            MRUThread = findNextRunningThread(MRUThread, threads_num, running_queue);
        }

        // If the thread is running, execute the next instruction
        if (running_queue[MRUThread]) {
            Instruction inst;
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
                    break;
                case CMD_LOAD: {
                    unsigned addr = thread.regs.reg[inst.src1_index] +
                                    (inst.isSrc2Imm ? inst.src2_index_imm : thread.regs.reg[inst.src2_index_imm]);
                    SIM_MemDataRead(addr, &(thread.regs.reg[inst.dst_index]));
                    thread.latency = SIM_GetLoadLat(); // Set thread latency for memory load
                    running_queue[MRUThread] = false;
                    num_of_running_threads--;
                    waiting_queue.push_back(MRUThread);
                    break;
                }
                case CMD_STORE: {
                    unsigned addr = thread.regs.reg[inst.dst_index] +
                                    (inst.isSrc2Imm ? inst.src2_index_imm : thread.regs.reg[inst.src2_index_imm]);
                    SIM_MemDataWrite(addr, thread.regs.reg[inst.src1_index]);
                    thread.latency = SIM_GetStoreLat(); // Set thread latency for memory store
                    running_queue[MRUThread] = false;
                    num_of_running_threads--;
                    waiting_queue.push_back(MRUThread);
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
    void handleMemoryAccess(threadClass& thread, int& num_of_running_threads, vector<int>& waiting_queue, int& removed_thread) {
        if (thread.latency > 0) {
            // Decrement latency for threads waiting for memory access
            thread.latency--;

            // If latency reaches 0, mark thread as ready and remove from waiting_queue
            if (thread.latency == 0) {
                num_of_running_threads++;
                thread.latency = -1; // Mark thread as ready
                removed_thread = thread.tid;
                for (auto it = waiting_queue.begin(); it != waiting_queue.end(); ++it) {
                    if (*it == removed_thread) {
                        waiting_queue.erase(it);
                        break;
                    }
                }
            }
        }
    }

// Decrement latency of waiting threads and handle thread switching if necessary
    void decrementLatencyAndHandleSwitching(std::vector<int>& waiting_queue, bool* running_queue,
                                            int& num_of_running_threads, int& MRUThread, int threads_num, int switch_cycles, int& cycles_num) {
        // Decrement latency of waiting threads
        for (auto& thread_id : waiting_queue) {
            threadClass& thread = threads[thread_id];
            if (thread.latency > 0) {
                thread.latency--;
            }
        }

        // Handle thread switching
        if (num_of_running_threads > 0 && !running_queue[MRUThread]) {
            MRUThread = findNextRunningThread(MRUThread, threads_num, running_queue);
            if (!running_queue[MRUThread]) {
                // If still no running thread found, add switch cycles
                cycles_num += switch_cycles;
                // Decrement switch cycles from waiting threads
                for (auto& thread_id : waiting_queue) {
                    threadClass& thread = threads[thread_id];
                    if (thread.latency > 0) {
                        thread.latency -= switch_cycles;
                        if (thread.latency <= 0) {
                            // If latency reaches 0, mark thread as ready and remove from waiting_queue
                            num_of_running_threads++;
                            thread.latency = -1; // Mark thread as ready
                            for (auto it = waiting_queue.begin(); it != waiting_queue.end(); ++it) {
                                if (*it == thread_id) {
                                    waiting_queue.erase(it);
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    void executeAllInstructions(threadClass* threads, int threads_num, bool is_fineGrained, int switch_cycles) {
        bool* running_queue = new bool[threads_num];
        std::vector<int> waiting_queue;
        int num_of_working_threads;
        int num_of_running_threads;
        int MRUThread = 0;
        int cycles_num = 0;

        initializeQueues(running_queue, waiting_queue, threads_num, num_of_working_threads, num_of_running_threads);

        // Main execution loop
        while (num_of_working_threads > 0) {
            fetchAndExecuteInstruction(threads[MRUThread], is_fineGrained, MRUThread, switch_cycles,
                                       threads_num, cycles_num, num_of_running_threads, waiting_queue, running_queue);
            handleMemoryAccess(threads[MRUThread], num_of_running_threads, waiting_queue);
            decrementLatencyAndHandleSwitching(waiting_queue, running_queue, num_of_running_threads,
                                               MRUThread, threads_num, switch_cycles, cycles_num);
            cycles_num++;
        }

        delete[] running_queue;
    }

}

coreClass* block_core;
coreClass* finegrained_core;

void CORE_BlockedMT() {
    block_core = new coreClass(SIM_GetThreadsNum(), false,SIM_GetSwitchCycles());
    block_core->executeAllInstructions();
}

void CORE_FinegrainedMT() {
    finegrained_core = new coreClass(SIM_GetThreadsNum(),true);
    finegrained_core->executeAllInstructions();
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
