/* 046267 Computer Architecture - HW #4 */

#include <vector>
#include "core_api.h"
#include "sim_api.h"

using std::vector;

class singleThread {
public:
    int tid;
    int latency;
    tcontext regs;
    unsigned inst_num;

    explicit singleThread(int tid = 0) : tid(tid), latency(0), inst_num(0) {
        for (int i = 0; i < REGS_COUNT; ++i) {
            regs.reg[i] = 0;
        }
    }
};

class singleCore {
public:
    int threads_num;
    singleThread* threads;
    int MRUThread;
    int cycles_num;
    int inst_num;
    int load_latency;
    int store_latency;
    int switch_cycles;
    bool is_fineGrained;

    singleCore(int threadNum, bool is_fineGrained, int switch_latency = 0)
            : threads_num(threadNum), threads(new singleThread[threadNum]),
              MRUThread(0), cycles_num(0), inst_num(0),
              load_latency(SIM_GetLoadLat()), store_latency(SIM_GetStoreLat()),
              switch_cycles(switch_latency), is_fineGrained(is_fineGrained) {
        for (int i = 0; i < threadNum; ++i) {
            threads[i] = singleThread(i);
        }
    }

    ~singleCore() {
        delete[] threads;
    }

    void executeAllInstructions() {
        bool* running_queue = new bool[threads_num];
        vector<int> waiting_queue;
        int num_of_running_threads = threads_num;

        initializeRunningQueue(running_queue);

        while (num_of_running_threads > 0) {
            int removed_thread = -1;

            if (num_of_running_threads > 0) {
                handleCurrentThread(running_queue, waiting_queue, num_of_running_threads, removed_thread);
            }

            handleWaitingThreads(running_queue, waiting_queue, num_of_running_threads);
            updateMRUThread(running_queue);
            cycles_num++;
        }

        delete[] running_queue;
    }

private:
    void initializeRunningQueue(bool* running_queue) {
        for (int i = 0; i < threads_num; ++i) {
            running_queue[i] = true;
        }
    }

    void handleCurrentThread(bool* running_queue, vector<int>& waiting_queue, int& num_of_running_threads, int& removed_thread) {
        if (!running_queue[MRUThread]) {
            findNextRunningThread(running_queue);
        }

        executeNextInstruction(running_queue, waiting_queue, num_of_running_threads, removed_thread);
    }

    void findNextRunningThread(bool* running_queue) {
        for (int i = 1; i < threads_num; ++i) {
            if (running_queue[(MRUThread + i) % threads_num]) {
                MRUThread = (MRUThread + i) % threads_num;
                return;
            }
        }
    }

    void executeNextInstruction(bool* running_queue, vector<int>& waiting_queue, int& num_of_running_threads, int& removed_thread) {
        inst_num++;
        Instruction inst;
        SIM_MemInstRead(threads[MRUThread].inst_num++, &inst, MRUThread);

        switch (inst.opcode) {
            case CMD_NOP:
                break;
            case CMD_ADD:
                threads[MRUThread].regs.reg[inst.dst_index] = threads[MRUThread].regs.reg[inst.src1_index] + threads[MRUThread].regs.reg[inst.src2_index_imm];
                break;
            case CMD_ADDI:
                threads[MRUThread].regs.reg[inst.dst_index] = threads[MRUThread].regs.reg[inst.src1_index] + inst.src2_index_imm;
                break;
            case CMD_SUB:
                threads[MRUThread].regs.reg[inst.dst_index] = threads[MRUThread].regs.reg[inst.src1_index] - threads[MRUThread].regs.reg[inst.src2_index_imm];
                break;
            case CMD_SUBI:
                threads[MRUThread].regs.reg[inst.dst_index] = threads[MRUThread].regs.reg[inst.src1_index] - inst.src2_index_imm;
                break;
            case CMD_HALT:
                running_queue[MRUThread] = false;
                num_of_running_threads--;
                break;
            case CMD_LOAD:
                handleLoadInstruction(inst, running_queue, waiting_queue, num_of_running_threads, removed_thread);
                break;
            case CMD_STORE:
                handleStoreInstruction(inst, running_queue, waiting_queue, num_of_running_threads, removed_thread);
                break;
        }

        if (is_fineGrained) {
            MRUThread = (MRUThread + 1) % threads_num;
        }
    }

    void handleLoadInstruction(Instruction& inst, bool* running_queue, vector<int>& waiting_queue, int& num_of_running_threads, int& removed_thread) {
        unsigned addr = threads[MRUThread].regs.reg[inst.src1_index] +
                        (inst.isSrc2Imm ? inst.src2_index_imm : threads[MRUThread].regs.reg[inst.src2_index_imm]);
        SIM_MemDataRead(addr, &(threads[MRUThread].regs.reg[inst.dst_index]));
        running_queue[MRUThread] = false;
        threads[MRUThread].latency = load_latency;
        num_of_running_threads--;
        removed_thread = MRUThread;
    }

    void handleStoreInstruction(Instruction& inst, bool* running_queue, vector<int>& waiting_queue, int& num_of_running_threads, int& removed_thread) {
        unsigned addr = threads[MRUThread].regs.reg[inst.dst_index] +
                        (inst.isSrc2Imm ? inst.src2_index_imm : threads[MRUThread].regs.reg[inst.src2_index_imm]);
        SIM_MemDataWrite(addr, threads[MRUThread].regs.reg[inst.src1_index]);
        running_queue[MRUThread] = false;
        threads[MRUThread].latency = store_latency;
        num_of_running_threads--;
        removed_thread = MRUThread;
    }

    void handleWaitingThreads(bool* running_queue, vector<int>& waiting_queue, int& num_of_running_threads) {
        auto it = waiting_queue.begin();
        while (it != waiting_queue.end()) {
            singleThread& thread = threads[*it];
            thread.latency--;

            if (thread.latency <= 0) {
                running_queue[*it] = true;
                it = waiting_queue.erase(it);
                num_of_running_threads++;
            } else {
                ++it;
            }
        }
    }

    void updateMRUThread(bool* running_queue) {
        if (!running_queue[MRUThread]) {
            if (!is_fineGrained) {
                findNextRunningThread(running_queue);
            }
        }
    }
};

singleCore* block_core;
singleCore* fine_grained_core;

void CORE_BlockedMT() {
    block_core = new singleCore(SIM_GetThreadsNum(), false, SIM_GetSwitchCycles());
    block_core->executeAllInstructions();
}

void CORE_FinegrainedMT() {
    fine_grained_core = new singleCore(SIM_GetThreadsNum(), true);
    fine_grained_core->executeAllInstructions();
}

double CORE_BlockedMT_CPI() {
    double tmp = 0;
    if (block_core->inst_num > 0)
        tmp = double(block_core->cycles_num) / double(block_core->inst_num);
    delete block_core;
    return tmp;
}

double CORE_FinegrainedMT_CPI() {
    double tmp = 0;
    if (fine_grained_core->inst_num > 0)
        tmp = double(fine_grained_core->cycles_num) / double(fine_grained_core->inst_num);
    delete fine_grained_core;
    return tmp;
}

void CORE_BlockedMT_CTX(tcontext* context, int threadid) {
    for (int i = 0; i < REGS_COUNT; ++i) {
        context[threadid].reg[i] = block_core->threads[threadid].regs.reg[i];
    }
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid) {
    for (int i = 0; i < REGS_COUNT; ++i) {
        context[threadid].reg[i] = fine_grained_core->threads[threadid].regs.reg[i];
    }
}
