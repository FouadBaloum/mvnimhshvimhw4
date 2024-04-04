/* 046267 Computer Architecture - HW #4 */

#include "core_api.h"
#include "sim_api.h"

#include "vector"

class Thread{
public:
    int tid;
    int latency;
    tcontext regs;
    unsigned inst_num;
    explicit Thread(int tid=0): tid(tid), latency(0),inst_num(0){
        for (int i = 0; i < REGS_COUNT; ++i) {
            regs.reg[i] =0;
        }
    }

};

class Core{
public:
    int threads_num;
    Thread* threads;
    int mythread;
    int cycles_num;
    int inst_num_core;
    int load_latency;
    int store_latency;
    int switch_cycles;
    bool is_fineGrained;
    Core(bool is_fineGrained=false ,int switch_latency=0) : threads_num(SIM_GetThreadsNum()),
    threads(new Thread[SIM_GetThreadsNum()]),mythread(0), cycles_num(0), inst_num_core(0),
    load_latency(SIM_GetLoadLat()), store_latency(SIM_GetStoreLat()),switch_cycles(switch_latency),is_fineGrained(is_fineGrained){
        for (int i = 0; i < SIM_GetThreadsNum(); ++i) {
            threads[i] = Thread(i);
        }
    }

    ~Core(){
        delete[] threads;
    }

    void updatelatency(bool* rqueue, std::vector<int>& queue, int penalty, int& number){
        auto it = queue.begin();
        while (it != queue.end()) {
            threads[*it].latency -= penalty;
            if (threads[*it].latency <= 0) {
                rqueue[*it] = true;
                it = queue.erase(it);
                number++;
            }
            else
                ++it;
        }
    }

    void newthred(bool* rqueue){
        for (int i = 1; i < threads_num; ++i) {
            if (rqueue[(mythread+i)%threads_num]){
                mythread = (mythread+i)%threads_num;
                break;
            }
        }
    }

    void doInstruction(Instruction inst, bool* rqueue, int& numr, int& numw, int& rem){
        switch (inst.opcode) {
            case CMD_NOP:
                break;
            case CMD_ADD:
                threads[mythread].regs.reg[inst.dst_index]=
                        threads[mythread].regs.reg[inst.src1_index]+threads[mythread].regs.reg[inst.src2_index_imm];
                break;
            case CMD_ADDI:
                threads[mythread].regs.reg[inst.dst_index]=
                        threads[mythread].regs.reg[inst.src1_index]+inst.src2_index_imm;
                break;
            case CMD_SUB:
                threads[mythread].regs.reg[inst.dst_index]=
                        threads[mythread].regs.reg[inst.src1_index]-threads[mythread].regs.reg[inst.src2_index_imm];
                break;
            case CMD_SUBI:
                threads[mythread].regs.reg[inst.dst_index]=
                        threads[mythread].regs.reg[inst.src1_index]-inst.src2_index_imm;
                break;
            case CMD_HALT:
                rqueue[mythread] = false;
                numw--;
                numr--;
                break;
            case CMD_LOAD: {
                rem = mythread;
                unsigned addr = threads[mythread].regs.reg[inst.src1_index] +
                                (inst.isSrc2Imm ? inst.src2_index_imm
                                                : threads[mythread].regs.reg[inst.src2_index_imm]);
                SIM_MemDataRead(addr, &(threads[mythread].regs.reg[inst.dst_index]));
                rqueue[mythread] = false;
                threads[mythread].latency = load_latency;
                numr--;
                break;
            }
            case CMD_STORE:
                rem = mythread;
                unsigned addr = threads[mythread].regs.reg[inst.dst_index] +
                                (inst.isSrc2Imm ? inst.src2_index_imm
                                                : threads[mythread].regs.reg[inst.src2_index_imm]);
                SIM_MemDataWrite(addr, threads[mythread].regs.reg[inst.src1_index]);
                rqueue[mythread] = false;
                threads[mythread].latency = store_latency;
                numr--;
                break;
        }
    }

    void run(){
        bool* running_queue= new bool[threads_num];
        std::vector<int> waiting_queue;
        int num_of_working_threads=threads_num;
        int num_of_running_threads=threads_num;
        for (int i = 0; i < threads_num; ++i) {
            running_queue[i] = true;
        }
        while (num_of_working_threads > 0){
            int removed_thread=-1;
            if (num_of_running_threads > 0){
                if (!running_queue[mythread]){
                    newthred(running_queue);
                }
                inst_num_core++;
                Instruction inst;
                SIM_MemInstRead(threads[mythread].inst_num++,&inst,mythread);
                doInstruction(inst,running_queue,num_of_running_threads,num_of_working_threads,removed_thread);
                if (is_fineGrained){
                    mythread = (mythread+1)%threads_num;
                }
            }
            updatelatency(running_queue,waiting_queue,1,num_of_running_threads);
            if (removed_thread >=0) {
                waiting_queue.push_back(removed_thread);
            }
            if (!is_fineGrained && num_of_running_threads>0 && !running_queue[mythread]) {
                newthred(running_queue);
                cycles_num += switch_cycles;
                updatelatency(running_queue,waiting_queue,switch_cycles,num_of_running_threads);
            }
            cycles_num++;
        }
        delete[] running_queue;
    }
};

Core* blockedMT;
Core* fine_grainedMT;

void CORE_BlockedMT() {
    blockedMT = new Core(SIM_GetSwitchCycles());
    blockedMT->run();
}

void CORE_FinegrainedMT() {
    fine_grainedMT = new Core(true);
    fine_grainedMT->run();
}

double CORE_BlockedMT_CPI(){
    double  rv = (blockedMT->inst_num_core>0 ? (blockedMT->cycles_num)/(blockedMT->inst_num_core) : 0);
    delete blockedMT;
    return rv;

}

double CORE_FinegrainedMT_CPI(){
    double rv = (fine_grainedMT->inst_num_core>0 ? (fine_grainedMT->cycles_num)/(fine_grainedMT->inst_num_core) : 0);
    delete fine_grainedMT;
    return rv;
}

void CORE_BlockedMT_CTX(tcontext* context, int threadid) {
    for (int i = 0; i < REGS_COUNT; ++i) {
        context[threadid].reg[i]= blockedMT->threads[threadid].regs.reg[i];
    }
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid) {
    for (int i = 0; i < REGS_COUNT; ++i) {
        context[threadid].reg[i]= fine_grainedMT->threads[threadid].regs.reg[i];
    }
}
