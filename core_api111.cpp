//
// Created by robotic on 02/07/2023.
//

/* 046267 Computer Architecture - HW #4 */

#include "core_api.h"
#include "sim_api.h"


#include "vector"
using std::vector;


class singleThread{
public:
    int tid;
    int latency;
    bool done;
    tcontext regs;
    unsigned inst_num;
    explicit singleThread(int tid=0): tid(tid), latency(0),inst_num(0),done(false){
        for (int i = 0; i < REGS_COUNT; ++i) {
            regs.reg[i] =0;
        }
    }

};

class blockedCore{
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
    blockedCore(int threadNum,bool is_fineGrained ,int switch_latency=0) : threads_num(threadNum), threads(new singleThread[threadNum]),
     MRUThread(0), cycles_num(0), inst_num(0),
    load_latency(SIM_GetLoadLat()), store_latency(SIM_GetStoreLat()),switch_cycles(switch_latency),is_fineGrained(is_fineGrained){
         for (int i = 0; i < threadNum; ++i) {
             threads[i] = singleThread(i);
         }
     }
    ~blockedCore(){
         delete[] threads;
     }

     void executeAllInstructions(){
         bool* running_queue= new bool[threads_num];
         vector<int> waiting_queue;
         int num_of_working_threads=threads_num;
         int num_of_running_threads=threads_num;
         for (int i = 0; i < threads_num; ++i) {
             running_queue[i] = true;
         }
         while (num_of_working_threads > 0){
             int removed_thread=-1;
             if (num_of_running_threads > 0){
                 if (!running_queue[MRUThread]){
                     for (int i = 1; i < threads_num; ++i) {
                         if (running_queue[(MRUThread+i)%threads_num]){
                             MRUThread = (MRUThread+i)%threads_num;
                             break;
						 }
                     }
                 }
                 inst_num++;
                 Instruction inst;
                 SIM_MemInstRead(threads[MRUThread].inst_num++,&inst,MRUThread);
                 switch (inst.opcode) {
                     case CMD_NOP:
                         break;
                     case CMD_ADD:
                         threads[MRUThread].regs.reg[inst.dst_index]=
                                 threads[MRUThread].regs.reg[inst.src1_index]+threads[MRUThread].regs.reg[inst.src2_index_imm];
                         break;
                     case CMD_ADDI:
                         threads[MRUThread].regs.reg[inst.dst_index]=
                                 threads[MRUThread].regs.reg[inst.src1_index]+inst.src2_index_imm;
                         break;
                     case CMD_SUB:
                         threads[MRUThread].regs.reg[inst.dst_index]=
                                 threads[MRUThread].regs.reg[inst.src1_index]-threads[MRUThread].regs.reg[inst.src2_index_imm];
                         break;
                     case CMD_SUBI:
                         threads[MRUThread].regs.reg[inst.dst_index]=
                                 threads[MRUThread].regs.reg[inst.src1_index]-inst.src2_index_imm;
                         break;
                     case CMD_HALT:
                         running_queue[MRUThread] = false;
                         num_of_working_threads--;
                         num_of_running_threads--;
                         break;
                     case CMD_LOAD: {
                         removed_thread = MRUThread;
                         unsigned addr = threads[MRUThread].regs.reg[inst.src1_index] +
                                         (inst.isSrc2Imm ? inst.src2_index_imm
                                                         : threads[MRUThread].regs.reg[inst.src2_index_imm]);
                         SIM_MemDataRead(addr, &(threads[MRUThread].regs.reg[inst.dst_index]));
                         running_queue[MRUThread] = false;
                         threads[removed_thread].latency = load_latency;
                         num_of_running_threads--;
                         break;
                     }
                     case CMD_STORE:
                         removed_thread = MRUThread;
                         unsigned addr = threads[MRUThread].regs.reg[inst.dst_index] +
                                         (inst.isSrc2Imm ? inst.src2_index_imm
                                                         : threads[MRUThread].regs.reg[inst.src2_index_imm]);
                         SIM_MemDataWrite(addr, threads[MRUThread].regs.reg[inst.src1_index]);
                         running_queue[MRUThread] = false;
                         threads[removed_thread].latency = store_latency;
                         num_of_running_threads--;
                         break;

                 }
                 if (is_fineGrained ){
                    MRUThread = (MRUThread+1)%threads_num;
                 }
             }
             auto it = waiting_queue.begin();
             while (it != waiting_queue.end()){
                 --threads[*it].latency;
                 if (threads[*it].latency <=0 ){
                     running_queue[*it] = true;
                     it =waiting_queue.erase(it); /// check may have error
                     num_of_running_threads++;
                 }
                 else
                     ++it;

             }
             if (removed_thread >=0) {
                 waiting_queue.push_back(removed_thread);
             }
             if (num_of_running_threads > 0) {
                 if (!running_queue[MRUThread]) {
                     if (!is_fineGrained) {
                         for (int i = 1; i < threads_num; ++i) {
                             if (running_queue[(MRUThread+i)%threads_num]) {
                                 MRUThread = (MRUThread + i) % threads_num;
                                 break;
                             }
                         }
                         cycles_num += switch_cycles;
                         it = waiting_queue.begin();
                         while (it != waiting_queue.end()) {
                             threads[*it].latency -= switch_cycles;
                             if (threads[*it].latency <= 0) {
                                 running_queue[*it] = true;
                                 it = waiting_queue.erase(it);/// check may have error
                                 num_of_running_threads++;
                             } else
                                 ++it;

                         }
                     }
                 }
             }
             cycles_num++;
         }
        delete[] running_queue;
     }
};

class fineGrainedMt {
public:
    int numThreads ;
    int loadLatency ;
    int storeLatency;
    double cycleNum;
    double instCount;
    singleThread* workingThreads;

    fineGrainedMt():numThreads(SIM_GetThreadsNum()), loadLatency(SIM_GetLoadLat()), storeLatency(SIM_GetStoreLat()),
                    cycleNum(0),instCount(0),workingThreads(new singleThread[numThreads]) {
        for (int i = 0; i < numThreads; i++) {
            workingThreads[i] = singleThread(i);
        }
    }
    ~fineGrainedMt(){
        delete[] workingThreads;
    }

    void executeInst() {
        int numOfWorkingThreads = numThreads;
        int threadIndex = 0;
        int busyThreadsCount = 0;
        while(numOfWorkingThreads) {
            Instruction currInst ;
            if(workingThreads[threadIndex].done || workingThreads[threadIndex].latency != 0) {
                threadIndex = (threadIndex+1)%numThreads;
                busyThreadsCount++;
                if (busyThreadsCount < numThreads){
                    continue;
                }
            }
            //cout <<  cycleNum << "	" ;
            cycleNum++;
            if(busyThreadsCount >= numThreads) {
                //cout << "CMD_NOP" << endl;
                for(int i = 0 ; i < numThreads ; i++) {
                    if (workingThreads[i].latency != 0) {
                        workingThreads[i].latency--;
                    }
                }
                busyThreadsCount= 0;
                continue;
            }
            busyThreadsCount = 0 ;

            for(int i = 0 ; i < numThreads ; i++) {
                if (workingThreads[i].latency != 0) {
                    workingThreads[i].latency--;
                }
            }

            //cout <<  threadIndex << "	" ;
            SIM_MemInstRead(workingThreads[threadIndex].inst_num++, &currInst, workingThreads[threadIndex].tid);
            instCount++;

            if(currInst.opcode == CMD_NOP) {
                //cout << "CMD_NOP" << endl;
            }
            if(currInst.opcode == CMD_ADD) {
                //cout << "CMD_ADD" <<endl;
                workingThreads[threadIndex].regs.reg[currInst.dst_index]=
                        workingThreads[threadIndex].regs.reg[currInst.src1_index]+
                        workingThreads[threadIndex].regs.reg[currInst.src2_index_imm];
            }
            if(currInst.opcode == CMD_SUB) {
                //cout << "CMD_SUB" << endl;
                workingThreads[threadIndex].regs.reg[currInst.dst_index]=
                        workingThreads[threadIndex].regs.reg[currInst.src1_index]-
                        workingThreads[threadIndex].regs.reg[currInst.src2_index_imm];
            }
            if(currInst.opcode == CMD_ADDI) {
                //cout << "CMD_ADDI" <<endl;
                workingThreads[threadIndex].regs.reg[currInst.dst_index]=
                        workingThreads[threadIndex].regs.reg[currInst.src1_index]+
                        currInst.src2_index_imm;
            }
            if(currInst.opcode == CMD_SUBI) {
                //cout << "CMD_SUBI" << endl;
                workingThreads[threadIndex].regs.reg[currInst.src1_index] - currInst.src2_index_imm;
            }
            if(currInst.opcode == CMD_LOAD) {
                workingThreads[threadIndex].latency = loadLatency;
                unsigned addr = workingThreads[threadIndex].regs.reg[currInst.src1_index] +
                                (currInst.isSrc2Imm ? currInst.src2_index_imm
                                                    : workingThreads[threadIndex].regs.reg[currInst.src2_index_imm]);
                SIM_MemDataRead(addr, &(workingThreads[threadIndex].regs.reg[currInst.dst_index]));
                //cout << "CMD_LOAD" <<endl;
            }
            if(currInst.opcode == CMD_STORE) {
                workingThreads[threadIndex].latency = storeLatency;
                unsigned addr = workingThreads[threadIndex].regs.reg[currInst.dst_index] +
                                (currInst.isSrc2Imm ? currInst.src2_index_imm
                                                    : workingThreads[threadIndex].regs.reg[currInst.src2_index_imm]);
                SIM_MemDataWrite(addr, workingThreads[threadIndex].regs.reg[currInst.src1_index]);
                //cout << "CMD_STORE" << endl;
            }
            if(currInst.opcode == CMD_HALT) {
                workingThreads[threadIndex].done = true;
                //cout << "CMD_HALT" <<endl;
            }
            threadIndex = (threadIndex+1) % numThreads;
            numOfWorkingThreads =0 ;
            for(int i = 0 ; i < numThreads ; i++) {
                if (!workingThreads[i].done) {
                    numOfWorkingThreads++;
                }
            }
        }
        //cout << "number of cycles: " << cycleNum<<endl;
    }
};

blockedCore* block_core;
fineGrainedMt* fine_grained_core;

void CORE_BlockedMT() {
    block_core = new blockedCore(SIM_GetThreadsNum(),true,SIM_GetSwitchCycles());
    block_core->executeAllInstructions();
}

void CORE_FinegrainedMT() {
    fine_grained_core = new fineGrainedMt();
    fine_grained_core->executeInst();
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
    if (fine_grained_core->instCount >0)
         tmp = double (fine_grained_core->cycleNum)/ double (fine_grained_core->instCount);
    delete fine_grained_core;
    return tmp;
}

void CORE_BlockedMT_CTX(tcontext* context, int threadid) {
    for (int i = 0; i < REGS_COUNT; ++i) {
        context[threadid].reg[i]= block_core->threads[threadid].regs.reg[i];
    }
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid) {
    for (int i = 0; i < REGS_COUNT; ++i) {
        context[threadid].reg[i]= fine_grained_core->workingThreads[threadid].regs.reg[i];
    }
}
