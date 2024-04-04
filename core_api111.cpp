/* 046267 Computer Architecture - HW #4 */

#include "core_api.h"
#include "sim_api.h"

#include <iostream>
using namespace std;

class Thread{
public:
    int id;
    int latency;
    bool done;
    tcontext regs;
    unsigned inst_num;

    explicit Thread(int id=0): id(id), latency(0),inst_num(0) , done(false){
        for (int i = 0; i < REGS_COUNT; ++i) {
            regs.reg[i] =0;
        }
    }

};

class blockedMT {
public:
    int numThreads ;
    int loadLatency ;
    int storeLatency;
    double cycleNum;
    double instCount;
    Thread* workingThreads;
    int switch_cycles;

    blockedMT():numThreads(SIM_GetThreadsNum()), loadLatency(SIM_GetLoadLat()), storeLatency(SIM_GetStoreLat()),
                cycleNum(0),instCount(0),workingThreads(new Thread[numThreads]),switch_cycles(SIM_GetSwitchCycles()) {
        for (int i = 0; i < numThreads; i++) {
            workingThreads[i] = Thread(i);
        }
    }
    ~blockedMT(){
        delete[] workingThreads;
    }

    void executeInst() {
        int numOfWorkingThreads = numThreads;
        int threadIndex = 0;
        int busyThreadsCount = 0;
        int prevthread = threadIndex;
        while(numOfWorkingThreads) {
            Instruction currInst ;
            if(workingThreads[threadIndex].done || workingThreads[threadIndex].latency > 0) {
                //prevthread = threadIndex;
                threadIndex = (threadIndex+1)%numThreads;
                busyThreadsCount++;
                if (busyThreadsCount < numThreads){
                    continue;
                }
            }
            //cout <<  cycleNum << "	" ;
            if(busyThreadsCount >= numThreads) {
                //cout << "CMD_NOP" << endl;
                for(int i = 0 ; i < numThreads ; i++) {
                    if (workingThreads[i].latency > 0) {
                        workingThreads[i].latency--;
                    }
                }
                busyThreadsCount= 0;
                cycleNum++;
                continue;
            }
            busyThreadsCount = 0 ;

            if (prevthread != threadIndex){
                cycleNum += switch_cycles;
                for (int i = 0; i < numThreads; ++i) {
                    if (workingThreads[i].latency > 0){
                        workingThreads[i].latency -= switch_cycles;
                    }
                }
            }
            prevthread = threadIndex;
            for(int i = 0 ; i < numThreads ; i++) {
                if (workingThreads[i].latency > 0) {
                    workingThreads[i].latency--;
                }
            }
            //cout <<  threadIndex << "	" ;
            SIM_MemInstRead(workingThreads[threadIndex].inst_num++, &currInst, workingThreads[threadIndex].id);
            instCount++;
            cycleNum++;
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
                workingThreads[threadIndex].regs.reg[currInst.dst_index]=
                        workingThreads[threadIndex].regs.reg[currInst.src1_index]-currInst.src2_index_imm;
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

class fineGrainedMt {
public:
    int numThreads ;
    int loadLatency ;
    int storeLatency;
    double cycleNum;
    double instCount;
    Thread* workingThreads;

    fineGrainedMt():numThreads(SIM_GetThreadsNum()), loadLatency(SIM_GetLoadLat()), storeLatency(SIM_GetStoreLat()),
                    cycleNum(0),instCount(0),workingThreads(new Thread[numThreads]) {
        for (int i = 0; i < numThreads; i++) {
            workingThreads[i] = Thread(i);
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
            SIM_MemInstRead(workingThreads[threadIndex].inst_num++, &currInst, workingThreads[threadIndex].id);
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
                workingThreads[threadIndex].regs.reg[currInst.dst_index]=
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

blockedMT* blocked;
void CORE_BlockedMT() {
    blocked = new blockedMT();
    blocked->executeInst();
}

fineGrainedMt* fine_grained;
void CORE_FinegrainedMT() {
    fine_grained = new fineGrainedMt();
    fine_grained->executeInst();
}

double CORE_BlockedMT_CPI(){
    double rv = (blocked->instCount >0 ? blocked->cycleNum/ blocked->instCount : 0 );
    delete blocked;
    return rv;
}

double CORE_FinegrainedMT_CPI(){
    double rv = (fine_grained->instCount >0 ? fine_grained->cycleNum/ fine_grained->instCount : 0 );
    delete fine_grained;
    return rv;
}

void CORE_BlockedMT_CTX(tcontext* context, int threadid) {
    for (int i = 0; i < REGS_COUNT; ++i) {
        context[threadid].reg[i]= blocked->workingThreads[threadid].regs.reg[i];
    }
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid) {
    for (int i = 0; i < REGS_COUNT; ++i) {
        context[threadid].reg[i]= fine_grained->workingThreads[threadid].regs.reg[i];
    }
}


