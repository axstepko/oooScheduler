#include <stdio.h>
#include <stdlib.h>
#include <queue>
#include <deque>
#include <iostream>

/**
 * @file p1.cpp
 * @author A. Stepko (alex@axstepko.com)
 * @brief Implements dynamic (trace) scheduling of RISC-style OoO pipeline of variable width.
 * @version 0.1
 * @date 2023-10-07
 *
 * @copyright Copyright (c) 2023
 *
 */

using namespace std;

#define NOP NULL

// Enables verbose program output in specific stages:
#define DEBUG_MODE //!< Main function, helper functions, etc.
#define FETCH_DEBUG
// #define DECODE_DEBUG
#define RENAME_DEBUG
#define DISPATCH_DEBUG
#define ISSUE_DEBUG
#define WRITEBACK_DEBUG
#define COMMIT_DEBUG

unsigned int ISSUE_WIDTH;           //!< User-configured parameter for machine width
unsigned int PREG_COUNT;            //!< User-configured parameter for the number of physical registers.
const unsigned int AREG_COUNT = 32; //!< Constant architectural width

/**
 * @brief Acts as the instruction memory ROB, and commit controller.
 *
 * @remark uses instrucion form of <iType> <op1> <op2> <op3>
 *
 */
typedef struct iRecord_t
{
    // INSTRUCTION MEMORY:
    char iType; //!< Instruction type
    int op1;    //!< First field read (destination register)
    int op2;    //!< Second field read (source 1)
    int op3;    //!< Third field read (source 2)

    // RENAMED REGISTERS:
    int op1_r; //!< Renamed first field read (destination register)
    int op2_r; //!< Renamed second field read (source 1)
    int op3_r; //!< Renamed third field read (source 2)

    // Done Flag:
    unsigned int instrComplete;

    unsigned int fetchIndex; //!< Index by which the instruction was fetched. fetchIndex = 0 is the first instruction fetched, fetchIndex = 1 is the second, etc.

    // Cycle tracker:
    unsigned int F, Dc, R, Di, IS, W, C; // Cycle where the thing completed
} iRecord_t;

/**
 * @brief Stores the location of instructions in the pipeline
 *
 */
typedef struct frontEndPipe_t
{
    iRecord_t *F;  //!< Fetch stage
    iRecord_t *Dc; //!< Decode stage
    iRecord_t *R;  //!< Rename stage
    iRecord_t *Di; //!< Dispatch stage
    iRecord_t *IS; //!< Issue stage
    iRecord_t *W;  //!< Writeback stage
    iRecord_t *C;  //!< Commit stage
} frontEndPipe_t;

/**
 * @brief Single-line entry for the reorder buffer.
 *
 */
typedef struct ROB_t
{
    iRecord_t *instr; //!< Full instruction record thus far
    bool ready;      //!< Flag for whether or not the instruction was completed. Marked in writeback
    bool committed;  //!< Flag for whether or not the instruction was committed
} ROB_t;

/**
 * @brief Single-line element for the issue queue.
 *
 */
typedef struct iqEntry_t
{
    iRecord_t *instr; //!< Contains op1, op2, op3, use renamed versions
    bool src1_ready;  //!< op2_r ready
    bool src2_ready;  //!< op3_r ready

    unsigned int age; //!< Age of the IQ entry tracked by iqAge global variable
} iqEntry_t;

iRecord_t instructions[256];      //!< Contains all instructions possible in the processor.
frontEndPipe_t *thePipelineState; //<! Pipeline information. Record of iRecord_t's

unsigned int *mapTable;       //!< System map table, depth of PREG_COUNT
unsigned int *readyTable;     //!< System ready table, size of PREG_COUNT elements
deque<unsigned int> freeList; //!< System free list of pReg's
deque<ROB_t *> reorderBuff;   //!< System re-order buffer (ROB). Points to current instruction of execution
deque<iqEntry_t> issueQueue;  //!< System issue queue. Linked with ROB
unsigned long long iqAge = 0; //!< IQ age tracker

deque<iRecord_t *> wBQueue; //!< Queue between IS and Writeback to handle pulls from the IQ in IS

deque<iRecord_t *> commitQueue; //!< Queue between WB and C to prompt commit to look at stuff.

/**
 * @brief Reads instructions from a file and places into instruction memory.
 *
 * @return int Number of instructions in the sytesm
 */
int initInstructions()
{
    unsigned int instIndex = -1; // Line '-1' is the entry to determine number of pReg's and issue width
    char lineBuff[32];

    // Open the file:
    FILE *instrSet = fopen("test.in", "r");
    if (instrSet == NULL)
    {
        perror("Error reading file\n");
        return 100;
    }
    while (fgets(lineBuff, sizeof(lineBuff), instrSet))
    {
        if (instIndex == -1)
        {
            sscanf(lineBuff, "%u, %u", &PREG_COUNT, &ISSUE_WIDTH);
#ifdef DEBUG_MODE
            printf("Detected physical register count of %u, Issue width of %u\n", PREG_COUNT, ISSUE_WIDTH);
#endif
        }
        else
            sscanf(lineBuff, "%c,%d,%d,%d", &instructions[instIndex].iType, &instructions[instIndex].op1, &instructions[instIndex].op2, &instructions[instIndex].op3);
        instIndex++;
    }

#ifdef DEBUG_MODE
    printf("Detected instruction count of %d\n", instIndex);
#endif
    fclose(instrSet);

    return instIndex;
}

/**
 * @brief Print records from the commit buffer
 *
 * @param pipe
 * @param numInstr
 */
void printRecords(unsigned int numInstr)
{
    FILE *outputFile = fopen("output.txt", "w");

#ifdef DEBUG_MODE
    printf("idx: F, Dc, R, Di, IS, WB, C\n");
#endif
    for (int i = 0; i < numInstr; i++)
    {
#ifdef DEBUG_MODE
        printf("%d: %d, %d, %d, %d, %d, %d, %d\n", i, instructions[i].F, instructions[i].Dc, instructions[i].R, instructions[i].Di, instructions[i].IS, instructions[i].W, instructions[i].C);
#endif
    }

    fclose(outputFile);
}

/**
 * @brief Commits instructions IN ORDER by examining the head of the ROB
 *
 * @param pipe Pipeline state of the machine
 * @param committedInsts Current number of committed instructions
 * @param cycle Current cycle of the machine
 * @return unsigned int New number of committed instructions
 */
unsigned int commit(frontEndPipe_t *pipe, unsigned int committedInsts, unsigned int cycle)
{
    int commitPull = 0;
#ifdef COMMIT_DEBUG
    printf("-- commit --\n");
    if (!reorderBuff.empty())
    {
        printf("ROB HEAD %d, depth=%lu\n", reorderBuff.front()->instr->op1_r, reorderBuff.size());
        printf("Commit Q size: %lu\n", commitQueue.size());
    }

    for (const ROB_t *robEntry : reorderBuff)
    {
        if (robEntry->ready == true)
        {
            cout << "-> op1_r for an element in reorderBuff: " << robEntry->instr->op1_r << endl;
        }
    }
#endif
    for (int i = 0; i < ISSUE_WIDTH; i++)
    {
        // Look at head of ROB and see if it can be committed:
        if (!reorderBuff.empty() && reorderBuff.front()->ready == true && commitPull < ISSUE_WIDTH)
        {
            printf("ROB %d ready=%d\n", reorderBuff.front()->instr->op1_r, reorderBuff.front()->ready);
            printf("COMMIT PULL %d\n", commitPull);
            // Pull from front of commit queue
            pipe[commitPull].C = reorderBuff.front()->instr;
            pipe[commitPull].C->C = cycle;
// #ifdef COMMIT_DEBUG
//             printf("Committed on %d cycle %d\n", commitQueue.front()->op1_r, pipe[commitPull].C->C);
// #endif
//             commitQueue.pop_front();

#ifdef COMMIT_DEBUG
            printf("poping %d from ROB\n", reorderBuff.front()->instr->op1_r);
            printf("commit queue depth: %lu\n", commitQueue.size());
#endif

            freeList.push_back(reorderBuff.front()->instr->op1_r); // Reclaim register on the free list
            // readyTable[reorderBuff.front().instr.op1_r] = false;

            reorderBuff.pop_front(); // May clear it out, need to be careful here
            if (!reorderBuff.empty())
            {
                // reorderBuff.front().instr.op1_r = 111;
                printf("popped front. ROB depth %lu\n", reorderBuff.size());
                printf("new ROB FRONT: %d\n", reorderBuff.front()->instr->op1_r);
                for (const ROB_t *robEntry : reorderBuff)
                {
                    if (robEntry->ready == true)
                        cout << "AFTER pop_front() reorderBuff: " << robEntry->instr->op1_r << endl;
                }
            }
            else
            {
                printf("ROB is empty. depth %lu", reorderBuff.size());
            }

            committedInsts++;

            for (const ROB_t *robEntry : reorderBuff)
            {
                if (robEntry->ready)
                {
                    cout << "op1_r for an element in reorderBuff: " << robEntry->instr->op1_r << endl;
                }
            }
            commitPull++;
        }
        else if (!reorderBuff.empty())
        {
            printf("reg not ready ROB %d ready=%d\n", reorderBuff.front()->instr->op1_r, reorderBuff.front()->ready);
        }
    }

    return committedInsts;
}

/**
 * @brief Pulls instructions, if available, from the WB queue and marks them ready for completion
 *
 * @param pipe
 * @param cycle
 */
void writeback(frontEndPipe_t *pipe, unsigned int cycle)
{
    iRecord_t *tempRec;
#ifdef WRITEBACK_DEBUG
    printf("-- writeback --\n");
#endif

    for (int i = 0; i < wBQueue.size(); i++)
    {
        printf("%c dest %d", wBQueue[i]->iType, wBQueue[i]->op1_r);
    }
    for (int i = 0; i < ISSUE_WIDTH; i++)
    {
        if (wBQueue.empty() == false)
        {
            // Pull element from front of wbQueue
            tempRec = wBQueue.front();
            tempRec->W = cycle;  // Mark completion time
            wBQueue.pop_front(); // Remove entry from deque
#ifdef WRITEBACK_DEBUG
            printf("Popped %c dest %d from wbQueue\n", tempRec->iType, tempRec->op1_r);
#endif

            // Send instruction to commit:
            commitQueue.push_back(tempRec);

            // Search through ROB for destreg to mark as complete. Only mark one entry as complete.
            if (reorderBuff.empty() == false)
            {
                for (int j = 0; j < reorderBuff.size(); j++)
                {
                    if (reorderBuff.at(j)->instr->op1_r == tempRec->op1_r)
                    {
                        // Mark instruction as complete
                        reorderBuff.at(j)->ready = true;

                        // Update the ready table for the destination register
                        readyTable[tempRec->op1_r] = true;
#ifdef WRITEBACK_DEBUG
                        printf("Marked ROB[%d]: %c dest %d as complete.\n", j, reorderBuff.at(j)->instr->iType, reorderBuff.at(j)->instr->op1_r);
#endif
                        // break;
                    }
                }
            }
        }
        else
        {
            pipe[i].C = NOP; // Othwerise commit gets a NOP
        }
    }
    if (!commitQueue.empty())
    {
        for (int i = 0; i < commitQueue.size(); i++)
        {
            printf("%c %d\n", commitQueue[i]->iType, commitQueue[i]->op1_r);
        }
    }
}

// void issue(frontEndPipe_t *pipe, unsigned int cycle)
// {
// #ifdef ISSUE_DEBUG
//     printf("-- issue -- \n");
// #endif
//     unsigned int IQpull = 0; //! Number of pulls from the IQ. IQpull < ISSUE_WIDTH.

//     for (int i = 0; i < ISSUE_WIDTH; i++)
//     {
//         if (issueQueue.empty() == false)
//         {
//             for (const iqEntry_t &iqEntry : issueQueue)
//             {
//                 cout << "-> op1_r for an element in iq " << iqEntry.instr->op1_r << endl;
//             }

//             int j = issueQueue.size() - 1; //!< Iterator to start at last element

//             while (j >= 0 && IQpull < ISSUE_WIDTH)
//             {
// #ifdef ISSUE_DEBUG
//                 printf("IS check IQ[%d]:\n", j);
// #endif
//                 if (issueQueue[j].src1_ready && issueQueue[j].src2_ready)
//                 {
// #ifdef ISSUE_DEBUG
//                     printf("PULL %d: Send IQ entry %d to writeback\n", IQpull, j);
//                     printf("Send itype %c dest %d to WB in IW %d\n", issueQueue[j].instr->iType, issueQueue[j].instr->op1_r, i);
//                     printf("IS cycle %d\n", cycle);
// #endif
//                     issueQueue[j].instr->IS = cycle; // Mark cycle of completion
//                     wBQueue.push_back(issueQueue[j].instr);

//                     issueQueue.erase(issueQueue.begin() + j);
//                     IQpull++;
//                     break;
//                 }
//                 j--;
//             }
//         }
// #ifdef ISSUE_DEBUG
//         else
//         {
//             printf("IQ is empty.\n");
//         }
// #endif
//     }
// }

void issue(frontEndPipe_t *pipe, unsigned int cycle)
{
#ifdef ISSUE_DEBUG
    printf("-- issue -- \n");
#endif

    unsigned int IQpull = 0;     // Number of total pulls from the IQ. IQpull < ISSUE_WIDTH.
    deque<unsigned int> wakeupQ; //!< List of elements to search in the IQ for wakeup

    for (int i = 0; i < ISSUE_WIDTH; i++)
    {
        if (!issueQueue.empty())
        {
            int j = 0; // Start with the first element in the issue queue

            while (j < issueQueue.size() && IQpull < ISSUE_WIDTH)
            {
#ifdef ISSUE_DEBUG
                printf("check IQ[%d]:\n", j);
#endif
                if (issueQueue[j].src1_ready && issueQueue[j].src2_ready)
                {
#ifdef ISSUE_DEBUG
                    printf("PULL %d: Send IQ entry %d to writeback\n", IQpull, j);
                    printf("Send itype %c dest %d to WB in IW %d\n", issueQueue[j].instr->iType, issueQueue[j].instr->op1_r, i);
                    printf("IS cycle %d\n", cycle);
#endif
                    issueQueue[j].instr->IS = cycle; // Mark cycle of completion
                    wBQueue.push_back(issueQueue[j].instr);

                    // Add wakeup elements to the local deque of wakeup registers:
                    wakeupQ.push_back(issueQueue[j].instr->op1_r);
                    wakeupQ.push_back(issueQueue[j].instr->op2_r);

                    issueQueue.erase(issueQueue.begin() + j);
                    IQpull++;
                }
                else
                {
                    j++; // Move to the next element in the issue queue
                }
            }
        }
#ifdef ISSUE_DEBUG
        else
        {
            printf("IQ is empty.\n");
        }
#endif
    }

    IQpull--; // Offset IQpull by 1 to be used in looking through the IQ again

    // Wakeup dependent instructions by finding their entries in the IQ:
    for (int i = 0; i < wakeupQ.size(); i++)
    {
        for (int j = 0; j < issueQueue.size(); j++) // Fixed the loop counter from 'i' to 'j'
        {
            if (issueQueue[j].instr->op2_r == wakeupQ[i])
            {
                printf("Woke up register src1 (p%d)\n", issueQueue[j].instr->op2_r);
                issueQueue[j].src1_ready = true;
            }
            if (issueQueue[j].instr->op3_r == wakeupQ[i])
            {
                printf("Woke up register src2 (p%d)\n", issueQueue[j].instr->op3_r);
                issueQueue[j].src2_ready = true;
            }
        }
    }
    wakeupQ.clear();

    printf("IQpull=%u\n", IQpull);
}

/**
 * @brief Dispatches instructions into the issue queue
 *
 * @param pipe Current state of the machine
 * @param cycle Current cycle of the machine
 */
void dispatch(frontEndPipe_t *pipe, unsigned int cycle)
{
#ifdef DISPATCH_DEBUG
    printf("-- dispatch --\n");
#endif
    iqEntry_t *slotEntry = (iqEntry_t *)malloc(sizeof(iqEntry_t)); //!< Slot to place into the issue queue
    ROB_t *ROBentry = (ROB_t *)malloc(sizeof(ROB_t));              //!< Slot to place into the FIFO ROB

    for (int i = 0; i < ISSUE_WIDTH; i++)
    {
        // Pipeline has already been advanced. data in Di is valid to use to generate IQ entry.

        if (pipe[i].Di != NOP)
        {
            pipe[i].Di->Di = cycle;
            slotEntry->instr = pipe[i].Di; // Point memory address of IQ entry to the in-order pipeline

            // Check instruction type to see if src1, src2 both need to be marked as ready or otherwise:
#ifdef DISPATCH_DEBUG
            printf("dispatch sees %c\n", pipe[i].Di->iType);
#endif

            switch (pipe[i].Di->iType)
            {
            case 'R': // op1 produces, all others from table
                slotEntry->src1_ready = readyTable[pipe[i].Di->op2_r];
                slotEntry->src2_ready = readyTable[pipe[i].Di->op3_r];
                readyTable[pipe[i].Di->op1_r] = false;
                break;
            case 'I': // op1 produces, op2 is dynamic, op3 is immediate.
                slotEntry->src1_ready = readyTable[pipe[i].Di->op2_r];
                slotEntry->src2_ready = true;          // op3 is immediate and therefore always ready.
                readyTable[pipe[i].Di->op1_r] = false; // op1 producer marked false
                break;
            case 'L':                         // op1 produces, op2 is immediate, op3 is dynamic
                slotEntry->src1_ready = true; // op2 is immediate and therefore always ready.
                slotEntry->src2_ready = readyTable[pipe[i].Di->op3_r];
                readyTable[pipe[i].Di->op1_r] = false; // op1 producer marked false
                break;
            case 'S': // Consumer only. All values ready to issue.
                slotEntry->src1_ready = true;
                slotEntry->src2_ready = true;
                break;
            default:
#ifdef DISPATCH_DEBUG
                printf("!! ERROR !!  ERRONEOUS TYPE PROVIDED IN ISSUE STAGE\n");
#endif
                break;
            };

            // Idiot check to always mark register 0 as ready:
            if (pipe[i].Di->op1_r == 0)
            {
                readyTable[pipe[i].Di->op1_r] = 0;
            }
            slotEntry->age = iqAge;
            iqAge++;

            // Generate ROB entry:
            ROBentry->instr = slotEntry->instr;
            ROBentry->committed = false;
            ROBentry->ready = false;

            // Send to the IQ and ROB:
            issueQueue.push_back(*(slotEntry));
            reorderBuff.push_back(ROBentry);
#ifdef DISPATCH_DEBUG
            printf("dispatch pushed %d to ROB, size now %lu. IQ size now %lu. Completed on cycle %d\n", reorderBuff.back()->instr->op1_r, reorderBuff.size(), issueQueue.size(), pipe[i].Di->Di);

            printf("target register for ROB is %d\n", reorderBuff.back()->instr->op1_r);
#endif
        }
        else
        {
#ifdef DISPATCH_DEBUG
            printf("Dispatch NOP\n");
#endif
        }
    }
#ifdef DISPATCH_DEBUG
    printf("--end of dispatch--\n\n");
#endif
}
// All loads wait until all older stores. Additional resource for what a load needs to leave the IQ
unsigned int rename(frontEndPipe_t *pipe, unsigned int cycle)
{
    // iRecord_t *batch = (iRecord_t *)calloc(ISSUE_WIDTH, sizeof(iRecord_t));
    unsigned int renameStall = false;
    // Rename (map) the architectural registers to their avaialble physical registers:
    for (int i = 0; i < ISSUE_WIDTH; i++)
    {
        pipe[i].Di = pipe[i].R; // Advance pipeline, containing renamed registers, to dispatch
        if (pipe[i].R != NOP)
        {
#ifdef RENAME_DEBUG
            printf("Rename %d advance\n", i);
            printf("rename sees %c\n", pipe[i].R->iType);
#endif
            pipe[i].R->R = cycle; // Mark cycle of completion.

            if (!freeList.empty())
            {
                // Examine instruction type to determine how many things need renamed:
                switch (pipe[i].R->iType)
                {
                case 'R': // op1 produce. op2, op3 dynamic
#ifdef RENAME_DEBUG
                    printf("R-type rename\n");
#endif
                    // Get op2 op3 from map table:
                    pipe[i].R->op2_r = mapTable[pipe[i].R->op2];
                    pipe[i].R->op3_r = mapTable[pipe[i].R->op3];
                    if (pipe[i].R->op1 != 0)
                    {
                        pipe[i].R->op1_r = freeList.front();         // Assign producer from freelist
                        freeList.pop_front();                        // Remove from free list
                        readyTable[pipe[i].R->op1_r] = false;        // Mark not ready
                        mapTable[pipe[i].R->op1] = pipe[i].R->op1_r; // Update map table value
                    }

                    break;
                case 'I':
#ifdef RENAME_DEBUG
                    printf("I-type rename\n");
#endif
                    pipe[i].R->op1_r = freeList.front();         // Assign producer from freelist
                    freeList.pop_front();                        // Remove from free list
                    readyTable[pipe[i].R->op1_r] = false;        // Mark not ready
                    mapTable[pipe[i].R->op1] = pipe[i].R->op1_r; // Update map table value

                    pipe[i].R->op2_r = mapTable[pipe[i].R->op2]; // Get op2 from map table:

                    pipe[i].R->op3_r = pipe[i].R->op3; // op3 is passed directly
                    break;

                case 'L':
#ifdef RENAME_DEBUG
                    printf("L-type rename\n");
#endif
                    if (pipe[i].R->op1 != 0)
                    {
                        pipe[i].R->op1_r = freeList.front(); // Assign producer from freelist
                        freeList.pop_front();                // Remove from free list

                        readyTable[pipe[i].R->op1_r] = false;        // Mark not ready
                        mapTable[pipe[i].R->op1] = pipe[i].R->op1_r; // Update map table value
                    }
                    // op2 is immediate, nothing needs to happen with it.

                    // Pull in other register from map table:
                    pipe[i].R->op3_r = mapTable[pipe[i].R->op3];

                    break;
                case 'S':
#ifdef RENAME_DEBUG
                    printf("S-type rename\n");
#endif
                    // Lookup registers from map table to rename. Don't need anything from the free list :)
                    pipe[i].R->op1_r = mapTable[pipe[i].R->op1]; // Source register
#ifdef RENAME_DEBUG
                    printf("renamed a%d to p%d\n", pipe[i].R->op1, pipe[i].R->op1_r);
#endif
                    pipe[i].R->op2_r = pipe[i].R->op2;           // Immediate value should be passed through directly
                    pipe[i].R->op3_r = mapTable[pipe[i].R->op3]; // Destination register
                    break;
                };
#ifdef RENAME_DEBUG
                printf("%c %d %d %d  ----> %c, %d, %d, %d\n", pipe[i].R->iType, pipe[i].R->op1, pipe[i].R->op2, pipe[i].R->op3, pipe[i].R->iType, pipe[i].R->op1_r, pipe[i].R->op2_r, pipe[i].R->op3_r);
#endif
            }
            else
            {
#ifdef RENAME_DEBUG
                printf("FREE LIST OUT OF REGISTERS!\n");
#endif
                renameStall = true;
            }
        }
    }
    return renameStall;
}

unsigned int decode(frontEndPipe_t *pipe, unsigned int cycle, unsigned int stall)
{
    for (int i = 0; i < ISSUE_WIDTH; i++)
    {
        if (pipe[i].Dc != NOP)
        {
#ifdef DECODE_DEBUG
            printf("Decode sees: %c index %d\n", pipe[i].Dc->iType, pipe[i].Dc->fetchIndex);
            printf("Decode %d advance\n", i);
#endif
            pipe[i].Dc->Dc = cycle; // Mark cycle of completion
            pipe[i].R = pipe[i].Dc; // Advance pipeline, containing renamed registers, to rename

            // Create a copy of the instruction and assign it to the Dc stage
            iRecord_t *decodedInstruction = (iRecord_t *)malloc(sizeof(iRecord_t));
            *decodedInstruction = *pipe[i].Dc;
            pipe[i].Dc = decodedInstruction;
        }
        else
        {
#ifdef DECODE_DEBUG
            printf("Decode NOP\n");
#endif
        }
    }
    return 0;
}

/**
 * @brief Fetches a batch of instructions for decode, if not told to stall
 *
 * @param stall Boolean state of the machine to stall
 * @param cycle Current cycle the machine is at
 * @param ICOUNT number of instructions completed
 */
void fetch(frontEndPipe_t *pipe, unsigned int cycle, unsigned int stall, unsigned int ICOUNT, unsigned int *fetchOffset)
{
    for (int i = 0; i < ISSUE_WIDTH; i++)
    {
        pipe[i].Dc = pipe[i].F; // Advance the pipeline stage

        if (*fetchOffset < ICOUNT)
        {
#ifdef FETCH_DEBUG
            printf("Fetch %d advance\n", i);
#endif
            pipe[i].F->F = cycle; // Mark cycle it was completed in

            // Create a copy of the instruction and assign it to the F stage
            // iRecord_t *fetchedInstruction = (iRecord_t *)malloc(sizeof(iRecord_t));
            //*fetchedInstruction = instructions[*fetchOffset];
            pipe[i].F = &instructions[*fetchOffset];
            pipe[i].F->fetchIndex = *fetchOffset;
#ifdef FETCH_DEBUG
            printf("Fetch offset %d: %c, %d, %d, %d on cycle %d\n", *fetchOffset, pipe[i].F->iType, pipe[i].F->op1, pipe[i].F->op2, pipe[i].F->op3, cycle);
#endif
            (*fetchOffset)++;
        }
        else
        {
#ifdef FETCH_DEBUG
            printf("Fetch complete\n");
#endif
            // No more instructions to fetch, set F stage to NOP
            for (int i = 0; i < ISSUE_WIDTH; i++)
            {
                pipe[i].F = NOP;
            }
        }
    }
}

void showdq(deque<unsigned int> g)
{
    deque<unsigned int>::iterator it;
    for (it = g.begin(); it != g.end(); ++it)
        cout << '\t' << *it;
    cout << '\n';
}

int main()
{
    unsigned int stall = false;
    unsigned int cycle = 0;                         //!< Current cycle of the system
    unsigned int completedInsts = 0;                //!< Number of completed instructions
    const unsigned int ICOUNT = initInstructions(); //!< Number of instructions to process in the system
    unsigned int fetchOffset = 0;                   //!< Batch to fetch instructions by.

    // Allocate memory for map and ready tables
    mapTable = (unsigned int *)calloc(AREG_COUNT, sizeof(unsigned int));   // Map table maps architectural registers to phyiscal registers and is thus AREG_COUNT wide
    readyTable = (unsigned int *)calloc(PREG_COUNT, sizeof(unsigned int)); // ready table is all the physical registers and is therefore PREG_COUNT wide

    //////////////////   MACHINE INITIALIZATION    /////////////////////////////
    /**
     * Initial register mapping in map table of A0->P0, A1->P1...A31->P31
     * all other physical registers are on the free list in increasing register order
     * Commit table is same size as instruction count, but contains only
     */

    // Init mapping table:
    for (int i = 0; i < AREG_COUNT; i++)
    {
        mapTable[i] = i;
    }

    for (int i = 0; i < AREG_COUNT; i++)
    {
        printf("%d => %d\n", i, mapTable[i]);
    }

    // Init ready table
    for (int i = 0; i < PREG_COUNT; i++)
    {
        readyTable[i] = true;
    }
    for (int i = 0; i < PREG_COUNT; i++)
        printf("%d => %d\n", i, readyTable[i]);

    // Init free list
    for (int i = AREG_COUNT; i < PREG_COUNT; i++)
    {
        freeList.push_back(i);
    }

#ifdef DEBUG_MODE
    cout << "Raw Free List is: ";
    showdq(freeList);
#endif

    // Allocate memory for pipeline:
    thePipelineState = (frontEndPipe_t *)calloc(ISSUE_WIDTH, sizeof(frontEndPipe_t));

    // Allocate memory for members in the pipeline and initialize.
    for (int i = 0; i < ISSUE_WIDTH; i++)
    {
        thePipelineState[i].F = (iRecord_t *)malloc(sizeof(iRecord_t));
        thePipelineState[i].Dc = (iRecord_t *)malloc(sizeof(iRecord_t));
        thePipelineState[i].R = (iRecord_t *)malloc(sizeof(iRecord_t));
        thePipelineState[i].Di = (iRecord_t *)malloc(sizeof(iRecord_t));
        thePipelineState[i].IS = (iRecord_t *)malloc(sizeof(iRecord_t));
        thePipelineState[i].W = (iRecord_t *)malloc(sizeof(iRecord_t));
        thePipelineState[i].C = (iRecord_t *)malloc(sizeof(iRecord_t));

        // Initialize the pipeline with NOPs:
        thePipelineState[i].F = thePipelineState[i].Dc = thePipelineState[i].R = thePipelineState[i].Di = thePipelineState[i].IS = thePipelineState[i].W = thePipelineState[i].C = NOP;
    }

    // // Fetch first batch of instructions:
    // for (int i = 0; i < ISSUE_WIDTH; i++)
    // {
    //     thePipelineState[i].F = (iRecord_t *)malloc(sizeof(iRecord_t)); // Allocate memory for iRecord_t
    //     thePipelineState[i].F->iType = &instructions[fetchOffset].iType;
    //     thePipelineState[i].F->op1 = &instructions[fetchOffset].op1;
    //     thePipelineState[i].F->op2 = &instructions[fetchOffset].op2;
    //     thePipelineState[i].F->op3 = &instructions[fetchOffset].op3;
    //     thePipelineState[i].F->fetchIndex = fetchOffset;
    //     fetchOffset++;
    // }

    // Fetch initial batch of instructions
    for (int i = 0; i < ISSUE_WIDTH; i++)
    {
        thePipelineState[i].F = instructions + fetchOffset;
        fetchOffset++;
    }

    printf("Will fetch %d", fetchOffset);

    for (int i = 0; i < ISSUE_WIDTH; i++)
    {
        printf("ISSUE_WIDTH=%d:\t%c, %d, %d, %d\n", i, thePipelineState[i].F->iType, thePipelineState[i].F->op1, thePipelineState[i].F->op2, thePipelineState[i].F->op3);
    }

    // Print instructions

    printf("INSTRUCTIONS TO BE PROCESSED:\n");
    for (int i = 0; i < ICOUNT; i++)
    {
        printf("%c %d %d %d\n", instructions[i].iType, instructions[i].op1, instructions[i].op2, instructions[i].op3);
    }
    printf("==========================================\n");

    while (completedInsts < ICOUNT)
    {
        printf("\n\n========= CYCLE %d ==========\n", cycle);
        // printf("RT:\n");
        // for(int i = 0; i < PREG_COUNT; i++)
        // {
        //     printf("%d  %d\n", i, readyTable[i]);
        // }

        printf("EXTERN ROB STATE:\n");
        for (int i = 0; i < reorderBuff.size(); i++)
        {
            printf("%c %d r%d\n", reorderBuff[i]->instr->iType, reorderBuff[i]->instr->op1_r, reorderBuff[i]->ready);
        }
        readyTable[0] = true; // Ensure p0 is always ready
        completedInsts = commit(thePipelineState, completedInsts, cycle);
        writeback(thePipelineState, cycle);
        issue(thePipelineState, cycle);
        dispatch(thePipelineState, cycle);
        stall = rename(thePipelineState, cycle);
        stall = decode(thePipelineState, cycle, stall);
        fetch(thePipelineState, cycle, stall, ICOUNT, &fetchOffset);

        printf("Completed insts %d\n", completedInsts);

        // completedInsts++;

        ++cycle;

        if (cycle == 15)
            break;
    }

    printRecords(ICOUNT);

    // Housekeping for the instructions:
    free(thePipelineState);

    // for (int i = 0; i < ISSUE_WIDTH; i++)
    // {
    //     free(thePipelineState[i].F);
    //     free(thePipelineState[i].Dc);
    //     free(thePipelineState[i].R);
    //     free(thePipelineState[i].Di);
    //     free(thePipelineState[i].IS);
    //     free(thePipelineState[i].W);
    //     free(thePipelineState[i].C);
    // }

    return 0;
}
