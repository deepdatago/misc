/* 
 * This example defines number of threads to run, and max total memory limit the 
 * threads can consume.  The threads will continue in a while loop to do following:
 * 1) randomly allocate a smaller memory block and insert it to a list
 * 2) when the limit of the each thread is reached (Max Memory / # of threads)
 *    then, it will release a portion of memory from its list to ensure the next 
 *    allocation will not exceeds the limit of this thread
 * 3) repeat from 1)
 */

#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <list>
#include <map>
#include <string.h>
#include <cmath>

#include <inttypes.h>
#include <mutex>
#include <fstream>

#include "PID_SGD/PID_SGD.h"

// #include <sys/types.h>
// #include <sys/sysctl.h>

using namespace std;

static map<pthread_t, uint64_t> gMemMap;

int numOfThreads = 2;

uint64_t oneMB = 1024 * 1024;

uint64_t gAllocatedMemory = 0;
mutex gMutex;
std::ofstream gLogFile;

void log(std::string iString)
{
	gLogFile << iString;
}

/*
uint64_t getTotalSystemMemory()
{
    long pages = sysconf(_SC_PHYS_PAGES);
    long page_size = sysconf(_SC_PAGE_SIZE);
    return pages * page_size;
}
*/
unsigned long getFreeMemoryInBytes() {
    log("Entering::getFreeMemoryInBytes\n");
    std::string token;
    std::ifstream file("/proc/meminfo");
    while(file >> token) {
        if(token == "MemFree:") {
            unsigned long mem;
            if(file >> mem) {
                return mem * 1024;
            } else {
                return 0;       
            }
        }
        // ignore rest of the line
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    log("Leaving::getFreeMemoryInBytes\n");
    return 0; // nothing found
}

uint64_t getSize(list<char*> &iList)
{
        log("Entering::getSize\n");
        uint64_t memorySize = 0;
        std::list<char*>::iterator iter = iList.begin();
        while(iter != iList.end())
        {
                memorySize += strlen(*iter);
                ++iter;
        }
        log("Leaving::getSize\n");
        return memorySize;
}

void releaseMemoryToSafeLevel(uint64_t allocSize, std::list<char*>& ptrList)
{
        log("Entering::releaseMemoryToSafeLevel\n");
	uint64_t removedSize = 0;
        pthread_t threadId = pthread_self();
	while(removedSize < allocSize)
	{
		std::list<char*>::iterator iter = ptrList.begin();
		char* lpTmp = *iter;
		if (lpTmp == NULL || iter == ptrList.end())
		{
			char outStr[256];
			sprintf(outStr, "\ttid: %lu: Nothing to release: threadAllocatedMemory(MB): %lu, already removedSize(MB): %lu, plan to allocSize(MB): %lu\n", threadId, getSize(ptrList)/oneMB, removedSize/oneMB, allocSize/oneMB);
			log(outStr);
			break;
		}
		removedSize += strlen(lpTmp);
		char outStr[256];
		sprintf(outStr, "\ttid: %lu: releaseMemory: removedSize(MB): %lu\n", threadId, removedSize/oneMB);
		log(outStr);
		ptrList.pop_front();
		delete lpTmp;
		lpTmp = NULL;
	}
        log("Leaving::releaseMemoryToSafeLevel\n");
}

uint64_t proposeRandMemoryToAllocate(uint64_t proposedMemInMB)
{
	return proposedMemInMB * oneMB;
	/*
	// allocate between [initMemory_inMB, 2*initMemory_inMB]
	uint64_t initMemory_inMB = proposedMemInMB;
	uint64_t allocInMB = 0;
	allocInMB = (rand() % (initMemory_inMB * 2) + initMemory_inMB);
	uint64_t allocSize = allocInMB * oneMB;
	return allocSize;
	*/
}

char* populateMemory(uint64_t allocSize)
{
	log("Entering::populateMemory\n");
	char* ptr = new char[allocSize+1];
	for (int i = 0; i < allocSize/100; ++i)
       	{
	       	ptr[i] = 'a';
       	}
       	ptr[allocSize] = '\0';
	log("Leaving::populateMemory\n");
       	return ptr;
}
/*
void processMemUsage(double& vm_usage, double& resident_set)
{
   using std::ios_base;
   using std::ifstream;
   using std::string;

   vm_usage     = 0.0;
   resident_set = 0.0;

   // 'file' stat seems to give the most reliable results
   //
   ifstream stat_stream("/proc/self/stat",ios_base::in);

   // dummy vars for leading entries in stat that we don't care about
   //
   string pid, comm, state, ppid, pgrp, session, tty_nr;
   string tpgid, flags, minflt, cminflt, majflt, cmajflt;
   string utime, stime, cutime, cstime, priority, nice;
   string O, itrealvalue, starttime;

   // the two fields we want
   //
   unsigned long vsize;
   long rss;

   stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
               >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
               >> utime >> stime >> cutime >> cstime >> priority >> nice
               >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest

   stat_stream.close();

   long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
   vm_usage     = vsize / 1024.0;
   resident_set = rss * page_size_kb;
}
*/
void *runMethod(void* ipInput)
{
        // sched_yield();
        PID_SGD* lpPID_SGD = (PID_SGD*) ipInput;
        uint64_t tid;
        // pthread_threadid_np(NULL, &tid);
        pthread_t threadId = pthread_self();
        printf("thread id: %lu\n", threadId);
        uint64_t allocInMB = 0;
        std::list<char*> ptrList;
        uint64_t proposedAllocInMB = 1;
       	double currentMax = 0.0;

       	bool lbComputed = false;
       	double lOutput = 0.0;
       	double lSetPoint = lpPID_SGD->GetSetpoint();
        
        while(true)
        {
		log("Begining::while\n");
                // allocate a random sized memory
        	uint64_t currentFreeMem = getFreeMemoryInBytes() / oneMB;
        	currentMax = currentFreeMem / (2.0 * numOfThreads);
        	lpPID_SGD->SetOutputLimits(0.0, currentMax);
                tie(lbComputed, lOutput) = lpPID_SGD->Compute(currentFreeMem * 1.0);
        	if (lbComputed == false)
        	{
        		lOutput = 0.0;
        	}
        	proposedAllocInMB = lOutput;
                uint64_t allocSize = proposeRandMemoryToAllocate(proposedAllocInMB);
                // if (getSize(ptrList) > getMemGoverningLimit())
                if (currentFreeMem < lSetPoint / oneMB)
                {
                        releaseMemoryToSafeLevel(allocSize, ptrList);
                }
                else if (proposedAllocInMB == 0)
                {
			int randNo = rand() % 10;
			if (randNo <=3) // 30% of chance to release some memory
			{
                        	releaseMemoryToSafeLevel(lSetPoint, ptrList);
			}
                }
                // printf("thread id: %llu allocated: %llu(MB) plan to allocate: %d(MB)\n", threadId, getSize(ptrList), allocSize/oneMB);
                // printf("free memory:  %llu already allocated: %llu(MB) plan to allocate: %d(MB)\n", currentFreeMem, getSize(ptrList)/oneMB, allocSize/oneMB);
                uint64_t currentSize = getSize(ptrList);
		// printf("freeMem: %lu\n", getFreeMemoryInBytes());

                char* ptr = populateMemory(allocSize);
                ptrList.push_back(ptr);
        	uint64_t newFreeMem = getFreeMemoryInBytes() / oneMB;
                bool useMax = false;
        	if (abs(proposedAllocInMB - currentMax) < 0.1)
        	{
        		useMax = true;
        	}

                char outputStr[256];
                sprintf(outputStr, "tid: %lu free memory:  %lu previously allocated: %lu(MB) plan to allocate: %lu(MB), after allocate size: %lu(MB), new Free memory(MB): %lu -----> use max? %s <------ \n", threadId, currentFreeMem, currentSize/oneMB, allocSize/oneMB, getSize(ptrList)/oneMB, newFreeMem, useMax ? "TRUE" : "FALSE");
		log(outputStr);
		log("\tEnding::while\n");

                // usleep(500000);
                // sleep(1);
        }
        return NULL;
}

int main ()
{
	gLogFile.open("/tmp/trace.log", std::ios_base::app);
        pthread_t       pthread[numOfThreads];
        void*           lRtn = NULL;
        srand(time(NULL));
        // printf("Total system memory: %lu\n", getTotalSystemMemory());

        double setPoint = 70; // 40MB free
        double P_On = 1;
        int controllerDirection = REVERSE;
        PID_SGD		lPID_SGD(setPoint,
        			P_On,
        			controllerDirection);
        lPID_SGD.SetMode(AUTOMATIC);

        for (int i = 0; i < numOfThreads; ++i)
        {
                // pthread_create(&pthread[i], NULL, runMethod, NULL);
                pthread_create(&pthread[i], NULL, runMethod, &lPID_SGD);
        }

        for (int i = 0; i < numOfThreads; ++i)
        {
                pthread_join(pthread[i], &lRtn);
        }

        return 0;
}
