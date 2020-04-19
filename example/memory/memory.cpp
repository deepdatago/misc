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

#include <inttypes.h>
#include <mutex>
#include <fstream>

#include "PID_SGD/PID_SGD.h"

// #include <sys/types.h>
// #include <sys/sysctl.h>

using namespace std;

static map<pthread_t, uint64_t> gMemMap;

int numOfThreads = 2;
uint64_t gGoverningMemoryLimit_inMB = 40;

uint64_t oneMB = 1024 * 1024;
uint64_t gGoverningMemoryLimit = gGoverningMemoryLimit_inMB * oneMB;

uint64_t gAllocatedMemory = 0;
mutex gMutex;

uint64_t getTotalSystemMemory()
{
    long pages = sysconf(_SC_PHYS_PAGES);
    long page_size = sysconf(_SC_PAGE_SIZE);
    return pages * page_size;
}

uint64_t getSize(list<char*> &iList)
{
        uint64_t memorySize = 0;
        std::list<char*>::iterator iter = iList.begin();
        while(iter != iList.end())
        {
                memorySize += strlen(*iter);
                ++iter;
        }
        return memorySize;
}

uint64_t getMemGoverningLimit()
{
        pthread_t threadId = pthread_self();
	unique_lock<mutex> lLock (gMutex);
	return gGoverningMemoryLimit / numOfThreads;
}

void releaseMemoryToSafeLevel(uint64_t allocSize, std::list<char*>& ptrList)
{
	uint64_t removedSize = 0;
	while(removedSize < allocSize)
	{
		std::list<char*>::iterator iter = ptrList.begin();
		char* lpTmp = *iter;
		if (lpTmp == NULL || iter == ptrList.end())
		{
			printf("threadAllocatedMemory: %lu, removedSize: %lu, allocSize: %lu\n", getSize(ptrList), removedSize, allocSize);
			break;
		}
		removedSize += strlen(lpTmp);
		ptrList.pop_front();
		delete lpTmp;
		lpTmp = NULL;
	}
}

uint64_t proposeRandMemoryToAllocate()
{
	// allocate between [initMemory_inMB, 2*initMemory_inMB]
	uint64_t initMemory_inMB = gGoverningMemoryLimit_inMB/10;
	uint64_t allocInMB = 0;
	allocInMB = (rand() % (initMemory_inMB * 2) + initMemory_inMB);
	uint64_t allocSize = allocInMB * oneMB;
	return allocSize;
}

char* populateMemory(uint64_t allocSize)
{
	char* ptr = new char[allocSize+1];
	for (int i = 0; i < allocSize; ++i)
       	{
	       	ptr[i] = 'a';
       	}
       	ptr[allocSize] = '\0';
       	return ptr;
}

unsigned long getFreeMemoryInBytes() {
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
    return 0; // nothing found
}

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

void *runMethod(void* ipInput)
{
        // sched_yield();

        uint64_t tid;
        // pthread_threadid_np(NULL, &tid);
        pthread_t threadId = pthread_self();
        printf("thread id: %lu\n", threadId);
        uint64_t allocInMB = 0;
        std::list<char*> ptrList;
        
        while(true)
        {
                // allocate a random sized memory
                uint64_t allocSize = proposeRandMemoryToAllocate();
                if (getSize(ptrList) > getMemGoverningLimit())
                {
                        releaseMemoryToSafeLevel(allocSize, ptrList);
                }
                // printf("thread id: %llu allocated: %llu(MB) plan to allocate: %d(MB)\n", tid, threadAllocatedMemory, allocInMB);
		// printf("freeMem: %lu\n", getFreeMemoryInBytes());

                char* ptr = populateMemory(allocSize);
                ptrList.push_back(ptr);

                usleep(500000);
                // sleep(1);
        }
        return NULL;
}

int main ()
{
        PID_SGD		lPID_SGD(0, 0, 0, 0, 0, 0, 0);
        pthread_t       pthread[numOfThreads];
        void*           lRtn = NULL;
        srand(time(NULL));
        printf("Total system memory: %lu\n", getTotalSystemMemory());

        for (int i = 0; i < numOfThreads; ++i)
        {
                pthread_create(&pthread[i], NULL, runMethod, NULL);
                // pthread_create(&pthread[i], NULL, runMethod, tmpBuffer);
        }

        for (int i = 0; i < numOfThreads; ++i)
        {
                pthread_join(pthread[i], &lRtn);
        }

        exit(0);
}
