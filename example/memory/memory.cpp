/* % g++ -lpthread Pthread.cpp */

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


using namespace std;

static map<pthread_t, unsigned int> gMemMap;

int numOfThreads = 2;
unsigned int AVAILABLE_MEMORY_inMB = 40;
unsigned initMemory_inMB = 4;

unsigned int oneMB = 1024 * 1024;
unsigned int AVAILABLE_MEMORY = AVAILABLE_MEMORY_inMB * oneMB;
unsigned initMemory = initMemory_inMB * oneMB;

unsigned int gAllocatedMemory = 0;
mutex gMutex;

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

unsigned int getMemLimit()
{
        pthread_t threadId = pthread_self();
	unique_lock<mutex> lLock (gMutex);
	return AVAILABLE_MEMORY / numOfThreads;
}

void releaseMemoryToSafeLevel(uint64_t allocSize, std::list<char*>& ptrList)
{
	unsigned int removedSize = 0;
	while(removedSize < allocSize)
	{
		std::list<char*>::iterator iter = ptrList.begin();
		char* lpTmp = *iter;
		if (lpTmp == NULL || iter == ptrList.end())
		{
			printf("threadAllocatedMemory: %lu, removedSize: %u, allocSize: %lu\n", getSize(ptrList), removedSize, allocSize);
			break;
		}
		removedSize += strlen(lpTmp);
		ptrList.pop_front();
		delete lpTmp;
		lpTmp = NULL;
	}
}

void *runMethod(void* ipInput)
{
        // sched_yield();

        uint64_t tid;
        // pthread_threadid_np(NULL, &tid);
        pthread_t threadId = pthread_self();
        printf("thread id: %lu\n", threadId);
        uint64_t threadAllocatedMemory = 0;
        unsigned int allocInMB = 0;
        std::list<char*> ptrList;
        
        while(true)
        {
                // allocate a random sized memory
                allocInMB = (rand() % (initMemory_inMB * 2) + initMemory_inMB);
                uint64_t allocSize = allocInMB * oneMB;
                if (threadAllocatedMemory > getMemLimit())
                {
                        releaseMemoryToSafeLevel(allocSize, ptrList);
                }
                // printf("thread id: %llu allocated: %llu(MB) plan to allocate: %d(MB)\n", tid, threadAllocatedMemory, allocInMB);
                char* ptr = new char[allocSize+1];
                for (int i = 0; i < allocSize; ++i)
                {
                        ptr[i] = 'a';
                }
                ptr[allocSize] = '\0';

                ptrList.push_back(ptr);
                threadAllocatedMemory = getSize(ptrList);
                usleep(100000);
                // sleep(1);
        }
        return NULL;
}

int main ()
{
        pthread_t       pthread[numOfThreads];
        void*           lRtn = NULL;
        srand(time(NULL));

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
