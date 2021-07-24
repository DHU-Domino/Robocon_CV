#ifndef SHARE_MEMORY_H
#define SHARE_MEMORY_H


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/sem.h>

using namespace std;

#define SHMKEY 1234
#define SHMSIZE 4096
#define SEMKEY 1234

class ShareMemory
{
public:
    union semun {
        int val;                    //Value for SETVAL
        struct semid_ds *buf;       //Buffer for IPC_STAT, IPC_SET
        unsigned short  *array;     //Array for GETALL, SETALL
        struct seminfo  *__buf;     //Buffer for IPC_INFO(Linux specific)
    };
    int shmid;
    void *shmaddr;
    int count = 0;
    int semid;
    union semun unsem;

    void sem_p(int semid);
    void sem_v(int semid);
    void initShareMemory();
    int getCount();
    void exitSM();
    ~ShareMemory();
};
#endif