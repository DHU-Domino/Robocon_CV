#include "ShareMemory.h"

using namespace std;

void ShareMemory::sem_p(int semid) {
    int ret;
    struct sembuf sbuf;

    sbuf.sem_num = 0;//第一个
    sbuf.sem_op = -1;//P操作
    sbuf.sem_flg = SEM_UNDO;//SEM_UNDO进程异常自动UNDO

    ret = semop(semid, &sbuf, 1);
    if (ret == -1) {
        perror("semop");
        return;
    }
}

void ShareMemory::sem_v(int semid) {
    int ret;
    struct sembuf sbuf;

    sbuf.sem_num = 0;//第一个
    sbuf.sem_op = 1;//V操作
    sbuf.sem_flg = SEM_UNDO;//SEM_UNDO进程异常自动UNDO

    ret = semop(semid, &sbuf, 1);
    if (ret == -1) {
        perror("semop");
        return;
    }
}

void ShareMemory::initShareMemory() {
    
    semid = semget(SEMKEY, 1, 0);       //创建信号量
    if (-1 == semid) {
        perror("semget");
        exit(1);
    }
    shmid = shmget(SHMKEY, SHMSIZE, 0); //创建共享内存
    if (-1 == shmid) {
        perror("shmget");
        exit(1);
    }
    shmaddr = shmat(shmid, NULL, 0);    //映射到虚拟地址空间
    if (NULL == shmaddr) {
        perror("shmat");
        exit(1);
    }
    *(int *)shmaddr = count;            //把数据写到内存
}

int ShareMemory::getCount() {
    
    sem_p(semid);                       //P操作
    count = *(int *)shmaddr;
    sem_v(semid);                       //V操作
}
void ShareMemory::exitSM() {
    shmdt(shmaddr);		                                        //解除映射
    sleep(1);
}
ShareMemory::~ShareMemory() {
    //ipcs -s
    //sudo ipcrm -s 10
    exitSM();
}