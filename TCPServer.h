#pragma once

#include <unistd.h>
#include <pthread.h>
#include <sys/types.h> 
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <map>
#include <cmath>
#include <mutex>
#include <tuple>
#include <ctime>
#include <chrono>
#include <random>
#include <memory>
#include <cctype>
#include <thread>
#include <vector>
#include <cstdio>
#include <string>
#include <cstring>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <exception>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "WzSerialportPlus.h"

using namespace std;

#define MAXPACKETSIZE 40960
#define MAX_CLIENT 10

struct ProdCons_data {
    SendData data;
    bool isUse;
};

void* SocketTask(void* arg);

class TCPServer {
public:
    int setup(int port, vector<int> opts = vector<int>());
    void accepted();
    void closed();
    static mutex mt;
    static mutex mtSerialData;

private:
    int sockfd, n, pid;
    static map< int, tuple<string, int, int> > mmap; // ( new_fd, make_tuple(new_ip, 0, num_client));

    struct sockaddr_in serverAddress;
    struct sockaddr_in clientAddress;
    pthread_t serverThread[ MAX_CLIENT ];

    static char msg[ MAXPACKETSIZE ];
    static int num_client;
    
    
    static void * Task(void * argv);

    friend void* SocketTask(void*  arg);
};
