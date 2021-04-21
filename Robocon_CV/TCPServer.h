#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h> 
#include <string.h>
#include <thread>
#include <algorithm>
#include <cctype>
#include <mutex>
#include <thread>

#include<winsock.h>
#pragma comment(lib,"ws2_32.lib")

using namespace std;

#define MAXPACKETSIZE 40960
#define MAX_CLIENT 10
//#define CODA_MSG 4

struct descript_socket {

    int socket     = -1;
    string ip      = "";
    int id         = -1; 
    std::string message;
    bool enable_message_runtime = false;
};

class TCPServer
{
public:
    int setup(int port, vector<int> opts = vector<int>());
    vector<descript_socket*> getMessage();
    int accepted();
    void Send(string msg, int id);
    void detach(int id);
    void clean(int id);
    bool is_online();
    string get_ip_addr(int id);
    int get_last_closed_sockets();
    void closed();

private:
    int sockfd, n, pid;
    sockaddr_in serverAddress;
    sockaddr_in clientAddress;
    thread serverThread[ MAX_CLIENT ];

    static vector<descript_socket*> newsockfd;
    static char msg[ MAXPACKETSIZE ];
    static vector<descript_socket*> Message;//[CODA_MSG];

    static bool isonline;
    static int last_closed;
    static int num_client;
    static std::mutex mt;
    static void * Task(void * argv);
};

#endif
