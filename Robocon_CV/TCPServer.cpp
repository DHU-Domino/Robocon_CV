#include "TCPServer.h" 
#include <windows.h>

char TCPServer::msg[MAXPACKETSIZE];
int TCPServer::num_client;
int TCPServer::last_closed;
bool TCPServer::isonline;
vector<descript_socket*> TCPServer::Message;
vector<descript_socket*> TCPServer::newsockfd;
std::mutex TCPServer::mt;

void* TCPServer::Task(void *arg)
{
    int n;
    struct descript_socket *desc = (struct descript_socket*) arg;

    cerr << "open client[ id:"<< desc->id <<" ip:"<< desc->ip <<" socket:"<< desc->socket<<" send:"<< desc->enable_message_runtime <<" ]" << endl;
    while(1)
    {
        n = recv(desc->socket, msg, MAXPACKETSIZE, 0);
        if(n != -1) 
        {
            if(n==0)
            {
               isonline = false;
               cerr << "close client[ id:"<< desc->id <<" ip:"<< desc->ip <<" socket:"<< desc->socket<<" ]" << endl;
               last_closed = desc->id;
               closesocket(desc->socket);

               int id = desc->id;
               auto new_end = std::remove_if(newsockfd.begin(), newsockfd.end(),
                                                   [id](descript_socket *device)
                                                       { return device->id == id; });
               newsockfd.erase(new_end, newsockfd.end());

               if(num_client>0) num_client--;
               break;
            }
            msg[n]=0;
            desc->message = string(msg);
            std::lock_guard<std::mutex> guard(mt);
            Message.push_back( desc );
        }
        Sleep(1);
    }
    if(desc != NULL)
        free(desc);
    cerr << "exit thread: " << this_thread::get_id() << endl;
    
    return 0;
}

int TCPServer::setup(int port, vector<int> opts)
{
    
    WORD sockVersion = MAKEWORD(2, 2);
    WSADATA wsaData;
    if (WSAStartup(sockVersion, &wsaData) != 0) {
        return -1;
    }
    int opt = 1;
    isonline = false;
    last_closed = -1;
    sockfd = socket(AF_INET,SOCK_STREAM,0);
    if (sockfd == INVALID_SOCKET) {
        printf("socket error !");
        return -1;
    }

    memset(&serverAddress,0,sizeof(serverAddress));

    serverAddress.sin_family      = AF_INET;
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddress.sin_port        = htons(port);

    if((::bind(sockfd, (LPSOCKADDR)&serverAddress, sizeof(serverAddress))) == SOCKET_ERROR){
        cerr << "Errore bind" << endl;
        return -1;
    }
    
    if(listen(sockfd,5) < 0){
        cerr << "Errore listen" << endl;
        return -1;
    }
    num_client = 0;
    isonline = true;
    return 0;
}

int TCPServer::accepted()
{
    int sosize = sizeof(SOCKADDR);
    descript_socket *so = new descript_socket;
    so->socket          = accept(sockfd,(struct sockaddr*)&clientAddress,&sosize);
    so->id              = num_client;
    so->ip              = inet_ntoa(clientAddress.sin_addr);
    newsockfd.push_back( so );
    cerr << "accept client[ id:" << newsockfd[num_client]->id << 
                          " ip:" << newsockfd[num_client]->ip << 
                      " handle:" << newsockfd[num_client]->socket << " ]" << endl;

    serverThread[num_client] = thread(&Task, newsockfd[num_client]);
    serverThread[num_client].detach();

    isonline=true;
    num_client++;
    return so->id;
}

vector<descript_socket*> TCPServer::getMessage()
{
    std::lock_guard<std::mutex> guard(mt);
    return Message;
}

void TCPServer::Send(string msg, int id)
{
    send(newsockfd[id]->socket,msg.c_str(),msg.length(),0);
}

int TCPServer::get_last_closed_sockets()
{
    return last_closed;
}

void TCPServer::clean(int id)
{
    Message[id]->message = "";
    memset(msg, 0, MAXPACKETSIZE);
}

string TCPServer::get_ip_addr(int id)
{
    return newsockfd[id]->ip;
}

bool TCPServer::is_online() 
{
    return isonline;
}

void TCPServer::detach(int id)
{
    closesocket(newsockfd[id]->socket);
    newsockfd[id]->ip = "";
    newsockfd[id]->id = -1;
    newsockfd[id]->message = "";
} 

void TCPServer::closed() 
{
    closesocket(sockfd);
}

