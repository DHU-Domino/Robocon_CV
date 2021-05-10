#ifndef SERVER_BASE_H
#define SERVER_BASE_H

#include <unistd.h>

#include <map>
#include <functional>

#include "support.h"
#include "thread_pool.h"
#include "req_res.h"

class Server_base
{
public:
    using Functions = std::map<std::string, std::function<void(Request &, Response &)>>;

    Server_base(uint16_t port)
    {
        server_impl::init_server(sock_fd, s_addr, port);
    }

    ~Server_base()
    {
        close(sock_fd);
    }

protected:
    Thread_pool tp;
    Functions paths_f;

    int sock_fd;
    sockaddr_in s_addr;
};

#endif // SERVER_BASE_H
