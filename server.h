#ifndef SERVER_H
#define SERVER_H

#include "req_res.h"
#include "support.h"
#include "server_base.h"

void _callback(int client, Server_base::Functions &paths_f)
{

    std::string req = server_impl::receive(client);
    std::string path = server_impl::get_request_path(req);

    if (paths_f.find(path) != paths_f.end())
    {
        Request r(path);
        server_impl::parse(req, r.headers, r.query);
        Response res(client);
        paths_f[path](r, res);
    }

    close(client);
}

namespace http
{

    class Server : public Server_base
    {
    public:
        using Callback = std::function<void(Request &, Response &)>;

        Server(uint16_t port) : Server_base(port)
        {
        }

        ~Server() = default;

        void listen()
        {
            int client;
            while ((client = server_impl::accept(sock_fd)) > 0)
                tp.exec(_callback, client, paths_f);
        }

        Server &get(std::string path, Callback f)
        {
            paths_f[path] = f;
            return *this;
        }
    };

};

#endif // SERVER_H
