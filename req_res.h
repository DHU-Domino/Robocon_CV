#ifndef REQ_RES_H
#define REQ_RES_H

#include <string>
#include <vector>

#include "support.h"

struct Request
{
    server_impl::Headers headers;
    server_impl::Params query;
    std::string path;

    Request(std::string p) : path(p)
    {
    }
};

struct Response
{
    int client;
    server_impl::Headers headers;

    Response(int clt) : client(clt), headers{"HTTP/1.1 200 OK"}
    {
    }

    bool write(std::string msg) const
    {
        return server_impl::send_h(client, headers) && server_impl::send_m(client, msg);
    }

    bool send_header() const
    {
        return server_impl::send_h(client, headers);
    }

    bool send_msg(std::string msg) const
    {
        return server_impl::send_m(client, msg);
    }
};

void operator>>(const Response &res,
                const std::string &msg)
{
    res.write(msg);
}

#endif // REQ_RES_H
