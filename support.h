#ifndef SUPPORT_H
#define SUPPORT_H

#include <sys/socket.h>
#include <arpa/inet.h>

#include <cstring>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <map>

namespace server_impl
{

    using Headers = std::vector<std::string>;
    using Params = std::map<std::string, std::string>;

    void parse_q(std::string str, Params &params)
    {
        size_t pos = 0;
        if (pos = str.find("?"); pos == std::string::npos)
            return;
        str.erase(0, pos + 1);
        if (pos = str.find("HTTP/"); pos == std::string::npos)
            return;
        str.erase(pos - 1);
        while (true)
        {
            if ((pos = str.find("=")); pos == std::string::npos)
                return;
            std::string s2 = str.substr(0, pos);
            str.erase(0, pos + 1);
            if (pos = str.find("&"); str == "" && pos == std::string::npos)
                return;
            params[s2] = str.substr(0, pos);
            str.erase(0, pos + 1);
        }
    }

    void parse(std::string str, Headers &headers, Params &params)
    {
        size_t pos = 0;
        pos = str.find("\r\n");
        parse_q(str.substr(0, pos), params);
        str.erase(0, pos + 2);

        while ((pos = str.find("\r\n")) != std::string::npos && str != "\r\n")
        {
            headers.push_back(str.substr(0, pos));
            str.erase(0, pos + 2);
        }
    }

    size_t get_first_free(std::vector<std::thread *> &tasks,
                          std::vector<std::atomic_bool *> &ended_t)
    {
        for (size_t i = 0; i < tasks.size(); ++i)
            if (*ended_t[i])
            {
                if (tasks[i]->joinable())
                    tasks[i]->join();
                *ended_t[i] = false;
                return i;
            }
        tasks.push_back(nullptr);
        ended_t.push_back(new std::atomic_bool(false));
        return tasks.size() - 1;
    }

    int accept(int sock_fd)
    {
        sockaddr_in c_addr;
        socklen_t c_length = sizeof(c_addr);
        return accept(sock_fd, reinterpret_cast<sockaddr *>(&c_addr), &c_length);
    }

    std::string receive(int fd)
    {
        char buffer[1024 * 8]{0};
        std::string str = "";
        while (recv(fd, buffer, sizeof(buffer), MSG_DONTWAIT) > 0)
            str += std::string(buffer);
        return str;
    }

    bool send_h(int fd, const std::vector<std::string> &headers)
    {
        std::string res = "";
        for (const auto &h : headers)
            res += h + "\r\n";
        res += "\r\n";
        return send(fd, res.c_str(), res.size(), MSG_NOSIGNAL) >= 0;
    }

    bool send_m(int fd, const std::string &msg)
    {
        std::string res = msg;
        return send(fd, res.c_str(), res.size(), MSG_NOSIGNAL) >= 0;
    }

    std::string get_request_path(const std::string &req)
    {
        if (req.find("?") != std::string::npos)
            return (req.size() > 0) ? req.substr(req.find("GET") + 4, req.find("?") - req.find("GET") - 4)
                                    : "";
        return (req.size() > 0) ? req.substr(req.find("GET") + 4, req.find("HTTP/") - req.find("GET") - 5)
                                : "";
    }

    void set_channel(int sock_fd, sockaddr_in &s_addr)
    {
        if (bind(sock_fd, reinterpret_cast<sockaddr *>(&s_addr), sizeof(s_addr)) < 0)
            throw std::runtime_error("binding_failed");
        listen(sock_fd, 32);
    }

    void init_server(int &sock_fd, sockaddr_in &s_addr, uint16_t port)
    {
        sock_fd = socket(AF_INET, SOCK_STREAM, 0);

        const int optVal = 1;
        const socklen_t optLen = sizeof(optVal);
        setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const void *>(&optVal), optLen);

        memset(&s_addr, 0, sizeof(s_addr));
        s_addr.sin_family = AF_INET;
        s_addr.sin_addr.s_addr = inet_addr("0.0.0.0");
        s_addr.sin_port = htons(port);

        set_channel(sock_fd, s_addr);
    }

};

#endif // SUPPORT_H
