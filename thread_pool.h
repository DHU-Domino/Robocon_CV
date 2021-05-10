#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <mutex>
#include <thread>
#include <vector>
#include <atomic>

class Thread_pool
{

private:
    class guarded_thread : std::thread
    {
    public:
        using std::thread::thread;
        ~guarded_thread()
        {
            if (joinable())
                join();
        }
    };

public:
    Thread_pool() = default;
    ~Thread_pool() = default;

    Thread_pool(Thread_pool &&) = delete;
    Thread_pool(const Thread_pool &) = delete;
    Thread_pool &operator=(Thread_pool &&) = delete;
    Thread_pool &operator=(const Thread_pool &) = delete;

    template <typename F, typename... Args>
    void exec(F &&f, Args &&...args)
    {
        std::unique_lock<std::mutex> lck(mtx);
        if (_pos.size() == 0)
        {
            _thread_list.push_back(std::make_unique<guarded_thread>([&](std::size_t id, F &f, Args &...args) {
                f(args...);
                std::unique_lock<std::mutex> lck(mtx);
                _pos.push_back(id);
            },
                                                                    _thread_list.size(), std::ref(f), std::ref(args)...));
            return;
        }
        _thread_list[_pos[0]] = std::make_unique<guarded_thread>([&](std::size_t id, F &f, Args &...args) {
            f(args...);
            std::unique_lock<std::mutex> lck(mtx);
            _pos.push_back(id);
        },
                                                                 _pos[0], std::ref(f), std::ref(args)...);
        _pos.erase(_pos.begin());
    }

    std::size_t active()
    {
        std::unique_lock<std::mutex> lck(mtx);
        return _thread_list.size() - _pos.size();
    }

private:
    std::mutex mtx;
    std::vector<std::unique_ptr<guarded_thread>> _thread_list;
    std::vector<std::size_t> _pos;
};

#endif // THREAD_POOL_H
