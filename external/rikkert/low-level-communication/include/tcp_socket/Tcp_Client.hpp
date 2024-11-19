#pragma once
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <mutex>
#include "Buffer.hpp"
#include "utils.hpp"

class TCP_Client
{
private:
    std::mutex mtx;

    bool running;
    bool had_connection;
    bool has_connection;

    std::string ip_adr;

    Buffer<uint8_t> *data_buffer_read;
    Buffer<uint8_t> *data_buffer_write;
    uint64_t data_buffer_max_size;

    uint32_t port;
    boost::asio::io_service io_service;
    boost::asio::ip::tcp::socket *socket;
    boost::asio::ip::tcp::endpoint endpoint;
    boost::thread th_connect;
    boost::thread th_read;
    boost::thread th_write;
    boost::thread th_callback;

    void (*callback_func)(void* obj, Buffer<uint8_t> *read_buffer);
    void (*on_connect)(const char *ip);
    void (*on_disconnect)(const char *ip);
    bool callback_new_th;
    boost::atomic<bool> has_new_data;

    void* obj;

    void callback();

    void thread_read_func();

    void thread_write_func();

    void thread_connect_func();

    void init_buffers(uint64_t size);

    bool th_callback_join();

    void open();

    void close();

public:
    TCP_Client();

    void start(std::string ip_adr, uint32_t port_);

    void stop();

    bool write(uint8_t *data, uint64_t size);

    bool is_running();

    bool is_connected();

    bool has_data();

    bool read(uint8_t *data, uint64_t &size);

    void set_callback(void* obj, void (*callback_func)(void* obj, Buffer<uint8_t> *read_buffer));

    void set_on_connect(void (*callback_func)(const char *ip));

    void set_on_disconnect(void (*callback_func)(const char *ip));

    void set_callback(void* obj, void (*callback_func)(void* obj, Buffer<uint8_t> *read_buffer), bool use_new_thread);

    uint64_t read_buffer_size();

    uint64_t write_buffer_size();

    void set_buffer_max_size(uint64_t max_size);

    uint64_t get_buffer_max_size();

    Buffer<uint8_t> *get_read_buffer();

    Buffer<uint8_t> *get_write_buffer();
};
