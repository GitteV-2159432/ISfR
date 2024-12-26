#pragma once
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <mutex>

using namespace boost::asio;
using ip::tcp;
using std::cout;
using std::endl;
using std::string;

#define THREAD_SLEEP_US 10
#define THREAD_SLEEP_US_CONNECT 10000

class Server
{
    std::mutex mtx;

    bool running;
    bool had_connection;
    bool need_reset;

    uint8_t *data_buffer_read;
    uint8_t *data_buffer_write;
    uint8_t *data_buffer_write_temp;
    uint64_t data_buffer_size_read;
    uint64_t data_buffer_size_write;
    uint64_t data_buffer_size_write_temp;
    uint64_t data_buffer_max_size_read = 4096;
    uint64_t data_buffer_max_size_write = 4096;

    uint32_t port;
    boost::asio::io_service io_service;
    tcp::socket *socket;
    tcp::acceptor *acceptor;
    boost::thread th;
    boost::thread th_reset;
    
    void* obj;

    void (*callback_func)(void* obj, uint8_t *data, uint64_t size);
    bool has_callback;

    void callback();

    void thread_read_func();

    void thread_write_func();

    void thread_connect_func();

    void thread_func();

    void reset();

public:
    Server();

    void start(uint32_t port_);

    void stop();

    bool write(uint8_t *data, uint64_t size);

    bool is_running();

    bool is_connected();

    bool has_data();

    bool read(uint8_t *data, uint64_t &size);

    void set_callback(void* obj, void (*callback_func)(void* obj, uint8_t *data, uint64_t size));

    uint64_t read_buffer_size();

    uint64_t write_buffer_size();

    void set_read_buffer_max_size(uint64_t max_size);

    void set_write_buffer_max_size(uint64_t max_size);

    uint64_t get_read_buffer_max_size();

    uint64_t get_write_buffer_max_size();
};

class Client
{
    std::mutex mtx;

    bool running;
    bool had_connection;
    bool need_reset;

    uint8_t *data_buffer_read;
    uint8_t *data_buffer_write;
    uint8_t *data_buffer_write_temp;
    uint64_t data_buffer_size_read;
    uint64_t data_buffer_size_write;
    uint64_t data_buffer_size_write_temp;
    uint64_t data_buffer_max_size_read = 4096;
    uint64_t data_buffer_max_size_write = 4096;

    char *ip_adr;
    uint32_t port;
    tcp::socket *socket;
    boost::asio::io_service io_service;
    boost::thread th;
    boost::thread th_reset;

    void (*callback_func)(uint8_t *data, uint64_t size);
    bool has_callback;

    void callback();

    void thread_read_func();

    void thread_write_func();

    void thread_connect_func();

    void thread_func();

    void reset();

public:
    Client();

    void start(char *ip_adr_, uint32_t port_);

    void stop();

    bool write(uint8_t *data, uint64_t size);

    bool is_running();

    bool is_connected();

    bool has_data();

    bool read(uint8_t *data, uint64_t &size);

    void set_callback(void (*callback_func)(uint8_t *data, uint64_t size));

    uint64_t read_buffer_size();

    uint64_t write_buffer_size();

    void set_read_buffer_max_size(uint64_t max_size);

    void set_write_buffer_max_size(uint64_t max_size);

    uint64_t get_read_buffer_max_size();

    uint64_t get_write_buffer_max_size();
};