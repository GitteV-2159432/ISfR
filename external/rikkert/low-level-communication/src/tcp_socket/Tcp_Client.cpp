#include "tcp_socket/Tcp_Client.hpp"

bool TCP_Client::is_running() { return running; }

bool TCP_Client::is_connected() { return has_connection; }

bool TCP_Client::has_data() { return data_buffer_read->size() > 0; }

// void TCP_Client::set_callback(void (*callback_func)(Buffer<uint8_t> *read_buffer)) { set_callback(callback_func, false); }
void TCP_Client::set_callback(void* obj, void (*callback_func)(void* obj, Buffer<uint8_t> *read_buffer)) { set_callback(obj, callback_func, false); }

uint64_t TCP_Client::read_buffer_size() { return data_buffer_read->size(); }

uint64_t TCP_Client::write_buffer_size() { return data_buffer_write->size(); }

void TCP_Client::set_buffer_max_size(uint64_t max_size) { init_buffers(max_size); }

uint64_t TCP_Client::get_buffer_max_size() { return data_buffer_max_size; }

Buffer<uint8_t> *TCP_Client::get_read_buffer() { return data_buffer_read; }

Buffer<uint8_t> *TCP_Client::get_write_buffer() { return data_buffer_write; }

void TCP_Client::callback()
{
    while (has_new_data)
    {
        has_new_data = false;
        (*callback_func)(this->obj, data_buffer_read);
    }
}

void TCP_Client::open()
{
    mtx.lock();
    boost::system::error_code error_code;
    socket->connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ip_adr), port), error_code);
    if (!error_code)
    {
        has_connection = true;
    }
    mtx.unlock();
}

void TCP_Client::close()
{
    mtx.lock();
    if (socket->is_open())
    {
        try
        {
            socket->close();
        }
        catch (...)
        {
        }
    }
    has_connection = false;
    mtx.unlock();
}

void TCP_Client::thread_read_func()
{
    uint8_t new_data[data_buffer_max_size];
    while (running)
    {
        sleep_us(THREAD_SLEEP_US);
        if (has_connection)
        {
            try
            {
                uint64_t n = socket->read_some(boost::asio::buffer(new_data, data_buffer_max_size));
                if (n > 0)
                {
                    data_buffer_read->push_back(new_data, n);
                    has_new_data = true;
                    if (callback_func != nullptr)
                    {
                        if (callback_new_th)
                        {
                            boost::thread thc(&TCP_Client::callback, this);
                        }
                        else
                        {
                            if (th_callback_join())
                            {
                                th_callback = boost::thread(&TCP_Client::callback, this);
                            }
                        }
                    }
                }
            }
            catch (...)
            {
                close();
            }
        }
    }
}

void TCP_Client::thread_write_func()
{
    uint8_t *data = new uint8_t[data_buffer_write->max_size()];
    while (running)
    {
        sleep_us(THREAD_SLEEP_US);
        if (has_connection)
        {
            try
            {
                if (data_buffer_write->size() > 0)
                {
                    const uint64_t size = data_buffer_write->pull_all(data);
                    socket->write_some(boost::asio::buffer(data, size));
                }
            }
            catch (...)
            {
                close();
            }
        }
    }
    delete data;
}

void TCP_Client::thread_connect_func()
{
    while (running)
    {
        sleep_us(THREAD_SLEEP_CONNECT_US);
        if (has_connection)
        {
            if (!had_connection)
            {
                had_connection = true;
                if (on_connect != nullptr)
                {
                    (*on_connect)(ip_adr.c_str());
                }
            }
        }
        else
        {
            if (had_connection)
            {
                had_connection = false;
                if (on_disconnect != nullptr)
                {
                    (*on_disconnect)(ip_adr.c_str());
                }
            }
            open();
        }
    }
}

TCP_Client::TCP_Client()
{
    running = false;
    has_connection = false;
    callback_func = nullptr;
    on_connect = nullptr;
    on_disconnect = nullptr;
    had_connection = false;
    init_buffers(4096);
    has_new_data = false;
}

void TCP_Client::start(std::string ip_adr, uint32_t port_)
{
    port = port_;
    socket = new boost::asio::ip::tcp::socket(io_service);
    this->ip_adr = ip_adr;
    endpoint = boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ip_adr), port);
    running = true;
    had_connection = false;
    th_connect = boost::thread(&TCP_Client::thread_connect_func, this);
    th_read = boost::thread(&TCP_Client::thread_read_func, this);
    th_write = boost::thread(&TCP_Client::thread_write_func, this);
}

void TCP_Client::stop()
{
    running = false;
    th_connect.detach();
    th_write.detach();
    th_read.detach();
    close();
}

bool TCP_Client::write(uint8_t *data, uint64_t size)
{
    if (has_connection)
    {
        return data_buffer_write->push_back(data, size);
    }
    return false;
}

bool TCP_Client::read(uint8_t *data, uint64_t &size)
{
    size = data_buffer_read->pull_all(data);
    return size > 0;
}

void TCP_Client::set_callback(void* obj, void (*callback_func)(void* obj, Buffer<uint8_t> *read_buffer), bool use_new_thread)
{
    this->obj = obj;
    this->callback_func = callback_func;
    callback_new_th = use_new_thread;
}

void TCP_Client::set_on_connect(void (*callback_func)(const char *ip))
{
    this->on_connect = callback_func;
}

void TCP_Client::set_on_disconnect(void (*callback_func)(const char *ip))
{
    this->on_disconnect = callback_func;
}

void TCP_Client::init_buffers(uint64_t size)
{
    if (data_buffer_read == nullptr)
    {
        delete data_buffer_read;
        delete data_buffer_write;
    }
    data_buffer_max_size = size;
    data_buffer_read = new Buffer<uint8_t>(size);
    data_buffer_write = new Buffer<uint8_t>(size);
}

bool TCP_Client::th_callback_join()
{
    if (th_callback.joinable() || th_callback.get_id() == boost::thread::id())
    {
        th_callback.join();
        return true;
    }
    return false;
}