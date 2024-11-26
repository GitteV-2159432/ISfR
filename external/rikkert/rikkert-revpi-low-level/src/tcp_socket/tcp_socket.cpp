#include "tcp_socket/tcp_socket.hpp"

void Server::callback()
{
    mtx.lock();
    const uint64_t size = data_buffer_size_read;
    uint8_t *data = (uint8_t *)malloc(size);
    memcpy(data, data_buffer_read, size);
    data_buffer_size_read = 0;
    mtx.unlock();
    (*callback_func)(this->obj, data, size);
    delete data;
}

void Client::callback()
{
    mtx.lock();
    const uint64_t size = data_buffer_size_read;
    uint8_t *data = (uint8_t *)malloc(size);
    memcpy(data, data_buffer_read, size);
    data_buffer_size_read = 0;
    mtx.unlock();
    (*callback_func)(data, size);
    delete data;
}

void Server::thread_read_func()
{
    boost::asio::streambuf receive_buffer;
    boost::system::error_code error;
    uint32_t n = boost::asio::read(*socket, receive_buffer, boost::asio::transfer_exactly(socket->available()), error);
    if (error && error != boost::asio::error::eof)
    {
        printf("Server: Failed to read\n");
        return;
    }
    const char *data = boost::asio::buffer_cast<const char *>(receive_buffer.data());
    mtx.lock();
    if (data_buffer_size_read + n > data_buffer_max_size_read)
    {
        memcpy(data_buffer_read + data_buffer_size_read, data, data_buffer_max_size_read - data_buffer_size_read);
        data_buffer_size_read = data_buffer_max_size_read;
    }
    else
    {
        memcpy(data_buffer_read + data_buffer_size_read, data, n);
        data_buffer_size_read += n;
    }
    mtx.unlock();

    if (has_callback)
    {
        boost::thread thc(&Server::callback, this);
    }
}

void Client::thread_read_func()
{
    boost::asio::streambuf receive_buffer;
    boost::system::error_code error;
    uint32_t n = boost::asio::read(*socket, receive_buffer, boost::asio::transfer_exactly(socket->available()), error);
    if (error && error != boost::asio::error::eof)
    {
        printf("Client: Failed to read\n");
        return;
    }
    const char *data = boost::asio::buffer_cast<const char *>(receive_buffer.data());
    mtx.lock();
    if (data_buffer_size_read + n > data_buffer_max_size_read)
    {
        memcpy(data_buffer_read + data_buffer_size_read, data, data_buffer_max_size_read - data_buffer_size_read);
        data_buffer_size_read = data_buffer_max_size_read;
    }
    else
    {
        memcpy(data_buffer_read + data_buffer_size_read, data, n);
        data_buffer_size_read += n;
    }
    mtx.unlock();

    if (has_callback)
    {
        boost::thread thc(&Client::callback, this);
    }
}

void Server::thread_write_func()
{
    if (data_buffer_size_write_temp == 0 && data_buffer_size_write > 0)
    {
        mtx.lock();
        data_buffer_size_write_temp = data_buffer_size_write;
        data_buffer_size_write = 0;
        memcpy(data_buffer_write_temp, data_buffer_write, data_buffer_size_write_temp);
        mtx.unlock();
    }
    if (data_buffer_size_write_temp > 0)
    {
        try
        {
            boost::asio::write(*socket, boost::asio::buffer(data_buffer_write_temp, data_buffer_size_write_temp));
            data_buffer_size_write_temp = 0;
        }
        catch (...)
        {
            need_reset = true;
        }
    }
}

void Client::thread_write_func()
{
    if (data_buffer_size_write_temp == 0 && data_buffer_size_write > 0)
    {
        mtx.lock();
        data_buffer_size_write_temp = data_buffer_size_write;
        data_buffer_size_write = 0;
        memcpy(data_buffer_write_temp, data_buffer_write, data_buffer_size_write_temp);
        mtx.unlock();
    }
    if (data_buffer_size_write_temp > 0)
    {
        try
        {
            boost::asio::write(*socket, boost::asio::buffer(data_buffer_write_temp, data_buffer_size_write_temp));
            data_buffer_size_write_temp = 0;
        }
        catch (...)
        {
            need_reset = true;
        }
    }
}

void Server::thread_connect_func()
{
    boost::system::error_code error_code;
    acceptor->accept(*socket, error_code);
    if (!error_code)
    {
        printf("Server: Connected to %s\n",socket->remote_endpoint().address().to_string().c_str());
        had_connection = true;
    }
}

void Client::thread_connect_func()
{
    boost::system::error_code error_code;
    socket->connect(tcp::endpoint(boost::asio::ip::address::from_string(ip_adr), port), error_code);
    if (!error_code)
    {
        printf("Client: Connected to %s\n",socket->remote_endpoint().address().to_string().c_str());
        had_connection = true;
    }
}

void Server::thread_func()
{
    while (running)
    {
        if (socket->is_open())
        {
            boost::this_thread::sleep_for(boost::chrono::microseconds{THREAD_SLEEP_US});
            if (socket->available())
            {
                try
                {
                    thread_read_func();
                }
                catch (...)
                {
                }
            }
            thread_write_func();
        }
        else
        {
            boost::this_thread::sleep_for(boost::chrono::microseconds{THREAD_SLEEP_US_CONNECT});
            if (had_connection)
            {
                printf("Server: Connection lost\n");
                had_connection = false;
            }
            try
            {
                thread_connect_func();
            }
            catch (...)
            {
            }
        }
    }
}

void Client::thread_func()
{
    while (running)
    {

        if (socket->is_open())
        {
            boost::this_thread::sleep_for(boost::chrono::microseconds{THREAD_SLEEP_US});
            if (socket->available())
            {
                try
                {
                    thread_read_func();
                }
                catch (...)
                {
                }
            }
            thread_write_func();
        }
        else
        {
            boost::this_thread::sleep_for(boost::chrono::microseconds{THREAD_SLEEP_US_CONNECT});
            if (had_connection)
            {
                printf("Client: Connection lost\n");
                had_connection = false;
            }
            try
            {
                thread_connect_func();
            }
            catch (...)
            {
            }
        }
    }
}

void Server::reset()
{
    while (running)
    {
        boost::this_thread::sleep_for(boost::chrono::microseconds(THREAD_SLEEP_US));
        if (need_reset)
        {
            need_reset = false;
            running = false;
            th.join();
            socket->close();
            running = true;
            th = boost::thread(&Server::thread_func, this);
        }
    }
}

void Client::reset()
{
    while (running)
    {
        boost::this_thread::sleep_for(boost::chrono::microseconds(THREAD_SLEEP_US));
        if (need_reset)
        {
            need_reset = false;
            running = false;
            th.join();
            socket->close();
            running = true;
            th = boost::thread(&Client::thread_func, this);
        }
    }
}

Server::Server()
{
    running = false;
    need_reset = false;
    data_buffer_size_read = 0;
    data_buffer_size_write = 0;
    data_buffer_size_write_temp = 0;
    has_callback = false;
    had_connection = false;
    data_buffer_read = (uint8_t *)malloc(data_buffer_max_size_read);
    data_buffer_write = (uint8_t *)malloc(data_buffer_max_size_write);
    data_buffer_write_temp = (uint8_t *)malloc(data_buffer_max_size_write);
}

Client::Client()
{
    running = false;
    need_reset = false;
    data_buffer_size_read = 0;
    data_buffer_size_write = 0;
    data_buffer_size_write_temp = 0;
    has_callback = false;
    had_connection = false;
    data_buffer_read = (uint8_t *)malloc(data_buffer_max_size_read);
    data_buffer_write = (uint8_t *)malloc(data_buffer_max_size_write);
    data_buffer_write_temp = (uint8_t *)malloc(data_buffer_max_size_write);
}

void Server::start(uint32_t port_)
{
    port = port_;
    socket = new tcp::socket(io_service);
    acceptor = new tcp::acceptor(io_service, tcp::endpoint(tcp::v4(), port));
    running = true;
    th_reset = boost::thread(&Server::reset, this);
    th = boost::thread(&Server::thread_func, this);
    printf("Server: Started\n");
}

void Client::start(char *ip_adr_, uint32_t port_)
{
    ip_adr = ip_adr_;
    port = port_;
    socket = new tcp::socket(io_service);
    running = true;
    th_reset = boost::thread(&Client::reset, this);
    th = boost::thread(&Client::thread_func, this);
    printf("Client: Started\n");
}

void Server::stop()
{
    running = false;
    had_connection = false;
    th.join();
    th_reset.join();
    socket->close();
    printf("Server: Stopped\n");
}

void Client::stop()
{
    running = false;
    had_connection = false;
    th.join();
    th_reset.join();
    socket->close();
    printf("Client: Stopped\n");
}

bool Server::write(uint8_t *data, uint64_t size)
{
    if (socket->is_open())
    {
        mtx.lock();
        if (data_buffer_size_write + size > data_buffer_max_size_write)
        {
            mtx.unlock();
            return false;
        }
        else
        {
            memcpy(data_buffer_write + data_buffer_size_write, data, size);
            data_buffer_size_write += size;
        }
        mtx.unlock();
        return true;
    }
    return false;
}

bool Client::write(uint8_t *data, uint64_t size)
{
    if (socket->is_open())
    {
        mtx.lock();
        if (data_buffer_size_write + size > data_buffer_max_size_write)
        {
            mtx.unlock();
            return false;
        }
        else
        {
            memcpy(data_buffer_write + data_buffer_size_write, data, size);
            data_buffer_size_write += size;
        }
        mtx.unlock();
        return true;
    }
    return false;
}

bool Server::is_running()
{
    return running;
}

bool Client::is_running()
{
    return running;
}

bool Server::is_connected()
{
    return socket->is_open();
}

bool Client::is_connected()
{
    return socket->is_open();
}

bool Server::has_data()
{
    return data_buffer_size_read > 0;
}

bool Client::has_data()
{
    return data_buffer_size_read > 0;
}

bool Server::read(uint8_t *data, uint64_t &size)
{
    mtx.lock();
    if (has_data())
    {
        size = data_buffer_size_read;
        memcpy(data, data_buffer_read, size);
        data_buffer_size_read = 0;
        mtx.unlock();
        return true;
    }
    mtx.unlock();
    return false;
}

bool Client::read(uint8_t *data, uint64_t &size)
{
    mtx.lock();
    if (has_data())
    {
        size = data_buffer_size_read;
        memcpy(data, data_buffer_read, size);
        data_buffer_size_read = 0;
        mtx.unlock();
        return true;
    }
    mtx.unlock();
    return false;
}

void Server::set_callback(void* obj, void (*callback_func)(void* obj, uint8_t *data, uint64_t size))
{
    this->obj = obj;
    this->callback_func = callback_func;
    has_callback = true;
}

void Client::set_callback(void (*callback_func)(uint8_t *data, uint64_t size))
{
    this->callback_func = callback_func;
    has_callback = true;
}

uint64_t Server::get_read_buffer_max_size()
{
    return data_buffer_max_size_read;
}

uint64_t Client::get_read_buffer_max_size()
{
    return data_buffer_max_size_read;
}

uint64_t Server::get_write_buffer_max_size()
{
    return data_buffer_max_size_write;
}

uint64_t Client::get_write_buffer_max_size()
{
    return data_buffer_max_size_write;
}

uint64_t Server::read_buffer_size()
{
    return data_buffer_size_read;
}

uint64_t Client::read_buffer_size()
{
    return data_buffer_size_read;
}

uint64_t Server::write_buffer_size()
{
    return data_buffer_size_write;
}

uint64_t Client::write_buffer_size()
{
    return data_buffer_size_write;
}

void Server::set_read_buffer_max_size(uint64_t max_size)
{
    data_buffer_max_size_read = max_size;
}

void Client::set_read_buffer_max_size(uint64_t max_size)
{
    data_buffer_max_size_read = max_size;
}

void Server::set_write_buffer_max_size(uint64_t max_size)
{
    data_buffer_max_size_write = max_size;
}

void Client::set_write_buffer_max_size(uint64_t max_size)
{
    data_buffer_max_size_write = max_size;
}