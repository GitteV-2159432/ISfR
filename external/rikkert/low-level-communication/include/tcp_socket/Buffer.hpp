#pragma once
#include <mutex>
#include <cstring>
#include <iostream>

enum BufferType
{
    FIFO,
    LIFO
};

template <typename T>
class Buffer
{
private:
    std::mutex mtx;

    T *buffer;

    uint64_t buffer_size;
    uint64_t buffer_index;
    uint64_t max_buffer_size;

    bool allow_overflow;

    BufferType buffer_type;

    void init(uint64_t size, BufferType type);

public:
    Buffer(uint64_t size, BufferType type) { init(size, type); }

    Buffer() { init(0, FIFO); }

    Buffer(uint64_t size) { init(size, FIFO); }

    ~Buffer() { delete buffer; }

    bool push_back(void *obj, uint64_t size);

    bool push_back(void *obj) { return push_back(obj, 1); }

    bool push_back(T obj) { return push_back(&obj); }

    uint64_t get(void *obj, uint64_t size, uint64_t index);

    uint64_t get(void *obj, uint64_t size) { return get(obj, size, 0); }

    uint64_t get_all(void *obj) { return get(obj, 0); }

    bool at(void *obj, uint64_t index) { return get(obj, 1, index); }

    T at(uint64_t index);

    bool get(void *obj) { return at(obj, 0); }

    T get() { return at(0); }

    uint64_t pull(void *obj, uint64_t size);

    uint64_t pull(void *obj) { return pull(obj, 1); }

    uint64_t pull_all(void *obj) { return pull(obj, 0); }

    T pull();

    uint64_t size() { return buffer_size; }

    uint64_t max_size() { return max_buffer_size; }

    void set_overflow(bool overflow) { allow_overflow = overflow; }

    bool get_overflow() { return allow_overflow; }

    void clear();
};

template <typename T>
void Buffer<T>::init(uint64_t size, BufferType type)
{
    max_buffer_size = size;
    buffer_type = type;
    allow_overflow = false;
    buffer = (T *)malloc(sizeof(T) * max_buffer_size);
    clear();
}

template <typename T>
bool Buffer<T>::push_back(void *obj, uint64_t size)
{
    bool succes = false;
    mtx.lock();
    if (buffer_size + size > max_buffer_size && allow_overflow && size <= max_buffer_size)
    {
        buffer_size = max_buffer_size - size;
        buffer_index = (buffer_index + size) % max_buffer_size;
    }

    if (buffer_size + size <= max_buffer_size)
    {
        uint64_t i_start = (buffer_index + buffer_size) % max_buffer_size;
        uint64_t i_end = (buffer_index + buffer_size + size - 1) % max_buffer_size;

        if (i_start > i_end)
        {
            uint64_t size_1 = (max_buffer_size - i_start);
            memcpy(buffer + i_start, obj, sizeof(T) * size_1);
            memcpy(buffer, (T*)obj + size_1, sizeof(T) * (size - size_1));
        }
        else
        {
            memcpy(buffer + i_start, obj, sizeof(T) * size);
        }
        buffer_size += size;
        succes = true;
    }
    mtx.unlock();
    return succes;
}

template <typename T>
uint64_t Buffer<T>::get(void *obj, uint64_t size, uint64_t index)
{
    uint64_t get_size = 0;
    uint64_t index_ = index;
    if (index == -1)
        index_ == buffer_size - 1;

    mtx.lock();
    if (buffer_size >= size && buffer_size > 0)
    {
        uint64_t size_ = size;
        if (size == 0)
            size_ = buffer_size;

        uint64_t i_start = 0;
        uint64_t i_end = size_ - 1;
        if (buffer_type == FIFO)
        {
            i_start = (buffer_index + index_) % max_buffer_size;
            i_end = (buffer_index + size_ - 1 + index_) % max_buffer_size;
        }
        else
        {
            i_start = (buffer_index + buffer_size - index_ - size_) % max_buffer_size;
            i_end = (buffer_index + buffer_size - 1 - index_) % max_buffer_size;
        }
        if (i_start > i_end)
        {
            uint64_t size_1 = (max_buffer_size - i_start);
            memcpy(obj, buffer + i_start, sizeof(T) * size_1);
            memcpy((T*)obj + size_1, buffer, sizeof(T) * (size_ - size_1));
        }
        else
            memcpy(obj, buffer + i_start, sizeof(T) * size_);

        get_size = size;
    }
    mtx.unlock();
    return get_size;
}

template <typename T>
T Buffer<T>::at(uint64_t index)
{
    T obj;
    at(&obj, index);
    return obj;
}

template <typename T>
uint64_t Buffer<T>::pull(void *obj, uint64_t size)
{
    uint64_t get_size = 0;
    mtx.lock();
    if (buffer_size >= size && buffer_size > 0)
    {
        uint64_t size_ = size;
        if (size == 0)
            size_ = buffer_size;

        uint64_t i_start = 0;
        uint64_t i_end = size_ - 1;
        if (buffer_type == FIFO)
        {
            i_start = buffer_index;
            i_end = (buffer_index + size_ - 1) % max_buffer_size;

            buffer_index += size_;
            buffer_index = buffer_index % max_buffer_size;
        }
        else
        {
            i_start = (buffer_index + buffer_size - size_) % max_buffer_size;
            i_end = (buffer_index + buffer_size - 1) % max_buffer_size;
        }
        if (i_start > i_end)
        {
            uint64_t size_1 = (max_buffer_size - i_start);
            memcpy(obj, buffer + i_start, sizeof(T) * size_1);
            memcpy((T*)obj + size_1, buffer, sizeof(T) * (size_ - size_1));
        }
        else
            memcpy(obj, buffer + i_start, sizeof(T) * size_);

        buffer_size -= size_;
        get_size = size_;
    }
    mtx.unlock();
    return get_size;
}

template <typename T>
T Buffer<T>::pull()
{
    T obj;
    pull(&obj);
    return obj;
}

template <typename T>
void Buffer<T>::clear()
{
    mtx.lock();
    buffer_size = 0;
    buffer_index = 0;
    mtx.unlock();
}