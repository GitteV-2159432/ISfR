#pragma once

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include <ctime>
#include <fstream>

#ifndef THREAD_SLEEP_US
#define THREAD_SLEEP_US 100
#endif
#ifndef THREAD_SLEEP_CONNECT_US
#define THREAD_SLEEP_CONNECT_US 10000
#endif

const std::string FE[] = {"b", "kb", "mb", "gb"};

uint64_t millis();

uint64_t micros();

void sleep_us(uint64_t us);

void sleep_ms(uint64_t ms);

void sleep_s(uint64_t s);

float constrain(float value, float min, float max);

float deadzone(float axis_in, float deadzone_mid, float deadzone_end);

float map(float value_in, float in_min, float in_max, float out_min, float out_max);

std::string get_timestamp();

std::string to_hex(uint8_t byte);

std::string data_to_hex(uint8_t *buf, uint64_t size, const uint16_t PRINT_WIDTH);

void flip_data(uint8_t *data_in, uint8_t *data_out, uint64_t size);

void flip_data(uint8_t *data, uint64_t size);

std::string parse_bytes(uint64_t bytes);

void memcpy_flipped(uint8_t *__restrict__ __dest, const uint8_t *__restrict__ __src, size_t __n);

bool is_pi();

bool mount_usb();

void check_path(const char filepath[]);

void unmount_usb();