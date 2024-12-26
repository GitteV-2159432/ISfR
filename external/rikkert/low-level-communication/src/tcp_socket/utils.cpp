#include "tcp_socket/utils.hpp"

uint64_t millis()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

uint64_t micros()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void sleep_us(uint64_t us)
{
    boost::this_thread::sleep_for(boost::chrono::microseconds(us));
}

void sleep_ms(uint64_t ms)
{
    boost::this_thread::sleep_for(boost::chrono::milliseconds(ms));
}

void sleep_s(uint64_t s)
{
    boost::this_thread::sleep_for(boost::chrono::seconds(s));
}

float constrain(float value, float min, float max)
{
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}

float deadzone(float axis_in, float deadzone_mid, float deadzone_end)
{
    float axis_out = 0;
    if (axis_in > deadzone_mid)
        axis_out = (axis_in - deadzone_mid) / (1 - deadzone_mid - deadzone_end);

    if (axis_in < -deadzone_mid)
        axis_out = (axis_in + deadzone_mid) / (1 - deadzone_mid - deadzone_end);

    return constrain(axis_out, -1, 1);
}

float map(float value_in, float in_min, float in_max, float out_min, float out_max)
{
    float out = (out_max - out_min) * (value_in - in_min) / (in_max - in_min) + out_min;
    return constrain(out, out_min, out_max);
}

std::string get_timestamp()
{
    char ts[256];
    std::time_t now = std::time(0);
    std::tm *ltm = std::localtime(&now);
    sprintf(ts, "%4d%2d%2d_%2d%2d%2d", ltm->tm_year + 1900, ltm->tm_mon + 1, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
    for (uint8_t i = 0; i < strlen(ts); i++)
    {
        if (ts[i] == ' ')
            ts[i] = '0';
    }
    return std::string(ts);
}

std::string to_hex(uint8_t byte)
{
    std::string s = "0x";
    uint8_t h1 = byte >> 4;
    uint8_t h2 = byte & 0x0f;
    s += (h1 > 9 ? (char)(h1 - 10 + 'A') : (char)(h1 + '0'));
    s += (h2 > 9 ? (char)(h2 - 10 + 'A') : (char)(h2 + '0'));
    return s;
}

std::string data_to_hex(uint8_t *buf, uint64_t size, const uint16_t PRINT_WIDTH)
{
    std::string line = " : ";
    std::string lines = "";
    for (uint64_t i = 0; i < size; i++)
    {
        uint8_t byte = buf[i];

        line += (byte > 32 && byte < 127) ? (char)byte : '.';

        lines.append(to_hex(byte)).append(" ");

        if (i % PRINT_WIDTH == PRINT_WIDTH - 1 || i == size - 1)
        {
            for (uint8_t j = i % PRINT_WIDTH + 1; j < PRINT_WIDTH; j++)
            {
                lines.append("     ");
                // line += '.';
            }
            lines.append(line).append("\n");
            line = " : ";
        }
    }
    return lines;
}

void flip_data(uint8_t *data_in, uint8_t *data_out, uint64_t size)
{
    for (uint64_t i = 0; i < size; i++)
    {
        data_out[size - i - 1] = data_in[i];
    }
}

void flip_data(uint8_t *data, uint64_t size)
{
    uint8_t *temp = new uint8_t[size];
    for (uint64_t i = 0; i < size; i++)
    {
        temp[size - i - 1] = data[i];
    }
    memcpy(data, temp, size);
    delete temp;
}

std::string parse_bytes(uint64_t bytes)
{
    char buf[256];
    uint8_t divider = 0;
    if (bytes > 1000)
        divider = 1;
    if (bytes > 1000000)
        divider = 2;
    if (bytes > 1000000000)
        divider = 3;
    float div = powf(1000, (float)divider);
    sprintf(buf, "%.2f %s", (float)bytes / div, FE[divider].c_str());
    return std::string(buf);
}

void memcpy_flipped(uint8_t *__restrict__ __dest, const uint8_t *__restrict__ __src, size_t __n)
{
    for (size_t i = 0; i < __n; i++)
    {
        __dest[i] = __src[__n - i - 1];
    }
}

bool is_pi()
{
#ifdef __arm__
    return true;
#else
    return false;
#endif
}

bool mount_usb()
{
    bool usb_mounted = false;
    if (boost::filesystem::exists("/dev/sda1"))
    {
        system("sudo mount -t exfat /dev/sda1 /mnt/usb0 -o umask=000");
        return true;
    }
    else if (boost::filesystem::exists("/dev/sdb1"))
    {
        system("sudo mount -t exfat /dev/sdb1 /mnt/usb0 -o umask=000");
        return true;
    }
    return false;
}

void check_path(const char filepath[])
{
    if (!boost::filesystem::exists(filepath))
        system(std::string("sudo mkdir ").append(filepath).c_str());
}

void unmount_usb()
{
    if (boost::filesystem::exists("/mnt/usb0"))
        system("sudo umount /mnt/usb0");
}