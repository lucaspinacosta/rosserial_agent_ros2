#include "rosserial_agent_ros2/serial_port.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cstring>
#include <chrono>
#include <thread>

namespace
{
    speed_t baud_to_flag(int baud)
    {
        switch (baud)
        {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
#ifdef B921600
        case 921600:
            return B921600;
#endif
        default:
            return B115200;
        }
    }
}

SerialPort::SerialPort() : fd_(-1) {}
SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string &device, int baud)
{
    close();
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
        return false;

    if (!configure_termios(baud))
    {
        close();
        return false;
    }

    // switch to blocking reads (VMIN/VTIME handle latency)
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
    return true;
}

bool SerialPort::configure_termios(int baud)
{
    struct termios tio{};
    if (tcgetattr(fd_, &tio) != 0)
        return false;

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS; // no HW flow
    tio.c_cflag &= ~PARENB;  // 8N1
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

    // VMIN/VTIME: return quickly; adjust if you want
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1; // 0.1 s max block

    speed_t spd = baud_to_flag(baud);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    if (tcsetattr(fd_, TCSANOW, &tio) != 0)
        return false;
    tcflush(fd_, TCIOFLUSH);
    return true;
}

ssize_t SerialPort::read(uint8_t *buf, size_t maxlen)
{
    if (fd_ < 0)
        return -1;
    return ::read(fd_, buf, maxlen);
}

ssize_t SerialPort::write(const uint8_t *buf, size_t len)
{
    if (fd_ < 0)
        return -1;
    return ::write(fd_, buf, len);
}

void SerialPort::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

void SerialPort::pulse_dtr(int low_ms, int after_ms)
{
    if (fd_ < 0)
        return;
#ifdef TIOCM_DTR
    int status = 0;
    if (ioctl(fd_, TIOCMGET, &status) == 0)
    {
        status &= ~TIOCM_DTR;
        ioctl(fd_, TIOCMSET, &status);
        std::this_thread::sleep_for(std::chrono::milliseconds(low_ms));
        status |= TIOCM_DTR;
        ioctl(fd_, TIOCMSET, &status);
    }
#endif
    if (after_ms > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(after_ms));
}

void SerialPort::pulse_rts(int low_ms, int after_ms)
{
    if (fd_ < 0)
        return;
#ifdef TIOCM_RTS
    int status = 0;
    if (ioctl(fd_, TIOCMGET, &status) == 0)
    {
        status &= ~TIOCM_RTS;
        ioctl(fd_, TIOCMSET, &status);
        std::this_thread::sleep_for(std::chrono::milliseconds(low_ms));
        status |= TIOCM_RTS;
        ioctl(fd_, TIOCMSET, &status);
    }
#endif
    if (after_ms > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(after_ms));
}
