#pragma once
#include <cstdint>
#include <cstddef>
#include <string>

class SerialPort
{
public:
    SerialPort();
    ~SerialPort();

    bool open(const std::string &device, int baud);
    ssize_t read(uint8_t *buf, size_t maxlen);
    ssize_t write(const uint8_t *buf, size_t len);
    void close();

    // Pulses for Arduino-like reset behavior
    void pulse_dtr(int low_ms = 50, int after_ms = 1500);
    void pulse_rts(int low_ms = 50, int after_ms = 1500);

private:
    int fd_;
    bool configure_termios(int baud);
};
