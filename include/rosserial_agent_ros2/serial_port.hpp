#pragma once
#include <cstdint>
#include <cstddef>
#include <string>

// Simple POSIX serial port wrapper (Linux).
// Open with 8N1, no flow control, VMIN/VTIME tuned for low-latency reads.
class SerialPort
{
public:
    SerialPort();
    ~SerialPort();

    // Open device (e.g., "/dev/ttyACM0") at 'baud' (e.g., 115200).
    // Returns true on success.
    bool open(const std::string &device, int baud);

    // Read up to maxlen bytes; returns number of bytes read (>=0).
    // This call blocks for up to ~10ms (per VTIME).
    ssize_t read(uint8_t *buf, size_t maxlen);

    // Write exactly len bytes; returns bytes written (>=0).
    ssize_t write(const uint8_t *buf, size_t len);

    // Close if open.
    void close();

private:
    int fd_;
    bool configure_termios(int baud);
};
