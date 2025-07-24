#ifndef HARDWARE_SERIAL_STUB_H
#define HARDWARE_SERIAL_STUB_H
#include <deque>
#include <stdint.h>
#include <vector>
class HardwareSerial {
public:
    std::deque<uint8_t> read_queue;
    std::vector<uint8_t> written;
    void begin(unsigned long) {}
    size_t write(const uint8_t* d, size_t l) {
        written.insert(written.end(), d, d + l);
        return l;
    }
    int available() { return read_queue.size(); }
    int read() {
        if (read_queue.empty())
            return -1;
        uint8_t v = read_queue.front();
        read_queue.pop_front();
        return v;
    }
};

extern HardwareSerial Serial;
#endif
