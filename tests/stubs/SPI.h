#ifndef SPI_H_STUB
#define SPI_H_STUB
#include <deque>
#include <stdint.h>
#include <vector>
#define MSBFIRST  1
#define SPI_MODE3 3
class SPISettings {
public:
    SPISettings(uint32_t, uint8_t, uint8_t) {
    }
};

class SPIClass {
public:
    std::deque<uint16_t> read16_queue;
    std::deque<uint8_t> read_queue;
    std::vector<uint8_t> written;
    std::vector<uint16_t> transferred16;
    std::vector<uint8_t> transferred;
    void begin() {
    }
    void beginTransaction(const SPISettings&) {
    }
    void endTransaction() {
    }
    uint16_t transfer16(uint16_t v) {
        transferred16.push_back(v);
        if (read16_queue.empty())
            return 0;
        uint16_t r = read16_queue.front();
        read16_queue.pop_front();
        return r;
    }
    uint8_t transfer(uint8_t v) {
        transferred.push_back(v);
        if (read_queue.empty())
            return 0;
        uint8_t r = read_queue.front();
        read_queue.pop_front();
        return r;
    }
    void writeBytes(const uint8_t* d, size_t l) {
        written.insert(written.end(), d, d + l);
    }
};

extern SPIClass SPI;
#endif
