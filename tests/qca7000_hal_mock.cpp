#include "arduino_stubs.hpp"
#include "port/esp32s3/qca7000.hpp"
#include <cstring>

SPIClass* spi_used = nullptr;
int spi_cs = -1;

bool reset_called = false;

uint8_t myethtransmitbuffer[V2GTP_BUFFER_SIZE];
size_t myethtransmitlen = 0;
uint8_t myethreceivebuffer[V2GTP_BUFFER_SIZE];
size_t myethreceivelen = 0;
const char* PLC_TAG = "mock";

bool qca7000setup(SPIClass* spi, int cs) {
    spi_used = spi; spi_cs = cs; return true;
}

bool qca7000ResetAndCheck() {
    reset_called = true;
    return true;
}
uint16_t qca7000ReadInternalReg(uint16_t) { return 0; }
bool qca7000ReadSignature(uint16_t* sig, uint16_t* ver) { if(sig) *sig = 0xAA55; if(ver) *ver=1; return true; }
size_t spiQCA7000checkForReceivedData(uint8_t* dst, size_t len) {
    size_t c = myethreceivelen > len ? len : myethreceivelen;
    memcpy(dst, myethreceivebuffer, c);
    myethreceivelen = 0;
    return c;
}
bool spiQCA7000SendEthFrame(const uint8_t* f, size_t l) {
    if(l>sizeof(myethtransmitbuffer)) l = sizeof(myethtransmitbuffer);
    memcpy(myethtransmitbuffer, f, l); myethtransmitlen = l; return true;
}
bool qca7000startSlac() { return true; }
uint8_t qca7000getSlacResult() { return 0; }
void qca7000Process() {}
