#include "arduino_stubs.hpp"
#include "port/esp32s3/qca7000.hpp"
#include <cstring>

SPIClass* spi_used = nullptr;
int spi_cs = -1;
int spi_rst = -1;

bool reset_called = false;

uint8_t myethtransmitbuffer[V2GTP_BUFFER_SIZE];
size_t myethtransmitlen = 0;
uint8_t myethreceivebuffer[V2GTP_BUFFER_SIZE];
size_t myethreceivelen = 0;
const char* PLC_TAG = "mock";

uint16_t mock_signature = 0;
uint16_t mock_wrbuf = 0;
uint16_t mock_intr_cause = 0;

bool qca7000setup(SPIClass* spi, int cs, int rst) {
    spi_used = spi; spi_cs = cs; spi_rst = rst; return true;
}

void qca7000teardown() { spi_used = nullptr; }

bool qca7000ResetAndCheck() {
    reset_called = true;
    return true;
}
uint16_t qca7000ReadInternalReg(uint16_t reg) {
    if (reg == SPI_REG_SIGNATURE)
        return mock_signature;
    if (reg == SPI_REG_WRBUF_SPC_AVA)
        return mock_wrbuf;
    if (reg == SPI_REG_INTR_CAUSE)
        return mock_intr_cause;
    return 0;
}
bool qca7000CheckAlive() {
    uint16_t sig = qca7000ReadInternalReg(SPI_REG_SIGNATURE);
    (void)qca7000ReadInternalReg(SPI_REG_WRBUF_SPC_AVA);
    uint16_t cause = qca7000ReadInternalReg(SPI_REG_INTR_CAUSE);
    return sig == 0xAA55 && (cause & SPI_INT_CPU_ON);
}
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
void qca7000ProcessSlice(uint32_t) {}
bool qca7000DriverFatal() { return false; }
void qca7000SetErrorCallback(qca7000_error_cb_t, void*, bool*) {}
void qca7000SetNmk(const uint8_t[slac::defs::NMK_LEN]) {}
