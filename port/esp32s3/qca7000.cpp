/*
 *  QCA7000 / PLC-Stamp-Micro driver – 2025-07-23
 *  -------------------------------------------------
 *  hardReset(): 10 ms LOW → 100 ms HIGH
 *               then poll signature/WRBUF every 5 ms (≤200 ms)
 *               keeps SPI @ 1 MHz until lock-in
 *  rest of logic unchanged (8 MHz after init)
 */
#include "qca7000.hpp"
#include "serial_logger.h"

const char* PLC_TAG = "PLC_IF";

/*└─└─ Endian helpers └─└─*/
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
static inline uint16_t htole16(uint16_t v) { return v; }
static inline uint16_t le16toh(uint16_t v) { return v; }
#else
static inline uint16_t htole16(uint16_t v) { return (v >> 8) | (v << 8); }
static inline uint16_t le16toh(uint16_t v) { return (v >> 8) | (v << 8); }
#endif

/*└─└─ Public scratch buffers └─└─*/
uint8_t myethtransmitbuffer[V2GTP_BUFFER_SIZE]{};
size_t  myethtransmitlen  = 0;
uint8_t myethreceivebuffer[V2GTP_BUFFER_SIZE]{};
size_t  myethreceivelen   = 0;

/*└─└─ Consts └─└─*/
static constexpr uint16_t SIG        = 0xAA55;
static constexpr uint16_t WRBUF_RST  = 0x0C5B;
static constexpr uint32_t FAST_HZ    = 8000000;
static constexpr uint32_t SLOW_HZ    = 1000000;
static constexpr uint16_t SOF_WORD   = 0xAAAA;
static constexpr uint16_t EOF_WORD   = 0x5555;
static constexpr uint16_t TX_HDR     = 8;
static constexpr uint16_t RX_HDR     = 12;
static constexpr uint16_t FTR_LEN    = 2;
static constexpr uint16_t INTR_MASK  = SPI_INT_CPU_ON | SPI_INT_PKT_AVLBL |
                                       SPI_INT_RDBUF_ERR | SPI_INT_WRBUF_ERR;

/*└─└─ Globals └─└─*/
static SPIClass* g_spi = nullptr;
static int       g_cs  = -1;
static SPISettings setSlow(SLOW_HZ, MSBFIRST, SPI_MODE3);
static SPISettings setFast(FAST_HZ, MSBFIRST, SPI_MODE3);

/*└─└─ Tiny RX ring (4 frames) └─└─*/
namespace {
struct RxEntry { size_t len; uint8_t data[V2GTP_BUFFER_SIZE]; };
static RxEntry ring[4];
static volatile uint8_t head = 0, tail = 0;

inline bool ringEmpty()               { return head == tail; }
inline void ringPush(const uint8_t* d, size_t l)
{
    noInterrupts();
    if (l > V2GTP_BUFFER_SIZE) l = V2GTP_BUFFER_SIZE;
    memcpy(ring[head].data, d, l); ring[head].len = l;
    head = (head + 1) & 3;  if (head == tail) tail = (tail + 1) & 3;
    interrupts();
}
inline bool ringPop(const uint8_t** d, size_t* l)
{
    noInterrupts();
    if (ringEmpty()) { interrupts(); return false; }
    *d = ring[tail].data; *l = ring[tail].len;  tail = (tail + 1) & 3;
    interrupts(); return true;
}
} // unnamed namespace

/*└─└─ SPI helpers └─└─*/
static inline uint16_t cmd16(bool rd,bool intr,uint16_t reg)
{ return (rd?0x8000u:0)|(intr?0x4000u:0)|(reg&0x3FFFu); }

static uint16_t spiRd16_fast(uint16_t reg)
{
    g_spi->beginTransaction(setFast);
    digitalWrite(g_cs,LOW);
    g_spi->transfer16(cmd16(true,true,reg));
    uint16_t v=g_spi->transfer16(0);
    digitalWrite(g_cs,HIGH); g_spi->endTransaction(); return v;
}

static void spiWr16_fast(uint16_t reg,uint16_t val)
{
    g_spi->beginTransaction(setFast);
    digitalWrite(g_cs,LOW);
    g_spi->transfer16(cmd16(false,true,reg));
    g_spi->transfer16(val);
    digitalWrite(g_cs,HIGH); g_spi->endTransaction();
}

/*└─└─ Hard-reset with adaptive poll └─└─*/
static bool hardReset()
{
    /* 1. pulse RST */
    pinMode(PLC_SPI_RST_PIN,OUTPUT);
    digitalWrite(PLC_SPI_RST_PIN,LOW);  delay(10);
    digitalWrite(PLC_SPI_RST_PIN,HIGH); delay(100);              // >=40 ms spec

    /* 2. helper for 1 MHz reads */
    auto slowRd16 = [&](uint16_t reg)->uint16_t{
        g_spi->beginTransaction(setSlow);
        digitalWrite(g_cs,LOW);
        g_spi->transfer16(cmd16(true,true,reg));
        uint16_t v=g_spi->transfer16(0);
        digitalWrite(g_cs,HIGH); g_spi->endTransaction(); return v;
    };

    /* 3. poll signature & WRBUF up to 200 ms */
    uint32_t t0=millis(); uint16_t sig=0, buf=0;
    do {
        sig = slowRd16(SPI_REG_SIGNATURE);
        buf = slowRd16(SPI_REG_WRBUF_SPC_AVA);
        if (sig==SIG && buf==WRBUF_RST) break;
        delay(5);
    } while (millis()-t0 < 200);

    if (sig!=SIG || buf!=WRBUF_RST) {
        ESP_LOGE(PLC_TAG,"Reset probe failed (SIG=0x%04X BUF=0x%04X)",sig,buf);
        return false;
    }
    ESP_LOGI(PLC_TAG,"Reset probe OK (SIG=0x%04X)",sig);

    /* 4. wait (optional) for CPU_ON */
    t0 = millis();
    while (!(slowRd16(SPI_REG_INTR_CAUSE)&SPI_INT_CPU_ON) && millis()-t0<80);

    /* 5. clear pending & switch to fast */
    spiWr16_fast(SPI_REG_INTR_CAUSE,0xFFFF);
    return true;
}

/*└─└─ Public helpers (unchanged) └─└─*/
uint16_t qca7000ReadInternalReg(uint8_t r){return spiRd16_fast(r<<8);}
bool qca7000ReadSignature(uint16_t* s,uint16_t* v){
    uint16_t sig=qca7000ReadInternalReg(0x1A), ver=qca7000ReadInternalReg(0x1B);
    if(s)*s=sig; if(v)*v=ver; return sig==SIG;
}

/*└─└─ TX helper └─└─*/
static bool txFrame(const uint8_t* eth,size_t ethLen)
{
    if(ethLen>1522) return false;
    size_t frameLen=ethLen;
    if(frameLen<60) frameLen=60;                     // pad short frames
    uint16_t spiLen = TX_HDR + frameLen + FTR_LEN;
    if(spiRd16_fast(SPI_REG_WRBUF_SPC_AVA)<spiLen) return false;

    spiWr16_fast(SPI_REG_BFR_SIZE,spiLen);

    g_spi->beginTransaction(setFast);
    digitalWrite(g_cs,LOW);
    g_spi->transfer16(cmd16(false,false,0));
    g_spi->transfer16(SOF_WORD); g_spi->transfer16(SOF_WORD);
    g_spi->transfer16(htole16(static_cast<uint16_t>(frameLen))); g_spi->transfer16(0);
    if(ethLen) g_spi->writeBytes(eth,ethLen);
    if(frameLen>ethLen){ uint8_t pad[60]{}; g_spi->writeBytes(pad,frameLen-ethLen); }
    g_spi->transfer16(EOF_WORD);
    digitalWrite(g_cs,HIGH); g_spi->endTransaction(); return true;
}

/*└─└─ RX fetch └─└─*/
static void fetchRx()
{
    uint16_t avail=spiRd16_fast(SPI_REG_RDBUF_BYTE_AVA);
    if(avail<RX_HDR+FTR_LEN||avail>V2GTP_BUFFER_SIZE) return;

    uint16_t requested=avail;
    spiWr16_fast(SPI_REG_BFR_SIZE,requested);

    static uint8_t buf[V2GTP_BUFFER_SIZE+2];
    g_spi->beginTransaction(setFast);
    digitalWrite(g_cs,LOW);
    g_spi->transfer16(cmd16(true,false,0));
    for(uint16_t i=0;i<avail+2;++i) buf[i]=g_spi->transfer(0);
    digitalWrite(g_cs,HIGH); g_spi->endTransaction();

    const uint8_t* p=buf+2;
    uint32_t len=(uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
    if(len!=requested){ESP_LOGE(PLC_TAG,"RX len mismatch: req=%u got=%u",requested,len);return;}
    if(memcmp(p+4,"\xAA\xAA\xAA\xAA",4)!=0) return;
    uint16_t fl=le16toh(static_cast<uint16_t>((p[9]<<8)|p[8]));
    if(p[RX_HDR+fl]!=0x55||p[RX_HDR+fl+1]!=0x55) return;
    ringPush(p+RX_HDR,fl);
}

/*└─└─ Public TX/RX API └─└─*/
bool spiQCA7000SendEthFrame(const uint8_t* f,size_t l){
    bool ok=txFrame(f,l);
    if(ok&&l<=V2GTP_BUFFER_SIZE){memcpy(myethtransmitbuffer,f,l);myethtransmitlen=l;}
    return ok;
}
size_t spiQCA7000checkForReceivedData(uint8_t* d,size_t m){
    fetchRx(); const uint8_t* s; size_t l;
    if(!ringPop(&s,&l)) return 0; size_t c=l>m?m:l; memcpy(d,s,c);
    size_t store=l>V2GTP_BUFFER_SIZE?V2GTP_BUFFER_SIZE:l;
    memcpy(myethreceivebuffer,s,store); myethreceivelen=l; return c;
}
bool myEthTransmit(const uint8_t* f,size_t l){return spiQCA7000SendEthFrame(f,l);}

/* SLAC stub */
static uint8_t g_slac=0;
bool    qca7000startSlac(){g_slac=0;return txFrame(nullptr,0);}
uint8_t qca7000getSlacResult(){fetchRx();return g_slac;}

/* ISR-poll */
void qca7000Process(){
    spiWr16_fast(SPI_REG_INTR_ENABLE,0);
    uint16_t cause=spiRd16_fast(SPI_REG_INTR_CAUSE);
    spiWr16_fast(SPI_REG_INTR_CAUSE,cause);
    spiWr16_fast(SPI_REG_INTR_ENABLE,INTR_MASK);
    if(!cause) return;
    if(cause&SPI_INT_CPU_ON){hardReset();qca7000setup(g_spi,g_cs);return;}
    if(cause&(SPI_INT_WRBUF_ERR|SPI_INT_RDBUF_ERR)){hardReset();return;}
    if(cause&SPI_INT_PKT_AVLBL) fetchRx();
}

/*└─└─ Setup └─└─*/
bool qca7000setup(SPIClass* bus,int csPin)
{
    ESP_LOGI(PLC_TAG,"QCA7000 setup: bus=%p CS=%d",bus,csPin);
    g_spi=bus; g_cs=csPin;
    g_spi->begin(SCK_PIN,MISO_PIN,MOSI_PIN,-1);
    pinMode(g_cs,OUTPUT); digitalWrite(g_cs,HIGH);

    if(!hardReset()){ ESP_LOGE(PLC_TAG,"hardReset failed – modem missing"); return false; }

    spiWr16_fast(SPI_REG_INTR_ENABLE,INTR_MASK);
    ESP_LOGI(PLC_TAG,"QCA7000 ready");
    return true;
}
bool qca7000ResetAndCheck(){return hardReset();}

