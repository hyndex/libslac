#include <cassert>
#include <slac/channel.hpp>
#include <slac/transport.hpp>

class DummyLink : public slac::transport::Link {
public:
    bool open() override { return true; }
    bool write(const uint8_t*, size_t, uint32_t) override { return true; }
    bool read(uint8_t*, size_t, size_t* out_len, uint32_t) override {
        if (out_len)
            *out_len = 0;
        return true; // emulate timeout with zero bytes
    }
    const uint8_t* mac() const override {
        static const uint8_t mac[6] = {0};
        return mac;
    }
};

int main() {
    DummyLink link;
    slac::Channel channel(&link);
    assert(channel.open());
    slac::messages::HomeplugMessage msg;
    bool ok = channel.read(msg, 10);
    assert(!ok);
    assert(channel.got_timeout());
    return 0;
}
