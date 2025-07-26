#include <cassert>
#include <slac/channel.hpp>
#include <slac/transport.hpp>

class DummyLink : public slac::transport::Link {
public:
    bool open() override {
        return true;
    }
    slac::transport::LinkError write(const uint8_t*, size_t, uint32_t) override {
        return slac::transport::LinkError::Ok;
    }
    slac::transport::LinkError read(uint8_t*, size_t, size_t* out_len, uint32_t) override {
        if (out_len)
            *out_len = 0;
        return slac::transport::LinkError::Ok;
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
    return 0;
}
