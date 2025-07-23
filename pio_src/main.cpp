#include <slac/channel.hpp>
#include <slac/transport.hpp>
int main() {
    slac::transport::Link* link = nullptr; // placeholder
    slac::Channel ch(link);
    return 0;
}
