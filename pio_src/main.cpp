#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <slac/channel.hpp>
#include <slac/transport.hpp>

#ifdef ARDUINO
static slac::transport::Link* g_link = nullptr; // placeholder
static slac::Channel* channel = nullptr;

void setup() {
    channel = new slac::Channel(g_link);
}

void loop() {
    // placeholder main loop
}
#else
int main() {
    slac::transport::Link* link = nullptr; // placeholder
    slac::Channel ch(link);
    return 0;
}
#endif
