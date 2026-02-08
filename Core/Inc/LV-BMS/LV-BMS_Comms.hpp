#ifndef COMMS_HPP
#define COMMS_HPP

#include "ST-LIB.hpp"
#include "Communications/Packets/DataPackets.hpp"

struct Comms {
    enum class Master : uint8_t {
        CONNECTED,
        DISCONNECTED
    };

    // -----------------IP's/Ports-----------------
    // -----------------Sockets-----------------
    // -----------------Packets-----------------
    // -----------------Orders-----------------
    // -----------------Functions-----------------

    static void init();
    static inline void start() {
        DataPackets::start();
    }
};

#endif // COMMS_HPP