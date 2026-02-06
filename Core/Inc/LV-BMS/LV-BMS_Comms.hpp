#ifndef COMMS_HPP
#define COMMS_HPP

#include "ST-LIB.hpp"

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
    static void add_packets();
    static void add_orders();

    static void turn_on_pfm_callback();
    static void turn_off_pfm_callback();
    static void set_pfm_frequency_callback();
    static void set_pfm_dead_time_callback();

    // -----------------Flags-----------------
    static inline bool received_turn_on_pfm{};
    static inline bool received_turn_off_pfm{};
    static inline bool received_set_pfm_frequency{};
    static inline bool received_set_pfm_dead_time{};

};

#endif // COMMS_HPP