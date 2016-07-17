#ifndef RF24_H
#define RF24_H

#include <stdint.h>

#define RF24_PACKET_LEN 5

class RF24 {
public:
    RF24(){};
    void init();
    void rx_enable(uint8_t node);
    // Returns payload length (or 0 if no packet present).
    uint8_t rx(uint8_t *buf);
    bool tx_avail();
    void tx(uint8_t node, const uint8_t *buf);
    /* -1: retry
       -2: error
        >=0: Success
        */
    int8_t tx_poll();
    void set_address(uint32_t addr);
    void wake();
    void sleep();
private:
    uint8_t address[5];
    uint8_t cmd(uint8_t cmd, const uint8_t *data_in, uint8_t *data_out, uint8_t data_len);
    void write_reg(uint8_t reg, uint8_t val);
    uint8_t read_reg(uint8_t reg);
    uint8_t read_status();
    void set_node(uint8_t reg, uint8_t node);
};

#endif
