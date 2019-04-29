#ifndef ADS1115_HPP_
#define ADS1115_HPP_

#include <cstdint>
#include "driver/i2c.h"

class ADS1115 {
public:
    typedef enum {
        ADDR_PIN_GROUND = 0x48,
        ADDR_PIN_VDD = 0x49,
        ADDR_PIN_SDA = 0x4a,
        ADDR_PIN_SCL = 0x4b
    } slave_address_t;

    typedef enum { // register address
        CONVERSION_REGISTER_ADDR = 0,
        CONFIG_REGISTER_ADDR,
        LO_THRESH_REGISTER_ADDR,
        HI_THRESH_REGISTER_ADDR,
        MAX_REGISTER_ADDR
    } register_addresses_t;

    typedef enum { // multiplex options
        MUX_0_1 = 0,
        MUX_0_3,
        MUX_1_3,
        MUX_2_3,
        MUX_0_GND,
        MUX_1_GND,
        MUX_2_GND,
        MUX_3_GND,
    } mux_t;

    typedef enum { // full-scale resolution options
        FSR_6_144 = 0,
        FSR_4_096,
        FSR_2_048,
        FSR_1_024,
        FSR_0_512,
        FSR_0_256,
    } fsr_t;

    typedef enum { // samples per second
        SPS_8 = 0,
        SPS_16,
        SPS_32,
        SPS_64,
        SPS_128,
        SPS_250,
        SPS_475,
        SPS_860
    } sps_t;

    typedef enum {
        MODE_CONTINUOUS = 0,
        MODE_SINGLE
    } mode_t;

    typedef union { // configuration register
        struct {
            uint16_t COMP_QUE:2;  // bits 0..  1  Comparator queue and disable
            uint16_t COMP_LAT:1;  // bit  2       Latching Comparator
            uint16_t COMP_POL:1;  // bit  3       Comparator Polarity
            uint16_t COMP_MODE:1; // bit  4       Comparator Mode
            uint16_t DR:3;        // bits 5..  7  Data rate
            uint16_t MODE:1;      // bit  8       Device operating mode
            uint16_t PGA:3;       // bits 9..  11 Programmable gain amplifier configuration
            uint16_t MUX:3;       // bits 12.. 14 Input multiplexer configuration
            uint16_t OS:1;        // bit  15      Operational status or single-shot conversion start
        } bit;
        uint16_t reg;
    } config_register_t;

    static const double fsr[6];
    static const uint16_t sps[8];

    ADS1115(i2c_port_t i2c_port, slave_address_t address);

    void setMux(mux_t mux); // set multiplexer
    void setPga(fsr_t fsr); // set fsr
    void setMode(mode_t mode); // set read mode
    void setSps(sps_t sps); // set sampling speed
    void setMaxTicks(TickType_t max_ticks); // maximum wait ticks for i2c bus

    int16_t getRaw(); // get voltage in bits
    double getVoltage(); // get voltage in volts

private:
    i2c_port_t i2c_port;
    slave_address_t address;
    TickType_t max_ticks; // maximum wait ticks for i2c bus
    bool changed; // save if a value was changed or not
    
    register_addresses_t last_reg; // save last accessed register
    config_register_t config;
    
    esp_err_t writeRegister(register_addresses_t reg, uint16_t data);
    esp_err_t readRegister(register_addresses_t reg, uint8_t* data, uint8_t len);
};

#endif