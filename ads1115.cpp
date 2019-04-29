#include "ads1115.hpp"
#include "esp_log.h"

const double ADS1115::fsr[] = {6.144, 4.096, 2.048, 1.024, 0.512, 0.256};
const uint16_t ADS1115::sps[] = {8,16,32,64,128,250,475,860};

ADS1115::ADS1115(i2c_port_t i2c_port, slave_address_t address) :
    i2c_port(i2c_port),
    address(address),
    max_ticks(10 / portTICK_PERIOD_MS),
    changed(true),
    last_reg(MAX_REGISTER_ADDR) {
    
    config.bit.OS = 1; // always start conversion
    config.bit.MUX = MUX_0_GND;
    config.bit.PGA = FSR_4_096;
    config.bit.MODE = MODE_SINGLE;
    config.bit.DR = SPS_64;
    config.bit.COMP_MODE = 0;
    config.bit.COMP_POL = 0;
    config.bit.COMP_LAT = 0;
    config.bit.COMP_QUE = 0b11;
}

void ADS1115::setMux(mux_t mux) {
    config.bit.MUX = mux;
    changed = 1;
}

void ADS1115::setPga(fsr_t fsr) {
    config.bit.PGA = fsr;
    changed = 1;
}

void ADS1115::setMode(mode_t mode) {
    config.bit.MODE = mode;
    changed = 1;
}

void ADS1115::setSps(sps_t sps) {
    config.bit.DR = sps;
    changed = 1;
}

void ADS1115::setMaxTicks(TickType_t max_ticks) {
    max_ticks = max_ticks;
}

int16_t ADS1115::getRaw() {
    const static char* TAG = "ads1115_get_raw";
    const static uint8_t len = 2;
    
    uint8_t data[2];
    esp_err_t err;
    bool tmp; // temporary bool for reading from queue

    // see if we need to send configuration data
    if( (config.bit.MODE == MODE_SINGLE) || (changed) ) { // if it's single-ended or a setting changed
        err = writeRegister(CONFIG_REGISTER_ADDR, config.reg);

        if(err) {
            ESP_LOGE(TAG,"could not write to device: %s", esp_err_to_name(err));
            return 0;
        }

        changed = 0; // say that the data is unchanged now
    }

    // wait for 1 ms longer than the sampling rate, plus a little bit for rounding
    vTaskDelay( (( (1000/sps[config.bit.DR]) + 1) / portTICK_PERIOD_MS) + 1);

    err = readRegister(CONVERSION_REGISTER_ADDR, data, len);

    if(err) {
        ESP_LOGE(TAG,"could not read from device: %s",esp_err_to_name(err));
        return 0;
    }

    return ((uint16_t)data[0] << 8) | (uint16_t)data[1];
}

double ADS1115::getVoltage() {
  const int16_t bits = (1L<<15)-1;

  int16_t raw = getRaw();

  return (double)raw * fsr[config.bit.PGA] / (double)bits;
}

esp_err_t ADS1115::writeRegister(register_addresses_t reg, uint16_t data) {
    i2c_cmd_handle_t cmd;
    esp_err_t ret;
    uint8_t out[2];

    out[0] = data >> 8; // get 8 greater bits
    out[1] = data & 0xFF; // get 8 lower bits
  
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // generate a start command
    i2c_master_write_byte(cmd,(address<<1) | I2C_MASTER_WRITE,1); // specify address and write command
    i2c_master_write_byte(cmd,reg,1); // specify register
    i2c_master_write(cmd,out,2,1); // write it
    i2c_master_stop(cmd); // generate a stop command
    ret = i2c_master_cmd_begin(i2c_port, cmd, max_ticks); // send the i2c command
    i2c_cmd_link_delete(cmd);

    last_reg = reg; // change the internally saved register

    return ret;
}


esp_err_t ADS1115::readRegister(register_addresses_t reg, uint8_t* data, uint8_t len) {
    i2c_cmd_handle_t cmd;
    esp_err_t ret;

    if(last_reg != reg) { // if we're not on the correct register, change it
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd,(address<<1) | I2C_MASTER_WRITE,1);
        i2c_master_write_byte(cmd,reg,1);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(i2c_port, cmd, max_ticks);
        i2c_cmd_link_delete(cmd);
        last_reg = reg;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // generate start command
    i2c_master_write_byte(cmd,(address<<1) | I2C_MASTER_READ,1); // specify address and read command
    i2c_master_read(cmd, data, len, I2C_MASTER_ACK); // read all wanted data
    i2c_master_stop(cmd); // generate stop command
    ret = i2c_master_cmd_begin(i2c_port, cmd, max_ticks); // send the i2c command
    i2c_cmd_link_delete(cmd);
    
    return ret;
}