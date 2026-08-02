#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>
#include <string.h>

#include "VL53L1X_platform.h"
#include "VL53L1X_api.h"
#include "VL53L1X_types.h"

/* ----- Global variables ----- */
static uint8_t buffer[VL53L1X_I2C_BUF_SIZE + 2];  
static i2c_inst_t* i2c_dev = NULL;  


/* ----- Static helper functions ----- */
static VL53L1X_Status_t Pico_I2CRead(uint16_t addr, uint8_t *buff, uint8_t len) {
  return (i2c_read_blocking(i2c_dev, addr, buff, len, false) == len) - 1;
}

static VL53L1X_Status_t Pico_I2CWrite(uint16_t addr, uint8_t *buff, uint8_t len) {
  return (i2c_write_blocking(i2c_dev, addr, buff, len, false) == len) - 1;
}

/* ----- Library functions ----- */
VL53L1X_Status_t VL53L1X_I2C_Init(uint16_t addr, i2c_inst_t* i2c_device) {
  VL53L1X_Status_t status;
  i2c_dev = i2c_device;

  // NOTA: As chamadas i2c_init e gpio_set_function foram removidas daqui[cite: 7]
  // pois o barramento i2c0 já foi inicializado corretamente no sensor_tof.cpp
  // para os pinos 4 e 5.

  uint16_t sensorId;
  status = VL53L1X_GetSensorId(addr, &sensorId);
  if (sensorId != VL53L1X_SENSOR_ID) { 
    return -1;
  }

  return status;
}

VL53L1X_Status_t VL53L1X_WriteMulti(uint16_t addr, uint16_t reg, uint8_t *data, uint32_t count) {
  if ((count + 1) > VL53L1X_I2C_BUF_SIZE)
    return -1;

  buffer[0] = 0xFF & (reg >> 8);
  buffer[1] = 0xFF & (reg		);
  memcpy(&buffer[2], data, count);

  return Pico_I2CWrite(addr, buffer, (count + 2));
}

VL53L1X_Status_t VL53L1X_ReadMulti(uint16_t addr, uint16_t reg, uint8_t *data, uint32_t count) {
  VL53L1X_Status_t status;

  if ((count + 1) > VL53L1X_I2C_BUF_SIZE)
    return -1;

  buffer[0] = reg >> 8;
  buffer[1] = reg & 0xFF;

  status = Pico_I2CWrite(addr, buffer, 2);
  if (!status) {
    data[0] = reg;
    status = Pico_I2CRead(addr, data, count);
  }
  return status;
}

VL53L1X_Status_t VL53L1X_RdWord(uint16_t addr, uint16_t index, uint16_t* data) {
  VL53L1X_Status_t status = VL53L1X_ReadMulti(addr, index, (uint8_t*)data, 2);
  *data = ntohs(*data);
  return status;
}

VL53L1X_Status_t VL53L1X_RdDWord(uint16_t addr, uint16_t index, uint32_t* data) {
  VL53L1X_Status_t status = VL53L1X_ReadMulti(addr, index, (uint8_t*)data, 4);
  *data = ntohl(*data);
  return status;
}

VL53L1X_Status_t VL53L1X_RdByte(uint16_t addr, uint16_t index, uint8_t* data) {
  return VL53L1X_ReadMulti(addr, index, data, 1);
}

VL53L1X_Status_t VL53L1X_WrByte(uint16_t addr, uint16_t index, uint8_t data) {
  return VL53L1X_WriteMulti(addr, index, (uint8_t*)&data, 1);
}

VL53L1X_Status_t VL53L1X_WrWord(uint16_t addr, uint16_t index, uint16_t data) {
  data = htons(data);
  return VL53L1X_WriteMulti(addr, index, (uint8_t*)&data, 2);
}

VL53L1X_Status_t VL53L1X_WrDWord(uint16_t addr, uint16_t index, uint32_t data) {
  data = htonl(data);
  return VL53L1X_WriteMulti(addr, index, (uint8_t*)&data, 4);
}

VL53L1X_Status_t VL53L1X_WaitMs(uint16_t addr, int32_t wait_ms) {
  sleep_ms(wait_ms);
  return 0;
}