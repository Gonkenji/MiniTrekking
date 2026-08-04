#include "VL53L1X.h"
#include "pico/stdlib.h"

/* =========================================================================
   IMPLEMENTAÇÃO DE PLATAFORMA (baseado em VL53L1X_platform.c)
   ========================================================================= */

static uint8_t buffer[VL53L1X_I2C_BUF_SIZE + 2];  
static i2c_inst_t* i2c_dev = NULL;  

static VL53L1X_Status_t Pico_I2CRead(uint16_t addr, uint8_t *buff, uint8_t len) {
  return (i2c_read_blocking(i2c_dev, addr, buff, len, false) == len) - 1;
}

static VL53L1X_Status_t Pico_I2CWrite(uint16_t addr, uint8_t *buff, uint8_t len) {
  return (i2c_write_blocking(i2c_dev, addr, buff, len, false) == len) - 1;
}

VL53L1X_Status_t VL53L1X_I2C_Init(uint16_t addr, i2c_inst_t* i2c_device) {
  VL53L1X_Status_t status;
  i2c_dev = i2c_device;

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
  // Troca a ordem dos bytes de 16 bits
  *data = __builtin_bswap16(*data);
  return status;
}

VL53L1X_Status_t VL53L1X_RdDWord(uint16_t addr, uint16_t index, uint32_t* data) {
  VL53L1X_Status_t status = VL53L1X_ReadMulti(addr, index, (uint8_t*)data, 4);
  // Troca a ordem dos bytes de 32 bits
  *data = __builtin_bswap32(*data);
  return status;
}

VL53L1X_Status_t VL53L1X_RdByte(uint16_t addr, uint16_t index, uint8_t* data) {
  return VL53L1X_ReadMulti(addr, index, data, 1);
}

VL53L1X_Status_t VL53L1X_WrByte(uint16_t addr, uint16_t index, uint8_t data) {
  return VL53L1X_WriteMulti(addr, index, (uint8_t*)&data, 1);
}

VL53L1X_Status_t VL53L1X_WrWord(uint16_t addr, uint16_t index, uint16_t data) {
  // Troca a ordem dos bytes de 16 bits antes de enviar
  data = __builtin_bswap16(data);
  return VL53L1X_WriteMulti(addr, index, (uint8_t*)&data, 2);
}

VL53L1X_Status_t VL53L1X_WrDWord(uint16_t addr, uint16_t index, uint32_t data) {
  // Troca a ordem dos bytes de 32 bits antes de enviar
  data = __builtin_bswap32(data);
  return VL53L1X_WriteMulti(addr, index, (uint8_t*)&data, 4);
}

VL53L1X_Status_t VL53L1X_WaitMs(uint16_t addr, int32_t wait_ms) {
  sleep_ms(wait_ms);
  return 0;
}

/* =========================================================================
   IMPLEMENTAÇÃO DE API (baseado em VL53L1X_api.c)
   ========================================================================= */

static const uint8_t status_rtn[24] = { 255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
  255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
  255, 255, 11, 12
};

const uint8_t VL53L1X_DEFAULT_CONFIGURATION[] = {
  0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08,
  0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x0F,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02,
  0x0a, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8,
  0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01,
  0xcc, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08,
  0xb8, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x89, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
  0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00,
  0x00
};

VL53L1X_Status_t VL53L1X_GetSWVersion(VL53L1X_Version_t *pVersion) {
  pVersion->major = 1; 
  pVersion->minor = 0;
  pVersion->build = 0;
  pVersion->revision = 0;
  return 0;
}

VL53L1X_Status_t VL53L1X_SetI2CAddress(uint16_t dev, uint8_t new_address) {
  return VL53L1X_WrByte(dev, 0x0001 /*VL53L1X_I2C_SLAVE__DEVICE_ADDRESS*/, new_address >> 1);
}

VL53L1X_Status_t VL53L1X_SensorInit(uint16_t dev) {
  VL53L1X_Status_t status;

  for (uint8_t tmp_addr = 0x2D; tmp_addr <= 0x87; tmp_addr++){
    status = VL53L1X_WrByte(dev, tmp_addr, VL53L1X_DEFAULT_CONFIGURATION[tmp_addr - 0x2D]);
  }

  status = VL53L1X_StartRanging(dev);

  uint8_t tmp;
  do {
    status = VL53L1X_CheckForDataReady(dev, &tmp);
  } while (tmp == 0);

  status = VL53L1X_ClearInterrupt(dev);
  status = VL53L1X_StopRanging(dev);
  status = VL53L1X_WrByte(dev, 0x0008 /*VL53L1X_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND*/, 0x09);
  status = VL53L1X_WrByte(dev, 0x0B, 0); 
  return status;
}

VL53L1X_Status_t VL53L1X_ClearInterrupt(uint16_t dev) {
  return VL53L1X_WrByte(dev, 0x0086 /*SYSTEM__INTERRUPT_CLEAR*/, 0x01);
}

VL53L1X_Status_t VL53L1X_SetInterruptPolarity(uint16_t dev, uint8_t nPol) {
  uint8_t tmp;
  VL53L1X_Status_t status;

  status = VL53L1X_RdByte(dev, 0x0030 /*GPIO_HV_MUX__CTRL*/, &tmp);
  tmp = tmp & 0xEF;
  status = VL53L1X_WrByte(dev, 0x0030, tmp | (!(nPol & 1)) << 4);
  return status;
}

VL53L1X_Status_t VL53L1X_GetInterruptPolarity(uint16_t dev, uint8_t* iPol) {
  uint8_t tmp;
  VL53L1X_Status_t status;

  status = VL53L1X_RdByte(dev, 0x0030 /*GPIO_HV_MUX__CTRL*/, &tmp);
  tmp = tmp & 0x10;
  *iPol = !(tmp>>4);
  return status;
}

VL53L1X_Status_t VL53L1X_StartRanging(uint16_t dev) {
  return VL53L1X_WrByte(dev, 0x0087 /*SYSTEM__MODE_START*/, 0x40);
}

VL53L1X_Status_t VL53L1X_StopRanging(uint16_t dev) {
  return VL53L1X_WrByte(dev, 0x0087 /*SYSTEM__MODE_START*/, 0x00);
}

VL53L1X_Status_t VL53L1X_CheckForDataReady(uint16_t dev, uint8_t* isDataReady) {
  uint8_t tmp, iPol;
  VL53L1X_Status_t status;

  status = VL53L1X_GetInterruptPolarity(dev, &iPol);
  status = VL53L1X_RdByte(dev, 0x0031 /*GPIO__TIO_HV_STATUS*/, &tmp);

  if (status == 0)
    *isDataReady = (tmp & 0x1) == iPol;
  return status;
}

VL53L1X_Status_t VL53L1X_SetTimingBudgetInMs(uint16_t dev, uint16_t timingBudgetMs) {
  uint16_t DM;
  uint32_t rangeA, rangeB;
  VL53L1X_Status_t status;

  status = VL53L1X_GetDistanceMode(dev, &DM);
  if (status != 0)
    return 1;

  if (DM == 1) {
    switch (timingBudgetMs) {
      case 15: rangeA = 0x001D; rangeB = 0x0027; break;
      case 20: rangeA = 0x0051; rangeB = 0x006E; break;
      case 33: rangeA = 0x00D6; rangeB = 0x006E; break;
      case 50: rangeA = 0x01AE; rangeB = 0x01E8; break;
      case 100: rangeA = 0x02E1; rangeB = 0x0388; break;
      case 200: rangeA = 0x03E1; rangeB = 0x0496; break;
      case 500: rangeA = 0x0591; rangeB = 0x05C1; break;
      default: return 1;
    }
  } else {
    switch (timingBudgetMs) {
      case 20: rangeA = 0x001E; rangeB = 0x0022; break;
      case 33: rangeA = 0x0060; rangeB = 0x006E; break;
      case 50: rangeA = 0x00AD; rangeB = 0x00C6; break;
      case 100: rangeA = 0x01CC; rangeB = 0x01EA; break;
      case 200: rangeA = 0x02D9; rangeB = 0x02F8; break;
      case 500: rangeA = 0x048F; rangeB = 0x04A4; break;
      default: return 1;
    }
  }

  status |= VL53L1X_WrWord(dev, 0x005E /*RANGE_CONFIG__TIMEOUT_MACROP_A_HI*/, rangeA);
  status |= VL53L1X_WrWord(dev, 0x0061 /*RANGE_CONFIG__TIMEOUT_MACROP_B_HI*/, rangeB);
  return status;
}

VL53L1X_Status_t VL53L1X_GetTimingBudgetInMs(uint16_t dev, uint16_t* timingBudget) {
  uint16_t Temp;
  VL53L1X_Status_t status = 0;

  status = VL53L1X_RdWord(dev, 0x005E /*RANGE_CONFIG__TIMEOUT_MACROP_A_HI*/, &Temp);
  switch (Temp) {
    case 0x001D: *timingBudget = 15; break;
    case 0x0051: case 0x001E: *timingBudget = 20; break;
    case 0x00D6: case 0x0060: *timingBudget = 33; break;
    case 0x01AE: case 0x00AD: *timingBudget = 50; break;
    case 0x02E1: case 0x01CC: *timingBudget = 100; break;
    case 0x03E1: case 0x02D9: *timingBudget = 200; break;
    case 0x0591: case 0x048F: *timingBudget = 500; break;
    default: status = 1; *timingBudget = 0;
  }
  return status;
}

VL53L1X_Status_t VL53L1X_SetDistanceMode(uint16_t dev, uint16_t DM) {
  uint16_t TB;
  VL53L1X_Status_t status;

  status = VL53L1X_GetTimingBudgetInMs(dev, &TB);
  if (status != 0) return -1;
  switch (DM) {
    case 1:
      status |= VL53L1X_WrByte(dev, 0x004B /*PHASECAL_CONFIG__TIMEOUT_MACROP*/, 0x14);
      status |= VL53L1X_WrByte(dev, 0x0060 /*RANGE_CONFIG__VCSEL_PERIOD_A*/, 0x07);
      status |= VL53L1X_WrByte(dev, 0x0063 /*RANGE_CONFIG__VCSEL_PERIOD_B*/, 0x05);
      status |= VL53L1X_WrByte(dev, 0x0069 /*RANGE_CONFIG__VALID_PHASE_HIGH*/, 0x38);
      status |= VL53L1X_WrWord(dev, 0x0078 /*SD_CONFIG__WOI_SD0*/, 0x0705);
      status |= VL53L1X_WrWord(dev, 0x007A /*SD_CONFIG__INITIAL_PHASE_SD0*/, 0x0606);
      break;
    case 2:
      status |= VL53L1X_WrByte(dev, 0x004B, 0x0A);
      status |= VL53L1X_WrByte(dev, 0x0060, 0x0F);
      status |= VL53L1X_WrByte(dev, 0x0063, 0x0D);
      status |= VL53L1X_WrByte(dev, 0x0069, 0xB8);
      status |= VL53L1X_WrWord(dev, 0x0078, 0x0F0D);
      status |= VL53L1X_WrWord(dev, 0x007A, 0x0E0E);
      break;
    default: status = -1; break;
  }

  if (status == 0) status |= VL53L1X_SetTimingBudgetInMs(dev, TB);
  return status;
}

VL53L1X_Status_t VL53L1X_GetDistanceMode(uint16_t dev, uint16_t* DM) {
  uint8_t tDM;
  VL53L1X_Status_t status;

  status = VL53L1X_RdByte(dev, 0x004B /*PHASECAL_CONFIG__TIMEOUT_MACROP*/, &tDM);
  if (tDM == 0x14) *DM = 1;
  else if (tDM == 0x0A) *DM = 2;
  else *DM = 0;

  return status;
}

VL53L1X_Status_t VL53L1X_SetInterMeasurementInMs(uint16_t dev, uint32_t IM) {
  uint16_t clockPLL;
  VL53L1X_Status_t status;

  status = VL53L1X_RdWord(dev, 0x0006 /*VL53L1X_RESULT__OSC_CALIBRATE_VAL*/, &clockPLL);
  clockPLL = clockPLL&0x3FF;
  status |= VL53L1X_WrDWord(dev, 0x006C /*VL53L1X_SYSTEM__INTERMEASUREMENT_PERIOD*/, (uint32_t)(clockPLL * IM * 1.075));
  return status;
}

VL53L1X_Status_t VL53L1X_GetInterMeasurementInMs(uint16_t dev, uint16_t* pIM) {
  uint16_t clockPLL;
  VL53L1X_Status_t status;
  uint32_t tmp;

  status = VL53L1X_RdDWord(dev, 0x006C, &tmp);
  status |= VL53L1X_RdWord(dev, 0x0006, &clockPLL);
  clockPLL = clockPLL&0x3FF;
  *pIM = (uint16_t)(((uint16_t)tmp)/(clockPLL*1.065));
  return status;
}

VL53L1X_Status_t VL53L1X_BootState(uint16_t dev, uint8_t* state) {
  return VL53L1X_RdByte(dev, 0x00E5 /*VL53L1X_FIRMWARE__SYSTEM_STATUS*/, state);
}

VL53L1X_Status_t VL53L1X_GetSensorId(uint16_t dev, uint16_t* sensorId) {
  return VL53L1X_RdWord(dev, 0x010F /*VL53L1X_IDENTIFICATION__MODEL_ID*/, sensorId);
}

VL53L1X_Status_t VL53L1X_GetDistance(uint16_t dev, uint16_t* distance) {
  return VL53L1X_RdWord(dev, 0x0096 /*VL53L1X_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0*/, distance);
}

VL53L1X_Status_t VL53L1X_GetSignalPerSpad(uint16_t dev, uint16_t* signalRate) {
  VL53L1X_Status_t status;
  uint16_t spNb=1, signal;

  status = VL53L1X_RdWord(dev, 0x0098 /*VL53L1X_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0*/, &signal);
  status |= VL53L1X_RdWord(dev, 0x008C /*VL53L1X_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0*/, &spNb);
  *signalRate = (uint16_t)(200.0*signal/spNb);
  return status;
}

VL53L1X_Status_t VL53L1X_GetAmbientPerSpad(uint16_t dev, uint16_t* ambPerSp) {
  VL53L1X_Status_t status;
  uint16_t ambRate, spNb = 1;

  status = VL53L1X_RdWord(dev, 0x0090 /*RESULT__AMBIENT_COUNT_RATE_MCPS_SD*/, &ambRate);
  status |= VL53L1X_RdWord(dev, 0x008C /*VL53L1X_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0*/, &spNb);
  *ambPerSp = (uint16_t)(200.0 * ambRate / spNb);
  return status;
}

VL53L1X_Status_t VL53L1X_GetSignalRate(uint16_t dev, uint16_t* signal) {
  VL53L1X_Status_t status;
  uint16_t tmp;

  status = VL53L1X_RdWord(dev, 0x0098 /*VL53L1X_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0*/, &tmp);
  *signal = tmp*8;
  return status;
}

VL53L1X_Status_t VL53L1X_GetSpadNb(uint16_t dev, uint16_t* spNb) {
  VL53L1X_Status_t status;
  uint16_t tmp;

  status = VL53L1X_RdWord(dev, 0x008C /*VL53L1X_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0*/, &tmp);
  *spNb = tmp >> 8;
  return status;
}

VL53L1X_Status_t VL53L1X_GetAmbientRate(uint16_t dev, uint16_t* ambRate) {
  VL53L1X_Status_t status;
  uint16_t tmp;

  status = VL53L1X_RdWord(dev, 0x0090 /*RESULT__AMBIENT_COUNT_RATE_MCPS_SD*/, &tmp);
  *ambRate = tmp*8;
  return status;
}

VL53L1X_Status_t VL53L1X_GetRangeStatus(uint16_t dev, uint8_t* rangeStatus) {
  VL53L1X_Status_t status;
  uint8_t rgSt;

  *rangeStatus = 255;
  status = VL53L1X_RdByte(dev, 0x0089 /*VL53L1X_RESULT__RANGE_STATUS*/, &rgSt);
  rgSt = rgSt & 0x1F;
  if (rgSt < 24)
    *rangeStatus = status_rtn[rgSt];
  return status;
}

VL53L1X_Status_t VL53L1X_GetResult(uint16_t dev, VL53L1X_Result_t* result) {
  VL53L1X_Status_t status;
  uint8_t tmp[17];

  status = VL53L1X_ReadMulti(dev, 0x0089 /*VL53L1X_RESULT__RANGE_STATUS*/, tmp, 17);

  if (status != 0) return status;

  uint8_t RgSt = tmp[0] & 0x1F;
  if (RgSt < 24) RgSt = status_rtn[RgSt];

  result->status = RgSt;
  result->ambient = (tmp[7] << 8 | tmp[8]) * 8;
  result->numSPADs = tmp[3];
  result->sigPerSPAD = (tmp[15] << 8 | tmp[16]) * 8;
  result->distance = tmp[13] << 8 | tmp[14];

  return status;
}

VL53L1X_Status_t VL53L1X_SetOffset(uint16_t dev, int16_t offset) {
  VL53L1X_Status_t status;

  status = VL53L1X_WrWord(dev, 0x001E /*ALGO__PART_TO_PART_RANGE_OFFSET_MM*/, (uint16_t)(offset*4));
  status |= VL53L1X_WrWord(dev, 0x0020 /*MM_CONFIG__INNER_OFFSET_MM*/, 0x0);
  status |= VL53L1X_WrWord(dev, 0x0022 /*MM_CONFIG__OUTER_OFFSET_MM*/, 0x0);
  return status;
}

VL53L1X_Status_t  VL53L1X_GetOffset(uint16_t dev, int16_t* offset) {
  VL53L1X_Status_t status = 0;
  uint16_t tmp;

  status = VL53L1X_RdWord(dev,0x001E /*ALGO__PART_TO_PART_RANGE_OFFSET_MM*/, &tmp);
  *offset = (int16_t)((tmp << 3) >> 5);
  return status;
}

VL53L1X_Status_t VL53L1X_SetXtalk(uint16_t dev, uint16_t xtalk) {
  VL53L1X_Status_t status;

  status = VL53L1X_WrWord(dev, 0x0018 /*ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS*/, 0x0000);
  status |= VL53L1X_WrWord(dev, 0x001A /*ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS*/, 0x0000);
  status |= VL53L1X_WrWord(dev, 0x0016 /*ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS*/, (xtalk<<9)/1000);
  return status;
}

VL53L1X_Status_t VL53L1X_GetXtalk(uint16_t dev, uint16_t* xtalk ) {
  VL53L1X_Status_t status;

  status = VL53L1X_RdWord(dev, 0x0016 /*ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS*/, xtalk);
  *xtalk = (uint16_t)((*xtalk*1000)>>9);
  return status;
}

VL53L1X_Status_t VL53L1X_SetDistanceThreshold(uint16_t dev, uint16_t threshLow,
            uint16_t threshHigh, uint8_t window, uint8_t iOnNoTarget) {
  VL53L1X_Status_t status;
  uint8_t tmp;

  status = VL53L1X_RdByte(dev, 0x0033 /*SYSTEM__INTERRUPT_CONFIG_GPIO*/, &tmp);
  tmp = tmp & 0x47;
  if (iOnNoTarget == 0) {
    status |= VL53L1X_WrByte(dev, 0x0033, (tmp | (window & 0x07)));
  } else {
    status |= VL53L1X_WrByte(dev, 0x0033, ((tmp | (window & 0x07)) | 0x40));
  }
  status |= VL53L1X_WrWord(dev, 0x0072 /*SYSTEM__THRESH_HIGH*/, threshHigh);
  status |= VL53L1X_WrWord(dev, 0x0074 /*SYSTEM__THRESH_LOW*/, threshLow);
  return status;
}

VL53L1X_Status_t VL53L1X_GetDistanceThresholdWindow(uint16_t dev, uint16_t* window) {
  VL53L1X_Status_t status;
  uint8_t tmp;

  status = VL53L1X_RdByte(dev, 0x0033 /*SYSTEM__INTERRUPT_CONFIG_GPIO*/, &tmp);
  *window = (uint16_t)(tmp & 0x7);
  return status;
}

VL53L1X_Status_t VL53L1X_GetDistanceThresholdLow(uint16_t dev, uint16_t *low) {
  return VL53L1X_RdWord(dev, 0x0074 /*SYSTEM__THRESH_LOW*/, low);
}

VL53L1X_Status_t VL53L1X_GetDistanceThresholdHigh(uint16_t dev, uint16_t *high) {
  return VL53L1X_RdWord(dev, 0x0072 /*SYSTEM__THRESH_HIGH*/, high);
}

VL53L1X_Status_t VL53L1X_SetROICenter(uint16_t dev, uint8_t ROICenter) {
  return VL53L1X_WrByte(dev, 0x007F /*ROI_CONFIG__USER_ROI_CENTRE_SPAD*/, ROICenter);
}

VL53L1X_Status_t VL53L1X_GetROICenter(uint16_t dev, uint8_t *ROICenter) {
  return VL53L1X_RdByte(dev, 0x007F /*ROI_CONFIG__USER_ROI_CENTRE_SPAD*/, ROICenter);
}

VL53L1X_Status_t VL53L1X_SetROI(uint16_t dev, uint16_t x, uint16_t y) {
  VL53L1X_Status_t status;
  uint8_t opticalCenter;

  status = VL53L1X_RdByte(dev, 0x013F /*VL53L1X_ROI_CONFIG__MODE_ROI_CENTRE_SPAD*/, &opticalCenter);

  x = x > 16 ? 16 : x;
  y = y > 16 ? 16 : y;
  if (x > 10 || y > 10) opticalCenter = 199;

  status |= VL53L1X_WrByte(dev, 0x007F /*ROI_CONFIG__USER_ROI_CENTRE_SPAD*/, opticalCenter);
  status |= VL53L1X_WrByte(dev, 0x0080 /*ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE*/, (y - 1) << 4 | (x - 1));
  return status;
}

VL53L1X_Status_t VL53L1X_GetROI_XY(uint16_t dev, uint16_t *ROI_X, uint16_t *ROI_Y) {
  VL53L1X_Status_t status;
  uint8_t tmp;

  status = VL53L1X_RdByte(dev, 0x0080 /*ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE*/, &tmp);
  *ROI_X = ((uint16_t)tmp & 0x0F) + 1;
  *ROI_Y = (((uint16_t)tmp & 0xF0) >> 4) + 1;
  return status;
}

VL53L1X_Status_t VL53L1X_SetSignalThreshold(uint16_t dev, uint16_t signal) {
  return VL53L1X_WrWord(dev, 0x0066 /*RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS*/, signal >> 3);
}

VL53L1X_Status_t VL53L1X_GetSignalThreshold(uint16_t dev, uint16_t* signal) {
  VL53L1X_Status_t status;
  uint16_t tmp;

  status = VL53L1X_RdWord(dev, 0x0066 /*RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS*/, &tmp);
  *signal = tmp << 3;
  return status;
}

VL53L1X_Status_t VL53L1X_SetSigmaThreshold(uint16_t dev, uint16_t sigma) {
  if (sigma > 0x3FFF) return -1;
  return VL53L1X_WrWord(dev, 0x0064 /*RANGE_CONFIG__SIGMA_THRESH*/, sigma << 2);
}

VL53L1X_Status_t VL53L1X_GetSigmaThreshold(uint16_t dev, uint16_t* sigma) {
  VL53L1X_Status_t status;
  uint16_t tmp;

  status = VL53L1X_RdWord(dev, 0x0064 /*RANGE_CONFIG__SIGMA_THRESH*/, &tmp);
  *sigma = tmp >> 2;
  return status;
}

VL53L1X_Status_t VL53L1X_StartTemperatureUpdate(uint16_t dev) {
  VL53L1X_Status_t status;

  status = VL53L1X_WrByte(dev, 0x0008 /*VL53L1X_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND*/, 0x81);
  status |= VL53L1X_WrByte(dev, 0x0B, 0x92);
  status |= VL53L1X_StartRanging(dev);

  uint8_t tmp;
  do {
    status = VL53L1X_CheckForDataReady(dev, &tmp);
  } while(tmp == 0);

  status |= VL53L1X_ClearInterrupt(dev);
  status |= VL53L1X_StopRanging(dev);
  status |= VL53L1X_WrByte(dev, 0x0008 /*VL53L1X_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND*/, 0x09);
  status |= VL53L1X_WrByte(dev, 0x0B, 0); 
  return status;
}

/* =========================================================================
   IMPLEMENTAÇÃO DE CALIBRAÇÃO (baseado em VL53L1X_calibration.c)
   ========================================================================= */

VL53L1X_Status_t VL53L1X_CalibrateOffset(uint16_t dev, uint16_t targetDistInMm, int16_t *offset) {
  int16_t avgDist = 0;
  uint16_t distance;
  VL53L1X_Status_t status;

  status = VL53L1X_WrWord(dev, 0x001E /*ALGO__PART_TO_PART_RANGE_OFFSET_MM*/, 0x0);
  status |= VL53L1X_WrWord(dev, 0x0020 /*MM_CONFIG__INNER_OFFSET_MM*/, 0x0);
  status |= VL53L1X_WrWord(dev, 0x0022 /*MM_CONFIG__OUTER_OFFSET_MM*/, 0x0);
  status |= VL53L1X_StartRanging(dev);  
  for (uint8_t i=0; i<NUM_CALIBRATION_SAMPLES; i++) {
    uint8_t tmp;
    do {
      status += VL53L1X_CheckForDataReady(dev, &tmp);
    } while (tmp == 0);
    status |= VL53L1X_GetDistance(dev, &distance);
    status |= VL53L1X_ClearInterrupt(dev);
    avgDist = avgDist + distance;
  }
  status |= VL53L1X_StopRanging(dev);
  avgDist = avgDist / NUM_CALIBRATION_SAMPLES;
  *offset = targetDistInMm - avgDist;
  status |= VL53L1X_WrWord(dev, 0x001E /*ALGO__PART_TO_PART_RANGE_OFFSET_MM*/, *offset*4);
  return status;
}

VL53L1X_Status_t VL53L1X_CalibrateXtalk(uint16_t dev, uint16_t targetDistInMm, uint16_t *xtalk) {
  float avgSigRate = 0;
  float avgDist = 0;
  float avgSpadNb = 0;
  uint16_t distance = 0, sr, spadNum;
  uint32_t calXtalk;
  VL53L1X_Status_t status = 0;

  status |= VL53L1X_WrWord(dev, 0x0016,0);
  status |= VL53L1X_StartRanging(dev);
  for (uint8_t i = 0; i < 50; i++) {
    uint8_t tmp;
    do {
      status |= VL53L1X_CheckForDataReady(dev, &tmp);
    } while (tmp == 0);
    status |= VL53L1X_GetSignalRate(dev, &sr);
    status |= VL53L1X_GetDistance(dev, &distance);
    status |= VL53L1X_ClearInterrupt(dev);
    status |= VL53L1X_GetSpadNb(dev, &spadNum);
    avgDist = avgDist + distance;
    avgSpadNb = avgSpadNb + spadNum;
    avgSigRate = avgSigRate + sr;
  }
  status |= VL53L1X_StopRanging(dev);
  avgDist = avgDist / NUM_CALIBRATION_SAMPLES;
  avgSpadNb = avgSpadNb / NUM_CALIBRATION_SAMPLES;
  avgSigRate = avgSigRate / NUM_CALIBRATION_SAMPLES;
  calXtalk = (uint16_t)(512*(avgSigRate*(1-(avgDist/targetDistInMm)))/avgSpadNb);
  if (calXtalk > 0xffff) calXtalk = 0xffff;
  *xtalk = (uint16_t)((calXtalk*1000)>>9);
  status |= VL53L1X_WrWord(dev, 0x0016, (uint16_t)calXtalk);
  return status;
}