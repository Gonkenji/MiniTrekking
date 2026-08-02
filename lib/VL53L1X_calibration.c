#include <stdint.h>
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
#include "VL53L1X_types.h"

VL53L1X_Status_t VL53L1X_CalibrateOffset(uint16_t dev, uint16_t targetDistInMm, int16_t *offset) {
  int16_t avgDist = 0;
  uint16_t distance;
  VL53L1X_Status_t status;

  status = VL53L1X_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0);
  status |= VL53L1X_WrWord(dev, MM_CONFIG__INNER_OFFSET_MM, 0x0);
  status |= VL53L1X_WrWord(dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
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
  status |= VL53L1X_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, *offset*4);
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
  if (calXtalk > 0xffff)
    calXtalk = 0xffff;
  *xtalk = (uint16_t)((calXtalk*1000)>>9);
  status |= VL53L1X_WrWord(dev, 0x0016, (uint16_t)calXtalk);
  return status;
}