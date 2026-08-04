#ifndef _VL53L1X_H_
#define _VL53L1X_H_

#include <stdint.h>
#include <string.h>
#include "hardware/i2c.h"

// --- Constants ---
#define VL53L1X_I2C_BUF_SIZE 256
#define NUM_CALIBRATION_SAMPLES 50
#define VL53L1X_SENSOR_ID 0xEACC

// --- Types (from VL53L1X_types.h) ---
typedef int8_t VL53L1X_Status_t;

typedef struct {
  uint8_t status;		// Status of measurement
  uint16_t distance;	// Distance in mm
  uint16_t ambient;	// Ambient
  uint16_t sigPerSPAD;// Signal per SPAD
  uint16_t numSPADs;	// Number SPADs
} VL53L1X_Result_t;

typedef struct {
  uint8_t      major;    /*!< major number */
  uint8_t      minor;    /*!< minor number */
  uint8_t      build;    /*!< build number */
  uint32_t     revision; /*!< revision number */
} VL53L1X_Version_t;

// --- Platform API ---
VL53L1X_Status_t VL53L1X_I2C_Init(uint16_t addr, i2c_inst_t* i2c_device);
VL53L1X_Status_t VL53L1X_WriteMulti(uint16_t addr, uint16_t reg, uint8_t *data, uint32_t count);
VL53L1X_Status_t VL53L1X_ReadMulti(uint16_t addr, uint16_t reg, uint8_t *data, uint32_t count);
VL53L1X_Status_t VL53L1X_RdWord(uint16_t addr, uint16_t index, uint16_t* data);
VL53L1X_Status_t VL53L1X_RdDWord(uint16_t addr, uint16_t index, uint32_t* data);
VL53L1X_Status_t VL53L1X_RdByte(uint16_t addr, uint16_t index, uint8_t* data);
VL53L1X_Status_t VL53L1X_WrByte(uint16_t addr, uint16_t index, uint8_t data);
VL53L1X_Status_t VL53L1X_WrWord(uint16_t addr, uint16_t index, uint16_t data);
VL53L1X_Status_t VL53L1X_WrDWord(uint16_t addr, uint16_t index, uint32_t data);
VL53L1X_Status_t VL53L1X_WaitMs(uint16_t addr, int32_t wait_ms);

// --- Core API ---
VL53L1X_Status_t VL53L1X_GetSWVersion(VL53L1X_Version_t *pVersion);
VL53L1X_Status_t VL53L1X_SetI2CAddress(uint16_t dev, uint8_t new_address);
VL53L1X_Status_t VL53L1X_SensorInit(uint16_t dev);
VL53L1X_Status_t VL53L1X_ClearInterrupt(uint16_t dev);
VL53L1X_Status_t VL53L1X_SetInterruptPolarity(uint16_t dev, uint8_t nPol);
VL53L1X_Status_t VL53L1X_GetInterruptPolarity(uint16_t dev, uint8_t* iPol);
VL53L1X_Status_t VL53L1X_StartRanging(uint16_t dev);
VL53L1X_Status_t VL53L1X_StopRanging(uint16_t dev);
VL53L1X_Status_t VL53L1X_CheckForDataReady(uint16_t dev, uint8_t* isDataReady);
VL53L1X_Status_t VL53L1X_SetTimingBudgetInMs(uint16_t dev, uint16_t timingBudgetMs);
VL53L1X_Status_t VL53L1X_GetTimingBudgetInMs(uint16_t dev, uint16_t* timingBudget);
VL53L1X_Status_t VL53L1X_SetDistanceMode(uint16_t dev, uint16_t DM);
VL53L1X_Status_t VL53L1X_GetDistanceMode(uint16_t dev, uint16_t* DM);
VL53L1X_Status_t VL53L1X_SetInterMeasurementInMs(uint16_t dev, uint32_t IM);
VL53L1X_Status_t VL53L1X_GetInterMeasurementInMs(uint16_t dev, uint16_t* pIM);
VL53L1X_Status_t VL53L1X_BootState(uint16_t dev, uint8_t* state);
VL53L1X_Status_t VL53L1X_GetSensorId(uint16_t dev, uint16_t* sensorId);
VL53L1X_Status_t VL53L1X_GetDistance(uint16_t dev, uint16_t* distance);
VL53L1X_Status_t VL53L1X_GetSignalPerSpad(uint16_t dev, uint16_t* signalRate);
VL53L1X_Status_t VL53L1X_GetAmbientPerSpad(uint16_t dev, uint16_t* ambPerSp);
VL53L1X_Status_t VL53L1X_GetSignalRate(uint16_t dev, uint16_t* signal);
VL53L1X_Status_t VL53L1X_GetSpadNb(uint16_t dev, uint16_t* spNb);
VL53L1X_Status_t VL53L1X_GetAmbientRate(uint16_t dev, uint16_t* ambRate);
VL53L1X_Status_t VL53L1X_GetRangeStatus(uint16_t dev, uint8_t* rangeStatus);
VL53L1X_Status_t VL53L1X_GetResult(uint16_t dev, VL53L1X_Result_t* result);
VL53L1X_Status_t VL53L1X_SetOffset(uint16_t dev, int16_t offset);
VL53L1X_Status_t VL53L1X_GetOffset(uint16_t dev, int16_t* offset);
VL53L1X_Status_t VL53L1X_SetXtalk(uint16_t dev, uint16_t xtalk);
VL53L1X_Status_t VL53L1X_GetXtalk(uint16_t dev, uint16_t* xtalk);
VL53L1X_Status_t VL53L1X_SetDistanceThreshold(uint16_t dev, uint16_t threshLow, uint16_t threshHigh, uint8_t window, uint8_t iOnNoTarget);
VL53L1X_Status_t VL53L1X_GetDistanceThresholdWindow(uint16_t dev, uint16_t* window);
VL53L1X_Status_t VL53L1X_GetDistanceThresholdLow(uint16_t dev, uint16_t *low);
VL53L1X_Status_t VL53L1X_GetDistanceThresholdHigh(uint16_t dev, uint16_t *high);
VL53L1X_Status_t VL53L1X_SetROICenter(uint16_t dev, uint8_t ROICenter);
VL53L1X_Status_t VL53L1X_GetROICenter(uint16_t dev, uint8_t *ROICenter);
VL53L1X_Status_t VL53L1X_SetROI(uint16_t dev, uint16_t x, uint16_t y);
VL53L1X_Status_t VL53L1X_GetROI_XY(uint16_t dev, uint16_t *ROI_X, uint16_t *ROI_Y);
VL53L1X_Status_t VL53L1X_SetSignalThreshold(uint16_t dev, uint16_t signal);
VL53L1X_Status_t VL53L1X_GetSignalThreshold(uint16_t dev, uint16_t* signal);
VL53L1X_Status_t VL53L1X_SetSigmaThreshold(uint16_t dev, uint16_t sigma);
VL53L1X_Status_t VL53L1X_GetSigmaThreshold(uint16_t dev, uint16_t* sigma);
VL53L1X_Status_t VL53L1X_StartTemperatureUpdate(uint16_t dev);

// --- Calibration API ---
VL53L1X_Status_t VL53L1X_CalibrateOffset(uint16_t dev, uint16_t targetDistInMm, int16_t *offset);
VL53L1X_Status_t VL53L1X_CalibrateXtalk(uint16_t dev, uint16_t targetDistInMm, uint16_t *xtalk);

#endif  // _VL53L1X_H_