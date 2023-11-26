#ifndef __OV7725_H__
#define __OV7725_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../Common/camera.h"


#define OV7725_I2C_ADDR_READ    0x42
#define OV7725_I2C_ADDR_WRITE   0x43

#define OV7725_ID               0x77


void     ov7725_Init(uint16_t DeviceAddr, uint32_t resolution);
void     ov7725_Config(uint16_t DeviceAddr, uint32_t feature, uint32_t value, uint32_t BR_value);
uint16_t ov7725_ReadID(uint16_t DeviceAddr);

void     CAMERA_IO_Init(void);
void     CAMERA_IO_Write(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t  CAMERA_IO_Read(uint8_t addr, uint8_t reg);
void     CAMERA_Delay(uint32_t delay);

/* CAMERA driver structure */
extern CAMERA_DrvTypeDef   ov7725_drv;
/**
  * @}
  */
#ifdef __cplusplus
}
#endif


#endif
