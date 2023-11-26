#include "ov7725.h"
#include "ov7725_regs.h"

CAMERA_DrvTypeDef   ov7725_drv =
{
  ov7725_Init,
  ov7725_ReadID,
  ov7725_Config,
};

/* Minimal working configuration for RGB565 VGA resolution (640x480) 20 FPS*/
static const uint8_t OV7725_VGA[][2] = {
  {COM7, COM7_RES_VGA | COM7_FMT_RGB | COM7_FMT_RGB565},
  {COM4, COM4_PLL_4x},
  {CLKRC, 0x02}, // 0x02 - for 20 FPS
  {COM3, 0x00},

  // Extended configuration
  {DSP_CTRL1, 0x3F}, // Enable SDE
  {SDE, SDE_CONT_BRIGHT_EN | SDE_SATURATION_EN},

  {0x13, 0xff},
  {0x0e, 0xe5}
};


/* Minimal working configuration for RGB565 QVGA resolution (320x240) 20 FPS*/
static const uint8_t OV7725_QVGA[][2] = {
  {COM7, COM7_RES_QVGA | COM7_FMT_RGB | COM7_FMT_RGB565},
  {COM4, COM4_PLL_4x},
  {CLKRC, 0x04}, // 0x02 - for 20 FPS, 0x04 - for 30 FPS
  {COM3, COM3_VFLIP|COM3_MIRROR},
  {COM9, 0x4E},

  {HSTART,    0x3f},
  {HSIZE,     0x50},
  {VSTART,    0x03},
  {VSIZE,     0x78},
  {HREF,      0x00},
  {HOUTSIZE,  0x50},
  {VOUTSIZE,  0x78},
  {EXHCH,     0x00},
};


/**
 * @brief  Initializes the OV7725 CAMERA component.
 * @param  DeviceAddr: Device address on communication Bus.
 * @param  resolution: Camera resolution
 * @retval None
 */
void ov7725_Init(uint16_t DeviceAddr, uint32_t resolution)
{
  uint32_t index;

  /* Initialize I2C */
  CAMERA_IO_Init();

  /* Prepare the camera to be configured by resetting all its registers */
  CAMERA_IO_Write(DeviceAddr, COM7, COM7_RESET);
  CAMERA_Delay(200);

  /* Initialize OV7725 */
  switch (resolution) {
  case CAMERA_R160x120:
  {
    /* Not supported resolution */
    break;
  }
  case CAMERA_R320x240:
  {

    for (index = 0; index < (sizeof(OV7725_QVGA) / 2); index++) {
      CAMERA_IO_Write(DeviceAddr, OV7725_QVGA[index][0], OV7725_QVGA[index][1]);
      CAMERA_Delay(2);
    }
    break;
  }
  case CAMERA_R480x272:
  {
    /* Not supported resolution */
    break;
  }
  case CAMERA_R640x480:
  {
    for (index = 0; index < (sizeof(OV7725_VGA) / 2); index++) {
      CAMERA_IO_Write(DeviceAddr, OV7725_VGA[index][0], OV7725_VGA[index][1]);
      CAMERA_Delay(2);
    }
    break;
  }
  default:
  {
    break;
  }
  }
}

/**
 * @brief  Configures the OV7725 camera feature.
 * @param  DeviceAddr: Device address on communication Bus.
 * @param  feature: Camera feature to be configured
 * @param  value: Value to be configured
 * @param  brightness_value: Brightness value to be configured
 * @retval None
 */
void ov7725_Config(uint16_t DeviceAddr, uint32_t feature, uint32_t value,
    uint32_t brightness_value)
{
  switch(feature)
  {
  case CAMERA_CONTRAST_BRIGHTNESS:
  {
    if (brightness_value == CAMERA_BRIGHTNESS_LEVEL4) {
      CAMERA_IO_Write(DeviceAddr, 0xAB, 0x06);
      CAMERA_IO_Write(DeviceAddr, 0x9B, 0x28);
    }
    else if (brightness_value == CAMERA_BRIGHTNESS_LEVEL3) {
      CAMERA_IO_Write(DeviceAddr, 0xAB, 0x06);
      CAMERA_IO_Write(DeviceAddr, 0x9B, 0x18);
    }
    else if (brightness_value == CAMERA_BRIGHTNESS_LEVEL2) {
      CAMERA_IO_Write(DeviceAddr, 0xAB, 0x06);
      CAMERA_IO_Write(DeviceAddr, 0x9B, 0x08);
    }
    else if (brightness_value == CAMERA_BRIGHTNESS_LEVEL1) {
      CAMERA_IO_Write(DeviceAddr, 0xAB, 0x0e);
      CAMERA_IO_Write(DeviceAddr, 0x9B, 0x08);
    }
    else if (brightness_value == CAMERA_BRIGHTNESS_LEVEL0) {
      CAMERA_IO_Write(DeviceAddr, 0xAB, 0x0e);
      CAMERA_IO_Write(DeviceAddr, 0x9B, 0x18);
    }
  }
  case CAMERA_BLACK_WHITE:
  {
    uint8_t reg_value = CAMERA_IO_Read(DeviceAddr, SDE);
    if (value == CAMERA_BLACK_WHITE_BW) {
      reg_value |= SDE_GRAYSCALE_EN;
      reg_value &= ~SDE_NEGATIVE_EN;
      CAMERA_IO_Write(DeviceAddr, SDE, reg_value);
    }
    else if (value == CAMERA_BLACK_WHITE_NEGATIVE) {
      reg_value &= ~SDE_GRAYSCALE_EN;
      reg_value |= SDE_NEGATIVE_EN;
      CAMERA_IO_Write(DeviceAddr, SDE, reg_value);
    }
    else if (value == CAMERA_BLACK_WHITE_BW_NEGATIVE) {
      reg_value |= SDE_GRAYSCALE_EN;
      reg_value |= SDE_NEGATIVE_EN;
      CAMERA_IO_Write(DeviceAddr, SDE, reg_value);
    }
    else if (value == CAMERA_BLACK_WHITE_NORMAL) {
      reg_value &= ~SDE_GRAYSCALE_EN;
      reg_value &= ~SDE_NEGATIVE_EN;
      CAMERA_IO_Write(DeviceAddr, SDE, reg_value);
    }
    reg_value = CAMERA_IO_Read(DeviceAddr, DSP_CTRL1);
    reg_value |= DSP_CTRL1_SDE_EN;
    CAMERA_IO_Write(DeviceAddr, DSP_CTRL1, reg_value);
    break;
  }
  case CAMERA_COLOR_EFFECT:
  {
    if (value == CAMERA_COLOR_EFFECT_NONE) {
      uint8_t reg_value = CAMERA_IO_Read(DeviceAddr, DSP_CTRL1);
      reg_value &= ~DSP_CTRL1_SDE_EN;
      CAMERA_IO_Write(DeviceAddr, DSP_CTRL1, reg_value);
      CAMERA_IO_Write(DeviceAddr, SDE, 0x00);
    }
    else if (value == CAMERA_COLOR_EFFECT_ANTIQUE) {
      uint8_t value = CAMERA_IO_Read(DeviceAddr, DSP_CTRL1);
      value |= DSP_CTRL1_SDE_EN;
      CAMERA_IO_Write(DeviceAddr, DSP_CTRL1, value);
      CAMERA_IO_Write(DeviceAddr, SDE, SDE_V_FIXED_EN | SDE_U_FIXED_EN);
      CAMERA_IO_Write(DeviceAddr, UFIX, 0x40);
      CAMERA_IO_Write(DeviceAddr, VFIX, 0xA0);
    }
    else if (value == CAMERA_COLOR_EFFECT_BLUE) {
      uint8_t value = CAMERA_IO_Read(DeviceAddr, DSP_CTRL1);
      value |= DSP_CTRL1_SDE_EN;
      CAMERA_IO_Write(DeviceAddr, DSP_CTRL1, value);
      CAMERA_IO_Write(DeviceAddr, SDE, SDE_V_FIXED_EN | SDE_U_FIXED_EN);
      CAMERA_IO_Write(DeviceAddr, UFIX, 0xA0);
      CAMERA_IO_Write(DeviceAddr, VFIX, 0x40);
    }
    else if (value == CAMERA_COLOR_EFFECT_GREEN) {
      uint8_t value = CAMERA_IO_Read(DeviceAddr, DSP_CTRL1);
      value |= DSP_CTRL1_SDE_EN;
      CAMERA_IO_Write(DeviceAddr, DSP_CTRL1, value);
      CAMERA_IO_Write(DeviceAddr, SDE, SDE_V_FIXED_EN | SDE_U_FIXED_EN);
      CAMERA_IO_Write(DeviceAddr, UFIX, 0x60);
      CAMERA_IO_Write(DeviceAddr, VFIX, 0x60);
    }
    else if (value == CAMERA_COLOR_EFFECT_RED) {
      uint8_t value = CAMERA_IO_Read(DeviceAddr, DSP_CTRL1);
      value |= DSP_CTRL1_SDE_EN;
      CAMERA_IO_Write(DeviceAddr, DSP_CTRL1, value);
      CAMERA_IO_Write(DeviceAddr, SDE, SDE_V_FIXED_EN | SDE_U_FIXED_EN);
      CAMERA_IO_Write(DeviceAddr, UFIX, 0x80);
      CAMERA_IO_Write(DeviceAddr, VFIX, 0xC0);
    }

      break;
    }
  default:
    {
      break;
    }
  }
}

/**
 * @brief  Read the OV7725 Camera identity.
 * @param  DeviceAddr: Device address on communication Bus.
 * @retval the OV7725 ID
 */
uint16_t ov7725_ReadID(uint16_t DeviceAddr)
{
  /* Initialize I2C */
  CAMERA_IO_Init();

  /* Get the camera ID */
  return CAMERA_IO_Read(DeviceAddr, REG_PID);
}



