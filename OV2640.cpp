/**
  ******************************************************************************
  * @file
  * @author
  * @version
  * @date
  * @brief   OV2640
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "OV2640.h"
#include "dma.h"
#include "dcmi.h"
#include "TFT_eSPI.h"
extern TFT_eSPI tft;
/** @addtogroup DCMI_Camera
  * @{
  */
static OV2640_TypeDef *_OV2640;
extern TwoWire Wire2;

const unsigned char OV2640_UXGA[][2] =
{
        0xff, 0x00,
        0x2c, 0xff,
        0x2e, 0xdf,
        0xff, 0x01,
        0x3c, 0x32,
        0x11, 0x00,
        0x09, 0x02,
        0x04, 0xFF, /*0x80,*/
        0x13, 0xe5,
        0x14, 0x48,
        0x2c, 0x0c,
        0x33, 0x78,
        0x3a, 0x33,
        0x3b, 0xfB,
        0x3e, 0x00,
        0x43, 0x11,
        0x16, 0x10,
        0x4a, 0x81,
        0x21, 0x99,
        0x24, 0x40,
        0x25, 0x38,
        0x26, 0x82,
        0x5c, 0x00,
        0x63, 0x00,
        0x46, 0x3f,
        0x0c, 0x38,/*0x3c*/
        0x61, 0x70,
        0x62, 0x80,
        0x7c, 0x05,
        0x20, 0x80,
        0x28, 0x30,
        0x6c, 0x00,
        0x6d, 0x80,
        0x6e, 0x00,
        0x70, 0x02,
        0x71, 0x94,
        0x73, 0xc1,
        0x3d, 0x34,
        0x5a, 0x57,
        0x12, 0x00,
        0x11, 0x00,
        0x17, 0x11,
        0x18, 0x75,
        0x19, 0x01,
        0x1a, 0x97,
        0x32, 0x36,
        0x03, 0x0f,
        0x37, 0x40,
        0x4f, 0xbb,
        0x50, 0x9c,
        0x5a, 0x57,
        0x6d, 0x80,
        0x6d, 0x38,
        0x39, 0x02,
        0x35, 0x88,
        0x22, 0x0a,
        0x37, 0x40,
        0x23, 0x00,
        0x34, 0xa0,
        0x36, 0x1a,
        0x06, 0x02,
        0x07, 0xc0,
        0x0d, 0xb7,
        0x0e, 0x01,
        0x4c, 0x00,
        0xff, 0x00,
        0xe5, 0x7f,
        0xf9, 0xc0,
        0x41, 0x24,
        0xe0, 0x14,
        0x76, 0xff,
        0x33, 0xa0,
        0x42, 0x20,
        0x43, 0x18,
        0x4c, 0x00,
        0x87, 0xd0,
        0x88, 0x3f,
        0xd7, 0x03,
        0xd9, 0x10,
        0xd3, 0x80,  /*0x82*/
        0xc8, 0x08,
        0xc9, 0x80,
        0x7d, 0x00,
        0x7c, 0x03,
        0x7d, 0x48,
        0x7c, 0x08,
        0x7d, 0x20,
        0x7d, 0x10,
        0x7d, 0x0e,
        0x90, 0x00,
        0x91, 0x0e,
        0x91, 0x1a,
        0x91, 0x31,
        0x91, 0x5a,
        0x91, 0x69,
        0x91, 0x75,
        0x91, 0x7e,
        0x91, 0x88,
        0x91, 0x8f,
        0x91, 0x96,
        0x91, 0xa3,
        0x91, 0xaf,
        0x91, 0xc4,
        0x91, 0xd7,
        0x91, 0xe8,
        0x91, 0x20,
        0x92, 0x00,
        0x93, 0x06,
        0x93, 0xe3,
        0x93, 0x02,
        0x93, 0x02,
        0x93, 0x00,
        0x93, 0x04,
        0x93, 0x00,
        0x93, 0x03,
        0x93, 0x00,
        0x93, 0x00,
        0x93, 0x00,
        0x93, 0x00,
        0x93, 0x00,
        0x93, 0x00,
        0x93, 0x00,
        0x96, 0x00,
        0x97, 0x08,
        0x97, 0x19,
        0x97, 0x02,
        0x97, 0x0c,
        0x97, 0x24,
        0x97, 0x30,
        0x97, 0x28,
        0x97, 0x26,
        0x97, 0x02,
        0x97, 0x98,
        0x97, 0x80,
        0x97, 0x00,
        0x97, 0x00,
        0xc3, 0xef,
        0xff, 0x00,
        0xba, 0xdc,
        0xbb, 0x08,
        0xb6, 0x24,
        0xb8, 0x33,
        0xb7, 0x20,
        0xb9, 0x30,
        0xb3, 0xb4,
        0xb4, 0xca,
        0xb5, 0x43,
        0xb0, 0x5c,
        0xb1, 0x4f,
        0xb2, 0x06,
        0xc7, 0x00,
        0xc6, 0x51,
        0xc5, 0x11,
        0xc4, 0x9c,
        0xbf, 0x00,
        0xbc, 0x64,
        0xa6, 0x00,
        0xa7, 0x1e,
        0xa7, 0x6b,
        0xa7, 0x47,
        0xa7, 0x33,
        0xa7, 0x00,
        0xa7, 0x23,
        0xa7, 0x2e,
        0xa7, 0x85,
        0xa7, 0x42,
        0xa7, 0x33,
        0xa7, 0x00,
        0xa7, 0x23,
        0xa7, 0x1b,
        0xa7, 0x74,
        0xa7, 0x42,
        0xa7, 0x33,
        0xa7, 0x00,
        0xa7, 0x23,
        0xc0, 0xc8,
        0xc1, 0x96,
        0x8c, 0x00,
        0x86, 0x3d,
        0x50, 0x92,
        0x51, 0x90,
        0x52, 0x2c,
        0x53, 0x00,
        0x54, 0x00,
        0x55, 0x88,
        0x5a, 0x50,
        0x5b, 0x3c,
        0x5c, 0x00,
        0xd3, 0x04, /*04*/
        0x7f, 0x00,
        0xda, 0x00,
        0xe5, 0x1f,
        0xe1, 0x67,
        0xe0, 0x00,
        0xdd, 0x7f,
        0x05, 0x00,
        0xff, 0x00,
        0xe0, 0x04,
        0xc0, 0xc8,
        0xc1, 0x96,
        0x86, 0x3d,
        0x50, 0x92,
        0x51, 0x90,
        0x52, 0x2c,
        0x53, 0x00,
        0x54, 0x00,
        0x55, 0x88,
        0x57, 0x00,
        0x5a, 0x50,
        0x5b, 0x3c,
        0x5c, 0x00,
        0xd3, 0x04,
        0xe0, 0x00,
        0xFF, 0x00,
        0x05, 0x00,
        0xDA, 0x08,
        0xda, 0x09,
        0x98, 0x00,
        0x99, 0x00,
        0x00, 0x00,

        0xff, 0x01,
        0x11, 0x00,
};


void OV2640_Init(OV2640_TypeDef *OV2640)
{
  analogWriteFrequency(25000000);
  analogWrite(DCMI_PWM, 127);
  
  MX_DCMI_Init();
  MX_DMA_Init();

  delay(500);
  
  _OV2640 = OV2640;
  pinMode(PH12, OUTPUT);
  pinMode(PE7, OUTPUT);
  digitalWrite(PH12, LOW);
  delay(100);
  digitalWrite(PH12, HIGH);
  delay(100);
  digitalWrite(PE7, LOW);
  delay(100);

}

/**
  * @brief  Resets the OV2640 camera.
  * @param  None
  * @retval None
  */
void OV2640_Reset(void)
{
  /*OV2640*/
  OV2640_WriteReg(OV2640_DSP_RA_DLMT, 0x01);
  OV2640_WriteReg(OV2640_SENSOR_COM7, 0x80);
}

/**
  * @brief
  * @param  OV2640ID:
  * @retval None
  */
void OV2640_ReadID(OV2640_IDTypeDef *OV2640ID)
{

  OV2640_WriteReg(OV2640_DSP_RA_DLMT, 0x01);

  OV2640ID->Manufacturer_ID1 = OV2640_ReadReg(OV2640_SENSOR_MIDH);
  OV2640ID->Manufacturer_ID2 = OV2640_ReadReg(OV2640_SENSOR_MIDL);
  OV2640ID->PIDH = OV2640_ReadReg(OV2640_SENSOR_PIDH);
  OV2640ID->PIDL = OV2640_ReadReg(OV2640_SENSOR_PIDL);
}

/**
  * @brief
  * @param
  * @retval 0
  */
u8 OV2640_OutSize_Set(u16 width, u16 height)
{
  u16 outh;
  u16 outw;
  u8 temp;
  if (width % 4)
    return 1;
  if (height % 4)
    return 2;
  outw = width / 4;
  outh = height / 4;
  OV2640_WriteReg(0XFF, 0X00);
  OV2640_WriteReg(0XE0, 0X04);
  OV2640_WriteReg(0X50, outw & 0X00);
  OV2640_WriteReg(0X5A, outw & 0XFF);
  OV2640_WriteReg(0X5B, outh & 0XFF);
  temp = (outw >> 8) & 0X03;
  temp |= (outh >> 6) & 0X04;
  OV2640_WriteReg(0X5C, temp);
  OV2640_WriteReg(0XE0, 0X00);
  return 0;
}

/**
  * @brief
  * @param  None
  * @retval None
  */
void OV2640_UXGAConfig(void)
{
  uint32_t i;

  OV2640_Reset();

  for (i = 0; i < (sizeof(OV2640_UXGA) / 2); i++)
  {
    OV2640_WriteReg(OV2640_UXGA[i][0], OV2640_UXGA[i][1]);
  }

  OV2640_OutSize_Set(_OV2640->frame->width, _OV2640->frame->height);
}

/**
  * @brief
  * @param
  *         0x00 Auto
  *         0x01 Sunny
  *         0x02 Cloudy
  *         0x03 Office
  *         0x04 Home

  * @retval None
  */
void OV2640_LightMode(uint8_t mode)
{
  switch (mode)
  {

  case 0: //Auto
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0xc7, 0x00); //AWB on
    break;

  case 1: //Sunny
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0xc7, 0x40); //AWB off
    OV2640_WriteReg(0xcc, 0x5e);
    OV2640_WriteReg(0xcd, 0x41);
    OV2640_WriteReg(0xce, 0x54);

    break;

  case 2: //Cloudy
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0xc7, 0x40); //AWB off
    OV2640_WriteReg(0xcc, 0x65);
    OV2640_WriteReg(0xcd, 0x41);
    OV2640_WriteReg(0xce, 0x4f);
    break;

  case 3: //Office
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0xc7, 0x40); //AWB off
    OV2640_WriteReg(0xcc, 0x52);
    OV2640_WriteReg(0xcd, 0x41);
    OV2640_WriteReg(0xce, 0x66);
    break;

  case 4: //Home
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0xc7, 0x40); //AWB off
    OV2640_WriteReg(0xcc, 0x42);
    OV2640_WriteReg(0xcd, 0x3f);
    OV2640_WriteReg(0xce, 0x71);
    break;
  }
}

/**
  * @brief
  * @param
  *         0x00 Antique
  *         0x01 Bluish
  *         0x02 Greenish
  *         0x03 Reddish
  *         0x04 B&W
  *         0x05 Negative
  *         0x06 B&W negative
  *         0x07 Normal

  * @retval None
  */
void OV2640_SpecialEffects(uint8_t mode)
{
  switch (mode)
  {
  case 0:
    // Antique
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x18);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0x40);
    OV2640_WriteReg(0x7d, 0xa6);
    break;

  case 1:
    //Bluish
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x18);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0xa0);
    OV2640_WriteReg(0x7d, 0x40);

    break;

  case 2:
    //Greenish
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x18);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0x40);
    OV2640_WriteReg(0x7d, 0x40);
    break;

  case 3:
    // Reddish
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x18);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0x40);
    OV2640_WriteReg(0x7d, 0xc0);
    break;

  case 4:
    // B&W
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x18);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0x80);
    OV2640_WriteReg(0x7d, 0x80);
    break;

  case 5:
    //Negative
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x40);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0x80);
    OV2640_WriteReg(0x7d, 0x80);

    break;

  case 6:
    //B&W negative
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x58);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0x80);
    OV2640_WriteReg(0x7d, 0x80);

    break;

  case 7:
    //Normal
    OV2640_WriteReg(0xff, 0x00);
    OV2640_WriteReg(0x7c, 0x00);
    OV2640_WriteReg(0x7d, 0x00);
    OV2640_WriteReg(0x7c, 0x05);
    OV2640_WriteReg(0x7d, 0x80);
    OV2640_WriteReg(0x7d, 0x80);

    break;
  }
}

/**
  * @brief  Configures the OV2640 camera brightness.
  * @param  Brightness: Brightness value, where Brightness can be:
  *         0x40 for Brightness +2,
  *         0x30 for Brightness +1,
  *         0x20 for Brightness 0,
  *         0x10 for Brightness -1,
  *         0x00 for Brightness -2,
  * @retval None
  */
void OV2640_BrightnessConfig(uint8_t Brightness)
{
  OV2640_WriteReg(0xff, 0x00);
  OV2640_WriteReg(0x7c, 0x00);
  OV2640_WriteReg(0x7d, 0x04);
  OV2640_WriteReg(0x7c, 0x09);
  OV2640_WriteReg(0x7d, Brightness);
  OV2640_WriteReg(0x7d, 0x00);
}

/**
  * @brief  Configures the OV2640 camera Black and white mode.
  * @param  BlackWhite: BlackWhite value, where BlackWhite can be:
  *         0x18 for B&W,
  *         0x40 for Negative,
  *         0x58 for B&W negative,
  *         0x00 for Normal,
  * @retval None
  */
void OV2640_BandWConfig(uint8_t BlackWhite)
{
  OV2640_WriteReg(0xff, 0x00);
  OV2640_WriteReg(0x7c, 0x00);
  OV2640_WriteReg(0x7d, BlackWhite);
  OV2640_WriteReg(0x7c, 0x05);
  OV2640_WriteReg(0x7d, 0x80);
  OV2640_WriteReg(0x7d, 0x80);
}

/**
  * @brief  Configures the OV2640 camera color effects.
  * @param  value1: Color effects value1
  * @param  value2: Color effects value2
  *         where value1 and value2 can be:
  *         value1 = 0x40, value2 = 0xa6 for Antique,
  *         value1 = 0xa0, value2 = 0x40 for Bluish,
  *         value1 = 0x40, value2 = 0x40 for Greenish,
  *         value1 = 0x40, value2 = 0xc0 for Reddish,
  * @retval None
  */
void OV2640_ColorEffectsConfig(uint8_t value1, uint8_t value2)
{
  OV2640_WriteReg(0xff, 0x00);
  OV2640_WriteReg(0x7c, 0x00);
  OV2640_WriteReg(0x7d, 0x18);
  OV2640_WriteReg(0x7c, 0x05);
  OV2640_WriteReg(0x7d, value1);
  OV2640_WriteReg(0x7d, value2);
}

/**
  * @brief  Configures the OV2640 camera contrast.
  * @param  value1: Contrast value1
  * @param  value2: Contrast value2
  *         where value1 and value2 can be:
  *         value1 = 0x28, value2 = 0x0c for Contrast +2,
  *         value1 = 0x24, value2 = 0x16 for Contrast +1,
  *         value1 = 0x20, value2 = 0x20 for Contrast 0,
  *         value1 = 0x1c, value2 = 0x2a for Contrast -1,
  *         value1 = 0x18, value2 = 0x34 for Contrast -2,
  * @retval None
  */
void OV2640_ContrastConfig(uint8_t value1, uint8_t value2)
{
  OV2640_WriteReg(0xff, 0x00);
  OV2640_WriteReg(0x7c, 0x00);
  OV2640_WriteReg(0x7d, 0x04);
  OV2640_WriteReg(0x7c, 0x07);
  OV2640_WriteReg(0x7d, 0x20);
  OV2640_WriteReg(0x7d, value1);
  OV2640_WriteReg(0x7d, value2);
  OV2640_WriteReg(0x7d, 0x06);
}

void OV2640_Start(void)
{
  OV2640_DMA_Config(_OV2640->frame->buffer, (_OV2640->frame->length)/4);
}

/**
  * @brief
  * @param
  * @param
  */
void OV2640_DMA_Config(uint8_t *DMA_Memory0BaseAddr, uint32_t DMA_BufferSize)
{

  __HAL_RCC_DMA2_CLK_ENABLE();

  _OV2640->dma->Instance = DMA2_Stream1;
  _OV2640->dma->Init.Request = DMA_REQUEST_DCMI;
  _OV2640->dma->Init.Direction = DMA_PERIPH_TO_MEMORY;
  _OV2640->dma->Init.PeriphInc = DMA_PINC_DISABLE;
  _OV2640->dma->Init.MemInc = DMA_MINC_ENABLE;
  _OV2640->dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  _OV2640->dma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  _OV2640->dma->Init.Mode = DMA_CIRCULAR;
  _OV2640->dma->Init.Priority = DMA_PRIORITY_VERY_HIGH;
  _OV2640->dma->Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  _OV2640->dma->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  _OV2640->dma->Init.MemBurst = DMA_MBURST_INC8;
  _OV2640->dma->Init.PeriphBurst = DMA_PBURST_SINGLE;

  __HAL_LINKDMA(_OV2640->dcmi, DMA_Handle, *(_OV2640->dma));
  HAL_DMA_Init(_OV2640->dma);

  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

  HAL_DCMI_Start_DMA(_OV2640->dcmi, DCMI_MODE_SNAPSHOT, (uint32_t)DMA_Memory0BaseAddr, DMA_BufferSize);
}

/**
  * @brief
  * @param  None
  * @retval None
  */
extern "C"{
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
  _OV2640->fps++;
  _OV2640->show = 1;
}
}



/**
  * @brief
  * @param  ַ
  * @param
  * @retval
  */
void OV2640_WriteReg(uint8_t Addr, uint8_t Data)
{

  _OV2640->cam_i2c->beginTransmission(OV2640_DEVICE_ADDRESS);
  _OV2640->cam_i2c->write(Addr);
  _OV2640->cam_i2c->write(Data);
  _OV2640->cam_i2c->endTransmission();
  return;
}

/**
  * @brief
  * @param
  */
uint8_t OV2640_ReadReg(uint8_t Addr)
{
  uint8_t Data = 0;

  _OV2640->cam_i2c->beginTransmission(OV2640_DEVICE_ADDRESS);
  _OV2640->cam_i2c->write(Addr);
  _OV2640->cam_i2c->endTransmission();
  _OV2640->cam_i2c->requestFrom(OV2640_DEVICE_ADDRESS, 1);
  if (_OV2640->cam_i2c->available())
  {
      Data = _OV2640->cam_i2c->read();
  }
  
 return Data;

}
