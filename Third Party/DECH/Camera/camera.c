#include "ov9655.h"
#include "camera.h"
#include "sccb.h"

#include "stm32f7xx_dcmi.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_utils.h"

/** @defgroup STM32746G_DISCOVERY_CAMERA_Private_Defines STM32746G_DISCOVERY_CAMERA Private Defines
  * @{
  */
#define CAMERA_VGA_RES_X                640
#define CAMERA_VGA_RES_Y                480
#define CAMERA_480x272_RES_X            480
#define CAMERA_480x272_RES_Y            272
#define CAMERA_QVGA_RES_X               320
#define CAMERA_QVGA_RES_Y               240
#define CAMERA_QQVGA_RES_X              160
#define CAMERA_QQVGA_RES_Y              120
/**
  * @}
  */ 

static int CameraCurrentCaptureMode;
static int CameraCurrentResolution;

static int camera_start_capture(uint32_t mode, uint32_t buffer);
static void camera_reg_init(uint8_t dev_addr, const unsigned char config[][2], int num);
static void camera_gpio_init(void);
static uint32_t camera_get_pixels_size(uint32_t resolution);

/**
  * @brief  Initializes the camera.
  * @param  Resolution : camera sensor requested resolution (x, y) : standard resolution
  *         naming QQVGA, QVGA, VGA ...
  * @retval Camera status
  */
int camera_init(int Resolution)
{ 
  DCMI_InitTypeDef dcmi_init_struct;
  LL_DMA_InitTypeDef dma_init_struct;
  DCMI_CROPInitTypeDef dcmi_crop_init_struct;
  int status = 0;
  
  camera_gpio_init();
  
  /* Power up camera */
  camera_pwr_up();
  
  /* Serial init. */
  sccb_init();
  
  sccb_read_reg(OV9655_SCCB_ADDR, OV9655_SENSOR_PIDH);
  
  while(sccb_wait() != SCCB_STOP_STATE);
  
  /* Read ID of Camera module via I2C */
  if(sccb_get_reg_val() == OV9655_ID)
  {
    /*** Enable peripherals and GPIO clocks ***/
    /* Enable DCMI clock */
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DCMI);
    
    /* Enable CAMERA_DMA clock */
    CAMERA_DMA_ENCLK();
    
    LL_DMA_StructInit(&dma_init_struct);
    
    dma_init_struct.PeriphOrM2MSrcAddress  = (uint32_t)&(DCMI->DR);
    dma_init_struct.MemoryOrM2MDstAddress  = 0x00000000U;
    dma_init_struct.Direction              = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    dma_init_struct.Mode                   = LL_DMA_MODE_CIRCULAR;
    dma_init_struct.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
    dma_init_struct.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
    dma_init_struct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
    dma_init_struct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
    dma_init_struct.NbData                 = 0x00000000U;
    dma_init_struct.Channel                = LL_DMA_CHANNEL_1;
    dma_init_struct.Priority               = LL_DMA_PRIORITY_HIGH;
    dma_init_struct.FIFOMode               = LL_DMA_FIFOMODE_DISABLE;
    dma_init_struct.FIFOThreshold          = LL_DMA_FIFOTHRESHOLD_FULL;
    dma_init_struct.MemBurst               = LL_DMA_MBURST_SINGLE;
    dma_init_struct.PeriphBurst            = LL_DMA_PBURST_SINGLE;
    LL_DMA_Init(CAMERA_DMA, CAMERA_DMA_STREAM, &dma_init_struct);

    /*** Configures the DCMI to interface with the camera module ***/
    /* DCMI configuration */
    
    DCMI_StructInit(&dcmi_init_struct);
    
    dcmi_init_struct.CaptureRate      = DCMI_CR_ALL_FRAME;
    dcmi_init_struct.HSPolarity       = DCMI_HSPOLARITY_LOW;
    dcmi_init_struct.SynchroMode      = DCMI_SYNCHRO_HARDWARE;
    dcmi_init_struct.VSPolarity       = DCMI_VSPOLARITY_HIGH;
    dcmi_init_struct.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
    dcmi_init_struct.PCKPolarity      = DCMI_PCKPOLARITY_RISING;
    DCMI_Init(&dcmi_init_struct);
    
    /* Configure Event IT */
    NVIC_SetPriority(CAMERA_DMA_IRQN, CAMERA_DMA_IRQN_PRIO);
    NVIC_EnableIRQ(CAMERA_DMA_IRQN);
    
    /* Camera Module Initialization via I2C to the wanted 'Resolution' */
    if(Resolution == CAMERA_R480x272)
    {
          /* For 480x272 resolution, the OV9655 sensor is set to VGA resolution
           * as OV9655 doesn't supports 480x272 resolution,
           * then DCMI is configured to output a 480x272 cropped window */
      camera_reg_init(OV9655_SCCB_ADDR, OV9655_VGA, (sizeof(OV9655_VGA)/2));
      
      dcmi_crop_init_struct.CaptureCount = (CAMERA_480x272_RES_X * 2) - 1;
      dcmi_crop_init_struct.VerticalLineCount = CAMERA_480x272_RES_Y - 1;
      dcmi_crop_init_struct.HorizontalOffsetCount = (CAMERA_VGA_RES_X - CAMERA_480x272_RES_X)/2;
      dcmi_crop_init_struct.VerticalStartLine = (CAMERA_VGA_RES_Y - CAMERA_480x272_RES_Y)/2;
      DCMI_CROPConfig(&dcmi_crop_init_struct);
      DCMI_CROPCmd(ENABLE);
    }
    else
    {
      switch(Resolution)
      {
      case OV9655_R160X120:
        camera_reg_init(OV9655_SCCB_ADDR, OV9655_QQVGA, (sizeof(OV9655_QQVGA)/2));
        break;
        
      case OV9655_R320X240:
        camera_reg_init(OV9655_SCCB_ADDR, OV9655_QVGA, (sizeof(OV9655_QVGA)/2));
        break;
        
      case OV9655_R640X480:
        camera_reg_init(OV9655_SCCB_ADDR, OV9655_VGA, (sizeof(OV9655_VGA)/2));
        break;
      default:
        status = -1;
        break;
      }
      
      DCMI_CROPCmd(DISABLE);
    }
    
    CameraCurrentResolution = Resolution;
  }
  else
  {
    /* Return CAMERA_NOT_SUPPORTED status */
    status = -2;
  }

  return status;
}

/**
  * @brief  Initializes the OV9655 CAMERA component.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  resolution: Camera resolution
  * @retval None
  */
static void camera_reg_init(uint8_t dev_addr, const unsigned char config[][2], int num)
{
  unsigned int i;
  
  /* Prepare the camera to be configured by resetting all its registers */
  sccb_write_reg(dev_addr, OV9655_SENSOR_COM7, 0x80);
  LL_mDelay(200);
  
  /* Initialize OV9655 */
  for(i = 0; i < num; i++)
  {
    sccb_write_reg(dev_addr, config[i][0], config[i][1]);
    LL_mDelay(2);
  }
}

/**
  * @brief  CANERA power up
  * @retval None
  */
void camera_pwr_up(void)
{
  /* De-assert the camera POWER_DOWN pin (active high) */
  LL_GPIO_ResetOutputPin(GPIOH, LL_GPIO_PIN_13);
  
  LL_mDelay(3);     /* POWER_DOWN de-asserted during 3ms */
}


/**
  * @brief  CAMERA power down
  * @retval None
  */
void camera_pwr_down(void)
{
  /* Assert the camera POWER_DOWN pin (active high) */
  LL_GPIO_SetOutputPin(GPIOH, LL_GPIO_PIN_13);
}

/**
  * @brief  Start capture with continuous mode.
  * @param  buffer: The destination memory Buffer address.
  * @retval HAL status
  */
int camera_capture_continuous(uint32_t buffer)
{
  return camera_start_capture(DCMI_MODE_CONTINUOUS, buffer);
}

/**
  * @brief  Start capture with snapshot mode.
  * @param  buffer: The destination memory Buffer address.
  * @retval HAL status
  */
int camera_capture_snapshot(uint32_t buffer)
{
  return camera_start_capture(DCMI_MODE_SNAPSHOT, buffer);
}

/**
  * @brief  Enables DCMI DMA request and enables DCMI capture  
  * @param  mode:  DCMI capture mode snapshot or continuous grab.
  * @param  buffer: The destination memory Buffer address.
  * @retval HAL status
  */
static int camera_start_capture(uint32_t mode, uint32_t buffer)
{  
  /* Initialize the second memory address */
  uint32_t SecondMemAddress = 0;
  uint32_t TransferCount = 0;
  uint32_t TransferLength1 = 0;
  uint32_t TransferLength0;
  
  TransferLength0 = camera_get_pixels_size(CameraCurrentResolution);
  
  /* Enable DCMI by setting DCMIEN bit */
  DCMI_Cmd(ENABLE);
  
  /* Configure the DCMI Mode */
  DCMI_CaptureModeConfig(mode);
  CameraCurrentCaptureMode = mode;
  
  if(TransferLength0 <= 0xFFFF)
  {
    /* Disable the peripheral */
    LL_DMA_DisableStream(CAMERA_DMA, CAMERA_DMA_STREAM);  
    
    /* Enable the DMA Stream */
    LL_DMA_SetMemoryAddress(CAMERA_DMA, CAMERA_DMA_STREAM, buffer);
    LL_DMA_SetDataLength(CAMERA_DMA, CAMERA_DMA_STREAM, TransferLength0);
    
    /* Enable all interrupts */
    LL_DMA_EnableIT_TC(CAMERA_DMA, CAMERA_DMA_STREAM);
    LL_DMA_EnableIT_HT(CAMERA_DMA, CAMERA_DMA_STREAM);
    LL_DMA_EnableIT_TE(CAMERA_DMA, CAMERA_DMA_STREAM);
    LL_DMA_EnableIT_DME(CAMERA_DMA, CAMERA_DMA_STREAM);
    LL_DMA_EnableIT_FE(CAMERA_DMA, CAMERA_DMA_STREAM);
    
    /* Enable the DMA */
    LL_DMA_EnableStream(CAMERA_DMA, CAMERA_DMA_STREAM);
  }
  else /* DCMI_DOUBLE_BUFFER Mode */
  {
    /* Set the DMA memory1 conversion complete callback */
    /* Initialize transfer parameters */
    TransferCount = 1;
    TransferLength1 = TransferLength0;
      
    /* Get the number of buffer */
    while(TransferLength1 > 0xFFFF)
    {
      TransferLength1 = (TransferLength1/2);
      TransferCount = TransferCount*2;
    }

    /* Update DCMI counter  and transfer number*/
    TransferCount = (TransferCount - 2);

    /* Update second memory address */
    SecondMemAddress = (uint32_t)(buffer + (4*TransferLength1));
    
    /* Start DMA multi buffer transfer */
    /* Disable the peripheral */
    LL_DMA_DisableStream(CAMERA_DMA, CAMERA_DMA_STREAM);  

    /* Enable the Double buffer mode */
    LL_DMA_EnableDoubleBufferMode(CAMERA_DMA, CAMERA_DMA_STREAM);

    /* Configure DMA Stream destination address */
    LL_DMA_SetMemoryAddress(CAMERA_DMA, CAMERA_DMA_STREAM, buffer);
    LL_DMA_SetMemory1Address(CAMERA_DMA, CAMERA_DMA_STREAM, SecondMemAddress);
    LL_DMA_SetDataLength(CAMERA_DMA, CAMERA_DMA_STREAM, TransferLength1);
    
    /* Enable all interrupts */
    LL_DMA_EnableIT_TC(CAMERA_DMA, CAMERA_DMA_STREAM);
    LL_DMA_EnableIT_HT(CAMERA_DMA, CAMERA_DMA_STREAM);
    LL_DMA_EnableIT_TE(CAMERA_DMA, CAMERA_DMA_STREAM);
    LL_DMA_EnableIT_DME(CAMERA_DMA, CAMERA_DMA_STREAM);
    LL_DMA_EnableIT_FE(CAMERA_DMA, CAMERA_DMA_STREAM);
    
    /* Enable the DMA */
    LL_DMA_EnableStream(CAMERA_DMA, CAMERA_DMA_STREAM);
  }
  
  /* Enable Capture */
  DCMI_CaptureCmd(ENABLE);

  /* Return function status */
  return 0;
}

/**
  * @brief  CANERA DCMI gpio init
  * @retval None
  */
static void camera_gpio_init(void)
{
  LL_GPIO_InitTypeDef gpio_init_structure;
  
  /* Enable GPIO clocks */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  
  /*** Configure the GPIO Power Down ***/
  /* Configure DCMI GPIO as alternate function */
  gpio_init_structure.Pin       = LL_GPIO_PIN_13;
  gpio_init_structure.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init_structure.Mode      = LL_GPIO_MODE_OUTPUT;
  gpio_init_structure.Pull      = LL_GPIO_PULL_NO;
  gpio_init_structure.Speed     = LL_GPIO_SPEED_FREQ_HIGH;
  LL_GPIO_Init(GPIOH, &gpio_init_structure);

  /*** Configure the GPIO ***/
  /* Configure DCMI GPIO as alternate function */
  gpio_init_structure.Pin       = LL_GPIO_PIN_4 | LL_GPIO_PIN_6;
  gpio_init_structure.Mode      = LL_GPIO_MODE_ALTERNATE;
  gpio_init_structure.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init_structure.Pull      = LL_GPIO_PULL_UP;
  gpio_init_structure.Speed     = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOA, &gpio_init_structure);

  gpio_init_structure.Pin       = LL_GPIO_PIN_3;
  gpio_init_structure.Mode      = LL_GPIO_MODE_ALTERNATE;
  gpio_init_structure.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init_structure.Pull      = LL_GPIO_PULL_UP;
  gpio_init_structure.Speed     = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOD, &gpio_init_structure);

  gpio_init_structure.Pin       = LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
  gpio_init_structure.Mode      = LL_GPIO_MODE_ALTERNATE;
  gpio_init_structure.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init_structure.Pull      = LL_GPIO_PULL_UP;
  gpio_init_structure.Speed     = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOE, &gpio_init_structure);

  gpio_init_structure.Pin       = LL_GPIO_PIN_9;
  gpio_init_structure.Mode      = LL_GPIO_MODE_ALTERNATE;
  gpio_init_structure.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init_structure.Pull      = LL_GPIO_PULL_UP;
  gpio_init_structure.Speed     = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOG, &gpio_init_structure);

  gpio_init_structure.Pin       = LL_GPIO_PIN_9 | LL_GPIO_PIN_10  | LL_GPIO_PIN_11  |\
                                  LL_GPIO_PIN_12 | LL_GPIO_PIN_14;
  gpio_init_structure.Mode      = LL_GPIO_MODE_ALTERNATE;
  gpio_init_structure.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init_structure.Pull      = LL_GPIO_PULL_UP;
  gpio_init_structure.Speed     = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOH, &gpio_init_structure);
}

/**
  * @brief  Get the capture size in pixels unit.
  * @param  resolution: the current resolution.
  * @retval capture size in pixels unit.
  */
static uint32_t camera_get_pixels_size(uint32_t resolution)
{ 
  uint32_t size = 0;
  
  /* Get capture size */
  switch (resolution)
  {
  case CAMERA_R160x120:
    {
      size =  0x2580;
    }
    break;    
  case CAMERA_R320x240:
    {
      size =  0x9600;
    }
    break;
  case CAMERA_R480x272:
    {
      size =  0xFF00;
    }
    break;
  case CAMERA_R640x480:
    {
      size =  0x25800;
    }    
    break;
  default:
    {
      break;
    }
  }
  
  return size;
}

void CAMERA_DMA_IRQHANDLER(void)
{
  if(LL_DMA_IsActiveFlag_TC1(CAMERA_DMA))
  {
    LL_DMA_ClearFlag_TC1(CAMERA_DMA);
  }
  
  if(LL_DMA_IsActiveFlag_HT1(CAMERA_DMA))
  {
    LL_DMA_ClearFlag_HT1(CAMERA_DMA);
  }

  if(LL_DMA_IsActiveFlag_TE1(CAMERA_DMA))
  {
    LL_DMA_ClearFlag_TE1(CAMERA_DMA);
  }
  
  if(LL_DMA_IsActiveFlag_DME1(CAMERA_DMA))
  {
    LL_DMA_ClearFlag_DME1(CAMERA_DMA);
  }
  
  if(LL_DMA_IsActiveFlag_FE1(CAMERA_DMA))
  {
    LL_DMA_ClearFlag_FE1(CAMERA_DMA);
  }
}