#include "sccb.h"

#include "stm32f7xx_ll_i2c.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_gpio.h"

#define SCCB_I2C                        I2C1
#define SCCB_I2C_ENCLK()                LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1)

#define SCCB_I2C_EV_IRQN                I2C1_EV_IRQn
#define SCCB_I2C_EV_IRQN_PRIO           0
#define SCCB_I2C_EV_IRQHANDLER          I2C1_EV_IRQHandler
#define SCCB_I2C_ER_IRQN                I2C1_ER_IRQn
#define SCCB_I2C_ER_IRQN_PRIO           0
#define SCCB_I2C_ER_IRQHANDLER          I2C1_ER_IRQHandler

/* Definition for I2Cx Pins */
#define SCCB_SCL_PIN                    LL_GPIO_PIN_8
#define SCCB_SCL_GPIO_PORT              GPIOB
#define SCCB_SCL_AF                     LL_GPIO_AF_4
#define SCCB_SCL_PIN_ENCLK()            LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB)

#define SCCB_SDA_PIN                    LL_GPIO_PIN_9
#define SCCB_SDA_GPIO_PORT              GPIOB
#define SCCB_SDA_AF                     LL_GPIO_AF_4
#define SCCB_SDA_PIN_ENCLK()            LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB)

typedef enum sccb_task
{
  SCCB_WRITE_REG,
  SCCB_READ_REG,
  SCCB_IDLE,
}sccb_task_t;

static sccb_task_t sccb_task = SCCB_IDLE;
static sccb_state_t sccb_state = SCCB_STOP_STATE;
static uint8_t dev_addr;
static uint8_t dev_reg_addr;
static uint8_t dev_reg_val;
static uint32_t write_cnt;

/* Timing register value is computed with the STM32CubeMX Tool,
  * Fast Mode @400kHz with I2CCLK = 216 MHz,
  * rise time = 100ns, fall time = 20ns
  * Timing Value = (uint32_t)0x00A01E5D
  */
#define I2C_TIMING               0x00A01E5D

static void sccb_gpio_init(void);

/**
  * @brief  Initializes I2C HAL.
  * @param  i2c_handler : I2C handler
  * @retval None
  */
void sccb_init(void)
{
  LL_I2C_InitTypeDef i2c_initstruct;
  
  sccb_gpio_init();
  
  SCCB_I2C_ENCLK();
  
  /* External, camera and Arduino connector  I2C configuration */
  i2c_initstruct.PeripheralMode  = LL_I2C_MODE_I2C;
  i2c_initstruct.Timing          = I2C_TIMING;
  i2c_initstruct.AnalogFilter    = LL_I2C_ANALOGFILTER_ENABLE;
  i2c_initstruct.DigitalFilter   = 0x00;
  i2c_initstruct.OwnAddress1     = 0x00;
  i2c_initstruct.TypeAcknowledge = LL_I2C_ACK;
  i2c_initstruct.OwnAddrSize     = LL_I2C_OWNADDRESS1_7BIT;
  
  /* Init the I2C */
  LL_I2C_Init(SCCB_I2C, &i2c_initstruct);
  
  /* Configure Event IT */
  NVIC_SetPriority(SCCB_I2C_EV_IRQN, SCCB_I2C_EV_IRQN_PRIO);
  NVIC_EnableIRQ(SCCB_I2C_EV_IRQN);

  /* Configure Error IT */
  NVIC_SetPriority(SCCB_I2C_ER_IRQN, SCCB_I2C_ER_IRQN_PRIO);
  NVIC_EnableIRQ(SCCB_I2C_ER_IRQN);
  
  /* (5) Enable I2C1 transfer complete/error interrupts:
   *  - Enable Receive Interrupt
   *  - Enable Not acknowledge received interrupt
   *  - Enable Error interrupts
   *  - Enable Stop interrupt
   */
  LL_I2C_EnableIT_TX(SCCB_I2C);
  LL_I2C_EnableIT_TC(SCCB_I2C);
  LL_I2C_EnableIT_RX(SCCB_I2C);
  LL_I2C_EnableIT_NACK(SCCB_I2C);
  LL_I2C_EnableIT_ERR(SCCB_I2C);
  LL_I2C_EnableIT_STOP(SCCB_I2C);
  
  sccb_state = SCCB_WRITE_STATE;
  sccb_task = SCCB_IDLE;
}


static void sccb_gpio_init(void)
{
  LL_GPIO_InitTypeDef gpio_init_structure;
  
  /*** Configure the GPIOs ***/
  /* Enable GPIO clock */
  SCCB_SCL_PIN_ENCLK();
  SCCB_SDA_PIN_ENCLK();
  
  /* Configure I2C Tx as alternate function */
  gpio_init_structure.Pin       = SCCB_SCL_PIN;
  gpio_init_structure.OutputType= LL_GPIO_OUTPUT_OPENDRAIN;
  gpio_init_structure.Mode      = LL_GPIO_MODE_ALTERNATE;
  gpio_init_structure.Pull      = LL_GPIO_PULL_NO;
  gpio_init_structure.Speed     = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Alternate = SCCB_SCL_AF;
  LL_GPIO_Init(SCCB_SCL_GPIO_PORT, &gpio_init_structure);
  
  /* Configure I2C Rx as alternate function */
  gpio_init_structure.Pin       = SCCB_SDA_PIN;
  gpio_init_structure.OutputType= LL_GPIO_OUTPUT_OPENDRAIN;
  gpio_init_structure.Mode      = LL_GPIO_MODE_ALTERNATE;
  gpio_init_structure.Pull      = LL_GPIO_PULL_NO;
  gpio_init_structure.Speed     = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Alternate = SCCB_SDA_AF;
  LL_GPIO_Init(SCCB_SDA_GPIO_PORT, &gpio_init_structure);
}

void sccb_read_reg(uint8_t addr, uint8_t reg)
{
  dev_addr = addr;
  dev_reg_addr = reg;
  dev_reg_val = 0;
  write_cnt = 0;
  
  sccb_task = SCCB_READ_REG;
  sccb_state = SCCB_WRITE_STATE;
  
  LL_I2C_HandleTransfer(SCCB_I2C, dev_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
}

void sccb_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
  dev_addr = addr;
  dev_reg_val = val;
  dev_reg_addr = reg;
  write_cnt = 0;
  
  sccb_task = SCCB_WRITE_REG;
  sccb_state = SCCB_WRITE_STATE;
  
  LL_I2C_HandleTransfer(SCCB_I2C, dev_addr, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
}

sccb_state_t sccb_wait(void)
{
  return sccb_state;
}

uint8_t sccb_get_reg_val(void)
{
  return dev_reg_val;
}

void SCCB_I2C_EV_IRQHANDLER(void)
{
  /* Check RXNE flag value in ISR register */
  if(LL_I2C_IsActiveFlag_RXNE(SCCB_I2C))
  {
    if(sccb_task == SCCB_READ_REG)
    {
      dev_reg_val = LL_I2C_ReceiveData8(SCCB_I2C);
    }
    else
    {
      
    }
  }
  
  /* Check STOP flag value in ISR register */
  if(LL_I2C_IsActiveFlag_STOP(SCCB_I2C))
  {
    /* End of Transfer */
    LL_I2C_ClearFlag_STOP(SCCB_I2C);
    
    switch(sccb_state)
    {
    case SCCB_WRITE_STATE:
      if(sccb_task == SCCB_READ_REG)
      {
        LL_I2C_HandleTransfer(SCCB_I2C, dev_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
        sccb_state = SCCB_READ_STATE;
      }
      else
      {
        sccb_task = SCCB_IDLE;
        sccb_state = SCCB_STOP_STATE;
      }
      break;
      
    case SCCB_READ_STATE:
      sccb_task = SCCB_IDLE;
      sccb_state = SCCB_STOP_STATE;
      break;
    }
  }
  
  /* Check STOP flag value in ISR register */
  if(LL_I2C_IsActiveFlag_TC(SCCB_I2C))
  {
  }
  
  /* Check STOP flag value in ISR register */
  if(LL_I2C_IsActiveFlag_TXE(SCCB_I2C))
  {
    if(sccb_task == SCCB_WRITE_REG || sccb_task == SCCB_READ_REG)
    {
      if(write_cnt == 0)
      {
        LL_I2C_TransmitData8(SCCB_I2C, dev_reg_addr);
      }
      else if(write_cnt == 1)
      {
        LL_I2C_TransmitData8(SCCB_I2C, dev_reg_val);
      }
      else
      {
        
      }
    }
    else
    {
      
    }
    
    write_cnt++;
  }
}

void SCCB_I2C_ER_IRQHANDLER(void)
{
  
}



