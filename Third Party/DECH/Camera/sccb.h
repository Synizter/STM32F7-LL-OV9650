/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SCCB_H
#define __SCCB_H

#include "stm32f7xx.h"

typedef enum sccb_state
{
  SCCB_WRITE_STATE,
  SCCB_READ_STATE,
  SCCB_STOP_STATE
}sccb_state_t;

void sccb_init(void);
void sccb_read_reg(uint8_t addr, uint8_t reg);
void sccb_write_reg(uint8_t addr, uint8_t reg, uint8_t val);

sccb_state_t sccb_wait(void);
uint8_t sccb_get_reg_val(void);

#endif /* __SCCB_H */