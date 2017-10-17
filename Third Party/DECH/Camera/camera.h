/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAMERA_H
#define __CAMERA_H

#include "stm32f7xx.h"
#include "ov9655.h"

#define CAMERA_DMA                      DMA2
#define CAMERA_DMA_STREAM               LL_DMA_STREAM_1
#define CAMERA_DMA_ENCLK()              LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2)
#define CAMERA_DMA_IRQN                 DMA2_Stream1_IRQn
#define CAMERA_DMA_IRQN_PRIO            5
#define CAMERA_DMA_IRQHANDLER           DMA2_Stream1_IRQHandler

/** @defgroup CAMERA_Exported_Constants
  * @{
  */
#define CAMERA_R160x120                 OV9655_R160X120   /* QQVGA Resolution                     */
#define CAMERA_R320x240                 OV9655_R320X240   /* QVGA Resolution                      */
#define CAMERA_R480x272                 0x03              /* 480x272 Resolution                   */
#define CAMERA_R640x480                 OV9655_R640X480   /* VGA Resolution                       */

#define CAMERA_CONTRAST_BRIGHTNESS      0x00   /* Camera contrast brightness features  */
#define CAMERA_BLACK_WHITE              0x01   /* Camera black white feature           */
#define CAMERA_COLOR_EFFECT             0x03   /* Camera color effect feature          */

#define CAMERA_BRIGHTNESS_LEVEL0        0x00   /* Brightness level -2         */
#define CAMERA_BRIGHTNESS_LEVEL1        0x01   /* Brightness level -1         */
#define CAMERA_BRIGHTNESS_LEVEL2        0x02   /* Brightness level 0          */
#define CAMERA_BRIGHTNESS_LEVEL3        0x03   /* Brightness level +1         */
#define CAMERA_BRIGHTNESS_LEVEL4        0x04   /* Brightness level +2         */

#define CAMERA_CONTRAST_LEVEL0          0x05   /* Contrast level -2           */
#define CAMERA_CONTRAST_LEVEL1          0x06   /* Contrast level -1           */
#define CAMERA_CONTRAST_LEVEL2          0x07   /* Contrast level  0           */
#define CAMERA_CONTRAST_LEVEL3          0x08   /* Contrast level +1           */
#define CAMERA_CONTRAST_LEVEL4          0x09   /* Contrast level +2           */    
    
#define CAMERA_BLACK_WHITE_BW           0x00   /* Black and white effect      */
#define CAMERA_BLACK_WHITE_NEGATIVE     0x01   /* Negative effect             */
#define CAMERA_BLACK_WHITE_BW_NEGATIVE  0x02   /* BW and Negative effect      */
#define CAMERA_BLACK_WHITE_NORMAL       0x03   /* Normal effect               */
                                        
#define CAMERA_COLOR_EFFECT_NONE        0x00   /* No effects                  */
#define CAMERA_COLOR_EFFECT_BLUE        0x01   /* Blue effect                 */
#define CAMERA_COLOR_EFFECT_GREEN       0x02   /* Green effect                */
#define CAMERA_COLOR_EFFECT_RED         0x03   /* Red effect                  */
#define CAMERA_COLOR_EFFECT_ANTIQUE     0x04   /* Antique effect              */

/**
  * @}
  */

int camera_init(int Resolution);
void camera_pwr_up(void);
void camera_pwr_down(void);
int camera_capture_continuous(uint32_t buffer);
int camera_capture_snapshot(uint32_t buffer);

#endif /* __CAMERA_H */