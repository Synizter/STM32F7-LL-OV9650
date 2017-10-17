
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F7xx_DCMI_H
#define __STM32F7xx_DCMI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx.h"

/** @addtogroup STM32F7xx_StdPeriph_Driver
  * @{
  */

/** @addtogroup DCMI
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/
/** 
  * @brief   DCMI Init structure definition  
  */ 
typedef struct
{
  uint32_t CaptureMode;      /*!< Specifies the Capture Mode: Continuous or Snapshot.
                                       This parameter can be a value of @ref DCMI_Capture_Mode */

  uint32_t SynchroMode;      /*!< Specifies the Synchronization Mode: Hardware or Embedded.
                                       This parameter can be a value of @ref DCMI_Synchronization_Mode */

  uint32_t PCKPolarity;      /*!< Specifies the Pixel clock polarity: Falling or Rising.
                                       This parameter can be a value of @ref DCMI_PIXCK_Polarity */

  uint32_t VSPolarity;       /*!< Specifies the Vertical synchronization polarity: High or Low.
                                       This parameter can be a value of @ref DCMI_VSYNC_Polarity */

  uint32_t HSPolarity;       /*!< Specifies the Horizontal synchronization polarity: High or Low.
                                       This parameter can be a value of @ref DCMI_HSYNC_Polarity */

  uint32_t CaptureRate;      /*!< Specifies the frequency of frame capture: All, 1/2 or 1/4.
                                       This parameter can be a value of @ref DCMI_Capture_Rate */

  uint32_t ExtendedDataMode; /*!< Specifies the data width: 8-bit, 10-bit, 12-bit or 14-bit.
                                       This parameter can be a value of @ref DCMI_Extended_Data_Mode */
  
  uint32_t JPEGMode;                    /*!< Enable or Disable the JPEG mode.                                
                                             This parameter can be a value of @ref DCMI_MODE_JPEG            */

  uint32_t ByteSelectMode;              /*!< Specifies the data to be captured by the interface 
                                            This parameter can be a value of @ref DCMIEx_Byte_Select_Start      */
                                            
  uint32_t ByteSelectStart;             /*!< Specifies if the data to be captured by the interface is even or odd
                                            This parameter can be a value of @ref DCMIEx_Byte_Select_Start     */

  uint32_t LineSelectMode;              /*!< Specifies the line of data to be captured by the interface 
                                            This parameter can be a value of @ref DCMIEx_Line_Select_Mode      */
                                            
  uint32_t LineSelectStart;             /*!< Specifies if the line of data to be captured by the interface is even or odd
                                            This parameter can be a value of @ref DCMIEx_Line_Select_Start     */
} DCMI_InitTypeDef;

/** 
  * @brief   DCMI CROP Init structure definition  
  */ 
typedef struct
{
  uint16_t VerticalStartLine;      /*!< Specifies the Vertical start line count from which the image capture
                                             will start. This parameter can be a value between 0x00 and 0x1FFF */

  uint16_t HorizontalOffsetCount;  /*!< Specifies the number of pixel clocks to count before starting a capture.
                                             This parameter can be a value between 0x00 and 0x3FFF */

  uint16_t VerticalLineCount;      /*!< Specifies the number of lines to be captured from the starting point.
                                             This parameter can be a value between 0x00 and 0x3FFF */

  uint16_t CaptureCount;           /*!< Specifies the number of pixel clocks to be captured from the starting
                                             point on the same line.
                                             This parameter can be a value between 0x00 and 0x3FFF */
} DCMI_CROPInitTypeDef;

/** 
  * @brief   DCMI Embedded Synchronisation CODE Init structure definition  
  */ 
typedef struct
{
  uint8_t DCMI_FrameStartCode; /*!< Specifies the code of the frame start delimiter. */
  uint8_t DCMI_LineStartCode;  /*!< Specifies the code of the line start delimiter. */
  uint8_t DCMI_LineEndCode;    /*!< Specifies the code of the line end delimiter. */
  uint8_t DCMI_FrameEndCode;   /*!< Specifies the code of the frame end delimiter. */
} DCMI_CodesInitTypeDef;

/* Exported constants --------------------------------------------------------*/

/** @defgroup DCMI_Exported_Constants
  * @{
  */

/** @defgroup DCMI_Capture_Mode 
  * @{
  */ 
#define DCMI_MODE_CONTINUOUS           ((uint32_t)0x00000000U)  /*!< The received data are transferred continuously 
                                                                    into the destination memory through the DMA             */
#define DCMI_MODE_SNAPSHOT             ((uint32_t)DCMI_CR_CM)  /*!< Once activated, the interface waits for the start of 
                                                                    frame and then transfers a single frame through the DMA */
#define IS_DCMI_CAPTURE_MODE(MODE)(((MODE) == DCMI_MODE_CONTINUOUS) || \
                                   ((MODE) == DCMI_MODE_SNAPSHOT))
/**
  * @}
  */ 

/** @defgroup DCMI_Synchronization_Mode
  * @{
  */ 
#define DCMI_SYNCHRO_HARDWARE        ((uint32_t)0x00000000U)   /*!< Hardware synchronization data capture (frame/line start/stop)
                                                                   is synchronized with the HSYNC/VSYNC signals                  */
#define DCMI_SYNCHRO_EMBEDDED        ((uint32_t)DCMI_CR_ESS)  /*!< Embedded synchronization data capture is synchronized with 
                                                                   synchronization codes embedded in the data flow  */
#define IS_DCMI_SYNCHRO(MODE)(((MODE) == DCMI_SYNCHRO_HARDWARE) || \
                              ((MODE) == DCMI_SYNCHRO_EMBEDDED))
/**
  * @}
  */ 

/** @defgroup DCMI_PIXCK_Polarity 
  * @{
  */ 
#define DCMI_PCKPOLARITY_FALLING    ((uint32_t)0x00000000U)      /*!< Pixel clock active on Falling edge */
#define DCMI_PCKPOLARITY_RISING     ((uint32_t)DCMI_CR_PCKPOL)  /*!< Pixel clock active on Rising edge  */
#define IS_DCMI_PCKPOLARITY(POLARITY)(((POLARITY) == DCMI_PCKPOLARITY_FALLING) || \
                                      ((POLARITY) == DCMI_PCKPOLARITY_RISING))
/**
  * @}
  */ 

/** @defgroup DCMI_VSYNC_Polarity 
  * @{
  */ 
#define DCMI_VSPOLARITY_LOW     ((uint32_t)0x00000000U)     /*!< Vertical synchronization active Low  */
#define DCMI_VSPOLARITY_HIGH    ((uint32_t)DCMI_CR_VSPOL)  /*!< Vertical synchronization active High */
#define IS_DCMI_VSPOLARITY(POLARITY)(((POLARITY) == DCMI_VSPOLARITY_LOW) || \
                                     ((POLARITY) == DCMI_VSPOLARITY_HIGH))
/**
  * @}
  */ 

/** @defgroup DCMI_HSYNC_Polarity 
  * @{
  */ 
#define DCMI_HSPOLARITY_LOW     ((uint32_t)0x00000000U)     /*!< Horizontal synchronization active Low  */
#define DCMI_HSPOLARITY_HIGH    ((uint32_t)DCMI_CR_HSPOL)  /*!< Horizontal synchronization active High */
#define IS_DCMI_HSPOLARITY(POLARITY)(((POLARITY) == DCMI_HSPOLARITY_LOW) || \
                                     ((POLARITY) == DCMI_HSPOLARITY_HIGH))
/**
  * @}
  */ 

/** @defgroup DCMI_Capture_Rate 
  * @{
  */ 
#define DCMI_CR_ALL_FRAME            ((uint32_t)0x00000000U)      /*!< All frames are captured        */
#define DCMI_CR_ALTERNATE_2_FRAME    ((uint32_t)DCMI_CR_FCRC_0)  /*!< Every alternate frame captured */
#define DCMI_CR_ALTERNATE_4_FRAME    ((uint32_t)DCMI_CR_FCRC_1)  /*!< One frame in 4 frames captured */
#define IS_DCMI_CAPTURE_RATE(RATE) (((RATE) == DCMI_CR_ALL_FRAME) || \
                                    ((RATE) == DCMI_CR_ALTERNATE_2_FRAME) ||\
                                    ((RATE) == DCMI_CR_ALTERNATE_4_FRAME))
/**
  * @}
  */ 

/** @defgroup DCMI_Extended_Data_Mode 
  * @{
  */ 
#define DCMI_EXTEND_DATA_8B     ((uint32_t)0x00000000U)                       /*!< Interface captures 8-bit data on every pixel clock  */
#define DCMI_EXTEND_DATA_10B    ((uint32_t)DCMI_CR_EDM_0)                    /*!< Interface captures 10-bit data on every pixel clock */
#define DCMI_EXTEND_DATA_12B    ((uint32_t)DCMI_CR_EDM_1)                    /*!< Interface captures 12-bit data on every pixel clock */
#define DCMI_EXTEND_DATA_14B    ((uint32_t)(DCMI_CR_EDM_0 | DCMI_CR_EDM_1))  /*!< Interface captures 14-bit data on every pixel clock */
#define IS_DCMI_EXTENDED_DATA(DATA)(((DATA) == DCMI_EXTEND_DATA_8B) || \
                                    ((DATA) == DCMI_EXTEND_DATA_10B) ||\
                                    ((DATA) == DCMI_EXTEND_DATA_12B) ||\
                                    ((DATA) == DCMI_EXTEND_DATA_14B))
/**
  * @}
  */ 

/** @defgroup DCMI_MODE_JPEG DCMI MODE JPEG
  * @{
  */
#define DCMI_JPEG_DISABLE   ((uint32_t)0x00000000U)    /*!< Mode JPEG Disabled  */
#define DCMI_JPEG_ENABLE    ((uint32_t)DCMI_CR_JPEG)  /*!< Mode JPEG Enabled   */

/**
  * @}
  */

/** @defgroup DCMIEx_Byte_Select_Mode DCMIEx Byte Select Mode
  * @{
  */
#define DCMI_BSM_ALL                 ((uint32_t)0x00000000) /*!< Interface captures all received data */
#define DCMI_BSM_OTHER               ((uint32_t)DCMI_CR_BSM_0) /*!< Interface captures every other byte from the received data */
#define DCMI_BSM_ALTERNATE_4         ((uint32_t)DCMI_CR_BSM_1) /*!< Interface captures one byte out of four */
#define DCMI_BSM_ALTERNATE_2         ((uint32_t)(DCMI_CR_BSM_0 | DCMI_CR_BSM_1)) /*!< Interface captures two bytes out of four */
#define IS_DCMI_BYTE_SELECT_MODE(MODE)(((MODE) == DCMI_BSM_ALL) || \
                                       ((MODE) == DCMI_BSM_OTHER) || \
                                       ((MODE) == DCMI_BSM_ALTERNATE_4) || \
                                       ((MODE) == DCMI_BSM_ALTERNATE_2))
/**
  * @}
  */

/** @defgroup DCMIEx_Byte_Select_Start DCMIEx Byte Select Start
  * @{
  */ 
#define DCMI_OEBS_ODD               ((uint32_t)0x00000000) /*!< Interface captures first data from the frame/line start, second one being dropped */
#define DCMI_OEBS_EVEN              ((uint32_t)DCMI_CR_OEBS) /*!< Interface captures second data from the frame/line start, first one being dropped */
#define IS_DCMI_BYTE_SELECT_START(POLARITY)(((POLARITY) == DCMI_OEBS_ODD) || \
                                            ((POLARITY) == DCMI_OEBS_EVEN))
/**
  * @}
  */

/** @defgroup DCMIEx_Line_Select_Mode DCMIEx Line Select Mode
  * @{
  */
#define DCMI_LSM_ALL                 ((uint32_t)0x00000000) /*!< Interface captures all received lines */
#define DCMI_LSM_ALTERNATE_2         ((uint32_t)DCMI_CR_LSM) /*!< Interface captures one line out of two */
#define IS_DCMI_LINE_SELECT_MODE(MODE)(((MODE) == DCMI_LSM_ALL) || \
                                       ((MODE) == DCMI_LSM_ALTERNATE_2))
/**
  * @}
  */

/** @defgroup DCMIEx_Line_Select_Start DCMIEx Line Select Start
  * @{
  */ 
#define DCMI_OELS_ODD               ((uint32_t)0x00000000) /*!< Interface captures first line from the frame start, second one being dropped */
#define DCMI_OELS_EVEN              ((uint32_t)DCMI_CR_OELS) /*!< Interface captures second line from the frame start, first one being dropped */
#define IS_DCMI_LINE_SELECT_START(POLARITY)(((POLARITY) == DCMI_OELS_ODD) || \
                                            ((POLARITY) == DCMI_OELS_EVEN))
/**
  * @}
  */

/** @defgroup DCMI_interrupt_sources 
  * @{
  */ 
#define DCMI_IT_FRAME    ((uint32_t)DCMI_IER_FRAME_IE)
#define DCMI_IT_OVR      ((uint32_t)DCMI_IER_OVR_IE)
#define DCMI_IT_ERR      ((uint32_t)DCMI_IER_ERR_IE)
#define DCMI_IT_VSYNC    ((uint32_t)DCMI_IER_VSYNC_IE)
#define DCMI_IT_LINE     ((uint32_t)DCMI_IER_LINE_IE)
#define IS_DCMI_CONFIG_IT(IT) ((((IT) & (uint32_t)0xFFFFFFE0) == 0x00000000) && ((IT) != 0x00000000))
#define IS_DCMI_GET_IT(IT) (((IT) == DCMI_IT_FRAME) || \
                            ((IT) == DCMI_IT_OVR) || \
                            ((IT) == DCMI_IT_ERR) || \
                            ((IT) == DCMI_IT_VSYNC) || \
                            ((IT) == DCMI_IT_LINE))
/**
  * @}
  */ 


/** @defgroup DCMI_Flags 
  * @{
  */ 
/** 
  * @brief   DCMI SR register  
  */ 
#define DCMI_FLAG_HSYNC     ((uint32_t)0x2001)
#define DCMI_FLAG_VSYNC     ((uint32_t)0x2002)
#define DCMI_FLAG_FNE       ((uint32_t)0x2004)
/** 
  * @brief   DCMI RISR register  
  */ 
#define DCMI_FLAG_FRAMERI    ((uint32_t)DCMI_RISR_FRAME_RIS)
#define DCMI_FLAG_OVFRI      ((uint32_t)DCMI_RISR_OVF_RIS)
#define DCMI_FLAG_ERRRI      ((uint32_t)DCMI_RISR_ERR_RIS)
#define DCMI_FLAG_VSYNCRI    ((uint32_t)DCMI_RISR_VSYNC_RIS)
#define DCMI_FLAG_LINERI     ((uint32_t)DCMI_RISR_LINE_RIS)
/** 
  * @brief   DCMI MISR register  
  */ 
#define DCMI_FLAG_FRAMEMI    ((uint32_t)0x1001)
#define DCMI_FLAG_OVFMI      ((uint32_t)0x1002)
#define DCMI_FLAG_ERRMI      ((uint32_t)0x1004)
#define DCMI_FLAG_VSYNCMI    ((uint32_t)0x1008)
#define DCMI_FLAG_LINEMI     ((uint32_t)0x1010)
#define IS_DCMI_GET_FLAG(FLAG) (((FLAG) == DCMI_FLAG_HSYNC) || \
                                ((FLAG) == DCMI_FLAG_VSYNC) || \
                                ((FLAG) == DCMI_FLAG_FNE) || \
                                ((FLAG) == DCMI_FLAG_FRAMERI) || \
                                ((FLAG) == DCMI_FLAG_OVFRI) || \
                                ((FLAG) == DCMI_FLAG_ERRRI) || \
                                ((FLAG) == DCMI_FLAG_VSYNCRI) || \
                                ((FLAG) == DCMI_FLAG_LINERI) || \
                                ((FLAG) == DCMI_FLAG_FRAMEMI) || \
                                ((FLAG) == DCMI_FLAG_OVFMI) || \
                                ((FLAG) == DCMI_FLAG_ERRMI) || \
                                ((FLAG) == DCMI_FLAG_VSYNCMI) || \
                                ((FLAG) == DCMI_FLAG_LINEMI))
                                
#define IS_DCMI_CLEAR_FLAG(FLAG) ((((FLAG) & (uint32_t)0xFFFFFFE0) == 0x00000000) && ((FLAG) != 0x00000000))
/**
  * @}
  */ 

/**
  * @}
  */ 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 

/*  Function used to set the DCMI configuration to the default reset state ****/ 
void DCMI_DeInit(void);

/* Initialization and Configuration functions *********************************/
void DCMI_Init(DCMI_InitTypeDef* DCMI_InitStruct);
void DCMI_StructInit(DCMI_InitTypeDef* DCMI_InitStruct);
void DCMI_CROPConfig(DCMI_CROPInitTypeDef* DCMI_CROPInitStruct);
void DCMI_CROPCmd(FunctionalState NewState);
void DCMI_SetEmbeddedSynchroCodes(DCMI_CodesInitTypeDef* DCMI_CodesInitStruct);
void DCMI_JPEGCmd(FunctionalState NewState);
void DCMI_ByteModeConfig(uint32_t DCMI_ByteSelectMode, uint32_t DCMI_ByteSelectStart);
void DCMI_LineModeConfig(uint32_t DCMI_LineSelectMode, uint32_t DCMI_LineSelectStart);
void DCMI_CaptureModeConfig(uint32_t DCMI_CaptureMode);

/* Image capture functions ****************************************************/
void DCMI_Cmd(FunctionalState NewState);
void DCMI_CaptureCmd(FunctionalState NewState);
uint32_t DCMI_ReadData(void);

/* Interrupts and flags management functions **********************************/
void DCMI_ITConfig(uint32_t DCMI_IT, FunctionalState NewState);
FlagStatus DCMI_GetFlagStatus(uint32_t DCMI_FLAG);
void DCMI_ClearFlag(uint32_t DCMI_FLAG);
ITStatus DCMI_GetITStatus(uint32_t DCMI_IT);
void DCMI_ClearITPendingBit(uint32_t DCMI_IT);

#ifdef __cplusplus
}
#endif

#endif /*__STM32F7xx_DCMI_H */

/**
  * @}
  */ 

/**
  * @}
  */ 