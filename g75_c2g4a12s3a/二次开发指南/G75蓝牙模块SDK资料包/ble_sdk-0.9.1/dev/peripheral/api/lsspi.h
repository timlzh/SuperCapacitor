#ifndef LSSPI_H_
#define LSSPI_H_
#include <stdbool.h>
#include "reg_spi_type.h"
#include "HAL_def.h"
#include "sdk_config.h"
#include "reg_base_addr.h"

#define SPI2 ((reg_spi_t *)SPI2_BASE_ADDR)


#define SPI_CLOCK   (SDK_PCLK_MHZ*1000000)


/**
  * @brief  SPI Configuration Structure definition
  */
typedef struct
{
  uint32_t Mode;                /*!< Specifies the SPI operating mode.
                                     This parameter can be a value of @ref SPI_Mode */

  uint32_t Direction;           /*!< Specifies the SPI bidirectional mode state.
                                     This parameter can be a value of @ref SPI_Direction */

  uint32_t DataSize;            /*!< Specifies the SPI data size.
                                     This parameter can be a value of @ref SPI_Data_Size */

  uint32_t CLKPolarity;         /*!< Specifies the serial clock steady state.
                                     This parameter can be a value of @ref SPI_Clock_Polarity */

  uint32_t CLKPhase;            /*!< Specifies the clock active edge for the bit capture.
                                     This parameter can be a value of @ref SPI_Clock_Phase */

  uint32_t NSS;                 /*!< Specifies whether the NSS signal is managed by
                                     hardware (NSS pin) or by software using the SSI bit.
                                     This parameter can be a value of @ref SPI_Slave_Select_management */

  uint32_t BaudRatePrescaler;   /*!< Specifies the Baud Rate prescaler value which will be
                                     used to configure the transmit and receive SCK clock.
                                     This parameter can be a value of @ref SPI_BaudRate_Prescaler
                                     @note The communication clock is derived from the master
                                     clock. The slave clock does not need to be set. */

  uint32_t FirstBit;            /*!< Specifies whether data transfers start from MSB or LSB bit.
                                     This parameter can be a value of @ref SPI_MSB_LSB_transmission */

  uint32_t TIMode;              /*!< Specifies if the TI mode is enabled or not.
                                     This parameter can be a value of @ref SPI_TI_mode */

  uint32_t CRCCalculation;      /*!< Specifies if the CRC calculation is enabled or not.
                                     This parameter can be a value of @ref SPI_CRC_Calculation */
} SPI_InitTypeDef;

/**
  * @brief  HAL SPI State structure definition
  */
typedef enum
{
  HAL_SPI_STATE_RESET      = 0x00U,    /*!< Peripheral not Initialized                         */
  HAL_SPI_STATE_READY      = 0x01U,    /*!< Peripheral Initialized and ready for use           */
  HAL_SPI_STATE_BUSY       = 0x02U,    /*!< an internal process is ongoing                     */
  HAL_SPI_STATE_BUSY_TX    = 0x03U,    /*!< Data Transmission process is ongoing               */
  HAL_SPI_STATE_BUSY_RX    = 0x04U,    /*!< Data Reception process is ongoing                  */
  HAL_SPI_STATE_BUSY_TX_RX = 0x05U,    /*!< Data Transmission and Reception process is ongoing */
  HAL_SPI_STATE_ERROR      = 0x06U,    /*!< SPI error state                                    */
  HAL_SPI_STATE_ABORT      = 0x07U     /*!< SPI abort is ongoing                               */
} HAL_SPI_StateTypeDef;

/**
  * @brief  SPI handle Structure definition
  */
 struct SPI_DMA_Env
{
    uint8_t DMA_Channel;
    bool DMA_EN;
};

struct SPI_Interrupt_Env
{
    uint8_t *Data;
    uint16_t Count;
};

typedef struct __SPI_HandleTypeDef
{
  reg_spi_t               	 *Instance;      /*!< SPI registers base address               */
  SPI_InitTypeDef            Init;           /*!< SPI communication parameters             */
  uint8_t                    *pTxBuffPtr;    /*!< Pointer to SPI Tx transfer Buffer        */
  uint16_t                   TxXferSize;     /*!< SPI Tx Transfer size                     */
  uint16_t              	 TxXferCount;    /*!< SPI Tx Transfer Counter                  */
  uint8_t                    *pRxBuffPtr;    /*!< Pointer to SPI Rx transfer Buffer        */
  uint16_t                   RxXferSize;     /*!< SPI Rx Transfer size                     */
  uint16_t              	 RxXferCount;    /*!< SPI Rx Transfer Counter                  */
  void (*RxISR)(struct __SPI_HandleTypeDef *hspi);   /*!< function pointer on Rx ISR       */
  void (*TxISR)(struct __SPI_HandleTypeDef *hspi);   /*!< function pointer on Tx ISR       */
  void                       *DMAC_Instance;
  union{
        struct SPI_DMA_Env DMA;
  }Tx_Env,Rx_Env;
  HAL_LockTypeDef            Lock;           /*!< Locking object                           */
  HAL_SPI_StateTypeDef  	 State;          /*!< SPI communication state                  */
  uint32_t              	 ErrorCode;      /*!< SPI Error code                           */

} SPI_HandleTypeDef;


/* Exported constants --------------------------------------------------------*/
/** @defgroup SPI_Error_Code SPI Error Code
  * @{
  */
#define HAL_SPI_ERROR_NONE              (0x00000000U)   /*!< No error                               */
#define HAL_SPI_ERROR_MODF              (0x00000001U)   /*!< MODF error                             */
#define HAL_SPI_ERROR_CRC               (0x00000002U)   /*!< CRC error                              */
#define HAL_SPI_ERROR_OVR               (0x00000004U)   /*!< OVR error                              */
#define HAL_SPI_ERROR_DMA               (0x00000010U)   /*!< DMA transfer error                     */
#define HAL_SPI_ERROR_FLAG              (0x00000020U)   /*!< Error on RXNE/TXE/BSY Flag             */
#define HAL_SPI_ERROR_ABORT             (0x00000040U)   /*!< Error during SPI Abort procedure       */
/**
  * @}
  */

/** @defgroup SPI_Mode SPI Mode
  * @{
  */
#define SPI_MODE_SLAVE                  (0x00000000U)
#define SPI_MODE_MASTER                 (SPI_CR1_MSTR_MASK | SPI_CR1_SSI_MASK)
/**
  * @}
  */

/** @defgroup SPI_Direction SPI Direction Mode
  * @{
  */
#define SPI_DIRECTION_2LINES            (0x00000000U)
#define SPI_DIRECTION_2LINES_RXONLY     SPI_CR1_RXONLY_MASK
#define SPI_DIRECTION_1LINE             SPI_CR1_BIDIMODE_MASK
/**
  * @}
  */

/** @defgroup SPI_Data_Size SPI Data Size
  * @{
  */
#define SPI_DATASIZE_8BIT               (0x00000007U<<SPI_CR2_DS_POS)
#define SPI_DATASIZE_16BIT              SPI_CR2_DS_MASK
/**
  * @}
  */

/** @defgroup SPI_Clock_Polarity SPI Clock Polarity
  * @{
  */
#define SPI_POLARITY_LOW                (0x00000000U)
#define SPI_POLARITY_HIGH               SPI_CR1_CPOL_MASK
/**
  * @}
  */

/** @defgroup SPI_Clock_Phase SPI Clock Phase
  * @{
  */
#define SPI_PHASE_1EDGE                 (0x00000000U)
#define SPI_PHASE_2EDGE                 SPI_CR1_CPHA_MASK
/**
  * @}
  */

/** @defgroup SPI_Slave_Select_management SPI Slave Select Management
  * @{
  */
#define SPI_NSS_SOFT                    SPI_CR1_SSM_MASK
#define SPI_NSS_HARD_INPUT              (0x00000000U)
#define SPI_NSS_HARD_OUTPUT             (SPI_CR2_SSOE_MASK << 16U)
/**
  * @}
  */

/** @defgroup SPI_BaudRate_Prescaler SPI BaudRate Prescaler
  * @{
  */
#define SPI_BAUDRATEPRESCALER_8         (0x00000002<<SPI_CR1_BR_POS)
#define SPI_BAUDRATEPRESCALER_16        (0x00000003<<SPI_CR1_BR_POS)
#define SPI_BAUDRATEPRESCALER_32        (0x00000004<<SPI_CR1_BR_POS)
#define SPI_BAUDRATEPRESCALER_64        (0x00000005<<SPI_CR1_BR_POS)
#define SPI_BAUDRATEPRESCALER_128       (0x00000006<<SPI_CR1_BR_POS)
#define SPI_BAUDRATEPRESCALER_256       (0x00000007<<SPI_CR1_BR_POS)
/**
  * @}
  */

/** @defgroup SPI_MSB_LSB_transmission SPI MSB LSB Transmission
  * @{
  */
#define SPI_FIRSTBIT_MSB                (0x00000000U)
#define SPI_FIRSTBIT_LSB                SPI_CR1_LSBFIRST_MASK
/**
  * @}
  */

/** @defgroup SPI_TI_mode SPI TI Mode
  * @{
  */
#define SPI_TIMODE_DISABLE              (0x00000000U)
/**
  * @}
  */

/** @defgroup SPI_Interrupt_definition SPI Interrupt Definition
  * @{
  */
#define SPI_IT_TXE                      SPI_IER_TXEIE_MASK
#define SPI_IT_RXNE                     SPI_IER_RXNEIE_MASK
#define SPI_IT_ERR                      (SPI_IER_MODFIE_MASK | SPI_IER_OVRIE_MASK)
/**
  * @}
  */

/** @defgroup SPI_Flags_definition SPI Flags Definition
  * @{
  */
#define SPI_FLAG_RXNE                   SPI_SR_RXNE_MASK   /* SPI status flag: Rx buffer not empty flag       */
#define SPI_FLAG_TXE                    SPI_SR_TXE_MASK    /* SPI status flag: Tx buffer empty flag           */
#define SPI_FLAG_BSY                    SPI_SR_BSY_MASK    /* SPI status flag: Busy flag                      */
#define SPI_FLAG_CRCERR                 SPI_SR_CRCERR_MASK /* SPI Error flag: CRC error flag                  */
#define SPI_FLAG_MODF                   SPI_SR_MODF_MASK   /* SPI Error flag: Mode fault flag                 */
#define SPI_FLAG_OVR                    SPI_SR_OVR_MASK    /* SPI Error flag: Overrun flag                    */
#define SPI_FLAG_MASK                   (SPI_SR_RXNE_MASK | SPI_SR_TXE_MASK | SPI_SR_BSY_MASK\
                                         | SPI_SR_CRCERR_MASK | SPI_SR_MODF_MASK | SPI_SR_OVR_MASK)
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup SPI_Exported_Macros SPI Exported Macros
  * @{
  */

/** @brief  Reset SPI handle state.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
#define __HAL_SPI_RESET_HANDLE_STATE(__HANDLE__)                do{                                                  \
                                                                    (__HANDLE__)->State = HAL_SPI_STATE_RESET;       \
                                                                    (__HANDLE__)->MspInitCallback = NULL;            \
                                                                    (__HANDLE__)->MspDeInitCallback = NULL;          \
                                                                  } while(0)
#else
#define __HAL_SPI_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_SPI_STATE_RESET)
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

/** @brief  Enable the specified SPI interrupts.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__ specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval None
  */
#define __HAL_SPI_ENABLE_IT(__HANDLE__, __INTERRUPT__)   SET_BIT((__HANDLE__)->Instance->IER, (__INTERRUPT__))

/** @brief  Disable the specified SPI interrupts.
  * @param  __HANDLE__ specifies the SPI handle.
  *         This parameter can be SPIx where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__ specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval None
  */
#define __HAL_SPI_DISABLE_IT(__HANDLE__, __INTERRUPT__)  SET_BIT((__HANDLE__)->Instance->IDR, (__INTERRUPT__))

/** @brief  Clear the specified SPI interrupts IF.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__ specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval None
  */
#define __HAL_SPI_CLEAR_IF(__HANDLE__, __INTERRUPT__)   SET_BIT((__HANDLE__)->Instance->ICR, (__INTERRUPT__))

/** @brief  Check whether the specified SPI interrupt source is enabled or not.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__ specifies the SPI interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
#define __HAL_SPI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->IVS\
                                                              & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/** @brief  Check whether the specified SPI flag is set or not.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_FLAG_RXNE: Receive buffer not empty flag
  *            @arg SPI_FLAG_TXE: Transmit buffer empty flag
  *            @arg SPI_FLAG_CRCERR: CRC error flag
  *            @arg SPI_FLAG_MODF: Mode fault flag
  *            @arg SPI_FLAG_OVR: Overrun flag
  *            @arg SPI_FLAG_BSY: Busy flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_SPI_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->SR) & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the SPI CRCERR pending flag.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_CLEAR_CRCERRFLAG(__HANDLE__) ((__HANDLE__)->Instance->SR = (uint16_t)(~SPI_FLAG_CRCERR))

/** @brief  Clear the SPI MODF pending flag.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_CLEAR_MODFFLAG(__HANDLE__)             \
  do{                                                    \
     uint32_t tmpreg_modf = 0x00U;                   \
    tmpreg_modf = (__HANDLE__)->Instance->SR;            \
    SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_SPE_MASK); \
    UNUSED(tmpreg_modf);                                 \
  } while(0U)

/** @brief  Clear the SPI OVR pending flag.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_CLEAR_OVRFLAG(__HANDLE__)        \
  do{                                              \
    uint32_t tmpreg_ovr = 0x00U;              \
    tmpreg_ovr = (__HANDLE__)->Instance->SR;       \
    tmpreg_ovr = (__HANDLE__)->Instance->DR;       \
    UNUSED(tmpreg_ovr);                            \
  } while(0U)

	/** @brief  Clear the SPI OVR pending flag.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */

/** @brief  Enable the SPI peripheral.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_SPE_MASK)

/** @brief  Disable the SPI peripheral.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_DISABLE(__HANDLE__) CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_SPE_MASK)

/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup SPI_Private_Constants SPI Private Constants
  * @{
  */
#define SPI_INVALID_CRC_ERROR     0U          /* CRC error wrongly detected */
#define SPI_VALID_CRC_ERROR       1U          /* CRC error is true */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup SPI_Private_Macros SPI Private Macros
  * @{
  */

/** @brief  Set the SPI transmit-only mode.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define SPI_1LINE_TX(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_BIDIOE_MASK)

/** @brief  Set the SPI receive-only mode.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define SPI_1LINE_RX(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_BIDIOE_MASK)

/** @brief  Reset the CRC calculation of the SPI.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define SPI_RESET_CRC(__HANDLE__) do{CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_CRCEN_MASK);\
                                       SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_CRCEN_MASK);}while(0U)

/** @brief  Check whether the specified SPI flag is set or not.
  * @param  __SR__  copy of SPI SR regsiter.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_FLAG_RXNE: Receive buffer not empty flag
  *            @arg SPI_FLAG_TXE: Transmit buffer empty flag
  *            @arg SPI_FLAG_CRCERR: CRC error flag
  *            @arg SPI_FLAG_MODF: Mode fault flag
  *            @arg SPI_FLAG_OVR: Overrun flag
  *            @arg SPI_FLAG_BSY: Busy flag
  * @retval SET or RESET.
  */
#define SPI_CHECK_FLAG(__SR__, __FLAG__)         ((((__SR__) & ((__FLAG__) & SPI_FLAG_MASK)) == ((__FLAG__) & SPI_FLAG_MASK)) ? SET : RESET)

/** @brief  Check whether the specified SPI Interrupt is set or not.
  * @param  __IER__  copy of SPI CR2 regsiter.
  * @param  __INTERRUPT__ specifies the SPI interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval SET or RESET.
  */
#define SPI_CHECK_IT_SOURCE(__IVS__, __INTERRUPT__)      ((((__IVS__) & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/** @brief  Checks if SPI Mode parameter is in allowed range.
  * @param  __MODE__ specifies the SPI Mode.
  *         This parameter can be a value of @ref SPI_Mode
  * @retval None
  */
#define IS_SPI_MODE(__MODE__) (((__MODE__) == SPI_MODE_SLAVE) || \
                               ((__MODE__) == SPI_MODE_MASTER))

/** @brief  Checks if SPI Direction Mode parameter is in allowed range.
  * @param  __MODE__ specifies the SPI Direction Mode.
  *         This parameter can be a value of @ref SPI_Direction
  * @retval None
  */
#define IS_SPI_DIRECTION(__MODE__) (((__MODE__) == SPI_DIRECTION_2LINES)        || \
                                    ((__MODE__) == SPI_DIRECTION_2LINES_RXONLY) || \
                                    ((__MODE__) == SPI_DIRECTION_1LINE))

/** @brief  Checks if SPI Direction Mode parameter is 2 lines.
  * @param  __MODE__ specifies the SPI Direction Mode.
  * @retval None
  */
#define IS_SPI_DIRECTION_2LINES(__MODE__) ((__MODE__) == SPI_DIRECTION_2LINES)

/** @brief  Checks if SPI Direction Mode parameter is 1 or 2 lines.
  * @param  __MODE__ specifies the SPI Direction Mode.
  * @retval None
  */
#define IS_SPI_DIRECTION_2LINES_OR_1LINE(__MODE__) (((__MODE__) == SPI_DIRECTION_2LINES) || \
                                                    ((__MODE__) == SPI_DIRECTION_1LINE))

/** @brief  Checks if SPI Data Size parameter is in allowed range.
  * @param  __DATASIZE__ specifies the SPI Data Size.
  *         This parameter can be a value of @ref SPI_Data_Size
  * @retval None
  */
#define IS_SPI_DATASIZE(__DATASIZE__) (((__DATASIZE__) == SPI_DATASIZE_16BIT) || \
                                       ((__DATASIZE__) == SPI_DATASIZE_8BIT))

/** @brief  Checks if SPI Serial clock steady state parameter is in allowed range.
  * @param  __CPOL__ specifies the SPI serial clock steady state.
  *         This parameter can be a value of @ref SPI_Clock_Polarity
  * @retval None
  */
#define IS_SPI_CPOL(__CPOL__) (((__CPOL__) == SPI_POLARITY_LOW) || \
                               ((__CPOL__) == SPI_POLARITY_HIGH))

/** @brief  Checks if SPI Clock Phase parameter is in allowed range.
  * @param  __CPHA__ specifies the SPI Clock Phase.
  *         This parameter can be a value of @ref SPI_Clock_Phase
  * @retval None
  */
#define IS_SPI_CPHA(__CPHA__) (((__CPHA__) == SPI_PHASE_1EDGE) || \
                               ((__CPHA__) == SPI_PHASE_2EDGE))

/** @brief  Checks if SPI Slave Select parameter is in allowed range.
  * @param  __NSS__ specifies the SPI Slave Select management parameter.
  *         This parameter can be a value of @ref SPI_Slave_Select_management
  * @retval None
  */
#define IS_SPI_NSS(__NSS__) (((__NSS__) == SPI_NSS_SOFT)       || \
                             ((__NSS__) == SPI_NSS_HARD_INPUT) || \
                             ((__NSS__) == SPI_NSS_HARD_OUTPUT))

/** @brief  Checks if SPI Baudrate prescaler parameter is in allowed range.
  * @param  __PRESCALER__ specifies the SPI Baudrate prescaler.
  *         This parameter can be a value of @ref SPI_BaudRate_Prescaler
  * @retval None
  */
#define IS_SPI_BAUDRATE_PRESCALER(__PRESCALER__) (((__PRESCALER__) == SPI_BAUDRATEPRESCALER_8)   || \
                                                  ((__PRESCALER__) == SPI_BAUDRATEPRESCALER_16)  || \
                                                  ((__PRESCALER__) == SPI_BAUDRATEPRESCALER_32)  || \
                                                  ((__PRESCALER__) == SPI_BAUDRATEPRESCALER_64)  || \
                                                  ((__PRESCALER__) == SPI_BAUDRATEPRESCALER_128) || \
                                                  ((__PRESCALER__) == SPI_BAUDRATEPRESCALER_256))

/** @brief  Checks if SPI MSB LSB transmission parameter is in allowed range.
  * @param  __BIT__ specifies the SPI MSB LSB transmission (whether data transfer starts from MSB or LSB bit).
  *         This parameter can be a value of @ref SPI_MSB_LSB_transmission
  * @retval None
  */
#define IS_SPI_FIRST_BIT(__BIT__) (((__BIT__) == SPI_FIRSTBIT_MSB) || \
                                   ((__BIT__) == SPI_FIRSTBIT_LSB))

/** @brief  Checks if SPI TI mode parameter is disabled.
  * @param  __MODE__ SPI_TIMODE_DISABLE. Device not support Ti Mode.
  *         This parameter can be a value of @ref SPI_TI_mode
  * @retval None
  */
#define IS_SPI_TIMODE(__MODE__) ((__MODE__) == SPI_TIMODE_DISABLE)

/** @brief  Checks if SPI CRC calculation enabled state is in allowed range.
  * @param  __CALCULATION__ specifies the SPI CRC calculation enable state.
  *         This parameter can be a value of @ref SPI_CRC_Calculation
  * @retval None
  */
#define IS_SPI_CRC_CALCULATION(__CALCULATION__) (((__CALCULATION__) == SPI_CRCCALCULATION_DISABLE) || \
                                                 ((__CALCULATION__) == SPI_CRCCALCULATION_ENABLE))

/** @brief  Checks if SPI polynomial value to be used for the CRC calculation, is in allowed range.
  * @param  __POLYNOMIAL__ specifies the SPI polynomial value to be used for the CRC calculation.
  *         This parameter must be a number between Min_Data = 0 and Max_Data = 65535
  * @retval None
  */
#define IS_SPI_CRC_POLYNOMIAL(__POLYNOMIAL__) (((__POLYNOMIAL__) >= 0x1U) && ((__POLYNOMIAL__) <= 0xFFFFU) && (((__POLYNOMIAL__)&0x1U) != 0U))

/** @brief  Checks if DMA handle is valid.
  * @param  __HANDLE__ specifies a DMA Handle.
  * @retval None
  */
#define IS_SPI_DMA_HANDLE(__HANDLE__) ((__HANDLE__) != NULL)

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup SPI_Private_Functions SPI Private Functions
  * @{
  */
uint8_t SPI_ISCRCErrorValid(SPI_HandleTypeDef *hspi);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SPI_Exported_Functions
  * @{
  */

/** @addtogroup SPI_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  ********************************/
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);


/** @addtogroup SPI_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions  ***************************************************/
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                             uint16_t Size);


HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi,void *Data,uint16_t Count);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi,void *Data,uint16_t Count);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi,void *TX_Data,void *RX_Data,uint16_t Count);

void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

void HAL_SPI_TxDMACpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxDMACpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxDMACpltCallback(SPI_HandleTypeDef *hspi);

/**
  * @}
  */

/** @addtogroup SPI_Exported_Functions_Group3
  * @{
  */
/* Peripheral State and Error functions ***************************************/
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi);
uint32_t             HAL_SPI_GetError(SPI_HandleTypeDef *hspi);
/**
  * @}
  */
#endif

/************************ (C) COPYRIGHT linkedsemi *************END OF FILE****/
