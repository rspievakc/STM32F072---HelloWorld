#ifndef INCLUDED_RINGBUF_H
#define INCLUDED_RINGBUF_H

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_uart.h"
#include "stm32f0xx_hal_dma.h"

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define OW_CMD_RSCRATCHPAD          0xBE        /*!< Read scratchpad command for 1-Wire devices */
#define OW_CMD_WSCRATCHPAD          0x4E        /*!< Write scratchpad command for 1-Wire devices */
#define OW_CMD_CPYSCRATCHPAD        0x48        /*!< Copy scratchpad command for 1-Wire devices */
#define OW_CMD_RECEEPROM            0xB8
#define OW_CMD_RPWRSUPPLY           0xB4
#define OW_CMD_SEARCHROM            0xF0        /*!< Search ROM command */
#define OW_CMD_READROM              0x33        /*!< Read ROM command */
#define OW_CMD_MATCHROM             0x55        /*!< Match ROM command. Select device with specific ROM */
#define OW_CMD_SKIPROM              0xCC        /*!< Skip ROM, select all devices */

// method declarations
int  OW_First(UART_HandleTypeDef* USARTx);
int  OW_Next(UART_HandleTypeDef* USARTx);
int  OW_Verify(UART_HandleTypeDef* USARTx);
void OW_TargetSetup(UART_HandleTypeDef* USARTx, unsigned char family_code);
void OW_FamilySkipSetup(UART_HandleTypeDef* USARTx);
int  OW_Reset(UART_HandleTypeDef* USARTx);
void OW_WriteByte(UART_HandleTypeDef* USARTx, unsigned char byte_value);
void OW_WriteBit(UART_HandleTypeDef* USARTx, unsigned char bit_value);
unsigned char OW_ReadBit(UART_HandleTypeDef* USARTx);
int  OW_Search(UART_HandleTypeDef* USARTx);
unsigned char OW_crc8(unsigned char value);

#endif
