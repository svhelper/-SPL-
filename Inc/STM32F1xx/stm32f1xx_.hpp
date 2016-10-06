#ifndef __stm32f1xx_hpp__
#define __stm32f1xx_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/


/************************************************************************/
/*                                                                      */
/************************************************************************/
//////////////////////////////////////////////////////////////////////////
#ifndef METALLL_HSE_FREQ_HZ
#	error "Please define METALLL_HSE_FREQ_HZ!"
#endif /* METALLL_HSE_FREQ_HZ */

#ifndef METALLL_LSE_FREQ_HZ
#	error "Please define METALLL_LSE_FREQ_HZ!"
#endif /* METALLL_LSE_FREQ_HZ */


/************************************************************************/
/*                                                                      */
/************************************************************************/
#ifndef HSI_VALUE
#	define HSI_VALUE				8000000UL  /* 8MHz */
#endif /* HSI_VALUE */

#ifndef LSI_VALUE
#	define LSI_VALUE				40000UL  /* 40kHz */
#endif /* LSI_VALUE */

#ifndef HSI_CALIBRATION_DEFAULT
#	define HSI_CALIBRATION_DEFAULT	0x10UL
#endif /*HSI_CALIBRATION_DEFAULT*/


/************************************************************************/
/*                                                                      */
/************************************************************************/
#ifndef NVIC_PRIORITYGROUP
#	define  NVIC_PRIORITYGROUP		NVIC_PRIORITYGROUP_4
#endif /*TICK_INT_PRIORITY*/

#ifndef TICK_INT_PRIORITY
#	define  TICK_INT_PRIORITY		0x00UL    /*!< tick interrupt priority (lowest by default)  */
#endif /*TICK_INT_PRIORITY*/


/************************************************************************/
/*                                                                      */
/************************************************************************/
//	----------##---+------+--+-----+-----+-----+-----+-----+------+----------+----------+
//	UART/USART##CMD|  ID  |AF|  CK |  TX |  RX | CTS | RTS |  DMA |  DmaChTX |  DmaChRX |
//	----------##---+------+--+-----+-----+-----+-----+-----+------+----------+----------+
#define DEFINE_UARTS(CMD)                                                                \
	DEF_USART_##CMD(uart_1, 0, PA8 , PA9 , PA10, PA11, PA12, dma_1, channel_4, channel_5)\
	DEF_USART_##CMD(uart_1, 1, PA8 , PB6 , PB7 , PA11, PA12, dma_1, channel_4, channel_5)\
	                                                                                     \
	DEF_USART_##CMD(uart_2, 0, PA4 , PA2 , PA3 , PA0 , PA1 , dma_1, channel_7, channel_6)\
	DEF_USART_##CMD(uart_2, 1, PD7 , PD5 , PD6 , PD3 , PD4 , dma_1, channel_7, channel_6)\
	                                                                                     \
	DEF_USART_##CMD(uart_3, 0, PB12, PB10, PB11, PB13, PB14, dma_1, channel_2, channel_3)\
	DEF_USART_##CMD(uart_3, 1, PC12, PC10, PC11, PB13, PB14, dma_1, channel_2, channel_3)\
	DEF_USART_##CMD(uart_3, 2, PD10, PD8 , PD9 , PD11, PD12, dma_1, channel_2, channel_3)\
//	----------##---+------+--+-----+-----+-----+-----+-----+------+----------+----+



/************************************************************************/
/*                                                                      */
/************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

//////////////////////////////////////////////////////////////////////////
#include "gpio.hpp"
#include "dma.hpp"
#include "uart.hpp"

//////////////////////////////////////////////////////////////////////////
//#include <stm32f1xx.h>
#include __METALLL_PORT(registers.hpp)
#include __METALLL_PORT(clock.hpp)
#include __METALLL_PORT(gpio.hpp)
#include __METALLL_PORT(uart.hpp)
//#include __METALLL_PORT(dma.hpp)

/************************************************************************/
/*                                                                      */
/************************************************************************/
#endif /*__stm32f1xx_hpp__*/
