#ifndef __mcu_cfg_hpp__
#define __mcu_cfg_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

/************************************************************************/
/* Check configuration                                                  */
/************************************************************************/
#ifndef METALLL_MCU
# error "Please define MCU configuration!"

/*
	Please define MCU configuration using exact or partial definition!

 example of exact definition:
   #define METALLL_MCU						STM32F103TB

 example of partial definition:
   #define METALLL_MCU						STM32F103XX
   
 or in command line:
   -DMETALLL_MCU=STM32F103TB
 */

#endif /*METALLL_MCU*/

/************************************************************************/
/* Declaration of MCU with their generic characteristics                */
/************************************************************************/

/*** STM32F103XX  *******************************************************/
//                              	         ##   +---------+---------+--------+---+----+---+---+----+---+---+----+---+---+----+----+
//                              	    _MCU_##CMD(  type   |  flash  |  ram   |tim|atim|spi|i2c|uart|usb|can|gpio|adc|ach|freq|pins)
//                              	         ##   +---------+---------+--------+---+----+---+---+----+---+---+----+---+---+----+----+
#define STM32F103T8(CMD)				_MCU_##CMD(STM32F1XX,  64*1024, 20*1024,  3,   1,  1,  1,   2,  1,  1,  26,  2, 10,  72,  36)
#define STM32F103TB(CMD)				_MCU_##CMD(STM32F1XX, 128*1024, 20*1024,  3,   1,  1,  1,   2,  1,  1,  26,  2, 10,  72,  36)
#define STM32F103C8(CMD)				_MCU_##CMD(STM32F1XX,  64*1024, 20*1024,  3,   1,  2,  2,   3,  1,  1,  37,  2, 10,  72,  48)
#define STM32F103CB(CMD)				_MCU_##CMD(STM32F1XX, 128*1024, 20*1024,  3,   1,  2,  2,   3,  1,  1,  37,  2, 10,  72,  48)
#define STM32F103R8(CMD)				_MCU_##CMD(STM32F1XX,  64*1024, 20*1024,  3,   1,  2,  2,   3,  1,  1,  51,  2, 16,  72,  64)
#define STM32F103RB(CMD)				_MCU_##CMD(STM32F1XX, 128*1024, 20*1024,  3,   1,  2,  2,   3,  1,  1,  51,  2, 16,  72,  64)
#define STM32F103V8(CMD)				_MCU_##CMD(STM32F1XX,  64*1024, 20*1024,  3,   1,  2,  2,   3,  1,  1,  80,  2, 16,  72, 100)
#define STM32F103VB(CMD)				_MCU_##CMD(STM32F1XX, 128*1024, 20*1024,  3,   1,  2,  2,   3,  1,  1,  80,  2, 16,  72, 100)
//                              	         ##   +---------+---------+--------+---+----+---+---+----+---+---+----+---+---+----+----+

// Forward definition of partial definitions to their maximal confiruration
#define STM32F103TX						STM32F103TB
#define STM32F103CX						STM32F103CB
#define STM32F103RX						STM32F103RB
#define STM32F103VX						STM32F103VB

#define STM32F103X8						STM32F103V8
#define STM32F103XB						STM32F103VB
#define STM32F103XX						STM32F103VB

/************************************************************************/
/* Resolve MCU configuration                                            */
/************************************************************************/
#define _MCU_TYPE(type, flash, ram, tim, atim, spi, i2c, uart, usb, can, gpio, adc, ach, freq, pins) 		type
#define _MCU_TYPESTR(type, flash, ram, tim, atim, spi, i2c, uart, usb, can, gpio, adc, ach, freq, pins) 	__2STR(type)
#define _MCU_FLASH(type, flash, ram, tim, atim, spi, i2c, uart, usb, can, gpio, adc, ach, freq, pins)		(flash)
#define _MCU_RAM(type, flash, ram, tim, atim, spi, i2c, uart, usb, can, gpio, adc, ach, freq, pins)			(ram)
#define _MCU_FREQ(type, flash, ram, tim, atim, spi, i2c, uart, usb, can, gpio, adc, ach, freq, pins)		((freq) * 1000000)
#define _MCU_PINS(type, flash, ram, tim, atim, spi, i2c, uart, usb, can, gpio, adc, ach, freq, pins)		(pins)

//------------------------------------------------------------------------
#define __MCU_TYPE_STR					METALLL_MCU(TYPESTR)		// string MCU type
#define __MCU_MODEL_STR					__2STR(METALLL_MCU)			// string MCU model

#define __MCU_TYPE						METALLL_MCU(TYPE)			// MCU type
#define __MCU_MODEL						METALLL_MCU					// MCU model
#define __MCU_FLASH						METALLL_MCU(FLASH)			// FLASH size
#define __MCU_RAM						METALLL_MCU(RAM)			// RAM size
#define __MCU_MAX_FREQUENCY_HZ			METALLL_MCU(FREQ)			// Maximal MCU clock frequency in Hz
#define __MCU_PACKAGE_PINS				METALLL_MCU(PINS)			// Number of pins in the MCU package


/************************************************************************/
/* Resolve MetaLLL configuration                                        */
/************************************************************************/
#define _MCU_MCUTYPE(type, flash, ram, tim, atim, spi, i2c, uart, usb, can, gpio, adc, ach, freq, pins)		type
#define METALLL_MCUTYPE					METALLL_MCU(MCUTYPE)

#define __2STR2(s)						#s
#define __2STR(s)						__2STR2(s)
#define __CONCAT5STR2(a, b, c, d, e)	__2STR(a ## b ## c ## d ## e)
#define __CONCAT5STR(a, b, c, d, e)		__CONCAT5STR2(a, b, c, d, e)
#define __METALLL_PORT(file)			__CONCAT5STR(METALLL_MCUTYPE, /, METALLL_MCUTYPE, _, file)

// Include the main header for selected MCU
#include __METALLL_PORT(.hpp)


/************************************************************************/
/*                                                                      */
/************************************************************************/
#endif /*__mcu_cfg_hpp__*/
