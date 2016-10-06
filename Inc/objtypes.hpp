#ifndef __objtypes_hpp__
#define __objtypes_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {

//////////////////////////////////////////////////////////////////////////
namespace objtype {
typedef enum
{
	// service/defaults
	invalid,
	
	// system
	mcu,
	sysclock,
	osc,
	pll,
	
	// periphery low-level
	gpio,
	adc,
	dac,
	dma,
	tim,
	rtc,
	wd,
	fsmc,
	sdio,
	usb,
	can,
	spi,
	sqi,
	i2s,
	i2c,
	uart,
	ethernet,

	// periphery middle-level
	clock,
	rtc_clock,
	rtc_alarm,
	mcu_voltage,
	mcu_temperature,
	adc_calibration,
	spi_target,
	i2c_target,
	lcd_cmd,
	lcd_data,
	btn_arr,
	sdcard,
	flash,
	pwm,
	usb_dev,
	usb_host,
	usb_otg,
	can_open,
	usbh_msc,

	// periphery high-level
	lcd,
	keypad,
	led,
	fat_fs,
} objtype;
} // namespace objtype

//////////////////////////////////////////////////////////////////////////
namespace obj {
template<objtype::objtype objtype_id = objtype::invalid, uint32_t obj_id = 0>
class obj
{
public:
	static const objtype::objtype	_objtype_id		= objtype_id	;
	static const uint32_t			_obj_id			= obj_id		;
};
} // namespace obj

//////////////////////////////////////////////////////////////////////////
namespace dummy {
	
	class obj : public ::mcu::obj::obj< ::mcu::objtype::invalid > {};

} // namespace dummy


/************************************************************************/
/*                                                                      */
/************************************************************************/
#define __VAR_ARGS10__(CMD, CDM, CDL, d, ...) \
	__VAR_ARGS_##CMD(d##0, __VA_ARGS__) __VAR_ARGS_##CDM() __VAR_ARGS_##CMD(d##1, __VA_ARGS__) __VAR_ARGS_##CDM() \
	__VAR_ARGS_##CMD(d##2, __VA_ARGS__) __VAR_ARGS_##CDM() __VAR_ARGS_##CMD(d##3, __VA_ARGS__) __VAR_ARGS_##CDM() \
	__VAR_ARGS_##CMD(d##4, __VA_ARGS__) __VAR_ARGS_##CDM() __VAR_ARGS_##CMD(d##5, __VA_ARGS__) __VAR_ARGS_##CDM() \
	__VAR_ARGS_##CMD(d##6, __VA_ARGS__) __VAR_ARGS_##CDM() __VAR_ARGS_##CMD(d##7, __VA_ARGS__) __VAR_ARGS_##CDM() \
	__VAR_ARGS_##CMD(d##8, __VA_ARGS__) __VAR_ARGS_##CDM() __VAR_ARGS_##CMD(d##9, __VA_ARGS__) __VAR_ARGS_##CDL() \

#define __VAR_ARGS_DIV__(CMD, CDM, CDL, ...) \
	__VAR_ARGS10__(CMD, CDM, CDM, 0, __VA_ARGS__) __VAR_ARGS10__(CMD, CDM, CDM, 1, __VA_ARGS__) \
	__VAR_ARGS10__(CMD, CDM, CDM, 2, __VA_ARGS__) __VAR_ARGS10__(CMD, CDM, CDM, 3, __VA_ARGS__) \
	__VAR_ARGS10__(CMD, CDM, CDM, 4, __VA_ARGS__) __VAR_ARGS10__(CMD, CDM, CDL, 5, __VA_ARGS__) \

#define __VAR_ARGS_COMMA__(CMD, ...)		__VAR_ARGS_DIV__(CMD, COMMA, SPACE, __VA_ARGS__)
#define __VAR_ARGS_SEMICOLON__(CMD, ...)	__VAR_ARGS_DIV__(CMD, SEMICOLON, SEMICOLON, __VA_ARGS__)
#define __VAR_ARGS_OR__(CMD, ...)			__VAR_ARGS_DIV__(CMD, OR, SPACE, __VA_ARGS__)
#define __VAR_ARGS_AND__(CMD, ...)			__VAR_ARGS_DIV__(CMD, AND, SPACE, __VA_ARGS__)
#define __VAR_ARGS_SPACE__(CMD, ...)		__VAR_ARGS_DIV__(CMD, SPACE, SPACE, __VA_ARGS__)

#define __VAR_ARGS__(CMD, ...)				__VAR_ARGS_SPACE__(CMD, __VA_ARGS__)

//------------------------------------------------------------------------
#define __VAR_ARGS_COMMA()				,
#define __VAR_ARGS_SEMICOLON()			;
#define __VAR_ARGS_OR()					|
#define __VAR_ARGS_AND()				&
#define __VAR_ARGS_SPACE()				

//------------------------------------------------------------------------
#define __VAR_ARGS_DEF(ind, ...)		typename p##ind __VA_ARGS__
#define _VAR_ARGS_DEF(...)				__VAR_ARGS_COMMA__(DEF, __VA_ARGS__)

//------------------------------------------------------------------------
#define __VAR_ARGS_LIST(ind)			p##ind,
#define _VAR_ARGS_LIST()				__VAR_ARGS_COMMA__(LIST)

//////////////////////////////////////////////////////////////////////////

} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__objtypes_hpp__*/
