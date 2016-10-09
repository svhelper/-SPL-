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
namespace obj {namespace type_id {
typedef enum
{
	// service/defaults
	invalid,		// used as return value
	dummy,			// used as default value
	eol,			// used as End Of List mark
	
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
} type_id;
}} // namespace obj::type_id

//////////////////////////////////////////////////////////////////////////
namespace obj {
template<obj::type_id::type_id objtype_id = obj::type_id::invalid, uint32_t obj_id = 0>
class obj
{
public:
	static const type_id::type_id	_type_id		= objtype_id	;
	static const uint32_t			_id				= obj_id		;
};
} // namespace obj

//////////////////////////////////////////////////////////////////////////
namespace eol {
	
	class obj : public ::mcu::obj::obj< ::mcu::obj::type_id::eol > {};

} // namespace dummy

namespace invalid {
	
	class obj : public ::mcu::obj::obj< ::mcu::obj::type_id::invalid > {};

} // namespace dummy

namespace dummy {
	
	class obj : public ::mcu::obj::obj< ::mcu::obj::type_id::dummy > {};

	typedef ::mcu::eol::obj		eol;
	typedef ::mcu::invalid::obj	invalid;

} // namespace dummy

//////////////////////////////////////////////////////////////////////////
namespace obj {
struct eq
{
	template <class _1, class _2> struct check
	{
		static const bool _result =	(_1::obj::_type_id	== _2::obj::_type_id	)
								&&	(_1::obj::_id		== _2::obj::_id			);
	};
};
} // namespace obj


/************************************************************************/
/*                                                                      */
/************************************************************************/
#define __VAR_ARGS10__(CMDF, CMDM, CMDL,  d, ...) \
	__VAR_ARGS_##CMDF(d##0, __VA_ARGS__) __VAR_ARGS_##CMDM(d##1, __VA_ARGS__) \
	__VAR_ARGS_##CMDM(d##2, __VA_ARGS__) __VAR_ARGS_##CMDM(d##3, __VA_ARGS__) \
	__VAR_ARGS_##CMDM(d##4, __VA_ARGS__) __VAR_ARGS_##CMDM(d##5, __VA_ARGS__) \
	__VAR_ARGS_##CMDM(d##6, __VA_ARGS__) __VAR_ARGS_##CMDM(d##7, __VA_ARGS__) \
	__VAR_ARGS_##CMDM(d##8, __VA_ARGS__) __VAR_ARGS_##CMDL(d##9, __VA_ARGS__) \

#define __VAR_ARGS_DIV__(CMDF, CMDM, CMDL,  ...) \
	__VAR_ARGS10__(CMDF, CMDM, CMDM, 0, __VA_ARGS__) __VAR_ARGS10__(CMDM, CMDM, CMDM, 1, __VA_ARGS__) \
	__VAR_ARGS10__(CMDM, CMDM, CMDM, 2, __VA_ARGS__) __VAR_ARGS10__(CMDM, CMDM, CMDL, 3, __VA_ARGS__) \

//#define __VAR_ARGS_DIV__(CMDF, CMDM, CMDL,  ...) \
//	__VAR_ARGS10__(CMDF, CMDM, CMDM, 0, __VA_ARGS__) __VAR_ARGS10__(CMDM, CMDM, CMDM, 1, __VA_ARGS__) \
//	__VAR_ARGS10__(CMDM, CMDM, CMDM, 2, __VA_ARGS__) __VAR_ARGS10__(CMDM, CMDM, CMDM, 3, __VA_ARGS__) \
//	__VAR_ARGS10__(CMDM, CMDM, CMDM, 4, __VA_ARGS__) __VAR_ARGS10__(CMDM, CMDM, CMDL, 5, __VA_ARGS__) \

//////////////////////////////////////////////////////////////////////////
#define __VAR_ARGS__(CMD, ...)			__VAR_ARGS_DIV__(CMD, CMD, CMD, __VA_ARGS__)

//------------------------------------------------------------------------
#define __VAR_ARGS_SKIP(ind, ...)		

//////////////////////////////////////////////////////////////////////////
#define __VAR_ARGS_DEF_(ind, ...)		typename p##ind __VA_ARGS__,
#define __VAR_ARGS_DEFE(ind, ...)		typename p##ind __VA_ARGS__
#define _VAR_ARGS_DEF(...)				__VAR_ARGS_DIV__(DEF_, DEF_, DEFE, __VA_ARGS__)

#define _VAR_ARGS_DEF_SHIFT()			__VAR_ARGS_DIV__(SKIP, DEF_, DEFE)
#define _VAR_ARGS_DEF_POP()				__VAR_ARGS_DIV__(SKIP, DEF_, DEFE)

//------------------------------------------------------------------------
#define __VAR_ARGS_LISTS(ind, ...)		p##ind,
#define __VAR_ARGS_LIST_(ind, ...)		p##ind,
#define __VAR_ARGS_LISTE(ind, ...)		p##ind
#define _VAR_ARGS_LIST()				__VAR_ARGS_DIV__(LISTS, LIST_, LISTE)

#define _VAR_ARGS_LIST_SHIFT()			__VAR_ARGS_DIV__(SKIP, LIST_, LISTE)
#define _VAR_ARGS_LIST_POP()			__VAR_ARGS_DIV__(SKIP, LIST_, LISTE)

//////////////////////////////////////////////////////////////////////////

} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__objtypes_hpp__*/
