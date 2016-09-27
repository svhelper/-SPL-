#ifndef __stm32f1xx_clock_hpp__
#define __stm32f1xx_clock_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

//////////////////////////////////////////////////////////////////////////
#include <stm32f1xx.h>


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {
namespace clock {

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace config {
	typedef enum
	{
		high_speed_internall,
		low_speed_internall,
		pll_high_speed_internall,

		high_speed_externall,
		low_speed_externall,
		pll_high_speed_externall,
	} clock_source;
} // namespace config

/************************************************************************/
/*                                                                      */
/************************************************************************/
template<config::clock_source ClockSource, uint32_t Clock_Hz,
		uint32_t PllPreDiv = 1, uint32_t PllMull = 1>
class clock_t
{
protected:
	class _const_
	{
		friend class clock_t;
		static const config::clock_source	_ClockSource	=	ClockSource;
		static const uint32_t				_Clock_Hz		=	//(_ClockSource == config::pll_high_speed_internall) ? 8000000UL :
																Clock_Hz;
		static const uint32_t				_PllPreDiv		=	(_ClockSource == config::pll_high_speed_internall) ? 2 :
																(_ClockSource == config::pll_high_speed_externall) ? PllPreDiv :
																1;
		static const uint32_t				_PllMull		=	(_ClockSource == config::pll_high_speed_internall) ? PllMull :
																(_ClockSource == config::pll_high_speed_externall) ? PllMull :
																1;

		STATIC_ASSERT(FAIL_IF(_Clock_Hz > 72000000UL), "Clock_Hz should not be more than 72MHz");
		
		STATIC_ASSERT(FAIL_IF(_ClockSource == config::high_speed_internall && _Clock_Hz != 8000000UL), "For HSI, Clock_Hz should be 8MHz");
		STATIC_ASSERT(FAIL_IF(_ClockSource == config::low_speed_internall  && _Clock_Hz !=   40000UL), "For LSI, Clock_Hz should be 40kHz");
		
		STATIC_ASSERT(FAIL_IF(_ClockSource == config::pll_high_speed_internall && _PllPreDiv != 2), "For PLL_HSI, PllPreDiv should be 2");
		STATIC_ASSERT(FAIL_IF(_ClockSource == config::pll_high_speed_externall && (_PllPreDiv < 1 || _PllPreDiv > 2)), "For PLL_HSE, PllPreDiv should be between 1 and 2");

		STATIC_ASSERT(FAIL_IF(_ClockSource == config::pll_high_speed_internall && (_PllMull < 2 || _PllMull > 16)), "For PLL_HSI, PllMull should be between 2 and 16");
		STATIC_ASSERT(FAIL_IF(_ClockSource == config::pll_high_speed_externall && (_PllMull < 2 || _PllMull > 16)), "For PLL_HSE, PllMull should be between 2 and 16");

		//STATIC_ASSERT(FAIL_IF(_ClockSource == config::pll_high_speed_externall && _Clock_Hz == 72000000UL && !(_PllPreDiv == 1) && !(_PllMull == 9)), "???");
	};
	
public:
	static const config::clock_source	_ClockSource	= _const_::_ClockSource;
	static const uint32_t				_Clock_Hz		= _const_::_Clock_Hz;
	static const uint32_t				_PllPreDiv		= _const_::_PllPreDiv;
	static const uint32_t				_PllMull		= _const_::_PllMull;
	
public:
	static void set_core_clock()
	{
	}
};

/************************************************************************/
/*                                                                      */
/************************************************************************/
template<uint32_t Clock_Hz> class hsi_t : public clock_t<config::high_speed_internall, Clock_Hz> {};
template<uint32_t Clock_Hz> class lsi_t : public clock_t<config::low_speed_internall,  Clock_Hz> {};
template<uint32_t Clock_Hz> class hse_t : public clock_t<config::high_speed_externall, Clock_Hz> {};
template<uint32_t Clock_Hz> class lse_t : public clock_t<config::low_speed_externall,  Clock_Hz> {};

/************************************************************************/
/*                                                                      */
/************************************************************************/
template<class Clock_T, uint32_t Clock_Hz = 72000000UL>
class pll_t
	: public clock_t<
		(	(Clock_T::_ClockSource == config::high_speed_internall) ? config::pll_high_speed_internall :
			(Clock_T::_ClockSource == config::high_speed_externall) ? config::pll_high_speed_externall :
			Clock_T::_ClockSource),
		Clock_Hz,
		(	(Clock_T::_ClockSource == config::high_speed_internall) ? 2 :
			(Clock_T::_ClockSource == config::high_speed_externall) ? ((Clock_Hz / Clock_T::_Clock_Hz) > 16 ? 2 : 1) : 
			0),
		// Division by zero during compilation?
		//  - Check ClockSource - it may be HSE or HSI only!
		(Clock_Hz / Clock_T::_Clock_Hz / (
			(Clock_T::_ClockSource == config::high_speed_internall) ? 2 :
			(Clock_T::_ClockSource == config::high_speed_externall) ? ((Clock_Hz / Clock_T::_Clock_Hz) > 16 ? 2 : 1) : 
			0))
		>
{
public:
	typedef Clock_T clock_t;

public:
	static void set_core_clock()
	{
	}
};

/************************************************************************/
/*                                                                      */
/************************************************************************/
typedef hsi_t<8000000UL> hsi_default;
typedef lsi_t<  40000UL> lsi_default;

typedef hse_t<8000000UL> hse_default;
typedef lse_t<  32768UL> lse_default;

typedef pll_t<hsi_default> pll_default;
typedef pll_t<hse_default> pll_hse_default;

/************************************************************************/
/*                                                                      */
/************************************************************************/


} // namespace clock
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__stm32f1xx_clock_hpp__*/
