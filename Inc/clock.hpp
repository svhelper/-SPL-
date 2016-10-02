#ifndef __clock_hpp__
#define __clock_hpp__

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

namespace clock {
	
	namespace state {
		typedef enum
		{
			disable,
			enable = !disable,
		} state;
	} // namespace state

	namespace osc_type {
		typedef enum
		{
			high_speed_internal,
			low_speed_internal,

			high_speed_external,
			high_speed_external_bypass,
			low_speed_external,
			low_speed_external_bypass,
		} osc_type;
	} // namespace osc_type
	
	static const uint32_t CALIBRATION_DEF	= 0x7FFFFFFF;
	static const uint32_t CLOCK_HZ_DEF		= 0xFFFFFFFF;
	
} // namespace clock


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace clock {

template < state::state state, int32_t calibration = ::mcu::clock::CALIBRATION_DEF, uint32_t Clock_Hz = CLOCK_HZ_DEF >	class osc_hsi;
template < state::state state, uint32_t Clock_Hz = CLOCK_HZ_DEF >														class osc_lsi;
template < state::state state, uint32_t Clock_Hz = CLOCK_HZ_DEF >														class osc_hse;
template < state::state state, uint32_t Clock_Hz = CLOCK_HZ_DEF >														class osc_hse_bypass;
template < state::state state, uint32_t Clock_Hz = CLOCK_HZ_DEF >														class osc_lse;
template < state::state state, uint32_t Clock_Hz = CLOCK_HZ_DEF >														class osc_lse_bypass;

template < state::state state >																							class pll_hsi;
template < state::state state >																							class pll_hse;
template < state::state state >																							class pll_hse_bypass;

//////////////////////////////////////////////////////////////////////////	
typedef osc_hsi			< state::enable > osc_hsi_def;
typedef osc_lsi			< state::enable > osc_lsi_def;
typedef osc_hse			< state::enable > osc_hse_def;
typedef osc_hse_bypass	< state::enable > osc_hse_bypass_def;
typedef osc_lse			< state::enable > osc_lse_def;
typedef osc_lse_bypass	< state::enable > osc_lse_bypass_def;

typedef pll_hsi			< state::enable > pll_hsi_def;
typedef pll_hse			< state::enable > pll_hse_def;
typedef pll_hse_bypass	< state::enable > pll_hse_bypass_def;

//------------------------------------------------------------------------
//template< class OSC, uint32_t Clock_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
//class pll_auto : public ::mcu::stm32::clock::pll_auto_range< OSC, Clock_Hz, Clock_Hz, true, true, state > {};

//template< class OSC, uint32_t ClockMin_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, uint32_t ClockMax_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
//class pll_min : public ::mcu::stm32::clock::pll_auto_range< OSC, ClockMin_Hz, ClockMax_Hz, false, false, state > {};

//template< class OSC, uint32_t ClockMin_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, uint32_t ClockMax_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
//class pll_max : public ::mcu::stm32::clock::pll_auto_range< OSC, ClockMin_Hz, ClockMax_Hz, true, false, state > {};

//template< class OSC, uint32_t ClockMin_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, uint32_t ClockMax_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
//class pll_usb_min : public ::mcu::stm32::clock::pll_auto_range< OSC, ClockMin_Hz, ClockMax_Hz, false, true, state > {};

//template< class OSC, uint32_t ClockMin_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, uint32_t ClockMax_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
//class pll_usb_max : public ::mcu::stm32::clock::pll_auto_range< OSC, ClockMin_Hz, ClockMax_Hz, true, true, state > {};


//////////////////////////////////////////////////////////////////////////	
template<
		class		ClockSource,
		uint32_t	CortexSysTimer_ms = 1000
	>
class sysclock_max;

//------------------------------------------------------------------------

typedef sysclock_max<osc_hsi_def       > sysclock_osc_hsi_def;
typedef sysclock_max<osc_hse_def       > sysclock_osc_hse_def;
typedef sysclock_max<pll_hsi_def       > sysclock_pll_hsi_def;
typedef sysclock_max<pll_hse_def       > sysclock_pll_hse_def;
typedef sysclock_max<pll_hse_bypass_def> sysclock_pll_hse_bypass_def;

} // namespace clock

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__clock_hpp__*/
