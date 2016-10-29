#ifndef __stm32f1xx_mcu_hpp__
#define __stm32f1xx_mcu_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/



//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

//////////////////////////////////////////////////////////////////////////
#include <_aux_list.hpp>

//////////////////////////////////////////////////////////////////////////
#include <objtypes.hpp>
#include <gpio.hpp>
#include <dma.hpp>
#include <clock.hpp>

//////////////////////////////////////////////////////////////////////////
#include <mcu_cfg.hpp>


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {

template <bool execute, class sysclock, class CUR>
struct list_modules_init_exec
{
	static bool _exec()
	{
		return false;
	}
};

template <class sysclock, class CUR>
struct list_modules_init_exec<true, sysclock, CUR>
{
	static bool _exec()
	{
		CUR::template init<sysclock>();
		return true;
	}
};

template <class sysclock>
struct list_modules_init
{
	template <class CUR>
	static bool _cb()
	{
		return list_modules_init_exec<
			(
				CUR ::obj::_type_id != obj::type_id::invalid &&
				CUR ::obj::_type_id != obj::type_id::dummy &&
				CUR ::obj::_type_id != obj::type_id::eol
			), sysclock, CUR
			>::_exec();
	}
};

//////////////////////////////////////////////////////////////////////////
template <bool execute, class sysclock, class CUR>
struct list_modules_sw_starting_exec
{
	static bool _exec()
	{
		return false;
	}
};

template <class sysclock, class CUR>
struct list_modules_sw_starting_exec<true, sysclock, CUR>
{
	static bool _exec()
	{
		CUR::template on_sysclock_changing<sysclock>::starting();
		return true;
	}
};

template <class sysclock>
struct list_modules_sw_starting
{
	template <class CUR>
	static bool _cb()
	{
		return list_modules_sw_starting_exec<
			(
				CUR ::obj::_type_id != obj::type_id::invalid &&
				CUR ::obj::_type_id != obj::type_id::dummy &&
				CUR ::obj::_type_id != obj::type_id::eol
			), sysclock, CUR
			>::_exec();
	}
};

//////////////////////////////////////////////////////////////////////////
template <bool execute, class sysclock, class CUR>
struct list_modules_sw_finished_exec
{
	static bool _exec()
	{
		return false;
	}
};

template <class sysclock, class CUR>
struct list_modules_sw_finished_exec<true, sysclock, CUR>
{
	static bool _exec()
	{
		CUR::template on_sysclock_changing<sysclock>::finished();
		return true;
	}
};

template <class sysclock>
struct list_modules_sw_finished
{
	template <class CUR>
	static bool _cb()
	{
		return list_modules_sw_finished_exec<
			(
				CUR ::obj::_type_id != obj::type_id::invalid &&
				CUR ::obj::_type_id != obj::type_id::dummy &&
				CUR ::obj::_type_id != obj::type_id::eol
			), sysclock, CUR
			>::_exec();
	}
};

//////////////////////////////////////////////////////////////////////////
template< typename sysclock, class modules >
class mcu_port
{
private:
	mcu_port();
	~mcu_port();

public:
	typedef sysclock	_sysclock;
	typedef modules		_modules;

public:
	static void init()
	{
		static const uint32_t _RCC_APB2ENR_AFIOEN_BB = FROM_ADDRESS_BIT_POS_TO_BB(&RCC->APB2ENR, RCC_APB2ENR_AFIOEN_Pos);
		SET_BB_REG(_RCC_APB2ENR_AFIOEN_BB);			// SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);
		
		// configure JTAG
		//MODIFY_REG(AFIO->MAPR, AFIO_MAPR_SWJ_CFG_Msk, AFIO_MAPR_SWJ_CFG_RESET      );		// Full SWJ (JTAG-DP + SW-DP) : Reset State
		//MODIFY_REG(AFIO->MAPR, AFIO_MAPR_SWJ_CFG_Msk, AFIO_MAPR_SWJ_CFG_NOJNTRST   );		// Full SWJ (JTAG-DP + SW-DP) but without JNTRST
		MODIFY_REG(AFIO->MAPR, AFIO_MAPR_SWJ_CFG_Msk, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);		// JTAG-DP Disabled and SW-DP Enabled
		//MODIFY_REG(AFIO->MAPR, AFIO_MAPR_SWJ_CFG_Msk, AFIO_MAPR_SWJ_CFG_DISABLE    );		// JTAG-DP Disabled and SW-DP Disabled 

		// enable debugging in Low Power mode
		SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP);

		// Allow termination of __WFE() by events (pending interrupts)
		SET_BIT(SCB->SCR, SCB_SCR_SEVONPEND);

		// Initilize system clock
		_sysclock::init();
		
		SystemCoreClock = _sysclock::_cfg_::_Clock_Hz;									// compatibility with Standard Periphery Library

		// Initialize list of modules
		bool init_done = _modules::template traverse_exec< list_modules_init<_sysclock> >();

//		_modules::template init<_sysclock>();
//		_modules::_next::template init<_sysclock>();
	}

	template< typename sysclock_new >
	static void switch_sysclock()
	{
		bool sw_start = _modules::template traverse_exec< list_modules_sw_starting<sysclock_new> >();

		// if clock souce is PLL, deactive it
		if(READ_BIT(RCC->CFGR, RCC_CFGR_SWS_Msk) == RCC_CFGR_SWS_PLL)
		{
			MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_HSI);					// Switching to Clock Source
			while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI) {}			// Waiting for finalization
		}

		// Initilize system clock
		sysclock_new::init();
		
		SystemCoreClock = sysclock_new::_cfg_::_Clock_Hz;								// compatibility with Standard Periphery Library

		bool sw_done  = _modules::template traverse_exec< list_modules_sw_finished<sysclock_new> >();
	}
};

} // namespace mcu


/************************************************************************/
/*                                                                      */
/************************************************************************/
#endif /*__stm32f1xx_mcu_hpp__*/
