#ifndef __stm32f1xx_clock_hpp__
#define __stm32f1xx_clock_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

#include <clock.hpp>

//////////////////////////////////////////////////////////////////////////
#include "CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {
namespace stm32 {
namespace clock {

	//////////////////////////////////////////////////////////////////////////
	namespace limits {
		static const uint32_t USB_PPM	= 2500;

		class max
		{
		public:
			static const uint32_t SYSCLK	= __MCU_MAX_FREQUENCY_HZ;

			static const uint32_t HSECLK	= 25000000UL;
			static const uint32_t LSECLK	= 1000000UL;
		
			static const uint32_t HCLK		= SYSCLK;
			static const uint32_t USBCLK	= (uint32_t)(48000000UL * (1.E6 + USB_PPM) / 1.E6);
			static const uint32_t APB1CLK	= 36000000UL;
			static const uint32_t APB2CLK	= SYSCLK;
			static const uint32_t ADCCLK	= 14000000UL;

			static const uint32_t FLASHCLK_LATENCY_0_Hz	= 24000000UL;			// Zero wait state
			static const uint32_t FLASHCLK_LATENCY_1_Hz	= 48000000UL;			// One wait state
			static const uint32_t FLASHCLK_LATENCY_2_Hz	= 72000000UL;			// Two wait states
		};

		class min
		{
		public:
			static const uint32_t SYSCLK	= 1UL;

			static const uint32_t HSECLK	= 1000000UL;
			static const uint32_t LSECLK	= 1UL;
		
			static const uint32_t HCLK		= 1UL;
			static const uint32_t USBCLK	= (uint32_t)(48000000UL * (1.E6 - USB_PPM) / 1.E6);
			static const uint32_t APB1CLK	= 1UL;	// 10000000UL;
			static const uint32_t APB2CLK	= 1UL;
			static const uint32_t ADCCLK	= 1UL;
		};
	} // namespace limits
	
	//////////////////////////////////////////////////////////////////////////
	namespace config {
		
		template < ::mcu::clock::osc_type::osc_type type, uint32_t Clock_Hz, ::mcu::clock::state::state state, int32_t calibration = ::mcu::clock::CALIBRATION_DEF>
		class osc_cfg;

		//------------------------------------------------------------------------
		template <uint32_t Clock_Hz, ::mcu::clock::state::state state, int32_t calibration>
		class osc_cfg< ::mcu::clock::osc_type::high_speed_internal, Clock_Hz, state, calibration>
		{
		private:
			static const int32_t calibration_val = (calibration == ::mcu::clock::CALIBRATION_DEF) ? HSI_CALIBRATION_DEFAULT : calibration;
		
			STATIC_ASSERT(Clock_Hz == HSI_VALUE, "HSI clock is invalid");
			STATIC_ASSERT(calibration_val >= 0x00 && calibration_val <= 0x1F, "HSI calibration value out of range");
		
			static const uint32_t _RCC_CR_HSION_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSION_Pos);
			static const uint32_t _RCC_CR_HSIRDY_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSIRDY_Pos);

		public:
			static const ::mcu::clock::osc_type::osc_type	_OscType		= ::mcu::clock::osc_type::high_speed_internal;
			static const uint32_t							_Clock_Hz		= Clock_Hz;
			static const ::mcu::clock::state::state			_State			= state;
			static const int32_t							_Calibration	= calibration_val;
		
			static const uint32_t							_pll_prediv		= 1;
			static const bool								_usb_div_1_5	= false;
			static const uint32_t							_pll_mul		= 1;
			static const uint32_t							_usb_clock		= 0;
			static const bool								_usb_active		= false;
		
			static void init()
			{
				(_State == ::mcu::clock::state::enable) ? start() : stop();
				
				/* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
				MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, (uint32_t)_Calibration << RCC_CR_HSITRIM_Pos);
			}
			
			static void enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? start() : stop();
			}
			
			static void start()
			{
				// enabling
				SET_BB_REG(_RCC_CR_HSION_BB);

				/* Wait till HSI is ready */
				while(IS_BB_REG_RESET(_RCC_CR_HSIRDY_BB)) {}
			}
			
			static void stop()
			{
				// disabling
				RESET_BB_REG(_RCC_CR_HSION_BB);

				/* Wait till HSI is ready */
				while(IS_BB_REG_SET(_RCC_CR_HSIRDY_BB)) {}
			}
		};

		//------------------------------------------------------------------------
		template <uint32_t Clock_Hz, ::mcu::clock::state::state state, int32_t calibration>
		class osc_cfg< ::mcu::clock::osc_type::low_speed_internal, Clock_Hz, state, calibration >
		{
		private:
			STATIC_ASSERT(Clock_Hz == LSI_VALUE, "LSI clock is invalid");
			STATIC_ASSERT(calibration == ::mcu::clock::CALIBRATION_DEF, "LSI has not calibration");
		
			static const uint32_t _RCC_CSR_LSION_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CSR, RCC_CSR_LSION_Pos);
			static const uint32_t _RCC_CSR_LSIRDY_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CSR, RCC_CSR_LSIRDY_Pos);

		public:
			static const ::mcu::clock::osc_type::osc_type	_OscType		= ::mcu::clock::osc_type::low_speed_internal;
			static const uint32_t							_Clock_Hz		= Clock_Hz;
			static const ::mcu::clock::state::state			_State			= state;
			static const int32_t							_Calibration	= calibration;
		
			static const uint32_t							_pll_prediv		= 1;
			static const bool								_usb_div_1_5	= false;
			static const uint32_t							_pll_mul		= 1;
			static const uint32_t							_usb_clock		= 0;
			static const bool								_usb_active		= false;

			static void init()
			{
				(_State == ::mcu::clock::state::enable) ? start() : stop();
			}
			
			static void enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? start() : stop();
			}
			
			static void start()
			{
				// enabling
				SET_BB_REG(_RCC_CSR_LSION_BB);

				/* Wait till LSI is ready */
				while(IS_BB_REG_RESET(_RCC_CSR_LSIRDY_BB)) {}

				/*  To have a fully stabilized clock in the specified range, a software delay of 1ms should be added.*/
				//HAL_Delay(1);
			}
			
			static void stop()
			{
				// disabling
				RESET_BB_REG(_RCC_CSR_LSION_BB);

				/* Wait till LSI is ready */
				while(IS_BB_REG_SET(_RCC_CSR_LSIRDY_BB)) {}
			}
		};

		//------------------------------------------------------------------------
		template <uint32_t Clock_Hz, ::mcu::clock::state::state state, int32_t calibration>
		class osc_cfg< ::mcu::clock::osc_type::high_speed_external, Clock_Hz, state, calibration>
		{
		private:
			STATIC_ASSERT(Clock_Hz >= mcu::stm32::clock::limits::min::HSECLK && Clock_Hz <= mcu::stm32::clock::limits::max::HSECLK, "HSE clock out of range");
			STATIC_ASSERT(calibration == ::mcu::clock::CALIBRATION_DEF, "HSE has not calibration");
		
			static const uint32_t _RCC_CR_HSEON_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSEON_Pos);
			static const uint32_t _RCC_CR_HSEBYP_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSEBYP_Pos);
			static const uint32_t _RCC_CR_HSERDY_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSERDY_Pos);

		public:
			static const ::mcu::clock::osc_type::osc_type	_OscType		= ::mcu::clock::osc_type::high_speed_external;
			static const uint32_t							_Clock_Hz		= Clock_Hz;
			static const ::mcu::clock::state::state			_State			= state;
			static const int32_t							_Calibration	= calibration;

			static const uint32_t							_pll_prediv		= 1;
			static const bool								_usb_div_1_5	= false;
			static const uint32_t							_pll_mul		= 1;
			static const uint32_t							_usb_clock		= 0;
			static const bool								_usb_active		= false;
		
			static void init()
			{
				(_State == ::mcu::clock::state::enable) ? start() : stop();
			}
			
			static void enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? start() : stop();
			}
			
			static void start()
			{
				// enabling
				RESET_BB_REG(_RCC_CR_HSEON_BB);
				__NOP();
				RESET_BB_REG(_RCC_CR_HSEBYP_BB);
				SET_BB_REG(_RCC_CR_HSEON_BB);

				/* Wait till HSE is ready */
				while(IS_BB_REG_RESET(_RCC_CR_HSERDY_BB)) {}
			}
			
			static void stop()
			{
				// disabling
				RESET_BB_REG(_RCC_CR_HSEON_BB);

				/* Wait till HSI is ready */
				while(IS_BB_REG_SET(_RCC_CR_HSERDY_BB)) {}
				
				RESET_BB_REG(_RCC_CR_HSEBYP_BB);
			}
		};

		//------------------------------------------------------------------------
		template <uint32_t Clock_Hz, ::mcu::clock::state::state state, int32_t calibration>
		class osc_cfg< ::mcu::clock::osc_type::high_speed_external_bypass, Clock_Hz, state, calibration>
		{
		private:
			STATIC_ASSERT(Clock_Hz >= mcu::stm32::clock::limits::min::HSECLK && Clock_Hz <= mcu::stm32::clock::limits::max::HSECLK, "HSE clock out of range");
			STATIC_ASSERT(calibration == ::mcu::clock::CALIBRATION_DEF, "HSE has not calibration");
		
			static const uint32_t _RCC_CR_HSEON_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSEON_Pos);
			static const uint32_t _RCC_CR_HSEBYP_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSEBYP_Pos);
			static const uint32_t _RCC_CR_HSERDY_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSERDY_Pos);

		public:
			static const ::mcu::clock::osc_type::osc_type	_OscType		= ::mcu::clock::osc_type::high_speed_external_bypass;
			static const uint32_t							_Clock_Hz		= Clock_Hz;
			static const ::mcu::clock::state::state			_State			= state;
			static const int32_t							_Calibration	= calibration;

			static const uint32_t							_pll_prediv		= 1;
			static const bool								_usb_div_1_5	= false;
			static const uint32_t							_pll_mul		= 1;
			static const uint32_t							_usb_clock		= 0;
			static const bool								_usb_active		= false;
		
			static void init()
			{
				(_State == ::mcu::clock::state::enable) ? start() : stop();
			}
			
			static void enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? start() : stop();
			}
			
			static void start()
			{
				// enabling
				RESET_BB_REG(_RCC_CR_HSEON_BB);
				__NOP();
				SET_BB_REG(_RCC_CR_HSEBYP_BB);
				SET_BB_REG(_RCC_CR_HSEON_BB);

				/* Wait till HSE is ready */
				while(IS_BB_REG_RESET(_RCC_CR_HSERDY_BB)) {}
			}
			
			static void stop()
			{
				// disabling
				RESET_BB_REG(_RCC_CR_HSEON_BB);

				/* Wait till HSI is ready */
				while(IS_BB_REG_SET(_RCC_CR_HSERDY_BB)) {}
				
				RESET_BB_REG(_RCC_CR_HSEBYP_BB);
			}
		};

		//------------------------------------------------------------------------
		template <uint32_t Clock_Hz, ::mcu::clock::state::state state, int32_t calibration>
		class osc_cfg< ::mcu::clock::osc_type::low_speed_external, Clock_Hz, state, calibration>
		{
		private:
			STATIC_ASSERT(Clock_Hz >= mcu::stm32::clock::limits::min::LSECLK && Clock_Hz <= mcu::stm32::clock::limits::max::LSECLK, "LSE clock out of range");
			STATIC_ASSERT(calibration == ::mcu::clock::CALIBRATION_DEF, "LSE has not calibration");
		
			static const uint32_t _RCC_APB1ENR_PWREN_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->APB1ENR, RCC_APB1ENR_PWREN_Pos);
			static const uint32_t _PWR_CR_DBP_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&PWR->CR, PWR_CR_DBP_Pos);
			
			static const uint32_t _RCC_BDCR_LSEON_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->BDCR, RCC_BDCR_LSEON_Pos);
			static const uint32_t _RCC_BDCR_LSEBYP_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->BDCR, RCC_BDCR_LSEBYP_Pos);
			static const uint32_t _RCC_BDCR_LSERDY_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->BDCR, RCC_BDCR_LSERDY_Pos);

		public:
			static const ::mcu::clock::osc_type::osc_type	_OscType		= ::mcu::clock::osc_type::low_speed_external;
			static const uint32_t							_Clock_Hz		= Clock_Hz;
			static const ::mcu::clock::state::state			_State			= state;
			static const int32_t							_Calibration	= calibration;

			static const uint32_t							_pll_prediv		= 1;
			static const bool								_usb_div_1_5	= false;
			static const uint32_t							_pll_mul		= 1;
			static const uint32_t							_usb_clock		= 0;
			static const bool								_usb_active		= false;
		
			static void init()
			{
				(_State == ::mcu::clock::state::enable) ? start() : stop();
			}
			
			static void enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? start() : stop();
			}
			
			static void start()
			{
				// enabling
				/* Enable Power Clock*/
				SET_BB_REG(_RCC_APB1ENR_PWREN_BB);
				__NOP();

				  /* Enable write access to Backup domain */
				  SET_BB_REG(_PWR_CR_DBP_BB);

				while(IS_BB_REG_RESET(_PWR_CR_DBP_BB)) {}

				/* Set the new LSE configuration */
				RESET_BB_REG(_RCC_BDCR_LSEON_BB);
				__NOP();
				RESET_BB_REG(_RCC_BDCR_LSEBYP_BB);
				SET_BB_REG(_RCC_BDCR_LSEON_BB);

				/* Wait till LSE is ready */  
				while(IS_BB_REG_RESET(_RCC_BDCR_LSERDY_BB)) {}
			}
			
			static void stop()
			{
				// disabling
				/* Set the new LSE configuration */
				RESET_BB_REG(_RCC_BDCR_LSEON_BB);

				/* Wait till LSE is ready */  
				while(IS_BB_REG_SET(_RCC_BDCR_LSERDY_BB)) {}

				RESET_BB_REG(_RCC_BDCR_LSEBYP_BB);
			}
		};

		//------------------------------------------------------------------------
		template <uint32_t Clock_Hz, ::mcu::clock::state::state state, int32_t calibration>
		class osc_cfg< ::mcu::clock::osc_type::low_speed_external_bypass, Clock_Hz, state, calibration>
		{
		private:
			STATIC_ASSERT(Clock_Hz >= mcu::stm32::clock::limits::min::LSECLK && Clock_Hz <= mcu::stm32::clock::limits::max::LSECLK, "LSE clock out of range");
			STATIC_ASSERT(calibration == ::mcu::clock::CALIBRATION_DEF, "LSE has not calibration");
		
			static const uint32_t _RCC_APB1ENR_PWREN_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->APB1ENR, RCC_APB1ENR_PWREN_Pos);
			static const uint32_t _PWR_CR_DBP_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&PWR->CR, PWR_CR_DBP_Pos);
			
			static const uint32_t _RCC_BDCR_LSEON_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->BDCR, RCC_BDCR_LSEON_Pos);
			static const uint32_t _RCC_BDCR_LSEBYP_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->BDCR, RCC_BDCR_LSEBYP_Pos);
			static const uint32_t _RCC_BDCR_LSERDY_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->BDCR, RCC_BDCR_LSERDY_Pos);

		public:
			static const ::mcu::clock::osc_type::osc_type	_OscType		= ::mcu::clock::osc_type::low_speed_external_bypass;
			static const uint32_t							_Clock_Hz		= Clock_Hz;
			static const ::mcu::clock::state::state			_State			= state;
			static const int32_t							_Calibration	= calibration;

			static const uint32_t							_pll_prediv		= 1;
			static const bool								_usb_div_1_5	= false;
			static const uint32_t							_pll_mul		= 1;
			static const uint32_t							_usb_clock		= 0;
			static const bool								_usb_active		= false;
		
			static void init()
			{
				(_State == ::mcu::clock::state::enable) ? start() : stop();
			}
			
			static void enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? start() : stop();
			}
			
			static void start()
			{
				// enabling
				/* Enable Power Clock*/
				SET_BB_REG(_RCC_APB1ENR_PWREN_BB);
				__NOP();

				/* Enable write access to Backup domain */
				SET_BB_REG(_PWR_CR_DBP_BB);

				while(IS_BB_REG_RESET(_PWR_CR_DBP_BB)) {}

				/* Set the new LSE configuration */
				RESET_BB_REG(_RCC_BDCR_LSEON_BB);
				__NOP();
				SET_BB_REG(_RCC_BDCR_LSEBYP_BB);
				SET_BB_REG(_RCC_BDCR_LSEON_BB);

				/* Wait till LSE is ready */  
				while(IS_BB_REG_RESET(_RCC_BDCR_LSERDY_BB)) {}
			}
			
			static void stop()
			{
				// disabling
				/* Set the new LSE configuration */
				RESET_BB_REG(_RCC_BDCR_LSEON_BB);

				/* Wait till LSE is ready */  
				while(IS_BB_REG_SET(_RCC_BDCR_LSERDY_BB)) {}

				RESET_BB_REG(_RCC_BDCR_LSEBYP_BB);
			}
		};
	} // namespace config

	//////////////////////////////////////////////////////////////////////////
	template <
		::mcu::clock::osc_type::osc_type	type,
		uint32_t							Clock_Hz = 0,
		::mcu::clock::state::state			state = ::mcu::clock::state::enable,
		int32_t								calibration = ::mcu::clock::CALIBRATION_DEF
		>
	class osc
	{
	public:
		typedef config::osc_cfg<type, Clock_Hz, state, calibration> _cfg_;
		
		static const uint32_t	_Clock_Hz		= _cfg_::_Clock_Hz;

		static void init()
		{
			_cfg_::init();
		}
		
		static void enable(::mcu::clock::state::state new_state)
		{
			(new_state == ::mcu::clock::state::enable) ? start() : stop();
		}
			
		static void start()
		{
			// enabling
			_cfg_::start();
		}
		
		static void stop()
		{
			// disabling
			_cfg_::stop();
		}
	};

	//------------------------------------------------------------------------
	typedef osc< ::mcu::clock::osc_type::high_speed_internal		, HSI_VALUE> osc_hsi;
	typedef osc< ::mcu::clock::osc_type::low_speed_internal			, LSI_VALUE> osc_lsi;

	typedef osc< ::mcu::clock::osc_type::high_speed_external		, METALLL_HSE_FREQ_HZ> osc_hse;
	typedef osc< ::mcu::clock::osc_type::high_speed_external_bypass	, METALLL_HSE_FREQ_HZ> osc_hse_bypass;
	typedef osc< ::mcu::clock::osc_type::low_speed_external			, METALLL_LSE_FREQ_HZ> osc_lse;
	typedef osc< ::mcu::clock::osc_type::low_speed_external_bypass	, METALLL_LSE_FREQ_HZ> osc_lse_bypass;


	//////////////////////////////////////////////////////////////////////////
	namespace config {
		
		template < ::mcu::clock::osc_type::osc_type type, typename OSC
					, uint32_t ClockMin_Hz, uint32_t ClockMax_Hz, bool maximal
					, ::mcu::clock::state::state state
					, bool pll_usb >
		class pll_cfg;

		namespace _internal_ {
			//------------------------------------------------------------------------
			template <uint32_t InClock_Hz, uint32_t ClockMin_Hz, uint32_t ClockMax_Hz, bool maximal, uint32_t pll_mul_min, uint32_t pll_mul_max>
			class pll_find_cfg
			{
			public:
				static const uint32_t	_freq_usb_min	= mcu::stm32::clock::limits::min::USBCLK;
				static const uint32_t	_freq_usb_max	= mcu::stm32::clock::limits::max::USBCLK;
				
				static const uint32_t	_InClock_Hz		= InClock_Hz	;
				static const uint32_t	_ClockMin_Hz	= ClockMin_Hz	;
				static const uint32_t	_ClockMax_Hz	= ClockMax_Hz	;
				static const bool		_maximal		= maximal		;
				static const uint32_t	_pll_mul_min	= pll_mul_min	;
				static const uint32_t	_pll_mul_max	= pll_mul_max	;
			};
			
			template <class CFG, int32_t find_dir, int32_t mul_value, uint32_t cnt>
			class pll_find_clock2
			{
				static const uint32_t freq			= mul_value * CFG::_InClock_Hz;
				
				static const bool freq_valid		= (freq >= CFG::_ClockMin_Hz && freq <= CFG::_ClockMax_Hz);
				
				typedef pll_find_clock2<CFG, find_dir, mul_value + find_dir, cnt - 1> pll_find_clock2_next;
				
			public:
				static const uint32_t pll_mul = (freq_valid) ?  mul_value : pll_find_clock2_next::pll_mul;
			};
			
			template <class CFG, int32_t find_dir, int32_t mul_value>
			class pll_find_clock2<CFG, find_dir, mul_value, 0>
			{
			public:
				static const uint32_t pll_mul = 0;
			};
			
			template <class CFG, bool maximal>
			class pll_find_clock
			{
			private:
				STATIC_ASSERT(maximal == CFG::_maximal, "internall error");
			
				typedef pll_find_clock2<CFG, -1, CFG::_pll_mul_max, (CFG::_pll_mul_max - CFG::_pll_mul_min)> _pll_mul_max;
				typedef pll_find_clock2<CFG, +1, CFG::_pll_mul_min, (CFG::_pll_mul_max - CFG::_pll_mul_min)> _pll_mul_min;
				
			public:
				static const uint32_t	pll_mul		= CFG::_maximal ? _pll_mul_max::pll_mul : _pll_mul_min::pll_mul;
				static const uint32_t	_Clock_Hz	= CFG::_InClock_Hz * pll_mul;
				static const bool		usb_div_1_5	= !(_Clock_Hz >= CFG::_freq_usb_min && _Clock_Hz <= CFG::_freq_usb_max);
				static const uint32_t	usb_clock	= usb_div_1_5 ? (uint32_t)(_Clock_Hz / 1.5) : _Clock_Hz;
				static const bool		usb_active	= usb_clock >= CFG::_freq_usb_min && usb_clock <= CFG::_freq_usb_max;
			};
			
			//------------------------------------------------------------------------
			template <uint32_t InClock_Hz, uint32_t ClockMin_Hz, uint32_t ClockMax_Hz, float usb_div, int32_t find_dir, int32_t mul_value, uint32_t cnt>
			class pll_find_usb_clock2
			{
				static const uint32_t freq			= mul_value * InClock_Hz;
				static const uint32_t freq_usb		= (bool)(usb_div > 0) ? mul_value * (uint32_t)(InClock_Hz / usb_div) : 0;
				static const uint32_t freq_usb_min	= mcu::stm32::clock::limits::min::USBCLK;
				static const uint32_t freq_usb_max	= mcu::stm32::clock::limits::max::USBCLK;
				
				static const bool freq_valid		= (freq >= ClockMin_Hz && freq <= ClockMax_Hz);
				static const bool usb_acceptable	= (bool)(usb_div > 0) ? (freq_usb >= freq_usb_min && freq_usb <= freq_usb_max) : true;
				
				typedef pll_find_usb_clock2<InClock_Hz, ClockMin_Hz, ClockMax_Hz, usb_div, find_dir, mul_value + find_dir, cnt - 1> pll_find_usb_clock2_next;
				
			public:
				static const uint32_t pll_mul = (freq_valid && usb_acceptable) ?  mul_value : pll_find_usb_clock2_next::pll_mul;
			};
			
			template <uint32_t InClock_Hz, uint32_t ClockMin_Hz, uint32_t ClockMax_Hz, float usb_div, int32_t find_dir, int32_t mul_value>
			class pll_find_usb_clock2<InClock_Hz, ClockMin_Hz, ClockMax_Hz, usb_div, find_dir, mul_value, 0>
			{
			public:
				static const uint32_t pll_mul = 0;
			};
			
			template <class CFG, bool maximal>
			class pll_find_usb_clock;
			
			template <class CFG>
			class pll_find_usb_clock<CFG, true>
			{
			private:
				STATIC_ASSERT(true == CFG::_maximal, "internall error");
				
				static const uint32_t pll_mul_1_5 = pll_find_usb_clock2<
					CFG::_InClock_Hz, CFG::_ClockMin_Hz, CFG::_ClockMax_Hz,
					1.5,
					-1, CFG::_pll_mul_max, (CFG::_pll_mul_max - CFG::_pll_mul_min) + 1
					>::pll_mul;
				
				static const uint32_t pll_mul_1_0 = pll_find_usb_clock2<
					CFG::_InClock_Hz, CFG::_ClockMin_Hz, CFG::_ClockMax_Hz,
					1.0,
					-1, CFG::_pll_mul_max, (CFG::_pll_mul_max - CFG::_pll_mul_min) + 1
					>::pll_mul;
			
			public:
				static const bool		usb_div_1_5 = (CFG::_InClock_Hz * pll_mul_1_5) >= (CFG::_InClock_Hz * pll_mul_1_0);		// choice the maximal output frequency from PLL
				static const uint32_t	pll_mul = usb_div_1_5 ? pll_mul_1_5 : pll_mul_1_0;
				static const uint32_t	_Clock_Hz = CFG::_InClock_Hz * pll_mul;
				static const uint32_t	usb_clock	= usb_div_1_5 ? (uint32_t)(_Clock_Hz / 1.5) : _Clock_Hz;
				static const bool		usb_active	= usb_clock >= CFG::_freq_usb_min && usb_clock <= CFG::_freq_usb_max;
			};
			
			template <class CFG>
			class pll_find_usb_clock<CFG, false>
			{
			private:
				static const uint32_t pll_mul_1_5 = pll_find_usb_clock2<
					CFG::_InClock_Hz, CFG::_ClockMin_Hz, CFG::_ClockMax_Hz,
					1.5,
					+1, CFG::_pll_mul_min, (CFG::_pll_mul_max - CFG::_pll_mul_min) + 1
					>::pll_mul;
				
				static const uint32_t pll_mul_1_0 = pll_find_usb_clock2<
					CFG::_InClock_Hz, CFG::_ClockMin_Hz, CFG::_ClockMax_Hz,
					1.0,
					+1, CFG::_pll_mul_min, (CFG::_pll_mul_max - CFG::_pll_mul_min) + 1
					>::pll_mul;

			public:
				static const bool		usb_div_1_5	= pll_mul_1_5 && (CFG::_InClock_Hz * pll_mul_1_5) < (CFG::_InClock_Hz * pll_mul_1_0);		// choice the minimal output frequency from PLL
				static const uint32_t	pll_mul		= usb_div_1_5 ? pll_mul_1_5 : pll_mul_1_0;
				static const uint32_t	_Clock_Hz	= CFG::_InClock_Hz * pll_mul;
				static const uint32_t	usb_clock	= usb_div_1_5 ? (uint32_t)(_Clock_Hz / 1.5) : _Clock_Hz;
				static const bool		usb_active	= usb_clock >= CFG::_freq_usb_min && usb_clock <= CFG::_freq_usb_max;
			};
			
			//------------------------------------------------------------------------
			template <class CFG, bool usb, int32_t find_dir, int32_t prediv_value, uint32_t cnt>
			class pll_find_clock_prediv2
			{
				typedef pll_find_cfg<CFG::_InClock_Hz / prediv_value, CFG::_ClockMin_Hz, CFG::_ClockMax_Hz,
					CFG::_maximal, CFG::_pll_mul_min, CFG::_pll_mul_max> _cfg_;
				
				typedef pll_find_usb_clock	< _cfg_, _cfg_::_maximal >	_pll_usb_cfg;
				typedef pll_find_clock		< _cfg_, _cfg_::_maximal >	_pll_cfg;
				
				typedef pll_find_clock_prediv2<CFG, usb, find_dir, prediv_value + find_dir, cnt - 1> pll_find_clock_prediv2_next;
				
			public:
				static const uint32_t prediv =
					(usb && _pll_usb_cfg::pll_mul)	? prediv_value :
					(!usb && _pll_cfg::pll_mul)		? prediv_value :
					pll_find_clock_prediv2_next::prediv;
			};
			
			template <class CFG, bool usb, int32_t find_dir, int32_t prediv_value>
			class pll_find_clock_prediv2<CFG, usb, find_dir, prediv_value, 0>
			{
			public:
				static const uint32_t prediv = 0;
			};

			template <class CFG, bool usb, uint32_t prediv_min, uint32_t prediv_max>
			class pll_find_clock_prediv
			{
				typedef pll_find_clock_prediv2<CFG, usb, +1, prediv_min, (prediv_max - prediv_min)+1> _pll_find_max;
				typedef pll_find_clock_prediv2<CFG, usb, -1, prediv_max, (prediv_max - prediv_min)+1> _pll_find_min;
				
			public:
				static const uint32_t	pll_prediv	= CFG::_maximal ? _pll_find_max::prediv : _pll_find_min::prediv;
			
			private:
				typedef pll_find_cfg<pll_prediv ? CFG::_InClock_Hz / pll_prediv : 0, CFG::_ClockMin_Hz, CFG::_ClockMax_Hz,
					CFG::_maximal, CFG::_pll_mul_min, CFG::_pll_mul_max> _cfg_;
				
				typedef pll_find_usb_clock	< _cfg_, _cfg_::_maximal >	_pll_usb_cfg;
				typedef pll_find_clock		< _cfg_, _cfg_::_maximal >	_pll_cfg;

			public:
				static const bool		usb_div_1_5	= usb ? _pll_usb_cfg::usb_div_1_5	: _pll_cfg::usb_div_1_5	;
				static const uint32_t	pll_mul		= usb ? _pll_usb_cfg::pll_mul		: _pll_cfg::pll_mul		;
				static const uint32_t	_Clock_Hz	= usb ? _pll_usb_cfg::_Clock_Hz		: _pll_cfg::_Clock_Hz	;
				static const uint32_t	usb_clock	= usb ? _pll_usb_cfg::usb_clock		: _pll_cfg::usb_clock	;
				static const bool		usb_active	= usb ? _pll_usb_cfg::usb_active	: _pll_cfg::usb_active	;
			};
		} // namespace _internal_
		using namespace _internal_;
		
		//------------------------------------------------------------------------
		template < typename OSC
					, uint32_t ClockMin_Hz, uint32_t ClockMax_Hz, bool maximal
					, ::mcu::clock::state::state state
					, bool pll_usb >
		class pll_cfg< ::mcu::clock::osc_type::high_speed_internal, OSC, ClockMin_Hz, ClockMax_Hz, maximal, state, pll_usb>
		{
		private:
			typedef OSC osc;

			typedef pll_find_clock_prediv< pll_find_cfg<osc::_cfg_::_Clock_Hz, ClockMin_Hz, ClockMax_Hz, maximal, 2UL, 16UL>, pll_usb, 2, 2 >	_pll_cfg;

			static const uint32_t _RCC_CR_PLLON_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_PLLON_Pos);
			static const uint32_t _RCC_CR_PLLRDY_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_PLLRDY_Pos);

		public:
			static const ::mcu::clock::osc_type::osc_type	_OscType		= osc::_cfg_::_OscType;
			static const uint32_t							_Clock_Hz		= _pll_cfg::_Clock_Hz;
			static const ::mcu::clock::state::state			_State			= state;
			static const int32_t							_Calibration	= ::mcu::clock::CALIBRATION_DEF;
		
			static const uint32_t							_pll_prediv		= _pll_cfg::pll_prediv	;
			static const bool								_usb_div_1_5	= _pll_cfg::usb_div_1_5	;
			static const uint32_t							_pll_mul		= _pll_cfg::pll_mul		;
			static const uint32_t							_usb_clock		= _pll_cfg::usb_clock	;
			static const bool								_usb_active		= _pll_cfg::usb_active	;
		
			STATIC_ASSERT(_pll_mul >= 2 && _pll_mul <= 16, "The required clock frequency cannot be reached");
			STATIC_ASSERT(!pll_usb || _usb_active, "PLL cannot find clock frequency for USB");

		public:
			static void init()
			{
				(_State == ::mcu::clock::state::enable) ? start() : stop();
			}
			
			static void enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? start() : stop();
			}
			
			static void start()
			{
				if(READ_BIT(RCC->CFGR, RCC_CFGR_SWS_Pos) == RCC_CFGR_SWS_PLL)
				{
					return;
				}
				
				// enabling
				osc::start();

				/* Disable the main PLL. */
				RESET_BB_REG(_RCC_CR_PLLON_BB);

				/* Wait till PLL is disabled */
				while(IS_BB_REG_SET(_RCC_CR_PLLRDY_BB)) {}

				/* Configure the main PLL clock source and multiplication factors. */
				MODIFY_REG(RCC->CFGR, (RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL), ((0 /*RCC_CFGR_PLLSRC*/) | ((_pll_mul - 2) << RCC_CFGR_PLLMULL_Pos) ));

				/* Enable the main PLL. */
				SET_BB_REG(_RCC_CR_PLLON_BB);

				/* Wait till PLL is ready */
				while(IS_BB_REG_RESET(_RCC_CR_PLLRDY_BB)) {}
			}
			
			static void stop()
			{
				// disabling
				/* Disable the main PLL. */
				RESET_BB_REG(_RCC_CR_PLLON_BB);

				/* Wait till PLL is disabled */
				while(IS_BB_REG_SET(_RCC_CR_PLLRDY_BB)) {}
				
				osc::stop();
			}
		};

		//------------------------------------------------------------------------
		template < typename OSC
					, uint32_t ClockMin_Hz, uint32_t ClockMax_Hz, bool maximal
					, ::mcu::clock::state::state state
					, bool pll_usb >
		class pll_cfg< ::mcu::clock::osc_type::high_speed_external, OSC, ClockMin_Hz, ClockMax_Hz, maximal, state, pll_usb>
		{
		private:
			typedef OSC osc;
			typedef pll_find_clock_prediv< pll_find_cfg<osc::_cfg_::_Clock_Hz, ClockMin_Hz, ClockMax_Hz, maximal, 2UL, 16UL>, pll_usb, 1, 2 >	_pll_cfg;

			static const uint32_t _RCC_CR_PLLON_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_PLLON_Pos);
			static const uint32_t _RCC_CR_PLLRDY_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_PLLRDY_Pos);
			static const uint32_t _RCC_CFGR_PLLXTPRE_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CFGR, RCC_CFGR_PLLXTPRE_Pos);

		public:
			static const ::mcu::clock::osc_type::osc_type	_OscType		= osc::_cfg_::_OscType;
			static const uint32_t							_Clock_Hz		= _pll_cfg::_Clock_Hz;
			static const ::mcu::clock::state::state			_State			= state;
			static const int32_t							_Calibration	= ::mcu::clock::CALIBRATION_DEF;
		
			static const uint32_t							_pll_prediv		= _pll_cfg::pll_prediv	;
			static const bool								_usb_div_1_5	= _pll_cfg::usb_div_1_5	;
			static const uint32_t							_pll_mul		= _pll_cfg::pll_mul		;
			static const uint32_t							_usb_clock		= _pll_cfg::usb_clock	;
			static const bool								_usb_active		= _pll_cfg::usb_active	;
		
			STATIC_ASSERT(_pll_mul >= 2 && _pll_mul <= 16, "The required clock frequency cannot be reached");
			STATIC_ASSERT(!pll_usb || _usb_active, "PLL cannot find clock frequency for USB");

		public:
			static void init()
			{
				(_State == ::mcu::clock::state::enable) ? start() : stop();
			}
			
			static void Enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? start() : stop();
			}
			
			static void start()
			{
				if(READ_BIT(RCC->CFGR, RCC_CFGR_SWS_Pos) == RCC_CFGR_SWS_PLL)
				{
					return;
				}
				
				// enabling
				osc::start();

				/* Disable the main PLL. */
				RESET_BB_REG(_RCC_CR_PLLON_BB);

				/* Wait till PLL is disabled */
				while(IS_BB_REG_SET(_RCC_CR_PLLRDY_BB)) {}

				/* Set PREDIV1 Value */
				WRITE_BB_REG(_RCC_CFGR_PLLXTPRE_BB, _pll_prediv == 1 ? 0 : 1);
				//MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLXTPRE, (uint32_t)(pll_prediv == 1 ? 0 : RCC_CFGR_PLLXTPRE));
				
				/* Configure the main PLL clock source and multiplication factors. */
				MODIFY_REG(RCC->CFGR, (RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL), ((RCC_CFGR_PLLSRC) | ((_pll_mul - 2) << RCC_CFGR_PLLMULL_Pos) ));

				/* Enable the main PLL. */
				SET_BB_REG(_RCC_CR_PLLON_BB);

				/* Wait till PLL is ready */
				while(IS_BB_REG_RESET(_RCC_CR_PLLRDY_BB)) {}
			}
			
			static void stop()
			{
				// disabling
				osc::stop();
			}
		};

		//------------------------------------------------------------------------
		template < typename OSC
					, uint32_t ClockMin_Hz, uint32_t ClockMax_Hz, bool maximal
					, ::mcu::clock::state::state state
					, bool pll_usb >
		class pll_cfg< ::mcu::clock::osc_type::high_speed_external_bypass, OSC, ClockMin_Hz, ClockMax_Hz, maximal, state, pll_usb>
			: public pll_cfg< ::mcu::clock::osc_type::high_speed_external, OSC, ClockMin_Hz, ClockMax_Hz, maximal, state, pll_usb>
		{
		};
	} // namespace config

	//////////////////////////////////////////////////////////////////////////
	template <
		class						OSC,
		uint32_t					ClockMin_Hz		= ::mcu::stm32::clock::limits::max::SYSCLK,
		uint32_t					ClockMax_Hz		= ::mcu::stm32::clock::limits::max::SYSCLK,
		bool						maximal			= true,
		bool						usb				= true,
		::mcu::clock::state::state	state			= ::mcu::clock::state::enable
		>
	class pll_auto_range
	{
	protected:
		STATIC_ASSERT(ClockMin_Hz <= ::mcu::stm32::clock::limits::max::SYSCLK, "The PLL clock is out of range");
		STATIC_ASSERT(ClockMax_Hz <= ::mcu::stm32::clock::limits::max::SYSCLK, "The PLL clock is out of range");
		STATIC_ASSERT(ClockMin_Hz <= ClockMax_Hz, "Wrong range for PLL clock");

	public:
		typedef config::pll_cfg<OSC::_cfg_::_OscType, OSC, ClockMin_Hz, ClockMax_Hz, maximal, state, usb> _cfg_;
	
	public:
		static void init()
		{
			_cfg_::init();
		}

		static void enable(::mcu::clock::state::state new_state)
		{
			(new_state == ::mcu::clock::state::enable) ? start() : stop();
		}
			
		static void start()
		{
			// enabling
			_cfg_::start();
		}
		
		static void stop()
		{
			// disabling
			_cfg_::stop();
		}
	};


	//////////////////////////////////////////////////////////////////////////
	template <
		class					ClockSource											,
		uint32_t				HCLK_Max_Hz					= limits::max::HCLK		,
		uint32_t				APB1CLK_Max_Hz				= limits::max::APB1CLK	,
		uint32_t				APB2CLK_Max_Hz				= limits::max::APB2CLK	,
		uint32_t				CortexSysTimer_Max_Hz		= 1000000/*Hz*/ / 1000/*ms*/	,		// limits::max::HCLK
		uint32_t				ADCCLK_Max_Hz				= limits::max::ADCCLK	
		>
	class sysclock_auto
	{
	public:
		class _cfg_
		{
		protected:
			typedef ClockSource osc;
			
			STATIC_ASSERT(	osc::_cfg_::_OscType == ::mcu::clock::osc_type::high_speed_internal			||
							osc::_cfg_::_OscType == ::mcu::clock::osc_type::high_speed_external			||
							osc::_cfg_::_OscType == ::mcu::clock::osc_type::high_speed_external_bypass
						, "Invalid source type for system clock");

			STATIC_ASSERT(HCLK_Max_Hz			>= limits::min::HCLK	/*&& HCLK_Max_Hz			<= limits::max::HCLK	*/ , "HCLK_Max_Hz			- Invalid parameter");
			STATIC_ASSERT(APB1CLK_Max_Hz		>= limits::min::APB1CLK	/*&& APB1CLK_Max_Hz			<= limits::max::APB1CLK	*/ , "APB1CLK_Max_Hz		- Invalid parameter");
			STATIC_ASSERT(APB2CLK_Max_Hz		>= limits::min::APB2CLK	/*&& APB2CLK_Max_Hz			<= limits::max::APB2CLK	*/ , "APB2CLK_Max_Hz		- Invalid parameter");
			STATIC_ASSERT(CortexSysTimer_Max_Hz	>= limits::min::HCLK	/*&& CortexSysTimer_Max_Hz	<= limits::max::HCLK	*/ , "CortexSysTimer_Max_Hz	- Invalid parameter");
			STATIC_ASSERT(ADCCLK_Max_Hz			>= limits::min::ADCCLK	/*&& ADCCLK_Max_Hz			<= limits::max::ADCCLK	*/ , "ADCCLK_Max_Hz			- Invalid parameter");
		
			STATIC_ASSERT(osc::_cfg_::_Clock_Hz >= limits::min::HCLK && osc::_cfg_::_Clock_Hz <= limits::max::HCLK, "Invalid source for system clock");

		public:
			static const ::mcu::clock::osc_type::osc_type	_OscType		= osc::_cfg_::_OscType;
			static const uint32_t							_Clock_Hz		= osc::_cfg_::_Clock_Hz;
			static const ::mcu::clock::state::state			_State			= osc::_cfg_::_State;
			static const int32_t							_Calibration	= osc::_cfg_::_Calibration;
		
			static const uint32_t							_pll_prediv		= osc::_cfg_::_pll_prediv	;
			static const bool								_usb_div_1_5	= osc::_cfg_::_usb_div_1_5	;
			static const uint32_t							_pll_mul		= osc::_cfg_::_pll_mul		;
			static const uint32_t							_usb_clock		= osc::_cfg_::_usb_clock	;
			static const bool								_usb_active		= osc::_cfg_::_usb_active	;
		
			//------------------------------------------------------------------------
			static const uint32_t	_ahb_div	=
				(_Clock_Hz /   1 <= HCLK_Max_Hz) ?   1 :
				(_Clock_Hz /   2 <= HCLK_Max_Hz) ?   2 :
				(_Clock_Hz /   4 <= HCLK_Max_Hz) ?   4 :
				(_Clock_Hz /   8 <= HCLK_Max_Hz) ?   8 :
				(_Clock_Hz /  16 <= HCLK_Max_Hz) ?  16 :
			//	(_Clock_Hz /  32 <= HCLK_Max_Hz) ?  32 :
				(_Clock_Hz /  64 <= HCLK_Max_Hz) ?  64 :
				(_Clock_Hz / 128 <= HCLK_Max_Hz) ? 128 :
				(_Clock_Hz / 256 <= HCLK_Max_Hz) ? 256 :
				(_Clock_Hz / 512 <= HCLK_Max_Hz) ? 512 :
				0;
			
			STATIC_ASSERT(_ahb_div > 0, "The required clock frequency for HCLK cannot be reached");

			static const uint32_t	_HCLK_Hz	= _Clock_Hz / _ahb_div;
			STATIC_ASSERT(_HCLK_Hz >= limits::min::HCLK	&& _HCLK_Hz <= limits::max::HCLK, "The required clock frequency for HCLK cannot be reached (or Invalid parameter HCLK_Max_Hz)");

			//------------------------------------------------------------------------
			static const uint32_t	_apb1_div	=
				(_HCLK_Hz /   1 <= APB1CLK_Max_Hz) ?   1 :
				(_HCLK_Hz /   2 <= APB1CLK_Max_Hz) ?   2 :
				(_HCLK_Hz /   4 <= APB1CLK_Max_Hz) ?   4 :
				(_HCLK_Hz /   8 <= APB1CLK_Max_Hz) ?   8 :
				(_HCLK_Hz /  16 <= APB1CLK_Max_Hz) ?  16 :
				0;

			STATIC_ASSERT(_apb1_div > 0, "The required clock frequency for APB1 cannot be reached");

			static const uint32_t	_PCLK1_Hz	= _HCLK_Hz / _apb1_div;
			STATIC_ASSERT(_PCLK1_Hz >= limits::min::APB1CLK	&& _PCLK1_Hz <= limits::max::APB1CLK, "The required clock frequency for APB1 cannot be reached (or Invalid parameter APB1CLK_Max_Hz)");

			//------------------------------------------------------------------------
			static const uint32_t	_apb2_div	=
				(_HCLK_Hz /   1 <= APB2CLK_Max_Hz) ?   1 :
				(_HCLK_Hz /   2 <= APB2CLK_Max_Hz) ?   2 :
				(_HCLK_Hz /   4 <= APB2CLK_Max_Hz) ?   4 :
				(_HCLK_Hz /   8 <= APB2CLK_Max_Hz) ?   8 :
				(_HCLK_Hz /  16 <= APB2CLK_Max_Hz) ?  16 :
				0;

			STATIC_ASSERT(_apb2_div > 0, "The required clock frequency for APB2 cannot be reached");

			static const uint32_t	_PCLK2_Hz	= _HCLK_Hz / _apb2_div;
			STATIC_ASSERT(_PCLK2_Hz >= limits::min::APB2CLK	&& _PCLK2_Hz <= limits::max::APB2CLK, "The required clock frequency for APB2 cannot be reached (or Invalid parameter APB2CLK_Max_Hz)");
			
			//------------------------------------------------------------------------
			static const uint32_t	_CortexSysTimer_div	=
				(_HCLK_Hz / SysTick_LOAD_RELOAD_Msk / 1 <= CortexSysTimer_Max_Hz) ? 1 :
				(_HCLK_Hz / SysTick_LOAD_RELOAD_Msk / 8 <= CortexSysTimer_Max_Hz) ? 8 :
				0;

			STATIC_ASSERT(_CortexSysTimer_div > 0, "The required clock frequency for CortexSysTimer cannot be reached");

			static const uint32_t	_CortexSysTimer_load	= _HCLK_Hz / CortexSysTimer_Max_Hz / _CortexSysTimer_div;
			STATIC_ASSERT((_CortexSysTimer_load-1) <= SysTick_LOAD_RELOAD_Msk, "The required clock frequency for CortexSysTimer cannot be reached (or Invalid parameter CortexSysTimer_Max_Hz)");
			
			static const uint32_t	_CortexSysTimer_Hz	= _HCLK_Hz / _CortexSysTimer_load / _CortexSysTimer_div;
			STATIC_ASSERT(_CortexSysTimer_Hz >= limits::min::HCLK && _CortexSysTimer_Hz <= limits::max::HCLK, "The required clock frequency for CortexSysTimer cannot be reached (or Invalid parameter CortexSysTimer_Max_Hz)");
			
			//------------------------------------------------------------------------
			static const uint32_t	_ADC_div	=
				(_PCLK2_Hz / 2 <= ADCCLK_Max_Hz) ? 2 :
				(_PCLK2_Hz / 4 <= ADCCLK_Max_Hz) ? 4 :
				(_PCLK2_Hz / 6 <= ADCCLK_Max_Hz) ? 6 :
				(_PCLK2_Hz / 8 <= ADCCLK_Max_Hz) ? 8 :
				0;

			STATIC_ASSERT(_ADC_div > 0, "The required clock frequency for ADC cannot be reached");

			static const uint32_t	_ADC_Hz	= _PCLK2_Hz / _ADC_div;
			STATIC_ASSERT(_ADC_Hz >= limits::min::ADCCLK && _ADC_Hz <= limits::max::ADCCLK, "The required clock frequency for ADC cannot be reached (or Invalid parameter ADCCLK_Max_Hz)");
			
			//------------------------------------------------------------------------
#if defined(FLASH_ACR_LATENCY)
			static const uint32_t	_flash_latency	=
				(_HCLK_Hz <= limits::max::FLASHCLK_LATENCY_0_Hz) ? 0 :			// Zero wait state
				(_HCLK_Hz <= limits::max::FLASHCLK_LATENCY_1_Hz) ? 1 :			// One wait state
				(_HCLK_Hz <= limits::max::FLASHCLK_LATENCY_2_Hz) ? 2 :			// Two wait states
				0xFFFFFFFF;

			STATIC_ASSERT(_flash_latency <= 2, "internal error: Invalid Clock Source frequency");

#else /* FLASH_ACR_LATENCY */
			static const uint32_t	_flash_latency	= 0;
			
#endif /* FLASH_ACR_LATENCY */

			//------------------------------------------------------------------------
			static const uint32_t	_rcc_cfgr_hpre_div =
				(_ahb_div ==   1) ? RCC_CFGR_HPRE_DIV1   :
				(_ahb_div ==   2) ? RCC_CFGR_HPRE_DIV2   :
				(_ahb_div ==   4) ? RCC_CFGR_HPRE_DIV4   :
				(_ahb_div ==   8) ? RCC_CFGR_HPRE_DIV8   :
				(_ahb_div ==  16) ? RCC_CFGR_HPRE_DIV16  :
			//	(_ahb_div ==  32) ? RCC_CFGR_HPRE_DIV32  :
				(_ahb_div ==  64) ? RCC_CFGR_HPRE_DIV64  :
				(_ahb_div == 128) ? RCC_CFGR_HPRE_DIV128 :
				(_ahb_div == 256) ? RCC_CFGR_HPRE_DIV256 :
				(_ahb_div == 512) ? RCC_CFGR_HPRE_DIV512 :
				0;

			static const uint32_t _flash_acr_latency	= _flash_latency << FLASH_ACR_LATENCY_Pos;

			static const uint32_t _rcc_cfgr_sw =
				(_pll_mul > 1)														?	RCC_CFGR_SW_PLL :
				(_OscType == ::mcu::clock::osc_type::high_speed_internal)			?	RCC_CFGR_SW_HSI :
				(_OscType == ::mcu::clock::osc_type::high_speed_external)			?	RCC_CFGR_SW_HSE :
				(_OscType == ::mcu::clock::osc_type::high_speed_external_bypass)	?	RCC_CFGR_SW_HSE :
																						RCC_CFGR_SW_HSI;
			
			static const uint32_t _rcc_cfgr_sws =
				(_pll_mul > 1)														?	RCC_CFGR_SWS_PLL :
				(_OscType == ::mcu::clock::osc_type::high_speed_internal)			?	RCC_CFGR_SWS_HSI :
				(_OscType == ::mcu::clock::osc_type::high_speed_external)			?	RCC_CFGR_SWS_HSE :
				(_OscType == ::mcu::clock::osc_type::high_speed_external_bypass)	?	RCC_CFGR_SWS_HSE :
																						RCC_CFGR_SWS_HSI;

			static const uint32_t	_rcc_cfgr_ppre1	=
				(_apb1_div ==  1) ?	RCC_CFGR_PPRE1_DIV1  :
				(_apb1_div ==  2) ?	RCC_CFGR_PPRE1_DIV2  :
				(_apb1_div ==  4) ?	RCC_CFGR_PPRE1_DIV4  :
				(_apb1_div ==  8) ?	RCC_CFGR_PPRE1_DIV8  :
				(_apb1_div == 16) ?	RCC_CFGR_PPRE1_DIV16 :
									RCC_CFGR_PPRE1_DIV16;

			static const uint32_t	_rcc_cfgr_ppre2 = 
				(_apb2_div ==  1) ?	RCC_CFGR_PPRE2_DIV1  :
				(_apb2_div ==  2) ?	RCC_CFGR_PPRE2_DIV2  :
				(_apb2_div ==  4) ?	RCC_CFGR_PPRE2_DIV4  :
				(_apb2_div ==  8) ?	RCC_CFGR_PPRE2_DIV8  :
				(_apb2_div == 16) ?	RCC_CFGR_PPRE2_DIV16 :
									RCC_CFGR_PPRE2_DIV16;

			static const uint32_t	_rcc_cfgr_adcpre =
				(_ADC_div == 2) ? RCC_CFGR_ADCPRE_DIV2 :
				(_ADC_div == 4) ? RCC_CFGR_ADCPRE_DIV4 :
				(_ADC_div == 6) ? RCC_CFGR_ADCPRE_DIV6 :
				(_ADC_div == 8) ? RCC_CFGR_ADCPRE_DIV8 :
				0;
			
			static const uint32_t _rcc_cfgr_usbpre = _usb_div_1_5 ? 0 : RCC_CFGR_USBPRE;

			//------------------------------------------------------------------------
			
		public:
			static void init()
			{
#if defined(FLASH_ACR_PRFTBE)
				FLASH->ACR |= FLASH_ACR_PRFTBE;
#endif /* FLASH_ACR_PRFTBE */
				
				// Set Interrupt Group Priority
				NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP);

#if defined(FLASH_ACR_LATENCY)
				// Set maximal safe FLASH LATENCY during switching
				MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, 2);
#endif /* FLASH_ACR_LATENCY */

				// HCLK Configuration
				MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, _rcc_cfgr_hpre_div);
				
				// SYSCLK Configuration
				osc::init();														// Clock Source Initialization

				// if(_pll_mul)
				// {
				// 	// PLL is selected as System Clock Source
				// 	static const uint32_t _RCC_CR_PLLRDY_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_PLLRDY_Pos);
				// 	
				// 	if(IS_BB_REG_RESET(_RCC_CR_PLLRDY_BB))			// Check the PLL ready
				// 		return;
				// }
				// else if(_OscType == ::mcu::clock::osc_type::high_speed_internal)
				// {
				// 	// HSI is selected as System Clock Source
				// 	static const uint32_t _RCC_CR_HSIRDY_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSIRDY_Pos);
				// 	
				// 	if(IS_BB_REG_RESET(_RCC_CR_HSIRDY_BB))			// Check the HSI ready
				// 		return;
				// }
				// else if(_OscType == ::mcu::clock::osc_type::high_speed_external || _OscType == ::mcu::clock::osc_type::high_speed_external_bypass)
				// {
				// 	// HSE is selected as System Clock Source
				// 	static const uint32_t _RCC_CR_HSERDY_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSERDY_Pos);
				// 	
				// 	if(IS_BB_REG_RESET(_RCC_CR_HSERDY_BB))			// Check the HSE ready
				// 		return;			// FAILED
				// }
				// else
				// {
				// 	STATIC_ASSERT(false, "internal error: Invalid Clock Source type");
				// }

				MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, _rcc_cfgr_sw);					// Switching to Clock Source
				
				while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != _rcc_cfgr_sws) {}		// Waiting for finalization

#if defined(FLASH_ACR_LATENCY)
				MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, _flash_acr_latency);		// Initialization FLASH LATENCY
#endif /* FLASH_ACR_LATENCY */

				// PCLK1 Configuration
				MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, _rcc_cfgr_ppre1);

				// PCLK2 Configuration
				MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, _rcc_cfgr_ppre2);

				// Configure the SysTick to have interrupt in 1ms time basis
				SysTick_Config(_CortexSysTimer_load);
				(_CortexSysTimer_div == 1) ?	(void)(SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK) :
				(_CortexSysTimer_div == 8) ?	(void)(SysTick->CTRL &= ~SYSTICK_CLKSOURCE_HCLK) :
												(void)0;

				// Configure the SysTick IRQ priority
				//NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP /*NVIC_GetPriorityGrouping()*/, TICK_INT_PRIORITY, 0));

				// Configure the ADC clock source
				MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, _rcc_cfgr_adcpre);

				// Configure the USB clock source
				MODIFY_REG(RCC->CFGR, RCC_CFGR_USBPRE, _rcc_cfgr_usbpre);
			}
		};
		
	public:
		static void init()
		{
			_cfg_::init();
		}
	};
	
//////////////////////////////////////////////////////////////////////////
} // namespace clock
} // namespace stm32

/************************************************************************/
/*                                                                      */
/************************************************************************/


namespace clock {

template < state::state state, int32_t calibration, uint32_t Clock_Hz >
class osc_hsi : public ::mcu::stm32::clock::osc< osc_type::high_speed_internal, Clock_Hz == CLOCK_HZ_DEF ? HSI_VALUE : Clock_Hz, state, calibration> {};

template < state::state state, uint32_t Clock_Hz >
class osc_lsi : public ::mcu::stm32::clock::osc< osc_type::low_speed_internal, Clock_Hz == CLOCK_HZ_DEF ? LSI_VALUE : Clock_Hz, state > {};

template < state::state state, uint32_t Clock_Hz >
class osc_hse : public ::mcu::stm32::clock::osc< osc_type::high_speed_external, Clock_Hz == CLOCK_HZ_DEF ? METALLL_HSE_FREQ_HZ : Clock_Hz, state > {};

template < state::state state, uint32_t Clock_Hz >
class osc_hse_bypass : public ::mcu::stm32::clock::osc< osc_type::high_speed_external_bypass, Clock_Hz == CLOCK_HZ_DEF ? METALLL_HSE_FREQ_HZ : Clock_Hz, state > {};

template < state::state state, uint32_t Clock_Hz >
class osc_lse : public ::mcu::stm32::clock::osc< osc_type::low_speed_external, Clock_Hz == CLOCK_HZ_DEF ? METALLL_LSE_FREQ_HZ : Clock_Hz, state > {};

template < state::state state, uint32_t Clock_Hz >
class osc_lse_bypass : public ::mcu::stm32::clock::osc< osc_type::low_speed_external_bypass, Clock_Hz == CLOCK_HZ_DEF ? METALLL_LSE_FREQ_HZ : Clock_Hz, state > {};

//////////////////////////////////////////////////////////////////////////	
template < state::state state > class pll_hsi
	: public ::mcu::stm32::clock::pll_auto_range< osc_hsi_def, ::mcu::stm32::clock::limits::min::SYSCLK, ::mcu::stm32::clock::limits::max::SYSCLK, true, false, state >
{};

template < state::state state > class pll_hse
	: public ::mcu::stm32::clock::pll_auto_range< osc_hse_def, ::mcu::stm32::clock::limits::min::SYSCLK, ::mcu::stm32::clock::limits::max::SYSCLK, true, true, state >
{};

template < state::state state > class pll_hse_bypass
	: public ::mcu::stm32::clock::pll_auto_range< osc_hse_bypass_def, ::mcu::stm32::clock::limits::min::SYSCLK, ::mcu::stm32::clock::limits::max::SYSCLK, true, true, state >
{};

//------------------------------------------------------------------------
template< class OSC, uint32_t Clock_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
class pll_auto : public ::mcu::stm32::clock::pll_auto_range< OSC, Clock_Hz, Clock_Hz, true, false, state > {};

template< class OSC, uint32_t ClockMin_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, uint32_t ClockMax_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
class pll_min : public ::mcu::stm32::clock::pll_auto_range< OSC, ClockMin_Hz, ClockMax_Hz, false, false, state > {};

template< class OSC, uint32_t ClockMin_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, uint32_t ClockMax_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
class pll_max : public ::mcu::stm32::clock::pll_auto_range< OSC, ClockMin_Hz, ClockMax_Hz, true, false, state > {};

template< class OSC, uint32_t Clock_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
class pll_usb_auto : public ::mcu::stm32::clock::pll_auto_range< OSC, Clock_Hz, Clock_Hz, true, true, state > {};

template< class OSC, uint32_t ClockMin_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, uint32_t ClockMax_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
class pll_usb_min : public ::mcu::stm32::clock::pll_auto_range< OSC, ClockMin_Hz, ClockMax_Hz, false, true, state > {};

template< class OSC, uint32_t ClockMin_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, uint32_t ClockMax_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
class pll_usb_max : public ::mcu::stm32::clock::pll_auto_range< OSC, ClockMin_Hz, ClockMax_Hz, true, true, state > {};


//////////////////////////////////////////////////////////////////////////	
template<
		class		ClockSource																,
		uint32_t	CortexSysTimer_ms			= 1000										,
		uint32_t	HCLK_Max_Hz					= ::mcu::stm32::clock::limits::max::HCLK	,
		uint32_t	APB1CLK_Max_Hz				= ::mcu::stm32::clock::limits::max::APB1CLK	,
		uint32_t	APB2CLK_Max_Hz				= ::mcu::stm32::clock::limits::max::APB2CLK	,
		uint32_t	ADCCLK_Max_Hz				= ::mcu::stm32::clock::limits::max::ADCCLK	
	>
class sysclock_auto : public ::mcu::stm32::clock::sysclock_auto< ClockSource, HCLK_Max_Hz, APB1CLK_Max_Hz, APB2CLK_Max_Hz, 1000000/*Hz*/ / CortexSysTimer_ms, ADCCLK_Max_Hz > {};

template<
		class		ClockSource,
		uint32_t	CortexSysTimer_ms
	>
class sysclock_max : public sysclock_auto< ClockSource, CortexSysTimer_ms > {};

} // namespace clock

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__stm32f1xx_clock_hpp__*/
