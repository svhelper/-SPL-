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


//////////////////////////////////////////////////////////////////////////
#ifndef HSE_VALUE
#	define HSE_VALUE				8000000UL
#endif /* HSE_VALUE */

#ifndef LSE_VALUE
#	define LSE_VALUE				32768UL
#endif /* LSE_VALUE */


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
namespace mcu {

namespace clock {
	
	namespace state {
		typedef enum
		{
			disable,
			enable = !disable,
		} state;
	} // namespace state

	namespace config {
		typedef enum
		{
			high_speed_internal,
			low_speed_internal,

			high_speed_external,
			high_speed_external_bypass,
			low_speed_external,
			low_speed_external_bypass,
		} osc_type;
	} // namespace config
	
} // namespace clock

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace stm32 {
namespace clock {

	//////////////////////////////////////////////////////////////////////////
	namespace limits {
		static const uint32_t USB_PPM	= 2500;

		class max
		{
		public:
			static const uint32_t SYSCLK	= 72000000UL;

			static const uint32_t HSECLK	= 25000000UL;
			static const uint32_t LSECLK	= 1000000UL;
		
			static const uint32_t HCLK		= SYSCLK;
			static const uint32_t USBCLK	= (uint32_t)(48000000UL * (1.E6 + USB_PPM) / 1.E6);
			static const uint32_t APB1CLK	= 36000000UL;
			static const uint32_t APB2CLK	= 72000000UL;
			static const uint32_t ADCCLK	= 14000000UL;
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
		
		template < ::mcu::clock::config::osc_type type, uint32_t Clock_Hz, ::mcu::clock::state::state state, int32_t calibration = 0x7FFFFFFF>
		class osc_cfg;

		//------------------------------------------------------------------------
		template <uint32_t Clock_Hz, ::mcu::clock::state::state state, int32_t calibration>
		class osc_cfg< ::mcu::clock::config::high_speed_internal, Clock_Hz, state, calibration>
		{
		private:
			static const int32_t calibration_val = (calibration == 0x7FFFFFFF) ? HSI_CALIBRATION_DEFAULT : calibration;
		
			STATIC_ASSERT(Clock_Hz == HSI_VALUE, "HSI clock is invalid");
			STATIC_ASSERT(calibration_val >= 0x00 && calibration_val <= 0x1F, "HSI calibration value out of range");
		
			static const uint32_t _RCC_CR_HSION_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSION_Pos);
			static const uint32_t _RCC_CR_HSIRDY_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSIRDY_Pos);

		public:
			static const ::mcu::clock::config::osc_type	_OscType		= ::mcu::clock::config::high_speed_internal;
			static const uint32_t						_Clock_Hz		= Clock_Hz;
			static const ::mcu::clock::state::state		_State			= state;
			static const int32_t						_Calibration	= calibration_val;
		
			static const uint32_t						_pll_prediv		= 1;
			static const bool							_usb_div_1_5	= false;
			static const uint32_t						_pll_mul		= 1;
			static const uint32_t						_usb_clock		= 0;
			static const bool							_usb_active		= false;
		
			static void Init()
			{
				(_State == ::mcu::clock::state::enable) ? Start() : Stop();
				
				/* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
				MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, (uint32_t)_Calibration << RCC_CR_HSITRIM_Pos);
			}
			
			static void Enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? Start() : Stop();
			}
			
			static void Start()
			{
				// enabling
				SET_BB_REG(_RCC_CR_HSION_BB);

				/* Wait till HSI is ready */
				while(IS_BB_REG_RESET(_RCC_CR_HSIRDY_BB)) {}
			}
			
			static void Stop()
			{
				// disabling
				RESET_BB_REG(_RCC_CR_HSION_BB);

				/* Wait till HSI is ready */
				while(IS_BB_REG_SET(_RCC_CR_HSIRDY_BB)) {}
			}
		};

		//------------------------------------------------------------------------
		template <uint32_t Clock_Hz, ::mcu::clock::state::state state, int32_t calibration>
		class osc_cfg< ::mcu::clock::config::low_speed_internal, Clock_Hz, state, calibration >
		{
		private:
			STATIC_ASSERT(Clock_Hz == LSI_VALUE, "LSI clock is invalid");
			STATIC_ASSERT(calibration == 0x7FFFFFFF, "LSI has not calibration");
		
			static const uint32_t _RCC_CSR_LSION_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CSR, RCC_CSR_LSION_Pos);
			static const uint32_t _RCC_CSR_LSIRDY_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CSR, RCC_CSR_LSIRDY_Pos);

		public:
			static const ::mcu::clock::config::osc_type	_OscType		= ::mcu::clock::config::low_speed_internal;
			static const uint32_t						_Clock_Hz		= Clock_Hz;
			static const ::mcu::clock::state::state		_State			= state;
			static const int32_t						_Calibration	= calibration;
		
			static const uint32_t						_pll_prediv		= 1;
			static const bool							_usb_div_1_5	= false;
			static const uint32_t						_pll_mul		= 1;
			static const uint32_t						_usb_clock		= 0;
			static const bool							_usb_active		= false;

			static void Init()
			{
				(_State == ::mcu::clock::state::enable) ? Start() : Stop();
			}
			
			static void Enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? Start() : Stop();
			}
			
			static void Start()
			{
				// enabling
				SET_BB_REG(_RCC_CSR_LSION_BB);

				/* Wait till LSI is ready */
				while(IS_BB_REG_RESET(_RCC_CSR_LSIRDY_BB)) {}

				/*  To have a fully stabilized clock in the specified range, a software delay of 1ms should be added.*/
				//HAL_Delay(1);
			}
			
			static void Stop()
			{
				// disabling
				RESET_BB_REG(_RCC_CSR_LSION_BB);

				/* Wait till LSI is ready */
				while(IS_BB_REG_SET(_RCC_CSR_LSIRDY_BB)) {}
			}
		};

		//------------------------------------------------------------------------
		template <uint32_t Clock_Hz, ::mcu::clock::state::state state, int32_t calibration>
		class osc_cfg< ::mcu::clock::config::high_speed_external, Clock_Hz, state, calibration>
		{
		private:
			STATIC_ASSERT(Clock_Hz >= mcu::stm32::clock::limits::min::HSECLK && Clock_Hz <= mcu::stm32::clock::limits::max::HSECLK, "HSE clock out of range");
			STATIC_ASSERT(calibration == 0x7FFFFFFF, "HSE has not calibration");
		
			static const uint32_t _RCC_CR_HSEON_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSEON_Pos);
			static const uint32_t _RCC_CR_HSEBYP_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSEBYP_Pos);
			static const uint32_t _RCC_CR_HSERDY_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSERDY_Pos);

		public:
			static const ::mcu::clock::config::osc_type	_OscType		= ::mcu::clock::config::high_speed_external;
			static const uint32_t						_Clock_Hz		= Clock_Hz;
			static const ::mcu::clock::state::state		_State			= state;
			static const int32_t						_Calibration	= calibration;

			static const uint32_t						_pll_prediv		= 1;
			static const bool							_usb_div_1_5	= false;
			static const uint32_t						_pll_mul		= 1;
			static const uint32_t						_usb_clock		= 0;
			static const bool							_usb_active		= false;
		
			static void Init()
			{
				(_State == ::mcu::clock::state::enable) ? Start() : Stop();
			}
			
			static void Enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? Start() : Stop();
			}
			
			static void Start()
			{
				// enabling
				RESET_BB_REG(_RCC_CR_HSEON_BB);
				__NOP();
				RESET_BB_REG(_RCC_CR_HSEBYP_BB);
				SET_BB_REG(_RCC_CR_HSEON_BB);

				/* Wait till HSE is ready */
				while(IS_BB_REG_RESET(_RCC_CR_HSERDY_BB)) {}
			}
			
			static void Stop()
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
		class osc_cfg< ::mcu::clock::config::high_speed_external_bypass, Clock_Hz, state, calibration>
		{
		private:
			STATIC_ASSERT(Clock_Hz >= mcu::stm32::clock::limits::min::HSECLK && Clock_Hz <= mcu::stm32::clock::limits::max::HSECLK, "HSE clock out of range");
			STATIC_ASSERT(calibration == 0x7FFFFFFF, "HSE has not calibration");
		
			static const uint32_t _RCC_CR_HSEON_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSEON_Pos);
			static const uint32_t _RCC_CR_HSEBYP_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSEBYP_Pos);
			static const uint32_t _RCC_CR_HSERDY_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_HSERDY_Pos);

		public:
			static const ::mcu::clock::config::osc_type	_OscType		= ::mcu::clock::config::high_speed_external_bypass;
			static const uint32_t						_Clock_Hz		= Clock_Hz;
			static const ::mcu::clock::state::state		_State			= state;
			static const int32_t						_Calibration	= calibration;

			static const uint32_t						_pll_prediv		= 1;
			static const bool							_usb_div_1_5	= false;
			static const uint32_t						_pll_mul		= 1;
			static const uint32_t						_usb_clock		= 0;
			static const bool							_usb_active		= false;
		
			static void Init()
			{
				(_State == ::mcu::clock::state::enable) ? Start() : Stop();
			}
			
			static void Enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? Start() : Stop();
			}
			
			static void Start()
			{
				// enabling
				RESET_BB_REG(_RCC_CR_HSEON_BB);
				__NOP();
				SET_BB_REG(_RCC_CR_HSEBYP_BB);
				SET_BB_REG(_RCC_CR_HSEON_BB);

				/* Wait till HSE is ready */
				while(IS_BB_REG_RESET(_RCC_CR_HSERDY_BB)) {}
			}
			
			static void Stop()
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
		class osc_cfg< ::mcu::clock::config::low_speed_external, Clock_Hz, state, calibration>
		{
		private:
			STATIC_ASSERT(Clock_Hz >= mcu::stm32::clock::limits::min::LSECLK && Clock_Hz <= mcu::stm32::clock::limits::max::LSECLK, "LSE clock out of range");
			STATIC_ASSERT(calibration == 0x7FFFFFFF, "LSE has not calibration");
		
			static const uint32_t _RCC_APB1ENR_PWREN_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->APB1ENR, RCC_APB1ENR_PWREN_Pos);
			static const uint32_t _PWR_CR_DBP_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&PWR->CR, PWR_CR_DBP_Pos);
			
			static const uint32_t _RCC_BDCR_LSEON_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->BDCR, RCC_BDCR_LSEON_Pos);
			static const uint32_t _RCC_BDCR_LSEBYP_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->BDCR, RCC_BDCR_LSEBYP_Pos);
			static const uint32_t _RCC_BDCR_LSERDY_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->BDCR, RCC_BDCR_LSERDY_Pos);

		public:
			static const ::mcu::clock::config::osc_type	_OscType		= ::mcu::clock::config::low_speed_external;
			static const uint32_t						_Clock_Hz		= Clock_Hz;
			static const ::mcu::clock::state::state		_State			= state;
			static const int32_t						_Calibration	= calibration;

			static const uint32_t						_pll_prediv		= 1;
			static const bool							_usb_div_1_5	= false;
			static const uint32_t						_pll_mul		= 1;
			static const uint32_t						_usb_clock		= 0;
			static const bool							_usb_active		= false;
		
			static void Init()
			{
				(_State == ::mcu::clock::state::enable) ? Start() : Stop();
			}
			
			static void Enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? Start() : Stop();
			}
			
			static void Start()
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
			
			static void Stop()
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
		class osc_cfg< ::mcu::clock::config::low_speed_external_bypass, Clock_Hz, state, calibration>
		{
		private:
			STATIC_ASSERT(Clock_Hz >= mcu::stm32::clock::limits::min::LSECLK && Clock_Hz <= mcu::stm32::clock::limits::max::LSECLK, "LSE clock out of range");
			STATIC_ASSERT(calibration == 0x7FFFFFFF, "LSE has not calibration");
		
			static const uint32_t _RCC_APB1ENR_PWREN_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->APB1ENR, RCC_APB1ENR_PWREN_Pos);
			static const uint32_t _PWR_CR_DBP_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&PWR->CR, PWR_CR_DBP_Pos);
			
			static const uint32_t _RCC_BDCR_LSEON_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->BDCR, RCC_BDCR_LSEON_Pos);
			static const uint32_t _RCC_BDCR_LSEBYP_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->BDCR, RCC_BDCR_LSEBYP_Pos);
			static const uint32_t _RCC_BDCR_LSERDY_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->BDCR, RCC_BDCR_LSERDY_Pos);

		public:
			static const ::mcu::clock::config::osc_type	_OscType		= ::mcu::clock::config::low_speed_external_bypass;
			static const uint32_t						_Clock_Hz		= Clock_Hz;
			static const ::mcu::clock::state::state		_State			= state;
			static const int32_t						_Calibration	= calibration;

			static const uint32_t						_pll_prediv		= 1;
			static const bool							_usb_div_1_5	= false;
			static const uint32_t						_pll_mul		= 1;
			static const uint32_t						_usb_clock		= 0;
			static const bool							_usb_active		= false;
		
			static void Init()
			{
				(_State == ::mcu::clock::state::enable) ? Start() : Stop();
			}
			
			static void Enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? Start() : Stop();
			}
			
			static void Start()
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
			
			static void Stop()
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
		::mcu::clock::config::osc_type	type,
		uint32_t						Clock_Hz = 0,
		::mcu::clock::state::state		state = ::mcu::clock::state::enable,
		int32_t							calibration = 0x7FFFFFFF
		>
	class osc
	{
	public:
		typedef config::osc_cfg<type, Clock_Hz, state, calibration> _cfg_;
		
		static const uint32_t	_Clock_Hz		= _cfg_::_Clock_Hz;

		static void Init()
		{
			_cfg_::Init();
		}
		
		static void Enable(::mcu::clock::state::state new_state)
		{
			(new_state == ::mcu::clock::state::enable) ? Start() : Stop();
		}
			
		static void Start()
		{
			// enabling
			_cfg_::Start();
		}
		
		static void Stop()
		{
			// disabling
			_cfg_::Stop();
		}
	};

	//------------------------------------------------------------------------
	typedef osc< ::mcu::clock::config::high_speed_internal			, HSI_VALUE> osc_hsi;
	typedef osc< ::mcu::clock::config::low_speed_internal			, LSI_VALUE> osc_lsi;

	typedef osc< ::mcu::clock::config::high_speed_external			, HSE_VALUE> osc_hse;
	typedef osc< ::mcu::clock::config::high_speed_external_bypass	, HSE_VALUE> osc_hse_bypass;
	typedef osc< ::mcu::clock::config::low_speed_external			, LSE_VALUE> osc_lse;
	typedef osc< ::mcu::clock::config::low_speed_external_bypass	, LSE_VALUE> osc_lse_bypass;


	//////////////////////////////////////////////////////////////////////////
	namespace config {
		
		template < ::mcu::clock::config::osc_type type, typename OSC
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
				typedef pll_find_cfg<CFG::_InClock_Hz / pll_prediv, CFG::_ClockMin_Hz, CFG::_ClockMax_Hz,
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
		class pll_cfg< ::mcu::clock::config::high_speed_internal, OSC, ClockMin_Hz, ClockMax_Hz, maximal, state, pll_usb>
		{
		private:
			typedef OSC osc;

			typedef pll_find_clock_prediv< pll_find_cfg<osc::_cfg_::_Clock_Hz, ClockMin_Hz, ClockMax_Hz, maximal, 2UL, 16UL>, pll_usb, 2, 2 >	_pll_cfg;

			static const uint32_t _RCC_CR_PLLON_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_PLLON_Pos);
			static const uint32_t _RCC_CR_PLLRDY_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_PLLRDY_Pos);

		public:
			static const ::mcu::clock::config::osc_type	_OscType		= osc::_cfg_::_OscType;
			static const uint32_t						_Clock_Hz		= _pll_cfg::_Clock_Hz;
			static const ::mcu::clock::state::state		_State			= state;
			static const int32_t						_Calibration	= 0x7FFFFFFF;
		
			static const uint32_t						_pll_prediv		= _pll_cfg::pll_prediv	;
			static const bool							_usb_div_1_5	= _pll_cfg::usb_div_1_5	;
			static const uint32_t						_pll_mul		= _pll_cfg::pll_mul		;
			static const uint32_t						_usb_clock		= _pll_cfg::usb_clock	;
			static const bool							_usb_active		= _pll_cfg::usb_active	;
		
			STATIC_ASSERT(_pll_mul >= 2 && _pll_mul <= 16, "The required clock frequency cannot be reached");
			STATIC_ASSERT(!pll_usb || _usb_active, "PLL cannot find clock frequency for USB");

		public:
			static void Init()
			{
				(_State == ::mcu::clock::state::enable) ? Start() : Stop();
			}
			
			static void Enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? Start() : Stop();
			}
			
			static void Start()
			{
				if(READ_BIT(RCC->CFGR, RCC_CFGR_SWS_Pos) == RCC_CFGR_SWS_PLL)
				{
					return;
				}
				
				// enabling
				osc::Start();

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
			
			static void Stop()
			{
				// disabling
				/* Disable the main PLL. */
				RESET_BB_REG(_RCC_CR_PLLON_BB);

				/* Wait till PLL is disabled */
				while(IS_BB_REG_SET(_RCC_CR_PLLRDY_BB)) {}
				
				osc::Stop();
			}
		};

		//------------------------------------------------------------------------
		template < typename OSC
					, uint32_t ClockMin_Hz, uint32_t ClockMax_Hz, bool maximal
					, ::mcu::clock::state::state state
					, bool pll_usb >
		class pll_cfg< ::mcu::clock::config::high_speed_external, OSC, ClockMin_Hz, ClockMax_Hz, maximal, state, pll_usb>
		{
		private:
			typedef OSC osc;
			typedef pll_find_clock_prediv< pll_find_cfg<osc::_cfg_::_Clock_Hz, ClockMin_Hz, ClockMax_Hz, maximal, 2UL, 16UL>, pll_usb, 1, 2 >	_pll_cfg;

			static const uint32_t _RCC_CR_PLLON_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_PLLON_Pos);
			static const uint32_t _RCC_CR_PLLRDY_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CR, RCC_CR_PLLRDY_Pos);
			static const uint32_t _RCC_CFGR_PLLXTPRE_BB	= FROM_ADDRESS_BIT_POS_TO_BB(&RCC->CFGR, RCC_CFGR_PLLXTPRE_Pos);

		public:
			static const ::mcu::clock::config::osc_type	_OscType		= osc::_cfg_::_OscType;
			static const uint32_t						_Clock_Hz		= _pll_cfg::_Clock_Hz;
			static const ::mcu::clock::state::state		_State			= state;
			static const int32_t						_Calibration	= 0x7FFFFFFF;
		
			static const uint32_t						_pll_prediv		= _pll_cfg::pll_prediv	;
			static const bool							_usb_div_1_5	= _pll_cfg::usb_div_1_5	;
			static const uint32_t						_pll_mul		= _pll_cfg::pll_mul		;
			static const uint32_t						_usb_clock		= _pll_cfg::usb_clock	;
			static const bool							_usb_active		= _pll_cfg::usb_active	;
		
			STATIC_ASSERT(_pll_mul >= 2 && _pll_mul <= 16, "The required clock frequency cannot be reached");
			STATIC_ASSERT(!pll_usb || _usb_active, "PLL cannot find clock frequency for USB");

		public:
			static void Init()
			{
				(_State == ::mcu::clock::state::enable) ? Start() : Stop();
			}
			
			static void Enable(::mcu::clock::state::state new_state)
			{
				(new_state == ::mcu::clock::state::enable) ? Start() : Stop();
			}
			
			static void Start()
			{
				if(READ_BIT(RCC->CFGR, RCC_CFGR_SWS_Pos) == RCC_CFGR_SWS_PLL)
				{
					return;
				}
				
				// enabling
				osc::Start();

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
			
			static void Stop()
			{
				// disabling
				osc::Stop();
			}
		};

		//------------------------------------------------------------------------
		template < typename OSC
					, uint32_t ClockMin_Hz, uint32_t ClockMax_Hz, bool maximal
					, ::mcu::clock::state::state state
					, bool pll_usb >
		class pll_cfg< ::mcu::clock::config::high_speed_external_bypass, OSC, ClockMin_Hz, ClockMax_Hz, maximal, state, pll_usb>
			: public pll_cfg< ::mcu::clock::config::high_speed_external, OSC, ClockMin_Hz, ClockMax_Hz, maximal, state, pll_usb>
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
		static void Init()
		{
			_cfg_::Init();
		}

		static void Enable(::mcu::clock::state::state new_state)
		{
			(new_state == ::mcu::clock::state::enable) ? Start() : Stop();
		}
			
		static void Start()
		{
			// enabling
			_cfg_::Start();
		}
		
		static void Stop()
		{
			// disabling
			_cfg_::Stop();
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
			
			STATIC_ASSERT(	osc::_cfg_::_OscType == ::mcu::clock::config::high_speed_internal			||
							osc::_cfg_::_OscType == ::mcu::clock::config::high_speed_external			||
							osc::_cfg_::_OscType == ::mcu::clock::config::high_speed_external_bypass
						, "Invalid source type for system clock");

			STATIC_ASSERT(HCLK_Max_Hz			>= limits::min::HCLK	/*&& HCLK_Max_Hz			<= limits::max::HCLK	*/ , "HCLK_Max_Hz			- Invalid parameter");
			STATIC_ASSERT(APB1CLK_Max_Hz		>= limits::min::APB1CLK	/*&& APB1CLK_Max_Hz			<= limits::max::APB1CLK	*/ , "APB1CLK_Max_Hz		- Invalid parameter");
			STATIC_ASSERT(APB2CLK_Max_Hz		>= limits::min::APB2CLK	/*&& APB2CLK_Max_Hz			<= limits::max::APB2CLK	*/ , "APB2CLK_Max_Hz		- Invalid parameter");
			STATIC_ASSERT(CortexSysTimer_Max_Hz	>= limits::min::HCLK	/*&& CortexSysTimer_Max_Hz	<= limits::max::HCLK	*/ , "CortexSysTimer_Max_Hz	- Invalid parameter");
			STATIC_ASSERT(ADCCLK_Max_Hz			>= limits::min::ADCCLK	/*&& ADCCLK_Max_Hz			<= limits::max::ADCCLK	*/ , "ADCCLK_Max_Hz			- Invalid parameter");
		
			STATIC_ASSERT(osc::_cfg_::_Clock_Hz >= limits::min::HCLK && osc::_cfg_::_Clock_Hz <= limits::max::HCLK, "Invalid source for system clock");

		public:
			static const ::mcu::clock::config::osc_type	_OscType		= osc::_cfg_::_OscType;
			static const uint32_t						_Clock_Hz		= osc::_cfg_::_Clock_Hz;
			static const ::mcu::clock::state::state		_State			= osc::_cfg_::_State;
			static const int32_t						_Calibration	= osc::_cfg_::_Calibration;
		
			static const uint32_t						_pll_prediv		= osc::_cfg_::_pll_prediv	;
			static const bool							_usb_div_1_5	= osc::_cfg_::_usb_div_1_5	;
			static const uint32_t						_pll_mul		= osc::_cfg_::_pll_mul		;
			static const uint32_t						_usb_clock		= osc::_cfg_::_usb_clock	;
			static const bool							_usb_active		= osc::_cfg_::_usb_active	;
		
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
				(_HCLK_Hz / 0xFFFFFFFF / 1 <= CortexSysTimer_Max_Hz) ? 1 :
				(_HCLK_Hz / 0xFFFFFFFF / 8 <= CortexSysTimer_Max_Hz) ? 8 :
				0;

			STATIC_ASSERT(_CortexSysTimer_div > 0, "The required clock frequency for CortexSysTimer cannot be reached");

			static const uint32_t	_CortexSysTimer_load	= _HCLK_Hz / CortexSysTimer_Max_Hz / _CortexSysTimer_div;
			
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
		};
		
	public:
	};
	
//////////////////////////////////////////////////////////////////////////
} // namespace clock
} // namespace stm32

/************************************************************************/
/*                                                                      */
/************************************************************************/


namespace clock {

template < state::state state, int32_t calibration = 0x7FFFFFFF, uint32_t Clock_Hz = HSI_VALUE >
class osc_hsi : public ::mcu::stm32::clock::osc< config::high_speed_internal, Clock_Hz, state, calibration> {};

template < state::state state, uint32_t Clock_Hz = LSI_VALUE >
class osc_lsi : public ::mcu::stm32::clock::osc< config::low_speed_internal, Clock_Hz, state > {};

template < state::state state, uint32_t Clock_Hz = HSE_VALUE >
class osc_hse : public ::mcu::stm32::clock::osc< config::high_speed_external, Clock_Hz, state > {};

template < state::state state, uint32_t Clock_Hz = HSE_VALUE >
class osc_hse_bypass : public ::mcu::stm32::clock::osc< config::high_speed_external_bypass, Clock_Hz, state > {};

template < state::state state, uint32_t Clock_Hz = LSE_VALUE >
class osc_lse : public ::mcu::stm32::clock::osc< config::low_speed_external, Clock_Hz, state > {};

template < state::state state, uint32_t Clock_Hz = LSE_VALUE >
class osc_lse_bypass : public ::mcu::stm32::clock::osc< config::low_speed_external_bypass, Clock_Hz, state > {};

//////////////////////////////////////////////////////////////////////////	
typedef osc_hsi			< state::enable > osc_hsi_def;
typedef osc_lsi			< state::enable > osc_lsi_def;
typedef osc_hse			< state::enable > osc_hse_def;
typedef osc_hse_bypass	< state::enable > osc_hse_bypass_def;
typedef osc_lse			< state::enable > osc_lse_def;
typedef osc_lse_bypass	< state::enable > osc_lse_bypass_def;

//////////////////////////////////////////////////////////////////////////	
typedef ::mcu::stm32::clock::pll_auto_range< osc_hsi_def,			::mcu::stm32::clock::limits::min::SYSCLK, ::mcu::stm32::clock::limits::max::SYSCLK, true, false > pll_hsi_def;
typedef ::mcu::stm32::clock::pll_auto_range< osc_hse_def,			::mcu::stm32::clock::limits::min::SYSCLK, ::mcu::stm32::clock::limits::max::SYSCLK, true, true  > pll_hse_def;
typedef ::mcu::stm32::clock::pll_auto_range< osc_hse_bypass_def,	::mcu::stm32::clock::limits::min::SYSCLK, ::mcu::stm32::clock::limits::max::SYSCLK, true, true  > pll_hse_bypass_def;

//------------------------------------------------------------------------
template< class OSC, uint32_t Clock_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
class pll_auto : public ::mcu::stm32::clock::pll_auto_range< OSC, Clock_Hz, Clock_Hz, true, true, state > {};

template< class OSC, uint32_t ClockMin_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, uint32_t ClockMax_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
class pll_min : public ::mcu::stm32::clock::pll_auto_range< OSC, ClockMin_Hz, ClockMax_Hz, false, false, state > {};

template< class OSC, uint32_t ClockMin_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, uint32_t ClockMax_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
class pll_max : public ::mcu::stm32::clock::pll_auto_range< OSC, ClockMin_Hz, ClockMax_Hz, true, false, state > {};

template< class OSC, uint32_t ClockMin_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, uint32_t ClockMax_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
class pll_usb_min : public ::mcu::stm32::clock::pll_auto_range< OSC, ClockMin_Hz, ClockMax_Hz, false, true, state > {};

template< class OSC, uint32_t ClockMin_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, uint32_t ClockMax_Hz = ::mcu::stm32::clock::limits::max::SYSCLK, state::state state = state::enable >
class pll_usb_max : public ::mcu::stm32::clock::pll_auto_range< OSC, ClockMin_Hz, ClockMax_Hz, true, true, state > {};


} // namespace clock

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__stm32f1xx_clock_hpp__*/
