#ifndef __stm32f1xx_gpio_hpp__
#define __stm32f1xx_gpio_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

#include <gpio.hpp>

//////////////////////////////////////////////////////////////////////////
#include <stm32f1xx.h>
#include <stm32f1xx_registers.hpp>


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {

/************************************************************************/
/*                                                                      */
/************************************************************************/
#if defined(GPIOA_BASE)
#	define IF_GPIOA_EXISTS(...)			__VA_ARGS__
#else /*defined(GPIOA_BASE)*/
#	define IF_GPIOA_EXISTS(...)			
#endif /*defined(GPIOA_BASE)*/

#if defined(GPIOB_BASE)
#	define IF_GPIOB_EXISTS(...)			__VA_ARGS__
#else /*defined(GPIOB_BASE)*/
#	define IF_GPIOB_EXISTS(...)			
#endif /*defined(GPIOB_BASE)*/

#if defined(GPIOC_BASE)
#	define IF_GPIOC_EXISTS(...)			__VA_ARGS__
#else /*defined(GPIOC_BASE)*/
#	define IF_GPIOC_EXISTS(...)			
#endif /*defined(GPIOC_BASE)*/

#if defined(GPIOD_BASE)
#	define IF_GPIOD_EXISTS(...)			__VA_ARGS__
#else /*defined(GPIOD_BASE)*/
#	define IF_GPIOD_EXISTS(...)			
#endif /*defined(GPIOD_BASE)*/

#if defined(GPIOE_BASE)
#	define IF_GPIOE_EXISTS(...)			__VA_ARGS__
#else /*defined(GPIOE_BASE)*/
#	define IF_GPIOE_EXISTS(...)			
#endif /*defined(GPIOE_BASE)*/

#if defined(GPIOF_BASE)
#	define IF_GPIOF_EXISTS(...)			__VA_ARGS__
#else /*defined(GPIOF_BASE)*/
#	define IF_GPIOF_EXISTS(...)			
#endif /*defined(GPIOF_BASE)*/

#if defined(GPIOG_BASE)
#	define IF_GPIOG_EXISTS(...)			__VA_ARGS__
#else /*defined(GPIOG_BASE)*/
#	define IF_GPIOG_EXISTS(...)			
#endif /*defined(GPIOG_BASE)*/

#if defined(GPIOH_BASE)
#	define IF_GPIOH_EXISTS(...)			__VA_ARGS__
#else /*defined(GPIOH_BASE)*/
#	define IF_GPIOH_EXISTS(...)			
#endif /*defined(GPIOH_BASE)*/

#if defined(GPIOI_BASE)
#	define IF_GPIOI_EXISTS(...)			__VA_ARGS__
#else /*defined(GPIOI_BASE)*/
#	define IF_GPIOI_EXISTS(...)			
#endif /*defined(GPIOI_BASE)*/

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace stm32 {
namespace port {
	//////////////////////////////////////////////////////////////////////////
	typedef enum
	{
		inv_address = 0,
		
		IF_GPIOA_EXISTS(A = GPIOA_BASE,)
		IF_GPIOB_EXISTS(B = GPIOB_BASE,)
		IF_GPIOC_EXISTS(C = GPIOC_BASE,)
		IF_GPIOD_EXISTS(D = GPIOD_BASE,)
		IF_GPIOE_EXISTS(E = GPIOE_BASE,)
		IF_GPIOF_EXISTS(F = GPIOF_BASE,)
		IF_GPIOG_EXISTS(G = GPIOG_BASE,)
		IF_GPIOH_EXISTS(H = GPIOH_BASE,)
		IF_GPIOI_EXISTS(I = GPIOI_BASE,)
	} address;

	typedef enum
	{
		inv_pin = 255,
		
		pin_0   = 0,
		pin_1,
		pin_2,
		pin_3,
		pin_4,
		pin_5,
		pin_6,
		pin_7,
		pin_8,
		pin_9,
		pin_10,
		pin_11,
		pin_12,
		pin_13,
		pin_14,
		pin_15,
	} pin;
	
	using namespace ::mcu::gpio;
	
	template<pin_id::pin_id PinID>
	class convert
	{
	public:
		static const address to_address =
			IF_GPIOA_EXISTS((PinID >= pin_id::PA0 && PinID <= pin_id::PA15) ? A :)
			IF_GPIOB_EXISTS((PinID >= pin_id::PB0 && PinID <= pin_id::PB15) ? B :)
			IF_GPIOC_EXISTS((PinID >= pin_id::PC0 && PinID <= pin_id::PC15) ? C :)
			IF_GPIOD_EXISTS((PinID >= pin_id::PD0 && PinID <= pin_id::PD15) ? D :)
			IF_GPIOE_EXISTS((PinID >= pin_id::PE0 && PinID <= pin_id::PE15) ? E :)
			IF_GPIOF_EXISTS((PinID >= pin_id::PF0 && PinID <= pin_id::PF15) ? F :)
			IF_GPIOG_EXISTS((PinID >= pin_id::PG0 && PinID <= pin_id::PG15) ? G :)
			IF_GPIOH_EXISTS((PinID >= pin_id::PH0 && PinID <= pin_id::PH15) ? H :)
			IF_GPIOI_EXISTS((PinID >= pin_id::PI0 && PinID <= pin_id::PI15) ? I :)
			inv_address;

		static const pin to_pin =
			IF_GPIOA_EXISTS((PinID >= pin_id::PA0 && PinID <= pin_id::PA15) ? static_cast<pin>(PinID % 32) :)
			IF_GPIOB_EXISTS((PinID >= pin_id::PB0 && PinID <= pin_id::PB15) ? static_cast<pin>(PinID % 32) :)
			IF_GPIOC_EXISTS((PinID >= pin_id::PC0 && PinID <= pin_id::PC15) ? static_cast<pin>(PinID % 32) :)
			IF_GPIOD_EXISTS((PinID >= pin_id::PD0 && PinID <= pin_id::PD15) ? static_cast<pin>(PinID % 32) :)
			IF_GPIOE_EXISTS((PinID >= pin_id::PE0 && PinID <= pin_id::PE15) ? static_cast<pin>(PinID % 32) :)
			IF_GPIOF_EXISTS((PinID >= pin_id::PF0 && PinID <= pin_id::PF15) ? static_cast<pin>(PinID % 32) :)
			IF_GPIOG_EXISTS((PinID >= pin_id::PG0 && PinID <= pin_id::PG15) ? static_cast<pin>(PinID % 32) :)
			IF_GPIOH_EXISTS((PinID >= pin_id::PH0 && PinID <= pin_id::PH15) ? static_cast<pin>(PinID % 32) :)
			IF_GPIOI_EXISTS((PinID >= pin_id::PI0 && PinID <= pin_id::PI15) ? static_cast<pin>(PinID % 32) :)
			inv_pin;
	};
} // namespace port

namespace registers {
	//////////////////////////////////////////////////////////////////////////
	using namespace ::mcu::gpio;
	
	//////////////////////////////////////////////////////////////////////////
	class _const_
	{
	public:
		typedef enum {
			/* Definitions for bit manipulation of CRL and CRH registers */
			GPIO_CR_MODE_INPUT         = ((uint32_t)0x0 << 0),			/*!< 00: Input mode (reset state)              */
			GPIO_CR_MODE_OUTPUT_10MHZ  = ((uint32_t)0x1 << 0),			/*!< 01: Output mode, max speed 10 MHz         */
			GPIO_CR_MODE_OUTPUT_02MHZ  = ((uint32_t)0x2 << 0),			/*!< 10: Output mode, max speed  2 MHz         */
			GPIO_CR_MODE_OUTPUT_50MHZ  = ((uint32_t)0x3 << 0),			/*!< 11: Output mode, max speed 50 MHz         */
			GPIO_CR_CNF_ANALOG         = ((uint32_t)0x0 << 2),			/*!< 00: Analog mode                           */
			GPIO_CR_CNF_INPUT_FLOATING = ((uint32_t)0x1 << 2),			/*!< 01: Floating input (reset state)          */
			GPIO_CR_CNF_INPUT_PU_PD    = ((uint32_t)0x2 << 2),			/*!< 10: Input with pull-up / pull-down        */
			GPIO_CR_CNF_GP_OUTPUT_PP   = ((uint32_t)0x0 << 2),			/*!< 00: General purpose output push-pull      */
			GPIO_CR_CNF_GP_OUTPUT_OD   = ((uint32_t)0x1 << 2),			/*!< 01: General purpose output Open-drain     */
			GPIO_CR_CNF_AF_OUTPUT_PP   = ((uint32_t)0x2 << 2),			/*!< 10: Alternate function output Push-pull   */
			GPIO_CR_CNF_AF_OUTPUT_OD   = ((uint32_t)0x3 << 2),			/*!< 11: Alternate function output Open-drain  */
		} REG_GPIO_CR;
	};
	
	//////////////////////////////////////////////////////////////////////////
	template <
		mode::mode			Mode      ,
		speed::speed		Speed     ,
		state::state		DefState  ,
		pull::pull			Pull      ,
		flag::flag			Flag      ,
		port::address		Port      ,
		port::pin			Pin       
		>
	class REG_GPIO;
	
	//////////////////////////////////////////////////////////////////////////
	template <
		speed::speed		Speed     ,
		state::state		DefState  ,
		pull::pull			Pull      ,
		flag::flag			Flag      ,
		port::address		Port      ,
		port::pin			Pin       
		>
	class REG_GPIO<mode::analog, Speed, DefState, Pull, Flag, Port, Pin>
	{
		/* If we are configuring the pin in INPUT analog mode */
		static const uint32_t MODE = _const_::GPIO_CR_MODE_INPUT;
		static const uint32_t CNF  = _const_::GPIO_CR_CNF_ANALOG;
		
	public:
		static const uint32_t CR = MODE | CNF;
		static const uint32_t ODR_MASK  = 0;
		static const uint32_t ODR       = 0;
		static const uint32_t ODR_SET   = ODR;
		static const uint32_t ODR_RESET = ODR;
	};
	
	//////////////////////////////////////////////////////////////////////////
	template <
		speed::speed		Speed     ,
		state::state		DefState  ,
		pull::pull			Pull      ,
		flag::flag			Flag      ,
		port::address		Port      ,
		port::pin			Pin       
		>
	class REG_GPIO<mode::input, Speed, DefState, Pull, Flag, Port, Pin>
	{
		static const uint32_t pull_up_dn = ((Pull == pull::up) || (Pull == pull::down)) ? 0x01UL : 0x00UL;
		static const uint32_t MODE = _const_::GPIO_CR_MODE_INPUT;
		static const uint32_t CNF  = pull_up_dn ? _const_::GPIO_CR_CNF_INPUT_PU_PD : _const_::GPIO_CR_CNF_INPUT_FLOATING;

	public:
		static const uint32_t CR = MODE | CNF;
		static const uint32_t ODR_MASK  = (pull_up_dn << Pin);
		static const uint32_t ODR       = (Pull == pull::up) ? (0x01UL << Pin) : (0x00UL << Pin);
		static const uint32_t ODR_SET   = ODR;
		static const uint32_t ODR_RESET = ODR;
	};
	
	//////////////////////////////////////////////////////////////////////////
	template <
		speed::speed		Speed     ,
		state::state		DefState  ,
		pull::pull			Pull      ,
		flag::flag			Flag      ,
		port::address		Port      ,
		port::pin			Pin       
		>
	class REG_GPIO<mode::output, Speed, DefState, Pull, Flag, Port, Pin>
	{
	private:
		static const uint32_t MODE =
			(Speed == speed::low   ) ? _const_::GPIO_CR_MODE_OUTPUT_02MHZ :
			(Speed == speed::medium) ? _const_::GPIO_CR_MODE_OUTPUT_10MHZ :
			(Speed == speed::high  ) ? _const_::GPIO_CR_MODE_OUTPUT_50MHZ :
			                           _const_::GPIO_CR_MODE_OUTPUT_02MHZ;
		
		static const uint32_t CNF = 
			(Flag == flag::output_push_pull ) ? _const_::GPIO_CR_CNF_GP_OUTPUT_PP : /* If we are configuring the pin in OUTPUT push-pull mode */
			(Flag == flag::output_open_drain) ? _const_::GPIO_CR_CNF_GP_OUTPUT_OD : /* If we are configuring the pin in OUTPUT open-drain mode */
												  _const_::GPIO_CR_CNF_GP_OUTPUT_PP;

	public:
		static const uint32_t CR = MODE | CNF;
		static const uint32_t ODR_MASK  = (0x01UL << Pin);
		static const uint32_t ODR       = (DefState != state::reset) ? (0x01UL << Pin) : (0x00UL << Pin);
		static const uint32_t ODR_SET   = (0x01UL << Pin);	/* Set the corresponding ODR bit */
		static const uint32_t ODR_RESET = (0x00UL << Pin);	/* Reset the corresponding ODR bit */
	};
	
	//////////////////////////////////////////////////////////////////////////
	template <
		speed::speed		Speed     ,
		state::state		DefState  ,
		pull::pull			Pull      ,
		flag::flag			Flag      ,
		port::address		Port      ,
		port::pin			Pin       
		>
	class REG_GPIO<mode::alt_input, Speed, DefState, Pull, Flag, Port, Pin>
	{
		static const uint32_t pull_up_dn = ((Pull == pull::up) || (Pull == pull::down)) ? 0x01UL : 0x00UL;
		static const uint32_t MODE = _const_::GPIO_CR_MODE_INPUT;
		static const uint32_t CNF  = pull_up_dn ? _const_::GPIO_CR_CNF_INPUT_PU_PD : _const_::GPIO_CR_CNF_INPUT_FLOATING;

	public:
		static const uint32_t CR = MODE | CNF;
		static const uint32_t ODR_MASK  = (pull_up_dn << Pin);
		static const uint32_t ODR       = (Pull == pull::up) ? (0x01UL << Pin) : (0x00UL << Pin);
		static const uint32_t ODR_SET   = ODR;
		static const uint32_t ODR_RESET = ODR;
	};
	
	//////////////////////////////////////////////////////////////////////////
	template <
		speed::speed		Speed     ,
		state::state		DefState  ,
		pull::pull			Pull      ,
		flag::flag			Flag      ,
		port::address		Port      ,
		port::pin			Pin       
		>
	class REG_GPIO<mode::alt_output, Speed, DefState, Pull, Flag, Port, Pin>
	{
	private:
		static const uint32_t MODE =
			(Speed == speed::low   ) ? _const_::GPIO_CR_MODE_OUTPUT_02MHZ :
			(Speed == speed::medium) ? _const_::GPIO_CR_MODE_OUTPUT_10MHZ :
			(Speed == speed::high  ) ? _const_::GPIO_CR_MODE_OUTPUT_50MHZ :
			                           _const_::GPIO_CR_MODE_OUTPUT_02MHZ;
		
		static const uint32_t CNF = 
			(Flag == flag::output_push_pull ) ? _const_::GPIO_CR_CNF_AF_OUTPUT_PP : /* If we are configuring the pin in OUTPUT push-pull mode */
			(Flag == flag::output_open_drain) ? _const_::GPIO_CR_CNF_AF_OUTPUT_OD : /* If we are configuring the pin in OUTPUT open-drain mode */
												_const_::GPIO_CR_CNF_AF_OUTPUT_PP;

	public:
		static const uint32_t CR = MODE | CNF;
		static const uint32_t ODR_MASK  = 0;
		static const uint32_t ODR       = 0;
		static const uint32_t ODR_SET   = ODR;
		static const uint32_t ODR_RESET = ODR;
	};
	
	//////////////////////////////////////////////////////////////////////////
	template <
		mode::mode	Mode      ,
		pin_id::pin_id	PinID
		> class REG_RCC
	{
	public:
		static const uint32_t APB2ENR_AFIOEN = (Mode == mode::alt_input || Mode == mode::alt_output) ? RCC_APB2ENR_AFIOEN : 0;
		
		static const uint32_t APB2ENR_MASK = APB2ENR_AFIOEN | (
			IF_GPIOA_EXISTS((PinID >= pin_id::PA0 && PinID <= pin_id::PA31) ? RCC_APB2ENR_IOPAEN :)
			IF_GPIOB_EXISTS((PinID >= pin_id::PB0 && PinID <= pin_id::PB31) ? RCC_APB2ENR_IOPBEN :)
			IF_GPIOC_EXISTS((PinID >= pin_id::PC0 && PinID <= pin_id::PC31) ? RCC_APB2ENR_IOPCEN :)
			IF_GPIOD_EXISTS((PinID >= pin_id::PD0 && PinID <= pin_id::PD31) ? RCC_APB2ENR_IOPDEN :)
			IF_GPIOE_EXISTS((PinID >= pin_id::PE0 && PinID <= pin_id::PE31) ? RCC_APB2ENR_IOPEEN :)
			IF_GPIOF_EXISTS((PinID >= pin_id::PF0 && PinID <= pin_id::PF31) ? RCC_APB2ENR_IOPFEN :)
			IF_GPIOG_EXISTS((PinID >= pin_id::PG0 && PinID <= pin_id::PG31) ? RCC_APB2ENR_IOPGEN :)
			IF_GPIOH_EXISTS((PinID >= pin_id::PH0 && PinID <= pin_id::PH31) ? RCC_APB2ENR_IOPHEN :)
			IF_GPIOI_EXISTS((PinID >= pin_id::PI0 && PinID <= pin_id::PI31) ? RCC_APB2ENR_IOPIEN :)
			0);
		static const uint32_t APB2ENR = APB2ENR_MASK;
	};

} // namespace registers
} // namespace stm32

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace gpio {
template <	pin_id::pin_id		PinID     ,
			mode::mode			Mode      ,
			speed::speed		Speed     ,
			state::state		DefState  ,
			pull::pull			Pull      ,
			flag::flag			Flag      
		>
class gpio_base
{
private:
	class _const_
	{
		friend class gpio_base;
		
		static const stm32::port::address PORT   = stm32::port::convert<PinID>::to_address;
		static const stm32::port::pin     PIN    = stm32::port::convert<PinID>::to_pin;
		
		//static const uint32_t ODR_BB      = FROM_ADDRESS_BIT_POS_TO_BB(&(((GPIO_TypeDef*)PORT)->ODR), PIN);
		//static const uint32_t IDR_BB      = FROM_ADDRESS_BIT_POS_TO_BB(&(((GPIO_TypeDef*)PORT)->IDR), PIN);
		
		static const uint32_t GPIO_OFFSET = ((uint32_t)PORT - PERIPH_BASE);
		static const uint32_t ODR_BB      = PERIPH_BB_BASE + (GPIO_OFFSET + ((uint32_t)(&((GPIO_TypeDef*)0)->ODR))) * 32 + (PIN) * 4;
		static const uint32_t IDR_BB      = PERIPH_BB_BASE + (GPIO_OFFSET + ((uint32_t)(&((GPIO_TypeDef*)0)->IDR))) * 32 + (PIN) * 4;
	};
	
protected:
	gpio_base();
	~gpio_base();

public:
	class _cfg_
	{
	public:
		static const pin_id::pin_id			_PinID			= PinID;
		static const stm32::port::address	_Port			= _const_::PORT;
		static const stm32::port::pin		_Pin			= _const_::PIN;
		static const mode::mode				_Mode			= Mode;
		static const speed::speed			_Speed			= Speed;
		static const state::state			_DefPinState	= DefState;
		static const pull::pull				_Pull			= Pull;
		static const flag::flag				_Flag			= Flag;

	private:
		STATIC_ASSERT(FAIL_IF(_PinID != pin_id::invalid && _Port == stm32::port::inv_address), "The MCU has not defined port:pin configuration");
		STATIC_ASSERT(FAIL_IF(_PinID != pin_id::invalid && _Pin  == stm32::port::inv_pin    ), "The MCU has not defined port:pin configuration");

		static const bool verified = config::check_params<_Mode, _Speed, _DefPinState, _Pull, _Flag>::verified;

		typedef stm32::registers::REG_GPIO<_Mode, _Speed, _DefPinState, _Pull, _Flag, _Port, _Pin> REG_GPIO;
		typedef stm32::registers::REG_RCC<_Mode, _PinID> REG_RCC;
	
	public:
		static const uint32_t _cr = REG_GPIO::CR;

		static const uint32_t _odr_mask  = REG_GPIO::ODR_MASK;
		static const uint32_t _odr_val   = REG_GPIO::ODR;
		static const uint32_t _odr_set   = REG_GPIO::ODR_SET;
		static const uint32_t _odr_reset = REG_GPIO::ODR_RESET;

		static const uint32_t _rcc_apb2enr_mask = REG_RCC::APB2ENR_MASK;
		static const uint32_t _rcc_apb2enr      = REG_RCC::APB2ENR;
	};
	
public:
	static void init()
	{
		// Enable peripheral clock
		if(_cfg_::_rcc_apb2enr_mask)
		{
			SET_BIT (RCC->APB2ENR, _cfg_::_rcc_apb2enr);
			__NOP();
		}
		
		update();

		/*--------------------- EXTI Mode Configuration ------------------------*/
		/* Configure the External Interrupt or event for the current IO */
//		if(_cfg_::_Mode == mode::input && (_cfg_::_Flag & flag::flags_input) != 0)
//		{
//			uint32_t temp;
//			
//			/* Enable AFIO Clock */
//			__HAL_RCC_AFIO_CLK_ENABLE();
//			temp = AFIO->EXTICR[_cfg_::_Pin >> 2];
//			CLEAR_BIT(temp, ((uint32_t)0x0F) << (4 * (_cfg_::_Pin & 0x03)));
//			SET_BIT(temp, (GPIO_GET_INDEX(((GPIO_TypeDef*)_cfg_::_Port))) << (4 * (_cfg_::_Pin & 0x03)));
//			AFIO->EXTICR[_cfg_::_Pin >> 2] = temp;


//			/* Configure the interrupt mask */
//			if((_cfg_::_Mode & _const_::GPIO_MODE_IT) == _const_::GPIO_MODE_IT)
//			{
//				SET_BIT(EXTI->IMR, (0x01UL << _cfg_::_Pin)); 
//			} 
//			else
//			{
//				CLEAR_BIT(EXTI->IMR, (0x01UL << _cfg_::_Pin)); 
//			} 

//			/* Configure the event mask */
//			if((_cfg_::_Mode & _const_::GPIO_MODE_EVT) == _const_::GPIO_MODE_EVT)
//			{
//				SET_BIT(EXTI->EMR, (0x01UL << _cfg_::_Pin)); 
//			} 
//			else
//			{
//				CLEAR_BIT(EXTI->EMR, (0x01UL << _cfg_::_Pin)); 
//			}

//			/* Enable or disable the rising trigger */
//			if((_cfg_::_Mode & _const_::RISING_EDGE) == _const_::RISING_EDGE)
//			{
//				SET_BIT(EXTI->RTSR, (0x01UL << _cfg_::_Pin)); 
//			} 
//			else
//			{
//				CLEAR_BIT(EXTI->RTSR, (0x01UL << _cfg_::_Pin)); 
//			}

//			/* Enable or disable the falling trigger */
//			if((_cfg_::_Mode & _const_::FALLING_EDGE) == _const_::FALLING_EDGE)
//			{
//				SET_BIT(EXTI->FTSR, (0x01UL << _cfg_::_Pin)); 
//			} 
//			else
//			{
//				CLEAR_BIT(EXTI->FTSR, (0x01UL << _cfg_::_Pin)); 
//			}
//		}
	}
	
	static void update()
	{
		// Update ODR (pu/pd for input; state for output)
		(_cfg_::_odr_mask) ?
			(
				(_cfg_::_odr_val) ?
					(void)(((GPIO_TypeDef*)_cfg_::_Port)->BSRR = _cfg_::_odr_mask) :
					(void)(((GPIO_TypeDef*)_cfg_::_Port)->BRR  = _cfg_::_odr_mask)
			) :
			(void)(0);

		// configure port
		(_cfg_::_Pin < stm32::port::pin_8) ?
			(void)(MODIFY_REG(((GPIO_TypeDef*)_cfg_::_Port)->CRL,
						((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << ((_cfg_::_Pin - 0) << 2)),
						(_cfg_::_cr << ((_cfg_::_Pin - 0) << 2)))) :
			(void)(MODIFY_REG(((GPIO_TypeDef*)_cfg_::_Port)->CRH,
						((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << ((_cfg_::_Pin - 8) << 2)),
						(_cfg_::_cr << ((_cfg_::_Pin - 8) << 2))));
	}
	
	static bool get()
	{
		// accessible at any time
		return IS_BB_REG_SET(_const_::IDR_BB);
	}
	static bool get_out()
	{
		STATIC_ASSERT(_cfg_::_Mode == mode::output, "Accessible in OUTPUT mode");
		return IS_BB_REG_SET(_const_::ODR_BB);
	}
	static void set()
	{
		STATIC_ASSERT(_cfg_::_Mode == mode::output, "Accessible in OUTPUT mode");
		SET_BB_REG(_const_::ODR_BB);
	}
	static void reset()
	{
		STATIC_ASSERT(_cfg_::_Mode == mode::output, "Accessible in OUTPUT mode");
		RESET_BB_REG(_const_::ODR_BB);
	}

	static void write(bool val)
	{
		STATIC_ASSERT(_cfg_::_Mode == mode::output, "Accessible in OUTPUT mode");
		WRITE_BB_REG(_const_::ODR_BB, val);
	}
	static bool read()
	{
		// accessible at any time
		return IS_BB_REG_SET(_const_::IDR_BB);
	}
	
	template <state::state NewState>
	class _write_ : public gpio_base<_cfg_::_PinID, _cfg_::_Mode, _cfg_::_Speed, NewState    , _cfg_::_Pull, _cfg_::_Flag>
	{ STATIC_ASSERT(_cfg_::_Mode == mode::output, "Accessible in OUTPUT mode"); };
	
	class _set_   : public gpio_base<_cfg_::_PinID, _cfg_::_Mode, _cfg_::_Speed, state::set  , _cfg_::_Pull, _cfg_::_Flag>
	{ STATIC_ASSERT(_cfg_::_Mode == mode::output, "Accessible in OUTPUT mode"); };
	
	class _reset_ : public gpio_base<_cfg_::_PinID, _cfg_::_Mode, _cfg_::_Speed, state::reset, _cfg_::_Pull, _cfg_::_Flag>
	{ STATIC_ASSERT(_cfg_::_Mode == mode::output, "Accessible in OUTPUT mode"); };
};

/************************************************************************/
/*                                                                      */
/************************************************************************/
template <	pin_id::pin_id		PinID
		>
class analog : public gpio_base<PinID, mode::analog, speed::low, state::reset, pull::no, flag::flag_no> { };

/************************************************************************/
/*                                                                      */
/************************************************************************/
template <	pin_id::pin_id		PinID,
			pull::pull			Pull      = pull::no
		>
class input : public gpio_base<PinID, mode::input, speed::low, state::reset, Pull, flag::flag_no> { };

/************************************************************************/
/*                                                                      */
/************************************************************************/
template <	pin_id::pin_id		PinID,
			state::state		DefState  = state::reset,
			speed::speed		Speed     = speed::low,
			bool				OpenDrain = false
		>
class output : public gpio_base<PinID, mode::output, Speed, DefState, pull::no, OpenDrain ? flag::output_open_drain : flag::output_push_pull> { };

/************************************************************************/
/*                                                                      */
/************************************************************************/
template <	pin_id::pin_id		PinID,
			pull::pull			Pull      = pull::no
		>
class alt_input : public gpio_base<PinID, mode::alt_input, speed::low, state::reset, Pull, flag::flag_no> { };

/************************************************************************/
/*                                                                      */
/************************************************************************/
template <	pin_id::pin_id		PinID,
			speed::speed		Speed     = speed::low,
			bool				OpenDrain = false
		>
class alt_output : public gpio_base<PinID, mode::alt_output, Speed, state::reset, pull::no, OpenDrain ? flag::alt_output_open_drain : flag::alt_output_push_pull> { };

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace outputs {
	template <	class Port,
				state::state State
			>
	class write
	{
	private:
		class _const_
		{
			friend class gpio_base;
			
			static const stm32::port::address PORT   = Port::_cfg_::_Port;
			static const stm32::port::pin     PIN    = Port::_cfg_::_Pin;
			
			static const uint32_t GPIO_OFFSET = ((uint32_t)PORT - PERIPH_BASE);
			static const uint32_t ODR_BB      = PERIPH_BB_BASE + (GPIO_OFFSET + ((uint32_t)(&((GPIO_TypeDef*)0)->ODR))) * 32 + (PIN) * 4;
			static const uint32_t IDR_BB      = PERIPH_BB_BASE + (GPIO_OFFSET + ((uint32_t)(&((GPIO_TypeDef*)0)->IDR))) * 32 + (PIN) * 4;
		};
		
	public:
		class _cfg_
		{
		public:
			static const pin_id::pin_id			_PinID			= Port::_cfg_::_PinID;
			static const stm32::port::address	_Port			= Port::_cfg_::_Port;
			static const stm32::port::pin		_Pin			= Port::_cfg_::_Pin;
			static const mode::mode			_Mode			= Port::_cfg_::_Mode;
			static const speed::speed			_Speed			= Port::_cfg_::_Speed;
			static const state::state			_DefPinState	= State;
			static const pull::pull				_Pull			= Port::_cfg_::_Pull;
			static const flag::flag				_Flag			= Port::_cfg_::_Flag;

		private:
			STATIC_ASSERT(FAIL_IF(_PinID != pin_id::invalid && _Port == stm32::port::inv_address), "The MCU has not defined port:pin configuration");
			STATIC_ASSERT(FAIL_IF(_PinID != pin_id::invalid && _Pin  == stm32::port::inv_pin    ), "The MCU has not defined port:pin configuration");

			static const bool verified = config::check_params<_Mode, _Speed, _DefPinState, _Pull, _Flag>::verified;

			typedef stm32::registers::REG_GPIO<_Mode, _Speed, _DefPinState, _Pull, _Flag, _Port, _Pin> REG_GPIO;
			typedef stm32::registers::REG_RCC<_Mode, _PinID> REG_RCC;
		
		public:
			static const uint32_t _cr = REG_GPIO::CR;

			static const uint32_t _odr_mask  = REG_GPIO::ODR_MASK;
			static const uint32_t _odr_val   = REG_GPIO::ODR;
			static const uint32_t _odr_set   = REG_GPIO::ODR_SET;
			static const uint32_t _odr_reset = REG_GPIO::ODR_RESET;
			
			static const uint32_t _rcc_apb2enr_mask = REG_RCC::APB2ENR_MASK;
			static const uint32_t _rcc_apb2enr      = REG_RCC::APB2ENR;
		};
	};

} // namespace outputs


/************************************************************************/
/*                                                                      */
/************************************************************************/
class gpio_dummy
{
public:
	class _cfg_
	{
	public:
		static const pin_id::pin_id			_PinID			= pin_id::invalid;
		static const stm32::port::address	_Port			= stm32::port::inv_address;
		static const stm32::port::pin		_Pin			= stm32::port::inv_pin;
		static const mode::mode				_Mode			= mode::analog;
		static const speed::speed			_Speed			= speed::low;
		static const state::state			_DefPinState	= state::reset;
		static const pull::pull				_Pull			= pull::no;
		static const flag::flag				_Flag			= flag::flag_no;

		static const uint32_t _cr = 0;
	
		static const uint32_t _odr_mask		= 0;
		static const uint32_t _odr_val		= 0;
		static const uint32_t _odr_set		= 0;
		static const uint32_t _odr_reset	= 0;
	
		static const uint32_t _rcc_apb2enr_mask	= 0;
		static const uint32_t _rcc_apb2enr		= 0;
	};
};

typedef gpio_dummy gpio_invalid;

/************************************************************************/
/*                                                                      */
/************************************************************************/
template <class p00 = gpio_dummy, class p01 = gpio_dummy, class p02 = gpio_dummy, class p03 = gpio_dummy,
		  class p04 = gpio_dummy, class p05 = gpio_dummy, class p06 = gpio_dummy, class p07 = gpio_dummy,
		  class p08 = gpio_dummy, class p09 = gpio_dummy, class p10 = gpio_dummy, class p11 = gpio_dummy,
		  class p12 = gpio_dummy, class p13 = gpio_dummy, class p14 = gpio_dummy, class p15 = gpio_dummy,
		  class p16 = gpio_dummy, class p17 = gpio_dummy, class p18 = gpio_dummy, class p19 = gpio_dummy,
		  class p20 = gpio_dummy, class p21 = gpio_dummy, class p22 = gpio_dummy, class p23 = gpio_dummy,
		  class p24 = gpio_dummy, class p25 = gpio_dummy, class p26 = gpio_dummy, class p27 = gpio_dummy,
		  class p28 = gpio_dummy, class p29 = gpio_dummy, class p30 = gpio_dummy, class p31 = gpio_dummy,
		  class p32 = gpio_dummy, class p33 = gpio_dummy, class p34 = gpio_dummy, class p35 = gpio_dummy,
		  class p36 = gpio_dummy, class p37 = gpio_dummy, class p38 = gpio_dummy, class p39 = gpio_dummy>
class atomic
{
#define ASSEMBLE_EXP(exp, ...)				__ASSEM_##exp##(p00, __VA_ARGS__) __ASSEM_##exp##(p01, __VA_ARGS__) \
											__ASSEM_##exp##(p02, __VA_ARGS__) __ASSEM_##exp##(p03, __VA_ARGS__) \
											__ASSEM_##exp##(p04, __VA_ARGS__) __ASSEM_##exp##(p05, __VA_ARGS__) \
											__ASSEM_##exp##(p06, __VA_ARGS__) __ASSEM_##exp##(p07, __VA_ARGS__) \
											__ASSEM_##exp##(p08, __VA_ARGS__) __ASSEM_##exp##(p09, __VA_ARGS__) \
											__ASSEM_##exp##(p10, __VA_ARGS__) __ASSEM_##exp##(p11, __VA_ARGS__) \
											__ASSEM_##exp##(p12, __VA_ARGS__) __ASSEM_##exp##(p13, __VA_ARGS__) \
											__ASSEM_##exp##(p14, __VA_ARGS__) __ASSEM_##exp##(p15, __VA_ARGS__) \
											__ASSEM_##exp##(p16, __VA_ARGS__) __ASSEM_##exp##(p17, __VA_ARGS__) \
											__ASSEM_##exp##(p18, __VA_ARGS__) __ASSEM_##exp##(p19, __VA_ARGS__) \
											__ASSEM_##exp##(p20, __VA_ARGS__) __ASSEM_##exp##(p21, __VA_ARGS__) \
											__ASSEM_##exp##(p22, __VA_ARGS__) __ASSEM_##exp##(p23, __VA_ARGS__) \
											__ASSEM_##exp##(p24, __VA_ARGS__) __ASSEM_##exp##(p25, __VA_ARGS__) \
											__ASSEM_##exp##(p26, __VA_ARGS__) __ASSEM_##exp##(p27, __VA_ARGS__) \
											__ASSEM_##exp##(p28, __VA_ARGS__) __ASSEM_##exp##(p29, __VA_ARGS__) \
											__ASSEM_##exp##(p30, __VA_ARGS__) __ASSEM_##exp##(p31, __VA_ARGS__) \
											__ASSEM_##exp##(p32, __VA_ARGS__) __ASSEM_##exp##(p33, __VA_ARGS__) \
											__ASSEM_##exp##(p34, __VA_ARGS__) __ASSEM_##exp##(p35, __VA_ARGS__) \
											__ASSEM_##exp##(p36, __VA_ARGS__) __ASSEM_##exp##(p37, __VA_ARGS__) \
											__ASSEM_##exp##(p38, __VA_ARGS__) __ASSEM_##exp##(p39, __VA_ARGS__) \

#	define __ASSEM_ALL(type, member)		(type::member) |
#define ASSEMBLE_ALL(member)				(ASSEMBLE_EXP(ALL, member) 0)

#	define __ASSEM_IF(type, port, member)	(port == type::_cfg_::_Port ? type::member : 0) |
#define ASSEMBLE_IF(port, member)			(ASSEMBLE_EXP(IF, port, member) 0)
	
#	define __ASSEM_CRL_MASK(type, p)		((p == type::_cfg_::_Port && type::_cfg_::_Pin < stm32::port::pin_8) ? ((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << ((type::_cfg_::_Pin - 0) << 2)) : 0) |
#define ASSEMBLE_CRL_MASK(port)				(ASSEMBLE_EXP(CRL_MASK, port) 0)

#	define __ASSEM_CRL(type, p)				((p == type::_cfg_::_Port && type::_cfg_::_Pin < stm32::port::pin_8) ? (type::_cfg_::_cr << ((type::_cfg_::_Pin - 0) << 2)) : 0) |
#define ASSEMBLE_CRL(port)					(ASSEMBLE_EXP(CRL, port) 0)

#	define __ASSEM_CRH_MASK(type, p)		((p == type::_cfg_::_Port && type::_cfg_::_Pin >= stm32::port::pin_8) ? ((GPIO_CRH_MODE8 | GPIO_CRH_CNF8) << ((type::_cfg_::_Pin - 8) << 2)) : 0) |
#define ASSEMBLE_CRH_MASK(port)				(ASSEMBLE_EXP(CRH_MASK, port) 0)

#	define __ASSEM_CRH(type, p)				((p == type::_cfg_::_Port && type::_cfg_::_Pin >= stm32::port::pin_8) ? (type::_cfg_::_cr << ((type::_cfg_::_Pin - 8) << 2)) : 0) |
#define ASSEMBLE_CRH(port)					(ASSEMBLE_EXP(CRH, port) 0)

public:
	static void init()
	{
		static const uint32_t apb2enr_mask = ASSEMBLE_ALL(_cfg_::_rcc_apb2enr_mask);
		static const uint32_t apb2enr = ASSEMBLE_ALL(_cfg_::_rcc_apb2enr);

		// enable GPIO clock
		if(apb2enr_mask)
		{
			SET_BIT(RCC->APB2ENR, apb2enr);
			__NOP();
		}
		
#define __INIT_PORT(PORT)		\
		do { \
			static const uint32_t odr_mask = ASSEMBLE_IF(stm32::port::PORT, _cfg_::_odr_mask); \
			static const uint32_t odr_val  = odr_mask & ASSEMBLE_ALL(_cfg_::_odr_val); \
			static const uint32_t crl_mask = ASSEMBLE_CRL_MASK(stm32::port::PORT); \
			static const uint32_t crl_val  = crl_mask & ASSEMBLE_CRL(stm32::port::PORT); \
			static const uint32_t crh_mask = ASSEMBLE_CRH_MASK(stm32::port::PORT); \
			static const uint32_t crh_val  = crh_mask & ASSEMBLE_CRH(stm32::port::PORT); \
			\
			if(odr_mask) /* initialize Output state */ \
				MODIFY_REG(GPIO##PORT->ODR, odr_mask, odr_val); \
			if(crl_mask) /* initialize mode */ \
				MODIFY_REG(GPIO##PORT->CRL, crl_mask, crl_val); \
			if(crh_mask) /* initialize mode */ \
				MODIFY_REG(GPIO##PORT->CRH, crh_mask, crh_val); \
			/* TODO: EXTI Mode Configuration */ \
		} while(0)

		IF_GPIOA_EXISTS(__INIT_PORT(A);)
		IF_GPIOB_EXISTS(__INIT_PORT(B);)
		IF_GPIOC_EXISTS(__INIT_PORT(C);)
		IF_GPIOD_EXISTS(__INIT_PORT(D);)
		IF_GPIOE_EXISTS(__INIT_PORT(E);)
		IF_GPIOF_EXISTS(__INIT_PORT(F);)
		IF_GPIOG_EXISTS(__INIT_PORT(G);)
		IF_GPIOH_EXISTS(__INIT_PORT(H);)
		IF_GPIOI_EXISTS(__INIT_PORT(I);)
#undef __INIT_PORT

	}
	
	static void update()
	{
#define __INIT_PORT(PORT)		\
		do { \
			static const uint32_t odr_mask = ASSEMBLE_IF(stm32::port::PORT, _cfg_::_odr_mask); \
			static const uint32_t odr_val  = odr_mask & ASSEMBLE_ALL(_cfg_::_odr_val); \
			static const uint32_t crl_mask = ASSEMBLE_CRL_MASK(stm32::port::PORT); \
			static const uint32_t crl_val  = crl_mask & ASSEMBLE_CRL(stm32::port::PORT); \
			static const uint32_t crh_mask = ASSEMBLE_CRH_MASK(stm32::port::PORT); \
			static const uint32_t crh_val  = crh_mask & ASSEMBLE_CRH(stm32::port::PORT); \
			\
			if(odr_mask) /* initialize Output state */ \
				MODIFY_REG(GPIO##PORT->ODR, odr_mask, odr_val); \
			if(crl_mask) /* initialize mode */ \
				MODIFY_REG(GPIO##PORT->CRL, crl_mask, crl_val); \
			if(crh_mask) /* initialize mode */ \
				MODIFY_REG(GPIO##PORT->CRH, crh_mask, crh_val); \
		} while(0)

		IF_GPIOA_EXISTS(__INIT_PORT(A);)
		IF_GPIOB_EXISTS(__INIT_PORT(B);)
		IF_GPIOC_EXISTS(__INIT_PORT(C);)
		IF_GPIOD_EXISTS(__INIT_PORT(D);)
		IF_GPIOE_EXISTS(__INIT_PORT(E);)
		IF_GPIOF_EXISTS(__INIT_PORT(F);)
		IF_GPIOG_EXISTS(__INIT_PORT(G);)
		IF_GPIOH_EXISTS(__INIT_PORT(H);)
		IF_GPIOI_EXISTS(__INIT_PORT(I);)
#undef __INIT_PORT

	}

	static void write()
	{
#define __INIT_PORT(PORT)		\
		do { \
			static const uint32_t odr_mask = ASSEMBLE_IF(stm32::port::PORT, _cfg_::_odr_mask); \
			static const uint32_t odr_val  = odr_mask & ASSEMBLE_ALL(_cfg_::_odr_val); \
			\
			if(odr_mask) /* initialize Output state */ \
				MODIFY_REG(GPIO##PORT->ODR, odr_mask, odr_val); \
		} while(0)

		IF_GPIOA_EXISTS(__INIT_PORT(A);)
		IF_GPIOB_EXISTS(__INIT_PORT(B);)
		IF_GPIOC_EXISTS(__INIT_PORT(C);)
		IF_GPIOD_EXISTS(__INIT_PORT(D);)
		IF_GPIOE_EXISTS(__INIT_PORT(E);)
		IF_GPIOF_EXISTS(__INIT_PORT(F);)
		IF_GPIOG_EXISTS(__INIT_PORT(G);)
		IF_GPIOH_EXISTS(__INIT_PORT(H);)
		IF_GPIOI_EXISTS(__INIT_PORT(I);)
#undef __INIT_PORT

	}

	static void set()
	{
#define __INIT_PORT(PORT)		\
		do { \
			static const uint32_t odr_mask = ASSEMBLE_IF(stm32::port::PORT, _cfg_::_odr_mask); \
			/*static const uint32_t odr_set  = odr_mask & ASSEMBLE_ALL(_cfg_::_odr_set);*/ \
			\
			if(odr_mask) \
				WRITE_REG(GPIO##PORT->BSRR, odr_mask); \
		} while(0)

		IF_GPIOA_EXISTS(__INIT_PORT(A);)
		IF_GPIOB_EXISTS(__INIT_PORT(B);)
		IF_GPIOC_EXISTS(__INIT_PORT(C);)
		IF_GPIOD_EXISTS(__INIT_PORT(D);)
		IF_GPIOE_EXISTS(__INIT_PORT(E);)
		IF_GPIOF_EXISTS(__INIT_PORT(F);)
		IF_GPIOG_EXISTS(__INIT_PORT(G);)
		IF_GPIOH_EXISTS(__INIT_PORT(H);)
		IF_GPIOI_EXISTS(__INIT_PORT(I);)
#undef __INIT_PORT

	}

	static void reset()
	{
#define __INIT_PORT(PORT)		\
		do { \
			static const uint32_t odr_mask = ASSEMBLE_IF(stm32::port::PORT, _cfg_::_odr_mask); \
			/*static const uint32_t odr_reset  = odr_mask & ASSEMBLE_ALL(_cfg_::_odr_reset);*/ \
			\
			if(odr_mask) \
				WRITE_REG(GPIO##PORT->BRR, odr_mask); \
		} while(0)

		IF_GPIOA_EXISTS(__INIT_PORT(A);)
		IF_GPIOB_EXISTS(__INIT_PORT(B);)
		IF_GPIOC_EXISTS(__INIT_PORT(C);)
		IF_GPIOD_EXISTS(__INIT_PORT(D);)
		IF_GPIOE_EXISTS(__INIT_PORT(E);)
		IF_GPIOF_EXISTS(__INIT_PORT(F);)
		IF_GPIOG_EXISTS(__INIT_PORT(G);)
		IF_GPIOH_EXISTS(__INIT_PORT(H);)
		IF_GPIOI_EXISTS(__INIT_PORT(I);)
#undef __INIT_PORT

	}
};

} // namespace gpio
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__stm32f1xx_gpio_hpp__*/
