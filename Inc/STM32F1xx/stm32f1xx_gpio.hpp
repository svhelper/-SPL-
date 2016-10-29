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
#include "CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"
#include "stm32f1xx/stm32f1xx_registers.hpp"
#include "stm32f1xx/stm32f1xx_adc.hpp"


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
		bool						valid     ,
		::mcu::gpio::mode::mode		Mode      ,
		::mcu::gpio::speed::speed	Speed     ,
		::mcu::gpio::state::state	DefState  ,
		::mcu::gpio::pull::pull		Pull      ,
		::mcu::gpio::flag::flag		Flag      ,
		port::address				Port      ,
		port::pin					Pin       
		>
	class REG_GPIO;

	//------------------------------------------------------------------------
	template <
		::mcu::gpio::mode::mode		Mode      ,
		::mcu::gpio::speed::speed	Speed     ,
		::mcu::gpio::state::state	DefState  ,
		::mcu::gpio::pull::pull		Pull      ,
		::mcu::gpio::flag::flag		Flag      ,
		port::address				Port      ,
		port::pin					Pin       
		>
	class REG_GPIO< false, Mode, Speed, DefState, Pull, Flag, Port, Pin >
	{
	public:
		static const uint32_t CR		= 0;
		static const uint32_t CRL_MASK	= 0;
		static const uint32_t CRL		= 0;
		static const uint32_t CRH_MASK	= 0;
		static const uint32_t CRH		= 0;
		static const uint32_t ODR_MASK  = 0;
		static const uint32_t ODR       = 0;
		static const uint32_t ODR_SET   = 0;
		static const uint32_t ODR_RESET = 0;
	};
	
	//------------------------------------------------------------------------
	template <
		::mcu::gpio::speed::speed	Speed     ,
		::mcu::gpio::state::state	DefState  ,
		::mcu::gpio::pull::pull		Pull      ,
		::mcu::gpio::flag::flag		Flag      ,
		port::address				Port      ,
		port::pin					Pin       
		>
	class REG_GPIO< true, ::mcu::gpio::mode::analog, Speed, DefState, Pull, Flag, Port, Pin >
	{
		/* If we are configuring the pin in INPUT analog mode */
		static const uint32_t MODE = _const_::GPIO_CR_MODE_INPUT;
		static const uint32_t CNF  = _const_::GPIO_CR_CNF_ANALOG;
		
	public:
		static const uint32_t CR = MODE | CNF;
		static const uint32_t CRL_MASK	= ((Pin <  port::pin_8) ? (GPIO_CRL_MODE0 | GPIO_CRL_CNF0) : 0) << ((Pin % 8) << 2);
		static const uint32_t CRL		= ((Pin <  port::pin_8) ? (CR)                             : 0) << ((Pin % 8) << 2);
		static const uint32_t CRH_MASK	= ((Pin >= port::pin_8) ? (GPIO_CRL_MODE0 | GPIO_CRL_CNF0) : 0) << ((Pin % 8) << 2);
		static const uint32_t CRH		= ((Pin >= port::pin_8) ? (CR)                             : 0) << ((Pin % 8) << 2);
		static const uint32_t ODR_MASK  = 0;
		static const uint32_t ODR       = 0;
		static const uint32_t ODR_SET   = ODR;
		static const uint32_t ODR_RESET = ODR;
	};
	
	//------------------------------------------------------------------------
	template <
		::mcu::gpio::speed::speed	Speed     ,
		::mcu::gpio::state::state	DefState  ,
		::mcu::gpio::pull::pull		Pull      ,
		::mcu::gpio::flag::flag		Flag      ,
		port::address				Port      ,
		port::pin					Pin       
		>
	class REG_GPIO< true, ::mcu::gpio::mode::input, Speed, DefState, Pull, Flag, Port, Pin >
	{
		static const uint32_t pull_up_dn = ((Pull == pull::up) || (Pull == pull::down)) ? 0x01UL : 0x00UL;
		static const uint32_t MODE = _const_::GPIO_CR_MODE_INPUT;
		static const uint32_t CNF  = pull_up_dn ? _const_::GPIO_CR_CNF_INPUT_PU_PD : _const_::GPIO_CR_CNF_INPUT_FLOATING;

	public:
		static const uint32_t CR = MODE | CNF;
		static const uint32_t CRL_MASK	= ((Pin <  port::pin_8) ? (GPIO_CRL_MODE0 | GPIO_CRL_CNF0) : 0) << ((Pin % 8) << 2);
		static const uint32_t CRL		= ((Pin <  port::pin_8) ? (CR)                             : 0) << ((Pin % 8) << 2);
		static const uint32_t CRH_MASK	= ((Pin >= port::pin_8) ? (GPIO_CRL_MODE0 | GPIO_CRL_CNF0) : 0) << ((Pin % 8) << 2);
		static const uint32_t CRH		= ((Pin >= port::pin_8) ? (CR)                             : 0) << ((Pin % 8) << 2);
		static const uint32_t ODR_MASK  = (Port == port::inv_address || Pin == port::inv_pin) ? 0 : (pull_up_dn << Pin);
		static const uint32_t ODR       = (Port == port::inv_address || Pin == port::inv_pin) ? 0 : (Pull == pull::up) ? (0x01UL << Pin) : (0x00UL << Pin);
		static const uint32_t ODR_SET   = ODR;
		static const uint32_t ODR_RESET = ODR;
	};
	
	//------------------------------------------------------------------------
	template <
		::mcu::gpio::speed::speed	Speed     ,
		::mcu::gpio::state::state	DefState  ,
		::mcu::gpio::pull::pull		Pull      ,
		::mcu::gpio::flag::flag		Flag      ,
		port::address				Port      ,
		port::pin					Pin       
		>
	class REG_GPIO< true, ::mcu::gpio::mode::output, Speed, DefState, Pull, Flag, Port, Pin >
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
		static const uint32_t CRL_MASK	= ((Pin <  port::pin_8) ? (GPIO_CRL_MODE0 | GPIO_CRL_CNF0) : 0) << ((Pin % 8) << 2);
		static const uint32_t CRL		= ((Pin <  port::pin_8) ? (CR)                             : 0) << ((Pin % 8) << 2);
		static const uint32_t CRH_MASK	= ((Pin >= port::pin_8) ? (GPIO_CRL_MODE0 | GPIO_CRL_CNF0) : 0) << ((Pin % 8) << 2);
		static const uint32_t CRH		= ((Pin >= port::pin_8) ? (CR)                             : 0) << ((Pin % 8) << 2);
		static const uint32_t ODR_MASK  = (Port == port::inv_address || Pin == port::inv_pin) ? 0 : (0x01UL << Pin);
		static const uint32_t ODR       = (Port == port::inv_address || Pin == port::inv_pin) ? 0 : (DefState != state::reset) ? (0x01UL << Pin) : (0x00UL << Pin);
		static const uint32_t ODR_SET   = (0x01UL << Pin);	/* Set the corresponding ODR bit */
		static const uint32_t ODR_RESET = (0x00UL << Pin);	/* Reset the corresponding ODR bit */
	};
	
	//------------------------------------------------------------------------
	template <
		::mcu::gpio::speed::speed	Speed     ,
		::mcu::gpio::state::state	DefState  ,
		::mcu::gpio::pull::pull		Pull      ,
		::mcu::gpio::flag::flag		Flag      ,
		port::address				Port      ,
		port::pin					Pin       
		>
	class REG_GPIO< true, ::mcu::gpio::mode::alt_input, Speed, DefState, Pull, Flag, Port, Pin >
	{
		static const uint32_t pull_up_dn = ((Pull == pull::up) || (Pull == pull::down)) ? 0x01UL : 0x00UL;
		static const uint32_t MODE = _const_::GPIO_CR_MODE_INPUT;
		static const uint32_t CNF  = pull_up_dn ? _const_::GPIO_CR_CNF_INPUT_PU_PD : _const_::GPIO_CR_CNF_INPUT_FLOATING;

	public:
		static const uint32_t CR = MODE | CNF;
		static const uint32_t CRL_MASK	= ((Pin <  port::pin_8) ? (GPIO_CRL_MODE0 | GPIO_CRL_CNF0) : 0) << ((Pin % 8) << 2);
		static const uint32_t CRL		= ((Pin <  port::pin_8) ? (CR)                             : 0) << ((Pin % 8) << 2);
		static const uint32_t CRH_MASK	= ((Pin >= port::pin_8) ? (GPIO_CRL_MODE0 | GPIO_CRL_CNF0) : 0) << ((Pin % 8) << 2);
		static const uint32_t CRH		= ((Pin >= port::pin_8) ? (CR)                             : 0) << ((Pin % 8) << 2);
		static const uint32_t ODR_MASK  = (Port == port::inv_address || Pin == port::inv_pin) ? 0 : (pull_up_dn << Pin);
		static const uint32_t ODR       = (Port == port::inv_address || Pin == port::inv_pin) ? 0 : (Pull == pull::up) ? (0x01UL << Pin) : (0x00UL << Pin);
		static const uint32_t ODR_SET   = ODR;
		static const uint32_t ODR_RESET = ODR;
	};
	
	//------------------------------------------------------------------------
	template <
		::mcu::gpio::speed::speed	Speed     ,
		::mcu::gpio::state::state	DefState  ,
		::mcu::gpio::pull::pull		Pull      ,
		::mcu::gpio::flag::flag		Flag      ,
		port::address				Port      ,
		port::pin					Pin       
		>
	class REG_GPIO< true, ::mcu::gpio::mode::alt_output, Speed, DefState, Pull, Flag, Port, Pin >
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
		static const uint32_t CRL_MASK	= ((Pin <  port::pin_8) ? (GPIO_CRL_MODE0 | GPIO_CRL_CNF0) : 0) << ((Pin % 8) << 2);
		static const uint32_t CRL		= ((Pin <  port::pin_8) ? (CR)                             : 0) << ((Pin % 8) << 2);
		static const uint32_t CRH_MASK	= ((Pin >= port::pin_8) ? (GPIO_CRL_MODE0 | GPIO_CRL_CNF0) : 0) << ((Pin % 8) << 2);
		static const uint32_t CRH		= ((Pin >= port::pin_8) ? (CR)                             : 0) << ((Pin % 8) << 2);
		static const uint32_t ODR_MASK  = 0;
		static const uint32_t ODR       = 0;
		static const uint32_t ODR_SET   = ODR;
		static const uint32_t ODR_RESET = ODR;
	};
	
	//////////////////////////////////////////////////////////////////////////
	template <
		bool						valid     ,
		::mcu::gpio::mode::mode		Mode      ,
		::mcu::gpio::pin_id::pin_id	PinID
		> class REG_RCC
	{
	public:
		static const uint32_t APB2ENR_AFIOEN = (Mode == ::mcu::gpio::mode::alt_input || Mode == ::mcu::gpio::mode::alt_output) ? RCC_APB2ENR_AFIOEN : 0;
		
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
	
	//------------------------------------------------------------------------
	template <
		::mcu::gpio::mode::mode		Mode      ,
		::mcu::gpio::pin_id::pin_id	PinID
		>
	class REG_RCC<false, Mode, PinID>
	{
	public:
		static const uint32_t APB2ENR_MASK = 0;
		static const uint32_t APB2ENR = 0;
	};
	
	//////////////////////////////////////////////////////////////////////////
	template <
		stm32::port::address PORT,
		stm32::port::pin     PIN 
		> class GPIO_REG_TO_BB
	{
	public:
		static const uint32_t _GPIO_ODR_BB = FROM_ADDRESS_BIT_POS_TO_BB(&(((GPIO_TypeDef*)(uint32_t)PORT)->ODR), PIN);
		static const uint32_t _GPIO_IDR_BB = FROM_ADDRESS_BIT_POS_TO_BB(&(((GPIO_TypeDef*)(uint32_t)PORT)->IDR), PIN);
	};

	//------------------------------------------------------------------------
	template < stm32::port::pin PIN > class GPIO_REG_TO_BB<stm32::port::inv_address, PIN>
	{
	public:
		static const uint32_t _GPIO_ODR_BB = 0;
		static const uint32_t _GPIO_IDR_BB = 0;
	};

	//------------------------------------------------------------------------
	template < stm32::port::address PORT > class GPIO_REG_TO_BB<PORT, stm32::port::inv_pin>
	{
	public:
		static const uint32_t _GPIO_ODR_BB = 0;
		static const uint32_t _GPIO_IDR_BB = 0;
	};

	//------------------------------------------------------------------------
	template <> class GPIO_REG_TO_BB<stm32::port::inv_address, stm32::port::inv_pin>
	{
	public:
		static const uint32_t _GPIO_ODR_BB = 0;
		static const uint32_t _GPIO_IDR_BB = 0;
	};

	//////////////////////////////////////////////////////////////////////////
	template < bool valid, uint32_t crl_mask = 0, uint32_t crl_val = 0, uint32_t crh_mask = 0, uint32_t crh_val = 0, uint32_t odr_mask = 0, uint32_t odr_val = 0 >
	struct _p$_cfg
	{
		static const uint32_t _crl_mask	= crl_mask	;
		static const uint32_t _crl_val	= crl_val	;
		static const uint32_t _crh_mask	= crh_mask	;
		static const uint32_t _crh_val	= crh_val	;
		static const uint32_t _odr_mask	= odr_mask	;
		static const uint32_t _odr_val	= odr_val	;
	};

	//------------------------------------------------------------------------
	template < uint32_t crl_mask, uint32_t crl_val, uint32_t crh_mask, uint32_t crh_val, uint32_t odr_mask, uint32_t odr_val >
	struct _p$_cfg< false, crl_mask, crl_val, crh_mask, crh_val, odr_mask, odr_val >
	{
		static const uint32_t _crl_mask	= 0;
		static const uint32_t _crl_val	= 0;
		static const uint32_t _crh_mask	= 0;
		static const uint32_t _crh_val	= 0;
		static const uint32_t _odr_mask	= 0;
		static const uint32_t _odr_val	= 0;
	};
} // namespace registers
} // namespace stm32

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace gpio {
template < class _CFG_ >
class gpio_port
{
private:
	class _const_
	{
		friend class gpio_port;
		
		static const stm32::port::address PORT   = stm32::port::convert<_CFG_::_PinID>::to_address;
		static const stm32::port::pin     PIN    = stm32::port::convert<_CFG_::_PinID>::to_pin;
		
		static const uint32_t _GPIO_ODR_BB      = stm32::registers::GPIO_REG_TO_BB<PORT, PIN>::_GPIO_ODR_BB;
		static const uint32_t _GPIO_IDR_BB      = stm32::registers::GPIO_REG_TO_BB<PORT, PIN>::_GPIO_IDR_BB;
	};

public:
	struct get_adc_ch : public stm32::adc::adc_ch_set < (::mcu::adc::adc_id::adc_id)_CFG_::_adc::obj::_id, ::mcu::adc::channel_id::invalid, _CFG_::_PinID >
	{ };

	class _port_
	{
	public:
		static const pin_id::pin_id			_PinID			= _CFG_::_PinID;
		static const stm32::port::address	_Port			= _const_::PORT;
		static const stm32::port::pin		_Pin			= _const_::PIN;
		static const mode::mode				_Mode			= _CFG_::_Mode;
		static const speed::speed			_Speed			= _CFG_::_Speed;
		static const state::state			_DefPinState	= _CFG_::_DefState;
		static const pull::pull				_Pull			= _CFG_::_Pull;
		static const flag::flag				_Flag			= _CFG_::_Flag;
	
	private:
		STATIC_ASSERT(FAIL_IF(_PinID != pin_id::invalid && _Port == stm32::port::inv_address), "The MCU has not defined port:pin configuration");
		STATIC_ASSERT(FAIL_IF(_PinID != pin_id::invalid && _Pin  == stm32::port::inv_pin    ), "The MCU has not defined port:pin configuration");

		static const bool valid = config::check_params<_Mode, _Speed, _DefPinState, _Pull, _Flag>::verified && (_PinID != pin_id::invalid);

		typedef stm32::registers::REG_GPIO<valid, _Mode, _Speed, _DefPinState, _Pull, _Flag, _Port, _Pin> REG_GPIO;
		typedef stm32::registers::REG_RCC<valid, _Mode, _PinID> REG_RCC;
	
	public:
		static const uint32_t _rcc_apb2enr_mask = REG_RCC::APB2ENR_MASK;
		static const uint32_t _rcc_apb2enr      = REG_RCC::APB2ENR;

		static const uint32_t _crl_mask  = REG_GPIO::CRL_MASK;
		static const uint32_t _crl_val   = REG_GPIO::CRL;
		static const uint32_t _crh_mask  = REG_GPIO::CRH_MASK;
		static const uint32_t _crh_val   = REG_GPIO::CRH;

		static const uint32_t _odr_mask  = REG_GPIO::ODR_MASK;
		static const uint32_t _odr_val   = REG_GPIO::ODR;
		static const uint32_t _odr_set   = REG_GPIO::ODR_SET;
		static const uint32_t _odr_reset = REG_GPIO::ODR_RESET;

		static const uint32_t
			_adc_smpr1 = 0,
			_adc_smpr2 = 0;

		typedef stm32::registers::_p$_cfg< IF_GPIOA_EXISTS(((_PinID >= pin_id::PA0 && _PinID <= pin_id::PA31) ? valid : false) ||) false, _crl_mask, _crl_val, _crh_mask, _crh_val, _odr_mask, _odr_val > _pa_cfg;
		typedef stm32::registers::_p$_cfg< IF_GPIOB_EXISTS(((_PinID >= pin_id::PB0 && _PinID <= pin_id::PB31) ? valid : false) ||) false, _crl_mask, _crl_val, _crh_mask, _crh_val, _odr_mask, _odr_val > _pb_cfg;
		typedef stm32::registers::_p$_cfg< IF_GPIOC_EXISTS(((_PinID >= pin_id::PC0 && _PinID <= pin_id::PC31) ? valid : false) ||) false, _crl_mask, _crl_val, _crh_mask, _crh_val, _odr_mask, _odr_val > _pc_cfg;
		typedef stm32::registers::_p$_cfg< IF_GPIOD_EXISTS(((_PinID >= pin_id::PD0 && _PinID <= pin_id::PD31) ? valid : false) ||) false, _crl_mask, _crl_val, _crh_mask, _crh_val, _odr_mask, _odr_val > _pd_cfg;
		typedef stm32::registers::_p$_cfg< IF_GPIOE_EXISTS(((_PinID >= pin_id::PE0 && _PinID <= pin_id::PE31) ? valid : false) ||) false, _crl_mask, _crl_val, _crh_mask, _crh_val, _odr_mask, _odr_val > _pe_cfg;
		typedef stm32::registers::_p$_cfg< IF_GPIOF_EXISTS(((_PinID >= pin_id::PF0 && _PinID <= pin_id::PF31) ? valid : false) ||) false, _crl_mask, _crl_val, _crh_mask, _crh_val, _odr_mask, _odr_val > _pf_cfg;
		typedef stm32::registers::_p$_cfg< IF_GPIOG_EXISTS(((_PinID >= pin_id::PG0 && _PinID <= pin_id::PG31) ? valid : false) ||) false, _crl_mask, _crl_val, _crh_mask, _crh_val, _odr_mask, _odr_val > _pg_cfg;
		typedef stm32::registers::_p$_cfg< IF_GPIOH_EXISTS(((_PinID >= pin_id::PH0 && _PinID <= pin_id::PH31) ? valid : false) ||) false, _crl_mask, _crl_val, _crh_mask, _crh_val, _odr_mask, _odr_val > _ph_cfg;
		typedef stm32::registers::_p$_cfg< IF_GPIOI_EXISTS(((_PinID >= pin_id::PI0 && _PinID <= pin_id::PI31) ? valid : false) ||) false, _crl_mask, _crl_val, _crh_mask, _crh_val, _odr_mask, _odr_val > _pi_cfg;
	};

public:
	static void init()
	{
		// Enable peripheral clock
		if(_port_::_rcc_apb2enr_mask)
		{
			SET_BIT (RCC->APB2ENR, _port_::_rcc_apb2enr);
			__NOP();
		}
		
		update();

		/*--------------------- EXTI Mode Configuration ------------------------*/
		/* Configure the External Interrupt or event for the current IO */
//		if(_port_::_Mode == mode::input && (_port_::_Flag & flag::flags_input) != 0)
//		{
//			uint32_t temp;
//			
//			/* Enable AFIO Clock */
//			__HAL_RCC_AFIO_CLK_ENABLE();
//			temp = AFIO->EXTICR[_port_::_Pin >> 2];
//			CLEAR_BIT(temp, ((uint32_t)0x0F) << (4 * (_port_::_Pin & 0x03)));
//			SET_BIT(temp, (GPIO_GET_INDEX(((GPIO_TypeDef*)_port_::_Port))) << (4 * (_port_::_Pin & 0x03)));
//			AFIO->EXTICR[_port_::_Pin >> 2] = temp;


//			/* Configure the interrupt mask */
//			if((_port_::_Mode & _const_::GPIO_MODE_IT) == _const_::GPIO_MODE_IT)
//			{
//				SET_BIT(EXTI->IMR, (0x01UL << _port_::_Pin)); 
//			} 
//			else
//			{
//				CLEAR_BIT(EXTI->IMR, (0x01UL << _port_::_Pin)); 
//			} 

//			/* Configure the event mask */
//			if((_port_::_Mode & _const_::GPIO_MODE_EVT) == _const_::GPIO_MODE_EVT)
//			{
//				SET_BIT(EXTI->EMR, (0x01UL << _port_::_Pin)); 
//			} 
//			else
//			{
//				CLEAR_BIT(EXTI->EMR, (0x01UL << _port_::_Pin)); 
//			}

//			/* Enable or disable the rising trigger */
//			if((_port_::_Mode & _const_::RISING_EDGE) == _const_::RISING_EDGE)
//			{
//				SET_BIT(EXTI->RTSR, (0x01UL << _port_::_Pin)); 
//			} 
//			else
//			{
//				CLEAR_BIT(EXTI->RTSR, (0x01UL << _port_::_Pin)); 
//			}

//			/* Enable or disable the falling trigger */
//			if((_port_::_Mode & _const_::FALLING_EDGE) == _const_::FALLING_EDGE)
//			{
//				SET_BIT(EXTI->FTSR, (0x01UL << _port_::_Pin)); 
//			} 
//			else
//			{
//				CLEAR_BIT(EXTI->FTSR, (0x01UL << _port_::_Pin)); 
//			}
//		}
	}
	
	static void update()
	{
		// Update ODR (pu/pd for input; state for output)
		(_port_::_odr_mask) ?
			(
				(_port_::_odr_val) ?
					(void)(((GPIO_TypeDef*)_port_::_Port)->BSRR = _port_::_odr_mask) :
					(void)(((GPIO_TypeDef*)_port_::_Port)->BRR  = _port_::_odr_mask)
			) :
			(void)(0);

		// configure port
		(_port_::_crl_mask) ?
			(void)(MODIFY_REG(((GPIO_TypeDef*)_port_::_Port)->CRL, _port_::_crl_mask, _port_::_crl_val)) :
			(void)(0);
		(_port_::_crh_mask) ?
			(void)(MODIFY_REG(((GPIO_TypeDef*)_port_::_Port)->CRH, _port_::_crh_mask, _port_::_crh_val)) :
			(void)(0);
	}
	
	static bool get()
	{
		// accessible at any time
		return IS_BB_REG_SET(_const_::_GPIO_IDR_BB);
	}
	static bool get_out()
	{
		STATIC_ASSERT(_port_::_Mode == mode::output, "Accessible in OUTPUT mode");
		return IS_BB_REG_SET(_const_::_GPIO_ODR_BB);
	}
	static void set()
	{
		STATIC_ASSERT(_port_::_Mode == mode::output, "Accessible in OUTPUT mode");
		SET_BB_REG(_const_::_GPIO_ODR_BB);
	}
	static void reset()
	{
		STATIC_ASSERT(_port_::_Mode == mode::output, "Accessible in OUTPUT mode");
		RESET_BB_REG(_const_::_GPIO_ODR_BB);
	}

	static void write(bool val)
	{
		STATIC_ASSERT(_port_::_Mode == mode::output, "Accessible in OUTPUT mode");
		WRITE_BB_REG(_const_::_GPIO_ODR_BB, val);
	}
	static bool read()
	{
		// accessible at any time
		return IS_BB_REG_SET(_const_::_GPIO_IDR_BB);
	}
};

/************************************************************************/
/*                                                                      */
/************************************************************************/
template < class LIST >
class atomic_port
{
	struct list_gpio_merge
	{
		typedef ::mcu::obj::obj< ::mcu::obj::type_id::gpio > obj_gpio;
		
		template < class L, class R >
		struct _p$_cfg_merge
		{
			static const uint32_t _crl_mask	= L::_crl_mask	| R::_crl_mask	;
			static const uint32_t _crl_val	= L::_crl_val	| R::_crl_val	;
			static const uint32_t _crh_mask	= L::_crh_mask	| R::_crh_mask	;
			static const uint32_t _crh_val	= L::_crh_val	| R::_crh_val	;
			static const uint32_t _odr_mask	= L::_odr_mask	| R::_odr_mask	;
			static const uint32_t _odr_val	= L::_odr_val	| R::_odr_val	;
		};
		
		struct _zero : public obj_gpio
		{
			struct _port_
			{
				static const uint32_t _rcc_apb2enr_mask = 0;
				static const uint32_t _rcc_apb2enr      = 0;
				static const uint32_t _adc_smpr1        = 0;
				static const uint32_t _adc_smpr2        = 0;

				typedef stm32::registers::_p$_cfg<false> _pa_cfg;
				typedef stm32::registers::_p$_cfg<false> _pb_cfg;
				typedef stm32::registers::_p$_cfg<false> _pc_cfg;
				typedef stm32::registers::_p$_cfg<false> _pd_cfg;
				typedef stm32::registers::_p$_cfg<false> _pe_cfg;
				typedef stm32::registers::_p$_cfg<false> _pf_cfg;
				typedef stm32::registers::_p$_cfg<false> _pg_cfg;
				typedef stm32::registers::_p$_cfg<false> _ph_cfg;
				typedef stm32::registers::_p$_cfg<false> _pi_cfg;
			};
		};

		template <class CUR, class NEXT> struct _cb
		{
			struct _result : public obj_gpio
			{
				//typedef _zero::_port_ _port_;
				struct _port_
				{
					typedef typename ::aux::if_c< CUR ::obj::_type_id == obj_gpio::obj::_type_id, CUR , _zero >::_result ::_port_ _l;
					typedef typename ::aux::if_c< NEXT::obj::_type_id == obj_gpio::obj::_type_id, NEXT, _zero >::_result ::_port_ _r;

					static const uint32_t _rcc_apb2enr_mask = _l::_rcc_apb2enr_mask | _r::_rcc_apb2enr_mask;
					static const uint32_t _rcc_apb2enr      = _l::_rcc_apb2enr      | _r::_rcc_apb2enr     ;
					static const uint32_t _adc_smpr1        = _l::_adc_smpr1        | _r::_adc_smpr1       ;
					static const uint32_t _adc_smpr2        = _l::_adc_smpr2        | _r::_adc_smpr2       ;

					typedef _p$_cfg_merge<typename _l::_pa_cfg, typename _r::_pa_cfg> _pa_cfg;
					typedef _p$_cfg_merge<typename _l::_pb_cfg, typename _r::_pb_cfg> _pb_cfg;
					typedef _p$_cfg_merge<typename _l::_pc_cfg, typename _r::_pc_cfg> _pc_cfg;
					typedef _p$_cfg_merge<typename _l::_pd_cfg, typename _r::_pd_cfg> _pd_cfg;
					typedef _p$_cfg_merge<typename _l::_pe_cfg, typename _r::_pe_cfg> _pe_cfg;
					typedef _p$_cfg_merge<typename _l::_pf_cfg, typename _r::_pf_cfg> _pf_cfg;
					typedef _p$_cfg_merge<typename _l::_pg_cfg, typename _r::_pg_cfg> _pg_cfg;
					typedef _p$_cfg_merge<typename _l::_ph_cfg, typename _r::_ph_cfg> _ph_cfg;
					typedef _p$_cfg_merge<typename _l::_pi_cfg, typename _r::_pi_cfg> _pi_cfg;
				};
			};
		};
	};

	struct __port_helper
	{
		template <uint32_t PORT, class CFG>
		static void init()
		{
			if(CFG::_odr_mask) /* initialize Output state */
				MODIFY_REG(((GPIO_TypeDef*)(uint32_t)PORT)->ODR, CFG::_odr_mask, CFG::_odr_val);
			if(CFG::_crl_mask) /* initialize mode */
				MODIFY_REG(((GPIO_TypeDef*)(uint32_t)PORT)->CRL, CFG::_crl_mask, CFG::_crl_val);
			if(CFG::_crh_mask) /* initialize mode */
				MODIFY_REG(((GPIO_TypeDef*)(uint32_t)PORT)->CRH, CFG::_crh_mask, CFG::_crh_val);
			/* TODO: EXTI Mode Configuration */
		}

		template <uint32_t PORT, class CFG>
		static void update()
		{
			if(CFG::_odr_mask) /* initialize Output state */
				MODIFY_REG(((GPIO_TypeDef*)(uint32_t)PORT)->ODR, CFG::_odr_mask, CFG::_odr_val);
			if(CFG::_crl_mask) /* initialize mode */
				MODIFY_REG(((GPIO_TypeDef*)(uint32_t)PORT)->CRL, CFG::_crl_mask, CFG::_crl_val);
			if(CFG::_crh_mask) /* initialize mode */
				MODIFY_REG(((GPIO_TypeDef*)(uint32_t)PORT)->CRH, CFG::_crh_mask, CFG::_crh_val);
		}

		template <uint32_t PORT, class CFG>
		static void write()
		{
			if(CFG::_odr_mask)
				MODIFY_REG(((GPIO_TypeDef*)(uint32_t)PORT)->ODR, CFG::_odr_mask, CFG::_odr_val);
		}

		template <uint32_t PORT, class CFG>
		static void set()
		{
			if(CFG::_odr_mask)
				WRITE_REG(((GPIO_TypeDef*)(uint32_t)PORT)->BSRR, CFG::_odr_mask);
		}

		template <uint32_t PORT, class CFG>
		static void reset()
		{
			if(CFG::_odr_mask)
				WRITE_REG(((GPIO_TypeDef*)(uint32_t)PORT)->BRR, CFG::_odr_mask);
		}
	};
	
public:
	typedef LIST pins;
	typedef typename pins::template traverse<list_gpio_merge>::_result::_port_ pins_cfg;

public:
	template <class sysclock>
	class on_sysclock_changing
	{
	protected:
		on_sysclock_changing();
		~on_sysclock_changing();
	
	public:
		static void starting() { }			// Disable the peripheral
		static void finished() { }			// Enable peripheral clock
	};

	static void init()
	{
		// enable GPIO clock
		if(pins_cfg::_rcc_apb2enr_mask)
		{
			SET_BIT(RCC->APB2ENR, pins_cfg::_rcc_apb2enr);
			__NOP();
		}
		
		IF_GPIOA_EXISTS( __port_helper::template init<(uint32_t)GPIOA, pins_cfg::_pa_cfg>(); )
		IF_GPIOB_EXISTS( __port_helper::template init<(uint32_t)GPIOB, pins_cfg::_pb_cfg>(); )
		IF_GPIOC_EXISTS( __port_helper::template init<(uint32_t)GPIOC, pins_cfg::_pc_cfg>(); )
		IF_GPIOD_EXISTS( __port_helper::template init<(uint32_t)GPIOD, pins_cfg::_pd_cfg>(); )
		IF_GPIOE_EXISTS( __port_helper::template init<(uint32_t)GPIOE, pins_cfg::_pe_cfg>(); )
		IF_GPIOF_EXISTS( __port_helper::template init<(uint32_t)GPIOF, pins_cfg::_pf_cfg>(); )
		IF_GPIOG_EXISTS( __port_helper::template init<(uint32_t)GPIOG, pins_cfg::_pg_cfg>(); )
		IF_GPIOH_EXISTS( __port_helper::template init<(uint32_t)GPIOH, pins_cfg::_ph_cfg>(); )
		IF_GPIOI_EXISTS( __port_helper::template init<(uint32_t)GPIOI, pins_cfg::_pi_cfg>(); )
	}
	
	static void update()
	{
		IF_GPIOA_EXISTS( __port_helper::template update<(uint32_t)GPIOA, pins_cfg::_pa_cfg>(); )
		IF_GPIOB_EXISTS( __port_helper::template update<(uint32_t)GPIOB, pins_cfg::_pb_cfg>(); )
		IF_GPIOC_EXISTS( __port_helper::template update<(uint32_t)GPIOC, pins_cfg::_pc_cfg>(); )
		IF_GPIOD_EXISTS( __port_helper::template update<(uint32_t)GPIOD, pins_cfg::_pd_cfg>(); )
		IF_GPIOE_EXISTS( __port_helper::template update<(uint32_t)GPIOE, pins_cfg::_pe_cfg>(); )
		IF_GPIOF_EXISTS( __port_helper::template update<(uint32_t)GPIOF, pins_cfg::_pf_cfg>(); )
		IF_GPIOG_EXISTS( __port_helper::template update<(uint32_t)GPIOG, pins_cfg::_pg_cfg>(); )
		IF_GPIOH_EXISTS( __port_helper::template update<(uint32_t)GPIOH, pins_cfg::_ph_cfg>(); )
		IF_GPIOI_EXISTS( __port_helper::template update<(uint32_t)GPIOI, pins_cfg::_pi_cfg>(); )
	}

	static void write()
	{
		IF_GPIOA_EXISTS( __port_helper::template write<(uint32_t)GPIOA, pins_cfg::_pa_cfg>(); )
		IF_GPIOB_EXISTS( __port_helper::template write<(uint32_t)GPIOB, pins_cfg::_pb_cfg>(); )
		IF_GPIOC_EXISTS( __port_helper::template write<(uint32_t)GPIOC, pins_cfg::_pc_cfg>(); )
		IF_GPIOD_EXISTS( __port_helper::template write<(uint32_t)GPIOD, pins_cfg::_pd_cfg>(); )
		IF_GPIOE_EXISTS( __port_helper::template write<(uint32_t)GPIOE, pins_cfg::_pe_cfg>(); )
		IF_GPIOF_EXISTS( __port_helper::template write<(uint32_t)GPIOF, pins_cfg::_pf_cfg>(); )
		IF_GPIOG_EXISTS( __port_helper::template write<(uint32_t)GPIOG, pins_cfg::_pg_cfg>(); )
		IF_GPIOH_EXISTS( __port_helper::template write<(uint32_t)GPIOH, pins_cfg::_ph_cfg>(); )
		IF_GPIOI_EXISTS( __port_helper::template write<(uint32_t)GPIOI, pins_cfg::_pi_cfg>(); )
	}

	static void set()
	{
		IF_GPIOA_EXISTS( __port_helper::template set<(uint32_t)GPIOA, pins_cfg::_pa_cfg>(); )
		IF_GPIOB_EXISTS( __port_helper::template set<(uint32_t)GPIOB, pins_cfg::_pb_cfg>(); )
		IF_GPIOC_EXISTS( __port_helper::template set<(uint32_t)GPIOC, pins_cfg::_pc_cfg>(); )
		IF_GPIOD_EXISTS( __port_helper::template set<(uint32_t)GPIOD, pins_cfg::_pd_cfg>(); )
		IF_GPIOE_EXISTS( __port_helper::template set<(uint32_t)GPIOE, pins_cfg::_pe_cfg>(); )
		IF_GPIOF_EXISTS( __port_helper::template set<(uint32_t)GPIOF, pins_cfg::_pf_cfg>(); )
		IF_GPIOG_EXISTS( __port_helper::template set<(uint32_t)GPIOG, pins_cfg::_pg_cfg>(); )
		IF_GPIOH_EXISTS( __port_helper::template set<(uint32_t)GPIOH, pins_cfg::_ph_cfg>(); )
		IF_GPIOI_EXISTS( __port_helper::template set<(uint32_t)GPIOI, pins_cfg::_pi_cfg>(); )
	}

	static void reset()
	{
		IF_GPIOA_EXISTS( __port_helper::template reset<(uint32_t)GPIOA, pins_cfg::_pa_cfg>(); )
		IF_GPIOB_EXISTS( __port_helper::template reset<(uint32_t)GPIOB, pins_cfg::_pb_cfg>(); )
		IF_GPIOC_EXISTS( __port_helper::template reset<(uint32_t)GPIOC, pins_cfg::_pc_cfg>(); )
		IF_GPIOD_EXISTS( __port_helper::template reset<(uint32_t)GPIOD, pins_cfg::_pd_cfg>(); )
		IF_GPIOE_EXISTS( __port_helper::template reset<(uint32_t)GPIOE, pins_cfg::_pe_cfg>(); )
		IF_GPIOF_EXISTS( __port_helper::template reset<(uint32_t)GPIOF, pins_cfg::_pf_cfg>(); )
		IF_GPIOG_EXISTS( __port_helper::template reset<(uint32_t)GPIOG, pins_cfg::_pg_cfg>(); )
		IF_GPIOH_EXISTS( __port_helper::template reset<(uint32_t)GPIOH, pins_cfg::_ph_cfg>(); )
		IF_GPIOI_EXISTS( __port_helper::template reset<(uint32_t)GPIOI, pins_cfg::_pi_cfg>(); )
	}
};

} // namespace gpio
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__stm32f1xx_gpio_hpp__*/
