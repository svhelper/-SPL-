#ifndef __gpio_hpp__
#define __gpio_hpp__

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
namespace gpio {

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace pin_id {
typedef enum
{
	 PA0,  PA1,  PA2,  PA3,  PA4,  PA5,  PA6,  PA7,  PA8,  PA9, PA10, PA11, PA12, PA13, PA14, PA15,
	PA16, PA17, PA18, PA19, PA20, PA21, PA22, PA23, PA24, PA25, PA26, PA27, PA28, PA29, PA30, PA31,

	 PB0,  PB1,  PB2,  PB3,  PB4,  PB5,  PB6,  PB7,  PB8,  PB9, PB10, PB11, PB12, PB13, PB14, PB15,
	PB16, PB17, PB18, PB19, PB20, PB21, PB22, PB23, PB24, PB25, PB26, PB27, PB28, PB29, PB30, PB31,

	 PC0,  PC1,  PC2,  PC3,  PC4,  PC5,  PC6,  PC7,  PC8,  PC9, PC10, PC11, PC12, PC13, PC14, PC15,
	PC16, PC17, PC18, PC19, PC20, PC21, PC22, PC23, PC24, PC25, PC26, PC27, PC28, PC29, PC30, PC31,

	 PD0,  PD1,  PD2,  PD3,  PD4,  PD5,  PD6,  PD7,  PD8,  PD9, PD10, PD11, PD12, PD13, PD14, PD15,
	PD16, PD17, PD18, PD19, PD20, PD21, PD22, PD23, PD24, PD25, PD26, PD27, PD28, PD29, PD30, PD31,

	 PE0,  PE1,  PE2,  PE3,  PE4,  PE5,  PE6,  PE7,  PE8,  PE9, PE10, PE11, PE12, PE13, PE14, PE15,
	PE16, PE17, PE18, PE19, PE20, PE21, PE22, PE23, PE24, PE25, PE26, PE27, PE28, PE29, PE30, PE31,

	 PF0,  PF1,  PF2,  PF3,  PF4,  PF5,  PF6,  PF7,  PF8,  PF9, PF10, PF11, PF12, PF13, PF14, PF15,
	PF16, PF17, PF18, PF19, PF20, PF21, PF22, PF23, PF24, PF25, PF26, PF27, PF28, PF29, PF30, PF31,

	 PG0,  PG1,  PG2,  PG3,  PG4,  PG5,  PG6,  PG7,  PG8,  PG9, PG10, PG11, PG12, PG13, PG14, PG15,
	PG16, PG17, PG18, PG19, PG20, PG21, PG22, PG23, PG24, PG25, PG26, PG27, PG28, PG29, PG30, PG31,

	 PH0,  PH1,  PH2,  PH3,  PH4,  PH5,  PH6,  PH7,  PH8,  PH9, PH10, PH11, PH12, PH13, PH14, PH15,
	PH16, PH17, PH18, PH19, PH20, PH21, PH22, PH23, PH24, PH25, PH26, PH27, PH28, PH29, PH30, PH31,

	 PI0,  PI1,  PI2,  PI3,  PI4,  PI5,  PI6,  PI7,  PI8,  PI9, PI10, PI11, PI12, PI13, PI14, PI15,
	PI16, PI17, PI18, PI19, PI20, PI21, PI22, PI23, PI24, PI25, PI26, PI27, PI28, PI29, PI30, PI31,

	invalid			= 0xFFFF,
} pin_id;
} // namespace pin_id


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace state {
typedef enum state
{ 
	reset               = 0,
	set                 = !reset,
} state;
} // namespace state

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mode {
typedef enum
{
	analog,			/*!< Analog input */
	input,			/*!< Digital input */
	output,			/*!< Digital output */
	alt_input,		/*!< Alternative Function input */
	alt_output,		/*!< Alternative Function output */
} mode;
} // namespace mode

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace flag {
typedef enum
{
	//------------------------------------------------------------------------
	// Service values
	flags_mode_position	= 8,
	flags_mode_mask		= (0xFF << flags_mode_position),
	
	//------------------------------------------------------------------------
	// Default for all
	flag_no				= 0,				/*!< No flags                                                                       */

	//------------------------------------------------------------------------
	// Analog input
	flags_analog		= (mode::analog << 8),

	//------------------------------------------------------------------------
	// Digital input
	flags_input			= (mode::input << 8),
	input_int_rising,						/*!< External Interrupt Mode with Rising edge trigger detection                     */
	input_int_falling,						/*!< External Interrupt Mode with Falling edge trigger detection                    */
	input_int_both,							/*!< External Interrupt Mode with both of Rising and Falling edge trigger detection */
	input_evt_rising,						/*!< External Event Mode with Rising edge trigger detection                         */
	input_evt_falling,						/*!< External Event Mode with Falling edge trigger detection                        */
	input_evt_both,							/*!< External Event Mode with both of Rising and Falling edge trigger detection     */
	
	//------------------------------------------------------------------------
	// Digital output
	flags_output		= (mode::output << 8),
	output_push_pull,						/*!< Output Push Pull Mode                                                          */
	output_open_drain,						/*!< Output Open Drain Mode                                                         */

	//------------------------------------------------------------------------
	// Alternative Function input
	flags_alt_input		= (mode::alt_input << 8),

	//------------------------------------------------------------------------
	// Alternative Function output
	flags_alt_output	= (mode::alt_output << 8),
	alt_output_push_pull,					/*!< Output Push Pull Mode                                                          */
	alt_output_open_drain,					/*!< Output Open Drain Mode                                                         */
} flag;
} // namespace flag

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace speed {
typedef enum
{
	low,			/*!< Low speed */
	medium,			/*!< Medium speed */
	high,			/*!< High speed */
} speed;
} // namespace speed

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace pull {
typedef enum
{
	no,				/*!< No Pull-up or Pull-down activation  */
	up,				/*!< Pull-up activation                  */
	down,			/*!< Pull-down activation                */
} pull;
} // namespace pull

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace config {

	//////////////////////////////////////////////////////////////////////////
	template <
		mode::mode		Mode      ,
		speed::speed	Speed     ,
		state::state	DefState  ,
		pull::pull		Pull      ,
		flag::flag		Flag      
		>
	class check_params;

	//------------------------------------------------------------------------
	template < speed::speed Speed, state::state DefState, pull::pull Pull, flag::flag Flag >
	class check_params<mode::analog, Speed, DefState, Pull, Flag>
	{
		STATIC_ASSERT(Pull == pull::no, "The ANALOG configuration of PIN, must not be configured in PULL UP/DOWN mode");
		STATIC_ASSERT(Flag == flag::flag_no || (Flag & flag::flags_mode_mask) == flag::flags_analog, "Wrong flag for the ANALOG configuration of PIN");

	public:
		static const bool verified = true;
	};

	//------------------------------------------------------------------------
	template < speed::speed Speed, state::state DefState, pull::pull Pull, flag::flag Flag >
	class check_params<mode::input, Speed, DefState, Pull, Flag>
	{
		STATIC_ASSERT(Flag == flag::flag_no || (Flag & flag::flags_mode_mask) == flag::flags_input, "Wrong flag for the INPUT configuration of PIN");
		
	public:
		static const bool verified = true;
	};

	//------------------------------------------------------------------------
	template < speed::speed Speed, state::state DefState, pull::pull Pull, flag::flag Flag >
	class check_params<mode::output, Speed, DefState, Pull, Flag>
	{
		STATIC_ASSERT(Flag == flag::flag_no || (Flag & flag::flags_mode_mask) == flag::flags_output, "Wrong flag for the OUTPUT configuration of PIN");
		
	public:
		static const bool verified = true;
	};

	//------------------------------------------------------------------------
	template < speed::speed Speed, state::state DefState, pull::pull Pull, flag::flag Flag >
	class check_params<mode::alt_input, Speed, DefState, Pull, Flag>
	{
		STATIC_ASSERT(Flag == flag::flag_no || (Flag & flag::flags_mode_mask) == flag::flags_alt_input, "Wrong flag for the ALTERNATIVE INPUT configuration of PIN");
		
	public:
		static const bool verified = true;
	};

	//------------------------------------------------------------------------
	template < speed::speed Speed, state::state DefState, pull::pull Pull, flag::flag Flag >
	class check_params<mode::alt_output, Speed, DefState, Pull, Flag>
	{
		STATIC_ASSERT(Flag == flag::flag_no || (Flag & flag::flags_mode_mask) == flag::flags_alt_output, "Wrong flag for the ALTERNATIVE OUTPUT configuration of PIN");
		
	public:
		static const bool verified = true;
	};
	
} // namespace config

/************************************************************************/
/*                                                                      */
/************************************************************************/	
template <	pin_id::pin_id	Pin       = pin_id::invalid,
			mode::mode		Mode      = mode::analog,
			speed::speed	Speed     = speed::low,
			state::state	DefState  = state::reset,
			pull::pull		Pull      = pull::no,
			flag::flag		Flag      = flag::flag_no
		>
class gpio_base;

/************************************************************************/
/*                                                                      */
/************************************************************************/
class gpio_dummy;
typedef gpio_dummy gpio_invalid;

template <typename p00 = gpio_dummy, typename p01 = gpio_dummy, typename p02 = gpio_dummy, typename p03 = gpio_dummy,
		  typename p04 = gpio_dummy, typename p05 = gpio_dummy, typename p06 = gpio_dummy, typename p07 = gpio_dummy,
		  typename p08 = gpio_dummy, typename p09 = gpio_dummy, typename p10 = gpio_dummy, typename p11 = gpio_dummy,
		  typename p12 = gpio_dummy, typename p13 = gpio_dummy, typename p14 = gpio_dummy, typename p15 = gpio_dummy,
		  typename p16 = gpio_dummy, typename p17 = gpio_dummy, typename p18 = gpio_dummy, typename p19 = gpio_dummy,
		  typename p20 = gpio_dummy, typename p21 = gpio_dummy, typename p22 = gpio_dummy, typename p23 = gpio_dummy,
		  typename p24 = gpio_dummy, typename p25 = gpio_dummy, typename p26 = gpio_dummy, typename p27 = gpio_dummy,
		  typename p28 = gpio_dummy, typename p29 = gpio_dummy, typename p30 = gpio_dummy, typename p31 = gpio_dummy,
		  typename p32 = gpio_dummy, typename p33 = gpio_dummy, typename p34 = gpio_dummy, typename p35 = gpio_dummy,
		  typename p36 = gpio_dummy, typename p37 = gpio_dummy, typename p38 = gpio_dummy, typename p39 = gpio_dummy>
class atomic;

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace gpio
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__gpio_hpp__*/
