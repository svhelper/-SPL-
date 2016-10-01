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
namespace port {

typedef enum
{
	PA0,
	PA1,
	PA2,
	PA3,
	PA4,
	PA5,
	PA6,
	PA7,
	PA8,
	PA9,
	PA10,
	PA11,
	PA12,
	PA13,
	PA14,
	PA15,
	PA16,
	PA17,
	PA18,
	PA19,
	PA20,
	PA21,
	PA22,
	PA23,
	PA24,
	PA25,
	PA26,
	PA27,
	PA28,
	PA29,
	PA30,
	PA31,

	PB0,
	PB1,
	PB2,
	PB3,
	PB4,
	PB5,
	PB6,
	PB7,
	PB8,
	PB9,
	PB10,
	PB11,
	PB12,
	PB13,
	PB14,
	PB15,
	PB16,
	PB17,
	PB18,
	PB19,
	PB20,
	PB21,
	PB22,
	PB23,
	PB24,
	PB25,
	PB26,
	PB27,
	PB28,
	PB29,
	PB30,
	PB31,

	PC0,
	PC1,
	PC2,
	PC3,
	PC4,
	PC5,
	PC6,
	PC7,
	PC8,
	PC9,
	PC10,
	PC11,
	PC12,
	PC13,
	PC14,
	PC15,
	PC16,
	PC17,
	PC18,
	PC19,
	PC20,
	PC21,
	PC22,
	PC23,
	PC24,
	PC25,
	PC26,
	PC27,
	PC28,
	PC29,
	PC30,
	PC31,

	PD0,
	PD1,
	PD2,
	PD3,
	PD4,
	PD5,
	PD6,
	PD7,
	PD8,
	PD9,
	PD10,
	PD11,
	PD12,
	PD13,
	PD14,
	PD15,
	PD16,
	PD17,
	PD18,
	PD19,
	PD20,
	PD21,
	PD22,
	PD23,
	PD24,
	PD25,
	PD26,
	PD27,
	PD28,
	PD29,
	PD30,
	PD31,

	PE0,
	PE1,
	PE2,
	PE3,
	PE4,
	PE5,
	PE6,
	PE7,
	PE8,
	PE9,
	PE10,
	PE11,
	PE12,
	PE13,
	PE14,
	PE15,
	PE16,
	PE17,
	PE18,
	PE19,
	PE20,
	PE21,
	PE22,
	PE23,
	PE24,
	PE25,
	PE26,
	PE27,
	PE28,
	PE29,
	PE30,
	PE31,

	PF0,
	PF1,
	PF2,
	PF3,
	PF4,
	PF5,
	PF6,
	PF7,
	PF8,
	PF9,
	PF10,
	PF11,
	PF12,
	PF13,
	PF14,
	PF15,
	PF16,
	PF17,
	PF18,
	PF19,
	PF20,
	PF21,
	PF22,
	PF23,
	PF24,
	PF25,
	PF26,
	PF27,
	PF28,
	PF29,
	PF30,
	PF31,

	PG0,
	PG1,
	PG2,
	PG3,
	PG4,
	PG5,
	PG6,
	PG7,
	PG8,
	PG9,
	PG10,
	PG11,
	PG12,
	PG13,
	PG14,
	PG15,
	PG16,
	PG17,
	PG18,
	PG19,
	PG20,
	PG21,
	PG22,
	PG23,
	PG24,
	PG25,
	PG26,
	PG27,
	PG28,
	PG29,
	PG30,
	PG31,

	PH0,
	PH1,
	PH2,
	PH3,
	PH4,
	PH5,
	PH6,
	PH7,
	PH8,
	PH9,
	PH10,
	PH11,
	PH12,
	PH13,
	PH14,
	PH15,
	PH16,
	PH17,
	PH18,
	PH19,
	PH20,
	PH21,
	PH22,
	PH23,
	PH24,
	PH25,
	PH26,
	PH27,
	PH28,
	PH29,
	PH30,
	PH31,

	PI0,
	PI1,
	PI2,
	PI3,
	PI4,
	PI5,
	PI6,
	PI7,
	PI8,
	PI9,
	PI10,
	PI11,
	PI12,
	PI13,
	PI14,
	PI15,
	PI16,
	PI17,
	PI18,
	PI19,
	PI20,
	PI21,
	PI22,
	PI23,
	PI24,
	PI25,
	PI26,
	PI27,
	PI28,
	PI29,
	PI30,
	PI31,

	invalid_port			= 0xFFFF,
} pins;

} // namespace port


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
namespace config {

	//////////////////////////////////////////////////////////////////////////
	typedef enum
	{
		analog,
		input,
		output,
		alt_input,
		alt_output,
	} mode;
	
	//////////////////////////////////////////////////////////////////////////
	typedef enum
	{
		flags_mode_position	= 8,
		flags_mode_mask		= (0xFF << flags_mode_position),
		
		flag_no				= 0,				/*!< No flags                                                                       */

		flags_analog		= (analog << 8),

		flags_input			= (input << 8),
		input_int_rising,						/*!< External Interrupt Mode with Rising edge trigger detection                     */
		input_int_falling,						/*!< External Interrupt Mode with Falling edge trigger detection                    */
		input_int_both,							/*!< External Interrupt Mode with both of Rising and Falling edge trigger detection */
		input_evt_rising,						/*!< External Event Mode with Rising edge trigger detection                         */
		input_evt_falling,						/*!< External Event Mode with Falling edge trigger detection                        */
		input_evt_both,							/*!< External Event Mode with both of Rising and Falling edge trigger detection     */
		
		flags_output		= (output << 8),
		output_push_pull,						/*!< Output Push Pull Mode                                                          */
		output_open_drain,						/*!< Output Open Drain Mode                                                         */

		flags_alt_input		= (alt_input << 8),

		flags_alt_output	= (alt_output << 8),
		alt_output_push_pull,					/*!< Output Push Pull Mode                                                          */
		alt_output_open_drain,					/*!< Output Open Drain Mode                                                         */
	} flag;

	//////////////////////////////////////////////////////////////////////////
	typedef enum
	{
		speed_freq_low,		/*!< Low speed */
		speed_freq_medium,	/*!< Medium speed */
		speed_freq_high,	/*!< High speed */
	} speed;

	//////////////////////////////////////////////////////////////////////////
	typedef enum
	{
		pull_no,			/*!< No Pull-up or Pull-down activation  */
		pull_up,			/*!< Pull-up activation                  */
		pull_down,			/*!< Pull-down activation                */
	} pull_mode;
	
	//////////////////////////////////////////////////////////////////////////
	template <
		config::mode		Mode      ,
		config::speed		Speed     ,
		state::state		DefState  ,
		config::pull_mode	Pull      ,
		config::flag		Flag      
		>
	class check_params;

	template <
		config::speed		Speed     ,
		state::state		DefState  ,
		config::pull_mode	Pull      ,
		config::flag		Flag      
		>
	class check_params<analog, Speed, DefState, Pull, Flag>
	{
		STATIC_ASSERT(Pull == config::pull_no, "The ANALOG configuration of PIN, must not be configured in PULL UP/DOWN mode");
		STATIC_ASSERT(Flag == config::flag_no || (Flag & flags_mode_mask) == flags_analog, "Wrong flag for the ANALOG configuration of PIN");

	public:
		static const bool verified = true;
	};
	
	template <
		config::speed		Speed     ,
		state::state		DefState  ,
		config::pull_mode	Pull      ,
		config::flag		Flag      
		>
	class check_params<input, Speed, DefState, Pull, Flag>
	{
		STATIC_ASSERT(Flag == config::flag_no || (Flag & flags_mode_mask) == flags_input, "Wrong flag for the INPUT configuration of PIN");
		
	public:
		static const bool verified = true;
	};
	
	template <
		config::speed		Speed     ,
		state::state		DefState  ,
		config::pull_mode	Pull      ,
		config::flag		Flag      
		>
	class check_params<output, Speed, DefState, Pull, Flag>
	{
		STATIC_ASSERT(Flag == config::flag_no || (Flag & flags_mode_mask) == flags_output, "Wrong flag for the OUTPUT configuration of PIN");
		
	public:
		static const bool verified = true;
	};
	
	template <
		config::speed		Speed     ,
		state::state		DefState  ,
		config::pull_mode	Pull      ,
		config::flag		Flag      
		>
	class check_params<alt_input, Speed, DefState, Pull, Flag>
	{
		STATIC_ASSERT(Flag == config::flag_no || (Flag & flags_mode_mask) == flags_alt_input, "Wrong flag for the ALTERNATIVE INPUT configuration of PIN");
		
	public:
		static const bool verified = true;
	};
	
	template <
		config::speed		Speed     ,
		state::state		DefState  ,
		config::pull_mode	Pull      ,
		config::flag		Flag      
		>
	class check_params<alt_output, Speed, DefState, Pull, Flag>
	{
		STATIC_ASSERT(Flag == config::flag_no || (Flag & flags_mode_mask) == flags_alt_output, "Wrong flag for the ALTERNATIVE OUTPUT configuration of PIN");
		
	public:
		static const bool verified = true;
	};
	
} // namespace config

/************************************************************************/
/*                                                                      */
/************************************************************************/	
template <	port::pins			Pin       = port::invalid_port,
			config::mode		Mode      = config::analog,
			config::speed		Speed     = config::speed_freq_low,
			state::state		DefState  = state::reset,
			config::pull_mode	Pull      = config::pull_no,
			config::flag		Flag      = config::flag_no
		>
class gpio_base;

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace outputs {
	template <class Port, state::state State> class write;

	//////////////////////////////////////////////////////////////////////////
	template <class Port>
	class reset: public write<Port, state::reset> { };

	//////////////////////////////////////////////////////////////////////////
	template <class Port>
	class set: public write<Port, state::set> { };

} // namespace outputs

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace gpio
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__gpio_hpp__*/
