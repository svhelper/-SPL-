#ifndef __device_hpp__
#define __device_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/



/************************************************************************/
/*                                                                      */
/************************************************************************/
#define METALLL_MCU						STM32F103C8
#define METALLL_HSE_FREQ_HZ				8000000UL
#define METALLL_LSE_FREQ_HZ				32768UL


//////////////////////////////////////////////////////////////////////////
#include <mcu.hpp>


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace device {
	// Assigning GPIO
	using namespace ::mcu::gpio;
	
	typedef output<pin_id::PA0> ch1_r680;
	typedef analog<pin_id::PA1> ch1_adc;
	typedef output<pin_id::PA2> ch1_r470k;

	typedef output<pin_id::PA3> ch2_r680;
	typedef analog<pin_id::PA4> ch2_adc;
	typedef output<pin_id::PA5> ch2_r470k;

	typedef output<pin_id::PB0> ch3_r680;
	typedef analog<pin_id::PB1> ch3_adc;
	typedef output<pin_id::PB2> ch3_r470k;

	typedef analog<pin_id::PA6> vbat_adc;
	typedef analog<pin_id::PA7> vref_adc;

	typedef output<pin_id::PB13, state::reset> lcd_back;
	typedef output<pin_id::PB9,  state::reset> lcd_a0;
	typedef output<pin_id::PB4,  state::set  > lcd_cs;
	typedef output<pin_id::PB8,  state::reset> lcd_res;

	typedef alt_output<pin_id::PB3, speed::high> lcd_sck;
	typedef alt_output<pin_id::PB5, speed::high> lcd_mosi;

	typedef input<pin_id::PB6> enc_a;
	typedef input<pin_id::PB7> enc_b;

	typedef alt_output<pin_id::PA9> uart1_tx;
	typedef alt_input<pin_id::PA10> uart1_rx;

	/************************************************************************/
	/*                                                                      */
	/************************************************************************/
	typedef atomic<	ch1_r680, ch1_adc, ch1_r470k,
					ch2_r680, ch2_adc, ch2_r470k,
					ch3_r680, ch3_adc, ch3_r470k,
					vbat_adc, vref_adc,
					lcd_back, lcd_a0, lcd_cs, lcd_res,
					lcd_sck, lcd_mosi,
					enc_a, enc_b,
					//uart1_tx, uart1_rx,
					::mcu::dummy::eol
			> all_pins;
} // namespace device

namespace device {
	// Declaration of modules
	using namespace ::mcu;
	using namespace ::mcu::uart;
	
	//typedef uart_def< uart_id::uart_1, uart::mode::tx_only, 115200*8, data_bits::eight, stop_bits::one > uart1;
	typedef uart_gpio< uart_id::uart_1, uart::mode::tx_only, 115200*8, data_bits::eight, stop_bits::one, parity::none, flow_control::none, uart1_tx, dummy::obj > uart1;
	
	//typedef uart_def< uart_id::uart_1, uart::mode::tx_rx, 115200*8, data_bits::eight, stop_bits::one > uart1;
	//typedef uart_gpio< uart_id::uart_1, uart::mode::tx_rx, 115200*8, data_bits::eight, stop_bits::one, parity::none, flow_control::none, uart1_tx, uart1_rx > uart1;
	//typedef uart_def< uart_id::uart_1, uart::mode::tx_rx, 115200*8, data_bits::eight, stop_bits::one, uart::parity::none, uart::flow_control::none, 0 > uart1;

} // namespace device


/************************************************************************/
/*                                                                      */
/************************************************************************/
#endif /*__device_hpp__*/
