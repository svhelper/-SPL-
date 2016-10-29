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
	using namespace ::mcu::adc;
	
	//------------------------------------------------------------------------
	//typedef adc_def<adc_id::adc_1> adc1;
	//typedef adc_def< adc_id::adc_1, ::mcu::adc::data_bits::_12bits_right > adc1;
	typedef adc_def< adc_id::adc_1, ::mcu::adc::data_bits::_12bits_right, ::mcu::adc::flags::calibrate_at_startup, ::mcu::adc::sampling_delay_min::_15 > adc1;
	
	//------------------------------------------------------------------------
	typedef output<pin_id::PA0> ch1_r680;
	typedef analog<pin_id::PA1, adc1> ch1_adc;
	typedef output<pin_id::PA2> ch1_r470k;

	typedef output<pin_id::PA3> ch2_r680;
	typedef analog<pin_id::PA4, adc1> ch2_adc;
	typedef output<pin_id::PA5> ch2_r470k;

	typedef output<pin_id::PB0> ch3_r680;
	typedef analog<pin_id::PB1, adc1> ch3_adc;
	typedef output<pin_id::PB2> ch3_r470k;

	typedef analog<pin_id::PA6, adc1> vbat_adc;
	typedef analog<pin_id::PA7, adc1> vref_adc;

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
					//lcd_sck, lcd_mosi,
					enc_a, enc_b,
					//uart1_tx, uart1_rx,
					::mcu::dummy::eol
			> all_pins;
} // namespace device

namespace device {
	// Declaration of modules
	using namespace ::mcu;
	//using namespace ::mcu::uart;
	
	//typedef uart_def< uart_id::uart_1, ::mcu::uart::mode::tx_only, 115200*8, ::mcu::uart::data_bits::eight, ::mcu::uart::stop_bits::one > uart1;
	//typedef uart_gpio< uart_id::uart_1, ::mcu::uart::mode::tx_only, 115200*8, ::mcu::uart::data_bits::eight, ::mcu::uart::stop_bits::one, ::mcu::uart::parity::none, ::mcu::uart::flow_control::none, uart1_tx > uart1;
	
	typedef uart_def< uart_id::uart_1, ::mcu::uart::mode::tx_rx, 115200*8/512, ::mcu::uart::data_bits::eight, ::mcu::uart::stop_bits::one > uart1;
	//typedef uart_gpio< uart_id::uart_1, ::mcu::uart::mode::tx_rx, 115200*8, ::mcu::uart::data_bits::eight, ::mcu::uart::stop_bits::one, ::mcu::uart::parity::none, ::mcu::uart::flow_control::none, uart1_tx, uart1_rx > uart1;
	//typedef uart_def< uart_id::uart_1, ::mcu::uart::mode::tx_rx, 115200*8, ::mcu::uart::data_bits::eight, ::mcu::uart::stop_bits::one, ::mcu::uart::parity::none, ::mcu::uart::flow_control::none, ALT_FUNC_ID_AUTO > uart1;
	//typedef uart_gpio< uart_id::uart_1, ::mcu::uart::mode::tx_rx, 115200*8, ::mcu::uart::data_bits::eight, ::mcu::uart::stop_bits::one, ::mcu::uart::parity::none, ::mcu::uart::flow_control::none, uart1_tx, uart1_rx, dummy::obj, dummy::obj, dummy::obj, 3, 0 > uart1;
	
	//typedef uart_gpio< uart_id::uart_1, ::mcu::uart::mode::tx_only, 115200*8, ::mcu::uart::data_bits::eight, ::mcu::uart::stop_bits::one, ::mcu::uart::parity::none, ::mcu::uart::flow_control::none, uart1_tx, dummy::obj, dummy::obj, dummy::obj, dummy::obj, 3, 0 > uart1;
	//typedef uart_gpio< uart_id::uart_1, ::mcu::uart::mode::rx_only, 115200*8, ::mcu::uart::data_bits::eight, ::mcu::uart::stop_bits::one, ::mcu::uart::parity::none, ::mcu::uart::flow_control::none, dummy::obj, enc_b, dummy::obj, dummy::obj, dummy::obj, 3, 1 > uart1;

	//------------------------------------------------------------------------
//	typedef spi_def< spi_id::spi_1 > spi1;
//	typedef spi_def< spi_id::spi_1						
//			, ::mcu::spi::mode::master_CPOL_1_CPHA_1	
//			, ::mcu::spi::BAUDRATE_DEF					
//			, ::mcu::spi::data_bits::_8_normal			
//			, ::mcu::spi::bus::tx_rx_2_line				
//			, ::mcu::gpio::pin_id::invalid				
//			, ::mcu::gpio::pin_id::invalid				
//			, ::mcu::gpio::pin_id::invalid				
//			, ::mcu::gpio::pin_id::invalid				
//			, ::mcu::spi::ALT_FUNC_ID_DEF				
//		> spi1;
//	typedef spi_gpio< spi_id::spi_1						
//			, ::mcu::spi::mode::master_CPOL_1_CPHA_1	
//			, ::mcu::spi::BAUDRATE_DEF					
//			, ::mcu::spi::data_bits::_8_normal			
//			, ::mcu::spi::bus::tx_only					
//			, lcd_sck									
//			, lcd_mosi									
////			, dummy::invalid							
////			, dummy::invalid							
////			, ::mcu::spi::ALT_FUNC_ID_AUTO				
//		> spi1;
	typedef spi_def< spi_id::spi_1						
			, ::mcu::spi::mode::master_CPOL_1_CPHA_1
			, ::mcu::spi::BAUDRATE_DEF
			, ::mcu::spi::data_bits::_8_normal
			, ::mcu::spi::bus::tx_only
			, 1
		> spi1;
	
	//------------------------------------------------------------------------
	typedef ::mcu::clock::sysclock_pll_hse_def sysclk_def;
	typedef dma::dma<dma::dma_id::dma_1> dma1;
	
	typedef ::mcu::mcu<sysclk_def, dma1, uart1, spi1, adc1, all_pins> _mcu;
} // namespace device


/************************************************************************/
/*                                                                      */
/************************************************************************/
#endif /*__device_hpp__*/
