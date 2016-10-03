#ifndef __uart_hpp__
#define __uart_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

#include <gpio.hpp>
#include <dma.hpp>

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {
namespace uart {

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace uart_id {
typedef enum
{
	uart_1,
	uart_2,
	uart_3,
	uart_4,
	uart_5,
	uart_6,
	uart_7,
	uart_8,
	uart_9,
	uart_10,
	uart_11,
	uart_12,
	uart_13,
	uart_14,
	uart_15,
	uart_16,
	
	invalid			= 0xFFFF,
} uart_id;
} // namespace uart_id

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace parity {
typedef enum
{
	none,
	odd,
	even,
	mark,
	space,
} parity;
} // namespace parity

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mode {
typedef enum
{
	// service values
	sync_mask		= 1UL << 3,
	
	// async mode
	tx_only			= 1UL << 0,
	rx_only			= 1UL << 1,
	tx_rx			= tx_only | rx_only,
	
	// sync mode
	sync_tx_only	= sync_mask | tx_only,
	sync_rx_only	= sync_mask | rx_only,
	sync_tx_rx		= sync_mask | tx_only | rx_only,

	// aliases
	rx_tx			= tx_rx,
	both			= tx_rx,
	sync_rx_tx		= sync_tx_rx,
	sync_both		= sync_tx_rx,
} mode;
} // namespace mode

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace flow_control {
typedef enum
{
	none			= 0,
	rts				= 1UL << 0,
	cts				= 1UL << 1,
	rts_cts			= rts | cts,

	// aliases
	cts_rts			= rts_cts,
	hardware		= rts_cts,
} flow_control;
} // namespace flow_control

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace stop_bits {
typedef enum
{
	half,
	one,
	one_and_half,
	two,
	
	// aliases
	_0_5	= half,
	_1		= one,
	_1_5	= one_and_half,
	_2		= two,
} stop_bits;
} // namespace stop_bits

/************************************************************************/
/*                                                                      */
/************************************************************************/

const uint32_t	BAUDRATE_DEF			= 115200UL;		// bod
const uint32_t	BAUDRATE_ACCURACY_MAX	= 3;			// percentage
} //namespace uart

/************************************************************************/
/*                                                                      */
/************************************************************************/
using namespace ::mcu::gpio;
using namespace ::mcu::uart;

namespace uart {
template <
			uart_id::uart_id			UartID								,
			mode::mode					Mode		= mode::tx_rx			,
			uint32_t					BaudRate	= BAUDRATE_DEF			,
			uint32_t					DataBits	= 8						,
			stop_bits::stop_bits		StopBits	= stop_bits::one		,
			parity::parity				Parity		= parity::none			,
			flow_control::flow_control	FlowControl	= flow_control::none	,
			pin_id::pin_id				TxPinID		= pin_id::invalid		,
			pin_id::pin_id				RxPinID		= pin_id::invalid		,
			pin_id::pin_id				RtsPinID	= pin_id::invalid		,
			pin_id::pin_id				CtsPinID	= pin_id::invalid		,
			pin_id::pin_id				CkPinID		= pin_id::invalid		,
			uint32_t					BaudRateAccuracyMax	= BAUDRATE_ACCURACY_MAX
		>
class uart_base;

template <
			uart_id::uart_id			UartID								,
			mode::mode					Mode		= mode::tx_rx			,
			uint32_t					BaudRate	= BAUDRATE_DEF			,
			uint32_t					DataBits	= 8						,
			stop_bits::stop_bits		StopBits	= stop_bits::one		,
			parity::parity				Parity		= parity::none			,
			flow_control::flow_control	FlowControl	= flow_control::none	,
			class						PinTx		= gpio_invalid			,
			class						PinRx		= gpio_invalid			,
			class						PinRts		= gpio_invalid			,
			class						PinCts		= gpio_invalid			,
			class						PinCk		= gpio_invalid			,
			uint32_t					BaudRateAccuracyMax	= BAUDRATE_ACCURACY_MAX
		>
class uart_gpio : public uart_base<
			UartID					,
			Mode					,
			BaudRate				,
			DataBits				,
			StopBits				,
			Parity					,
			FlowControl				,
			PinTx::_cfg_::_PinID	,
			PinRx::_cfg_::_PinID	,
			PinRts::_cfg_::_PinID	,
			PinCts::_cfg_::_PinID	,
			PinCk::_cfg_::_PinID	,
			BaudRateAccuracyMax
	>
{};

template <
			uart_id::uart_id			UartID								,
			mode::mode					Mode		= mode::tx_rx			,
			uint32_t					BaudRate	= BAUDRATE_DEF			,
			uint32_t					DataBits	= 8						,
			stop_bits::stop_bits		StopBits	= stop_bits::one		,
			parity::parity				Parity		= parity::none			,
			flow_control::flow_control	FlowControl	= flow_control::none	,
			uint32_t					AltFuncId	= 0						,
			uint32_t					BaudRateAccuracyMax	= BAUDRATE_ACCURACY_MAX
		>
class uart_def;


/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace uart
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__uart_hpp__*/
