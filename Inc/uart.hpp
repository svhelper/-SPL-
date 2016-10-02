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
	tx_only,
	rx_only,
	tx_rx,
	sync_tx_only,
	sync_rx_only,
	sync_tx_rx,

	rx_tx			= tx_rx,
	both			= tx_rx,
	sync_rx_tx		= sync_tx_rx,
	sync_both		= sync_tx_rx,
} mode;
} // namespace mode

namespace flow_control {
typedef enum
{
	none,
	rts,
	cts,
	rts_cts,
} flow_control;
} // namespace flow_control

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
			uint32_t					StopBits	= 1						,
			parity::parity				Parity		= parity::none			,
			flow_control::flow_control	FlowControl	= flow_control::none	,
			class						PinTx		= gpio_invalid			,
			class						PinRx		= gpio_invalid			,
			class						PinRts		= gpio_invalid			,
			class						PinCts		= gpio_invalid			,
			class						PinCk		= gpio_invalid			,
			uint32_t					BaudRateAccuracyMax	= BAUDRATE_ACCURACY_MAX
		>
class uart_base;

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace uart
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__uart_hpp__*/
