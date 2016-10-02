#ifndef __stm32f1xx_uart_hpp__
#define __stm32f1xx_uart_hpp__

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
#include <stm32f1xx_gpio.hpp>


/************************************************************************/
/*                                                                      */
/************************************************************************/
//	----------##---+------+--+-----+-----+-----+-----+-----+---+----+----+
//	UART/USART##CMD|  ID  |AF|  CK |  TX |  RX | CTS | RTS |DMA|chTX|chRX|
//	----------##---+------+--+-----+-----+-----+-----+-----+---+----+----+
#define DEFINE_UARTS(CMD)                                                 \
	DEF_USART_##CMD(uart_1, 0,  PA8,  PA9, PA10, PA11, PA12,  1,   4,   5)\
	DEF_USART_##CMD(uart_1, 1,  PA8,  PB6,  PB7, PA11, PA12,  1,   4,   5)\
	                                                                      \
	DEF_USART_##CMD(uart_2, 0,  PA4,  PA2,  PA3,  PA0,  PA1,  1,   7,   6)\
	DEF_USART_##CMD(uart_2, 0,  PD7,  PD5,  PD6,  PD3,  PD4,  1,   7,   6)\
	                                                                      \
	DEF_USART_##CMD(uart_3, 0, PB12, PB10, PB11, PB13, PB14,  1,   2,   3)\
	DEF_USART_##CMD(uart_3, 1, PC12, PC10, PC11, PB13, PB14,  1,   2,   3)\
	DEF_USART_##CMD(uart_3, 2, PD10,  PD8,  PD9, PD11, PD12,  1,   2,   3)\
//	----------##---+------+--+-----+-----+-----+-----+-----+---+----+----+

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {

/************************************************************************/
/*                                                                      */
/************************************************************************/
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
} //namespace uart

namespace stm32 {
	
namespace uart {
	using namespace ::mcu::uart;
	using namespace ::mcu::gpio;
	
	//////////////////////////////////////////////////////////////////////////
	template <
			::mcu::uart::uart_id::uart_id			UartID		,
			::mcu::uart::mode::mode					Mode		,
			uint32_t								BaudRate	,
			uint32_t								DataBits	,
			uint32_t								StopBits	,
			::mcu::uart::parity::parity				Parity		,
			::mcu::uart::flow_control::flow_control	FlowControl	,
			gpio::pin_id::pin_id					CkPinID		,
			gpio::pin_id::pin_id					TxPinID		,
			gpio::pin_id::pin_id					RxPinID		,
			gpio::pin_id::pin_id					RtsPinID	,
			gpio::pin_id::pin_id					CtsPinID	,
			uint32_t								BaudRateAccuracyMax
		>
	class uart_set;
	
#define _DEF_USART_MODE(UartID, ALT_CFG_ID, MODE, FLOW_CTL, CkPinID, TxPinID, RxPinID, RtsPinID, CtsPinID, Dma, DmaChTX, DmaChRX)		\
	template <uint32_t BaudRate, uint32_t DataBits, uint32_t StopBits, parity::parity Parity, uint32_t BaudRateAccuracyMax> \
	class uart_set< ::mcu::uart::uart_id::UartID, ::mcu::uart::mode::MODE, BaudRate, DataBits, StopBits, Parity, \
		::mcu::uart::flow_control::FLOW_CTL, gpio::pin_id::CkPinID, gpio::pin_id::TxPinID, gpio::pin_id::RxPinID, gpio::pin_id::RtsPinID, gpio::pin_id::CtsPinID, \
		BaudRateAccuracyMax > \
	{ \
	public: \
		static const uint32_t _alt_cfg_id = ALT_CFG_ID; \
		static const uint32_t _dma        = Dma; \
		static const uint32_t _dma_ch_tx  = DmaChTX; \
		static const uint32_t _dma_ch_rx  = DmaChRX; \
		static const bool     _verified   = true; \
	};
	
#define DEF_UART_MODE(UartID, ALT_CFG_ID, TxPinID, RxPinID, CtsPinID, RtsPinID, Dma, DmaChTX, DmaChRX)		\
	_DEF_USART_MODE(UartID, ALT_CFG_ID, tx_only,      none,    invalid, TxPinID, invalid, invalid,  invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, rx_only,      none,    invalid, invalid, RxPinID, invalid,  invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, tx_rx,        none,    invalid, TxPinID, RxPinID, invalid,  invalid,  Dma, DmaChTX, DmaChRX) \
	\
	_DEF_USART_MODE(UartID, ALT_CFG_ID, tx_only,      rts,     invalid, TxPinID, invalid, RtsPinID, invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, rx_only,      rts,     invalid, invalid, RxPinID, RtsPinID, invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, tx_rx,        rts,     invalid, TxPinID, RxPinID, RtsPinID, invalid,  Dma, DmaChTX, DmaChRX) \
	\
	_DEF_USART_MODE(UartID, ALT_CFG_ID, tx_only,      cts,     invalid, TxPinID, invalid, invalid,  CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, rx_only,      cts,     invalid, invalid, RxPinID, invalid,  CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, tx_rx,        cts,     invalid, TxPinID, RxPinID, invalid,  CtsPinID, Dma, DmaChTX, DmaChRX) \
	\
	_DEF_USART_MODE(UartID, ALT_CFG_ID, tx_only,      rts_cts, invalid, TxPinID, invalid, RtsPinID, CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, rx_only,      rts_cts, invalid, invalid, RxPinID, RtsPinID, CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, tx_rx,        rts_cts, invalid, TxPinID, RxPinID, RtsPinID, CtsPinID, Dma, DmaChTX, DmaChRX)
	
#define DEF_USART_MODE(UartID, ALT_CFG_ID, CkPinID, TxPinID, RxPinID, CtsPinID, RtsPinID, Dma, DmaChTX, DmaChRX)		\
	DEF_UART_MODE(UartID, ALT_CFG_ID, TxPinID, RxPinID, CtsPinID, RtsPinID, Dma, DmaChTX, DmaChRX) \
	\
	_DEF_USART_MODE(UartID, ALT_CFG_ID, sync_tx_only, none,    CkPinID, TxPinID, invalid, invalid,  invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, sync_rx_only, none,    CkPinID, invalid, RxPinID, invalid,  invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, sync_tx_rx,   none,    CkPinID, TxPinID, RxPinID, invalid,  invalid,  Dma, DmaChTX, DmaChRX) \
	\
	_DEF_USART_MODE(UartID, ALT_CFG_ID, sync_tx_only, rts,     CkPinID, TxPinID, invalid, RtsPinID, invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, sync_rx_only, rts,     CkPinID, invalid, RxPinID, RtsPinID, invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, sync_tx_rx,   rts,     CkPinID, TxPinID, RxPinID, RtsPinID, invalid,  Dma, DmaChTX, DmaChRX) \
	\
	_DEF_USART_MODE(UartID, ALT_CFG_ID, sync_tx_only, cts,     CkPinID, TxPinID, invalid, invalid,  CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, sync_rx_only, cts,     CkPinID, invalid, RxPinID, invalid,  CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, sync_tx_rx,   cts,     CkPinID, TxPinID, RxPinID, invalid,  CtsPinID, Dma, DmaChTX, DmaChRX) \
	\
	_DEF_USART_MODE(UartID, ALT_CFG_ID, sync_tx_only, rts_cts, CkPinID, TxPinID, invalid, RtsPinID, CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, sync_rx_only, rts_cts, CkPinID, invalid, RxPinID, RtsPinID, CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_MODE(UartID, ALT_CFG_ID, sync_tx_rx,   rts_cts, CkPinID, TxPinID, RxPinID, RtsPinID, CtsPinID, Dma, DmaChTX, DmaChRX)

DEFINE_UARTS(MODE)
	
} // namespace uart

namespace registers {
	//////////////////////////////////////////////////////////////////////////

} // namespace registers
} // namespace stm32

/************************************************************************/
/*                                                                      */
/************************************************************************/
using namespace ::mcu::gpio;
using namespace ::mcu::uart;

namespace uart {
template <
			uart_id::uart_id			UartID								,
			mode::mode					Mode		= mode::tx_rx			,
			uint32_t					BaudRate	= 115200				,
			uint32_t					DataBits	= 8						,
			uint32_t					StopBits	= 1						,
			parity::parity				Parity		= parity::none			,
			flow_control::flow_control	FlowControl	= flow_control::none	,
			class						PinTx		= gpio_invalid			,
			class						PinRx		= gpio_invalid			,
			class						PinRts		= gpio_invalid			,
			class						PinCts		= gpio_invalid			,
			class						PinCk		= gpio_invalid			,
			uint32_t					BaudRateAccuracyMax	= 3				
		>
class uart_base
{
private:
	
	typedef stm32::uart::uart_set<
		UartID, Mode, BaudRate, DataBits, StopBits, Parity, FlowControl,
		PinCk::_cfg_::_PinID,
		PinTx::_cfg_::_PinID, PinRx::_cfg_::_PinID,
		PinRts::_cfg_::_PinID, PinCts::_cfg_::_PinID,
		BaudRateAccuracyMax
		> _set_;
	
	static const bool _verified = _set_::_verified;

public:
	class _cfg_
	{
	public:
		static const uart_id::uart_id			_UartID			= UartID		;
		static const mode::mode					_Mode			= Mode			;
		static const uint32_t					_BaudRate		= BaudRate		;
		static const uint32_t					_DataBits		= DataBits		;
		static const uint32_t					_StopBits		= StopBits		;
		static const parity::parity				_Parity			= Parity		;
		static const flow_control::flow_control	_FlowControl	= FlowControl	;
		typedef      PinTx						_PinTx							;
		typedef      PinRx						_PinRx							;
		typedef      PinRts						_PinRts							;
		typedef      PinCts						_PinCts							;
		typedef      PinCk						_PinCk							;
	};
	
public:
	static void init()
	{
		// Enable peripheral clock
//		if(_cfg_::_rcc_apb2enr_mask)
//		{
//			SET_BIT (RCC->APB2ENR, _cfg_::_rcc_apb2enr);
//			__NOP();
//		}
	}
	
	static void update()
	{
	}
};

} // namespace uart
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__stm32f1xx_uart_hpp__*/
