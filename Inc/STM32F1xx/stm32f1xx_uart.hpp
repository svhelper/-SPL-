#ifndef __stm32f1xx_uart_hpp__
#define __stm32f1xx_uart_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

//#include <gpio.hpp>
//#include <dma.hpp>
#include <uart.hpp>

//////////////////////////////////////////////////////////////////////////
#include <stm32f1xx.h>
#include <stm32f1xx_registers.hpp>
#include <stm32f1xx_gpio.hpp>


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {
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
			::mcu::gpio::pin_id::pin_id				CkPinID		,
			::mcu::gpio::pin_id::pin_id				TxPinID		,
			::mcu::gpio::pin_id::pin_id				RxPinID		,
			::mcu::gpio::pin_id::pin_id				RtsPinID	,
			::mcu::gpio::pin_id::pin_id				CtsPinID	,
			uint32_t								BaudRateAccuracyMax
		>
	class uart_set;
	
#define _DEF_USART_CHKMODE(UartID, ALT_CFG_ID, MODE, FLOW_CTL, CkPinID, TxPinID, RxPinID, RtsPinID, CtsPinID, DmaID, DmaChTX, DmaChRX)		\
	template <uint32_t BaudRate, uint32_t DataBits, uint32_t StopBits, parity::parity Parity, uint32_t BaudRateAccuracyMax> \
	class uart_set< ::mcu::uart::uart_id::UartID, ::mcu::uart::mode::MODE, \
		BaudRate, DataBits, StopBits, Parity, \
		::mcu::uart::flow_control::FLOW_CTL, ::mcu::gpio::pin_id::CkPinID, \
		::mcu::gpio::pin_id::TxPinID, ::mcu::gpio::pin_id::RxPinID, \
		::mcu::gpio::pin_id::RtsPinID, ::mcu::gpio::pin_id::CtsPinID, BaudRateAccuracyMax > \
	{ \
	public: \
		static const uint32_t _alt_cfg_id = ALT_CFG_ID; \
		static const dma::dma_id::dma_id	_dma        = dma::dma_id::DmaID; \
		static const dma::channel::channel	_dma_ch_tx  = dma::channel::DmaChTX; \
		static const dma::channel::channel	_dma_ch_rx  = dma::channel::DmaChRX; \
		static const bool     _verified   = true; \
	};
	
#define DEF_UART_CHKMODE(UartID, ALT_CFG_ID, TxPinID, RxPinID, CtsPinID, RtsPinID, Dma, DmaChTX, DmaChRX)		\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_only,      none,    invalid, TxPinID, invalid, invalid,  invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, rx_only,      none,    invalid, invalid, RxPinID, invalid,  invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_rx,        none,    invalid, TxPinID, RxPinID, invalid,  invalid,  Dma, DmaChTX, DmaChRX) \
	\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_only,      rts,     invalid, TxPinID, invalid, RtsPinID, invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, rx_only,      rts,     invalid, invalid, RxPinID, RtsPinID, invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_rx,        rts,     invalid, TxPinID, RxPinID, RtsPinID, invalid,  Dma, DmaChTX, DmaChRX) \
	\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_only,      cts,     invalid, TxPinID, invalid, invalid,  CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, rx_only,      cts,     invalid, invalid, RxPinID, invalid,  CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_rx,        cts,     invalid, TxPinID, RxPinID, invalid,  CtsPinID, Dma, DmaChTX, DmaChRX) \
	\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_only,      rts_cts, invalid, TxPinID, invalid, RtsPinID, CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, rx_only,      rts_cts, invalid, invalid, RxPinID, RtsPinID, CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_rx,        rts_cts, invalid, TxPinID, RxPinID, RtsPinID, CtsPinID, Dma, DmaChTX, DmaChRX)
	
#define DEF_USART_CHKMODE(UartID, ALT_CFG_ID, CkPinID, TxPinID, RxPinID, CtsPinID, RtsPinID, Dma, DmaChTX, DmaChRX)		\
	DEF_UART_CHKMODE(UartID, ALT_CFG_ID, TxPinID, RxPinID, CtsPinID, RtsPinID, Dma, DmaChTX, DmaChRX) \
	\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_only, none,    CkPinID, TxPinID, invalid, invalid,  invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_rx_only, none,    CkPinID, invalid, RxPinID, invalid,  invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_rx,   none,    CkPinID, TxPinID, RxPinID, invalid,  invalid,  Dma, DmaChTX, DmaChRX) \
	\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_only, rts,     CkPinID, TxPinID, invalid, RtsPinID, invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_rx_only, rts,     CkPinID, invalid, RxPinID, RtsPinID, invalid,  Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_rx,   rts,     CkPinID, TxPinID, RxPinID, RtsPinID, invalid,  Dma, DmaChTX, DmaChRX) \
	\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_only, cts,     CkPinID, TxPinID, invalid, invalid,  CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_rx_only, cts,     CkPinID, invalid, RxPinID, invalid,  CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_rx,   cts,     CkPinID, TxPinID, RxPinID, invalid,  CtsPinID, Dma, DmaChTX, DmaChRX) \
	\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_only, rts_cts, CkPinID, TxPinID, invalid, RtsPinID, CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_rx_only, rts_cts, CkPinID, invalid, RxPinID, RtsPinID, CtsPinID, Dma, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_rx,   rts_cts, CkPinID, TxPinID, RxPinID, RtsPinID, CtsPinID, Dma, DmaChTX, DmaChRX)

DEFINE_UARTS(CHKMODE)


/************************************************************************/
/*                                                                      */
/************************************************************************/
template <uart_id::uart_id UartId, class sysclock, uint32_t PCLK_Hz>
class uart_cfg2
{
public:
	static const uint32_t	_PCLK_Hz = PCLK_Hz;
};
//////////////////////////////////////////////////////////////////////////
template <uart_id::uart_id UartId, class sysclock> class uart_cfg   :
	public uart_cfg2<UartId, sysclock, sysclock::_cfg_::_PCLK1_Hz> {};
template <class sysclock> class uart_cfg<uart_id::uart_1, sysclock> :
	public uart_cfg2<uart_id::uart_1, sysclock, sysclock::_cfg_::_PCLK2_Hz> {};
template <class sysclock> class uart_cfg<uart_id::uart_6, sysclock> :
	public uart_cfg2<uart_id::uart_1, sysclock, sysclock::_cfg_::_PCLK2_Hz> {};

/************************************************************************/
/*                                                                      */
/************************************************************************/

} // namespace uart

/************************************************************************/
/*                                                                      */
/************************************************************************/
//namespace registers {
//} // namespace registers

/************************************************************************/
/*                                                                      */
/************************************************************************/

} // namespace stm32

/************************************************************************/
/*                                                                      */
/************************************************************************/
using namespace ::mcu::gpio;
using namespace ::mcu::uart;

namespace uart {
template <
			uart_id::uart_id			UartID				,
			mode::mode					Mode				,
			uint32_t					BaudRate			,
			uint32_t					DataBits			,
			uint32_t					StopBits			,
			parity::parity				Parity				,
			flow_control::flow_control	FlowControl			,
			class						PinTx				,
			class						PinRx				,
			class						PinRts				,
			class						PinCts				,
			class						PinCk				,
			uint32_t					BaudRateAccuracyMax	
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
	
	static const bool _verified = _set_::_verified;			// Check mode<->pins configuration

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
		static const uint32_t					_BaudRateAccuracyMax	= BaudRateAccuracyMax;
	};
	
public:
	template <class sysclock>
	class on_sysclock_changing
	{
	protected:
		on_sysclock_changing();
		~on_sysclock_changing();
	
	public:
		struct _baud_calc_
		{
			typedef stm32::uart::uart_cfg<_cfg_::_UartID, sysclock> uart_cfg;
			static const uint32_t _PCLK_Hz = uart_cfg::_PCLK_Hz;
			static const int32_t  _percent_max = +(int32_t)_cfg_::_BaudRateAccuracyMax;
			static const int32_t  _percent_min = -(int32_t)_cfg_::_BaudRateAccuracyMax;
		
			static const uint64_t _mantissa100 = (100ULL * _PCLK_Hz) / (16ULL * _cfg_::_BaudRate);
			static const uint32_t _mantissa    = (uint32_t)(_mantissa100 / 100);
			static const uint32_t _fraction    = (uint32_t)(((_mantissa100 % 100) * 16 + 100/2) / 100);
			
			STATIC_ASSERT(_mantissa <= (USART_BRR_DIV_Mantissa_Msk >> USART_BRR_DIV_Mantissa_Pos), "The baud rate too small, to fit BRR register");
			STATIC_ASSERT(_fraction <= (USART_BRR_DIV_Fraction_Msk >> USART_BRR_DIV_Fraction_Pos), "The baud rate too small, to fit BRR register");
			
			// calculate actual baud rate
			static const uint32_t _BaudRate_Real = (_mantissa || _fraction) ? (100ULL * _PCLK_Hz) / (16 * (_mantissa * 100ULL + _fraction * 100 / 16)) : 0xFFFFFFFF;
			
			// calculate baud rate accuracy
			static const int32_t  _percent = ((int32_t)_BaudRate_Real - (int32_t)_cfg_::_BaudRate) * 100 / (int32_t)_cfg_::_BaudRate;

			// assemble value for BRR register
			static const uint32_t _usart_BRR   = (_mantissa << USART_BRR_DIV_Mantissa_Pos) | (_fraction << USART_BRR_DIV_Fraction_Pos);
		};
		STATIC_ASSERT(_baud_calc_::_percent >= _baud_calc_::_percent_min && _baud_calc_::_percent <= _baud_calc_::_percent_max, "The required baud rate cannot be reached on the selected system clock!");
		
	public:
		static void starting()
		{
			// Disable peripheral clock
//			if(_cfg_::_rcc_apb2enr_mask)
//			{
//				SET_BIT (RCC->APB2ENR, _cfg_::_rcc_apb2enr);
//				__NOP();
//			}
		}
		static void finished()
		{
			// Enable peripheral clock
//			if(_cfg_::_rcc_apb2enr_mask)
//			{
//				SET_BIT (RCC->APB2ENR, _cfg_::_rcc_apb2enr);
//				__NOP();
//			}
		}
	};
	
	template <class sysclock>
	static void init()
	{
		on_sysclock_changing<sysclock>::finished();
	}
	
	static void update()
	{
	}
};

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace uart
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__stm32f1xx_uart_hpp__*/
