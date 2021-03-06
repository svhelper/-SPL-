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
#include "CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"
#include "stm32f1xx/stm32f1xx_registers.hpp"
#include "stm32f1xx/stm32f1xx_gpio.hpp"


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
			::mcu::uart::data_bits::data_bits		DataBits	,
			::mcu::uart::stop_bits::stop_bits		StopBits	,
			::mcu::uart::parity::parity				Parity		,
			::mcu::uart::flow_control::flow_control	FlowControl	,
			::mcu::gpio::pin_id::pin_id				CkPinID		,
			::mcu::gpio::pin_id::pin_id				TxPinID		,
			::mcu::gpio::pin_id::pin_id				RxPinID		,
			::mcu::gpio::pin_id::pin_id				RtsPinID	,
			::mcu::gpio::pin_id::pin_id				CtsPinID	,
			uint32_t								BaudRateAccuracyMax,
			uint32_t								AltFuncId
		>
	class uart_set
	{
	public:
		static const bool     _verified   = false;
	};
	
#define _DEF_USART_CHKMODE(UartID, ALT_CFG_ID, MODE, FLOW_CTL, CkPinID, TxPinID, RxPinID, RtsPinID, CtsPinID, DmaID, DmaChTX, DmaChRX)		\
	/* Verification whole configuration, if specified */ \
	template <uint32_t BaudRate, ::mcu::uart::data_bits::data_bits DataBits, ::mcu::uart::stop_bits::stop_bits StopBits, \
		::mcu::uart::parity::parity Parity, uint32_t BaudRateAccuracyMax> \
	class uart_set< ::mcu::uart::uart_id::UartID, ::mcu::uart::mode::MODE, \
		BaudRate, DataBits, StopBits, Parity, \
		::mcu::uart::flow_control::FLOW_CTL, ::mcu::gpio::pin_id::CkPinID, \
		::mcu::gpio::pin_id::TxPinID, ::mcu::gpio::pin_id::RxPinID, \
		::mcu::gpio::pin_id::RtsPinID, ::mcu::gpio::pin_id::CtsPinID, BaudRateAccuracyMax, ALT_CFG_ID > \
	{ \
	public: \
		static const uint32_t _alt_cfg_id = ALT_CFG_ID; \
		static const dma::dma_id::dma_id	_dma_id     = dma::dma_id::DmaID; \
		static const dma::channel::channel	_dma_ch_tx  = dma::channel::DmaChTX; \
		static const dma::channel::channel	_dma_ch_rx  = dma::channel::DmaChRX; \
		static const bool     _verified   = true; \
	}; \
	/* Verification of automatic AF configuration */ \
	template <uint32_t BaudRate, ::mcu::uart::data_bits::data_bits DataBits, ::mcu::uart::stop_bits::stop_bits StopBits, \
		::mcu::uart::parity::parity Parity, uint32_t BaudRateAccuracyMax> \
	class uart_set< ::mcu::uart::uart_id::UartID, ::mcu::uart::mode::MODE, \
		BaudRate, DataBits, StopBits, Parity, \
		::mcu::uart::flow_control::FLOW_CTL, ::mcu::gpio::pin_id::CkPinID, \
		::mcu::gpio::pin_id::TxPinID, ::mcu::gpio::pin_id::RxPinID, \
		::mcu::gpio::pin_id::RtsPinID, ::mcu::gpio::pin_id::CtsPinID, BaudRateAccuracyMax, ::mcu::uart::ALT_FUNC_ID_AUTO > \
	{ \
	public: \
		static const uint32_t _alt_cfg_id = ALT_CFG_ID; \
		static const dma::dma_id::dma_id	_dma_id     = dma::dma_id::DmaID; \
		static const dma::channel::channel	_dma_ch_tx  = dma::channel::DmaChTX; \
		static const dma::channel::channel	_dma_ch_rx  = dma::channel::DmaChRX; \
		static const bool     _verified   = true; \
	}; \
	/* Automatic selection of AF configuration */ \
	template <uint32_t BaudRate, ::mcu::uart::data_bits::data_bits DataBits, ::mcu::uart::stop_bits::stop_bits StopBits, \
		::mcu::uart::parity::parity Parity, uint32_t BaudRateAccuracyMax> \
	class uart_set< ::mcu::uart::uart_id::UartID, ::mcu::uart::mode::MODE, \
		BaudRate, DataBits, StopBits, Parity, \
		::mcu::uart::flow_control::FLOW_CTL, ::mcu::gpio::pin_id::invalid, \
		::mcu::gpio::pin_id::invalid, ::mcu::gpio::pin_id::invalid, \
		::mcu::gpio::pin_id::invalid, ::mcu::gpio::pin_id::invalid, \
		BaudRateAccuracyMax, ALT_CFG_ID > \
	{ \
	public: \
		static const uint32_t _alt_cfg_id = ALT_CFG_ID; \
		static const dma::dma_id::dma_id	_dma_id     = dma::dma_id::DmaID; \
		static const dma::channel::channel	_dma_ch_tx  = dma::channel::DmaChTX; \
		static const dma::channel::channel	_dma_ch_rx  = dma::channel::DmaChRX; \
		static const bool     _verified   = true; \
	};
	
#define DEF_UART_CHKMODE(UartID, ALT_CFG_ID, CkPinID, TxPinID, RxPinID, CtsPinID, RtsPinID, DmaID, DmaChTX, DmaChRX)		\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_only,      none,    CkPinID, TxPinID, invalid, invalid,  invalid,  DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, rx_only,      none,    CkPinID, invalid, RxPinID, invalid,  invalid,  DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_rx,        none,    CkPinID, TxPinID, RxPinID, invalid,  invalid,  DmaID, DmaChTX, DmaChRX) \
	\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_only,      rts,     CkPinID, TxPinID, invalid, RtsPinID, invalid,  DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, rx_only,      rts,     CkPinID, invalid, RxPinID, RtsPinID, invalid,  DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_rx,        rts,     CkPinID, TxPinID, RxPinID, RtsPinID, invalid,  DmaID, DmaChTX, DmaChRX) \
	\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_only,      cts,     CkPinID, TxPinID, invalid, invalid,  CtsPinID, DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, rx_only,      cts,     CkPinID, invalid, RxPinID, invalid,  CtsPinID, DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_rx,        cts,     CkPinID, TxPinID, RxPinID, invalid,  CtsPinID, DmaID, DmaChTX, DmaChRX) \
	\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_only,      rts_cts, CkPinID, TxPinID, invalid, RtsPinID, CtsPinID, DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, rx_only,      rts_cts, CkPinID, invalid, RxPinID, RtsPinID, CtsPinID, DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, tx_rx,        rts_cts, CkPinID, TxPinID, RxPinID, RtsPinID, CtsPinID, DmaID, DmaChTX, DmaChRX)
	
#define DEF_USART_CHKMODE(UartID, ALT_CFG_ID, CkPinID, TxPinID, RxPinID, CtsPinID, RtsPinID, DmaID, DmaChTX, DmaChRX)		\
	DEF_UART_CHKMODE(UartID, ALT_CFG_ID, invalid, TxPinID, RxPinID, CtsPinID, RtsPinID, DmaID, DmaChTX, DmaChRX) \
	\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_only, none,    CkPinID, TxPinID, invalid, invalid,  invalid,  DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_rx_only, none,    CkPinID, invalid, RxPinID, invalid,  invalid,  DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_rx,   none,    CkPinID, TxPinID, RxPinID, invalid,  invalid,  DmaID, DmaChTX, DmaChRX) \
	\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_only, rts,     CkPinID, TxPinID, invalid, RtsPinID, invalid,  DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_rx_only, rts,     CkPinID, invalid, RxPinID, RtsPinID, invalid,  DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_rx,   rts,     CkPinID, TxPinID, RxPinID, RtsPinID, invalid,  DmaID, DmaChTX, DmaChRX) \
	\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_only, cts,     CkPinID, TxPinID, invalid, invalid,  CtsPinID, DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_rx_only, cts,     CkPinID, invalid, RxPinID, invalid,  CtsPinID, DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_rx,   cts,     CkPinID, TxPinID, RxPinID, invalid,  CtsPinID, DmaID, DmaChTX, DmaChRX) \
	\
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_only, rts_cts, CkPinID, TxPinID, invalid, RtsPinID, CtsPinID, DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_rx_only, rts_cts, CkPinID, invalid, RxPinID, RtsPinID, CtsPinID, DmaID, DmaChTX, DmaChRX) \
	_DEF_USART_CHKMODE(UartID, ALT_CFG_ID, sync_tx_rx,   rts_cts, CkPinID, TxPinID, RxPinID, RtsPinID, CtsPinID, DmaID, DmaChTX, DmaChRX)

DEFINE_UARTS(CHKMODE)

	//////////////////////////////////////////////////////////////////////////
	template <
			::mcu::uart::uart_id::uart_id			UartID		,
			uint32_t								AfID		,
			::mcu::uart::mode::mode					Mode		,
			::mcu::uart::flow_control::flow_control	FlowControl	
		>
	class uart_cfg_def
	{
	public:
		static const uart::uart_id::uart_id			_UartID			= UartID				;
		static const uint32_t						_alt_cfg_id		= AfID					;
		static const ::mcu::gpio::pin_id::pin_id	_CkPinID		= ::mcu::gpio::pin_id::invalid	;
		static const ::mcu::gpio::pin_id::pin_id	_TxPinID		= ::mcu::gpio::pin_id::invalid	;
		static const ::mcu::gpio::pin_id::pin_id	_RxPinID		= ::mcu::gpio::pin_id::invalid	;
		static const ::mcu::gpio::pin_id::pin_id	_RtsPinID		= ::mcu::gpio::pin_id::invalid	;
		static const ::mcu::gpio::pin_id::pin_id	_CtsPinID		= ::mcu::gpio::pin_id::invalid	;
		static const dma::dma_id::dma_id			_dma_id      	= dma::dma_id::invalid	;
		static const dma::channel::channel			_dma_ch_tx  	= dma::channel::invalid	;
		static const dma::channel::channel			_dma_ch_rx  	= dma::channel::invalid	;
	};


#define DEF_UART_CFG_DEF(UartID, ALT_CFG_ID, CkPinID, TxPinID, RxPinID, CtsPinID, RtsPinID, DmaID, DmaChTX, DmaChRX)		\
	template < \
			::mcu::uart::mode::mode					Mode		, \
			::mcu::uart::flow_control::flow_control	FlowControl	  \
		> class uart_cfg_def<uart::uart_id::UartID, ALT_CFG_ID, Mode, FlowControl> \
	{ \
	public: \
		static const uart::uart_id::uart_id			_UartID			= uart::uart_id::UartID		; \
		static const uint32_t						_alt_cfg_id		= ALT_CFG_ID				; \
		static const ::mcu::gpio::pin_id::pin_id	_CkPinID		= (Mode        & ::mcu::uart::mode::sync_mask	) ? ::mcu::gpio::pin_id::CkPinID	: ::mcu::gpio::pin_id::invalid	; \
		static const ::mcu::gpio::pin_id::pin_id	_TxPinID		= (Mode        & ::mcu::uart::mode::tx_only		) ? ::mcu::gpio::pin_id::TxPinID	: ::mcu::gpio::pin_id::invalid	; \
		static const ::mcu::gpio::pin_id::pin_id	_RxPinID		= (Mode        & ::mcu::uart::mode::rx_only		) ? ::mcu::gpio::pin_id::RxPinID	: ::mcu::gpio::pin_id::invalid	; \
		static const ::mcu::gpio::pin_id::pin_id	_RtsPinID		= (FlowControl & ::mcu::uart::flow_control::rts	) ? ::mcu::gpio::pin_id::RtsPinID	: ::mcu::gpio::pin_id::invalid	; \
		static const ::mcu::gpio::pin_id::pin_id	_CtsPinID		= (FlowControl & ::mcu::uart::flow_control::cts	) ? ::mcu::gpio::pin_id::CtsPinID	: ::mcu::gpio::pin_id::invalid	; \
		static const dma::dma_id::dma_id			_dma_id        	= dma::dma_id::DmaID		; \
		static const dma::channel::channel			_dma_ch_tx  	= dma::channel::DmaChTX		; \
		static const dma::channel::channel			_dma_ch_rx  	= dma::channel::DmaChRX		; \
	}; \
	
#define DEF_USART_CFG_DEF(UartID, ALT_CFG_ID, CkPinID, TxPinID, RxPinID, CtsPinID, RtsPinID, DmaID, DmaChTX, DmaChRX)		\
	DEF_UART_CFG_DEF(UartID, ALT_CFG_ID, CkPinID, TxPinID, RxPinID, CtsPinID, RtsPinID, DmaID, DmaChTX, DmaChRX)

DEFINE_UARTS(CFG_DEF)

	//////////////////////////////////////////////////////////////////////////

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

	//////////////////////////////////////////////////////////////////////////
	template <uart_id::uart_id UartId, IRQn_Type IRQn>
	class uart_irqn2
	{
	public:
		static const IRQn_Type _IRQn = IRQn;
	};
	//////////////////////////////////////////////////////////////////////////
	template <uart_id::uart_id UartId> class uart_irqn;
	//{
	//	STATIC_ASSERT(false, "The required configuration was not found!");
	//};
	template <> class uart_irqn<uart_id::uart_1> :		public uart_irqn2<uart_id::uart_1, USART1_IRQn> {};
	template <> class uart_irqn<uart_id::uart_2> :		public uart_irqn2<uart_id::uart_2, USART2_IRQn> {};
	template <> class uart_irqn<uart_id::uart_3> :		public uart_irqn2<uart_id::uart_3, USART3_IRQn> {};

/************************************************************************/
/*                                                                      */
/************************************************************************/

} // namespace uart

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace registers {
	template<uart_id::uart_id UartID, uint32_t AfID,
		uint32_t AFIO_MAPR_REMAP_Pos, uint32_t AFIO_MAPR_REMAP_Msk,
		uint32_t UART_REG, uint32_t RCC_APBENR_BB
		>
	class REG_UART_REMAP_BASE
	{
	public:
		static const uint32_t _AFIO_MAPR_REMAP_POS		= AFIO_MAPR_REMAP_Pos;
		static const uint32_t _AFIO_MAPR_REMAP_MASK		= AFIO_MAPR_REMAP_Msk;
		static const uint32_t _AFIO_MAPR_REMAP			= AfID << _AFIO_MAPR_REMAP_POS;
		
		static const uint32_t _AFIO_MAPR2_REMAP_POS		= 0;
		static const uint32_t _AFIO_MAPR2_REMAP_MASK	= 0;
		static const uint32_t _AFIO_MAPR2_REMAP			= 0;

		static const uint32_t _UART_REG					= UART_REG;
		static const uint32_t _RCC_APBENR_BB			= RCC_APBENR_BB;

		static const uint32_t _REMAP_MAX = (_AFIO_MAPR_REMAP_MASK >> _AFIO_MAPR_REMAP_POS) + 1;
		
		STATIC_ASSERT(AfID < _REMAP_MAX, "Internall error: The required configureation cannot be applied in the registers");
	};

	template<uart_id::uart_id UartID, uint32_t AfID> class REG_UART_REMAP;
	
	template<uint32_t AfID> class REG_UART_REMAP<uart_id::uart_1, AfID>
		: public REG_UART_REMAP_BASE<
			uart_id::uart_1, AfID,
			AFIO_MAPR_USART1_REMAP_Pos, AFIO_MAPR_USART1_REMAP_Msk,
			(uint32_t)USART1, FROM_ADDRESS_BIT_POS_TO_BB(&RCC->APB2ENR, RCC_APB2ENR_USART1EN_Pos)
		> {};

	template<uint32_t AfID> class REG_UART_REMAP<uart_id::uart_2, AfID>
		: public REG_UART_REMAP_BASE<
			uart_id::uart_2, AfID,
			AFIO_MAPR_USART2_REMAP_Pos, AFIO_MAPR_USART2_REMAP_Msk,
			(uint32_t)USART2, FROM_ADDRESS_BIT_POS_TO_BB(&RCC->APB1ENR, RCC_APB1ENR_USART2EN_Pos)
		> {};

	template<uint32_t AfID> class REG_UART_REMAP<uart_id::uart_3, AfID>
		: public REG_UART_REMAP_BASE<
			uart_id::uart_3, AfID,
			AFIO_MAPR_USART3_REMAP_Pos, AFIO_MAPR_USART3_REMAP_Msk,
			(uint32_t)USART3, FROM_ADDRESS_BIT_POS_TO_BB(&RCC->APB1ENR, RCC_APB1ENR_USART3EN_Pos)
		> {};

} // namespace registers

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

template < class CFG >
class uart_port
{
private:
	
	typedef stm32::uart::uart_set<
		CFG::_UartID, CFG::_Mode, CFG::_BaudRate, CFG::_DataBits, CFG::_StopBits, CFG::_Parity, CFG::_FlowControl,
		CFG::_CkPinID, CFG::_TxPinID, CFG::_RxPinID, CFG::_RtsPinID, CFG::_CtsPinID,
		CFG::_BaudRateAccuracyMax,
		CFG::_AltFuncId
		> _set_;
	
	STATIC_ASSERT(_set_::_verified != false, "The required configuration was not found! Please check mode<->pins::AF configuration.");
	
	typedef stm32::registers::REG_UART_REMAP<CFG::_UartID, _set_::_alt_cfg_id> REG_REMAP;
	STATIC_ASSERT(_set_::_alt_cfg_id < REG_REMAP::_REMAP_MAX, "Internall error: The required configureation cannot be applied in the registers");

public:
	class _cfg_
	{
	public:
		static const uart_id::uart_id			_UartID			= CFG::_UartID		;
		static const mode::mode					_Mode			= CFG::_Mode		;
		static const uint32_t					_BaudRate		= CFG::_BaudRate	;
		static const data_bits::data_bits		_DataBits		= CFG::_DataBits	;
		static const stop_bits::stop_bits		_StopBits		= CFG::_StopBits	;
		static const parity::parity				_Parity			= CFG::_Parity		;
		static const flow_control::flow_control	_FlowControl	= CFG::_FlowControl	;
		
		static const uint32_t					_alt_cfg_id		= _set_::_alt_cfg_id;

		static const ::mcu::gpio::pin_id::pin_id
			_TxPinID	= stm32::uart::uart_cfg_def<_UartID, _alt_cfg_id, _Mode, _FlowControl>::_TxPinID,
			_RxPinID	= stm32::uart::uart_cfg_def<_UartID, _alt_cfg_id, _Mode, _FlowControl>::_RxPinID,
			_RtsPinID	= stm32::uart::uart_cfg_def<_UartID, _alt_cfg_id, _Mode, _FlowControl>::_RtsPinID,
			_CtsPinID	= stm32::uart::uart_cfg_def<_UartID, _alt_cfg_id, _Mode, _FlowControl>::_CtsPinID,
			_CkPinID	= stm32::uart::uart_cfg_def<_UartID, _alt_cfg_id, _Mode, _FlowControl>::_CkPinID;
		
		STATIC_ASSERT((CFG::_TxPinID	== mcu::gpio::pin_id::invalid ? _TxPinID	: CFG::_TxPinID		) == _TxPinID	, "Invalid configuration for TxPinID");
		STATIC_ASSERT((CFG::_RxPinID	== mcu::gpio::pin_id::invalid ? _RxPinID	: CFG::_RxPinID		) == _RxPinID	, "Invalid configuration for RxPinID");
		STATIC_ASSERT((CFG::_RtsPinID	== mcu::gpio::pin_id::invalid ? _RtsPinID	: CFG::_RtsPinID	) == _RtsPinID	, "Invalid configuration for RtsPinID");
		STATIC_ASSERT((CFG::_CtsPinID	== mcu::gpio::pin_id::invalid ? _CtsPinID	: CFG::_CtsPinID	) == _CtsPinID	, "Invalid configuration for CtsPinID");
		STATIC_ASSERT((CFG::_CkPinID	== mcu::gpio::pin_id::invalid ? _CkPinID	: CFG::_CkPinID		) == _CkPinID	, "Invalid configuration for CkPinID");

		typedef      alt_output<_TxPinID	>	_PinTx								;
		typedef      alt_input <_RxPinID	>	_PinRx								;
		typedef      alt_output<_RtsPinID	>	_PinRts								;
		typedef      alt_input <_CtsPinID	>	_PinCts								;
		typedef      alt_output<_CkPinID	>	_PinCk								;
		
		static const uint32_t					_BaudRateAccuracyMax	= CFG::_BaudRateAccuracyMax;
		static const uint32_t					_AltFuncId				= CFG::_AltFuncId;
		
		static const IRQn_Type					_IRQn					= stm32::uart::uart_irqn<_UartID>::_IRQn;
		
		static const uint32_t _AFIO_MAPR_REMAP_MASK		= REG_REMAP::_AFIO_MAPR_REMAP_MASK	;
		static const uint32_t _AFIO_MAPR_REMAP			= REG_REMAP::_AFIO_MAPR_REMAP		;
		static const uint32_t _AFIO_MAPR2_REMAP_MASK	= REG_REMAP::_AFIO_MAPR2_REMAP_MASK	;
		static const uint32_t _AFIO_MAPR2_REMAP			= REG_REMAP::_AFIO_MAPR2_REMAP		;
		
		static const uint32_t _UART_REG					= REG_REMAP::_UART_REG				;
		static const uint32_t _USART_CR1_UE_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((USART_TypeDef*)_UART_REG)->CR1, USART_CR1_UE_Pos);
		static const uint32_t _USART_CR1_TE_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((USART_TypeDef*)_UART_REG)->CR1, USART_CR1_TE_Pos);
		static const uint32_t _USART_SR_TXE_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((USART_TypeDef*)_UART_REG)->SR, USART_SR_TXE_Pos);
		static const uint32_t _USART_SR_RXNE_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((USART_TypeDef*)_UART_REG)->SR, USART_SR_RXNE_Pos);
		static const uint32_t _USART_SR_TC_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((USART_TypeDef*)_UART_REG)->SR, USART_SR_TC_Pos);
		//static const uint32_t _USART_DR					= (uint32_t)(&((USART_TypeDef*)(uint32_t)_UART_REG)->DR);
		static const uint32_t _USART_DR					= _UART_REG + (uint32_t)(&((USART_TypeDef*)0)->DR);
		
		static const uint32_t _USART_CR1				=
			( USART_CR1_UE ) |
			(	(_Parity == parity::none && _DataBits == data_bits::_9) ? (USART_CR1_M) :
				(_Parity != parity::none && _DataBits == data_bits::_8) ? (USART_CR1_M) :
				(0) ) |
			//( USART_CR1_WAKE ) |
			( (_Parity == parity::odd) ? (USART_CR1_PS) : (0) ) |
			(	(_Parity != parity::none) ? (
				(0 /*USART_CR1_PEIE*/) |		// <-- TODO: Implement IRQ
				(USART_CR1_PCE) |
				(0) ) : (0) ) |
			(	(_Mode & mode::tx_only) ? (
				( USART_CR1_TE ) |
				//( USART_CR1_TXEIE ) |
				( 0 /*USART_CR1_TCIE*/) |		// <-- TODO: Implement IRQ
				(0) ) : (0) ) |
			(	(_Mode & mode::rx_only) ? (
				( USART_CR1_RE ) |
				//( USART_CR1_RXNEIE ) |
				( 0 /*USART_CR1_RWU*/) |
				(0) ) : (0) ) |
			( 0 /*USART_CR1_IDLEIE*/) |		// <-- TODO: Implement IRQ
			( 0 );
		
		static const uint32_t _USART_CR2				=
			(	0 /*USART_CR2_LINEN*/ |
				0 /*USART_CR2_LBDIE*/ |
				0 /*USART_CR2_LBDL*/ |
				0 ) |
			(	(_StopBits == stop_bits::_1  )	? (0) :
				(_StopBits == stop_bits::_0_5)	? (USART_CR2_STOP_0) :
				(_StopBits == stop_bits::_2  )	? (USART_CR2_STOP_1) :
				(_StopBits == stop_bits::_1_5)	? (USART_CR2_STOP_1 | USART_CR2_STOP_0) :
				(0)	) |
			( (_Mode & mode::sync_mask) ? (
				USART_CR2_CLKEN |
				0 /*USART_CR2_CPOL*/ |
				0 /*USART_CR2_CPHA*/ |
				0 ) : 0 ) |
			( 0 /*USART_CR2_ADD*/ ) |
			( 0 );
		
		static const uint32_t _USART_CR3				=
			( (_cfg_::_FlowControl & flow_control::cts) ? (
				(USART_CR3_CTSE) |
				(0 /*USART_CR3_CTSIE*/) |		// <-- TODO: Implement IRQ
				(0) ) : (0) ) |
			( (_cfg_::_FlowControl & flow_control::rts) ? (
				(USART_CR3_RTSE) |
				(0) ) : (0) ) |
			( 0 /*USART_CR3_DMAT*/ ) |			// <-- TODO: Implement DMA
			( 0 /*USART_CR3_DMAR*/ ) |			// <-- TODO: Implement DMA
			( 0 /* smartcard */ ? (
				( 0 /*USART_CR3_SCEN*/ ) |
				( 0 /*USART_CR3_NACK*/ ) |
				(0) ) : (0) ) |
			( 0 /* bidirectional */ ? (
				( 0 /*USART_CR3_HDSEL*/ ) |
				(0) ) : (0) ) |
			( 0 /* IrDA */ ? (
				( 0 /*USART_CR3_IREN*/ ) |
				( 0 /*USART_CR3_IRLP*/ ) |
				(0) ) : (0) ) |
			( 0 /*USART_CR3_EIE*/ ) |			// <-- TODO: Implement IRQ
			( 0 );
		
		static const uint32_t _RCC_APBENR_BB			= REG_REMAP::_RCC_APBENR_BB			;
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
			static const uint32_t _USART_BRR   = (_mantissa << USART_BRR_DIV_Mantissa_Pos) | (_fraction << USART_BRR_DIV_Fraction_Pos);
		};
		STATIC_ASSERT(_baud_calc_::_percent >= _baud_calc_::_percent_min && _baud_calc_::_percent <= _baud_calc_::_percent_max, "The required baud rate cannot be reached on the selected system clock!");
		
	public:
		static void starting()
		{
			// wait for completion of transmission
			while(	IS_BB_REG_SET(_cfg_::_USART_CR1_TE_BB) &&
					( IS_BB_REG_RESET(_cfg_::_USART_SR_TXE_BB) || IS_BB_REG_RESET(_cfg_::_USART_SR_TC_BB) )
				) {}

			// wait for completion of reception
			// <-- TODO: Implement

			// Disable the peripheral
			RESET_BB_REG(_cfg_::_USART_CR1_UE_BB);
		}
		static void finished()
		{
			// Enable peripheral clock

			// BRR Configuration
			((USART_TypeDef*)_cfg_::_UART_REG)->BRR = _baud_calc_::_USART_BRR;

			// CR3 Configuration
			((USART_TypeDef*)_cfg_::_UART_REG)->CR3 = _cfg_::_USART_CR3;

			// CR2 Configuration
			((USART_TypeDef*)_cfg_::_UART_REG)->CR2 = _cfg_::_USART_CR2;

			// CR1 Configuration
			((USART_TypeDef*)_cfg_::_UART_REG)->CR1 = _cfg_::_USART_CR1;

		}
	};
	
	template <class sysclock>
	static void init()
	{
		// Peripheral clock enable
		SET_BB_REG(_cfg_::_RCC_APBENR_BB);
		__NOP();
		
		// Enable GPIO
		::mcu::gpio::atomic<
			typename _cfg_::_PinTx, typename _cfg_::_PinRx,
			typename _cfg_::_PinRts, typename _cfg_::_PinCts,
			typename _cfg_::_PinCk
			>::init();

		// Remap AFIO, if required
		if(_cfg_::_AFIO_MAPR_REMAP_MASK)
			MODIFY_REG(AFIO->MAPR, _cfg_::_AFIO_MAPR_REMAP_MASK, _cfg_::_AFIO_MAPR_REMAP);
		if(_cfg_::_AFIO_MAPR2_REMAP_MASK)
			MODIFY_REG(AFIO->MAPR2, _cfg_::_AFIO_MAPR2_REMAP_MASK, _cfg_::_AFIO_MAPR2_REMAP);
		
		// Disable the peripheral
		RESET_BB_REG(_cfg_::_USART_CR1_UE_BB);		// on_sysclock_changing<sysclock>::starting();
		
		// Set the UART Communication parameters
		on_sysclock_changing<sysclock>::finished();
	}
	
	static void update()
	{
	}

	//////////////////////////////////////////////////////////////////////////
	static bool has_data()
	{
		return IS_BB_REG_SET(_cfg_::_USART_SR_RXNE_BB);
	}

	static bool is_tx_empty()
	{
		return IS_BB_REG_SET(_cfg_::_USART_SR_TXE_BB);
	}

	//////////////////////////////////////////////////////////////////////////
	static void putc(char c)
	{
		while(IS_BB_REG_RESET(_cfg_::_USART_SR_TXE_BB))
		{
			//NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(_cfg_::_IRQn));
			//__WFE();
		}
		WRITE_BB_REG(_cfg_::_USART_DR, c);
	}

	static char getc()
	{
		while(IS_BB_REG_RESET(_cfg_::_USART_SR_RXNE_BB))
		{
			//NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(_cfg_::_IRQn));
			//__WFE();
		}
		return READ_BB_REG(_cfg_::_USART_DR);
	}
	
	//////////////////////////////////////////////////////////////////////////
	static uint32_t read(void* buf, uint32_t size, uint32_t timeout = TIMEOUT_INFINITE)
	{
		uint32_t len = 0;
		uint8_t* p = (uint8_t*)buf;
		for( ;size; len++, size--)
		{
			while(IS_BB_REG_RESET(_cfg_::_USART_SR_RXNE_BB))
			{
				//NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(_cfg_::_IRQn));
				//__WFE();
			}
			*p++ = READ_BB_REG(_cfg_::_USART_DR);
		}
		return len;
	}

	static uint32_t write(const void* buf, uint32_t size, uint32_t timeout = TIMEOUT_INFINITE)
	{
		uint32_t len = 0;
		uint8_t* p = (uint8_t*)buf;
		for( ;size; len++, size--)
		{
			while(IS_BB_REG_RESET(_cfg_::_USART_SR_TXE_BB))
			{
				//NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(_cfg_::_IRQn));
				//__WFE();
			}
			WRITE_BB_REG(_cfg_::_USART_DR, *p++);
		}
		return len;
	}
};

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace uart
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__stm32f1xx_uart_hpp__*/
