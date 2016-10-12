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
namespace data_bits {
typedef enum
{
	four	= 4,
	five	= 5,
	six		= 6,
	seven	= 7,
	eight	= 8,
	nine	= 9,
	
	// aliases
	_4		= four,
	_5		= five,
	_6		= six,
	_7		= seven,
	_8		= eight,
	_9		= nine,
} data_bits;
} // namespace data_bits

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
const uint32_t	ALT_FUNC_ID_DEF			= 0;			// Default - the primary of Alternative GPIO Function
const uint32_t	ALT_FUNC_ID_AUTO		= 0xFFFFFFFF;	// Auto choosing of Alternative GPIO Function
const uint32_t	TIMEOUT_INFINITE		= 0xFFFFFFFF;
} //namespace uart

/************************************************************************/
/*                                                                      */
/************************************************************************/
using namespace ::mcu::gpio;
using namespace ::mcu::uart;

namespace uart { namespace config {
	template <
				uart_id::uart_id			UartID										,
				mode::mode					Mode		= mode::tx_rx					,
				uint32_t					BaudRate	= BAUDRATE_DEF					,
				data_bits::data_bits		DataBits	= data_bits::eight				,
				stop_bits::stop_bits		StopBits	= stop_bits::one				,
				parity::parity				Parity		= parity::none					,
				flow_control::flow_control	FlowControl	= flow_control::none			,
				::mcu::gpio::pin_id::pin_id	TxPinID		= ::mcu::gpio::pin_id::invalid	,
				::mcu::gpio::pin_id::pin_id	RxPinID		= ::mcu::gpio::pin_id::invalid	,
				::mcu::gpio::pin_id::pin_id	RtsPinID	= ::mcu::gpio::pin_id::invalid	,
				::mcu::gpio::pin_id::pin_id	CtsPinID	= ::mcu::gpio::pin_id::invalid	,
				::mcu::gpio::pin_id::pin_id	CkPinID		= ::mcu::gpio::pin_id::invalid	,
				uint32_t					BaudRateAccuracyMax	= BAUDRATE_ACCURACY_MAX	,
				uint32_t					AltFuncId	= ALT_FUNC_ID_AUTO				
			>
	struct config
	{
		static const uart_id::uart_id				_UartID			= UartID		;
		static const mode::mode						_Mode			= Mode		    ;
		static const uint32_t						_BaudRate		= BaudRate	    ;
		static const data_bits::data_bits			_DataBits		= DataBits	    ;
		static const stop_bits::stop_bits			_StopBits		= StopBits	    ;
		static const parity::parity					_Parity			= Parity		;
		static const flow_control::flow_control		_FlowControl	= FlowControl	;
		static const ::mcu::gpio::pin_id::pin_id	_TxPinID		= TxPinID		;
		static const ::mcu::gpio::pin_id::pin_id	_RxPinID		= RxPinID		;
		static const ::mcu::gpio::pin_id::pin_id	_RtsPinID		= RtsPinID	    ;
		static const ::mcu::gpio::pin_id::pin_id	_CtsPinID		= CtsPinID	    ;
		static const ::mcu::gpio::pin_id::pin_id	_CkPinID		= CkPinID		;
		static const uint32_t						_BaudRateAccuracyMax	= BaudRateAccuracyMax;
		static const uint32_t						_AltFuncId		= AltFuncId		;
	};

	
	template <
				uart_id::uart_id			UartID								,
				mode::mode					Mode		= mode::tx_rx			,
				uint32_t					BaudRate	= BAUDRATE_DEF			,
				data_bits::data_bits		DataBits	= data_bits::eight		,
				stop_bits::stop_bits		StopBits	= stop_bits::one		,
				parity::parity				Parity		= parity::none			,
				flow_control::flow_control	FlowControl	= flow_control::none	,
				class						PinTx		= dummy::obj			,
				class						PinRx		= dummy::obj			,
				class						PinRts		= dummy::obj			,
				class						PinCts		= dummy::obj			,
				class						PinCk		= dummy::obj			,
				uint32_t					BaudRateAccuracyMax	= BAUDRATE_ACCURACY_MAX,
				uint32_t					AltFuncId	= ALT_FUNC_ID_AUTO				
			>
	struct config_gpio : public config<
				UartID					,
				Mode					,
				BaudRate				,
				DataBits				,
				StopBits				,
				Parity					,
				FlowControl				,
				::mcu::gpio::config::get_config<PinTx >::_cfg_::_PinID	,
				::mcu::gpio::config::get_config<PinRx >::_cfg_::_PinID	,
				::mcu::gpio::config::get_config<PinRts>::_cfg_::_PinID	,
				::mcu::gpio::config::get_config<PinCts>::_cfg_::_PinID	,
				::mcu::gpio::config::get_config<PinCk >::_cfg_::_PinID	,
				BaudRateAccuracyMax		,
				AltFuncId				
		>
	{};

	template <
				uart_id::uart_id			UartID								,
				mode::mode					Mode		= mode::tx_rx			,
				uint32_t					BaudRate	= BAUDRATE_DEF			,
				data_bits::data_bits		DataBits	= data_bits::eight		,
				stop_bits::stop_bits		StopBits	= stop_bits::one		,
				parity::parity				Parity		= parity::none			,
				flow_control::flow_control	FlowControl	= flow_control::none	,
				uint32_t					AltFuncId	= ALT_FUNC_ID_DEF		,
				uint32_t					BaudRateAccuracyMax	= BAUDRATE_ACCURACY_MAX
			>
	struct config_def : public config<
				UartID						,
				Mode						,
				BaudRate					,
				DataBits					,
				StopBits					,
				Parity						,
				FlowControl					,
				::mcu::gpio::pin_id::invalid,
				::mcu::gpio::pin_id::invalid,
				::mcu::gpio::pin_id::invalid,
				::mcu::gpio::pin_id::invalid,
				::mcu::gpio::pin_id::invalid,
				BaudRateAccuracyMax			,
				AltFuncId					
		>
	{};

} } //namespace uart::config

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace uart {

template < class CFG >
class uart_port;

template < class CFG >
class uart
	: public uart_port< CFG >
	, public obj::obj< obj::type_id::uart, CFG::_UartID >
{
public:
	typedef CFG _cfg_;

public:
	template <class sysclock>
	class on_sysclock_changing
	{
	protected:
		on_sysclock_changing();
		~on_sysclock_changing();

	public:
		static void starting()
		{
			// Disable the peripheral
			uart_port< CFG >::template on_sysclock_changing<sysclock>::starting();
		}
		static void finished()
		{
			// Enable peripheral
			uart_port< CFG >::template on_sysclock_changing<sysclock>::finished();
		}
	};
	
	template <class sysclock>
	static void init()
	{
		uart_port< CFG >::template init<sysclock>();
	}
	
	static void update()
	{
		uart_port< CFG >::update();
	}

	//////////////////////////////////////////////////////////////////////////
	static bool has_data()
	{
		STATIC_ASSERT(_cfg_::_Mode & ::mcu::uart::mode::rx_only, "The UART is not configured for RX operations!");
		return uart_port< CFG >::has_data();
	}

	static bool is_tx_empty()
	{
		STATIC_ASSERT(_cfg_::_Mode & ::mcu::uart::mode::tx_only, "The UART is not configured for TX operations!");
		return uart_port< CFG >::is_tx_empty();
	}

	//////////////////////////////////////////////////////////////////////////
	static void putc(char c)
	{
		STATIC_ASSERT(_cfg_::_Mode & ::mcu::uart::mode::tx_only, "The UART is not configured for TX operations!");
		uart_port< CFG >::putc(c);
	}

	static char getc()
	{
		STATIC_ASSERT(_cfg_::_Mode & ::mcu::uart::mode::rx_only, "The UART is not configured for RX operations!");
		return uart_port< CFG >::getc();
	}

	//////////////////////////////////////////////////////////////////////////
	static uint32_t read(void* buf, uint32_t size, uint32_t timeout = TIMEOUT_INFINITE)
	{
		STATIC_ASSERT(_cfg_::_Mode & ::mcu::uart::mode::rx_only, "The UART is not configured for RX operations!");
		return uart_port< CFG >::read(buf, size, timeout);
	}

	static uint32_t write(const void* buf, uint32_t size, uint32_t timeout = TIMEOUT_INFINITE)
	{
		STATIC_ASSERT(_cfg_::_Mode & ::mcu::uart::mode::tx_only, "The UART is not configured for TX operations!");
		return uart_port< CFG >::write(buf, size, timeout);
	}
};

/************************************************************************/
/*                                                                      */
/************************************************************************/
template <
			uart_id::uart_id			UartID								,
			mode::mode					Mode		= mode::tx_rx			,
			uint32_t					BaudRate	= BAUDRATE_DEF			,
			data_bits::data_bits		DataBits	= data_bits::eight		,
			stop_bits::stop_bits		StopBits	= stop_bits::one		,
			parity::parity				Parity		= parity::none			,
			flow_control::flow_control	FlowControl	= flow_control::none	,
			class						PinTx		= dummy::obj			,
			class						PinRx		= dummy::obj			,
			class						PinRts		= dummy::obj			,
			class						PinCts		= dummy::obj			,
			class						PinCk		= dummy::obj			,
			uint32_t					BaudRateAccuracyMax	= BAUDRATE_ACCURACY_MAX,
			uint32_t					AltFuncId	= ALT_FUNC_ID_AUTO		
		>
class uart_gpio : public uart< config::config_gpio<
			UartID				,
			Mode				,
			BaudRate			,
			DataBits			,
			StopBits			,
			Parity				,
			FlowControl			,
			PinTx				,
			PinRx				,
			PinRts				,
			PinCts				,
			PinCk				,
			BaudRateAccuracyMax	,
			AltFuncId
	> >
{};

/************************************************************************/
/*                                                                      */
/************************************************************************/
template <
			uart_id::uart_id			UartID								,
			mode::mode					Mode		= mode::tx_rx			,
			uint32_t					BaudRate	= BAUDRATE_DEF			,
			data_bits::data_bits		DataBits	= data_bits::eight		,
			stop_bits::stop_bits		StopBits	= stop_bits::one		,
			parity::parity				Parity		= parity::none			,
			flow_control::flow_control	FlowControl	= flow_control::none	,
			uint32_t					AltFuncId	= 0						,
			uint32_t					BaudRateAccuracyMax	= BAUDRATE_ACCURACY_MAX
		>
class uart_def : public uart< config::config_def<
			UartID			,
			Mode			,
			BaudRate		,
			DataBits		,
			StopBits		,
			Parity			,
			FlowControl		,
			AltFuncId		,
			BaudRateAccuracyMax
	> >
{};

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace uart
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__uart_hpp__*/
