#ifndef __spi_hpp__
#define __spi_hpp__

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
namespace spi {

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace spi_id {
typedef enum
{
	spi_1,
	spi_2,
	spi_3,
	spi_4,
	spi_5,
	spi_6,
	spi_7,
	spi_8,
	spi_9,
	spi_10,
	spi_11,
	spi_12,
	spi_13,
	spi_14,
	spi_15,
	spi_16,
	
	invalid			= 0xFFFF,
} spi_id;
} // namespace spi_id

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mode {
typedef enum
{
	// service
	_master_pos		= 8,
	_master_mask	= 1UL << _master_pos,
	_master			= 1UL << _master_pos,
	_slave			= 0UL << _master_pos,
	
	_cpha_$_mask	= 1UL << 0,
	_cpha_0			= 0UL << 0,
	_cpha_1			= 1UL << 0,

	_cpol_$_mask	= 1UL << 1,
	_cpol_0			= 0UL << 1,
	_cpol_1			= 1UL << 1,

	// aliases
    slave_CPOL_0_CPHA_0		= _cpol_0 | _cpha_0 | _slave,
    slave_CPOL_0_CPHA_1		= _cpol_0 | _cpha_1 | _slave,
    slave_CPOL_1_CPHA_0		= _cpol_1 | _cpha_0 | _slave,
    slave_CPOL_1_CPHA_1		= _cpol_1 | _cpha_1 | _slave,

    master_CPOL_0_CPHA_0	= _cpol_0 | _cpha_0 | _master,
    master_CPOL_0_CPHA_1	= _cpol_0 | _cpha_1 | _master,
    master_CPOL_1_CPHA_0	= _cpol_1 | _cpha_0 | _master,
    master_CPOL_1_CPHA_1	= _cpol_1 | _cpha_1 | _master,

	// values
	slave_m0		= slave_CPOL_0_CPHA_0,
	slave_m1		= slave_CPOL_0_CPHA_1,
	slave_m2		= slave_CPOL_1_CPHA_0,
	slave_m3		= slave_CPOL_1_CPHA_1,

	master_m0		= master_CPOL_0_CPHA_0,
	master_m1		= master_CPOL_0_CPHA_1,
	master_m2		= master_CPOL_1_CPHA_0,
	master_m3		= master_CPOL_1_CPHA_1,
} mode;
} // namespace mode

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace data_bits {
typedef enum
{
	// service
	_$sb_pos		= 15,
	_$sb_mask	= (1UL << _$sb_pos),
	_lsb		= (1UL << _$sb_pos),
	_msb		= (0UL << _$sb_pos),
	
	_8			= 8,
	_16			= 16,
	_data_mask	= (1UL << _$sb_pos)-1,
	
	// values
	eight_msb	=  _8 | _msb,
	sixteen_msb	= _16 | _msb,

	eight_lsb	=  _8 | _lsb,
	sixteen_lsb	= _16 | _lsb,
	
	// aliases
	eight_normal	= eight_msb,
	_8_normal		= eight_msb,
	_8_msb			= eight_msb,

	sixteen_normal	= sixteen_msb,
	_16_normal		= sixteen_msb,
	_16_msb			= sixteen_msb,

	eight_reverse	= eight_lsb,
	_8_reverse		= eight_lsb,
	_8_lsb			= eight_lsb,

	sixteen_reverse	= sixteen_lsb,
	_16_reverse		= sixteen_lsb,
	_16_lsb			= sixteen_lsb,

} data_bits;
} // namespace data_bits

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace bus {
typedef enum
{
	// service
	_tx					= 1UL << 0,
	_rx					= 1UL << 1,
	_$_line				= 1UL << 2,
	_nss				= 1UL << 3,

	_1_line				= _$_line,
	_2_line				= 0UL,

	// software nSS
	tx_only				= _tx | _2_line,
	rx_only				= _rx | _2_line,

	tx_rx_2_line		= _tx | _rx | _2_line,
	tx_rx_1_line		= _tx | _rx | _1_line,

	// hardware nSS
	tx_only_nss			= _tx | _2_line | _nss,
	rx_only_nss			= _rx | _2_line | _nss,

	tx_rx_2_line_nss	= _tx | _rx | _2_line | _nss,
	tx_rx_1_line_nss	= _tx | _rx | _1_line | _nss,

	// aliases
} bus;
} // namespace bus

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace crc {
typedef enum
{
	// values
	disable,
	enable,
} crc;
} // namespace crc

/************************************************************************/
/*                                                                      */
/************************************************************************/

const uint32_t	BAUDRATE_MAX			= 0xFFFFFFFF;	// Max possible / Infinite
const uint32_t	BAUDRATE_DEF			= 10000000UL;	// 10MHz
const uint32_t	ALT_FUNC_ID_DEF			= 0;			// Default - the primary of Alternative GPIO Function
const uint32_t	ALT_FUNC_ID_AUTO		= 0xFFFFFFFF;	// Auto choosing of Alternative GPIO Function
const uint32_t	TIMEOUT_INFINITE		= 0xFFFFFFFF;
} //namespace spi

/************************************************************************/
/*                                                                      */
/************************************************************************/
using namespace ::mcu::gpio;
using namespace ::mcu::spi;

namespace spi { namespace config {
	template <
				spi_id::spi_id				SpiID										,
				mode::mode					Mode		= mode::master_CPOL_1_CPHA_1	,
				uint32_t					BaudRateMax	= BAUDRATE_DEF					,
				data_bits::data_bits		DataBits	= data_bits::_8_normal			,
				bus::bus					Bus			= bus::tx_rx_2_line				,
				::mcu::gpio::pin_id::pin_id	SclkPinID	= ::mcu::gpio::pin_id::invalid	,
				::mcu::gpio::pin_id::pin_id	MosiPinID	= ::mcu::gpio::pin_id::invalid	,
				::mcu::gpio::pin_id::pin_id	MisoPinID	= ::mcu::gpio::pin_id::invalid	,
				::mcu::gpio::pin_id::pin_id	NssPinID	= ::mcu::gpio::pin_id::invalid	,
				uint32_t					AltFuncId	= ALT_FUNC_ID_AUTO				,
				bool						UseDma		= false							
			>
	struct config
	{
		static const spi_id::spi_id					_SpiID			= SpiID			;
		static const mode::mode						_Mode			= Mode		    ;
		static const uint32_t						_BaudRateMax	= BaudRateMax	;
		static const data_bits::data_bits			_DataBits		= DataBits		;
		static const bus::bus						_Bus			= Bus			;
		static const ::mcu::gpio::pin_id::pin_id	_SclkPinID		= SclkPinID		;
		static const ::mcu::gpio::pin_id::pin_id	_MosiPinID		= MosiPinID		;
		static const ::mcu::gpio::pin_id::pin_id	_MisoPinID		= MisoPinID		;
		static const ::mcu::gpio::pin_id::pin_id	_NssPinID		= NssPinID		;
		static const uint32_t						_AltFuncId		= AltFuncId		;
		static const bool							_UseDma			= UseDma		;
	};

	template <
				spi_id::spi_id				SpiID										,
				mode::mode					Mode		= mode::master_CPOL_1_CPHA_0	,
				uint32_t					BaudRateMax	= BAUDRATE_DEF					,
				data_bits::data_bits		DataBits	= data_bits::_8_normal			,
				bus::bus					Bus			= bus::tx_rx_2_line				,
				class						SclkPin		= dummy::obj					,
				class						MosiPin		= dummy::obj					,
				class						MisoPin		= dummy::obj					,
				class						NssPin		= dummy::obj					,
				uint32_t					AltFuncId	= ALT_FUNC_ID_AUTO				
			>
	struct config_gpio : public config<
				SpiID			,
				Mode			,
				BaudRateMax		,
				DataBits		,
				Bus				,
				::mcu::gpio::config::get_config<SclkPin	>::_cfg_::_PinID,
				::mcu::gpio::config::get_config<MosiPin	>::_cfg_::_PinID,
				::mcu::gpio::config::get_config<MisoPin	>::_cfg_::_PinID,
				::mcu::gpio::config::get_config<NssPin	>::_cfg_::_PinID,
				AltFuncId		
		>
	{};

	template <
				spi_id::spi_id				SpiID										,
				mode::mode					Mode		= mode::master_CPOL_1_CPHA_0	,
				uint32_t					BaudRateMax	= BAUDRATE_DEF					,
				data_bits::data_bits		DataBits	= data_bits::_8_normal			,
				bus::bus					Bus			= bus::tx_rx_2_line				,
				uint32_t					AltFuncId	= ALT_FUNC_ID_DEF				
			>
	struct config_def : public config<
				SpiID			,
				Mode			,
				BaudRateMax		,
				DataBits		,
				Bus				,
				::mcu::gpio::pin_id::invalid,
				::mcu::gpio::pin_id::invalid,
				::mcu::gpio::pin_id::invalid,
				::mcu::gpio::pin_id::invalid,
				AltFuncId		
		>
	{};

} } //namespace spi::config

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace spi {

template < class CFG >
class spi_port;

template < class CFG >
class spi
	: public spi_port< CFG >
	, public obj::obj< obj::type_id::spi, CFG::_SpiID >
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
			spi_port< CFG >::template on_sysclock_changing<sysclock>::starting();
		}
		static void finished()
		{
			// Enable peripheral
			spi_port< CFG >::template on_sysclock_changing<sysclock>::finished();
		}
	};
	
	template <class sysclock>
	static void init()
	{
		spi_port< CFG >::template init<sysclock>();
	}
	
	static void update()
	{
		spi_port< CFG >::update();
	}

	//////////////////////////////////////////////////////////////////////////
	static bool has_data()
	{
		STATIC_ASSERT(_cfg_::_Bus & ::mcu::spi::bus::_rx, "The SPI is not configured for RX operations!");
		return spi_port< CFG >::has_data();
	}

	static bool is_tx_empty()
	{
		STATIC_ASSERT(_cfg_::_Bus & ::mcu::spi::bus::_tx, "The SPI is not configured for TX operations!");
		return spi_port< CFG >::is_tx_empty();
	}

	//////////////////////////////////////////////////////////////////////////
	static void putc(char c)
	{
		STATIC_ASSERT(_cfg_::_Bus & ::mcu::spi::bus::_tx, "The SPI is not configured for TX operations!");
		spi_port< CFG >::putc(c);
	}

	static char getc()
	{
		STATIC_ASSERT(_cfg_::_Bus & ::mcu::spi::bus::_rx, "The SPI is not configured for RX operations!");
		return spi_port< CFG >::getc();
	}

	//////////////////////////////////////////////////////////////////////////
	static uint32_t read(void* buf, uint32_t size, uint32_t timeout = TIMEOUT_INFINITE)
	{
		STATIC_ASSERT(_cfg_::_Bus & ::mcu::spi::bus::_rx, "The SPI is not configured for RX operations!");
		return spi_port< CFG >::read(buf, size, timeout);
	}

	static uint32_t write(const void* buf, uint32_t size, uint32_t timeout = TIMEOUT_INFINITE)
	{
		STATIC_ASSERT(_cfg_::_Bus & ::mcu::spi::bus::_tx, "The SPI is not configured for TX operations!");
		return spi_port< CFG >::write(buf, size, timeout);
	}
};


/************************************************************************/
/*                                                                      */
/************************************************************************/
template <
			spi_id::spi_id				SpiID										,
			mode::mode					Mode		= mode::master_CPOL_1_CPHA_0	,
			uint32_t					BaudRateMax	= BAUDRATE_DEF					,
			data_bits::data_bits		DataBits	= data_bits::_8_normal			,
			bus::bus					Bus			= bus::tx_rx_2_line				,
			class						SclkPin		= dummy::obj					,
			class						MosiPin		= dummy::obj					,
			class						MisoPin		= dummy::obj					,
			class						NssPin		= dummy::obj					,
			uint32_t					AltFuncId	= ALT_FUNC_ID_AUTO				
		>
class spi_gpio : public spi< config::config_gpio<SpiID, Mode, BaudRateMax, DataBits, Bus, SclkPin, MosiPin, MisoPin, NssPin, AltFuncId> >
{};

template <
			spi_id::spi_id				SpiID										,
			mode::mode					Mode		= mode::master_CPOL_1_CPHA_0	,
			uint32_t					BaudRateMax	= BAUDRATE_DEF					,
			data_bits::data_bits		DataBits	= data_bits::_8_normal			,
			bus::bus					Bus			= bus::tx_rx_2_line				,
			uint32_t					AltFuncId	= ALT_FUNC_ID_DEF				
		>
class spi_def : public spi< config::config_def<SpiID, Mode, BaudRateMax, DataBits, Bus, AltFuncId> >
{};

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace spi
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__spi_hpp__*/
