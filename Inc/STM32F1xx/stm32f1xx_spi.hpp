#ifndef __stm32f1xx_spi_hpp__
#define __stm32f1xx_spi_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

//#include <gpio.hpp>
//#include <dma.hpp>
#include <spi.hpp>

//////////////////////////////////////////////////////////////////////////
#include "CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"
#include "stm32f1xx/stm32f1xx_registers.hpp"
#include "stm32f1xx/stm32f1xx_gpio.hpp"


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {
namespace stm32 {
	
namespace spi {
	using namespace ::mcu::spi;
	using namespace ::mcu::gpio;
	
	//////////////////////////////////////////////////////////////////////////
	template <
			::mcu::spi::spi_id::spi_id				SpiID		,
			::mcu::spi::mode::mode					Mode		,
			uint32_t /*::mcu::spi::mode::mode*/		Master		,
			uint32_t								BaudRate	,
			::mcu::spi::data_bits::data_bits		DataBits	,
			::mcu::spi::bus::bus					Bus			,
			::mcu::gpio::pin_id::pin_id				SclkPinID	,
			::mcu::gpio::pin_id::pin_id				MosiPinID	,
			::mcu::gpio::pin_id::pin_id				MisoPinID	,
			::mcu::gpio::pin_id::pin_id				NssPinID	,
			uint32_t								AltFuncId	
		>
	class spi_set
	{
	public:
		static const bool     _verified   = false;
	};
	
#define _DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, Master, Bus, SclkPinID, MosiPinID, MisoPinID, NssPinID, DmaID, DmaChTX, DmaChRX)		\
	/* Verification whole configuration, if specified */ \
	template < ::mcu::spi::mode::mode Mode, uint32_t BaudRate, ::mcu::spi::data_bits::data_bits DataBits > \
	class spi_set< ::mcu::spi::spi_id::SpiID, Mode, ::mcu::spi::mode::Master, BaudRate, DataBits, \
		::mcu::spi::bus::Bus, \
		::mcu::gpio::pin_id::SclkPinID, ::mcu::gpio::pin_id::MosiPinID, \
		::mcu::gpio::pin_id::MisoPinID, ::mcu::gpio::pin_id::NssPinID, \
		ALT_CFG_ID > \
	{ \
	public: \
		static const uint32_t _alt_cfg_id = ALT_CFG_ID; \
		static const bool     _verified   = true; \
	}; \
	/* Verification of automatic AF configuration */ \
	template < ::mcu::spi::mode::mode Mode, uint32_t BaudRate, ::mcu::spi::data_bits::data_bits DataBits > \
	class spi_set< ::mcu::spi::spi_id::SpiID, Mode, ::mcu::spi::mode::Master, BaudRate, DataBits, \
		::mcu::spi::bus::Bus, \
		::mcu::gpio::pin_id::SclkPinID, ::mcu::gpio::pin_id::MosiPinID, \
		::mcu::gpio::pin_id::MisoPinID, ::mcu::gpio::pin_id::NssPinID, \
		::mcu::spi::ALT_FUNC_ID_AUTO > \
	{ \
	public: \
		static const uint32_t _alt_cfg_id = ALT_CFG_ID; \
		static const bool     _verified   = true; \
	}; \
	/* Automatic selection of AF configuration */ \
	template < ::mcu::spi::mode::mode Mode, uint32_t BaudRate, ::mcu::spi::data_bits::data_bits DataBits > \
	class spi_set< ::mcu::spi::spi_id::SpiID, Mode, ::mcu::spi::mode::Master, BaudRate, DataBits, \
		::mcu::spi::bus::Bus, \
		::mcu::gpio::pin_id::invalid, ::mcu::gpio::pin_id::invalid, \
		::mcu::gpio::pin_id::invalid, ::mcu::gpio::pin_id::invalid, \
		ALT_CFG_ID > \
	{ \
	public: \
		static const uint32_t _alt_cfg_id = ALT_CFG_ID; \
		static const bool     _verified   = true; \
	}; \
	

#define DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, SclkPinID, MosiPinID, MisoPinID, NssPinID, DmaID, DmaChTX, DmaChRX)		\
	/* MASTER: software nSS */ \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _master, tx_only,			SclkPinID, MosiPinID,	invalid,	invalid,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _master, rx_only,			SclkPinID, invalid,		MisoPinID,	invalid,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _master, tx_rx_2_line,		SclkPinID, MosiPinID,	MisoPinID,	invalid,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _master, tx_rx_1_line,		SclkPinID, MosiPinID,	invalid,	invalid,	DmaID, DmaChTX, DmaChRX) \
	/* MASTER: hardware nSS */ \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _master, tx_only_nss,		SclkPinID, MosiPinID,	invalid,	NssPinID,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _master, rx_only_nss,		SclkPinID, invalid,		MisoPinID,	NssPinID,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _master, tx_rx_2_line_nss,	SclkPinID, MosiPinID,	MisoPinID,	NssPinID,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _master, tx_rx_1_line_nss,	SclkPinID, MosiPinID,	invalid,	NssPinID,	DmaID, DmaChTX, DmaChRX) \
	/* SLAVE: software nSS */ \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _slave, tx_only,			SclkPinID, invalid,		MisoPinID,	invalid,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _slave, rx_only,			SclkPinID, MosiPinID,	invalid,	invalid,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _slave, tx_rx_2_line,		SclkPinID, MosiPinID,	MisoPinID,	invalid,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _slave, tx_rx_1_line,		SclkPinID, invalid,		MisoPinID,	invalid,	DmaID, DmaChTX, DmaChRX) \
	/* SLAVE: hardware nSS */ \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _slave, tx_only_nss,		SclkPinID, invalid,		MisoPinID,	NssPinID,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _slave, rx_only_nss,		SclkPinID, MosiPinID,	invalid,	NssPinID,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _slave, tx_rx_2_line_nss,	SclkPinID, MosiPinID,	MisoPinID,	NssPinID,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CHKMODE(SpiID, ALT_CFG_ID, _slave, tx_rx_1_line_nss,	SclkPinID, invalid,		MisoPinID,	NssPinID,	DmaID, DmaChTX, DmaChRX) \

DEFINE_SPIS(CHKMODE)

	//////////////////////////////////////////////////////////////////////////
	template <
			::mcu::spi::spi_id::spi_id				SpiID		,
			uint32_t								AfID		,
			uint32_t /*::mcu::spi::mode::mode*/		Master		,
			::mcu::spi::bus::bus					Bus			
		>
	class spi_cfg_def
	{
	public:
		static const spi::spi_id::spi_id			_SpiID			= SpiID								;
		static const uint32_t						_alt_cfg_id		= AfID								;
		static const ::mcu::spi::bus::bus			_Bus			= Bus								;
		static const ::mcu::gpio::pin_id::pin_id	_SclkPinID		= ::mcu::gpio::pin_id::invalid		;
		static const ::mcu::gpio::pin_id::pin_id	_MosiPinID		= ::mcu::gpio::pin_id::invalid		;
		static const ::mcu::gpio::pin_id::pin_id	_MisoPinID		= ::mcu::gpio::pin_id::invalid		;
		static const ::mcu::gpio::pin_id::pin_id	_NssPinID		= ::mcu::gpio::pin_id::invalid		;
		static const dma::dma_id::dma_id			_dma_id      	= dma::dma_id::invalid				;
		static const dma::channel::channel			_dma_ch_tx  	= dma::channel::invalid				;
		static const dma::channel::channel			_dma_ch_rx  	= dma::channel::invalid				;
	};


#define _DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, Master, Bus, SclkPinID, MosiPinID, MisoPinID, NssPinID, DmaID, DmaChTX, DmaChRX)		\
	template < > class spi_cfg_def< spi::spi_id::SpiID, ALT_CFG_ID, ::mcu::spi::mode::Master, ::mcu::spi::bus::Bus > \
	{ \
	public: \
		static const spi::spi_id::spi_id			_SpiID			= spi::spi_id::SpiID				; \
		static const uint32_t						_alt_cfg_id		= ALT_CFG_ID						; \
		static const ::mcu::spi::bus::bus			_Bus			= ::mcu::spi::bus::Bus				; \
		static const ::mcu::gpio::pin_id::pin_id	_SclkPinID		= ::mcu::gpio::pin_id::SclkPinID	; \
		static const ::mcu::gpio::pin_id::pin_id	_MosiPinID		= ::mcu::gpio::pin_id::MosiPinID	; \
		static const ::mcu::gpio::pin_id::pin_id	_MisoPinID		= ::mcu::gpio::pin_id::MisoPinID	; \
		static const ::mcu::gpio::pin_id::pin_id	_NssPinID		= ::mcu::gpio::pin_id::NssPinID		; \
		static const dma::dma_id::dma_id			_dma_id        	= dma::dma_id::DmaID				; \
		static const dma::channel::channel			_dma_ch_tx  	= dma::channel::DmaChTX				; \
		static const dma::channel::channel			_dma_ch_rx  	= dma::channel::DmaChRX				; \
	}; \
	
#define DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, SclkPinID, MosiPinID, MisoPinID, NssPinID, DmaID, DmaChTX, DmaChRX)		\
	/* MASTER: software nSS */ \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _master, tx_only,			SclkPinID, MosiPinID,	invalid,	invalid,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _master, rx_only,			SclkPinID, invalid,		MisoPinID,	invalid,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _master, tx_rx_2_line,		SclkPinID, MosiPinID,	MisoPinID,	invalid,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _master, tx_rx_1_line,		SclkPinID, MosiPinID,	invalid,	invalid,	DmaID, DmaChTX, DmaChRX) \
	/* MASTER: hardware nSS */ \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _master, tx_only_nss,		SclkPinID, MosiPinID,	invalid,	NssPinID,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _master, rx_only_nss,		SclkPinID, invalid,		MisoPinID,	NssPinID,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _master, tx_rx_2_line_nss,	SclkPinID, MosiPinID,	MisoPinID,	NssPinID,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _master, tx_rx_1_line_nss,	SclkPinID, MosiPinID,	invalid,	NssPinID,	DmaID, DmaChTX, DmaChRX) \
	/* SLAVE: software nSS */ \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _slave, tx_only,			SclkPinID, invalid,		MisoPinID,	invalid,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _slave, rx_only,			SclkPinID, MosiPinID,	invalid,	invalid,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _slave, tx_rx_2_line,		SclkPinID, MosiPinID,	MisoPinID,	invalid,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _slave, tx_rx_1_line,		SclkPinID, invalid,		MisoPinID,	invalid,	DmaID, DmaChTX, DmaChRX) \
	/* SLAVE: hardware nSS */ \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _slave, tx_only_nss,		SclkPinID, invalid,		MisoPinID,	NssPinID,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _slave, rx_only_nss,		SclkPinID, MosiPinID,	invalid,	NssPinID,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _slave, tx_rx_2_line_nss,	SclkPinID, MosiPinID,	MisoPinID,	NssPinID,	DmaID, DmaChTX, DmaChRX) \
	_DEF_SPI_CFG_DEF(SpiID, ALT_CFG_ID, _slave, tx_rx_1_line_nss,	SclkPinID, invalid,		MisoPinID,	NssPinID,	DmaID, DmaChTX, DmaChRX) \

DEFINE_SPIS(CFG_DEF)

	

	//////////////////////////////////////////////////////////////////////////

	template <spi_id::spi_id SpiId, class sysclock, uint32_t PCLK_Hz>
	class spi_cfg2
	{
	public:
		static const uint32_t	_PCLK_Hz = PCLK_Hz;
	};
	//////////////////////////////////////////////////////////////////////////
	template <spi_id::spi_id SpiId, class sysclock> class spi_cfg   :
		public spi_cfg2<SpiId, sysclock, sysclock::_cfg_::_PCLK1_Hz> {};
	template <class sysclock> class spi_cfg<spi_id::spi_1, sysclock> :
		public spi_cfg2<spi_id::spi_1, sysclock, sysclock::_cfg_::_PCLK2_Hz> {};

/************************************************************************/
/*                                                                      */
/************************************************************************/

} // namespace spi

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace registers {
	template<spi_id::spi_id SpiID, uint32_t AfID,
		uint32_t AFIO_MAPR_REMAP_Pos, uint32_t AFIO_MAPR_REMAP_Msk,
		uint32_t SPI_REG, uint32_t RCC_APBENR_BB
		>
	class REG_SPI_REMAP_BASE
	{
	public:
		static const uint32_t _AFIO_MAPR_REMAP_POS		= AFIO_MAPR_REMAP_Pos;
		static const uint32_t _AFIO_MAPR_REMAP_MASK		= AFIO_MAPR_REMAP_Msk;
		static const uint32_t _AFIO_MAPR_REMAP			= AfID << _AFIO_MAPR_REMAP_POS;
		
		static const uint32_t _AFIO_MAPR2_REMAP_POS		= 0;
		static const uint32_t _AFIO_MAPR2_REMAP_MASK	= 0;
		static const uint32_t _AFIO_MAPR2_REMAP			= 0;

		static const uint32_t _SPI_REG					= SPI_REG;
		static const uint32_t _RCC_APBENR_BB			= RCC_APBENR_BB;

		static const uint32_t _REMAP_MAX = (_AFIO_MAPR_REMAP_MASK >> _AFIO_MAPR_REMAP_POS) + 1;
		
		STATIC_ASSERT(AfID < _REMAP_MAX, "Internall error: The required configureation cannot be applied in the registers");
	};

	template<spi_id::spi_id SpiID, uint32_t AfID> class REG_SPI_REMAP;
	
	template<uint32_t AfID> class REG_SPI_REMAP<spi_id::spi_1, AfID>
		: public REG_SPI_REMAP_BASE<
			spi_id::spi_1, AfID,
			AFIO_MAPR_SPI1_REMAP_Pos, AFIO_MAPR_SPI1_REMAP_Msk,
			(uint32_t)SPI1, FROM_ADDRESS_BIT_POS_TO_BB(&RCC->APB2ENR, RCC_APB2ENR_SPI1EN_Pos)
		> {};

	template<uint32_t AfID> class REG_SPI_REMAP<spi_id::spi_2, AfID>
		: public REG_SPI_REMAP_BASE<
			spi_id::spi_2, AfID,
			0, 0,
			(uint32_t)SPI2, FROM_ADDRESS_BIT_POS_TO_BB(&RCC->APB1ENR, RCC_APB1ENR_SPI2EN_Pos)
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
using namespace ::mcu::spi;

namespace spi {

template < class CFG >
class spi_port
{
private:
	typedef stm32::spi::spi_set< CFG::_SpiID, CFG::_Mode, CFG::_Mode & ::mcu::spi::mode::_master,
		CFG::_BaudRate, CFG::_DataBits, CFG::_Bus,
		CFG::_SclkPinID, CFG::_MosiPinID, CFG::_MisoPinID, CFG::_NssPinID, CFG::_AltFuncId > _set_;
	
	STATIC_ASSERT(_set_::_verified != false, "The required configuration was not found! Please check mode<->pins::AF configuration.");
	
	typedef stm32::registers::REG_SPI_REMAP<CFG::_SpiID, _set_::_alt_cfg_id> REG_REMAP;
	STATIC_ASSERT(_set_::_alt_cfg_id < REG_REMAP::_REMAP_MAX, "Internall error: The required configureation cannot be applied in the registers");
	
public:
	class _cfg_
	{
	public:
		static const spi_id::spi_id				_SpiID			= CFG::_SpiID			;
		static const mode::mode					_Mode			= CFG::_Mode		    ;
		static const uint32_t					_BaudRate		= CFG::_BaudRate		;
		static const data_bits::data_bits		_DataBits		= CFG::_DataBits		;
		static const bus::bus					_Bus			= CFG::_Bus				;
		static const uint32_t					_AltFuncId		= CFG::_AltFuncId		;
		
		static const uint32_t					_alt_cfg_id		= _set_::_alt_cfg_id;

		static const ::mcu::gpio::pin_id::pin_id
			_SclkPinID	= stm32::spi::spi_cfg_def<_SpiID, _alt_cfg_id, _Mode & ::mcu::spi::mode::_master, _Bus>::_SclkPinID	,
			_MosiPinID	= stm32::spi::spi_cfg_def<_SpiID, _alt_cfg_id, _Mode & ::mcu::spi::mode::_master, _Bus>::_MosiPinID	,
			_MisoPinID	= stm32::spi::spi_cfg_def<_SpiID, _alt_cfg_id, _Mode & ::mcu::spi::mode::_master, _Bus>::_MisoPinID	,
			_NssPinID	= stm32::spi::spi_cfg_def<_SpiID, _alt_cfg_id, _Mode & ::mcu::spi::mode::_master, _Bus>::_NssPinID	;
		
		STATIC_ASSERT((CFG::_SclkPinID	== mcu::gpio::pin_id::invalid ? _SclkPinID	: CFG::_SclkPinID	) == _SclkPinID	, "Invalid configuration for SclkPinID	");
		STATIC_ASSERT((CFG::_MosiPinID	== mcu::gpio::pin_id::invalid ? _MosiPinID	: CFG::_MosiPinID	) == _MosiPinID	, "Invalid configuration for MosiPinID	");
		STATIC_ASSERT((CFG::_MisoPinID	== mcu::gpio::pin_id::invalid ? _MisoPinID	: CFG::_MisoPinID	) == _MisoPinID	, "Invalid configuration for MisoPinID	");
		STATIC_ASSERT((CFG::_NssPinID	== mcu::gpio::pin_id::invalid ? _NssPinID	: CFG::_NssPinID	) == _NssPinID	, "Invalid configuration for NssPinID	");

		typedef      alt_output<_SclkPinID	, speed::high	>	_SclkPin					;
		typedef      alt_output<_MosiPinID	, speed::high	>	_MosiPin					;
		typedef      alt_input <_MisoPinID					>	_MisoPin					;
		typedef      alt_output<_NssPinID	, speed::high	>	_NssPin						;
		
		static const uint32_t _AFIO_MAPR_REMAP_MASK		= REG_REMAP::_AFIO_MAPR_REMAP_MASK	;
		static const uint32_t _AFIO_MAPR_REMAP			= REG_REMAP::_AFIO_MAPR_REMAP		;
		static const uint32_t _AFIO_MAPR2_REMAP_MASK	= REG_REMAP::_AFIO_MAPR2_REMAP_MASK	;
		static const uint32_t _AFIO_MAPR2_REMAP			= REG_REMAP::_AFIO_MAPR2_REMAP		;
		
		static const uint32_t _SPI_REG					= REG_REMAP::_SPI_REG				;

		static const uint32_t _SPI_CR1_SPE_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((SPI_TypeDef*)_SPI_REG)->CR1, SPI_CR1_SPE_Pos);
		
		static const uint32_t _SPI_SR_BSY_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((SPI_TypeDef*)_SPI_REG)->SR, SPI_SR_BSY_Pos);
		static const uint32_t _SPI_SR_OVR_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((SPI_TypeDef*)_SPI_REG)->SR, SPI_SR_OVR_Pos);
		static const uint32_t _SPI_SR_CRCERR_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((SPI_TypeDef*)_SPI_REG)->SR, SPI_SR_CRCERR_Pos);
		static const uint32_t _SPI_SR_TXE_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((SPI_TypeDef*)_SPI_REG)->SR, SPI_SR_TXE_Pos);
		static const uint32_t _SPI_SR_RXNE_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((SPI_TypeDef*)_SPI_REG)->SR, SPI_SR_RXNE_Pos);

		//static const uint32_t _SPI_DR					= (uint32_t)(&((SPI_TypeDef*)(uint32_t)_SPI_REG)->DR);
		static const uint32_t _SPI_DR					= _SPI_REG + (uint32_t)(&((SPI_TypeDef*)0)->DR);

		static const uint32_t _SPI_CR1				=
			(	(((_Bus & bus::_$_line) == bus::_1_line) && ((_Bus & bus::_tx) == bus::_tx) && ((_Bus & bus::_rx) == bus::_rx)) ? (SPI_CR1_BIDIMODE) :
				(0) ) |
			(	(((_Bus & bus::_$_line) == bus::_1_line) && ((_Bus & bus::_tx) != bus::_tx) && ((_Bus & bus::_rx) == bus::_rx)) ? (SPI_CR1_BIDIOE) :		// <-- check SLAVE mode
				(0) ) |
			//(SPI_CR1_CRCEN) |
			(	((_DataBits & data_bits::_data_mask) == data_bits::_16) ? (SPI_CR1_DFF) :
				(0) ) |
			(	(((_Bus & bus::_tx) != bus::_tx) && ((_Bus & bus::_rx) == bus::_rx)) ? (SPI_CR1_RXONLY) :
				(0) ) |
			(	((_Bus & bus::_nss) != bus::_nss) ? (SPI_CR1_SSM) :
				(0) ) |
			(	((_Mode & mode::_master_mask) == mode::_master) ? (SPI_CR1_MSTR | SPI_CR1_SSI) :
				(0) ) |
			(	((_DataBits & data_bits::_$sb_mask) == data_bits::_lsb) ? (SPI_CR1_LSBFIRST) :
				(0) ) |
			(SPI_CR1_SPE) |
			//(	((_Mode & mode::_master_mask) == mode::_master) ? (SPI_CR1_MSTR | SPI_CR1_SSI) :
			//	(0) ) |
			(	((_Mode & mode::_cpol_$_mask) == mode::_cpol_1) ? (SPI_CR1_CPOL) :
				(0) ) |
			(	((_Mode & mode::_cpha_$_mask) == mode::_cpha_1) ? (SPI_CR1_CPHA) :
				(0) ) |
			(0);

		static const uint32_t _SPI_CR2				=
			//(SPI_CR2_TXEIE) |
			//(SPI_CR2_RXNEIE) |
			//(SPI_CR2_ERRIE) |
			(	((_Mode & mode::_master_mask) == mode::_master && (_Bus & bus::_nss) == bus::_nss) ? (SPI_CR2_SSOE) :
				(0) ) |
			//(SPI_CR2_TXDMAEN) |
			//(SPI_CR2_RXDMAEN) |
			(0);

		static const uint32_t _SPI_CRCPR				=
			(0);

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
			typedef stm32::spi::spi_cfg<_cfg_::_SpiID, sysclock> spi_cfg;
			static const uint32_t _PCLK_Hz = spi_cfg::_PCLK_Hz;

			static const uint32_t _br =
				(_PCLK_Hz /   2) <= _cfg_::_BaudRate ?	0 :
				(_PCLK_Hz /   4) <= _cfg_::_BaudRate ?	1 :
				(_PCLK_Hz /   8) <= _cfg_::_BaudRate ?	2 :
				(_PCLK_Hz /  16) <= _cfg_::_BaudRate ?	3 :
				(_PCLK_Hz /  32) <= _cfg_::_BaudRate ?	4 :
				(_PCLK_Hz /  64) <= _cfg_::_BaudRate ?	5 :
				(_PCLK_Hz / 128) <= _cfg_::_BaudRate ?	6 :
														7 ;
			// calculate actual baud rate
			static const uint32_t _BaudRate_Real = _PCLK_Hz / (1UL << (_br + 1));
			
			// calculate baud rate accuracy
			//static const int32_t  _percent = ((int32_t)_BaudRate_Real - (int32_t)_cfg_::_BaudRate) * 100 / (int32_t)_cfg_::_BaudRate;

			// assemble value for BRR register
			static const uint32_t _SPI_CR1 = (_cfg_::_SPI_CR1 & ~SPI_CR1_BR_Msk) | ((_br << SPI_CR1_BR_Pos) & SPI_CR1_BR_Msk);
		};
		//STATIC_ASSERT(_baud_calc_::_percent >= _baud_calc_::_percent_min && _baud_calc_::_percent <= _baud_calc_::_percent_max, "The required baud rate cannot be reached on the selected system clock!");
		
	public:
		static void starting()
		{
			// Disable the peripheral
			RESET_BB_REG(_cfg_::_SPI_CR1_SPE_BB);			// <-- TODO: Implement waiting for IO completion before disabling
		}
		static void finished()
		{
			// Enable peripheral clock

			// CR1 Configuration
			((SPI_TypeDef*)_cfg_::_SPI_REG)->CR1 = _baud_calc_::_SPI_CR1;

			// CR2 Configuration
			((SPI_TypeDef*)_cfg_::_SPI_REG)->CR2 = _cfg_::_SPI_CR2;

			// CR1 Configuration
			((SPI_TypeDef*)_cfg_::_SPI_REG)->CRCPR = _cfg_::_SPI_CRCPR;
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
			typename _cfg_::_SclkPin, typename _cfg_::_MosiPin,
			typename _cfg_::_MisoPin, typename _cfg_::_NssPin
			>::init();

		// Remap AFIO, if required
		if(_cfg_::_AFIO_MAPR_REMAP_MASK)
			MODIFY_REG(AFIO->MAPR, _cfg_::_AFIO_MAPR_REMAP_MASK, _cfg_::_AFIO_MAPR_REMAP);
		if(_cfg_::_AFIO_MAPR2_REMAP_MASK)
			MODIFY_REG(AFIO->MAPR2, _cfg_::_AFIO_MAPR2_REMAP_MASK, _cfg_::_AFIO_MAPR2_REMAP);

		//// Peripheral DMA init

		//hdma_spi1_tx.Instance = DMA1_Channel3;
		//hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		//hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		//hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
		//hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		//hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		//hdma_spi1_tx.Init.Mode = DMA_NORMAL;
		//hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
		//if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
		//{
		//Error_Handler();
		//}

		//__HAL_LINKDMA(hspi,hdmatx,hdma_spi1_tx);

		// Disable the peripheral
		RESET_BB_REG(_cfg_::_SPI_CR1_SPE_BB);		// on_sysclock_changing<sysclock>::starting();

		// Set the SPI Communication parameters
		on_sysclock_changing<sysclock>::finished();
	}
	
	static void update()
	{
	}

	//////////////////////////////////////////////////////////////////////////
	static bool has_data()
	{
		return IS_BB_REG_SET(_cfg_::_SPI_SR_RXNE_BB);
	}

	static bool is_tx_empty()
	{
		return IS_BB_REG_SET(_cfg_::_SPI_SR_TXE_BB);
	}

	//////////////////////////////////////////////////////////////////////////
	static void putc(char c)
	{
		while(IS_BB_REG_RESET(_cfg_::_SPI_SR_TXE_BB)) {}
		WRITE_BB_REG(_cfg_::_SPI_DR, c);
	}

	static char getc()
	{
		while(IS_BB_REG_RESET(_cfg_::_SPI_SR_RXNE_BB)) {}
		return READ_BB_REG(_cfg_::_SPI_DR);
	}
	
	//////////////////////////////////////////////////////////////////////////
	static uint32_t read(void* buf, uint32_t size, uint32_t timeout = TIMEOUT_INFINITE)
	{
		uint32_t len = 0;
		uint8_t* p = (uint8_t*)buf;
		for( ;size; len++, size--)
		{
			while(IS_BB_REG_RESET(_cfg_::_SPI_SR_RXNE_BB)) {}
			*p++ = READ_BB_REG(_cfg_::_SPI_DR);
		}
		return len;
	}

	static uint32_t write(const void* buf, uint32_t size, uint32_t timeout = TIMEOUT_INFINITE)
	{
		uint32_t len = 0;
		uint8_t* p = (uint8_t*)buf;
		for( ;size; len++, size--)
		{
			while(IS_BB_REG_RESET(_cfg_::_SPI_SR_TXE_BB)) {}
			WRITE_BB_REG(_cfg_::_SPI_DR, *p++);
		}
		//while(IS_BB_REG_RESET(_cfg_::_SPI_SR_TXE_BB)) {}
		while(IS_BB_REG_SET(_cfg_::_SPI_SR_BSY_BB)) {}
		return len;
	}
};

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace spi
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__stm32f1xx_spi_hpp__*/
