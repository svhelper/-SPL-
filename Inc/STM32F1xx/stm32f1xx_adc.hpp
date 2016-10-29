#ifndef __stm32f1xx_adc_hpp__
#define __stm32f1xx_adc_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

#include <gpio.hpp>
#include <dma.hpp>
#include <adc.hpp>

//////////////////////////////////////////////////////////////////////////
#include "CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"
#include "stm32f1xx/stm32f1xx_registers.hpp"
#include "stm32f1xx/stm32f1xx_gpio.hpp"


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {
namespace stm32 {
	
namespace adc {
	using namespace ::mcu::adc;
	using namespace ::mcu::gpio;

	//////////////////////////////////////////////////////////////////////////
	template <
			::mcu::adc::adc_id::adc_id				AdcID		,
			uint32_t								DataBits	,
			::mcu::dma::dma_id::dma_id				DmaID		= ::mcu::dma::dma_id::invalid	,
			::mcu::dma::channel::channel			DmaCh		= ::mcu::dma::channel::invalid	
		>
	struct adc_set
	{
		static const ::mcu::adc::adc_id::adc_id			_AdcID		= AdcID		;
		static const uint32_t							_DataBits	= ::mcu::adc::data_bits::_max	;
		static const ::mcu::dma::dma_id::dma_id			_DmaID		= DmaID		;
		static const ::mcu::dma::channel::channel		_DmaCh		= DmaCh		;
		static const uint32_t							_alt_cfg_id	= 0			;
		
		static const bool     _verified   = false;
	};
	
#define DEF_ADC_CFG_CFG(ADCID, BITS, DMAID, DMACH) \
	/* verification */ \
	template < \
			::mcu::dma::dma_id::dma_id				DmaID		, \
			::mcu::dma::channel::channel			DmaCh		  \
		> \
	struct adc_set< ::mcu::adc::adc_id::ADCID, BITS, DmaID, DmaCh > \
	{ \
		static const ::mcu::adc::adc_id::adc_id			_AdcID		= ::mcu::adc::adc_id::ADCID		; \
		static const uint32_t							_DataBits	= BITS							; \
		static const ::mcu::dma::dma_id::dma_id			_DmaID		= ::mcu::dma::dma_id::DMAID		; \
		static const ::mcu::dma::channel::channel		_DmaCh		= ::mcu::dma::channel::DMACH	; \
		static const uint32_t							_alt_cfg_id	= 0								; \
	 \
		static const bool								_verified   = true; \
	}; \
	/* default configuration */ \
	template < \
			::mcu::dma::dma_id::dma_id				DmaID		, \
			::mcu::dma::channel::channel			DmaCh		  \
		> \
	struct adc_set< ::mcu::adc::adc_id::ADCID, ::mcu::adc::data_bits::_max, DmaID, DmaCh > \
	{ \
		static const ::mcu::adc::adc_id::adc_id			_AdcID		= ::mcu::adc::adc_id::ADCID		; \
		static const uint32_t							_DataBits	= BITS							; \
		static const ::mcu::dma::dma_id::dma_id			_DmaID		= ::mcu::dma::dma_id::DMAID		; \
		static const ::mcu::dma::channel::channel		_DmaCh		= ::mcu::dma::channel::DMACH	; \
		static const uint32_t							_alt_cfg_id	= 0								; \
	 \
		static const bool								_verified   = true; \
	}; \

	//------------------------------------------------------------------------
	template <
			::mcu::adc::adc_id::adc_id				AdcID		,
			::mcu::adc::channel_id::channel_id		ChannelID	,
			::mcu::gpio::pin_id::pin_id				PinID		
		>
	struct adc_ch_set
	{
		static const ::mcu::adc::adc_id::adc_id				_AdcID		= ::mcu::adc::adc_id::invalid		;
		static const ::mcu::adc::channel_id::channel_id		_ChannelID	= ::mcu::adc::channel_id::invalid	;
		static const ::mcu::gpio::pin_id::pin_id			_PinID		= ::mcu::gpio::pin_id::invalid		;

		static const bool     _verified   = false;
	};

#define DEF_ADC_CH_CFG(ADCID, CHANNELID, PINID) \
	/* verification */ \
	template < > \
	struct adc_ch_set < \
			::mcu::adc::adc_id::ADCID			, \
			::mcu::adc::channel_id::CHANNELID	, \
			::mcu::gpio::pin_id::PINID			  \
		> \
	{ \
		static const ::mcu::adc::adc_id::adc_id				_AdcID		= ::mcu::adc::adc_id::ADCID			; \
		static const ::mcu::adc::channel_id::channel_id		_ChannelID	= ::mcu::adc::channel_id::CHANNELID	; \
		static const ::mcu::gpio::pin_id::pin_id			_PinID		= ::mcu::gpio::pin_id::PINID		; \
	 \
		static const bool									_verified   = true; \
	}; \
	/* GPIO PinID to ADC ChannelID */ \
	template < > \
	struct adc_ch_set < \
			::mcu::adc::adc_id::ADCID			, \
			::mcu::adc::channel_id::invalid		, \
			::mcu::gpio::pin_id::PINID			  \
		> \
	{ \
		static const ::mcu::adc::adc_id::adc_id				_AdcID		= ::mcu::adc::adc_id::ADCID			; \
		static const ::mcu::adc::channel_id::channel_id		_ChannelID	= ::mcu::adc::channel_id::CHANNELID	; \
		static const ::mcu::gpio::pin_id::pin_id			_PinID		= ::mcu::gpio::pin_id::PINID		; \
	 \
		static const bool									_verified   = true; \
	}; \
	/* ADC ChannelID to GPIO PinID */ \
	template < > \
	struct adc_ch_set < \
			::mcu::adc::adc_id::ADCID			, \
			::mcu::adc::channel_id::CHANNELID	, \
			::mcu::gpio::pin_id::invalid		  \
		> \
	{ \
		static const ::mcu::adc::adc_id::adc_id				_AdcID		= ::mcu::adc::adc_id::ADCID			; \
		static const ::mcu::adc::channel_id::channel_id		_ChannelID	= ::mcu::adc::channel_id::CHANNELID	; \
		static const ::mcu::gpio::pin_id::pin_id			_PinID		= ::mcu::gpio::pin_id::PINID		; \
	 \
		static const bool									_verified   = true; \
	}; \
	
DEFINE_ADCS(CFG)

	
	//////////////////////////////////////////////////////////////////////////
	template<class TYPE, ::mcu::adc::adc_id::adc_id AdcID, uint32_t DataBits>
	struct check_type2
	{
		typedef adc_set<AdcID, DataBits> _set_;
		static const bool _valid =
			_set_::_verified == true &&
			_set_::_DataBits <= (sizeof(TYPE) * 8);
	};
	
	template<class TYPE, ::mcu::adc::adc_id::adc_id AdcID, uint32_t DataBits>
	struct check_type		{ static const bool _valid = false; };

	template< ::mcu::adc::adc_id::adc_id AdcID, uint32_t DataBits >	struct check_type< uint8_t, AdcID, DataBits> : public check_type2< uint8_t, AdcID, DataBits>	{ };
	template< ::mcu::adc::adc_id::adc_id AdcID, uint32_t DataBits >	struct check_type<uint16_t, AdcID, DataBits> : public check_type2<uint16_t, AdcID, DataBits>	{ };
	template< ::mcu::adc::adc_id::adc_id AdcID, uint32_t DataBits > struct check_type<uint32_t, AdcID, DataBits> : public check_type2<uint32_t, AdcID, DataBits>	{ };
	template< ::mcu::adc::adc_id::adc_id AdcID, uint32_t DataBits > struct check_type<  int8_t, AdcID, DataBits> : public check_type2<  int8_t, AdcID, DataBits>	{ };
	template< ::mcu::adc::adc_id::adc_id AdcID, uint32_t DataBits > struct check_type< int16_t, AdcID, DataBits> : public check_type2< int16_t, AdcID, DataBits>	{ };
	template< ::mcu::adc::adc_id::adc_id AdcID, uint32_t DataBits > struct check_type< int32_t, AdcID, DataBits> : public check_type2< int32_t, AdcID, DataBits>	{ };
	
	//////////////////////////////////////////////////////////////////////////

	template <adc_id::adc_id AdcId, class sysclock, uint32_t PCLK_Hz>
	class adc_cfg2
	{
	public:
		static const uint32_t	_PCLK_Hz = PCLK_Hz;
	};
	//////////////////////////////////////////////////////////////////////////
	template <adc_id::adc_id AdcId, class sysclock> class adc_cfg   :
		public adc_cfg2<AdcId, sysclock, sysclock::_cfg_::_ADC_Hz> {};
	//template <adc_id::adc_id AdcId, class sysclock> class adc_cfg<AdcId, sysclock> :
	//	public adc_cfg2<AdcId, sysclock, sysclock::_cfg_::_ADC_Hz> {};
	
	//////////////////////////////////////////////////////////////////////////

	template <adc_id::adc_id AdcId, IRQn_Type IRQn>
	class adc_irqn2
	{
	public:
		static const IRQn_Type _IRQn = IRQn;
	};
	//////////////////////////////////////////////////////////////////////////
	template <adc_id::adc_id AdcId> class adc_irqn   :
		public adc_irqn2<AdcId, ADC1_2_IRQn> {};
	//template <> class adc_irqn<adc_1> :
	//	public adc_irqn2<adc_1, ADC1_2_IRQn> {};

/************************************************************************/
/*                                                                      */
/************************************************************************/
	static const uint32_t adc_channel_tempsensor	= 16;			// ADC internal channel (no connection on device pin)
	static const uint32_t adc_channel_vrefint		= 17;			// ADC internal channel (no connection on device pin)

	static const ::mcu::adc::sampling_delay_min::sampling_delay_min		adc_sampling_delay_min_default = ::mcu::adc::sampling_delay_min::_normal;

	static const uint32_t ADC_PRECALIBRATION_DELAY_CLOCKS	= 2;

/************************************************************************/
/*                                                                      */
/************************************************************************/

} // namespace adc

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace registers {
	template<adc_id::adc_id AdcID, uint32_t AfID,
		uint32_t AFIO_MAPR_REMAP_Pos, uint32_t AFIO_MAPR_REMAP_Msk,
		uint32_t ADC_REG, uint32_t RCC_APBENR_BB
		>
	class REG_ADC_REMAP_BASE
	{
	public:
		static const uint32_t _AFIO_MAPR_REMAP_POS		= AFIO_MAPR_REMAP_Pos;
		static const uint32_t _AFIO_MAPR_REMAP_MASK		= AFIO_MAPR_REMAP_Msk;
		static const uint32_t _AFIO_MAPR_REMAP			= AfID << _AFIO_MAPR_REMAP_POS;
		
		static const uint32_t _AFIO_MAPR2_REMAP_POS		= 0;
		static const uint32_t _AFIO_MAPR2_REMAP_MASK	= 0;
		static const uint32_t _AFIO_MAPR2_REMAP			= 0;

		static const uint32_t _ADC_REG					= ADC_REG;
		static const uint32_t _RCC_APBENR_BB			= RCC_APBENR_BB;

		static const uint32_t _REMAP_MAX = (_AFIO_MAPR_REMAP_MASK >> _AFIO_MAPR_REMAP_POS) + 1;
		
		STATIC_ASSERT(AfID < _REMAP_MAX, "Internall error: The required configureation cannot be applied in the registers");
	};

	template<adc_id::adc_id AdcID, uint32_t AfID> class REG_ADC_REMAP;
	
	template<uint32_t AfID> class REG_ADC_REMAP<adc_id::adc_1, AfID>
		: public REG_ADC_REMAP_BASE<
			adc_id::adc_1, AfID,
			0, 0,
			(uint32_t)ADC1, FROM_ADDRESS_BIT_POS_TO_BB(&RCC->APB2ENR, RCC_APB2ENR_ADC1EN_Pos)
		> {};

	template<uint32_t AfID> class REG_ADC_REMAP<adc_id::adc_2, AfID>
		: public REG_ADC_REMAP_BASE<
			adc_id::adc_2, AfID,
			0, 0,
			(uint32_t)ADC2, FROM_ADDRESS_BIT_POS_TO_BB(&RCC->APB2ENR, RCC_APB2ENR_ADC2EN_Pos)
		> {};

//	template<uint32_t AfID> class REG_ADC_REMAP<adc_id::adc_3, AfID>
//		: public REG_ADC_REMAP_BASE<
//			adc_id::adc_3, AfID,
//			0, 0,
//			(uint32_t)ADC3, FROM_ADDRESS_BIT_POS_TO_BB(&RCC->APB2ENR, RCC_APB2ENR_ADC3EN_Pos)
//		> {};

	//////////////////////////////////////////////////////////////////////////
	template <
			class _cfg_,
			::mcu::adc::adc_id::adc_id adc_id,
			::mcu::adc::sampling_delay_min::sampling_delay_min sampling_delay_min
		>
	struct adc_init_channel2
	{
		//static const uint32_t _smp =
		//	(sampling_delay_min <   2) ?	(0) :	//   1.5 cycles
		//	(sampling_delay_min <   8) ?	(1) :	//   7.5 cycles
		//	(sampling_delay_min <  14) ?	(2) :	//  13.5 cycles
		//	(sampling_delay_min <  29) ?	(3) :	//  28.5 cycles
		//	(sampling_delay_min <  42) ?	(4) :	//  41.5 cycles
		//	(sampling_delay_min <  56) ?	(5) :	//  55.5 cycles
		//	(sampling_delay_min <  72) ?	(6) :	//  71.5 cycles
		//	(sampling_delay_min < 240) ?	(7) :	// 239.5 cycles
		//									(7);

		static void exec(uint32_t sq)
		{
			((ADC_TypeDef*)_cfg_::_ADC_REG)->SQR3 = (sq << ADC_SQR3_SQ1_Pos) & ADC_SQR3_SQ1_Msk;

			STATIC_ASSERT(sampling_delay_min == ::mcu::adc::sampling_delay_min::_default, "The on fly changing of sampling delay is not implemented!");
// TODO: use sq for reconfiguration of sampling delay
//			if(sampling_delay_min != ::mcu::adc::sampling_delay_min::_default)
//				((ADC_TypeDef*)_cfg_::_ADC_REG)->SMPR$ = (((ADC_TypeDef*)_cfg_::_ADC_REG)->SMPR$ & ~ADC_SMPR$_SMP$_Msk) | ((_smp << ADC_SMPR$_SMP$_Pos) & ADC_SMPR$_SMP$_Msk);
		}
	};
	
	template <
			class _cfg_,
			::mcu::adc::adc_id::adc_id adc_id,
			::mcu::adc::channel_id::channel_id channel_id,
			::mcu::adc::sampling_delay_min::sampling_delay_min sampling_delay_min
		>
	struct adc_init_channel
	{
		static void exec()
		{
			adc_init_channel2< _cfg_, adc_id, sampling_delay_min >::exec(channel_id);
		}
	};
	
	template < class _cfg_, ::mcu::adc::adc_id::adc_id adc_id, ::mcu::adc::sampling_delay_min::sampling_delay_min sampling_delay_min >
	struct adc_init_channel<_cfg_, adc_id, ::mcu::adc::channel_id::channel_vref, sampling_delay_min>
	{
		static const uint32_t _ADC_CR2_TSVREFE_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&((ADC_TypeDef*)_cfg_::_ADC_REG)->CR2, ADC_CR2_TSVREFE_Pos);
		
		static void exec()
		{
			SET_BB_REG(_ADC_CR2_TSVREFE_BB);
			adc_init_channel2< _cfg_, adc_id, sampling_delay_min >::exec(::mcu::stm32::adc::adc_channel_vrefint);
			//RESET_BB_REG(_ADC_CR2_TSVREFE_BB);
		}
	};
	
	template < class _cfg_, ::mcu::adc::adc_id::adc_id adc_id, ::mcu::adc::sampling_delay_min::sampling_delay_min sampling_delay_min >
	struct adc_init_channel<_cfg_, adc_id, ::mcu::adc::channel_id::channel_temp, sampling_delay_min>
	{
		static const uint32_t _ADC_CR2_TSVREFE_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&((ADC_TypeDef*)_cfg_::_ADC_REG)->CR2, ADC_CR2_TSVREFE_Pos);
		
		static void exec()
		{
			SET_BB_REG(_ADC_CR2_TSVREFE_BB);
			adc_init_channel2< _cfg_, adc_id, sampling_delay_min >::exec(::mcu::stm32::adc::adc_channel_tempsensor);
			//RESET_BB_REG(_ADC_CR2_TSVREFE_BB);
		}
	};

} // namespace registers

/************************************************************************/
/*                                                                      */
/************************************************************************/

} // namespace stm32

/************************************************************************/
/*                                                                      */
/************************************************************************/
using namespace ::mcu::gpio;
using namespace ::mcu::adc;

namespace adc {

template < class CFG >
class adc_port
{
private:
	typedef stm32::adc::adc_set<CFG::_AdcID, CFG::_DataBits> _set_;
	STATIC_ASSERT(_set_::_verified != false, "The required configuration was not found! Please check mode<->pins::AF configuration.");
	
	typedef stm32::registers::REG_ADC_REMAP<CFG::_AdcID, _set_::_alt_cfg_id> REG_REMAP;
	STATIC_ASSERT(_set_::_alt_cfg_id < REG_REMAP::_REMAP_MAX, "Internall error: The required configureation cannot be applied in the registers");

public:
	template<class TYPE> struct check_type
		: public stm32::adc::check_type<TYPE, CFG::_AdcID, _set_::_DataBits /*CFG::_DataBits & ::mcu::adc::data_bits::_$bits_mask*/>
	{};

	struct prefered_type
	{
		template<class T> struct get_type { typedef T _type; };
		
		typedef
			typename aux::if_c< (_set_::_DataBits <= (sizeof( uint8_t) * 8)), get_type< uint8_t>,
				typename aux::if_c< (_set_::_DataBits <= (sizeof(uint16_t) * 8)), get_type<uint16_t>, get_type<uint32_t> >::_result
			>::_result::_type
			_type;

		static const _type _max = (_type)((1UL << _set_::_DataBits) - 1);
	};
	
	class _cfg_
	{
	public:
		static const adc_id::adc_id				_AdcID					= CFG::_AdcID		;
		static const data_bits::data_bits		_DataBits				=
			(data_bits::data_bits)(
				(CFG::_DataBits & data_bits::_align_mask) |
				(_set_::_DataBits & data_bits::_$bits_mask)
			);
		static const flags::flags				_calibrate_at_startup	= (flags::flags)(CFG::_Flags & flags::calibrate_at_startup);
		static const bool						_skip_first_convertion	= true;
		
		static const sampling_delay_min::sampling_delay_min		_sampling_delay_min_default	=
			CFG::_DefSamplingDelayMin == sampling_delay_min::_default
			? ::mcu::stm32::adc::adc_sampling_delay_min_default
			: CFG::_DefSamplingDelayMin;
		
		static const IRQn_Type					_IRQn	= stm32::adc::adc_irqn<_AdcID>::_IRQn;
		
		static const bool						_UseDma	= CFG::_UseDma && _set_::_DmaID != ::mcu::dma::dma_id::invalid && _set_::_DmaCh != ::mcu::dma::channel::invalid;

		static const uint32_t _ADC_REG					= REG_REMAP::_ADC_REG				;
		static const uint32_t _ADC_CR2_ADON_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((ADC_TypeDef*)_ADC_REG)->CR2, ADC_CR2_ADON_Pos);
		static const uint32_t _ADC_CR2_SWSTART_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&((ADC_TypeDef*)_ADC_REG)->CR2, ADC_CR2_SWSTART_Pos);
		static const uint32_t _ADC_CR2_CONT_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((ADC_TypeDef*)_ADC_REG)->CR2, ADC_CR2_CONT_Pos);
		static const uint32_t _ADC_CR2_DMA_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((ADC_TypeDef*)_ADC_REG)->CR2, ADC_CR2_DMA_Pos);
		static const uint32_t _ADC_CR2_RSTCAL_BB		= FROM_ADDRESS_BIT_POS_TO_BB(&((ADC_TypeDef*)_ADC_REG)->CR2, ADC_CR2_RSTCAL_Pos);
		static const uint32_t _ADC_CR2_CAL_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((ADC_TypeDef*)_ADC_REG)->CR2, ADC_CR2_CAL_Pos);
//		static const uint32_t _ADC_CR1_UE_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((ADC_TypeDef*)_ADC_REG)->CR1, ADC_CR1_UE_Pos);
		static const uint32_t _ADC_CR1_EOCIE_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((ADC_TypeDef*)_ADC_REG)->CR1, ADC_CR1_EOSIE_Pos);
		static const uint32_t _ADC_SR_EOC_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((ADC_TypeDef*)_ADC_REG)->SR, ADC_SR_EOS_Pos);
//		static const uint32_t _ADC_SR_RXNE_BB			= FROM_ADDRESS_BIT_POS_TO_BB(&((ADC_TypeDef*)_ADC_REG)->SR, ADC_SR_RXNE_Pos);
		//static const uint32_t _ADC_DR					= (uint32_t)(&((ADC_TypeDef*)(uint32_t)_ADC_REG)->DR);
		static const uint32_t _ADC_DR					= _ADC_REG + (uint32_t)(&((ADC_TypeDef*)0)->DR);
		static const uint32_t _ADC_SR					= _ADC_REG + (uint32_t)(&((ADC_TypeDef*)0)->SR);
		
		static const uint32_t _ADC_CR1				=
			//( ADC_CR1_AWDEN ) |
			//( ADC_CR1_JAWDEN ) |
			//( (0UL) << ADC_CR1_DUALMOD_Pos ) |
			//( (0UL) << ADC_CR1_DISCNUM_Pos ) |
			//( ADC_CR1_JDISCEN ) |
			//( ADC_CR1_DISCEN ) |
			//( ADC_CR1_JAUTO ) |
			//( ADC_CR1_AWDSGL ) |
			//( ADC_CR1_SCAN ) |
			//( ADC_CR1_JEOCIE ) |
			//( ADC_CR1_AWDIE ) |
			//( ADC_CR1_EOCIE ) |
			//( (0UL) << ADC_CR1_AWDCH_Pos ) |
			(0);

		static const uint32_t _ADC_CR2				=
			//( ADC_CR2_TSVREFE ) |
			//( ADC_CR2_SWSTART ) |
			//( ADC_CR2_JSWSTART ) |
			//( ADC_CR2_EXTTRIG ) |
			( 7UL << ADC_CR2_EXTSEL_Pos ) |			// SWSTART
			//( ADC_CR2_JEXTTRIG ) |
			//( 0UL << ADC_CR2_JEXTSEL ) |
			( ((_DataBits & data_bits::_align_mask) == data_bits::_left) ? (ADC_CR2_ALIGN) : (0) ) |
			//( ADC_CR2_DMA ) |
			//( _calibrate_at_startup ? (ADC_CR2_RSTCAL | ADC_CR2_CAL) : (0) ) |
			//( ADC_CR2_CONT ) |
			( ADC_CR2_ADON ) |
			(0);

		static const uint32_t _smp_def =
			(_sampling_delay_min_default <   2) ?	(0) :	//   1.5 cycles
			(_sampling_delay_min_default <   8) ?	(1) :	//   7.5 cycles
			(_sampling_delay_min_default <  14) ?	(2) :	//  13.5 cycles
			(_sampling_delay_min_default <  29) ?	(3) :	//  28.5 cycles
			(_sampling_delay_min_default <  42) ?	(4) :	//  41.5 cycles
			(_sampling_delay_min_default <  56) ?	(5) :	//  55.5 cycles
			(_sampling_delay_min_default <  72) ?	(6) :	//  71.5 cycles
			(_sampling_delay_min_default < 240) ?	(7) :	// 239.5 cycles
													(7);

		static const uint32_t _ADC_SMPR1			=
			((_smp_def << ADC_SMPR1_SMP10_Pos) & ADC_SMPR1_SMP10_Msk) |
			((_smp_def << ADC_SMPR1_SMP11_Pos) & ADC_SMPR1_SMP11_Msk) |
			((_smp_def << ADC_SMPR1_SMP12_Pos) & ADC_SMPR1_SMP12_Msk) |
			((_smp_def << ADC_SMPR1_SMP13_Pos) & ADC_SMPR1_SMP13_Msk) |
			((_smp_def << ADC_SMPR1_SMP14_Pos) & ADC_SMPR1_SMP14_Msk) |
			((_smp_def << ADC_SMPR1_SMP15_Pos) & ADC_SMPR1_SMP15_Msk) |
			((_smp_def << ADC_SMPR1_SMP16_Pos) & ADC_SMPR1_SMP16_Msk) |
			((_smp_def << ADC_SMPR1_SMP17_Pos) & ADC_SMPR1_SMP17_Msk) |
			(0);

		static const uint32_t _ADC_SMPR2			=
			((_smp_def << ADC_SMPR2_SMP0_Pos) & ADC_SMPR2_SMP0_Msk) |
			((_smp_def << ADC_SMPR2_SMP1_Pos) & ADC_SMPR2_SMP1_Msk) |
			((_smp_def << ADC_SMPR2_SMP2_Pos) & ADC_SMPR2_SMP2_Msk) |
			((_smp_def << ADC_SMPR2_SMP3_Pos) & ADC_SMPR2_SMP3_Msk) |
			((_smp_def << ADC_SMPR2_SMP4_Pos) & ADC_SMPR2_SMP4_Msk) |
			((_smp_def << ADC_SMPR2_SMP5_Pos) & ADC_SMPR2_SMP5_Msk) |
			((_smp_def << ADC_SMPR2_SMP6_Pos) & ADC_SMPR2_SMP6_Msk) |
			((_smp_def << ADC_SMPR2_SMP7_Pos) & ADC_SMPR2_SMP7_Msk) |
			((_smp_def << ADC_SMPR2_SMP8_Pos) & ADC_SMPR2_SMP8_Msk) |
			((_smp_def << ADC_SMPR2_SMP9_Pos) & ADC_SMPR2_SMP9_Msk) |
			(0);

		static const uint32_t _RCC_APBENR_BB		= REG_REMAP::_RCC_APBENR_BB			;

		//////////////////////////////////////////////////////////////////////////
		static bool wait(uint32_t adc_event, IRQn_Type _irqn, uint32_t timeout)
		{
			bool res = false;

			if(IS_BB_REG_RESET(_cfg_::_ADC_CR2_ADON_BB))
				return res;								// the ADC is not initialized yet

			// Clear regular group conversion flag
			(void)READ_BB_REG(_cfg_::_ADC_DR);			//RESET_BB_REG(_cfg_::_ADC_SR_EOC_BB);

			// Start conversion of regular group
			// Force convert
			SET_BB_REG(_cfg_::_ADC_CR2_ADON_BB);		//SET_BB_REG(_cfg_::_ADC_CR2_SWSTART_BB);

			// Reset the pending flag - enabling staying in WFE
			NVIC_ClearPendingIRQ(_irqn);

			// Wait until End of Conversion flag is raised
			while(IS_BB_REG_RESET(adc_event))			//while(!NVIC_GetPendingIRQ(_irqn))
				__WFE();

			res = true;

			// Disable Events from ADC to execute WFE
			RESET_BB_REG(_cfg_::_ADC_CR1_EOCIE_BB);

			// Disable DMA
			RESET_BB_REG(_cfg_::_ADC_CR2_DMA_BB);

			return res;
		}
	};
	
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
			RESET_BB_REG(_cfg_::_ADC_CR2_ADON_BB);			// <-- TODO: Implement waiting for IO completion before disabling
		}
		static void finished()
		{
			// Enable peripheral clock
			SET_BB_REG(_cfg_::_ADC_CR2_ADON_BB);
		}
	};
	
	template <class sysclock>
	static void init()
	{
		// Peripheral clock enable
		SET_BB_REG(_cfg_::_RCC_APBENR_BB);
		__NOP();
		
		// Disable the peripheral
		RESET_BB_REG(_cfg_::_ADC_CR2_ADON_BB);		// on_sysclock_changing<sysclock>::starting();
		
		// Set the ADC Communication parameters
		on_sysclock_changing<sysclock>::finished();

		// CR1 Configuration
		((ADC_TypeDef*)_cfg_::_ADC_REG)->CR1 = _cfg_::_ADC_CR1;

		// CR2 Configuration
		((ADC_TypeDef*)_cfg_::_ADC_REG)->CR2 = _cfg_::_ADC_CR2;

		// reset all and setup L
		((ADC_TypeDef*)_cfg_::_ADC_REG)->SQR1 = (0 << ADC_SQR1_L_Pos) & ADC_SQR1_L_Msk;

		if(_cfg_::_calibrate_at_startup)
		{
			// when callibration enabled, add delay before enabling of ADC
			uint32_t dly = sysclock::_cfg_::_HCLK_Hz
				/ ::mcu::stm32::adc::adc_cfg<_cfg_::_AdcID, sysclock>::_PCLK_Hz
				* ::mcu::stm32::adc::ADC_PRECALIBRATION_DELAY_CLOCKS;
			while(dly--) {}
		}
		
		// CR2 Configuration
		((ADC_TypeDef*)_cfg_::_ADC_REG)->CR2 = _cfg_::_ADC_CR2;
		
		if(_cfg_::_calibrate_at_startup)
		{
			// Resets ADC calibration registers
		    SET_BB_REG(_cfg_::_ADC_CR2_RSTCAL_BB);

			// Wait for calibration reset completion
			while(IS_BB_REG_SET(_cfg_::_ADC_CR2_RSTCAL_BB)) {}

			// Start ADC calibration
			SET_BB_REG(_cfg_::_ADC_CR2_CAL_BB);

			// Wait for calibration completion
			while(IS_BB_REG_SET(_cfg_::_ADC_CR2_CAL_BB)) {}
		}

		// initialize sampling delay to default
		((ADC_TypeDef*)_cfg_::_ADC_REG)->SMPR1 = _cfg_::_ADC_SMPR1;
		((ADC_TypeDef*)_cfg_::_ADC_REG)->SMPR2 = _cfg_::_ADC_SMPR2;

		// skip the first convertion
		if(_cfg_::_skip_first_convertion)
		{
			typedef adc_channel<adc<CFG>, channel_id::channel_vref, sampling_delay_min::_default> adc_ch;
			(void)read<adc_ch, prefered_type::_type>(TIMEOUT_INFINITE);
		}
	}
	
	static void update()
	{
	}

	//////////////////////////////////////////////////////////////////////////
	static bool recalibrate()
	{
		return false;//adc_port< CFG >::recalibrate();
	}

	//////////////////////////////////////////////////////////////////////////
	template<class ADC_CH, class TYPE>
	static TYPE read(uint32_t timeout = TIMEOUT_INFINITE)
	{
		uint32_t res = prefered_type::_max + 1;
		
		stm32::registers::adc_init_channel<_cfg_, _cfg_::_AdcID, ADC_CH::_ChannelID, ADC_CH::_SamplingDelayMin>::exec();
		
		// Enable Events from ADC to execute WFE
		SET_BB_REG(_cfg_::_ADC_CR1_EOCIE_BB);

		if(!_cfg_::wait(_cfg_::_ADC_SR_EOC_BB, _cfg_::_IRQn, timeout))
			return res;

		// Return ADC converted value and reset ADC_SR_EOC
		res = READ_BB_REG(_cfg_::_ADC_DR);

		return res;
	}

	//////////////////////////////////////////////////////////////////////////
	struct read_wfe
	{
		template<class ADC_CH, class TYPE>
		static uint32_t exec(TYPE* buf, uint32_t size, uint32_t timeout = TIMEOUT_INFINITE)
		{
			uint32_t cnt = 0;

			stm32::registers::adc_init_channel<_cfg_, _cfg_::_AdcID, ADC_CH::_ChannelID, ADC_CH::_SamplingDelayMin>::exec();

			// Disable DMA
			RESET_BB_REG(_cfg_::_ADC_CR2_DMA_BB);
			
			// Enable Events from ADC to execute WFE
			SET_BB_REG(_cfg_::_ADC_CR1_EOCIE_BB);

			// Clear regular group conversion flag
			(void)READ_BB_REG(_cfg_::_ADC_DR);			//RESET_BB_REG(_cfg_::_ADC_SR_EOC_BB);

			while(size--)
			{
				// Start conversion of regular group
				//SET_BB_REG(_cfg_::_ADC_CR2_SWSTART_BB);
				
				// Force convert
				SET_BB_REG(_cfg_::_ADC_CR2_ADON_BB);

				// Reset the pending flag - enabling staying in WFE
				NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(_cfg_::_IRQn));

				// Wait until End of Conversion flag is raised
				while(IS_BB_REG_RESET(_cfg_::_ADC_SR_EOC_BB)) //while(!NVIC_GetPendingIRQ(static_cast<IRQn_Type>(_cfg_::_IRQn)))
				{
					__WFE();
				}

				//if(IS_BB_REG_RESET(_cfg_::_ADC_SR_EOC_BB))
				//	return cnt;

				// Clear regular group conversion flag
				//WRITE_BB_REG(_cfg_::_ADC_SR, ~(ADC_SR_STRT | ADC_SR_EOC));

				// Return ADC converted value and reset ADC_SR_EOC
				*buf++ = READ_BB_REG(_cfg_::_ADC_DR);
				cnt++;
			}

			// Disable Events from ADC to execute WFE
			RESET_BB_REG(_cfg_::_ADC_CR1_EOCIE_BB);

			// Disable the peripheral
			//RESET_BB_REG(_cfg_::_ADC_CR2_ADON_BB);

			return cnt;
		}
	};
	
	//////////////////////////////////////////////////////////////////////////
	struct read_dma
	{
		template<class ADC_CH, class TYPE>
		static uint32_t exec(TYPE* buf, uint32_t size, uint32_t timeout = TIMEOUT_INFINITE)
		{
			typedef ::mcu::dma::dma<_set_::_DmaID> _dma_;

			// Disable Events from ADC to execute WFE
			//RESET_BB_REG(_cfg_::_ADC_CR1_EOCIE_BB);

			stm32::registers::adc_init_channel<_cfg_, _cfg_::_AdcID, ADC_CH::_ChannelID, ADC_CH::_SamplingDelayMin>::exec();

			// Clear regular group conversion flag
			(void)READ_BB_REG(_cfg_::_ADC_DR);			//RESET_BB_REG(_cfg_::_ADC_SR_EOC_BB);

			// Enable Continuous conversion mode for regular group 
			SET_BB_REG(_cfg_::_ADC_CR2_CONT_BB);
			
			return _dma_::template read<_cfg_::_ADC_CR2_DMA_BB, _set_::_DmaCh, dma::level::low>(_cfg_::_ADC_DR, buf, size, timeout);
		}
	};
	
	//////////////////////////////////////////////////////////////////////////
	template<class ADC_CH, class TYPE>
	static uint32_t read(TYPE* buf, uint32_t size, uint32_t timeout = TIMEOUT_INFINITE)
	{
		return ::aux::if_c<_cfg_::_UseDma, read_dma, read_wfe>::_result ::template
			exec<ADC_CH>(buf, size, timeout);
	}
};

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace adc
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__stm32f1xx_adc_hpp__*/
