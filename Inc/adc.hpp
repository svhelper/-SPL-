#ifndef __adc_hpp__
#define __adc_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

#include "objtypes.hpp"
#include "_aux_if.hpp"


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {
namespace adc {

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace adc_id {
typedef enum
{
	adc_1,
	adc_2,
	adc_3,
	adc_4,
	adc_5,
	adc_6,
	adc_7,
	adc_8,

	invalid			= 0xFFFF,
} adc_id;
} // namespace adc_id

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace channel_id {

typedef enum
{
	channel_0,
	channel_1,
	channel_2,
	channel_3,
	channel_4,
	channel_5,
	channel_6,
	channel_7,
	channel_8,
	channel_9,
	channel_10,
	channel_11,
	channel_12,
	channel_13,
	channel_14,
	channel_15,
	
	channel_vref	= 0x8000,
	channel_temp,
	
	invalid			= 0xFFFF,
} channel_id;

} // namespace channel_id

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace data_bits {
typedef enum
{
	//service
	_$bits_mask		= (1UL << 6) -1,
	_align_pos		= 7,
	_align_mask		= 1UL << _align_pos,
	
	_right			= 0UL << _align_pos,
	_left			= 1UL << _align_pos,

	// valuse
	_max_left		= _$bits_mask | _left,
	_max_right		= _$bits_mask | _right,
	
	_4bits_right	=  4 | _right,
	_5bits_right	=  5 | _right,
	_6bits_right	=  6 | _right,
	_7bits_right	=  7 | _right,
	_8bits_right	=  8 | _right,
	_9bits_right	=  9 | _right,
	_10bits_right	= 10 | _right,
	_12bits_right	= 12 | _right,
	_14bits_right	= 14 | _right,
	_16bits_right	= 16 | _right,

	_4bits_left		=  4 | _left,
	_5bits_left		=  5 | _left,
	_6bits_left		=  6 | _left,
	_7bits_left		=  7 | _left,
	_8bits_left		=  8 | _left,
	_9bits_left		=  9 | _left,
	_10bits_left	= 10 | _left,
	_12bits_left	= 12 | _left,
	_14bits_left	= 14 | _left,
	_16bits_left	= 16 | _left,

	// aliases
	_max			= _$bits_mask | _right,
	
	_4bits			=  4 | _right,
	_5bits			=  5 | _right,
	_6bits			=  6 | _right,
	_7bits			=  7 | _right,
	_8bits			=  8 | _right,
	_9bits			=  9 | _right,
	_10bits			= 10 | _right,
	_12bits			= 12 | _right,
	_14bits			= 14 | _right,
	_16bits			= 16 | _right,
} data_bits;
} // namespace data_bits

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace flags {
typedef enum
{
	// valuse
	none					= 0,
	calibrate_at_startup	= 1UL << 0,
} flags;
} // namespace flags

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace sampling_delay_min {
typedef enum
{
	// values
	_0						= 0,
	_1						= 1,
	_2						= 2,
	_3						= 3,
	_5						= 5,
	_10						= 10,
	_15						= 15,
	_20						= 20,
	_30						= 30,
	_50						= 50,
	_75						= 75,
	_100					= 100,
	_150					= 150,
	_200					= 200,
	_300					= 300,
	
	_none					= 0,
	_short					= 10,
	_normal					= 30,
	_long					= 0xFFFF,
	
	// service
	_default				= 0xFFFFFF,
} sampling_delay_min;
} // namespace sampling_delay_min

/************************************************************************/
/*                                                                      */
/************************************************************************/
const uint32_t	TIMEOUT_INFINITE		= 0xFFFFFFFF;
} //namespace adc

/************************************************************************/
/*                                                                      */
/************************************************************************/
using namespace ::mcu::adc;

namespace adc { namespace config {
	template <
				adc_id::adc_id							AdcID											,
				data_bits::data_bits					DataBits			= data_bits::_max_right		,
				uint32_t								Flags				= flags::calibrate_at_startup,
				sampling_delay_min::sampling_delay_min	DefSamplingDelayMin = sampling_delay_min::_default,
				bool									UseDma				= true						
			>
	struct config
	{
		static const adc_id::adc_id								_AdcID		= AdcID		;
		static const data_bits::data_bits						_DataBits	= DataBits	;
		static const uint32_t									_Flags		= Flags		;
		static const sampling_delay_min::sampling_delay_min		_DefSamplingDelayMin = DefSamplingDelayMin;
		static const bool										_UseDma		= UseDma	;
	};

} } //namespace adc::config


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace adc {
/************************************************************************/
/*                                                                      */
/************************************************************************/
template <	class									ADC,
			channel_id::channel_id					ChannelID	,
			sampling_delay_min::sampling_delay_min	SamplingDelayMin		= sampling_delay_min::_default
	>
class adc_channel
{
	typedef adc_channel<ADC, ChannelID, SamplingDelayMin> adc_ch_raw;

public:
	static const uint32_t			adc_base_bits	= ADC::_cfg_::_DataBits & data_bits::_$bits_mask;
	static const int32_t			additional_bits	= 0;

public:
	static const channel_id::channel_id					_ChannelID			= ChannelID			;
	static const uint32_t								_DataBits			= (ADC::_cfg_::_DataBits & data_bits::_$bits_mask)	;
	static const sampling_delay_min::sampling_delay_min	_SamplingDelayMin	= SamplingDelayMin	;

	typedef typename ADC::val_type	val_type;
	static const val_type			val_max		= ADC::val_max;

	//////////////////////////////////////////////////////////////////////////
	static typename ADC::val_type read_raw(uint32_t timeout = TIMEOUT_INFINITE)
	{
		return ADC::template read< adc_ch_raw, typename ADC::val_type >(timeout);
	}

	//////////////////////////////////////////////////////////////////////////
	template<class TYPE, uint32_t SIZE>
	static uint32_t read_raw(TYPE (&buf)[SIZE], uint32_t timeout = TIMEOUT_INFINITE)
	{
		return ADC::template read< adc_ch_raw, TYPE >(buf, SIZE, timeout);
	}
	
	//////////////////////////////////////////////////////////////////////////
	template<class TYPE>
	static uint32_t read(TYPE* buf, uint32_t size, uint32_t timeout = TIMEOUT_INFINITE)
	{
		return ADC::template read< adc_ch_raw, TYPE >(buf, size, timeout);
	}
};

/************************************************************************/
/*                                                                      */
/************************************************************************/
template <	class									ADC,
			channel_id::channel_id					ChannelID	,
			uint32_t								DataBits	,
			sampling_delay_min::sampling_delay_min	SamplingDelayMin		= sampling_delay_min::_default
	>
class adc_channel_bits
{
public:
	static const uint32_t			adc_base_bits	= ADC::_cfg_::_DataBits & data_bits::_$bits_mask;
	static const int32_t			additional_bits	= (int32_t)DataBits - (int32_t)adc_base_bits;

private:
	typedef adc_channel<ADC, ChannelID, SamplingDelayMin> adc_ch_raw;
	
	struct inc
	{
		static const uint32_t			sample_count	= 1UL << (2*additional_bits);

		typedef uint32_t				val_type;
		static const val_type			val_max		=	(ADC::val_max * sample_count) >> additional_bits;

		static val_type read_raw(uint32_t timeout = TIMEOUT_INFINITE)
		{
			val_type res = 0;
			uint32_t i;

			for(i=sample_count; i; i--)
				res += adc_ch_raw::read_raw(timeout);

			//typedef typename ADC::val_type adc_type;
			//adc_type arr[sample_count];
			//if(sample_count != adc_ch_raw::read_raw(arr, timeout))
			//	return val_max+1;
			//for(i=0; i<sample_count; i++)
			//	res += arr[i];

			res >>= additional_bits;
			return res;
		}
	};

	struct dec
	{
		typedef uint32_t				val_type;
		static const val_type			val_max		=	(1UL << DataBits) - 1;

		static val_type read_raw(uint32_t timeout = TIMEOUT_INFINITE)
		{
			return adc_ch_raw::read_raw(timeout) >> (adc_base_bits - _DataBits);
		}
	};

public:
	static const channel_id::channel_id					_ChannelID			= ChannelID			;
	static const uint32_t								_DataBits			= DataBits			;
	static const sampling_delay_min::sampling_delay_min	_SamplingDelayMin	= SamplingDelayMin	;

	typedef typename ::aux::if_c< (_DataBits > adc_base_bits), inc, dec > :: _result exec;

	typedef uint32_t				val_type;
	static const val_type			val_max		=	exec::val_max;

	//////////////////////////////////////////////////////////////////////////
	static val_type read_raw(uint32_t timeout = TIMEOUT_INFINITE)
	{
		return exec::read_raw(timeout);
	}
};


/************************************************************************/
/*                                                                      */
/************************************************************************/
template < class CFG >
class adc_port;

template < class CFG >
class adc
	: public adc_port< CFG >
	, public obj::obj< obj::type_id::adc, CFG::_AdcID >
{
public:
	typedef CFG _cfg_;

	typedef typename adc_port< CFG >::prefered_type::_type	val_type;
	static const val_type	val_max = adc_port< CFG >::prefered_type::_max;
	

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
			adc_port< CFG >::template on_sysclock_changing<sysclock>::starting();
		}
		static void finished()
		{
			// Enable peripheral
			adc_port< CFG >::template on_sysclock_changing<sysclock>::finished();
		}
	};
	
	template <class sysclock>
	static void init()
	{
		adc_port< CFG >::template init<sysclock>();
	}
	
	static void update()
	{
		adc_port< CFG >::update();
	}

	//////////////////////////////////////////////////////////////////////////
	static bool recalibrate()
	{
		return adc_port< CFG >::recalibrate();
	}

	//////////////////////////////////////////////////////////////////////////
	template<class ADC_CH, class TYPE>
	static TYPE read(uint32_t timeout = TIMEOUT_INFINITE)
	{
		STATIC_ASSERT(adc_port< CFG >::template check_type<TYPE> :: _valid == true, "The target type is incompatible with ADC dimension or will be truncated");
		return adc_port< CFG >::template read<ADC_CH, TYPE>(timeout);
	}

	//////////////////////////////////////////////////////////////////////////
	template<class ADC_CH, class TYPE>
	static uint32_t read(TYPE* buf, uint32_t size, uint32_t timeout = TIMEOUT_INFINITE)
	{
		STATIC_ASSERT(adc_port< CFG >::template check_type<TYPE> :: _valid == true, "The target type is incompatible with ADC dimension");
		return adc_port< CFG >::template read<ADC_CH>(buf, size, timeout);
	}

	//////////////////////////////////////////////////////////////////////////
	template <sampling_delay_min::sampling_delay_min	SamplingDelayMin = sampling_delay_min::_default>
	class channel_vref : public adc_channel<adc<_cfg_>, channel_id::channel_vref, SamplingDelayMin>
	{
		typedef adc_channel<adc<_cfg_>, channel_id::channel_vref, SamplingDelayMin> ch;

	public:
		static uint32_t read(uint32_t timeout = TIMEOUT_INFINITE)
		{
			return (uint32_t)(__MCU_INT_VREF * 1000UL) * ch::val_max / ch::read_raw(timeout);
		}
	};
	//------------------------------------------------------------------------
	template <sampling_delay_min::sampling_delay_min	SamplingDelayMin = sampling_delay_min::_default>
	class channel_temp : public adc_channel<adc<_cfg_>, channel_id::channel_temp, SamplingDelayMin> {};
};

/************************************************************************/
/*                                                                      */
/************************************************************************/

template <
			adc_id::adc_id			AdcID									,
			data_bits::data_bits	DataBits	= data_bits::_max_right		,
			uint32_t				Flags		= flags::calibrate_at_startup,
			sampling_delay_min::sampling_delay_min	DefSamplingDelayMin = sampling_delay_min::_default
		>
class adc_def: public adc< config::config<AdcID, DataBits, Flags, DefSamplingDelayMin> >
{ };

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace adc
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__adc_hpp__*/
