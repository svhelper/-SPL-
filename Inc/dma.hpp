#ifndef __dma_hpp__
#define __dma_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

#include "objtypes.hpp"

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {
namespace dma {

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace dma_id {
typedef enum
{
	dma_1,
	dma_2,
	dma_3,
	dma_4,
	dma_5,
	dma_6,
	dma_7,
	dma_8,

	invalid			= 0xFFFF,
} dma_id;
} // namespace dma_id

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace channel {

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
	
	invalid			= 0xFFFF,
} channel;

} // namespace channel

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mode {
typedef enum
{
	// service
	_src_mask			= 1UL << 0,
	_dst_mask			= 1UL << 1,
	
	_src_mem			= (unsigned)( 0) & _src_mask,
	_dst_mem			= (unsigned)( 0) & _dst_mask,
	_src_periph			= (unsigned)(-1) & _src_mask,
	_dst_periph			= (unsigned)(-1) & _dst_mask,
	
	// values
	mem_to_mem			= _src_mem | _dst_mem,
	mem_to_periph		= _src_mem | _dst_periph,
	periph_to_periph	= _src_periph | _dst_periph,
	periph_to_mem		= _src_periph | _dst_mem,
} mode;
} // namespace mode

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace level {
typedef enum
{
	low,
	medium,
	high,
	very_high,
} level;
} // namespace level

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace size {
typedef enum
{
	byte,
	half_word,
	word,
} size;
} // namespace size

namespace step {
typedef enum
{
	auto_def,
	none,

	byte,
	half_word,
	word,
} step;
} // namespace size

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace dma

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace dma { namespace config {
//	template <
//				dma_id::dma_id		DmaID,
//				channel::channel	ChannelID,
//				mode::mode			Mode		= mode::mem_to_mem,
//				size::size			SrcSize		= size::word,
//				step::step			SrcInc		= step::auto_def,
//				size::size			DstSize		= size::word,
//				step::step			DstInc		= step::auto_def,
//				level::level		Level		= level::low
//			>
//	struct config
//	{
//		static const dma_id::dma_id				_DmaID		= DmaID		;
//		static const channel::channel			_ChannelID	= ChannelID	;
//		static const mode::mode					_Mode		= Mode		;
//		static const size::size					_SrcSize	= SrcSize	;
//		static const step::step					_SrcInc		=
//			SrcInc						!= step::auto_def		? SrcInc			:
//			(Mode & mode::_src_mask)	== mode::_src_periph	? step::none		:
//			SrcSize						== size::byte			? step::byte		:
//			SrcSize						== size::half_word		? step::half_word	:
//			SrcSize						== size::word			? step::word		:
//			SrcInc;
//		static const size::size					_DstSize	= DstSize	;
//		static const step::step					_DstInc		=
//			DstInc						!= step::auto_def		? DstInc			:
//			(Mode & mode::_dst_mask)	== mode::_dst_periph	? step::none		:
//			DstSize						== size::byte			? step::byte		:
//			DstSize						== size::half_word		? step::half_word	:
//			DstSize						== size::word			? step::word		:
//			DstInc;
//		static const level::level				_Level		= Level		;
//	};

//	//------------------------------------------------------------------------
//	template <size::size SIZE, step::step STEP> struct convert_c
//	{
//		static const size::size		_size = SIZE;
//		static const step::step		_step = STEP;
//	};

//	template <class TYPE> struct convert { };
//	template <> struct convert<uint8_t > : public convert_c<size::byte,			step::byte		> { };
//	template <> struct convert<uint16_t> : public convert_c<size::half_word,	step::half_word	> { };
//	template <> struct convert<uint32_t> : public convert_c<size::word,			step::word		> { };

} } //namespace dma::config

/************************************************************************/
/*                                                                      */
/************************************************************************/	
namespace dma {

template < dma_id::dma_id DmaID >
class dma_port;

template < dma_id::dma_id DmaID >
class dma
	: public dma_port< DmaID >
	, public obj::obj< obj::type_id::dma, DmaID >
{
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
			dma_port< DmaID >::template on_sysclock_changing<sysclock>::starting();
		}
		static void finished()
		{
			// Enable peripheral
			dma_port< DmaID >::template on_sysclock_changing<sysclock>::finished();
		}
	};
	
	template <class sysclock>
	static void init()
	{
		dma_port< DmaID >::template init<sysclock>();
	}
	
	static void update()
	{
		dma_port< DmaID >::update();
	}

	template<uint32_t PeriphDmaEn, channel::channel ChannelID, level::level Level, class PeriphType, class MemType>
	static uint32_t read(PeriphType PeriphAddr, MemType *MemAddr, uint32_t Size, uint32_t timeout = TIMEOUT_INFINITE)
	{
		return dma_port< DmaID >::template read<PeriphDmaEn, ChannelID, Level, PeriphType, MemType>(PeriphAddr, MemAddr, Size, timeout);
	}

	template<uint32_t PeriphDmaEn, channel::channel ChannelID, level::level Level, class PeriphType, class MemType>
	static uint32_t write(PeriphType PeriphAddr, const MemType *MemAddr, uint32_t Size, uint32_t timeout = TIMEOUT_INFINITE)
	{
		return dma_port< DmaID >::template write<PeriphDmaEn, ChannelID, Level, PeriphType, MemType>(PeriphAddr, MemAddr, Size, timeout);
	}
};


//------------------------------------------------------------------------
//template <
//			dma_id::dma_id		DmaID,
//			channel::channel	ChannelID,
//			level::level		Level		= level::low,
//			size::size			SrcSize		= size::word,
//			step::step			SrcInc		= step::auto_def,
//			size::size			DstSize		= size::word,
//			step::step			DstInc		= step::auto_def
//		>
//struct mem_to_mem : public dma< config::config< DmaID, ChannelID, mode::mem_to_mem, SrcSize, SrcInc, DstSize, DstInc, Level > >
//{};

//template <
//			dma_id::dma_id		DmaID,
//			channel::channel	ChannelID,
//			level::level		Level		= level::low,
//			size::size			DataSize	= size::word,
//			step::step			PeriphInc	= step::none
//		>
//struct periph_to_mem : public dma< config::config< DmaID, ChannelID, mode::periph_to_mem, DataSize, PeriphInc, DataSize, step::auto_def, Level > >
//{};

//template <
//			dma_id::dma_id		DmaID,
//			channel::channel	ChannelID,
//			level::level		Level		= level::low,
//			size::size			DataSize	= size::word,
//			step::step			PeriphInc	= step::none
//		>
//struct mem_to_periph : public dma< config::config< DmaID, ChannelID, mode::mem_to_periph, DataSize, step::auto_def, DataSize, PeriphInc, Level > >
//{};

////------------------------------------------------------------------------
//template <
//			dma_id::dma_id		DmaID,
//			channel::channel	ChannelID,
//			class				SrcType,
//			class				DstType,
//			level::level		Level		= level::low
//		>
//struct periph_to_mem_by_type
//	: public dma<
//		config::config<
//				DmaID, ChannelID, mode::periph_to_mem,
//				config::convert<SrcType>::size, step::auto_def,
//				config::convert<DstType>::size, step::auto_def,
//				Level
//			>
//		>
//{};

////------------------------------------------------------------------------
//template <
//			dma_id::dma_id		DmaID,
//			channel::channel	ChannelID,
//			level::level		Level		= level::low
//		>
//struct periph_to_mem_byte : public periph_to_mem< DmaID, ChannelID, Level, size::byte > {};

//template <
//			dma_id::dma_id		DmaID,
//			channel::channel	ChannelID,
//			level::level		Level		= level::low
//		>
//struct periph_to_mem_half_word : public periph_to_mem< DmaID, ChannelID, Level, size::half_word > {};

//template <
//			dma_id::dma_id		DmaID,
//			channel::channel	ChannelID,
//			level::level		Level		= level::low
//		>
//struct periph_to_mem_word : public periph_to_mem< DmaID, ChannelID, Level, size::word > {};

////------------------------------------------------------------------------
//template <
//			dma_id::dma_id		DmaID,
//			channel::channel	ChannelID,
//			level::level		Level		= level::low
//		>
//struct mem_to_periph_byte : public mem_to_periph< DmaID, ChannelID, Level, size::byte > {};

//template <
//			dma_id::dma_id		DmaID,
//			channel::channel	ChannelID,
//			level::level		Level		= level::low
//		>
//struct mem_to_periph_half_word : public mem_to_periph< DmaID, ChannelID, Level, size::half_word > {};

//template <
//			dma_id::dma_id		DmaID,
//			channel::channel	ChannelID,
//			level::level		Level		= level::low
//		>
//struct mem_to_periph_word : public mem_to_periph< DmaID, ChannelID, Level, size::word > {};

//------------------------------------------------------------------------

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace dma
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__dma_hpp__*/
