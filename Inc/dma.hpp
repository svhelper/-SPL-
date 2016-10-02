#ifndef __dma_hpp__
#define __dma_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>


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
	mem_to_mem,
	periph_to_periph,
	periph_to_mem,
	mem_to_periph,
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

/************************************************************************/
/*                                                                      */
/************************************************************************/	
template <
			dma_id::dma_id		DmaID,
			channel::channel	ChannelID,
			mode::mode			Mode		= mode::mem_to_mem,
			size::size			SrcSize		= size::word,
			size::size			DstSize		= size::word,
			level::level		Level		= level::low
		>
class dma_base;

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace dma
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__dma_hpp__*/
