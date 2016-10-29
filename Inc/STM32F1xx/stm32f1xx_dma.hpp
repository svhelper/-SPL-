#ifndef __stm32f1xx_dma_hpp__
#define __stm32f1xx_dma_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

//////////////////////////////////////////////////////////////////////////
#include "dma.hpp"

//////////////////////////////////////////////////////////////////////////
#include "CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"
#include "stm32f1xx/stm32f1xx_registers.hpp"

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {
namespace stm32 {

namespace dma {
	using namespace ::mcu::dma;

	template <
		dma_id::dma_id DmaID,
		uint32_t RCC_APBENR_BB
		>
	struct dma_set_c
	{
		static const dma_id::dma_id		_DmaID				= DmaID;
		static const uint32_t			_RCC_APBENR_BB		= RCC_APBENR_BB;
	};

	template <dma_id::dma_id DmaID>
	struct dma_set;

#ifdef RCC_AHBENR_DMA1EN_Pos
	template <> struct dma_set<dma_id::dma_1> : public dma_set_c< dma_id::dma_1, FROM_ADDRESS_BIT_POS_TO_BB(&RCC->AHBENR, RCC_AHBENR_DMA1EN_Pos) > {};
#endif /*RCC_AHBENR_DMA1EN_Pos*/
#ifdef RCC_AHBENR_DMA2EN_Pos
	template <> struct dma_set<dma_id::dma_2> : public dma_set_c< dma_id::dma_2, FROM_ADDRESS_BIT_POS_TO_BB(&RCC->AHBENR, RCC_AHBENR_DMA2EN_Pos) > {};
#endif /*RCC_AHBENR_DMA2EN_Pos*/
#ifdef RCC_AHBENR_DMA3EN_Pos
	template <> struct dma_set<dma_id::dma_2> : public dma_set_c< dma_id::dma_3, FROM_ADDRESS_BIT_POS_TO_BB(&RCC->AHBENR, RCC_AHBENR_DMA3EN_Pos) > {};
#endif /*RCC_AHBENR_DMA2EN_Pos*/

	//------------------------------------------------------------------------
	template <class TYPE, size::size SIZE, step::step STEP> struct convert_c
	{
		typedef TYPE _type;
		
		static const size::size		_size			= SIZE;
		static const step::step		_step			= STEP;
		
		static const uint32_t		_bytes			= sizeof(TYPE);
		static const uint32_t		_ccr_size		=
			(sizeof(TYPE) * 8) ==  8 ?	0 :
			(sizeof(TYPE) * 8) == 16 ?	1 :
			(sizeof(TYPE) * 8) == 32 ?	2 :
										3 ;
		static const uint32_t		_ccr_msize		= _ccr_size << DMA_CCR_MSIZE_Pos;
		static const uint32_t		_ccr_psize		= _ccr_size << DMA_CCR_PSIZE_Pos;
	};

	template <class TYPE> struct convert { };
	template <> struct convert<uint8_t > : public convert_c<uint8_t ,	size::byte,			step::byte		> { };
	template <> struct convert<uint16_t> : public convert_c<uint16_t,	size::half_word,	step::half_word	> { };
	template <> struct convert<uint32_t> : public convert_c<uint32_t,	size::word,			step::word		> { };

	//////////////////////////////////////////////////////////////////////////
	template <dma_id::dma_id DmaId, channel::channel DmaCh, IRQn_Type IRQn, uint32_t DMA_REG, uint32_t DMA_CH_REG>
	class dma_ch_set_c
	{
	public:
		static const dma_id::dma_id		_DmaId		= DmaId;
		static const channel::channel	_DmaCh		= DmaCh;
		static const IRQn_Type			_IRQn		= IRQn;
		static const uint32_t			_DMA_REG	= DMA_REG;
		static const uint32_t			_DMA_CH_REG	= DMA_CH_REG;
		static const uint32_t			_DMA_IFCR	= 
			(DmaCh == channel::channel_1) ? (DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1) :
			(DmaCh == channel::channel_2) ? (DMA_IFCR_CGIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTEIF2) :
			(DmaCh == channel::channel_3) ? (DMA_IFCR_CGIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3) :
			(DmaCh == channel::channel_4) ? (DMA_IFCR_CGIF4 | DMA_IFCR_CTCIF4 | DMA_IFCR_CHTIF4 | DMA_IFCR_CTEIF4) :
			(DmaCh == channel::channel_5) ? (DMA_IFCR_CGIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5) :
			(DmaCh == channel::channel_6) ? (DMA_IFCR_CGIF6 | DMA_IFCR_CTCIF6 | DMA_IFCR_CHTIF6 | DMA_IFCR_CTEIF6) :
			(DmaCh == channel::channel_7) ? (DMA_IFCR_CGIF7 | DMA_IFCR_CTCIF7 | DMA_IFCR_CHTIF7 | DMA_IFCR_CTEIF7) :
			(0);
	};
	
	template <dma_id::dma_id DmaId, channel::channel DmaCh> class dma_ch_set;

#ifdef DMA1_Channel0_BASE
	template <> class dma_ch_set<dma_id::dma_1, channel::channel_0> : public dma_ch_set_c<dma_id::dma_1, channel::channel_0, DMA1_Channel0_IRQn, DMA1_BASE, DMA1_Channel0_BASE> {};
#endif //DMA1_Channel0_BASE
#ifdef DMA1_Channel1_BASE
	template <> class dma_ch_set<dma_id::dma_1, channel::channel_1> : public dma_ch_set_c<dma_id::dma_1, channel::channel_1, DMA1_Channel1_IRQn, DMA1_BASE, DMA1_Channel1_BASE> {};
#endif //DMA1_Channel1_BASE
#ifdef DMA1_Channel2_BASE
	template <> class dma_ch_set<dma_id::dma_1, channel::channel_2> : public dma_ch_set_c<dma_id::dma_1, channel::channel_2, DMA1_Channel2_IRQn, DMA1_BASE, DMA1_Channel2_BASE> {};
#endif //DMA1_Channel2_BASE
#ifdef DMA1_Channel3_BASE
	template <> class dma_ch_set<dma_id::dma_1, channel::channel_3> : public dma_ch_set_c<dma_id::dma_1, channel::channel_3, DMA1_Channel3_IRQn, DMA1_BASE, DMA1_Channel3_BASE> {};
#endif //DMA1_Channel3_BASE
#ifdef DMA1_Channel4_BASE
	template <> class dma_ch_set<dma_id::dma_1, channel::channel_4> : public dma_ch_set_c<dma_id::dma_1, channel::channel_4, DMA1_Channel4_IRQn, DMA1_BASE, DMA1_Channel4_BASE> {};
#endif //DMA1_Channel4_BASE
#ifdef DMA1_Channel5_BASE
	template <> class dma_ch_set<dma_id::dma_1, channel::channel_5> : public dma_ch_set_c<dma_id::dma_1, channel::channel_5, DMA1_Channel5_IRQn, DMA1_BASE, DMA1_Channel5_BASE> {};
#endif //DMA1_Channel5_BASE
#ifdef DMA1_Channel6_BASE
	template <> class dma_ch_set<dma_id::dma_1, channel::channel_6> : public dma_ch_set_c<dma_id::dma_1, channel::channel_6, DMA1_Channel6_IRQn, DMA1_BASE, DMA1_Channel6_BASE> {};
#endif //DMA1_Channel6_BASE
#ifdef DMA1_Channel7_BASE
	template <> class dma_ch_set<dma_id::dma_1, channel::channel_7> : public dma_ch_set_c<dma_id::dma_1, channel::channel_7, DMA1_Channel7_IRQn, DMA1_BASE, DMA1_Channel7_BASE> {};
#endif //DMA1_Channel7_BASE

#ifdef DMA2_Channel0_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_0> : public dma_ch_set_c<dma_id::dma_2, channel::channel_0, DMA2_Channel0_IRQn, DMA2_BASE, DMA2_Channel0_BASE> {};
#endif //DMA2_Channel0_BASE
#ifdef DMA2_Channel1_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_1> : public dma_ch_set_c<dma_id::dma_2, channel::channel_1, DMA2_Channel1_IRQn, DMA2_BASE, DMA2_Channel1_BASE> {};
#endif //DMA2_Channel1_BASE
#ifdef DMA2_Channel2_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_2> : public dma_ch_set_c<dma_id::dma_2, channel::channel_2, DMA2_Channel2_IRQn, DMA2_BASE, DMA2_Channel2_BASE> {};
#endif //DMA2_Channel2_BASE
#ifdef DMA2_Channel3_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_3> : public dma_ch_set_c<dma_id::dma_2, channel::channel_3, DMA2_Channel3_IRQn, DMA2_BASE, DMA2_Channel3_BASE> {};
#endif //DMA2_Channel3_BASE
#ifdef DMA2_Channel4_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_4> : public dma_ch_set_c<dma_id::dma_2, channel::channel_4, DMA2_Channel4_IRQn, DMA2_BASE, DMA2_Channel4_BASE> {};
#endif //DMA2_Channel4_BASE
#ifdef DMA2_Channel5_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_5> : public dma_ch_set_c<dma_id::dma_2, channel::channel_5, DMA2_Channel5_IRQn, DMA2_BASE, DMA2_Channel5_BASE> {};
#endif //DMA2_Channel5_BASE
#ifdef DMA2_Channel6_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_6> : public dma_ch_set_c<dma_id::dma_2, channel::channel_6, DMA2_Channel6_IRQn, DMA2_BASE, DMA2_Channel6_BASE> {};
#endif //DMA2_Channel6_BASE
#ifdef DMA2_Channel7_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_7> : public dma_ch_set_c<dma_id::dma_2, channel::channel_7, DMA2_Channel7_IRQn, DMA2_BASE, DMA2_Channel7_BASE> {};
#endif //DMA2_Channel7_BASE

#ifdef DMA3_Channel0_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_0> : public dma_ch_set_c<dma_id::dma_2, channel::channel_0, DMA3_Channel0_IRQn, DMA3_BASE, DMA3_Channel0_BASE> {};
#endif //DMA3_Channel0_BASE
#ifdef DMA3_Channel1_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_1> : public dma_ch_set_c<dma_id::dma_2, channel::channel_1, DMA3_Channel1_IRQn, DMA3_BASE, DMA3_Channel1_BASE> {};
#endif //DMA3_Channel1_BASE
#ifdef DMA3_Channel2_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_2> : public dma_ch_set_c<dma_id::dma_2, channel::channel_2, DMA3_Channel2_IRQn, DMA3_BASE, DMA3_Channel2_BASE> {};
#endif //DMA3_Channel2_BASE
#ifdef DMA3_Channel3_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_3> : public dma_ch_set_c<dma_id::dma_2, channel::channel_3, DMA3_Channel3_IRQn, DMA3_BASE, DMA3_Channel3_BASE> {};
#endif //DMA3_Channel3_BASE
#ifdef DMA3_Channel4_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_4> : public dma_ch_set_c<dma_id::dma_2, channel::channel_4, DMA3_Channel4_IRQn, DMA3_BASE, DMA3_Channel4_BASE> {};
#endif //DMA3_Channel4_BASE
#ifdef DMA3_Channel5_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_5> : public dma_ch_set_c<dma_id::dma_2, channel::channel_5, DMA3_Channel5_IRQn, DMA3_BASE, DMA3_Channel5_BASE> {};
#endif //DMA3_Channel5_BASE
#ifdef DMA3_Channel6_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_6> : public dma_ch_set_c<dma_id::dma_2, channel::channel_6, DMA3_Channel6_IRQn, DMA3_BASE, DMA3_Channel6_BASE> {};
#endif //DMA3_Channel6_BASE
#ifdef DMA3_Channel7_BASE
	template <> class dma_ch_set<dma_id::dma_2, channel::channel_7> : public dma_ch_set_c<dma_id::dma_2, channel::channel_7, DMA3_Channel7_IRQn, DMA3_BASE, DMA3_Channel7_BASE> {};
#endif //DMA3_Channel7_BASE

} // namespace dma

///************************************************************************/
///*                                                                      */
///************************************************************************/
//namespace registers {
//} // namespace registers

///************************************************************************/
///*                                                                      */
///************************************************************************/

} // namespace stm32

/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace dma {

template < dma_id::dma_id DmaID >
class dma_port
{
private:
	typedef ::mcu::stm32::dma::dma_set< DmaID > _set_;

public:
	class _cfg_
	{
	public:
		static const dma_id::dma_id				_DmaID					= DmaID;

		static const uint32_t					_RCC_APBENR_BB			= _set_::_RCC_APBENR_BB;
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
		}
		static void finished()
		{
			// Enable peripheral clock
		}
	};
	
	template <class sysclock>
	static void init()
	{
		// Peripheral clock enable
		SET_BB_REG(_cfg_::_RCC_APBENR_BB);
		__NOP();
	}
	
	static void update()
	{
	}

	template<uint32_t PeriphDmaEn, channel::channel ChannelID, level::level Level, class PeriphType, class MemType>
	static uint32_t read(PeriphType PeriphAddr, MemType *MemAddr, uint32_t Size, uint32_t timeout)
	{
		typedef ::mcu::stm32::dma::dma_ch_set<DmaID, ChannelID> _chset_;
		
		static const uint32_t _DMA_CCRx =
			//( DMA_CCR_MEM2MEM ) |				// Memory to memory mode
			(
				(Level == level::low	)	?	(0 << DMA_CCR_PL_Pos) :
				(Level == level::medium	)	?	(1 << DMA_CCR_PL_Pos) :
				(Level == level::high	)	?	(2 << DMA_CCR_PL_Pos) :
												(3 << DMA_CCR_PL_Pos)
				) |			// Channel priority level
			( ::mcu::stm32::dma::convert<MemType>::_ccr_msize ) |		// Memory size
			( ::mcu::stm32::dma::convert<MemType>::_ccr_psize ) |		// Peripheral size
			( DMA_CCR_MINC ) |			// Memory increment mode
			//( DMA_CCR_PINC ) |		// Peripheral increment mode
			//( DMA_CCR_CIRC ) |		// Circular mode
			//( DMA_CCR_DIR ) |			// Data transfer direction: 0=mem -> periphery; 1=mem -> periphery
			//( DMA_CCR_TEIE ) |		// Transfer error interrupt enable
			//( DMA_CCR_HTIE ) |		// Half transfer interrupt enable
			( DMA_CCR_TCIE ) |			// Transfer complete interrupt enable
			( DMA_CCR_EN ) |			// Channel enable
			(0);
		
		STATIC_ASSERT(_chset_::_DMA_IFCR != 0, "Internal error: Invalid definition of MCU periphery");

		// clear flags
		((DMA_TypeDef*)_chset_::_DMA_REG)->IFCR = _chset_::_DMA_IFCR;
		
		// configure channel
		((DMA_Channel_TypeDef*)_chset_::_DMA_CH_REG)->CNDTR = Size;
		((DMA_Channel_TypeDef*)_chset_::_DMA_CH_REG)->CPAR = (uint32_t)PeriphAddr;
		((DMA_Channel_TypeDef*)_chset_::_DMA_CH_REG)->CMAR = (uint32_t)MemAddr;

		// start transferring
		((DMA_Channel_TypeDef*)_chset_::_DMA_CH_REG)->CCR = _DMA_CCRx;
		
		//NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(_chset_::_IRQn));
		NVIC->ICPR[_chset_::_IRQn >> 5UL] = 1UL << (_chset_::_IRQn & 0x1FUL);

		// Enable DMA for periphery
		SET_BB_REG(PeriphDmaEn);

		// Wait for completion of transferring
		while(!(NVIC->ISPR[_chset_::_IRQn >> 5UL] & (1UL << (_chset_::_IRQn & 0x1FUL))))	// while(!NVIC_GetPendingIRQ(static_cast<IRQn_Type>(_chset_::_IRQn)))
			__WFE();

		// Disable DMA for periphery
		RESET_BB_REG(PeriphDmaEn);

		// stop transferring
		((DMA_Channel_TypeDef*)_chset_::_DMA_CH_REG)->CCR = 0;

		//NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(_chset_::_IRQn));
		NVIC->ICPR[_chset_::_IRQn >> 5UL] = 1UL << (_chset_::_IRQn & 0x1FUL);

		return Size;
	}

	template<uint32_t PeriphDmaEn, channel::channel ChannelID, level::level Level, class PeriphType, class MemType>
	static uint32_t write(PeriphType PeriphAddr, const MemType *MemAddr, uint32_t Size, uint32_t timeout)
	{
		typedef ::mcu::stm32::dma::dma_ch_set<DmaID, ChannelID> _chset_;
		
		static const uint32_t _DMA_CCRx =
			//( DMA_CCR_MEM2MEM ) |				// Memory to memory mode
			(
				(Level == level::low	)	?	(0 << DMA_CCR_PL_Pos) :
				(Level == level::medium	)	?	(1 << DMA_CCR_PL_Pos) :
				(Level == level::high	)	?	(2 << DMA_CCR_PL_Pos) :
												(3 << DMA_CCR_PL_Pos)
				) |			// Channel priority level
			( ::mcu::stm32::dma::convert<MemType>::_ccr_msize ) |		// Memory size
			( ::mcu::stm32::dma::convert<MemType>::_ccr_psize ) |		// Peripheral size
			( DMA_CCR_MINC ) |			// Memory increment mode
			//( DMA_CCR_PINC ) |		// Peripheral increment mode
			//( DMA_CCR_CIRC ) |		// Circular mode
			( DMA_CCR_DIR ) |			// Data transfer direction: 0=mem -> periphery; 1=mem -> periphery
			//( DMA_CCR_TEIE ) |		// Transfer error interrupt enable
			//( DMA_CCR_HTIE ) |		// Half transfer interrupt enable
			( DMA_CCR_TCIE ) |			// Transfer complete interrupt enable
			( DMA_CCR_EN ) |			// Channel enable
			(0);
		
		STATIC_ASSERT(_chset_::_DMA_IFCR != 0, "Internal error: Invalid definition of MCU periphery");

		// clear flags
		((DMA_TypeDef*)_chset_::_DMA_REG)->IFCR = _chset_::_DMA_IFCR;
		
		// configure channel
		((DMA_Channel_TypeDef*)_chset_::_DMA_CH_REG)->CNDTR = Size;
		((DMA_Channel_TypeDef*)_chset_::_DMA_CH_REG)->CPAR = (uint32_t)PeriphAddr;
		((DMA_Channel_TypeDef*)_chset_::_DMA_CH_REG)->CMAR = (uint32_t)MemAddr;

		// start transferring
		((DMA_Channel_TypeDef*)_chset_::_DMA_CH_REG)->CCR = _DMA_CCRx;
		
		//NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(_chset_::_IRQn));
		NVIC->ICPR[_chset_::_IRQn >> 5UL] = 1UL << (_chset_::_IRQn & 0x1FUL);

		// Enable DMA for periphery
		SET_BB_REG(PeriphDmaEn);

		// Wait for completion of transferring
		while(!(NVIC->ISPR[_chset_::_IRQn >> 5UL] & (1UL << (_chset_::_IRQn & 0x1FUL))))	// while(!NVIC_GetPendingIRQ(static_cast<IRQn_Type>(_chset_::_IRQn)))
			__WFE();

		// Disable DMA for periphery
		RESET_BB_REG(PeriphDmaEn);

		// stop transferring
		((DMA_Channel_TypeDef*)_chset_::_DMA_CH_REG)->CCR = 0;

		//NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(_chset_::_IRQn));
		NVIC->ICPR[_chset_::_IRQn >> 5UL] = 1UL << (_chset_::_IRQn & 0x1FUL);

		return Size;
	}
};

/************************************************************************/
/*                                                                      */
/************************************************************************/
} // namespace dma
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__stm32f1xx_dma_hpp__*/
