#ifndef __stm32f1xx_registers_hpp__
#define __stm32f1xx_registers_hpp__

#ifndef __cplusplus
# error "This file must be included to the C++ progect"
#endif /*__cplusplus*/

//////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <static_assert.hpp>

//////////////////////////////////////////////////////////////////////////
#include <stm32f1xx.h>


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace mcu {
namespace registers {


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace stm32 {

#ifdef BKP
	BKP_TypeDef bkp					__attribute__((at(BKP_BASE)));
#endif /*BKP*/

#ifdef PWR
	PWR_TypeDef pwr					__attribute__((at(PWR_BASE)));
#endif /*PWR*/

#ifdef RCC
	RCC_TypeDef rcc					__attribute__((at(RCC_BASE)));
#endif /*RCC*/

#ifdef CRC
	CRC_TypeDef crc					__attribute__((at(CRC_BASE)));
#endif /*CRC*/

#ifdef FLASH
	FLASH_TypeDef flash				__attribute__((at(FLASH_R_BASE)));
#endif /*FLASH*/

#ifdef OB
	OB_TypeDef ob					__attribute__((at(OB_BASE)));
#endif /*OB*/

#ifdef DBGMCU
	DBGMCU_TypeDef dbgmcu			__attribute__((at(DBGMCU_BASE)));
#endif /*DBGMCU*/

#ifdef WWDG
	WWDG_TypeDef wwdg				__attribute__((at(WWDG_BASE)));
#endif /*WWDG*/

#ifdef IWDG
	IWDG_TypeDef iwdg				__attribute__((at(IWDG_BASE)));
#endif /*IWDG*/

} // namespace stm32


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace adc {

#ifdef ADC1
	ADC_TypeDef adc1				__attribute__((at(ADC1_BASE)));
#endif /*ADC1*/

#ifdef ADC2
	ADC_TypeDef adc2				__attribute__((at(ADC2_BASE)));
#endif /*ADC2*/

#ifdef ADC12_COMMON
	ADC_Common_TypeDef common		__attribute__((at(ADC1_BASE)));
#endif /*ADC12_COMMON*/
} // namespace adc


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace gpio {

#ifdef GPIOA
	GPIO_TypeDef gpio_a				__attribute__((at(GPIOA_BASE)));
#endif /*GPIOA*/

#ifdef GPIOB
	GPIO_TypeDef gpio_b				__attribute__((at(GPIOB_BASE)));
#endif /*GPIOB*/

#ifdef GPIOC
	GPIO_TypeDef gpio_c				__attribute__((at(GPIOC_BASE)));
#endif /*GPIOC*/

#ifdef GPIOD
	GPIO_TypeDef gpio_d				__attribute__((at(GPIOD_BASE)));
#endif /*GPIOD*/

#ifdef GPIOE
	GPIO_TypeDef gpio_e				__attribute__((at(GPIOE_BASE)));
#endif /*GPIOE*/

#ifdef GPIOF
	GPIO_TypeDef gpio_f				__attribute__((at(GPIOF_BASE)));
#endif /*GPIOF*/

#ifdef GPIOG
	GPIO_TypeDef gpio_g				__attribute__((at(GPIOG_BASE)));
#endif /*GPIOG*/

#ifdef GPIOH
	GPIO_TypeDef gpio_h				__attribute__((at(GPIOH_BASE)));
#endif /*GPIOH*/

#ifdef GPIOI
	GPIO_TypeDef gpio_i				__attribute__((at(GPIOI_BASE)));
#endif /*GPIOI*/

#ifdef EXTI
	EXTI_TypeDef exti				__attribute__((at(EXTI_BASE)));
#endif /*EXTI*/

#ifdef AFIO
	AFIO_TypeDef afio				__attribute__((at(AFIO_BASE)));
#endif /*AFIO*/

} // namespace gpio


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace timer {

#ifdef TIM1
	TIM_TypeDef tim1				__attribute__((at(TIM1_BASE)));
#endif /*TIM1*/

#ifdef TIM2
	TIM_TypeDef tim2				__attribute__((at(TIM2_BASE)));
#endif /*TIM2*/

#ifdef TIM3
	TIM_TypeDef tim3				__attribute__((at(TIM3_BASE)));
#endif /*TIM3*/

#ifdef TIM4
	TIM_TypeDef tim4				__attribute__((at(TIM4_BASE)));
#endif /*TIM4*/

#ifdef TIM5
	TIM_TypeDef tim5				__attribute__((at(TIM5_BASE)));
#endif /*TIM5*/

#ifdef TIM6
	TIM_TypeDef tim6				__attribute__((at(TIM6_BASE)));
#endif /*TIM6*/

#ifdef TIM7
	TIM_TypeDef tim7				__attribute__((at(TIM7_BASE)));
#endif /*TIM7*/

#ifdef TIM8
	TIM_TypeDef tim8				__attribute__((at(TIM8_BASE)));
#endif /*TIM8*/

#ifdef TIM9
	TIM_TypeDef tim9				__attribute__((at(TIM9_BASE)));
#endif /*TIM9*/

#ifdef TIM10
	TIM_TypeDef tim10				__attribute__((at(TIM10_BASE)));
#endif /*TIM10*/

#ifdef TIM11
	TIM_TypeDef tim11				__attribute__((at(TIM11_BASE)));
#endif /*TIM11*/

#ifdef TIM12
	TIM_TypeDef tim12				__attribute__((at(TIM12_BASE)));
#endif /*TIM12*/

#ifdef TIM13
	TIM_TypeDef tim13				__attribute__((at(TIM13_BASE)));
#endif /*TIM13*/

#ifdef TIM14
	TIM_TypeDef tim14				__attribute__((at(TIM14_BASE)));
#endif /*TIM14*/

#ifdef RTC
	RTC_TypeDef rtc					__attribute__((at(RTC_BASE)));
#endif /*RTC*/

} // namespace timer


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace uart {

#ifdef USART1
	USART_TypeDef usart1			__attribute__((at(USART1_BASE)));
#endif /*USART1*/

#ifdef USART2
	USART_TypeDef usart2			__attribute__((at(USART2_BASE)));
#endif /*USART2*/

#ifdef USART3
	USART_TypeDef usart3			__attribute__((at(USART3_BASE)));
#endif /*USART3*/
	
#ifdef UART4
	UART_TypeDef uart4				__attribute__((at(UART4_BASE)));
#endif /*UART4*/

#ifdef UART5
	UART_TypeDef uart5				__attribute__((at(UART5_BASE)));
#endif /*UART5*/

#ifdef USART6
	USART_TypeDef usart6			__attribute__((at(USART6_BASE)));
#endif /*USART6*/

} // namespace uart


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace spi {

#ifdef SPI1
	SPI_TypeDef spi1				__attribute__((at(SPI1_BASE)));
#endif /*SPI1*/

#ifdef SPI2
	SPI_TypeDef spi2				__attribute__((at(SPI2_BASE)));
#endif /*SPI2*/

#ifdef SPI3
	SPI_TypeDef spi3				__attribute__((at(SPI3_BASE)));
#endif /*SPI3*/

} // namespace spi


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace i2c {

#ifdef I2C1
	I2C_TypeDef i2c1				__attribute__((at(I2C1_BASE)));
#endif /*I2C1*/

#ifdef I2C2
	I2C_TypeDef i2c2				__attribute__((at(I2C2_BASE)));
#endif /*I2C2*/

#ifdef I2C3
	I2C_TypeDef i2c3				__attribute__((at(I2C3_BASE)));
#endif /*I2C3*/

} // namespace i2c


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace usb {

#ifdef USB
	USB_TypeDef usb					__attribute__((at(USB_BASE)));
#endif /*USB*/

} // namespace usb


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace can {

#ifdef CAN1
	CAN_TypeDef can1				__attribute__((at(CAN1_BASE)));
#endif /*CAN1*/

#ifdef CAN2
	CAN_TypeDef can2				__attribute__((at(CAN2_BASE)));
#endif /*CAN2*/

} // namespace can


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace sdio {

#ifdef SDIO
	SDIO_TypeDef sdio				__attribute__((at(SDIO_BASE)));
#endif /*SDIO*/

} // namespace sdio


/************************************************************************/
/*                                                                      */
/************************************************************************/
namespace dma {

#ifdef DMA1
	DMA_TypeDef dma1				__attribute__((at(DMA1_BASE)));
#endif /*DMA1*/

#ifdef DMA1_Channel1
	DMA_Channel_TypeDef dma1_ch1	__attribute__((at(DMA1_Channel1_BASE)));
#endif /*DMA1_Channel1*/

#ifdef DMA1_Channel2
	DMA_Channel_TypeDef dma1_ch2	__attribute__((at(DMA1_Channel2_BASE)));
#endif /*DMA1_Channel2*/

#ifdef DMA1_Channel3
	DMA_Channel_TypeDef dma1_ch3	__attribute__((at(DMA1_Channel3_BASE)));
#endif /*DMA1_Channel3*/

#ifdef DMA1_Channel4
	DMA_Channel_TypeDef dma1_ch4	__attribute__((at(DMA1_Channel4_BASE)));
#endif /*DMA1_Channel4*/

#ifdef DMA1_Channel5
	DMA_Channel_TypeDef dma1_ch5	__attribute__((at(DMA1_Channel5_BASE)));
#endif /*DMA1_Channel5*/

#ifdef DMA1_Channel6
	DMA_Channel_TypeDef dma1_ch6	__attribute__((at(DMA1_Channel6_BASE)));
#endif /*DMA1_Channel6*/

#ifdef DMA1_Channel7
	DMA_Channel_TypeDef dma1_ch7	__attribute__((at(DMA1_Channel7_BASE)));
#endif /*DMA1_Channel7*/

#ifdef DMA1_Channel8
	DMA_Channel_TypeDef dma1_ch8	__attribute__((at(DMA1_Channel8_BASE)));
#endif /*DMA1_Channel8*/


#ifdef DMA2
	DMA_TypeDef dma2				__attribute__((at(DMA2_BASE)));
#endif /*DMA2*/

#ifdef DMA2_Channel1
	DMA_Channel_TypeDef dma2_ch1	__attribute__((at(DMA2_Channel1_BASE)));
#endif /*DMA2_Channel1*/

#ifdef DMA2_Channel2
	DMA_Channel_TypeDef dma2_ch2	__attribute__((at(DMA2_Channel2_BASE)));
#endif /*DMA2_Channel2*/

#ifdef DMA2_Channel3
	DMA_Channel_TypeDef dma2_ch3	__attribute__((at(DMA2_Channel3_BASE)));
#endif /*DMA2_Channel3*/

#ifdef DMA2_Channel4
	DMA_Channel_TypeDef dma2_ch4	__attribute__((at(DMA2_Channel4_BASE)));
#endif /*DMA2_Channel4*/

#ifdef DMA2_Channel5
	DMA_Channel_TypeDef dma2_ch5	__attribute__((at(DMA2_Channel5_BASE)));
#endif /*DMA2_Channel5*/

#ifdef DMA2_Channel6
	DMA_Channel_TypeDef dma2_ch6	__attribute__((at(DMA2_Channel6_BASE)));
#endif /*DMA2_Channel6*/

#ifdef DMA2_Channel7
	DMA_Channel_TypeDef dma2_ch7	__attribute__((at(DMA2_Channel7_BASE)));
#endif /*DMA2_Channel7*/

#ifdef DMA2_Channel8
	DMA_Channel_TypeDef dma2_ch8	__attribute__((at(DMA2_Channel8_BASE)));
#endif /*DMA2_Channel8*/

} // namespace dma


template<uint32_t addr, uint32_t bit>
class from_address
{
public:
	static const bool is_sram	= addr >= SRAM_BASE && (addr - SRAM_BASE) < 0x80000;
	static const bool is_periph	= addr >= PERIPH_BASE && (addr - PERIPH_BASE) < 0x80000;

	STATIC_ASSERT(is_sram || is_periph, "The address cannot be matched to BIT BANDING range");
	STATIC_ASSERT(bit < 32, "Invalid bit offset");

	static const uint32_t to_bb	= is_sram ? SRAM_BB_BASE + (addr - SRAM_BASE) * 32 + (bit) * 4 :
											PERIPH_BB_BASE + (addr - PERIPH_BASE) * 32 + (bit) * 4;
};

#define FROM_ADDRESS_BIT_POS_TO_BB(member_ptr, bit)		::mcu::registers::from_address<(uint32_t)(member_ptr), bit>::to_bb
//#define FROM_ADDRESS_BIT_MASK_TO_BB(member_ptr, bit)		::mcu::registers::from_address<(uint32_t)(member_ptr), POSITION_VAL(bit)>::to_bb

#define WRITE_BB_REG(address, value)			(*(__IO uint32_t *)(address) = value)
#define READ_BB_REG(address)					(*(__IO uint32_t *)(address))

#define SET_BB_REG(address)						WRITE_BB_REG(address, !RESET)
#define RESET_BB_REG(address)					WRITE_BB_REG(address, RESET)

#define IS_BB_REG_SET(address)					(READ_BB_REG(address) != RESET)
#define IS_BB_REG_RESET(address)				(READ_BB_REG(address) == RESET)

} // namespace registers
} // namespace mcu
//////////////////////////////////////////////////////////////////////////
#endif /*__stm32f1xx_registers_hpp__*/
