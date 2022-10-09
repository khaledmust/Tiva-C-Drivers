/*
 * tm4c123xxx.h
 */

#ifndef DRIVERS_INC_TM4C123XXX_H_
#define DRIVERS_INC_TM4C123XXX_H_

#include <stdint.h>

#define ENABLE	1
#define DISABLE	0

/** @defgroup group1 Memory blocks Base Address
 *  @{
 */
#define FLASH_BASEADDR			0x00000000U
#define SRAM_BASEADDR			0x20000000U
#define ROM_BASEADDR			0x400AF000U
/** @} */


/** @defgroup group2 APB and AHB bus peripheral base address
 *  @{
 */
#define APB_PERIPH_BASEADDR		0x40000000U
#define AHB_PERIPH_BASEADDR		0x40020000U
/** @} */

/** @defgroup  group3 Base address of peripherals which are connected to AHB bus.
 *  @{
 */
#define GPIOA_BASEADDR			0x40058000U
#define GPIOB_BASEADDR			0x40059000U
#define GPIOC_BASEADDR			0x4005A000U
#define GPIOD_BASEADDR			0x4005B000U
#define GPIOE_BASEADDR			0x4005C000U
#define GPIOF_BASEADDR			0x4005D000U
/** @} */

/** @defgroup  group4 Base address of peripherals which are connected to APB bus.
 *  @{
 */
#define I2C0_BASEADDR			0x40020000U
#define I2C1_BASEADDR			0x40021000U
#define I2C2_BASEADDR			0x40022000U
#define I2C3_BASEADDR			0x40023000U

#define SSI0_BASEADDR			0x40008000U
#define SSI1_BASEADDR			0x40009000U
#define SSI2_BASEADDR			0x4000A000U
#define SSI3_BASEADDR			0x4000B000U

#define UART0_BASEADDR			0x4000C000U
#define UART1_BASEADDR			0x4000D000U
#define UART2_BASEADDR			0x4000E000U
#define UART3_BASEADDR			0x4000F000U
#define UART4_BASEADDR			0x40010000U
#define UART5_BASEADDR			0x40011000U
#define UART6_BASEADDR			0x40012000U
#define UART7_BASEADDR			0x40013000U

#define SYS_CTRL_BASEADDR		0x400FE000U
#define SYS_GPIO_AHB_CTRL		0x400FE06CU
#define SYS_GPIO_CLK_CTRL		0x400FE608U

/** @} */

/************************************************
 *  GPIO peripheral registers definition structure.
 ***********************************************/
typedef struct
{
//	volatile uint32_t DATA_BITS;		/**< Data registers.					Address offset: 0x000 */
	volatile uint8_t MASK[1024];		/**< Masks.								Address offset: */
	volatile uint32_t DIR;				/**< Direction register.				Address offset: 0x400 */
	volatile uint32_t IS;				/**< Interrupt sense register.			Address offset: 0x404 */
	volatile uint32_t IBE;				/**< Interrupt both edges register.		Address offset: 0x408 */
	volatile uint32_t IEV;				/**< Interrupt event register.			Address offset: 0x40C */
	volatile uint32_t IM;				/**< Interrupt mask register.			Address offset: 0x410 */
	volatile uint32_t RIS;				/**< Raw interrupt status register.		Address offset: 0x414 */
	volatile uint32_t MIS;				/**< Masked interrupt status register.	Address offset: 0x418 */
	volatile uint32_t ICR;				/**< Interrupt clear register.			Address offset: 0x41C */
	volatile uint32_t AFSEL;			/**< Alternate function selection.		Address offset: 0x420 */
	volatile uint32_t RESERVED [55];	/**< RESERVED. */
	volatile uint32_t DR2R;				/**< 2-mA drive select register.		Address offset: 0x500 */
	volatile uint32_t DR4R;				/**< 4-mA drive select register.		Address offset: 0x504 */
	volatile uint32_t DR8R;				/**< 8-mA drive select register.		Address offset: 0x508 */
	volatile uint32_t ODR;				/**< Open drain select register.		Address offset: 0x50C */
	volatile uint32_t PUR;				/**< Pull-up select register.			Address offset: 0x510 */
	volatile uint32_t PDR;				/**< Pull-down select register.			Address offset: 0x514 */
	volatile uint32_t SLR;				/**< Slew rate control select register.	Address offset: 0x518 */
	volatile uint32_t DEN;				/**< Digital enable register.			Address offset: 0x51C */
	volatile uint32_t LOCK;				/**< Lock register.						Address offset: 0x520 */
	volatile uint32_t CR;				/**< Commit register.					Address offset: 0x524 */
	volatile uint32_t AMSEL;			/**< Analog mode select register.		Address offset: 0x528 */
	volatile uint32_t PCTL;				/**< Port control register.				Address offset: 0x52C */
	volatile uint32_t ADCCTL;			/**< ADC control register.				Address offset: 0x530 */
	volatile uint32_t DMACTL;			/**< DMA control register.				Address offset: 0x534 */
}GPIO_RegDef_t;

/************************************************
 *  RCC peripheral registers definition structure.
 ***********************************************/
//typedef struct
//{
//	volatile uint32_t DID0; 			/**< Device Identification 0. 										Address offset: 0x000 */
//	volatile uint32_t DID1;				/**< Device Identification 1. 										Address offset: 0x004 */
//	volatile uint32_t RESERVED1 [10]; 	/**< RESERVED. */
//	volatile uint32_t BORCTL; 			/**< Brown-Out Reset Control. 										Address offset: 0x030 */
//	volatile uint32_t RESERVED2 [7];		/**< RESERVED. */
//	volatile uint32_t RIS;				/**< Raw Interrupt Status. 											Address offset: 0x050 */
//	volatile uint32_t IMC;				/**< Interrupt Mask Control. 										Address offset: 0x054 */
//	volatile uint32_t MISC;				/**< Masked Interrupt Status and Clear. 							Address offset: 0x058 */
//	volatile uint32_t RESC;				/**< Reset Cause. 													Address offset: 0x05C */
//	volatile uint32_t RCC;				/**< Run-Mode Clock Configuration. 									Address offset: 0x060 */
//	volatile uint32_t RESERVED3 [2];
//	volatile uint32_t GPIOHBCTL;		/**< GPIO High-Performance Bus Control. 							Address offset: 0x06C */
//	volatile uint32_t RESERVED4 [166];	/**< RESERVED. */
//	volatile uint32_t WD;				/**< Watchdog timer run mode clock gating control.					Address offset: 0x600 */
//	volatile uint32_t TIMER;			/**< General-purpose timer run mode clock gating control.			Address offset: 0x604 */
//	volatile uint32_t GPIO;				/**< General-purpose input/output run mode clock gating control.	Address offset: 0x608 */
//	volatile uint32_t DMA;				/**< Direct memory access run mode clock gating control. 			Address offset: 0x60C */
//	volatile uint32_t RESERVED5;		/**< Reserved. */
//	volatile uint32_t HIB;				/**< Hibernation run mode clock gating control.						Address offset: 0x614 */
//	volatile uint32_t UART;				/**< UART run mode clock gating control.  							Address offset: 0x618 */
//	volatile uint32_t SSI;				/**< SSI run mode clock gating control.  							Address offset: 0x61C */
//	volatile uint32_t I2C;				/**< I2C run mode clock gating control.  							Address offset: 0x620 */
//	uint32_t RESERVED6;					/**< Reserved. */
//	volatile uint32_t USB;				/**< USB run mode clock gating control.  							Address offset: 0x628 */
//	uint32_t RESERVED7;					/**< Reserved. */
//	uint32_t RESERVED8;					/**< Reserved. */
//	volatile uint32_t CAN;				/**< CAN run mode clock gating control.  							Address offset: 0x634 */
//	volatile uint32_t ADC;				/**< ADC run mode clock gating control.  							Address offset: 0x638 */
//	volatile uint32_t ACMP;				/**< Analog comparator run mode clock gating control. 				Address offset: 0x63C */
//	volatile uint32_t PWM;				/**< PWM run mode clock gating control.								Address offset: 0x640 */
//	volatile uint32_t QEI;				/**< Quadrature encoder interface run mode clock gating control.	Address offset: 0x644 */
//	uint32_t RESERVED9;					/**< Reserved. */
//	uint32_t RESERVED10;					/**< Reserved. */
//	uint32_t RESERVED11;					/**< Reserved. */
//	uint32_t RESERVED12;					/**< Reserved. */
//	volatile uint32_t EEPROM;			/**< EEPROM run mode clock gating control. 							Address offset: 0x658 */
//	volatile uint32_t WTIMER;			/**< Wide timer run mode clock gating control.						Address offset: 0x65C */
//}SYS_CTRL_RegDef_t;

/************************************************
 *  Peripheral definitions.
 ***********************************************/
#define GPIOA		((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t *) GPIOF_BASEADDR)

/************************************************
 *  Clock enable Macros for GPIOx peripherals.
 ***********************************************/
#define GPIOA_PCLK_EN	*( (uint32_t *) (SYS_GPIO_CLK_CTRL) ) |= (1 << 0)
#define GPIOB_PCLK_EN	*( (uint32_t *) (SYS_GPIO_CLK_CTRL) ) |= (1 << 1)
#define GPIOC_PCLK_EN	*( (uint32_t *) (SYS_GPIO_CLK_CTRL) ) |= (1 << 2)
#define GPIOD_PCLK_EN	*( (uint32_t *) (SYS_GPIO_CLK_CTRL) ) |= (1 << 3)
#define GPIOE_PCLK_EN	*( (uint32_t *) (SYS_GPIO_CLK_CTRL) ) |= (1 << 4)
#define GPIOF_PCLK_EN	*( (uint32_t *) (SYS_GPIO_CLK_CTRL) ) |= (1 << 5)

/************************************************
 *  Clock disable Macros for GPIOx peripherals.
 ***********************************************/
#define GPIOA_PCLK_DISABLE	*( (uint32_t *) (SYS_GPIO_CLK_CTRL) ) &= ~(1 << 0)
#define GPIOB_PCLK_DISABLE	*( (uint32_t *) (SYS_GPIO_CLK_CTRL) ) &= ~(1 << 1)
#define GPIOC_PCLK_DISABLE	*( (uint32_t *) (SYS_GPIO_CLK_CTRL) ) &= ~(1 << 2)
#define GPIOD_PCLK_DISABLE	*( (uint32_t *) (SYS_GPIO_CLK_CTRL) ) &= ~(1 << 3)
#define GPIOE_PCLK_DISABLE	*( (uint32_t *) (SYS_GPIO_CLK_CTRL) ) &= ~(1 << 4)
#define GPIOF_PCLK_DISABLE	*( (uint32_t *) (SYS_GPIO_CLK_CTRL) ) &= ~(1 << 5)

/************************************************
 *  Clock enable Macros for AHP bus.
 ***********************************************/
#define GPIOA_AHP_BUS_ENABLE	*( (uint32_t *) (SYS_GPIO_AHB_CTRL) ) |= (1 << 0)
#define GPIOB_AHP_BUS_ENABLE	*( (uint32_t *) (SYS_GPIO_AHB_CTRL) ) |= (1 << 1)
#define GPIOC_AHP_BUS_ENABLE	*( (uint32_t *) (SYS_GPIO_AHB_CTRL) ) |= (1 << 2)
#define GPIOD_AHP_BUS_ENABLE	*( (uint32_t *) (SYS_GPIO_AHB_CTRL) ) |= (1 << 3)
#define GPIOE_AHP_BUS_ENABLE	*( (uint32_t *) (SYS_GPIO_AHB_CTRL) ) |= (1 << 4)
#define GPIOF_AHP_BUS_ENABLE	*( (uint32_t *) (SYS_GPIO_AHB_CTRL) ) |= (1 << 5)

/************************************************
 *  Clock disable Macros for AHP bus.
 ***********************************************/
#define GPIOA_AHP_BUS_DISABLE	*( (uint32_t *) (SYS_GPIO_AHB_CTRL) ) &= ~(1 << 0)
#define GPIOB_AHP_BUS_DISABLE	*( (uint32_t *) (SYS_GPIO_AHB_CTRL) ) &= ~(1 << 1)
#define GPIOC_AHP_BUS_DISABLE	*( (uint32_t *) (SYS_GPIO_AHB_CTRL) ) &= ~(1 << 2)
#define GPIOD_AHP_BUS_DISABLE	*( (uint32_t *) (SYS_GPIO_AHB_CTRL) ) &= ~(1 << 3)
#define GPIOE_AHP_BUS_DISABLE	*( (uint32_t *) (SYS_GPIO_AHB_CTRL) ) &= ~(1 << 4)
#define GPIOF_AHP_BUS_DISABLE	*( (uint32_t *) (SYS_GPIO_AHB_CTRL) ) &= ~(1 << 5)


#endif /* DRIVERS_INC_TM4C123XXX_H_ */
