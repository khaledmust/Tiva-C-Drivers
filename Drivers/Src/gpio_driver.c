/*
 * gpio_driver.c
 *
 *  Created on: Sep 5, 2022
 *      Author: Khaled Mustafa
 */
#include "../Drivers/Inc/gpio_driver.h"

/************************************************
 *  @brief Enables or disables the clock to the selected GPIO port,
 *  if GPIO_AHP_BUS_STATUS is enabled it provides clock access to the AHP bus.
 *  The function works by first checking the input ClkState followed by checking of
 *  the input base address to determine which port requires the enable or disable of
 *  the clock.
 *  @param pGPIOx Base address to the port to be controlled.
 *  @param ClkState The state of the clock either enable or disable.
 *  @return None
 ***********************************************/
void GPIO_ClockCtrl (GPIO_RegDef_t *pGPIOx, uint8_t ClkState)
{
	if (ClkState == ENABLE)
	{
		/* Enable the clock access to the selected GPIO. */
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN;

#if (GPIO_AHP_BUS_STATUS == ENABLE)
			GPIOA_AHP_BUS_ENABLE;
#endif

		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN;

#if (GPIO_AHP_BUS_STATUS == ENABLE)
			GPIOB_AHP_BUS_ENABLE;
#endif

		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN;

#if (GPIO_AHP_BUS_STATUS == ENABLE)
			GPIOC_AHP_BUS_ENABLE;
#endif

		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN;

#if (GPIO_AHP_BUS_STATUS == ENABLE)
			GPIOD_AHP_BUS_ENABLE;
#endif

		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN;

#if (GPIO_AHP_BUS_STATUS == ENABLE)
			GPIOE_AHP_BUS_ENABLE;
#endif

		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN;

#if (GPIO_AHP_BUS_STATUS == ENABLE)
			GPIOF_AHP_BUS_ENABLE;
#endif

		}
	}
	else if (ClkState == DISABLE)
	{
		/* Disable the clock access to the selected GPIO. */
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DISABLE;
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DISABLE;
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DISABLE;
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DISABLE;
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DISABLE;
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DISABLE;
		}
	}
}

/************************************************
 *  @brief Initialize the GPIO pin with the user configuration.
 *  The initialization function performs the following:
 *  1. Set the direction of the GPIO port pins.
 *  2. Configure the GPIOAFSEL register to program each bit as a GPIO or alternate pin.
 *  3. Set the drive strength for each of the pins.
 *  4. Program each pad in the port to have either pull-up, pull-down, or open drain functionality.
 *  5. To enable GPIO pins as digital I/O, set the appropriate DEN.
 *  @param pGPIOHandle Address to the GPIO handle containing the base address of the GPIO pin to be configured and user configurations.
 *  @return None
 ***********************************************/
void GPIO_Init (GPIO_Handle_t *pGPIOHandle)
{
	/* 1. Set the direction of the selected pin. */
	if (pGPIOHandle -> pGPIO_PinConfig -> GPIO_PinDirection == GPIO_DIR_INPUT)
	{
		pGPIOHandle -> pGPIOx -> DIR &= ~ ( pGPIOHandle -> pGPIO_PinConfig -> GPIO_PinNum );
	}
	else if (pGPIOHandle -> pGPIO_PinConfig -> GPIO_PinDirection == GPIO_DIR_OUTPUT)
	{
		pGPIOHandle -> pGPIOx -> DIR |= ( pGPIOHandle -> pGPIO_PinConfig -> GPIO_PinNum );
	}

	/* 2. Configure the function of the selected pin. */
	if (pGPIOHandle -> pGPIO_PinConfig -> GPIO_AlternateFunction == 1)
	{
		GPIO_Unlock(pGPIOHandle);

		pGPIOHandle -> pGPIOx -> AFSEL = 1;

		GPIO_Lock(pGPIOHandle);
	}
	else
	{
		/* The state of the pin is GPIO by default. */
	}

	/* 3. Set the drive strength for the selected pin if the pin is configured as output. */
	if (pGPIOHandle -> pGPIO_PinConfig -> GPIO_PinDirection == GPIO_DIR_OUTPUT)
	{
		if (pGPIOHandle -> pGPIO_PinConfig -> GPIO_CurrentStrenght == GPIO_DR2R)
		{
			/* Set the 2mA register for the selected pins. */
			pGPIOHandle -> pGPIOx -> DR2R = pGPIOHandle -> pGPIO_PinConfig -> GPIO_PinNum;
		}
		else if (pGPIOHandle -> pGPIO_PinConfig -> GPIO_CurrentStrenght == GPIO_DR4R)
		{
			/* Set the 4mA register for the selected pins. */
			pGPIOHandle -> pGPIOx -> DR4R = pGPIOHandle -> pGPIO_PinConfig -> GPIO_PinNum;
		}
		else if (pGPIOHandle -> pGPIO_PinConfig -> GPIO_CurrentStrenght == GPIO_DR84)
		{
			/* Set the 8mA register for the selected pins. */
			pGPIOHandle -> pGPIOx -> DR8R = pGPIOHandle -> pGPIO_PinConfig -> GPIO_PinNum;
		}
	}

	/* 4. Set the attachment type of the selected pin. */
	switch (pGPIOHandle -> pGPIO_PinConfig -> GPIO_PinType)
	{	/* Note that, if a pin is configured as an input pin the OD configuration will have no effect. */
		case GPIO_ODR:
			pGPIOHandle -> pGPIOx -> ODR = ( pGPIOHandle -> pGPIO_PinConfig -> GPIO_PinNum );
			break;

		case GPIO_PU:
			GPIO_Unlock(pGPIOHandle);
			pGPIOHandle -> pGPIOx -> PUR = pGPIOHandle -> pGPIO_PinConfig -> GPIO_PinNum;
			GPIO_Lock(pGPIOHandle);
			break;

		case GPIO_PD:
			GPIO_Unlock(pGPIOHandle);
			pGPIOHandle -> pGPIOx -> PDR = pGPIOHandle -> pGPIO_PinConfig -> GPIO_PinNum;
			GPIO_Lock(pGPIOHandle);
			break;

		default:
			break;
	}

	/* 5. Enable the digital functionality of the selected pin. */
	GPIO_Unlock(pGPIOHandle);

	pGPIOHandle -> pGPIOx -> DEN |= ( pGPIOHandle -> pGPIO_PinConfig -> GPIO_PinNum);

	GPIO_Lock(pGPIOHandle);
}

/************************************************
 *  @brief De-initialize the GPIO pin.
 *  De-initialization is done by setting the value of a 6-bit register corresponding to the required port.
 *  @see Page 314 of the Tiva-C launchpad documentation.
 *  @param pGPIOHandle Address to the GPIO handle containing the base address of the GPIO pin to be configured and user configurations.
 *  @return None
 ***********************************************/
void GPIO_DeInit (GPIO_Handle_t *pGPIOHandle)
{

}

/************************************************
 *  @brief Read the value of a bit corresponding to the pin selected.
 *  @param pGPIOx Base address to the port to be controlled.
 *  @param PinNum Pin number to be read from.
 *  @return Value of the pin read.
 ***********************************************/
uint8_t GPIO_ReadPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
	/* Calculate the value of the mask. */
	uint8_t mask = PinNum;

	/* Calculate the address of the mask. */
	uint8_t maskOffset = mask << 2;

	return ( pGPIOx -> MASK [maskOffset] );
}

/************************************************
 * 	@fn GPIO_ReadPort
 *  @brief Read a value of a 8-bit corresponding to the port selected.
 *  @param pGPIOx Base address to the port to be controlled.
 *  @return Value of the port read.
 ***********************************************/
uint8_t GPIO_ReadPort (GPIO_RegDef_t *pGPIOx)
{

}

/************************************************
 *  @brief Write the binary value in the data register to the pin selected.
 *  Tiva C provides modification of individual bits by using bits [9:2] of the address bus as a mask, thus providing 256 locations that are
 *  memory mapped (all the possible combinations of the 8-bits data register) so to write to any any bit we must calculate the corresponding
 *  mask offset and this add it to the base address.
 *  This is done by first the mask itself by 1 << pin number to be configured.
 *  Then shifting this mask by 2 as we only use the bits from 9:2 of the 10 bits.
 *  Then add this offset to the base address, but since I am pointing to a structure I can access the memory location via the mask structure
 *  defined in that structure.
 *  @param pGPIOHandle Address to the GPIO handle containing the base address of the GPIO pin to be configured and user configurations.
 *  @param PinValue Value to be written on the pin.
 *  @return None
 ***********************************************/
void GPIO_WritePin (GPIO_Handle_t *pGPIOHandle, uint8_t PinValue)
{
	/* Calculate the value of the mask. */
	uint8_t mask = ( pGPIOHandle -> pGPIO_PinConfig -> GPIO_PinNum );

	/* Calculate the address of the mask. */
	uint8_t maskOffset = mask << 2;

	pGPIOHandle -> pGPIOx -> MASK [maskOffset] = PinValue;
}

/************************************************
 *  @brief Write an 8-bit value in the data register of the selected port.
 *  @param pGPIOx Base address to the port to be controlled.
 *  @param PortValue 8-bit value to be written on the whole port.
 *  @return None
 ***********************************************/
void GPIO_WritePort (GPIO_Handle_t *pGPIOHandle, uint8_t PortValue)
{
	// I will write on index 1020
	pGPIOHandle -> pGPIOx -> MASK [1023] = PortValue;
}

/************************************************
 *  @brief Toggle the state of a pin.
 *  The functionality of this function is similar to that of GPIO_WritePin.
 *  @param pGPIOx Base address to the port to be controlled.
 *  @param PinNum Pin number to be toggled.
 *  @return None
 ***********************************************/
void GPIO_TogglePin (GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
	/* Calculate the value of the mask. */
	uint8_t mask = PinNum;

	/* Calculate the address of the mask. */
	uint8_t maskOffset = mask << 2;

	pGPIOx -> MASK [maskOffset] ^= PinNum;
}

/************************************************
 *  @brief Locks the AFSEL, PUR, PDR, and DEN register from saving any changes.
 *  This is achieved by writing GPIO_LOCK value into the LOCK register of the GPIO.
 *  @param pGPIOHandle Address to the GPIO handle containing the base address of the GPIO pin to be configured, and the user configurations.
 *  @return None
 ***********************************************/
static void GPIO_Lock (GPIO_Handle_t *pGPIOHandle)
{
	pGPIOHandle -> pGPIOx -> LOCK = GPIO_LOCK;
}

/************************************************
 *  @brief Unlocks the AFSEL, PUR, PDR, and DEN register to allow saving any changes.
 *  This is achieved by writing GPIO_UNLOCK value into the LOCK register of the GPIO.
 *  @param pGPIOHandle Address to the GPIO handle containing the base address of the GPIO pin to be configured, and the user configurations.
 *  @return None
 ***********************************************/
static void GPIO_Unlock (GPIO_Handle_t *pGPIOHandle)
{
	/* Enable the write access to the commit register. */
	pGPIOHandle -> pGPIOx -> LOCK = GPIO_UNLOCK;

	/* Commit the change done to either the AFSEL, PUR, PUD, or DEN registers. */
	pGPIOHandle -> pGPIOx -> CR |= ( pGPIOHandle -> pGPIO_PinConfig -> GPIO_PinNum );
}
