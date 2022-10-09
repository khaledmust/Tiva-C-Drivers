#include "tm4c123xxx.h"
#include "gpio_driver.h"

//0x4005D000

#define SYSCTL_RCGCGPIO_R (*(( volatile unsigned long *)0x400FE608 ) )
#define GPIO_PORTF_DATA_R (*(( volatile unsigned long *)0x40025038 ) )
#define GPIO_PORTF_DIR_R  (*(( volatile unsigned long *)0x4005D400 ) )
#define GPIO_PORTF_DEN_R  (*(( volatile unsigned long *)0x4005D51C ) )

int main (void)
{
	GPIO_ClockCtrl(GPIOF, ENABLE);

	GPIO_PinConfig_t user_config = {
			.GPIO_PinNum = GPIO_PIN_2,
			.GPIO_PinDirection = GPIO_DIR_OUTPUT,
//			.GPIO_CurrentStrenght = GPIO_DR2R,
			.GPIO_PinType = GPIO_PU
	};

//	GPIO_PinConfig_t user_config2 = {
//				.GPIO_PinNum = GPIO_PIN_3,
//				.GPIO_PinDirection = GPIO_DIR_OUTPUT,
//				.GPIO_CurrentStrenght = GPIO_DR2R,
//				.GPIO_PinType = GPIO_PU
//		};

	GPIO_Handle_t user_port = {
			.pGPIOx = GPIOF,
			.pGPIO_PinConfig = &user_config
	};

//	GPIO_Handle_t user_port2 = {
//				.pGPIOx = GPIOF,
//				.pGPIO_PinConfig = &user_config2
//		};

	/* Testing the the direction and digital enable of Port F. */
//	GPIO_PORTF_DEN_R  = 0x0E;  // Enable PORTF Pin1, 2 and 3 as a digital pins
//	GPIO_PORTF_DIR_R  = 0x0E;  // Configure ORTF Pin1, 2 and 3 digital output pins

	GPIO_Init(&user_port);

	while (1)
	{

//		GPIO_WritePort(&user_port, PORT_HIGH);
//
		GPIO_TogglePin(GPIOF, GPIO_PIN_2);
//
		GPIO_TogglePin(GPIOF, GPIO_PIN_2);
//
//		GPIO_TogglePin(GPIOF, GPIO_PIN_3);
//
//		GPIO_TogglePin(GPIOF, GPIO_PIN_3);
		/* Setting the blue LED. */
//		*( (uint32_t *) (0x4005D010) ) = 0xFF;
//		GPIO_WritePin(&user_port, PIN_LOW);
//		GPIO_WritePin(&user_port, PIN_HIGH);
	}
	return 0;
}
