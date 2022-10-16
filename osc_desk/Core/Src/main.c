/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "gpio.h"
#include "stm32f1xx.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define DATA_SIZE			512

uint8_t flag_redy;

uint16_t adc_data[DATA_SIZE];
uint8_t adc_data_byte[DATA_SIZE * 2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
typedef enum {
	gpio_mode_output_PP_2MHz = 2, /**< Push-Pull 2MHz */
	gpio_mode_output_PP_10MHz = 1, /**< Push-Pull 10MHz */
	gpio_mode_output_PP_50MHz = 3, /**< Push-Pull 50MHz */
	gpio_mode_output_OD_2MHz = 6, /**< Open-Drain 2MHz */
	gpio_mode_output_OD_10MHz = 5, /**< Open-Drain 10MHz */
	gpio_mode_output_OD_50MHz = 7, /**< Open-Drain 50MHz */
	gpio_mode_alternate_PP_2MHz = 10, /**< Alternate Push-Pull 2MHz */
	gpio_mode_alternate_PP_10MHz = 9, /**< Alternate Push-Pull 10MHz */
	gpio_mode_alternate_PP_50MHz = 11, /**< Alternate Push-Pull 50MHz */
	gpio_mode_alternate_OD_2MHz = 14, /**< Alternate Open-Drain 2MHz */
	gpio_mode_alternate_OD_10MHz = 13, /**< Alternate Open-Drain 10MHz */
	gpio_mode_alternate_OD_50MHz = 15, /**< Alternate Open-Drain 50MHz */
	gpio_mode_input_analog = 0, /**< Analog input */
	gpio_mode_input_floating = 4, /**< Floating input (Hi-Z) */
	gpio_mode_input_pupd = 8 /**< Digital input with pull-up/down */
} gpio_mode_t;
static void gpio_pin_mode(GPIO_TypeDef *gpio, uint8_t pin, gpio_mode_t mode) {
	pin *= 4;

	if (pin < 32) {
		gpio->CRL &= ~((uint32_t) (0x0f << pin));
		gpio->CRL |= (uint32_t) (mode << pin);
	} else {
		pin -= 32;
		gpio->CRH &= ~((uint32_t) (0x0f << pin));
		gpio->CRH |= (uint32_t) (mode << pin);
	}
}
static void Gpio_Init(void) {
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

	gpio_pin_mode(GPIOA, 0, gpio_mode_input_analog);		//ADC
	gpio_pin_mode(GPIOC, 13, gpio_mode_output_PP_50MHz);		//LED

}
static void adc_init(void) {

	Gpio_Init();

	RCC->AHBENR |= RCC_AHBENR_DMA1EN;			// Включаем тактирование DMA1
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// Deinit DMA1 Channel1
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;		// Отключаем DMA1 CH1
	//DMA1_Channel1->CCR = 0;								// Reset DMA1 Channel1 control register
	DMA1->IFCR |= DMA_ISR_GIF1 | DMA_ISR_TCIF1 | DMA_ISR_HTIF1 | DMA_ISR_TEIF1; // Reset interrupt pending bits for DMA1 Channel1

	// Настраиваем DMA
	DMA1_Channel1->CNDTR = DATA_SIZE;		// Количество передаваемых данных
	DMA1_Channel1->CPAR = (uint32_t) &(ADC1->DR);			// Адрес перифирии
	DMA1_Channel1->CMAR = (uint32_t) adc_data;		// Адрес памяти
	DMA1_Channel1->CCR |= DMA_CCR_MINC					// Инкремент памяти
	| DMA_CCR_PSIZE_0				// Режим периферии 16 бит
			| DMA_CCR_MSIZE_0				// Режим памяти 16 бит
			| DMA_CCR_PL						// Channel Priority level hight
			| DMA_CCR_CIRC						// Circular mode
			| DMA_CCR_TEIE				// Transfer error interrupt enable
			| DMA_CCR_TCIE;				// Transfer Complite interrupt enable
	DMA1_Channel1->CCR |= DMA_CCR_EN;

	// Очистим настройки ADC1
	ADC1->CR1 = 0;
	ADC1->CR2 = 0;
	ADC1->SMPR2 = 0;
	ADC1->SQR1 = 0;
	ADC1->SQR2 = 0;
	ADC1->SQR3 = 0;

	//ADC1->CR1 |= ADC_CR1_SCAN;		// Scan mode
	ADC1->CR2 |= ADC_CR2_CONT;		// Continuous Conversion
	ADC1->CR2 |= ADC_CR2_DMA;			// DMA mode
	ADC1->CR1 |= ADC_CR1_EOCIE;

	ADC1->SQR1 |= ((uint32_t) (0) << 20);

	ADC1->SMPR2 &= ~ADC_SMPR2_SMP0;	// Channel 0 Sample time selection 111: 239.5 cycles
	ADC1->SQR3 |= (0x00 << (5 * 0));			// Pin0

	ADC1->CR2 |= ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL;		//ADC_CR2_SWSTART;
	ADC1->CR2 |= ADC_CR2_DMA;				// DMA mode

	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	__enable_irq();
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DMA1_Channel1_IRQHandler(void) {

	if ((DMA1->ISR & DMA_ISR_TCIF1)) {
		GPIOC->BSRR |= GPIO_BSRR_BR13;

		// Transfer Complete flag
		DMA1_Channel1->CCR &= ~DMA_CCR_EN;				// Отключаем DMA.
		DMA1->IFCR |= DMA_IFCR_CTCIF1;		// Channel1 Transfer Complete clear

		for (uint16_t x = 0; x < DATA_SIZE * 2; x++)
			adc_data_byte[x] = ((uint8_t*)&adc_data)[x];

		CDC_Transmit_FS((uint8_t*) adc_data_byte, DATA_SIZE * 2);

		flag_redy = 1;

	}
}
extern uint8_t recive_data[100];

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */

	adc_init();

	GPIOC->BSRR |= GPIO_BSRR_BS13;



	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		for(uint8_t x = 0; x < 10; x ++)
		{
			if(recive_data[x] == 0x10)
			{
				ADC1->CR2 |= ADC_CR2_ADON;
				ADC1->CR2 |= ADC_CR2_SWSTART;
				DMA1_Channel1->CCR |= DMA_CCR_TCIE | DMA_CCR_EN;

				GPIOC->BSRR |= GPIO_BSRR_BS13;
				recive_data[0] = 0;
			}
		}

		//if (recive_data[0] == 0x10) {
			//HAL_Delay(30);


		//}

		if(flag_redy)
		{
			flag_redy = 0;





		}

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
