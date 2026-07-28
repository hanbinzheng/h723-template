/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "fdcan.h"
#include "gpio.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis.h"

#include "bsp_dwt.h"
#include "bsp_fdcan.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_usart.h"

#include "buzzer.h"
#include "dbus.h"
#include "f103.h"
#include "motor.h"
#include "sbus.h"
#include "ws2812.h"

#include "log.h"
#include "vofa.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MCU_MAIN_FREQ 550 /* MHz */
#define APB1_FREQ 275	  /* MHz */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct spi_inst *spi6 = NULL;
struct tim_inst *tim5 = NULL;
struct tim_inst *tim12 = NULL;
struct usart_inst *usart5 = NULL;
struct gpio_inst *gpio_pc15 = NULL;
const struct sbus_data *sbus = NULL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void bsp_init(void);
void device_init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void usart5_callback(uint8_t *rx_buff, uint16_t len)
{
	if (len == SBUS_FRAME_LENGTH) {
		sbus_update(rx_buff);

		/* if get data and data is safe, feed the dog */
		if (sbus_get_data()->safe == SBUS_SAFE) {
			HAL_IWDG_Refresh(&hiwdg1);
		}
	}
}

void task()
{
	chassis_task();
	motor_set_command();
}

void bsp_init()
{
	/* dwt config */
	dwt_init(MCU_MAIN_FREQ);

	/* spi config */
	struct spi_config spi6_config = {
	    .hspi = &hspi6,
	    .cs_port = NULL,
	    .cs_pin = 0,
	};
	spi6 = spi_register(&spi6_config);

	/* tim config */
	struct tim_config tim5_config = {
	    .htim = &htim5,
	    .mode = TIM_INTERRUPT_MODE,
	    .callback = task,
	};
	struct tim_config tim12_config = {
	    .htim = &htim12,
	    .channel = TIM_CHANNEL_2,
	    .clk_freq = APB1_FREQ * 1000000,
	    .mode = TIM_PWM_MODE,
	};
	tim5 = tim_register(&tim5_config);
	tim12 = tim_register(&tim12_config);

	/* usart config */
	struct usart_config usart5_config = {
	    .huart = &huart5,
	    .rx_mode = USART_RECEIVE_IDLE_DMA_CIRCULAR,
	    .tx_mode = USART_TRANSMIT_NONE,
	    .callback = usart5_callback,
	};
	usart5 = usart_register(&usart5_config);

	/* gpio config */
	struct gpio_config gpio_pc15_config = {
	    .port = GPIOC,
	    .pin = GPIO_PIN_15,
	    .pin_init_state = GPIO_PIN_SET,
	};
	gpio_pc15 = gpio_register(&gpio_pc15_config);
}

void device_init()
{
	buzzer_register(tim12);
	ws2812_register(spi6);
	f103_init();
	motor_init();
	vofa_init();
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

	/* Enable the CPU Cache */

	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();

	/* Enable D-Cache---------------------------------------------------------*/
	SCB_EnableDCache();

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI6_Init();
	MX_TIM12_Init();
	MX_UART7_Init();
	MX_USART10_UART_Init();
	MX_UART5_Init();
	MX_FDCAN1_Init();
	MX_FDCAN2_Init();
	MX_FDCAN3_Init();
	MX_TIM5_Init();
	MX_IWDG1_Init();
	/* USER CODE BEGIN 2 */
	bsp_init();
	device_init();
	can_start();

	LOG_DEBUG("========================================");	 /* for yellow color */
	LOG_ERROR("FUCKING ROBOMASTER STM32 RTT Terminal Test"); /* for red color */
	LOG_DEBUG("========================================");	 /* for yellow color */

	buzzer_ctrl(4000, 50, 0.5);
	// float target = 0.0f;
	// float actual = 0.0f;
	// float time_ticks = 0.0f;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// target = 100.0f * __builtin_sinf(time_ticks);
		// actual += (target - actual) * 0.1f;
		// time_ticks += 0.05f;
		// dwt_delay_ms(10);
		// vofa_send(target, actual, 0.0f);
		dwt_delay_ms(10);
		HAL_IWDG_Refresh(&hiwdg1); /* feed the dog */
		sbus = sbus_get_data();
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 12;
	RCC_OscInitStruct.PLL.PLLN = 275;
	RCC_OscInitStruct.PLL.PLLP = 1;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
				      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
				      RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
	PeriphClkInitStruct.PLL2.PLL2M = 24;
	PeriphClkInitStruct.PLL2.PLL2N = 200;
	PeriphClkInitStruct.PLL2.PLL2P = 2;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
	PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
	PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct = {0};

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	 */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);

	/** Initializes and configures the Region and the memory to be protected
	 */
	MPU_InitStruct.Number = MPU_REGION_NUMBER1;
	MPU_InitStruct.BaseAddress = 0x30007000;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4KB;
	MPU_InitStruct.SubRegionDisable = 0x0;
	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
