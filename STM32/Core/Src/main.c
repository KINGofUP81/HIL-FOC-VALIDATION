/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdint.h>
#include "cdc_rx.h"
#include "hil_tx.h"
#include "foc_maths.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* simple RX buffer */

/* for rate-limited printing */
uint32_t last_print_ms = 0;
Currents_t rx_currents;
volatile uint8_t frame_ready;
/* USER CODE END PTD */
float id;
float iq;
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_USB_Device_Init();


  //int i =0;
  /*while(i <= 4){
  	  CDC_Print("USB CDC ACTIVE\r\n");
  	  HAL_Delay(1000);
  }*/


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(1000);  // wait for USB enumeration


/* USB C BINARY FRAME PARSE CHECK :
  uint8_t test_frame[14] = {
      0xAA,
      0x00, 0x00, 0x80, 0x3F, //1.0f
      0x00, 0x00, 0x00, 0x40, //2.0f
      0x00, 0x00, 0x40, 0x40, //3.0fS
	  0x55

  };

  while (1)
  {
      static uint32_t last_tx = 0;

      if (HAL_GetTick() - last_tx > 500)
      {
          last_tx = HAL_GetTick();

          while (CDC_Transmit_FS(test_frame, 14) == USBD_BUSY);
      }
  }

  // USER CODE END 3

  //echo_frame transmit

/*while(1){
  static uint32_t last_tx = 0;
  if (frame_ready && (HAL_GetTick() - last_tx) > 500)
  {
      frame_ready = 0;
      last_tx = HAL_GetTick();

      uint8_t echo_frame[14];

      echo_frame[0]  = 0xAA;   // SOF
      echo_frame[13] = 0x55;   // EOF

      memcpy(&echo_frame[1],  &rx_currents.ia, 4);  // ia @ [1..4]
      memcpy(&echo_frame[5],  &rx_currents.ib, 4);  // ib @ [5..8]
      memcpy(&echo_frame[9],  &rx_currents.ic, 4);  // ic @ [9..12]

      while (CDC_Transmit_FS(echo_frame, 14) == USBD_BUSY);
  }
}
  }*/
  while (1)
      {
          /* 1️⃣ If new RX frame arrived */
          if (frame_ready)
          {
              frame_ready = 0;

              /* 2️⃣ Run math */
              IAlphaBeta_t iab;
              clarke_transform(
                  rx_currents.ia,
                  rx_currents.ib,
                  rx_currents.ic,
                  &iab
              );

              Park(
                  iab.i_alpha,
                  iab.i_beta,
                  rx_currents.theta,
                  &id,
                  &iq
              );
          }

          /* 3️⃣ Periodic TX (rate-limited) */
          static uint32_t last_tx = 0;
          if (HAL_GetTick() - last_tx >= 10)
          {
              last_tx = HAL_GetTick();

              float tx_vars[6] = {
                  rx_currents.ia,
                  rx_currents.ib,
                  rx_currents.ic,
                  id,
                  iq,
				  rx_currents.theta
              };

              HIL_SendFloats(tx_vars, 6);
          }
      }

      // other non-blocking work here


}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
  }
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
