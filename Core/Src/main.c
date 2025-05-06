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
#include "gpio.h"
#include "lcd.h"
#include "quadspi.h"
#include "spi.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gyro.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "LSM303C.h"
#include "calculations.h"
#include "flash.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}
void print_heading(float M_X, float M_Y) {
  float heading = calculate_heading(M_X, M_Y);
  printf("Heading (Północ): %.2f°\r\n", heading);
}

volatile uint8_t gyro_data_ready_flag = 0;
volatile uint8_t acc_data_ready_flag = 0;
volatile uint8_t mag_data_ready_flag = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GYRO_INT_Pin) {
    gyro_data_ready_flag = 1;
  }
  if (GPIO_Pin == ACCEL_INT_Pin) {
    acc_data_ready_flag = 1;
  }
  if (GPIO_Pin == MAG_DRDY_Pin) {
    mag_data_ready_flag = 1;
  }
}
/* USER CODE END 0 */

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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_QUADSPI_Init();
  MX_LCD_Init();
  /* USER CODE BEGIN 2 */
  printf("User Inits!\r\n");
  gyro_init();
  // gyro_selftest_calibrate();
  acc_init();
  // accel_selftest_calibrate();
  mag_init();
  printf("Inicjalizacja QSPI Flash...\r\n");
  if (Flash_Init() != QSPI_OK) {
    printf("BŁĄD: Inicjalizacja Flash nie powiodła się!\r\n");
  }
  printf("Inicjalizacja Flash OK.\r\n");
  FullProcessedData acc_full_data;
  FullProcessedData mag_full_data;
  GyroFullProcessedData gyro_full_data;
  // RawData acc_raw_data;
  // uint8_t flash_id=2;
  Orientation current_orientation = {0.0f, 0.0f, 0.0f};
  uint32_t last_update_time = 0;
  uint32_t current_time = 0;
  float dt = 0.0f;

  // Inicjalizacja przed pętlą
  last_update_time = HAL_GetTick();
  const char *napis_do_zapisu = "test";
  uint32_t dlugosc_napisu = strlen(napis_do_zapisu) + 1;
  uint32_t adres_flash = 0x00010000;

  uint8_t bufor_zapisu[16];
  uint8_t bufor_odczytu[16];

  strcpy((char *)bufor_zapisu, napis_do_zapisu);

  if (Flash_Erase_SubSector(adres_flash) == QSPI_OK) {
    printf("Kasowanie subsektora OK.\r\n");
    if (Flash_Write(bufor_zapisu, adres_flash, dlugosc_napisu) == QSPI_OK) {
      printf("Zapis danych OK.\r\n");
      memset(bufor_odczytu, 0xFF, sizeof(bufor_odczytu));
      if (Flash_Read(bufor_odczytu, adres_flash, dlugosc_napisu) == QSPI_OK) {
        printf("Odczytano napis: '%s'\r\n", (char *)bufor_odczytu);
      } else {
        printf("BŁĄD podczas odczytu danych z Flash!\r\n");
      }
    } else {
      printf("BŁĄD podczas zapisu danych do Flash!\r\n");
    }
  } else {
    printf("BŁĄD podczas kasowania subsektora!\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    current_time = HAL_GetTick();
    // Oblicz dt w sekundach (HAL_GetTick zwraca milisekundy)
    dt = (float)(current_time - last_update_time) / 1000.0f;
    last_update_time = current_time;

    // Zabezpieczenie przed zerowym dt przy pierwszym uruchomieniu lub bardzo
    // szybkich pętlach
    //   if (dt <= 0.0f) {
    //     dt = 0.01f; // Przyjmij jakąś małą wartość lub kontynuuj
    //   }
    //   if (gyro_data_ready_flag)
    //   {
    //     gyro_data_ready_flag = 0;
    //     gyro_read_data(&gyro_full_data);
    //   }
    //   if (acc_data_ready_flag)
    //   {
    //     acc_data_ready_flag = 0;
    //     acc_read_data(&acc_full_data);
    //   }
    //   if (mag_data_ready_flag)
    //   {
    //     mag_data_ready_flag = 0;
    //     mag_read_data(&mag_full_data);
    //   }
    //
    //   Vector3f acc_vec = {acc_full_data.x, acc_full_data.y, acc_full_data.z};
    //   Vector3f mag_vec = {mag_full_data.x, mag_full_data.y, mag_full_data.z};
    //   // WAŻNE: Upewnij się, że dane z żyroskopu są w stopniach na sekundę!
    //   Vector3f gyro_vec = {gyro_full_data.x_dps, gyro_full_data.y_dps,
    //   gyro_full_data.z_dps};
    //
    //   calculate_orientation_complementary(&current_orientation, acc_vec,
    //   mag_vec, gyro_vec, dt);
    //   printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
    //          gyro_full_data.x_dps, gyro_full_data.y_dps,
    //          gyro_full_data.z_dps, acc_full_data.x, acc_full_data.y,
    //          acc_full_data.z, mag_full_data.x, mag_full_data.y,
    //          mag_full_data.z, current_orientation.roll,
    //          current_orientation.pitch, current_orientation.yaw); // Używamy
    //          zaktualizowanej zmiennej
    //
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
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
    printf("Error occurred!\r\n");
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
