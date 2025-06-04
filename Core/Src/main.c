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
#include "stm32l4xx_hal.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gyro.h"
#include "menu.h"
#include "rtt_logger.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "LSM303C.h"
#include "calculations.h"
#include "flash.h"

#include "bsp_lcd_glass.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WROCLAW_LATITUDE 51.1079f
#define WROCLAW_LONGITUDE 17.0595f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const float gps_latitude = WROCLAW_LATITUDE;
const float gps_longitude = WROCLAW_LONGITUDE;

uint8_t button_action_ready = 1;
FullProcessedData acc_full_data = {0};
FullProcessedData mag_full_data = {0};
GyroFullProcessedData gyro_full_data = {0};
Orientation current_orientation = {0.0f, 0.0f, 0.0f};
Vector3f predkosc_w_ukladzie_swiata = {0.0f, 0.0f, 0.0f};
Vector3f przemieszczenie_w_ukladzie_swiata = {0.0f, 0.0f, 0.0f};
Vector3f przyspieszenie_liniowe_swiat;

uint32_t last_update_time = 0;
uint32_t current_time = 0;
float dt = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Retarget printf to UART2
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

/**
 * @brief Oblicza sumę kontrolną CRC-16/CCITT-FALSE.
 * @param data Wskaźnik na tablicę bajtów danych.
 * @param length Długość danych w bajtach.
 * @return 16-bitowa suma kontrolna CRC.
 *
 * @note Standard CRC-16/CCITT-FALSE:
 * - Wielomian (Polynomial): 0x1021 (x^16 + x^12 + x^5 + 1)
 * - Wartość początkowa (Initial Value): 0xFFFF
 * - Refleksja wejścia/wyjścia (Reflect In/Out): Nie
 * - XOR na wyjściu (XOR Out): 0x0000
 */
uint16_t crc16_ccitt_false(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF; // Wartość początkowa
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i]
           << 8; // XOR z kolejnym bajtem danych (przesuniętym)
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {          // Jeśli najstarszy bit jest ustawiony
        crc = (crc << 1) ^ 0x1021; // Przesuń w lewo i XOR z wielomianem
      } else {
        crc <<= 1; // Tylko przesuń w lewo
      }
    }
  }
  return crc;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  menu_t menu;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  INFO_LOG("INIT STARTING");
  // LCD
  BSP_LCD_GLASS_Init();
  // IMU
  gyro_init();
  gyro_selftest_calibrate();
  acc_init();
  accel_selftest_calibrate();
  mag_init();
  // FLASH
  if (Flash_Init() != QSPI_OK) {
    printf("BŁĄD: Inicjalizacja Flash nie powiodła się!\r\n");
    Error_Handler();
  }
  // MENU
  menu = menu_new();
  INFO_LOG("INIT DONE");

  // Test wyświetlacza LCD
  /*
  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_DisplayString((uint8_t *)"STM32L4"); // Wyświetli "STM32L"
  HAL_Delay(2000);
  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_DisplayString((uint8_t *)"TEST  ");
  BSP_LCD_GLASS_BarLevelConfig(BATTERYLEVEL_FULL);
  HAL_Delay(2000);
  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_ScrollSentence((uint8_t *)"LCD OK ", 2, SCROLL_SPEED_MEDIUM);
  BSP_LCD_GLASS_BarLevelConfig(
      BATTERYLEVEL_OFF); // Wyłącz paski baterii po teście
  printf("LCD Test Finished.\r\n");
  */

  last_update_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint8_t joy_center = HAL_GPIO_ReadPin(JOY_CENTER_GPIO_Port, JOY_CENTER_Pin);
    uint8_t joy_up = HAL_GPIO_ReadPin(JOY_UP_GPIO_Port, JOY_UP_Pin);
    uint8_t joy_down = HAL_GPIO_ReadPin(JOY_DOWN_GPIO_Port, JOY_DOWN_Pin);
    uint8_t joy_left = HAL_GPIO_ReadPin(JOY_LEFT_GPIO_Port, JOY_LEFT_Pin);
    uint8_t joy_right = HAL_GPIO_ReadPin(JOY_RIGHT_GPIO_Port, JOY_RIGHT_Pin);
    if (joy_center && button_action_ready) {
      DEBUG_LOG("JOY CENTER");
      menu_action(&menu, ENTER);
      button_action_ready = 0;
    }
    if (joy_up && button_action_ready) {
      DEBUG_LOG("JOY UP");
      menu_action(&menu, UP);
      button_action_ready = 0;
    }
    if (joy_down && button_action_ready) {
      DEBUG_LOG("JOY DOWN");
      menu_action(&menu, DOWN);
      button_action_ready = 0;
    }
    if (joy_left && button_action_ready) {
      DEBUG_LOG("JOY LEFT");
      menu_action(&menu, LEFT);
      button_action_ready = 0;
    }
    if (joy_right && button_action_ready) {
      DEBUG_LOG("JOY RIGHT");
      menu_action(&menu, RIGHT);
      button_action_ready = 0;
    }
    if (!(joy_center || joy_up || joy_down || joy_left || joy_right)) {
      button_action_ready = 1;
    }
    menu_update(&menu);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    current_time = HAL_GetTick();
    dt = (float)(current_time - last_update_time) / 1000.0f;
    last_update_time = current_time;

    if (dt <= 0.0f) {
      dt = 0.001f; // Minimalny dt, aby uniknąć dzielenia przez zero lub
                   // problemów
    }

    // Odczyt danych z czujników na podstawie flag przerwań
    if (gyro_data_ready_flag) {
      gyro_data_ready_flag = 0;
      gyro_read_data(
          &gyro_full_data); // Funkcja powinna wypełniać gyro_full_data
    }
    if (acc_data_ready_flag) {
      acc_data_ready_flag = 0;
      acc_read_data(&acc_full_data); // Funkcja powinna wypełniać acc_full_data
    }
    if (mag_data_ready_flag) {
      mag_data_ready_flag = 0;
      mag_read_data(&mag_full_data); // Funkcja powinna wypełniać mag_full_data
    }

    // Tworzenie wektorów z odczytanych danych
    Vector3f acc_vec = {acc_full_data.x, acc_full_data.y, acc_full_data.z};
    Vector3f mag_vec = {mag_full_data.x, mag_full_data.y, mag_full_data.z};
    Vector3f gyro_vec = {gyro_full_data.x_dps, gyro_full_data.y_dps,
                         gyro_full_data.z_dps};

    // Obliczanie orientacji
    calculate_orientation_complementary(&current_orientation, acc_vec, mag_vec,
                                        gyro_vec, dt);

    // --- Implementacja wysyłania danych z CRC i GPS ---
    char data_payload_buffer[250]; // Zwiększony bufor na dane CSV (12 IMU + 2
                                   // GPS)
    char full_frame_buffer[270]; // Zwiększony bufor na pełną ramkę (CSV + CRC)

    // 1. Formatowanie ładunku danych CSV (12 wartości IMU + 2 wartości GPS)
    // Używamy %.5f dla współrzędnych GPS dla większej precyzji
    sprintf(data_payload_buffer,
            "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%."
            "5f,%.5f",
            gyro_full_data.x_dps, gyro_full_data.y_dps, gyro_full_data.z_dps,
            acc_full_data.x, acc_full_data.y, acc_full_data.z, mag_full_data.x,
            mag_full_data.y, mag_full_data.z, current_orientation.roll,
            current_orientation.pitch, current_orientation.yaw, gps_latitude,
            gps_longitude); // Dodane wartości GPS

    // 2. Obliczanie sumy kontrolnej CRC-16 dla ładunku
    uint16_t crc_value = crc16_ccitt_false((uint8_t *)data_payload_buffer,
                                           strlen(data_payload_buffer));

    // 3. Tworzenie pełnej ramki: dane_csv*CRC_HEX\r\n
    sprintf(full_frame_buffer, "%s*%04X\r\n", data_payload_buffer, crc_value);

    // OBLICZENIA
    oblicz_przyspieszenie_liniowe_swiat(acc_vec, &current_orientation,
                                        STALA_GRAWITACYJNA,
                                        &przyspieszenie_liniowe_swiat);
    //
    aktualizuj_predkosc_swiat(&predkosc_w_ukladzie_swiata,
                              przyspieszenie_liniowe_swiat, dt);
    //
    /*
    aktualizuj_przemieszczenie_swiat(&przemieszczenie_w_ukladzie_swiata,
                                     predkosc_w_ukladzie_swiata, dt);

    float przebyty_dystans_calkowity_temp = 0;
    aktualizuj_przebyty_dystans(&przebyty_dystans_calkowity_temp,
                                predkosc_w_ukladzie_swiata, dt);
                                */
    INFO_LOG("%f, %f, %f", predkosc_w_ukladzie_swiata.x,
             predkosc_w_ukladzie_swiata.y, predkosc_w_ukladzie_swiata.z);
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
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  printf("!!! CRITICAL Error_Handler() !!!\r\n");
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
