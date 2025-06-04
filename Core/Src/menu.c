#include "menu.h"
#include "LSM303C.h"
#include "bsp_lcd_glass.h"
#include "calculations.h"
#include "gyro.h"
#include "rtt_logger.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

extern FullProcessedData acc_full_data;
extern GyroFullProcessedData gyro_full_data;
extern Orientation current_orientation;
extern Vector3f predkosc_w_ukladzie_swiata;
extern float dt;

static char lcd_buffer[7];

static void _format_value_3char(char *buf, size_t buf_len, float value) {
  if (fabsf(value) >= 10.0f) {
    int int_val = (int)roundf(value);
    if (int_val > 99)
      int_val = 99;
    else if (int_val < -99)
      int_val = -99;
    snprintf(buf, buf_len, "%d", int_val);
  } else {
    snprintf(buf, buf_len, "%.1f", value);
  }
}

static void _format_speed_value_4char(char *buf, size_t buf_len, float value) {
  if (value >= 1000.0f) {
    strncpy(buf, ">999", buf_len - 1);
  } else if (value >= 100.0f) {
    snprintf(buf, buf_len, "%.0f", value); // "123" (3 znaki)
  } else if (value >= 10.0f) {
    snprintf(buf, buf_len, "%.1f", value); // "12.3" (4 znaki)
  } else {
    snprintf(buf, buf_len, "%.1f", value); // "1.2" (3 znaki)
  }
  buf[buf_len - 1] = '\0';
}

static void _menu_display_main() {
  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_DisplayString((uint8_t *)"-MENU-");
  /* DEBUG_LOG("MENU: Main"); */
}

static void _menu_display_compass_item() {
  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_DisplayString((uint8_t *)"COMPAS");
  DEBUG_LOG("MENU: Compass Item");
}

static void _menu_display_speed_current_item() {
  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_DisplayString((uint8_t *)"SPEED");
  DEBUG_LOG("MENU: Speed Current Item");
}

static void _menu_display_speed_avr_item() {
  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_DisplayString((uint8_t *)"SPDAVR");
  DEBUG_LOG("MENU: Speed Average Item");
}

static void _menu_display_speed_max_item() {
  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_DisplayString((uint8_t *)"SPDMAX");
  DEBUG_LOG("MENU: Speed Max Item");
}

static void _menu_display_imu_data_item() {
  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_DisplayString((uint8_t *)"IMUDTA");
  DEBUG_LOG("MENU: IMU Data Item");
}

static void _screen_display_compass_data() {
  float yaw_degrees = current_orientation.yaw;
  // Normalizacja yaw do 0-359.9 stopni
  while (yaw_degrees < 0.0f)
    yaw_degrees += 360.0f;
  while (yaw_degrees >= 360.0f)
    yaw_degrees -= 360.0f;

  snprintf(lcd_buffer, sizeof(lcd_buffer), "H%05.1f", yaw_degrees);
  // "H123.4" (6 znaków), "H012.3" (6 znaków)
  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_DisplayString((uint8_t *)lcd_buffer);
  DEBUG_LOG("SCREEN: Compass Data (Yaw: %.1f)", yaw_degrees);
}

static void _screen_display_speed_current_data(menu_t *menu) {
  // Obliczanie modułu 3D prędkości
  menu->last_calculated_speed =
      sqrtf(predkosc_w_ukladzie_swiata.x * predkosc_w_ukladzie_swiata.x +
            predkosc_w_ukladzie_swiata.y * predkosc_w_ukladzie_swiata.y);

  char val_str[5];
  _format_speed_value_4char(val_str, sizeof(val_str),
                            menu->last_calculated_speed);
  snprintf(lcd_buffer, sizeof(lcd_buffer), "V %s", val_str);

  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_DisplayString((uint8_t *)lcd_buffer);
  DEBUG_LOG("SCREEN: Current Speed: %.2f", menu->last_calculated_speed);
}

static void _update_speed_stats(menu_t *menu) {
  if (dt > 0.0001f) { // Aktualizuj tylko jeśli czas płynie
    menu->total_speed_sum +=
        menu->last_calculated_speed; // Użyj ostatniej obliczonej
    menu->speed_sample_count++;
    if (menu->last_calculated_speed > menu->max_speed_recorded) {
      menu->max_speed_recorded = menu->last_calculated_speed;
    }
  }
}

static void _screen_display_speed_avr_data(menu_t *menu) {
  _update_speed_stats(
      menu); // Aktualizuj statystyki przy każdym odświeżeniu ekranu
  float avg_speed = 0.0f;
  if (menu->speed_sample_count > 0) {
    avg_speed = menu->total_speed_sum / menu->speed_sample_count;
  }
  char val_str[4];
  _format_value_3char(val_str, sizeof(val_str), avg_speed);
  snprintf(lcd_buffer, sizeof(lcd_buffer), "AVR%s", val_str);

  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_DisplayString((uint8_t *)lcd_buffer);
  DEBUG_LOG("SCREEN: Average Speed: %.2f", avg_speed);
}

static void _screen_display_speed_max_data(menu_t *menu) {
  _update_speed_stats(menu); // Aktualizuj statystyki
  char val_str[4];
  _format_value_3char(val_str, sizeof(val_str), menu->max_speed_recorded);
  snprintf(lcd_buffer, sizeof(lcd_buffer), "MAX%s", val_str);

  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_DisplayString((uint8_t *)lcd_buffer);
  DEBUG_LOG("SCREEN: Max Speed: %.2f", menu->max_speed_recorded);
}

static void _screen_display_imu_data(const char *prefix, float value) {
  char val_str[4];
  _format_value_3char(val_str, sizeof(val_str), value);
  snprintf(lcd_buffer, sizeof(lcd_buffer), "%s %s", prefix, val_str);

  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_DisplayString((uint8_t *)lcd_buffer);
  DEBUG_LOG("SCREEN: IMU %s: %s (raw: %.2f)", prefix, val_str, value);
}

menu_t menu_new() {
  menu_t menu = {.current_state = MENU_ITEM_MAIN,
                 .previous_menu_item = MENU_ITEM_MAIN,
                 .total_speed_sum = 0.0f,
                 .speed_sample_count = 0,
                 .max_speed_recorded = 0.0f,
                 .last_calculated_speed = 0.0f};
  return menu;
}

void menu_action(menu_t *menu, menu_actions_t action) {
  if (menu->current_state <= MENU_ITEM_IMU_DATA) {
    menu->previous_menu_item = menu->current_state;
  }

  switch (menu->current_state) {
  // --- Nawigacja w głównym menu ---
  case MENU_ITEM_MAIN:
    if (action == RIGHT)
      menu->current_state = MENU_ITEM_COMPASS;
    else if (action == LEFT)
      menu->current_state = MENU_ITEM_IMU_DATA;
    else if (action == DOWN)
      menu->current_state = MENU_ITEM_MAIN;
    break;
  case MENU_ITEM_COMPASS:
    if (action == RIGHT)
      menu->current_state = MENU_ITEM_SPEED_CURRENT;
    else if (action == LEFT)
      menu->current_state = MENU_ITEM_MAIN;
    else if (action == DOWN)
      menu->current_state = SCREEN_DATA_COMPASS;
    break;
  case MENU_ITEM_SPEED_CURRENT:
    if (action == RIGHT)
      menu->current_state = MENU_ITEM_SPEED_AVR;
    else if (action == LEFT)
      menu->current_state = MENU_ITEM_COMPASS;
    else if (action == DOWN)
      menu->current_state = SCREEN_DATA_SPEED_CURRENT;
    break;
  case MENU_ITEM_SPEED_AVR:
    if (action == RIGHT)
      menu->current_state = MENU_ITEM_SPEED_MAX;
    else if (action == LEFT)
      menu->current_state = MENU_ITEM_SPEED_CURRENT;
    else if (action == DOWN)
      menu->current_state = SCREEN_DATA_SPEED_AVR;
    break;
  case MENU_ITEM_SPEED_MAX:
    if (action == RIGHT)
      menu->current_state = MENU_ITEM_IMU_DATA;
    else if (action == LEFT)
      menu->current_state = MENU_ITEM_SPEED_AVR;
    else if (action == DOWN)
      menu->current_state = SCREEN_DATA_SPEED_MAX;
    break;
  case MENU_ITEM_IMU_DATA:
    if (action == RIGHT)
      menu->current_state = MENU_ITEM_MAIN;
    else if (action == LEFT)
      menu->current_state = MENU_ITEM_SPEED_MAX;
    else if (action == DOWN)
      menu->current_state = SCREEN_DATA_IMU_AX;
    break;

  case SCREEN_DATA_COMPASS:
  case SCREEN_DATA_SPEED_CURRENT:
    if (action == UP)
      menu->current_state = menu->previous_menu_item;
    break;

  case SCREEN_DATA_SPEED_AVR:
  case SCREEN_DATA_SPEED_MAX:
    if (action == UP)
      menu->current_state = menu->previous_menu_item;
    else if (action == ENTER) {
      menu->total_speed_sum = 0.0f;
      menu->speed_sample_count = 0;
      if (menu->current_state == SCREEN_DATA_SPEED_MAX) {
        menu->max_speed_recorded = 0.0f;
      }
      DEBUG_LOG("Speed stats reset for %s",
                menu->current_state == SCREEN_DATA_SPEED_AVR ? "AVR" : "MAX");
    }
    break;

  // Ekrany IMU
  case SCREEN_DATA_IMU_AX:
    if (action == UP)
      menu->current_state = MENU_ITEM_IMU_DATA;
    else if (action == RIGHT)
      menu->current_state = SCREEN_DATA_IMU_AY;
    else if (action == LEFT)
      menu->current_state = SCREEN_DATA_IMU_GZ;
    break;
  case SCREEN_DATA_IMU_AY:
    if (action == UP)
      menu->current_state = MENU_ITEM_IMU_DATA;
    else if (action == RIGHT)
      menu->current_state = SCREEN_DATA_IMU_AZ;
    else if (action == LEFT)
      menu->current_state = SCREEN_DATA_IMU_AX;
    break;
  case SCREEN_DATA_IMU_AZ:
    if (action == UP)
      menu->current_state = MENU_ITEM_IMU_DATA;
    else if (action == RIGHT)
      menu->current_state = SCREEN_DATA_IMU_GX;
    else if (action == LEFT)
      menu->current_state = SCREEN_DATA_IMU_AY;
    break;
  case SCREEN_DATA_IMU_GX:
    if (action == UP)
      menu->current_state = MENU_ITEM_IMU_DATA;
    else if (action == RIGHT)
      menu->current_state = SCREEN_DATA_IMU_GY;
    else if (action == LEFT)
      menu->current_state = SCREEN_DATA_IMU_AZ;
    break;
  case SCREEN_DATA_IMU_GY:
    if (action == UP)
      menu->current_state = MENU_ITEM_IMU_DATA;
    else if (action == RIGHT)
      menu->current_state = SCREEN_DATA_IMU_GZ;
    else if (action == LEFT)
      menu->current_state = SCREEN_DATA_IMU_GX;
    break;
  case SCREEN_DATA_IMU_GZ:
    if (action == UP)
      menu->current_state = MENU_ITEM_IMU_DATA;
    else if (action == RIGHT)
      menu->current_state = SCREEN_DATA_IMU_AX;
    else if (action == LEFT)
      menu->current_state = SCREEN_DATA_IMU_GY;
    break;

  default:
    break;
  }
}

void menu_update(menu_t *menu) {
  if (menu->current_state == SCREEN_DATA_SPEED_CURRENT ||
      menu->current_state == SCREEN_DATA_SPEED_AVR ||
      menu->current_state == SCREEN_DATA_SPEED_MAX) {
    menu->last_calculated_speed =
        sqrtf(predkosc_w_ukladzie_swiata.x * predkosc_w_ukladzie_swiata.x +
              predkosc_w_ukladzie_swiata.y * predkosc_w_ukladzie_swiata.y);
  }

  switch (menu->current_state) {
  case MENU_ITEM_MAIN:
    _menu_display_main();
    break;
  case MENU_ITEM_COMPASS:
    _menu_display_compass_item();
    break;
  case MENU_ITEM_SPEED_CURRENT:
    _menu_display_speed_current_item();
    break;
  case MENU_ITEM_SPEED_AVR:
    _menu_display_speed_avr_item();
    break;
  case MENU_ITEM_SPEED_MAX:
    _menu_display_speed_max_item();
    break;
  case MENU_ITEM_IMU_DATA:
    _menu_display_imu_data_item();
    break;

  case SCREEN_DATA_COMPASS:
    _screen_display_compass_data();
    break;
  case SCREEN_DATA_SPEED_CURRENT:
    _screen_display_speed_current_data(menu);
    break;
  case SCREEN_DATA_SPEED_AVR:
    _screen_display_speed_avr_data(menu);
    break;
  case SCREEN_DATA_SPEED_MAX:
    _screen_display_speed_max_data(menu);
    break;
  case SCREEN_DATA_IMU_AX:
    _screen_display_imu_data("AX", acc_full_data.x);
    break;
  case SCREEN_DATA_IMU_AY:
    _screen_display_imu_data("AY", acc_full_data.y);
    break;
  case SCREEN_DATA_IMU_AZ:
    _screen_display_imu_data("AZ", acc_full_data.z);
    break;
  case SCREEN_DATA_IMU_GX:
    _screen_display_imu_data("GX", gyro_full_data.x_dps);
    break;
  case SCREEN_DATA_IMU_GY:
    _screen_display_imu_data("GY", gyro_full_data.y_dps);
    break;
  case SCREEN_DATA_IMU_GZ:
    _screen_display_imu_data("GZ", gyro_full_data.z_dps);
    break;
  default:
    menu->current_state = MENU_ITEM_MAIN;
    _menu_display_main();
    break;
  }
}
