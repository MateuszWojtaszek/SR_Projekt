#ifndef MENU_H
#define MENU_H

#include <stdint.h>

typedef enum {
  MENU_ITEM_MAIN,
  MENU_ITEM_COMPASS,
  MENU_ITEM_SPEED_CURRENT,
  MENU_ITEM_SPEED_AVR,
  MENU_ITEM_SPEED_MAX,
  MENU_ITEM_IMU_DATA,

  // Ekrany wyświetlania danych
  SCREEN_DATA_COMPASS,
  SCREEN_DATA_SPEED_CURRENT,
  SCREEN_DATA_SPEED_AVR,
  SCREEN_DATA_SPEED_MAX,
  SCREEN_DATA_IMU_AX,
  SCREEN_DATA_IMU_AY,
  SCREEN_DATA_IMU_AZ,
  SCREEN_DATA_IMU_GX,
  SCREEN_DATA_IMU_GY,
  SCREEN_DATA_IMU_GZ,
} menu_state_t;

typedef enum { LEFT, RIGHT, UP, DOWN, ENTER } menu_actions_t;

typedef struct {
  menu_state_t current_state;
  menu_state_t previous_menu_item;

  float total_speed_sum;
  uint32_t speed_sample_count;
  float max_speed_recorded;
  float last_calculated_speed;
} menu_t;

menu_t menu_new();
void menu_action(menu_t *menu, menu_actions_t action);
void menu_update(menu_t *menu);

#endif // !MENU_H
