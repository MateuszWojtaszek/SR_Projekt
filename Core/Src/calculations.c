//
// Created by Mateusz Wojtaszek on 20/04/2025.
// Zmodyfikowany do użycia filtru komplementarnego z kwaternionami.
// Dodano obliczenia prędkości i dystansu.
// Rozwiązano problemy z brakującymi definicjami funkcji.
//

#include "calculations.h"
#include <math.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

#ifndef COMPLEMENTARY_FILTER_ALPHA
#define COMPLEMENTARY_FILTER_ALPHA 0.98f // Większa waga dla żyroskopu
#endif

typedef struct {
  float w, x, y, z;
} Quaternion;
static Quaternion quaternion_normalize(Quaternion q) {
  float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (norm == 0.0f) {
    // [niepewne] Zabezpieczenie przed dzieleniem przez zero.
    // Kwaternion zerowy jest niepoprawny dla reprezentacji rotacji.
    // Zwrócenie kwaternionu identycznościowego jest bezpiecznym wyjściem.
    return (Quaternion){1.0f, 0.0f, 0.0f, 0.0f};
  }
  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  return q;
}

static Quaternion quaternion_multiply(Quaternion q1, Quaternion q2) {
  Quaternion result;
  result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
  return result;
}

static Quaternion quaternion_from_euler(float roll_rad, float pitch_rad,
                                        float yaw_rad) {
  float cy = cosf(yaw_rad * 0.5f);
  float sy = sinf(yaw_rad * 0.5f);
  float cp = cosf(pitch_rad * 0.5f);
  float sp = sinf(pitch_rad * 0.5f);
  float cr = cosf(roll_rad * 0.5f);
  float sr = sinf(roll_rad * 0.5f);

  Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

static void quaternion_to_euler(Quaternion q, float *roll_rad, float *pitch_rad,
                                float *yaw_rad) {
  // Roll (obrót wokół osi X)
  float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
  float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
  *roll_rad = atan2f(sinr_cosp, cosr_cosp);

  // Pitch (obrót wokół osi Y)
  float sinp = 2.0f * (q.w * q.y - q.z * q.x);
  if (fabsf(sinp) >= 1.0f)
    *pitch_rad = copysignf(M_PI / 2.0f, sinp);

  else
    *pitch_rad = asinf(sinp);

  float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
  float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  *yaw_rad = atan2f(siny_cosp, cosy_cosp);
}

static Quaternion quaternion_conjugate(Quaternion q) {
  return (Quaternion){q.w, -q.x, -q.y, -q.z};
}

static Vector3f quaternion_rotate_vector(Quaternion q, Vector3f v) {
  Quaternion v_quat = {0.0f, v.x, v.y, v.z};
  Quaternion q_conj = quaternion_conjugate(q);
  // temp = q * v_quat
  Quaternion temp = quaternion_multiply(q, v_quat);
  // rotated_v_quat = temp * q_conj
  Quaternion rotated_v_quat = quaternion_multiply(temp, q_conj);
  return (Vector3f){rotated_v_quat.x, rotated_v_quat.y, rotated_v_quat.z};
}

static Quaternion quaternion_lerp_norm(Quaternion q_start, Quaternion q_end,
                                       float t) {
  Quaternion q_res;
  float dot_product = q_start.w * q_end.w + q_start.x * q_end.x +
                      q_start.y * q_end.y + q_start.z * q_end.z;

  if (dot_product < 0.0f) {
    q_end.w = -q_end.w;
    q_end.x = -q_end.x;
    q_end.y = -q_end.y;
    q_end.z = -q_end.z;
  }

  q_res.w = (1.0f - t) * q_start.w + t * q_end.w;
  q_res.x = (1.0f - t) * q_start.x + t * q_end.x;
  q_res.y = (1.0f - t) * q_start.y + t * q_end.y;
  q_res.z = (1.0f - t) * q_start.z + t * q_end.z;
  return quaternion_normalize(q_res);
}

float vector_magnitude(Vector3f v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}
void calculate_orientation_complementary(Orientation *ori, Vector3f acc,
                                         Vector3f mag, Vector3f gyro,
                                         float dt) {
  Quaternion q_previous_state = quaternion_from_euler(
      ori->roll * DEG_TO_RAD, ori->pitch * DEG_TO_RAD, ori->yaw * DEG_TO_RAD);
  q_previous_state =
      quaternion_normalize(q_previous_state); // Upewnienie się o normalizacji

  float acc_norm_val = vector_magnitude(acc);
  Vector3f acc_normalized = acc;
  if (acc_norm_val == 0.0f) {
    acc_norm_val = 1.0f;
  }
  acc_normalized.x /= acc_norm_val;
  acc_normalized.y /= acc_norm_val;
  acc_normalized.z /= acc_norm_val;

  float acc_pitch_rad = asinf(-acc_normalized.x);
  float acc_roll_rad = atan2f(acc_normalized.y, acc_normalized.z);
  float mag_x_comp = mag.x * cosf(acc_pitch_rad) +
                     mag.y * sinf(acc_pitch_rad) * sinf(acc_roll_rad) +
                     mag.z * sinf(acc_pitch_rad) * cosf(acc_roll_rad);
  float mag_y_comp = mag.y * cosf(acc_roll_rad) - mag.z * sinf(acc_roll_rad);
  float mag_yaw_rad = atan2f(-mag_y_comp, mag_x_comp);

  Quaternion q_acc_mag =
      quaternion_from_euler(acc_roll_rad, acc_pitch_rad, mag_yaw_rad);
  q_acc_mag = quaternion_normalize(q_acc_mag);

  float gx_rad_s = gyro.x * DEG_TO_RAD;
  float gy_rad_s = gyro.y * DEG_TO_RAD;
  float gz_rad_s = gyro.z * DEG_TO_RAD;

  Quaternion q_gyro_predicted;
  q_gyro_predicted.w =
      q_previous_state.w +
      0.5f * dt *
          (-gx_rad_s * q_previous_state.x - gy_rad_s * q_previous_state.y -
           gz_rad_s * q_previous_state.z);
  q_gyro_predicted.x = q_previous_state.x + 0.5f * dt *
                                                (gx_rad_s * q_previous_state.w +
                                                 gz_rad_s * q_previous_state.y -
                                                 gy_rad_s * q_previous_state.z);
  q_gyro_predicted.y = q_previous_state.y + 0.5f * dt *
                                                (gy_rad_s * q_previous_state.w -
                                                 gz_rad_s * q_previous_state.x +
                                                 gx_rad_s * q_previous_state.z);
  q_gyro_predicted.z = q_previous_state.z + 0.5f * dt *
                                                (gz_rad_s * q_previous_state.w +
                                                 gy_rad_s * q_previous_state.x -
                                                 gx_rad_s * q_previous_state.y);

  q_gyro_predicted = quaternion_normalize(q_gyro_predicted);

  float alpha = COMPLEMENTARY_FILTER_ALPHA;
  Quaternion q_fused = quaternion_lerp_norm(q_acc_mag, q_gyro_predicted, alpha);

  float roll_rad_final, pitch_rad_final, yaw_rad_final;
  quaternion_to_euler(q_fused, &roll_rad_final, &pitch_rad_final,
                      &yaw_rad_final);
  ori->roll = roll_rad_final * RAD_TO_DEG;
  ori->pitch = pitch_rad_final * RAD_TO_DEG;
  ori->yaw = yaw_rad_final * RAD_TO_DEG;
  while (ori->yaw >= 360.0f)
    ori->yaw -= 360.0f;
  while (ori->yaw < 0.0f)
    ori->yaw += 360.0f;
  if (ori->pitch > 90.0f)
    ori->pitch = 90.0f;
  else if (ori->pitch < -90.0f)
    ori->pitch = -90.0f;
}

void oblicz_przyspieszenie_liniowe_swiat(Vector3f acc_sensor,
                                         const Orientation *ori, float g_val,
                                         Vector3f *acc_linear_world) {
  Quaternion q_wb = quaternion_from_euler(
      ori->roll * DEG_TO_RAD, ori->pitch * DEG_TO_RAD, ori->yaw * DEG_TO_RAD);
  q_wb = quaternion_normalize(q_wb);

  Vector3f gravity_world = {0.0f, 0.0f, g_val};

  Quaternion q_bw = quaternion_conjugate(q_wb);
  Vector3f gravity_sensor_frame = quaternion_rotate_vector(q_bw, gravity_world);

  Vector3f acc_linear_sensor;
  acc_linear_sensor.x = acc_sensor.x - gravity_sensor_frame.x;
  acc_linear_sensor.y = acc_sensor.y - gravity_sensor_frame.y;
  acc_linear_sensor.z = acc_sensor.z - gravity_sensor_frame.z;

  *acc_linear_world = quaternion_rotate_vector(q_wb, acc_linear_sensor);
}

void aktualizuj_predkosc_swiat(Vector3f *predkosc_swiat,
                               Vector3f acc_linear_world, float dt) {
  predkosc_swiat->x += acc_linear_world.x * dt;
  predkosc_swiat->y += acc_linear_world.y * dt;
  predkosc_swiat->z += acc_linear_world.z * dt;
}

void aktualizuj_przemieszczenie_swiat(Vector3f *przemieszczenie_swiat,
                                      Vector3f predkosc_swiat, float dt) {
  przemieszczenie_swiat->x += predkosc_swiat.x * dt;
  przemieszczenie_swiat->y += predkosc_swiat.y * dt;
  przemieszczenie_swiat->z += predkosc_swiat.z * dt;
}

void aktualizuj_przebyty_dystans(float *calkowity_dystans,
                                 Vector3f predkosc_swiat, float dt) {
  float chwilowa_szybkosc = vector_magnitude(predkosc_swiat);
  *calkowity_dystans += chwilowa_szybkosc * dt;
}
