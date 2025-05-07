//
// Created by Mateusz Wojtaszek on 20/04/2025.
// Zmodyfikowany do użycia filtru komplementarnego
// Dodano obliczenia prędkości i dystansu
//
#include "calculations.h"
#include <math.h> // Upewnij się, że jest dołączony

// --- Funkcja Orientacji (Filtr Komplementarny) ---
void calculate_orientation_complementary(Orientation *ori, Vector3f acc, Vector3f mag, Vector3f gyro, float dt) {
    // 1. Normalizacja wektora akcelerometru
    float acc_norm = vector_magnitude(acc);
    if (acc_norm == 0.0f) acc_norm = 1.0f; // Zapobieganie dzieleniu przez zero
    acc.x /= acc_norm;
    acc.y /= acc_norm;
    acc.z /= acc_norm;

    // 2. Obliczenie Roll i Pitch z akcelerometru (w stopniach)
    float acc_pitch_rad = asinf(-acc.x);
    float acc_roll_rad = atan2f(acc.y, acc.z);

    float acc_pitch_deg = acc_pitch_rad * RAD_TO_DEG;
    float acc_roll_deg = acc_roll_rad * RAD_TO_DEG;

    // 3. Obliczenie Yaw z magnetometru z kompensacją przechyłu (Tilt Compensation) (w stopniach)
    float roll_rad_filt = ori->roll * DEG_TO_RAD;
    float pitch_rad_filt = ori->pitch * DEG_TO_RAD;

    float mag_x_comp = mag.x * cosf(pitch_rad_filt) + mag.y * sinf(pitch_rad_filt) * sinf(roll_rad_filt) + mag.z * sinf(pitch_rad_filt) * cosf(roll_rad_filt);
    float mag_y_comp = mag.y * cosf(roll_rad_filt) - mag.z * sinf(roll_rad_filt);

    float mag_yaw_deg = atan2f(-mag_y_comp, mag_x_comp) * RAD_TO_DEG;

    if (mag_yaw_deg < 0.0f) {
        mag_yaw_deg += 360.0f;
    }

    // 4. Integracja żyroskopu (w stopniach)
    float gyro_roll_delta = gyro.x * dt;
    float gyro_pitch_delta = gyro.y * dt;
    float gyro_yaw_delta = gyro.z * dt;

    float gyro_roll_pred = ori->roll + gyro_roll_delta;
    float gyro_pitch_pred = ori->pitch + gyro_pitch_delta;
    float gyro_yaw_pred = ori->yaw + gyro_yaw_delta;

    if (gyro_yaw_pred >= 360.0f) {
        gyro_yaw_pred -= 360.0f;
    } else if (gyro_yaw_pred < 0.0f) {
        gyro_yaw_pred += 360.0f;
    }

    // 5. Zastosowanie filtru komplementarnego
    float alpha = COMPLEMENTARY_FILTER_ALPHA;

    ori->roll = alpha * gyro_roll_pred + (1.0f - alpha) * acc_roll_deg;
    ori->pitch = alpha * gyro_pitch_pred + (1.0f - alpha) * acc_pitch_deg;

    float yaw_error = mag_yaw_deg - gyro_yaw_pred;
    while (yaw_error > 180.0f) yaw_error -= 360.0f;
    while (yaw_error <= -180.0f) yaw_error += 360.0f;

    ori->yaw = gyro_yaw_pred + yaw_error * (1.0f - alpha);

    if (ori->yaw >= 360.0f) {
        ori->yaw -= 360.0f;
    } else if (ori->yaw < 0.0f) {
        ori->yaw += 360.0f;
    }

    if (ori->pitch > 90.0f) {
        ori->pitch = 90.0f;
    } else if (ori->pitch < -90.0f) {
        ori->pitch = -90.0f;
    }
}

// --- Nowe Funkcje Prędkości i Dystansu ---

/**
 * @brief Oblicza liniowe przyspieszenie obiektu w układzie świata, kompensując grawitację.
 * @param acc_sensor Surowe dane z akcelerometru (WAŻNE: w m/s^2!).
 * @param ori Aktualna orientacja obiektu.
 * @param g_val Wartość przyspieszenia grawitacyjnego (np. STALA_GRAWITACYJNA).
 * @param acc_linear_world Wskaźnik do Vector3f, gdzie zostanie zapisane obliczone przyspieszenie liniowe w układzie świata (m/s^2).
 */
void oblicz_przyspieszenie_liniowe_swiat(Vector3f acc_sensor, const Orientation *ori, float g_val, Vector3f *acc_linear_world) {
    Vector3f acc_linear_sensor;
    Vector3f gravity_sensor_frame;

    // Konwersja kątów orientacji na radiany
    float roll_rad = ori->roll * DEG_TO_RAD;
    float pitch_rad = ori->pitch * DEG_TO_RAD;
    float yaw_rad = ori->yaw * DEG_TO_RAD;

    // Obliczenie składowych wektora grawitacji w układzie czujnika
    // Zakładamy, że grawitacja w układzie świata działa wzdłuż osi Z (np. [0,0,-g_val] dla Z-up lub [0,0,g_val] dla Z-down).
    // Poniższe wzory odpowiadają transformacji wektora [0,0,-g_val] (Z-up, grawitacja "ciągnie" w dół) do układu czujnika.
    // Lub transformacji [0,0,g_val] (Z-down) do układu czujnika, gdzie g_val jest dodatnie.
    // Te składowe reprezentują to, co akcelerometr odczytałby z samej grawitacji przy danej orientacji.
    gravity_sensor_frame.x = g_val * (-sinf(pitch_rad));
    gravity_sensor_frame.y = g_val * (cosf(pitch_rad) * sinf(roll_rad));
    gravity_sensor_frame.z = g_val * (cosf(pitch_rad) * cosf(roll_rad));
    // Uwaga: Jeśli konwencja osi Z czujnika jest "w górę" gdy leży płasko, a grawitacja "ciągnie w dół",
    // to akcelerometr odczyta g_val na osi Z. Wzory na roll/pitch w calculate_orientation_complementary
    // (acc_pitch_rad = asinf(-acc.x), acc_roll_rad = atan2f(acc.y, acc.z)) sugerują, że
    // acc.x to "do przodu", acc.y to "w lewo", acc.z to "w dół" (jeśli płasko, odczytuje +g na Z).
    // W takim przypadku wektor grawitacji w układzie świata to [0,0,g_val] (Z-down).
    // Wtedy składowe grawitacji w ramce sensora to:
    // G_sensor_x = g_val * (-sin(pitch))
    // G_sensor_y = g_val * (cos(pitch) * sin(roll))
    // G_sensor_z = g_val * (cos(pitch) * cos(roll))
    // To wydaje się spójne.

    // Usunięcie składowej grawitacyjnej z odczytów akcelerometru
    // acc_sensor powinien być już w m/s^2
    acc_linear_sensor.x = acc_sensor.x - gravity_sensor_frame.x;
    acc_linear_sensor.y = acc_sensor.y - gravity_sensor_frame.y;
    acc_linear_sensor.z = acc_sensor.z - gravity_sensor_frame.z - 998.0f;

    // Transformacja liniowego przyspieszenia z układu czujnika do układu świata
    // Używamy macierzy rotacji z układu ciała do układu świata (R_world_from_body)
    // R_wb = Rz(yaw) * Ry(pitch) * Rx(roll)
    float c_r = cosf(roll_rad);
    float s_r = sinf(roll_rad);
    float c_p = cosf(pitch_rad);
    float s_p = sinf(pitch_rad);
    float c_y = cosf(yaw_rad);
    float s_y = sinf(yaw_rad);

    // Rząd pierwszy macierzy R_wb
    float r11 = c_y * c_p;
    float r12 = c_y * s_p * s_r - s_y * c_r;
    float r13 = c_y * s_p * c_r + s_y * s_r;
    // Rząd drugi macierzy R_wb
    float r21 = s_y * c_p;
    float r22 = s_y * s_p * s_r + c_y * c_r;
    float r23 = s_y * s_p * c_r - c_y * s_r;
    // Rząd trzeci macierzy R_wb
    float r31 = -s_p;
    float r32 = c_p * s_r;
    float r33 = c_p * c_r;

    acc_linear_world->x = r11 * acc_linear_sensor.x + r12 * acc_linear_sensor.y + r13 * acc_linear_sensor.z;
    acc_linear_world->y = r21 * acc_linear_sensor.x + r22 * acc_linear_sensor.y + r23 * acc_linear_sensor.z;
    acc_linear_world->z = r31 * acc_linear_sensor.x + r32 * acc_linear_sensor.y + r33 * acc_linear_sensor.z;
}

/**
 * @brief Aktualizuje wektor prędkości obiektu w układzie świata.
 * @param predkosc_swiat Wskaźnik do Vector3f przechowującego aktualną prędkość w układzie świata (m/s); wartość jest aktualizowana.
 * @param acc_linear_world Przyspieszenie liniowe obiektu w układzie świata (m/s^2).
 * @param dt Czas, jaki upłynął od ostatniej aktualizacji (w sekundach).
 */
void aktualizuj_predkosc_swiat(Vector3f *predkosc_swiat, Vector3f acc_linear_world, float dt) {
    predkosc_swiat->x += acc_linear_world.x * dt;
    predkosc_swiat->y += acc_linear_world.y * dt;
    predkosc_swiat->z += acc_linear_world.z * dt;
}

/**
 * @brief Aktualizuje wektor przemieszczenia obiektu w układzie świata.
 * @param przemieszczenie_swiat Wskaźnik do Vector3f przechowującego aktualne przemieszczenie w układzie świata (m); wartość jest aktualizowana.
 * @param predkosc_swiat Aktualna prędkość obiektu w układzie świata (m/s).
 * @param dt Czas, jaki upłynął od ostatniej aktualizacji (w sekundach).
 */
void aktualizuj_przemieszczenie_swiat(Vector3f *przemieszczenie_swiat, Vector3f predkosc_swiat, float dt) {
    przemieszczenie_swiat->x += predkosc_swiat.x * dt;
    przemieszczenie_swiat->y += predkosc_swiat.y * dt;
    przemieszczenie_swiat->z += predkosc_swiat.z * dt;
}

/**
 * @brief Aktualizuje całkowity przebyty dystans (skalar).
 * @param calkowity_dystans Wskaźnik do float przechowującego całkowity przebyty dystans (m); wartość jest aktualizowana.
 * @param predkosc_swiat Aktualna prędkość obiektu w układzie świata (m/s).
 * @param dt Czas, jaki upłynął od ostatniej aktualizacji (w sekundach).
 */
void aktualizuj_przebyty_dystans(float *calkowity_dystans, Vector3f predkosc_swiat, float dt) {
    float chwilowa_szybkosc = vector_magnitude(predkosc_swiat);
    *calkowity_dystans += chwilowa_szybkosc * dt;
}


// --- Funkcje średnich wartości (bez zmian) ---
Vector3f calculate_average_acceleration(Vector3f* data, uint16_t samples) {
    Vector3f avg = {0.0f, 0.0f, 0.0f};
    if (samples == 0) return avg;

    uint16_t i;
    for (i = 0; i < samples; i++) {
        avg.x += data[i].x;
        avg.y += data[i].y;
        avg.z += data[i].z;
    }
    avg.x /= (float)samples;
    avg.y /= (float)samples;
    avg.z /= (float)samples;
    return avg;
}

Vector3f calculate_average_angular_velocity(Vector3f* data, uint16_t samples) {
     Vector3f avg = {0.0f, 0.0f, 0.0f};
    if (samples == 0) return avg;

    uint16_t i;
    for (i = 0; i < samples; i++) {
        avg.x += data[i].x;
        avg.y += data[i].y;
        avg.z += data[i].z;
    }
    avg.x /= (float)samples;
    avg.y /= (float)samples;
    avg.z /= (float)samples;
    return avg;
}

// --- Funkcje pomocnicze (bez zmian) ---
float vector_magnitude(Vector3f v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}