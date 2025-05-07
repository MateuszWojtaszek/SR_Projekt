//
// Created by Mateusz Wojtaszek on 20/04/2025.
// Zmodyfikowany do użycia filtru komplementarnego
// Dodano obliczenia prędkości i dystansu
//

#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#include <stdint.h>
#include <math.h> // Dodajemy dla M_PI i funkcji matematycznych

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD_TO_DEG (180.0f / (float)M_PI)
#define DEG_TO_RAD ((float)M_PI / 180.0f)

// Współczynnik filtra komplementarnego (0 < alpha < 1)
// Wyższy alpha -> większe zaufanie do żyroskopu (szybsza reakcja, ale większy dryft)
// Niższy alpha -> większe zaufanie do akcelerometru/magnetometru (stabilniejszy, ale wolniejszy i podatny na zakłócenia)
#define COMPLEMENTARY_FILTER_ALPHA 0.98f

// Standardowa wartość przyspieszenia grawitacyjnego (w m/s^2)
#define STALA_GRAWITACYJNA 9.80665f

typedef struct {
    float roll;   // Roll angle in degrees
    float pitch;  // Pitch angle in degrees
    float yaw;    // Yaw (heading) in degrees
} Orientation;

typedef struct {
    float x;
    float y;
    float z;
} Vector3f;

// --- Funkcja Orientacji (Filtr Komplementarny) ---
/**
 * @brief Oblicza orientację (roll, pitch, yaw) używając filtru komplementarnego.
 * @param ori Wskaźnik do struktury Orientation, która przechowuje stan i jest aktualizowana.
 * @param acc Wektor przyspieszenia (np. w g lub m/s^2 - jednostka ważna dla interpretacji, ale filtr działa na normalizowanych wartościach dla roll/pitch).
 * @param mag Wektor pola magnetycznego (np. w Gaussach, uT - skalibrowany!).
 * @param gyro Wektor prędkości kątowej (WAŻNE: w stopniach na sekundę!).
 * @param dt Czas, jaki upłynął od ostatniego wywołania funkcji (w sekundach).
 */
void calculate_orientation_complementary(Orientation *ori, Vector3f acc, Vector3f mag, Vector3f gyro, float dt);

// --- Funkcje Prędkości i Dystansu ---

/**
 * @brief Oblicza liniowe przyspieszenie obiektu w układzie świata, kompensując grawitację.
 * @param acc_sensor Surowe dane z akcelerometru (WAŻNE: w m/s^2!).
 * @param ori Aktualna orientacja obiektu.
 * @param g_val Wartość przyspieszenia grawitacyjnego (np. STALA_GRAWITACYJNA).
 * @param acc_linear_world Wskaźnik do Vector3f, gdzie zostanie zapisane obliczone przyspieszenie liniowe w układzie świata (m/s^2).
 */
void oblicz_przyspieszenie_liniowe_swiat(Vector3f acc_sensor, const Orientation *ori, float g_val, Vector3f *acc_linear_world);

/**
 * @brief Aktualizuje wektor prędkości obiektu w układzie świata.
 * @param predkosc_swiat Wskaźnik do Vector3f przechowującego aktualną prędkość w układzie świata (m/s); wartość jest aktualizowana.
 * @param acc_linear_world Przyspieszenie liniowe obiektu w układzie świata (m/s^2).
 * @param dt Czas, jaki upłynął od ostatniej aktualizacji (w sekundach).
 */
void aktualizuj_predkosc_swiat(Vector3f *predkosc_swiat, Vector3f acc_linear_world, float dt);

/**
 * @brief Aktualizuje wektor przemieszczenia obiektu w układzie świata.
 * @param przemieszczenie_swiat Wskaźnik do Vector3f przechowującego aktualne przemieszczenie w układzie świata (m); wartość jest aktualizowana.
 * @param predkosc_swiat Aktualna prędkość obiektu w układzie świata (m/s).
 * @param dt Czas, jaki upłynął od ostatniej aktualizacji (w sekundach).
 */
void aktualizuj_przemieszczenie_swiat(Vector3f *przemieszczenie_swiat, Vector3f predkosc_swiat, float dt);

/**
 * @brief Aktualizuje całkowity przebyty dystans (skalar).
 * @param calkowity_dystans Wskaźnik do float przechowującego całkowity przebyty dystans (m); wartość jest aktualizowana.
 * @param predkosc_swiat Aktualna prędkość obiektu w układzie świata (m/s).
 * @param dt Czas, jaki upłynął od ostatniej aktualizacji (w sekundach).
 */
void aktualizuj_przebyty_dystans(float *calkowity_dystans, Vector3f predkosc_swiat, float dt);


// --- Oryginalna funkcja (można zostawić do porównań lub usunąć) ---
// Orientation calculate_orientation_from_accel_mag(Vector3f acc, Vector3f mag);

// --- Funkcje średnich wartości (bez zmian) ---
Vector3f calculate_average_acceleration(Vector3f* data, uint16_t samples);
Vector3f calculate_average_angular_velocity(Vector3f* data, uint16_t samples);

// --- Funkcje pomocnicze (bez zmian) ---
float vector_magnitude(Vector3f v);


#endif // CALCULATIONS_H