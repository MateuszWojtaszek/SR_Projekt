//
// Created by Mateusz Wojtaszek on 20/04/2025.
// Zmodyfikowany do użycia filtru komplementarnego
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
 * @param acc Wektor przyspieszenia (np. w g lub m/s^2).
 * @param mag Wektor pola magnetycznego (np. w Gaussach, uT - skalibrowany!).
 * @param gyro Wektor prędkości kątowej (WAŻNE: w stopniach na sekundę!).
 * @param dt Czas, jaki upłynął od ostatniego wywołania funkcji (w sekundach).
 */
void calculate_orientation_complementary(Orientation *ori, Vector3f acc, Vector3f mag, Vector3f gyro, float dt);


// --- Oryginalna funkcja (można zostawić do porównań lub usunąć) ---
// Orientation calculate_orientation_from_accel_mag(Vector3f acc, Vector3f mag);

// --- Funkcje średnich wartości (bez zmian) ---
Vector3f calculate_average_acceleration(Vector3f* data, uint16_t samples);
Vector3f calculate_average_angular_velocity(Vector3f* data, uint16_t samples);

// --- Funkcje pomocnicze (bez zmian) ---
float vector_magnitude(Vector3f v);

// --- Funkcja obliczania kursu tylko z magnetometru (bez kompensacji, można zostawić lub usunąć) ---
// float calculate_heading_from_mag(Vector3f mag); // Została zintegrowana w calculate_orientation_complementary

#endif // CALCULATIONS_H