//
// Created by Mateusz Wojtaszek on 20/04/2025.
// Zmodyfikowany do użycia filtru komplementarnego
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
    // Unikamy niestabilności w okolicach +/- 90 stopni dla pitch
    float acc_pitch_rad = asinf(-acc.x); // arcsin daje kąt od -PI/2 do PI/2 (-90 do +90 stopni)
    float acc_roll_rad = atan2f(acc.y, acc.z);

    // Jeśli pitch jest blisko +/- 90 stopni, roll staje się niestabilny z atan2(y, z)
    // Można użyć atan2(acc.y, acc.x) lub innych metod, ale dla prostoty zostawiamy atan2(y,z)
    // Alternatywnie, w takich sytuacjach można bardziej polegać na żyroskopie.

    float acc_pitch_deg = acc_pitch_rad * RAD_TO_DEG;
    float acc_roll_deg = acc_roll_rad * RAD_TO_DEG;


    // 3. Obliczenie Yaw z magnetometru z kompensacją przechyłu (Tilt Compensation) (w stopniach)
    // Konwersja aktualnych roll/pitch (z poprzedniego kroku filtra) na radiany do obliczeń
    float roll_rad_filt = ori->roll * DEG_TO_RAD;
    float pitch_rad_filt = ori->pitch * DEG_TO_RAD; // Używamy pitch z filtra, nie acc_pitch

    // Kompensacja magnetometru - używamy roll/pitch z *poprzedniego* stanu filtra
    float mag_x_comp = mag.x * cosf(pitch_rad_filt) + mag.y * sinf(pitch_rad_filt) * sinf(roll_rad_filt) + mag.z * sinf(pitch_rad_filt) * cosf(roll_rad_filt);
    float mag_y_comp = mag.y * cosf(roll_rad_filt) - mag.z * sinf(roll_rad_filt);

    float mag_yaw_deg = atan2f(-mag_y_comp, mag_x_comp) * RAD_TO_DEG;

    // Normalizacja yaw do zakresu 0-360 stopni
    if (mag_yaw_deg < 0.0f) {
        mag_yaw_deg += 360.0f;
    }
     // Korekta: Czasami yaw z magnetometru potrzebuje dodatkowego przesunięcia (deklinacja magnetyczna + orientacja czujnika)
     // Należy to skalibrować eksperymentalnie lub uwzględnić deklinację dla lokalizacji.

    // 4. Integracja żyroskopu (w stopniach)
    // gyro jest w stopniach/sekundę, dt w sekundach
    float gyro_roll_delta = gyro.x * dt;
    float gyro_pitch_delta = gyro.y * dt;
    float gyro_yaw_delta = gyro.z * dt;

    // Prosta integracja (można zastosować bardziej zaawansowane metody, np. kwaterniony, jeśli wymagana jest większa precyzja)
    float gyro_roll_pred = ori->roll + gyro_roll_delta;
    float gyro_pitch_pred = ori->pitch + gyro_pitch_delta;
    float gyro_yaw_pred = ori->yaw + gyro_yaw_delta;

    // Normalizacja yaw z żyroskopu do zakresu 0-360
     if (gyro_yaw_pred >= 360.0f) {
        gyro_yaw_pred -= 360.0f;
    } else if (gyro_yaw_pred < 0.0f) {
        gyro_yaw_pred += 360.0f;
    }

    // 5. Zastosowanie filtru komplementarnego
    float alpha = COMPLEMENTARY_FILTER_ALPHA;

    ori->roll = alpha * gyro_roll_pred + (1.0f - alpha) * acc_roll_deg;
    ori->pitch = alpha * gyro_pitch_pred + (1.0f - alpha) * acc_pitch_deg;

    // Obsługa Yaw: Ze względu na możliwe duże różnice i "przeskakiwanie" przez 0/360 stopni,
    // prosty filtr może być problematyczny. Potrzebne jest poprawne obsłużenie różnicy kątów.
    // Prostsze podejście: resetuj yaw z mag co jakiś czas lub użyj mag_yaw tylko w zakresie (1-alpha).
    // Bardziej zaawansowane: oblicz najkrótszą różnicę kątową.

    // Proste podejście dla yaw (może wymagać dostrojenia alpha lub metody):
     // Oblicz różnicę kątową (shortest angular difference)
     float yaw_error = mag_yaw_deg - gyro_yaw_pred;
     while (yaw_error > 180.0f) yaw_error -= 360.0f;
     while (yaw_error <= -180.0f) yaw_error += 360.0f;

     // Zastosuj korektę z wagą (1-alpha)
     ori->yaw = gyro_yaw_pred + yaw_error * (1.0f - alpha);


    // Normalizacja finalnego yaw do 0-360 stopni
    if (ori->yaw >= 360.0f) {
        ori->yaw -= 360.0f;
    } else if (ori->yaw < 0.0f) {
        ori->yaw += 360.0f;
    }

     // Ograniczenie pitch do +/- 90 stopni, aby uniknąć problemów z Gimbal Lock w reprezentacji Eulera
     if (ori->pitch > 90.0f) {
         ori->pitch = 90.0f;
     } else if (ori->pitch < -90.0f) {
         ori->pitch = -90.0f;
     }
     // Podobnie można ograniczyć roll, jeśli jest taka potrzeba (np. do +/- 180)
}


// --- Oryginalna funkcja (można usunąć lub zostawić jako komentarz) ---
/*
Orientation calculate_orientation_from_accel_mag(Vector3f acc, Vector3f mag) {
    // ... (kod oryginalnej funkcji) ...
}
*/

// --- Funkcje średnich wartości (bez zmian) ---
Vector3f calculate_average_acceleration(Vector3f* data, uint16_t samples) {
    Vector3f avg = {0.0f, 0.0f, 0.0f};
    if (samples == 0) return avg; // Dodano zabezpieczenie przed dzieleniem przez zero

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
    if (samples == 0) return avg; // Dodano zabezpieczenie przed dzieleniem przez zero

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

/*
// Funkcja usunięta lub zintegrowana
float calculate_heading_from_mag(Vector3f mag) {
    float heading = atan2f(mag.y, mag.x) * RAD_TO_DEG;
    if (heading < 0.0f) {
        heading += 360.0f;
    }
    return heading;
}
*/