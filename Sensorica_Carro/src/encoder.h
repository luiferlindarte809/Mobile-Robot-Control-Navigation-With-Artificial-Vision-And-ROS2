#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ====== Pines (ajusta a tu carrier Yahboom) ======
#define ENCODER_GPIO_H1A   (6)
#define ENCODER_GPIO_H1B   (7)
#define ENCODER_GPIO_H2A   (47)
#define ENCODER_GPIO_H2B   (48)
#define ENCODER_GPIO_H3A   (11)
#define ENCODER_GPIO_H3B   (12)
#define ENCODER_GPIO_H4A   (1)
#define ENCODER_GPIO_H4B   (2)

// ====== Config física para convertir a m/s ======
// Circunferencia de rueda en metros (Ø 65 mm -> ~0.2042 m)
#ifndef ENCODER_WHEEL_CIRC_M
#define ENCODER_WHEEL_CIRC_M   (0.1615f)
#endif
// Ticks del encoder por vuelta de rueda (incluye x4 si usas cuadratura)
#ifndef ENCODER_TICKS_PER_REV
#define ENCODER_TICKS_PER_REV  (1040)
#endif

// Límites + filtro
#define ENCODER_PCNT_HIGH_LIMIT   ( 32767)
#define ENCODER_PCNT_LOW_LIMIT    (-32768)
#define ENCODER_GLITCH_NS         (1000)   // 1 us

// Inversión de sentido (1 invierte el signo reportado)
// Dejamos M3 y M4 invertidos para que avanzar sea positivo.
#ifndef ENCODER_INVERT_M1
#define ENCODER_INVERT_M1   (0)
#endif
#ifndef ENCODER_INVERT_M2
#define ENCODER_INVERT_M2   (0)
#endif
#ifndef ENCODER_INVERT_M3
#define ENCODER_INVERT_M3   (1)
#endif
#ifndef ENCODER_INVERT_M4
#define ENCODER_INVERT_M4   (1)
#endif

typedef enum {
    ENCODER_ID_M1 = 0,
    ENCODER_ID_M2,
    ENCODER_ID_M3,
    ENCODER_ID_M4
} encoder_id_t;

void Encoder_Init(void);

// --- Posición (contador acumulado, con signo corregido) ---
int  Encoder_Get_Count_M1(void);
int  Encoder_Get_Count_M2(void);
int  Encoder_Get_Count_M3(void);
int  Encoder_Get_Count_M4(void);
int  Encoder_Get_Count(uint8_t encoder_id);
void Encoder_Get_Count_All(int* c1, int* c2, int* c3, int* c4);

// --- Velocidad en m/s (calculada por Δpulsos/Δt con timestamp interno) ---
float Encoder_Get_Speed_M1(void);
float Encoder_Get_Speed_M2(void);
float Encoder_Get_Speed_M3(void);
float Encoder_Get_Speed_M4(void);
void  Encoder_Get_Speed_All(float* v1, float* v2, float* v3, float* v4);

// Limpia contadores PCNT
void Encoder_Clear_All(void);

#ifdef __cplusplus
}
#endif