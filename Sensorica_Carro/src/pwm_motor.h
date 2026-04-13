#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --------- Pines PWM ----------
#define PWM_GPIO_M1A   (4)
#define PWM_GPIO_M1B   (5)
#define PWM_GPIO_M2A   (15)
#define PWM_GPIO_M2B   (16)
#define PWM_GPIO_M3A   (9)
#define PWM_GPIO_M3B   (10)
#define PWM_GPIO_M4A   (13)
#define PWM_GPIO_M4B   (14)

// --------- Config PWM ----------
#define PWM_FREQ_HZ        (25000)   // 25 kHz
#define PWM_RES_BITS       (11)      // 0..2047
#define PWM_MAX_DUTY       ((1U << PWM_RES_BITS) - 1)

// Rango de mando “tipo porcentaje”
#define PWM_MOTOR_MAX_VALUE   (100)  // => 100%
#define PWM_MOTOR_DEAD_ZONE   (56)   // compensa zona muerta

typedef enum {
  MOTOR_ID_M1 = 0, MOTOR_ID_M2, MOTOR_ID_M3, MOTOR_ID_M4, MOTOR_ID_ALL
} motor_id_t;

#define STOP_BRAKE  true
#define STOP_COAST  false

void PwmMotor_Init(void);
void PwmMotor_Set_Speed(motor_id_t id, int speed_percent);      // -100..100
void PwmMotor_Set_Speed_All(int s1, int s2, int s3, int s4);    // -100..100
void PwmMotor_Stop(motor_id_t id, bool brake);

// (Opcional) cambiar mapa de dirección (+1 normal / -1 invertido)
void PwmMotor_Set_Direction_Map(int8_t d1, int8_t d2, int8_t d3, int8_t d4);

#ifdef __cplusplus
}
#endif