#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  // Ganancias
  float Kp, Ki, Kd;
  // Discretización
  float Ts;
  // Estado
  float integ;      // integrador
  float prev_y;     // medición anterior
  float d_filt;     // derivada filtrada
  // Config
  float out_min, out_max;  // saturación (p.ej. -100..100)
  float aw_beta;           // anti-windup back-calculation (0..1)
  float d_alpha;           // filtro derivada (0..1) (1 = más lento)
  // Flags
  bool  first_update;
} PID_t;

void  PID_Init(PID_t* p, float Kp, float Ki, float Kd, float Ts);
void  PID_SetTunings(PID_t* p, float Kp, float Ki, float Kd);
void  PID_SetOutputLimits(PID_t* p, float out_min, float out_max);
void  PID_SetAntiWindup(PID_t* p, float aw_beta);
void  PID_SetDerivativeFilter(PID_t* p, float d_alpha);
void  PID_Reset(PID_t* p);
float PID_Update(PID_t* p, float r, float y);

#ifdef __cplusplus
}
#endif