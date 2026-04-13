#include "pid.h"

static inline float clampf_(float v, float lo, float hi){
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

void PID_Init(PID_t* p, float Kp, float Ki, float Kd, float Ts)
{
  p->Kp = Kp; p->Ki = Ki; p->Kd = Kd; p->Ts = (Ts>0)?Ts:0.01f;
  p->integ = 0.0f; p->prev_y = 0.0f; p->d_filt = 0.0f;
  p->out_min = -100.0f; p->out_max = +100.0f;
  p->aw_beta = 0.3f;         // típico 0.2..0.5
  p->d_alpha = 0.6f;         // 0..1 (1 = más lento)
  p->first_update = true;
}

void PID_SetTunings(PID_t* p, float Kp, float Ki, float Kd){
  p->Kp = Kp; p->Ki = Ki; p->Kd = Kd;
}

void PID_SetOutputLimits(PID_t* p, float out_min, float out_max){
  if (out_max < out_min){ float t = out_max; out_max = out_min; out_min = t; }
  p->out_min = out_min; p->out_max = out_max;
}

void PID_SetAntiWindup(PID_t* p, float aw_beta){
  if (aw_beta < 0.0f) aw_beta = 0.0f;
  if (aw_beta > 1.0f) aw_beta = 1.0f;
  p->aw_beta = aw_beta;
}

void PID_SetDerivativeFilter(PID_t* p, float d_alpha){
  if (d_alpha < 0.0f) d_alpha = 0.0f;
  if (d_alpha > 1.0f) d_alpha = 1.0f;
  p->d_alpha = d_alpha;
}

void PID_Reset(PID_t* p){
  p->integ = 0.0f; p->prev_y = 0.0f; p->d_filt = 0.0f; p->first_update = true;
}

// PID con derivada sobre la medida (para evitar "kick" por escalón en r)
float PID_Update(PID_t* p, float r, float y)
{
  float e = r - y;

  if (p->first_update){
    p->prev_y = y;
    p->d_filt  = 0.0f;
    p->first_update = false;
  }

  // dy/dt y filtro 1er orden
  float dy = (y - p->prev_y) / p->Ts;
  p->prev_y = y;
  p->d_filt = p->d_alpha * p->d_filt + (1.0f - p->d_alpha) * dy;

  // Acciones
  float up = p->Kp * e;
  p->integ += p->Ki * p->Ts * e;   // integrar antes de saturar
  float ud = - p->Kd * p->d_filt;  // derivada sobre medida (resta)

  float u_unsat = up + p->integ + ud;

  // Saturación
  float u_sat = clampf_(u_unsat, p->out_min, p->out_max);

  // Anti-windup (back-calculation)
  if (p->aw_beta > 0.0f){
    p->integ += p->aw_beta * (u_sat - u_unsat);
  }

  return u_sat;
}