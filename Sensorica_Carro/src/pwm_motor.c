#include "pwm_motor.h"
#include "driver/ledc.h"
#include "esp_check.h"

typedef struct {
  int pinA, pinB;
  ledc_channel_t chA, chB;
} motor_hw_t;

static motor_hw_t M[4] = {
  { PWM_GPIO_M1A, PWM_GPIO_M1B, LEDC_CHANNEL_0, LEDC_CHANNEL_1 },
  { PWM_GPIO_M2A, PWM_GPIO_M2B, LEDC_CHANNEL_2, LEDC_CHANNEL_3 },
  { PWM_GPIO_M3A, PWM_GPIO_M3B, LEDC_CHANNEL_4, LEDC_CHANNEL_5 },
  { PWM_GPIO_M4A, PWM_GPIO_M4B, LEDC_CHANNEL_6, LEDC_CHANNEL_7 },
};

static bool   g_stop_brake = true;
// Corrige el sentido: M1=+1, M2=+1, M3=-1, M4=-1
static int8_t g_dir[4] = { +1, +1, -1, -1 };

void PwmMotor_Set_Direction_Map(int8_t d1, int8_t d2, int8_t d3, int8_t d4) {
  g_dir[0] = (d1>=0)?+1:-1;
  g_dir[1] = (d2>=0)?+1:-1;
  g_dir[2] = (d3>=0)?+1:-1;
  g_dir[3] = (d4>=0)?+1:-1;
}

static inline int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static uint32_t speed_to_duty(int sp) {
  int mag = sp < 0 ? -sp : sp;
  if (mag == 0) return 0;

  // duty_% = DZ + (100-DZ)*(mag/100.0)
  int duty_pct = PWM_MOTOR_DEAD_ZONE + ( (PWM_MOTOR_MAX_VALUE - PWM_MOTOR_DEAD_ZONE) * mag ) / PWM_MOTOR_MAX_VALUE;
  // O directamente en 0..100:
  // int duty_pct = PWM_MOTOR_DEAD_ZONE + ( (100 - PWM_MOTOR_DEAD_ZONE) * mag ) / 100;

  if (duty_pct > 100) duty_pct = 100;
  return (uint32_t)duty_pct * PWM_MAX_DUTY / 100;
}


static void motor_coast(const motor_hw_t* m) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, m->chA, 0); 
  ledc_update_duty(LEDC_LOW_SPEED_MODE, m->chA);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, m->chB, 0); 
  ledc_update_duty(LEDC_LOW_SPEED_MODE, m->chB);
}

static void motor_brake(const motor_hw_t* m) {
  // Para muchos drivers tipo TB6612: HIGH/HIGH = freno
  ledc_set_duty(LEDC_LOW_SPEED_MODE, m->chA, PWM_MAX_DUTY); 
  ledc_update_duty(LEDC_LOW_SPEED_MODE, m->chA);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, m->chB, PWM_MAX_DUTY); 
  ledc_update_duty(LEDC_LOW_SPEED_MODE, m->chB);
}

static void motor_forward(const motor_hw_t* m, uint32_t duty) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, m->chA, duty); 
  ledc_update_duty(LEDC_LOW_SPEED_MODE, m->chA);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, m->chB, 0);    
  ledc_update_duty(LEDC_LOW_SPEED_MODE, m->chB);
}

static void motor_reverse(const motor_hw_t* m, uint32_t duty) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, m->chA, 0);    
  ledc_update_duty(LEDC_LOW_SPEED_MODE, m->chA);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, m->chB, duty); 
  ledc_update_duty(LEDC_LOW_SPEED_MODE, m->chB);
}

void PwmMotor_Init(void) {
  ledc_timer_config_t tcfg = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = PWM_RES_BITS,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = PWM_FREQ_HZ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

  for (int i = 0; i < 4; ++i) {
    ledc_channel_config_t cA = {
      .gpio_num = M[i].pinA, .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = M[i].chA, .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0
    };
    ledc_channel_config_t cB = cA; cB.gpio_num = M[i].pinB; cB.channel = M[i].chB;
    ESP_ERROR_CHECK(ledc_channel_config(&cA));
    ESP_ERROR_CHECK(ledc_channel_config(&cB));
    motor_coast(&M[i]);
  }
}

void PwmMotor_Set_Speed(motor_id_t id, int speed_percent) {
  if (id == MOTOR_ID_ALL) {
    for (int i = 0; i < 4; ++i) PwmMotor_Set_Speed(i, speed_percent);
    return;
  }
  int i = (int)id; if (i < 0 || i > 3) return;

  // 👇 NUEVO: invierte la convención global (ahora +% = avanzar)
  speed_percent = -speed_percent;

  // Aplica inversión de sentido específica (M3/M4 venían invertidos)
  speed_percent *= g_dir[i];

  speed_percent = clampi(speed_percent, -PWM_MOTOR_MAX_VALUE, PWM_MOTOR_MAX_VALUE);
  if (speed_percent > 0)      motor_forward(&M[i], speed_to_duty(speed_percent));
  else if (speed_percent < 0) motor_reverse(&M[i], speed_to_duty(speed_percent));
  else                        (g_stop_brake ? motor_brake(&M[i]) : motor_coast(&M[i]));
}


void PwmMotor_Set_Speed_All(int s1, int s2, int s3, int s4) {
  PwmMotor_Set_Speed(MOTOR_ID_M1, s1);
  PwmMotor_Set_Speed(MOTOR_ID_M2, s2);
  PwmMotor_Set_Speed(MOTOR_ID_M3, s3);
  PwmMotor_Set_Speed(MOTOR_ID_M4, s4);
}

void PwmMotor_Stop(motor_id_t id, bool brake) {
  g_stop_brake = brake;
  if (id == MOTOR_ID_ALL) { for (int i=0;i<4;++i) (brake? motor_brake(&M[i]) : motor_coast(&M[i])); return; }
  int i = (int)id; if (i<0||i>3) return;
  (brake? motor_brake(&M[i]) : motor_coast(&M[i]));
}