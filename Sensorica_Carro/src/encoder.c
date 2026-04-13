#include "encoder.h"
#include "esp_check.h"
#include "driver/pulse_cnt.h"   // PCNT v5 API
#include "esp_timer.h"
#include <string.h>

static pcnt_unit_handle_t s_unit_m1 = NULL;
static pcnt_unit_handle_t s_unit_m2 = NULL;
static pcnt_unit_handle_t s_unit_m3 = NULL;
static pcnt_unit_handle_t s_unit_m4 = NULL;

// Para velocidad (estado interno)
typedef struct {
    int     last_count;     // último recuento corregido por signo
    int64_t last_time_us;   // timestamp de última muestra
    float   last_speed_mps; // velocidad más reciente
} enc_speed_state_t;

static enc_speed_state_t s_state[4]; // M1..M4

// Aplica inversión por motor
static inline int apply_invert(int raw, int idx) {
    switch (idx) {
        case 0: return ENCODER_INVERT_M1 ? -raw : raw;
        case 1: return ENCODER_INVERT_M2 ? -raw : raw;
        case 2: return ENCODER_INVERT_M3 ? -raw : raw;
        case 3: return ENCODER_INVERT_M4 ? -raw : raw;
        default: return raw;
    }
}

static void encoder_one_init(int gpioA, int gpioB, pcnt_unit_handle_t *out_unit)
{
    pcnt_unit_config_t unit_cfg = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT,
        .low_limit  = ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, out_unit));

    pcnt_glitch_filter_config_t filter_cfg = { .max_glitch_ns = ENCODER_GLITCH_NS };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*out_unit, &filter_cfg));

    pcnt_channel_handle_t ch_a = NULL, ch_b = NULL;
    pcnt_chan_config_t ch_a_cfg = { .edge_gpio_num = gpioA, .level_gpio_num = gpioB };
    pcnt_chan_config_t ch_b_cfg = { .edge_gpio_num = gpioB, .level_gpio_num = gpioA };
    ESP_ERROR_CHECK(pcnt_new_channel(*out_unit, &ch_a_cfg, &ch_a));
    ESP_ERROR_CHECK(pcnt_new_channel(*out_unit, &ch_b_cfg, &ch_b));

    // Cuadratura estándar A/B
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        ch_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        ch_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*out_unit, ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*out_unit, ENCODER_PCNT_LOW_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(*out_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*out_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(*out_unit));
}

void Encoder_Init(void)
{
    encoder_one_init(ENCODER_GPIO_H1A, ENCODER_GPIO_H1B, &s_unit_m1);
    encoder_one_init(ENCODER_GPIO_H2A, ENCODER_GPIO_H2B, &s_unit_m2);
    encoder_one_init(ENCODER_GPIO_H3A, ENCODER_GPIO_H3B, &s_unit_m3);
    encoder_one_init(ENCODER_GPIO_H4A, ENCODER_GPIO_H4B, &s_unit_m4);

    // Estado de velocidad
    memset(s_state, 0, sizeof(s_state));
    int now_c[4];
    // leer contadores iniciales ya con signo corregido
    now_c[0] = Encoder_Get_Count_M1();
    now_c[1] = Encoder_Get_Count_M2();
    now_c[2] = Encoder_Get_Count_M3();
    now_c[3] = Encoder_Get_Count_M4();
    int64_t t = esp_timer_get_time();
    for (int i=0;i<4;++i) {
        s_state[i].last_count = now_c[i];
        s_state[i].last_time_us = t;
        s_state[i].last_speed_mps = 0.0f;
    }
}

static inline int encoder_get_raw(pcnt_unit_handle_t u)
{
    int v = 0;
    if (u) pcnt_unit_get_count(u, &v);
    return v;
}

int Encoder_Get_Count_M1(void) { return apply_invert(encoder_get_raw(s_unit_m1), 0); }
int Encoder_Get_Count_M2(void) { return apply_invert(encoder_get_raw(s_unit_m2), 1); }
int Encoder_Get_Count_M3(void) { return apply_invert(encoder_get_raw(s_unit_m3), 2); }
int Encoder_Get_Count_M4(void) { return apply_invert(encoder_get_raw(s_unit_m4), 3); }

int Encoder_Get_Count(uint8_t id)
{
    switch (id) {
        case ENCODER_ID_M1: return Encoder_Get_Count_M1();
        case ENCODER_ID_M2: return Encoder_Get_Count_M2();
        case ENCODER_ID_M3: return Encoder_Get_Count_M3();
        case ENCODER_ID_M4: return Encoder_Get_Count_M4();
        default: return 0;
    }
}

void Encoder_Get_Count_All(int* c1, int* c2, int* c3, int* c4)
{
    if (c1) *c1 = Encoder_Get_Count_M1();
    if (c2) *c2 = Encoder_Get_Count_M2();
    if (c3) *c3 = Encoder_Get_Count_M3();
    if (c4) *c4 = Encoder_Get_Count_M4();
}

void Encoder_Clear_All(void)
{
    if (s_unit_m1) pcnt_unit_clear_count(s_unit_m1);
    if (s_unit_m2) pcnt_unit_clear_count(s_unit_m2);
    if (s_unit_m3) pcnt_unit_clear_count(s_unit_m3);
    if (s_unit_m4) pcnt_unit_clear_count(s_unit_m4);

    // Re-sincroniza el estado de velocidad
    int c1 = Encoder_Get_Count_M1();
    int c2 = Encoder_Get_Count_M2();
    int c3 = Encoder_Get_Count_M3();
    int c4 = Encoder_Get_Count_M4();
    int64_t t = esp_timer_get_time();
    int c[4] = {c1,c2,c3,c4};
    for (int i=0;i<4;++i) {
        s_state[i].last_count = c[i];
        s_state[i].last_time_us = t;
        s_state[i].last_speed_mps = 0.0f;
    }
}

// ---- Velocidad (m/s) -------------------------------------------------

static float pulses_to_mps(float pulses_per_sec)
{
    // v = (pulsos/s) * (circunferencia / ticks_rev)
    return pulses_per_sec * (ENCODER_WHEEL_CIRC_M / ENCODER_TICKS_PER_REV);
}

static float encoder_get_speed_idx(int idx, pcnt_unit_handle_t u)
{
    // Lee conteo actual (con signo ya invertido)
    int cur = apply_invert(encoder_get_raw(u), idx);
    int64_t now = esp_timer_get_time();

    int dp = cur - s_state[idx].last_count;            // delta pulses
    int64_t dt_us = now - s_state[idx].last_time_us;   // delta time (us)

    float v_mps = s_state[idx].last_speed_mps;

    if (dt_us > 0) {
        float pulses_per_sec = (float)dp / ((float)dt_us / 1e6f);
        v_mps = pulses_to_mps(pulses_per_sec);
    }

    // Actualiza estado
    s_state[idx].last_count = cur;
    s_state[idx].last_time_us = now;
    s_state[idx].last_speed_mps = v_mps;

    return v_mps;
}

float Encoder_Get_Speed_M1(void) { return encoder_get_speed_idx(0, s_unit_m1); }
float Encoder_Get_Speed_M2(void) { return encoder_get_speed_idx(1, s_unit_m2); }
float Encoder_Get_Speed_M3(void) { return encoder_get_speed_idx(2, s_unit_m3); }
float Encoder_Get_Speed_M4(void) { return encoder_get_speed_idx(3, s_unit_m4); }

void Encoder_Get_Speed_All(float* v1, float* v2, float* v3, float* v4)
{
    if (v1) *v1 = Encoder_Get_Speed_M1();
    if (v2) *v2 = Encoder_Get_Speed_M2();
    if (v3) *v3 = Encoder_Get_Speed_M3();
    if (v4) *v4 = Encoder_Get_Speed_M4();
}