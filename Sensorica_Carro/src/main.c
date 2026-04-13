// main.c
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "encoder.h"
#include "pwm_motor.h"
#include "servo.h"
#include "uart1.h"
#include "lidar_ms200.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD(x) ((x) * (float)M_PI / 180.0f)

// ==================== Configuración ===========================
#define CTRL_HZ                 (100)               // 100 Hz => 10 ms
#define LOG_EVERY_N_CYCLES      (10)                // Log cada 100 ms aprox.
static const char *TAG_CTRL = "CTRL";
static const char *TAG_CMD  = "CMD";
static const float dt_sec   = 1.0f / CTRL_HZ;       // 0.01 s

// ==================== Globals pedidas =========================
// ---- globals ----
static volatile float g_ref_mps[4] = {0,0,0,0};   // las usa el PID
static const float VEL_MAX_MPS = 1.41f;           // tu tope medido

static inline float clampf(float v, float lo, float hi){
    return (v<lo)?lo: (v>hi)?hi: v;
}

// ==================== PID simple por motor ====================
typedef struct {
    float Kp, Ki, Kd;
    float i_term;
    float prev_err;
    float u;        // última salida (para telemetría)
} pid_t;

// Ganancias iniciales (ajústalas en campo)
static pid_t g_pid[4] = {
    { .Kp=160.0f/2, .Ki=80.0f*20, .Kd=0.2f*18, .i_term=0, .prev_err=0, .u=0 }, // M1
    { .Kp=160.0f/2, .Ki=80.0f*20, .Kd=0.2f*18, .i_term=0, .prev_err=0, .u=0 }, // M2
    { .Kp=160.0f/2, .Ki=80.0f*20, .Kd=0.2f*18, .i_term=0, .prev_err=0, .u=0 }, // M3
    { .Kp=160.0f/2, .Ki=80.0f*20, .Kd=0.2f*18, .i_term=0, .prev_err=0, .u=0 }, // M4
};

// Paso de PID con anti-windup condicional y saturación dura [-100,100]
static float pid_step(pid_t *p, float err)
{
    const float derr = (err - p->prev_err) / dt_sec;

    // salida P+D sin integral
    float u_pd = p->Kp * err + p->Kd * derr;

    // probar saturación con integral actual
    float u_test = u_pd + p->i_term;
    bool at_upper = (u_test >= 100.0f);
    bool at_lower = (u_test <= -100.0f);

    // ¿la integral ayuda a salir de la saturación?
    bool helps_upper = at_upper && (err < 0.0f);   // empuja hacia abajo
    bool helps_lower = at_lower && (err > 0.0f);   // empuja hacia arriba

    if (!at_upper && !at_lower) {
        p->i_term += p->Ki * err * dt_sec;
    } else if (helps_upper || helps_lower) {
        p->i_term += p->Ki * err * dt_sec;
    } // else: congela integral

    float u = u_pd + p->i_term;
    u = clampf(u, -100.0f, 100.0f);

    p->prev_err = err;
    p->u = u;
    return u;
}

// ==================== Tarea de comandos (tu versión) =========
// ---- tarea de comandos seriales ----
static void serial_cmd_task(void *arg)
{
    setvbuf(stdin, NULL, _IONBF, 0);  // stdin sin buffer (por si acaso)
    char line[128];

    ESP_LOGI("CMD","Comandos:");
    ESP_LOGI("CMD","  R v1 v2 v3 v4        (m/s)  Ej: R 0.8 0.8 0.8 0.8");
    ESP_LOGI("CMD","  S ang1 ang2          (deg)  Ej: S 30 -20   # S1=+30°, S2=-20°");

    while (1) {
        if (fgets(line, sizeof(line), stdin) != NULL) {
            // Normaliza: ignora espacios iniciales
            char *p = line;
            while (*p==' ' || *p=='\t') p++;

            // Acepta S/s o R/r como alias de “set ref”
            if (*p=='R' || *p=='r') {
                float r1, r2, r3, r4;
                int n = sscanf(p+1, "%f %f %f %f", &r1, &r2, &r3, &r4);
                if (n == 4) {
                    r1 = clampf(r1, -VEL_MAX_MPS, VEL_MAX_MPS);
                    r2 = clampf(r2, -VEL_MAX_MPS, VEL_MAX_MPS);
                    r3 = clampf(r3, -VEL_MAX_MPS, VEL_MAX_MPS);
                    r4 = clampf(r4, -VEL_MAX_MPS, VEL_MAX_MPS);

                    g_ref_mps[0] = r1;
                    g_ref_mps[1] = r2;
                    g_ref_mps[2] = r3;
                    g_ref_mps[3] = r4;

                    ESP_LOGI("CMD","Refs OK: r1=%.3f r2=%.3f r3=%.3f r4=%.3f m/s", r1,r2,r3,r4);
                } else {
                    ESP_LOGW("CMD","Formato inválido. Usa: S v1 v2 v3 v4 (m/s)");
                }

            } else if (*p=='S' || *p=='s') {
                // S Angulo1 Angulo2
                float a1f, a2f;
                int n = sscanf(p+1, "%f %f", &a1f, &a2f);
                if (n == 2) {
                    int16_t ang1 = (int16_t)lroundf(a1f);
                    int16_t ang2 = (int16_t)lroundf(a2f);
                    ang1 = clampf(ang1, -90, 90);
                    ang2 = clampf(ang2, -90, 20);

                    ESP_LOGI("CMD","Servo S1=%d°, S2=%d°", (int)ang1, (int)ang2);
                    Servo_Set_Angle(SERVO_ID_S1, ang1);
                    Servo_Set_Angle(SERVO_ID_S2, ang2);
                } else {
                    ESP_LOGW("CMD","Formato inválido. Usa: S ang1 ang2  (ej: S 30 -20)");
                }
            } else if (*p!='\0' && *p!='\r' && *p!='\n') {
                ESP_LOGW("CMD","Comando no reconocido: %s", p);
            }
        } else {
            // si no hay línea, duerme un poquito
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ==================== Lazo de control 100 Hz ==================
static void control_task(void *arg)
{
    const TickType_t period_ticks = pdMS_TO_TICKS(1000 / CTRL_HZ);
    configASSERT(period_ticks > 0); // evita assert xTaskDelayUntil

    TickType_t last = xTaskGetTickCount();
    uint32_t cyc = 0;

    // Estado inicial
    for (int i=0;i<4;++i) { g_pid[i].i_term=0; g_pid[i].prev_err=0; g_pid[i].u=0; }

    ESP_LOGI(TAG_CTRL, "Control a %d Hz (dt=%.2f ms)", CTRL_HZ, dt_sec*1000.0f);

    while (1) {
        xTaskDelayUntil(&last, period_ticks);

        // Lee velocidades actuales (m/s)
        float v1, v2, v3, v4;
        Encoder_Get_Speed_All(&v1, &v2, &v3, &v4);
        const float v[4] = { v1, v2, v3, v4 };

        // Para cada motor: e = r - v -> PID -> PWM
        for (int i=0;i<4;++i) {
            const float r = g_ref_mps[i];
            const float e = r - v[i];
            const float u = pid_step(&g_pid[i], e);         // -100..100
            PwmMotor_Set_Speed((motor_id_t)i, (int)lroundf(u));
        }


        // Telemetría + LIDAR cada 100 ms
        if (++cyc % LOG_EVERY_N_CYCLES == 0) {
            // --- LIDAR: proyección mínima derecha (80..99°) e izquierda (260..279°) ---
            float Min_proy_D = 12000.0f;  // mm (umbral alto)
            float Min_proy_I = 12000.0f;

            for (int angle = 80; angle < 100; ++angle) {
                uint16_t d = Lidar_Ms200_Get_Distance(angle); // mm
                float proj = fabsf((float)d * sinf(DEG2RAD((float)angle))); // mm
                if (proj < Min_proy_D) Min_proy_D = proj;
            }
            for (int angle = 260; angle < 280; ++angle) {
                uint16_t d = Lidar_Ms200_Get_Distance(angle); // mm
                float proj = fabsf((float)d * sinf(DEG2RAD((float)angle))); // mm
                if (proj < Min_proy_I) Min_proy_I = proj;
            }

            int64_t t_ms = esp_timer_get_time() / 1000;
            // Dentro del if (++cyc % LOG_EVERY_N_CYCLES == 0) { ... }
            ESP_LOGI(TAG_CTRL,
                    "@JSON{\"m1\":[%.3f,%.3f,%.1f],"
                        "\"m2\":[%.3f,%.3f,%.1f],"
                        "\"m3\":[%.3f,%.3f,%.1f],"
                        "\"m4\":[%.3f,%.3f,%.1f],"
                        "\"mins\":[%.1f,%.1f]}",
                    g_ref_mps[0], v1, g_pid[0].u,     // m1 = [ref, vel, u]
                    g_ref_mps[1], v2, g_pid[1].u,     // m2 = [ref, vel, u]
                    g_ref_mps[2], v3, g_pid[2].u,     // m3 = [ref, vel, u]
                    g_ref_mps[3], v4, g_pid[3].u,     // m4 = [ref, vel, u]
                    Min_proy_I, Min_proy_D);          // mins = [dIzq, dDer]
        }
    }
}

// ============================ app_main ========================
void app_main(void)
{
    // Init HW
    Uart1_Init();
    Lidar_Ms200_Init();
    Servo_Init();
    PwmMotor_Init();
    Encoder_Init();
    PwmMotor_Stop(MOTOR_ID_ALL, STOP_BRAKE);  // inmóvil al arrancar

    Servo_Set_Angle(SERVO_ID_S1, 0);
    Servo_Set_Angle(SERVO_ID_S2, -60);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Tasks
    xTaskCreatePinnedToCore(serial_cmd_task, "serial_cmd", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(control_task,    "ctrl_100hz", 4096, NULL, 5, NULL, 0);
}