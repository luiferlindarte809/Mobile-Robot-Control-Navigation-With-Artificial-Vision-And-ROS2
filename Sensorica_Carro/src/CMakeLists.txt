# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
  SRCS "main.c" "ms200.c" "lidar_ms200.c" "uart1.c" "ring_buffer.c" "servo.c" "encoder.c" "pwm_motor.c" "pid.c"
  INCLUDE_DIRS "."
  PRIV_REQUIRES driver freertos esp_timer log
)