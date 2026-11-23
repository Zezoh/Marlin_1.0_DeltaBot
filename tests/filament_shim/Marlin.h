// Shim for the host test of filament.cpp: the machine is simulated in filament_test.cpp
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#define OVERSAMPLENR 16
#define PSTR(x) x
#define PROGMEM
#define CRITICAL_SECTION_START
#define CRITICAL_SECTION_END
#include "Configuration.h"
#define RUNOUT_PIN 14
#define FAN_PIN 9
#define FAN_SAFETY_PIN 8
extern int fanSpeed;
#define HIGH 1
#define LOW 0
int sim_read(int pin);
#define READ(p) sim_read(p)
extern "C" void sim_write(int pin, int v);
#define WRITE(p, v) sim_write(p, v)
#define WRITE_UNUSED(p, v)
#define SET_INPUT(p)
#define SET_OUTPUT(p)
extern int g_verbose;
#define SAY(...) do { if (g_verbose) printf(__VA_ARGS__); } while (0)
#define SERIAL_ECHO_START SAY("echo:")
#define SERIAL_ERROR_START SAY("Error:")
#define SERIAL_ECHOPGM(x) SAY("%s", x)
#define SERIAL_ECHOLNPGM(x) SAY("%s\n", x)
#define SERIAL_ERRORLNPGM(x) SAY("%s\n", x)
#define SERIAL_ERRORPGM(x) SAY("%s", x)
#define SERIAL_ERRORLN(x) SAY("%s\n", x)
#define SERIAL_ECHOLN(x) SAY("%s\n", x)
#define SERIAL_ECHOPAIR(name, value) SAY("%s%g", name, (double)(value))
#define SERIAL_PROTOCOLPGM(x) SAY("%s", x)
#define SERIAL_PROTOCOL_F(x, y) SAY("%.*f", (int)(y), (double)(x))
#define SERIAL_PROTOCOL(x) SAY("%s", x)
#define SERIAL_PROTOCOLLN(x) SAY("%s\n", x)
#define SERIAL_PROTOCOLLNPGM(x) SAY("%s\n", x)
inline void serialprintPGM(const char *s) { SAY("%s", s); }
#define MSG_OK "ok"
enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#endif
extern float current_position[4], max_pos[3];
extern bool axis_known_position[3];
extern bool cancel_heatup;
bool IsStopped();
void manage_heater();
void manage_inactivity(bool ignore_stepper_queue = false);
unsigned long millis();
void home_all_axes();
void move_to(float x, float y, float z, float feed_mm_s);
void move_e(float mm, float feed_mm_s);
void cancel_print();
void disable_heater();
bool machine_idle();
