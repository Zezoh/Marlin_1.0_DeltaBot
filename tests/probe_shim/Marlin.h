// Shim for host tests of probe.cpp: the machine is simulated in probe_test.cpp
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#define OVERSAMPLENR 16
#define PSTR(x) x
#define PROGMEM
#include "Configuration.h"
#define sq(x) ((x)*(x))
enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};
extern int g_verbose;
#define SAY(...) do { if (g_verbose) printf(__VA_ARGS__); } while (0)
#define SERIAL_ECHO_START SAY("echo:")
#define SERIAL_ERROR_START SAY("Error:")
#define SERIAL_ECHOPGM(x) SAY("%s", x)
#define SERIAL_ECHOLNPGM(x) SAY("%s\n", x)
#define SERIAL_ERRORLNPGM(x) SAY("%s\n", x)
#define SERIAL_ECHOLN(x) SAY("%s\n", x)
#define SERIAL_ECHOPAIR(name, value) SAY("%s%g", name, (double)(value))
#define SERIAL_PROTOCOLPGM(x) SAY("%s", x)
#define SERIAL_PROTOCOL_F(x, y) SAY("%.*f", (int)(y), (double)(x))
#define SERIAL_PROTOCOLLN(x) SAY("%s\n", x)
extern float current_position[4], delta[3], delta_radius, delta_diagonal_rod, delta_tower_angle[3], endstop_adj[3], add_homing[3];
extern bool axis_known_position[3];
extern uint8_t active_extruder;
void calculate_delta(float cartesian[3]);
void recalc_delta_settings(float radius, float diagonal_rod);
void home_all_axes();
void move_to(float x, float y, float z, float feed_mm_s);
bool IsStopped();
void keep_steppers_on();
void manage_heater();
void manage_inactivity(bool ignore_stepper_queue = false);
unsigned long millis();
