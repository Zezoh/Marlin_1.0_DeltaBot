#!/usr/bin/env python3
"""Generates kin_src.h and arc_src.h from the real firmware sources so that the tests run the real code."""
import re, sys
root = sys.argv[1]; out = sys.argv[2]
s = open(root + '/Marlin/Marlin_main.cpp').read()
def grab(pattern):
    m = re.search(pattern, s, flags=re.S)
    assert m, pattern
    return m.group(0)
rd = grab(r'void recalc_delta_settings\(float radius, float diagonal_rod\)\n\{.*?\n\}\n')
cd = grab(r'void calculate_delta\(float cartesian\[3\]\)\n\{.*?\n\}\n')
dr = grab(r'bool delta_reachable\(const float cartesian\[3\]\)\n\{.*?\n\}\n')
vars_ = grab(r'float delta\[3\] = .*?float delta_segments_per_second ?= DELTA_SEGMENTS_PER_SECOND;\n')
common = '''#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <vector>
#define sq(x) ((x)*(x))
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3
#define NUM_AXIS 4
#define DELTA_SEGMENTS_PER_SECOND 200
#define DELTA_DIAGONAL_ROD 210.0
#define DELTA_RADIUS 90.0
#include "mesh.h"
''' + vars_ + rd + dr + cd
open(out + '/kin_src.h', 'w').write(common)
mc = open(root + '/Marlin/motion_control.cpp').read()
adv = open(root + '/Marlin/Configuration_adv.h').read()
for inc in ['#include "Marlin.h"\n', '#include "stepper.h"\n', '#include "planner.h"\n', '#include "language.h"\n']:
    mc = mc.replace(inc, '')
consts = ''.join('#define %s %s\n' % (n, re.search(r'#define %s\s+([0-9.]+)' % n, adv).group(1)) for n in ['MM_PER_ARC_SEGMENT', 'N_ARC_CORRECTION', 'ARC_TOLERANCE_MM', 'ARC_MAX_SEGMENT_MM', 'ARC_RADIUS_ERROR_MM'])
arc = common + consts + '''
#define SERIAL_ERROR_START
#define SERIAL_ERRORLNPGM(x) printf("ERROR: %s\\n",x)
#define MSG_ERR_OUT_OF_REACH "Position out of reach of the delta arms, move ignored"
void clamp_to_software_endstops(float*){}
static float last_c[3];
static void calc_rec(float c[3]){ last_c[0]=c[0]; last_c[1]=c[1]; last_c[2]=c[2]; calculate_delta(c); }
#define calculate_delta calc_rec
struct Seg{float c[3],len,e,cart[3];}; std::vector<Seg> segs;
void plan_buffer_line(const float&x,const float&y,const float&z,const float&e,float f,const uint8_t&,float len){ segs.push_back({{x,y,z},len,e,{last_c[0],last_c[1],last_c[2]}}); }
''' + mc
open(out + '/arc_src.h', 'w').write(arc)
