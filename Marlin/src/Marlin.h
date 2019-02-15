/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#ifndef MARLIN_H
#define MARLIN_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

#include "inc/MarlinConfig.h"

#ifdef DEBUG_GCODE_PARSER
  #include "parser.h"
#endif

#include "core/enum.h"
#include "core/types.h"
#include "HAL/fastio.h"
#include "core/utility.h"
#include "core/serial.h"

void idle(
  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    bool no_stepper_sleep = false  // pass true to keep steppers from disabling on timeout
  #endif
);

void manage_inactivity(const bool ignore_stepper_queue=false);

extern const char axis_codes[XYZE];

#if ENABLED(DUAL_X_CARRIAGE) || ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
  extern bool extruder_duplication_enabled;
#endif

#if HAS_X_ENABLE
  #define  enable_X() X_ENABLE_WRITE( X_ENABLE_ON)
  #define disable_X() do{ X_ENABLE_WRITE(!X_ENABLE_ON); CBI(axis_known_position, X_AXIS); }while(0)
#else
  #define  enable_X() NOOP
  #define disable_X() NOOP
#endif

#if HAS_Y_ENABLE
  #define  enable_Y() Y_ENABLE_WRITE( Y_ENABLE_ON)
  #define disable_Y() do{ Y_ENABLE_WRITE(!Y_ENABLE_ON); CBI(axis_known_position, Y_AXIS); }while(0)
#else
  #define  enable_Y() NOOP
  #define disable_Y() NOOP
#endif

#if HAS_Z_ENABLE
  #define  enable_Z() Z_ENABLE_WRITE( Z_ENABLE_ON)
  #define disable_Z() do{ Z_ENABLE_WRITE(!Z_ENABLE_ON); CBI(axis_known_position, Z_AXIS); }while(0)
#else
  #define  enable_Z() NOOP
  #define disable_Z() NOOP
#endif

#if ENABLED(MIXING_EXTRUDER)

  /**
   * Mixing steppers synchronize their enable (and direction) together
   */
  #if MIXING_STEPPERS > 4
    #define  enable_E0() { E0_ENABLE_WRITE( E_ENABLE_ON); E1_ENABLE_WRITE( E_ENABLE_ON); E2_ENABLE_WRITE( E_ENABLE_ON); E3_ENABLE_WRITE( E_ENABLE_ON); E4_ENABLE_WRITE( E_ENABLE_ON); }
    #define disable_E0() { E0_ENABLE_WRITE(!E_ENABLE_ON); E1_ENABLE_WRITE(!E_ENABLE_ON); E2_ENABLE_WRITE(!E_ENABLE_ON); E3_ENABLE_WRITE(!E_ENABLE_ON); E4_ENABLE_WRITE(!E_ENABLE_ON); }
  #elif MIXING_STEPPERS > 3
    #define  enable_E0() { E0_ENABLE_WRITE( E_ENABLE_ON); E1_ENABLE_WRITE( E_ENABLE_ON); E2_ENABLE_WRITE( E_ENABLE_ON); E3_ENABLE_WRITE( E_ENABLE_ON); }
    #define disable_E0() { E0_ENABLE_WRITE(!E_ENABLE_ON); E1_ENABLE_WRITE(!E_ENABLE_ON); E2_ENABLE_WRITE(!E_ENABLE_ON); E3_ENABLE_WRITE(!E_ENABLE_ON); }
  #elif MIXING_STEPPERS > 2
    #define  enable_E0() { E0_ENABLE_WRITE( E_ENABLE_ON); E1_ENABLE_WRITE( E_ENABLE_ON); E2_ENABLE_WRITE( E_ENABLE_ON); }
    #define disable_E0() { E0_ENABLE_WRITE(!E_ENABLE_ON); E1_ENABLE_WRITE(!E_ENABLE_ON); E2_ENABLE_WRITE(!E_ENABLE_ON); }
  #else
    #define  enable_E0() { E0_ENABLE_WRITE( E_ENABLE_ON); E1_ENABLE_WRITE( E_ENABLE_ON); }
    #define disable_E0() { E0_ENABLE_WRITE(!E_ENABLE_ON); E1_ENABLE_WRITE(!E_ENABLE_ON); }
  #endif
  #define  enable_E1() NOOP
  #define disable_E1() NOOP
  #define  enable_E2() NOOP
  #define disable_E2() NOOP
  #define  enable_E3() NOOP
  #define disable_E3() NOOP
  #define  enable_E4() NOOP
  #define disable_E4() NOOP

#else // !MIXING_EXTRUDER

  #if HAS_E0_ENABLE
    #define  enable_E0() E0_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_E0() E0_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_E0() NOOP
    #define disable_E0() NOOP
  #endif

  #if E_STEPPERS > 1 && HAS_E1_ENABLE
    #define  enable_E1() E1_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_E1() E1_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_E1() NOOP
    #define disable_E1() NOOP
  #endif

  #if E_STEPPERS > 2 && HAS_E2_ENABLE
    #define  enable_E2() E2_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_E2() E2_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_E2() NOOP
    #define disable_E2() NOOP
  #endif

  #if E_STEPPERS > 3 && HAS_E3_ENABLE
    #define  enable_E3() E3_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_E3() E3_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_E3() NOOP
    #define disable_E3() NOOP
  #endif

  #if E_STEPPERS > 4 && HAS_E4_ENABLE
    #define  enable_E4() E4_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_E4() E4_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_E4() NOOP
    #define disable_E4() NOOP
  #endif

#endif // !MIXING_EXTRUDER

#if ENABLED(G38_PROBE_TARGET)
  extern bool G38_move,        // flag to tell the interrupt handler that a G38 command is being run
              G38_endstop_hit; // flag from the interrupt handler to indicate if the endstop went active
#endif

void enable_all_steppers();
void disable_e_stepper(const uint8_t e);
void disable_e_steppers();
void disable_all_steppers();

void sync_plan_position();
void sync_plan_position_e();

#define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position()



void kill(const char*);

void quickstop_stepper();

extern uint8_t marlin_debug_flags;
#define DEBUGGING(F) (marlin_debug_flags & (DEBUG_## F))

extern bool Running;
inline bool IsRunning() { return  Running; }
inline bool IsStopped() { return !Running; }





/**
 * Feedrate scaling and conversion
 */
extern float feedrate_mm_s;
extern int16_t feedrate_percentage;

#define MMS_SCALED(MM_S) ((MM_S)*feedrate_percentage*0.01f)



extern uint8_t axis_homed, axis_known_position;

constexpr uint8_t xyz_bits = _BV(X_AXIS) | _BV(Y_AXIS) | _BV(Z_AXIS);
FORCE_INLINE bool all_axes_homed() { return (axis_homed & xyz_bits) == xyz_bits; }
FORCE_INLINE bool all_axes_known() { return (axis_known_position & xyz_bits) == xyz_bits; }

extern volatile bool wait_for_heatup;

#if HAS_RESUME_CONTINUE
  extern volatile bool wait_for_user;
#endif

#if HAS_AUTO_REPORTING || ENABLED(HOST_KEEPALIVE_FEATURE)
  extern bool suspend_auto_report;
#endif

extern float current_position[XYZE], destination[XYZE];

#define NATIVE_TO_LOGICAL(POS, AXIS) (POS)
#define LOGICAL_TO_NATIVE(POS, AXIS) (POS)
#define LOGICAL_X_POSITION(POS) NATIVE_TO_LOGICAL(POS, X_AXIS)
#define LOGICAL_Y_POSITION(POS) NATIVE_TO_LOGICAL(POS, Y_AXIS)
#define LOGICAL_Z_POSITION(POS) NATIVE_TO_LOGICAL(POS, Z_AXIS)
#define RAW_X_POSITION(POS)     LOGICAL_TO_NATIVE(POS, X_AXIS)
#define RAW_Y_POSITION(POS)     LOGICAL_TO_NATIVE(POS, Y_AXIS)
#define RAW_Z_POSITION(POS)     LOGICAL_TO_NATIVE(POS, Z_AXIS)

// Hotend Offsets
#if HOTENDS > 1
  extern float hotend_offset[XYZ][HOTENDS];
#endif

// Software Endstops
extern float soft_endstop_min[XYZ], soft_endstop_max[XYZ];

extern bool soft_endstops_enabled;
void clamp_to_software_endstops(float target[XYZ]);

#define MAX_COORDINATE_SYSTEMS 9

void tool_change(const uint8_t tmp_extruder, const float fr_mm_s=0.0, bool no_move=false);

void home_all_axes();

void report_current_position();

#if ENABLED(G26_MESH_VALIDATION)
  extern bool g26_debug_flag;
#elif ENABLED(AUTO_BED_LEVELING_UBL)
  constexpr bool g26_debug_flag = false;
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #define _GET_MESH_X(I) (bilinear_start[X_AXIS] + (I) * bilinear_grid_spacing[X_AXIS])
  #define _GET_MESH_Y(J) (bilinear_start[Y_AXIS] + (J) * bilinear_grid_spacing[Y_AXIS])
#elif ENABLED(MESH_BED_LEVELING)
  #define _GET_MESH_X(I) mbl.index_to_xpos[I]
  #define _GET_MESH_Y(J) mbl.index_to_ypos[J]
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  extern int bilinear_grid_spacing[2], bilinear_start[2];
  extern float bilinear_grid_factor[2],
               z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
  float bilinear_z_offset(const float raw[XYZ]);
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(MESH_BED_LEVELING)
  typedef float (*element_2d_fn)(const uint8_t, const uint8_t);
  void print_2d_array(const uint8_t sx, const uint8_t sy, const uint8_t precision, const element_2d_fn fn);
#endif

#if HAS_LEVELING
  bool leveling_is_valid();
  void set_bed_leveling_enabled(const bool enable=true);
  void reset_bed_level();
#endif

#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
  void set_z_fade_height(const float zfh, const bool do_report=true);
#endif

#if HAS_BED_PROBE
  extern float zprobe_zoffset;
  bool set_probe_deployed(const bool deploy);
  #ifdef Z_AFTER_PROBING
    void move_z_after_probing();
  #endif
  enum ProbePtRaise : unsigned char {
    PROBE_PT_NONE,  // No raise or stow after run_z_probe
    PROBE_PT_STOW,  // Do a complete stow after run_z_probe
    PROBE_PT_RAISE, // Raise to "between" clearance after run_z_probe
    PROBE_PT_BIG_RAISE  // Raise to big clearance after run_z_probe
  };
  float probe_pt(const float &rx, const float &ry, const ProbePtRaise raise_after=PROBE_PT_NONE, const uint8_t verbose_level=0, const bool probe_relative=true);
  #define DEPLOY_PROBE() set_probe_deployed(true)
  #define STOW_PROBE() set_probe_deployed(false)
#else
  #define DEPLOY_PROBE()
  #define STOW_PROBE()
#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  extern MarlinBusyState busy_state;
  #define KEEPALIVE_STATE(n) do{ busy_state = n; }while(0)
#else
  #define KEEPALIVE_STATE(n) NOOP
#endif

#if ENABLED(FILAMENT_WIDTH_SENSOR)
  extern bool filament_sensor;         // Flag that filament sensor readings should control extrusion
  extern float filament_width_nominal, // Theoretical filament diameter i.e., 3.00 or 1.75
               filament_width_meas;    // Measured filament diameter
  extern uint8_t meas_delay_cm;        // Delay distance
  extern int8_t measurement_delay[MAX_MEASUREMENT_DELAY + 1],  // Ring buffer to delay measurement
                filwidth_delay_index[2]; // Ring buffer indexes. Used by planner, temperature, and main code
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  extern int8_t did_pause_print;
  extern AdvancedPauseMenuResponse advanced_pause_menu_response;
  extern float filament_change_unload_length[EXTRUDERS],
               filament_change_load_length[EXTRUDERS];
#endif

#if HAS_POWER_SWITCH
  extern bool powersupply_on;
  #define PSU_PIN_ON()  do{ OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE); powersupply_on = true; }while(0)
  #define PSU_PIN_OFF() do{ OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP); powersupply_on = false; }while(0)
#endif

// Handling multiple extruders pins
extern uint8_t active_extruder;

#if ENABLED(MIXING_EXTRUDER)
  extern float mixing_factor[MIXING_STEPPERS];
#endif

inline void set_current_from_destination() { COPY(current_position, destination); }
inline void set_destination_from_current() { COPY(destination, current_position); }
void prepare_move_to_destination();

/**
 * Blocking movement and shorthand functions
 */
void do_blocking_move_to(const float rx, const float ry, const float rz, const float &fr_mm_s=0);
void do_blocking_move_to_x(const float &rx, const float &fr_mm_s=0);
void do_blocking_move_to_z(const float &rz, const float &fr_mm_s=0);
void do_blocking_move_to_xy(const float &rx, const float &ry, const float &fr_mm_s=0);

#if ENABLED(ARC_SUPPORT)
  void plan_arc(const float(&cart)[XYZE], const float(&offset)[2], const bool clockwise);
#endif

#define HAS_AXIS_UNHOMED_ERR (                                                     \
         HAS_PROBING_PROCEDURE                                                     \
      || HOTENDS > 1                                                               \
      || ENABLED(NOZZLE_CLEAN_FEATURE)                                             \
      || ENABLED(NOZZLE_PARK_FEATURE)                                              \
      || (ENABLED(ADVANCED_PAUSE_FEATURE) && ENABLED(HOME_BEFORE_FILAMENT_CHANGE)) \
      || HAS_M206_COMMAND                                                          \
    ) || ENABLED(NO_MOTION_BEFORE_HOMING)

#if HAS_AXIS_UNHOMED_ERR
  bool axis_unhomed_error(const bool x=true, const bool y=true, const bool z=true);
#endif

/**
 * position_is_reachable family of functions
 */

// Return true if the given position is within the machine bounds.
inline bool position_is_reachable(const float &rx, const float &ry) {
  // Add 0.001 margin to deal with float imprecision
  return WITHIN(rx, X_MIN_POS - 0.001f, X_MAX_POS + 0.001f)
      && WITHIN(ry, Y_MIN_POS - 0.001f, Y_MAX_POS + 0.001f);
}

#if HAS_BED_PROBE
  /**
   * Return whether the given position is within the bed, and whether the nozzle
   * can reach the position required to put the probe at the given position.
   *
   * Example: For a probe offset of -10,+10, then for the probe to reach 0,0 the
   *          nozzle must be be able to reach +10,-10.
   */
  inline bool position_is_reachable_by_probe(const float &rx, const float &ry) {
    return position_is_reachable(rx - (X_PROBE_OFFSET_FROM_EXTRUDER), ry - (Y_PROBE_OFFSET_FROM_EXTRUDER))
        && WITHIN(rx, MIN_PROBE_X - 0.001f, MAX_PROBE_X + 0.001f)
        && WITHIN(ry, MIN_PROBE_Y - 0.001f, MAX_PROBE_Y + 0.001f);
  }
#else
  FORCE_INLINE bool position_is_reachable_by_probe(const float &rx, const float &ry) { return position_is_reachable(rx, ry); }
#endif

#endif // MARLIN_H
