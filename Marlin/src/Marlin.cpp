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

/**
 * About Marlin
 *
 * This firmware is a mashup between Sprinter and grbl.
 *  - https://github.com/kliment/Sprinter
 *  - https://github.com/grbl/grbl
 */



#include "Marlin.h"

#include "lcd/ultralcd.h"
#include "module/planner.h"
#include "module/stepper.h"
#include "module/endstops.h"
#include "module/temperature.h"
#include "sd/cardreader.h"
#include "module/configuration_store.h"
#include "core/language.h"
#include "pins_arduino.h"
#include "math.h"
#include "feature/nozzle/nozzle.h"
#include "module/printcounter.h"
#include "libs/duration_t.h"
#include "core/types.h"
#include "gcode/queue.h"
#include "gcode/parser.h"

#if ENABLED(AUTO_POWER_CONTROL)
  #include "power.h"
#endif

#if ABL_PLANAR
  #include "vector_3.h"
  #if ENABLED(AUTO_BED_LEVELING_LINEAR)
    #include "least_squares_fit.h"
  #endif
#elif ENABLED(MESH_BED_LEVELING)
  #include "mesh_bed_leveling.h"
#endif

#if ENABLED(BEZIER_CURVE_SUPPORT)
  #include "planner_bezier.h"
#endif

#if ENABLED(FWRETRACT)
  #include "fwretract.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "power_loss_recovery.h"
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #include "feature/runout/runout.h"
#endif

#if HAS_BUZZER && DISABLED(LCD_USE_I2C_BUZZER)
  #include "libs/buzzer.h"
#endif

#if ENABLED(USE_WATCHDOG)
  #include "HAL/watchdog.h"
#endif

#if ENABLED(MAX7219_DEBUG)
  #include "Max7219_Debug_LEDs.h"
#endif

#if HAS_COLOR_LEDS
  #include "leds.h"
#endif

#if HAS_SERVOS
  #include "servo.h"
#endif

#if HAS_DIGIPOTSS
  #include <SPI.h>
#endif

#if HAS_TRINAMIC
  #include "tmc_util.h"
#endif

#if ENABLED(DAC_STEPPER_CURRENT)
  #include "stepper_dac.h"
#endif

#if ENABLED(EXPERIMENTAL_I2CBUS)
  #include "twibus.h"
#endif

#if ENABLED(I2C_POSITION_ENCODERS)
  #include "I2CPositionEncoder.h"
#endif

#if ENABLED(M100_FREE_MEMORY_WATCHER)
  void gcode_M100();
  void M100_dump_routine(const char * const title, const char *start, const char *end);
#endif

#if ENABLED(G26_MESH_VALIDATION)
  bool g26_debug_flag; // =false
  void gcode_G26();
#endif

#if ENABLED(SDSUPPORT)
  CardReader card;
#endif

#if ENABLED(EXPERIMENTAL_I2CBUS)
  TWIBus i2c;
#endif

#if ENABLED(G38_PROBE_TARGET)
  bool G38_move = false,
       G38_endstop_hit = false;
#endif





bool Running = true;

uint8_t marlin_debug_flags = DEBUG_NONE;

/**
 * Cartesian Current Position
 *   Used to track the native machine position as moves are queued.
 *   Used by 'buffer_line_to_current_position' to do a move after changing it.
 *   Used by 'SYNC_PLAN_POSITION_KINEMATIC' to update 'planner.position'.
 */
float current_position[XYZE] = { 0 };

/**
 * Cartesian Destination
 *   The destination for a move, filled in by G-code movement commands,
 *   and expected by functions like 'prepare_move_to_destination'.
 *   Set with 'gcode_get_destination' or 'set_destination_from_current'.
 */
float destination[XYZE] = { 0 };

/**
 * axis_homed
 *   Flags that each linear axis was homed.
 *   XYZ on cartesian, ABC on delta, ABZ on SCARA.
 *
 * axis_known_position
 *   Flags that the position is known in each linear axis. Set when homed.
 *   Cleared whenever a stepper powers off, potentially losing its position.
 */
uint8_t axis_homed, axis_known_position; // = 0









/**
 * Feed rates are often configured with mm/m
 * but the planner and stepper like mm/s units.
 */
static const float homing_feedrate_mm_s[] PROGMEM = {
  MMM_TO_MMS(HOMING_FEEDRATE_XY), MMM_TO_MMS(HOMING_FEEDRATE_XY),
  MMM_TO_MMS(HOMING_FEEDRATE_Z), 0
};
FORCE_INLINE float homing_feedrate(const AxisEnum a) { return pgm_read_float(&homing_feedrate_mm_s[a]); }

float feedrate_mm_s = MMM_TO_MMS(1500.0f);
static float saved_feedrate_mm_s;
int16_t feedrate_percentage = 100, saved_feedrate_percentage;

// Initialized by settings.load()
bool axis_relative_modes[XYZE] = AXIS_RELATIVE_MODES;



// Software Endstops are based on the configured limits.
float soft_endstop_min[XYZ] = { X_MIN_BED, Y_MIN_BED, Z_MIN_POS },
      soft_endstop_max[XYZ] = { X_MAX_BED, Y_MAX_BED, Z_MAX_POS };

bool soft_endstops_enabled = true;



// The active extruder (tool). Set with T<extruder> command.
uint8_t active_extruder; // = 0;

// Relative Mode. Enable with G91, disable with G90.
static bool relative_mode; // = false;

// For M109 and M190, this flag may be cleared (by M108) to exit the wait loop
volatile bool wait_for_heatup = true;

// For M0/M1, this flag may be cleared (by M108) to exit the wait-for-user loop
#if HAS_RESUME_CONTINUE
  volatile bool wait_for_user; // = false;
#endif

#if HAS_AUTO_REPORTING || ENABLED(HOST_KEEPALIVE_FEATURE)
  bool suspend_auto_report; // = false
#endif

const char axis_codes[XYZE] = { 'X', 'Y', 'Z', 'E' };

#define RAW_AXIS_CODES(I) axis_codes[I]



// Inactivity shutdown
millis_t previous_move_ms; // = 0;
static millis_t max_inactive_time; // = 0;
static millis_t stepper_inactive_time = (DEFAULT_STEPPER_DEACTIVE_TIME) * 1000UL;

// Buzzer - I2C on the LCD or a BEEPER_PIN
#if ENABLED(LCD_USE_I2C_BUZZER)
  #define BUZZ(d,f) lcd_buzz(d, f)
#elif PIN_EXISTS(BEEPER)
  Buzzer buzzer;
  #define BUZZ(d,f) buzzer.tone(d, f)
#else
  #define BUZZ(d,f) NOOP
#endif

uint8_t target_extruder;

#if HAS_BED_PROBE
  float zprobe_zoffset; // Initialized by settings.load()
#endif

#if HAS_ABL
  float xy_probe_feedrate_mm_s = MMM_TO_MMS(XY_PROBE_SPEED);
  #define XY_PROBE_FEEDRATE_MM_S xy_probe_feedrate_mm_s
#elif defined(XY_PROBE_SPEED)
  #define XY_PROBE_FEEDRATE_MM_S MMM_TO_MMS(XY_PROBE_SPEED)
#else
  #define XY_PROBE_FEEDRATE_MM_S PLANNER_XY_FEEDRATE()
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #define ADJUST_DELTA(V) if (planner.leveling_active) { delta[Z_AXIS] += bilinear_z_offset(V); }
#endif

#if HAS_HEATED_BED && ENABLED(WAIT_FOR_BED_HEATER)
  const static char msg_wait_for_bed_heating[] PROGMEM = "Wait for bed heating...\n";
#endif

// Extruder offsets
#if HOTENDS > 1
  float hotend_offset[XYZ][HOTENDS];  // Initialized by settings.load()
#endif





#if HAS_POWER_SWITCH
  bool powersupply_on = (
    #if ENABLED(PS_DEFAULT_OFF)
      false
    #else
      true
    #endif
  );
  #if ENABLED(AUTO_POWER_CONTROL)
    #define PSU_ON()  powerManager.power_on()
    #define PSU_OFF() powerManager.power_off()
  #else
    #define PSU_ON()  PSU_PIN_ON()
    #define PSU_OFF() PSU_PIN_OFF()
  #endif
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  int bilinear_grid_spacing[2], bilinear_start[2];
  float bilinear_grid_factor[2],
        z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
  #if ENABLED(ABL_BILINEAR_SUBDIVISION)
    #define ABL_BG_SPACING(A) bilinear_grid_spacing_virt[A]
    #define ABL_BG_FACTOR(A)  bilinear_grid_factor_virt[A]
    #define ABL_BG_POINTS_X   ABL_GRID_POINTS_VIRT_X
    #define ABL_BG_POINTS_Y   ABL_GRID_POINTS_VIRT_Y
    #define ABL_BG_GRID(X,Y)  z_values_virt[X][Y]
  #else
    #define ABL_BG_SPACING(A) bilinear_grid_spacing[A]
    #define ABL_BG_FACTOR(A)  bilinear_grid_factor[A]
    #define ABL_BG_POINTS_X   GRID_MAX_POINTS_X
    #define ABL_BG_POINTS_Y   GRID_MAX_POINTS_Y
    #define ABL_BG_GRID(X,Y)  z_values[X][Y]
  #endif
#endif



float cartes[XYZ] = { 0 };

#if ENABLED(FILAMENT_WIDTH_SENSOR)
  bool filament_sensor; // = false;                             // M405 turns on filament sensor control. M406 turns it off.
  float filament_width_nominal = DEFAULT_NOMINAL_FILAMENT_DIA,  // Nominal filament width. Change with M404.
        filament_width_meas = DEFAULT_MEASURED_FILAMENT_DIA;    // Measured filament diameter
  uint8_t meas_delay_cm = MEASUREMENT_DELAY_CM;                 // Distance delay setting
  int8_t measurement_delay[MAX_MEASUREMENT_DELAY + 1],          // Ring buffer to delayed measurement. Store extruder factor after subtracting 100
         filwidth_delay_index[2] = { 0, -1 };                   // Indexes into ring buffer
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  AdvancedPauseMenuResponse advanced_pause_menu_response;
  float filament_change_unload_length[EXTRUDERS],
        filament_change_load_length[EXTRUDERS];
#endif

#if ENABLED(MIXING_EXTRUDER)
  float mixing_factor[MIXING_STEPPERS]; // Reciprocal of mix proportion. 0.0 = off, otherwise >= 1.0.
  #if MIXING_VIRTUAL_TOOLS > 1
    float mixing_virtual_tool_mix[MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS];
  #endif
#endif





#ifdef CHDK
  millis_t chdkHigh = 0;
  bool chdkActive = false;
#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  MarlinBusyState busy_state = NOT_BUSY;
  static millis_t next_busy_signal_ms = 0;
  uint8_t host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;
#else
  #define host_keepalive() NOOP
#endif

#if ENABLED(I2C_POSITION_ENCODERS)
  I2CPositionEncodersMgr I2CPEM;
#endif



FORCE_INLINE float pgm_read_any(const float *p) { return pgm_read_float_near(p); }
FORCE_INLINE signed char pgm_read_any(const signed char *p) { return pgm_read_byte_near(p); }

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
  static const PROGMEM type array##_P[XYZ] = { X_##CONFIG, Y_##CONFIG, Z_##CONFIG }; \
  static inline type array(const AxisEnum axis) { return pgm_read_any(&array##_P[axis]); } \
  typedef void __void_##CONFIG##__

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,   MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,   MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,  HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,     MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm,   HOME_BUMP_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir, HOME_DIR);

/**
 * ***************************************************************************
 * ******************************** FUNCTIONS ********************************
 * ***************************************************************************
 */

void stop();

/**
   * Output a "busy" message at regular intervals
   * while the machine is not accepting commands.
   */
  void host_keepalive() {
    const millis_t ms = millis();
    if (!suspend_auto_report && host_keepalive_interval && busy_state != NOT_BUSY) {
      if (PENDING(ms, next_busy_signal_ms)) return;
      switch (busy_state) {
        case IN_HANDLER:
        case IN_PROCESS:
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPGM(MSG_BUSY_PROCESSING);
          break;
        case PAUSED_FOR_USER:
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_USER);
          break;
        case PAUSED_FOR_INPUT:
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_INPUT);
          break;
        default:
          break;
      }
    }
    next_busy_signal_ms = ms + host_keepalive_interval * 1000UL;
  }

#if HAS_CASE_LIGHT

  #ifndef INVERT_CASE_LIGHT
    #define INVERT_CASE_LIGHT false
  #endif
  uint8_t case_light_brightness;  // LCD routine wants INT
  bool case_light_on;

  #if ENABLED(CASE_LIGHT_USE_NEOPIXEL)
    LEDColor case_light_color =
      #ifdef CASE_LIGHT_NEOPIXEL_COLOR
        CASE_LIGHT_NEOPIXEL_COLOR
      #else
        { 255, 255, 255, 255 }
      #endif
    ;
  #endif

  void update_case_light() {
    const uint8_t i = case_light_on ? case_light_brightness : 0, n10ct = INVERT_CASE_LIGHT ? 255 - i : i;

    #if ENABLED(CASE_LIGHT_USE_NEOPIXEL)

      leds.set_color(
        MakeLEDColor(case_light_color.r, case_light_color.g, case_light_color.b, case_light_color.w, n10ct),
        false
      );

    #else // !CASE_LIGHT_USE_NEOPIXEL

      SET_OUTPUT(CASE_LIGHT_PIN);
      if (USEABLE_HARDWARE_PWM(CASE_LIGHT_PIN))
        analogWrite(CASE_LIGHT_PIN, n10ct);
      else {
        const bool s = case_light_on ? !INVERT_CASE_LIGHT : INVERT_CASE_LIGHT;
        WRITE(CASE_LIGHT_PIN, s ? HIGH : LOW);
      }

    #endif // !CASE_LIGHT_USE_NEOPIXEL
  }
#endif // HAS_CASE_LIGHT
  
void get_cartesian_from_steppers();
void set_current_from_steppers_for_axis(const AxisEnum axis);

#if ENABLED(ARC_SUPPORT)
  void plan_arc(const float (&cart)[XYZE], const float (&offset)[2], const bool clockwise);
#endif

#if ENABLED(BEZIER_CURVE_SUPPORT)
  void plan_cubic_move(const float (&cart)[XYZE], const float (&offset)[4]);
#endif

void report_current_position();
void report_current_position_detail();

#if ENABLED(DEBUG_LEVELING_FEATURE)
  void print_xyz(const char* prefix, const char* suffix, const float x, const float y, const float z) {
    serialprintPGM(prefix);
    SERIAL_CHAR('(');
    SERIAL_ECHO(x);
    SERIAL_ECHOPAIR(", ", y);
    SERIAL_ECHOPAIR(", ", z);
    SERIAL_CHAR(')');
    if (suffix) serialprintPGM(suffix); else SERIAL_EOL();
  }

  void print_xyz(const char* prefix, const char* suffix, const float xyz[]) {
    print_xyz(prefix, suffix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS]);
  }

  #define DEBUG_POS(SUFFIX,VAR) do { \
    print_xyz(PSTR("  " STRINGIFY(VAR) "="), PSTR(" : " SUFFIX "\n"), VAR); }while(0)
#endif

/**
 * sync_plan_position
 *
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 *
 * This is not possible for Hangprinter because current_position and position are different sizes
 */
void sync_plan_position() {
  #if DISABLED(HANGPRINTER)
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("sync_plan_position", current_position);
    #endif
    planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_CART]);
  #endif
}
void sync_plan_position_e() { planner.set_e_position_mm(current_position[E_CART]); }



#if ENABLED(SDSUPPORT)
  #include "sd/SdFatUtil.h"
  int freeMemory() { return SdFatUtil::FreeRam(); }
#else
extern "C" {
  extern char __bss_end;
  extern char __heap_start;
  extern void* __brkval;

  int freeMemory() {
    int free_memory;
    if (int(__brkval) == 0)
      free_memory = (int(&free_memory)) - (int(&__bss_end));
    else
      free_memory = (int(&free_memory)) - (int(__brkval));
    return free_memory;
  }
}
#endif // !SDSUPPORT

#if ENABLED(DIGIPOT_I2C)
  extern void digipot_i2c_set_current(uint8_t channel, float current);
  extern void digipot_i2c_init();
#endif







void setup_killpin() {
  #if HAS_KILL
    SET_INPUT_PULLUP(KILL_PIN);
  #endif
}

void setup_powerhold() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if HAS_POWER_SWITCH
    #if ENABLED(PS_DEFAULT_OFF)
      PSU_OFF();
    #else
      PSU_ON();
    #endif
  #endif
}

void suicide() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, LOW);
  #endif
}



/**
 * Stepper Reset (RigidBoard, et.al.)
 */
#if HAS_STEPPER_RESET
  void disableStepperDrivers() {
    OUT_WRITE(STEPPER_RESET_PIN, LOW);  // drive it down to hold in reset motor driver chips
  }
  void enableStepperDrivers() { SET_INPUT(STEPPER_RESET_PIN); }  // set to input, which allows it to be pulled high by pullups
#endif

#if ENABLED(EXPERIMENTAL_I2CBUS) && I2C_SLAVE_ADDRESS > 0

  void i2c_on_receive(int bytes) { // just echo all bytes received to serial
    i2c.receive(bytes);
  }

  void i2c_on_request() {          // just send dummy data for now
    i2c.reply("Hello World!\n");
  }

#endif









#if ENABLED(DUAL_X_CARRIAGE) || ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
  bool extruder_duplication_enabled = false; // Used in Dual X mode 2
#endif







/**
 * Set an axis' current position to its home position (after homing).
 *
 * For Core and Cartesian robots this applies one-to-one when an
 * individual axis has been homed.
 *
 * DELTA should wait until all homing is done before setting the XYZ
 * current_position to home, because homing is a single operation.
 * In the case where the axis positions are already known and previously
 * homed, DELTA could home to X or Y individually by moving either one
 * to the center. However, homing Z always homes XY and Z.
 *
 * SCARA should wait until all XY homing is done before setting the XY
 * current_position to home, because neither X nor Y is at home until
 * both are at home. Z can however be homed individually.
 *
 * Callers must sync the planner position after calling this!
 */
static void set_axis_is_at_home(const AxisEnum axis) {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR(">>> set_axis_is_at_home(", axis_codes[axis]);
      SERIAL_CHAR(')');
      SERIAL_EOL();
    }
  #endif

  SBI(axis_known_position, axis);
  SBI(axis_homed, axis);

  #if HAS_POSITION_SHIFT
    position_shift[axis] = 0;
    update_software_endstops(axis);
  #endif

    current_position[axis] = base_home_pos(axis);


  /**
   * Z Probe Z Homing? Account for the probe's Z offset.
   */
  #if HAS_BED_PROBE && Z_HOME_DIR < 0
    if (axis == Z_AXIS) {
      #if HOMING_Z_WITH_PROBE

        current_position[Z_AXIS] -= zprobe_zoffset;

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_ECHOLNPGM("*** Z HOMED WITH PROBE (Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN) ***");
            SERIAL_ECHOLNPAIR("> zprobe_zoffset = ", zprobe_zoffset);
          }
        #endif

      #elif ENABLED(DEBUG_LEVELING_FEATURE)

        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("*** Z HOMED TO ENDSTOP (Z_MIN_PROBE_ENDSTOP) ***");

      #endif
    }
  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      #if HAS_HOME_OFFSET
        SERIAL_ECHOPAIR("> home_offset[", axis_codes[axis]);
        SERIAL_ECHOLNPAIR("] = ", home_offset[axis]);
      #endif
      DEBUG_POS("", current_position);
      SERIAL_ECHOPAIR("<<< set_axis_is_at_home(", axis_codes[axis]);
      SERIAL_CHAR(')');
      SERIAL_EOL();
    }
  #endif

  #if ENABLED(I2C_POSITION_ENCODERS)
    I2CPEM.homed(axis);
  #endif
}

/**
 * Homing bump feedrate (mm/s)
 */
inline float get_homing_bump_feedrate(const AxisEnum axis) {
  #if HOMING_Z_WITH_PROBE
    if (axis == Z_AXIS) return MMM_TO_MMS(Z_PROBE_SPEED_SLOW);
  #endif
  static const uint8_t homing_bump_divisor[] PROGMEM = HOMING_BUMP_DIVISOR;
  uint8_t hbd = pgm_read_byte(&homing_bump_divisor[axis]);
  if (hbd < 1) {
    hbd = 10;
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM("Warning: Homing Bump Divisor < 1");
  }
  return homing_feedrate(axis) / hbd;
}

/**
 * Some planner shorthand inline functions
 */

/**
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 *
 * Impossible on Hangprinter because current_position and position are of different sizes
 */
inline void buffer_line_to_current_position() {
  #if DISABLED(HANGPRINTER) // emptying this function probably breaks do_blocking_move_to()
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_CART], feedrate_mm_s, active_extruder);
  #endif
}

/**
 * Move the planner to the position stored in the destination array, which is
 * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
 */
inline void buffer_line_to_destination(const float &fr_mm_s) {
  #if ENABLED(HANGPRINTER)
    UNUSED(fr_mm_s);
  #else
    planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_CART], fr_mm_s, active_extruder);
  #endif
}



/**
 * Plan a move to (X, Y, Z) and set the current_position.
 * The final current_position may not be the one that was requested
 * Caution: 'destination' is modified by this function.
 */
void do_blocking_move_to(const float rx, const float ry, const float rz, const float &fr_mm_s/*=0.0*/) {
  const float old_feedrate_mm_s = feedrate_mm_s;

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) print_xyz(PSTR(">>> do_blocking_move_to"), NULL, LOGICAL_X_POSITION(rx), LOGICAL_Y_POSITION(ry), LOGICAL_Z_POSITION(rz));
  #endif

  const float z_feedrate = fr_mm_s ? fr_mm_s : homing_feedrate(Z_AXIS);

	// If Z needs to raise, do it before moving XY
	if (current_position[Z_AXIS] < rz) {
		feedrate_mm_s = z_feedrate;
		current_position[Z_AXIS] = rz;
		buffer_line_to_current_position();
	}

	feedrate_mm_s = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;
	current_position[X_AXIS] = rx;
	current_position[Y_AXIS] = ry;
	buffer_line_to_current_position();

	// If Z needs to lower, do it after moving XY
	if (current_position[Z_AXIS] > rz) {
		feedrate_mm_s = z_feedrate;
		current_position[Z_AXIS] = rz;
		buffer_line_to_current_position();
	}

  planner.synchronize();

  feedrate_mm_s = old_feedrate_mm_s;

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< do_blocking_move_to");
  #endif
}
void do_blocking_move_to_x(const float &rx, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(rx, current_position[Y_AXIS], current_position[Z_AXIS], fr_mm_s);
}
void do_blocking_move_to_z(const float &rz, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], rz, fr_mm_s);
}
void do_blocking_move_to_xy(const float &rx, const float &ry, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(rx, ry, current_position[Z_AXIS], fr_mm_s);
}

//
// Prepare to do endstop or probe moves
// with custom feedrates.
//
//  - Save current feedrates
//  - Reset the rate multiplier
//  - Reset the command timeout
//  - Enable the endstops (for endstop moves)
//
void setup_for_endstop_or_probe_move() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("setup_for_endstop_or_probe_move", current_position);
  #endif
  saved_feedrate_mm_s = feedrate_mm_s;
  saved_feedrate_percentage = feedrate_percentage;
  feedrate_percentage = 100;
}

void clean_up_after_endstop_or_probe_move() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("clean_up_after_endstop_or_probe_move", current_position);
  #endif
  feedrate_mm_s = saved_feedrate_mm_s;
  feedrate_percentage = saved_feedrate_percentage;
}

#if HAS_AXIS_UNHOMED_ERR

  bool axis_unhomed_error(const bool x/*=true*/, const bool y/*=true*/, const bool z/*=true*/) {
    #if ENABLED(HOME_AFTER_DEACTIVATE)
      const bool xx = x && !TEST(axis_known_position, X_AXIS),
                 yy = y && !TEST(axis_known_position, Y_AXIS),
                 zz = z && !TEST(axis_known_position, Z_AXIS);
    #else
      const bool xx = x && !TEST(axis_homed, X_AXIS),
                 yy = y && !TEST(axis_homed, Y_AXIS),
                 zz = z && !TEST(axis_homed, Z_AXIS);
    #endif
    if (xx || yy || zz) {
      SERIAL_ECHO_START();
      SERIAL_ECHOPGM(MSG_HOME " ");
      if (xx) SERIAL_ECHOPGM(MSG_X);
      if (yy) SERIAL_ECHOPGM(MSG_Y);
      if (zz) SERIAL_ECHOPGM(MSG_Z);
      SERIAL_ECHOLNPGM(" " MSG_FIRST);

      #if ENABLED(ULTRA_LCD)
        lcd_status_printf_P(0, PSTR(MSG_HOME " %s%s%s " MSG_FIRST), xx ? MSG_X : "", yy ? MSG_Y : "", zz ? MSG_Z : "");
      #endif
      return true;
    }
    return false;
  }

#endif // HAS_AXIS_UNHOMED_ERR



#if HAS_BED_PROBE

  // TRIGGERED_WHEN_STOWED_TEST can easily be extended to servo probes, ... if needed.
  #if ENABLED(PROBE_IS_TRIGGERED_WHEN_STOWED_TEST)
    #if ENABLED(Z_MIN_PROBE_ENDSTOP)
      #define _TRIGGERED_WHEN_STOWED_TEST (READ(Z_MIN_PROBE_PIN) != Z_MIN_PROBE_ENDSTOP_INVERTING)
    #else
      #define _TRIGGERED_WHEN_STOWED_TEST (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING)
    #endif
  #endif

  #if QUIET_PROBING
    void probing_pause(const bool p) {
      #if ENABLED(PROBING_HEATERS_OFF)
        thermalManager.pause(p);
      #endif
      #if ENABLED(PROBING_FANS_OFF)
        thermalManager.pause_fans(p);
      #endif
      if (p) safe_delay(
        #if DELAY_BEFORE_PROBING > 25
          DELAY_BEFORE_PROBING
        #else
          25
        #endif
      );
    }
  #endif // QUIET_PROBING

  #if ENABLED(BLTOUCH)

    void bltouch_command(int angle) {
      MOVE_SERVO(Z_PROBE_SERVO_NR, angle);  // Give the BL-Touch the command and wait
      safe_delay(BLTOUCH_DELAY);
    }

    bool set_bltouch_deployed(const bool deploy) {
      if (deploy && TEST_BLTOUCH()) {      // If BL-Touch says it's triggered
        bltouch_command(BLTOUCH_RESET);    //  try to reset it.
        bltouch_command(BLTOUCH_DEPLOY);   // Also needs to deploy and stow to
        bltouch_command(BLTOUCH_STOW);     //  clear the triggered condition.
        safe_delay(1500);                  // Wait for internal self-test to complete.
                                           //  (Measured completion time was 0.65 seconds
                                           //   after reset, deploy, and stow sequence)
        if (TEST_BLTOUCH()) {              // If it still claims to be triggered...
          SERIAL_ERROR_START();
          SERIAL_ERRORLNPGM(MSG_STOP_BLTOUCH);
          stop();                          // punt!
          return true;
        }
      }

      bltouch_command(deploy ? BLTOUCH_DEPLOY : BLTOUCH_STOW);

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) {
          SERIAL_ECHOPAIR("set_bltouch_deployed(", deploy);
          SERIAL_CHAR(')');
          SERIAL_EOL();
        }
      #endif

      return false;
    }

  #endif // BLTOUCH

  /**
   * Raise Z to a minimum height to make room for a probe to move
   */
  inline void do_probe_raise(const float z_raise) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOPAIR("do_probe_raise(", z_raise);
        SERIAL_CHAR(')');
        SERIAL_EOL();
      }
    #endif

    float z_dest = z_raise;
    if (zprobe_zoffset < 0) z_dest -= zprobe_zoffset;

    NOMORE(z_dest, Z_MAX_POS);

    if (z_dest > current_position[Z_AXIS])
      do_blocking_move_to_z(z_dest);
  }

  // returns false for ok and true for failure
  bool set_probe_deployed(const bool deploy) {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        DEBUG_POS("set_probe_deployed", current_position);
        SERIAL_ECHOLNPAIR("deploy: ", deploy);
      }
    #endif

    if (endstops.z_probe_enabled == deploy) return false;

    // Make room for probe to deploy (or stow)
    // Fix-mounted probe should only raise for deploy
    #if ENABLED(FIX_MOUNTED_PROBE)
      const bool deploy_stow_condition = deploy;
    #else
      constexpr bool deploy_stow_condition = true;
    #endif

    // For beds that fall when Z is powered off only raise for trusted Z
    #if ENABLED(UNKNOWN_Z_NO_RAISE)
      const bool unknown_condition = TEST(axis_known_position, Z_AXIS);
    #else
      constexpr float unknown_condition = true;
    #endif

    if (deploy_stow_condition && unknown_condition)
      do_probe_raise(MAX(Z_CLEARANCE_BETWEEN_PROBES, Z_CLEARANCE_DEPLOY_PROBE));

    #if ENABLED(Z_PROBE_SLED) || ENABLED(Z_PROBE_ALLEN_KEY)
      #if ENABLED(Z_PROBE_SLED)
        #define _AUE_ARGS true, false, false
      #else
        #define _AUE_ARGS
      #endif
      if (axis_unhomed_error(_AUE_ARGS)) {
        SERIAL_ERROR_START();
        SERIAL_ERRORLNPGM(MSG_STOP_UNHOMED);
        stop();
        return true;
      }
    #endif

    const float oldXpos = current_position[X_AXIS],
                oldYpos = current_position[Y_AXIS];

    #ifdef _TRIGGERED_WHEN_STOWED_TEST

      // If endstop is already false, the Z probe is deployed
      if (_TRIGGERED_WHEN_STOWED_TEST == deploy) {     // closed after the probe specific actions.
                                                       // Would a goto be less ugly?
        //while (!_TRIGGERED_WHEN_STOWED_TEST) idle(); // would offer the opportunity
                                                       // for a triggered when stowed manual probe.

        if (!deploy) endstops.enable_z_probe(false); // Switch off triggered when stowed probes early
                                                     // otherwise an Allen-Key probe can't be stowed.
    #endif

        #if ENABLED(SOLENOID_PROBE)

          #if HAS_SOLENOID_1
            WRITE(SOL1_PIN, deploy);
          #endif

        #elif ENABLED(Z_PROBE_SLED)

          dock_sled(!deploy);

        #elif HAS_Z_SERVO_PROBE && DISABLED(BLTOUCH)

          MOVE_SERVO(Z_PROBE_SERVO_NR, z_servo_angle[deploy ? 0 : 1]);

        #elif ENABLED(Z_PROBE_ALLEN_KEY)

          deploy ? run_deploy_moves_script() : run_stow_moves_script();

        #endif

    #ifdef _TRIGGERED_WHEN_STOWED_TEST
      } // _TRIGGERED_WHEN_STOWED_TEST == deploy

      if (_TRIGGERED_WHEN_STOWED_TEST == deploy) { // State hasn't changed?

        if (IsRunning()) {
          SERIAL_ERROR_START();
          SERIAL_ERRORLNPGM("Z-Probe failed");
          LCD_ALERTMESSAGEPGM("Err: ZPROBE");
        }
        stop();
        return true;

      } // _TRIGGERED_WHEN_STOWED_TEST == deploy

    #endif

    do_blocking_move_to(oldXpos, oldYpos, current_position[Z_AXIS]); // return to position before deploy
    endstops.enable_z_probe(deploy);
    return false;
  }

  /**
   * @brief Used by run_z_probe to do a single Z probe move.
   *
   * @param  z        Z destination
   * @param  fr_mm_s  Feedrate in mm/s
   * @return true to indicate an error
   */
  static bool do_probe_move(const float z, const float fr_mm_s) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS(">>> do_probe_move", current_position);
    #endif

    #if HAS_HEATED_BED && ENABLED(WAIT_FOR_BED_HEATER)
      // Wait for bed to heat back up between probing points
      if (thermalManager.isHeatingBed()) {
        serialprintPGM(msg_wait_for_bed_heating);
        LCD_MESSAGEPGM(MSG_BED_HEATING);
        while (thermalManager.isHeatingBed()) safe_delay(200);
        lcd_reset_status();
      }
    #endif

    // Deploy BLTouch at the start of any probe
    #if ENABLED(BLTOUCH)
      if (set_bltouch_deployed(true)) return true;
    #endif

    #if QUIET_PROBING
      probing_pause(true);
    #endif

    // Move down until probe triggered
    do_blocking_move_to_z(z, fr_mm_s);

    // Check to see if the probe was triggered
    const bool probe_triggered = TEST(endstops.trigger_state(),
      #if ENABLED(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN)
        Z_MIN
      #else
        Z_MIN_PROBE
      #endif
    );

    #if QUIET_PROBING
      probing_pause(false);
    #endif

    // Retract BLTouch immediately after a probe if it was triggered
    #if ENABLED(BLTOUCH)
      if (probe_triggered && set_bltouch_deployed(false)) return true;
    #endif

    endstops.hit_on_purpose();

    // Get Z where the steppers were interrupted
    set_current_from_steppers_for_axis(Z_AXIS);

    // Tell the planner where we actually are
    SYNC_PLAN_POSITION_KINEMATIC();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("<<< do_probe_move", current_position);
    #endif

    return !probe_triggered;
  }

  /**
   * @details Used by probe_pt to do a single Z probe at the current position.
   *          Leaves current_position[Z_AXIS] at the height where the probe triggered.
   *
   * @return The raw Z position where the probe was triggered
   */
  static float run_z_probe() {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS(">>> run_z_probe", current_position);
    #endif

    // Stop the probe before it goes too low to prevent damage.
    // If Z isn't known then probe to -10mm.
    const float z_probe_low_point = TEST(axis_known_position, Z_AXIS) ? -zprobe_zoffset + Z_PROBE_LOW_POINT : -10.0;

    // Double-probing does a fast probe followed by a slow probe
    #if MULTIPLE_PROBING == 2

      // Do a first probe at the fast speed
      if (do_probe_move(z_probe_low_point, MMM_TO_MMS(Z_PROBE_SPEED_FAST))) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_ECHOLNPGM("FAST Probe fail!");
            DEBUG_POS("<<< run_z_probe", current_position);
          }
        #endif
        return NAN;
      }

      float first_probe_z = current_position[Z_AXIS];

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPAIR("1st Probe Z:", first_probe_z);
      #endif

      // move up to make clearance for the probe
      do_blocking_move_to_z(current_position[Z_AXIS] + Z_CLEARANCE_MULTI_PROBE, MMM_TO_MMS(Z_PROBE_SPEED_FAST));

    #else

      // If the nozzle is well over the travel height then
      // move down quickly before doing the slow probe
      float z = Z_CLEARANCE_DEPLOY_PROBE + 5.0;
      if (zprobe_zoffset < 0) z -= zprobe_zoffset;

      if (current_position[Z_AXIS] > z) {
        // If we don't make it to the z position (i.e. the probe triggered), move up to make clearance for the probe
        if (!do_probe_move(z, MMM_TO_MMS(Z_PROBE_SPEED_FAST)))
          do_blocking_move_to_z(current_position[Z_AXIS] + Z_CLEARANCE_BETWEEN_PROBES, MMM_TO_MMS(Z_PROBE_SPEED_FAST));
      }
    #endif

    #if MULTIPLE_PROBING > 2
      float probes_total = 0;
      for (uint8_t p = MULTIPLE_PROBING + 1; --p;) {
    #endif

        // move down slowly to find bed
        if (do_probe_move(z_probe_low_point, MMM_TO_MMS(Z_PROBE_SPEED_SLOW))) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOLNPGM("SLOW Probe fail!");
              DEBUG_POS("<<< run_z_probe", current_position);
            }
          #endif
          return NAN;
        }

    #if MULTIPLE_PROBING > 2
        probes_total += current_position[Z_AXIS];
        if (p > 1) do_blocking_move_to_z(current_position[Z_AXIS] + Z_CLEARANCE_MULTI_PROBE, MMM_TO_MMS(Z_PROBE_SPEED_FAST));
      }
    #endif

    #if MULTIPLE_PROBING > 2

      // Return the average value of all probes
      const float measured_z = probes_total * (1.0f / (MULTIPLE_PROBING));

    #elif MULTIPLE_PROBING == 2

      const float z2 = current_position[Z_AXIS];

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) {
          SERIAL_ECHOPAIR("2nd Probe Z:", z2);
          SERIAL_ECHOLNPAIR(" Discrepancy:", first_probe_z - z2);
        }
      #endif

      // Return a weighted average of the fast and slow probes
      const float measured_z = (z2 * 3.0 + first_probe_z * 2.0) * 0.2;

    #else

      // Return the single probe result
      const float measured_z = current_position[Z_AXIS];

    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("<<< run_z_probe", current_position);
    #endif

    return measured_z;
  }

  /**
   * - Move to the given XY
   * - Deploy the probe, if not already deployed
   * - Probe the bed, get the Z position
   * - Depending on the 'stow' flag
   *   - Stow the probe, or
   *   - Raise to the BETWEEN height
   * - Return the probed Z position
   */
  float probe_pt(const float &rx, const float &ry, const ProbePtRaise raise_after/*=PROBE_PT_NONE*/, const uint8_t verbose_level/*=0*/, const bool probe_relative/*=true*/) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOPAIR(">>> probe_pt(", LOGICAL_X_POSITION(rx));
        SERIAL_ECHOPAIR(", ", LOGICAL_Y_POSITION(ry));
        SERIAL_ECHOPAIR(", ", raise_after == PROBE_PT_RAISE ? "raise" : raise_after == PROBE_PT_STOW ? "stow" : "none");
        SERIAL_ECHOPAIR(", ", int(verbose_level));
        SERIAL_ECHOPAIR(", ", probe_relative ? "probe" : "nozzle");
        SERIAL_ECHOLNPGM("_relative)");
        DEBUG_POS("", current_position);
      }
    #endif

    // TODO: Adapt for SCARA, where the offset rotates
    float nx = rx, ny = ry;
    if (probe_relative) {
      if (!position_is_reachable_by_probe(rx, ry)) return NAN;  // The given position is in terms of the probe
      nx -= (X_PROBE_OFFSET_FROM_EXTRUDER);                     // Get the nozzle position
      ny -= (Y_PROBE_OFFSET_FROM_EXTRUDER);
    }
    else if (!position_is_reachable(nx, ny)) return NAN;        // The given position is in terms of the nozzle

    const float nz =
      #if ENABLED(DELTA)
        // Move below clip height or xy move will be aborted by do_blocking_move_to
        MIN(current_position[Z_AXIS], delta_clip_start_height)
      #else
        current_position[Z_AXIS]
      #endif
    ;

    const float old_feedrate_mm_s = feedrate_mm_s;
    feedrate_mm_s = XY_PROBE_FEEDRATE_MM_S;

    // Move the probe to the starting XYZ
    do_blocking_move_to(nx, ny, nz);

    float measured_z = NAN;
    if (!DEPLOY_PROBE()) {
      measured_z = run_z_probe() + zprobe_zoffset;

      const bool big_raise = raise_after == PROBE_PT_BIG_RAISE;
      if (big_raise || raise_after == PROBE_PT_RAISE)
        do_blocking_move_to_z(current_position[Z_AXIS] + (big_raise ? 25 : Z_CLEARANCE_BETWEEN_PROBES), MMM_TO_MMS(Z_PROBE_SPEED_FAST));
      else if (raise_after == PROBE_PT_STOW)
        if (STOW_PROBE()) measured_z = NAN;
    }

    if (verbose_level > 2) {
      SERIAL_PROTOCOLPGM("Bed X: ");
      SERIAL_PROTOCOL_F(LOGICAL_X_POSITION(rx), 3);
      SERIAL_PROTOCOLPGM(" Y: ");
      SERIAL_PROTOCOL_F(LOGICAL_Y_POSITION(ry), 3);
      SERIAL_PROTOCOLPGM(" Z: ");
      SERIAL_PROTOCOL_F(measured_z, 3);
      SERIAL_EOL();
    }

    feedrate_mm_s = old_feedrate_mm_s;

    if (isnan(measured_z)) {
      LCD_MESSAGEPGM(MSG_ERR_PROBING_FAILED);
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM(MSG_ERR_PROBING_FAILED);
    }

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< probe_pt");
    #endif

    return measured_z;
  }

#endif // HAS_BED_PROBE

#if HAS_LEVELING

  bool leveling_is_valid() {
    return
      #if ENABLED(MESH_BED_LEVELING)
        mbl.has_mesh()
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        !!bilinear_grid_spacing[X_AXIS]
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        ubl.mesh_is_valid()
      #else // 3POINT, LINEAR
        true
      #endif
    ;
  }

  /**
   * Turn bed leveling on or off, fixing the current
   * position as-needed.
   *
   * Disable: Current position = physical position
   *  Enable: Current position = "unleveled" physical position
   */
  void set_bed_leveling_enabled(const bool enable/*=true*/) {

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      const bool can_change = (!enable || leveling_is_valid());
    #else
      constexpr bool can_change = true;
    #endif

    if (can_change && enable != planner.leveling_active) {

      planner.synchronize();

      #if ENABLED(MESH_BED_LEVELING)

        if (!enable)
          planner.apply_leveling(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);

        const bool enabling = enable && leveling_is_valid();
        planner.leveling_active = enabling;
        if (enabling) planner.unapply_leveling(current_position);

      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        #if PLANNER_LEVELING
          if (planner.leveling_active) {                       // leveling from on to off
            // change unleveled current_position to physical current_position without moving steppers.
            planner.apply_leveling(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
            planner.leveling_active = false;                   // disable only AFTER calling apply_leveling
          }
          else {                                               // leveling from off to on
            planner.leveling_active = true;                    // enable BEFORE calling unapply_leveling, otherwise ignored
            // change physical current_position to unleveled current_position without moving steppers.
            planner.unapply_leveling(current_position);
          }
        #else
          // UBL equivalents for apply/unapply_leveling
          #if ENABLED(SKEW_CORRECTION)
            float pos[XYZ] = { current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] };
            planner.skew(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS]);
          #else
            const float (&pos)[XYZE] = current_position;
          #endif
          if (planner.leveling_active) {
            current_position[Z_AXIS] += ubl.get_z_correction(pos[X_AXIS], pos[Y_AXIS]);
            planner.leveling_active = false;
          }
          else {
            planner.leveling_active = true;
            current_position[Z_AXIS] -= ubl.get_z_correction(pos[X_AXIS], pos[Y_AXIS]);
          }
        #endif

      #else // ABL

        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          // Force bilinear_z_offset to re-calculate next time
          const float reset[XYZ] = { -9999.999, -9999.999, 0 };
          (void)bilinear_z_offset(reset);
        #endif

        // Enable or disable leveling compensation in the planner
        planner.leveling_active = enable;

        if (!enable)
          // When disabling just get the current position from the steppers.
          // This will yield the smallest error when first converted back to steps.
          set_current_from_steppers_for_axis(
            #if ABL_PLANAR
              ALL_AXES
            #else
              Z_AXIS
            #endif
          );
        else
          // When enabling, remove compensation from the current position,
          // so compensation will give the right stepper counts.
          planner.unapply_leveling(current_position);

        SYNC_PLAN_POSITION_KINEMATIC();

      #endif // ABL
    }
  }

  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)

    void set_z_fade_height(const float zfh, const bool do_report/*=true*/) {

      if (planner.z_fade_height == zfh) return;

      const bool leveling_was_active = planner.leveling_active;
      set_bed_leveling_enabled(false);

      planner.set_z_fade_height(zfh);

      if (leveling_was_active) {
        const float oldpos[] = { current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] };
        set_bed_leveling_enabled(true);
        if (do_report && memcmp(oldpos, current_position, sizeof(oldpos)))
          report_current_position();
      }
    }

  #endif // LEVELING_FADE_HEIGHT

  /**
   * Reset calibration results to zero.
   */
  void reset_bed_level() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("reset_bed_level");
    #endif
    set_bed_leveling_enabled(false);
    #if ENABLED(MESH_BED_LEVELING)
      mbl.reset();
    #elif ENABLED(AUTO_BED_LEVELING_UBL)
      ubl.reset();
    #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
      bilinear_start[X_AXIS] = bilinear_start[Y_AXIS] =
      bilinear_grid_spacing[X_AXIS] = bilinear_grid_spacing[Y_AXIS] = 0;
      for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
        for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
          z_values[x][y] = NAN;
    #elif ABL_PLANAR
      planner.bed_level_matrix.set_to_identity();
    #endif
  }

#endif // HAS_LEVELING

#if ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(MESH_BED_LEVELING)

  /**
   * Enable to produce output in JSON format suitable
   * for SCAD or JavaScript mesh visualizers.
   *
   * Visualize meshes in OpenSCAD using the included script.
   *
   *   buildroot/shared/scripts/MarlinMesh.scad
   */
  //#define SCAD_MESH_OUTPUT

  /**
   * Print calibration results for plotting or manual frame adjustment.
   */
  void print_2d_array(const uint8_t sx, const uint8_t sy, const uint8_t precision, const element_2d_fn fn) {
    #ifndef SCAD_MESH_OUTPUT
      for (uint8_t x = 0; x < sx; x++) {
        for (uint8_t i = 0; i < precision + 2 + (x < 10 ? 1 : 0); i++)
          SERIAL_PROTOCOLCHAR(' ');
        SERIAL_PROTOCOL(int(x));
      }
      SERIAL_EOL();
    #endif
    #ifdef SCAD_MESH_OUTPUT
      SERIAL_PROTOCOLLNPGM("measured_z = ["); // open 2D array
    #endif
    for (uint8_t y = 0; y < sy; y++) {
      #ifdef SCAD_MESH_OUTPUT
        SERIAL_PROTOCOLPGM(" [");           // open sub-array
      #else
        if (y < 10) SERIAL_PROTOCOLCHAR(' ');
        SERIAL_PROTOCOL(int(y));
      #endif
      for (uint8_t x = 0; x < sx; x++) {
        SERIAL_PROTOCOLCHAR(' ');
        const float offset = fn(x, y);
        if (!isnan(offset)) {
          if (offset >= 0) SERIAL_PROTOCOLCHAR('+');
          SERIAL_PROTOCOL_F(offset, int(precision));
        }
        else {
          #ifdef SCAD_MESH_OUTPUT
            for (uint8_t i = 3; i < precision + 3; i++)
              SERIAL_PROTOCOLCHAR(' ');
            SERIAL_PROTOCOLPGM("NAN");
          #else
            for (uint8_t i = 0; i < precision + 3; i++)
              SERIAL_PROTOCOLCHAR(i ? '=' : ' ');
          #endif
        }
        #ifdef SCAD_MESH_OUTPUT
          if (x < sx - 1) SERIAL_PROTOCOLCHAR(',');
        #endif
      }
      #ifdef SCAD_MESH_OUTPUT
        SERIAL_PROTOCOLCHAR(' ');
        SERIAL_PROTOCOLCHAR(']');                     // close sub-array
        if (y < sy - 1) SERIAL_PROTOCOLCHAR(',');
      #endif
      SERIAL_EOL();
    }
    #ifdef SCAD_MESH_OUTPUT
      SERIAL_PROTOCOLPGM("];");                       // close 2D array
    #endif
    SERIAL_EOL();
  }

#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)

  /**
   * Extrapolate a single point from its neighbors
   */
  static void extrapolate_one_point(const uint8_t x, const uint8_t y, const int8_t xdir, const int8_t ydir) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOPGM("Extrapolate [");
        if (x < 10) SERIAL_CHAR(' ');
        SERIAL_ECHO(int(x));
        SERIAL_CHAR(xdir ? (xdir > 0 ? '+' : '-') : ' ');
        SERIAL_CHAR(' ');
        if (y < 10) SERIAL_CHAR(' ');
        SERIAL_ECHO(int(y));
        SERIAL_CHAR(ydir ? (ydir > 0 ? '+' : '-') : ' ');
        SERIAL_CHAR(']');
      }
    #endif
    if (!isnan(z_values[x][y])) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM(" (done)");
      #endif
      return;  // Don't overwrite good values.
    }
    SERIAL_EOL();

    // Get X neighbors, Y neighbors, and XY neighbors
    const uint8_t x1 = x + xdir, y1 = y + ydir, x2 = x1 + xdir, y2 = y1 + ydir;
    float a1 = z_values[x1][y ], a2 = z_values[x2][y ],
          b1 = z_values[x ][y1], b2 = z_values[x ][y2],
          c1 = z_values[x1][y1], c2 = z_values[x2][y2];

    // Treat far unprobed points as zero, near as equal to far
    if (isnan(a2)) a2 = 0.0; if (isnan(a1)) a1 = a2;
    if (isnan(b2)) b2 = 0.0; if (isnan(b1)) b1 = b2;
    if (isnan(c2)) c2 = 0.0; if (isnan(c1)) c1 = c2;

    const float a = 2 * a1 - a2, b = 2 * b1 - b2, c = 2 * c1 - c2;

    // Take the average instead of the median
    z_values[x][y] = (a + b + c) / 3.0;

    // Median is robust (ignores outliers).
    // z_values[x][y] = (a < b) ? ((b < c) ? b : (c < a) ? a : c)
    //                                : ((c < b) ? b : (a < c) ? a : c);
  }

  //Enable this if your SCARA uses 180° of total area
  //#define EXTRAPOLATE_FROM_EDGE

  #if ENABLED(EXTRAPOLATE_FROM_EDGE)
    #if GRID_MAX_POINTS_X < GRID_MAX_POINTS_Y
      #define HALF_IN_X
    #elif GRID_MAX_POINTS_Y < GRID_MAX_POINTS_X
      #define HALF_IN_Y
    #endif
  #endif

  /**
   * Fill in the unprobed points (corners of circular print surface)
   * using linear extrapolation, away from the center.
   */
  static void extrapolate_unprobed_bed_level() {
    #ifdef HALF_IN_X
      constexpr uint8_t ctrx2 = 0, xlen = GRID_MAX_POINTS_X - 1;
    #else
      constexpr uint8_t ctrx1 = (GRID_MAX_POINTS_X - 1) / 2, // left-of-center
                        ctrx2 = (GRID_MAX_POINTS_X) / 2,     // right-of-center
                        xlen = ctrx1;
    #endif

    #ifdef HALF_IN_Y
      constexpr uint8_t ctry2 = 0, ylen = GRID_MAX_POINTS_Y - 1;
    #else
      constexpr uint8_t ctry1 = (GRID_MAX_POINTS_Y - 1) / 2, // top-of-center
                        ctry2 = (GRID_MAX_POINTS_Y) / 2,     // bottom-of-center
                        ylen = ctry1;
    #endif

    for (uint8_t xo = 0; xo <= xlen; xo++)
      for (uint8_t yo = 0; yo <= ylen; yo++) {
        uint8_t x2 = ctrx2 + xo, y2 = ctry2 + yo;
        #ifndef HALF_IN_X
          const uint8_t x1 = ctrx1 - xo;
        #endif
        #ifndef HALF_IN_Y
          const uint8_t y1 = ctry1 - yo;
          #ifndef HALF_IN_X
            extrapolate_one_point(x1, y1, +1, +1);   //  left-below + +
          #endif
          extrapolate_one_point(x2, y1, -1, +1);     // right-below - +
        #endif
        #ifndef HALF_IN_X
          extrapolate_one_point(x1, y2, +1, -1);     //  left-above + -
        #endif
        extrapolate_one_point(x2, y2, -1, -1);       // right-above - -
      }

  }

  static void print_bilinear_leveling_grid() {
    SERIAL_ECHOLNPGM("Bilinear Leveling Grid:");
    print_2d_array(GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y, 3,
      [](const uint8_t ix, const uint8_t iy) { return z_values[ix][iy]; }
    );
  }

  #if ENABLED(ABL_BILINEAR_SUBDIVISION)

    #define ABL_GRID_POINTS_VIRT_X (GRID_MAX_POINTS_X - 1) * (BILINEAR_SUBDIVISIONS) + 1
    #define ABL_GRID_POINTS_VIRT_Y (GRID_MAX_POINTS_Y - 1) * (BILINEAR_SUBDIVISIONS) + 1
    #define ABL_TEMP_POINTS_X (GRID_MAX_POINTS_X + 2)
    #define ABL_TEMP_POINTS_Y (GRID_MAX_POINTS_Y + 2)
    float z_values_virt[ABL_GRID_POINTS_VIRT_X][ABL_GRID_POINTS_VIRT_Y];
    int bilinear_grid_spacing_virt[2] = { 0 };
    float bilinear_grid_factor_virt[2] = { 0 };

    static void print_bilinear_leveling_grid_virt() {
      SERIAL_ECHOLNPGM("Subdivided with CATMULL ROM Leveling Grid:");
      print_2d_array(ABL_GRID_POINTS_VIRT_X, ABL_GRID_POINTS_VIRT_Y, 5,
        [](const uint8_t ix, const uint8_t iy) { return z_values_virt[ix][iy]; }
      );
    }

    #define LINEAR_EXTRAPOLATION(E, I) ((E) * 2 - (I))
    float bed_level_virt_coord(const uint8_t x, const uint8_t y) {
      uint8_t ep = 0, ip = 1;
      if (!x || x == ABL_TEMP_POINTS_X - 1) {
        if (x) {
          ep = GRID_MAX_POINTS_X - 1;
          ip = GRID_MAX_POINTS_X - 2;
        }
        if (WITHIN(y, 1, ABL_TEMP_POINTS_Y - 2))
          return LINEAR_EXTRAPOLATION(
            z_values[ep][y - 1],
            z_values[ip][y - 1]
          );
        else
          return LINEAR_EXTRAPOLATION(
            bed_level_virt_coord(ep + 1, y),
            bed_level_virt_coord(ip + 1, y)
          );
      }
      if (!y || y == ABL_TEMP_POINTS_Y - 1) {
        if (y) {
          ep = GRID_MAX_POINTS_Y - 1;
          ip = GRID_MAX_POINTS_Y - 2;
        }
        if (WITHIN(x, 1, ABL_TEMP_POINTS_X - 2))
          return LINEAR_EXTRAPOLATION(
            z_values[x - 1][ep],
            z_values[x - 1][ip]
          );
        else
          return LINEAR_EXTRAPOLATION(
            bed_level_virt_coord(x, ep + 1),
            bed_level_virt_coord(x, ip + 1)
          );
      }
      return z_values[x - 1][y - 1];
    }

    static float bed_level_virt_cmr(const float p[4], const uint8_t i, const float t) {
      return (
          p[i-1] * -t * sq(1 - t)
        + p[i]   * (2 - 5 * sq(t) + 3 * t * sq(t))
        + p[i+1] * t * (1 + 4 * t - 3 * sq(t))
        - p[i+2] * sq(t) * (1 - t)
      ) * 0.5;
    }

    static float bed_level_virt_2cmr(const uint8_t x, const uint8_t y, const float &tx, const float &ty) {
      float row[4], column[4];
      for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 4; j++) {
          column[j] = bed_level_virt_coord(i + x - 1, j + y - 1);
        }
        row[i] = bed_level_virt_cmr(column, 1, ty);
      }
      return bed_level_virt_cmr(row, 1, tx);
    }

    void bed_level_virt_interpolate() {
      bilinear_grid_spacing_virt[X_AXIS] = bilinear_grid_spacing[X_AXIS] / (BILINEAR_SUBDIVISIONS);
      bilinear_grid_spacing_virt[Y_AXIS] = bilinear_grid_spacing[Y_AXIS] / (BILINEAR_SUBDIVISIONS);
      bilinear_grid_factor_virt[X_AXIS] = RECIPROCAL(bilinear_grid_spacing_virt[X_AXIS]);
      bilinear_grid_factor_virt[Y_AXIS] = RECIPROCAL(bilinear_grid_spacing_virt[Y_AXIS]);
      for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
        for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
          for (uint8_t ty = 0; ty < BILINEAR_SUBDIVISIONS; ty++)
            for (uint8_t tx = 0; tx < BILINEAR_SUBDIVISIONS; tx++) {
              if ((ty && y == GRID_MAX_POINTS_Y - 1) || (tx && x == GRID_MAX_POINTS_X - 1))
                continue;
              z_values_virt[x * (BILINEAR_SUBDIVISIONS) + tx][y * (BILINEAR_SUBDIVISIONS) + ty] =
                bed_level_virt_2cmr(
                  x + 1,
                  y + 1,
                  (float)tx / (BILINEAR_SUBDIVISIONS),
                  (float)ty / (BILINEAR_SUBDIVISIONS)
                );
            }
    }
  #endif // ABL_BILINEAR_SUBDIVISION

  // Refresh after other values have been updated
  void refresh_bed_level() {
    bilinear_grid_factor[X_AXIS] = RECIPROCAL(bilinear_grid_spacing[X_AXIS]);
    bilinear_grid_factor[Y_AXIS] = RECIPROCAL(bilinear_grid_spacing[Y_AXIS]);
    #if ENABLED(ABL_BILINEAR_SUBDIVISION)
      bed_level_virt_interpolate();
    #endif
  }

#endif // AUTO_BED_LEVELING_BILINEAR

#if ENABLED(SENSORLESS_HOMING)

  /**
   * Set sensorless homing if the axis has it, accounting for Core Kinematics.
   */
  void sensorless_homing_per_axis(const AxisEnum axis, const bool enable=true) {
    switch (axis) {
      #if X_SENSORLESS
        case X_AXIS:
          tmc_sensorless_homing(stepperX, enable);
          #if CORE_IS_XY && Y_SENSORLESS
            tmc_sensorless_homing(stepperY, enable);
          #elif CORE_IS_XZ && Z_SENSORLESS
            tmc_sensorless_homing(stepperZ, enable);
          #endif
          break;
      #endif
      #if Y_SENSORLESS
        case Y_AXIS:
          tmc_sensorless_homing(stepperY, enable);
          #if CORE_IS_XY && X_SENSORLESS
            tmc_sensorless_homing(stepperX, enable);
          #elif CORE_IS_YZ && Z_SENSORLESS
            tmc_sensorless_homing(stepperZ, enable);
          #endif
          break;
      #endif
      #if Z_SENSORLESS
        case Z_AXIS:
          tmc_sensorless_homing(stepperZ, enable);
          #if CORE_IS_XZ && X_SENSORLESS
            tmc_sensorless_homing(stepperX, enable);
          #elif CORE_IS_YZ && Y_SENSORLESS
            tmc_sensorless_homing(stepperY, enable);
          #endif
          break;
      #endif
      default: break;
    }
  }

#endif // SENSORLESS_HOMING

/**
 * Home an individual linear axis
 */
static void do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s=0) {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR(">>> do_homing_move(", axis_codes[axis]);
      SERIAL_ECHOPAIR(", ", distance);
      SERIAL_ECHOPGM(", ");
      if (fr_mm_s)
        SERIAL_ECHO(fr_mm_s);
      else {
        SERIAL_ECHOPAIR("[", homing_feedrate(axis));
        SERIAL_CHAR(']');
      }
      SERIAL_ECHOLNPGM(")");
    }
  #endif

  #if HOMING_Z_WITH_PROBE && HAS_HEATED_BED && ENABLED(WAIT_FOR_BED_HEATER)
    // Wait for bed to heat back up between probing points
    if (axis == Z_AXIS && distance < 0 && thermalManager.isHeatingBed()) {
      serialprintPGM(msg_wait_for_bed_heating);
      LCD_MESSAGEPGM(MSG_BED_HEATING);
      while (thermalManager.isHeatingBed()) safe_delay(200);
      lcd_reset_status();
    }
  #endif

  // Only do some things when moving towards an endstop
  const int8_t axis_home_dir =
    #if ENABLED(DUAL_X_CARRIAGE)
      (axis == X_AXIS) ? x_home_dir(active_extruder) :
    #endif
    home_dir(axis);
  const bool is_home_dir = (axis_home_dir > 0) == (distance > 0);

  if (is_home_dir) {

    #if HOMING_Z_WITH_PROBE && QUIET_PROBING
      if (axis == Z_AXIS) probing_pause(true);
    #endif

    // Disable stealthChop if used. Enable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      sensorless_homing_per_axis(axis);
    #endif
  }

  // Tell the planner the axis is at 0
  current_position[axis] = 0;

  // Do the move, which is required to hit an endstop
  #if IS_SCARA
    SYNC_PLAN_POSITION_KINEMATIC();
    current_position[axis] = distance;
    inverse_kinematics(current_position);
    planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], current_position[E_CART], fr_mm_s ? fr_mm_s : homing_feedrate(axis), active_extruder);
  #elif ENABLED(HANGPRINTER) // TODO: Hangprinter homing is not finished (Jan 7, 2018)
    SYNC_PLAN_POSITION_KINEMATIC();
    current_position[axis] = distance;
    inverse_kinematics(current_position);
    planner.buffer_line(line_lengths[A_AXIS], line_lengths[B_AXIS], line_lengths[C_AXIS], line_lengths[D_AXIS], current_position[E_CART], fr_mm_s ? fr_mm_s : homing_feedrate(axis), active_extruder);
  #else
    sync_plan_position();
    current_position[axis] = distance; // Set delta/cartesian axes directly
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_CART], fr_mm_s ? fr_mm_s : homing_feedrate(axis), active_extruder);
  #endif

  planner.synchronize();

  if (is_home_dir) {

    #if HOMING_Z_WITH_PROBE && QUIET_PROBING
      if (axis == Z_AXIS) probing_pause(false);
    #endif

    endstops.validate_homing_move();

    // Re-enable stealthChop if used. Disable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      sensorless_homing_per_axis(axis, false);
    #endif
  }

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR("<<< do_homing_move(", axis_codes[axis]);
      SERIAL_CHAR(')');
      SERIAL_EOL();
    }
  #endif
}

/**
 * Home an individual "raw axis" to its endstop.
 * This applies to XYZ on Cartesian and Core robots, and
 * to the individual ABC steppers on DELTA and SCARA.
 *
 * At the end of the procedure the axis is marked as
 * homed and the current position of that axis is updated.
 * Kinematic robots should wait till all axes are homed
 * before updating the current position.
 */

static void homeaxis(const AxisEnum axis) {

  #if IS_SCARA
    // Only Z homing (with probe) is permitted
    if (axis != Z_AXIS) { BUZZ(100, 880); return; }
  #else
    #define CAN_HOME(A) \
      (axis == _AXIS(A) && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
    if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;
  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR(">>> homeaxis(", axis_codes[axis]);
      SERIAL_CHAR(')');
      SERIAL_EOL();
    }
  #endif

  const int axis_home_dir = (
    #if ENABLED(DUAL_X_CARRIAGE)
      axis == X_AXIS ? x_home_dir(active_extruder) :
    #endif
    home_dir(axis)
  );

  // Homing Z towards the bed? Deploy the Z probe or endstop.
  #if HOMING_Z_WITH_PROBE
    if (axis == Z_AXIS && DEPLOY_PROBE()) return;
  #endif

  // Set flags for X, Y, Z motor locking
  #if ENABLED(X_DUAL_ENDSTOPS) || ENABLED(Y_DUAL_ENDSTOPS) || ENABLED(Z_DUAL_ENDSTOPS)
    switch (axis) {
      #if ENABLED(X_DUAL_ENDSTOPS)
        case X_AXIS:
      #endif
      #if ENABLED(Y_DUAL_ENDSTOPS)
        case Y_AXIS:
      #endif
      #if ENABLED(Z_DUAL_ENDSTOPS)
        case Z_AXIS:
      #endif
      stepper.set_homing_dual_axis(true);
      default: break;
    }
  #endif

  // Fast move towards endstop until triggered
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Home 1 Fast:");
  #endif

  #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
    // BLTOUCH needs to be deployed every time
    if (axis == Z_AXIS && set_bltouch_deployed(true)) return;
  #endif

  do_homing_move(axis, 1.5f * max_length(axis) * axis_home_dir);

  #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
    // BLTOUCH needs to be stowed after trigger to rearm itself
    if (axis == Z_AXIS) set_bltouch_deployed(false);
  #endif

  // When homing Z with probe respect probe clearance
  const float bump = axis_home_dir * (
    #if HOMING_Z_WITH_PROBE
      (axis == Z_AXIS && (Z_HOME_BUMP_MM)) ? MAX(Z_CLEARANCE_BETWEEN_PROBES, Z_HOME_BUMP_MM) :
    #endif
    home_bump_mm(axis)
  );

  // If a second homing move is configured...
  if (bump) {
    // Move away from the endstop by the axis HOME_BUMP_MM
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Move Away:");
    #endif
    do_homing_move(axis, -bump
      #if HOMING_Z_WITH_PROBE
        , axis == Z_AXIS ? MMM_TO_MMS(Z_PROBE_SPEED_FAST) : 0.00
      #endif
    );

    // Slow move towards endstop until triggered
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Home 2 Slow:");
    #endif

    #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
      // BLTOUCH needs to be deployed every time
      if (axis == Z_AXIS && set_bltouch_deployed(true)) return;
    #endif

    do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));

    #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
      // BLTOUCH needs to be stowed after trigger to rearm itself
      if (axis == Z_AXIS) set_bltouch_deployed(false);
    #endif
  }

  /**
   * Home axes that have dual endstops... differently
   */
  #if ENABLED(X_DUAL_ENDSTOPS) || ENABLED(Y_DUAL_ENDSTOPS) || ENABLED(Z_DUAL_ENDSTOPS)
    const bool pos_dir = axis_home_dir > 0;
    #if ENABLED(X_DUAL_ENDSTOPS)
      if (axis == X_AXIS) {
        const float adj = ABS(endstops.x_endstop_adj);
        if (adj) {
          if (pos_dir ? (endstops.x_endstop_adj > 0) : (endstops.x_endstop_adj < 0)) stepper.set_x_lock(true); else stepper.set_x2_lock(true);
          do_homing_move(axis, pos_dir ? -adj : adj);
          stepper.set_x_lock(false);
          stepper.set_x2_lock(false);
        }
      }
    #endif
    #if ENABLED(Y_DUAL_ENDSTOPS)
      if (axis == Y_AXIS) {
        const float adj = ABS(endstops.y_endstop_adj);
        if (adj) {
          if (pos_dir ? (endstops.y_endstop_adj > 0) : (endstops.y_endstop_adj < 0)) stepper.set_y_lock(true); else stepper.set_y2_lock(true);
          do_homing_move(axis, pos_dir ? -adj : adj);
          stepper.set_y_lock(false);
          stepper.set_y2_lock(false);
        }
      }
    #endif
    #if ENABLED(Z_DUAL_ENDSTOPS)
      if (axis == Z_AXIS) {
        const float adj = ABS(endstops.z_endstop_adj);
        if (adj) {
          if (pos_dir ? (endstops.z_endstop_adj > 0) : (endstops.z_endstop_adj < 0)) stepper.set_z_lock(true); else stepper.set_z2_lock(true);
          do_homing_move(axis, pos_dir ? -adj : adj);
          stepper.set_z_lock(false);
          stepper.set_z2_lock(false);
        }
      }
    #endif
    stepper.set_homing_dual_axis(false);
  #endif

  #if IS_SCARA

    set_axis_is_at_home(axis);
    SYNC_PLAN_POSITION_KINEMATIC();

  #elif ENABLED(DELTA)

    // Delta has already moved all three towers up in G28
    // so here it re-homes each tower in turn.
    // Delta homing treats the axes as normal linear axes.

    // retrace by the amount specified in delta_endstop_adj + additional dist in order to have minimum steps
    if (delta_endstop_adj[axis] * Z_HOME_DIR <= 0) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("delta_endstop_adj:");
      #endif
      do_homing_move(axis, delta_endstop_adj[axis] - (MIN_STEPS_PER_SEGMENT + 1) * planner.steps_to_mm[axis] * Z_HOME_DIR);
    }

  #else

    // For cartesian/core machines,
    // set the axis to its home position
    set_axis_is_at_home(axis);
    sync_plan_position();

    destination[axis] = current_position[axis];

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("> AFTER set_axis_is_at_home", current_position);
    #endif

  #endif

  // Put away the Z probe
  #if HOMING_Z_WITH_PROBE
    if (axis == Z_AXIS && STOW_PROBE()) return;
  #endif

  // Clear retracted status if homing the Z axis
  #if ENABLED(FWRETRACT)
    if (axis == Z_AXIS) fwretract.hop_amount = 0.0;
  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR("<<< homeaxis(", axis_codes[axis]);
      SERIAL_CHAR(')');
      SERIAL_EOL();
    }
  #endif
} // homeaxis()

#if ENABLED(MIXING_EXTRUDER)

  void normalize_mix() {
    float mix_total = 0.0;
    for (uint8_t i = 0; i < MIXING_STEPPERS; i++) mix_total += mixing_factor[i];
    // Scale all values if they don't add up to ~1.0
    if (!NEAR(mix_total, 1.0)) {
      SERIAL_PROTOCOLLNPGM("Warning: Mix factors must add up to 1.0. Scaling.");
      const float inverse_sum = RECIPROCAL(mix_total);
      for (uint8_t i = 0; i < MIXING_STEPPERS; i++) mixing_factor[i] *= inverse_sum;
    }
  }

  #if ENABLED(DIRECT_MIXING_IN_G1)
    // Get mixing parameters from the GCode
    // The total "must" be 1.0 (but it will be normalized)
    // If no mix factors are given, the old mix is preserved
    void gcode_get_mix() {
      const char mixing_codes[] = { 'A', 'B'
        #if MIXING_STEPPERS > 2
          , 'C'
          #if MIXING_STEPPERS > 3
            , 'D'
            #if MIXING_STEPPERS > 4
              , 'H'
              #if MIXING_STEPPERS > 5
                , 'I'
              #endif // MIXING_STEPPERS > 5
            #endif // MIXING_STEPPERS > 4
          #endif // MIXING_STEPPERS > 3
        #endif // MIXING_STEPPERS > 2
      };
      byte mix_bits = 0;
      for (uint8_t i = 0; i < MIXING_STEPPERS; i++) {
        if (parser.seenval(mixing_codes[i])) {
          SBI(mix_bits, i);
          mixing_factor[i] = MAX(parser.value_float(), 0.0);
        }
      }
      // If any mixing factors were included, clear the rest
      // If none were included, preserve the last mix
      if (mix_bits) {
        for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
          if (!TEST(mix_bits, i)) mixing_factor[i] = 0.0;
        normalize_mix();
      }
    }
  #endif

#endif


/**
 * Get the stepper positions in the cartes[] array.
 * Forward kinematics are applied for DELTA and SCARA.
 *
 * The result is in the current coordinate space with
 * leveling applied. The coordinates need to be run through
 * unapply_leveling to obtain machine coordinates suitable
 * for current_position, etc.
 */
void get_cartesian_from_steppers() {
  #if ENABLED(DELTA)
    forward_kinematics_DELTA(
      planner.get_axis_position_mm(A_AXIS),
      planner.get_axis_position_mm(B_AXIS),
      planner.get_axis_position_mm(C_AXIS)
    );
  #elif ENABLED(HANGPRINTER)
    forward_kinematics_HANGPRINTER(
      planner.get_axis_position_mm(A_AXIS),
      planner.get_axis_position_mm(B_AXIS),
      planner.get_axis_position_mm(C_AXIS),
      planner.get_axis_position_mm(D_AXIS)
    );
  #else
    #if IS_SCARA
      forward_kinematics_SCARA(
        planner.get_axis_position_degrees(A_AXIS),
        planner.get_axis_position_degrees(B_AXIS)
      );
    #else
      cartes[X_AXIS] = planner.get_axis_position_mm(X_AXIS);
      cartes[Y_AXIS] = planner.get_axis_position_mm(Y_AXIS);
    #endif
    cartes[Z_AXIS] = planner.get_axis_position_mm(Z_AXIS);
  #endif
}

/**
 * Set the current_position for an axis based on
 * the stepper positions, removing any leveling that
 * may have been applied.
 *
 * To prevent small shifts in axis position always call
 * SYNC_PLAN_POSITION_KINEMATIC after updating axes with this.
 *
 * To keep hosts in sync, always call report_current_position
 * after updating the current_position.
 */
void set_current_from_steppers_for_axis(const AxisEnum axis) {
  get_cartesian_from_steppers();
  #if PLANNER_LEVELING
    planner.unapply_leveling(cartes);
  #endif
  if (axis == ALL_AXES)
    COPY(current_position, cartes);
  else
    current_position[axis] = cartes[axis];
}

#if IS_CARTESIAN
#if ENABLED(SEGMENT_LEVELED_MOVES)

  /**
   * Prepare a segmented move on a CARTESIAN setup.
   *
   * This calls planner.buffer_line several times, adding
   * small incremental moves. This allows the planner to
   * apply more detailed bed leveling to the full move.
   */
  inline void segmented_line_to_destination(const float &fr_mm_s, const float segment_size=LEVELED_SEGMENT_LENGTH) {

    const float xdiff = destination[X_AXIS] - current_position[X_AXIS],
                ydiff = destination[Y_AXIS] - current_position[Y_AXIS];

    // If the move is only in Z/E don't split up the move
    if (!xdiff && !ydiff) {
      planner.buffer_line_kinematic(destination, fr_mm_s, active_extruder);
      return;
    }

    // Remaining cartesian distances
    const float zdiff = destination[Z_AXIS] - current_position[Z_AXIS],
                ediff = destination[E_CART] - current_position[E_CART];

    // Get the linear distance in XYZ
    // If the move is very short, check the E move distance
    // No E move either? Game over.
    float cartesian_mm = SQRT(sq(xdiff) + sq(ydiff) + sq(zdiff));
    if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = ABS(ediff);
    if (UNEAR_ZERO(cartesian_mm)) return;

    // The length divided by the segment size
    // At least one segment is required
    uint16_t segments = cartesian_mm / segment_size;
    NOLESS(segments, 1);

    // The approximate length of each segment
    const float inv_segments = 1.0f / float(segments),
                cartesian_segment_mm = cartesian_mm * inv_segments,
                segment_distance[XYZE] = {
                  xdiff * inv_segments,
                  ydiff * inv_segments,
                  zdiff * inv_segments,
                  ediff * inv_segments
                };

    // SERIAL_ECHOPAIR("mm=", cartesian_mm);
    // SERIAL_ECHOLNPAIR(" segments=", segments);
    // SERIAL_ECHOLNPAIR(" segment_mm=", cartesian_segment_mm);

    // Get the raw current position as starting point
    float raw[XYZE];
    COPY(raw, current_position);

    // Calculate and execute the segments
    while (--segments) {
      static millis_t next_idle_ms = millis() + 200UL;
      thermalManager.manage_heater();  // This returns immediately if not really needed.
      if (ELAPSED(millis(), next_idle_ms)) {
        next_idle_ms = millis() + 200UL;
        idle();
      }
      LOOP_XYZE(i) raw[i] += segment_distance[i];
      if (!planner.buffer_line_kinematic(raw, fr_mm_s, active_extruder, cartesian_segment_mm))
        break;
    }

    // Since segment_distance is only approximate,
    // the final move must be to the exact destination.
    planner.buffer_line_kinematic(destination, fr_mm_s, active_extruder, cartesian_segment_mm);
  }

#elif ENABLED(MESH_BED_LEVELING)

  /**
   * Prepare a mesh-leveled linear move in a Cartesian setup,
   * splitting the move where it crosses mesh borders.
   */
  void mesh_line_to_destination(const float fr_mm_s, uint8_t x_splits=0xFF, uint8_t y_splits=0xFF) {
    // Get current and destination cells for this line
    int cx1 = mbl.cell_index_x(current_position[X_AXIS]),
        cy1 = mbl.cell_index_y(current_position[Y_AXIS]),
        cx2 = mbl.cell_index_x(destination[X_AXIS]),
        cy2 = mbl.cell_index_y(destination[Y_AXIS]);
    NOMORE(cx1, GRID_MAX_POINTS_X - 2);
    NOMORE(cy1, GRID_MAX_POINTS_Y - 2);
    NOMORE(cx2, GRID_MAX_POINTS_X - 2);
    NOMORE(cy2, GRID_MAX_POINTS_Y - 2);

    // Start and end in the same cell? No split needed.
    if (cx1 == cx2 && cy1 == cy2) {
      buffer_line_to_destination(fr_mm_s);
      set_current_from_destination();
      return;
    }

    #define MBL_SEGMENT_END(A) (current_position[_AXIS(A)] + (destination[_AXIS(A)] - current_position[_AXIS(A)]) * normalized_dist)
    #define MBL_SEGMENT_END_E (current_position[E_CART] + (destination[E_CART] - current_position[E_CART]) * normalized_dist)

    float normalized_dist, end[XYZE];
    const int8_t gcx = MAX(cx1, cx2), gcy = MAX(cy1, cy2);

    // Crosses on the X and not already split on this X?
    // The x_splits flags are insurance against rounding errors.
    if (cx2 != cx1 && TEST(x_splits, gcx)) {
      // Split on the X grid line
      CBI(x_splits, gcx);
      COPY(end, destination);
      destination[X_AXIS] = mbl.index_to_xpos[gcx];
      normalized_dist = (destination[X_AXIS] - current_position[X_AXIS]) / (end[X_AXIS] - current_position[X_AXIS]);
      destination[Y_AXIS] = MBL_SEGMENT_END(Y);
    }
    // Crosses on the Y and not already split on this Y?
    else if (cy2 != cy1 && TEST(y_splits, gcy)) {
      // Split on the Y grid line
      CBI(y_splits, gcy);
      COPY(end, destination);
      destination[Y_AXIS] = mbl.index_to_ypos[gcy];
      normalized_dist = (destination[Y_AXIS] - current_position[Y_AXIS]) / (end[Y_AXIS] - current_position[Y_AXIS]);
      destination[X_AXIS] = MBL_SEGMENT_END(X);
    }
    else {
      // Must already have been split on these border(s)
      buffer_line_to_destination(fr_mm_s);
      set_current_from_destination();
      return;
    }

    destination[Z_AXIS] = MBL_SEGMENT_END(Z);
    destination[E_CART] = MBL_SEGMENT_END_E;

    // Do the split and look for more borders
    mesh_line_to_destination(fr_mm_s, x_splits, y_splits);

    // Restore destination from stack
    COPY(destination, end);
    mesh_line_to_destination(fr_mm_s, x_splits, y_splits);
  }

#elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

  #define CELL_INDEX(A,V) ((V - bilinear_start[_AXIS(A)]) * ABL_BG_FACTOR(_AXIS(A)))

  /**
   * Prepare a bilinear-leveled linear move on Cartesian,
   * splitting the move where it crosses grid borders.
   */
  void bilinear_line_to_destination(const float fr_mm_s, uint16_t x_splits=0xFFFF, uint16_t y_splits=0xFFFF) {
    // Get current and destination cells for this line
    int cx1 = CELL_INDEX(X, current_position[X_AXIS]),
        cy1 = CELL_INDEX(Y, current_position[Y_AXIS]),
        cx2 = CELL_INDEX(X, destination[X_AXIS]),
        cy2 = CELL_INDEX(Y, destination[Y_AXIS]);
    cx1 = constrain(cx1, 0, ABL_BG_POINTS_X - 2);
    cy1 = constrain(cy1, 0, ABL_BG_POINTS_Y - 2);
    cx2 = constrain(cx2, 0, ABL_BG_POINTS_X - 2);
    cy2 = constrain(cy2, 0, ABL_BG_POINTS_Y - 2);

    // Start and end in the same cell? No split needed.
    if (cx1 == cx2 && cy1 == cy2) {
      buffer_line_to_destination(fr_mm_s);
      set_current_from_destination();
      return;
    }

    #define LINE_SEGMENT_END(A) (current_position[_AXIS(A)] + (destination[_AXIS(A)] - current_position[_AXIS(A)]) * normalized_dist)
    #define LINE_SEGMENT_END_E (current_position[E_CART] + (destination[E_CART] - current_position[E_CART]) * normalized_dist)

    float normalized_dist, end[XYZE];
    const int8_t gcx = MAX(cx1, cx2), gcy = MAX(cy1, cy2);

    // Crosses on the X and not already split on this X?
    // The x_splits flags are insurance against rounding errors.
    if (cx2 != cx1 && TEST(x_splits, gcx)) {
      // Split on the X grid line
      CBI(x_splits, gcx);
      COPY(end, destination);
      destination[X_AXIS] = bilinear_start[X_AXIS] + ABL_BG_SPACING(X_AXIS) * gcx;
      normalized_dist = (destination[X_AXIS] - current_position[X_AXIS]) / (end[X_AXIS] - current_position[X_AXIS]);
      destination[Y_AXIS] = LINE_SEGMENT_END(Y);
    }
    // Crosses on the Y and not already split on this Y?
    else if (cy2 != cy1 && TEST(y_splits, gcy)) {
      // Split on the Y grid line
      CBI(y_splits, gcy);
      COPY(end, destination);
      destination[Y_AXIS] = bilinear_start[Y_AXIS] + ABL_BG_SPACING(Y_AXIS) * gcy;
      normalized_dist = (destination[Y_AXIS] - current_position[Y_AXIS]) / (end[Y_AXIS] - current_position[Y_AXIS]);
      destination[X_AXIS] = LINE_SEGMENT_END(X);
    }
    else {
      // Must already have been split on these border(s)
      buffer_line_to_destination(fr_mm_s);
      set_current_from_destination();
      return;
    }

    destination[Z_AXIS] = LINE_SEGMENT_END(Z);
    destination[E_CART] = LINE_SEGMENT_END_E;

    // Do the split and look for more borders
    bilinear_line_to_destination(fr_mm_s, x_splits, y_splits);

    // Restore destination from stack
    COPY(destination, end);
    bilinear_line_to_destination(fr_mm_s, x_splits, y_splits);
  }

#endif // AUTO_BED_LEVELING_BILINEAR
#endif // IS_CARTESIAN

#if !UBL_SEGMENTED
#if IS_KINEMATIC

  #if IS_SCARA
    /**
     * Before raising this value, use M665 S[seg_per_sec] to decrease
     * the number of segments-per-second. Default is 200. Some deltas
     * do better with 160 or lower. It would be good to know how many
     * segments-per-second are actually possible for SCARA on AVR.
     *
     * Longer segments result in less kinematic overhead
     * but may produce jagged lines. Try 0.5mm, 1.0mm, and 2.0mm
     * and compare the difference.
     */
    #define SCARA_MIN_SEGMENT_LENGTH 0.5f
  #endif

  /**
   * Prepare a linear move in a DELTA, SCARA or HANGPRINTER setup.
   *
   * This calls planner.buffer_line several times, adding
   * small incremental moves for DELTA, SCARA or HANGPRINTER.
   *
   * For Unified Bed Leveling (Delta or Segmented Cartesian)
   * the ubl.prepare_segmented_line_to method replaces this.
   */
  inline bool prepare_kinematic_move_to(const float (&rtarget)[XYZE]) {

    // Get the top feedrate of the move in the XY plane
    const float _feedrate_mm_s = MMS_SCALED(feedrate_mm_s);

    const float xdiff = rtarget[X_AXIS] - current_position[X_AXIS],
                ydiff = rtarget[Y_AXIS] - current_position[Y_AXIS]
                #if ENABLED(HANGPRINTER)
                  , zdiff = rtarget[Z_AXIS] - current_position[Z_AXIS]
                #endif
                ;

    // If the move is only in Z/E (for Hangprinter only in E) don't split up the move
    if (!xdiff && !ydiff
      #if ENABLED(HANGPRINTER)
        && !zdiff
      #endif
    ) {
      planner.buffer_line_kinematic(rtarget, _feedrate_mm_s, active_extruder);
      return false; // caller will update current_position
    }

    // Fail if attempting move outside printable radius
    if (!position_is_reachable(rtarget[X_AXIS], rtarget[Y_AXIS])) return true;

    // Remaining cartesian distances
    const float
                #if DISABLED(HANGPRINTER)
                  zdiff = rtarget[Z_AXIS] - current_position[Z_AXIS],
                #endif
                ediff = rtarget[E_CART] - current_position[E_CART];

    // Get the linear distance in XYZ
    // If the move is very short, check the E move distance
    // No E move either? Game over.
    float cartesian_mm = SQRT(sq(xdiff) + sq(ydiff) + sq(zdiff));
    if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = ABS(ediff);
    if (UNEAR_ZERO(cartesian_mm)) return true;

    // Minimum number of seconds to move the given distance
    const float seconds = cartesian_mm / _feedrate_mm_s;

    // The number of segments-per-second times the duration
    // gives the number of segments
    uint16_t segments = delta_segments_per_second * seconds;

    // For SCARA enforce a minimum segment size
    #if IS_SCARA
      NOMORE(segments, cartesian_mm * (1.0f / float(SCARA_MIN_SEGMENT_LENGTH)));
    #endif

    // At least one segment is required
    NOLESS(segments, 1);

    // The approximate length of each segment
    const float inv_segments = 1.0f / float(segments),
                segment_distance[XYZE] = {
                  xdiff * inv_segments,
                  ydiff * inv_segments,
                  zdiff * inv_segments,
                  ediff * inv_segments
                };

    #if !HAS_FEEDRATE_SCALING
      const float cartesian_segment_mm = cartesian_mm * inv_segments;
    #endif

    /*
    SERIAL_ECHOPAIR("mm=", cartesian_mm);
    SERIAL_ECHOPAIR(" seconds=", seconds);
    SERIAL_ECHOPAIR(" segments=", segments);
    #if !HAS_FEEDRATE_SCALING
      SERIAL_ECHOPAIR(" segment_mm=", cartesian_segment_mm);
    #endif
    SERIAL_EOL();
    //*/

    #if HAS_FEEDRATE_SCALING
      // SCARA needs to scale the feed rate from mm/s to degrees/s
      // i.e., Complete the angular vector in the given time.
      const float segment_length = cartesian_mm * inv_segments,
                  inv_segment_length = 1.0f / segment_length, // 1/mm/segs
                  inverse_secs = inv_segment_length * _feedrate_mm_s;

      float oldA = planner.position_float[A_AXIS],
            oldB = planner.position_float[B_AXIS]
            #if ENABLED(DELTA_FEEDRATE_SCALING)
              , oldC = planner.position_float[C_AXIS]
            #endif
            ;

      /*
      SERIAL_ECHOPGM("Scaled kinematic move: ");
      SERIAL_ECHOPAIR(" segment_length (inv)=", segment_length);
      SERIAL_ECHOPAIR(" (", inv_segment_length);
      SERIAL_ECHOPAIR(") _feedrate_mm_s=", _feedrate_mm_s);
      SERIAL_ECHOPAIR(" inverse_secs=", inverse_secs);
      SERIAL_ECHOPAIR(" oldA=", oldA);
      SERIAL_ECHOPAIR(" oldB=", oldB);
      #if ENABLED(DELTA_FEEDRATE_SCALING)
        SERIAL_ECHOPAIR(" oldC=", oldC);
      #endif
      SERIAL_EOL();
      safe_delay(5);
      //*/
    #endif

    // Get the current position as starting point
    float raw[XYZE];
    COPY(raw, current_position);

    // Calculate and execute the segments
    while (--segments) {

      static millis_t next_idle_ms = millis() + 200UL;
      thermalManager.manage_heater();  // This returns immediately if not really needed.
      if (ELAPSED(millis(), next_idle_ms)) {
        next_idle_ms = millis() + 200UL;
        idle();
      }

      LOOP_XYZE(i) raw[i] += segment_distance[i];
      #if ENABLED(DELTA) && HOTENDS < 2
        DELTA_IK(raw); // Delta can inline its kinematics
      #elif ENABLED(HANGPRINTER)
        HANGPRINTER_IK(raw); // Modifies line_lengths[ABCD]
      #else
        inverse_kinematics(raw);
      #endif

      ADJUST_DELTA(raw); // Adjust Z if bed leveling is enabled

      #if ENABLED(SCARA_FEEDRATE_SCALING)
        // For SCARA scale the feed rate from mm/s to degrees/s
        // i.e., Complete the angular vector in the given time.
        if (!planner.buffer_segment(delta[A_AXIS], delta[B_AXIS], raw[Z_AXIS], raw[E_CART], HYPOT(delta[A_AXIS] - oldA, delta[B_AXIS] - oldB) * inverse_secs, active_extruder, segment_length))
          break;
        /*
        SERIAL_ECHO(segments);
        SERIAL_ECHOPAIR(": X=", raw[X_AXIS]); SERIAL_ECHOPAIR(" Y=", raw[Y_AXIS]);
        SERIAL_ECHOPAIR(" A=", delta[A_AXIS]); SERIAL_ECHOPAIR(" B=", delta[B_AXIS]);
        SERIAL_ECHOLNPAIR(" F", HYPOT(delta[A_AXIS] - oldA, delta[B_AXIS] - oldB) * inverse_secs * 60);
        safe_delay(5);
        //*/
        oldA = delta[A_AXIS]; oldB = delta[B_AXIS];
      #elif ENABLED(DELTA_FEEDRATE_SCALING)
        // For DELTA scale the feed rate from Effector mm/s to Carriage mm/s
        // i.e., Complete the linear vector in the given time.
        if (!planner.buffer_segment(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], raw[E_AXIS], SQRT(sq(delta[A_AXIS] - oldA) + sq(delta[B_AXIS] - oldB) + sq(delta[C_AXIS] - oldC)) * inverse_secs, active_extruder, segment_length))
          break;
        /*
        SERIAL_ECHO(segments);
        SERIAL_ECHOPAIR(": X=", raw[X_AXIS]); SERIAL_ECHOPAIR(" Y=", raw[Y_AXIS]);
        SERIAL_ECHOPAIR(" A=", delta[A_AXIS]); SERIAL_ECHOPAIR(" B=", delta[B_AXIS]); SERIAL_ECHOPAIR(" C=", delta[C_AXIS]);
        SERIAL_ECHOLNPAIR(" F", SQRT(sq(delta[A_AXIS] - oldA) + sq(delta[B_AXIS] - oldB) + sq(delta[C_AXIS] - oldC)) * inverse_secs * 60);
        safe_delay(5);
        //*/
        oldA = delta[A_AXIS]; oldB = delta[B_AXIS]; oldC = delta[C_AXIS];
      #elif ENABLED(HANGPRINTER)
        if (!planner.buffer_line(line_lengths[A_AXIS], line_lengths[B_AXIS], line_lengths[C_AXIS], line_lengths[D_AXIS], raw[E_CART], _feedrate_mm_s, active_extruder, cartesian_segment_mm))
          break;
      #else
        if (!planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], raw[E_CART], _feedrate_mm_s, active_extruder, cartesian_segment_mm))
          break;
      #endif
    }

    // Ensure last segment arrives at target location.
    #if HAS_FEEDRATE_SCALING
      inverse_kinematics(rtarget);
      ADJUST_DELTA(rtarget);
    #endif

    #if ENABLED(SCARA_FEEDRATE_SCALING)
      const float diff2 = HYPOT2(delta[A_AXIS] - oldA, delta[B_AXIS] - oldB);
      if (diff2) {
        planner.buffer_segment(delta[A_AXIS], delta[B_AXIS], rtarget[Z_AXIS], rtarget[E_CART], SQRT(diff2) * inverse_secs, active_extruder, segment_length);
        /*
        SERIAL_ECHOPAIR("final: A=", delta[A_AXIS]); SERIAL_ECHOPAIR(" B=", delta[B_AXIS]);
        SERIAL_ECHOPAIR(" adiff=", delta[A_AXIS] - oldA); SERIAL_ECHOPAIR(" bdiff=", delta[B_AXIS] - oldB);
        SERIAL_ECHOLNPAIR(" F", SQRT(diff2) * inverse_secs * 60);
        SERIAL_EOL();
        safe_delay(5);
        //*/
      }
    #elif ENABLED(DELTA_FEEDRATE_SCALING)
      const float diff2 = sq(delta[A_AXIS] - oldA) + sq(delta[B_AXIS] - oldB) + sq(delta[C_AXIS] - oldC);
      if (diff2) {
        planner.buffer_segment(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], rtarget[E_AXIS], SQRT(diff2) * inverse_secs, active_extruder, segment_length);
        /*
        SERIAL_ECHOPAIR("final: A=", delta[A_AXIS]); SERIAL_ECHOPAIR(" B=", delta[B_AXIS]); SERIAL_ECHOPAIR(" C=", delta[C_AXIS]);
        SERIAL_ECHOPAIR(" adiff=", delta[A_AXIS] - oldA); SERIAL_ECHOPAIR(" bdiff=", delta[B_AXIS] - oldB); SERIAL_ECHOPAIR(" cdiff=", delta[C_AXIS] - oldC);
        SERIAL_ECHOLNPAIR(" F", SQRT(diff2) * inverse_secs * 60);
        SERIAL_EOL();
        safe_delay(5);
        //*/
      }
    #else
      planner.buffer_line_kinematic(rtarget, _feedrate_mm_s, active_extruder, cartesian_segment_mm);
    #endif

    return false; // caller will update current_position
  }

#else // !IS_KINEMATIC

  /**
   * Prepare a linear move in a Cartesian setup.
   *
   * When a mesh-based leveling system is active, moves are segmented
   * according to the configuration of the leveling system.
   *
   * Returns true if current_position[] was set to destination[]
   */
  inline bool prepare_move_to_destination_cartesian() {
    #if HAS_MESH
      if (planner.leveling_active && planner.leveling_active_at_z(destination[Z_AXIS])) {
        #if ENABLED(AUTO_BED_LEVELING_UBL)
          ubl.line_to_destination_cartesian(MMS_SCALED(feedrate_mm_s), active_extruder);  // UBL's motion routine needs to know about
          return true;                                                                    // all moves, including Z-only moves.
        #elif ENABLED(SEGMENT_LEVELED_MOVES)
          segmented_line_to_destination(MMS_SCALED(feedrate_mm_s));
          return false; // caller will update current_position
        #else
          /**
           * For MBL and ABL-BILINEAR only segment moves when X or Y are involved.
           * Otherwise fall through to do a direct single move.
           */
          if (current_position[X_AXIS] != destination[X_AXIS] || current_position[Y_AXIS] != destination[Y_AXIS]) {
            #if ENABLED(MESH_BED_LEVELING)
              mesh_line_to_destination(MMS_SCALED(feedrate_mm_s));
            #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
              bilinear_line_to_destination(MMS_SCALED(feedrate_mm_s));
            #endif
            return true;
          }
        #endif
      }
    #endif // HAS_MESH

    buffer_line_to_destination(MMS_SCALED(feedrate_mm_s));
    return false; // caller will update current_position
  }

#endif // !IS_KINEMATIC
#endif // !UBL_SEGMENTED

#if ENABLED(DUAL_X_CARRIAGE)

  /**
   * Unpark the carriage, if needed
   */
  inline bool dual_x_carriage_unpark() {
    if (active_extruder_parked)
      switch (dual_x_carriage_mode) {

        case DXC_FULL_CONTROL_MODE: break;

        case DXC_AUTO_PARK_MODE:
          if (current_position[E_CART] == destination[E_CART]) {
            // This is a travel move (with no extrusion)
            // Skip it, but keep track of the current position
            // (so it can be used as the start of the next non-travel move)
            if (delayed_move_time != 0xFFFFFFFFUL) {
              set_current_from_destination();
              NOLESS(raised_parked_position[Z_AXIS], destination[Z_AXIS]);
              delayed_move_time = millis();
              return true;
            }
          }
          // unpark extruder: 1) raise, 2) move into starting XY position, 3) lower
          for (uint8_t i = 0; i < 3; i++)
            if (!planner.buffer_line(
              i == 0 ? raised_parked_position[X_AXIS] : current_position[X_AXIS],
              i == 0 ? raised_parked_position[Y_AXIS] : current_position[Y_AXIS],
              i == 2 ? current_position[Z_AXIS] : raised_parked_position[Z_AXIS],
              current_position[E_CART],
              i == 1 ? PLANNER_XY_FEEDRATE() : planner.max_feedrate_mm_s[Z_AXIS],
              active_extruder)
            ) break;
          delayed_move_time = 0;
          active_extruder_parked = false;
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Clear active_extruder_parked");
          #endif
          break;

        case DXC_DUPLICATION_MODE:
          if (active_extruder == 0) {
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) {
                SERIAL_ECHOPAIR("Set planner X", inactive_extruder_x_pos);
                SERIAL_ECHOLNPAIR(" ... Line to X", current_position[X_AXIS] + duplicate_extruder_x_offset);
              }
            #endif
            // move duplicate extruder into correct duplication position.
            planner.set_position_mm(inactive_extruder_x_pos, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_CART]);
            if (!planner.buffer_line(
              current_position[X_AXIS] + duplicate_extruder_x_offset,
              current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_CART],
              planner.max_feedrate_mm_s[X_AXIS], 1)
            ) break;
            planner.synchronize();
            SYNC_PLAN_POSITION_KINEMATIC();
            extruder_duplication_enabled = true;
            active_extruder_parked = false;
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Set extruder_duplication_enabled\nClear active_extruder_parked");
            #endif
          }
          else {
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Active extruder not 0");
            #endif
          }
          break;
      }
    return false;
  }

#endif // DUAL_X_CARRIAGE

/**
 * Prepare a single move and get ready for the next one
 *
 * This may result in several calls to planner.buffer_line to
 * do smaller moves for DELTA, SCARA, HANGPRINTER, mesh moves, etc.
 *
 * Make sure current_position[E] and destination[E] are good
 * before calling or cold/lengthy extrusion may get missed.
 */
void prepare_move_to_destination() {
  clamp_to_software_endstops(destination);

  #if ENABLED(PREVENT_COLD_EXTRUSION) || ENABLED(PREVENT_LENGTHY_EXTRUDE)

    if (!DEBUGGING(DRYRUN)) {
      if (destination[E_CART] != current_position[E_CART]) {
        #if ENABLED(PREVENT_COLD_EXTRUSION)
          if (thermalManager.tooColdToExtrude(active_extruder)) {
            current_position[E_CART] = destination[E_CART]; // Behave as if the move really took place, but ignore E part
            SERIAL_ECHO_START();
            SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
          }
        #endif // PREVENT_COLD_EXTRUSION
        #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
          if (ABS(destination[E_CART] - current_position[E_CART]) * planner.e_factor[active_extruder] > (EXTRUDE_MAXLENGTH)) {
            current_position[E_CART] = destination[E_CART]; // Behave as if the move really took place, but ignore E part
            SERIAL_ECHO_START();
            SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
          }
        #endif // PREVENT_LENGTHY_EXTRUDE
      }
    }

  #endif

  #if ENABLED(DUAL_X_CARRIAGE)
    if (dual_x_carriage_unpark()) return;
  #endif

  if (
    #if UBL_SEGMENTED
      ubl.prepare_segmented_line_to(destination, MMS_SCALED(feedrate_mm_s))
    #elif IS_KINEMATIC
      prepare_kinematic_move_to(destination)
    #else
      prepare_move_to_destination_cartesian()
    #endif
  ) return;

  set_current_from_destination();
}

#if ENABLED(ARC_SUPPORT)

  #if N_ARC_CORRECTION < 1
    #undef N_ARC_CORRECTION
    #define N_ARC_CORRECTION 1
  #endif

  /**
   * Plan an arc in 2 dimensions
   *
   * The arc is approximated by generating many small linear segments.
   * The length of each segment is configured in MM_PER_ARC_SEGMENT (Default 1mm)
   * Arcs should only be made relatively large (over 5mm), as larger arcs with
   * larger segments will tend to be more efficient. Your slicer should have
   * options for G2/G3 arc generation. In future these options may be GCode tunable.
   */
  void plan_arc(
    const float (&cart)[XYZE], // Destination position
    const float (&offset)[2], // Center of rotation relative to current_position
    const bool clockwise      // Clockwise?
  ) {
    #if ENABLED(CNC_WORKSPACE_PLANES)
      AxisEnum p_axis, q_axis, l_axis;
      switch (workspace_plane) {
        default:
        case PLANE_XY: p_axis = X_AXIS; q_axis = Y_AXIS; l_axis = Z_AXIS; break;
        case PLANE_ZX: p_axis = Z_AXIS; q_axis = X_AXIS; l_axis = Y_AXIS; break;
        case PLANE_YZ: p_axis = Y_AXIS; q_axis = Z_AXIS; l_axis = X_AXIS; break;
      }
    #else
      constexpr AxisEnum p_axis = X_AXIS, q_axis = Y_AXIS, l_axis = Z_AXIS;
    #endif

    // Radius vector from center to current location
    float r_P = -offset[0], r_Q = -offset[1];

    const float radius = HYPOT(r_P, r_Q),
                center_P = current_position[p_axis] - r_P,
                center_Q = current_position[q_axis] - r_Q,
                rt_X = cart[p_axis] - center_P,
                rt_Y = cart[q_axis] - center_Q,
                linear_travel = cart[l_axis] - current_position[l_axis],
                extruder_travel = cart[E_CART] - current_position[E_CART];

    // CCW angle of rotation between position and target from the circle center. Only one atan2() trig computation required.
    float angular_travel = ATAN2(r_P * rt_Y - r_Q * rt_X, r_P * rt_X + r_Q * rt_Y);
    if (angular_travel < 0) angular_travel += RADIANS(360);
    if (clockwise) angular_travel -= RADIANS(360);

    // Make a circle if the angular rotation is 0 and the target is current position
    if (angular_travel == 0 && current_position[p_axis] == cart[p_axis] && current_position[q_axis] == cart[q_axis])
      angular_travel = RADIANS(360);

    const float flat_mm = radius * angular_travel,
                mm_of_travel = linear_travel ? HYPOT(flat_mm, linear_travel) : ABS(flat_mm);
    if (mm_of_travel < 0.001f) return;

    uint16_t segments = FLOOR(mm_of_travel / (MM_PER_ARC_SEGMENT));
    NOLESS(segments, 1);

    /**
     * Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
     * and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
     *     r_T = [cos(phi) -sin(phi);
     *            sin(phi)  cos(phi)] * r ;
     *
     * For arc generation, the center of the circle is the axis of rotation and the radius vector is
     * defined from the circle center to the initial position. Each line segment is formed by successive
     * vector rotations. This requires only two cos() and sin() computations to form the rotation
     * matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
     * all double numbers are single precision on the Arduino. (True double precision will not have
     * round off issues for CNC applications.) Single precision error can accumulate to be greater than
     * tool precision in some cases. Therefore, arc path correction is implemented.
     *
     * Small angle approximation may be used to reduce computation overhead further. This approximation
     * holds for everything, but very small circles and large MM_PER_ARC_SEGMENT values. In other words,
     * theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
     * to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
     * numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
     * issue for CNC machines with the single precision Arduino calculations.
     *
     * This approximation also allows plan_arc to immediately insert a line segment into the planner
     * without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
     * a correction, the planner should have caught up to the lag caused by the initial plan_arc overhead.
     * This is important when there are successive arc motions.
     */
    // Vector rotation matrix values
    float raw[XYZE];
    const float theta_per_segment = angular_travel / segments,
                linear_per_segment = linear_travel / segments,
                extruder_per_segment = extruder_travel / segments,
                sin_T = theta_per_segment,
                cos_T = 1 - 0.5f * sq(theta_per_segment); // Small angle approximation

    // Initialize the linear axis
    raw[l_axis] = current_position[l_axis];

    // Initialize the extruder axis
    raw[E_CART] = current_position[E_CART];

    const float fr_mm_s = MMS_SCALED(feedrate_mm_s);

    millis_t next_idle_ms = millis() + 200UL;

    #if HAS_FEEDRATE_SCALING
      // SCARA needs to scale the feed rate from mm/s to degrees/s
      const float inv_segment_length = 1.0f / (MM_PER_ARC_SEGMENT),
                  inverse_secs = inv_segment_length * fr_mm_s;
      float oldA = planner.position_float[A_AXIS],
            oldB = planner.position_float[B_AXIS]
            #if ENABLED(DELTA_FEEDRATE_SCALING)
              , oldC = planner.position_float[C_AXIS]
            #endif
            ;
    #endif

    #if N_ARC_CORRECTION > 1
      int8_t arc_recalc_count = N_ARC_CORRECTION;
    #endif

    for (uint16_t i = 1; i < segments; i++) { // Iterate (segments-1) times

      thermalManager.manage_heater();
      if (ELAPSED(millis(), next_idle_ms)) {
        next_idle_ms = millis() + 200UL;
        idle();
      }

      #if N_ARC_CORRECTION > 1
        if (--arc_recalc_count) {
          // Apply vector rotation matrix to previous r_P / 1
          const float r_new_Y = r_P * sin_T + r_Q * cos_T;
          r_P = r_P * cos_T - r_Q * sin_T;
          r_Q = r_new_Y;
        }
        else
      #endif
      {
        #if N_ARC_CORRECTION > 1
          arc_recalc_count = N_ARC_CORRECTION;
        #endif

        // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
        // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
        // To reduce stuttering, the sin and cos could be computed at different times.
        // For now, compute both at the same time.
        const float cos_Ti = cos(i * theta_per_segment), sin_Ti = sin(i * theta_per_segment);
        r_P = -offset[0] * cos_Ti + offset[1] * sin_Ti;
        r_Q = -offset[0] * sin_Ti - offset[1] * cos_Ti;
      }

      // Update raw location
      raw[p_axis] = center_P + r_P;
      raw[q_axis] = center_Q + r_Q;
      raw[l_axis] += linear_per_segment;
      raw[E_CART] += extruder_per_segment;

      clamp_to_software_endstops(raw);

      #if HAS_FEEDRATE_SCALING
        inverse_kinematics(raw);
        ADJUST_DELTA(raw);
      #endif

      #if ENABLED(SCARA_FEEDRATE_SCALING)
        // For SCARA scale the feed rate from mm/s to degrees/s
        // i.e., Complete the angular vector in the given time.
        if (!planner.buffer_segment(delta[A_AXIS], delta[B_AXIS], raw[Z_AXIS], raw[E_CART], HYPOT(delta[A_AXIS] - oldA, delta[B_AXIS] - oldB) * inverse_secs, active_extruder, MM_PER_ARC_SEGMENT))
          break;
        oldA = delta[A_AXIS]; oldB = delta[B_AXIS];
      #elif ENABLED(DELTA_FEEDRATE_SCALING)
        // For DELTA scale the feed rate from Effector mm/s to Carriage mm/s
        // i.e., Complete the linear vector in the given time.
        if (!planner.buffer_segment(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], raw[E_AXIS], SQRT(sq(delta[A_AXIS] - oldA) + sq(delta[B_AXIS] - oldB) + sq(delta[C_AXIS] - oldC)) * inverse_secs, active_extruder, MM_PER_ARC_SEGMENT))
          break;
        oldA = delta[A_AXIS]; oldB = delta[B_AXIS]; oldC = delta[C_AXIS];
      #elif HAS_UBL_AND_CURVES
        float pos[XYZ] = { raw[X_AXIS], raw[Y_AXIS], raw[Z_AXIS] };
        planner.apply_leveling(pos);
        if (!planner.buffer_segment(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], raw[E_CART], fr_mm_s, active_extruder, MM_PER_ARC_SEGMENT))
          break;
      #else
        if (!planner.buffer_line_kinematic(raw, fr_mm_s, active_extruder))
          break;
      #endif
    }

    // Ensure last segment arrives at target location.
    #if HAS_FEEDRATE_SCALING
      inverse_kinematics(cart);
      ADJUST_DELTA(cart);
    #endif

    #if ENABLED(SCARA_FEEDRATE_SCALING)
      const float diff2 = HYPOT2(delta[A_AXIS] - oldA, delta[B_AXIS] - oldB);
      if (diff2)
        planner.buffer_segment(delta[A_AXIS], delta[B_AXIS], cart[Z_AXIS], cart[E_CART], SQRT(diff2) * inverse_secs, active_extruder, MM_PER_ARC_SEGMENT);
    #elif ENABLED(DELTA_FEEDRATE_SCALING)
      const float diff2 = sq(delta[A_AXIS] - oldA) + sq(delta[B_AXIS] - oldB) + sq(delta[C_AXIS] - oldC);
      if (diff2)
        planner.buffer_segment(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], cart[E_CART], SQRT(diff2) * inverse_secs, active_extruder, MM_PER_ARC_SEGMENT);
    #elif HAS_UBL_AND_CURVES
      float pos[XYZ] = { cart[X_AXIS], cart[Y_AXIS], cart[Z_AXIS] };
      planner.apply_leveling(pos);
      planner.buffer_segment(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], cart[E_CART], fr_mm_s, active_extruder, MM_PER_ARC_SEGMENT);
    #else
      planner.buffer_line_kinematic(cart, fr_mm_s, active_extruder);
    #endif

    COPY(current_position, cart);
  } // plan_arc

#endif // ARC_SUPPORT

#if ENABLED(BEZIER_CURVE_SUPPORT)

  void plan_cubic_move(const float (&cart)[XYZE], const float (&offset)[4]) {
    cubic_b_spline(current_position, cart, offset, MMS_SCALED(feedrate_mm_s), active_extruder);
    COPY(current_position, cart);
  }

#endif // BEZIER_CURVE_SUPPORT

#if ENABLED(MORGAN_SCARA)

  /**
   * Morgan SCARA Forward Kinematics. Results in cartes[].
   * Maths and first version by QHARLEY.
   * Integrated into Marlin and slightly restructured by Joachim Cerny.
   */
  void forward_kinematics_SCARA(const float &a, const float &b) {

    float a_sin = sin(RADIANS(a)) * L1,
          a_cos = cos(RADIANS(a)) * L1,
          b_sin = sin(RADIANS(b)) * L2,
          b_cos = cos(RADIANS(b)) * L2;

    cartes[X_AXIS] = a_cos + b_cos + SCARA_OFFSET_X;  //theta
    cartes[Y_AXIS] = a_sin + b_sin + SCARA_OFFSET_Y;  //theta+phi

    /*
      SERIAL_ECHOPAIR("SCARA FK Angle a=", a);
      SERIAL_ECHOPAIR(" b=", b);
      SERIAL_ECHOPAIR(" a_sin=", a_sin);
      SERIAL_ECHOPAIR(" a_cos=", a_cos);
      SERIAL_ECHOPAIR(" b_sin=", b_sin);
      SERIAL_ECHOLNPAIR(" b_cos=", b_cos);
      SERIAL_ECHOPAIR(" cartes[X_AXIS]=", cartes[X_AXIS]);
      SERIAL_ECHOLNPAIR(" cartes[Y_AXIS]=", cartes[Y_AXIS]);
    //*/
  }

  /**
   * Morgan SCARA Inverse Kinematics. Results in delta[].
   *
   * See http://forums.reprap.org/read.php?185,283327
   *
   * Maths and first version by QHARLEY.
   * Integrated into Marlin and slightly restructured by Joachim Cerny.
   */
  void inverse_kinematics(const float raw[XYZ]) {

    static float C2, S2, SK1, SK2, THETA, PSI;

    float sx = raw[X_AXIS] - SCARA_OFFSET_X,  // Translate SCARA to standard X Y
          sy = raw[Y_AXIS] - SCARA_OFFSET_Y;  // With scaling factor.

    if (L1 == L2)
      C2 = HYPOT2(sx, sy) / L1_2_2 - 1;
    else
      C2 = (HYPOT2(sx, sy) - (L1_2 + L2_2)) / (2.0 * L1 * L2);

    S2 = SQRT(1 - sq(C2));

    // Unrotated Arm1 plus rotated Arm2 gives the distance from Center to End
    SK1 = L1 + L2 * C2;

    // Rotated Arm2 gives the distance from Arm1 to Arm2
    SK2 = L2 * S2;

    // Angle of Arm1 is the difference between Center-to-End angle and the Center-to-Elbow
    THETA = ATAN2(SK1, SK2) - ATAN2(sx, sy);

    // Angle of Arm2
    PSI = ATAN2(S2, C2);

    delta[A_AXIS] = DEGREES(THETA);        // theta is support arm angle
    delta[B_AXIS] = DEGREES(THETA + PSI);  // equal to sub arm angle (inverted motor)
    delta[C_AXIS] = raw[Z_AXIS];

    /*
      DEBUG_POS("SCARA IK", raw);
      DEBUG_POS("SCARA IK", delta);
      SERIAL_ECHOPAIR("  SCARA (x,y) ", sx);
      SERIAL_ECHOPAIR(",", sy);
      SERIAL_ECHOPAIR(" C2=", C2);
      SERIAL_ECHOPAIR(" S2=", S2);
      SERIAL_ECHOPAIR(" Theta=", THETA);
      SERIAL_ECHOLNPAIR(" Phi=", PHI);
    //*/
  }

#endif // MORGAN_SCARA

#if ENABLED(TEMP_STAT_LEDS)

  static uint8_t red_led = -1;  // Invalid value to force leds initializzation on startup
  static millis_t next_status_led_update_ms = 0;

  void handle_status_leds(void) {
    if (ELAPSED(millis(), next_status_led_update_ms)) {
      next_status_led_update_ms += 500; // Update every 0.5s
      float max_temp = 0.0;
      #if HAS_HEATED_BED
        max_temp = MAX(thermalManager.degTargetBed(), thermalManager.degBed());
      #endif
      HOTEND_LOOP()
        max_temp = MAX3(max_temp, thermalManager.degHotend(e), thermalManager.degTargetHotend(e));
      const uint8_t new_led = (max_temp > 55.0) ? HIGH : (max_temp < 54.0 || red_led == -1) ? LOW : red_led;
      if (new_led != red_led) {
        red_led = new_led;
        #if PIN_EXISTS(STAT_LED_RED)
          WRITE(STAT_LED_RED_PIN, new_led);
        #endif
        #if PIN_EXISTS(STAT_LED_BLUE)
          WRITE(STAT_LED_BLUE_PIN, !new_led);
        #endif
      }
    }
  }

#endif

void enable_all_steppers() {
  #if ENABLED(AUTO_POWER_CONTROL)
    powerManager.power_on();
  #endif
  #if ENABLED(HANGPRINTER)
    enable_A();
    enable_B();
    enable_C();
    enable_D();
  #else
    enable_X();
    enable_Y();
    enable_Z();
    enable_E4();
  #endif
  enable_E0();
  enable_E1();
  enable_E2();
  enable_E3();
}

void disable_e_stepper(const uint8_t e) {
  switch (e) {
    case 0: disable_E0(); break;
    case 1: disable_E1(); break;
    case 2: disable_E2(); break;
    case 3: disable_E3(); break;
    case 4: disable_E4(); break;
  }
}

void disable_e_steppers() {
  disable_E0();
  disable_E1();
  disable_E2();
  disable_E3();
  disable_E4();
}

void disable_all_steppers() {
  disable_X();
  disable_Y();
  disable_Z();
  disable_e_steppers();
}

/**
 * Manage several activities:
 *  - Check for Filament Runout
 *  - Keep the command buffer full
 *  - Check for maximum inactive time between commands
 *  - Check for maximum inactive time between stepper commands
 *  - Check if pin CHDK needs to go LOW
 *  - Check for KILL button held down
 *  - Check for HOME button held down
 *  - Check if cooling fan needs to be switched on
 *  - Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 */
void manage_inactivity(const bool ignore_stepper_queue/*=false*/) {

  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    runout.run();
  #endif

  if (commands_in_queue < BUFSIZE) get_available_commands();

  const millis_t ms = millis();

  if (max_inactive_time && ELAPSED(ms, previous_move_ms + max_inactive_time)) {
    SERIAL_ERROR_START();
    SERIAL_ECHOLNPAIR(MSG_KILL_INACTIVE_TIME, parser.command_ptr);
    kill(PSTR(MSG_KILLED));
  }

  // Prevent steppers timing-out in the middle of M600
  #if ENABLED(ADVANCED_PAUSE_FEATURE) && ENABLED(PAUSE_PARK_NO_STEPPER_TIMEOUT)
    #define MOVE_AWAY_TEST !did_pause_print
  #else
    #define MOVE_AWAY_TEST true
  #endif

  if (stepper_inactive_time) {
    if (planner.has_blocks_queued())
      previous_move_ms = ms; // reset_stepper_timeout to keep steppers powered
    else if (MOVE_AWAY_TEST && !ignore_stepper_queue && ELAPSED(ms, previous_move_ms + stepper_inactive_time)) {
      #if ENABLED(DISABLE_INACTIVE_X)
        disable_X();
      #endif
      #if ENABLED(DISABLE_INACTIVE_Y)
        disable_Y();
      #endif
      #if ENABLED(DISABLE_INACTIVE_Z)
        disable_Z();
      #endif
      #if ENABLED(DISABLE_INACTIVE_E)
        disable_e_steppers();
      #endif
      #if ENABLED(AUTO_BED_LEVELING_UBL) && ENABLED(ULTIPANEL)  // Only needed with an LCD
        if (ubl.lcd_map_control) ubl.lcd_map_control = defer_return_to_status = false;
      #endif
    }
  }

  #ifdef CHDK // Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive && ELAPSED(ms, chdkHigh + CHDK_DELAY)) {
      chdkActive = false;
      WRITE(CHDK, LOW);
    }
  #endif

  #if HAS_KILL

    // Check if the kill button was pressed and wait just in case it was an accidental
    // key kill key press
    // -------------------------------------------------------------------------------
    static int killCount = 0;   // make the inactivity button a bit less responsive
    const int KILL_DELAY = 750;
    if (!READ(KILL_PIN))
      killCount++;
    else if (killCount > 0)
      killCount--;

    // Exceeded threshold and we can confirm that it was not accidental
    // KILL the machine
    // ----------------------------------------------------------------
    if (killCount >= KILL_DELAY) {
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM(MSG_KILL_BUTTON);
      kill(PSTR(MSG_KILLED));
    }
  #endif

  #if HAS_HOME
    // Check to see if we have to home, use poor man's debouncer
    // ---------------------------------------------------------
    static int homeDebounceCount = 0;   // poor man's debouncing count
    const int HOME_DEBOUNCE_DELAY = 2500;
    if (!IS_SD_PRINTING() && !READ(HOME_PIN)) {
      if (!homeDebounceCount) {
        enqueue_and_echo_commands_P(PSTR("G28"));
        LCD_MESSAGEPGM(MSG_AUTO_HOME);
      }
      if (homeDebounceCount < HOME_DEBOUNCE_DELAY)
        homeDebounceCount++;
      else
        homeDebounceCount = 0;
    }
  #endif

  #if HAS_CONTROLLER_FAN
    thermalManager.controllerFan(); // Check if fan should be turned on to cool stepper drivers down
  #endif

  #if ENABLED(AUTO_POWER_CONTROL)
    powerManager.check();
  #endif

  #if ENABLED(EXTRUDER_RUNOUT_PREVENT)
    if (thermalManager.degHotend(active_extruder) > EXTRUDER_RUNOUT_MINTEMP
      && ELAPSED(ms, previous_move_ms + (EXTRUDER_RUNOUT_SECONDS) * 1000UL)
      && !planner.has_blocks_queued()
    ) {
      #if ENABLED(SWITCHING_EXTRUDER)
        bool oldstatus;
        switch (active_extruder) {
          default: oldstatus = E0_ENABLE_READ; enable_E0(); break;
          #if E_STEPPERS > 1
            case 2: case 3: oldstatus = E1_ENABLE_READ; enable_E1(); break;
            #if E_STEPPERS > 2
              case 4: oldstatus = E2_ENABLE_READ; enable_E2(); break;
            #endif // E_STEPPERS > 2
          #endif // E_STEPPERS > 1
        }
      #else // !SWITCHING_EXTRUDER
        bool oldstatus;
        switch (active_extruder) {
          default: oldstatus = E0_ENABLE_READ; enable_E0(); break;
          #if E_STEPPERS > 1
            case 1: oldstatus = E1_ENABLE_READ; enable_E1(); break;
            #if E_STEPPERS > 2
              case 2: oldstatus = E2_ENABLE_READ; enable_E2(); break;
              #if E_STEPPERS > 3
                case 3: oldstatus = E3_ENABLE_READ; enable_E3(); break;
                #if E_STEPPERS > 4
                  case 4: oldstatus = E4_ENABLE_READ; enable_E4(); break;
                #endif // E_STEPPERS > 4
              #endif // E_STEPPERS > 3
            #endif // E_STEPPERS > 2
          #endif // E_STEPPERS > 1
        }
      #endif // !SWITCHING_EXTRUDER

      const float olde = current_position[E_CART];
      current_position[E_CART] += EXTRUDER_RUNOUT_EXTRUDE;
      planner.buffer_line_kinematic(current_position, MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED), active_extruder);
      current_position[E_CART] = olde;
      planner.set_e_position_mm(olde);
      planner.synchronize();

      #if ENABLED(SWITCHING_EXTRUDER)
        switch (active_extruder) {
          default: oldstatus = E0_ENABLE_WRITE(oldstatus); break;
          #if E_STEPPERS > 1
            case 2: case 3: oldstatus = E1_ENABLE_WRITE(oldstatus); break;
            #if E_STEPPERS > 2
              case 4: oldstatus = E2_ENABLE_WRITE(oldstatus); break;
            #endif // E_STEPPERS > 2
          #endif // E_STEPPERS > 1
        }
      #else // !SWITCHING_EXTRUDER
        switch (active_extruder) {
          case 0: E0_ENABLE_WRITE(oldstatus); break;
          #if E_STEPPERS > 1
            case 1: E1_ENABLE_WRITE(oldstatus); break;
            #if E_STEPPERS > 2
              case 2: E2_ENABLE_WRITE(oldstatus); break;
              #if E_STEPPERS > 3
                case 3: E3_ENABLE_WRITE(oldstatus); break;
                #if E_STEPPERS > 4
                  case 4: E4_ENABLE_WRITE(oldstatus); break;
                #endif // E_STEPPERS > 4
              #endif // E_STEPPERS > 3
            #endif // E_STEPPERS > 2
          #endif // E_STEPPERS > 1
        }
      #endif // !SWITCHING_EXTRUDER

      previous_move_ms = ms; // reset_stepper_timeout to keep steppers powered
    }
  #endif // EXTRUDER_RUNOUT_PREVENT

  #if ENABLED(DUAL_X_CARRIAGE)
    // handle delayed move timeout
    if (delayed_move_time && ELAPSED(ms, delayed_move_time + 1000UL) && IsRunning()) {
      // travel moves have been received so enact them
      delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      set_destination_from_current();
      prepare_move_to_destination();
    }
  #endif

  #if ENABLED(TEMP_STAT_LEDS)
    handle_status_leds();
  #endif

  #if ENABLED(MONITOR_DRIVER_STATUS)
    monitor_tmc_driver();
  #endif

  planner.check_axes_activity();
}

/**
 * Standard idle routine keeps the machine alive
 */
void idle(
  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    bool no_stepper_sleep/*=false*/
  #endif
) {
  #if ENABLED(MAX7219_DEBUG)
    max7219.idle_tasks();
  #endif

  lcd_update();

  host_keepalive();

  manage_inactivity(
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      no_stepper_sleep
    #endif
  );

  thermalManager.manage_heater();

  #if ENABLED(PRINTCOUNTER)
    print_job_timer.tick();
  #endif

  #if HAS_BUZZER && DISABLED(LCD_USE_I2C_BUZZER)
    buzzer.tick();
  #endif

  #if ENABLED(I2C_POSITION_ENCODERS)
    static millis_t i2cpem_next_update_ms;
    if (planner.has_blocks_queued() && ELAPSED(millis(), i2cpem_next_update_ms)) {
      I2CPEM.update();
      i2cpem_next_update_ms = millis() + I2CPE_MIN_UPD_TIME_MS;
    }
  #endif

  #if HAS_AUTO_REPORTING
    if (!suspend_auto_report) {
      #if ENABLED(AUTO_REPORT_TEMPERATURES)
        thermalManager.auto_report_temperatures();
      #endif
      #if ENABLED(AUTO_REPORT_SD_STATUS)
        card.auto_report_sd_status();
      #endif
    }
  #endif
}

/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void kill(const char* lcd_msg) {
  SERIAL_ERROR_START();
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);

  thermalManager.disable_all_heaters();
  disable_all_steppers();

  #if ENABLED(ULTRA_LCD)
    kill_screen(lcd_msg);
  #else
    UNUSED(lcd_msg);
  #endif

  _delay_ms(600); // Wait a short time (allows messages to get out before shutting down.
  cli(); // Stop interrupts

  _delay_ms(250); //Wait to ensure all interrupts routines stopped
  thermalManager.disable_all_heaters(); //turn off heaters again

  #ifdef ACTION_ON_KILL
    SERIAL_ECHOLNPGM("//action:" ACTION_ON_KILL);
  #endif

  #if HAS_POWER_SWITCH
    PSU_OFF();
  #endif

  suicide();
  while (1) {
    #if ENABLED(USE_WATCHDOG)
      watchdog_reset();
    #endif
  } // Wait for reset
}

/**
 * Turn off heaters and stop the print in progress
 * After a stop the machine may be resumed with M999
 */
void stop() {
  thermalManager.disable_all_heaters(); // 'unpause' taken care of in here

  #if ENABLED(PROBING_FANS_OFF)
    thermalManager.pause_fans(false); // put things back the way they were
  #endif

  if (IsRunning()) {
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
    safe_delay(350);       // allow enough time for messages to get out before stopping
    Running = false;
  }
}

/**
 * Marlin entry-point: Set up before the program loop
 *  - Set up the kill pin, filament runout, power hold
 *  - Start the serial port
 *  - Print startup messages and diagnostics
 *  - Get EEPROM or default settings
 *  - Initialize managers for:
 *    • temperature
 *    • planner
 *    • watchdog
 *    • stepper
 *    • photo pin
 *    • servos
 *    • LCD controller
 *    • Digipot I2C
 *    • Z probe sled
 *    • status LEDs
 */
void setup() {

  #if ENABLED(MAX7219_DEBUG)
    max7219.init();
  #endif

  #if ENABLED(DISABLE_JTAG)
    // Disable JTAG on AT90USB chips to free up pins for IO
    MCUCR = 0x80;
    MCUCR = 0x80;
  #endif

  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    runout.setup();
  #endif

  setup_killpin();

  setup_powerhold();

  #if HAS_STEPPER_RESET
    disableStepperDrivers();
  #endif

  MYSERIAL0.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START();

  // Prepare communication for TMC drivers
  #if HAS_DRIVER(TMC2130)
    tmc_init_cs_pins();
  #endif
  #if HAS_DRIVER(TMC2208)
    tmc2208_serial_begin();
  #endif

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if (mcu &  1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if (mcu &  2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if (mcu &  4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if (mcu &  8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if (mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR = 0;

  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_CHAR(' ');
  SERIAL_ECHOLNPGM(SHORT_BUILD_VERSION);
  SERIAL_EOL();

  #if defined(STRING_DISTRIBUTION_DATE) && defined(STRING_CONFIG_H_AUTHOR)
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
    SERIAL_ECHOPGM(STRING_DISTRIBUTION_DATE);
    SERIAL_ECHOLNPGM(MSG_AUTHOR STRING_CONFIG_H_AUTHOR);
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM("Compiled: " __DATE__);
  #endif

  SERIAL_ECHO_START();
  SERIAL_ECHOPAIR(MSG_FREE_MEMORY, freeMemory());
  SERIAL_ECHOLNPAIR(MSG_PLANNER_BUFFER_BYTES, int(sizeof(block_t))*(BLOCK_BUFFER_SIZE));

  queue_setup();

  // Load data from EEPROM if available (or use defaults)
  // This also updates variables in the planner, elsewhere
  (void)settings.load();

   ZERO(current_position);

  // Vital to init stepper/planner equivalent for current_position
  SYNC_PLAN_POSITION_KINEMATIC();

  thermalManager.init();    // Initialize temperature loop

  print_job_timer.init();   // Initial setup of print job timer

  endstops.init();          // Init endstops and pullups

  stepper.init();           // Init stepper. This enables interrupts!


  #if HAS_PHOTOGRAPH
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  #endif

  #if HAS_CASE_LIGHT
    case_light_on = CASE_LIGHT_DEFAULT_ON;
    case_light_brightness = CASE_LIGHT_DEFAULT_BRIGHTNESS;
    update_case_light();
  #endif

  #if ENABLED(SPINDLE_LASER_ENABLE)
    OUT_WRITE(SPINDLE_LASER_ENABLE_PIN, !SPINDLE_LASER_ENABLE_INVERT);  // init spindle to off
    #if SPINDLE_DIR_CHANGE
      OUT_WRITE(SPINDLE_DIR_PIN, SPINDLE_INVERT_DIR ? 255 : 0);  // init rotation to clockwise (M3)
    #endif
    #if ENABLED(SPINDLE_LASER_PWM)
      SET_OUTPUT(SPINDLE_LASER_PWM_PIN);
      analogWrite(SPINDLE_LASER_PWM_PIN, SPINDLE_LASER_PWM_INVERT ? 255 : 0);  // set to lowest speed
    #endif
  #endif

  #if HAS_BED_PROBE
    endstops.enable_z_probe(false);
  #endif

  #if ENABLED(USE_CONTROLLER_FAN)
    SET_OUTPUT(CONTROLLER_FAN_PIN); //Set pin used for driver cooling fan
  #endif

  #if HAS_STEPPER_RESET
    enableStepperDrivers();
  #endif

  #if ENABLED(DIGIPOT_I2C)
    digipot_i2c_init();
  #endif

  #if ENABLED(DAC_STEPPER_CURRENT)
    dac_init();
  #endif

  #if (ENABLED(Z_PROBE_SLED) || ENABLED(SOLENOID_PROBE)) && HAS_SOLENOID_1
    OUT_WRITE(SOL1_PIN, LOW); // turn it off
  #endif

  #if HAS_HOME
    SET_INPUT_PULLUP(HOME_PIN);
  #endif

  #if PIN_EXISTS(STAT_LED_RED)
    OUT_WRITE(STAT_LED_RED_PIN, LOW); // turn it off
  #endif

  #if PIN_EXISTS(STAT_LED_BLUE)
    OUT_WRITE(STAT_LED_BLUE_PIN, LOW); // turn it off
  #endif

  #if HAS_COLOR_LEDS
    leds.setup();
  #endif

  #if ENABLED(RGB_LED) || ENABLED(RGBW_LED)
    SET_OUTPUT(RGB_LED_R_PIN);
    SET_OUTPUT(RGB_LED_G_PIN);
    SET_OUTPUT(RGB_LED_B_PIN);
    #if ENABLED(RGBW_LED)
      SET_OUTPUT(RGB_LED_W_PIN);
    #endif
  #endif

  #if ENABLED(MK2_MULTIPLEXER)
    SET_OUTPUT(E_MUX0_PIN);
    SET_OUTPUT(E_MUX1_PIN);
    SET_OUTPUT(E_MUX2_PIN);
  #endif

  #if HAS_FANMUX
    fanmux_init();
  #endif
  
  lcd_init();
  lcd_reset_status();

  #if ENABLED(SHOW_BOOTSCREEN)
    lcd_bootscreen();
  #endif

  #if ENABLED(MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1
    // Virtual Tools 0, 1, 2, 3 = Filament 1, 2, 3, 4, etc.
    for (uint8_t t = 0; t < MIXING_VIRTUAL_TOOLS && t < MIXING_STEPPERS; t++)
      for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
        mixing_virtual_tool_mix[t][i] = (t == i) ? 1.0 : 0.0;

    // Remaining virtual tools are 100% filament 1
    #if MIXING_STEPPERS < MIXING_VIRTUAL_TOOLS
      for (uint8_t t = MIXING_STEPPERS; t < MIXING_VIRTUAL_TOOLS; t++)
        for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
          mixing_virtual_tool_mix[t][i] = (i == 0) ? 1.0 : 0.0;
    #endif

    // Initialize mixing to tool 0 color
    for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
      mixing_factor[i] = mixing_virtual_tool_mix[0][i];
  #endif

  #if ENABLED(BLTOUCH)
    // Make sure any BLTouch error condition is cleared
    bltouch_command(BLTOUCH_RESET);
    set_bltouch_deployed(false);
  #endif

  #if ENABLED(I2C_POSITION_ENCODERS)
    I2CPEM.init();
  #endif

  #if ENABLED(EXPERIMENTAL_I2CBUS) && I2C_SLAVE_ADDRESS > 0
    i2c.onReceive(i2c_on_receive);
    i2c.onRequest(i2c_on_request);
  #endif

  #if DO_SWITCH_EXTRUDER
    move_extruder_servo(0);  // Initialize extruder servo
  #endif

  #if ENABLED(SWITCHING_NOZZLE)
    move_nozzle_servo(0);  // Initialize nozzle servo
  #endif

  #if ENABLED(PARKING_EXTRUDER)
    #if ENABLED(PARKING_EXTRUDER_SOLENOIDS_INVERT)
      pe_activate_magnet(0);
      pe_activate_magnet(1);
    #else
      pe_deactivate_magnet(0);
      pe_deactivate_magnet(1);
    #endif
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    check_print_job_recovery();
  #endif

  #if ENABLED(USE_WATCHDOG)
    watchdog_init();
  #endif

  #if ENABLED(HANGPRINTER)
    enable_A();
    enable_B();
    enable_C();
    enable_D();
  #endif

  #if ENABLED(SDSUPPORT) && DISABLED(ULTRA_LCD)
    card.beginautostart();
  #endif
}

/**
 * The main Marlin program loop
 *
 *  - Abort SD printing if flagged
 *  - Save or log commands to SD
 *  - Process available commands (if not saving)
 *  - Call heater manager
 *  - Call inactivity manager
 *  - Call endstop manager
 *  - Call LCD update
 */
void loop() {

  #if ENABLED(SDSUPPORT)

    card.checkautostart();

    if (card.abort_sd_printing) {
      card.stopSDPrint(
        #if SD_RESORT
          true
        #endif
      );
      clear_command_queue();
      quickstop_stepper();
      print_job_timer.stop();
      thermalManager.disable_all_heaters();
      thermalManager.halt_fans();
      wait_for_heatup = false;
      #if ENABLED(POWER_LOSS_RECOVERY)
        card.removeJobRecoveryFile();
      #endif
    }

  #endif // SDSUPPORT

  if (commands_in_queue < BUFSIZE) get_available_commands();
  advance_command_queue();
  endstops.event_handler();
  idle();
}
