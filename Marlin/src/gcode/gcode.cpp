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

#include "gcode.h"
GcodeSuite gcode;

#include "parser.h"
#include "queue.h"

#include "../Marlin.h" // for idle() and suspend_auto_report

millis_t GcodeSuite::previous_move_ms;

bool GcodeSuite::axis_relative_modes[] = AXIS_RELATIVE_MODES;

/**
 * Set target_extruder from the T parameter or the active_extruder
 *
 * Returns TRUE if the target is invalid
 */
int8_t GcodeSuite::get_target_extruder_from_command() {
  // if (parser.seenval('T')) {
  //   const int8_t e = parser.value_byte();
  //   if (e >= EXTRUDERS) {
  //     SERIAL_ECHO_START();
  //     SERIAL_CHAR('M');
  //     SERIAL_ECHO(code);
  //     SERIAL_ECHOLNPAIR(" " MSG_INVALID_EXTRUDER " ", e);
  //     return true;
  //   }
  //   target_extruder = e;
  // }
  // else
  //   target_extruder = active_extruder;

  // return false;
}


/**
 * Process the parsed command and dispatch it to its handler
 */
void GcodeSuite::process_parsed_command(
      #if USE_EXECUTE_COMMANDS_IMMEDIATE
      const bool no_ok = false
    #endif
  ) {
  // KEEPALIVE_STATE(IN_HANDLER);

  // // Handle a known G, M, or T
  // switch (parser.command_letter) {
  //   case 'G': switch (parser.codenum) {

  //     case 0: case 1: gcode_G0_G1(                                // G0: Fast Move, G1: Linear Move
  //                       #if IS_SCARA
  //                         parser.codenum == 0
  //                       #endif
  //                     ); break;

  //     #if ENABLED(ARC_SUPPORT) && DISABLED(SCARA)
  //       case 2: case 3: gcode_G2_G3(parser.codenum == 2); break;  // G2: CW ARC, G3: CCW ARC
  //     #endif

  //     case 4: gcode_G4(); break;                                  // G4: Dwell

  //     #if ENABLED(BEZIER_CURVE_SUPPORT)
  //       case 5: gcode_G5(); break;                                // G5: Cubic B_spline
  //     #endif

  //     #if ENABLED(UNREGISTERED_MOVE_SUPPORT)
  //       case 6: gcode_G6(); break;                                // G6: Direct stepper move
  //     #endif

  //     #if ENABLED(FWRETRACT)
  //       case 10: gcode_G10(); break;                              // G10: Retract
  //       case 11: gcode_G11(); break;                              // G11: Prime
  //     #endif

  //     #if ENABLED(NOZZLE_CLEAN_FEATURE)
  //       case 12: gcode_G12(); break;                              // G12: Clean Nozzle
  //     #endif

  //     #if ENABLED(CNC_WORKSPACE_PLANES)
  //       case 17: gcode_G17(); break;                              // G17: Select Plane XY
  //       case 18: gcode_G18(); break;                              // G18: Select Plane ZX
  //       case 19: gcode_G19(); break;                              // G19: Select Plane YZ
  //     #endif

  //     #if ENABLED(INCH_MODE_SUPPORT)
  //       case 20: gcode_G20(); break;                              // G20: Inch Units
  //       case 21: gcode_G21(); break;                              // G21: Millimeter Units
  //     #endif

  //     #if ENABLED(G26_MESH_VALIDATION)
  //       case 26: gcode_G26(); break;                              // G26: Mesh Validation Pattern
  //     #endif

  //     #if ENABLED(NOZZLE_PARK_FEATURE)
  //       case 27: gcode_G27(); break;                              // G27: Park Nozzle
  //     #endif

  //     case 28: gcode_G28(false); break;                           // G28: Home one or more axes

  //     #if HAS_LEVELING
  //       case 29: gcode_G29(); break;                              // G29: Detailed Z probe
  //     #endif

  //     #if HAS_BED_PROBE
  //       case 30: gcode_G30(); break;                              // G30: Single Z probe
  //     #endif

  //     #if ENABLED(Z_PROBE_SLED)
  //       case 31: gcode_G31(); break;                              // G31: Dock sled
  //       case 32: gcode_G32(); break;                              // G32: Undock sled
  //     #endif

  //     #if ENABLED(DELTA_AUTO_CALIBRATION)
  //       case 33: gcode_G33(); break;                              // G33: Delta Auto-Calibration
  //     #endif

  //     #if ENABLED(G38_PROBE_TARGET)
  //       case 38:
  //         if (parser.subcode == 2 || parser.subcode == 3)
  //           gcode_G38(parser.subcode == 2);                       // G38.2, G38.3: Probe towards object
  //         break;
  //     #endif

  //     #if HAS_MESH
  //       case 42: gcode_G42(); break;                              // G42: Move to mesh point
  //     #endif

  //     case 90: relative_mode = false; break;                      // G90: Absolute coordinates
  //     case 91: relative_mode = true; break;                       // G91: Relative coordinates

  //     case 92: gcode_G92(); break;                                // G92: Set Position
  //     #if ENABLED(MECHADUINO_I2C_COMMANDS)
  //       case 95: gcode_G95(); break;                                // G95: Set torque mode
  //       case 96: gcode_G96(); break;                                // G96: Mark encoder reference point
  //     #endif

  //     #if ENABLED(DEBUG_GCODE_PARSER)
  //       case 800: parser.debug(); break;                          // G800: GCode Parser Test for G
  //     #endif

  //     default: parser.unknown_command_error();
  //   }
  //   break;

  //   case 'M': switch (parser.codenum) {
  //     #if HAS_RESUME_CONTINUE
  //       case 0: case 1: gcode_M0_M1(); break;                     // M0: Unconditional stop, M1: Conditional stop
  //     #endif

  //     #if ENABLED(SPINDLE_LASER_ENABLE)
  //       case 3: gcode_M3_M4(true); break;                         // M3: Laser/CW-Spindle Power
  //       case 4: gcode_M3_M4(false); break;                        // M4: Laser/CCW-Spindle Power
  //       case 5: gcode_M5(); break;                                // M5: Laser/Spindle OFF
  //     #endif

  //     case 17: gcode_M17(); break;                                // M17: Enable all steppers

  //     #if ENABLED(SDSUPPORT)
  //       case 20: gcode_M20(); break;                              // M20: List SD Card
  //       case 21: gcode_M21(); break;                              // M21: Init SD Card
  //       case 22: gcode_M22(); break;                              // M22: Release SD Card
  //       case 23: gcode_M23(); break;                              // M23: Select File
  //       case 24: gcode_M24(); break;                              // M24: Start SD Print
  //       case 25: gcode_M25(); break;                              // M25: Pause SD Print
  //       case 26: gcode_M26(); break;                              // M26: Set SD Index
  //       case 27: gcode_M27(); break;                              // M27: Get SD Status
  //       case 28: gcode_M28(); break;                              // M28: Start SD Write
  //       case 29: gcode_M29(); break;                              // M29: Stop SD Write
  //       case 30: gcode_M30(); break;                              // M30: Delete File
  //       case 32: gcode_M32(); break;                              // M32: Select file, Start SD Print
  //       #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
  //         case 33: gcode_M33(); break;                            // M33: Report longname path
  //       #endif
  //       #if ENABLED(SDCARD_SORT_ALPHA) && ENABLED(SDSORT_GCODE)
  //         case 34: gcode_M34(); break;                            // M34: Set SD card sorting options
  //       #endif
  //       case 928: gcode_M928(); break;                            // M928: Start SD write
  //     #endif // SDSUPPORT

  //     case 31: gcode_M31(); break;                                // M31: Report print job elapsed time

  //     case 42: gcode_M42(); break;                                // M42: Change pin state
  //     #if ENABLED(PINS_DEBUGGING)
  //       case 43: gcode_M43(); break;                              // M43: Read/monitor pin and endstop states
  //     #endif

  //     #if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
  //       case 48: gcode_M48(); break;                              // M48: Z probe repeatability test
  //     #endif
  //     #if ENABLED(G26_MESH_VALIDATION)
  //       case 49: gcode_M49(); break;                              // M49: Toggle the G26 Debug Flag
  //     #endif

  //     #if ENABLED(ULTRA_LCD) && ENABLED(LCD_SET_PROGRESS_MANUALLY)
  //       case 73: gcode_M73(); break;                              // M73: Set Print Progress %
  //     #endif
  //     case 75: gcode_M75(); break;                                // M75: Start Print Job Timer
  //     case 76: gcode_M76(); break;                                // M76: Pause Print Job Timer
  //     case 77: gcode_M77(); break;                                // M77: Stop Print Job Timer
  //     #if ENABLED(PRINTCOUNTER)
  //       case 78: gcode_M78(); break;                              // M78: Report Print Statistics
  //     #endif

  //     #if ENABLED(M100_FREE_MEMORY_WATCHER)
  //       case 100: gcode_M100(); break;                            // M100: Free Memory Report
  //     #endif

  //     case 104: gcode_M104(); break;                              // M104: Set Hotend Temperature
  //     case 110: gcode_M110(); break;                              // M110: Set Current Line Number
  //     case 111: gcode_M111(); break;                              // M111: Set Debug Flags

  //     #if DISABLED(EMERGENCY_PARSER)
  //       case 108: gcode_M108(); break;                            // M108: Cancel Waiting
  //       case 112: gcode_M112(); break;                            // M112: Emergency Stop
  //       case 410: gcode_M410(); break;                            // M410: Quickstop. Abort all planned moves
  //     #else
  //       case 108: case 112: case 410: break;                      // Silently drop as handled by emergency parser
  //     #endif

  //     #if ENABLED(HOST_KEEPALIVE_FEATURE)
  //       case 113: gcode_M113(); break;                            // M113: Set Host Keepalive Interval
  //     #endif

  //     case 105: gcode_M105(); KEEPALIVE_STATE(NOT_BUSY); return;  // M105: Report Temperatures (and say "ok")

  //     #if ENABLED(AUTO_REPORT_TEMPERATURES)
  //       case 155: gcode_M155(); break;                            // M155: Set Temperature Auto-report Interval
  //     #endif

  //     case 109: gcode_M109(); break;                              // M109: Set Hotend Temperature. Wait for target.

  //     #if HAS_HEATED_BED
  //       case 140: gcode_M140(); break;                            // M140: Set Bed Temperature
  //       case 190: gcode_M190(); break;                            // M190: Set Bed Temperature. Wait for target.
  //     #endif

  //     #if FAN_COUNT > 0
  //       case 106: gcode_M106(); break;                            // M106: Set Fan Speed
  //       case 107: gcode_M107(); break;                            // M107: Fan Off
  //     #endif

  //     #if ENABLED(PARK_HEAD_ON_PAUSE)
  //       case 125: gcode_M125(); break;                            // M125: Park (for Filament Change)
  //     #endif

  //     #if ENABLED(BARICUDA)
  //       #if HAS_HEATER_1
  //         case 126: gcode_M126(); break;                          // M126: Valve 1 Open
  //         case 127: gcode_M127(); break;                          // M127: Valve 1 Closed
  //       #endif
  //       #if HAS_HEATER_2
  //         case 128: gcode_M128(); break;                          // M128: Valve 2 Open
  //         case 129: gcode_M129(); break;                          // M129: Valve 2 Closed
  //       #endif
  //     #endif

  //     #if HAS_POWER_SWITCH
  //       case 80: gcode_M80(); break;                              // M80: Turn on Power Supply
  //     #endif
  //     case 81: gcode_M81(); break;                                // M81: Turn off Power and Power Supply

  //     case 82: gcode_M82(); break;                                // M82: Disable Relative E-Axis
  //     case 83: gcode_M83(); break;                                // M83: Set Relative E-Axis
  //     case 18: case 84: gcode_M18_M84(); break;                   // M18/M84: Disable Steppers / Set Timeout
  //     case 85: gcode_M85(); break;                                // M85: Set inactivity stepper shutdown timeout
  //     case 92: gcode_M92(); break;                                // M92: Set steps-per-unit
  //     case 114: gcode_M114(); break;                              // M114: Report Current Position
  //     case 115: gcode_M115(); break;                              // M115: Capabilities Report
  //     case 117: gcode_M117(); break;                              // M117: Set LCD message text
  //     case 118: gcode_M118(); break;                              // M118: Print a message in the host console
  //     case 119: gcode_M119(); break;                              // M119: Report Endstop states
  //     case 120: gcode_M120(); break;                              // M120: Enable Endstops
  //     case 121: gcode_M121(); break;                              // M121: Disable Endstops

  //     #if ENABLED(ULTIPANEL)
  //       case 145: gcode_M145(); break;                            // M145: Set material heatup parameters
  //     #endif

  //     #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  //       case 149: gcode_M149(); break;                            // M149: Set Temperature Units, C F K
  //     #endif

  //     #if HAS_COLOR_LEDS
  //       case 150: gcode_M150(); break;                            // M150: Set Status LED Color
  //     #endif

  //     #if ENABLED(MIXING_EXTRUDER)
  //       case 163: gcode_M163(); break;                            // M163: Set Mixing Component
  //       #if MIXING_VIRTUAL_TOOLS > 1
  //         case 164: gcode_M164(); break;                          // M164: Save Current Mix
  //       #endif
  //       #if ENABLED(DIRECT_MIXING_IN_G1)
  //         case 165: gcode_M165(); break;                          // M165: Set Multiple Mixing Components
  //       #endif
  //     #endif

  //     #if DISABLED(NO_VOLUMETRICS)
  //       case 200: gcode_M200(); break;                            // M200: Set Filament Diameter, Volumetric Extrusion
  //     #endif

  //     case 201: gcode_M201(); break;                              // M201: Set Max Printing Acceleration (units/sec^2)
  //     #if 0
  //       case 202: gcode_M202(); break;                            // M202: Not used for Sprinter/grbl gen6
  //     #endif
  //     case 203: gcode_M203(); break;                              // M203: Set Max Feedrate (units/sec)
  //     case 204: gcode_M204(); break;                              // M204: Set Acceleration
  //     case 205: gcode_M205(); break;                              // M205: Set Advanced settings

  //     #if HAS_M206_COMMAND
  //       case 206: gcode_M206(); break;                            // M206: Set Home Offsets
  //       case 428: gcode_M428(); break;                            // M428: Set Home Offsets based on current position
  //     #endif

  //     #if ENABLED(FWRETRACT)
  //       case 207: gcode_M207(); break;                            // M207: Set Retract Length, Feedrate, Z lift
  //       case 208: gcode_M208(); break;                            // M208: Set Additional Prime Length and Feedrate
  //       case 209:
  //         if (MIN_AUTORETRACT <= MAX_AUTORETRACT) gcode_M209();   // M209: Turn Auto-Retract on/off
  //         break;
  //     #endif

  //     case 211: gcode_M211(); break;                              // M211: Enable/Disable/Report Software Endstops

  //     #if HOTENDS > 1
  //       case 218: gcode_M218(); break;                            // M218: Set Tool Offset
  //     #endif

  //     case 220: gcode_M220(); break;                              // M220: Set Feedrate Percentage
  //     case 221: gcode_M221(); break;                              // M221: Set Flow Percentage
  //     case 226: gcode_M226(); break;                              // M226: Wait for Pin State

  //     #if defined(CHDK) || HAS_PHOTOGRAPH
  //       case 240: gcode_M240(); break;                            // M240: Trigger Camera
  //     #endif

  //     #if HAS_LCD_CONTRAST
  //       case 250: gcode_M250(); break;                            // M250: Set LCD Contrast
  //     #endif

  //     #if ENABLED(EXPERIMENTAL_I2CBUS)
  //       case 260: gcode_M260(); break;                            // M260: Send Data to i2c slave
  //       case 261: gcode_M261(); break;                            // M261: Request Data from i2c slave
  //     #endif

  //     #if HAS_SERVOS
  //       case 280: gcode_M280(); break;                            // M280: Set Servo Position
  //     #endif

  //     #if ENABLED(BABYSTEPPING)
  //       case 290: gcode_M290(); break;                            // M290: Babystepping
  //     #endif

  //     #if HAS_BUZZER
  //       case 300: gcode_M300(); break;                            // M300: Add Tone/Buzz to Queue
  //     #endif

  //     #if ENABLED(PIDTEMP)
  //       case 301: gcode_M301(); break;                            // M301: Set Hotend PID parameters
  //     #endif

  //     #if ENABLED(PREVENT_COLD_EXTRUSION)
  //       case 302: gcode_M302(); break;                            // M302: Set Minimum Extrusion Temp
  //     #endif

  //     case 303: gcode_M303(); break;                              // M303: PID Autotune

  //     #if ENABLED(PIDTEMPBED)
  //       case 304: gcode_M304(); break;                            // M304: Set Bed PID parameters
  //     #endif

  //     #if HAS_MICROSTEPS
  //       case 350: gcode_M350(); break;                            // M350: Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
  //       case 351: gcode_M351(); break;                            // M351: Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
  //     #endif

  //     case 355: gcode_M355(); break;                              // M355: Set Case Light brightness

  //     #if ENABLED(MORGAN_SCARA)
  //       case 360: if (gcode_M360()) return; break;                // M360: SCARA Theta pos1
  //       case 361: if (gcode_M361()) return; break;                // M361: SCARA Theta pos2
  //       case 362: if (gcode_M362()) return; break;                // M362: SCARA Psi pos1
  //       case 363: if (gcode_M363()) return; break;                // M363: SCARA Psi pos2
  //       case 364: if (gcode_M364()) return; break;                // M364: SCARA Psi pos3 (90 deg to Theta)
  //     #endif

  //     case 400: gcode_M400(); break;                              // M400: Synchronize. Wait for moves to finish.

  //     #if HAS_BED_PROBE
  //       case 401: gcode_M401(); break;                            // M401: Deploy Probe
  //       case 402: gcode_M402(); break;                            // M402: Stow Probe
  //     #endif

  //     #if ENABLED(FILAMENT_WIDTH_SENSOR)
  //       case 404: gcode_M404(); break;                            // M404: Set/Report Nominal Filament Width
  //       case 405: gcode_M405(); break;                            // M405: Enable Filament Width Sensor
  //       case 406: gcode_M406(); break;                            // M406: Disable Filament Width Sensor
  //       case 407: gcode_M407(); break;                            // M407: Report Measured Filament Width
  //     #endif

  //     #if ENABLED(FILAMENT_RUNOUT_SENSOR)
  //       case 412: gcode_M412(); break;                            // M412: Filament Runout Sensor
  //     #endif

  //     #if HAS_LEVELING
  //       case 420: gcode_M420(); break;                            // M420: Set Bed Leveling Enabled / Fade
  //     #endif

  //     #if HAS_MESH
  //       case 421: gcode_M421(); break;                            // M421: Set a Mesh Z value
  //     #endif

  //     case 500: gcode_M500(); break;                              // M500: Store Settings in EEPROM
  //     case 501: gcode_M501(); break;                              // M501: Read Settings from EEPROM
  //     case 502: gcode_M502(); break;                              // M502: Revert Settings to defaults
  //     #if DISABLED(DISABLE_M503)
  //       case 503: gcode_M503(); break;                            // M503: Report Settings (in SRAM)
  //     #endif
  //     #if ENABLED(EEPROM_SETTINGS)
  //       case 504: gcode_M504(); break;                            // M504: Validate EEPROM
  //     #endif

  //     #if ENABLED(SDSUPPORT)
  //       case 524: gcode_M524(); break;                            // M524: Abort SD print job
  //     #endif

  //     #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
  //       case 540: gcode_M540(); break;                            // M540: Set Abort on Endstop Hit for SD Printing
  //     #endif

  //     #if ENABLED(ADVANCED_PAUSE_FEATURE)
  //       case 600: gcode_M600(); break;                            // M600: Pause for Filament Change
  //       case 603: gcode_M603(); break;                            // M603: Configure Filament Change
  //     #endif

  //     #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
  //       case 605: gcode_M605(); break;                            // M605: Set Dual X Carriage movement mode
  //     #endif

  //     #if ENABLED(DELTA) || ENABLED(HANGPRINTER)
  //       case 665: gcode_M665(); break;                            // M665: Delta / Hangprinter Configuration
  //     #endif
  //     #if ENABLED(DELTA) || ENABLED(X_DUAL_ENDSTOPS) || ENABLED(Y_DUAL_ENDSTOPS) || ENABLED(Z_DUAL_ENDSTOPS)
  //       case 666: gcode_M666(); break;                            // M666: DELTA/Dual Endstop Adjustment
  //     #endif

  //     #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
  //       case 701: gcode_M701(); break;                            // M701: Load Filament
  //       case 702: gcode_M702(); break;                            // M702: Unload Filament
  //     #endif

  //     #if ENABLED(MAX7219_GCODE)
  //       case 7219: gcode_M7219(); break;                          // M7219: Set LEDs, columns, and rows
  //     #endif

  //     #if ENABLED(DEBUG_GCODE_PARSER)
  //       case 800: parser.debug(); break;                          // M800: GCode Parser Test for M
  //     #endif

  //     #if HAS_BED_PROBE
  //       case 851: gcode_M851(); break;                            // M851: Set Z Probe Z Offset
  //     #endif

  //     #if ENABLED(SKEW_CORRECTION_GCODE)
  //       case 852: gcode_M852(); break;                            // M852: Set Skew factors
  //     #endif

  //     #if ENABLED(I2C_POSITION_ENCODERS)
  //       case 860: gcode_M860(); break;                            // M860: Report encoder module position
  //       case 861: gcode_M861(); break;                            // M861: Report encoder module status
  //       case 862: gcode_M862(); break;                            // M862: Perform axis test
  //       case 863: gcode_M863(); break;                            // M863: Calibrate steps/mm
  //       case 864: gcode_M864(); break;                            // M864: Change module address
  //       case 865: gcode_M865(); break;                            // M865: Check module firmware version
  //       case 866: gcode_M866(); break;                            // M866: Report axis error count
  //       case 867: gcode_M867(); break;                            // M867: Toggle error correction
  //       case 868: gcode_M868(); break;                            // M868: Set error correction threshold
  //       case 869: gcode_M869(); break;                            // M869: Report axis error
  //     #endif

  //     #if ENABLED(LIN_ADVANCE)
  //       case 900: gcode_M900(); break;                            // M900: Set Linear Advance K factor
  //     #endif

  //     case 907: gcode_M907(); break;                              // M907: Set Digital Trimpot Motor Current using axis codes.

  //     #if HAS_DIGIPOTSS || ENABLED(DAC_STEPPER_CURRENT)
  //       case 908: gcode_M908(); break;                            // M908: Direct Control Digital Trimpot
  //       #if ENABLED(DAC_STEPPER_CURRENT)
  //         case 909: gcode_M909(); break;                          // M909: Print Digipot/DAC current value (As with Printrbot RevF)
  //         case 910: gcode_M910(); break;                          // M910: Commit Digipot/DAC value to External EEPROM (As with Printrbot RevF)
  //       #endif
  //     #endif

  //     #if HAS_DRIVER(TMC2130) || HAS_DRIVER(TMC2208)
  //       #if ENABLED(TMC_DEBUG)
  //         case 122: gcode_M122(); break;                          // M122: Debug TMC steppers
  //       #endif
  //       case 906: gcode_M906(); break;                            // M906: Set motor current in milliamps using axis codes X, Y, Z, E
  //       case 911: gcode_M911(); break;                            // M911: Report TMC prewarn triggered flags
  //       case 912: gcode_M912(); break;                            // M911: Clear TMC prewarn triggered flags
  //       #if ENABLED(HYBRID_THRESHOLD)
  //         case 913: gcode_M913(); break;                          // M913: Set HYBRID_THRESHOLD speed.
  //       #endif
  //       #if ENABLED(SENSORLESS_HOMING)
  //         case 914: gcode_M914(); break;                          // M914: Set SENSORLESS_HOMING sensitivity.
  //       #endif
  //       #if ENABLED(TMC_Z_CALIBRATION)
  //         case 915: gcode_M915(); break;                          // M915: TMC Z axis calibration routine
  //       #endif
  //     #endif

  //     case 999: gcode_M999(); break;                              // M999: Restart after being Stopped

  //     default: parser.unknown_command_error();
  //   }
  //   break;

  //   case 'T': gcode_T(parser.codenum); break;                     // T: Tool Select

  //   default: parser.unknown_command_error();
  // }

  // KEEPALIVE_STATE(NOT_BUSY);
  // ok_to_send();
}