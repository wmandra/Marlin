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
#pragma once

#include "../inc/MarlinConfig.h"

/**
 * GCode line number handling. Hosts may opt to include line numbers when
 * sending commands to Marlin, and lines will be checked for sequentiality.
 * M110 N<int> sets the current line number.
 */
extern long gcode_LastN, Stopped_gcode_LastN;

/**
 * GCode Command Queue
 * A simple ring buffer of BUFSIZE command strings.
 *
 * Commands are copied into this buffer by the command injectors
 * (immediate, serial, sd card) and they are processed sequentially by
 * the main loop. The process_next_command function parses the next
 * command and hands off execution to individual handler functions.
 */
extern uint8_t commands_in_queue, // Count of commands in the queue
               cmd_queue_index_r; // Ring buffer read (out) position

extern char command_queue[BUFSIZE][MAX_CMD_SIZE];

void queue_setup();
bool enqueue_and_echo_command(const char* cmd);           // Add a single command to the end of the buffer. Return false on failure.
void enqueue_and_echo_commands_P(const char * const cmd); // Set one or more commands to be prioritized over the next Serial/SD command.
void clear_command_queue();

void flush_and_request_resend();
void ok_to_send();

void get_available_commands();
void process_next_command();
void advance_command_queue();

#define HAS_LCD_QUEUE_NOW (ENABLED(ULTIPANEL) && (ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(PID_AUTOTUNE_MENU) || ENABLED(ADVANCED_PAUSE_FEATURE)))
#define HAS_QUEUE_NOW (ENABLED(SDSUPPORT) || HAS_LCD_QUEUE_NOW)
#if HAS_QUEUE_NOW
  // Return only when commands are actually enqueued
  void enqueue_and_echo_command_now(const char* cmd);
  #if HAS_LCD_QUEUE_NOW
    void enqueue_and_echo_commands_now_P(const char * const cmd);
  #endif
#endif
