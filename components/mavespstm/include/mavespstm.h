/**
 * @file mavespstm.h
 * @brief MAVLink ESP32/STM32 Library - Main Header
 * 
 * mavespstm - A lightweight MAVLink library for ESP32 and STM32 companion computers.
 * 
 * This library provides:
 * - MAVLink 2 message parsing
 * - Message packing and CRC calculation
 * - Common message type definitions
 * 
 * Usage:
 *   #include "mavespstm.h"
 * 
 * @version 1.0.0
 */

#ifndef MAVESPSTM_H
#define MAVESPSTM_H

// Include all mavespstm components
#include "mavespstm/mavlink_types.h"
#include "mavespstm/mavlink_checksum.h"
#include "mavespstm/mavlink_parser.h"
#include "mavespstm/mavlink_messages.h"

#endif // MAVESPSTM_H
