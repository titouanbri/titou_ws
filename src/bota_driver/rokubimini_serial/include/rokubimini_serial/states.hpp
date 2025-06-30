/*
 * states.hpp
 *
 *  Created on:   Dec 19, 2016
 *  Author(s):    Christian Gehring
 */

#pragma once

namespace rokubimini
{
namespace serial
{
enum class ConnectionState : unsigned int
{
  DISCONNECTED = 0,
  ISCONNECTING,
  CONNECTED
};

enum class ModeState : unsigned int
{
  RUN_MODE = 0,
  CONFIG_MODE,
  INIT_MODE
};

enum class ErrorState : unsigned int
{
  NO_ERROR = 0,
  OFFSET_ERROR,
  CALIBRATION_ERROR,
  PACKET_READING_ERROR,
  SYNC_ERROR
};

}  // namespace serial
}  // namespace rokubimini
