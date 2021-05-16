#ifndef SWRI_CONSOLE_DEFINES_H_
#define SWRI_CONSOLE_DEFINES_H_

#include <rcl_interfaces/msg/log.hpp>

namespace LogLevelMask {
  const uint8_t DEBUG = 0x01;
  const uint8_t INFO = 0x02;
  const uint8_t WARN = 0x04;
  const uint8_t ERROR = 0x08;
  const uint8_t FATAL = 0x10;
};

#endif // DEFINES_H
