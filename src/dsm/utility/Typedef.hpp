
#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#ifndef __APPLE__
#include <execution>
#define DSM_EXECUTION std::execution::par_unseq,
#else
#define DSM_PAR
#endif

namespace dsm {

  using Id = uint32_t;
  using Size = uint32_t;
  using Delay = uint16_t;
  using Time = uint64_t;

  enum Direction : uint8_t {
    RIGHT = 0,     // delta < 0
    STRAIGHT = 1,  // delta == 0
    LEFT = 2,      // delta > 0
    UTURN = 3,     // std::abs(delta) > std::numbers::pi
    RIGHTANDSTRAIGHT = 4,
    LEFTANDSTRAIGHT = 5,
    ANY = 6
  };
  constexpr std::array<std::string_view, 7> directionToString{
      "RIGHT", "STRAIGHT", "LEFT", "UTURN", "RIGHT&STRAIGHT", "LEFT&STRAIGHT", "ANY"};
  enum class TrafficLightOptimization : uint8_t { SINGLE_TAIL = 0, DOUBLE_TAIL = 1 };
  enum train_t : uint8_t {
    BUS = 0,           // Autobus
    SFM = 1,           // Servizio Ferroviario Metropolitano
    R = 2,             // Regionale
    RV = 3,            // Regionale Veloce
    IC = 4,            // InterCity (Notte)
    FRECCIA = 5,       // Frecciabianca / Frecciargento
    FRECCIAROSSA = 6,  // Frecciarossa
    ES = 7,            // Eurostar
  };
  enum log_level_t : uint8_t { DEBUG = 0, WARNING = 1, INFO = 2, ERROR = 3 };

};  // namespace dsm
