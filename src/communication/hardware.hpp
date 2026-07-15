#pragma once
//
// robot::Robot drives everything through this abstract interface, so the whole
// behaviour layer builds and is tested on a host against a fake. The real
// Servo2040 implementation (Pico SDK / Pimoroni) lives in the platform layer
// and is the only place that pulls in the SDK

#include <cstdint>

#include "core/config.hpp"

namespace hw {

struct Interface {
  virtual ~Interface() = default;

  // Drive the 18 servos. Angles are calibrated servo-space degrees, leg-major
  // (index == cfg::servo_channel(leg, joint))
  virtual void write_servos(const float (&deg)[cfg::N_SERVOS]) = 0;

  // Read back the 18 servo angles the board is currently holding (same units and
  // ordering as write_servos). Used to seed the stand-up from where the legs
  // actually are, rather than assuming a pose.
  virtual void read_servos(float (&deg)[cfg::N_SERVOS]) = 0;

  // Enable/disable the servo power rail (cfg::POWER_CUTOFF_PIN)
  virtual void set_power(bool on) = 0;

  // Bus telemetry
  virtual float read_voltage() = 0; // volts
  virtual float read_current() = 0; // amps

  // Status LED(s)
  virtual void set_led(uint8_t mode, uint8_t r, uint8_t g, uint8_t b,
                       float freq_hz) = 0;

  // Advance any time-based LED animation (e.g. blink). The robot ticks this
  // every control cycle; SOLID modes are a no-op
  virtual void update_leds() = 0;

  // Provisioned physical calibration. The pure core reasons in logical channels
  // and servo-space degrees and never needs these; only a real driver does, so
  // they default to no-ops. A host fake ignores them and renders logical
  // channels directly.
  //   set_servo_calibration: measured pulse widths for one servo. Maps the
  //     servo-space clamp [min_deg, max_deg] onto [min_us, max_us], zero at mid_us.
  //   set_servo_pin: which physical output drives a logical channel (re-wiring).
  virtual void set_servo_calibration(uint8_t /*channel*/, uint16_t /*min_us*/,
                                     uint16_t /*mid_us*/, uint16_t /*max_us*/) {}
  virtual void set_servo_pin(uint8_t /*channel*/, uint8_t /*pin*/) {}

  // Low-level calibration jog: drive one servo to a raw pulse, energizing the
  // rail on demand, independent of the FSM. pulse_us == 0 releases (disables)
  // that channel; releasing the last active channel cuts the rail. No-op on a fake.
  virtual void jog_servo(uint8_t /*channel*/, uint16_t /*pulse_us*/) {}
};

struct Proxy : hw::Interface {

  // State
  float servos[cfg::N_SERVOS] = {};
  bool wrote = false;
  bool powered = false;
  float volts = 7.4f;
  float amps = 0.5f;
  int led_calls = 0;
  int led_updates = 0;

  void write_servos(const float (&deg)[cfg::N_SERVOS]) override {
    for (int i = 0; i < cfg::N_SERVOS; ++i)
      servos[i] = deg[i];
    wrote = true;
  }

  void read_servos(float (&deg)[cfg::N_SERVOS]) override {
    for (int i = 0; i < cfg::N_SERVOS; ++i)
      deg[i] = servos[i];
  }

  void set_power(bool on) override { powered = on; }
  
  float read_voltage() override { return volts; }
  
  float read_current() override { return amps; }
  
  void set_led(uint8_t, uint8_t, uint8_t, uint8_t, float) override {
    ++led_calls;
  }

  void update_leds() override { ++led_updates; }
};

} // namespace hw