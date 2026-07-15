#pragma once
//
// This is the only file that talks to the Pimoroni SDK: 
// - 18 servos on PIO0
// - 6 WS2812 pixels on PIO1
// - the servo power trace (cfg::POWER_CUTOFF_PIN)
// - analog mux (voltage / current)
// 
// Everything else (robot::Robot, gait, body, kinematics) is SDK-free and 
// host-tested against a fake
//

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "servo2040.hpp" // Pimoroni driver (distinct from this file, servo2040_hal.hpp)
#include "analog.hpp"
#include "analogmux.hpp"

#include "core/config.hpp"
#include "communication/protocol.hpp" // proto::LedMode
#include "communication/hardware.hpp"

namespace hal {

using namespace plasma;
using namespace servo;

static_assert(servo2040::NUM_SERVOS == cfg::N_SERVOS,
              "Servo2040 has a different servo count than cfg::N_SERVOS");

class Servo2040 : public hw::Interface {
public:
  Servo2040()
      : servos_(pio0, 0, servo2040::SERVO_1, servo2040::NUM_SERVOS),
        leds_(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA),
        vol_adc_(servo2040::SHARED_ADC, servo2040::VOLTAGE_GAIN),
        cur_adc_(servo2040::SHARED_ADC, servo2040::CURRENT_GAIN,
                 servo2040::SHUNT_RESISTOR, servo2040::CURRENT_OFFSET),
        mux_(servo2040::ADC_ADDR_0, servo2040::ADC_ADDR_1, servo2040::ADC_ADDR_2,
             PIN_UNUSED, servo2040::SHARED_ADC) {}

  // Call once at boot, before the Robot runs
  void init() {
    servos_.init();
    leds_.start();
    gpio_init(cfg::POWER_CUTOFF_PIN);
    gpio_set_dir(cfg::POWER_CUTOFF_PIN, GPIO_OUT);
    gpio_put(cfg::POWER_CUTOFF_PIN, 0); // start de-energized
    for (uint i = 0; i < servo2040::NUM_SERVOS; ++i)
      pin_map_[i] = static_cast<uint8_t>(i); // identity until provisioned
  }

  // hw::Interface

  // Drive all 18 servos atomically: stage every value, then load() once. deg is
  // indexed by LOGICAL channel; pin_map_ routes it to the physical output. The
  // servo-space degree -> pulse conversion uses each servo's provisioned
  // calibration (see set_servo_calibration).
  void write_servos(const float (&deg)[cfg::N_SERVOS]) override {
    for (uint i = 0; i < servo2040::NUM_SERVOS; ++i)
      servos_.value(pin_map_[i], deg[i], /*load=*/false);
    servos_.load();
  }

  // Read back the angle the board is holding on each LOGICAL channel (the
  // ServoCluster tracks the last value per servo). Mirrors legacy get_servo_angle().
  void read_servos(float (&deg)[cfg::N_SERVOS]) override {
    for (uint i = 0; i < servo2040::NUM_SERVOS; ++i)
      deg[i] = servos_.value(pin_map_[i]);
  }

  // Energize = enable the servo outputs then connect the power trace; the reverse
  // to de-energize (drop power before killing the PWM)
  void set_power(bool on) override {
    if (on) {
      servos_.enable_all();
      gpio_put(cfg::POWER_CUTOFF_PIN, 1);
    } else {
      gpio_put(cfg::POWER_CUTOFF_PIN, 0);
      servos_.disable_all();
    }
  }

  float read_voltage() override {
    mux_.select(servo2040::VOLTAGE_SENSE_ADDR);
    return vol_adc_.read_voltage();
  }
  float read_current() override {
    mux_.select(servo2040::CURRENT_SENSE_ADDR);
    return cur_adc_.read_current();
  }

  // One colour for the whole strip; BLINK is driven by update_leds()
  void set_led(uint8_t mode, uint8_t r, uint8_t g, uint8_t b,
               float freq_hz) override {
    led_mode_ = mode;
    lr_ = r; lg_ = g; lb_ = b;
    led_freq_ = freq_hz;
    led_toggle_us_ = time_us_64();
    led_on_ = true;
    apply(true);
  }

  // Advance the LED blink. No-op in SOLID mode (the strip is already set).
  // hw::Interface: the robot ticks this every control cycle
  void update_leds() override {
    if (led_mode_ != uint8_t(proto::LedMode::BLINK) || led_freq_ <= 0.0f)
      return;
    const uint64_t half_us = uint64_t(1.0e6f / (2.0f * led_freq_));
    const uint64_t now = time_us_64();
    if (now - led_toggle_us_ >= half_us) {
      led_toggle_us_ = now;
      led_on_ = !led_on_;
      apply(led_on_);
    }
  }

  // Board-specific setup helper (not part of hw::Interface): called once at boot,
  // before the board is handed to the robot. Briefly displays an animation 
  // so it's obvious the board booted
  void boot_flash() {

    // (208, 107, 51) is a cool orange shade I found that matches the color
    // of my robot

    // Flash all the leds at once a single time
    // for (uint8_t i = 0; i < servo2040::NUM_LEDS; ++i) {leds_.set_rgb(i, 208, 107, 51);
    // sleep_ms(150);
    // for (uint8_t i = 0; i < servo2040::NUM_LEDS; ++i) leds_.set_rgb(i, 0, 0, 0);

    // Flash one led at a time, one after the other 
    for (uint8_t i = 0; i < servo2040::NUM_LEDS; ++i) {
      leds_.set_rgb(i, 208, 107, 51);
      sleep_ms(50);
      leds_.set_rgb(i, 0, 0, 0);
    }
  }

  // Provisioned per-servo calibration: the servo's own pulse<->angle transfer
  // function, measured on the BARE servo before assembly, so it carries no
  // knowledge of where the joint ends up mechanically. Fixed convention:
  //   min_us <-> -90 deg,  mid_us <-> 0 deg,  max_us <-> +90 deg
  // (mid_us is just (min_us+max_us)/2). The core has already turned kinematics
  // into servo-space degrees via cfg::DIRECTION/TRIM_DEG and clamped to the
  // joint's cfg::RANGE. That clamp can run past +-90 (e.g. the tibia sweeps
  // down to -120 deg) so limits are disabled here: the calibration must
  // extrapolate that range from the -90/+90 reference points, not clip it.
  void set_servo_calibration(uint8_t channel, uint16_t min_us, uint16_t mid_us,
                             uint16_t max_us) override {
    if (channel >= cfg::N_SERVOS) return;
    Calibration &cal = servos_.calibration(pin_map_[channel]);
    cal.apply_three_pairs(min_us, mid_us, max_us, -90.0f, 0.0f, 90.0f);
    cal.limit_to_calibration(false, false); // extrapolate past +-90 (e.g. tibia)
  }

  // Re-wire: which physical ServoCluster output drives a logical channel.
  void set_servo_pin(uint8_t channel, uint8_t pin) override {
    if (channel < cfg::N_SERVOS && pin < cfg::N_SERVOS) pin_map_[channel] = pin;
  }

  // Low-level jog for the offline calibration script. Energizes the rail on the
  // first held servo and drops it once nothing is held; independent of the FSM.
  void jog_servo(uint8_t channel, uint16_t pulse_us) override {
    if (channel >= cfg::N_SERVOS) return;
    const uint8_t phys = pin_map_[channel];
    if (pulse_us == 0) { // release
      servos_.disable(phys);
      if (jog_held_[channel]) { jog_held_[channel] = false; --jog_active_; }
      if (jog_active_ == 0) gpio_put(cfg::POWER_CUTOFF_PIN, 0);
      return;
    }
    if (jog_active_ == 0) gpio_put(cfg::POWER_CUTOFF_PIN, 1); // energize on demand
    servos_.enable(phys);
    servos_.pulse(phys, static_cast<float>(pulse_us)); // Pimoroni pulse() is in us
    if (!jog_held_[channel]) { jog_held_[channel] = true; ++jog_active_; }
  }

private:
  void apply(bool on) {
    for (uint i = 0; i < servo2040::NUM_LEDS; ++i)
      leds_.set_rgb(i, on ? lr_ : 0, on ? lg_ : 0, on ? lb_ : 0);
  }

  ServoCluster servos_;
  WS2812 leds_;
  Analog vol_adc_, cur_adc_;
  AnalogMux mux_;

  uint8_t led_mode_ = 0, lr_ = 0, lg_ = 0, lb_ = 0;
  float led_freq_ = 0.0f;
  uint64_t led_toggle_us_ = 0;
  bool led_on_ = true;

  // Provisioning state
  uint8_t pin_map_[cfg::N_SERVOS] = {};   // logical channel -> physical output
  uint8_t jog_active_ = 0;                // servos currently held by jog_servo()
  bool jog_held_[cfg::N_SERVOS] = {};
};

} // namespace hal
