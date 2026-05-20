// Copyright 2012 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// Modified by Sonic Insurgence, June 2021
//
// IMPROVED MIDI IMPLEMENTATION
//
// Set the tempo knop to its minimum position. After receiving a
// MIDI Start or MIDI Continue message Grids switches into a
// "clocked_by_midi" mode. In this mode Grids advances with every
// MIDI clock message, whereas a rising edge at its clock input
// no longer has an effect. The MIDI Start message also resets
// the Grids engine.
// As an external reset in "clocked_by_midi" mode would break
// the synchronization with the external MIDI master device the
// function of the reset input as well as the reset button changes.
// They now assume a "mute" functionality that suppresses all drum
// and accent outputs. When muted all three output leds will light
// up permanently.
// To leave the "clocked_by_midi" mode turn the tempo knob to the
// right to activate internal clocking.
//
// I also disabled the retriggering of the outputs when a
// reset signal has been received.

// Modified by Andrew Perry (semi-sensible synth), 2024
// Added MIDI out drum triggers on channel 10

#include <avr/eeprom.h>

#include "avrlib/adc.h"
#include "avrlib/boot.h"
#include "avrlib/op.h"
#include "avrlib/watchdog_timer.h"

#include "grids/clock.h"
#include "grids/hardware_config.h"
#include "grids/pattern_generator.h"
#include "grids/midi.h"
// #include "avrlib/software_serial.h"

using namespace avrlib;
using namespace grids;

Leds leds;
// Inputs inputs;
ResetInput reset_input;
ButtonInput button_input;
ClockInput clock_input;

AdcInputScanner adc;
ShiftRegister shift_register;
// MidiInput midi;
MidiIO midi;

enum Parameter
{
  PARAMETER_NONE,
  PARAMETER_WAITING,
  PARAMETER_CLOCK_RESOLUTION,
  PARAMETER_TAP_TEMPO,
  PARAMETER_SWING,
  PARAMETER_GATE_MODE,
  PARAMETER_OUTPUT_MODE,
  PARAMETER_CLOCK_OUTPUT,
  PARAMETER_BANK
};

uint32_t tap_duration = 0;
uint8_t led_pattern;
uint8_t led_off_timer;

int8_t swing_amount;

volatile Parameter parameter = PARAMETER_NONE;
volatile bool long_press_detected = false;
const uint8_t kUpdatePeriod = F_CPU / 32 / 8000;

uint8_t clocked_by_midi = 0; // 1 = MIDI Clock advances Grids, 2 = MIDI Stop received
uint8_t mute = 0;            // 1 = drum and accent outputs are muted
uint8_t external_clock = 0;  // 1 = Grids is in external clock mode (clock knob = min,
                             // accept pulses on clock input or by midi clock messages)

// Accent CV soft-PWM state: hold levels between triggers rather than short pulses.
// In drums mode (no output clock): accented channels → 5V, normal triggers → ~3V (PWM),
// no trigger → 0V. Uses 800 Hz soft-PWM (8 kHz ISR / 10 ticks), 60 % duty for ~3V.
static uint8_t accent_cv_high = 0;   // bits 0-2: channels latched at 5V (accented)
static uint8_t accent_cv_normal = 0; // bits 0-2: channels latched at ~3V (normal trigger)
static uint8_t accent_pwm_counter = 0;
static uint8_t sr_base_state = 0;    // shift register state with accent bits stripped

inline void UpdateLeds()
{
  uint8_t pattern;
  if (parameter == PARAMETER_NONE)
  {
    if (led_off_timer)
    {
      --led_off_timer;
      if (!led_off_timer)
      {
        led_pattern = 0;
      }
    }
    if (mute)
    {
      // indicate muted outputs by turning
      led_pattern ^= LED_BD | LED_SD | LED_HH; // all 3 leds on (at 50 % duty cycle)
    }
    pattern = led_pattern;
    if (pattern_generator.tap_tempo())
    {
      if (pattern_generator.on_beat())
      {
        pattern |= LED_CLOCK;
      }
    }
    else
    {
      if (pattern_generator.on_first_beat())
      {
        pattern |= LED_CLOCK;
      }
    }
  }
  else
  {
    pattern = LED_CLOCK;
    switch (parameter)
    {
    case PARAMETER_CLOCK_RESOLUTION:
      pattern |= LED_BD >> pattern_generator.clock_resolution();
      break;

    case PARAMETER_CLOCK_OUTPUT:
      if (pattern_generator.output_clock())
      {
        pattern |= LED_ALL;
      }
      break;

    case PARAMETER_SWING:
      if (pattern_generator.swing())
      {
        pattern |= LED_ALL;
      }
      break;

    case PARAMETER_OUTPUT_MODE:
      if (pattern_generator.output_mode() == OUTPUT_MODE_DRUMS)
      {
        pattern |= LED_ALL;
      }
      break;

    case PARAMETER_TAP_TEMPO:
      if (pattern_generator.tap_tempo())
      {
        pattern |= LED_ALL;
      }
      break;

    case PARAMETER_GATE_MODE:
      if (pattern_generator.gate_mode())
      {
        pattern |= LED_ALL;
      }
      break;

    case PARAMETER_BANK:
      if (pattern_generator.bank() == 0) {
        pattern |= LED_BD;
      } else if (pattern_generator.bank() == 1) {
        pattern |= LED_BD | LED_SD;
      } else {
        pattern |= LED_BD | LED_SD | LED_HH;
      }
      break;
    }
  }
  leds.Write(pattern);
}

inline void BufferMidiMessages(uint8_t state)
{
  if (state & 0x01)
  { // BD: velocity 127 if accented (bit 3), else 99
    uint8_t vel = (state & 0x08) ? 127 : 99;
    grids::MidiDevice::BufferNote(MIDI_CHANNEL, BD_NOTE, vel);
  }
  if (state & 0x02)
  { // SD: velocity 127 if accented (bit 4), else 99
    uint8_t vel = (state & 0x10) ? 127 : 99;
    grids::MidiDevice::BufferNote(MIDI_CHANNEL, SD_NOTE, vel);
  }
  if (state & 0x04)
  { // HH: open hi-hat note + vel 127 if accented (bit 5), closed + vel 99 otherwise
    if (state & 0x20)
    {
      grids::MidiDevice::BufferNote(MIDI_CHANNEL, HH_ACCENT_NOTE, 127);
    }
    else
    {
      grids::MidiDevice::BufferNote(MIDI_CHANNEL, HH_NOTE, 99);
    }
  }
}

inline void UpdateShiftRegister()
{
  static uint8_t previous_state = 0;
  uint8_t state = pattern_generator.state();

  if (mute)
  {
    state &= ~(0x07); // clear drum bits (BD, SD, HH)
    if (pattern_generator.output_mode() == OUTPUT_MODE_DRUMS)
    {
      if (pattern_generator.output_clock())
      {
        state &= ~(OUTPUT_BIT_COMMON); // clear common accent bit
      }
      else
      {
        state &= ~(0x07 << 3); // clear all 3 accent bits
      }
    }
  }

  if (state != previous_state)
  {
    previous_state = state;

    // In drums mode without output clock, accent bits (3-5) are per-channel velocity CVs.
    // Track the held accent state and strip accent bits from the base SR value so the
    // ISR can re-apply them each tick via soft-PWM (accented=5V, normal=~3V).
    bool drums_accent_mode = (pattern_generator.output_mode() == OUTPUT_MODE_DRUMS &&
                              !pattern_generator.output_clock());
    if (drums_accent_mode)
    {
      uint8_t trigger_bits = state & 0x07;
      uint8_t accent_bits  = (state >> 3) & 0x07;
      uint8_t normal_bits  = trigger_bits & ~accent_bits;
      // Update per-channel held state only for channels that just fired a trigger.
      // Channels that did not fire retain their previous level.
      accent_cv_high   = (accent_cv_high   & ~trigger_bits) | accent_bits;
      accent_cv_normal = (accent_cv_normal & ~trigger_bits) | normal_bits;
      sr_base_state = state & ~0x38; // accent bits handled by soft-PWM
    }
    else
    {
      sr_base_state = state;
      accent_cv_high   = 0;
      accent_cv_normal = 0;
    }

    // Buffer MIDI messages
    BufferMidiMessages(state);

    if (!state)
    {
      // Switch off the LEDs, but not now.
      led_off_timer = 200;
    }
    else
    {
      // Switch on the LEDs with a new pattern.
      led_pattern = pattern_generator.led_pattern();
      led_off_timer = 0;
    }
  }
}

uint8_t ticks_granularity[] = {6, 3, 1};

inline void HandleClockResetInputs()
{
  // static uint8_t previous_inputs;
  static bool previous_clock_value;
  static bool previous_reset_value;

  // uint8_t inputs_value = ~inputs.Read();
  bool clock_value = ~clock_input.Read();
  bool reset_value = reset_input.Read();
  uint8_t num_ticks = 0;
  uint8_t increment = ticks_granularity[pattern_generator.clock_resolution()];

  // CLOCK
  if (clock.bpm() < 40 && !clock.locked())
  {
    if (!external_clock)
    {
      external_clock = 1;
      mute = 1; // activate mute when entering external clock mode
    }
    if ((clock_value) && !(previous_clock_value))
    {
      if (!clocked_by_midi)
      {
        num_ticks = increment;
      }
    }
    if (!(clock_value) && (previous_clock_value))
    {
      pattern_generator.ClockFallingEdge();
    }
    if (midi.readable())
    {
      uint8_t byte = midi.ImmediateRead();
      if (byte == 0xf8)
      { // MIDI Clock message
        if (clocked_by_midi == 1)
        {
          num_ticks = 1;
        }
      }
      else if (byte == 0xfa)
      { // MIDI Start message
        pattern_generator.Reset();
        clocked_by_midi = 1;
      }
      else if (byte == 0xfb)
      { // MIDI Continue message
        clocked_by_midi = 1;
      }
      else if (byte == 0xfc)
      { // MIDI Stop message
        clocked_by_midi = 2;
        // AllNotesOff();
      }
    }
  }
  else
  {
    if (external_clock)
    {
      external_clock = 0;
      mute = 0; // deactivate mute when leaving external clock mode
      led_pattern = 0;
    }
    if (clocked_by_midi)
    {
      clocked_by_midi = 0;
      mute = 0;
      pattern_generator.Reset();
    }
    clock.Tick();
    clock.Wrap(swing_amount);
    if (clock.raising_edge())
    {
      num_ticks = increment;
    }
    if (clock.past_falling_edge())
    {
      pattern_generator.ClockFallingEdge();
    }
  }

  // RESET
  if (clocked_by_midi)
  {
    if ((reset_value) && !(previous_reset_value))
    {
      mute = 1; // activate mute on rising edge
    }
    if (!(reset_value) && (previous_reset_value))
    {
      mute = 0; // deactivate mute on falling edge
      led_pattern = 0;
    }
  }
  else
  {
    if ((reset_value) && !(previous_reset_value))
    {
      pattern_generator.Reset();
      // AllNotesOff();

      // !! HACK AHEAD !!
      //
      // Earlier versions of the firmware retriggered the outputs whenever a
      // RESET signal was received. This allowed for nice drill'n'bass effects,
      // but made synchronization with another sequencer a bit glitchy (risk of
      // double notes at the beginning of a pattern). It was later decided
      // to remove this behaviour and make the RESET transparent (just set the
      // step index without producing any trigger) - similar to the MIDI START
      // message. However, the factory testing script relies on the old behaviour.
      // To solve this problem, we reproduce this behaviour the first 5 times the
      // module is powered. After the 5th power-on (or settings change) cycle,
      // this odd behaviour disappears.
      /* ### As we don't do factory testing we don't need the hack. ###
      if (pattern_generator.factory_testing() ||
        clock.bpm() >= 40 ||
        clock.locked()) {
      */
      if (clock.bpm() >= 40 || clock.locked())
      {
        // I don't like the retriggering. So I comment it out.
        // pattern_generator.Retrigger();
        clock.Reset();
      }
    }
  }
  // previous_inputs = inputs_value;
  previous_clock_value = clock_value;
  previous_reset_value = reset_value;

  if (num_ticks)
  {
    swing_amount = pattern_generator.swing_amount();
    pattern_generator.TickClock(num_ticks);
  }
}

enum SwitchState
{
  SWITCH_STATE_JUST_PRESSED = 0xfe,
  SWITCH_STATE_PRESSED = 0x00,
  SWITCH_STATE_JUST_RELEASED = 0x01,
  SWITCH_STATE_RELEASED = 0xff
};

inline void HandleTapButton()
{
  static uint8_t switch_state = 0xff;
  static uint16_t switch_hold_time = 0;

  switch_state = switch_state << 1;
  if (button_input.Read())
  {
    switch_state |= 1;
  }

  if (switch_state == SWITCH_STATE_JUST_PRESSED)
  {
    if (parameter == PARAMETER_NONE)
    {
      if (clocked_by_midi)
      { // in clocked_by_midi mode, button toggles mute state
        if (mute)
        {
          mute = 0;
          led_pattern = 0;
        }
        else
        {
          mute = 1;
        }
      }
      else
      {
        if (external_clock == 1)
        {           // in external clock mode
          mute = 0; // the first button press unmutes the outputs
          led_pattern = 0;
          external_clock = 2;
        }
        if (!pattern_generator.tap_tempo())
        {
          pattern_generator.Reset();
          /*  no need for the hack (see above)
          if (pattern_generator.factory_testing() ||
              clock.bpm() >= 40 ||
              clock.locked()) {
*/
          if (clock.bpm() >= 40 || clock.locked())
          {
            clock.Reset();
          }
        }
        else
        {
          uint32_t new_bpm = (F_CPU * 60L) / (32L * kUpdatePeriod * tap_duration);
          if (new_bpm >= 30 && new_bpm <= 480)
          {
            clock.Update(new_bpm, pattern_generator.clock_resolution());
            clock.Reset();
            clock.Lock();
          }
          else
          {
            clock.Unlock();
          }
          tap_duration = 0;
        }
      }
    }
    switch_hold_time = 0;
  }
  else if (switch_state == SWITCH_STATE_PRESSED)
  {
    ++switch_hold_time;
    if (switch_hold_time == 500)
    {
      long_press_detected = true;
    }
  }
}

ISR(TIMER2_COMPA_vect, ISR_NOBLOCK)
{
  static uint8_t switch_debounce_prescaler;

  ++tap_duration;
  ++switch_debounce_prescaler;
  if (switch_debounce_prescaler >= 10)
  {
    // Debounce RESET/TAP switch and perform switch action.
    HandleTapButton();
    switch_debounce_prescaler = 0;
  }

  HandleClockResetInputs();

  adc.Scan();

  pattern_generator.IncrementPulseCounter();
  UpdateShiftRegister();
  UpdateLeds();

  // When muted, suppress accent CV levels.
  if (mute)
  {
    accent_cv_high   = 0;
    accent_cv_normal = 0;
  }

  // Write shift register every tick: base trigger/clock/random bits plus
  // soft-PWM accent bits (accented=always HIGH, normal=60 % duty cycle →~3V).
  uint8_t pwm_accent = accent_cv_high;
  if (accent_pwm_counter < 6)
  {
    pwm_accent |= accent_cv_normal;
  }
  if (++accent_pwm_counter >= 10)
  {
    accent_pwm_counter = 0;
  }
  shift_register.Write(sr_base_state | (pwm_accent << 3));
}

static int16_t pot_values[8];

void ScanPots()
{
  if (long_press_detected)
  {
    if (parameter == PARAMETER_NONE)
    {
      // Freeze pot values
      for (uint8_t i = 0; i < 8; ++i)
      {
        pot_values[i] = adc.Read8(i);
      }
      parameter = PARAMETER_WAITING;
    }
    else
    {
      parameter = PARAMETER_NONE;
      pattern_generator.SaveSettings();
    }
    long_press_detected = false;
  }

  if (parameter == PARAMETER_NONE)
  {
    uint8_t bpm = adc.Read8(ADC_CHANNEL_TEMPO);
    bpm = U8U8MulShift8(bpm, 220) + 20;
    if (bpm != clock.bpm() && !clock.locked())
    {
      clock.Update(bpm, pattern_generator.clock_resolution());
    }
    PatternGeneratorSettings *settings = pattern_generator.mutable_settings();
    settings->options.drums.x = ~adc.Read8(ADC_CHANNEL_X_CV);
    settings->options.drums.y = ~adc.Read8(ADC_CHANNEL_Y_CV);
    settings->options.drums.randomness = ~adc.Read8(ADC_CHANNEL_RANDOMNESS_CV);
    settings->density[0] = ~adc.Read8(ADC_CHANNEL_BD_DENSITY_CV);
    settings->density[1] = ~adc.Read8(ADC_CHANNEL_SD_DENSITY_CV);
    settings->density[2] = ~adc.Read8(ADC_CHANNEL_HH_DENSITY_CV);
  }
  else
  {
    for (uint8_t i = 0; i < 8; ++i)
    {
      int16_t value = adc.Read8(i);
      int16_t delta = value - pot_values[i];
      if (delta < 0)
      {
        delta = -delta;
      }
      if (delta > 32)
      {
        pot_values[i] = value;
        switch (i)
        {
        case ADC_CHANNEL_BD_DENSITY_CV:
          parameter = PARAMETER_CLOCK_RESOLUTION;
          pattern_generator.set_clock_resolution((255 - value) >> 6);
          clock.Update(clock.bpm(), pattern_generator.clock_resolution());
          pattern_generator.Reset();
          break;

        case ADC_CHANNEL_SD_DENSITY_CV:
          parameter = PARAMETER_TAP_TEMPO;
          pattern_generator.set_tap_tempo(!(value & 0x80));
          if (!pattern_generator.tap_tempo())
          {
            clock.Unlock();
          }
          break;

        case ADC_CHANNEL_HH_DENSITY_CV:
          parameter = PARAMETER_SWING;
          pattern_generator.set_swing(!(value & 0x80));
          break;

        case ADC_CHANNEL_X_CV:
          parameter = PARAMETER_OUTPUT_MODE;
          pattern_generator.set_output_mode(!(value & 0x80) ? 1 : 0);
          break;

        case ADC_CHANNEL_Y_CV:
          parameter = PARAMETER_GATE_MODE;
          pattern_generator.set_gate_mode(!(value & 0x80));
          break;

        case ADC_CHANNEL_RANDOMNESS_CV:
          parameter = PARAMETER_CLOCK_OUTPUT;
          pattern_generator.set_output_clock(!(value & 0x80));
          break;

        case ADC_CHANNEL_TEMPO:
          parameter = PARAMETER_BANK;
          {
            uint8_t b = value / 85;
            if (b > 2) b = 2;
            pattern_generator.set_bank(b);
          }
          break;
        }
      }
    }
  }
}

void TestMidiOutput()
{
  // Simple direct MIDI test - Bass Drum note
  midi.Write(0x99); // Note On, Channel 10
  midi.Write(0x24); // Bass Drum
  midi.Write(0x7F); // Full velocity
  _delay_ms(500);   // Wait half second
  midi.Write(0x89); // Note Off, Channel 10
  midi.Write(0x24); // Bass Drum
  midi.Write(0x00); // Zero velocity

  // simpler
  // midi.Write(0x90); // Just Note On, channel 1
  //_delay_ms(1000);  // Longer delay

  // send a stream of simple alternating 0101 0101 and 1010 1010
  /*
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = 0x55; // Alternating 0101 0101
  _delay_ms(100);
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = 0xAA; // Alternating 1010 1010
  _delay_ms(100);
  */
}

void Init()
{

#if (F_CPU != 16000000UL)
// #error "Wrong F_CPU setting - should be 16MHz"
#endif

  sei();
  UCSR0B = 0; // Disable UART before configuration

  // Configure UART for MIDI
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit, no parity
  UBRR0 = (F_CPU / 16 / 31250) - 1;       // Set baud rate for 31250
  UCSR0B = (1 << TXEN0);                  // Enable transmitter

  // Don't call midi.Init() as we're configuring UART directly
  // grids::MidiDevice::Init(midi);

  leds.set_mode(DIGITAL_OUTPUT);
  reset_input.EnablePullUpResistor();
  button_input.EnablePullUpResistor();
  clock_input.EnablePullUpResistor();

  clock.Init();
  adc.Init();
  adc.set_num_inputs(ADC_CHANNEL_LAST);
  Adc::set_reference(ADC_DEFAULT);
  Adc::set_alignment(ADC_LEFT_ALIGNED);
  pattern_generator.Init();
  shift_register.Init();

  TCCR2A = _BV(WGM21);
  TCCR2B = 3;
  OCR2A = kUpdatePeriod - 1;
  TIMSK2 |= _BV(1);

  // Test MIDI output
  /*
  TestMidiOutput();
  TestMidiOutput();
  TestMidiOutput();
  TestMidiOutput();

  while (true)
  {
    _delay_ms(1000);
  }
  */
}

int main(void)
{
  ResetWatchdog();
  Init();
  clock.Update(120, pattern_generator.clock_resolution());

  while (1)
  {
    // Use any spare cycles to read the CVs and update the potentiometers
    ScanPots();

    // Transmit MIDI messages from the buffer
    cli(); // Disable interrupts to safely access buffer indices
    bool has_messages = (buffer_tail != buffer_head);
    sei(); // Re-enable interrupts

    if (has_messages)
    {
      grids::MidiDevice::SendBuffer();
    }
  }
}