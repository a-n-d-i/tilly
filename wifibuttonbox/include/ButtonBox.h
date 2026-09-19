#pragma once

#include <Arduino.h>
#include <OneButton.h>
#include <PCF8574.h>
#include "board_pins.h"

// Reads the 6 buttons on the PCF8574T over I2C and turns them into
// click / long-press / two-buttons-together events.
//
// Debounce and short/long press classification come from the OneButton
// library (millis()-based state machine, no blocking delay()). The PCF8574
// INT line (GPIO17) wakes update() promptly on any change; a periodic
// fallback poll keeps long-press timing advancing even while the raw
// I2C byte is not changing.
class ButtonBox {
public:
  enum class EventType {
    Click,
    LongPressStart,
    LongPressStop,
    ComboStart,
    ComboStop,
  };

  struct Event {
    EventType type;
    uint8_t mask;  // bit i set => button i involved
  };

  void begin();

  // Call as often as convenient (e.g. every loop() iteration); it is a
  // no-op except when the interrupt fired or the fallback interval elapsed.
  void update();

  // Drains the internal event queue one entry at a time.
  bool popEvent(Event &event);

  uint8_t pressedMask() const { return _pressedMask; }

private:
  static constexpr uint32_t kFallbackPollMs = 10;
  static constexpr uint8_t kQueueSize = 8;
  // Combo requires the same sustained hold as a long press, so a brief
  // overlap while pressing two buttons in quick succession can't trigger it.
  static constexpr unsigned int kLongPressMs = 800;

  PCF8574 _pcf{PCF8574_ADDR};
  OneButton _buttons[BUTTON_COUNT];
  uint8_t _pressedMask = 0;
  bool _comboActive = false;
  uint8_t _comboMask = 0;
  bool _comboCandidatePending = false;
  uint32_t _comboCandidateStartMs = 0;
  uint32_t _lastPollMs = 0;
  // Buttons currently hidden from their own click/long-press callbacks
  // because they're part of an active (or this-tick-forming) combo.
  uint8_t _suppressMask = 0;

  Event _queue[kQueueSize];
  uint8_t _qHead = 0;
  uint8_t _qTail = 0;

  void _pushEvent(EventType type, uint8_t mask);

  static void _onClick(void *param);
  static void _onLongPressStart(void *param);
  static void _onLongPressStop(void *param);
};
