#include "ButtonBox.h"
#include <Wire.h>

static ButtonBox *s_instance = nullptr;
static volatile bool s_pcfIrq = false;

static void IRAM_ATTR pcfIsr()
{
  s_pcfIrq = true;
}

void ButtonBox::begin()
{
  s_instance = this;

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  _pcf.begin();  // writes all pins high -> inputs with weak pull-ups

  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    _buttons[i] = OneButton();  // no physical pin; state fed manually via tick(bool)
    _buttons[i].setPressMs(kLongPressMs);
    _buttons[i].attachClick(&ButtonBox::_onClick, (void *)(uintptr_t)i);
    _buttons[i].attachLongPressStart(&ButtonBox::_onLongPressStart, (void *)(uintptr_t)i);
    _buttons[i].attachLongPressStop(&ButtonBox::_onLongPressStop, (void *)(uintptr_t)i);
  }

  // PCF8574 INT is open-drain, pulled low on any input change until the
  // port is read; most breakouts already have a pull-up, INPUT_PULLUP here
  // is just a safety net.
  pinMode(PCF8574_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PCF8574_INT_PIN), pcfIsr, FALLING);
}

void ButtonBox::update()
{
  uint32_t now = millis();
  bool due = s_pcfIrq || (now - _lastPollMs >= kFallbackPollMs);
  if (!due) {
    return;
  }
  s_pcfIrq = false;
  _lastPollMs = now;

  uint8_t raw = _pcf.read8();  // active low: bit clear = pressed

  uint8_t newMask = 0;
  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    if (!(raw & (1 << i))) {
      newMask |= (1 << i);
    }
  }
  uint8_t popcount = __builtin_popcount(newMask);

  // Figure out *before* ticking the individual buttons whether this hold
  // already is, or is only just now becoming, a recognised combo, so their
  // click/long-press callbacks can be muted for exactly that tick too -
  // otherwise a combo also reports as two single long-presses.
  bool comboActiveThisTick = _comboActive ||
      (popcount >= 2 && _comboCandidatePending && (now - _comboCandidateStartMs >= kLongPressMs));
  _suppressMask = comboActiveThisTick ? newMask : 0;

  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    _buttons[i].tick((newMask & (1 << i)) != 0);
  }
  _pressedMask = newMask;

  if (popcount >= 2) {
    if (_comboActive) {
      // already reported; nothing to do until the mask drops below 2
    } else if (!_comboCandidatePending) {
      _comboCandidatePending = true;
      _comboCandidateStartMs = now;
    } else if (now - _comboCandidateStartMs >= kLongPressMs) {
      _comboCandidatePending = false;
      _comboActive = true;
      _comboMask = newMask;
      _pushEvent(EventType::ComboStart, _comboMask);
    }
  } else {
    _comboCandidatePending = false;
    if (_comboActive) {
      _comboActive = false;
      _pushEvent(EventType::ComboStop, _comboMask);
    }
  }
}

bool ButtonBox::popEvent(Event &event)
{
  if (_qHead == _qTail) {
    return false;
  }
  event = _queue[_qTail];
  _qTail = (_qTail + 1) % kQueueSize;
  return true;
}

void ButtonBox::_pushEvent(EventType type, uint8_t mask)
{
  uint8_t next = (_qHead + 1) % kQueueSize;
  if (next == _qTail) {
    return;  // queue full, drop rather than block
  }
  _queue[_qHead] = {type, mask};
  _qHead = next;
}

void ButtonBox::_onClick(void *param)
{
  if (!s_instance) {
    return;
  }
  uint8_t idx = (uint8_t)(uintptr_t)param;
  if (s_instance->_suppressMask & (1 << idx)) {
    return;  // part of a combo - the Combo event covers it instead
  }
  s_instance->_pushEvent(EventType::Click, (uint8_t)(1 << idx));
}

void ButtonBox::_onLongPressStart(void *param)
{
  if (!s_instance) {
    return;
  }
  uint8_t idx = (uint8_t)(uintptr_t)param;
  if (s_instance->_suppressMask & (1 << idx)) {
    return;
  }
  s_instance->_pushEvent(EventType::LongPressStart, (uint8_t)(1 << idx));
}

void ButtonBox::_onLongPressStop(void *param)
{
  if (!s_instance) {
    return;
  }
  uint8_t idx = (uint8_t)(uintptr_t)param;
  if (s_instance->_suppressMask & (1 << idx)) {
    return;
  }
  s_instance->_pushEvent(EventType::LongPressStop, (uint8_t)(1 << idx));
}
