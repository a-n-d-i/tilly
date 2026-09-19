#include "applog.h"
#include <stdarg.h>

static char s_lines[APPLOG_CAPACITY][APPLOG_LINE_LEN];
static size_t s_count = 0;  // number of valid lines (<= APPLOG_CAPACITY)
static size_t s_head = 0;   // slot the next line will be written to

void appLog(const char *fmt, ...) {
  char msg[APPLOG_LINE_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);

  char *line = s_lines[s_head];
  snprintf(line, APPLOG_LINE_LEN, "[%lu] %s", millis(), msg);

  s_head = (s_head + 1) % APPLOG_CAPACITY;
  if (s_count < APPLOG_CAPACITY) s_count++;

  Serial.println(line);
}

size_t appLogCount() {
  return s_count;
}

const char *appLogLine(size_t index) {
  if (index >= s_count) return "";
  // Oldest held line sits at s_head once the buffer has wrapped, or at 0
  // while it's still filling up for the first time.
  size_t oldest = (s_count < APPLOG_CAPACITY) ? 0 : s_head;
  return s_lines[(oldest + index) % APPLOG_CAPACITY];
}
