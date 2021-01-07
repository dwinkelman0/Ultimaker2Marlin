#include "Thermal.h"

#include "commands.h"

CommandQueue::CommandQueue() {
  Serial.setTimeout(1000);
  memset(buffer_, sizeof(buffer_), 0);
}

bool CommandQueue::loadNextLine() {
  // Reset
  for (int i = 0; i < 26; ++i) {
    ptrs_[i] = NULL;
  }
  buffer_[0] = '\0';

  // Check if there is data to read
  if (Serial.available() == 0) return;

  // Copy the line into memory
  int numRead = Serial.readBytesUntil('\n', buffer_, sizeof(buffer_) - 1);
  buffer_[numRead] = '\0';

  // Find all codes
  char *it = buffer_;
  int numCodes = 0;
  while (*it) {
    if ('A' <= *it && *it <= 'Z') {
      ptrs_[*it - 'A'] = it;
      numCodes++;
    }
    else if (isWhitespace(*it)) {
      *it = '\0';
    }
    ++it;
  }

  // Flush the newline
  return numCodes > 0;
}

bool CommandQueue::getInt(const char code, int32_t *number) const {
  if ('A' <= code && code <= 'Z') {
    const char *str = ptrs_[code - 'A'];
    if (str) {
      *number = str[1] ? String(str + 1).toInt() : 0;
      return true;
    }
  }
  *number = 0;
  return false;
}

bool CommandQueue::getFloat(const char code, float *number) const {
  if ('A' <= code && code <= 'Z') {
    const char *str = ptrs_[code - 'A'];
    if (str) {
      *number = str[1] ? String(str + 1).toFloat() : 0.0f;
      return true;
    }
  }
  *number = 0.0f;
  return false;
}
