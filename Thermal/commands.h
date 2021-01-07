#ifndef _COMMANDS_H_
#define _COMMANDS_H_

class CommandQueue {
 public:
  CommandQueue();

  /* Load a line of input from Serial; returns true if at least one code is
   *  found
   */
  bool loadNextLine();

  /* Parse an int or float for a code; returns true if the code is found,
   *  false if not; if the code has no number, it defaults to 0
   */
  bool getInt(const char code, int32_t *number) const;
  bool getFloat(const char code, float *number) const;

 private:
  /* Stores a line of input */
  char buffer_[1024];

  /* Each index represents a letter, running from 'A' to 'Z'; if there is no
   *  for that letter, the index is NULL; otherwise, it is a pointer to the
   *  location in buffer_ where the code letter occurs (i.e. the value is the
   *  second character in the string)
   */
  char *ptrs_[26];
};

#endif
