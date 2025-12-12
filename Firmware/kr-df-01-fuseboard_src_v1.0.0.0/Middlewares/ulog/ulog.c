/**
 MIT License

 Copyright (c) 2019 R. Dunbar Poor <rdpoor@gmail.com>

 Permission is hereby granted, free of charge, to any person obtaining a copy of
 this software and associated documentation files (the "Software"), to deal in
 the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 the Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * \file ulog.c
 *
 * \brief uLog: lightweight logging for embedded systems
 *
 * See ulog.h for sparse documentation.
 */

#include "ulog.h"

#ifdef ULOG_ENABLED  // whole file...

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

// =============================================================================
// types and definitions

typedef struct
{
  ulog_function_t fn;
  ulog_level_t threshold;
} subscriber_t;

// =============================================================================
// local storage

static subscriber_t s_subscribers[ULOG_MAX_SUBSCRIBERS];
static char s_message[ULOG_MAX_MESSAGE_LENGTH];

// =============================================================================
// user-visible code

void ulog_init()
{
  memset(s_subscribers, 0, sizeof(s_subscribers));
}

// search the s_subscribers table to install or update fn
ulog_err_t ulog_subscribe(ulog_function_t fn, ulog_level_t threshold)
{
  int available_slot = -1;
  int i;
  for (i = 0; i < ULOG_MAX_SUBSCRIBERS; i++)
  {
    if (s_subscribers[i].fn == fn)
    {
      // already subscribed: update threshold and return immediately.
      s_subscribers[i].threshold = threshold;
      return ULOG_ERR_NONE;

    }
    else if (s_subscribers[i].fn == NULL)
    {
      // found a free slot
      available_slot = i;
    }
  }
  // fn is not yet a subscriber.  assign if possible.
  if (available_slot == -1)
  {
    return ULOG_ERR_SUBSCRIBERS_EXCEEDED;
  }
  s_subscribers[available_slot].fn = fn;
  s_subscribers[available_slot].threshold = threshold;
  return ULOG_ERR_NONE;
}

// search the s_subscribers table to remove
ulog_err_t ulog_unsubscribe(ulog_function_t fn)
{
  int i;
  for (i = 0; i < ULOG_MAX_SUBSCRIBERS; i++)
  {
    if (s_subscribers[i].fn == fn)
    {
      s_subscribers[i].fn = NULL;    // mark as empty
      return ULOG_ERR_NONE;
    }
  }
  return ULOG_ERR_NOT_SUBSCRIBED;
}

const char* ulog_level_name(ulog_level_t severity)
{
  switch (severity)
  {
  case ULOG_TRACE_LEVEL:
    return "TRC";
  case ULOG_DEBUG_LEVEL:
    return "DBG";
  case ULOG_INFO_LEVEL:
    return "INF";
  case ULOG_WARNING_LEVEL:
    return "WRN";
  case ULOG_ERROR_LEVEL:
    return "ERR";
  case ULOG_CRITICAL_LEVEL:
    return "CRITICAL";
  case ULOG_ALWAYS_LEVEL:
    return "ALWAYS";
  default:
    return "UNK";
  }
}

void ulog_message(ulog_level_t severity, const char* file_path, const uint32_t file_path_length, const uint32_t line, const char *fmt, ...)
{
  uint32_t file_index;
  uint32_t len;
  uint32_t i;
  va_list ap;

  // Extract file name
  for (file_index = file_path_length - 1U; file_index > 0U; file_index--)
  {
    if ((file_path[file_index] == '/') || (file_path[file_index] == '\\'))
  {
      file_index++;
      break;
    }
  }

  // Header of message
  len = snprintf(s_message, ULOG_MAX_MESSAGE_LENGTH, "%s:%lu ", &file_path[file_index], line);

  // Space left for actual message ?
  if (len < ULOG_MAX_MESSAGE_LENGTH)
  {
    va_start(ap, fmt);
    len += vsnprintf(&s_message[len], (ULOG_MAX_MESSAGE_LENGTH) - len, fmt, ap);
    va_end(ap);
  }

  // Truncate if needed (because snprintf returns number of characters that WOULD have been written).
  if (len >= ULOG_MAX_MESSAGE_LENGTH)
  {
    s_message[ULOG_MAX_MESSAGE_LENGTH - 3U] = '.';
    s_message[ULOG_MAX_MESSAGE_LENGTH - 2U] = '.';
    s_message[ULOG_MAX_MESSAGE_LENGTH - 1U] = '.';
  }

  for (i = 0; i < ULOG_MAX_SUBSCRIBERS; i++)
  {
    if (s_subscribers[i].fn != NULL)
    {
      if (severity >= s_subscribers[i].threshold)
      {
        s_subscribers[i].fn(severity, s_message);
      }
    }
  }
}

// =============================================================================
// private code

#endif  // #ifdef ULOG_ENABLED
