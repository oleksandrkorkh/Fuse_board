#include "watchdog.h"
#include "log_flash.h"
#include "ulog.h"
#include "main.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <tx_api.h>
#include "tx_extension.h"
#include "nvm.h"
#include <assert.h>
#include "crc.h"

// Note: ASSERT_FATAL will cause ulog to put a message to queue here
_Static_assert(LOG_FLASH_MAX_LOG_LINE_SIZE > 0, "LOG_FLASH_MAX_LOG_LINE_SIZE too small");

/* Private define ------------------------------------------------------------*/
//#define LOG_DEBUG_TEST_FIELDS
#define LOG_EVENT_QUEUE_MEM_SIZE 512
#define LOG_EMPTY_CRC            0x0
#define LOG_STRUCT_SIZE_BYTES   (16 * 4) /* 16 * 32-bits = 64 bytes. Threadx queue message max size if 16 * 32bits */

/* Private types -------------------------------------------------------------*/
typedef struct
{
  ulog_level_t severity; // 1 byte
  char msg[LOG_STRUCT_SIZE_BYTES - sizeof(ulog_level_t)]; // 63 bytes
} LOG_MESSAGE;

typedef struct {
  uint32_t crc32;
  uint32_t line;
} LOG_MEMORY;

/* Private variables ---------------------------------------------------------*/
static UCHAR    log_event_queue_mem[LOG_EVENT_QUEUE_MEM_SIZE];
static TX_QUEUE log_event_queue;

static LOG_MEMORY log_memory[LOG_FLASH_MAX_LOG_LINES] = {0};
static const char log_file[]                          = "log";

/* Public variables ----------------------------------------------------------*/
extern CRC_HandleTypeDef hcrc;

/* Private function prototypes -----------------------------------------------*/
static bool log_flash_memory_init();
static void log_flash_send_to_queue(ulog_level_t severity, const char *msg);

/* Public functions ----------------------------------------------------------*/

/**
 * Initialize log to flash saving
 */
void log_flash_init()
{
  UINT status = tx_queue_create(&log_event_queue, (char* )"Log events", TX_16_ULONG, log_event_queue_mem, LOG_EVENT_QUEUE_MEM_SIZE);
  ASSERT_FATAL(status == TX_SUCCESS, "Queue creation failed with status: %u", status);

  /* Try to initialize Flash logging.
   * If it does not succeed, then there's no point to subscribe to log messages. */
  if (log_flash_memory_init())
  {
    ulog_err_t result = ulog_subscribe(log_flash_send_to_queue, ULOG_WARNING_LEVEL);
    ASSERT_FATAL(result == ULOG_ERR_NONE, "Failed to subscribe to ulog!: %u", result);
  }
}

/*
 * Try to load previous logs from log file.
 * File system could be inaccessible, so fail gracefully.
 * @return true if log file loaded and parsed, false if not.
 */
static bool log_flash_memory_init()
{
  uint8_t log_flash_load_buf[LOG_FLASH_MAX_SIZE] = { 0 };

  if (!nvm_is_mounted())
  {
    return false;
  }

  if (!nvm_read(log_file, log_flash_load_buf, LOG_FLASH_MAX_SIZE, NVM_READ_CREATE))
  {
    // Error already reported
    return false;
  }

  char (*log_flash_array)[LOG_FLASH_MAX_LOG_LINE_SIZE] = (char (*)[LOG_FLASH_MAX_LOG_LINE_SIZE])log_flash_load_buf;

  for (uint16_t cur_log_line = 0; cur_log_line < LOG_FLASH_MAX_LOG_LINES; cur_log_line++)
  {
    uint32_t crc;
    unsigned long line;

    int ret = sscanf(log_flash_array[cur_log_line], "%" SCNx32 ":%*[^:]:%*[^:]:%lu:", &crc, &line);
    if (ret == 2)
    {
      log_memory[cur_log_line].crc32 = crc;
      log_memory[cur_log_line].line  = line;
    }
    else
    {
      log_memory[cur_log_line].crc32 = 0x0;
      log_memory[cur_log_line].line  = 0x0;
    }
  }

  return true;
}

void log_flash_send_to_queue(ulog_level_t severity, const char *msg)
{
  UINT status;
  LOG_MESSAGE log_item;

  log_item.severity = severity;
  snprintf(log_item.msg, sizeof(log_item.msg), msg);

  status = tx_queue_send(&log_event_queue, &log_item, TX_NO_WAIT); // Must not suspend in IRQ
  ASSERT_FATAL(status == TX_SUCCESS, "Failed to send message, status: %u", status);
}

// Used to get file path, line and message from ulog message
// returns true if parsed successfully
static bool log_ulog_parse_msg(const char *msg, char *file_path, uint8_t file_path_length, unsigned long *line, char *message, size_t message_size)
{
  ASSERT_FATAL(msg != NULL, "msg is NULL");

  char temp_file_path[file_path_length];
  int i = 0;
  bool success = true;
  uint16_t max_msg_size = sizeof(((LOG_MESSAGE *)0)->msg);

  while ((msg[i] != ':') && (msg[i] != '\0') && (i < file_path_length))
  {
    temp_file_path[i] = msg[i];
    i++;
  }
  temp_file_path[i] = '\0';
  strncpy(file_path, temp_file_path, file_path_length);

  if (msg[i] != ':')
  {
    success = false;
  }
  else
  {
    i++;
    *line = strtoul(&msg[i], NULL, 10);

    while ((msg[i] != '\0') && (msg[i] != ' ') && (i < max_msg_size))
    {
      i++;
    }

    while ((msg[i] == ' ') && (i < max_msg_size))
    {
      i++;
    }

    if (msg[i] == '\0')
    {
      success = false;
    }
    else
    {
      strncpy(message, &msg[i], message_size);
    }
  }

  return success;
}

// returns -1 if crc not found
static int16_t check_preexisting_log_crc(uint32_t crc)
{
  int16_t result = -1;

  for (uint16_t line = 0; line < LOG_FLASH_MAX_LOG_LINES; line++)
  {
    uint32_t found_crc = log_memory[line].crc32;
    if (found_crc == LOG_EMPTY_CRC)
    {
      break;
    }
    else if (found_crc == crc)
    {
      result = line;
      break;
    }
  }

  return result;
}

static int16_t find_next_empty_line(bool *log_space_left)
{
  uint16_t line_count = sizeof(log_memory) / sizeof(LOG_MEMORY);

  int16_t result = -1;
  for (uint16_t line = 0; line < line_count; line++)
  {
    uint32_t found_crc = log_memory[line].crc32;
    if (found_crc == LOG_EMPTY_CRC)
    {
      result = line;
      *log_space_left = true;
      break;
    }
  }

  if (result < 0)
  {
    *log_space_left = false;
  }

  return result;
}

static void log_flash_create_line(ulog_level_t severity, char *message, char *output_buffer, size_t output_buffer_size,
                                  char *file_path, unsigned long line, uint32_t crc)
{
  char truncated_severity_name[3];
  memcpy(truncated_severity_name, ulog_level_name(severity), 3);

  size_t used_space = snprintf(output_buffer, output_buffer_size,
                               "%" PRIx32 ":%s:%s:%lu:", crc, truncated_severity_name, file_path, line);

  if (used_space < output_buffer_size)
  {
    size_t remaining_space = output_buffer_size - used_space - 1; // -1 is space for \0
    size_t message_length = strlen(message);

    if (message_length > remaining_space)
    {
      memcpy(output_buffer + used_space, message, remaining_space);
      memcpy(output_buffer + output_buffer_size - 4, "...", 3);
    }
    else
    {
      memcpy(output_buffer + used_space, message, message_length);
    }

    output_buffer[output_buffer_size - 1] = '\0';
  }
  else
  {
    if (output_buffer_size >= 4)
    {
      memcpy(output_buffer + output_buffer_size - 4, "...", 3);
    }
    output_buffer[output_buffer_size - 1] = '\0';
  }
}


static bool log_flash_save(const char *output_buffer, uint32_t parsed_buf_size, uint32_t crc, uint32_t line)
{
  /* TODO: in case of live format, previous logs will still be in local array
     but not in flash. Next line is still calculated according to how filled local array is
     Currently corrected after next reboot due to array being reloaded */

  bool err = true;
  bool log_space_left = false;
  uint32_t next_line_write = find_next_empty_line(&log_space_left);
  if (log_space_left)
  {
    log_memory[next_line_write].crc32 = crc;
    log_memory[next_line_write].line  = line;
    err = nvm_write(log_file, (uint8_t *)output_buffer, parsed_buf_size, NVM_OP_APPEND);
  }

  return err;
}

static bool log_ulog_parser(ulog_level_t severity, const char *msg, char *parsed_buf, uint32_t *crc_ptr, uint32_t *line_ptr)
{
  // Main log parts, must fit into the output buffer
  uint32_t calc_crc;
  unsigned long line;
  char file_path[LOG_FLASH_MAX_LOG_LINE_SIZE - sizeof(line) - sizeof(calc_crc)] = {0};
  bool new_line = true;

  ASSERT_FATAL(sizeof(file_path) > 0, "file_path buffer has ended up too small")

  char message      [LOG_FLASH_MAX_LOG_LINE_SIZE] = {0};
  char output_buffer[LOG_FLASH_MAX_LOG_LINE_SIZE] = {0};

  bool parsed = log_ulog_parse_msg(msg, file_path, sizeof(file_path), &line, message, sizeof(message));

  if (parsed)
  {
    *line_ptr = line;
    char crc_input_buffer[LOG_FLASH_MAX_LOG_LINE_SIZE - sizeof(calc_crc)] = {0};

    // Get line and file_path to calculate current log crc
    snprintf(crc_input_buffer, sizeof(crc_input_buffer), "%s:%lu", file_path, line);
    calc_crc = crc_gen_crc32((uint32_t *)crc_input_buffer, sizeof(crc_input_buffer));

    // Check if a log with same crc has been written to log before)
    int16_t crc_match_line = check_preexisting_log_crc(calc_crc);
    if (crc_match_line >= 0)
    {
      new_line = false; // If same log entry was found in memory, do not write a new line
    }
    else
    {
      // Creating a new string line to be written to flash
      *crc_ptr = calc_crc;
      log_flash_create_line(severity, message, output_buffer, sizeof(output_buffer), file_path, line, calc_crc);
    }
  }
  else
  {
    // In case of parse failure, write that fact to log
    snprintf(output_buffer, sizeof(output_buffer), "Failed to parse msg");
    output_buffer[sizeof(output_buffer) - 1] = '\0';
  }

  if (parsed && new_line)
  {
    snprintf(parsed_buf, sizeof(LOG_MESSAGE), output_buffer);
  }

  return (parsed && new_line);
}

/**
 * Log to file thread.
 */
static void log_thread_entry()
{
  const uint32_t update_interval_ms = 100;

  /* Loop gets shorter than interval time if log queue is filled faster than loop works,
   * so use variadic interval. */
  watchdog_configure_thread(WATCHDOG_THREAD_LOG, WATCHDOG_INTERVAL_VARIADIC, 2000);

  while (true)
  {
    LOG_MESSAGE incoming_log;
    UINT status = tx_queue_receive(&log_event_queue, &incoming_log, TX_TICKS_MS(update_interval_ms));
    if (status == TX_SUCCESS)
    {
      char parsed_buf[LOG_FLASH_MAX_LOG_LINE_SIZE];
      uint32_t parsed_buf_size = LOG_FLASH_MAX_LOG_LINE_SIZE;
      uint32_t crc;
      uint32_t line;

      if (log_ulog_parser(incoming_log.severity, incoming_log.msg, parsed_buf, &crc, &line))
      {
        log_flash_save(parsed_buf, parsed_buf_size, crc, line); // In case format was done, saving will recreate the file
      }
    }
    else if (status == TX_QUEUE_EMPTY)
    {
      /* Continue with loop */
    }
    else
    {
      /* Ignore errors, because it's a logging system and
       * we don't want logging system errors to create snowball effect. */
    }

    watchdog_kick_thread(WATCHDOG_THREAD_LOG);
  }
}

VOID Log_Thread_Entry(ULONG)
{
  log_thread_entry();
}

