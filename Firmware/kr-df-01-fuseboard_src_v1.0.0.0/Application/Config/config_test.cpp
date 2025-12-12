#include "config_test.h"
#include "config.h"
#include "config_default.h"
#include "main.h"

// Separated tests to keep operation queue checks static in config.c
// Only for testing in debug
void config_save_load_test_1()
{
  CONFIG_PORT port = {0};
  port.reserved[0] = 0xFF;
  (void)config_set_port_data(&port, 0);

  CONFIG_PORT port_in = {0};
  config_get_port_data(&port_in, 0);

  if (port_in.reserved[0] != 0xFF)
  {
    Error_Handler();
  }
}

void config_save_load_test_2()
{
  CONFIG_PORT port = {0};
  port.reserved[0] = 0x00;
  (void)config_set_port_data(&port, 0);

  CONFIG_PORT port_in = {0};
  config_get_port_data(&port_in, 0);

  if (port_in.reserved[0] != 0x00)
  {
    Error_Handler();
  }
}

void config_save_load_test_3()
{
  /* Check the port from test_2 */
  CONFIG_PORT port_in = {0};
  config_get_port_data(&port_in, 0);
  if (port_in.reserved[0] != 0xFF)
  {
    Error_Handler();
  }

  /* Now test IP with 0xFFFFFFFF */
  CONFIG_GENERAL general = {0};
  general.ip_address = 0xFFFFFFFF;
  (void)config_set_general_data(&general);

  CONFIG_GENERAL general_in = {0};
  config_get_general_data(&general_in);

  if (general_in.ip_address != 0xFFFFFFFF)
  {
    Error_Handler();
  }
}

void config_save_load_test_4()
{
  /* Set IP to 0x00000000 */
  CONFIG_GENERAL general = {0};
  general.ip_address = 0x00000000;
  (void)config_set_general_data(&general);

  CONFIG_GENERAL general_in = {0};
  config_get_general_data(&general_in);

  if (general_in.ip_address != 0x00000000)
  {
    Error_Handler();
  }
}

void config_save_load_test_5()
{
  /* Confirm IP remains 0x00000000 */
  CONFIG_GENERAL general_in = {0};
  config_get_general_data(&general_in);

  if (general_in.ip_address != 0xFFFFFFFF)
  {
    Error_Handler();
  }
}
