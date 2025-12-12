/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_netxduo.c
 * @author  MCD Application Team
 * @brief   NetXDuo applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_netxduo.h"

/* Private includes ----------------------------------------------------------*/
#include "nxd_dhcp_client.h"
/* USER CODE BEGIN Includes */
#include <string.h>
#include <inttypes.h>
#include "nx_stm32_eth_config.h"
#include "nxd_mdns.h"
#include "tx_extension.h"
#include "main.h"
#include "dp_info.h"
#include "config.h"
#include "util.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  NX_TCP_SOCKET socket;
  char name[16];

  TX_QUEUE tx_queue;
  TX_QUEUE rx_queue;

  /* Buffer and indexes used to provide byte-level reception on top of packet-based reception */
  uint8_t rx_buffer[APP_TCP_BUF_SIZE];
  uint32_t rx_count;
  uint32_t rx_index;

  ULONG remote_ip;
  ULONG remote_port;

  TX_THREAD thread;
} tcp_conn_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRIMARY_IF              0 // Primary network inteface
#define LINK_UP_TIME_THRESHOLD  TX_TICKS_MS(1000)  // 1 second in OS ticks

#define TCP_TX_QUEUE_MEM_SIZE (sizeof(NX_PACKET*) * APP_TCP_BUF_COUNT)
#define TCP_RX_QUEUE_MEM_SIZE (sizeof(NX_PACKET*) * APP_TCP_BUF_COUNT)
#define TCP_TX_QUEUES_SHARED_MEM_SIZE (APP_TCP_CONN_LIMIT * TCP_TX_QUEUE_MEM_SIZE)
#define TCP_RX_QUEUES_SHARED_MEM_SIZE (APP_TCP_CONN_LIMIT * TCP_RX_QUEUE_MEM_SIZE)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TX_THREAD      NxAppThread;
NX_PACKET_POOL NxAppPool;
NX_IP          NetXDuoEthIpInstance;
TX_SEMAPHORE   DHCPSemaphore;
NX_DHCP        DHCPClient;
/* USER CODE BEGIN PV */
NX_MDNS        MDNS;

__ALIGN_BEGIN static UCHAR tcp_tx_queues_shared_mem[TCP_TX_QUEUES_SHARED_MEM_SIZE] __ALIGN_END;
__ALIGN_BEGIN static UCHAR tcp_rx_queues_shared_mem[TCP_RX_QUEUES_SHARED_MEM_SIZE] __ALIGN_END;
static TX_SEMAPHORE app_tcp_active_listener;
static tcp_conn_t   app_tcp_conns[APP_TCP_CONN_LIMIT];
static enum LINK_STATUS link_status = LINK_STATUS_NOT_AVAILABLE;

/* For mDNS */
/* TODO Not sure if this much RAM is needed, it's based on example */
static uint8_t mdns_stack[2048];
static uint8_t mdns_local_cache[2048];
static uint8_t mdns_peer_cache[2048];

/* Hostname will be constructed from device serial number */
static char hostname[32] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static VOID App_Main_Thread_Entry (ULONG thread_input);
static VOID ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr);
/* USER CODE BEGIN PFP */
static bool activate_mdns(NX_IP *ip_ptr);
static VOID app_tcp_conn_thread_entry(ULONG thread_input);
static VOID app_ip_debug_dump(void);
static void dhcp_state_change(NX_DHCP *dhcp_ptr, UCHAR new_state);
static void ipv4_to_text(char *buffer, ULONG addr);

/* USER CODE END PFP */

/**
  * @brief  Application NetXDuo Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_NetXDuo_Init(VOID *memory_ptr)
{
  UINT ret = NX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

   /* USER CODE BEGIN App_NetXDuo_MEM_POOL */

  /* USER CODE END App_NetXDuo_MEM_POOL */
  /* USER CODE BEGIN 0 */

  /* Construct hostname (0-9, aA-zZ and - are allowed) */
  snprintf(hostname, sizeof(hostname), "fuseboard-%" PRIu32, dp_get_serial_number());

  /* USER CODE END 0 */

  /* Initialize the NetXDuo system. */
  CHAR *pointer;
  nx_system_initialize();

    /* Allocate the memory for packet_pool.  */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_APP_PACKET_POOL_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Create the Packet pool to be used for packet allocation,
   * If extra NX_PACKET are to be used the NX_APP_PACKET_POOL_SIZE should be increased
   */
  ret = nx_packet_pool_create(&NxAppPool, "NetXDuo App Pool", DEFAULT_PAYLOAD_SIZE, pointer, NX_APP_PACKET_POOL_SIZE);

  if (ret != NX_SUCCESS)
  {
    return NX_POOL_ERROR;
  }

    /* Allocate the memory for Ip_Instance */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, Nx_IP_INSTANCE_THREAD_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

   /* Create the main NX_IP instance */
  ret = nx_ip_create(&NetXDuoEthIpInstance, "NetX Ip instance", NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK, &NxAppPool, nx_stm32_eth_driver,
                     pointer, Nx_IP_INSTANCE_THREAD_SIZE, NX_APP_INSTANCE_PRIORITY);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

    /* Allocate the memory for ARP */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_ARP_CACHE_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Enable the ARP protocol and provide the ARP cache size for the IP instance */

  /* USER CODE BEGIN ARP_Protocol_Initialization */

  /* USER CODE END ARP_Protocol_Initialization */

  ret = nx_arp_enable(&NetXDuoEthIpInstance, (VOID *)pointer, DEFAULT_ARP_CACHE_SIZE);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  /* Enable the ICMP */

  /* USER CODE BEGIN ICMP_Protocol_Initialization */

  /* USER CODE END ICMP_Protocol_Initialization */

  ret = nx_icmp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  /* Enable TCP Protocol */

  /* USER CODE BEGIN TCP_Protocol_Initialization */

  /* USER CODE END TCP_Protocol_Initialization */

  ret = nx_tcp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  /* Enable the UDP protocol required for  DHCP communication */

  /* USER CODE BEGIN UDP_Protocol_Initialization */

  /* USER CODE END UDP_Protocol_Initialization */

  ret = nx_udp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

   /* Allocate the memory for main thread   */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_APP_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Create the main thread */
  ret = tx_thread_create(&NxAppThread, "NetXDuo App thread", App_Main_Thread_Entry , 0, pointer, NX_APP_THREAD_STACK_SIZE,
                         NX_APP_THREAD_PRIORITY, NX_APP_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (ret != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  /* Create the DHCP client */

  /* USER CODE BEGIN DHCP_Protocol_Initialization */
  /* Use hostname to tell to DHCP server who is asking for the IP */
  /* USER CODE END DHCP_Protocol_Initialization */

  ret = nx_dhcp_create(&DHCPClient, &NetXDuoEthIpInstance, "DHCP Client");

  if (ret != NX_SUCCESS)
  {
    return NX_DHCP_ERROR;
  }

  /* set DHCP notification callback  */
  tx_semaphore_create(&DHCPSemaphore, "DHCP Semaphore", 0);

  /* USER CODE BEGIN MX_NetXDuo_Init */

  tx_semaphore_create(&app_tcp_active_listener, (CHAR* )"Active TCP listener", 1);

  for (UINT i = 0; i != APP_TCP_CONN_LIMIT; ++i)
  {
    ULOG_DEBUG("Initializing TCP conn #%u", i);
    tcp_conn_t* conn = &app_tcp_conns[i];
    memset(conn, 0, sizeof(tcp_conn_t));
    snprintf(conn->name, sizeof(conn->name), "TCP conn #%u", i);

    ret = tx_byte_allocate(byte_pool, (VOID**)&pointer, APP_TCP_THREAD_SIZE, TX_NO_WAIT);
    if (ret != TX_SUCCESS)
    {
      return TX_POOL_ERROR;
    }
    ret = tx_thread_create(&conn->thread, conn->name, app_tcp_conn_thread_entry, i, pointer, APP_TCP_THREAD_SIZE,
        APP_TCP_THREAD_PRIO, APP_TCP_THREAD_PRIO, TX_NO_TIME_SLICE, TX_DONT_START);
    if (ret != TX_SUCCESS)
    {
      return TX_THREAD_ERROR;
    }

    ret = tx_queue_create(&conn->tx_queue, conn->name, sizeof(NX_PACKET*) / 4,
        tcp_tx_queues_shared_mem + (i * TCP_TX_QUEUE_MEM_SIZE), TCP_TX_QUEUE_MEM_SIZE);
    if (ret != TX_SUCCESS)
    {
      return TX_QUEUE_ERROR;
    }
    ret = tx_queue_create(&conn->rx_queue, conn->name, sizeof(NX_PACKET*) / 4,
        tcp_rx_queues_shared_mem + (i * TCP_RX_QUEUE_MEM_SIZE), TCP_RX_QUEUE_MEM_SIZE);
    if (ret != TX_SUCCESS)
    {
      return TX_QUEUE_ERROR;
    }
  }

  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, 2 * DEFAULT_MEMORY_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* USER CODE END MX_NetXDuo_Init */

  return ret;
}

/**
* @brief  ip address change callback.
* @param ip_instance: NX_IP instance
* @param ptr: user data
* @retval none
*/
static VOID ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr)
{
  /* USER CODE BEGIN ip_address_change_notify_callback */
  char addr_str[16];
  ipv4_to_text(addr_str, ip_instance->nx_ip_interface[PRIMARY_IF].nx_interface_ip_address);
  ULOG_INFO("IP address changed to %s", addr_str);
  activate_mdns(ip_instance);
  /* USER CODE END ip_address_change_notify_callback */

  /* release the semaphore as soon as an IP address is available */
  tx_semaphore_put(&DHCPSemaphore);
}

/**
* @brief  Main thread entry.
* @param thread_input: ULONG user argument used by the thread entry
* @retval none
*/
static VOID App_Main_Thread_Entry (ULONG thread_input)
{
  /* USER CODE BEGIN Nx_App_Thread_Entry 0 */

  DP_IP_MODE ip_mode_current = DP_IP_MODE_NOT_AVAILABLE;
  DP_IP_MODE ip_mode_new = DP_IP_MODE_NOT_AVAILABLE;
  ULONG addr, mask;
  ULONG actual_status;
  ULONG link_first_up = 0;
  bool link_up = false;
  bool ip_resolved = false;
  char addr_str[16];

  /* USER CODE END Nx_App_Thread_Entry 0 */

  UINT ret = NX_SUCCESS;

  /* USER CODE BEGIN Nx_App_Thread_Entry 1 */

  /* Register the IP address change callback */
  ret = nx_ip_address_change_notify(&NetXDuoEthIpInstance, ip_address_change_notify_callback, NULL);
  if (ret != NX_SUCCESS)
  {
    ULOG_ERROR("IP address change notify error 0x%X", ret);
    /* Not critical */
  }

  /* Register DHCP state change callback */
  ret = nx_dhcp_state_change_notify(&DHCPClient, dhcp_state_change);
  if (ret != NX_SUCCESS)
  {
    ULOG_ERROR("DHCP state change notify error 0x%X", ret);
    /* Not critical */
  }

  /* Link check, IP set and check loop */
  while (true)
  {
    /* Wait a while */
    tx_thread_sleep(TX_TICKS_MS(10));

    /* Check link status */
    ret = nx_ip_interface_status_check(&NetXDuoEthIpInstance, PRIMARY_IF, NX_IP_LINK_ENABLED, &actual_status, 10);
    if (ret == NX_SUCCESS)
    {
      if (link_first_up == 0)
      {
        link_first_up = tx_time_get();
      }
      else if ((tx_time_get() >= link_first_up + LINK_UP_TIME_THRESHOLD) && (!link_up))
      {
        // Link is up now
        ULOG_INFO("Network cable is connected (stable).");
        link_up = true;

        // Workaround for NetXDuo issue? It does not (re)enable driver if link is made, so do it manually.
        // Related issue:
        // https://community.st.com/t5/stm32-mcus-embedded-software/netxduo-link-state-not-initialized-properly/td-p/658302
        (void)nx_ip_driver_direct_command(&NetXDuoEthIpInstance, NX_LINK_ENABLE, &actual_status);

        // Trigger acquiring same/new IP
        ip_mode_current = DP_IP_MODE_NOT_AVAILABLE;
        ip_resolved = false;
      }
    }
    else
    {
      /* Reset link flags */
      link_first_up = 0;
      if (link_up)
      {
        ULOG_WARNING("Network cable disconnected (0x%X).", ret);
        link_up = false;
        ip_resolved = false;
        link_status = LINK_STATUS_DISCONNECTED;

        /* Suspending connection threads due to no ethernet connection */
        for (UINT i = 0; i != APP_TCP_CONN_LIMIT; ++i)
        {
          tx_thread_suspend(&app_tcp_conns[i].thread);
        }

        /* In case dynamic IP was set, stop DHCP client and re-init it */
        if (ip_mode_current == DP_IP_MODE_DYNAMIC)
        {
          (void)nx_dhcp_stop(&DHCPClient);
          (void)nx_dhcp_reinitialize(&DHCPClient);
        }
      }
    }

    /* No need to proceed if link is not up */
    if (!link_up)
    {
      continue;
    }

    /* Check for IP mode set (or change) request using local variable as a buffer for maximum thread safety. */
    ip_mode_new = dp_get_ip_mode();
    if (ip_mode_new != ip_mode_current)
    {
      /* No IP yet */
      ip_resolved = false;

      /* If DHCP is not required, then make sure it is stopped */
      if (ip_mode_new != DP_IP_MODE_DYNAMIC)
      {
        ret = nx_dhcp_stop(&DHCPClient);
        if (ret != NX_SUCCESS)
        {
          ULOG_WARNING("DHCP stop error 0x%X", ret);
          /* Let's hope it's not a big deal and continue */
        }
      }

      /* Static IP ? */
      if (ip_mode_new == DP_IP_MODE_STATIC)
      {
        /* TODO Static IP shall come from main loop */
        CONFIG_GENERAL gen;
        config_get_general_data(&gen);


        addr = gen.ip_address;
        mask = USER_DEFAULT_NET_MASK;

        /* Set static IP */
        ipv4_to_text(addr_str, addr);
        ULOG_INFO("Setting static IP address %s", addr_str);
        ret = nx_ip_address_set(&NetXDuoEthIpInstance, addr, mask);
        if (ret == NX_SUCCESS)
        {
          /* Acquiring IP (until verifying that static IP was set) */
          link_status = LINK_STATUS_AQUIRING_IP;

          /* Remember new mode */
          ip_mode_current = ip_mode_new;
        }
        else
        {
          ULOG_ERROR("Failed to set IP error 0x%X", ret);

          /* Try again in a second */
          tx_thread_sleep(TX_TICKS_MS(1000));
        }
      }
      /* Dynamic IP ? */
      else if (ip_mode_new == DP_IP_MODE_DYNAMIC)
      {
        /* Before DHCP start, revert to default "no IP" */
        ret = nx_ip_address_set(&NetXDuoEthIpInstance, NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK);
        if (ret != NX_SUCCESS)
        {
          ULOG_WARNING("Set default IP error 0x%X", ret);
          /* Let's hope it's not a big deal and continue */
        }

        /* Start the DHCP client */
        ULOG_INFO("Start DHCP client");
        ret = nx_dhcp_start(&DHCPClient);
        if (ret == NX_SUCCESS)
        {
          /* Acquiring IP */
          link_status = LINK_STATUS_AQUIRING_IP;

          /* Remember new mode */
          ip_mode_current = ip_mode_new;
        }
        else
        {
          ULOG_ERROR("DHCP start error 0x%X", ret);

          /* Try again in a second */
          tx_thread_sleep(TX_TICKS_MS(1000));
        }
      }
    }

    /* If link is up then verify IP status */
    if (link_up)
    {
      ret = nx_ip_interface_status_check(&NetXDuoEthIpInstance, PRIMARY_IF, NX_IP_ADDRESS_RESOLVED, &actual_status, 10);
      if ((ret == NX_SUCCESS) && (actual_status == NX_IP_ADDRESS_RESOLVED))
      {
        ret = nx_ip_interface_address_get(&NetXDuoEthIpInstance, PRIMARY_IF, &addr, &mask);
        if (ret == NX_SUCCESS)
        {
          if ((addr != NX_APP_DEFAULT_IP_ADDRESS) && (!ip_resolved))
          {
            ipv4_to_text(addr_str, addr);
            ULOG_INFO("IP address is %s", addr_str);

            /* Set new status */
            ip_resolved = true;
            link_status = LINK_STATUS_CONNECTED;

            /* Resume transmission threads */
            for (UINT i = 0; i != APP_TCP_CONN_LIMIT; ++i)
            {
              tx_thread_resume(&app_tcp_conns[i].thread);
            }
          }
        }
        else if (ret != NX_NOT_SUCCESSFUL)
        {
          ULOG_ERROR("IP address get error 0x%X", ret);
        }
      }
      else if (ret == NX_NOT_SUCCESSFUL)
      {
        if (ip_resolved)
        {
          ULOG_WARNING("IP address not resolved.");
          ip_resolved = false;
        }
      }
      else
      {
        ULOG_ERROR("IP status check error 0x%X", ret);
      }
    }
  }

  /* Should never reach here */
  return;

  /* USER CODE END Nx_App_Thread_Entry 1 */

  /* register the IP address change callback */
  ret = nx_ip_address_change_notify(&NetXDuoEthIpInstance, ip_address_change_notify_callback, NULL);
  if (ret != NX_SUCCESS)
  {
    /* USER CODE BEGIN IP address change callback error */
    /* USER CODE END IP address change callback error */
  }

  /* start the DHCP client */
  ret = nx_dhcp_start(&DHCPClient);
  if (ret != NX_SUCCESS)
  {
    /* USER CODE BEGIN DHCP client start error */
    /* USER CODE END DHCP client start error */
  }

  /* wait until an IP address is ready */
  if(tx_semaphore_get(&DHCPSemaphore, NX_APP_DEFAULT_TIMEOUT) != TX_SUCCESS)
  {
    /* USER CODE BEGIN DHCPSemaphore get error */
    /* USER CODE END DHCPSemaphore get error */
  }

  /* USER CODE BEGIN Nx_App_Thread_Entry 2 */

  /* USER CODE END Nx_App_Thread_Entry 2 */

}
/* USER CODE BEGIN 1 */
static void dhcp_state_change(NX_DHCP *dhcp_ptr, UCHAR new_state)
{
  ULOG_DEBUG("DHCP state %u", new_state);
}

static void app_ip_debug_dump(void)
{
  ULONG ip_total_packets_sent, ip_total_bytes_sent, ip_total_packets_received, ip_total_bytes_received,
      ip_invalid_packets, ip_receive_packets_dropped, ip_receive_checksum_errors, ip_send_packets_dropped,
      ip_total_fragments_sent, ip_total_fragments_received;
  nx_ip_info_get(&NetXDuoEthIpInstance, &ip_total_packets_sent, &ip_total_bytes_sent, &ip_total_packets_received,
      &ip_total_bytes_received, &ip_invalid_packets, &ip_receive_packets_dropped, &ip_receive_checksum_errors,
      &ip_send_packets_dropped, &ip_total_fragments_sent, &ip_total_fragments_received);
  ULOG_DEBUG("NetXDuo IP dump:\r\n"
      "ip_total_pkt_tx: %u\r\n"
      "ip_total_byte_tx: %u\r\n"
      "ip_total_pkt_rx: %u\r\n"
      "ip_total_bkt_rx: %u\r\n"
      "ip_invalid_pkt: %u\r\n"
      "ip_rx_pkt_dropped: %u\r\n"
      "ip_rx_checksum_errors: %u\r\n"
      "ip_tx_pkt_dropped: %u\r\n"
      "ip_total_frags_tx: %u\r\n"
      "ip_total_frags_rx: %u ", ip_total_packets_sent, ip_total_bytes_sent, ip_total_packets_received,
      ip_total_bytes_received, ip_invalid_packets, ip_receive_packets_dropped, ip_receive_checksum_errors,
      ip_send_packets_dropped, ip_total_fragments_sent, ip_total_fragments_received);
}

/**
 * Activate multicast DNS so others can resolve this device hostname
 * @param ip_ptr: Pointer to this device IP
 */
static bool activate_mdns(NX_IP *ip_ptr)
{
  UINT ret;

  /* Disable and delete previous mDNS instance (If it was active) */
  if (nx_mdns_disable(&MDNS, PRIMARY_IF) == NX_MDNS_SUCCESS)
  {
    (void)nx_mdns_delete(&MDNS);
  }

  /* Create mDNS instance */
  /* TODO Not 100% sure that same general packet pool (NxAppPool) can be used. */
  ret = nx_mdns_create(&MDNS, ip_ptr, &NxAppPool, NX_APP_THREAD_PRIORITY,
      (VOID *)mdns_stack, sizeof(mdns_stack), (UCHAR *)hostname,
      (VOID *)mdns_local_cache, sizeof(mdns_local_cache),
      (VOID *)mdns_peer_cache, sizeof(mdns_peer_cache),
      NULL);
  if (ret != NX_SUCCESS)
  {
    ULOG_ERROR("mDNS creation error: %u", ret);
    return false;
  }

  /* Enable the mDNS function.  */
  ret = nx_mdns_enable(&MDNS, PRIMARY_IF);
  if (ret != NX_MDNS_SUCCESS)
  {
    ULOG_ERROR("mDNS enabling error: %u", ret);
    return false;
  }

  return true;
}


static VOID tcp_connect_received(NX_TCP_SOCKET *socket_ptr, UINT port)
{

}

static VOID tcp_disconnect_received(NX_TCP_SOCKET *socket)
{

}

// Creates TCP socket and starts listening for a connection; blocks until listening begins.
static tcp_conn_t* app_tcp_conn_init(ULONG conn_index)
{
  UINT ret;
  tcp_conn_t* conn = &app_tcp_conns[conn_index];

  ret = nx_tcp_socket_create(&NetXDuoEthIpInstance, &conn->socket, conn->name, NX_IP_NORMAL, NX_FRAGMENT_OKAY,
      NX_IP_TIME_TO_LIVE, APP_TCP_DEFAULT_COMM_TIMEOUT, NX_NULL, tcp_disconnect_received);
  if (ret != NX_SUCCESS)
  {
    ULOG_CRITICAL("%s socket create err: %u", conn->name, ret);
    Error_Handler();
  }

  // Only one socket can listen to a connection on the same port:
  tx_semaphore_get(&app_tcp_active_listener, TX_WAIT_FOREVER);
  ret = nx_tcp_server_socket_listen(&NetXDuoEthIpInstance, APP_TCP_PORT, &conn->socket,
      APP_TCP_LISTEN_QUEUE_SIZE, tcp_connect_received);
  if (ret != NX_SUCCESS)
  {
    ULOG_CRITICAL("%s socket listen err: %u", conn->name, ret);
    Error_Handler();
  }

  ULOG_DEBUG("%s listening on port %u.", conn->name, APP_TCP_PORT);
  return conn;
}

// Waits for connection by peer. Returns TX_SUCCESS if connected, or NX_NOT_CONNECTED if not connected;
// may return internal error otherwise.
// Connection socket must be in listening state!
static UINT app_tcp_conn_wait_connect(tcp_conn_t *conn)
{
  UINT ret;
  ret = nx_tcp_server_socket_accept(&conn->socket, APP_TCP_DEFAULT_COMM_TIMEOUT);
  if (ret == NX_NOT_CONNECTED)
  {
    // Timeout.
    return ret;
  }

  if (ret != NX_SUCCESS)
  {
    ULOG_ERROR("%s socket accept err: %u", conn->name, ret);
    return ret;
  }

  nx_tcp_socket_peer_info_get(&conn->socket, &conn->remote_ip, &conn->remote_port);
  ULOG_INFO("%s connected to [%lu.%lu.%lu.%lu:%u]", conn->name, (conn->remote_ip >> 24) & 0xff,
      (conn->remote_ip >> 16) & 0xff, (conn->remote_ip >> 8) & 0xff, (conn->remote_ip & 0xff), conn->remote_port);
  nx_tcp_server_socket_unlisten(&NetXDuoEthIpInstance, APP_TCP_PORT);

  // Connection established; another socket can wait for a connection on this same port now:
  tx_semaphore_put(&app_tcp_active_listener);
  return NX_SUCCESS;
}

// Resets connection socket by disconnecting and unaccepting the active peer.
// Starts listening for a new connection on this socket; blocks until listening begins.
static VOID app_tcp_conn_reset(tcp_conn_t *conn)
{
  ULOG_INFO("%s disconnected from [%lu.%lu.%lu.%lu:%u]", conn->name, (conn->remote_ip >> 24) & 0xff,
      (conn->remote_ip >> 16) & 0xff, (conn->remote_ip >> 8) & 0xff, (conn->remote_ip & 0xff), conn->remote_port);

  // Reset peer info immediately to avoid user trying to communicate with it:
  conn->remote_ip = 0;
  conn->remote_port = 0;

  nx_tcp_socket_disconnect(&conn->socket, APP_TCP_DEFAULT_COMM_TIMEOUT);
  nx_tcp_server_socket_unaccept(&conn->socket);

  // Wait for another connection, but only after acquiring the listening rights on the port:
  tx_semaphore_get(&app_tcp_active_listener, TX_WAIT_FOREVER);
  nx_tcp_server_socket_listen(&NetXDuoEthIpInstance, APP_TCP_PORT, &conn->socket,
  APP_TCP_LISTEN_QUEUE_SIZE, tcp_connect_received);

  // Reset any leaked data from previous connection:
  UINT ret;
  NX_PACKET* packet;
  while (1)
  {
    ret = tx_queue_receive(&conn->tx_queue, &packet, TX_NO_WAIT);
    if (ret == TX_SUCCESS)
      nx_packet_release(packet);
    else
      break;
  }
  while (1)
  {
    ret = tx_queue_receive(&conn->rx_queue, &packet, TX_NO_WAIT);
    if (ret == TX_SUCCESS)
      nx_packet_release(packet);
    else
      break;
  }
}

// Checks for RX data and moves it to the RX queue if available.
// Returns NX_NO_PACKET if nothing to receive, NX_NOT_CONNECTED if connection has been lost;
// may return internal error otherwise.
static UINT app_tcp_conn_handle_rx(tcp_conn_t *conn)
{
  UINT ret;
  NX_PACKET* data_packet;

  ret = nx_tcp_socket_receive(&conn->socket, &data_packet, TX_NO_WAIT);
  if (ret == NX_NOT_CONNECTED || ret == NX_NO_PACKET)
    return ret;
  if (ret != NX_SUCCESS)
  {
    ULOG_ERROR("%s socket receive err: %u", conn->name, ret);
    return ret;
  }

  ret = tx_queue_send(&conn->rx_queue, &data_packet, TX_WAIT_FOREVER);
  if (ret != NX_SUCCESS)
  {
    ULOG_ERROR("%s sending RX data to queue failed: %u", conn->name, ret);
    return ret;
  }
  return NX_SUCCESS;
}

// Checks the TX queue and copies any outgoing data to the socket.
// Returns NX_NO_PACKET if nothing to transmit, NX_NOT_CONNECTED if connection has been lost;
// may return internal error otherwise.
static UINT app_tcp_conn_handle_tx(tcp_conn_t *conn)
{
  UINT ret;
  NX_PACKET* data_packet;

  ret = tx_queue_receive(&conn->tx_queue, &data_packet, TX_NO_WAIT);
  if (ret == TX_QUEUE_EMPTY)
    return NX_NO_PACKET;
  if (ret != TX_SUCCESS)
    return TX_QUEUE_ERROR;

  ret = nx_tcp_socket_send(&conn->socket, data_packet, TX_WAIT_FOREVER);
  if (ret != NX_SUCCESS)
  {
    ULOG_ERROR("%s sending TX data failed: %u", conn->name, ret);
    return ret;
  }
  nx_packet_release(data_packet);
  return NX_SUCCESS;
}

static VOID app_tcp_conn_thread_entry(ULONG thread_input)
{
  UINT ret;
  const ULONG conn_index = thread_input;
  tcp_conn_t* conn = app_tcp_conn_init(conn_index);

  while (1)
  {
    ret = app_tcp_conn_wait_connect(conn);
    if (ret != TX_SUCCESS)
      continue;

    while (1)
    {
      UINT rx_ret = app_tcp_conn_handle_rx(conn);
      if (rx_ret == NX_NOT_CONNECTED)
      {
        app_tcp_conn_reset(conn);
        break;
      }
      UINT tx_ret = app_tcp_conn_handle_tx(conn);
      if (tx_ret == NX_NOT_CONNECTED)
      {
        app_tcp_conn_reset(conn);
        break;
      }

      if (rx_ret != NX_SUCCESS && tx_ret != NX_SUCCESS)
      {
        tx_thread_sleep(0.1 * TX_TIMER_TICKS_PER_SECOND);
      }
    }
  }
}

static tcp_conn_t* get_conn(const tcp_peer_t *peer)
{
  for (UINT i = 0; i != APP_TCP_CONN_LIMIT; ++i)
  {
    tcp_conn_t* conn = &app_tcp_conns[i];
    if (conn->remote_ip == peer->ip && conn->remote_port == peer->port)
    {
      return conn;
    }
  }
  return NULL;
}

tcp_peer_t netxduo_tcp_get_next_peer(tcp_peer_t current_peer)
{
  tcp_peer_t next_peer = { .ip = 0, .port = 0, };
  UINT start_i = 0;

  if (current_peer.ip != 0 && current_peer.port != 0)
  {
    for (UINT i = 0; i != APP_TCP_CONN_LIMIT; ++i)
    {
      tcp_conn_t* conn = &app_tcp_conns[i];
      if (conn->remote_ip == current_peer.ip && conn->remote_port == current_peer.port)
      {
        start_i = i + 1;
        break;
      }
    }
    if (start_i == 0) {
      // `current_peer` has disconnected; return nothing.
      return next_peer;
    }
  }

  for (UINT i = start_i; i < APP_TCP_CONN_LIMIT; ++i)
  {
    tcp_conn_t* conn = &app_tcp_conns[i];
    if (conn->remote_ip != 0 && conn->remote_port != 0)
    {
      next_peer.ip = conn->remote_ip;
      next_peer.port = conn->remote_port;
      break;
    }
  }
  return next_peer;
}

/**
 * Try to read RX data bytes from TCP connection RX buffer.
 * @param conn: TCP connection pointer.
 * @param dst: Destination byte buffer.
 * @param len: Number of bytes to read (maximum).
 * @return Number of bytes actually read from RX buffer.
 */
static uint32_t netxduo_tcp_rx_buffer(tcp_conn_t* conn, uint8_t *dst, uint32_t len)
{
  if (conn->rx_index < conn->rx_count)
  {
    uint32_t bytes_available = conn->rx_count - conn->rx_index;
    uint32_t bytes_to_get = MIN(bytes_available, len);

    (void)memcpy(dst, &conn->rx_buffer[conn->rx_index], bytes_to_get);
    conn->rx_index += bytes_to_get;

    return bytes_to_get;
  }

  return 0;
}

/**
 * Byte-based TCP data reception.
 * It only uses waiting for timeout if there is already data in the buffer (or RX queue) and more is expected.
 * If there is no data in the buffer (or RX queue) then it returns immediately with return value TX_QUEUE_EMPTY.
 * This way it avoids blocking caller thread.
 */
uint32_t netxduo_tcp_rx(const tcp_peer_t *peer, uint8_t *dst, uint32_t request_len, uint32_t *return_len, uint32_t timeout_ms)
{
  ASSERT_FATAL(peer != NULL, "Peer is NULL");
  ASSERT_FATAL(dst != NULL, "Dst is NULL");
  ASSERT_FATAL(return_len != NULL, "Return length is NULL");

  UINT ret;
  NX_PACKET* data_packet;
  ULONG retrieve_len;
  ULONG timeout_ticks;
  uint32_t read_len;

  /* Zero RX check */
  if (request_len == 0)
  {
    *return_len = 0;
    return TX_SUCCESS;
  }

  /* Get TCP connection of the peer */
  tcp_conn_t* conn = get_conn(peer);
  if (conn == NULL)
  {
    ULOG_ERROR("No connection with peer %u:%u", peer->ip, peer->port);
    *return_len = 0;
    return NX_NOT_CONNECTED;
  }

  /* Get buffered bytes first (if there are) */
  read_len = netxduo_tcp_rx_buffer(conn, dst, request_len);
  *return_len = read_len;

  /* Try to get all the requested bytes from packet(s) */
  while (read_len < request_len)
  {
    /* Decide RX packat timeout.
     * If it's a first byte reading, don't use timeout and just check if there is a packet in RX queue.
     * If it's a consecutive byte or packet read, then use timeout on waiting for the next packet(s). */
    if (read_len == 0)
    {
      timeout_ticks = TX_NO_WAIT;
    }
    else
    {
      timeout_ticks = TX_TICKS_MS(timeout_ms);
    }

    /* Receive packet (within timeout if needed) */
    ret = tx_queue_receive(&conn->rx_queue, &data_packet, timeout_ticks);
    if (ret == TX_QUEUE_EMPTY)
    {
      /* It's a timeout */
      return TX_QUEUE_EMPTY;
    }
    else if (ret != TX_SUCCESS)
    {
      ULOG_ERROR("%s TCP RX queue receive error: %u", conn->name, ret);
      return TX_QUEUE_ERROR;
    }

    ret = nx_packet_data_retrieve(data_packet, conn->rx_buffer, &retrieve_len);
    if (ret == TX_SUCCESS)
    {
      conn->rx_count = (uint32_t)retrieve_len;
      conn->rx_index = 0;
    }
    else
    {
      ULOG_ERROR("%s TCP RX data retrieve error: %u", conn->name, ret);
      return TX_QUEUE_ERROR;
    }

    ret = nx_packet_release(data_packet);
    if (ret != TX_SUCCESS)
    {
      ULOG_ERROR("%s TCP RX data release error: %u", conn->name, ret);
      return TX_QUEUE_ERROR;
    }

    /* Append received bytes to destination buffer */
    read_len += netxduo_tcp_rx_buffer(conn, &dst[read_len], request_len - read_len);
    *return_len = read_len;
  }

  /* Got all bytes */
  return TX_SUCCESS;
}

/**
 * Byte-based TCP data transmission.
 */
uint32_t netxduo_tcp_tx(const tcp_peer_t *peer, const uint8_t *src, uint32_t len, uint32_t timeout_ms)
{
  ASSERT_FATAL(peer != NULL, "Peer is NULL");
  ASSERT_FATAL(src != NULL, "Dst is NULL");
  ASSERT_FATAL(len <= APP_TCP_BUF_SIZE, "len > APP_TCP_BUF_SIZE");

  UINT ret;
  NX_PACKET* data_packet;

  tcp_conn_t* conn = get_conn(peer);
  if (!conn)
  {
    ULOG_ERROR("%s no connection with peer %u:%u", conn->name, peer->ip, peer->port);
    return NX_NOT_CONNECTED;
  }

  ret = nx_packet_allocate(&NxAppPool, &data_packet, NX_TCP_PACKET, TX_NO_WAIT);
  if (ret != TX_SUCCESS)
  {
    ULOG_ERROR("%s TCP TX packet allocate error: %u", conn->name, ret);
    return NX_POOL_ERROR;
  }

  ret = nx_packet_data_append(data_packet, (uint8_t *)src, len, &NxAppPool, TX_NO_WAIT);
  if (ret != TX_SUCCESS)
  {
    ULOG_ERROR("%s TCP TX packet append error: %u", conn->name, ret);
    return NX_POOL_ERROR;
  }

  ret = tx_queue_send(&conn->tx_queue, &data_packet, TX_TICKS_MS(timeout_ms));
  if (ret != TX_SUCCESS)
  {
    ULOG_ERROR("%s TCP TX queue send error: %u", conn->name, ret);
    return TX_QUEUE_ERROR;
  }

  return TX_SUCCESS;
}

/**
 * Get link status
 * @return Link status
 */
enum LINK_STATUS netxduo_get_link_status()
{
  return link_status;
}

/**
 * IPv4 address to text
 * @param buffer: At least 16 character buffer (4*3+3+1)
 * @param addr: IPv4 address integer
 */
static void ipv4_to_text(char *buffer, ULONG addr)
{
  snprintf(buffer, 16, "%u.%u.%u.%u",
      (uint8_t)((addr >> 24) & 0xFF),
      (uint8_t)((addr >> 16) & 0xFF),
      (uint8_t)((addr >> 8) & 0xFF),
      (uint8_t)(addr & 0xFF));
}

/* USER CODE END 1 */
