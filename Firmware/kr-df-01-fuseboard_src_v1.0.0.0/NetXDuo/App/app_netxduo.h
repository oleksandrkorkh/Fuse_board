/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_netxduo.h
  * @author  MCD Application Team
  * @brief   NetXDuo applicative header file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_NETXDUO_H__
#define __APP_NETXDUO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "nx_api.h"

/* Private includes ----------------------------------------------------------*/
#include "nx_stm32_eth_driver.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "log.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
  uint32_t ip;
  uint32_t port;
} tcp_peer_t;

enum LINK_STATUS
{
  LINK_STATUS_NOT_AVAILABLE = 0,
  LINK_STATUS_DISCONNECTED,
  LINK_STATUS_AQUIRING_IP,    // Connected, but no IP yet
  LINK_STATUS_CONNECTED,      // Connected, has IP
};

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define APP_TCP_CONN_LIMIT 10
#define APP_TCP_PORT 6000
#define APP_TCP_DEFAULT_COMM_TIMEOUT 100
#define APP_TCP_LISTEN_QUEUE_SIZE 5
#define APP_TCP_THREAD_SIZE 1024
#define APP_TCP_THREAD_PRIO 10
#define APP_TCP_BUF_SIZE 256
#define APP_TCP_BUF_COUNT 2

/* USER CODE END EC */
/* The DEFAULT_PAYLOAD_SIZE should match with RxBuffLen configured via MX_ETH_Init */
#ifndef DEFAULT_PAYLOAD_SIZE
#define DEFAULT_PAYLOAD_SIZE      1536
#endif

#ifndef DEFAULT_ARP_CACHE_SIZE
#define DEFAULT_ARP_CACHE_SIZE    1024
#endif

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT MX_NetXDuo_Init(VOID *memory_ptr);

/* USER CODE BEGIN EFP */

// Returns the next connected TCP peer after `current_peer`, if any.
// Returns the very first connected TCP peer if `current_peer` is 0.0.0.0:0.
// Returns 0.0.0.0:0 if no more connected peers available.
tcp_peer_t netxduo_tcp_get_next_peer(tcp_peer_t current_peer);

uint32_t netxduo_tcp_rx(const tcp_peer_t *peer, uint8_t *dst, uint32_t request_len, uint32_t *return_len, uint32_t timeout_ms);
uint32_t netxduo_tcp_tx(const tcp_peer_t *peer, const uint8_t *src, uint32_t len, uint32_t timeout_ms);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define QUEUE_MAX_SIZE           512

#define DEFAULT_MEMORY_SIZE      1024
#define DEFAULT_PRIORITY         10
#define LINK_PRIORITY            11

/* USER CODE END PD */

#define NX_APP_DEFAULT_TIMEOUT               (10 * NX_IP_PERIODIC_RATE)

#define NX_APP_PACKET_POOL_SIZE              ((DEFAULT_PAYLOAD_SIZE + sizeof(NX_PACKET)) * 10)

#define NX_APP_THREAD_STACK_SIZE             2*1024

#define Nx_IP_INSTANCE_THREAD_SIZE           2*1024

#define NX_APP_THREAD_PRIORITY               10

#ifndef NX_APP_INSTANCE_PRIORITY
#define NX_APP_INSTANCE_PRIORITY             NX_APP_THREAD_PRIORITY
#endif

#define NX_APP_DEFAULT_IP_ADDRESS                   0

#define NX_APP_DEFAULT_NET_MASK                     0

/* USER CODE BEGIN 1 */

/* Can't set default IP and netmask in CubeMX when DHCP is activated. So use a workaround and undef them. */
#undef NX_APP_DEFAULT_IP_ADDRESS
#undef NX_APP_DEFAULT_NET_MASK

/* "No IP" settings. */
#define NX_APP_DEFAULT_IP_ADDRESS IP_ADDRESS(0, 0, 0, 0)
#define NX_APP_DEFAULT_NET_MASK   IP_ADDRESS(255, 255, 255, 0)

/* User default IP address */
#define USER_DEFAULT_IP_ADDRESS   IP_ADDRESS(192, 168, 1, 100)
#define USER_DEFAULT_NET_MASK     IP_ADDRESS(255, 255, 255, 0)

enum LINK_STATUS netxduo_get_link_status();

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_NETXDUO_H__ */
