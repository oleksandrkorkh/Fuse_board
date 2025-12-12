#pragma once

#include <optional>
#include <app_netxduo.h>
#include <stdint.h>
#include <nanomodbus.h>

#ifdef __cplusplus
extern "C"
{
#endif

namespace comms
{

void initialize_comms();

}
#ifdef __cplusplus
}
#endif
