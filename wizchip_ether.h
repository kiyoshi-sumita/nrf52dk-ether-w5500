#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#ifndef WIZCHIP_ETHER_H
#define WIZCHIP_ETHER_H

#ifdef __cplusplus
extern "C"
{
#endif

#define _WIZCHIP_DEBUG_

void  wizchip_select(void);
void  wizchip_deselect(void);
uint8_t spi_read_byte(void);
void spi_write_byte(uint8_t wb);
void spi_read_burst(uint8_t* pBuf, uint16_t len);
void spi_write_burst(uint8_t* pBuf, uint16_t len);

typedef struct _network_clients
{
    int socket;
} network_client_t;

typedef struct {
    union S_un {
        uint32_t S_addr;
        struct {
            uint8_t s_b1,s_b2,s_b3,s_b4;
        } S_un_b;
    } S_un;
} InAddr_t;

bool wizchip_is_connected_now(void) ;
int wizchip_connect(void);
int wizchip_client_connect(network_client_t* client, const char *domain, const uint16_t port);
int wizchip_client_close(network_client_t client);
int wizchip_client_read(network_client_t client, char *data, const uint16_t length);
int wizchip_client_write(network_client_t client, const char *data, const uint16_t length, int flgs);

#ifdef __cplusplus
}
#endif

#endif /* WIZCHIP_ETHER_H */
