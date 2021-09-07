/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
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
#include "wizchip_ether.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[128];    /**< TX buffer. */
static uint8_t       m_rx_buf[128];    /**< RX buffer. */
static const uint8_t m_length = 128;   /**< Transfer length. */

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    if (m_rx_buf[0] != 0) {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}

void  wizchip_select(void)
{
}

void  wizchip_deselect(void)
{
}

uint8_t spi_read_byte(void)
{
    memset(m_rx_buf, 0, m_length);
    memset(m_tx_buf, 0, m_length);
    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, 1));

    while (!spi_xfer_done) {
            __WFE();
    }
    return m_rx_buf[0];
}

void spi_write_byte(uint8_t wb)
{
    memset(m_rx_buf, 0, m_length);
    memset(m_tx_buf, 0, m_length);
    spi_xfer_done = false;
    m_tx_buf[0] = wb;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, 1));

    while (!spi_xfer_done) {
            __WFE();
    }
}

void spi_read_burst(uint8_t* pBuf, uint16_t len)
{
    memset(m_rx_buf, 0, m_length);
    memset(m_tx_buf, 0, m_length);
    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, len));

    while (!spi_xfer_done) {
            __WFE();
    }
    memcpy(pBuf, m_tx_buf, len);
    NRF_LOG_INFO("spi_read_burst %02X:%02X:%02X:%02X len=%d", *(pBuf), *(pBuf+1), *(pBuf+2), *(pBuf+3), len);
}

void spi_write_burst(uint8_t* pBuf, uint16_t len)
{
    memset(m_rx_buf, 0, m_length);
    memset(m_tx_buf, 0, m_length);
    spi_xfer_done = false;

    memcpy(m_tx_buf, pBuf, len);

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, len));

    while (!spi_xfer_done) {
            __WFE();
    }
    //NRF_LOG_INFO("spi_write_burst %02X:%02X:%02X:%02X len=%d", *(pBuf), *(pBuf+1), *(pBuf+2), *(pBuf+3), len);
}


int main(void)
{
    bsp_board_init(BSP_INIT_LEDS);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = NRF_GPIO_PIN_MAP(0,15);
    spi_config.miso_pin = NRF_GPIO_PIN_MAP(0,17);
    spi_config.mosi_pin = NRF_GPIO_PIN_MAP(0,16);
    spi_config.sck_pin  = NRF_GPIO_PIN_MAP(0,18);
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    NRF_LOG_INFO("SPI example started.");

    wizchip_connect();

    while (1) {
        NRF_LOG_FLUSH();

        bsp_board_led_invert(BSP_BOARD_LED_0);
        nrf_delay_ms(200);
    }
}
