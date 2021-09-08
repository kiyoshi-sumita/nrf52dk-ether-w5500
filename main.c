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

#ifndef DEBUG_SPI
static uint8_t       m_tx_buf[128];    /**< TX buffer. */
static uint8_t       m_rx_buf[128];    /**< RX buffer. */
static  uint8_t m_length = 128;   /**< Transfer length. */
#else
#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static  uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */
#endif

#define  WIZX_RESET     28

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    NRF_LOG_INFO(" Received:");
    NRF_LOG_HEXDUMP_INFO(m_rx_buf, 16);    
}

void wizchip_select(void)
{
    nrf_gpio_pin_clear(SPI_SS_PIN);
}

void wizchip_deselect(void)
{
    nrf_gpio_pin_set(SPI_SS_PIN);
}

uint8_t spi_read_byte(void)
{
    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 1, m_rx_buf, 1));

    while (!spi_xfer_done) {
            __WFE();
    }
    NRF_LOG_INFO("spi_read_byte %02X", m_rx_buf[0]);
    return m_rx_buf[0];
}

void spi_write_byte(uint8_t wb)
{
    spi_xfer_done = false;
    m_tx_buf[0] = wb;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 1, m_rx_buf, 1));

    while (!spi_xfer_done) {
            __WFE();
    }
}

void spi_read_burst(uint8_t* pBuf, uint16_t len)
{
    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, len, m_rx_buf, len));

    while (!spi_xfer_done) {
            __WFE();
    }
    memcpy(pBuf, m_tx_buf, len);
    NRF_LOG_INFO("spi_read_burst %02X:%02X:%02X:%02X len=%d", *(pBuf), *(pBuf+1), *(pBuf+2), *(pBuf+3), len);
}

void spi_write_burst(uint8_t* pBuf, uint16_t len)
{
    spi_xfer_done = false;

    memcpy(m_tx_buf, pBuf, len);

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, len, m_rx_buf, len));

    while (!spi_xfer_done) {
            __WFE();
    }
    memcpy(pBuf, m_rx_buf, len);
    NRF_LOG_INFO("spi_write_burst %02X:%02X:%02X:%02X len=%d", *(pBuf), *(pBuf+1), *(pBuf+2), *(pBuf+3), len);
}

int main(void)
{
    bsp_board_init(BSP_INIT_LEDS);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    nrf_gpio_cfg_output(WIZX_RESET);
    nrf_gpio_pin_set(WIZX_RESET);
    nrf_delay_ms(100);

    nrf_gpio_pin_clear(WIZX_RESET);
    nrf_delay_ms(5);
    nrf_gpio_pin_set(WIZX_RESET);
    nrf_delay_ms(10);

    nrf_gpio_cfg_output(SPI_SS_PIN);
    nrf_gpio_pin_set(SPI_SS_PIN);
    nrf_delay_ms(100);
    

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
//    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.ss_pin   = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    spi_config.frequency = NRF_DRV_SPI_FREQ_250K;
    spi_config.mode      = NRF_DRV_SPI_MODE_0;
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    NRF_LOG_INFO("SPI example started.");

    wizchip_connect();

    while (1) {

        // Reset rx buffer and transfer done flag
        m_tx_buf[0] = 0x00;
        m_tx_buf[1] = 0x00;
        m_tx_buf[2] = 0x01;
        m_tx_buf[3] = 0x00;
        m_length = 4;

        memset(m_rx_buf, 0, m_length);
        spi_xfer_done = false;

        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));

        while (!spi_xfer_done)
        {
            __WFE();
        }

        NRF_LOG_FLUSH();

        bsp_board_led_invert(BSP_BOARD_LED_0);
        nrf_delay_ms(200);
    }
}
