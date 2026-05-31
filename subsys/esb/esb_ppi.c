/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <hal/nrf_egu.h>
#include <hal/nrf_radio.h>
#include <hal/nrf_timer.h>

#include <nrfx_ppi.h>

#include <zephyr/logging/log.h>

#include "esb_peripherals.h"
#include "esb_ppi_api.h"

LOG_MODULE_DECLARE(esb, CONFIG_ESB_LOG_LEVEL);

static nrf_ppi_channel_t radio_addr_timer_stop;
static nrf_ppi_channel_t radio_event_capture;
static nrf_ppi_channel_t cc0_radio_enable;
static nrf_ppi_channel_t radio_rx_capture;
static nrf_ppi_channel_t txdis_rxen;
static nrf_ppi_channel_t cc1_radio_disable;
static nrf_ppi_channel_t addr_rssistart;
static nrf_ppi_channel_t rxdis_rssistop;
static nrf_ppi_channel_t rxready_rssistart;

static nrf_ppi_channel_group_t ppi_group;
static nrf_ppi_channel_group_t ppi_group2;

void pto_ppi_for_central_tx_set(void)
{
	/* 1. CC0 -> TXEN
	 * 2. TXDIS -> RXEN // group disable
	 * 3. CRCOK -> CC2, RSSISTOP
	 * 4. CC1 -> RXDIS
	 */
	uint32_t cc0_event =
		nrf_timer_event_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE0);
	uint32_t txen_task = nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_TXEN);
	uint32_t radio_disabled_event =
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	uint32_t rxen_task = nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_RXEN);
	uint32_t group_disable_task = nrf_ppi_task_group_disable_address_get(NRF_PPI, ppi_group);
	uint32_t crcok_event = nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_CRCOK);
	uint32_t cc2_capture_task =
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE2);
	uint32_t cc1_event =
		nrf_timer_event_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE1);
	uint32_t radio_disable_task = nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_DISABLE);

	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc0_radio_enable, cc0_event, txen_task);
	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, txdis_rxen, radio_disabled_event,
						rxen_task, group_disable_task);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_event_capture, crcok_event, cc2_capture_task);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc1_radio_disable, cc1_event, radio_disable_task);

	nrf_ppi_channel_include_in_group(NRF_PPI, txdis_rxen, ppi_group);

	uint32_t channels = BIT(cc0_radio_enable) | BIT(txdis_rxen) | BIT(radio_event_capture) |
			    BIT(cc1_radio_disable);

	nrf_ppi_channels_enable(NRF_PPI, channels);
}

void pto_ppi_for_central_tx_clear(void)
{
	uint32_t channels = BIT(cc0_radio_enable) | BIT(txdis_rxen) | BIT(radio_event_capture) |
			    BIT(cc1_radio_disable);

	nrf_ppi_channels_disable(NRF_PPI, channels);

	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc0_radio_enable, 0, 0);
	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, txdis_rxen, 0, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_event_capture, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc1_radio_disable, 0, 0);

	nrf_ppi_channel_remove_from_group(NRF_PPI, txdis_rxen, ppi_group);
}

void pto_ppi_for_peripheral_start_rx_desync_set(void)
{
	uint32_t cc0_event =
		nrf_timer_event_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE0);
	uint32_t radio_enable_task = nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_RXEN);
	uint32_t cc1_event =
		nrf_timer_event_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE1);
	uint32_t radio_disable_task = nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
	uint32_t radio_addr_event = nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS);
	uint32_t cc2_capture_task =
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE2);

	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc0_radio_enable, cc0_event, radio_enable_task);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc1_radio_disable, cc1_event, radio_disable_task);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_event_capture, radio_addr_event,
				       cc2_capture_task);

	uint32_t channels =
		BIT(radio_event_capture) | BIT(cc1_radio_disable) | BIT(cc0_radio_enable);

	nrf_ppi_channels_enable(NRF_PPI, channels);
}

void pto_ppi_for_peripheral_start_rx_desync_clear(void)
{
	uint32_t channels =
		BIT(radio_event_capture) | BIT(cc1_radio_disable) | BIT(cc0_radio_enable);

	nrf_ppi_channels_disable(NRF_PPI, channels);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc0_radio_enable, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc1_radio_disable, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_event_capture, 0, 0);
}

void pto_ppi_for_peripheral_prepare_rx_set(void)
{
	uint32_t cc0_event =
		nrf_timer_event_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE0);
	uint32_t radio_rxen_task = nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_RXEN);
	uint32_t cc1_event =
		nrf_timer_event_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE1);
	uint32_t radio_rxdis_task = nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
	uint32_t radio_addr_event = nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS);
	uint32_t cc2_capture_task =
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE2);

	uint32_t crcok_event = nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_CRCOK);
	uint32_t cc3_capture_task =
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE3);
	uint32_t group_disable_task = nrf_ppi_task_group_disable_address_get(NRF_PPI, ppi_group);

	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc0_radio_enable, cc0_event, radio_rxen_task);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc1_radio_disable, cc1_event, radio_rxdis_task);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_event_capture, radio_addr_event,
				       cc2_capture_task);
	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, radio_rx_capture, crcok_event,
						cc3_capture_task, group_disable_task);

	uint32_t channels = BIT(cc0_radio_enable) | BIT(cc1_radio_disable) |
			    BIT(radio_event_capture) | BIT(radio_rx_capture);

	nrf_ppi_channel_include_in_group(NRF_PPI, cc1_radio_disable, ppi_group);

	nrf_ppi_channels_enable(NRF_PPI, channels);
}

void pto_ppi_for_peripheral_prepare_rx_clear(void)
{
	uint32_t channels = BIT(cc0_radio_enable) | BIT(cc1_radio_disable) |
			    BIT(radio_event_capture) | BIT(radio_rx_capture);

	nrf_ppi_channels_disable(NRF_PPI, channels);

	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc0_radio_enable, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc1_radio_disable, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_event_capture, 0, 0);
	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, radio_rx_capture, 0, 0, 0);

	nrf_ppi_channel_remove_from_group(NRF_PPI, cc1_radio_disable, ppi_group);
}

void esb_ppi_for_fem_set(void)
{
	uint32_t egu_event = nrf_egu_event_address_get(ESB_EGU, ESB_EGU_EVENT);
	uint32_t timer_task =
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_START);

	nrf_ppi_channel_endpoint_setup(NRF_PPI, txdis_rxen, egu_event, timer_task);
	nrf_ppi_channel_enable(NRF_PPI, txdis_rxen);
}

void esb_ppi_for_fem_clear(void)
{
	nrf_ppi_channel_disable(NRF_PPI, txdis_rxen);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, txdis_rxen, 0, 0);
}

int esb_ppi_init(void)
{
	nrfx_err_t err;

	err = nrfx_ppi_channel_alloc(&radio_addr_timer_stop);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&radio_event_capture);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&cc0_radio_enable);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&radio_rx_capture);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&txdis_rxen);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&cc1_radio_disable);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&addr_rssistart);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&rxdis_rssistop);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&rxready_rssistart);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_group_alloc(&ppi_group);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("gppi_group_alloc failed with: %d\n", err);
		return -ENODEV;
	}

	err = nrfx_ppi_group_alloc(&ppi_group2);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("gppi_group2_alloc failed with: %d\n", err);
		return -ENODEV;
	}

	return 0;

error:
	LOG_ERR("gppi_channel_alloc failed with: %d\n", err);
	return -ENODEV;
}

uint32_t esb_ppi_radio_disabled_get(void)
{
	return nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
}

void esb_ppi_disable_all(void)
{
	uint32_t channels_mask = BIT(radio_rx_capture) | BIT(cc1_radio_disable) | BIT(txdis_rxen) |
				 BIT(radio_addr_timer_stop) | BIT(radio_event_capture) |
				 BIT(addr_rssistart) | BIT(cc0_radio_enable) |
				 BIT(rxready_rssistart);

	nrf_ppi_channels_disable(NRF_PPI, channels_mask);
}

void esb_ppi_deinit(void)
{
	nrfx_err_t err;

	err = nrfx_ppi_channel_free(radio_addr_timer_stop);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(radio_event_capture);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(cc0_radio_enable);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(radio_rx_capture);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(txdis_rxen);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(cc1_radio_disable);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(addr_rssistart);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(rxdis_rssistop);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(rxready_rssistart);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_group_free(ppi_group);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_group_free(ppi_group2);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	return;

/* Should not happen. */
error:
	__ASSERT(false, "Failed to free PPI resources");
}
