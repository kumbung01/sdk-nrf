/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <hal/nrf_egu.h>
#include <hal/nrf_radio.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_rtc.h>

#include <nrfx_ppi.h>

#include <zephyr/logging/log.h>

#include "esb_peripherals.h"
#include "esb_ppi_api.h"

LOG_MODULE_DECLARE(esb, CONFIG_ESB_LOG_LEVEL);

static nrf_ppi_channel_t radio_addr_timer_stop;
static nrf_ppi_channel_t crcok_cc2_rssistop;
static nrf_ppi_channel_t cc0_radio_enable;
static nrf_ppi_channel_t radio_rx_capture;
static nrf_ppi_channel_t txdis_rxen;
static nrf_ppi_channel_t cc1_radio_disable;
static nrf_ppi_channel_t addr_rssistart;
static nrf_ppi_channel_t txdis_g2en;
static nrf_ppi_channel_t radio_timer_disable;

static nrf_ppi_channel_group_t ppi_group;
static nrf_ppi_channel_group_t ppi_group2;

void pto_ppi_for_central_tx_set(bool setup)
{
	// cc0 -> txen
	uint32_t cc0_event =
		nrf_rtc_event_address_get(ESB_NRF_RTC_INSTANCE, NRF_RTC_EVENT_COMPARE_0);
	uint32_t txen_task = nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_TXEN);
	uint32_t timer_start_task =
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_START);

	// group1:  txdis -> rxen, group1 dis, group2 en
	uint32_t radio_disabled_event =
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	uint32_t rxen_task = nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_RXEN);
	uint32_t group1_disable_task = nrf_ppi_task_group_disable_address_get(NRF_PPI, ppi_group);
	uint32_t group2_enable_task = nrf_ppi_task_group_enable_address_get(NRF_PPI, ppi_group2);

	// group2: addr -> rssi en
	uint32_t addr_event = nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS);
	uint32_t rssistart_task = nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_RSSISTART);

	// group2: crcok -> cc2 capture, rssi dis
	uint32_t crcok_event = nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_CRCOK);
	uint32_t cc2_capture_task =
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE2);
	uint32_t rssistop_task = nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_RSSISTOP);

	// cc1 -> rxdis
	uint32_t cc1_event =
		nrf_timer_event_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE1);
	uint32_t radio_disable_task = nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_DISABLE);

	if (setup) {
		nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, cc0_radio_enable, cc0_event,
							txen_task, timer_start_task);
		nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, txdis_rxen, radio_disabled_event,
							rxen_task, group1_disable_task);
		nrf_ppi_channel_endpoint_setup(NRF_PPI, txdis_g2en, radio_disabled_event,
					       group2_enable_task);
		nrf_ppi_channel_endpoint_setup(NRF_PPI, addr_rssistart, addr_event, rssistart_task);
		nrf_ppi_channel_endpoint_setup(NRF_PPI, crcok_cc2_rssistop, crcok_event,
					       rssistop_task);
		nrf_ppi_channel_endpoint_setup(NRF_PPI, cc1_radio_disable, cc1_event,
					       radio_disable_task);

		nrf_ppi_channels_include_in_group(NRF_PPI, BIT(txdis_rxen) | BIT(txdis_g2en),
						  ppi_group);
		nrf_ppi_channels_include_in_group(
			NRF_PPI, BIT(addr_rssistart) | BIT(crcok_cc2_rssistop), ppi_group2);
	}

	uint32_t channels = BIT(cc0_radio_enable) | BIT(txdis_rxen) | BIT(txdis_g2en) |
			    BIT(addr_rssistart) | BIT(crcok_cc2_rssistop) | BIT(cc1_radio_disable);

	nrf_ppi_group_enable(NRF_PPI, ppi_group);
	nrf_ppi_group_disable(NRF_PPI, ppi_group2);

	nrf_ppi_channels_enable(NRF_PPI, channels);
}

void pto_ppi_for_central_tx_clear(bool setup)
{
	uint32_t channels = BIT(cc0_radio_enable) | BIT(txdis_rxen) | BIT(txdis_g2en) |
			    BIT(addr_rssistart) | BIT(crcok_cc2_rssistop) | BIT(cc1_radio_disable);

	nrf_ppi_channels_disable(NRF_PPI, channels);

	if (!setup) {
		return;
	}

	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc0_radio_enable, 0, 0);
	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, txdis_rxen, 0, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, txdis_g2en, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, addr_rssistart, 0, 0);
	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, crcok_cc2_rssistop, 0, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc1_radio_disable, 0, 0);

	nrf_ppi_channels_remove_from_group(NRF_PPI, BIT(txdis_rxen) | BIT(txdis_g2en), ppi_group);
	nrf_ppi_channels_remove_from_group(NRF_PPI, BIT(addr_rssistart) | BIT(crcok_cc2_rssistop),
					   ppi_group2);
}

void pto_ppi_for_peripheral_start_desync_set(bool setup)
{
	uint32_t cc0_event =
		nrf_rtc_event_address_get(ESB_NRF_RTC_INSTANCE, NRF_RTC_EVENT_COMPARE_0);
	uint32_t radio_enable_task = nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_RXEN);
	uint32_t timer_start_task =
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_START);
	uint32_t cc1_event =
		nrf_timer_event_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE1);
	uint32_t radio_disable_task = nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
	uint32_t radio_addr_event = nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS);
	uint32_t cc2_capture_task =
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE2);
	uint32_t radio_disabled_event =
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	uint32_t timer_shutdown_task =
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);

	if (setup) {
		nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, cc0_radio_enable, cc0_event,
							radio_enable_task, timer_start_task);
		nrf_ppi_channel_endpoint_setup(NRF_PPI, cc1_radio_disable, cc1_event,
					       radio_disable_task);
		nrf_ppi_channel_endpoint_setup(NRF_PPI, crcok_cc2_rssistop, radio_addr_event,
					       cc2_capture_task);
		nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_timer_disable, radio_disabled_event,
					       timer_shutdown_task);
	}

	uint32_t channels = BIT(crcok_cc2_rssistop) | BIT(cc1_radio_disable) |
			    BIT(cc0_radio_enable) | BIT(radio_timer_disable);

	nrf_ppi_channels_enable(NRF_PPI, channels);
}

void pto_ppi_for_peripheral_start_desync_clear(bool setup)
{
	uint32_t channels = BIT(crcok_cc2_rssistop) | BIT(cc1_radio_disable) |
			    BIT(cc0_radio_enable) | BIT(radio_timer_disable);

	nrf_ppi_channels_disable(NRF_PPI, channels);

	if (!setup) {
		return;
	}

	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc0_radio_enable, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc1_radio_disable, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, crcok_cc2_rssistop, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_timer_disable, 0, 0);
}

void pto_ppi_for_peripheral_prepare_rx_set(bool setup)
{
	uint32_t rtc_cc0_event =
		nrf_rtc_event_address_get(ESB_NRF_RTC_INSTANCE, NRF_RTC_EVENT_COMPARE_0);
	uint32_t timer_start_task =
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_START);

	uint32_t timer_cc0_event =
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

	uint32_t radio_disabled_event =
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	uint32_t timer_shutdown_task =
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);

	if (setup) {
		nrf_ppi_channel_endpoint_setup(NRF_PPI, cc0_radio_enable, rtc_cc0_event,
					       timer_start_task);
		nrf_ppi_channel_endpoint_setup(NRF_PPI, txdis_rxen, timer_cc0_event,
					       radio_rxen_task);
		nrf_ppi_channel_endpoint_setup(NRF_PPI, cc1_radio_disable, cc1_event,
					       radio_rxdis_task);
		nrf_ppi_channel_endpoint_setup(NRF_PPI, crcok_cc2_rssistop, radio_addr_event,
					       cc2_capture_task);
		nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_timer_disable, radio_disabled_event,
					       timer_shutdown_task);
	}

	uint32_t channels = BIT(cc0_radio_enable) | BIT(txdis_rxen) | BIT(cc1_radio_disable) |
			    BIT(crcok_cc2_rssistop) | BIT(radio_timer_disable);

	nrf_ppi_channels_enable(NRF_PPI, channels);
}

void pto_ppi_for_peripheral_prepare_rx_clear(bool setup)
{
	uint32_t channels = BIT(cc0_radio_enable) | BIT(txdis_rxen) | BIT(cc1_radio_disable) |
			    BIT(crcok_cc2_rssistop) | BIT(radio_timer_disable);

	nrf_ppi_channels_disable(NRF_PPI, channels);

	if (!setup) {
		return;
	}

	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc0_radio_enable, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, txdis_rxen, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc1_radio_disable, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, crcok_cc2_rssistop, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_timer_disable, 0, 0);

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

	err = nrfx_ppi_channel_alloc(&crcok_cc2_rssistop);
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

	err = nrfx_ppi_channel_alloc(&txdis_g2en);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&radio_timer_disable);
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
				 BIT(radio_addr_timer_stop) | BIT(crcok_cc2_rssistop) |
				 BIT(addr_rssistart) | BIT(cc0_radio_enable) |
				 BIT(radio_timer_disable);

	nrf_ppi_channels_disable(NRF_PPI, channels_mask);
}

void esb_ppi_deinit(void)
{
	nrfx_err_t err;

	err = nrfx_ppi_channel_free(radio_addr_timer_stop);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(crcok_cc2_rssistop);
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

	err = nrfx_ppi_channel_free(txdis_g2en);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(radio_timer_disable);
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
