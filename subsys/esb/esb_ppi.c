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

static nrf_ppi_channel_t cc0_start;
static nrf_ppi_channel_t cc0_trigger;
static nrf_ppi_channel_t radio_dis_en;
static nrf_ppi_channel_t radio_dis_trigger;
static nrf_ppi_channel_t addr_capture;
static nrf_ppi_channel_t cc1_end;
static nrf_ppi_channel_t egu_trigger;

#define CHANNELS_MASK                                                                              \
	(BIT(cc0_start) | BIT(cc0_trigger) | BIT(radio_dis_en) | BIT(radio_dis_trigger) |          \
	 BIT(addr_capture) | BIT(cc1_end) | BIT(egu_trigger))

static nrf_ppi_channel_group_t ppi_group;

void ppi_for_txrx_set(bool is_tx)
{
	nrf_radio_task_t radio_task_first = is_tx ? NRF_RADIO_TASK_TXEN : NRF_RADIO_TASK_RXEN;
	nrf_radio_task_t radio_task_second = is_tx ? NRF_RADIO_TASK_RXEN : NRF_RADIO_TASK_TXEN;

	// rtc cc0 -> radio task 1 enable, timer start
	uint32_t rtc_cc0_event =
		nrf_rtc_event_address_get(ESB_NRF_RTC_INSTANCE, NRF_RTC_EVENT_COMPARE_0);
	uint32_t timer_start_task =
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_START);
	uint32_t radio_endpoint_first = nrf_radio_task_address_get(NRF_RADIO, radio_task_first);
	uint32_t group1_enable_task = nrf_ppi_task_group_enable_address_get(NRF_PPI, ppi_group);

	// radio task 1 disable -> radio task 2 enable
	uint32_t radio_disabled_event =
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	uint32_t radio_endpoint_second = nrf_radio_task_address_get(NRF_RADIO, radio_task_second);

	// addr event -> timer capture
	uint32_t radio_addr_event = nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS);
	uint32_t timer_capture_task =
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_CAPTURE2);

	// timer cc1 -> egu trigger, gruop1 disable
	// bypass egu for ensuring that radio disable triggers after group1 is disabled
	uint32_t timer_cc1_event =
		nrf_timer_event_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE1);
	uint32_t egu_trigger_task = nrf_egu_task_address_get(ESB_EGU, ESB_EGU_TASK);
	uint32_t group1_disable_task = nrf_ppi_task_group_disable_address_get(NRF_PPI, ppi_group);

	// egu trigger -> radio disable, timer shutdown
	uint32_t egu_triggered_event = nrf_egu_event_address_get(ESB_EGU, ESB_EGU_EVENT);
	uint32_t radio_disable_task = nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
	uint32_t timer_shutdown_task =
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN);

	uint32_t channels = BIT(cc0_start) | BIT(cc0_trigger) | BIT(radio_dis_en) |
			    BIT(addr_capture) | BIT(cc1_end) | BIT(egu_trigger);

	uint32_t group1_channels = BIT(radio_dis_en) | BIT(addr_capture);

	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, cc0_start, rtc_cc0_event,
						radio_endpoint_first, timer_start_task);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc0_trigger, rtc_cc0_event, group1_enable_task);
	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, radio_dis_en, radio_disabled_event,
						radio_endpoint_second, group1_disable_task);

	nrf_ppi_channel_endpoint_setup(NRF_PPI, addr_capture, radio_addr_event, timer_capture_task);
	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, cc1_end, timer_cc1_event, egu_trigger_task,
						group1_disable_task);
	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, egu_trigger, egu_triggered_event,
						radio_disable_task, timer_shutdown_task);

	// if is_rx, then shutdown timer on radio disabled event
	if (!is_tx) {
		channels |= BIT(radio_dis_trigger);
		nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_dis_trigger, radio_disabled_event,
					       timer_shutdown_task);
	}

	// nrf_egu_event_clear(ESB_EGU, ESB_EGU_EVENT);
	nrf_ppi_channels_include_in_group(NRF_PPI, group1_channels, ppi_group);
	nrf_ppi_channels_enable(NRF_PPI, channels);
}

void ppi_for_txrx_clear(void)
{
	uint32_t group1_channels = BIT(radio_dis_en) | BIT(addr_capture);
	uint32_t channels = BIT(cc0_start) | BIT(cc0_trigger) | BIT(radio_dis_en) |
			    BIT(addr_capture) | BIT(cc1_end) | BIT(egu_trigger);

	nrf_ppi_channels_disable(NRF_PPI, channels);

	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc0_start, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_dis_en, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_dis_trigger, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, addr_capture, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, cc1_end, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, egu_trigger, 0, 0);

	nrf_ppi_channels_remove_from_group(NRF_PPI, group1_channels, ppi_group);
}

void esb_ppi_for_fem_set(void)
{
	// uint32_t egu_event = nrf_egu_event_address_get(ESB_EGU, ESB_EGU_EVENT);
	// uint32_t timer_task =
	// 	nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_START);

	// nrf_ppi_channel_endpoint_setup(NRF_PPI, txdis_rxen, egu_event, timer_task);
	// nrf_ppi_channel_enable(NRF_PPI, txdis_rxen);
}

void esb_ppi_for_fem_clear(void)
{
	// nrf_ppi_channel_disable(NRF_PPI, txdis_rxen);
	// nrf_ppi_channel_endpoint_setup(NRF_PPI, txdis_rxen, 0, 0);
}

int esb_ppi_init(void)
{
	nrfx_err_t err;

	err = nrfx_ppi_channel_alloc(&cc0_start);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&cc0_trigger);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&radio_dis_en);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&radio_dis_trigger);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&addr_capture);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&cc1_end);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&egu_trigger);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_group_alloc(&ppi_group);
	if (err != NRFX_SUCCESS) {
		goto error;
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
	nrf_ppi_channels_disable(NRF_PPI, CHANNELS_MASK);
}

void esb_ppi_deinit(void)
{
	nrfx_err_t err;

	err = nrfx_ppi_channel_free(cc0_start);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(cc0_trigger);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(radio_dis_en);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(radio_dis_trigger);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(addr_capture);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(cc1_end);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(egu_trigger);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_group_free(ppi_group);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	return;

/* Should not happen. */
error:
	__ASSERT(false, "Failed to free PPI resources");
}
