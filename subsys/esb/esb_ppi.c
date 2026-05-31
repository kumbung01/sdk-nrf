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

static nrf_ppi_channel_t radio_address_timer_stop;
<<<<<<< Updated upstream
static nrf_ppi_channel_t timer_compare0_radio_disable;
static nrf_ppi_channel_t timer_compare1_radio_txen;
static nrf_ppi_channel_t egu_ramp_up;
static nrf_ppi_channel_t egu_timer_start;
static nrf_ppi_channel_t disabled_egu;
static nrf_ppi_channel_t radio_end_timer_start;
=======
static nrf_ppi_channel_t radio_event_capture;
static nrf_ppi_channel_t cc0_radio_enable;
static nrf_ppi_channel_t radio_rx_capture;
static nrf_ppi_channel_t txdis_rxen;
static nrf_ppi_channel_t cc1_radio_disable;
static nrf_ppi_channel_t addr_rssistart;
static nrf_ppi_channel_t rxdis_rssistop;
static nrf_ppi_channel_t rxready_rssistart;
>>>>>>> Stashed changes

static nrf_ppi_channel_group_t ppi_group;
static nrf_ppi_channel_group_t ppi_group2;

<<<<<<< Updated upstream
=======
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

	uint32_t rxready_event = nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_RXREADY);
	uint32_t group_enable_task = nrf_ppi_task_group_enable_address_get(NRF_PPI, ppi_group2);
	uint32_t addr_event = nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS);
	uint32_t radio_rssistart_task =
		nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_RSSISTART);
	uint32_t radio_rssistop_task =
		nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_RSSISTOP);

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

>>>>>>> Stashed changes
void esb_ppi_for_fem_set(void)
{
	uint32_t egu_event = nrf_egu_event_address_get(ESB_EGU, ESB_EGU_EVENT);
	uint32_t timer_task = nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE,
							 NRF_TIMER_TASK_START);

	nrf_ppi_channel_endpoint_setup(NRF_PPI, txdis_rxen, egu_event, timer_task);
	nrf_ppi_channel_enable(NRF_PPI, txdis_rxen);
}

void esb_ppi_for_fem_clear(void)
{
<<<<<<< Updated upstream
	nrf_ppi_channel_disable(NRF_PPI, egu_timer_start);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, egu_timer_start, 0, 0);
}

void esb_ppi_for_txrx_set(bool rx, bool timer_start)
{
	uint32_t channels_mask;
	uint32_t egu_event = nrf_egu_event_address_get(ESB_EGU, ESB_EGU_EVENT);
	uint32_t egu_task = nrf_egu_task_address_get(ESB_EGU, ESB_EGU_TASK);
	uint32_t group_disable_task =
			nrf_ppi_task_group_disable_address_get(NRF_PPI, ramp_up_ppi_group);
	uint32_t radio_en_task = nrf_radio_task_address_get(NRF_RADIO,
						rx ? NRF_RADIO_TASK_RXEN : NRF_RADIO_TASK_TXEN);
	uint32_t radio_disabled_event = nrf_radio_event_address_get(NRF_RADIO,
						NRF_RADIO_EVENT_DISABLED);
	uint32_t timer_task = nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE,
							 NRF_TIMER_TASK_START);

	nrf_egu_event_clear(ESB_EGU, ESB_EGU_EVENT);
	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, egu_ramp_up, egu_event, radio_en_task,
						group_disable_task);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, disabled_egu, radio_disabled_event, egu_task);

	channels_mask = BIT(egu_ramp_up) | BIT(disabled_egu);

	if (timer_start) {
		nrf_ppi_channel_endpoint_setup(NRF_PPI, egu_timer_start, egu_event, timer_task);
		channels_mask |= BIT(egu_timer_start);
	}

	nrf_ppi_channel_include_in_group(NRF_PPI, egu_ramp_up, ramp_up_ppi_group);
	nrf_ppi_channels_enable(NRF_PPI, channels_mask);
}

void esb_ppi_for_txrx_clear(bool rx, bool timer_start)
{
	ARG_UNUSED(rx);

	uint32_t channels_mask = (BIT(egu_ramp_up) | BIT(disabled_egu));


	if (timer_start) {
		channels_mask |= BIT(egu_timer_start);
	}

	nrf_ppi_channels_disable(NRF_PPI, channels_mask);

	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, egu_ramp_up, 0, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, disabled_egu, 0, 0);

	if (timer_start) {
		nrf_ppi_channel_endpoint_setup(NRF_PPI, egu_timer_start, 0, 0);
	}

	nrf_ppi_channel_remove_from_group(NRF_PPI, egu_ramp_up, ramp_up_ppi_group);
}

void esb_ppi_for_retransmission_set(void)
{
	uint32_t channels_mask;

	uint32_t egu_task = nrf_egu_task_address_get(ESB_EGU, ESB_EGU_TASK);
	uint32_t radio_disabled_event = nrf_radio_event_address_get(NRF_RADIO,
						NRF_RADIO_EVENT_DISABLED);
	nrf_egu_event_clear(ESB_EGU, ESB_EGU_EVENT);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, disabled_egu, radio_disabled_event, egu_task);

	nrf_ppi_channel_endpoint_setup(NRF_PPI, timer_compare1_radio_txen,
		nrf_timer_event_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE1),
		nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_TXEN));

	channels_mask = BIT(disabled_egu) | BIT(timer_compare1_radio_txen);

	nrf_ppi_channels_enable(NRF_PPI, channels_mask);
}

void esb_ppi_for_retransmission_clear(void)
{
	uint32_t channels_mask;

	channels_mask = (BIT(disabled_egu) |
			 BIT(timer_compare1_radio_txen));

	nrf_ppi_channels_disable(NRF_PPI, channels_mask);

	nrf_ppi_channel_endpoint_setup(NRF_PPI, disabled_egu, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, timer_compare1_radio_txen, 0, 0);
}

void esb_ppi_for_wait_for_ack_set(void)
{
	uint32_t ppi_channels_mask;

	nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_address_timer_stop,
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS),
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_SHUTDOWN));

	nrf_ppi_channel_endpoint_setup(NRF_PPI, timer_compare0_radio_disable,
		nrf_timer_event_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE0),
		nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_DISABLE));

	ppi_channels_mask = (BIT(radio_address_timer_stop) |
			     BIT(timer_compare0_radio_disable));

	nrf_ppi_channels_enable(NRF_PPI, ppi_channels_mask);
}

void esb_ppi_for_wait_for_ack_clear(void)
{
	uint32_t ppi_channels_mask;

	ppi_channels_mask = (BIT(radio_address_timer_stop) |
			     BIT(timer_compare0_radio_disable));

	nrf_ppi_channels_disable(NRF_PPI, ppi_channels_mask);

	nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_address_timer_stop, 0, 0);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, timer_compare0_radio_disable, 0, 0);
}

void esb_ppi_for_wait_for_rx_set(void)
{
	uint32_t ppi_channels_mask;

	nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_end_timer_start,
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_END),
		nrf_timer_task_address_get(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_START));

	ppi_channels_mask = (BIT(radio_end_timer_start));

	nrf_ppi_channels_enable(NRF_PPI, ppi_channels_mask);
}

void esb_ppi_for_wait_for_rx_clear(void)
{
	uint32_t ppi_channels_mask;

	ppi_channels_mask = (BIT(radio_end_timer_start));

	nrf_ppi_channels_disable(NRF_PPI, ppi_channels_mask);

	nrf_ppi_channel_endpoint_setup(NRF_PPI, radio_end_timer_start, 0, 0);
=======
	nrf_ppi_channel_disable(NRF_PPI, txdis_rxen);
	nrf_ppi_channel_endpoint_setup(NRF_PPI, txdis_rxen, 0, 0);
>>>>>>> Stashed changes
}

int esb_ppi_init(void)
{
	nrfx_err_t err;

	err = nrfx_ppi_channel_alloc(&egu_ramp_up);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&disabled_egu);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&txdis_rxen);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&radio_address_timer_stop);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&timer_compare0_radio_disable);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_alloc(&timer_compare1_radio_txen);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

<<<<<<< Updated upstream
	if (IS_ENABLED(CONFIG_ESB_NEVER_DISABLE_TX)) {
		err = nrfx_ppi_channel_alloc(&radio_end_timer_start);
		if (err != NRFX_SUCCESS) {
			goto error;
		}
=======
	err = nrfx_ppi_channel_alloc(&rxdis_rssistop);
	if (err != NRFX_SUCCESS) {
		goto error;
>>>>>>> Stashed changes
	}

	err = nrfx_ppi_channel_alloc(&addr_rssistart);
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
<<<<<<< Updated upstream
	uint32_t channels_mask = (BIT(egu_ramp_up) |
				  BIT(disabled_egu) |
				  BIT(egu_timer_start) |
				  BIT(radio_address_timer_stop) |
				  BIT(timer_compare0_radio_disable) |
				  BIT(radio_end_timer_start) |
				  (IS_ENABLED(CONFIG_ESB_NEVER_DISABLE_TX) ?
					BIT(timer_compare1_radio_txen) : 0));
=======
	uint32_t channels_mask = BIT(radio_rx_capture) | BIT(cc1_radio_disable) | BIT(txdis_rxen) |
				 BIT(radio_address_timer_stop) | BIT(radio_event_capture) |
				 BIT(addr_rssistart) | BIT(cc0_radio_enable);
>>>>>>> Stashed changes

	nrf_ppi_channels_disable(NRF_PPI, channels_mask);
}

void esb_ppi_deinit(void)
{
	nrfx_err_t err;

	err = nrfx_ppi_channel_free(egu_ramp_up);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(disabled_egu);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(txdis_rxen);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(radio_address_timer_stop);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(timer_compare0_radio_disable);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppi_channel_free(timer_compare1_radio_txen);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

<<<<<<< Updated upstream
	if (IS_ENABLED(CONFIG_ESB_NEVER_DISABLE_TX)) {
		err = nrfx_ppi_channel_free(radio_end_timer_start);
		if (err != NRFX_SUCCESS) {
			goto error;
		}
=======
	err = nrfx_ppi_channel_free(addr_rssistart);
	if (err != NRFX_SUCCESS) {
		goto error;
>>>>>>> Stashed changes
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
