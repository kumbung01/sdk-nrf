/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <errno.h>
#include <nrf.h>
#include <esb.h>
#include <stddef.h>
#include <string.h>
#include <nrf_erratas.h>

#include <hal/nrf_radio.h>
#include <hal/nrf_timer.h>
#include <nrfx_timer.h>

#ifdef NRF53_SERIES
#include "hal/nrf_vreqctrl.h"
#endif /* NRF53_SERIES */

#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/random/random.h>

#include <mpsl_fem_protocol_api.h>

#include "esb_peripherals.h"
#include "esb_ppi_api.h"
#include "tx_api.h"

LOG_MODULE_REGISTER(esb, CONFIG_ESB_LOG_LEVEL);

/* Constants */

/* 2 Mb RX wait for acknowledgment time-out value.
 * Smallest reliable value: 160.
 */
#define RX_ACK_TIMEOUT_US_2MBPS	    160
/* 1 Mb RX wait for acknowledgment time-out value. */
#define RX_ACK_TIMEOUT_US_1MBPS	    300
/* 250 Kb RX wait for acknowledgment time-out value. */
#define RX_ACK_TIMEOUT_US_250KBPS   300
/* 1 Mb RX wait for acknowledgment time-out (combined with BLE). */
#define RX_ACK_TIMEOUT_US_1MBPS_BLE 300
/* 4 Mb RX wait for acknowledgment time-out value. */
#define RX_ACK_TIMEOUT_US_4MBPS	    160

/* Minimum retransmit time */
#define RETRANSMIT_DELAY_MIN 435

/* Radio Tx ramp-up time in microseconds. */
#define TX_RAMP_UP_TIME_US 129

/* Radio Rx fast ramp-up time in microseconds. */
#define TX_FAST_RAMP_UP_TIME_US 40

/* Radio Rx ramp-up time in microseconds. */
#define RX_RAMP_UP_TIME_US 124

/* Interrupt flags */
/* Interrupt mask value for TX success. */
#define INT_TX_SUCCESS_MSK	 BIT(0)
/* Interrupt mask value for TX failure. */
#define INT_TX_FAILED_MSK	 BIT(1)
/* Interrupt mask value for RX_DR. */
#define INT_RX_DATA_RECEIVED_MSK BIT(2)
#define INT_PERIPHERAL_SYNC_MSK	 BIT(3)

/* Mask value to signal updating BASE0 radio address. */
#define ADDR_UPDATE_MASK_BASE0	BIT(0)
/* Mask value to signal updating BASE1 radio address. */
#define ADDR_UPDATE_MASK_BASE1	BIT(1)
/* Mask value to signal updating radio prefixes. */
#define ADDR_UPDATE_MASK_PREFIX BIT(2)

/* Radio address event latency in microseconds. */
#define ADDR_EVENT_LATENCY_US (13)

/* The maximum value for PID. */
#define PID_MAX 3

#define BIT_MASK_UINT_8(x) (0xFF >> (8 - (x)))

/* Radio base frequency. */
#define RADIO_BASE_FREQUENCY 2400UL

/* NRF5340 Radio high voltage gain. */
#define NRF5340_HIGH_VOLTAGE_GAIN 3

/* Fast switching is available for the nRF54H20 SoC.
 * The nRF54H20 is a non-RSSISTOP device.
 */
#if defined(RADIO_SHORTS_DISABLED_RSSISTOP_Msk)
#define RADIO_SHORTS_COMMON                                                                        \
	(NRF_RADIO_SHORT_READY_START_MASK | ESB_SHORT_DISABLE_MASK |                               \
	 NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK | NRF_RADIO_SHORT_DISABLED_RSSISTOP_MASK)
#else
/* Devices without RSSISTOP task will stop RSSI measurement after specific period. */
#define RADIO_SHORTS_FAST_SWITCHING_NO_RSSISTOP                                                    \
	(NRF_RADIO_SHORT_READY_START_MASK | NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK)
#define RADIO_SHORTS_NO_FAST_SWITCHING_NO_RSSISTOP                                                 \
	(NRF_RADIO_SHORT_READY_START_MASK | ESB_SHORT_DISABLE_MASK |                               \
	 NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK)

#define RADIO_SHORTS_COMMON                                                                        \
	(IS_ENABLED(CONFIG_ESB_FAST_SWITCHING) ? RADIO_SHORTS_FAST_SWITCHING_NO_RSSISTOP           \
					       : RADIO_SHORTS_NO_FAST_SWITCHING_NO_RSSISTOP)
#endif /* !defined(RADIO_SHORTS_DISABLED_RSSISTOP_Msk) */

#define RADIO_SHORTS_BASIC (NRF_RADIO_SHORT_READY_START_MASK | ESB_SHORT_DISABLE_MASK)

/* Flag for changing radio channel. */
#define RF_CHANNEL_UPDATE_FLAG 0

/* Empty trim value */
#define TRIM_VALUE_EMPTY 0xFFFFFFFF

#define ERRATA_216_PRESENT DT_NODE_HAS_STATUS(DT_NODELABEL(cpurad_cpusys_errata216_mboxes), okay)
#define ERRATA_216_RADIO_ENABLE_DELAY_US  40
#define ERRATA_216_MIN_TIME_TO_DISABLE_US 100

/* Internal Enhanced ShockBurst module state. */
enum esb_state {
	ESB_STATE_IDLE, /* Idle. */
	ESB_STATE_CENTRAL_TX,

	ESB_STATE_PERIPHERAL_DESYNC,
	ESB_STATE_PERIPHERAL_RX,
	ESB_STATE_PERIPHERAL_RX_READY,
	ESB_STATE_PERIPHERAL_TX_ACK,
};

/* Pipe info PID and CRC and acknowledgment payload. */
struct pipe_info {
	uint16_t crc; /* CRC of the last received packet.
		       * Used to detect retransmits.
		       */
	bool sn;
	bool nesn;
	uint8_t tx_try;
	bool synced;
	struct esb_radio_pdu *pdu;
	// bool ack_payload; /* State of the transmission of ACK payloads. */
};

/* First-in, first-out queue of received payloads. */
struct payload_rx_fifo {
	/* Payload queue */
	struct esb_payload *payload[CONFIG_ESB_RX_FIFO_SIZE];

	uint32_t back;	/* Back of the queue (last in). */
	uint32_t front; /* Front of queue (first out). */
	uint32_t count; /* Number of elements in the queue. */
};

/* Fixed radio PDU header definition. */
struct esb_radio_fixed_pdu {
	/* Packet ID of the last received packet. Used to detect retransmits. */
	uint8_t pid: 2;
	uint8_t rfu: 6;
	uint8_t rfu1;
} __packed;

/* Dynamic length radio PDU header definition. */
struct esb_radio_dynamic_pdu {
	/* Payload length. */
#if CONFIG_ESB_MAX_PAYLOAD_LENGTH > 63
	uint8_t length;
#else
	uint8_t length: 6;
	uint8_t rfu0: 2;
#endif /* CONFIG_ESB_MAX_PAYLOAD_LENGTH > 63 */

	/* Disable acknowledge. */
	uint8_t no_ack: 1;

	/* Packet ID of the last received packet. Used to detect retransmits. */
	uint8_t pid: 2;
	uint8_t rfu1: 5;
} __packed;

struct my_esb_radio_pdu {
	uint8_t length: 6;
	uint8_t rfu0: 2;
	uint8_t ctrl: 1;
	uint8_t sn: 1;
	uint8_t nesn: 1;
	uint8_t rfu1: 5;
} __packed;

/* Radio PDU header definition. */
union esb_radio_pdu_type {
	/* Fixed PDU header. */
	struct esb_radio_fixed_pdu fixed_pdu;

	/* Dynamic PDU header. */
	struct esb_radio_dynamic_pdu dpl_pdu;
} __packed;

/* Radio PDU definition. */
struct esb_radio_pdu {
	/* PDU header. */
	struct my_esb_radio_pdu pdu;

	/* PDU data. */
	uint8_t data[];
} __packed;

/* Enhanced ShockBurst address.
 *
 * Enhanced ShockBurst addresses consist of a base address and a prefix
 * that is unique for each pipe. See @ref esb_addressing in the ESB user
 * guide for more information.
 */
struct esb_address {
	uint8_t base_addr_p0[4];   /* Base address for pipe 0, in big endian. */
	uint8_t base_addr_p1[4];   /* Base address for pipe 1-7, in big endian. */
	uint8_t pipe_prefixes[8];  /* Address prefix for pipe 0 to 7. */
	uint8_t num_pipes;	   /* Number of pipes available. */
	uint8_t addr_length;	   /* Length of the address plus the prefix. */
	uint8_t rx_pipes_enabled;  /* Bitfield for enabled pipes. */
	uint8_t rf_channel;	   /* Channel to use (between 0 and 100). */
	atomic_t rf_channel_flags; /* Flags for setting the channel. */
};

static nrfx_timer_t esb_timer = ESB_NRFX_TIMER_INSTANCE;

static bool esb_initialized;
static struct esb_config esb_cfg;
static volatile enum esb_state esb_state = ESB_STATE_IDLE;

static struct esb_tdma_context {
	uint32_t last_hb;      // last heartbeat timer value
	int32_t drift;	       // drift between central and peripheral
	uint32_t refslot;      // refslot from central
	uint32_t hb_loops;     // calculated hb loops
	uint32_t slotsize;     // slot size
	uint32_t window_size;  // slot window size (for rx, tx)
	uint32_t start;	       // current radio start timer value
	uint32_t timeout;      // current timeout timer value
	uint32_t desync_count; // number of times that peripheral has desynced
	uint32_t slotdiff;
	uint8_t pipe;	       // current pipe(fixed for periheral)
	uint8_t timeout_count; // desync count
	uint8_t addr_delay;    // calcualted addr delay
	uint8_t channel_idx;   // current channel index(for rssi, tx_power)
	uint8_t pipes;	       // number of pipes that are enabled

	bool is_hb; // true if current slot is heartbeat
} ctx;

struct esb_ctrl_packet {
	uint32_t refslot;
	uint32_t slotsize;
	uint8_t addr_delay;
	uint8_t pipes;
} __packed;

struct esb_header_dn {
	int8_t tx_power;
} __packed;

struct esb_header_up {
	uint8_t dummy[4];
} __packed;

struct esb_packet_dn {
	struct esb_header_dn header;
	uint8_t data[];
} __packed;

struct esb_packet_up {
	struct esb_header_up header;
	uint8_t data[];
} __packed;

struct esb_packet {
	struct esb_header header;
	uint8_t data[];
} __packed;

static sys_slist_t esb_conn_cb_list;
K_MSGQ_DEFINE(sync_event_msgq, sizeof(struct esb_evt), ESB_PIPE_COUNT * 2, 4);

#define DRIFT_LIMIT	 (50)
#define DESYNC_COUNT_MAX (16)
#define RADIO_MARGIN	 (30)
#define TIMEOUT_MARGIN	 (90)

#define HEARTBEAT_INTERVAL    1000000
#define HEARTBEAT_INTERVAL_MS (HEARTBEAT_INTERVAL / 1000)

// static uint64_t channel_map;
#define CHANNELS	39
#define CHANNEL_MASK(x) ((x) - 1)

#define DESYNC_AIRTIME_DEFAULT (150000)
#define SCALE		       (1024)
#define ALPHA		       (40)
#define RSSI_BASELINE	       (55)
#define TX_POWER_MAX	       (5)
#define TX_POWER_MIN	       (-12)
#define TX_POWER_BASE	       (0)
#define TX_POWER_STEP	       (4)
#define TX_POWER_STEP_DN       (-2)
#define TPMAX_SCALED	       (TX_POWER_MAX * SCALE)
#define TPMIN_SCALED	       (TX_POWER_MIN * SCALE)
#define TPSTEP_SCALED	       (TX_POWER_STEP * SCALE)

static struct pipe_info rx_pipe_info[ESB_PIPE_COUNT];
static struct pipe_info *rx_pipe_info_get(uint8_t pipe)
{
	return &rx_pipe_info[resolve_pipe(pipe)];
}

static struct payload_rx_fifo rx_fifo;

static void reset_rx_fifo(void)
{
	rx_fifo.back = 0;
	rx_fifo.front = 0;
	rx_fifo.count = 0;
}

static void initialize_rx_fifo(void)
{
	static struct esb_payload rx_payload[CONFIG_ESB_RX_FIFO_SIZE];

	for (size_t i = 0; i < CONFIG_ESB_RX_FIFO_SIZE; i++) {
		rx_fifo.payload[i] = &rx_payload[i];
	}

	reset_rx_fifo();
}

static inline uint32_t hash(uint32_t x)
{
	x *= 0x9E3779B1;
	x = (x ^ (x >> 16)) * 0x85ebca6b;
	x = (x ^ (x >> 13)) * 0xc2b2ae35;
	x = x ^ (x >> 16);

	return x;
}

static uint8_t get_channel(uint32_t seq)
{
	uint32_t x = hash(seq);
	ctx.channel_idx = x % CHANNELS;
	return 1 + (ctx.channel_idx * 2);
}

static void set_slotsize()
{
	ctx.slotsize = HEARTBEAT_INTERVAL / CONFIG_ESB_POLLING_RATE;

	ctx.window_size = ctx.slotsize / 2;

	// LOG_WRN("slot %u window %u slots %u", ctx.slotsize, ctx.window_size, ctx.pipes);
}

static void set_hb_loops(void)
{
	uint32_t loop_size = ctx.slotsize * ctx.pipes;
	ctx.hb_loops = HEARTBEAT_INTERVAL / loop_size;

	// LOG_WRN("HEARTBEAT LOOPS(%u) SIZE(%u)", ctx.hb_loops, ctx.hb_loops * loop_size);
}

static void set_control_packet(void *data)
{
	if (data == NULL) {
		return;
	}

	struct esb_ctrl_packet *control = (struct esb_ctrl_packet *)data;

	control->slotsize = ctx.slotsize;
	control->addr_delay = ctx.addr_delay;
	control->pipes = ctx.pipes;
	control->refslot = ctx.refslot;
}

static void apply_control_packet(void *data)
{
	if (data == NULL) {
		return;
	}

	struct esb_ctrl_packet *control = (struct esb_ctrl_packet *)data;

	ctx.slotsize = control->slotsize;
	ctx.window_size = ctx.slotsize / 2;
	ctx.pipes = control->pipes;
	ctx.addr_delay = control->addr_delay;
	ctx.refslot = control->refslot;

	// LOG_WRN("slot %u window %u slots %u", ctx.slotsize, ctx.window_size, ctx.pipes);
	set_hb_loops();
}

int moving_average(int new_val, int before)
{
	return (ALPHA * new_val + (SCALE - ALPHA) * before) / SCALE;
}

static void sync_op_work_cb(struct k_work *work)
{
	struct esb_evt evt;
	while (k_msgq_get(&sync_event_msgq, &evt, K_NO_WAIT) == 0) {
		struct esb_conn_cb *cb;
#if CONFIG_ESB_CENTRAL
		LOG_WRN("pipe %d %s", evt.sync.pipe, evt.sync.up ? "connected" : "disconnected");
#else
		LOG_WRN("central %s", evt.sync.up ? "connected" : "disconnected");
#endif
		SYS_SLIST_FOR_EACH_CONTAINER(&esb_conn_cb_list, cb, node) {
			if (evt.sync.up) {
				if (cb->connected) {
					cb->connected(evt.sync.pipe);
				}
			} else {
				if (cb->disconnected) {
					cb->disconnected(evt.sync.pipe);
				}
			}
		}
	}
}
K_WORK_DEFINE(sync_op_work, sync_op_work_cb);

#if CONFIG_ESB_CENTRAL

static struct peripheral_slot {
	uint32_t last_sync[ESB_PIPE_COUNT];
	uint8_t pipe_state;
} slots;

static bool is_slot_synced(uint8_t pipe)
{
	return (slots.pipe_state & BIT(resolve_pipe(pipe)));
}

static bool slot_sync_check(uint8_t pipe, uint32_t ref)
{
	uint32_t timeout = (ctx.hb_loops * ctx.slotsize * ctx.pipes * 2);

	bool in_sync = ((ref - slots.last_sync[resolve_pipe(pipe)]) <= timeout);
	WRITE_BIT(slots.pipe_state, resolve_pipe(pipe), in_sync);

	return in_sync;
}

static void slot_sync_update(uint8_t pipe, uint32_t ref)
{
	slots.last_sync[resolve_pipe(pipe)] = ref;
	WRITE_BIT(slots.pipe_state, resolve_pipe(pipe), true);
}

uint8_t esb_get_slot_state(void)
{
	return slots.pipe_state;
}
#else
uint8_t esb_get_slot_state(void)
{
	return BIT(ctx.pipe);
}
#endif

static uint32_t RSSI[ESB_PIPE_COUNT];
static bool rssi_initialized[ESB_PIPE_COUNT];

static void rssi_reset(uint8_t pipe)
{
	rssi_initialized[resolve_pipe(pipe)] = false;
}

static void rssi_reset_all(void)
{
	for (int pipe = 0; pipe < ARRAY_SIZE(RSSI); ++pipe) {
		// RSSI[pipe] = (RSSI_BASELINE * SCALE);
		rssi_reset(pipe);
	}
}

static void rssi_update(uint8_t pipe, uint8_t sample)
{
	uint32_t rssi_scaled = sample * SCALE;

	if (rssi_initialized[resolve_pipe(pipe)] == false) {
		rssi_initialized[resolve_pipe(pipe)] = true;
		RSSI[resolve_pipe(pipe)] = rssi_scaled;

		return;
	}

	RSSI[resolve_pipe(pipe)] = moving_average(rssi_scaled, RSSI[resolve_pipe(pipe)]);
}

static uint8_t rssi_get(uint8_t pipe)
{
	return (uint8_t)(RSSI[resolve_pipe(pipe)] / SCALE);
}

static int32_t TX_POWER[ESB_PIPE_COUNT];

static void tx_power_reset(uint8_t pipe)
{
	TX_POWER[resolve_pipe(pipe)] = TX_POWER_BASE;
}

static void tx_power_raise(uint8_t pipe)
{
	int new_power = TX_POWER[resolve_pipe(pipe)] + TX_POWER_STEP;
	TX_POWER[resolve_pipe(pipe)] = MIN(new_power, TX_POWER_MAX);
}

static int16_t tx_power_get(uint8_t pipe)
{
	return TX_POWER[resolve_pipe(pipe)];
}

static bool tx_power_update(uint8_t pipe, int8_t tx_power)
{
	int rssi_diff = rssi_get(pipe) - RSSI_BASELINE - tx_power;
	if (abs(rssi_diff) < 3) {
		return false;
	}

	int new_power =
		TX_POWER[resolve_pipe(pipe)] + rssi_diff > 0 ? TX_POWER_STEP : TX_POWER_STEP_DN;

	TX_POWER[resolve_pipe(pipe)] = CLAMP(new_power, TX_POWER_MIN, TX_POWER_MAX);

	return true;
}

#if !CONFIG_ESB_CENTRAL
static bool drift_initialized = true;
static inline void drift_reset(void)
{
	drift_initialized = true;
	ctx.drift = 0;
}

static inline void drift_update(int32_t drift)
{
	int32_t drift_scaled = drift * SCALE;

	if (drift_initialized) {
		drift_initialized = false;
		ctx.drift = drift_scaled;

		return;
	}

	ctx.drift = (drift_scaled + ctx.drift * 3) / 4;
}

static inline int32_t drift_get(uint32_t clock_diff)
{
	uint32_t hb_size = ctx.hb_loops * ctx.slotsize * ctx.pipes;
	if (hb_size == 0) {
		return 0;
	}

	uint32_t denominator = hb_size * SCALE;
	int64_t numerator = (int64_t)ctx.drift * clock_diff;
	int32_t drift = 0;

	if (numerator >= 0) {
		drift = (int32_t)((numerator + (denominator >> 1)) / denominator);
	} else {
		drift = (int32_t)((numerator - (denominator >> 1)) / denominator);
	}

	return drift;
}
#endif

/* Default address configuration for ESB.
 * Roughly equal to the nRF24Lxx defaults, except for the number of pipes,
 * because more pipes are supported.
 */
__ALIGN(4)
static struct esb_address esb_addr = {
	.base_addr_p0 = {0xE7, 0xE7, 0xE7, 0xE7},
	.base_addr_p1 = {0xC2, 0xC2, 0xC2, 0xC2},
	.pipe_prefixes = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8},
	.addr_length = 5,
	.num_pipes = CONFIG_ESB_PIPE_COUNT,
	.rf_channel = 2,
	.rx_pipes_enabled = 0xFF};

enum {
	ERRATA_216_DISABLED,
	ERRATA_216_ENABLED,
};
static atomic_t errata_216_status = ATOMIC_INIT(ERRATA_216_DISABLED);
static uint32_t errata_216_timer_shorts;

#if ERRATA_216_PRESENT
static const struct mbox_dt_spec on_channel =
	MBOX_DT_SPEC_GET(DT_NODELABEL(cpurad_cpusys_errata216_mboxes), on_req);
static const struct mbox_dt_spec off_channel =
	MBOX_DT_SPEC_GET(DT_NODELABEL(cpurad_cpusys_errata216_mboxes), off_req);
#endif /* ERRATA_216_PRESENT */

static esb_event_handler event_handler;

static void set_addr_delay(void)
{
	uint8_t ramp_up_delay =
		esb_cfg.use_fast_ramp_up ? TX_FAST_RAMP_UP_TIME_US : TX_RAMP_UP_TIME_US;

	/* preamble(8 bits) + address(addr_length bytes × 8 bits) */
	uint8_t preamble_addr_pcf_bits = 8 + (esb_addr.addr_length * 8) + 9;

	/* airtime in µs: bits, half if 2Mbps */
	uint8_t addr_airtime = esb_cfg.bitrate == ESB_BITRATE_2MBPS ? preamble_addr_pcf_bits / 2
								    : preamble_addr_pcf_bits;

	ctx.addr_delay = ramp_up_delay + addr_airtime;
	// LOG_WRN("ADDR_DELAY(%u)", ctx.addr_delay);
}

#if CONFIG_ESB_CENTRAL

void monitoring_work_cb(struct k_work *work)
{
	printk("pipe  rssi\n");
	printk("----  ----\n");
	for (int pipe = 0; pipe < ESB_PIPE_COUNT; ++pipe) {
		int rssi = -rssi_get(pipe);
		printk("%4u  %+4d dBm\n", pipe, rssi);
	}

	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	k_work_reschedule(dwork, K_SECONDS(1));
}
K_WORK_DELAYABLE_DEFINE(monitoring_work, monitoring_work_cb);

static void central_setup(void)
{
	nrf_radio_shorts_set(NRF_RADIO, RADIO_SHORTS_COMMON);
	nrf_timer_int_enable(esb_timer.p_reg, NRF_TIMER_INT_COMPARE1_MASK);
	ctx.pipes = ESB_PIPE_COUNT;
	set_slotsize();
	set_hb_loops();
	set_addr_delay();
	rssi_reset_all();

	ctx.refslot = sys_rand32_get();
	// LOG_WRN("SEED: %u", ctx.refslot);
}

#else

void monitoring_work_cb(struct k_work *work)
{
	int rssi = -rssi_get(ctx.pipe);
	int tx_power_cfg = tx_power_get(ctx.pipe);
	int tx_power = (int8_t)nrf_radio_txpower_get(NRF_RADIO);

	printk("rssi cfg  reg\n");
	printk("---- ---- ----\n");
	printk("%+4d %+4d %+4d dBm\n", rssi, tx_power_cfg, tx_power);

	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	k_work_reschedule(dwork, K_SECONDS(1));
}
K_WORK_DELAYABLE_DEFINE(monitoring_work, monitoring_work_cb);

static void peripheral_setup(void)
{
	nrf_radio_shorts_set(NRF_RADIO, RADIO_SHORTS_COMMON);
	nrf_radio_int_enable(NRF_RADIO, NRF_RADIO_INT_DISABLED_MASK);

	ctx.pipe = esb_cfg.pipe;

	tx_power_reset(ctx.pipe);

	nrf_radio_txaddress_set(NRF_RADIO, ctx.pipe);
	nrf_radio_rxaddresses_set(NRF_RADIO, BIT(ctx.pipe));
}
#endif

void pto_context_reset(void)
{
	ctx.start = 0;
	ctx.timeout = 0;
	ctx.refslot = 0;
	ctx.desync_count = 0;
	ctx.channel_idx = 0;
}

static uint8_t rx_payload_buffer[CONFIG_ESB_MAX_PAYLOAD_LENGTH + sizeof(struct esb_radio_pdu)];
static uint8_t tx_payload_buffer[CONFIG_ESB_MAX_PAYLOAD_LENGTH + sizeof(struct esb_radio_pdu)];

/* Run time variables */
static volatile uint32_t interrupt_flags;
static volatile struct tx_evt last_tx_evt;
static volatile uint32_t wait_for_ack_timeout_us;

static uint32_t radio_shorts_common = RADIO_SHORTS_COMMON;
static const bool fast_switching = IS_ENABLED(CONFIG_ESB_FAST_SWITCHING);

static const mpsl_fem_event_t rx_event = {
	.type = MPSL_FEM_EVENT_TYPE_TIMER,
	.event.timer =
		{
			.p_timer_instance = ESB_NRF_TIMER_INSTANCE,
			.compare_channel_mask =
				(BIT(NRF_TIMER_CC_CHANNEL2) | BIT(NRF_TIMER_CC_CHANNEL3)),
			.counter_period =
				{
					.end = RX_RAMP_UP_TIME_US,
				},
		},
};

static const mpsl_fem_event_t tx_event = {
	.type = MPSL_FEM_EVENT_TYPE_TIMER,
	.event.timer =
		{
			.p_timer_instance = ESB_NRF_TIMER_INSTANCE,
			.compare_channel_mask =
				(BIT(NRF_TIMER_CC_CHANNEL2) | BIT(NRF_TIMER_CC_CHANNEL3)),
			.counter_period =
				{
					.end = TX_RAMP_UP_TIME_US,
				},
		},
};

static mpsl_fem_event_t tx_time_shifted = {
	.type = MPSL_FEM_EVENT_TYPE_TIMER,
	.event.timer =
		{
			.p_timer_instance = ESB_NRF_TIMER_INSTANCE,
			.compare_channel_mask =
				(BIT(NRF_TIMER_CC_CHANNEL2) | BIT(NRF_TIMER_CC_CHANNEL3)),
		},
};

static mpsl_fem_event_t disable_event = {
	.type = MPSL_FEM_EVENT_TYPE_GENERIC,
};

/* These function pointers are changed dynamically, depending on protocol
 * configuration and state. Note that they will be 0 initialized.
 */
static void (*on_radio_disabled)(void);
static void (*on_timer_compare1)(void);
static void (*update_rf_payload_format)(uint32_t payload_length);

static void central_prepare_tx(void);
static void central_timeslot_end(void);

static void peripheral_start_desync(void);
static void peripheral_disabled_desync(void);
static void peripheral_prepare_rx(void);
static void peripheral_disabled_rx(void);
static void peripheral_disabled_tx_ack(void);

// static inline uint32_t

/*  Function to do bytewise bit-swap on an unsigned 32-bit value */
static uint32_t bytewise_bit_swap(const uint8_t *input)
{
#if __CORTEX_M == (0x04U)
	uint32_t inp = (*(uint32_t *)input);

	return sys_cpu_to_be32((uint32_t)__RBIT(inp));
#else
	uint32_t inp = sys_cpu_to_le32(*(uint32_t *)input);

	inp = (inp & 0xF0F0F0F0) >> 4 | (inp & 0x0F0F0F0F) << 4;
	inp = (inp & 0xCCCCCCCC) >> 2 | (inp & 0x33333333) << 2;
	inp = (inp & 0xAAAAAAAA) >> 1 | (inp & 0x55555555) << 1;
	return inp;
#endif
}

/* Convert a base address from nRF24L format to nRF5 format */
static uint32_t addr_conv(const uint8_t *addr)
{
	return __REV(bytewise_bit_swap(addr));
}

static inline void apply_errata143_workaround(void)
{
	/* Workaround for Errata 143
	 * Check if the most significant bytes of address 0 (including
	 * prefix) match those of another address. It's recommended to
	 * use a unique address 0 since this will avoid the 3dBm penalty
	 * incurred from the workaround.
	 */
	uint32_t base_address_mask = esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;

	/* Load the two addresses before comparing them to ensure
	 * defined ordering of volatile accesses.
	 */
	uint32_t addr0 = nrf_radio_base0_get(NRF_RADIO) & base_address_mask;
	uint32_t addr1 = nrf_radio_base1_get(NRF_RADIO) & base_address_mask;

	if (addr0 == addr1) {
		uint32_t radio_prefix0 = nrf_radio_prefix0_get(NRF_RADIO);
		uint32_t radio_prefix1 = nrf_radio_prefix1_get(NRF_RADIO);

		uint8_t prefix0 = radio_prefix0 & RADIO_PREFIX0_AP0_Msk;
		uint8_t prefix1 = (radio_prefix0 & RADIO_PREFIX0_AP1_Msk) >> RADIO_PREFIX0_AP1_Pos;
		uint8_t prefix2 = (radio_prefix0 & RADIO_PREFIX0_AP2_Msk) >> RADIO_PREFIX0_AP2_Pos;
		uint8_t prefix3 = (radio_prefix0 & RADIO_PREFIX0_AP3_Msk) >> RADIO_PREFIX0_AP3_Pos;
		uint8_t prefix4 = radio_prefix1 & RADIO_PREFIX1_AP4_Msk;
		uint8_t prefix5 = (radio_prefix1 & RADIO_PREFIX1_AP5_Msk) >> RADIO_PREFIX1_AP5_Pos;
		uint8_t prefix6 = (radio_prefix1 & RADIO_PREFIX1_AP6_Msk) >> RADIO_PREFIX1_AP6_Pos;
		uint8_t prefix7 = (radio_prefix1 & RADIO_PREFIX1_AP7_Msk) >> RADIO_PREFIX1_AP7_Pos;

		if ((prefix0 == prefix1) || (prefix0 == prefix2) || (prefix0 == prefix3) ||
		    (prefix0 == prefix4) || (prefix0 == prefix5) || (prefix0 == prefix6) ||
		    (prefix0 == prefix7)) {
			/* This will cause a 3dBm sensitivity loss,
			 * avoid using such address combinations if possible.
			 */
			*(volatile uint32_t *)0x40001774 =
				((*(volatile uint32_t *)0x40001774) & 0xfffffffe) | 0x01000000;
		}
	}
}

static void errata216_on(void)
{
#if ERRATA_216_PRESENT
	if (mbox_send_dt(&on_channel, NULL) != 0) {
		LOG_ERR("Failed to enable Errata 216");
		/* Should not happen. */
		__ASSERT_NO_MSG(false);
	} else {
		atomic_set(&errata_216_status, ERRATA_216_ENABLED);
	}
#endif /* ERRATA_216_PRESENT */
}

static void errata216_off(void)
{
#if ERRATA_216_PRESENT
	if (mbox_send_dt(&off_channel, NULL) != 0) {
		LOG_ERR("Failed to disable Errata 216");
		/* Should not happen. */
		__ASSERT_NO_MSG(false);
	} else {
		atomic_set(&errata_216_status, ERRATA_216_DISABLED);
	}
#endif /* ERRATA_216_PRESENT */
}

static void esb_fem_for_tx_set(bool ack)
{
	// uint32_t timer_shorts;

	// timer_shorts = NRF_TIMER_SHORT_COMPARE2_STOP_MASK;

	// if (ack) {
	// 	/* In case of expecting ACK reception, clear the timer to start counting from 0
	// 	 * on the RADIO_DISABLED event. This is needed to start external front-end module
	// 	 * with valid timeout because radio uses DISABLED_RXEN short.
	// 	 */
	// 	timer_shorts |= NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK;
	// }

	// if (mpsl_fem_pa_configuration_set(&tx_event, &disable_event) == 0) {
	// 	mpsl_fem_enable();
	// 	esb_ppi_for_fem_set();
	// } else {
	// 	/* We want to start counting ACK timeout and potential packet retransmission from
	// 	 * RADIO_DISABLED event so timer starts through EGU together with radio ramp-up,
	// 	 * we want to stop and clear it before RADIO_DISABLED event to start it again
	// 	 * when this event occurs. The timer value must be big enough to give us possibility
	// 	 * to reconfigure timer shorts before timer will be cleared by them.
	// 	 */
	// 	uint16_t ramp_up =
	// 		esb_cfg.use_fast_ramp_up ? TX_FAST_RAMP_UP_TIME_US : TX_RAMP_UP_TIME_US;
	// 	nrf_timer_cc_set(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL2, ramp_up);
	// }

	// if (IS_ENABLED(CONFIG_ESB_NEVER_DISABLE_TX)) {
	// 	uint32_t cc1 = nrfx_timer_capture_get(&esb_timer, NRF_TIMER_CC_CHANNEL1);
	// 	uint32_t cc2 = nrfx_timer_capture_get(&esb_timer, NRF_TIMER_CC_CHANNEL2);

	// 	if (cc1 > cc2) {
	// 		timer_shorts = NRF_TIMER_SHORT_COMPARE1_STOP_MASK;
	// 		timer_shorts |= NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK;
	// 	}
	// }

	// nrf_timer_shorts_set(esb_timer.p_reg, timer_shorts);
}

static void esb_fem_for_rx_set(void)
{
	// if (mpsl_fem_lna_configuration_set(&rx_event, &disable_event) == 0) {
	// 	mpsl_fem_enable();
	// 	esb_ppi_for_fem_set();
	// 	nrf_timer_shorts_set(esb_timer.p_reg, (NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK |
	// 					       NRF_TIMER_SHORT_COMPARE2_STOP_MASK));
	// }
}

static void esb_fem_for_ack_rx(void)
{
	/* Timer is running and timer's shorts and PPI connections have been configured. */
	mpsl_fem_pa_configuration_clear();
	mpsl_fem_lna_configuration_set(&rx_event, &disable_event);
}

static void esb_fem_for_tx_ack(void)
{
	/* Timer is running and timer's shorts and PPI connections have been configured. */
	mpsl_fem_lna_configuration_clear();
	mpsl_fem_pa_configuration_set(&tx_event, &disable_event);
}

static void esb_fem_reset(void)
{
	// #if NRF_TIMER_HAS_SHUTDOWN
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_SHUTDOWN);
	// #else
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_STOP);
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_CLEAR);
	// #endif

	mpsl_fem_lna_configuration_clear();
	mpsl_fem_pa_configuration_clear();

	esb_ppi_for_fem_clear();

	mpsl_fem_deactivate_now(MPSL_FEM_ALL);
	mpsl_fem_disable();
}

static void esb_fem_lna_reset(void)
{
	// #if NRF_TIMER_HAS_SHUTDOWN
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_SHUTDOWN);
	// #else
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_STOP);
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_CLEAR);
	// #endif

	esb_ppi_for_fem_clear();

	mpsl_fem_lna_configuration_clear();
	mpsl_fem_disable();
}

static void esb_fem_pa_reset(void)
{
	mpsl_fem_pa_configuration_clear();

	// #if NRF_TIMER_HAS_SHUTDOWN
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_SHUTDOWN);
	// #else
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_STOP);
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_CLEAR);
	// #endif

	esb_ppi_for_fem_clear();

	mpsl_fem_disable();
}

void esb_fem_for_rx_ack(void)
{
	mpsl_fem_pa_configuration_clear();
	mpsl_fem_lna_configuration_set(&rx_event, &disable_event);

	/* The timer is running, shorts must be reconfigured because we do not want to
	 * stop timer after front-end module was triggered. Timer needs to count ACK reception
	 * timeout and potential retransmission delay. Timer cannot be stopped here because
	 * these timeout are counted since RADIO_DISABLED event.
	 */
	// nrf_timer_shorts_disable(esb_timer.p_reg, (NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK |
	// 					   NRF_TIMER_SHORT_COMPARE2_STOP_MASK));
}

void esb_fem_for_tx_retry(void)
{
	// /* The radio is ramped-up with delay set in the timer compare channel 1.
	//  * Calculate the ramp-up time for external front-end module.
	//  */
	// tx_time_shifted.event.timer.counter_period.end =
	// 	nrf_timer_cc_get(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL1) + TX_RAMP_UP_TIME_US;

	// /* This starts the ESB TIMER on the radio disabled event. This connection is needed even
	//  * when front-end module is not used.
	//  */
	// esb_ppi_for_fem_set();

	// if (mpsl_fem_pa_configuration_set(&tx_time_shifted, &disable_event) == 0) {
	// 	/* In case of retransmission timer is configured to trigger RADIO_TXEN task after
	// 	 * retransmission timeout, but with external front-end module, it must also
	// 	 * schedule front-end module ramp-up which occurs later. Shorts need to be
	// 	 * reconfigured here to stop and clear the timer after front-end module will be
	// 	 * ramped-up.
	// 	 */
	// 	nrf_timer_shorts_set(esb_timer.p_reg, (NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK |
	// 					       NRF_TIMER_SHORT_COMPARE2_STOP_MASK));
	// }
}

void esb_fem_for_tx_retry_clear(void)
{
	// esb_ppi_for_fem_clear();

	// mpsl_fem_pa_configuration_clear();
	// mpsl_fem_deactivate_now(MPSL_FEM_ALL);

	// nrf_timer_shorts_disable(esb_timer.p_reg, (NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK |
	// 					   NRF_TIMER_SHORT_COMPARE2_STOP_MASK));
}

static void radio_start(void)
{
	// if (IS_ENABLED(ERRATA_216_PRESENT) &&
	//     atomic_get(&errata_216_status) == ERRATA_216_DISABLED) {
	// 	errata216_on();

	// 	nrfx_timer_compare(
	// 		&esb_timer, NRF_TIMER_CC_CHANNEL3,
	// 		nrfx_timer_us_to_ticks(&esb_timer, ERRATA_216_RADIO_ENABLE_DELAY_US), true);

	// 	errata_216_timer_shorts = esb_timer.p_reg->SHORTS;
	// 	nrf_timer_shorts_set(esb_timer.p_reg, (NRF_TIMER_SHORT_COMPARE3_STOP_MASK |
	// 					       NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK));
	// 	nrfx_timer_clear(&esb_timer);
	// 	nrf_timer_task_trigger(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_START);
	// } else {
	// 	/* Event generator unit is used to start radio and protocol timer if needed. */
	// 	nrf_egu_task_trigger(ESB_EGU, ESB_EGU_TASK);
	// }
}

static void update_rf_payload_format_esb_dpl(uint32_t payload_length)
{
	nrf_radio_packet_conf_t packet_config = {0};

	packet_config.s0len = 0;
	packet_config.s1len = 3;

	/* Using 6 bits or 8 bits for length */
	packet_config.lflen = (CONFIG_ESB_MAX_PAYLOAD_LENGTH <= 32) ? 6 : 8;
	packet_config.whiteen = false;
	packet_config.big_endian = true;
	packet_config.balen = (esb_addr.addr_length - 1);
	packet_config.statlen = 0;
	packet_config.maxlen = CONFIG_ESB_MAX_PAYLOAD_LENGTH;
#if defined(RADIO_PCNF0_PLEN_Msk)
	if (esb_cfg.bitrate == ESB_BITRATE_2MBPS) {
		packet_config.plen = NRF_RADIO_PREAMBLE_LENGTH_16BIT;
	}

#if defined(RADIO_MODE_MODE_Ble_2Mbit)
	if (esb_cfg.bitrate == ESB_BITRATE_2MBPS_BLE) {
		packet_config.plen = NRF_RADIO_PREAMBLE_LENGTH_16BIT;
	}
#endif /* defined(RADIO_MODE_MODE_Ble_2Mbit) */

#if defined(RADIO_MODE_MODE_Nrf_4Mbit_0BT6)
	if (esb_cfg.bitrate == ESB_BITRATE_4MBPS) {
		packet_config.plen = NRF_RADIO_PREAMBLE_LENGTH_16BIT;
	}
#endif /* defined(RADIO_MODE_MODE_Nrf_4Mbit_0BT6) */

#endif /* defined(RADIO_PCNF0_PLEN_Msk) */

	nrf_radio_packet_configure(NRF_RADIO, &packet_config);
}

static void update_rf_payload_format_esb(uint32_t payload_length)
{
	const nrf_radio_packet_conf_t packet_config = {.s0len = 1,
						       .lflen = 0,
						       .s1len = 1,
						       .whiteen = false,
						       .big_endian = true,
						       .balen = (esb_addr.addr_length - 1),
						       .statlen = payload_length,
						       .maxlen = payload_length};

	nrf_radio_packet_configure(NRF_RADIO, &packet_config);
}

static void update_radio_addresses(uint8_t update_mask)
{
	if ((update_mask & ADDR_UPDATE_MASK_BASE0) != 0) {
		nrf_radio_base0_set(NRF_RADIO, addr_conv(esb_addr.base_addr_p0));
	}

	if ((update_mask & ADDR_UPDATE_MASK_BASE1) != 0) {
		nrf_radio_base1_set(NRF_RADIO, addr_conv(esb_addr.base_addr_p1));
	}

	if ((update_mask & ADDR_UPDATE_MASK_PREFIX) != 0) {
		nrf_radio_prefix0_set(NRF_RADIO, bytewise_bit_swap(&esb_addr.pipe_prefixes[0]));
		nrf_radio_prefix1_set(NRF_RADIO, bytewise_bit_swap(&esb_addr.pipe_prefixes[4]));
	}

	/* Workaround for Errata 143 */
#if NRF52_ERRATA_143_ENABLE_WORKAROUND
	if (nrf52_errata_143()) {
		apply_errata143_workaround();
	}
#endif
}

#if defined(CONFIG_SOC_SERIES_NRF54HX) || defined(CONFIG_SOC_SERIES_NRF54LX)
static nrf_radio_txpower_t dbm_to_nrf_radio_txpower(int8_t tx_power)
{
	switch (tx_power) {
#if defined(RADIO_TXPOWER_TXPOWER_Neg100dBm)
	case -100:
		return RADIO_TXPOWER_TXPOWER_Neg100dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg100dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg70dBm)
	case -70:
		return RADIO_TXPOWER_TXPOWER_Neg70dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg70dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg46dBm)
	case -46:
		return RADIO_TXPOWER_TXPOWER_Neg46dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg46dBm) */

	case -40:
		return RADIO_TXPOWER_TXPOWER_Neg40dBm;

#if defined(RADIO_TXPOWER_TXPOWER_Neg30dBm)
	case -30:
		return RADIO_TXPOWER_TXPOWER_Neg30dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg30dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg28dBm)
	case -28:
		return RADIO_TXPOWER_TXPOWER_Neg28dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg28dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg22dBm)
	case -22:
		return RADIO_TXPOWER_TXPOWER_Neg22dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg22dBm) */

	case -20:
		return RADIO_TXPOWER_TXPOWER_Neg20dBm;

#if defined(RADIO_TXPOWER_TXPOWER_Neg18dBm)
	case -18:
		return RADIO_TXPOWER_TXPOWER_Neg18dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg18dBm) */

	case -16:
		return RADIO_TXPOWER_TXPOWER_Neg16dBm;

#if defined(RADIO_TXPOWER_TXPOWER_Neg14dBm)
	case -14:
		return RADIO_TXPOWER_TXPOWER_Neg14dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg14dBm) */

	case -12:
		return RADIO_TXPOWER_TXPOWER_Neg12dBm;

#if defined(RADIO_TXPOWER_TXPOWER_Neg10dBm)
	case -10:
		return RADIO_TXPOWER_TXPOWER_Neg10dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg10dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg9dBm)
	case -9:
		return RADIO_TXPOWER_TXPOWER_Neg9dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg9dBm) */

	case -8:
		return RADIO_TXPOWER_TXPOWER_Neg8dBm;

#if defined(RADIO_TXPOWER_TXPOWER_Neg7dBm)
	case -7:
		return RADIO_TXPOWER_TXPOWER_Neg7dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg7dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg6dBm)
	case -6:
		return RADIO_TXPOWER_TXPOWER_Neg6dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg6dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg5dBm)
	case -5:
		return RADIO_TXPOWER_TXPOWER_Neg5dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg5dBm) */

	case -4:
		return RADIO_TXPOWER_TXPOWER_Neg4dBm;

#if defined(RADIO_TXPOWER_TXPOWER_Neg3dBm)
	case -3:
		return RADIO_TXPOWER_TXPOWER_Neg3dBm;
#endif /* defined (RADIO_TXPOWER_TXPOWER_Neg3dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg2dBm)
	case -2:
		return RADIO_TXPOWER_TXPOWER_Neg2dBm;
#endif /* defined (RADIO_TXPOWER_TXPOWER_Neg2dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg1dBm)
	case -1:
		return RADIO_TXPOWER_TXPOWER_Neg1dBm;
#endif /* defined (RADIO_TXPOWER_TXPOWER_Neg1dBm) */

	case 0:
		return RADIO_TXPOWER_TXPOWER_0dBm;

#if defined(RADIO_TXPOWER_TXPOWER_Pos1dBm)
	case 1:
		return RADIO_TXPOWER_TXPOWER_Pos1dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos1dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos2dBm)
	case 2:
		return RADIO_TXPOWER_TXPOWER_Pos2dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos2dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos3dBm)
	case 3:
		return RADIO_TXPOWER_TXPOWER_Pos3dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos3dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos4dBm)
	case 4:
		return RADIO_TXPOWER_TXPOWER_Pos4dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos4dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos5dBm)
	case 5:
		return RADIO_TXPOWER_TXPOWER_Pos5dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos5dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos6dBm)
	case 6:
		return RADIO_TXPOWER_TXPOWER_Pos6dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos6dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos7dBm)
	case 7:
		return RADIO_TXPOWER_TXPOWER_Pos7dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos7dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos8dBm)
	case 8:
		return RADIO_TXPOWER_TXPOWER_Pos8dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos8dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos9dBm)
	case 9:
		return RADIO_TXPOWER_TXPOWER_Pos9dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos9dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos10dBm)
	case 10:
		return RADIO_TXPOWER_TXPOWER_Pos10dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos10dBm) */

	default:
		printk("TX power to enumerator conversion failed, defaulting to 0 dBm\n");
		return RADIO_TXPOWER_TXPOWER_0dBm;
	}
}
#endif /* defined(CONFIG_SOC_SERIES_NRF54HX) || defined(CONFIG_SOC_SERIES_NRF54LX) */

#if !(defined(CONFIG_SOC_SERIES_NRF54HX) || defined(CONFIG_SOC_SERIES_NRF54LX))
static mpsl_phy_t convert_bitrate_to_mpsl_phy(enum esb_bitrate bitrate)
{
	switch (bitrate) {
	case ESB_BITRATE_1MBPS:
		return MPSL_PHY_NRF_1Mbit;
	case ESB_BITRATE_2MBPS:
		return MPSL_PHY_NRF_2Mbit;
#if defined(RADIO_MODE_MODE_Nrf_250Kbit)
	case ESB_BITRATE_250KBPS:
		return MPSL_PHY_NRF_250Kbit;
#endif
	case ESB_BITRATE_1MBPS_BLE:
		return MPSL_PHY_BLE_1M;
#if defined(RADIO_MODE_MODE_Ble_2Mbit)
	case ESB_BITRATE_2MBPS_BLE:
		return MPSL_PHY_BLE_2M;
#endif
#if defined(RADIO_MODE_MODE_Nrf_4Mbit_0BT6)
	case ESB_BITRATE_4MBPS:
		return MPSL_PHY_NRF_4Mbit_0BT6;
#endif
	default:
		return MPSL_PHY_NRF_1Mbit;
	}
}
#endif /* !(defined(CONFIG_SOC_SERIES_NRF54HX) || defined(CONFIG_SOC_SERIES_NRF54LX)) */

static void update_radio_tx_power(void)
{
#if !(defined(CONFIG_SOC_SERIES_NRF54HX) || defined(CONFIG_SOC_SERIES_NRF54LX))
	int32_t err;
	mpsl_tx_power_split_t tx_power;

	(void)mpsl_fem_tx_power_split(esb_cfg.tx_output_power, &tx_power,
				      convert_bitrate_to_mpsl_phy(esb_cfg.bitrate),
				      (RADIO_BASE_FREQUENCY + esb_addr.rf_channel), false);

	err = mpsl_fem_pa_power_control_set(tx_power.fem_pa_power_control);
	if (err) {
		/* Should not happen. */
		__ASSERT_NO_MSG(false);
	}

#ifdef NRF53_SERIES
	bool high_voltage = false;

	if (tx_power.radio_tx_power > 0) {
		high_voltage = true;
		tx_power.radio_tx_power -= NRF5340_HIGH_VOLTAGE_GAIN;
	}

	nrf_vreqctrl_radio_high_voltage_set(NRF_VREQCTRL_NS, high_voltage);
#endif /* NRF53_SERIES */

	nrf_radio_txpower_set(NRF_RADIO, tx_power.radio_tx_power);
#else
	nrf_radio_txpower_set(NRF_RADIO, dbm_to_nrf_radio_txpower(esb_cfg.tx_output_power));
#endif /* !(defined(CONFIG_SOC_SERIES_NRF54HX) || defined(CONFIG_SOC_SERIES_NRF54LX)) */
}

static bool update_radio_bitrate(void)
{
	nrf_radio_mode_set(NRF_RADIO, esb_cfg.bitrate);

	switch (esb_cfg.bitrate) {

#if defined(RADIO_MODE_MODE_Nrf_4Mbit_0BT6)
	case ESB_BITRATE_4MBPS:
		wait_for_ack_timeout_us = RX_ACK_TIMEOUT_US_4MBPS;
		break;
#endif /* defined(RADIO_MODE_MODE_Nrf_4Mbit_0BT6) */

	case ESB_BITRATE_2MBPS:

#if defined(RADIO_MODE_MODE_Ble_2Mbit)
	case ESB_BITRATE_2MBPS_BLE:
#endif /* defined(RADIO_MODE_MODE_Ble_2Mbit) */

		wait_for_ack_timeout_us = RX_ACK_TIMEOUT_US_2MBPS;
		break;

	case ESB_BITRATE_1MBPS:
		wait_for_ack_timeout_us = RX_ACK_TIMEOUT_US_1MBPS;
		break;

#if defined(RADIO_MODE_MODE_Nrf_250Kbit)
	case ESB_BITRATE_250KBPS:
		wait_for_ack_timeout_us = RX_ACK_TIMEOUT_US_250KBPS;
		break;
#endif /* defined(RADIO_MODE_MODE_Nrf_250Kbit) */

	case ESB_BITRATE_1MBPS_BLE:
		wait_for_ack_timeout_us = RX_ACK_TIMEOUT_US_1MBPS_BLE;
		break;

	default:
		/* Should not be reached */
		return false;
	}

	return true;
}

static bool update_radio_protocol(void)
{
	switch (esb_cfg.protocol) {
	case ESB_PROTOCOL_ESB_DPL:
		update_rf_payload_format = update_rf_payload_format_esb_dpl;
		break;

	case ESB_PROTOCOL_ESB:
		update_rf_payload_format = update_rf_payload_format_esb;
		break;

	default:
		/* Should not be reached */
		return false;
	}
	return true;
}

static bool update_radio_crc(void)
{
	switch (esb_cfg.crc) {
	case ESB_CRC_16BIT:
		nrf_radio_crcinit_set(NRF_RADIO, 0xFFFFUL); /* Initial value */
		nrf_radio_crc_configure(NRF_RADIO, ESB_CRC_16BIT, NRF_RADIO_CRC_ADDR_INCLUDE,
					0x11021UL); /* CRC poly: x^16+x^12^x^5+1 */
		break;

	case ESB_CRC_8BIT:
		nrf_radio_crcinit_set(NRF_RADIO, 0xFFUL); /* Initial value */
		nrf_radio_crc_configure(NRF_RADIO, ESB_CRC_8BIT, NRF_RADIO_CRC_ADDR_INCLUDE,
					0x107UL); /* CRC poly: x^8+x^2^x^1+1 */
		break;

	case ESB_CRC_OFF:
		nrf_radio_crcinit_set(NRF_RADIO, 0x00UL);
		nrf_radio_crc_configure(NRF_RADIO, ESB_CRC_OFF, NRF_RADIO_CRC_ADDR_INCLUDE, 0x00UL);

		break;

	default:
		return false;
	}

	return true;
}

static bool update_radio_parameters(void)
{
	bool params_valid = true;

	params_valid &= update_radio_bitrate();
	params_valid &= update_radio_protocol();
	params_valid &= update_radio_crc();
	update_rf_payload_format(esb_cfg.payload_length);
	params_valid &= (esb_cfg.retransmit_delay >= RETRANSMIT_DELAY_MIN);

	return params_valid;
}

/*  Function to push the content of the rx_buffer to the RX FIFO.
 *
 *  The module will point the register NRF_RADIO->PACKETPTR to a buffer for
 *  receiving packets. After receiving a packet the module will call this
 *  function to copy the received data to the RX FIFO.
 *
 *  @param  pipe Pipe number to set for the packet.
 *  @param  pid  Packet ID.
 *
 *  @retval true   Operation successful.
 *  @retval false  Operation failed.
 */
static bool push_rx_fifo(uint8_t pipe, uint8_t pid, uint8_t rx_len, uint8_t *rx_data)
{
	if (rx_fifo.count >= CONFIG_ESB_RX_FIFO_SIZE) {
		return false;
	}

	if (rx_len > CONFIG_ESB_MAX_PAYLOAD_LENGTH) {
		return false;
	}

	rx_fifo.payload[rx_fifo.back]->length = rx_len;

	memcpy(rx_fifo.payload[rx_fifo.back]->data, rx_data, rx_len);

	rx_fifo.payload[rx_fifo.back]->pipe = pipe;
	rx_fifo.payload[rx_fifo.back]->rssi = rssi_get(pipe);
	rx_fifo.payload[rx_fifo.back]->pid = pid;
	// rx_fifo.payload[rx_fifo.back]->noack = !rx_pdu->type.dpl_pdu.no_ack;

	if (++rx_fifo.back >= CONFIG_ESB_RX_FIFO_SIZE) {
		rx_fifo.back = 0;
	}
	rx_fifo.count++;

	return true;
}

static void esb_timer_handler(nrf_timer_event_t event_type, void *context)
{
	if (nrf_timer_int_enable_check(esb_timer.p_reg, NRF_TIMER_INT_COMPARE1_MASK)) {
		nrf_timer_event_clear(esb_timer.p_reg, NRF_TIMER_EVENT_COMPARE1);
		if (on_timer_compare1 != NULL) {
			on_timer_compare1();
		}
	}

	if (IS_ENABLED(ERRATA_216_PRESENT) && event_type == NRF_TIMER_EVENT_COMPARE3) {
		nrf_timer_int_disable(esb_timer.p_reg, NRF_TIMER_INT_COMPARE3_MASK);

		if (atomic_get(&errata_216_status) == ERRATA_216_ENABLED) {
			/* This case is triggered after calling the radio_start() function */

			/* Restore timer shorts */
			nrf_timer_shorts_set(esb_timer.p_reg, errata_216_timer_shorts);

			nrf_egu_task_trigger(ESB_EGU, ESB_EGU_TASK);
		} else {
			/* This case is triggered during retransmission. */
			errata216_on();
		}
	}
}

static int sys_timer_init(void)
{
	nrfx_err_t nrfx_err;
	const nrfx_timer_config_t config = {
		.frequency = NRFX_MHZ_TO_HZ(1),
		.mode = NRF_TIMER_MODE_TIMER,
		.bit_width = NRF_TIMER_BIT_WIDTH_32,
	};

	nrfx_err = nrfx_timer_init(&esb_timer, &config, esb_timer_handler);
	if (nrfx_err != NRFX_SUCCESS) {
		LOG_ERR("Failed to initialize nrfx timer (err %d)", nrfx_err);
		return -EFAULT;
	}

	return 0;
}

static void sys_timer_deinit(void)
{
	nrfx_timer_uninit(&esb_timer);
}

static void set_evt_interrupt(void)
{
	if (IS_ENABLED(ESB_EVT_USING_EGU)) {
		nrf_egu_task_trigger(ESB_EGU, ESB_EGU_EVT_TASK);
	} else {
		NVIC_SetPendingIRQ(ESB_EVT_IRQ_NUMBER);
	}
}

static void set_tx_evt_interrupt(uint8_t pipe, bool success)
{
	struct pipe_info *info = rx_pipe_info_get(pipe);

	interrupt_flags |= (success ? INT_TX_SUCCESS_MSK : INT_TX_FAILED_MSK);

	last_tx_evt.tx_attempts = info->tx_try;
	info->tx_try = 0;

	set_evt_interrupt();
}

static void set_sync_evt_interrupt(uint8_t pipe, bool up)
{
	ctx.timeout_count = 0;
	if (!up) {
		ctx.desync_count++;
	}

	struct esb_evt event = {.sync.pipe = pipe, .sync.up = up};

	if (k_msgq_put(&sync_event_msgq, &event, K_NO_WAIT) == 0) {
		k_work_submit(&sync_op_work);
	}
}

static void set_rx_evt_interrupt(void)
{
	interrupt_flags |= INT_RX_DATA_RECEIVED_MSK;

	set_evt_interrupt();
}

static void fast_switchinng_set_channel(uint8_t channel)
{
	*(volatile uint32_t *)((uint8_t *)(NRF_RADIO) + 0x70C) &= ~(1 << 31);
	nrf_radio_frequency_set(NRF_RADIO, (RADIO_BASE_FREQUENCY + channel));
	*(volatile uint32_t *)((uint8_t *)(NRF_RADIO) + 0x07C) = 1;
}

/* Retrieve interrupt flags and reset them.
 *
 * @param[out] interrupts	Interrupt flags.
 */
static void get_and_clear_irqs(uint32_t *interrupts)
{
	__ASSERT_NO_MSG(interrupts != NULL);

	unsigned int key = irq_lock();

	*interrupts = interrupt_flags;
	interrupt_flags = 0;

	irq_unlock(key);
}

static void radio_irq_handler(void)
{
	if (nrf_radio_int_enable_check(NRF_RADIO, NRF_RADIO_INT_DISABLED_MASK) &&
	    nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_DISABLED)) {
		nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
		/* Call the correct on_radio_disable function, depending on the
		 * current protocol state.
		 */
		if (on_radio_disabled) {
			on_radio_disabled();
		}
	}

	if (nrf_radio_int_enable_check(NRF_RADIO, ESB_RADIO_INT_END_MASK) &&
	    nrf_radio_event_check(NRF_RADIO, ESB_RADIO_EVENT_END)) {
		/* The PHYEND event is called when fast switching is enabled
		 * instead of the DISABLE event.
		 * This event is handled in the analogous way to the disable event.
		 */
		if (on_radio_disabled) {
			on_radio_disabled();
		}
		nrf_radio_event_clear(NRF_RADIO, ESB_RADIO_EVENT_END);
	}

#if defined(CONFIG_ESB_FAST_CHANNEL_SWITCHING)
	if (nrf_radio_int_enable_check(NRF_RADIO, NRF_RADIO_INT_RXREADY_MASK) &&
	    nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_RXREADY)) {
		nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_RXREADY);
		if (atomic_test_and_clear_bit(&esb_addr.rf_channel_flags, RF_CHANNEL_UPDATE_FLAG)) {
			fast_switchinng_set_channel(esb_addr.rf_channel);
		}
	}
#endif /* defined(CONFIG_ESB_FAST_CHANNEL_SWITCHING) */
}

static void esb_evt_irq_handler(void)
{
	uint32_t interrupts;
	struct esb_evt event;

	get_and_clear_irqs(&interrupts);

	if (event_handler != NULL) {
		if (interrupts & INT_TX_SUCCESS_MSK) {
			event.evt_id = ESB_EVENT_TX_SUCCESS;
			event.tx = last_tx_evt;
			event_handler(&event);
		}
		if (interrupts & INT_TX_FAILED_MSK) {
			event.evt_id = ESB_EVENT_TX_FAILED;
			event_handler(&event);
		}
		if (interrupts & INT_RX_DATA_RECEIVED_MSK) {
			event.evt_id = ESB_EVENT_RX_RECEIVED;
			event_handler(&event);
		}
	}
}

#if IS_ENABLED(CONFIG_ESB_DYNAMIC_INTERRUPTS)

static void radio_dynamic_irq_handler(const void *args)
{
	ARG_UNUSED(args);
	radio_irq_handler();
	ISR_DIRECT_PM();
}

static void evt_dynamic_irq_handler(const void *args)
{
	ARG_UNUSED(args);
	if (IS_ENABLED(ESB_EVT_USING_EGU)) {
		nrf_egu_event_clear(ESB_EGU, ESB_EGU_EVT_EVENT);
	}
	esb_evt_irq_handler();
	ISR_DIRECT_PM();
}

static void timer_dynamic_irq_handler(const void *args)
{
	ARG_UNUSED(args);
	ESB_TIMER_IRQ_HANDLER();
	ISR_DIRECT_PM();
}

#else /* !IS_ENABLED(CONFIG_ESB_DYNAMIC_INTERRUPTS) */

ISR_DIRECT_DECLARE(esb_radio_direct_irq_handler)
{
	radio_irq_handler();

	ISR_DIRECT_PM();

	return 1;
}

ISR_DIRECT_DECLARE(esb_evt_direct_irq_handler)
{
	if (IS_ENABLED(ESB_EVT_USING_EGU)) {
		nrf_egu_event_clear(ESB_EGU, ESB_EGU_EVT_EVENT);
	}

	esb_evt_irq_handler();

	ISR_DIRECT_PM();

	return 1;
}

ISR_DIRECT_DECLARE(ESB_SYS_TIMER_IRQHandler)
{
	ISR_DIRECT_PM();

	return 1;
}

#endif /* IS_ENABLED(CONFIG_ESB_DYNAMIC_INTERRUPTS) */

static void initialize_fifos(void)
{
	init_tx();
	initialize_rx_fifo();
}

static void esb_irq_disable(void)
{
	irq_disable(ESB_RADIO_IRQ_NUMBER);
	irq_disable(ESB_EVT_IRQ_NUMBER);
	irq_disable(ESB_TIMER_IRQ);
}

int esb_init(const struct esb_config *config)
{
	int err;

	if (!config) {
		return -EINVAL;
	}

	if (esb_initialized) {
		esb_disable();
	}

	event_handler = config->event_handler;

	memcpy(&esb_cfg, config, sizeof(esb_cfg));

	if (fast_switching) {
		if (!esb_cfg.use_fast_ramp_up) {
			return -EINVAL;
		}
	}

	interrupt_flags = 0;

	memset(rx_pipe_info, 0, sizeof(rx_pipe_info));

	update_radio_parameters();

	/* Configure radio address registers according to ESB default values */
	nrf_radio_base0_set(NRF_RADIO, 0xE7E7E7E7);
	nrf_radio_base1_set(NRF_RADIO, 0x43434343);
	nrf_radio_prefix0_set(NRF_RADIO, 0x23C343E7);
	nrf_radio_prefix1_set(NRF_RADIO, 0x13E363A3);

	initialize_fifos();

	err = sys_timer_init();
	if (err) {
		LOG_ERR("Failed to initialize ESB system timer");
		return err;
	}

	err = esb_ppi_init();
	if (err) {
		LOG_ERR("Failed to initialize PPI");
		return err;
	}

	// disable_event.event.generic.event = esb_ppi_radio_disabled_get();

	nrf_radio_fast_ramp_up_enable_set(NRF_RADIO, esb_cfg.use_fast_ramp_up);

#if defined(CONFIG_ESB_FAST_CHANNEL_SWITCHING)
	nrf_radio_int_enable(NRF_RADIO, NRF_RADIO_INT_RXREADY_MASK);
#endif /* defined(CONFIG_ESB_FAST_CHANNEL_SWITCHING) */

#if IS_ENABLED(CONFIG_ESB_DYNAMIC_INTERRUPTS)

	/* Ensure IRQs are disabled before attaching. */
	esb_irq_disable();

	ARM_IRQ_DIRECT_DYNAMIC_CONNECT(ESB_RADIO_IRQ_NUMBER, CONFIG_ESB_RADIO_IRQ_PRIORITY, 0,
				       reschedule);
	ARM_IRQ_DIRECT_DYNAMIC_CONNECT(ESB_EVT_IRQ_NUMBER, CONFIG_ESB_EVENT_IRQ_PRIORITY, 0,
				       reschedule);
	ARM_IRQ_DIRECT_DYNAMIC_CONNECT(ESB_TIMER_IRQ, CONFIG_ESB_EVENT_IRQ_PRIORITY, 0, reschedule);

	irq_connect_dynamic(ESB_RADIO_IRQ_NUMBER, CONFIG_ESB_RADIO_IRQ_PRIORITY,
			    radio_dynamic_irq_handler, NULL, 0);
	irq_connect_dynamic(ESB_EVT_IRQ_NUMBER, CONFIG_ESB_EVENT_IRQ_PRIORITY,
			    evt_dynamic_irq_handler, NULL, 0);
	irq_connect_dynamic(ESB_TIMER_IRQ, CONFIG_ESB_EVENT_IRQ_PRIORITY, timer_dynamic_irq_handler,
			    NULL, 0);

#else /* !IS_ENABLED(CONFIG_ESB_DYNAMIC_INTERRUPTS) */

	IRQ_DIRECT_CONNECT(ESB_RADIO_IRQ_NUMBER, CONFIG_ESB_RADIO_IRQ_PRIORITY,
			   esb_radio_direct_irq_handler, 0);
	IRQ_DIRECT_CONNECT(ESB_EVT_IRQ_NUMBER, CONFIG_ESB_EVENT_IRQ_PRIORITY,
			   esb_evt_direct_irq_handler, 0);
	IRQ_DIRECT_CONNECT(ESB_TIMER_IRQ, CONFIG_ESB_RADIO_IRQ_PRIORITY, ESB_TIMER_IRQ_HANDLER, 0);

#endif /* IS_ENABLED(CONFIG_ESB_DYNAMIC_INTERRUPTS) */

	irq_enable(ESB_RADIO_IRQ_NUMBER);
	irq_enable(ESB_EVT_IRQ_NUMBER);
	if (IS_ENABLED(ESB_EVT_USING_EGU)) {
		nrf_egu_int_enable(ESB_EGU, ESB_EGU_EVT_INT);
	}
	irq_enable(ESB_TIMER_IRQ);

	esb_state = ESB_STATE_IDLE;
	esb_initialized = true;

	if (nrf52_errata_182()) {
		/* Check if the device is an nRF52832 Rev. 2. */
		/* Workaround for nRF52832 rev 2 errata 182 */
		*(volatile uint32_t *)0x4000173C |= (1 << 10);
	}

#if defined(CONFIG_SOC_SERIES_NRF54HX)
	/* Apply HMPAN-102 workaround for nRF54H series */
	*(volatile uint32_t *)0x5302C7E4 =
		(((*((volatile uint32_t *)0x5302C7E4)) & 0xFF000FFF) | 0x0012C000);

	/* Apply HMPAN-18 workaround for nRF54H series - load trim values*/
	if (*(volatile uint32_t *)0x0FFFE458 != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C734 = *(volatile uint32_t *)0x0FFFE458;
	}

	if (*(volatile uint32_t *)0x0FFFE45C != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C738 = *(volatile uint32_t *)0x0FFFE45C;
	}

	if (*(volatile uint32_t *)0x0FFFE460 != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C73C = *(volatile uint32_t *)0x0FFFE460;
	}

	if (*(volatile uint32_t *)0x0FFFE464 != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C740 = *(volatile uint32_t *)0x0FFFE464;
	}

	if (*(volatile uint32_t *)0x0FFFE468 != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C74C = *(volatile uint32_t *)0x0FFFE468;
	}

	if (*(volatile uint32_t *)0x0FFFE46C != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C7D8 = *(volatile uint32_t *)0x0FFFE46C;
	}

	if (*(volatile uint32_t *)0x0FFFE470 != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C840 = *(volatile uint32_t *)0x0FFFE470;
	}

	if (*(volatile uint32_t *)0x0FFFE474 != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C844 = *(volatile uint32_t *)0x0FFFE474;
	}

	if (*(volatile uint32_t *)0x0FFFE478 != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C848 = *(volatile uint32_t *)0x0FFFE478;
	}

	if (*(volatile uint32_t *)0x0FFFE47C != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C84C = *(volatile uint32_t *)0x0FFFE47C;
	}

	/* Apply HMPAN-103 workaround for nRF54H series*/
	if ((*(volatile uint32_t *)0x5302C8A0 == 0x80000000) ||
	    (*(volatile uint32_t *)0x5302C8A0 == 0x0058120E)) {
		*(volatile uint32_t *)0x5302C8A0 = 0x0058090E;
	}

	*(volatile uint32_t *)0x5302C8A4 = 0x00F8AA5F;
	*(volatile uint32_t *)0x5302C7AC = 0x8672827A;
	*(volatile uint32_t *)0x5302C7B0 = 0x7E768672;
	*(volatile uint32_t *)0x5302C7B4 = 0x0406007E;
#endif /* (CONFIG_SOC_SERIES_NRF54HX) */

#if CONFIG_ESB_CENTRAL
	central_setup();
#else
	peripheral_setup();
#endif
	// k_work_reschedule(&monitoring_work, K_SECONDS(1));

	return 0;
}

int esb_suspend(void)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	/*  Clear PPI */
	esb_ppi_disable_all();

	esb_state = ESB_STATE_IDLE;
	errata216_off();

	return 0;
}

void esb_disable(void)
{
	on_radio_disabled = NULL;

	esb_irq_disable();

	nrf_radio_shorts_disable(NRF_RADIO, 0xFFFFFFFF);
	nrf_radio_int_disable(NRF_RADIO, 0xFFFFFFFF);

	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);

	while (!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_DISABLED)) {
		/* wait for register to settle */
	}

	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);

	esb_ppi_disable_all();
	esb_fem_reset();

	sys_timer_deinit();
	esb_ppi_deinit();

	/* Radio ramp-up time to default mode */
	nrf_radio_fast_ramp_up_enable_set(NRF_RADIO, false);

	esb_state = ESB_STATE_IDLE;
	errata216_off();
	esb_initialized = false;

	memset(rx_pipe_info, 0, sizeof(rx_pipe_info));
}

bool esb_is_idle(void)
{
	return (esb_state == ESB_STATE_IDLE);
}

int esb_write_payload(const struct esb_payload *payload)
{
	if (!esb_initialized) {
		return -EACCES;
	}

	if (payload == NULL) {
		return -EINVAL;
	}

	if ((payload->length == 0) || (payload->length > CONFIG_ESB_MAX_PAYLOAD_LENGTH) ||
	    ((esb_cfg.protocol == ESB_PROTOCOL_ESB) &&
	     (payload->length > esb_cfg.payload_length))) {
		return -EMSGSIZE;
	}

	if (!tx_ok(payload->pipe, payload->length)) {
		return -ENOMEM;
	}

	if (resolve_pipe(payload->pipe) >= ESB_PIPE_COUNT) {
		return -EINVAL;
	}

	unsigned int key = irq_lock();

	put_tx(payload);

	irq_unlock(key);

#if !CONFIG_ESB_CENTRAL
	if (esb_state == ESB_STATE_PERIPHERAL_RX_READY) {
		peripheral_prepare_rx();
	}
#endif

	return 0;
}

int esb_tx_start(uint8_t pipe, uint32_t size)
{
	if (!tx_ok(pipe, size)) {
		return -ENOMEM;
	}

	tx_start(pipe);

	return 0;
}

int esb_tx_put(uint8_t *data, uint32_t size)
{
	tx_put(data, size);

	return 0;
}

int esb_tx_finish(void)
{
	tx_finish();

#if !CONFIG_ESB_CENTRAL
	if (esb_state == ESB_STATE_PERIPHERAL_RX_READY) {
		peripheral_prepare_rx();
	}
#endif

	return 0;
}

int esb_read_rx_payload(struct esb_payload *payload)
{
	if (!esb_initialized) {
		return -EACCES;
	}
	if (payload == NULL) {
		return -EINVAL;
	}

	if (rx_fifo.count == 0) {
		return -ENODATA;
	}

	unsigned int key = irq_lock();

	payload->length = rx_fifo.payload[rx_fifo.front]->length;
	payload->pipe = rx_fifo.payload[rx_fifo.front]->pipe;
	payload->rssi = rx_fifo.payload[rx_fifo.front]->rssi;
	payload->pid = rx_fifo.payload[rx_fifo.front]->pid;
	// payload->noack = rx_fifo.payload[rx_fifo.front]->noack;
	memcpy(payload->data, rx_fifo.payload[rx_fifo.front]->data, payload->length);

	if (++rx_fifo.front >= CONFIG_ESB_RX_FIFO_SIZE) {
		rx_fifo.front = 0;
	}

	rx_fifo.count--;

	irq_unlock(key);

	return 0;
}

int esb_tdma_start(void)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	LOG_WRN("tdma started");

#if CONFIG_ESB_CENTRAL
	NVIC_ClearPendingIRQ(ESB_TIMER_IRQ);
	irq_enable(ESB_TIMER_IRQ);
#else
	NVIC_ClearPendingIRQ(ESB_RADIO_IRQ_NUMBER);
	irq_enable(ESB_RADIO_IRQ_NUMBER);
#endif

#if CONFIG_ESB_CENTRAL
	central_prepare_tx();
#else // ESB_PERIPHERAL
	peripheral_start_desync();
#endif
	nrfx_timer_enable(&esb_timer);
	return 0;
}

int esb_tdma_stop(bool force)
{
#if !CONFIG_ESB_CENTRAL
	if (esb_state != ESB_STATE_PERIPHERAL_DESYNC && !force) {
		return -EINVAL;
	}
#endif

	on_radio_disabled = NULL;

	LOG_WRN("tdma stopped");
	nrfx_timer_disable(&esb_timer);

#if CONFIG_ESB_CENTRAL
	irq_disable(ESB_TIMER_IRQ);
#else
	irq_disable(ESB_RADIO_IRQ_NUMBER);
#endif
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);

	esb_state = ESB_STATE_IDLE;
	pto_context_reset();
	reset_tx_all();
	reset_rx_fifo();
}

static void central_prepare_tx(void)
{
	// LOG_WRN("start_tx_transaction");
	uint32_t now = nrfy_timer_capture_get(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL3);
	uint32_t slot_diff = (now - ctx.start) / ctx.slotsize + 1;
	uint32_t tx_start = ctx.start + (slot_diff * ctx.slotsize);

	if (tx_start - now < RADIO_MARGIN) {
		slot_diff++;
		tx_start += ctx.slotsize;
	}

	uint32_t next_slot = ctx.refslot + slot_diff;
	uint32_t timeout = tx_start + ctx.slotsize - TIMEOUT_MARGIN;

	nrf_timer_cc_set(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL0, tx_start);
	nrf_timer_cc_set(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL1, timeout);
	ctx.start = tx_start;
	ctx.timeout = timeout;
	ctx.refslot = next_slot;
	ctx.pipe = (ctx.pipe + slot_diff) % ctx.pipes;

	uint8_t pipe = ctx.pipe;
	struct pipe_info *pipe_info = rx_pipe_info_get(pipe);
	struct esb_radio_pdu *pdu = (struct esb_radio_pdu *)rx_payload_buffer;
	struct esb_packet_dn *tx_packet = (struct esb_packet_dn *)pdu->data;

	bool slot_synced = is_slot_synced(pipe);
	uint8_t packet_length = sizeof(struct esb_header_dn);

	// if slot is out of sync, then send refslot
	if (!slot_synced) {
		packet_length += sizeof(struct esb_ctrl_packet);
		set_control_packet(tx_packet->data);
		pdu->pdu.ctrl = true;
	}
	// if next packet can be transmitted, then read txbuf
	else {
		packet_length += copy_tx(pipe, tx_packet->data);
		pipe_info->tx_try = packet_length > sizeof(struct esb_header_dn) ? 1 : 0;
		pdu->pdu.ctrl = false;
	}

	pdu->pdu.length = packet_length;
	pdu->pdu.sn = pipe_info->sn;
	pdu->pdu.nesn = pipe_info->nesn;

	esb_addr.rf_channel = get_channel(next_slot);
	tx_packet->header.tx_power = esb_cfg.tx_output_power;

	on_timer_compare1 = central_timeslot_end;

	update_rf_payload_format(packet_length);
	nrf_radio_txaddress_set(NRF_RADIO, pipe);
	nrf_radio_rxaddresses_set(NRF_RADIO, BIT(pipe));
	nrf_radio_frequency_set(NRF_RADIO, (RADIO_BASE_FREQUENCY + esb_addr.rf_channel));

	update_radio_tx_power();

	nrf_radio_packetptr_set(NRF_RADIO, pdu);

	pto_ppi_for_central_tx_set();

	esb_state = ESB_STATE_CENTRAL_TX;

	if (nrf_timer_event_check(esb_timer.p_reg, NRF_TIMER_EVENT_COMPARE0) &&
	    (!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_TXREADY))) {
		nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_TXEN);
	}

	// LOG_WRN("PIPE %u NOW %u TX %u TO %u SL %u CH %u LEN %u", pipe, now,
	// tx_start, timeout, 	next_slot, esb_addr.rf_channel, packet_length);

	// LOG_WRN("PIPE %u SLOT %u CH %u RSSI %d LEN %u SN %d %d SYNC %d CTRL %d ",
	// pipe, next_slot, 	esb_addr.rf_channel, (int)(buf->header.rssi),
	// packet_length, pipe_info->sn, 	pipe_info->nesn, slot_synced,
	// pdu->pdu.ctrl);
}

static void central_timeslot_end(void)
{
	uint8_t pipe = ctx.pipe;
	uint8_t channel_idx = ctx.channel_idx;
	struct esb_radio_pdu *rx_pdu = (struct esb_radio_pdu *)rx_payload_buffer;
	struct esb_packet *rx_packet = (struct esb_packet *)rx_pdu->data;
	struct pipe_info *pipe_info = rx_pipe_info_get(pipe);

	pto_ppi_for_central_tx_clear();
	nrf_timer_event_clear(esb_timer.p_reg, NRF_TIMER_EVENT_COMPARE0);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_TXREADY);
	/* Just clear LNA configuration and disable front-end module. */
	mpsl_fem_lna_configuration_clear();
	mpsl_fem_disable();

	// check timeout
	uint32_t crcok_event = nrf_timer_cc_get(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL2);
	bool tx_failed = (ctx.timeout - crcok_event) > ctx.slotsize;
	bool synced_before = is_slot_synced(pipe);

	if (tx_failed) {
		if (synced_before) {
			if (pipe_info->tx_try > 0) {
				pipe_info->tx_try++;
			}

			if (!slot_sync_check(pipe, ctx.start)) {
				rssi_reset(pipe);
				set_sync_evt_interrupt(pipe, false);
			}
		}

		central_prepare_tx();
		return;
	}

	uint8_t rx_sn = rx_pdu->pdu.sn;
	uint8_t tx_nesn = pipe_info->nesn;
	uint8_t rx_nesn = rx_pdu->pdu.nesn;
	uint8_t tx_sn = pipe_info->sn;
	uint8_t tx_try = pipe_info->tx_try;
	bool ctrl = rx_pdu->pdu.ctrl;

	if (tx_try > 0 && rx_nesn != tx_sn) {
		pop_tx(pipe);
		pipe_info->sn = (!pipe_info->sn);
		set_tx_evt_interrupt(pipe, true);
	}

	slot_sync_update(pipe, ctx.start);

	if (!synced_before) {
		set_sync_evt_interrupt(pipe, true);
	}

	uint32_t crc = nrf_radio_rxcrc_get(NRF_RADIO);
	uint8_t rx_len = rx_pdu->pdu.length;
	uint8_t rssi = nrf_radio_rssi_sample_get(NRF_RADIO);
	bool retransmit_payload = (pipe_info->crc == crc) && (tx_nesn != rx_sn);
	bool send_rx_event = !retransmit_payload;

	rssi_update(pipe, rssi);
	if (send_rx_event) {
		if (ctrl) {
			pipe_info->crc = crc;
			pipe_info->nesn = (!pipe_info->nesn);
		} else if (push_rx_fifo(pipe, rx_sn, rx_len, rx_pdu->data)) {
			pipe_info->crc = crc;
			pipe_info->nesn = (!pipe_info->nesn);
			set_rx_evt_interrupt();
		}
	}

	// prepare for next slot
	central_prepare_tx();
#if 0
	LOG_WRN("pipe %u rx_len %u rssi -%u crc %lx tx[%d %d] rx[%d %d] r %d s %d", pipe, rx_len,
		rssi, crc, tx_sn, rx_nesn, tx_nesn, rx_sn, retransmit_payload,
		(retransmit_payload && rx_len > 0));
#endif
}

static void set_rx_packetptr(void)
{
	update_rf_payload_format(esb_cfg.payload_length);
	nrf_radio_packetptr_set(NRF_RADIO, rx_payload_buffer);
}

static void peripheral_start_desync(void)
{
	uint32_t airtime = MIN(ctx.slotsize * ctx.pipes * 200, HEARTBEAT_INTERVAL);
	airtime = (airtime != 0 ? airtime : DESYNC_AIRTIME_DEFAULT);
	uint32_t start = ctx.timeout + HEARTBEAT_INTERVAL - airtime;
	uint32_t timeout = start + airtime;
	nrf_timer_cc_set(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL0, start);
	nrf_timer_cc_set(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL1, timeout);
	ctx.timeout = timeout;

	if ((esb_state != ESB_STATE_PERIPHERAL_DESYNC) && (esb_state != ESB_STATE_IDLE)) {
		// LOG_WRN("state: %d", esb_state);
		set_sync_evt_interrupt(0, false);
	}

	esb_state = ESB_STATE_PERIPHERAL_DESYNC;
	on_radio_disabled = peripheral_disabled_desync;

	esb_addr.rf_channel = get_channel(sys_rand32_get());
	nrf_radio_frequency_set(NRF_RADIO, (RADIO_BASE_FREQUENCY + esb_addr.rf_channel));

	set_rx_packetptr();

	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_CRCOK);

	pto_ppi_for_peripheral_start_rx_desync_set();
	// LOG_WRN("start rx desync CHAN(%u), DESYNC(%u) PIPE(%u)", esb_addr.rf_channel,
	// 	ctx.desync_count, ctx.pipe);
}

static void peripheral_disabled_desync(void)
{
	struct esb_radio_pdu *pdu = (struct esb_radio_pdu *)rx_payload_buffer;
	struct esb_packet_dn *packet = (struct esb_packet_dn *)pdu->data;
	struct pipe_info *pipe_info = rx_pipe_info_get(0);
	pto_ppi_for_peripheral_start_rx_desync_clear();
	bool ctrl = pdu->pdu.ctrl;
	if (!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_CRCOK) || !ctrl) {
		peripheral_start_desync();
		return;
	}

	drift_reset();

	bool rx_sn = pdu->pdu.sn;
	bool rx_nesn = pdu->pdu.nesn;
	pipe_info->sn = !rx_nesn;
	pipe_info->nesn = !rx_sn;
	pipe_info->tx_try = 0;
	ctx.desync_count = 0;

	apply_control_packet(packet->data);

	ctx.last_hb = nrf_timer_cc_get(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL2) - ctx.addr_delay;

	set_sync_evt_interrupt(ctx.pipe, true);

	// LOG_WRN("disabled rx desync: REF(%u) SYNC(%u) SN[%d %d] CTRL %d", ctx.refslot,
	// ctx.last_hb, 	rx_sn, rx_nesn, ctrl);

	peripheral_prepare_rx();
}

static void peripheral_prepare_rx(void)
{
	struct pipe_info *pipe_info = rx_pipe_info_get(0);

	pto_ppi_for_peripheral_prepare_rx_clear();

	uint32_t loop_size = ctx.slotsize * ctx.pipes;
	uint32_t hb_size = ctx.hb_loops * loop_size;
	uint32_t now = nrfy_timer_capture_get(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL0);
	uint32_t hb_passed = (now - ctx.last_hb) / hb_size;
	uint32_t hb_loops_passed = (now - ctx.last_hb) / loop_size;
	uint32_t loops_diff = 0;

	// if we don't have any payload and packet hasn't been sent, then prepare
	// sync packet.
	bool send_data = (count_tx(0) > 0) || (pipe_info->tx_try > 0);
	if (!send_data && ctx.timeout_count == 0) {
		loops_diff = (hb_passed + 1) * ctx.hb_loops;
	}
	// we have to check right after packet is sent.
	else {
		loops_diff = hb_loops_passed + 1;
	}

	// calculate estimated tx start time then add diff
	uint32_t diff = loops_diff * loop_size;
	if (((ctx.last_hb + diff) - now) < RADIO_MARGIN) {
		loops_diff++;
		diff += loop_size;
	}

	ctx.is_hb = (loops_diff % ctx.hb_loops) == 0;

	int32_t drift = drift_get(diff);
	uint32_t rx_start = ctx.last_hb + diff + drift;
	uint32_t rx_timeout = rx_start + ctx.window_size;
	ctx.timeout = rx_timeout;

	ctx.slotdiff = loops_diff * ctx.pipes;
	uint32_t slot = ctx.refslot + ctx.slotdiff;
	esb_addr.rf_channel = get_channel(slot);
	nrf_radio_frequency_set(NRF_RADIO, (RADIO_BASE_FREQUENCY + esb_addr.rf_channel));

	nrf_timer_cc_set(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL0, rx_start);
	nrf_timer_cc_set(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL1, rx_timeout);

	esb_state = (ctx.is_hb && (!send_data)) ? ESB_STATE_PERIPHERAL_RX_READY
						: ESB_STATE_PERIPHERAL_RX;

	pto_ppi_for_peripheral_prepare_rx_set();

	on_radio_disabled = peripheral_disabled_rx;

	if (nrf_timer_event_check(esb_timer.p_reg, NRF_TIMER_EVENT_COMPARE0) &&
	    !nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_RXREADY)) {
		nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_RXEN);
	}

	// LOG_WRN("now %u start %u slot %u ch %u drift %ld hb %d tx %d dsync %u", now, rx_start,
	// slot, 	esb_addr.rf_channel, drift, ctx.is_hb, tx_power_get(ctx.channel_idx),
	// 	ctx.desync_count);
}

static void peripheral_disabled_rx(void)
{
	struct esb_radio_pdu *rx_pdu = (struct esb_radio_pdu *)rx_payload_buffer;
	struct esb_packet_dn *rx_packet = (struct esb_packet_dn *)rx_pdu->data;
	struct esb_radio_pdu *tx_pdu = (struct esb_radio_pdu *)tx_payload_buffer;
	struct esb_packet_up *tx_packet = (struct esb_packet_up *)tx_pdu->data;
	uint32_t flags = 0;

	// check timeout or crc error
	uint32_t sync = nrf_timer_cc_get(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL2) - ctx.addr_delay;
	uint32_t crcok = nrf_timer_cc_get(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL3);

	bool is_timeout = (ctx.timeout - crcok) > ctx.slotsize;
	if (is_timeout) {
		// LOG_WRN("timeout");
		if (++ctx.timeout_count > DESYNC_COUNT_MAX) {
			// something's wrong, disable radio and goto desync state
			pto_ppi_for_peripheral_prepare_rx_clear();
			peripheral_start_desync();
		} else {
			peripheral_prepare_rx();
		}

		return;
	}

	esb_state = ESB_STATE_PERIPHERAL_TX_ACK;

	// pto_ppi_for_peripheral_prepare_rx_clear();
	struct pipe_info *pipe_info = rx_pipe_info_get(0);

	// update TX
	uint8_t rx_sn = rx_pdu->pdu.sn;
	uint8_t tx_nesn = pipe_info->nesn;
	uint8_t rx_nesn = rx_pdu->pdu.nesn;
	uint8_t tx_sn = pipe_info->sn;
	uint8_t tx_try = pipe_info->tx_try;

	// bool tx_failed = (tx_try > 0) && (tx_sn == rx_nesn);
	bool tx_failed = (tx_sn == rx_nesn);
	bool retransmit_payload = (tx_nesn != rx_sn);

	int rx_len = rx_pdu->pdu.length;
	bool send_rx_event = !retransmit_payload && rx_len > sizeof(struct esb_header_dn);

	// we toggle nesn here before pushing rxbuf
	if (send_rx_event && rx_fifo.count < CONFIG_ESB_RX_FIFO_SIZE) {
		pipe_info->nesn = (!pipe_info->nesn);
	}

	uint8_t rssi = nrf_radio_rssi_sample_get(NRF_RADIO);
	rssi_update(0, rssi);

	uint8_t tx_len = tx_pdu->pdu.length;
	if (tx_failed) {
		if (++pipe_info->tx_try > DESYNC_COUNT_MAX) {
			set_tx_evt_interrupt(0, false);
			pto_ppi_for_peripheral_prepare_rx_clear();
			peripheral_start_desync();
			return;
		}

		tx_power_raise(0);
	} else {
		// if packet before was payload, then tx was successful.
		if (tx_try > 0) {
			set_tx_evt_interrupt(0, true);
			uint32_t popped = pop_tx(0);
		}

		tx_len = copy_tx(0, tx_pdu->data);
		pipe_info->tx_try = tx_len > 0;
	}

	bool tx_triggered = tx_len > 0 || ctx.is_hb || tx_failed;
	if (tx_triggered) {
		if (!tx_failed) {
			pipe_info->sn = !pipe_info->sn;
		}

		tx_pdu->pdu.ctrl = tx_len == 0;
		tx_pdu->pdu.length = tx_len;
		tx_pdu->pdu.sn = pipe_info->sn;
		tx_pdu->pdu.nesn = pipe_info->nesn;
		update_rf_payload_format(tx_pdu->pdu.length);
		nrf_radio_packetptr_set(NRF_RADIO, tx_pdu);

		esb_fem_for_tx_ack();
		if (ctx.is_hb) {
			int8_t tx_power = rx_packet->header.tx_power;
			tx_power_update(ctx.pipe, tx_power);
			esb_cfg.tx_output_power = tx_power_get(ctx.pipe);
			update_radio_tx_power();
		}

		nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_TXEN);
	}

	// set sync
	ctx.timeout_count = 0;
	on_radio_disabled = peripheral_disabled_tx_ack;
	int64_t hb_size = ctx.hb_loops * ctx.slotsize * ctx.pipes;
	int64_t drift = (int64_t)(sync - ctx.last_hb) - hb_size;
	if (ctx.is_hb) {
		if (abs(drift) <= DRIFT_LIMIT) {
			drift_update(drift);
		}

		ctx.last_hb = sync;
		ctx.refslot += ctx.slotdiff;
	}

	// push RX
	if (send_rx_event) {
		if (rx_pdu->pdu.ctrl == true) {
			//
		} else if (push_rx_fifo(0, rx_sn, rx_len - sizeof(struct esb_header_dn),
					rx_packet->data)) {
			set_rx_evt_interrupt();
		}
	}

	if (!tx_triggered) {
		peripheral_disabled_tx_ack();
	}

#if 0
	LOG_WRN("len [%u %d] tx[%d %d] rx[%d %d] t %d r %d s %d drift %lld rssi %d TR %d", tx_len,
		rx_len, tx_sn, rx_nesn, tx_nesn, rx_sn, pipe_info->tx_try, retransmit_payload,
		send_rx_event, drift, (int)(-rssi), tx_triggered);
#endif
}

static void peripheral_disabled_tx_ack(void)
{
	// LOG_WRN("disabled rx ack");
	nrf_timer_event_clear(esb_timer.p_reg, NRF_TIMER_EVENT_COMPARE0);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_RXREADY);

	set_rx_packetptr();
	peripheral_prepare_rx();
}

int esb_set_address_length(uint8_t length)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (!((length > 2) && (length < 6))) {
		return -EINVAL;
	}

	esb_addr.addr_length = length;

	update_rf_payload_format(esb_cfg.payload_length);

	return 0;
}

int esb_set_base_address_0(const uint8_t *addr)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (addr == NULL) {
		return -EINVAL;
	}

	memcpy(esb_addr.base_addr_p0, addr, sizeof(esb_addr.base_addr_p0));

	update_radio_addresses(ADDR_UPDATE_MASK_BASE0);

	return 0;
}

int esb_set_base_address_1(const uint8_t *addr)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (addr == NULL) {
		return -EINVAL;
	}

	memcpy(esb_addr.base_addr_p1, addr, sizeof(esb_addr.base_addr_p1));

	update_radio_addresses(ADDR_UPDATE_MASK_BASE1);

	return 0;
}

int esb_set_prefixes(const uint8_t *prefixes, uint8_t num_pipes)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (prefixes == NULL) {
		return -EINVAL;
	}

	if (!(num_pipes <= CONFIG_ESB_PIPE_COUNT)) {
		return -EINVAL;
	}

	memcpy(esb_addr.pipe_prefixes, prefixes, num_pipes);

	esb_addr.num_pipes = num_pipes;
	esb_addr.rx_pipes_enabled = BIT_MASK_UINT_8(num_pipes);

	update_radio_addresses(ADDR_UPDATE_MASK_PREFIX);

	return 0;
}

int esb_update_prefix(uint8_t pipe, uint8_t prefix)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	if (pipe >= CONFIG_ESB_PIPE_COUNT) {
		return -EINVAL;
	}

	esb_addr.pipe_prefixes[pipe] = prefix;

	update_radio_addresses(ADDR_UPDATE_MASK_PREFIX);

	return 0;
}

int esb_enable_pipes(uint8_t enable_mask)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	if ((enable_mask | BIT_MASK_UINT_8(CONFIG_ESB_PIPE_COUNT)) !=
	    BIT_MASK_UINT_8(CONFIG_ESB_PIPE_COUNT)) {
		return -EINVAL;
	}

	esb_addr.rx_pipes_enabled = enable_mask;

	return 0;
}

int esb_set_rf_channel(uint32_t channel)
{
	if (channel > 100) {
		return -EINVAL;
	}

	if (esb_state != ESB_STATE_IDLE) {
		// if (IS_ENABLED(CONFIG_ESB_FAST_CHANNEL_SWITCHING)) {
		// 	if (esb_state == ESB_STATE_PRX) {
		// 		fast_switchinng_set_channel(channel);
		// 	} else {
		// 		atomic_set_bit(&esb_addr.rf_channel_flags,
		// RF_CHANNEL_UPDATE_FLAG);
		// 	}
		// } else {
		// 	return -EBUSY;
		// }
		return -EBUSY;
	}

	esb_addr.rf_channel = channel;

	return 0;
}

int esb_get_rf_channel(uint32_t *channel)
{
	if (channel == NULL) {
		return -EINVAL;
	}

	*channel = esb_addr.rf_channel;

	return 0;
}

int esb_set_tx_power(int8_t tx_output_power)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	esb_cfg.tx_output_power = tx_output_power;

	return 0;
}

int esb_set_retransmit_delay(uint16_t delay)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (delay < RETRANSMIT_DELAY_MIN) {
		return -EINVAL;
	}

	esb_cfg.retransmit_delay = delay;

	return 0;
}

int esb_set_retransmit_count(uint16_t count)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	esb_cfg.retransmit_count = count;

	return 0;
}

int esb_set_bitrate(enum esb_bitrate bitrate)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	esb_cfg.bitrate = bitrate;

	return update_radio_bitrate() ? 0 : -EINVAL;
}

int esb_conn_cb_register(struct esb_conn_cb *cb)
{
	sys_slist_append(&esb_conn_cb_list, &cb->node);
	return 0;
}
