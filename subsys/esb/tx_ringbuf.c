#include <zephyr/kernel.h>
#include "tx_api.h"

static struct tx_buffer {
	struct ring_buf data;
	uint32_t peeked;
	uint32_t key;
} txbuf[ESB_PIPE_COUNT];

int count_tx(uint8_t pipe)
{
	return ring_buf_size_get(&txbuf[resolve_pipe(pipe)].data);
}

int copy_tx(uint8_t pipe, uint8_t *data)
{
	struct tx_buffer *buf = &txbuf[resolve_pipe(pipe)];

	buf->peeked = ring_buf_peek(&buf->data, data, CONFIG_ESB_MAX_PAYLOAD_LENGTH);
	return buf->peeked;
}

int pop_tx(uint8_t pipe)
{
	uint8_t data[CONFIG_ESB_MAX_PAYLOAD_LENGTH];

	struct tx_buffer *buf = &txbuf[resolve_pipe(pipe)];

	uint32_t popped = ring_buf_get(&buf->data, data, buf->peeked);
	buf->peeked -= popped;
	return popped;
}

int put_tx(const struct esb_payload *payload)
{

	uint32_t size = payload->length;

	struct tx_buffer *buf = &txbuf[resolve_pipe(payload->pipe)];

	if (ring_buf_space_get(&buf->data) < size) {
		return -ENOSPC;
	}

	ring_buf_put(&buf->data, payload->data, size);

	return 0;
}

int tx_ok(uint8_t pipe, uint32_t size)
{
	return ring_buf_space_get(&txbuf[resolve_pipe(pipe)].data) >= size;
}

void init_tx(void)
{
	static uint8_t tx_data[ESB_PIPE_COUNT][CONFIG_ESB_TX_RINGBUF_SIZE];

	for (int i = 0; i < ESB_PIPE_COUNT; ++i) {
		ring_buf_init(&txbuf[i].data, sizeof(*tx_data), tx_data[i]);
	}
}

void reset_tx(uint8_t pipe)
{
	struct tx_buffer *buf = &txbuf[resolve_pipe(pipe)];

	ring_buf_reset(&buf->data);
	buf->peeked = 0;
}

void reset_tx_all(void)
{
	for (int i = 0; i < ESB_PIPE_COUNT; ++i) {
		struct tx_buffer *buf = &txbuf[i];
		ring_buf_reset(&buf->data);
		buf->peeked = 0;
	}
}

static uint32_t key;
static struct tx_buffer *buf;
void tx_start(uint8_t pipe)
{
	key = irq_lock();
	buf = &txbuf[resolve_pipe(pipe)];
}

void tx_put(uint8_t *data, uint32_t size)
{
	if (buf == NULL) {
		return;
	}

	ring_buf_put(buf, data, size);
}

void tx_finish(void)
{
	buf = NULL;
	irq_unlock(key);
}