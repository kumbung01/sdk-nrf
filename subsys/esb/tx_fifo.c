#include "tx_api.h"
#include <esb.h>

/* Structure used by the PRX to organize ACK payloads for multiple pipes. */
struct payload_wrap {
	/* Pointer to the ACK payload. */
	struct esb_payload *p_payload;
	/* Value used to determine if the current payload pointer is used. */
	bool in_use;
	/* Pointer to the next ACK payload queued on the same pipe. */
	struct payload_wrap *p_next;
};

/* First-in, first-out queue of payloads to be transmitted. */
struct payload_tx_fifo {
	/* Payload queue */
	struct esb_payload *payload[CONFIG_ESB_TX_FIFO_SIZE];

	uint32_t back;	/* Back of the queue (last in). */
	uint32_t front; /* Front of queue (first out). */
	uint32_t count; /* Number of elements in the queue. */
};

/* FIFOs and buffers */

static struct payload_tx_fifo tx_fifo;

#if CONFIG_ESB_CENTRAL
/* Random access buffer variables for ACK payload handling */
static struct payload_wrap ack_pl_wrap[CONFIG_ESB_TX_FIFO_SIZE];
static struct payload_wrap *ack_pl_wrap_pipe[ESB_PIPE_COUNT];

static uint32_t count_pl_wrap(void)
{
	return tx_fifo.count;
}

static void reset_pl_wrap(uint8_t pipe)
{
	struct payload_wrap *wrap = ack_pl_wrap_pipe[resolve_pipe(pipe)];
	while (wrap != NULL) {
		wrap->in_use = false;
		wrap = wrap->p_next;
	}

	ack_pl_wrap_pipe[resolve_pipe(pipe)] = NULL;
}

static void init_pl_wrap(void)
{
	static struct esb_payload tx_payload[CONFIG_ESB_TX_FIFO_SIZE];

	for (size_t i = 0; i < CONFIG_ESB_TX_FIFO_SIZE; i++) {
		ack_pl_wrap[i].p_payload = &tx_payload[i];
		ack_pl_wrap[i].in_use = false;
		ack_pl_wrap[i].p_next = NULL;
	}

	for (size_t i = 0; i < ESB_PIPE_COUNT; i++) {
		ack_pl_wrap_pipe[i] = NULL;
	}
}

static uint8_t copy_pl_wrap(uint8_t pipe, uint8_t *buffer)
{
	struct payload_wrap *wrap = ack_pl_wrap_pipe[resolve_pipe(pipe)];
	uint8_t tx_len = 0;

	if (wrap == NULL) {
		return 0;
	}

	struct esb_payload *payload = wrap->p_payload;
	if (payload) {
		tx_len = payload->length;
		memcpy(buffer, payload->data, tx_len);
	}

	return tx_len;
}

static uint8_t pop_pl_wrap(uint8_t pipe)
{
	struct payload_wrap *wrap = ack_pl_wrap_pipe[resolve_pipe(pipe)];

	if (wrap == NULL) {
		return 0;
	}

	uint8_t len = wrap->p_payload->length;

	wrap->in_use = false;
	wrap = wrap->p_next;
	ack_pl_wrap_pipe[resolve_pipe(pipe)] = wrap;

	return len;
}

static struct payload_wrap *find_free_payload_cont(void)
{
	for (int i = 0; i < CONFIG_ESB_TX_FIFO_SIZE; i++) {
		if (!ack_pl_wrap[i].in_use) {
			return &ack_pl_wrap[i];
		}
	}

	return 0;
}

static int put_pl_wrap(const struct esb_payload *payload)
{
	struct payload_wrap *new_ack_payload = find_free_payload_cont();

	if (new_ack_payload != 0) {
		new_ack_payload->in_use = true;
		new_ack_payload->p_next = NULL;
		memcpy(new_ack_payload->p_payload, payload, sizeof(struct esb_payload));

		if (ack_pl_wrap_pipe[payload->pipe] == NULL) {
			ack_pl_wrap_pipe[payload->pipe] = new_ack_payload;
		} else {
			struct payload_wrap *pl = ack_pl_wrap_pipe[payload->pipe];

			while (pl->p_next != NULL) {
				pl = (struct payload_wrap *)pl->p_next;
			}
			pl->p_next = (struct payload_wrap *)new_ack_payload;
		}
		tx_fifo.count++;
	}

	return 0;
}

#else

static uint32_t count_tx_fifo(void)
{
	return tx_fifo.count;
}

static void reset_tx_fifo(void)
{
	tx_fifo.front = 0;
	tx_fifo.back = 0;
	tx_fifo.count = 0;
}

static void init_tx_fifo(void)
{
	static struct esb_payload tx_payload[CONFIG_ESB_TX_FIFO_SIZE];

	for (size_t i = 0; i < CONFIG_ESB_TX_FIFO_SIZE; i++) {
		tx_fifo.payload[i] = &tx_payload[i];
	}

	reset_tx_fifo();
}

static uint8_t copy_tx_fifo(uint8_t *buffer)
{
	uint8_t tx_len = 0;

	if (tx_fifo.count > 0) {
		struct esb_payload *payload = tx_fifo.payload[tx_fifo.front];
		memcpy(buffer, payload->data, payload->length);
		tx_len = payload->length;
	}

	return tx_len;
}

static uint8_t pop_tx_fifo(void)
{
	if (tx_fifo.count == 0) {
		return 0;
	}

	unsigned int key = irq_lock();

	tx_fifo.count--;
	if (++tx_fifo.front >= CONFIG_ESB_TX_FIFO_SIZE) {
		tx_fifo.front = 0;
	}

	irq_unlock(key);

	return 1;
}

static int put_tx_fifo(const struct esb_payload *payload)
{
	if (tx_fifo.count >= CONFIG_ESB_TX_FIFO_SIZE) {
		return -ENOBUFS;
	}

	memcpy(tx_fifo.payload[tx_fifo.back], payload, sizeof(struct esb_payload));

	if (++tx_fifo.back >= CONFIG_ESB_TX_FIFO_SIZE) {
		tx_fifo.back = 0;
	}

	tx_fifo.count++;

	return 0;
}

#endif // CONFIG_ESB_CENTRAL
int count_tx(uint8_t pipe)
{
#if CONFIG_ESB_CENTRAL
	return count_pl_wrap();
#else
	return count_tx_fifo();
#endif
}

int tx_ok(uint8_t pipe, uint32_t size)
{
	return count_tx(pipe) < CONFIG_ESB_TX_FIFO_SIZE;
}

int copy_tx(uint8_t pipe, uint8_t *buffer)
{
#if CONFIG_ESB_CENTRAL
	return copy_pl_wrap(pipe, buffer);
#else
	return copy_tx_fifo(buffer);
#endif
}

int pop_tx(uint8_t pipe)
{
#if CONFIG_ESB_CENTRAL
	return pop_pl_wrap(pipe);
#else
	return pop_tx_fifo();
#endif
}

int put_tx(const struct esb_payload *payload)
{
#if CONFIG_ESB_CENTRAL
	return put_pl_wrap(payload);
#else
	return put_tx_fifo(payload);
#endif
}

void reset_tx(uint8_t pipe)
{
#if CONFIG_ESB_CENTRAL
	reset_pl_wrap(pipe);
#else
	reset_tx_fifo();
#endif
}

void reset_tx_all(void)
{
#if CONFIG_ESB_CENTRAL
	init_pl_wrap();
#else
	reset_tx_fifo();
#endif
}

void init_tx(void)
{
#if CONFIG_ESB_CENTRAL
	init_pl_wrap();
#else
	init_tx_fifo();
#endif
}
