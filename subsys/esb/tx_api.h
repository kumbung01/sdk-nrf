#include <zephyr/kernel.h>
#include <esb.h>

inline uint8_t resolve_pipe(uint8_t pipe)
{
#if CONFIG_ESB_CENTRAL
	__ASSERT_NO_MSG(pipe < ESB_PIPE_COUNT);

	return pipe;
#else
	return 0;
#endif
}

int count_tx(uint8_t pipe);
int count_tx_net(uint8_t pipe);
int tx_ok(uint8_t pipe, uint32_t size);
int put_tx(const struct esb_payload *payload);
int copy_tx(uint8_t pipe, uint8_t *buffer);
int pop_tx(uint8_t pipe);
void reset_tx(uint8_t pipe);
void reset_tx_all(void);
void init_tx(void);

#if CONFIG_ESB_TX_RINGBUF
// api for tx ringbuf
void tx_start(uint8_t pipe);
void tx_put(uint8_t *data, uint32_t size);
void tx_finish(void);
#endif