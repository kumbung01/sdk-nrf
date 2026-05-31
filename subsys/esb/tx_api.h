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
int tx_ok(uint8_t pipe, uint32_t size);
int put_tx(const struct esb_payload *payload);
int copy_tx(uint8_t pipe, uint8_t *buffer);
int pop_tx(uint8_t pipe);
void reset_tx(uint8_t pipe);
void reset_tx_all(void);
void init_tx(void);