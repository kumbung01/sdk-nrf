#
# Copyright (c) 2019 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_library()
zephyr_library_include_directories(${ZEPHYR_NRF_MODULE_DIR}/include)
zephyr_library_sources(esb.c)

zephyr_library_sources_ifdef(CONFIG_ESB_TX_RINGBUF tx_ringbuf.c)
zephyr_library_sources_ifndef(CONFIG_ESB_TX_RINGBUF tx_fifo.c)

zephyr_library_sources_ifdef(CONFIG_HAS_HW_NRF_PPI esb_ppi.c)
zephyr_library_sources_ifndef(CONFIG_HAS_HW_NRF_PPI esb_dppi.c)
