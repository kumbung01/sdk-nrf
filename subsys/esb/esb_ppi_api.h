/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ESB_FEM_H__
#define ESB_FEM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

void esb_ppi_for_fem_set(void);

/** @brief Clear PPIs/DDPIs for the external front-end module.
 */
void esb_ppi_for_fem_clear(void);

uint32_t esb_ppi_radio_disabled_get(void);

/** @brief Disable all PPI/DPPI channels used by ESB.
 */
void esb_ppi_disable_all(void);

/** @brief Initialize PPIs/DPPIs for ESB.
 *
 * This function allocates PPI/DPPI channels for the ESB protocol.
 *
 * @retval If the operation was successful.
 *         Otherwise, a (negative) error code is returned.
 */
int esb_ppi_init(void);

/** @brief Deinitialize PPIs/DPPIs.
 *
 * This function frees the PPI/DPPI channels allocated by ESB.
 */
void esb_ppi_deinit(void);

void pto_ppi_for_central_tx_set(void);
void pto_ppi_for_central_tx_clear(void);
void pto_ppi_for_peripheral_start_desync_set(void);
void pto_ppi_for_peripheral_start_desync_clear(void);
void pto_ppi_for_peripheral_prepare_rx_set(void);
void pto_ppi_for_peripheral_prepare_rx_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* ESB_FEM_H__ */
