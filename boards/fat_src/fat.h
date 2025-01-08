/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FAT_H_
#define _FAT_H_

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/

void FAT_MagicStart(uint32_t delayTime_s);

void FAT_MagicPass(void);

void FAT_MagicFail(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _FAT_H_ */
