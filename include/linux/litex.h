/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Common LiteX header providing
 * helper functions for accessing CSRs.
 *
 * Implementation of the functions is provided by
 * the LiteX SoC Controller driver.
 *
 * Copyright (C) 2019-2020 Antmicro <www.antmicro.com>
 */

#ifndef _LINUX_LITEX_H
#define _LINUX_LITEX_H

#include <linux/io.h>
#include <linux/types.h>
#include <linux/compiler_types.h>

/*
 * litex_check_accessors is a function implemented in
 * drivers/soc/litex/litex_soc_controller.c
 * checking if the common LiteX CSR accessors
 * are safe to be used by the drivers;
 * returns true (1) if yes - false (0) if not
 *
 * Important: All drivers that use litex_set_reg/litex_get_reg
 * functions should make sure that LiteX SoC Controller driver
 * has verified LiteX CSRs read and write operations before
 * issuing any read/writes to the LiteX peripherals.
 *
 * Exemplary snippet that can be used at the beginning
 * of the driver's probe() function to ensure that LiteX
 * SoC Controller driver is properely initialized:
 *
 * if (!litex_check_accessors())
 *     return -EPROBE_DEFER;
 */
int litex_check_accessors(void);

void litex_set_reg(void __iomem *reg, unsigned long reg_sz, unsigned long val);

unsigned long litex_get_reg(void __iomem *reg, unsigned long reg_sz);


#endif /* _LINUX_LITEX_H */
