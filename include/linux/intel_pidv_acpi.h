/*
 * include/linux/intel_pidv_acpi.h
 *
 * Copyright (C) 2013 Intel Corp
 * Author: Vincent Tinelli (vincent.tinelli@intel.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#ifndef _INTEL_PIDV_ACPI_H
#define _INTEL_PIDV_ACPI_H

#include <linux/acpi.h>
#ifdef CONFIG_ACPI
#define ACPI_SIG_PIDV           "PIDV"

#define pidv_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr   = {				\
		.name = __stringify(_name),	\
		.mode = 0440,			\
	},					\
	.show   = _name##_show,			\
}

struct platform_id {
	u8 part_number[32];
	u8 ext_id_1[32];
	u8 ext_id_2[32];
	u8 uuid[16];
	u8 iafw_major;
	u8 iafw_minor;
	u8 secfw_major;
	u8 secfw_minor;
};

struct acpi_table_pidv {
	struct acpi_table_header header;
	struct platform_id pidv;
};

#endif
#endif
