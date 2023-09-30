// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 XiaoMi, Inc.
// Author: Deng yong jian <dengyongjian@xiaomi.com>

#ifndef __F_DTP_H
#define __F_DTP_H

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <linux/ioctl.h>
#include <linux/types.h>

enum packet_type {
	TYPE_HANDSHAKE = 2020,
	TYPE_HANDSHAKE_RESPONSE,
	TYPE_REQUEST_SEND_FILE,
	TYPE_REQUEST_SEND_FILE_RESPONSE,
	TYPE_MAX
};

struct dtp_packet_head {
	uint16_t type; /*packet type*/
	uint32_t length; /*the length of this head*/
	uint16_t command;
	uint64_t data;
	u8  param[0];
};

struct dtp_file_desc{
	int32_t fd;
	int64_t offset;
	uint64_t length;
	uint16_t command;
	uint32_t transaction_id;
};

struct dtp_event {
	uint32_t length;
	char *data;
};

#ifdef CONFIG_COMPAT
struct __compat_dtp_file_desc{
	compat_int_t	fd;
	compat_loff_t	offset;
	int64_t		length;
	uint16_t	command;
	uint32_t	transaction_id;
};

struct __compat_dtp_event {
	compat_size_t	length;
	compat_caddr_t	data;
};
#endif

#define DTP_SEND_FILE				_IOW('M', 0, struct dtp_file_desc)
#define DTP_RECEIVE_FILE			_IOW('M', 1, struct dtp_file_desc)
#define DTP_SEND_EVENT				_IOW('M', 3, struct dtp_event)
#ifdef CONFIG_COMPAT
#define COMPAT_DTP_SEND_FILE		_IOW('M', 0, struct __compat_dtp_file_desc)
#define COMPAT_DTP_RECEIVE_FILE		_IOW('M', 1, struct __compat_dtp_file_desc)
#define COMPAT_DTP_SEND_EVENT		_IOW('M', 3, 	struct __compat_dtp_event)
#endif

#endif

