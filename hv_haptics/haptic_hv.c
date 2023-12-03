// SPDX-License-Identifier: GPL-2.0
/*
 * Awinic high voltage LRA haptic driver
 *
 * Copyright (c) 2021-2023 awinic. All Rights Reserved.
 *
 * Author: Ethan <renzhiqiang@awinic.com>
 */

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/firmware.h>
#include <linux/proc_fs.h>
#include <linux/mman.h>

#include "haptic_hv.h"
#include "haptic_hv_reg.h"

#define HAPTIC_HV_DRIVER_VERSION	"v1.4.0"

#ifdef AW_ENABLE_PIN_CONTROL
static const char *pctl_names[] = {
	"awinic_reset_reset",
	"awinic_reset_active",
	"awinic_interrupt_active",
};
#endif

char *aw_ram_name = "haptic_ram.bin";
char aw_rtp_name[][AW_RTP_NAME_MAX] = {
	{"haptic_rtp_osc_24K_5s.bin"},
	{"haptic_rtp.bin"},
	{"haptic_rtp_lighthouse.bin"},
	{"haptic_rtp_silk.bin"},
	{"haptic_rtp_auto_sin.bin"},
};

#ifdef AW_TIKTAP
static struct aw_haptic *g_aw_haptic;
#endif
#ifdef AW_DOUBLE
struct aw_haptic *left;
struct aw_haptic *right;
#endif

/*********************************************************
 *
 * I2C Read/Write
 *
 *********************************************************/
int haptic_hv_i2c_reads(struct aw_haptic *aw_haptic, uint8_t reg_addr, uint8_t *buf, uint32_t len)
{
	int ret;
	struct i2c_msg msg[] = {
		[0] = {
			.addr = aw_haptic->i2c->addr,
			.flags = 0,
			.len = sizeof(uint8_t),
			.buf = &reg_addr,
			},
		[1] = {
			.addr = aw_haptic->i2c->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
			},
	};

	ret = i2c_transfer(aw_haptic->i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		aw_err("transfer failed.");
		return ret;
	} else if (ret != AW_I2C_READ_MSG_NUM) {
		aw_err("transfer failed(size error).");
		return -ENXIO;
	}

	return ret;
}

int haptic_hv_i2c_writes(struct aw_haptic *aw_haptic, uint8_t reg_addr, uint8_t *buf, uint32_t len)
{
	uint8_t *data = NULL;
	int ret = -1;

	data = kmalloc(len + 1, GFP_KERNEL);
	data[0] = reg_addr;
	memcpy(&data[1], buf, len);
	ret = i2c_master_send(aw_haptic->i2c, data, len + 1);
	if (ret < 0)
		aw_err("i2c master send 0x%02x err", reg_addr);
	kfree(data);

	return ret;
}

int haptic_hv_i2c_write_bits(struct aw_haptic *aw_haptic, uint8_t reg_addr,
			     uint32_t mask, uint8_t reg_data)
{
	uint8_t reg_val = 0;
	int ret = -1;

	ret = haptic_hv_i2c_reads(aw_haptic, reg_addr, &reg_val, AW_I2C_BYTE_ONE);
	if (ret < 0) {
		aw_err("i2c read error, ret=%d", ret);
		return ret;
	}
	reg_val &= mask;
	reg_val |= (reg_data & (~mask));
	ret = haptic_hv_i2c_writes(aw_haptic, reg_addr, &reg_val, AW_I2C_BYTE_ONE);
	if (ret < 0) {
		aw_err("i2c write error, ret=%d", ret);
		return ret;
	}

	return 0;
}

static int parse_dt_gpio(struct device *dev, struct aw_haptic *aw_haptic, struct device_node *np)
{
	int val = 0;

	aw_haptic->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw_haptic->reset_gpio < 0) {
		aw_err("no reset gpio provide");
		return -EPERM;
	}
	aw_info("reset gpio provide ok %d", aw_haptic->reset_gpio);
	aw_haptic->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw_haptic->irq_gpio < 0)
		aw_err("no irq gpio provided.");
	else
		aw_info("irq gpio provide ok irq = %d.", aw_haptic->irq_gpio);
	val = of_property_read_u8(np, "mode", &aw_haptic->info.mode);
	if (val != 0)
		aw_info("mode not found");
#ifdef AW_DOUBLE
	if (of_device_is_compatible(np, "awinic,haptic_hv_l")) {
		aw_info("compatible left vibrator.");
		memcpy(aw_haptic->name, "left", sizeof("left"));
		left = NULL;
		left = aw_haptic;
	} else if (of_device_is_compatible(np, "awinic,haptic_hv_r")) {
		aw_info("compatible right vibrator.");
		memcpy(aw_haptic->name, "right", sizeof("right"));
		right = NULL;
		right = aw_haptic;
	} else {
		aw_err("compatible failed.");
		return -ERANGE;
	}
#endif

	return 0;
}

#ifdef AW_ENABLE_PIN_CONTROL
static int select_pin_state(struct awinic *awinic, const char *name)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(awinic->pinctrl_state); i++) {
		int rc;
		const char *n = pctl_names[i];

		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(aw_haptic->pinctrl, aw_haptic->pinctrl_state[i]);
			if (rc)
				aw_err("cannot select '%s'", name);
			else
				aw_info("selected '%s'", name);

			return rc;
		}
	}

	aw_info("'%s' not found", name);
	return -EINVAL;
}

static void control_reset_pin(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	ret = select_pin_state(aw_haptic, "awinic_reset_active");
	if (ret < 0) {
		aw_err("%s select reset failed!\n", __func__);
		return;
	}
	usleep_range(5000, 5500);

	ret = select_pin_state(aw_haptic, "awinic_reset_reset");
	if (ret < 0) {
		aw_err("%s select reset failed!\n", __func__);
		return;
	}
	usleep_range(5000, 5500);

	ret = select_pin_state(aw_haptic, "awinic_reset_active");
	if (ret < 0) {
		aw_err("%s select reset failed!\n", __func__);
		return;
	}
	usleep_range(8000, 8500);
#endif

static void hw_reset(struct aw_haptic *aw_haptic)
{
	aw_info("enter");

	if (aw_haptic != NULL) {
#ifdef AW_ENABLE_PIN_CONTROL
		control_reset_pin(aw_haptic);
		return;
#else
		if (gpio_is_valid(aw_haptic->reset_gpio)) {
			gpio_set_value_cansleep(aw_haptic->reset_gpio, 0);
			usleep_range(5000, 5500);
			gpio_set_value_cansleep(aw_haptic->reset_gpio, 1);
			usleep_range(8000, 8500);
			return;
		}
#endif
	}

	aw_err("failed");
}

static void sw_reset(struct aw_haptic *aw_haptic)
{
	uint8_t reset = AW_BIT_RESET;

	aw_dbg("enter");
	haptic_hv_i2c_writes(aw_haptic, AW_REG_CHIPID, &reset, AW_I2C_BYTE_ONE);
	usleep_range(3000, 3500);
}

static int judge_value(uint8_t reg)
{
	int ret = 0;

	if (!reg)
		return -ERANGE;
	switch (reg) {
	case AW86925_BIT_RSTCFG_PRE_VAL:
	case AW86926_BIT_RSTCFG_PRE_VAL:
	case AW86927_BIT_RSTCFG_PRE_VAL:
	case AW86928_BIT_RSTCFG_PRE_VAL:
	case AW86925_BIT_RSTCFG_VAL:
	case AW86926_BIT_RSTCFG_VAL:
	case AW86927_BIT_RSTCFG_VAL:
	case AW86928_BIT_RSTCFG_VAL:
		ret = -ERANGE;
		break;
	default:
		break;
	}

	return ret;
}

static int read_chipid(struct aw_haptic *aw_haptic, uint32_t *reg_val)
{
	uint8_t value[2] = {0};
	int ret = 0;

	/* try the old way of read chip id */
	ret = haptic_hv_i2c_reads(aw_haptic, AW_REG_CHIPID, &value[0], AW_I2C_BYTE_ONE);
	if (ret < 0)
		return ret;

	ret = judge_value(value[0]);
	if (!ret) {
		*reg_val = value[0];
		return ret;
	}
	/* try the new way of read chip id */
	ret = haptic_hv_i2c_reads(aw_haptic, AW_REG_CHIPIDH, value, AW_I2C_BYTE_TWO);
	if (ret < 0)
		return ret;
	*reg_val = value[0] << 8 | value[1];

	return ret;
}

static int ctrl_init(struct aw_haptic *aw_haptic)
{
	uint32_t reg = 0;
	uint8_t cnt = 0;

	aw_info("enter");
	for (cnt = 0; cnt < AW_READ_CHIPID_RETRIES; cnt++) {
		/* hardware reset */
		hw_reset(aw_haptic);
		if (read_chipid(aw_haptic, &reg) < 0)
			aw_err("read chip id fail");
		switch (reg) {
#ifdef AW869X_DRIVER_ENABLE
		case AW8695_CHIPID:
		case AW8697_CHIPID:
			aw_haptic->func = &aw869x_func_list;
			return 0;
#endif
#ifdef AW869XX_DRIVER_ENABLE
		case AW86905_CHIPID:
		case AW86907_CHIPID:
		case AW86915_CHIPID:
		case AW86917_CHIPID:
			aw_haptic->func = &aw869xx_func_list;
			return 0;
#endif
#ifdef AW8671X_DRIVER_ENABLE
		case AW86715_CHIPID:
		case AW86716_CHIPID:
		case AW86717_CHIPID:
		case AW86718_CHIPID:
			aw_haptic->func = &aw8671x_func_list;
			return 0;
#endif
#ifdef AW8692X_DRIVER_ENABLE
		case AW86925_CHIPID:
		case AW86926_CHIPID:
		case AW86927_CHIPID:
		case AW86928_CHIPID:
			aw_haptic->func = &aw8692x_func_list;
			return 0;
#endif
#ifdef AW8693X_DRIVER_ENABLE
		case AW86936_CHIPID:
		case AW86937_CHIPID:
		case AW86938_CHIPID:
			aw_haptic->func = &aw8693x_func_list;
			return 0;
#endif
		default:
			aw_info("unexpected chipid!");
			break;
		}
		usleep_range(2000, 2500);
	}

	return -EINVAL;
}

static int parse_chipid(struct aw_haptic *aw_haptic)
{
	int ret = -1;
	uint32_t reg = 0;
	uint8_t cnt = 0;

	for (cnt = 0; cnt < AW_READ_CHIPID_RETRIES; cnt++) {
		ret = read_chipid(aw_haptic, &reg);
		if (ret < 0)
			aw_err("read chip id fail: %d", ret);
		switch (reg) {
#ifdef AW869X_DRIVER_ENABLE
		case AW8695_CHIPID:
			aw_haptic->chipid = AW8695_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_haptic->i2s_config = false;
			aw_info("detected aw8695.");
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			return ret;
		case AW8697_CHIPID:
			aw_haptic->chipid = AW8697_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L2;
			aw_haptic->i2s_config = false;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_info("detected aw8697.");
			return ret;
#endif
#ifdef AW869XX_DRIVER_ENABLE
		case AW86905_CHIPID:
			aw_haptic->chipid = AW86905_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_haptic->i2s_config = false;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_info("detected aw86905.");
			return 0;
		case AW86907_CHIPID:
			aw_haptic->chipid = AW86907_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L2;
			aw_haptic->i2s_config = false;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_info("detected aw86907.");
			return 0;
		case AW86915_CHIPID:
			aw_haptic->chipid = AW86915_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_haptic->i2s_config = true;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_info("detected aw86915.");
			return 0;
		case AW86917_CHIPID:
			aw_haptic->chipid = AW86917_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L2;
			aw_haptic->i2s_config = true;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_info("detected aw86917.");
			return 0;
#endif
#ifdef AW8671X_DRIVER_ENABLE
		case AW86715_CHIPID:
			aw_haptic->chipid = AW86715_CHIPID;
			aw_haptic->i2s_config = false;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_info("detected aw86715.");
			return 0;
		case AW86716_CHIPID:
			aw_haptic->chipid = AW86716_CHIPID;
			aw_haptic->i2s_config = true;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_info("detected aw86716.");
			return 0;
		case AW86717_CHIPID:
			aw_haptic->chipid = AW86717_CHIPID;
			aw_haptic->i2s_config = true;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_info("detected aw86717.");
			return 0;
		case AW86718_CHIPID:
			aw_haptic->chipid = AW86718_CHIPID;
			aw_haptic->i2s_config = true;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_info("detected aw86718.");
			return 0;
#endif
#ifdef AW8692X_DRIVER_ENABLE
		case AW86925_CHIPID:
			aw_haptic->chipid = AW86925_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_info("detected aw86925.");
			return 0;
		case AW86926_CHIPID:
			aw_haptic->chipid = AW86926_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_info("detected aw86926.");
			return 0;
		case AW86927_CHIPID:
			aw_haptic->chipid = AW86927_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_info("detected aw86927.");
			return 0;
		case AW86928_CHIPID:
			aw_haptic->chipid = AW86928_CHIPID;
			aw_haptic->bst_pc = AW_BST_PC_L1;
			aw_haptic->trim_lra_boundary = AW_TRIM_LRA_BOUNDARY;
			aw_info("detected aw86928.");
			return 0;
#endif
#ifdef AW8693X_DRIVER_ENABLE
		case AW86936_CHIPID:
			aw_haptic->chipid = AW86936_CHIPID;
			aw_haptic->i2s_config = true;
			aw_haptic->trim_lra_boundary = AW8693X_TRIM_LRA_BOUNDARY;
			aw_info("detected aw86936.");
			return 0;
		case AW86937_CHIPID:
			aw_haptic->chipid = AW86937_CHIPID;
			aw_haptic->i2s_config = true;
			aw_haptic->trim_lra_boundary = AW8693X_TRIM_LRA_BOUNDARY;
			aw_info("detected aw86937.");
			return 0;
		case AW86938_CHIPID:
			aw_haptic->chipid = AW86938_CHIPID;
			aw_haptic->i2s_config = true;
			aw_haptic->trim_lra_boundary = AW8693X_TRIM_LRA_BOUNDARY;
			aw_info("detected aw86938.");
			return 0;
#endif
		default:
			aw_info("unsupport device revision (0x%02X)", reg);
			break;
		}
		usleep_range(2000, 2500);
	}

	return -EINVAL;
}

static void ram_play(struct aw_haptic *aw_haptic, uint8_t mode)
{
#ifdef AW_DOUBLE
	if (mode == AW_RAM_MODE)
		aw_haptic->func->set_wav_loop(aw_haptic, 0x00, 0x00);
	aw_haptic->func->play_mode(aw_haptic, mode);
	if (aw_haptic->dual_flag) {
		aw_haptic->dual_flag = false;
		if (down_trylock(&left->sema) == 0) {
			down_interruptible(&left->sema);
		} else {
			up(&left->sema);
			up(&left->sema);
		}
	}
#else
	aw_haptic->func->play_mode(aw_haptic, mode);
#endif
	aw_haptic->func->play_go(aw_haptic, true);
}

static int get_ram_num(struct aw_haptic *aw_haptic)
{
	uint8_t wave_addr[2] = {0};
	uint32_t first_wave_addr = 0;

	if (!aw_haptic->ram_init) {
		aw_err("ram init faild, ram_num = 0!");
		return -EPERM;
	}
	mutex_lock(&aw_haptic->lock);
	/* RAMINIT Enable */
	aw_haptic->func->ram_init(aw_haptic, true);
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->set_ram_addr(aw_haptic);
	aw_haptic->func->get_first_wave_addr(aw_haptic, wave_addr);
	first_wave_addr = (wave_addr[0] << 8 | wave_addr[1]);
	aw_haptic->ram.ram_num = (first_wave_addr - aw_haptic->ram.base_addr - 1) / 4;
	aw_info("first wave addr = 0x%04x", first_wave_addr);
	aw_info("ram_num = %d", aw_haptic->ram.ram_num);
	/* RAMINIT Disable */
	aw_haptic->func->ram_init(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);

	return 0;
}

static void ram_vbat_comp(struct aw_haptic *aw_haptic, bool flag)
{
	int temp_gain = 0;

	if (flag) {
		if (aw_haptic->ram_vbat_comp == AW_RAM_VBAT_COMP_ENABLE) {
			aw_haptic->func->get_vbat(aw_haptic);
			temp_gain = aw_haptic->gain * AW_VBAT_REFER / aw_haptic->vbat;
			if (temp_gain > (128 * AW_VBAT_REFER / AW_VBAT_MIN)) {
				temp_gain = 128 * AW_VBAT_REFER / AW_VBAT_MIN;
				aw_dbg("gain limit=%d", temp_gain);
			}
			aw_haptic->func->set_gain(aw_haptic, temp_gain);
			aw_info("ram vbat comp open");
		} else {
			aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
			aw_info("ram vbat comp close");
		}
	} else {
		aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
		aw_info("ram vbat comp close");
	}
}

static int judge_cali_range(struct aw_haptic *aw_haptic)
{
	uint32_t f0_cali_min = 0;
	uint32_t f0_cali_max = 0;

	f0_cali_min = aw_haptic->info.f0_pre * (100 - aw_haptic->info.f0_cali_percent) / 100;
	f0_cali_max = aw_haptic->info.f0_pre * (100 + aw_haptic->info.f0_cali_percent) / 100;

	aw_info("f0_pre = %d, f0_cali_min = %d, f0_cali_max = %d, f0 = %d",
		aw_haptic->info.f0_pre, f0_cali_min, f0_cali_max, aw_haptic->f0);

	if (aw_haptic->f0 < f0_cali_min) {
		aw_err("f0 is too small, f0 = %d!", aw_haptic->f0);
#ifdef AW_USE_MAXIMUM_F0_CALI_DATA
		aw_haptic->f0_cali_data = aw_haptic->trim_lra_boundary;
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
#endif
		return -ERANGE;
	}

	if (aw_haptic->f0 > f0_cali_max) {
		aw_err("f0 is too large, f0 = %d!", aw_haptic->f0);
#ifdef AW_USE_MAXIMUM_F0_CALI_DATA
		aw_haptic->f0_cali_data = aw_haptic->trim_lra_boundary - 1;
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
#endif
		return -ERANGE;
	}

	return 0;
}

static void calculate_cali_data(struct aw_haptic *aw_haptic)
{
	char f0_cali_lra = 0;
	int f0_cali_step = 0;

	/* calculate cali step */
	f0_cali_step = 100000 * ((int)aw_haptic->f0 - (int)aw_haptic->info.f0_pre) /
				((int)aw_haptic->info.f0_pre * AW_OSC_CALI_ACCURACY);
	aw_info("f0_cali_step = %d", f0_cali_step);
	if (f0_cali_step >= 0) {	/*f0_cali_step >= 0 */
		if (f0_cali_step % 10 >= 5)
			f0_cali_step = aw_haptic->trim_lra_boundary + (f0_cali_step / 10 + 1);
		else
			f0_cali_step = aw_haptic->trim_lra_boundary + f0_cali_step / 10;
	} else {	/* f0_cali_step < 0 */
		if (f0_cali_step % 10 <= -5)
			f0_cali_step = aw_haptic->trim_lra_boundary + (f0_cali_step / 10 - 1);
		else
			f0_cali_step = aw_haptic->trim_lra_boundary + f0_cali_step / 10;
	}
	if (f0_cali_step >= aw_haptic->trim_lra_boundary)
		f0_cali_lra = (char)f0_cali_step - aw_haptic->trim_lra_boundary;
	else
		f0_cali_lra = (char)f0_cali_step + aw_haptic->trim_lra_boundary;
	/* update cali step */
	aw_haptic->f0_cali_data = (int)f0_cali_lra;
	aw_info("f0_cali_data = 0x%02X", aw_haptic->f0_cali_data);
}

static int f0_cali(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	aw_haptic->func->upload_lra(aw_haptic, AW_WRITE_ZERO);
	if (aw_haptic->func->get_f0(aw_haptic)) {
		aw_err("get f0 error");
	} else {
		ret = judge_cali_range(aw_haptic);
		if (ret < 0)
			return -ERANGE;
		calculate_cali_data(aw_haptic);
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
	}
	/* restore standby work mode */
	aw_haptic->func->play_stop(aw_haptic);

	return ret;
}

static int ram_f0_cali(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	aw_haptic->func->upload_lra(aw_haptic, AW_WRITE_ZERO);
	if (aw_haptic->func->ram_get_f0(aw_haptic)) {
		aw_err("get f0 error");
	} else {
		ret = judge_cali_range(aw_haptic);
		if (ret < 0)
			return -ERANGE;
		calculate_cali_data(aw_haptic);
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
	}
	/* restore standby work mode */
	aw_haptic->func->play_stop(aw_haptic);

	return ret;
}

static void pm_qos_enable(struct aw_haptic *aw_haptic, bool enable)
{
#ifdef KERNEL_OVER_5_10
	if (enable) {
		if (!cpu_latency_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			cpu_latency_qos_add_request(&aw_haptic->aw_pm_qos_req_vb,
						    CPU_LATENCY_QOC_VALUE);
	} else {
		if (cpu_latency_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			cpu_latency_qos_remove_request(&aw_haptic->aw_pm_qos_req_vb);
	}
#else
	if (enable) {
		if (!pm_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			pm_qos_add_request(&aw_haptic->aw_pm_qos_req_vb,
					   PM_QOS_CPU_DMA_LATENCY, AW_PM_QOS_VALUE_VB);
	} else {
		if (pm_qos_request_active(&aw_haptic->aw_pm_qos_req_vb))
			pm_qos_remove_request(&aw_haptic->aw_pm_qos_req_vb);
	}
#endif
}

static int rtp_osc_cali(struct aw_haptic *aw_haptic)
{
	uint32_t buf_len = 0;
	int ret = -1;
	const struct firmware *rtp_file;

	aw_haptic->rtp_cnt = 0;
	aw_haptic->timeval_flags = 1;

	/* fw loaded */
	ret = request_firmware(&rtp_file, aw_rtp_name[0], aw_haptic->dev);
	if (ret < 0) {
		aw_err("failed to read %s", aw_rtp_name[0]);
		return ret;
	}
	/*aw_haptic add stop,for irq interrupt during calibrate */
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->rtp_init = false;
	mutex_lock(&aw_haptic->rtp_lock);
	vfree(aw_haptic->aw_rtp);
	aw_haptic->aw_rtp = vmalloc(rtp_file->size + sizeof(int));
	if (!aw_haptic->aw_rtp) {
		release_firmware(rtp_file);
		mutex_unlock(&aw_haptic->rtp_lock);
		aw_err("error allocating memory");
		return -ENOMEM;
	}
	aw_haptic->aw_rtp->len = rtp_file->size;
	aw_haptic->rtp_len = rtp_file->size;
	aw_info("rtp file:[%s] size = %dbytes", aw_rtp_name[0], aw_haptic->aw_rtp->len);
	memcpy(aw_haptic->aw_rtp->data, rtp_file->data, rtp_file->size);
	release_firmware(rtp_file);
	mutex_unlock(&aw_haptic->rtp_lock);
	/* gain */
	ram_vbat_comp(aw_haptic, false);
	/* rtp mode config */
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	/* bst mode */
	aw_haptic->func->bst_mode_config(aw_haptic, AW_BST_BYPASS_MODE);
	disable_irq(gpio_to_irq(aw_haptic->irq_gpio));
	/* haptic go */
	aw_haptic->func->play_go(aw_haptic, true);
	mutex_lock(&aw_haptic->rtp_lock);
	pm_qos_enable(aw_haptic, true);
	while (1) {
		if (!aw_haptic->func->rtp_get_fifo_afs(aw_haptic)) {
#ifdef AW_ENABLE_RTP_PRINT_LOG
			aw_info("not almost_full, aw_haptic->rtp_cnt=%d", aw_haptic->rtp_cnt);
#endif
			if ((aw_haptic->aw_rtp->len - aw_haptic->rtp_cnt) <
			    (aw_haptic->ram.base_addr >> 2))
				buf_len = aw_haptic->aw_rtp->len - aw_haptic->rtp_cnt;
			else
				buf_len = (aw_haptic->ram.base_addr >> 2);
			if (aw_haptic->rtp_cnt != aw_haptic->aw_rtp->len) {
				if (aw_haptic->timeval_flags == 1) {
					aw_haptic->kstart = ktime_get();
					aw_haptic->timeval_flags = 0;
				}
				aw_haptic->func->set_rtp_data(aw_haptic,
					&aw_haptic->aw_rtp->data[aw_haptic->rtp_cnt], buf_len);
				aw_haptic->rtp_cnt += buf_len;
			}
		}
		if (aw_haptic->func->get_osc_status(aw_haptic)) {
			aw_haptic->kend = ktime_get();
			aw_info("osc trim playback done aw_haptic->rtp_cnt= %d",
				aw_haptic->rtp_cnt);
			break;
		}
		aw_haptic->kend = ktime_get();
		aw_haptic->microsecond = ktime_to_us(ktime_sub(aw_haptic->kend, aw_haptic->kstart));
		if (aw_haptic->microsecond > AW_OSC_CALI_MAX_LENGTH) {
			aw_info("osc trim time out! aw_haptic->rtp_cnt %d", aw_haptic->rtp_cnt);
			break;
		}
	}
	pm_qos_enable(aw_haptic, false);
	mutex_unlock(&aw_haptic->rtp_lock);
	enable_irq(gpio_to_irq(aw_haptic->irq_gpio));
	aw_haptic->microsecond = ktime_to_us(ktime_sub(aw_haptic->kend, aw_haptic->kstart));
	/* calibration osc */
	aw_info("microsecond: %llu", aw_haptic->microsecond);

	return 0;
}

static void rtp_trim_lra_cali(struct aw_haptic *aw_haptic)
{
#ifdef AW_OSC_MULTI_CALI
	uint8_t last_trim_code = 0;
	int temp = 0;
	int count = 5;
#endif
	uint32_t lra_trim_code = 0;
	/* 0.1 percent below no need to calibrate */
	uint32_t osc_cali_threshold = 10;
	uint32_t real_code = 0;
	uint32_t theory_time = 0;
	uint32_t real_time = 0;

	aw_haptic->func->upload_lra(aw_haptic, AW_WRITE_ZERO);
#ifdef AW_OSC_MULTI_CALI
	while (count) {
#endif
	rtp_osc_cali(aw_haptic);
	real_time = aw_haptic->microsecond;
	theory_time = aw_haptic->func->get_theory_time(aw_haptic);
	if (theory_time == real_time) {
		aw_info("theory_time == real_time: %d, no need to calibrate!", real_time);
		return;
	} else if (theory_time < real_time) {
		if ((real_time - theory_time) > (theory_time / AW_OSC_TRIM_PARAM)) {
			aw_info("(real_time - theory_time) > (theory_time/50), can't calibrate!");
			return;
		}

		if ((real_time - theory_time) < (osc_cali_threshold * theory_time / 10000)) {
			aw_info("real_time: %d, theory_time: %d, no need to calibrate!",
				real_time, theory_time);
			return;
		}

		real_code = 100000 * ((real_time - theory_time)) /
			    (theory_time * AW_OSC_CALI_ACCURACY);
		real_code = ((real_code % 10 < 5) ? 0 : 1) + real_code / 10;
		real_code = aw_haptic->trim_lra_boundary + real_code;
	} else if (theory_time > real_time) {
		if ((theory_time - real_time) > (theory_time / AW_OSC_TRIM_PARAM)) {
			aw_info("(theory_time - real_time) > (theory_time / 50), can't calibrate!");
			return;
		}
		if ((theory_time - real_time) < (osc_cali_threshold * theory_time / 10000)) {
			aw_info("real_time: %d, theory_time: %d, no need to calibrate!",
				real_time, theory_time);
			return;
		}

		real_code = (theory_time - real_time) / (theory_time / 100000) / AW_OSC_CALI_ACCURACY;
		real_code = ((real_code % 10 < 5) ? 0 : 1) + real_code / 10;
		real_code = aw_haptic->trim_lra_boundary - real_code;
	}
	if (real_code >= aw_haptic->trim_lra_boundary)
		lra_trim_code = real_code - aw_haptic->trim_lra_boundary;
	else
		lra_trim_code = real_code + aw_haptic->trim_lra_boundary;
#ifdef AW_OSC_MULTI_CALI
	last_trim_code = aw_haptic->func->get_trim_lra(aw_haptic);
	if (last_trim_code) {
		if (lra_trim_code >= aw_haptic->trim_lra_boundary) {
			temp = last_trim_code - (aw_haptic->trim_lra_boundary * 2 - lra_trim_code);
			if (temp < aw_haptic->trim_lra_boundary && temp > 0 && last_trim_code >= aw_haptic->trim_lra_boundary)
				temp = aw_haptic->trim_lra_boundary;
			if (temp < 0)
				temp = lra_trim_code + last_trim_code;
		} else if (lra_trim_code > 0 && lra_trim_code < aw_haptic->trim_lra_boundary) {
			temp = (last_trim_code + lra_trim_code) & (aw_haptic->trim_lra_boundary * 2 - 1);
			if (temp >= aw_haptic->trim_lra_boundary && last_trim_code < aw_haptic->trim_lra_boundary)
				temp = aw_haptic->trim_lra_boundary - 1;
			if ((aw_haptic->trim_lra_boundary * 2 - last_trim_code) <= lra_trim_code)
				temp = last_trim_code + lra_trim_code - aw_haptic->trim_lra_boundary * 2;
		} else {
			temp = last_trim_code;
		}
		aw_haptic->osc_cali_data = temp;
	} else {
		aw_haptic->osc_cali_data = lra_trim_code;
	}
#else
	aw_haptic->osc_cali_data = lra_trim_code;
#endif
	aw_haptic->func->upload_lra(aw_haptic, AW_OSC_CALI_LRA);
	aw_info("real_time: %d, theory_time: %d", real_time, theory_time);
	aw_info("real_code: %d, trim_lra: 0x%02X", real_code, lra_trim_code);
#ifdef AW_OSC_MULTI_CALI
	count--;
	}
#endif
}

static void input_stop_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, stop_work);

	mutex_lock(&aw_haptic->lock);
	hrtimer_cancel(&aw_haptic->timer);
	aw_haptic->func->play_stop(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
}

static void input_gain_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, gain_work);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
	mutex_unlock(&aw_haptic->lock);
}

static void input_vib_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, input_vib_work);

	mutex_lock(&aw_haptic->lock);
	/* Enter standby mode */
	hrtimer_cancel(&aw_haptic->timer);
	aw_haptic->func->play_stop(aw_haptic);
	if (aw_haptic->state) {
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
		if (aw_haptic->activate_mode == AW_RAM_MODE) {
			ram_vbat_comp(aw_haptic, false);
			aw_haptic->func->set_wav_seq(aw_haptic, 0x00, aw_haptic->index);
			aw_haptic->func->set_wav_seq(aw_haptic, 0x01, 0x00);
			aw_haptic->func->set_wav_loop(aw_haptic, 0x00, 0x00);
			ram_play(aw_haptic, AW_RAM_MODE);
		} else if (aw_haptic->activate_mode == AW_RAM_LOOP_MODE) {
			ram_vbat_comp(aw_haptic, true);
			aw_haptic->func->set_repeat_seq(aw_haptic, aw_haptic->index);
			ram_play(aw_haptic, AW_RAM_LOOP_MODE);
			/* run ms timer */
			hrtimer_start(&aw_haptic->timer, ktime_set(aw_haptic->duration / 1000,
				      (aw_haptic->duration % 1000) * 1000000), HRTIMER_MODE_REL);
		} else {
			aw_err("activate_mode error");
		}
	}
	mutex_unlock(&aw_haptic->lock);
}

static int input_upload_effect(struct input_dev *dev, struct ff_effect *effect,
			       struct ff_effect *old)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);
	short wav_id = 0;
	int wav_id_max = 0;
	int ret = 0;

	mutex_lock(&aw_haptic->lock);
	switch (effect->type) {
	case FF_CONSTANT:
		aw_haptic->activate_mode = AW_RAM_LOOP_MODE;
		aw_haptic->duration = effect->replay.length;
		aw_haptic->index = aw_haptic->ram.ram_num;
		aw_info("waveform id = %d", aw_haptic->index);
		break;
	case FF_PERIODIC:
		ret = copy_from_user(&wav_id, effect->u.periodic.custom_data, sizeof(short));
		if (ret) {
			aw_err("copy from user error %d!!", ret);
			mutex_unlock(&aw_haptic->lock);
			return -ERANGE;
		}
		aw_info("waveform id = %d", wav_id);
		wav_id_max = aw_haptic->rtp_num + aw_haptic->ram.ram_num - 1;
		if (wav_id > 0 && wav_id < aw_haptic->ram.ram_num) {
			aw_haptic->activate_mode = AW_RAM_MODE;
			aw_haptic->index = wav_id;
		} else if (wav_id > aw_haptic->ram.ram_num && wav_id <= wav_id_max) {
			aw_haptic->activate_mode = AW_RTP_MODE;
			aw_haptic->rtp_file_num = wav_id - aw_haptic->ram.ram_num;
		} else {
			aw_haptic->activate_mode = AW_STANDBY_MODE;
			aw_err("waveform id is error");
			mutex_unlock(&aw_haptic->lock);
			return -ERANGE;
		}
		break;
	default:
		aw_err("Unsupported effect type: %d", effect->type);
		break;
	}
	mutex_unlock(&aw_haptic->lock);

	return 0;
}

static int input_playback(struct input_dev *dev, int effect_id, int val)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);

	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return -ERANGE;
	}
	if (val > 0) {
		aw_haptic->state = 1;
	} else {
		queue_work(aw_haptic->work_queue, &aw_haptic->stop_work);
		return 0;
	}
	switch (aw_haptic->activate_mode) {
	case AW_RAM_MODE:
		queue_work(aw_haptic->work_queue, &aw_haptic->input_vib_work);
		break;
	case AW_RAM_LOOP_MODE:
		queue_work(aw_haptic->work_queue, &aw_haptic->input_vib_work);
		break;
	case AW_RTP_MODE:
		queue_work(aw_haptic->work_queue, &aw_haptic->rtp_work);
		break;
	default:
		aw_err("Unsupported mode: %d", aw_haptic->activate_mode);
		break;
	}

	return 0;
}

static int input_erase(struct input_dev *dev, int effect_id)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);

	aw_haptic->duration = 0;

	return 0;
}

static void input_set_gain(struct input_dev *dev, uint16_t gain)
{
	struct aw_haptic *aw_haptic = input_get_drvdata(dev);

	if (gain > 0x7fff)
		gain = 0x7fff;
	aw_haptic->gain = gain * 0x80 / 0x7fff;
	queue_work(aw_haptic->work_queue, &aw_haptic->gain_work);
	aw_info("aw_haptic->gain = 0x%02x", aw_haptic->gain);
}

static int input_framework_init(struct aw_haptic *aw_haptic)
{
	struct input_dev *input_dev;
	int ret = 0;

	input_dev = devm_input_allocate_device(aw_haptic->dev);
	if (input_dev == NULL)
		return -ENOMEM;
	input_dev->name = "aw-haptic-hv";
	input_set_drvdata(input_dev, aw_haptic);
	aw_haptic->input_dev = input_dev;
	input_set_capability(input_dev, EV_FF, FF_GAIN);
	input_set_capability(input_dev, EV_FF, FF_CONSTANT);
	input_set_capability(input_dev, EV_FF, FF_PERIODIC);
	input_set_capability(input_dev, EV_FF, FF_CUSTOM);
	ret = input_ff_create(input_dev, AW_EFFECT_NUMBER);
	if (ret < 0) {
		aw_err("create input FF device failed, rc=%d\n", ret);
		return ret;
	}
	input_dev->ff->upload = input_upload_effect;
	input_dev->ff->playback = input_playback;
	input_dev->ff->erase = input_erase;
	input_dev->ff->set_gain = input_set_gain;
	INIT_WORK(&aw_haptic->gain_work, input_gain_work_routine);
	INIT_WORK(&aw_haptic->stop_work, input_stop_work_routine);
	INIT_WORK(&aw_haptic->input_vib_work, input_vib_work_routine);
	ret = input_register_device(input_dev);
	if (ret < 0) {
		aw_err("register input device failed, rc=%d\n", ret);
		input_ff_destroy(aw_haptic->input_dev);
		return ret;
	}

	return ret;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct aw_haptic *aw_haptic = container_of(timer, struct aw_haptic, timer);

	aw_haptic->state = 0;
	queue_work(aw_haptic->work_queue, &aw_haptic->vibrator_work);

	return HRTIMER_NORESTART;
}

#ifdef AW_DURATION_DECIDE_WAVEFORM
static int ram_select_waveform(struct aw_haptic *aw_haptic)
{
	uint8_t wavseq = 0;
	uint8_t wavloop = 0;

	if (aw_haptic->duration <= 0) {
		aw_err("duration time %d error", aw_haptic->duration);
		return -ERANGE;
	}
	aw_haptic->activate_mode = AW_RAM_MODE;
	if ((aw_haptic->duration > 0) && (aw_haptic->duration < aw_haptic->info.duration_time[0])) {
		wavseq = 3;
	} else if ((aw_haptic->duration >= aw_haptic->info.duration_time[0]) &&
		   (aw_haptic->duration < aw_haptic->info.duration_time[1])) {
		wavseq = 2;
	} else if ((aw_haptic->duration >= aw_haptic->info.duration_time[1]) &&
		   (aw_haptic->duration < aw_haptic->info.duration_time[2])) {
		wavseq = 1;
	} else if (aw_haptic->duration >= aw_haptic->info.duration_time[2]) {
		wavseq = 4;
		wavloop = 15;
		aw_haptic->activate_mode = AW_RAM_LOOP_MODE;
	}
	aw_info("duration %d, select index %d", aw_haptic->duration, wavseq);
	aw_haptic->func->set_wav_seq(aw_haptic, 0, wavseq);
	aw_haptic->func->set_wav_loop(aw_haptic, 0, wavloop);
	aw_haptic->func->set_wav_seq(aw_haptic, 1, 0);
	aw_haptic->func->set_wav_loop(aw_haptic, 1, 0);

	return 0;
}
#endif

static void vibrator_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, vibrator_work);

	mutex_lock(&aw_haptic->lock);
	/* Enter standby mode */
	hrtimer_cancel(&aw_haptic->timer);
	aw_haptic->func->play_stop(aw_haptic);
	if (aw_haptic->state) {
#ifdef AW_DURATION_DECIDE_WAVEFORM
		ram_select_waveform(aw_haptic);
#endif
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
		if (aw_haptic->activate_mode == AW_RAM_MODE) {
			ram_vbat_comp(aw_haptic, false);
			ram_play(aw_haptic, AW_RAM_MODE);
		} else if (aw_haptic->activate_mode == AW_RAM_LOOP_MODE) {
			ram_vbat_comp(aw_haptic, true);
			ram_play(aw_haptic, AW_RAM_LOOP_MODE);
			/* run ms timer */
			hrtimer_start(&aw_haptic->timer, ktime_set(aw_haptic->duration / 1000,
				      (aw_haptic->duration % 1000) * 1000000), HRTIMER_MODE_REL);
		} else if (aw_haptic->activate_mode == AW_CONT_MODE) {
			aw_haptic->func->cont_config(aw_haptic);
			/* run ms timer */
			hrtimer_start(&aw_haptic->timer, ktime_set(aw_haptic->duration / 1000,
				      (aw_haptic->duration % 1000) * 1000000), HRTIMER_MODE_REL);
		} else {
			aw_err("activate_mode error");
		}
	}
	mutex_unlock(&aw_haptic->lock);
}

static int rtp_play(struct aw_haptic *aw_haptic)
{
	uint8_t glb_state_val = 0;
	uint32_t buf_len = 0;
	int ret = 0;
	struct aw_haptic_container *aw_rtp = aw_haptic->aw_rtp;

	while ((!aw_haptic->func->rtp_get_fifo_afs(aw_haptic))
	       && (aw_haptic->play_mode == AW_RTP_MODE)) {
#ifdef AW_ENABLE_RTP_PRINT_LOG
		aw_info("rtp cnt = %d", aw_haptic->rtp_cnt);
#endif
		if (!aw_rtp) {
			aw_info("aw_rtp is null, break!");
			ret = -ERANGE;
			break;
		}
		if (aw_haptic->rtp_cnt < (aw_haptic->ram.base_addr)) {
			if ((aw_rtp->len - aw_haptic->rtp_cnt) < (aw_haptic->ram.base_addr))
				buf_len = aw_rtp->len - aw_haptic->rtp_cnt;
			else
				buf_len = aw_haptic->ram.base_addr;
		} else if ((aw_rtp->len - aw_haptic->rtp_cnt) < (aw_haptic->ram.base_addr >> 2)) {
			buf_len = aw_rtp->len - aw_haptic->rtp_cnt;
		} else {
			buf_len = aw_haptic->ram.base_addr >> 2;
		}
#ifdef AW_ENABLE_RTP_PRINT_LOG
		aw_info("buf_len = %d", buf_len);
#endif
#ifdef AW_DOUBLE
	if (aw_haptic->rtp_cnt == 0 && aw_haptic->dual_flag) {
		aw_haptic->dual_flag = false;
		if (down_trylock(&left->sema) == 0) {
			down_interruptible(&left->sema);
		} else {
			up(&left->sema);
			up(&left->sema);
		}
		aw_info("dual rtp play start");
	}
#endif
		aw_haptic->func->set_rtp_data(aw_haptic,
					      &aw_rtp->data[aw_haptic->rtp_cnt], buf_len);
		aw_haptic->rtp_cnt += buf_len;
		glb_state_val = aw_haptic->func->get_glb_state(aw_haptic);
		if ((aw_haptic->rtp_cnt == aw_rtp->len)
		    || ((glb_state_val & AW_GLBRD_STATE_MASK) == AW_STATE_STANDBY)) {
			if (aw_haptic->rtp_cnt != aw_rtp->len)
				aw_err("rtp play suspend!");
			else
				aw_info("rtp update complete!");
			aw_haptic->rtp_cnt = 0;
			aw_haptic->rtp_init = false;
			break;
		}
	}

	return ret;
}

static int wait_enter_rtp_mode(struct aw_haptic *aw_haptic)
{
	bool rtp_work_flag = false;
	uint8_t ret = 0;
	int cnt = 200;

	while (cnt) {
		ret = aw_haptic->func->judge_rtp_going(aw_haptic);
		if (ret) {
			rtp_work_flag = true;
			aw_info("RTP_GO!");
			break;
		}
		cnt--;
		aw_info("wait for RTP_GO, glb_state=0x%02X", ret);
		usleep_range(2000, 2500);
	}
	if (!rtp_work_flag) {
		aw_haptic->func->play_stop(aw_haptic);
		aw_err("failed to enter RTP_GO status!");
		return -ERANGE;
	}

	return 0;
}

static void rtp_work_routine(struct work_struct *work)
{
	int ret = -1;
	const struct firmware *rtp_file;
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, rtp_work);

	if (!aw_haptic->ram.base_addr) {
		aw_err("base addr is 0, not allow rtp play");
		return;
	}
	mutex_lock(&aw_haptic->lock);
	hrtimer_cancel(&aw_haptic->timer);
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->set_rtp_aei(aw_haptic, false);
	aw_haptic->func->irq_clear(aw_haptic);
	if (!aw_haptic->state) {
		mutex_unlock(&aw_haptic->lock);
		return;
	}
	mutex_unlock(&aw_haptic->lock);
	mutex_lock(&aw_haptic->rtp_lock);
	/* fw loaded */
	ret = request_firmware(&rtp_file, aw_rtp_name[aw_haptic->rtp_file_num], aw_haptic->dev);
	if (ret < 0) {
		aw_err("failed to read %s", aw_rtp_name[aw_haptic->rtp_file_num]);
		mutex_unlock(&aw_haptic->rtp_lock);
		return;
	}
	aw_haptic->rtp_init = false;
	vfree(aw_haptic->aw_rtp);
	aw_haptic->aw_rtp = vmalloc(rtp_file->size + sizeof(int));
	if (!aw_haptic->aw_rtp) {
		release_firmware(rtp_file);
		aw_err("error allocating memory");
		mutex_unlock(&aw_haptic->rtp_lock);
		return;
	}
	aw_haptic->aw_rtp->len = rtp_file->size;
	aw_info("rtp file:[%s] size = %dbytes",
		aw_rtp_name[aw_haptic->rtp_file_num], aw_haptic->aw_rtp->len);
	memcpy(aw_haptic->aw_rtp->data, rtp_file->data, rtp_file->size);
	mutex_unlock(&aw_haptic->rtp_lock);
	release_firmware(rtp_file);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->rtp_init = true;

	aw_haptic->func->upload_lra(aw_haptic, AW_OSC_CALI_LRA);
	/* gain */
	ram_vbat_comp(aw_haptic, false);
	/* rtp mode config */
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	/* haptic go */
	aw_haptic->func->play_go(aw_haptic, true);
	usleep_range(2000, 2500);
	ret = wait_enter_rtp_mode(aw_haptic);
	if (ret < 0) {
		mutex_unlock(&aw_haptic->lock);
		return;
	}
	mutex_unlock(&aw_haptic->lock);

	mutex_lock(&aw_haptic->rtp_lock);
	pm_qos_enable(aw_haptic, true);
	aw_haptic->rtp_cnt = 0;
	rtp_play(aw_haptic);
	if (aw_haptic->play_mode == AW_RTP_MODE)
		aw_haptic->func->set_rtp_aei(aw_haptic, true);
	pm_qos_enable(aw_haptic, false);
	mutex_unlock(&aw_haptic->rtp_lock);
}

static irqreturn_t irq_handle(int irq, void *data)
{
	int irq_state = 0;
	struct aw_haptic *aw_haptic = data;

	do {
		irq_state = aw_haptic->func->get_irq_state(aw_haptic);
		if (irq_state == AW_IRQ_ALMOST_EMPTY) {
			if (aw_haptic->rtp_init) {
				mutex_lock(&aw_haptic->rtp_lock);
				rtp_play(aw_haptic);
				mutex_unlock(&aw_haptic->rtp_lock);
			} else {
				aw_info("rtp_init: %d", aw_haptic->rtp_init);
			}
		}
		if (aw_haptic->play_mode != AW_RTP_MODE)
			aw_haptic->func->set_rtp_aei(aw_haptic, false);
	} while (irq_state);

	return IRQ_HANDLED;
}

static int irq_config(struct device *dev, struct aw_haptic *aw_haptic)
{
	int ret = -1;
	int irq_flags = 0;

	if (gpio_is_valid(aw_haptic->irq_gpio) &&
	    !(aw_haptic->flags & AW_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		aw_haptic->func->interrupt_setup(aw_haptic);
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(dev, gpio_to_irq(aw_haptic->irq_gpio), NULL,
						irq_handle, irq_flags, "aw_haptic", aw_haptic);
		if (ret != 0) {
			aw_err("failed to request IRQ %d: %d",
			       gpio_to_irq(aw_haptic->irq_gpio), ret);
			return ret;
		}
	} else {
		dev_info(dev, "skipping IRQ registration");
		/* disable feature support if gpio was invalid */
		aw_haptic->flags |= AW_FLAG_SKIP_INTERRUPTS;
	}

	return 0;
}

static void ram_load(const struct firmware *cont, void *context)
{
	uint16_t check_sum = 0;
	int i = 0;
	int ret = 0;
	struct aw_haptic *aw_haptic = context;
	struct aw_haptic_container *aw_fw;

#ifdef AW_READ_BIN_FLEXBALLY
	static uint8_t load_cont;
	int ram_timer_val = 1000;

	load_cont++;
#endif
	if (!cont) {
		aw_err("failed to read %s", aw_ram_name);
		release_firmware(cont);
#ifdef AW_READ_BIN_FLEXBALLY
		if (load_cont <= 20) {
			schedule_delayed_work(&aw_haptic->ram_work,
					      msecs_to_jiffies(ram_timer_val));
			aw_info("start hrtimer:load_cont%d", load_cont);
		}
#endif
		return;
	}
	aw_info("loaded %s - size: %zu", aw_ram_name, cont ? cont->size : 0);
	/* check sum */
	for (i = 2; i < cont->size; i++)
		check_sum += cont->data[i];
	if (check_sum != (uint16_t)((cont->data[0] << 8) | (cont->data[1]))) {
		aw_err("check sum err: check_sum=0x%04x", check_sum);
		release_firmware(cont);
		return;
	}
	aw_info("check sum pass : 0x%04x", check_sum);
	aw_haptic->ram.check_sum = check_sum;

	/* aw ram update */
	aw_fw = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw_fw) {
		release_firmware(cont);
		aw_err("Error allocating memory");
		return;
	}
	aw_fw->len = cont->size;
	memcpy(aw_fw->data, cont->data, cont->size);
	release_firmware(cont);
	ret = aw_haptic->func->container_update(aw_haptic, aw_fw);
	if (ret) {
		aw_err("ram firmware update failed!");
	} else {
		aw_haptic->ram_init = true;
		aw_haptic->ram.len = aw_fw->len - aw_haptic->ram.ram_shift;
		mutex_lock(&aw_haptic->lock);
		aw_haptic->func->trig_init(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
		aw_info("ram firmware update complete!");
		get_ram_num(aw_haptic);
	}
	kfree(aw_fw);

#ifdef AW_BOOT_OSC_CALI
	rtp_trim_lra_cali(aw_haptic);
#endif
}

static int ram_update(struct aw_haptic *aw_haptic)
{
	aw_haptic->ram_init = false;
	aw_haptic->rtp_init = false;
	return request_firmware_nowait(THIS_MODULE, 1, aw_ram_name, aw_haptic->dev, GFP_KERNEL,
				       aw_haptic, ram_load);
}

static void ram_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, ram_work.work);

	ram_update(aw_haptic);
}

static void ram_work_init(struct aw_haptic *aw_haptic)
{
	int ram_timer_val = AW_RAM_WORK_DELAY_INTERVAL;

	INIT_DELAYED_WORK(&aw_haptic->ram_work, ram_work_routine);
	schedule_delayed_work(&aw_haptic->ram_work, msecs_to_jiffies(ram_timer_val));
}

/*****************************************************
 *
 * haptic audio
 *
 *****************************************************/
static int audio_ctrl_list_ins(struct aw_haptic *aw_haptic, struct aw_haptic_ctr *haptic_ctr)
{
	struct aw_haptic_ctr *p_new = NULL;
	struct aw_haptic_audio *haptic_audio = &aw_haptic->haptic_audio;

	p_new = kzalloc(sizeof(struct aw_haptic_ctr), GFP_KERNEL);
	if (p_new == NULL)
		return -ENOMEM;

	/* update new list info */
	p_new->cnt = haptic_ctr->cnt;
	p_new->cmd = haptic_ctr->cmd;
	p_new->play = haptic_ctr->play;
	p_new->wavseq = haptic_ctr->wavseq;
	p_new->loop = haptic_ctr->loop;
	p_new->gain = haptic_ctr->gain;
	INIT_LIST_HEAD(&(p_new->list));
	list_add(&(p_new->list), &(haptic_audio->ctr_list));

	return 0;
}

static void audio_ctrl_list_clr(struct aw_haptic_audio *haptic_audio)
{
	struct aw_haptic_ctr *p_ctr = NULL;
	struct aw_haptic_ctr *p_ctr_bak = NULL;

	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
		list_del(&p_ctr->list);
		kfree(p_ctr);
	}
}

static void audio_init(struct aw_haptic *aw_haptic)
{
	aw_dbg("enter!");
	aw_haptic->func->set_wav_seq(aw_haptic, 0x01, 0x00);
}

static void audio_off(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	mutex_lock(&aw_haptic->lock);
#ifdef AW_DOUBLE
	left->func->play_stop(left);
	right->func->play_stop(right);
	left->func->set_gain(left, 0x80);
	right->func->set_gain(right, 0x80);
#else
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->set_gain(aw_haptic, 0x80);
#endif
	audio_ctrl_list_clr(&aw_haptic->haptic_audio);
	mutex_unlock(&aw_haptic->lock);
}

static enum hrtimer_restart audio_timer_func(struct hrtimer *timer)
{
	struct aw_haptic *aw_haptic =
	    container_of(timer, struct aw_haptic, haptic_audio.timer);

	aw_dbg("enter");
	queue_work(aw_haptic->work_queue, &aw_haptic->haptic_audio.work);

	hrtimer_start(&aw_haptic->haptic_audio.timer,
		      ktime_set(aw_haptic->haptic_audio.timer_val / 1000000,
				(aw_haptic->haptic_audio.timer_val % 1000000) * 1000),
				HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

static void audio_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, haptic_audio.work);
	struct aw_haptic_audio *haptic_audio = NULL;
	struct aw_haptic_ctr *ctr = &aw_haptic->haptic_audio.ctr;
	struct aw_haptic_ctr *p_ctr = NULL;
	struct aw_haptic_ctr *p_ctr_bak = NULL;
	uint32_t ctr_list_flag = 0;
	uint32_t ctr_list_input_cnt = 0;
	uint32_t ctr_list_output_cnt = 0;
	uint32_t ctr_list_diff_cnt = 0;
	uint32_t ctr_list_del_cnt = 0;
	int rtp_is_going_on = 0;

	aw_dbg("enter");
	haptic_audio = &(aw_haptic->haptic_audio);
	mutex_lock(&aw_haptic->haptic_audio.lock);
	memset(ctr, 0, sizeof(struct aw_haptic_ctr));
	ctr_list_flag = 0;
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
		ctr_list_flag = 1;
		break;
	}
	if (ctr_list_flag == 0)
		aw_dbg("ctr list empty");
	if (ctr_list_flag == 1) {
		list_for_each_entry_safe(p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
			ctr_list_input_cnt = p_ctr->cnt;
			break;
		}
		list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
						 &(haptic_audio->ctr_list), list) {
			ctr_list_output_cnt = p_ctr->cnt;
			break;
		}
		if (ctr_list_input_cnt > ctr_list_output_cnt)
			ctr_list_diff_cnt = ctr_list_input_cnt - ctr_list_output_cnt;

		if (ctr_list_input_cnt < ctr_list_output_cnt)
			ctr_list_diff_cnt = 32 + ctr_list_input_cnt - ctr_list_output_cnt;

		if (ctr_list_diff_cnt > 2) {
			list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
							 &(haptic_audio->ctr_list), list) {
				if ((p_ctr->play == 0) &&
				    (AW_CMD_ENABLE == (AW_CMD_HAPTIC & p_ctr->cmd))) {
					list_del(&p_ctr->list);
					kfree(p_ctr);
					ctr_list_del_cnt++;
				}
				if (ctr_list_del_cnt == ctr_list_diff_cnt)
					break;
			}
		}
	}
	/* get the last data from list */
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
		ctr->cnt = p_ctr->cnt;
		ctr->cmd = p_ctr->cmd;
		ctr->play = p_ctr->play;
		ctr->wavseq = p_ctr->wavseq;
		ctr->loop = p_ctr->loop;
		ctr->gain = p_ctr->gain;
		list_del(&p_ctr->list);
		kfree(p_ctr);
		break;
	}
	if (ctr->play) {
		aw_info("cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d",
			ctr->cnt,  ctr->cmd, ctr->play, ctr->wavseq, ctr->loop, ctr->gain);
	}
	if (ctr->wavseq > aw_haptic->ram.ram_num) {
		aw_err("wavseq out of range!");
		mutex_unlock(&aw_haptic->haptic_audio.lock);
		return;
	}
	rtp_is_going_on = aw_haptic->func->judge_rtp_going(aw_haptic);
	if (rtp_is_going_on) {
		mutex_unlock(&aw_haptic->haptic_audio.lock);
		return;
	}
	mutex_unlock(&aw_haptic->haptic_audio.lock);

#ifdef AW_DOUBLE
	if (ctr->play == AW_PLAY_ENABLE) {
		mutex_lock(&aw_haptic->lock);
		/* haptic config */
		if (ctr->cmd & AW_CMD_L_EN) {
			left->func->play_stop(left);
			left->func->play_mode(left, AW_RAM_MODE);
			left->func->set_wav_seq(left, 0x00, ctr->wavseq);
			left->func->set_wav_seq(left, 0x01, 0x00);
			left->func->set_wav_loop(left, 0x00, ctr->loop);
			left->func->set_gain(left, ctr->gain);
		}

		if (ctr->cmd & AW_CMD_R_EN) {
			right->func->play_stop(right);
			right->func->play_mode(right, AW_RAM_MODE);
			right->func->set_wav_seq(right, 0x00, ctr->wavseq);
			right->func->set_wav_seq(right, 0x01, 0x00);
			right->func->set_wav_loop(right, 0x00, ctr->loop);
			right->func->set_gain(right, ctr->gain);
		}
		/* play go */
		if (ctr->cmd & AW_CMD_L_EN)
			left->func->play_go(left, true);
		if (ctr->cmd & AW_CMD_R_EN)
			right->func->play_go(right, true);
		mutex_unlock(&aw_haptic->lock);
	} else if (ctr->play == AW_PLAY_STOP) {
		mutex_lock(&aw_haptic->lock);
		left->func->play_stop(left);
		right->func->play_stop(right);
		mutex_unlock(&aw_haptic->lock);
	} else if (ctr->play == AW_PLAY_GAIN) {
		mutex_lock(&aw_haptic->lock);
		left->func->set_gain(left, ctr->gain);
		right->func->set_gain(right, ctr->gain);
		mutex_unlock(&aw_haptic->lock);
	}
#else
	if (ctr->cmd == AW_CMD_ENABLE) {
		if (ctr->play == AW_PLAY_ENABLE) {
			aw_info("haptic_audio_play_start");
			mutex_lock(&aw_haptic->lock);
			aw_haptic->func->play_stop(aw_haptic);
			aw_haptic->func->play_mode(aw_haptic, AW_RAM_MODE);
			aw_haptic->func->set_wav_seq(aw_haptic, 0x00, ctr->wavseq);
			aw_haptic->func->set_wav_seq(aw_haptic, 0x01, 0x00);
			aw_haptic->func->set_wav_loop(aw_haptic, 0x00, ctr->loop);
			aw_haptic->func->set_gain(aw_haptic, ctr->gain);
			aw_haptic->func->play_go(aw_haptic, true);
			mutex_unlock(&aw_haptic->lock);
		} else if (ctr->play == AW_PLAY_STOP) {
			mutex_lock(&aw_haptic->lock);
			aw_haptic->func->play_stop(aw_haptic);
			mutex_unlock(&aw_haptic->lock);
		} else if (ctr->play == AW_PLAY_GAIN) {
			mutex_lock(&aw_haptic->lock);
			aw_haptic->func->set_gain(aw_haptic, ctr->gain);
			mutex_unlock(&aw_haptic->lock);
		}
	}
#endif

}

/*****************************************************
 *
 * node
 *
 *****************************************************/
static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "state = %d\n", aw_haptic->state);
}

static ssize_t state_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	return count;
}

static ssize_t duration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&aw_haptic->timer)) {
		time_rem = hrtimer_get_remaining(&aw_haptic->timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return snprintf(buf, PAGE_SIZE, "duration = %lldms\n", time_ms);
}

static ssize_t duration_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	/* setting 0 on duration is NOP for now */
	if (val == 0)
		return count;
	aw_info("duration=%d", val);
	aw_haptic->duration = val;

	return count;
}

static ssize_t activate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "activate = %d\n", aw_haptic->state);
}

static ssize_t activate_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_info("value=%d", val);
	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return count;
	}
	mutex_lock(&aw_haptic->lock);
	aw_haptic->state = val;
	aw_haptic->activate_mode = aw_haptic->info.mode;
	mutex_unlock(&aw_haptic->lock);
	queue_work(aw_haptic->work_queue, &aw_haptic->vibrator_work);

	return count;
}

static ssize_t activate_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "activate_mode = %d\n", aw_haptic->activate_mode);
}

static ssize_t activate_mode_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&aw_haptic->lock);
	aw_haptic->activate_mode = val;
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t index_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_wav_seq(aw_haptic, 1);
	aw_haptic->index = aw_haptic->seq[0];
	mutex_unlock(&aw_haptic->lock);

	return snprintf(buf, PAGE_SIZE, "index = %d\n", aw_haptic->index);
}

static ssize_t index_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val > aw_haptic->ram.ram_num) {
		aw_err("input value out of range!");
		return count;
	}
	aw_info("value=%d", val);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->index = val;
	aw_haptic->func->set_repeat_seq(aw_haptic, aw_haptic->index);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t vmax_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "vmax = %dmV\n", aw_haptic->vmax);
}

static ssize_t vmax_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_info("value=%d", val);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->vmax = val;
	aw_haptic->func->set_bst_vol(aw_haptic, aw_haptic->vmax);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t gain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "gain = 0x%02X\n", aw_haptic->gain);
}

static ssize_t gain_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_info("value=0x%02x", val);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->gain = val;
	aw_haptic->func->set_gain(aw_haptic, aw_haptic->gain);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t seq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t i = 0;
	size_t count = 0;
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_wav_seq(aw_haptic, AW_SEQUENCER_SIZE);
	mutex_unlock(&aw_haptic->lock);
	for (i = 0; i < AW_SEQUENCER_SIZE; i++)
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d = %d\n", i + 1, aw_haptic->seq[i]);

	return count;
}

static ssize_t seq_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		if (databuf[0] >= AW_SEQUENCER_SIZE || databuf[1] > aw_haptic->ram.ram_num) {
			aw_err("input value out of range!");
			return count;
		}
		aw_info("seq%d=0x%02X", databuf[0], databuf[1]);
		mutex_lock(&aw_haptic->lock);
		aw_haptic->seq[databuf[0]] = (uint8_t)databuf[1];
		aw_haptic->func->set_wav_seq(aw_haptic, (uint8_t)databuf[0],
					     aw_haptic->seq[databuf[0]]);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t loop_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	mutex_lock(&aw_haptic->lock);
	count = aw_haptic->func->get_wav_loop(aw_haptic, buf);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t loop_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_info("seq%d loop=0x%02X", databuf[0], databuf[1]);
		mutex_lock(&aw_haptic->lock);
		aw_haptic->loop[databuf[0]] = (uint8_t)databuf[1];
		aw_haptic->func->set_wav_loop(aw_haptic, (uint8_t)databuf[0],
					      aw_haptic->loop[databuf[0]]);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	len = aw_haptic->func->get_reg(aw_haptic, len, buf);
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t reg_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	uint8_t val = 0;
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		val = (uint8_t)databuf[1];
		mutex_lock(&aw_haptic->lock);
		haptic_hv_i2c_writes(aw_haptic, (uint8_t)databuf[0], &val, AW_I2C_BYTE_ONE);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t rtp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "rtp_cnt = %u\n", aw_haptic->rtp_cnt);

	return len;
}

static ssize_t rtp_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
		aw_err("kstrtouint fail");
		return rc;
	}
	mutex_lock(&aw_haptic->lock);
	if ((val > 0) && (val < aw_haptic->rtp_num)) {
		aw_haptic->state = 1;
		aw_haptic->rtp_file_num = val;
		aw_info("aw_rtp_name[%d]: %s", val, aw_rtp_name[val]);
	} else if (val == 0) {
		aw_haptic->state = 0;
	} else {
		aw_haptic->state = 0;
		aw_err("input number error:%d", val);
	}
	mutex_unlock(&aw_haptic->lock);
	queue_work(aw_haptic->work_queue, &aw_haptic->rtp_work);

	return count;
}

static ssize_t ram_update_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i = 0;
	uint8_t *ram_buf = NULL;

	aw_info("ram len = %d", aw_haptic->ram.len);
	ram_buf = kzalloc(aw_haptic->ram.len, GFP_KERNEL);
	if (!ram_buf) {
		aw_err("Error allocating memory");
		return len;
	}
	mutex_lock(&aw_haptic->lock);
	/* RAMINIT Enable */
	aw_haptic->func->ram_init(aw_haptic, true);
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->set_ram_addr(aw_haptic);
	aw_haptic->func->get_ram_data(aw_haptic, ram_buf);
	for (i = 1; i <= aw_haptic->ram.len; i++) {
		len += snprintf(buf + len, PAGE_SIZE, "0x%02x,", *(ram_buf + i - 1));
		if (i % 16 == 0 || i == aw_haptic->ram.len) {
			len = 0;
			aw_info("%s", buf);
		}
	}
	kfree(ram_buf);
	/* RAMINIT Disable */
	aw_haptic->func->ram_init(aw_haptic, false);
	len = snprintf(buf, PAGE_SIZE, "Please check log\n");
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t ram_update_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val)
		ram_update(aw_haptic);

	return count;
}

static ssize_t ram_num_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ssize_t len = 0;

	get_ram_num(aw_haptic);
	len += snprintf(buf + len, PAGE_SIZE - len, "ram_num = %d\n", aw_haptic->ram.ram_num);

	return len;
}

static ssize_t f0_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->upload_lra(aw_haptic, AW_WRITE_ZERO);
	aw_haptic->func->get_f0(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "f0 = %u\n", aw_haptic->f0);

	return len;
}

static ssize_t ram_f0_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->upload_lra(aw_haptic, AW_WRITE_ZERO);
	aw_haptic->func->ram_get_f0(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "f0 = %u\n", aw_haptic->f0);

	return len;
}

static ssize_t cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
	aw_haptic->func->get_f0(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "f0 = %u\n", aw_haptic->f0);

	return len;
}

static ssize_t cali_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val) {
		mutex_lock(&aw_haptic->lock);
		f0_cali(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t ram_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
	aw_haptic->func->ram_get_f0(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "f0 = %u\n", aw_haptic->f0);

	return len;
}

static ssize_t ram_cali_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val) {
		mutex_lock(&aw_haptic->lock);
		ram_f0_cali(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}


static ssize_t cont_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	if (val) {
		aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
		aw_haptic->func->cont_config(aw_haptic);
	}
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t vbat_monitor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_vbat(aw_haptic);
	len += snprintf(buf + len, PAGE_SIZE - len, "vbat_monitor = %u\n", aw_haptic->vbat);
	mutex_unlock(&aw_haptic->lock);

	return len;
}

static ssize_t lra_resistance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ssize_t len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->get_lra_resistance(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "lra_resistance = %u\n", aw_haptic->lra);

	return len;
}

static ssize_t auto_boost_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "auto_boost = %d\n", aw_haptic->auto_boost);

	return len;
}

static ssize_t auto_boost_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	aw_haptic->func->auto_bst_enable(aw_haptic, val);
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t prct_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint8_t reg_val = 0;

	mutex_lock(&aw_haptic->lock);
	reg_val = aw_haptic->func->get_prctmode(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "prctmode = %d\n", reg_val);

	return len;
}

static ssize_t prct_mode_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t databuf[2] = { 0, 0 };
	uint32_t prtime = 0;
	uint32_t prlvl = 0;

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		prtime = databuf[0];
		prlvl = databuf[1];
		mutex_lock(&aw_haptic->lock);
		aw_haptic->func->protect_config(aw_haptic, prtime, prlvl);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t ram_vbat_comp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "ram_vbat_comp = %d\n",
			aw_haptic->ram_vbat_comp);

	return len;
}

static ssize_t ram_vbat_comp_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&aw_haptic->lock);
	if (val)
		aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_ENABLE;
	else
		aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_DISABLE;
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t osc_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "osc_cali_data = 0x%02X\n",
			aw_haptic->osc_cali_data);

	return len;
}

static ssize_t osc_cali_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&aw_haptic->lock);
	if (val == 1) {
		rtp_trim_lra_cali(aw_haptic);
	} else if (val == 2) {
		aw_haptic->func->upload_lra(aw_haptic, AW_OSC_CALI_LRA);
		rtp_osc_cali(aw_haptic);
	}
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t haptic_audio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", aw_haptic->haptic_audio.ctr.cnt);

	return len;
}

static ssize_t haptic_audio_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	uint32_t databuf[6] = { 0 };
	struct aw_haptic_ctr *hap_ctr = NULL;

	if (!aw_haptic->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return count;
	}
	if (sscanf(buf, "%u %u %u %u %u %u", &databuf[0], &databuf[1],
		   &databuf[2], &databuf[3], &databuf[4], &databuf[5]) == 6) {
		if (databuf[2]) {
			aw_info("cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d",
				databuf[0], databuf[1], databuf[2], databuf[3],
				databuf[4], databuf[5]);
		}

		hap_ctr = kzalloc(sizeof(struct aw_haptic_ctr), GFP_KERNEL);
		if (hap_ctr == NULL)
			return count;

		mutex_lock(&aw_haptic->haptic_audio.lock);
		hap_ctr->cnt = (uint8_t)databuf[0];
		hap_ctr->cmd = (uint8_t)databuf[1];
		hap_ctr->play = (uint8_t)databuf[2];
		hap_ctr->wavseq = (uint8_t)databuf[3];
		hap_ctr->loop = (uint8_t)databuf[4];
		hap_ctr->gain = (uint8_t)databuf[5];
		audio_ctrl_list_ins(aw_haptic, hap_ctr);
		if (hap_ctr->cmd == AW_CMD_STOP) {
			aw_info("haptic_audio stop");
			if (hrtimer_active(&aw_haptic->haptic_audio.timer)) {
				aw_info("cancel haptic_audio_timer");
				hrtimer_cancel(&aw_haptic->haptic_audio.timer);
				aw_haptic->haptic_audio.ctr.cnt = 0;
				audio_off(aw_haptic);
			}
		} else {
			if (hrtimer_active(&aw_haptic->haptic_audio.timer)) {
				/* */
			} else {
				aw_info("start haptic_audio_timer");
				audio_init(aw_haptic);
				hrtimer_start(&aw_haptic->haptic_audio.timer,
					      ktime_set(aw_haptic->haptic_audio.delay_val / 1000000,
							(aw_haptic->haptic_audio.delay_val %
							 1000000) * 1000), HRTIMER_MODE_REL);
			}
		}
		mutex_unlock(&aw_haptic->haptic_audio.lock);
		kfree(hap_ctr);
	}

	return count;
}

static ssize_t haptic_audio_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "haptic_audio.delay_val=%dus\n",
			aw_haptic->haptic_audio.delay_val);
	len += snprintf(buf + len, PAGE_SIZE - len, "haptic_audio.timer_val=%dus\n",
			aw_haptic->haptic_audio.timer_val);

	return len;
}

static ssize_t haptic_audio_time_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	uint32_t databuf[2] = { 0 };

	if (sscanf(buf, "%u %u", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->haptic_audio.delay_val = databuf[0];
		aw_haptic->haptic_audio.timer_val = databuf[1];
	}

	return count;
}

static ssize_t gun_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw_haptic->gun_type);
}

static ssize_t gun_type_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_dbg("value=%d", val);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->gun_type = val;
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t bullet_nr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw_haptic->bullet_nr);
}

static ssize_t bullet_nr_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_dbg("value=%d", val);
	mutex_lock(&aw_haptic->lock);
	aw_haptic->bullet_nr = val;
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t awrw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	int i = 0;
	ssize_t len = 0;

	if (aw_haptic->i2c_info.flag != AW_SEQ_READ) {
		aw_err("no read mode");
		return -ERANGE;
	}
	if (aw_haptic->i2c_info.reg_data == NULL) {
		aw_err("awrw lack param");
		return -ERANGE;
	}
	for (i = 0; i < aw_haptic->i2c_info.reg_num; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"0x%02x,", aw_haptic->i2c_info.reg_data[i]);
	}
	len += snprintf(buf + len - 1, PAGE_SIZE - len, "\n");

	return len;
}

static ssize_t awrw_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	uint8_t value = 0;
	char data_buf[5] = { 0 };
	uint32_t flag = 0;
	uint32_t reg_num = 0;
	uint32_t reg_addr = 0;
	int i = 0;
	int rc = 0;
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	if (sscanf(buf, "%x %x %x", &flag, &reg_num, &reg_addr) == 3) {
		if (!reg_num) {
			aw_err("param error");
			return -ERANGE;
		}
		aw_haptic->i2c_info.flag = flag;
		aw_haptic->i2c_info.reg_num = reg_num;
		if (aw_haptic->i2c_info.reg_data != NULL)
			kfree(aw_haptic->i2c_info.reg_data);
		aw_haptic->i2c_info.reg_data = kmalloc(reg_num, GFP_KERNEL);
		if (flag == AW_SEQ_WRITE) {
			if ((reg_num * 5) != (strlen(buf) - 3 * 5)) {
				aw_err("param error");
				return -ERANGE;
			}
			for (i = 0; i < reg_num; i++) {
				memcpy(data_buf, &buf[15 + i * 5], 4);
				data_buf[4] = '\0';
				rc = kstrtou8(data_buf, 0, &value);
				if (rc < 0) {
					aw_err("param error");
					return -ERANGE;
				}
				aw_haptic->i2c_info.reg_data[i] = value;
			}
			mutex_lock(&aw_haptic->lock);
			haptic_hv_i2c_writes(aw_haptic, (uint8_t)reg_addr,
					 aw_haptic->i2c_info.reg_data, reg_num);
			mutex_unlock(&aw_haptic->lock);
		} else if (flag == AW_SEQ_READ) {
			mutex_lock(&aw_haptic->lock);
			haptic_hv_i2c_reads(aw_haptic, reg_addr,
					 aw_haptic->i2c_info.reg_data, reg_num);
			mutex_unlock(&aw_haptic->lock);
		}
	} else {
		aw_err("param error");
	}

	return count;
}

static ssize_t f0_save_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "f0_cali_data = 0x%02X\n",
			aw_haptic->f0_cali_data);

	return len;
}

static ssize_t f0_save_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	uint32_t val = 0;
	int rc = 0;
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_haptic->f0_cali_data = val;

	return count;
}

static ssize_t osc_save_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "osc_cali_data = 0x%02X\n",
			aw_haptic->osc_cali_data);

	return len;
}

static ssize_t osc_save_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	uint32_t val = 0;
	int rc = 0;
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_haptic->osc_cali_data = val;

	return count;
}

#ifdef AW_DOUBLE
static ssize_t dual_cont_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	right->func->play_stop(right);
	left->func->play_stop(left);
	if (val) {
		right->func->upload_lra(right, AW_F0_CALI_LRA);
		left->func->upload_lra(left, AW_F0_CALI_LRA);
		right->func->cont_config(right);
		left->func->cont_config(left);
	}

	return count;
}

static ssize_t dual_index_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d %d\n", left->index, right->index);

	return len;
}

static ssize_t dual_index_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int index_l = 0;
	int index_r = 0;
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	if (sscanf(buf, "%d %d", &index_l, &index_r) == 2) {
		aw_info("index_l=%d index_r=%d", index_l, index_r);
		if (index_l > aw_haptic->ram.ram_num || index_r > aw_haptic->ram.ram_num) {
			aw_err("input value out of range!");
			return count;
		}
		mutex_lock(&aw_haptic->lock);
		left->index = index_l;
		left->func->set_repeat_seq(left, left->index);
		right->index = index_r;
		right->func->set_repeat_seq(right, right->index);
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t dual_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "dual_mode = %d\n", aw_haptic->activate_mode);
}

static ssize_t dual_mode_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&aw_haptic->lock);
	left->activate_mode = val;
	right->activate_mode = val;
	mutex_unlock(&aw_haptic->lock);

	return count;
}

static ssize_t dual_duration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&aw_haptic->timer)) {
		time_rem = hrtimer_get_remaining(&aw_haptic->timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return snprintf(buf, PAGE_SIZE, "duration = %lldms\n", time_ms);
}

static ssize_t dual_duration_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int duration_l = 0;
	int duration_r = 0;
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	if (sscanf(buf, "%d %d", &duration_l, &duration_r) == 2) {
		mutex_lock(&aw_haptic->lock);
		left->duration = duration_l;
		right->duration = duration_r;
		mutex_unlock(&aw_haptic->lock);
	}

	return count;
}

static ssize_t dual_activate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "activate = %d\n", aw_haptic->state);
}

static ssize_t dual_activate_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	uint32_t val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_info("value=%d", val);
	if (!left->ram_init || !right->ram_init) {
		aw_err("ram init failed, not allow to play!");
		return count;
	}
	mutex_lock(&aw_haptic->lock);
	if (down_trylock(&left->sema) == 0)
		up(&left->sema);
	else
		up(&left->sema);
	left->state = val;
	right->state = val;
	left->dual_flag = true;
	right->dual_flag = true;
	mutex_unlock(&aw_haptic->lock);
	queue_work(left->work_queue, &left->vibrator_work);
	queue_work(right->work_queue, &right->vibrator_work);

	return count;
}


static ssize_t dual_rtp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "left_rtp_cnt = %u\n", left->rtp_cnt);
	len += snprintf(buf + len, PAGE_SIZE - len, "right_rtp_cnt = %u\n", right->rtp_cnt);

	return len;
}

static ssize_t dual_rtp_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);
	int rtp_l = 0;
	int rtp_r = 0;

	mutex_lock(&aw_haptic->lock);
	if (down_trylock(&left->sema) == 0)
		up(&left->sema);
	else
		up(&left->sema);
	if (sscanf(buf, "%d %d", &rtp_l, &rtp_r) == 2) {
		if (rtp_l > 0 && rtp_l < aw_haptic->rtp_num) {
			left->state = 1;
			left->rtp_file_num = rtp_l;
		} else if (rtp_l == 0) {
			left->state = 0;
		} else {
			left->state = 0;
			aw_err("input number error:%d", rtp_l);
		}
		if (rtp_r > 0 && rtp_r < aw_haptic->rtp_num) {
			right->state = 1;
			right->rtp_file_num = rtp_r;
		} else if (rtp_r == 0) {
			right->state = 0;
		} else {
			right->state = 0;
			aw_err("input number error:%d", rtp_r);
		}
	}
	left->dual_flag = true;
	right->dual_flag = true;
	mutex_unlock(&aw_haptic->lock);
	queue_work(left->work_queue, &left->rtp_work);
	queue_work(right->work_queue, &right->rtp_work);

	return count;
}
#endif

static DEVICE_ATTR_RO(f0);
static DEVICE_ATTR_RO(ram_f0);
static DEVICE_ATTR_RW(seq);
static DEVICE_ATTR_RW(reg);
static DEVICE_ATTR_RW(vmax);
static DEVICE_ATTR_RW(gain);
static DEVICE_ATTR_RW(loop);
static DEVICE_ATTR_RW(rtp);
static DEVICE_ATTR_RW(cali);
static DEVICE_ATTR_RW(ram_cali);
static DEVICE_ATTR_WO(cont);
static DEVICE_ATTR_RW(awrw);
static DEVICE_ATTR_RW(state);
static DEVICE_ATTR_RW(index);
static DEVICE_ATTR_RO(ram_num);
static DEVICE_ATTR_RW(duration);
static DEVICE_ATTR_RW(activate);
static DEVICE_ATTR_RW(osc_cali);
static DEVICE_ATTR_RW(gun_type);
static DEVICE_ATTR_RW(prct_mode);
static DEVICE_ATTR_RW(bullet_nr);
static DEVICE_ATTR_RW(auto_boost);
static DEVICE_ATTR_RW(ram_update);
static DEVICE_ATTR_RW(haptic_audio);
static DEVICE_ATTR_RO(vbat_monitor);
static DEVICE_ATTR_RW(activate_mode);
static DEVICE_ATTR_RW(ram_vbat_comp);
static DEVICE_ATTR_RO(lra_resistance);
static DEVICE_ATTR_RW(haptic_audio_time);
static DEVICE_ATTR_RW(osc_save);
static DEVICE_ATTR_RW(f0_save);

#ifdef AW_DOUBLE
static DEVICE_ATTR_WO(dual_cont);
static DEVICE_ATTR_RW(dual_index);
static DEVICE_ATTR_RW(dual_mode);
static DEVICE_ATTR_RW(dual_duration);
static DEVICE_ATTR_RW(dual_activate);
static DEVICE_ATTR_RW(dual_rtp);
#endif

static struct attribute *vibrator_attributes[] = {
	&dev_attr_state.attr,
	&dev_attr_duration.attr,
	&dev_attr_activate.attr,
	&dev_attr_activate_mode.attr,
	&dev_attr_index.attr,
	&dev_attr_vmax.attr,
	&dev_attr_gain.attr,
	&dev_attr_seq.attr,
	&dev_attr_loop.attr,
	&dev_attr_reg.attr,
	&dev_attr_rtp.attr,
	&dev_attr_ram_update.attr,
	&dev_attr_ram_num.attr,
	&dev_attr_f0.attr,
	&dev_attr_ram_f0.attr,
	&dev_attr_f0_save.attr,
	&dev_attr_cali.attr,
	&dev_attr_ram_cali.attr,
	&dev_attr_cont.attr,
	&dev_attr_vbat_monitor.attr,
	&dev_attr_lra_resistance.attr,
	&dev_attr_auto_boost.attr,
	&dev_attr_prct_mode.attr,
	&dev_attr_ram_vbat_comp.attr,
	&dev_attr_osc_cali.attr,
	&dev_attr_osc_save.attr,
	&dev_attr_haptic_audio.attr,
	&dev_attr_haptic_audio_time.attr,
	&dev_attr_gun_type.attr,
	&dev_attr_bullet_nr.attr,
	&dev_attr_awrw.attr,
#ifdef AW_DOUBLE
	&dev_attr_dual_cont.attr,
	&dev_attr_dual_index.attr,
	&dev_attr_dual_mode.attr,
	&dev_attr_dual_duration.attr,
	&dev_attr_dual_activate.attr,
	&dev_attr_dual_rtp.attr,
#endif

	NULL
};

static struct attribute_group vibrator_attribute_group = {
	.attrs = vibrator_attributes
};

#ifdef AW_TIKTAP
static inline unsigned int tiktap_get_sys_msecs(void)
{
#if (KERNEL_VERSION(5, 4, 0) <= LINUX_VERSION_CODE)
	struct timespec64 ts64;

	ktime_get_coarse_real_ts64(&ts64);
#else
	struct timespec64 ts64 = current_kernel_time64();
#endif

	return jiffies_to_msecs(timespec64_to_jiffies(&ts64));
}

static void tiktap_work_routine(struct work_struct *work)
{
	struct aw_haptic *aw_haptic = container_of(work, struct aw_haptic, tiktap_work);
	struct mmap_buf_format *tiktap_buf = aw_haptic->start_buf;
	int count = 100;
#ifdef AW_DOUBLE
	int sync_cnt = 40;
#endif
	unsigned char reg_val = 0x10;
	unsigned char glb_state_val = 0;
	unsigned int write_start;
	unsigned int buf_cnt = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->tiktap_stop_flag = false;
	aw_haptic->func->play_mode(aw_haptic, AW_RTP_MODE);
	aw_haptic->func->upload_lra(aw_haptic, AW_F0_CALI_LRA);
	while (true && count--) {
		if (tiktap_buf->status == MMAP_BUF_DATA_VALID) {
			aw_haptic->func->play_go(aw_haptic, true);
			mdelay(1);
			break;
		} else if (aw_haptic->tiktap_stop_flag == true) {
			mutex_unlock(&aw_haptic->lock);
			return;
		}
		mdelay(1);
	}
	if (count <= 0) {
		aw_err("wait 100 ms but start_buf->status != VALID! status = 0x%02x",
		       tiktap_buf->status);
		aw_haptic->tiktap_stop_flag = true;
		mutex_unlock(&aw_haptic->lock);
		return;
	}
	aw_haptic->tiktap_ready = true;
	mutex_unlock(&aw_haptic->lock);

#ifdef AW_DOUBLE /* double tiktap work sync */
	while (sync_cnt--) {
		if ((left->tiktap_ready == true) && (right->tiktap_ready == true)) {
			/* aw_info("double vib is ready! start write tiktap data."); */
			break;
		} else if (aw_haptic->tiktap_stop_flag == true) {
			aw_haptic->tiktap_ready = false;
			return;
		}
		udelay(500);
	}
	if (sync_cnt <= 0) {
		aw_err("wait 20ms but double vib not ready!");
		aw_haptic->tiktap_stop_flag = true;
		aw_haptic->tiktap_ready = false;
		return;
	}
#endif

	mutex_lock(&aw_haptic->rtp_lock);
	pm_qos_enable(aw_haptic, true);
	write_start = tiktap_get_sys_msecs();
	while (true) {
		if (tiktap_get_sys_msecs() > (write_start + 800)) {
			aw_err("Failed! tiktap endless loop");
			break;
		}
		reg_val = aw_haptic->func->rtp_get_fifo_aes(aw_haptic);
		glb_state_val = aw_haptic->func->get_glb_state(aw_haptic);
		if ((glb_state_val & AW_GLBRD_STATE_MASK) != AW_STATE_RTP) {
			aw_err("tiktap glb_state != RTP_GO!, glb_state = 0x%02x", glb_state_val);
			break;
		}
		if ((aw_haptic->tiktap_stop_flag == true) ||
		    (tiktap_buf->status == MMAP_BUF_DATA_FINISHED) ||
		    (tiktap_buf->status == MMAP_BUF_DATA_INVALID)) {
			aw_err("tiktap exit! tiktap_buf->status = 0x%02x", tiktap_buf->status);
			break;
		} else if ((
#ifdef AW_DOUBLE
			   (aw_haptic == left && (tiktap_buf->status == TIKTAP_L_VALID_R_FINISHED)) ||
			   (aw_haptic == right && (tiktap_buf->status == TIKTAP_R_VALID_L_FINISHED)) ||
#endif
			   tiktap_buf->status == MMAP_BUF_DATA_VALID) && (reg_val & 0x01)) {
			aw_info("buf_cnt = %d, bit = %d, length = %d!",
				buf_cnt, tiktap_buf->bit, tiktap_buf->length);

			aw_haptic->func->set_rtp_data(aw_haptic, tiktap_buf->data, tiktap_buf->length);
#ifdef AW_DOUBLE
			if (aw_haptic == left) {
				tiktap_buf->status &= TIkTAP_LEFT_STATUS_MASK;
				tiktap_buf->status |= TIkTAP_LEFT_FINISHED_DONE; /* left done */
			} else if (aw_haptic == right) {
				tiktap_buf->status &= TIKTAP_RIGHT_STATUS_MASK;
				tiktap_buf->status |= TIKTAP_RIGHT_FINISHED_DONE; /* right done */
			}
#else
			tiktap_buf->status = MMAP_BUF_DATA_FINISHED;
#endif
			tiktap_buf = tiktap_buf->kernel_next;
			write_start = tiktap_get_sys_msecs();
			buf_cnt++;
		} else {
			mdelay(1);
		}
	}
	pm_qos_enable(aw_haptic, false);
	aw_haptic->tiktap_stop_flag = true;
	mutex_unlock(&aw_haptic->rtp_lock);
}

static void tiktap_clean_buf(struct aw_haptic *aw_haptic, int status)
{
	struct mmap_buf_format *tiktap_buf = aw_haptic->start_buf;
	int i = 0;

	for (i = 0; i < AW_TIKTAP_MMAP_BUF_SUM; i++) {
		tiktap_buf->status = status;
		tiktap_buf = tiktap_buf->kernel_next;
	}
}

static long tiktap_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct aw_haptic *aw_haptic = g_aw_haptic;
	unsigned int tmp = 0;

	int ret = 0;

	switch (cmd) {
	case TIKTAP_GET_HWINFO:
		aw_info("cmd = TIKTAP_GET_HWINFO!");
		tmp = aw_haptic->chipid;
		if (copy_to_user((void __user *)arg, &tmp, sizeof(unsigned int)))
			ret = -EFAULT;
		break;
	case TIKTAP_GET_F0:
		aw_info("cmd = TIKTAP_GET_F0!");
		tmp = aw_haptic->f0;
		if (copy_to_user((void __user *)arg, &tmp, sizeof(unsigned int)))
			ret = -EFAULT;
		break;
	case TIKTAP_STOP_MODE:
		aw_info("cmd = TIKTAP_STOP_MODE!");
		tiktap_clean_buf(aw_haptic, MMAP_BUF_DATA_INVALID);
#ifdef AW_DOUBLE
		left->tiktap_stop_flag = true;
		right->tiktap_stop_flag = true;
		left->tiktap_ready = false;
		right->tiktap_ready = false;
		mutex_lock(&left->lock);
		left->func->play_stop(left);
		mutex_unlock(&left->lock);
		mutex_lock(&right->lock);
		right->func->play_stop(right);
		mutex_unlock(&right->lock);
#else
		aw_haptic->tiktap_stop_flag = true;
		mutex_lock(&aw_haptic->lock);
		aw_haptic->func->play_stop(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
#endif
		break;
	case TIKTAP_RTP_MODE:
		/* aw_info("cmd = TIKTAP_RTP_MODE!"); */
		tiktap_clean_buf(aw_haptic, MMAP_BUF_DATA_INVALID);
#ifdef AW_DOUBLE
		left->tiktap_ready = false;
		right->tiktap_ready = false;
		left->tiktap_stop_flag = true;
		right->tiktap_stop_flag = true;

		mutex_lock(&left->lock);
		left->func->play_stop(left);
		mutex_unlock(&left->lock);
		mutex_lock(&right->lock);
		right->func->play_stop(right);
		mutex_unlock(&right->lock);

		queue_work(aw_haptic->work_queue, &right->tiktap_work);
		queue_work(aw_haptic->work_queue, &left->tiktap_work);
#else
		aw_haptic->tiktap_stop_flag = true;
		mutex_lock(&aw_haptic->lock);
		aw_haptic->func->play_stop(aw_haptic);
		mutex_unlock(&aw_haptic->lock);
		queue_work(aw_haptic->work_queue, &aw_haptic->tiktap_work);
#endif
		break;
	default:
		aw_info("unknown cmd = %d", cmd);
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long tiktap_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	ret = tiktap_unlocked_ioctl(file, cmd, arg);

	return ret;
}
#endif

static int tiktap_file_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long phys;
	struct aw_haptic *aw_haptic = g_aw_haptic;
	int ret = 0;

#if KERNEL_VERSION(4, 7, 0) < LINUX_VERSION_CODE
	vm_flags_t vm_flags = calc_vm_prot_bits(PROT_READ|PROT_WRITE, 0) |
			      calc_vm_flag_bits(MAP_SHARED);

	vm_flags |= current->mm->def_flags | VM_MAYREAD | VM_MAYWRITE |
		    VM_MAYEXEC | VM_SHARED | VM_MAYSHARE;

	if (vma && (pgprot_val(vma->vm_page_prot) != pgprot_val(vm_get_page_prot(vm_flags)))) {
		aw_err("vm_page_prot error!");
		return -EPERM;
	}

	if (vma && ((vma->vm_end - vma->vm_start) != (PAGE_SIZE << AW_TIKTAP_MMAP_PAGE_ORDER))) {
		aw_err("mmap size check err!");
		return -EPERM;
	}
#endif
	phys = virt_to_phys(aw_haptic->start_buf);

	ret = remap_pfn_range(vma, vma->vm_start, (phys >> PAGE_SHIFT),
			      (vma->vm_end - vma->vm_start), vma->vm_page_prot);
	if (ret) {
		aw_err("mmap failed!");
		return ret;
	}

	aw_info("success!");

	return ret;
}

#ifdef KERNEL_OVER_5_10
static const struct proc_ops tiktap_proc_ops = {
	.proc_mmap = tiktap_file_mmap,
	.proc_ioctl = tiktap_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.proc_compat_ioctl = tiktap_compat_ioctl,
#endif
};
#else
static const struct file_operations tiktap_proc_ops = {
	.owner = THIS_MODULE,
	.mmap = tiktap_file_mmap,
	.unlocked_ioctl = tiktap_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tiktap_compat_ioctl,
#endif
};
#endif

static int tiktap_init(struct aw_haptic *aw_haptic)
{
	/* contrl double vib init once */
	static int tiktap_init;
	static struct mmap_buf_format *tiktap_start_buf;
	static struct proc_dir_entry *tiktap_config_proc;

	if (tiktap_init == 0) {
		/* Create proc file node */
		tiktap_config_proc = NULL;
		tiktap_config_proc = proc_create(AW_TIKTAP_PROCNAME, 0664, NULL, &tiktap_proc_ops);
		if (tiktap_config_proc == NULL) {
			aw_err("create proc entry %s failed", AW_TIKTAP_PROCNAME);
			return -EPERM;
		}
		aw_info("create proc entry %s success", AW_TIKTAP_PROCNAME);

		/* Construct shared memory */
		tiktap_start_buf = (struct mmap_buf_format *)__get_free_pages(GFP_KERNEL, AW_TIKTAP_MMAP_PAGE_ORDER);
		if (tiktap_start_buf == NULL) {
			aw_err("Error __get_free_pages failed");
			return -ENOMEM;
		}
		SetPageReserved(virt_to_page(tiktap_start_buf));
		{
			struct mmap_buf_format *temp;
			unsigned int i = 0;

			temp = tiktap_start_buf;
			for (i = 1; i < AW_TIKTAP_MMAP_BUF_SUM; i++) {
				temp->kernel_next = (tiktap_start_buf + i);
				temp = temp->kernel_next;
			}
			temp->kernel_next = tiktap_start_buf;

			temp = tiktap_start_buf;
			for (i = 0; i < AW_TIKTAP_MMAP_BUF_SUM; i++) {
				temp->bit = i;
				temp = temp->kernel_next;
			}
		}

	}
	tiktap_init = 1;

	aw_haptic->aw_config_proc = tiktap_config_proc;
	aw_haptic->start_buf = tiktap_start_buf;
	/* init flag and work */
	aw_haptic->tiktap_stop_flag = true;
	aw_haptic->tiktap_ready = false;
	INIT_WORK(&aw_haptic->tiktap_work, tiktap_work_routine);

	return 0;
}
#endif

static int vibrator_init(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	aw_info("enter");

	ret = sysfs_create_group(&aw_haptic->i2c->dev.kobj, &vibrator_attribute_group);
	if (ret < 0) {
		aw_err("error creating sysfs attr files");
		return ret;
	}
	hrtimer_init(&aw_haptic->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw_haptic->timer.function = vibrator_timer_func;
	INIT_WORK(&aw_haptic->vibrator_work, vibrator_work_routine);
	INIT_WORK(&aw_haptic->rtp_work, rtp_work_routine);
	mutex_init(&aw_haptic->lock);
	mutex_init(&aw_haptic->rtp_lock);
	sema_init(&aw_haptic->sema, 1);

	return 0;
}

static void haptic_init(struct aw_haptic *aw_haptic)
{
	aw_info("enter");
	/* haptic audio */
	aw_haptic->haptic_audio.delay_val = 1;
	aw_haptic->haptic_audio.timer_val = 21318;
	aw_haptic->rtp_num = sizeof(aw_rtp_name) / sizeof(*aw_rtp_name);
	INIT_LIST_HEAD(&(aw_haptic->haptic_audio.ctr_list));
	hrtimer_init(&aw_haptic->haptic_audio.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw_haptic->haptic_audio.timer.function = audio_timer_func;
	INIT_WORK(&aw_haptic->haptic_audio.work, audio_work_routine);
	mutex_init(&aw_haptic->haptic_audio.lock);
	INIT_LIST_HEAD(&(aw_haptic->haptic_audio.list));

	/* haptic init */
	mutex_lock(&aw_haptic->lock);
	aw_haptic->bullet_nr = 0;
	aw_haptic->gun_type = 0xff;
	aw_haptic->activate_mode = aw_haptic->info.mode;
	aw_haptic->func->play_mode(aw_haptic, AW_STANDBY_MODE);
	aw_haptic->func->set_pwm(aw_haptic, AW_PWM_24K);
	/* misc value init */
	aw_haptic->func->misc_para_init(aw_haptic);
	aw_haptic->func->set_bst_peak_cur(aw_haptic);
	aw_haptic->func->set_bst_vol(aw_haptic, aw_haptic->vmax);
	aw_haptic->func->auto_bst_enable(aw_haptic, aw_haptic->info.is_enabled_auto_bst);
	aw_haptic->ram_vbat_comp = AW_RAM_VBAT_COMP_ENABLE;
	/* f0 calibration */
	f0_cali(aw_haptic);
	mutex_unlock(&aw_haptic->lock);
}

static int aw_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret = 0;
	struct aw_haptic *aw_haptic;
	struct device_node *np = i2c->dev.of_node;

	pr_info("<%s>%s: enter\n", AW_I2C_NAME, __func__);
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		pr_err("<%s>%s: check_functionality failed\n", AW_I2C_NAME, __func__);
		return -EIO;
	}

	aw_haptic = devm_kzalloc(&i2c->dev, sizeof(struct aw_haptic), GFP_KERNEL);
	if (aw_haptic == NULL)
		return -ENOMEM;

	aw_haptic->dev = &i2c->dev;
	aw_haptic->i2c = i2c;

	i2c_set_clientdata(i2c, aw_haptic);
	dev_set_drvdata(&i2c->dev, aw_haptic);
	ret = input_framework_init(aw_haptic);
	if (ret < 0)
		goto err_parse_dt;
	/* aw_haptic rst & int */
	if (np) {
		ret = parse_dt_gpio(&i2c->dev, aw_haptic, np);
		if (ret) {
			aw_err("failed to parse gpio");
			goto err_parse_dt;
		}
	} else {
		aw_haptic->reset_gpio = -1;
		aw_haptic->irq_gpio = -1;
	}

#ifdef AW_ENABLE_PIN_CONTROL
	aw_haptic->pinctrl = devm_pinctrl_get(&i2c->dev);
	if (IS_ERR(aw_haptic->pinctrl)) {
		ret = PTR_ERR(aw_haptic->pinctrl);
		if (ret != -EPROBE_DEFER) {
			aw_err("target doesnt use pinctrl");
			aw_haptic->pinctrl = NULL;
		}
		goto err_reset_gpio_request;
	}

	for (i = 0; i < ARRAY_SIZE(aw_haptic->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state = pinctrl_lookup_state(aw_haptic->pinctrl, n);
		if (!IS_ERR(state)) {
			aw_info("pinctrl: %s found", n);
			aw_haptic->pinctrl_state[i] = state;
			continue;
		}
		aw_info("pinctrl: %s not found", n);
		goto err_reset_gpio_request;
	}

	ret = select_pin_state(aw_haptic, "awinic_interrupt_active");
	if (ret) {
		aw_info("Interrupt pinctrl state change failed!");
		goto err_reset_gpio_request; // error probe not decided yet
	}
#else
	if (gpio_is_valid(aw_haptic->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw_haptic->reset_gpio, GPIOF_OUT_INIT_LOW, "aw_rst");
		if (ret) {
			aw_err("rst request failed");
			goto err_reset_gpio_request;
		}
	}

	if (gpio_is_valid(aw_haptic->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw_haptic->irq_gpio, GPIOF_DIR_IN, "aw_int");
		if (ret) {
			aw_err("int request failed");
			goto err_irq_gpio_request;
		}
	}
#endif

	/* aw func ptr init */
	ret = ctrl_init(aw_haptic);
	if (ret < 0) {
		aw_err("ctrl_init failed ret=%d", ret);
		goto err_ctrl_init;
	}

	ret = aw_haptic->func->check_qualify(aw_haptic);
	if (ret < 0) {
		aw_err("qualify check failed ret=%d", ret);
		goto err_ctrl_init;
	}

	/* aw_haptic chip id */
	ret = parse_chipid(aw_haptic);
	if (ret < 0) {
		aw_err("parse chipid failed ret=%d", ret);
		goto err_id;
	}

	sw_reset(aw_haptic);
	ret = aw_haptic->func->offset_cali(aw_haptic);
	if (ret < 0)
		sw_reset(aw_haptic);

	aw_haptic->func->parse_dt(&i2c->dev, aw_haptic, np);

#ifdef AW_SND_SOC_CODEC
	aw_haptic->func->snd_soc_init(&i2c->dev);
#endif

	/* aw_haptic irq */
	ret = irq_config(&i2c->dev, aw_haptic);
	if (ret != 0) {
		aw_err("irq_config failed ret=%d", ret);
		goto err_irq_config;
	}

#ifdef AW_TIKTAP
	g_aw_haptic = aw_haptic;
	ret = tiktap_init(aw_haptic);
	if (ret) {
		aw_err("tiktap_init failed ret = %d", ret);
		goto err_irq_config;
	}
#endif

	vibrator_init(aw_haptic);
	haptic_init(aw_haptic);
	aw_haptic->work_queue = create_singlethread_workqueue("aw_haptic_vibrator_work_queue");
	if (!aw_haptic->work_queue) {
		aw_err("Error creating aw_haptic_vibrator_work_queue");
		goto err_irq_config;
	}
	aw_haptic->func->creat_node(aw_haptic);
	ram_work_init(aw_haptic);
	aw_info("probe completed successfully!");

	return 0;

err_id:
err_ctrl_init:
#ifndef AW_ENABLE_PIN_CONTROL
err_irq_config:
	if (gpio_is_valid(aw_haptic->irq_gpio))
		devm_gpio_free(&i2c->dev, aw_haptic->irq_gpio);

err_irq_gpio_request:
	if (gpio_is_valid(aw_haptic->reset_gpio))
		devm_gpio_free(&i2c->dev, aw_haptic->reset_gpio);
#endif

err_parse_dt:
err_reset_gpio_request:
	devm_kfree(&i2c->dev, aw_haptic);
	aw_haptic = NULL;
	return ret;
}

static int aw_remove(struct i2c_client *i2c)
{
	struct aw_haptic *aw_haptic = i2c_get_clientdata(i2c);

	aw_info("enter");
	cancel_work_sync(&aw_haptic->gain_work);
	cancel_work_sync(&aw_haptic->input_vib_work);
	input_unregister_device(aw_haptic->input_dev);
	input_ff_destroy(aw_haptic->input_dev);
	cancel_delayed_work_sync(&aw_haptic->ram_work);
	cancel_work_sync(&aw_haptic->haptic_audio.work);
	hrtimer_cancel(&aw_haptic->haptic_audio.timer);
	cancel_work_sync(&aw_haptic->rtp_work);
	cancel_work_sync(&aw_haptic->vibrator_work);
	hrtimer_cancel(&aw_haptic->timer);
	mutex_destroy(&aw_haptic->lock);
	mutex_destroy(&aw_haptic->rtp_lock);
	mutex_destroy(&aw_haptic->haptic_audio.lock);
	destroy_workqueue(aw_haptic->work_queue);
	devm_free_irq(&i2c->dev, gpio_to_irq(aw_haptic->irq_gpio), aw_haptic);
#ifdef AW_SND_SOC_CODEC
#ifdef KERNEL_OVER_4_19
	snd_soc_unregister_component(&i2c->dev);
#else
	snd_soc_unregister_codec(&i2c->dev);
#endif
#endif
#ifdef AW_TIKTAP
	proc_remove(aw_haptic->aw_config_proc);
	aw_haptic->aw_config_proc = NULL;
	free_pages((unsigned long)aw_haptic->start_buf, AW_TIKTAP_MMAP_PAGE_ORDER);
	aw_haptic->start_buf = NULL;
#endif
#ifndef AW_ENABLE_PIN_CONTROL
	if (gpio_is_valid(aw_haptic->irq_gpio))
		devm_gpio_free(&i2c->dev, aw_haptic->irq_gpio);
	if (gpio_is_valid(aw_haptic->reset_gpio))
		devm_gpio_free(&i2c->dev, aw_haptic->reset_gpio);
#endif

	return 0;
}

static int aw_i2c_suspend(struct device *dev)
{
	int ret = 0;
	struct aw_haptic *aw_haptic = dev_get_drvdata(dev);

	mutex_lock(&aw_haptic->lock);
	aw_haptic->func->play_stop(aw_haptic);
	mutex_unlock(&aw_haptic->lock);

	return ret;
}

static int aw_i2c_resume(struct device *dev)
{
	int ret = 0;

	return ret;
}

static SIMPLE_DEV_PM_OPS(aw_pm_ops, aw_i2c_suspend, aw_i2c_resume);

static const struct i2c_device_id aw_i2c_id[] = {
	{AW_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw_i2c_id);

static const struct of_device_id aw_dt_match[] = {
#ifdef AW_DOUBLE
	{.compatible = "awinic,haptic_hv_r"},
	{.compatible = "awinic,haptic_hv_l"},
#else
	{.compatible = "awinic,haptic_hv"},
#endif
	{},
};

static struct i2c_driver aw_i2c_driver = {
	.driver = {
		   .name = AW_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(aw_dt_match),
#ifdef CONFIG_PM_SLEEP
		   .pm = &aw_pm_ops,
#endif
		   },
	.probe = aw_i2c_probe,
	.remove = aw_remove,
	.id_table = aw_i2c_id,
};

static int __init aw_i2c_init(void)
{
	int ret = 0;

	pr_info("<%s>%s: aw_haptic driver version %s\n", AW_I2C_NAME, __func__,
		HAPTIC_HV_DRIVER_VERSION);
	ret = i2c_add_driver(&aw_i2c_driver);
	if (ret) {
		pr_err("<%s>%s: fail to add aw_haptic device into i2c\n", AW_I2C_NAME, __func__);
		return ret;
	}

	return 0;
}
module_init(aw_i2c_init);

static void __exit aw_i2c_exit(void)
{
	i2c_del_driver(&aw_i2c_driver);
}
module_exit(aw_i2c_exit);

MODULE_DESCRIPTION("AWINIC Haptic Driver");
MODULE_LICENSE("GPL v2");
