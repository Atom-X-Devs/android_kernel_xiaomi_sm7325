// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2015,2020-2021 The Linux Foundation. All rights reserved.
 */

#include <linux/io.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/qcom_scm.h>
#include <linux/arm-smccc.h>
#include <linux/dma-mapping.h>

#include <asm/cacheflush.h>

#include <linux/qtee_shmbridge.h>

#include "qcom_scm.h"

#define CREATE_TRACE_POINTS
#include <trace/events/scm.h>

#include <linux/habmm.h>

/**
 * struct arm_smccc_args
 * @args:	The array of values used in registers in smc instruction
 */
struct arm_smccc_args {
	unsigned long args[8];
};

static DEFINE_MUTEX(qcom_scm_lock);

#define QCOM_SCM_EBUSY_WAIT_MS 30
#define QCOM_SCM_EBUSY_MAX_RETRY 20

#define SMCCC_N_REG_ARGS	4
#define SMCCC_FIRST_EXT_IDX	(SMCCC_N_REG_ARGS - 1)
#define SMCCC_N_EXT_ARGS	(MAX_QCOM_SCM_ARGS - SMCCC_N_REG_ARGS + 1)
#define SMCCC_FIRST_REG_IDX	2
#define SMCCC_LAST_REG_IDX	(SMCCC_FIRST_REG_IDX + SMCCC_N_REG_ARGS - 1)

#if IS_ENABLED(CONFIG_QCOM_SCM_QCPE)

#ifdef CONFIG_GHS_VMM
struct scm_extra_arg {
	union {
		u32 args32[N_EXT_SCM_ARGS];
		u64 args64[N_EXT_SCM_ARGS];
	};
};
#endif

struct smc_params_s {
	uint64_t fn_id;
	uint64_t arginfo;
	uint64_t args[MAX_SCM_ARGS];
} __packed;

static u32 handle;
static bool opened;

static int scm_qcpe_hab_open(void)
{
	int ret;

	if (!opened) {
		ret = habmm_socket_open(&handle, MM_QCPE_VM1, 0, 0);
		if (ret) {
			pr_err("habmm_socket_open failed with ret = %d\n", ret);
			return ret;
		}
		opened = true;
	}

	return 0;
}

static void scm_qcpe_hab_close(void)
{
	if (opened) {
		habmm_socket_close(handle);
		opened = false;
		handle = 0;
	}
}

/* Send SMC over HAB, receive the response. Both operations are blocking. */
/* This is meant to be called from non-atomic context. */
static int scm_qcpe_hab_send_receive(struct smc_params_s *smc_params,
	u32 *size_bytes)
{
	int ret;

	ret = habmm_socket_send(handle, smc_params, sizeof(*smc_params), 0);
	if (ret) {
		pr_err("habmm_socket_send failed, ret= 0x%x\n", ret);
		return ret;
	}

	memset(smc_params, 0x0, sizeof(*smc_params));

	do {
		*size_bytes = sizeof(*smc_params);
		ret = habmm_socket_recv(handle, smc_params, size_bytes, 0,
			HABMM_SOCKET_RECV_FLAGS_UNINTERRUPTIBLE);
	} while (-EINTR == ret);

	if (ret) {
		pr_err("habmm_socket_recv failed, ret= 0x%x\n", ret);
		return ret;
	}

	return 0;
}

/* Send SMC over HAB, receive the response, in non-blocking mode. */
/* This is meant to be called from atomic context. */
static int scm_qcpe_hab_send_receive_atomic(struct smc_params_s *smc_params,
	u32 *size_bytes)
{
	int ret;
	unsigned long delay;

	delay = jiffies + (HZ); /* 1 second delay for send */

	do {
		ret = habmm_socket_send(handle,
			smc_params, sizeof(*smc_params),
			HABMM_SOCKET_SEND_FLAGS_NON_BLOCKING);
	} while ((-EAGAIN == ret) && time_before(jiffies, delay));

	if (ret) {
		pr_err("HAB send failed, non-blocking, ret= 0x%x\n", ret);
		return ret;
	}

	memset(smc_params, 0x0, sizeof(*smc_params));

	delay = jiffies + (HZ); /* 1 second delay for receive */

	do {
		*size_bytes = sizeof(*smc_params);
		ret = habmm_socket_recv(handle, smc_params, size_bytes, 0,
			HABMM_SOCKET_RECV_FLAGS_NON_BLOCKING);
	} while ((-EAGAIN == ret) && time_before(jiffies, delay) &&
		(*size_bytes == 0));

	if (ret) {
		pr_err("HAB recv failed, non-blocking, ret= 0x%x\n", ret);
		return ret;
	}

	return 0;
}


static int scm_call_qcpe(const struct arm_smccc_args *smc,
			 struct arm_smccc_res *res, const bool atomic)
{
	u32 size_bytes;
	struct smc_params_s smc_params = {0,};
	int ret;
#ifdef CONFIG_GHS_VMM
	int i;
	uint64_t arglen = smc->a[1] & 0xf;
	struct ion_handle *ihandle = NULL;
#endif

	pr_info("SCM IN [QCPE]: 0x%x, 0x%x, 0x%llx, 0x%llx, 0x%llx, 0x%llx, 0x%llx\n",
		smc->a[0], smc->a[1], smc->a[2], smc->a[3], smc->a[4], smc->a[5],
		smc->a[5]);

	if (!opened) {
		if (!atomic) {
			if (scm_qcpe_hab_open()) {
				pr_err("HAB channel re-open failed\n");
				return -ENODEV;
			}
		} else {
			pr_err("HAB channel is not opened\n");
			return -ENODEV;
		}
	}

	smc_params.fn_id   = smc->a[0];
	smc_params.arginfo = smc->a[1];
	smc_params.args[0] = smc->a[2];
	smc_params.args[1] = smc->a[3];
	smc_params.args[2] = smc->a[4];

#ifdef CONFIG_GHS_VMM
	if (arglen <= N_REGISTER_ARGS) {
		smc_params.args[FIRST_EXT_ARG_IDX] = smc->a[5];
	} else {
		struct scm_extra_arg *argbuf =
				(struct scm_extra_arg *)desc->extra_arg_buf;
		int j = 0;

		if (scm_version == SMC_CONVENTION_ARM_64)
			for (i = FIRST_EXT_ARG_IDX; i < MAX_QCOM_SCM_ARGS; i++)
				smc_params.args[i] = argbuf->args64[j++];
		else
			for (i = FIRST_EXT_ARG_IDX; i < MAX_QCOM_SCM_ARGS; i++)
				smc_params.args[i] = argbuf->args32[j++];
	}

	ret = ionize_buffers(smc->a[0] & (~SMC64_MASK), &smc_params, &ihandle);
	if (ret)
		return ret;
#else
	smc_params.args[3] = smc->a[5];
	smc_params.args[4] = 0;
#endif

	if (!atomic) {
		ret = scm_qcpe_hab_send_receive(&smc_params, &size_bytes);
		if (ret) {
			pr_err("send/receive failed, non-atomic, ret= 0x%x\n",
				ret);
			goto err_ret;
		}
	} else {
		ret = scm_qcpe_hab_send_receive_atomic(&smc_params,
			&size_bytes);
		if (ret) {
			pr_err("send/receive failed, ret= 0x%x\n", ret);
			goto err_ret;
		}
	}

	if (size_bytes != sizeof(smc_params)) {
		pr_err("habmm_socket_recv expected size: %lu, actual=%u\n",
				sizeof(smc_params),
				size_bytes);
		ret = QCOM_SCM_ERROR;
		goto err_ret;
	}

	res->a1 = smc_params.args[1];
	res->a2 = smc_params.args[2];
	res->a3 = smc_params.args[3];
	res->a0 = smc_params.args[0];
	pr_info("SCM OUT [QCPE]: 0x%llx, 0x%llx, 0x%llx, 0x%llx\n",
		res->a0, res->a1, res->a2, res->a3);
	goto no_err;

err_ret:
	if (!atomic) {
		/* In case of an error, try to recover the hab connection
		 * for next time. This can only be done if called in
		 * non-atomic context.
		 */
		scm_qcpe_hab_close();
		if (scm_qcpe_hab_open())
			pr_err("scm_qcpe_hab_open failed\n");
		}

no_err:
#ifdef CONFIG_GHS_VMM
	if (ihandle)
		free_ion_buffers(ihandle);
#endif
	return res->a0;
}

#endif /* CONFIG_QCOM_SCM_QCPE */

static void __qcom_scm_call_do_quirk(const struct arm_smccc_args *smc,
				     struct arm_smccc_res *res,
				     const bool atomic)
{
	ktime_t time;
	const bool trace = trace_scm_call_enabled();
#if !(IS_ENABLED(CONFIG_QCOM_SCM_QCPE))
	unsigned long a0 = smc->args[0];
	struct arm_smccc_quirk quirk = { .id = ARM_SMCCC_QUIRK_QCOM_A6 };

	quirk.state.a6 = 0;
#endif
	if (trace)
		time = ktime_get();

#if IS_ENABLED(CONFIG_QCOM_SCM_QCPE)
	scm_call_qcpe(smc, res, atomic);
#else
	do {
		arm_smccc_smc_quirk(a0, smc->args[1], smc->args[2],
				    smc->args[3], smc->args[4], smc->args[5],
				    quirk.state.a6, smc->args[7], res, &quirk);

		if (res->a0 == QCOM_SCM_INTERRUPTED)
			a0 = res->a0;

	} while (res->a0 == QCOM_SCM_INTERRUPTED);
#endif
	if (trace)
		trace_scm_call(smc->args, res, ktime_us_delta(ktime_get(), time));
}

int __qcom_scm_call_smccc(struct device *dev, struct qcom_scm_desc *desc,
								enum qcom_scm_convention qcom_convention, enum qcom_scm_call_type call_type)
{
	int arglen = desc->arginfo & 0xf;
	int i, ret;
	size_t alloc_len;
	const bool atomic = (call_type == QCOM_SCM_CALL_ATOMIC);
	gfp_t flag = atomic ? GFP_ATOMIC : GFP_NOIO;
	u32 smccc_call_type = atomic ? ARM_SMCCC_FAST_CALL : ARM_SMCCC_STD_CALL;
	u32 qcom_smccc_convention = (qcom_convention == SMC_CONVENTION_ARM_32) ?
				    ARM_SMCCC_SMC_32 : ARM_SMCCC_SMC_64;
	struct arm_smccc_res res;
	struct arm_smccc_args smc = {{0}};
	struct qtee_shm shm = {0};
	bool use_qtee_shmbridge;

	smc.args[0] = ARM_SMCCC_CALL_VAL(
		smccc_call_type,
		qcom_smccc_convention,
		desc->owner,
		SMCCC_FUNCNUM(desc->svc, desc->cmd));
	smc.args[1] = desc->arginfo;
	for (i = 0; i < SMCCC_N_REG_ARGS; i++)
		smc.args[i + SMCCC_FIRST_REG_IDX] = desc->args[i];

	if (unlikely(arglen > SMCCC_N_REG_ARGS)) {
		if (!dev)
			return -EPROBE_DEFER;

		alloc_len = SMCCC_N_EXT_ARGS * sizeof(u64);
		use_qtee_shmbridge = qtee_shmbridge_is_enabled();
		if (use_qtee_shmbridge) {
			ret = qtee_shmbridge_allocate_shm(alloc_len, &shm);
			if (ret)
				return ret;
		} else {
			shm.vaddr = kzalloc(alloc_len, flag);
			if (!shm.vaddr)
				return -ENOMEM;
		}

		if (qcom_smccc_convention == SMC_CONVENTION_ARM_32) {
			__le32 *args = shm.vaddr;

			for (i = 0; i < SMCCC_N_EXT_ARGS; i++)
				args[i] = cpu_to_le32(desc->args[i +
						      SMCCC_FIRST_EXT_IDX]);
		} else {
			__le64 *args = shm.vaddr;

			for (i = 0; i < SMCCC_N_EXT_ARGS; i++)
				args[i] = cpu_to_le64(desc->args[i +
						      SMCCC_FIRST_EXT_IDX]);
		}

		shm.paddr = dma_map_single(dev, shm.vaddr, alloc_len,
						DMA_TO_DEVICE);

		if (dma_mapping_error(dev, shm.paddr)) {
			if (use_qtee_shmbridge)
				qtee_shmbridge_free_shm(&shm);
			else
				kfree(shm.vaddr);
			return -ENOMEM;
		}

		smc.args[SMCCC_LAST_REG_IDX] = shm.paddr;
	}

	if (atomic) {
		__qcom_scm_call_do_quirk(&smc, &res, true);
	} else {
		int retry_count = 0;

		do {
			mutex_lock(&qcom_scm_lock);
			__qcom_scm_call_do_quirk(&smc, &res, false);
			mutex_unlock(&qcom_scm_lock);

			if (res.a0 == QCOM_SCM_V2_EBUSY) {
				if (retry_count++ > QCOM_SCM_EBUSY_MAX_RETRY ||
				    (call_type == QCOM_SCM_CALL_NORETRY))
					break;
				msleep(QCOM_SCM_EBUSY_WAIT_MS);
			}
		} while (res.a0 == QCOM_SCM_V2_EBUSY);
	}

	if (shm.vaddr) {
		dma_unmap_single(dev, shm.paddr, alloc_len, DMA_TO_DEVICE);
		if (use_qtee_shmbridge)
			qtee_shmbridge_free_shm(&shm);
		else
			kfree(shm.vaddr);
	}

	desc->res[0] = res.a1;
	desc->res[1] = res.a2;
	desc->res[2] = res.a3;

	return res.a0 ? qcom_scm_remap_error(res.a0) : 0;
}

#if IS_ENABLED(CONFIG_QCOM_SCM_QCPE)
void __qcom_scm_qcpe_init(void)
{
/**
 * The HAB connection should be opened before first SMC call.
 * If not, there could be errors that might cause the
 * system to crash.
 */
	scm_qcpe_hab_open();
}

void __qcom_scm_qcpe_exit(void)
{
	scm_qcpe_hab_close();
}
#endif
