// SPDX-License-Identifier: GPL-2.0
/*
 * ioctl32.c: Conversion between 32bit and 64bit native ioctls.
 *
 * Copyright (C) 1997-2000  Jakub Jelinek  (jakub@redhat.com)
 * Copyright (C) 1998  Eddie C. Dost  (ecd@skynet.be)
 * Copyright (C) 2001,2002  Andi Kleen, SuSE Labs 
 * Copyright (C) 2003       Pavel Machek (pavel@ucw.cz)
 *
 * These routines maintain argument size conversion between 32bit and 64bit
 * ioctls.
 */

#include <linux/joystick.h>

#include <linux/types.h>
#include <linux/compat.h>
#include <linux/kernel.h>
#include <linux/capability.h>
#include <linux/compiler.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/ioctl.h>
#include <linux/if.h>
#include <linux/raid/md_u.h>
#include <linux/falloc.h>
#include <linux/file.h>
#include <linux/ppp-ioctl.h>
#include <linux/if_pppox.h>
#include <linux/tty.h>
#include <linux/vt_kern.h>
#include <linux/raw.h>
#include <linux/blkdev.h>
#include <linux/pci.h>
#include <linux/serial.h>
#include <linux/ctype.h>
#include <linux/syscalls.h>
#include <linux/gfp.h>
#include <linux/cec.h>

#include "internal.h"

#ifdef CONFIG_BLOCK
#include <linux/cdrom.h>
#include <linux/fd.h>
#include <scsi/scsi.h>
#include <scsi/scsi_ioctl.h>
#include <scsi/sg.h>
#endif

#include <linux/uaccess.h>
#include <linux/watchdog.h>

#include <linux/soundcard.h>

#include <linux/hiddev.h>


#include <linux/sort.h>

#ifdef CONFIG_SPARC
#include <linux/fb.h>
#include <asm/fbio.h>
#endif

#define convert_in_user(srcptr, dstptr)			\
({							\
	typeof(*srcptr) val;				\
							\
	get_user(val, srcptr) || put_user(val, dstptr);	\
})

static int do_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err;

	err = security_file_ioctl(file, cmd, arg);
	if (err)
		return err;

	return vfs_ioctl(file, cmd, arg);
}

#ifdef CONFIG_BLOCK
typedef struct sg_io_hdr32 {
	compat_int_t interface_id;	/* [i] 'S' for SCSI generic (required) */
	compat_int_t dxfer_direction;	/* [i] data transfer direction  */
	unsigned char cmd_len;		/* [i] SCSI command length ( <= 16 bytes) */
	unsigned char mx_sb_len;		/* [i] max length to write to sbp */
	unsigned short iovec_count;	/* [i] 0 implies no scatter gather */
	compat_uint_t dxfer_len;		/* [i] byte count of data transfer */
	compat_uint_t dxferp;		/* [i], [*io] points to data transfer memory
					      or scatter gather list */
	compat_uptr_t cmdp;		/* [i], [*i] points to command to perform */
	compat_uptr_t sbp;		/* [i], [*o] points to sense_buffer memory */
	compat_uint_t timeout;		/* [i] MAX_UINT->no timeout (unit: millisec) */
	compat_uint_t flags;		/* [i] 0 -> default, see SG_FLAG... */
	compat_int_t pack_id;		/* [i->o] unused internally (normally) */
	compat_uptr_t usr_ptr;		/* [i->o] unused internally */
	unsigned char status;		/* [o] scsi status */
	unsigned char masked_status;	/* [o] shifted, masked scsi status */
	unsigned char msg_status;		/* [o] messaging level data (optional) */
	unsigned char sb_len_wr;		/* [o] byte count actually written to sbp */
	unsigned short host_status;	/* [o] errors from host adapter */
	unsigned short driver_status;	/* [o] errors from software driver */
	compat_int_t resid;		/* [o] dxfer_len - actual_transferred */
	compat_uint_t duration;		/* [o] time taken by cmd (unit: millisec) */
	compat_uint_t info;		/* [o] auxiliary information */
} sg_io_hdr32_t;  /* 64 bytes long (on sparc32) */

typedef struct sg_iovec32 {
	compat_uint_t iov_base;
	compat_uint_t iov_len;
} sg_iovec32_t;

static int sg_build_iovec(sg_io_hdr_t __user *sgio, void __user *dxferp, u16 iovec_count)
{
	sg_iovec_t __user *iov = (sg_iovec_t __user *) (sgio + 1);
	sg_iovec32_t __user *iov32 = dxferp;
	int i;

	for (i = 0; i < iovec_count; i++) {
		u32 base, len;

		if (get_user(base, &iov32[i].iov_base) ||
		    get_user(len, &iov32[i].iov_len) ||
		    put_user(compat_ptr(base), &iov[i].iov_base) ||
		    put_user(len, &iov[i].iov_len))
			return -EFAULT;
	}

	if (put_user(iov, &sgio->dxferp))
		return -EFAULT;
	return 0;
}

static int sg_ioctl_trans(struct file *file, unsigned int cmd,
			sg_io_hdr32_t __user *sgio32)
{
	sg_io_hdr_t __user *sgio;
	u16 iovec_count;
	u32 data;
	void __user *dxferp;
	int err;
	int interface_id;

	if (get_user(interface_id, &sgio32->interface_id))
		return -EFAULT;
	if (interface_id != 'S')
		return do_ioctl(file, cmd, (unsigned long)sgio32);

	if (get_user(iovec_count, &sgio32->iovec_count))
		return -EFAULT;

	{
		void __user *top = compat_alloc_user_space(0);
		void __user *new = compat_alloc_user_space(sizeof(sg_io_hdr_t) +
				       (iovec_count * sizeof(sg_iovec_t)));
		if (new > top)
			return -EINVAL;

		sgio = new;
	}

	/* Ok, now construct.  */
	if (copy_in_user(&sgio->interface_id, &sgio32->interface_id,
			 (2 * sizeof(int)) +
			 (2 * sizeof(unsigned char)) +
			 (1 * sizeof(unsigned short)) +
			 (1 * sizeof(unsigned int))))
		return -EFAULT;

	if (get_user(data, &sgio32->dxferp))
		return -EFAULT;
	dxferp = compat_ptr(data);
	if (iovec_count) {
		if (sg_build_iovec(sgio, dxferp, iovec_count))
			return -EFAULT;
	} else {
		if (put_user(dxferp, &sgio->dxferp))
			return -EFAULT;
	}

	{
		unsigned char __user *cmdp;
		unsigned char __user *sbp;

		if (get_user(data, &sgio32->cmdp))
			return -EFAULT;
		cmdp = compat_ptr(data);

		if (get_user(data, &sgio32->sbp))
			return -EFAULT;
		sbp = compat_ptr(data);

		if (put_user(cmdp, &sgio->cmdp) ||
		    put_user(sbp, &sgio->sbp))
			return -EFAULT;
	}

	if (copy_in_user(&sgio->timeout, &sgio32->timeout,
			 3 * sizeof(int)))
		return -EFAULT;

	if (get_user(data, &sgio32->usr_ptr))
		return -EFAULT;
	if (put_user(compat_ptr(data), &sgio->usr_ptr))
		return -EFAULT;

	err = do_ioctl(file, cmd, (unsigned long) sgio);

	if (err >= 0) {
		void __user *datap;

		if (copy_in_user(&sgio32->pack_id, &sgio->pack_id,
				 sizeof(int)) ||
		    get_user(datap, &sgio->usr_ptr) ||
		    put_user((u32)(unsigned long)datap,
			     &sgio32->usr_ptr) ||
		    copy_in_user(&sgio32->status, &sgio->status,
				 (4 * sizeof(unsigned char)) +
				 (2 * sizeof(unsigned short)) +
				 (3 * sizeof(int))))
			err = -EFAULT;
	}

	return err;
}

struct compat_sg_req_info { /* used by SG_GET_REQUEST_TABLE ioctl() */
	char req_state;
	char orphan;
	char sg_io_owned;
	char problem;
	int pack_id;
	compat_uptr_t usr_ptr;
	unsigned int duration;
	int unused;
};

static int sg_grt_trans(struct file *file,
		unsigned int cmd, struct compat_sg_req_info __user *o)
{
	int err, i;
	sg_req_info_t __user *r;
	r = compat_alloc_user_space(sizeof(sg_req_info_t)*SG_MAX_QUEUE);
	err = do_ioctl(file, cmd, (unsigned long)r);
	if (err < 0)
		return err;
	for (i = 0; i < SG_MAX_QUEUE; i++) {
		void __user *ptr;
		int d;

		if (copy_in_user(o + i, r + i, offsetof(sg_req_info_t, usr_ptr)) ||
		    get_user(ptr, &r[i].usr_ptr) ||
		    get_user(d, &r[i].duration) ||
		    put_user((u32)(unsigned long)(ptr), &o[i].usr_ptr) ||
		    put_user(d, &o[i].duration))
			return -EFAULT;
	}
	return err;
}
#endif /* CONFIG_BLOCK */

struct sock_fprog32 {
	unsigned short	len;
	compat_caddr_t	filter;
};

#define PPPIOCSPASS32	_IOW('t', 71, struct sock_fprog32)
#define PPPIOCSACTIVE32	_IOW('t', 70, struct sock_fprog32)

static int ppp_sock_fprog_ioctl_trans(struct file *file,
		unsigned int cmd, struct sock_fprog32 __user *u_fprog32)
{
	struct sock_fprog __user *u_fprog64 = compat_alloc_user_space(sizeof(struct sock_fprog));
	void __user *fptr64;
	u32 fptr32;
	u16 flen;

	if (get_user(flen, &u_fprog32->len) ||
	    get_user(fptr32, &u_fprog32->filter))
		return -EFAULT;

	fptr64 = compat_ptr(fptr32);

	if (put_user(flen, &u_fprog64->len) ||
	    put_user(fptr64, &u_fprog64->filter))
		return -EFAULT;

	if (cmd == PPPIOCSPASS32)
		cmd = PPPIOCSPASS;
	else
		cmd = PPPIOCSACTIVE;

	return do_ioctl(file, cmd, (unsigned long) u_fprog64);
}

struct ppp_option_data32 {
	compat_caddr_t	ptr;
	u32			length;
	compat_int_t		transmit;
};
#define PPPIOCSCOMPRESS32	_IOW('t', 77, struct ppp_option_data32)

struct ppp_idle32 {
	compat_time_t xmit_idle;
	compat_time_t recv_idle;
};
#define PPPIOCGIDLE32		_IOR('t', 63, struct ppp_idle32)

static int ppp_gidle(struct file *file, unsigned int cmd,
		struct ppp_idle32 __user *idle32)
{
	struct ppp_idle __user *idle;
	__kernel_time_t xmit, recv;
	int err;

	idle = compat_alloc_user_space(sizeof(*idle));

	err = do_ioctl(file, PPPIOCGIDLE, (unsigned long) idle);

	if (!err) {
		if (get_user(xmit, &idle->xmit_idle) ||
		    get_user(recv, &idle->recv_idle) ||
		    put_user(xmit, &idle32->xmit_idle) ||
		    put_user(recv, &idle32->recv_idle))
			err = -EFAULT;
	}
	return err;
}

static int ppp_scompress(struct file *file, unsigned int cmd,
	struct ppp_option_data32 __user *odata32)
{
	struct ppp_option_data __user *odata;
	__u32 data;
	void __user *datap;

	odata = compat_alloc_user_space(sizeof(*odata));

	if (get_user(data, &odata32->ptr))
		return -EFAULT;

	datap = compat_ptr(data);
	if (put_user(datap, &odata->ptr))
		return -EFAULT;

	if (copy_in_user(&odata->length, &odata32->length,
			 sizeof(__u32) + sizeof(int)))
		return -EFAULT;

	return do_ioctl(file, PPPIOCSCOMPRESS, (unsigned long) odata);
}

/*
 * simple reversible transform to make our table more evenly
 * distributed after sorting.
 */
#define XFORM(i) (((i) ^ ((i) << 27) ^ ((i) << 17)) & 0xffffffff)

#define COMPATIBLE_IOCTL(cmd) XFORM((u32)cmd),
/* ioctl should not be warned about even if it's not implemented.
   Valid reasons to use this:
   - It is implemented with ->compat_ioctl on some device, but programs
   call it on others too.
   - The ioctl is not implemented in the native kernel, but programs
   call it commonly anyways.
   Most other reasons are not valid. */
#define IGNORE_IOCTL(cmd) COMPATIBLE_IOCTL(cmd)

static unsigned int ioctl_pointer[] = {
/* compatible ioctls first */
/* Little t */
COMPATIBLE_IOCTL(TIOCOUTQ)
/* 'X' - originally XFS but some now in the VFS */
COMPATIBLE_IOCTL(FITRIM)
#ifdef CONFIG_BLOCK
/* Big S */
COMPATIBLE_IOCTL(SCSI_IOCTL_GET_IDLUN)
COMPATIBLE_IOCTL(SCSI_IOCTL_DOORLOCK)
COMPATIBLE_IOCTL(SCSI_IOCTL_DOORUNLOCK)
COMPATIBLE_IOCTL(SCSI_IOCTL_TEST_UNIT_READY)
COMPATIBLE_IOCTL(SCSI_IOCTL_GET_BUS_NUMBER)
COMPATIBLE_IOCTL(SCSI_IOCTL_SEND_COMMAND)
COMPATIBLE_IOCTL(SCSI_IOCTL_PROBE_HOST)
COMPATIBLE_IOCTL(SCSI_IOCTL_GET_PCI)
#endif
/* Big V (don't complain on serial console) */
IGNORE_IOCTL(VT_OPENQRY)
IGNORE_IOCTL(VT_GETMODE)
/*
 * These two are only for the sbus rtc driver, but
 * hwclock tries them on every rtc device first when
 * running on sparc.  On other architectures the entries
 * are useless but harmless.
 */
COMPATIBLE_IOCTL(_IOR('p', 20, int[7])) /* RTCGET */
COMPATIBLE_IOCTL(_IOW('p', 21, int[7])) /* RTCSET */
#ifdef CONFIG_BLOCK
/* md calls this on random blockdevs */
IGNORE_IOCTL(RAID_VERSION)
/* qemu/qemu-img might call these two on plain files for probing */
IGNORE_IOCTL(CDROM_DRIVE_STATUS)
IGNORE_IOCTL(FDGETPRM32)
/* SG stuff */
COMPATIBLE_IOCTL(SG_SET_TIMEOUT)
COMPATIBLE_IOCTL(SG_GET_TIMEOUT)
COMPATIBLE_IOCTL(SG_EMULATED_HOST)
COMPATIBLE_IOCTL(SG_GET_TRANSFORM)
COMPATIBLE_IOCTL(SG_SET_RESERVED_SIZE)
COMPATIBLE_IOCTL(SG_GET_RESERVED_SIZE)
COMPATIBLE_IOCTL(SG_GET_SCSI_ID)
COMPATIBLE_IOCTL(SG_SET_FORCE_LOW_DMA)
COMPATIBLE_IOCTL(SG_GET_LOW_DMA)
COMPATIBLE_IOCTL(SG_SET_FORCE_PACK_ID)
COMPATIBLE_IOCTL(SG_GET_PACK_ID)
COMPATIBLE_IOCTL(SG_GET_NUM_WAITING)
COMPATIBLE_IOCTL(SG_SET_DEBUG)
COMPATIBLE_IOCTL(SG_GET_SG_TABLESIZE)
COMPATIBLE_IOCTL(SG_GET_COMMAND_Q)
COMPATIBLE_IOCTL(SG_SET_COMMAND_Q)
COMPATIBLE_IOCTL(SG_GET_VERSION_NUM)
COMPATIBLE_IOCTL(SG_NEXT_CMD_LEN)
COMPATIBLE_IOCTL(SG_SCSI_RESET)
COMPATIBLE_IOCTL(SG_GET_REQUEST_TABLE)
COMPATIBLE_IOCTL(SG_SET_KEEP_ORPHAN)
COMPATIBLE_IOCTL(SG_GET_KEEP_ORPHAN)
#endif
/* PPP stuff */
COMPATIBLE_IOCTL(PPPIOCGFLAGS)
COMPATIBLE_IOCTL(PPPIOCSFLAGS)
COMPATIBLE_IOCTL(PPPIOCGASYNCMAP)
COMPATIBLE_IOCTL(PPPIOCSASYNCMAP)
COMPATIBLE_IOCTL(PPPIOCGUNIT)
COMPATIBLE_IOCTL(PPPIOCGRASYNCMAP)
COMPATIBLE_IOCTL(PPPIOCSRASYNCMAP)
COMPATIBLE_IOCTL(PPPIOCGMRU)
COMPATIBLE_IOCTL(PPPIOCSMRU)
COMPATIBLE_IOCTL(PPPIOCSMAXCID)
COMPATIBLE_IOCTL(PPPIOCGXASYNCMAP)
COMPATIBLE_IOCTL(PPPIOCSXASYNCMAP)
COMPATIBLE_IOCTL(PPPIOCXFERUNIT)
/* PPPIOCSCOMPRESS is translated */
COMPATIBLE_IOCTL(PPPIOCGNPMODE)
COMPATIBLE_IOCTL(PPPIOCSNPMODE)
COMPATIBLE_IOCTL(PPPIOCGDEBUG)
COMPATIBLE_IOCTL(PPPIOCSDEBUG)
/* PPPIOCSPASS is translated */
/* PPPIOCSACTIVE is translated */
/* PPPIOCGIDLE is translated */
COMPATIBLE_IOCTL(PPPIOCNEWUNIT)
COMPATIBLE_IOCTL(PPPIOCATTACH)
COMPATIBLE_IOCTL(PPPIOCDETACH)
COMPATIBLE_IOCTL(PPPIOCSMRRU)
COMPATIBLE_IOCTL(PPPIOCCONNECT)
COMPATIBLE_IOCTL(PPPIOCDISCONN)
COMPATIBLE_IOCTL(PPPIOCATTCHAN)
COMPATIBLE_IOCTL(PPPIOCGCHAN)
COMPATIBLE_IOCTL(PPPIOCGL2TPSTATS)
/* Big A */
/* sparc only */
/* Big Q for sound/OSS */
COMPATIBLE_IOCTL(SNDCTL_SEQ_RESET)
COMPATIBLE_IOCTL(SNDCTL_SEQ_SYNC)
COMPATIBLE_IOCTL(SNDCTL_SYNTH_INFO)
COMPATIBLE_IOCTL(SNDCTL_SEQ_CTRLRATE)
COMPATIBLE_IOCTL(SNDCTL_SEQ_GETOUTCOUNT)
COMPATIBLE_IOCTL(SNDCTL_SEQ_GETINCOUNT)
COMPATIBLE_IOCTL(SNDCTL_SEQ_PERCMODE)
COMPATIBLE_IOCTL(SNDCTL_FM_LOAD_INSTR)
COMPATIBLE_IOCTL(SNDCTL_SEQ_TESTMIDI)
COMPATIBLE_IOCTL(SNDCTL_SEQ_RESETSAMPLES)
COMPATIBLE_IOCTL(SNDCTL_SEQ_NRSYNTHS)
COMPATIBLE_IOCTL(SNDCTL_SEQ_NRMIDIS)
COMPATIBLE_IOCTL(SNDCTL_MIDI_INFO)
COMPATIBLE_IOCTL(SNDCTL_SEQ_THRESHOLD)
COMPATIBLE_IOCTL(SNDCTL_SYNTH_MEMAVL)
COMPATIBLE_IOCTL(SNDCTL_FM_4OP_ENABLE)
COMPATIBLE_IOCTL(SNDCTL_SEQ_PANIC)
COMPATIBLE_IOCTL(SNDCTL_SEQ_OUTOFBAND)
COMPATIBLE_IOCTL(SNDCTL_SEQ_GETTIME)
COMPATIBLE_IOCTL(SNDCTL_SYNTH_ID)
COMPATIBLE_IOCTL(SNDCTL_SYNTH_CONTROL)
COMPATIBLE_IOCTL(SNDCTL_SYNTH_REMOVESAMPLE)
/* Big T for sound/OSS */
COMPATIBLE_IOCTL(SNDCTL_TMR_TIMEBASE)
COMPATIBLE_IOCTL(SNDCTL_TMR_START)
COMPATIBLE_IOCTL(SNDCTL_TMR_STOP)
COMPATIBLE_IOCTL(SNDCTL_TMR_CONTINUE)
COMPATIBLE_IOCTL(SNDCTL_TMR_TEMPO)
COMPATIBLE_IOCTL(SNDCTL_TMR_SOURCE)
COMPATIBLE_IOCTL(SNDCTL_TMR_METRONOME)
COMPATIBLE_IOCTL(SNDCTL_TMR_SELECT)
/* Little m for sound/OSS */
COMPATIBLE_IOCTL(SNDCTL_MIDI_PRETIME)
COMPATIBLE_IOCTL(SNDCTL_MIDI_MPUMODE)
COMPATIBLE_IOCTL(SNDCTL_MIDI_MPUCMD)
/* Big P for sound/OSS */
COMPATIBLE_IOCTL(SNDCTL_DSP_RESET)
COMPATIBLE_IOCTL(SNDCTL_DSP_SYNC)
COMPATIBLE_IOCTL(SNDCTL_DSP_SPEED)
COMPATIBLE_IOCTL(SNDCTL_DSP_STEREO)
COMPATIBLE_IOCTL(SNDCTL_DSP_GETBLKSIZE)
COMPATIBLE_IOCTL(SNDCTL_DSP_CHANNELS)
COMPATIBLE_IOCTL(SOUND_PCM_WRITE_FILTER)
COMPATIBLE_IOCTL(SNDCTL_DSP_POST)
COMPATIBLE_IOCTL(SNDCTL_DSP_SUBDIVIDE)
COMPATIBLE_IOCTL(SNDCTL_DSP_SETFRAGMENT)
COMPATIBLE_IOCTL(SNDCTL_DSP_GETFMTS)
COMPATIBLE_IOCTL(SNDCTL_DSP_SETFMT)
COMPATIBLE_IOCTL(SNDCTL_DSP_GETOSPACE)
COMPATIBLE_IOCTL(SNDCTL_DSP_GETISPACE)
COMPATIBLE_IOCTL(SNDCTL_DSP_NONBLOCK)
COMPATIBLE_IOCTL(SNDCTL_DSP_GETCAPS)
COMPATIBLE_IOCTL(SNDCTL_DSP_GETTRIGGER)
COMPATIBLE_IOCTL(SNDCTL_DSP_SETTRIGGER)
COMPATIBLE_IOCTL(SNDCTL_DSP_GETIPTR)
COMPATIBLE_IOCTL(SNDCTL_DSP_GETOPTR)
/* SNDCTL_DSP_MAPINBUF,  XXX needs translation */
/* SNDCTL_DSP_MAPOUTBUF,  XXX needs translation */
COMPATIBLE_IOCTL(SNDCTL_DSP_SETSYNCRO)
COMPATIBLE_IOCTL(SNDCTL_DSP_SETDUPLEX)
COMPATIBLE_IOCTL(SNDCTL_DSP_GETODELAY)
COMPATIBLE_IOCTL(SNDCTL_DSP_PROFILE)
COMPATIBLE_IOCTL(SOUND_PCM_READ_RATE)
COMPATIBLE_IOCTL(SOUND_PCM_READ_CHANNELS)
COMPATIBLE_IOCTL(SOUND_PCM_READ_BITS)
COMPATIBLE_IOCTL(SOUND_PCM_READ_FILTER)
/* Big C for sound/OSS */
COMPATIBLE_IOCTL(SNDCTL_COPR_RESET)
COMPATIBLE_IOCTL(SNDCTL_COPR_LOAD)
COMPATIBLE_IOCTL(SNDCTL_COPR_RDATA)
COMPATIBLE_IOCTL(SNDCTL_COPR_RCODE)
COMPATIBLE_IOCTL(SNDCTL_COPR_WDATA)
COMPATIBLE_IOCTL(SNDCTL_COPR_WCODE)
COMPATIBLE_IOCTL(SNDCTL_COPR_RUN)
COMPATIBLE_IOCTL(SNDCTL_COPR_HALT)
COMPATIBLE_IOCTL(SNDCTL_COPR_SENDMSG)
COMPATIBLE_IOCTL(SNDCTL_COPR_RCVMSG)
/* Big M for sound/OSS */
COMPATIBLE_IOCTL(SOUND_MIXER_READ_VOLUME)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_BASS)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_TREBLE)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_SYNTH)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_PCM)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_SPEAKER)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_LINE)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_MIC)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_CD)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_IMIX)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_ALTPCM)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_RECLEV)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_IGAIN)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_OGAIN)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_LINE1)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_LINE2)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_LINE3)
COMPATIBLE_IOCTL(MIXER_READ(SOUND_MIXER_DIGITAL1))
COMPATIBLE_IOCTL(MIXER_READ(SOUND_MIXER_DIGITAL2))
COMPATIBLE_IOCTL(MIXER_READ(SOUND_MIXER_DIGITAL3))
COMPATIBLE_IOCTL(MIXER_READ(SOUND_MIXER_PHONEIN))
COMPATIBLE_IOCTL(MIXER_READ(SOUND_MIXER_PHONEOUT))
COMPATIBLE_IOCTL(MIXER_READ(SOUND_MIXER_VIDEO))
COMPATIBLE_IOCTL(MIXER_READ(SOUND_MIXER_RADIO))
COMPATIBLE_IOCTL(MIXER_READ(SOUND_MIXER_MONITOR))
COMPATIBLE_IOCTL(SOUND_MIXER_READ_MUTE)
/* SOUND_MIXER_READ_ENHANCE,  same value as READ_MUTE */
/* SOUND_MIXER_READ_LOUD,  same value as READ_MUTE */
COMPATIBLE_IOCTL(SOUND_MIXER_READ_RECSRC)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_DEVMASK)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_RECMASK)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_STEREODEVS)
COMPATIBLE_IOCTL(SOUND_MIXER_READ_CAPS)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_VOLUME)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_BASS)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_TREBLE)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_SYNTH)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_PCM)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_SPEAKER)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_LINE)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_MIC)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_CD)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_IMIX)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_ALTPCM)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_RECLEV)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_IGAIN)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_OGAIN)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_LINE1)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_LINE2)
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_LINE3)
COMPATIBLE_IOCTL(MIXER_WRITE(SOUND_MIXER_DIGITAL1))
COMPATIBLE_IOCTL(MIXER_WRITE(SOUND_MIXER_DIGITAL2))
COMPATIBLE_IOCTL(MIXER_WRITE(SOUND_MIXER_DIGITAL3))
COMPATIBLE_IOCTL(MIXER_WRITE(SOUND_MIXER_PHONEIN))
COMPATIBLE_IOCTL(MIXER_WRITE(SOUND_MIXER_PHONEOUT))
COMPATIBLE_IOCTL(MIXER_WRITE(SOUND_MIXER_VIDEO))
COMPATIBLE_IOCTL(MIXER_WRITE(SOUND_MIXER_RADIO))
COMPATIBLE_IOCTL(MIXER_WRITE(SOUND_MIXER_MONITOR))
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_MUTE)
/* SOUND_MIXER_WRITE_ENHANCE,  same value as WRITE_MUTE */
/* SOUND_MIXER_WRITE_LOUD,  same value as WRITE_MUTE */
COMPATIBLE_IOCTL(SOUND_MIXER_WRITE_RECSRC)
COMPATIBLE_IOCTL(SOUND_MIXER_INFO)
COMPATIBLE_IOCTL(SOUND_OLD_MIXER_INFO)
COMPATIBLE_IOCTL(SOUND_MIXER_ACCESS)
COMPATIBLE_IOCTL(SOUND_MIXER_AGC)
COMPATIBLE_IOCTL(SOUND_MIXER_3DSE)
COMPATIBLE_IOCTL(SOUND_MIXER_PRIVATE1)
COMPATIBLE_IOCTL(SOUND_MIXER_PRIVATE2)
COMPATIBLE_IOCTL(SOUND_MIXER_PRIVATE3)
COMPATIBLE_IOCTL(SOUND_MIXER_PRIVATE4)
COMPATIBLE_IOCTL(SOUND_MIXER_PRIVATE5)
COMPATIBLE_IOCTL(SOUND_MIXER_GETLEVELS)
COMPATIBLE_IOCTL(SOUND_MIXER_SETLEVELS)
COMPATIBLE_IOCTL(OSS_GETVERSION)
/* Raw devices */
COMPATIBLE_IOCTL(RAW_SETBIND)
COMPATIBLE_IOCTL(RAW_GETBIND)
/* Watchdog */
COMPATIBLE_IOCTL(WDIOC_GETSUPPORT)
COMPATIBLE_IOCTL(WDIOC_GETSTATUS)
COMPATIBLE_IOCTL(WDIOC_GETBOOTSTATUS)
COMPATIBLE_IOCTL(WDIOC_GETTEMP)
COMPATIBLE_IOCTL(WDIOC_SETOPTIONS)
COMPATIBLE_IOCTL(WDIOC_KEEPALIVE)
COMPATIBLE_IOCTL(WDIOC_SETTIMEOUT)
COMPATIBLE_IOCTL(WDIOC_GETTIMEOUT)
COMPATIBLE_IOCTL(WDIOC_SETPRETIMEOUT)
COMPATIBLE_IOCTL(WDIOC_GETPRETIMEOUT)
/* Misc. */
COMPATIBLE_IOCTL(PCIIOC_CONTROLLER)
COMPATIBLE_IOCTL(PCIIOC_MMAP_IS_IO)
COMPATIBLE_IOCTL(PCIIOC_MMAP_IS_MEM)
COMPATIBLE_IOCTL(PCIIOC_WRITE_COMBINE)
/* hiddev */
COMPATIBLE_IOCTL(HIDIOCGVERSION)
COMPATIBLE_IOCTL(HIDIOCAPPLICATION)
COMPATIBLE_IOCTL(HIDIOCGDEVINFO)
COMPATIBLE_IOCTL(HIDIOCGSTRING)
COMPATIBLE_IOCTL(HIDIOCINITREPORT)
COMPATIBLE_IOCTL(HIDIOCGREPORT)
COMPATIBLE_IOCTL(HIDIOCSREPORT)
COMPATIBLE_IOCTL(HIDIOCGREPORTINFO)
COMPATIBLE_IOCTL(HIDIOCGFIELDINFO)
COMPATIBLE_IOCTL(HIDIOCGUSAGE)
COMPATIBLE_IOCTL(HIDIOCSUSAGE)
COMPATIBLE_IOCTL(HIDIOCGUCODE)
COMPATIBLE_IOCTL(HIDIOCGFLAG)
COMPATIBLE_IOCTL(HIDIOCSFLAG)
COMPATIBLE_IOCTL(HIDIOCGCOLLECTIONINDEX)
COMPATIBLE_IOCTL(HIDIOCGCOLLECTIONINFO)
/* joystick */
COMPATIBLE_IOCTL(JSIOCGVERSION)
COMPATIBLE_IOCTL(JSIOCGAXES)
COMPATIBLE_IOCTL(JSIOCGBUTTONS)
COMPATIBLE_IOCTL(JSIOCGNAME(0))

/* fat 'r' ioctls. These are handled by fat with ->compat_ioctl,
   but we don't want warnings on other file systems. So declare
   them as compatible here. */
#define VFAT_IOCTL_READDIR_BOTH32       _IOR('r', 1, struct compat_dirent[2])
#define VFAT_IOCTL_READDIR_SHORT32      _IOR('r', 2, struct compat_dirent[2])

IGNORE_IOCTL(VFAT_IOCTL_READDIR_BOTH32)
IGNORE_IOCTL(VFAT_IOCTL_READDIR_SHORT32)

#ifdef CONFIG_SPARC
/* Sparc framebuffers, handled in sbusfb_compat_ioctl() */
IGNORE_IOCTL(FBIOGTYPE)
IGNORE_IOCTL(FBIOSATTR)
IGNORE_IOCTL(FBIOGATTR)
IGNORE_IOCTL(FBIOSVIDEO)
IGNORE_IOCTL(FBIOGVIDEO)
IGNORE_IOCTL(FBIOSCURPOS)
IGNORE_IOCTL(FBIOGCURPOS)
IGNORE_IOCTL(FBIOGCURMAX)
IGNORE_IOCTL(FBIOPUTCMAP32)
IGNORE_IOCTL(FBIOGETCMAP32)
IGNORE_IOCTL(FBIOSCURSOR32)
IGNORE_IOCTL(FBIOGCURSOR32)
#endif
};

/*
 * Convert common ioctl arguments based on their command number
 *
 * Please do not add any code in here. Instead, implement
 * a compat_ioctl operation in the place that handleѕ the
 * ioctl for the native case.
 */
static long do_ioctl_trans(unsigned int cmd,
		 unsigned long arg, struct file *file)
{
	void __user *argp = compat_ptr(arg);

	switch (cmd) {
	case PPPIOCGIDLE32:
		return ppp_gidle(file, cmd, argp);
	case PPPIOCSCOMPRESS32:
		return ppp_scompress(file, cmd, argp);
	case PPPIOCSPASS32:
	case PPPIOCSACTIVE32:
		return ppp_sock_fprog_ioctl_trans(file, cmd, argp);
#ifdef CONFIG_BLOCK
	case SG_IO:
		return sg_ioctl_trans(file, cmd, argp);
	case SG_GET_REQUEST_TABLE:
		return sg_grt_trans(file, cmd, argp);
#endif
	}

	/*
	 * These take an integer instead of a pointer as 'arg',
	 * so we must not do a compat_ptr() translation.
	 */
	switch (cmd) {
	/* RAID */
	case HOT_REMOVE_DISK:
	case HOT_ADD_DISK:
	case SET_DISK_FAULTY:
	case SET_BITMAP_FILE:
		return vfs_ioctl(file, cmd, arg);
	}

	return -ENOIOCTLCMD;
}

static int compat_ioctl_check_table(unsigned int xcmd)
{
	int i;
	const int max = ARRAY_SIZE(ioctl_pointer) - 1;

	BUILD_BUG_ON(max >= (1 << 16));

	/* guess initial offset into table, assuming a
	   normalized distribution */
	i = ((xcmd >> 16) * max) >> 16;

	/* do linear search up first, until greater or equal */
	while (ioctl_pointer[i] < xcmd && i < max)
		i++;

	/* then do linear search down */
	while (ioctl_pointer[i] > xcmd && i > 0)
		i--;

	return ioctl_pointer[i] == xcmd;
}

COMPAT_SYSCALL_DEFINE3(ioctl, unsigned int, fd, unsigned int, cmd,
		       compat_ulong_t, arg32)
{
	unsigned long arg = arg32;
	struct fd f = fdget(fd);
	int error = -EBADF;
	if (!f.file)
		goto out;

	/* RED-PEN how should LSM module know it's handling 32bit? */
	error = security_file_ioctl(f.file, cmd, arg);
	if (error)
		goto out_fput;

	switch (cmd) {
	/* these are never seen by ->ioctl(), no argument or int argument */
	case FIOCLEX:
	case FIONCLEX:
	case FIFREEZE:
	case FITHAW:
	case FICLONE:
		goto do_ioctl;
	/* these are never seen by ->ioctl(), pointer argument */
	case FIONBIO:
	case FIOASYNC:
	case FIOQSIZE:
	case FS_IOC_FIEMAP:
	case FIGETBSZ:
	case FICLONERANGE:
	case FIDEDUPERANGE:
		goto found_handler;
	/*
	 * The next group is the stuff handled inside file_ioctl().
	 * For regular files these never reach ->ioctl(); for
	 * devices, sockets, etc. they do and one (FIONREAD) is
	 * even accepted in some cases.  In all those cases
	 * argument has the same type, so we can handle these
	 * here, shunting them towards do_vfs_ioctl().
	 * ->compat_ioctl() will never see any of those.
	 */
	/* pointer argument, never actually handled by ->ioctl() */
	case FIBMAP:
		goto found_handler;
	/* handled by some ->ioctl(); always a pointer to int */
	case FIONREAD:
		goto found_handler;
	/* these two get messy on amd64 due to alignment differences */
#if defined(CONFIG_X86_64)
	case FS_IOC_RESVSP_32:
	case FS_IOC_RESVSP64_32:
		error = compat_ioctl_preallocate(f.file, compat_ptr(arg));
		goto out_fput;
#else
	case FS_IOC_RESVSP:
	case FS_IOC_RESVSP64:
		goto found_handler;
#endif

	default:
		if (f.file->f_op->compat_ioctl) {
			error = f.file->f_op->compat_ioctl(f.file, cmd, arg);
			if (error != -ENOIOCTLCMD)
				goto out_fput;
		}

		if (!f.file->f_op->unlocked_ioctl)
			goto do_ioctl;
		break;
	}

	if (compat_ioctl_check_table(XFORM(cmd)))
		goto found_handler;

	error = do_ioctl_trans(cmd, arg, f.file);
	if (error == -ENOIOCTLCMD)
		error = -ENOTTY;

	goto out_fput;

 found_handler:
	arg = (unsigned long)compat_ptr(arg);
 do_ioctl:
	error = do_vfs_ioctl(f.file, fd, cmd, arg);
 out_fput:
	fdput(f);
 out:
	return error;
}

static int __init init_sys32_ioctl_cmp(const void *p, const void *q)
{
	unsigned int a, b;
	a = *(unsigned int *)p;
	b = *(unsigned int *)q;
	if (a > b)
		return 1;
	if (a < b)
		return -1;
	return 0;
}

static int __init init_sys32_ioctl(void)
{
	sort(ioctl_pointer, ARRAY_SIZE(ioctl_pointer), sizeof(*ioctl_pointer),
		init_sys32_ioctl_cmp, NULL);
	return 0;
}
__initcall(init_sys32_ioctl);
