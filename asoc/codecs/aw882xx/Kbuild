# We can build either as part of a standalone Kernel build or as
# an external module.  Determine which mechanism is being used
ifeq ($(MODNAME),)
	KERNEL_BUILD := 1
else
	KERNEL_BUILD := 0
endif

ifeq ($(KERNEL_BUILD), 1)
	AUDIO_ROOT := $(srctree)/techpack/audio
else
	ifeq ($(CONFIG_ARCH_LAHAINA), y)
		include $(AUDIO_ROOT)/config/lahainaauto.conf
		INCS    +=  -include $(AUDIO_ROOT)/config/lahainaautoconf.h
	endif
	ifeq ($(CONFIG_MACH_XIAOMI_REDWOOD), y)
		include $(AUDIO_ROOT)/config/redwoodaudio.conf
		export
		INCS    +=  -include $(AUDIO_ROOT)/config/redwoodaudioconf.h
	endif
	ifeq ($(CONFIG_MACH_XIAOMI_LISA), y)
		include $(AUDIO_ROOT)/config/lisaaudio.conf
		export
		INCS    +=  -include $(AUDIO_ROOT)/config/lisaaudioconf.h
	endif
endif

# As per target team, build is done as follows:
# Defconfig : build with default flags
# Slub      : defconfig  + CONFIG_SLUB_DEBUG := y +
#	      CONFIG_SLUB_DEBUG_ON := y + CONFIG_PAGE_POISONING := y
# Perf      : Using appropriate msmXXXX-perf_defconfig
#
# Shipment builds (user variants) should not have any debug feature
# enabled. This is identified using 'TARGET_BUILD_VARIANT'. Slub builds
# are identified using the CONFIG_SLUB_DEBUG_ON configuration. Since
# there is no other way to identify defconfig builds, QTI internal
# representation of perf builds (identified using the string 'perf'),
# is used to identify if the build is a slub or defconfig one. This
# way no critical debug feature will be enabled for perf and shipment
# builds. Other OEMs are also protected using the TARGET_BUILD_VARIANT
# config.

############ UAPI ############
UAPI_INC :=	-I$(AUDIO_ROOT)/include/uapi/audio

############ COMMON ############
COMMON_INC :=	-I$(AUDIO_ROOT)/include

############ AW882XX ############
PRIV_INC := -I$(AUDIO_ROOT)/asoc/codecs/aw882xx/inc

# for AW882XX Codec
ifdef CONFIG_SND_SOC_AW882XX
	AW882XX_OBJS += src/aw882xx.o
	AW882XX_OBJS += src/aw882xx_bin_parse.o
	AW882XX_OBJS += src/aw882xx_calib.o
	AW882XX_OBJS += src/aw882xx_device.o
	AW882XX_OBJS += src/aw882xx_dsp.o
	AW882XX_OBJS += src/aw882xx_init.o
	AW882XX_OBJS += src/aw882xx_monitor.o
	AW882XX_OBJS += src/aw882xx_spin.o
endif

LINUX_INC += -Iinclude/linux

INCS +=	$(COMMON_INC) $(UAPI_INC) $(PRIV_INC)

EXTRA_CFLAGS += $(INCS)


CDEFINES +=	-DANI_LITTLE_BYTE_ENDIAN \
		-DANI_LITTLE_BIT_ENDIAN \
		-DDOT11F_LITTLE_ENDIAN_HOST \
		-DANI_COMPILER_TYPE_GCC \
		-DANI_OS_TYPE_ANDROID=6 \
		-DPTT_SOCK_SVC_ENABLE \
		-Wall\
		-Werror\
		-D__linux__

KBUILD_CPPFLAGS += $(CDEFINES)

# Currently, for versions of gcc which support it, the kernel Makefile
# is disabling the maybe-uninitialized warning.  Re-enable it for the
# AUDIO driver.  Note that we must use EXTRA_CFLAGS here so that it
# will override the kernel settings.
ifeq ($(call cc-option-yn, -Wmaybe-uninitialized),y)
EXTRA_CFLAGS += -Wmaybe-uninitialized
endif
ifeq ($(call cc-option-yn, -Wheader-guard),y)
EXTRA_CFLAGS += -Wheader-guard
endif

# Module information used by KBuild framework
obj-$(CONFIG_SND_SOC_AW882XX) += aw882xx_dlkm.o
aw882xx_dlkm-y := $(AW882XX_OBJS)

# inject some build related information
DEFINES += -DBUILD_TIMESTAMP=\"$(shell date -u +'%Y-%m-%dT%H:%M:%SZ')\"
