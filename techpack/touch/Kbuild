KDIR := $(TOP)/kernel_platform/common

ifeq ($(TOUCH_ROOT),)
TOUCH_ROOT=$(srctree)/techpack/touch
endif

ifeq ($(CONFIG_ARCH_LAHAINA), y)
	include $(TOUCH_ROOT)/config/gki_lahainatouch.conf
	LINUX_INC += -include $(TOUCH_ROOT)/config/gki_lahainatouchconf.h
endif

LINUX_INC += -Iinclude/linux \
		-Iinclude/linux/drm \
		-Iinclude/linux/input

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

ccflags-y += $(LINUX_INC)

ifeq ($(call cc-option-yn, -Wmaybe-uninitialized),y)
EXTRA_CFLAGS += -Wmaybe-uninitialized
endif

ifeq ($(call cc-option-yn, -Wheader-guard),y)
EXTRA_CFLAGS += -Wheader-guard
endif

######### CONFIG_MSM_TOUCH ########

ifeq ($(CONFIG_TOUCHSCREEN_GOODIX_BRL), y)
	LINUX_INC += -include $(TOUCH_ROOT)/goodix_berlin_driver/goodix_ts_core.h

	goodix_ts-y := \
		 ./goodix_berlin_driver/goodix_ts_core.o \
		 ./goodix_berlin_driver/goodix_brl_hw.o \
		 ./goodix_berlin_driver/goodix_cfg_bin.o \
		 ./goodix_berlin_driver/goodix_ts_utils.o \
		 ./goodix_berlin_driver/goodix_brl_fwupdate.o \
		 ./goodix_berlin_driver/goodix_ts_gesture.o

	goodix_ts-$(CONFIG_TOUCHSCREEN_GOODIX_BRL_DEBUG) += \
		 ./goodix_berlin_driver/goodix_ts_inspect.o \
		 ./goodix_berlin_driver/goodix_ts_tools.o

	ifeq ($(CONFIG_TOUCHSCREEN_GOODIX_BRL_SPI), y)
		goodix_ts-y += ./goodix_berlin_driver/goodix_brl_spi.o
	else
		goodix_ts-y += ./goodix_berlin_driver/goodix_brl_i2c.o
	endif

	obj-$(CONFIG_MSM_TOUCH) += goodix_ts.o
endif

CDEFINES += -DBUILD_TIMESTAMP=\"$(shell date -u +'%Y-%m-%dT%H:%M:%SZ')\"
