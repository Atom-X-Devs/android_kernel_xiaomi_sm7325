KDIR := $(TOP)/kernel_platform/common

ifeq ($(VIBRATOR_ROOT),)
VIBRATOR_ROOT=$(srctree)/techpack/vibrator
endif

ifeq ($(CONFIG_ARCH_LAHAINA), y)
	include $(VIBRATOR_ROOT)/config/gki_lahaina.conf
	LINUX_INC += -include $(VIBRATOR_ROOT)/config/gki_lahainaconf.h
endif

LINUX_INC += -Iinclude/linux -Iinclude/linux/input

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

ifeq ($(CONFIG_AWINIC_HV_HAPTICS), y)
	LINUX_INC += -include $(VIBRATOR_ROOT)/hv_haptics/haptic_hv.h

	awinic_haptic-y := ./hv_haptics/haptic_hv.o

	ifeq ($(CONFIG_AW869X_DRIVER_ENABLE), y)
		awinic_haptic-y += ./hv_haptics/aw869x.o
	endif
	ifeq ($(CONFIG_AW869XX_DRIVER_ENABLE), y)
		awinic_haptic-y += ./hv_haptics/aw869xx.o
	endif
	ifeq ($(CONFIG_AW8691X_DRIVER_ENABLE), y)
		awinic_haptic-y += ./hv_haptics/aw8671x.o
	endif
	ifeq ($(CONFIG_AW8692X_DRIVER_ENABLE), y)
		awinic_haptic-y += ./hv_haptics/aw8692x.o
	endif
	ifeq ($(CONFIG_AW8693X_DRIVER_ENABLE), y)
		awinic_haptic-y += ./hv_haptics/aw8693x.o
	endif

	obj-$(CONFIG_AWINIC_VIBRATOR) += awinic_haptic.o
endif
