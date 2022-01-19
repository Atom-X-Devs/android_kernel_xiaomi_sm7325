ccflags-y := -I$(src) -Wall -Werror

# Need to add include paths:
LINUXINCLUDE += \
             -I$(srctree)techpack/audio/include/dsp
             -I$(srctree)techpack/audio/include/uapi
             -I$(srctree)techpack/aduio/include

obj-y += mius.o
obj-y += mius_mixer_controls.o
obj-y += mius_data_msm_io.o
obj-y += mius_data_userspace_io.o
obj-y += mius_data_userspace_ctrl.o
obj-y += mius_sysfs.o
