KBUILD_OPTIONS += AUDIO_ROOT=$(KERNEL_SRC)/$(M)
KBUILD_OPTIONS += MODNAME?=aw882xx_dlkm

modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS) VERBOSE=1
modules_install:
	$(MAKE) M=$(M) -C $(KERNEL_SRC) modules_install
clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean
