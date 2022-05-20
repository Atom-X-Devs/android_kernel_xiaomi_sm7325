ifneq ($(CONFIG_ARCH_QTI_VM), y)
ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY), y)
dtbo-$(CONFIG_ARCH_YUPIK) +=    \
	yupik-rumi-overlay.dtbo \
	yupik-idp-overlay.dtbo \
	yupik-atp-overlay.dtbo \
	yupik-qrd-overlay.dtbo \
	yupik-idps-amoled-overlay.dtbo \
	yupik-idp-pm7250b-overlay.dtbo \
	yupik-idp-hsp-overlay.dtbo \
	yupikp-idp-overlay.dtbo \
	yupikp-atp-overlay.dtbo \
	yupikp-qrd-overlay.dtbo \
	yupikp-idps-amoled-overlay.dtbo \
	yupikp-idp-hsp-overlay.dtbo \
	yupik-iot-idp-overlay.dtbo \
	yupik-iot-idp-pm7250b-overlay.dtbo \
	yupik-iot-qrd-overlay.dtbo \
	yupik-iot-hsp-overlay.dtbo \
	yupik-iot-hsp-pm7250b-overlay.dtbo \
	yupik-iot-idps-amoled-overlay.dtbo \
	yupikp-iot-idp-overlay.dtbo \
	yupikp-iot-idp-pm7250b-overlay.dtbo \
	yupikp-iot-qrd-overlay.dtbo \
	yupikp-iot-hsp-overlay.dtbo \
	yupikp-iot-hsp-pm7250b-overlay.dtbo \
	yupikp-iot-idps-amoled-overlay.dtbo

yupik-rumi-overlay.dtbo-base := yupik.dtb
yupik-idp-overlay.dtbo-base := yupik.dtb
yupik-atp-overlay.dtbo-base := yupik.dtb
yupik-qrd-overlay.dtbo-base := yupik.dtb
yupik-idps-amoled-overlay.dtbo-base := yupik.dtb
yupik-idp-pm7250b-overlay.dtbo-base := yupik.dtb
yupik-idp-hsp-overlay.dtbo-base := yupik-hsp.dtb
yupikp-idp-overlay.dtbo-base := yupikp.dtb
yupikp-atp-overlay.dtbo-base := yupikp.dtb
yupikp-qrd-overlay.dtbo-base := yupikp.dtb
yupikp-idps-amoled-overlay.dtbo-base := yupikp.dtb
yupikp-idp-hsp-overlay.dtbo-base := yupikp-hsp.dtb
yupik-iot-idp-overlay.dtbo-base := yupik-iot.dtb
yupik-iot-idp-pm7250b-overlay.dtbo-base := yupik-iot.dtb
yupik-iot-qrd-overlay.dtbo-base := yupik-iot.dtb
yupik-iot-hsp-overlay.dtbo-base := yupik-iot.dtb
yupik-iot-hsp-pm7250b-overlay.dtbo-base := yupik-iot.dtb
yupik-iot-idps-amoled-overlay.dtbo-base := yupik-iot.dtb
yupikp-iot-idp-overlay.dtbo-base := yupikp-iot.dtb
yupikp-iot-idp-pm7250b-overlay.dtbo-base := yupikp-iot.dtb
yupikp-iot-qrd-overlay.dtbo-base := yupikp-iot.dtb
yupikp-iot-hsp-overlay.dtbo-base := yupikp-iot.dtb
yupikp-iot-hsp-pm7250b-overlay.dtbo-base := yupikp-iot.dtb
yupikp-iot-idps-amoled-overlay.dtbo-base := yupikp-iot.dtb
else
dtb-$(CONFIG_ARCH_YUPIK) += yupik-rumi.dtb \
	yupik-idp.dtb \
	yupik-atp.dtb \
	yupik-qrd.dtb \
	yupik-idps-amoled.dtb \
	yupik-idp-pm7250b.dtb \
	yupik-idp-hsp.dtb \
	yupikp-idp.dtb \
	yupikp-atp.dtb \
	yupikp-qrd.dtb \
	yupikp-idps-amoled.dtb \
	yupikp-idp-hsp.dtb \
	yupik-iot-idp.dtb \
	yupik-iot-idp-pm7250b.dtb \
	yupik-iot-qrd.dtb \
	yupik-iot-hsp.dtb \
	yupik-iot-idps-amoled.dtb \
	yupikp-iot-idp.dtb \
	yupikp-iot-idp-pm7250b.dtb \
	yupikp-iot-qrd.dtb \
	yupikp-iot-hsp.dtb \
	yupik-iot-hsp-pm7250b.dtb \
	yupikp-iot-hsp-pm7250b.dtb \
	yupikp-iot-idps-amoled.dtb
endif
endif

ifeq ($(CONFIG_ARCH_YUPIK), y)
ifeq ($(CONFIG_ARCH_QTI_VM), y)
dtb-$(CONFIG_ARCH_QTI_VM) += yupik-vm-rumi.dtb \
	yupik-vm-atp.dtb \
	yupik-vm-idp.dtb \
	yupik-vm-qrd.dtb
endif
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
