FILESEXTRAPATHS:prepend := "${THISDIR}:"

SUMMARY = "PicoCalc soft PWM sound driver"
DESCRIPTION = "Software PWM-based sound driver for PicoCalc"
LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"

inherit module

PV = "1.0"
PR = "r0"

SRC_URI = "file://picocalc/picocalc_snd-softpwm"

S = "${WORKDIR}/picocalc/picocalc_snd-softpwm"

EXTRA_OEMAKE:append = " KSRC=${STAGING_KERNEL_DIR}"

COMPATIBLE_MACHINE = "luckfox-lyra"

do_install() {
    install -d ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${B}/picocalc_snd_softpwm.ko ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/
}