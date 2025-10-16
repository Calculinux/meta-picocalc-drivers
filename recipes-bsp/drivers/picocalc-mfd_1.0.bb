FILESEXTRAPATHS:prepend := "${THISDIR}:"

SUMMARY = "PicoCalc MFD core driver"
DESCRIPTION = "Multi-function device core driver for modular PicoCalc hardware support"
LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"

inherit module

PV = "1.0"
PR = "r0"

SRC_URI = "file://picocalc/picocalc_mfd"

S = "${WORKDIR}/picocalc/picocalc_mfd"

EXTRA_OEMAKE:append = " KSRC=${STAGING_KERNEL_DIR}"

COMPATIBLE_MACHINE = "luckfox-lyra"

do_install() {
    install -d ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${B}/picocalc_mfd.ko ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/
}