FILESEXTRAPATHS:prepend := "${THISDIR}:"

SUMMARY = "PicoCalc MFD LED driver"
DESCRIPTION = "Keyboard backlight LED driver for PicoCalc MFD architecture"
LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"

inherit module

PV = "1.0"
PR = "r0"

SRC_URI = "file://picocalc/picocalc_mfd_led"

S = "${WORKDIR}/picocalc/picocalc_mfd_led"

EXTRA_OEMAKE:append = " KSRC=${STAGING_KERNEL_DIR}"

COMPATIBLE_MACHINE = "luckfox-lyra"

# This driver depends on the MFD core driver
RDEPENDS:${PN} += "picocalc-mfd"

do_install() {
    install -d ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${B}/picocalc_mfd_led.ko ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/
}