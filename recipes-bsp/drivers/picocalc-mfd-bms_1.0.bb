FILESEXTRAPATHS:prepend := "${THISDIR}:"

SUMMARY = "PicoCalc MFD battery management driver"
DESCRIPTION = "Battery management system driver for PicoCalc MFD architecture"
LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"

inherit module

PV = "1.0"
PR = "r0"

SRC_URI = "file://picocalc/picocalc_mfd_bms"

S = "${WORKDIR}/picocalc/picocalc_mfd_bms"

EXTRA_OEMAKE:append = " KSRC=${STAGING_KERNEL_DIR}"

COMPATIBLE_MACHINE = "luckfox-lyra"

# Build-time dependency on MFD headers and runtime dependency on MFD driver
DEPENDS += "picocalc-mfd"
RDEPENDS:${PN} = "picocalc-mfd"

# Add include path for MFD headers
EXTRA_OEMAKE:append = " CFLAGS_MODULE=\"-I${STAGING_INCDIR}\""

do_install() {
    install -d ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${B}/picocalc_mfd_bms.ko ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/
}