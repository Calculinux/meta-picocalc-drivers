SUMMARY = "PicoCalc All Drivers Package"
DESCRIPTION = "Meta-package to install all PicoCalc hardware drivers"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

PV = "1.0"
PR = "r0"

COMPATIBLE_MACHINE = "luckfox-lyra"

# Include all MFD drivers and original drivers (except legacy keyboard)
RDEPENDS:${PN} = " \
    picocalc-lcd \
    picocalc-snd-pwm \
    picocalc-snd-softpwm \
    picocalc-mfd \
    picocalc-mfd-bms \
    picocalc-mfd-bkl \
    picocalc-mfd-kbd \
"

# This is a meta-package, no files to install
ALLOW_EMPTY:${PN} = "1"