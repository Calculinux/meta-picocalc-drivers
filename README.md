# meta-picocalc-drivers

OpenEmbedded/Yocto Project development layer for out-of-tree Linux kernel drivers for the PicoCalc device and related hardware.

## About

This layer provides BitBake recipes for building and installing kernel drivers that are not included in the mainline Linux kernel or are specific to the PicoCalc device. The layer is designed to be modular and extensible, allowing for easy addition of new driver recipes.

## Layer Dependencies

This layer depends on:
- meta-openembedded
- poky (core)

## Compatibility

This layer is compatible with the Yocto Project **walnascar** release.

## Setup

1. Clone this repository to your Yocto build environment:
   ```bash
   git clone <repository-url> /path/to/your/layers/meta-picocalc-drivers
   ```

2. Add the layer to your `conf/bblayers.conf`:
   ```
   BBLAYERS ?= " \\
     /path/to/poky/meta \\
     /path/to/poky/meta-poky \\
     /path/to/poky/meta-yocto-bsp \\
     /path/to/your/layers/meta-picocalc-drivers \\
     "
   ```

3. Add any required driver packages to your image or `conf/local.conf`:
   ```
   IMAGE_INSTALL:append = " picocalc-drivers-all"
   ```
   
   Or add individual drivers as needed:
   ```
   IMAGE_INSTALL:append = " picocalc-kbd picocalc-lcd"
   ```

## Available Drivers

This layer provides the following PicoCalc hardware drivers:

### Legacy Drivers
- **picocalc-kbd** - Keyboard driver for PicoCalc hardware
- **picocalc-lcd** - ILI9488 framebuffer driver for LCD display
- **picocalc-snd-pwm** - Hardware PWM-based sound driver
- **picocalc-snd-softpwm** - Software PWM-based sound driver

### MFD (Multi-Function Device) Drivers
The new modular architecture provides improved hardware abstraction:
- **picocalc-mfd** - Core MFD driver (required for MFD child drivers)
- **picocalc-mfd-bms** - Battery management system driver
- **picocalc-mfd-bkl** - LCD backlight control driver
- **picocalc-mfd-kbd** - Modular keyboard driver

### Meta Packages
- **picocalc-drivers-all** - Installs all legacy drivers and recommends MFD drivers

## Usage

### Installing All Drivers
```
IMAGE_INSTALL:append = " picocalc-drivers-all"
```

### Installing Specific Driver Sets
Legacy drivers only:
```
IMAGE_INSTALL:append = " picocalc-kbd picocalc-lcd picocalc-snd-pwm"
```

MFD drivers only:
```
IMAGE_INSTALL:append = " picocalc-mfd picocalc-mfd-bms picocalc-mfd-bkl picocalc-mfd-kbd"
```

### Adding New Drivers

To add a new driver to this layer:

1. Create a new recipe file in `recipes-bsp/drivers/`
2. Follow the BitBake recipe format for kernel modules
3. Ensure proper licensing information is included
4. Test the recipe builds successfully

### Example Recipe Structure

Driver recipes should inherit from the `module` class and include proper source handling, compilation, and installation routines. See the existing recipes in `recipes-bsp/drivers/` for examples.

## Contributing

When contributing new driver recipes:

1. Ensure all source code licensing is compatible and properly documented
2. Test builds on the target hardware when possible
3. Follow Yocto Project coding standards and best practices
4. Update this README if adding significant new functionality

## License

This layer is released under the MIT License. See the `LICENSE` file for details.

## Support

For issues and questions related to this layer, please refer to the Calculinux project documentation or file an issue in the project repository.