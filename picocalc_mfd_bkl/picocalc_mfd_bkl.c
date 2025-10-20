// SPDX-License-Identifier: GPL-2.0-only
/*
 * Backlight Driver for PicoCalc MFD
 */

#include <linux/backlight.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/ioport.h>

struct picocalc_mfd_bkl {
    struct regmap *regmap;
    struct backlight_device *bldev;
    unsigned int reg;
};

static int picocalc_bkl_update_status(struct backlight_device *bldev)
{
    struct picocalc_mfd_bkl *bkl = bl_get_data(bldev);
    int brightness = bldev->props.brightness;

    return regmap_write(bkl->regmap, (bkl->reg | (1<<7)) , brightness);
}

static int picocalc_bkl_get_brightness(struct backlight_device *bldev)
{
    struct picocalc_mfd_bkl *bkl = bl_get_data(bldev);
    u8 buf[2];
    int ret;

    ret = regmap_bulk_read(bkl->regmap, bkl->reg, buf, 2);
    if (ret < 0)
        return ret;

    /* The high byte contains the current brightness */
    return buf[1];
}

static const struct backlight_ops picocalc_bkl_ops = {
    .options = BL_CORE_SUSPENDRESUME,
    .update_status = picocalc_bkl_update_status,
    .get_brightness = picocalc_bkl_get_brightness,
};

static int picocalc_mfd_bkl_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct picocalc_mfd_bkl *bkl;
    struct backlight_properties props;

    bkl = devm_kzalloc(dev, sizeof(*bkl), GFP_KERNEL);
    if (!bkl)
        return -ENOMEM;

    u32 reg_addr;
    if (of_property_read_u32(pdev->dev.of_node, "reg", &reg_addr)) {
        dev_err(&pdev->dev, "Failed to get reg property\n");
        return -EINVAL;
    }
    bkl->reg = reg_addr;

    bkl->regmap = dev_get_regmap(dev->parent, NULL);
    if (!bkl->regmap) {
        dev_err(dev, "Failed to get parent regmap\n");
        return -EINVAL;
    }

    memset(&props, 0, sizeof(struct backlight_properties));
    props.type = BACKLIGHT_RAW;
    props.max_brightness = 255;

    bkl->bldev = devm_backlight_device_register(dev, dev_name(dev),
                                                dev->parent, bkl,
                                                &picocalc_bkl_ops, &props);
    if (IS_ERR(bkl->bldev)) {
        dev_err(dev, "Failed to register backlight device (error code: %ld)\n", PTR_ERR(bkl->bldev));
        return PTR_ERR(bkl->bldev);
    }
    
    if (!dev->of_node->phandle) {
        backlight_update_status(bkl->bldev);
    }

    platform_set_drvdata(pdev, bkl);

    return 0;
}

static const struct of_device_id picocalc_mfd_bkl_of_match[] = {
    { .compatible = "picocalc_mfd_bkl", },
    {}
};
MODULE_DEVICE_TABLE(of, picocalc_mfd_bkl_of_match);

static struct platform_driver picocalc_mfd_bkl_driver = {
    .driver = {
        .name = "picocalc_mfd_bkl",
        .of_match_table = picocalc_mfd_bkl_of_match,
    },
    .probe = picocalc_mfd_bkl_probe,
};

module_platform_driver(picocalc_mfd_bkl_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("John Laur <johnl@blurbco.com>");
MODULE_DESCRIPTION("LCD Backlight driver for PicoCalc MFD");