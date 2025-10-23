// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keyboard Driver for picocalc
 */

#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/of.h>


#include "picocalc_kbd_code.h"

//#include "config.h"
#include "debug_levels.h"

#define REG_ID_BAT (0x0b)
#define REG_ID_BKL (0x05)
#define REG_ID_FIF (0x09)
#define REG_ID_BK2 (0x0A)

#define PICOCALC_WRITE_MASK (1<<7)

#define KBD_BUS_TYPE        BUS_I2C
#define KBD_VENDOR_ID       0x0001
#define KBD_PRODUCT_ID      0x0001
#define KBD_VERSION_ID      0x0001

#define KBD_FIFO_SIZE               31



static uint64_t mouse_fast_move_thr_time = 150000000ull;
static int8_t mouse_move_step = 1;

// From keyboard firmware source
enum pico_key_state
{
    KEY_STATE_IDLE = 0,
    KEY_STATE_PRESSED = 1,
    KEY_STATE_HOLD = 2,
    KEY_STATE_RELEASED = 3,
    KEY_STATE_LONG_HOLD = 4,
};
struct key_fifo_item
{
    uint8_t _ : 4;
    enum pico_key_state state : 4;
    uint8_t scancode;
};

#define MOUSE_MOVE_LEFT  (1 << 1)
#define MOUSE_MOVE_RIGHT (1 << 2)
#define MOUSE_MOVE_UP    (1 << 3)
#define MOUSE_MOVE_DOWN  (1 << 4)

struct kbd_ctx
{
    struct work_struct work_struct;
    uint8_t version_number;

    struct i2c_client *i2c_client;
    struct regmap *regmap;
    struct input_dev *input_dev;

    // Map from input HID scancodes to Linux keycodes
    uint8_t *keycode_map;

    // Key state and touch FIFO queue
    uint8_t key_fifo_count;
    struct key_fifo_item key_fifo_data[KBD_FIFO_SIZE];
    uint64_t last_keypress_at;

    int mouse_mode;
    uint8_t mouse_move_dir;
};

// Parse 0 to 255 from string
static inline int parse_u8(char const* buf)
{
    int rc, result;

    // Parse string value
    if ((rc = kstrtoint(buf, 10, &result)) || (result < 0) || (result > 0xff)) {
        return -EINVAL;
    }
    return result;
}

// Read a single uint8_t value from I2C register
static inline int kbd_read_i2c_u8(struct kbd_ctx* ctx, uint8_t reg_addr,
    uint8_t* dst)
{
    unsigned int val;
    int ret;

    ret = regmap_read(ctx->regmap, reg_addr, &val);
    if (ret < 0) {
        dev_err(&ctx->i2c_client->dev,
            "%s Could not read from register 0x%02X, error: %d\n",
            __func__, reg_addr, ret);
        return ret;
    }

    *dst = val;
    return 0;
}

// Write a single uint8_t value to I2C register
static inline int kbd_write_i2c_u8(struct kbd_ctx* ctx, uint8_t reg_addr,
    uint8_t src)
{
    int ret;

    ret = regmap_write(ctx->regmap, reg_addr | PICOCALC_WRITE_MASK, src);
    if (ret < 0) {
        dev_err(&ctx->i2c_client->dev,
            "%s Could not write to register 0x%02X, Error: %d\n",
            __func__, reg_addr, ret);
        return ret;
    }

    return 0;
}

// Read a pair of uint8_t values from I2C register
static inline int kbd_read_i2c_2u8(struct kbd_ctx* ctx, uint8_t reg_addr,
    uint8_t* dst)
{
    int ret;

    ret = regmap_bulk_read(ctx->regmap, reg_addr, dst, 2);
    if (ret < 0) {
        dev_err(&ctx->i2c_client->dev,
            "%s Could not read from register 0x%02X, error: %d\n",
            __func__, reg_addr, ret);
        return ret;
    }

    return 0;
}

// Shared global state for global interfaces such as sysfs
struct kbd_ctx *g_ctx;

void input_fw_read_fifo(struct kbd_ctx* ctx)
{
    uint8_t fifo_idx;
    int rc;

    // Read number of FIFO items
    /*
    if (kbd_read_i2c_u8(ctx, REG_KEY, &ctx->key_fifo_count)) {
        return;
    }
    */
    ctx->key_fifo_count = 0;

    // Read and transfer all FIFO items
    for (fifo_idx = 0; fifo_idx < KBD_FIFO_SIZE; fifo_idx++) {

        uint8_t data[2];
        // Read 2 fifo items
        if ((rc = kbd_read_i2c_2u8(ctx, 0x09,
            (uint8_t*)&data))) {

            dev_err(&ctx->i2c_client->dev,
                "%s Could not read REG_FIF, Error: %d\n", __func__, rc);
            return;
        }

        if (data[0] == 0)
        {
            break;
        }

        ctx->key_fifo_data[fifo_idx]._ = 0;
        ctx->key_fifo_data[fifo_idx].state = data[0];
        ctx->key_fifo_data[fifo_idx].scancode= data[1];
        ctx->key_fifo_count++;
        // Advance FIFO position
        dev_info_fe(&ctx->i2c_client->dev,
            "%s %02d: 0x%02x%02x State %d Scancode %d\n",
            __func__, fifo_idx,
            ((uint8_t*)&ctx->key_fifo_data[fifo_idx])[0],
            ((uint8_t*)&ctx->key_fifo_data[fifo_idx])[1],
            ctx->key_fifo_data[fifo_idx].state,
            ctx->key_fifo_data[fifo_idx].scancode);
        /*
        printk("%02d: 0x%02x%02x State %d Scancode %d\n",
            fifo_idx,
            ((uint8_t*)&ctx->key_fifo_data[fifo_idx])[0],
            ((uint8_t*)&ctx->key_fifo_data[fifo_idx])[1],
            ctx->key_fifo_data[fifo_idx].state,
            ctx->key_fifo_data[fifo_idx].scancode);
        */
    }
}

static void key_report_event(struct kbd_ctx* ctx,
    struct key_fifo_item const* ev)
{
    uint8_t keycode;

    // Only handle key pressed, held, or released events
    if ((ev->state != KEY_STATE_PRESSED) && (ev->state != KEY_STATE_RELEASED)
     && (ev->state != KEY_STATE_HOLD)) {
        return;
    }

        /* right shift */
        if (ev->scancode == 0xA3)
        {
            if (ev->state == KEY_STATE_PRESSED)
            {
                ctx->mouse_mode = !ctx->mouse_mode;
            }
            return;
        }

        if (ctx->mouse_mode)
        {
            switch(ev->scancode)
            {
            /* KEY_BACKSPACE */
/*
            case '\b':
                  if (ev->state == KEY_STATE_PRESSED)
                  {
                      input_report_abs(ctx->input_dev, ABS_X, 0);
                      input_report_abs(ctx->input_dev, ABS_Y, 0);
                  }
                  return;
*/
            /* KEY_RIGHT */
            case 0xb7:
                  if (ev->state == KEY_STATE_PRESSED)
                  {
                      if (!(ctx->mouse_move_dir & MOUSE_MOVE_RIGHT))
                      ctx->last_keypress_at = ktime_get_boottime_ns();
                      ctx->mouse_move_dir |= MOUSE_MOVE_RIGHT;
                  }
                  else if (ev->state == KEY_STATE_RELEASED)
                  {
                      ctx->mouse_move_dir &= ~MOUSE_MOVE_RIGHT;
                  }
                  return;
            /* KEY_LEFT */
            case 0xb4:
                  if (ev->state == KEY_STATE_PRESSED)
                  {
                      if (!(ctx->mouse_move_dir & MOUSE_MOVE_LEFT))
                      ctx->last_keypress_at = ktime_get_boottime_ns();
                      ctx->mouse_move_dir |= MOUSE_MOVE_LEFT;
                  }
                  else if (ev->state == KEY_STATE_RELEASED)
                  {
                  ctx->last_keypress_at = ktime_get_boottime_ns();
                      ctx->mouse_move_dir &= ~MOUSE_MOVE_LEFT;
                  }
                  return;
            /* KEY_DOWN */
            case 0xb6:
                  if (ev->state == KEY_STATE_PRESSED)
                  {
                      if (!(ctx->mouse_move_dir & MOUSE_MOVE_DOWN))
                      ctx->last_keypress_at = ktime_get_boottime_ns();
                      ctx->mouse_move_dir |= MOUSE_MOVE_DOWN;
                  }
                  else if (ev->state == KEY_STATE_RELEASED)
                  {
                      ctx->mouse_move_dir &= ~MOUSE_MOVE_DOWN;
                  }
                  return;
            /* KEY_UP */
            case 0xb5:
                  if (ev->state == KEY_STATE_PRESSED)
                  {
                      if (!(ctx->mouse_move_dir & MOUSE_MOVE_UP))
                      ctx->last_keypress_at = ktime_get_boottime_ns();
                      ctx->mouse_move_dir |= MOUSE_MOVE_UP;
                  }
                  else if (ev->state == KEY_STATE_RELEASED)
                  {
                      ctx->mouse_move_dir &= ~MOUSE_MOVE_UP;
                  }
                  return;
            /* KEY_RIGHTBRACE */
            case ']':
              input_report_key(ctx->input_dev, BTN_LEFT, ev->state == KEY_STATE_PRESSED);
                  return;
            /* KEY_LEFTBRACE */
            case '[':
              input_report_key(ctx->input_dev, BTN_RIGHT, ev->state == KEY_STATE_PRESSED);
                  return;
            default:
                     break;
            }
        }

    // Post key scan event
    input_event(ctx->input_dev, EV_MSC, MSC_SCAN, ev->scancode);

    // Map input scancode to Linux input keycode
    keycode = keycodes[ev->scancode];

    dev_info_fe(&ctx->i2c_client->dev,
        "%s state %d, scancode %d mapped to keycode %d\n",
        __func__, ev->state, ev->scancode, keycode);
    //printk("state %d, scancode %d mapped to keycode %d\n",
    //  ev->state, ev->scancode, keycode);

    // Scancode mapped to ignored keycode
    if (keycode == 0) {
        return;

    // Scancode converted to keycode not in map
    } else if (keycode == KEY_UNKNOWN) {
        dev_warn(&ctx->i2c_client->dev,
            "%s Could not get Keycode for Scancode: [0x%02X]\n",
            __func__, ev->scancode);
        return;
    }

    // Update last keypress time
    g_ctx->last_keypress_at = ktime_get_boottime_ns();

/*
    if (keycode == KEY_STOP) {

        // Pressing power button sends Tmux prefix (Control + code 171 in keymap)
        if (ev->state == KEY_STATE_PRESSED) {
            input_report_key(ctx->input_dev, KEY_LEFTCTRL, TRUE);
            input_report_key(ctx->input_dev, 171, TRUE);
            input_report_key(ctx->input_dev, 171, FALSE);
            input_report_key(ctx->input_dev, KEY_LEFTCTRL, FALSE);

        // Short hold power buttion opens Tmux menu (Control + code 174 in keymap)
        } else if (ev->state == KEY_STATE_HOLD) {
            input_report_key(ctx->input_dev, KEY_LEFTCTRL, TRUE);
            input_report_key(ctx->input_dev, 174, TRUE);
            input_report_key(ctx->input_dev, 174, FALSE);
            input_report_key(ctx->input_dev, KEY_LEFTCTRL, FALSE);
        }
        return;
    }
    */

    // Subsystem key handling
    /*
    if (input_fw_consumes_keycode(ctx, &keycode, keycode, ev->state)
     || input_touch_consumes_keycode(ctx, &keycode, keycode, ev->state)
     || input_modifiers_consumes_keycode(ctx, &keycode, keycode, ev->state)
     || input_meta_consumes_keycode(ctx, &keycode, keycode, ev->state)) {
        return;
    }
    */

    // Ignore hold keys at this point
    if (ev->state == KEY_STATE_HOLD) {
        return;
    }

/*
    // Apply pending sticky modifiers
    keycode = input_modifiers_apply_pending(ctx, keycode);
*/

    // Report key to input system
    input_report_key(ctx->input_dev, keycode, ev->state == KEY_STATE_PRESSED);

    // Reset sticky modifiers
    // input_modifiers_reset(ctx);
}

static void input_workqueue_handler(struct work_struct *work_struct_ptr)
{
    struct kbd_ctx *ctx;
    uint8_t fifo_idx;

    // Get keyboard context from work struct
    ctx = container_of(work_struct_ptr, struct kbd_ctx, work_struct);

    input_fw_read_fifo(ctx);
    // Process FIFO items
    for (fifo_idx = 0; fifo_idx < ctx->key_fifo_count; fifo_idx++) {
        key_report_event(ctx, &ctx->key_fifo_data[fifo_idx]);
    }

    if (ctx->mouse_mode)
        {
            uint64_t press_time = ktime_get_boottime_ns() - ctx->last_keypress_at;
            if (press_time <= mouse_fast_move_thr_time)
            {
                mouse_move_step = 1;
            }
            else if (press_time <= 3 * mouse_fast_move_thr_time)
            {
                mouse_move_step = 2;
            }
            else
            {
                mouse_move_step = 4;
            }

            if (ctx->mouse_move_dir & MOUSE_MOVE_LEFT)
            {
                input_report_rel(ctx->input_dev, REL_X, -mouse_move_step);
            }
            if (ctx->mouse_move_dir & MOUSE_MOVE_RIGHT)
            {
                input_report_rel(ctx->input_dev, REL_X, mouse_move_step);
            }
            if (ctx->mouse_move_dir & MOUSE_MOVE_DOWN)
            {
                input_report_rel(ctx->input_dev, REL_Y, mouse_move_step);
            }
            if (ctx->mouse_move_dir & MOUSE_MOVE_UP)
            {
                input_report_rel(ctx->input_dev, REL_Y, -mouse_move_step);
            }
        }

    // Reset pending FIFO count
    ctx->key_fifo_count = 0;

    // Synchronize input system and clear client interrupt flag
    input_sync(ctx->input_dev);
    /*
    if (kbd_write_i2c_u8(ctx, REG_INT, 0)) {
        return;
    }
    */
}
static void kbd_timer_function(struct timer_list *data);
DEFINE_TIMER(g_kbd_timer,kbd_timer_function);

static void kbd_timer_function(struct timer_list *data)
{
    data = NULL;
    schedule_work(&g_ctx->work_struct);
    mod_timer(&g_kbd_timer, jiffies + HZ / 128);
}

int input_probe(struct i2c_client* i2c_client, struct regmap* regmap)
{
    int rc, i;

    // Allocate keyboard context (managed by device lifetime)
    g_ctx = devm_kzalloc(&i2c_client->dev, sizeof(*g_ctx), GFP_KERNEL);
    if (!g_ctx) {
        return -ENOMEM;
    }

    // Allocate and copy keycode array
    g_ctx->keycode_map = devm_kmemdup(&i2c_client->dev, keycodes, NUM_KEYCODES,
        GFP_KERNEL);
    if (!g_ctx->keycode_map) {
        return -ENOMEM;
    }

    // Initialize keyboard context
    g_ctx->i2c_client = i2c_client;
    g_ctx->regmap = regmap;
    g_ctx->last_keypress_at = ktime_get_boottime_ns();

    // Allocate input device
    if ((g_ctx->input_dev = devm_input_allocate_device(&i2c_client->dev)) == NULL) {
        dev_err(&i2c_client->dev,
            "%s Could not devm_input_allocate_device BBQX0KBD.\n", __func__);
        return -ENOMEM;
    }

    // Initialize input device
    g_ctx->input_dev->name = i2c_client->name;
    g_ctx->input_dev->id.bustype = KBD_BUS_TYPE;
    g_ctx->input_dev->id.vendor  = KBD_VENDOR_ID;
    g_ctx->input_dev->id.product = KBD_PRODUCT_ID;
    g_ctx->input_dev->id.version = KBD_VERSION_ID;

    // Initialize input device keycodes
    g_ctx->input_dev->keycode = keycodes; //g_ctx->keycode_map;
    g_ctx->input_dev->keycodesize = sizeof(keycodes[0]);
    g_ctx->input_dev->keycodemax = ARRAY_SIZE(keycodes);

    // Set input device keycode bits
    for (i = 0; i < NUM_KEYCODES; i++) {
        __set_bit(keycodes[i], g_ctx->input_dev->keybit);
    }
    __clear_bit(KEY_RESERVED, g_ctx->input_dev->keybit);
    __set_bit(EV_REP, g_ctx->input_dev->evbit);
    __set_bit(EV_KEY, g_ctx->input_dev->evbit);

    // Set input device capabilities
    input_set_capability(g_ctx->input_dev, EV_MSC, MSC_SCAN);
    input_set_capability(g_ctx->input_dev, EV_REL, REL_X);
    input_set_capability(g_ctx->input_dev, EV_REL, REL_Y);
/*
    input_set_capability(g_ctx->input_dev, EV_ABS, ABS_X);
    input_set_capability(g_ctx->input_dev, EV_ABS, ABS_Y);
    input_set_abs_params(g_ctx->input_dev, ABS_X, 0, 320, 4, 8);
    input_set_abs_params(g_ctx->input_dev, ABS_Y, 0, 320, 4, 8);
*/
    input_set_capability(g_ctx->input_dev, EV_KEY, BTN_LEFT);
    input_set_capability(g_ctx->input_dev, EV_KEY, BTN_RIGHT);

    // Request IRQ handler for I2C client and initialize workqueue
    /*
    if ((rc = devm_request_threaded_irq(&i2c_client->dev,
        i2c_client->irq, NULL, input_irq_handler, IRQF_SHARED | IRQF_ONESHOT,
        i2c_client->name, g_ctx))) {

        dev_err(&i2c_client->dev,
            "Could not claim IRQ %d; error %d\n", i2c_client->irq, rc);
        return rc;
    }
    */
    g_ctx->mouse_mode = FALSE;
    g_ctx->mouse_move_dir = 0;
    INIT_WORK(&g_ctx->work_struct, input_workqueue_handler);
    g_kbd_timer.expires = jiffies + HZ / 128;
    add_timer(&g_kbd_timer);

    // Register input device with input subsystem
    dev_info(&i2c_client->dev,
        "%s registering input device", __func__);
    if ((rc = input_register_device(g_ctx->input_dev))) {
        dev_err(&i2c_client->dev,
            "Failed to register input device, error: %d\n", rc);
        return rc;
    }

    return 0;
}

void input_shutdown(struct i2c_client* i2c_client)
{
    // Remove context from global state
    // (It is freed by the device-specific memory mananger)
    del_timer(&g_kbd_timer);
    g_ctx = NULL;
}

static int picocalc_mfd_kbd_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct i2c_client *i2c = to_i2c_client(dev->parent);
    struct regmap *regmap = dev_get_regmap(dev->parent, NULL);
    int rc;

    if (!regmap) {
        dev_err(dev, "Failed to get parent regmap\n");
        return -EINVAL;
    }

    // Initialize key handler system
    if ((rc = input_probe(i2c, regmap))) {
        return rc;
    }

    platform_set_drvdata(pdev, g_ctx);

    dev_info(dev, "Keyboard input driver registered successfully\n");
    return 0;
}

static int picocalc_mfd_kbd_remove(struct platform_device *pdev)
{
    struct kbd_ctx *ctx = platform_get_drvdata(pdev);

    dev_info_fe(&pdev->dev, "%s Removing picocalc-kbd.\n", __func__);

    input_shutdown(ctx->i2c_client);

    return 0;
}

static const struct of_device_id picocalc_mfd_kbd_of_match[] = {
    { .compatible = "picocalc-mfd-kbd", },
    {}
};
MODULE_DEVICE_TABLE(of, picocalc_mfd_kbd_of_match);

static struct platform_driver picocalc_mfd_kbd_driver = {
    .driver = {
        .name = "picocalc_mfd_kbd",
        .of_match_table = picocalc_mfd_kbd_of_match,
    },
    .probe    = picocalc_mfd_kbd_probe,
    .remove   = picocalc_mfd_kbd_remove,
};

module_platform_driver(picocalc_mfd_kbd_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("hiro <hiro@hiro.com>");
MODULE_DESCRIPTION("keyboard driver for picocalc");
MODULE_VERSION("0.01");