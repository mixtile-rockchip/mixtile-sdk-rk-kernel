// SPDX-License-Identifier: GPL-2.0-only
/*
 * ILI2116 / ILI2117 I2C touchscreen driver
 *
 *
 * Register 0x10: number of active touch points (1 byte)
 * Register 0x11: per-point data (5 bytes each, sequential reads advance)
 *   byte 0: [7] touch-down, [6:0] slot id (1-based)
 *   bytes 1-2: X coordinate (big-endian)
 *   bytes 3-4: Y coordinate (big-endian)
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <asm/unaligned.h>

#define ILI2116_MAX_TOUCHES	10

#define REG_TOUCHDATA		0x10
#define REG_TOUCH_DATA		0x11
#define REG_PANEL_INFO		0x20
#define REG_FIRMWARE_VERSION	0x40
#define REG_PROTOCOL_VERSION	0x42
#define REG_MCU_VERSION		0x61
#define REG_GET_MODE		0xc0
#define REG_GET_MODE_AP		0x5a
#define REG_DEBUG_MODE_PANEL	0xdb
#define REG_DEBUG_MODE_TOUCH	0xdc

struct ili2116_priv {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
	struct gpio_desc *reset_gpio;
	/* leftover from mutex_init in original — keep struct layout stable */
	unsigned long reserved[4];
	unsigned long chip_data;
	u16 max_x;	/* from panel_info[0:2] */
	u16 max_y;	/* from panel_info[2:4] */
};

static int ili2116_read_reg(struct i2c_client *client, u8 reg,
			    void *buf, size_t len)
{
	struct i2c_msg msg[2] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &reg,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		},
	};

	if (i2c_transfer(client->adapter, msg, 2) != 2) {
		dev_err(&client->dev, "i2c transfer failed\n");
		return -EIO;
	}
	return 0;
}

static void ili2116_hw_reset(struct gpio_desc *reset_gpio)
{
	if (!reset_gpio)
		return;

	gpiod_set_value_cansleep(reset_gpio, 0);
	usleep_range(50, 200);
	gpiod_set_value_cansleep(reset_gpio, 1);
	msleep(100);
}

/*
 * ili2116_reset_and_read_panel_data
 *
 * Reset sequence → read protocol version → firmware version → mode
 * → MCU version → panel info (resolution).
 * Stores max_x / max_y into priv.
 */
static int ili2116_reset_and_read_panel_data(struct ili2116_priv *priv)
{
	struct i2c_client *client = priv->client;
	struct device *dev = &client->dev;
	u8 proto[2], fw[4], mcu[5], panel[10], mode;
	int error;

	ili2116_hw_reset(priv->reset_gpio);

	error = ili2116_read_reg(client, REG_PROTOCOL_VERSION, proto, 2);
	if (error) {
		dev_err(dev, "Failed to get protocol version: %d\n", error);
		return error;
	}
	dev_info(dev, "protocol version %02x.%02x\n", proto[0], proto[1]);

	error = ili2116_read_reg(client, REG_FIRMWARE_VERSION, fw, 4);
	if (error) {
		dev_err(dev, "Failed to get firmware version, err: %d\n", error);
		return error;
	}
	dev_info(dev, "ILI2116 firmware version %d.%d.%d.%d\n",
		 fw[0], fw[1], fw[2], fw[3]);

	error = ili2116_read_reg(client, REG_GET_MODE, &mode, 1);
	if (error) {
		dev_err(dev, "Failed to get mode, err: %d\n", error);
		return error;
	}
	if (mode != REG_GET_MODE_AP)
		dev_warn(dev, "Touchscreen mode looks wrong: got 0x%02x\n", mode);

	error = ili2116_read_reg(client, REG_MCU_VERSION, mcu, 5);
	if (error) {
		dev_err(dev, "Failed to get MCU version, err: %d\n", error);
		return error;
	}
	dev_info(dev, "ILI2116 MCU kernel version %d.%d.%d.%d.%d\n",
		 mcu[0], mcu[1], mcu[2], mcu[3], mcu[4]);

	error = ili2116_read_reg(client, REG_PANEL_INFO, panel, 10);
	if (error) {
		dev_err(dev, "Failed to get panel information, err: %d\n", error);
		return error;
	}

	/* original stores these into priv->max_x / priv->max_y exactly like this */
	priv->max_x = get_unaligned_le16(&panel[0]);
	priv->max_y = get_unaligned_le16(&panel[2]);
	dev_info(dev, "panel size %dx%d\n", priv->max_x, priv->max_y);

	return 0;
}

/*
 * ili2116_work — workqueue callback, executed after ili2116_irq
 * queues it.  Reads touch data via I2C and reports through input.
 */
static void ili2116_work(struct work_struct *work)
{
	struct ili2116_priv *priv = container_of(work, struct ili2116_priv, work);
	struct i2c_client *client = priv->client;
	struct input_dev *input = priv->input;
	u8 touch_count;
	u8 buf[50];	/* max 10 points × 5 bytes */
	u8 debug_buf[32];
	int i, error;
	u8 *p;
	int slot;
	u16 x, y;

	error = ili2116_read_reg(client, REG_TOUCHDATA, &touch_count, 1);
	if (error) {
		dev_err(&client->dev,
			"Unable to get touch number, err = %d\n", error);
		return;
	}

	/*
	 * Original vendor driver treats 0xDB/0xDC as special debug
	 * payloads instead of invalid touch counts.
	 */
	if (touch_count == REG_DEBUG_MODE_PANEL ||
	    touch_count == REG_DEBUG_MODE_TOUCH) {
		error = ili2116_read_reg(client, REG_TOUCH_DATA,
					 debug_buf, sizeof(debug_buf));
		if (error) {
			dev_err(&client->dev,
				"Unable to get touch info, err = %d\n", error);
			return;
		}

		if (debug_buf[0] <= sizeof(debug_buf) - 1)
			print_hex_dump_debug(
				touch_count == REG_DEBUG_MODE_PANEL ?
					"ILI2116 panel: " : "ILI2116 touch: ",
				DUMP_PREFIX_OFFSET, 16, 1,
				&debug_buf[1], debug_buf[0], true);
		return;
	}

	if (touch_count > ILI2116_MAX_TOUCHES) {
		dev_err(&client->dev, "Invalid touch number: %d\n",
			touch_count);
		return;
	}

	for (i = 0; i < touch_count; i++) {
		error = ili2116_read_reg(client, REG_TOUCH_DATA,
					 buf + i * 5, 5);
		if (error) {
			dev_err(&client->dev,
				"Unable to get touch info, err = %d\n", error);
			return;
		}
	}

	for (i = 0; i < touch_count; i++) {
		p = buf + i * 5;

		/*
		 * Slot id is 1-based in hardware — subtract 1.
		 * Original: input_event(input, EV_ABS, ABS_MT_SLOT,
		 *                      (*p & 0x7F) - 1)
		 */
		slot = (*p & 0x7f) - 1;
		if (slot < 0 || slot >= ILI2116_MAX_TOUCHES)
			continue;

		input_event(input, EV_ABS, ABS_MT_SLOT, slot);

		/*
		 * Original always calls input_mt_report_slot_state,
		 * then checks *p >> 7 (bit 7 = touch-down) to decide
		 * whether to report coordinates.
		 */
		input_mt_report_slot_state(input, MT_TOOL_FINGER,
					   *p >> 7);

		if (!(*p >> 7))
			continue;

		x = get_unaligned_be16(p + 1);
		y = get_unaligned_be16(p + 3);

		/*
		 * Original checks chip_data & 1 for rotation:
		 *   if (chip_data & 1) {
		 *       tmp = max_x - 1 - y;
		 *       y = x;
		 *       x = tmp;
		 *   }
		 * For ili2116, chip_data is NULL → no rotation.
		 */
		if (priv->chip_data & 1) {
			u16 tmp = x;
			x = priv->max_y - 1 - y;
			y = tmp;
		}

		/*
		 * Original: input_event(input, EV_ABS,
		 *              ABS_MT_POSITION_X, x), same for Y
		 */
		input_event(input, EV_ABS, ABS_MT_POSITION_X, x);
		input_event(input, EV_ABS, ABS_MT_POSITION_Y, y);
	}

	input_mt_report_pointer_emulation(input, false);
	input_event(input, EV_SYN, SYN_REPORT, 0);
}

/*
 * ili2116_irq — hardirq handler.
 * Original: queue_work_on(4, system_highpri_wq, &priv->work);
 */
static irqreturn_t ili2116_irq(int irq, void *irq_data)
{
	struct ili2116_priv *priv = irq_data;

	queue_work(system_highpri_wq, &priv->work);

	return IRQ_HANDLED;
}

/*
 * ili2116_register_input_device
 *
 * Mirrors the original faithfully:
 *   - input_allocate_device
 *   - name / bustype / parent
 *   - evbit = EV_KEY | EV_ABS
 *   - keybit = BTN_TOUCH
 *   - ABS_X / ABS_Y from max_x / max_y
 *   - 10 MT slots (flags = 0)
 *   - ABS_MT_POSITION_X / Y
 *   - input_register_device
 */
static int ili2116_register_input_device(struct ili2116_priv *priv)
{
	struct device *dev = &priv->client->dev;
	struct input_dev *input;
	unsigned int max_x, max_y;
	int error;

	input = input_allocate_device();
	if (!input)
		return -ENOMEM;

	max_x = priv->max_x;
	max_y = priv->max_y;
	if (priv->chip_data & 1) {
		unsigned int tmp = max_x;

		max_x = max_y;
		max_y = tmp;
	}

	input->name = "ILI2116 Touchscreen";
	input->id.bustype = BUS_I2C;
	input->dev.parent = dev;

	input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_set_capability(input, EV_KEY, BTN_TOUCH);

	input_set_abs_params(input, ABS_X, 0, max_x, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, max_y, 0, 0);
	input_mt_init_slots(input, ILI2116_MAX_TOUCHES, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, max_y, 0, 0);

	input_set_drvdata(input, priv);
	priv->input = input;

	error = input_register_device(input);
	if (error)
		input_free_device(input);

	return error;
}

static int ili2116_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ili2116_priv *priv;
	int error;

	dev_dbg(dev, "Probing for ILI210X I2C Touschreen driver");

	if (!dev->of_node) {
		dev_err(dev, "No of_node!\n");
		return -EINVAL;
	}

	dev_info(dev, "i2c_device_id = %px\n", id);

	if (client->irq <= 0) {
		dev_err(dev, "No IRQ!\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;

	/* Original: mutex_init, rt_mutex_init — not preserved in port */

	priv->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio))
		return PTR_ERR(priv->reset_gpio);

	/* Original stores driver_data from i2c_device_id into priv[13] */
	if (id)
		priv->chip_data = (unsigned long)id->driver_data;

	i2c_set_clientdata(client, priv);
	INIT_WORK(&priv->work, ili2116_work);

	error = ili2116_reset_and_read_panel_data(priv);
	if (error)
		return error;

	error = ili2116_register_input_device(priv);
	if (error)
		return error;

	error = request_threaded_irq(client->irq,
				     ili2116_irq,	/* handler */
				     NULL,			/* thread_fn */
				     IRQF_TRIGGER_FALLING,
				     "ili2116", priv);
	if (error) {
		dev_err(dev, "Unable to request touchscreen IRQ, err: %d\n",
			error);
		input_unregister_device(priv->input);
		return error;
	}

	/*
	 * Original also creates a sysfs group for calibrate / firmware_update /
	 * firmware_version / kernel_version / protocol_version / mode.
	 * Skipped in this port — the attributes depend on chip variant.
	 */

	device_init_wakeup(dev, 1);

	return 0;
}

static void ili2116_i2c_remove(struct i2c_client *client)
{
	struct ili2116_priv *priv __maybe_unused = i2c_get_clientdata(client);

	free_irq(client->irq, priv);
	cancel_work_sync(&priv->work);
	input_unregister_device(priv->input);
}

static int ili2116_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(dev) && client->irq > 0)
		irq_set_irq_wake(client->irq, 1);

	return 0;
}

static int ili2116_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(dev) && client->irq > 0)
		irq_set_irq_wake(client->irq, 0);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ili2116_pm_ops,
				ili2116_i2c_suspend,
				ili2116_i2c_resume);

static const struct i2c_device_id ili2116_i2c_id[] = {
	{ "ili2116", 0 },
	{ "ili2117", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ili2116_i2c_id);

static const struct of_device_id ili2116_of_match[] = {
	{ .compatible = "ilitek,ili2116", .data = (void *)0 },
	{ .compatible = "ilitek,ili2117", .data = (void *)0 },
	{ }
};
MODULE_DEVICE_TABLE(of, ili2116_of_match);

static struct i2c_driver ili2116_driver = {
	.driver = {
		.name		= "ili2116",
		.of_match_table	= ili2116_of_match,
		.pm		= &ili2116_pm_ops,
	},
	.id_table	= ili2116_i2c_id,
	.probe		= ili2116_i2c_probe,
	.remove		= ili2116_i2c_remove,
};
module_i2c_driver(ili2116_driver);

MODULE_AUTHOR("John Keeping <john@metanate.com>");
MODULE_DESCRIPTION("ILI2116 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
