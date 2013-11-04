/*
 * TD4291 panel support
 *
 * based on d2l panel driver by Jerry Alexander <x0135174@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/i2c.h>

#include <video/omapdss.h>
#include <video/mipi_display.h>
#include <video/omap-panel-td4291.h>

#define TD4291_VERSION "1.14"

#define USE_GENERIC_ACCESS true

#define BUFFER_LENGTH 10

#define DELAY_TIME_MS 20
#define SLEEP_OUT_DELAY_TIME_MS 300
#define DISPLAY_ON_DELAY_TIME_MS 300
#define SLEEP_IN_DELAY_TIME_MS 300
#define DISPLAY_OFF_DELAY_TIME_MS 300

struct td4291_data {
	struct omap_dss_device *dssdev;
	struct backlight_device *bldev;
	struct dentry *debug_dir;
	struct mutex lock;
	struct panel_td4291_data *pdata;
	int config_channel;
	int pixel_channel;
	unsigned char rotate;
	unsigned int delay_time;
	unsigned int bl;
	unsigned int offset;
	unsigned char length;
	unsigned char index;
	unsigned char output_length;
	unsigned char buffer[BUFFER_LENGTH];
	unsigned char output[BUFFER_LENGTH * 2];
	bool enabled;
	bool ready_for_access;
	bool read_result;
	bool use_generic_access;
};

struct panel_config {
	struct omap_video_timings *timings;
	struct {
		unsigned int sleep_in;
		unsigned int sleep_out;
		unsigned int display_on;
		unsigned int display_off;
		unsigned int hw_reset;
	} sleep;
	struct {
		unsigned int high;
		unsigned int low;
	} reset_sequence;
};

struct td4291_reg {
	unsigned char data[10];
	int len;
};

struct td_4291_register_setting {
	unsigned char address;
	unsigned char value;
};

static struct omap_video_timings td4291_timings = {
	.x_res = 720,
	.y_res = 1280,
	.pixel_clock = 72384,
	.hfp = 96,
	.hsw = 38,
	.hbp = 74,
	.vfp = 12,
	.vsw = 4,
	.vbp = 4,
};

static struct panel_config td4291_panel_config = {
	.timings = &td4291_timings,
	.sleep = {
		.sleep_out = SLEEP_OUT_DELAY_TIME_MS,
		.display_on = DISPLAY_ON_DELAY_TIME_MS,
		.sleep_in = SLEEP_IN_DELAY_TIME_MS,
		.display_off = DISPLAY_OFF_DELAY_TIME_MS,
		.hw_reset = 500,
	},
	.reset_sequence = {
		.high = 10,
		.low = 30,
	},
};

static struct td4291_reg sleep_out[] = {
	{{ MIPI_DCS_EXIT_SLEEP_MODE, }, 1},
};

static struct td4291_reg display_on[] = {
	{{ MIPI_DCS_SET_DISPLAY_ON, }, 1},
};

static struct td4291_reg sleep_in[] = {
	{{ MIPI_DCS_ENTER_SLEEP_MODE, }, 1},
};

static struct td4291_reg display_off[] = {
	{{ MIPI_DCS_SET_DISPLAY_OFF, }, 1},
};

static struct td_4291_register_setting auo_reg_settings[] = {
	{
		.address = 0xb0, /* DSI_CFG_7_0 */
		.value = 0x00,
	},
	{
		.address = 0xb3, /* DSI_CFG_31_24 */
		.value = 0xf0,
	},
	{
		.address = 0x45, /* TCH_SL_LSB */
		.value = 0x11,
	},
	{
		.address = 0x55, /* BLANK_REG */
		.value = 0x00,
	},
};

static struct td_4291_register_setting yxt_reg_settings[] = {
	{
		.address = 0xb3, /* DSI_CFG_31_24 */
		.value = 0x70,
	},
	{
		.address = 0x45, /* TCH_SL_LSB */
		.value = 0x13,
	},
};

void synaptics_rmi4_touch_sleep(void);
void synaptics_rmi4_touch_wake(void);

void dsi_videomode_panel_preinit(struct omap_dss_device *dssdev);

static int td4291_dcs_write(struct omap_dss_device *dssdev, unsigned char *buf,
		int len, bool sync, bool generic);

static int td4291_dcs_write_sequence(struct omap_dss_device *dssdev,
		struct td4291_reg *seq, int len);

static void td4291_config(struct omap_dss_device *dssdev,
		struct td_4291_register_setting *reg_settings,
		unsigned char num_of_regs);

static int td4291_hw_reset(struct omap_dss_device *dssdev);

static ssize_t td4291_store_enable_display(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int retval;
	unsigned long input;
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	retval = strict_strtoul(buf, 10, &input);
	if (retval)
		return retval;

	if ((input != 0) && (input != 1))
		return -EINVAL;

	mutex_lock(&ad->lock);
	dsi_bus_lock(dssdev);

	if (input == 1)
		dsi_video_mode_enable(dssdev, MIPI_DSI_PACKED_PIXEL_STREAM_24);
	else
		dsi_video_mode_disable(dssdev);

	dsi_bus_unlock(dssdev);
	mutex_unlock(&ad->lock);

	return count;
}

static ssize_t td4291_store_reset(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int retval;
	unsigned long input;
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	retval = strict_strtoul(buf, 10, &input);
	if (retval)
		return retval;

	if (input != 1)
		return -EINVAL;

	mutex_lock(&ad->lock);
	dsi_bus_lock(dssdev);

	if (ad->enabled)
		dsi_video_mode_disable(dssdev);

	td4291_hw_reset(dssdev);

	td4291_dcs_write_sequence(dssdev, sleep_out,
			ARRAY_SIZE(sleep_out));
	msleep(td4291_panel_config.sleep.sleep_out);

	td4291_dcs_write_sequence(dssdev, display_on,
			ARRAY_SIZE(display_on));
	msleep(td4291_panel_config.sleep.display_on);

	if (ad->enabled)
		dsi_video_mode_enable(dssdev, MIPI_DSI_PACKED_PIXEL_STREAM_24);

	dsi_bus_unlock(dssdev);
	mutex_unlock(&ad->lock);

	return count;
}

static ssize_t td4291_store_length(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int retval;
	unsigned long input;
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	retval = strict_strtoul(buf, 10, &input);
	if (retval)
		return retval;

	if (input >= BUFFER_LENGTH)
		input = BUFFER_LENGTH - 1;

	ad->length = input;
	ad->index = 0;

	return count;
}

static ssize_t td4291_store_buffer(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int retval;
	unsigned long input;
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	retval = strict_strtoul(buf, 16, &input);
	if (retval)
		return retval;

	if (ad->index == (BUFFER_LENGTH - 1)) {
		dev_err(&dssdev->dev, "Write buffer already full\n");
		return -EINVAL;
	}

	ad->buffer[ad->index + 1] = (unsigned char)input;

	ad->length++;
	ad->index++;

	return count;
}

static ssize_t td4291_store_write(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int retval;
	unsigned long input;
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	retval = strict_strtoul(buf, 16, &input);
	if (retval)
		return retval;

	if (!ad->ready_for_access) {
		dev_err(&dssdev->dev, "Not ready to do read/write yet\n");
		return -ENODEV;
	}

	mutex_lock(&ad->lock);
	dsi_bus_lock(dssdev);

	if (ad->enabled) {
		dsi_video_mode_disable(dssdev);
		msleep(ad->delay_time);
	}

	ad->buffer[0] = (unsigned char)input;
	ad->length++;

	retval = td4291_dcs_write(dssdev, ad->buffer, ad->length, false, false);

	if (ad->enabled) {
		msleep(ad->delay_time);
		dsi_video_mode_enable(dssdev, MIPI_DSI_PACKED_PIXEL_STREAM_24);
	}

	dsi_bus_unlock(dssdev);
	mutex_unlock(&ad->lock);

	if (retval < 0) {
		dev_err(&dssdev->dev, "Failed to write (command 0x%02x)\n",
				(unsigned char)input);
	} else {
		retval = count;
	}

	ad->length = 0;
	ad->index = 0;

	return retval;
}

static ssize_t td4291_show_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	if (!ad->read_result)
		return -EINVAL;

	memcpy(buf, ad->output, ad->output_length * 2);
	buf[ad->output_length * 2] = '\n';
	buf[ad->output_length * 2 + 1] = '\0';

	return (ad->output_length * 2 + 1);
}

static ssize_t td4291_store_read(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int retval;
	unsigned char ii;
	unsigned long input;
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	retval = strict_strtoul(buf, 16, &input);
	if (retval)
		return retval;

	if (!ad->ready_for_access) {
		dev_err(&dssdev->dev, "Not ready to do read/write yet\n");
		return -ENODEV;
	}

	if (ad->length == 0)
		ad->length = 1;

	mutex_lock(&ad->lock);
	dsi_bus_lock(dssdev);

	if (ad->enabled) {
		dsi_video_mode_disable(dssdev);
		msleep(ad->delay_time);
	}

	retval = dsi_vc_dcs_read(ad->dssdev, ad->config_channel, input,
			ad->buffer, ad->length);

	if (ad->enabled) {
		msleep(ad->delay_time);
		dsi_video_mode_enable(dssdev, MIPI_DSI_PACKED_PIXEL_STREAM_24);
	}

	dsi_bus_unlock(dssdev);
	mutex_unlock(&ad->lock);

	if (retval < 0) {
		ad->read_result = false;
		dev_err(&dssdev->dev, "Failed to read (command 0x%02x)\n",
				(unsigned char)input);
	} else {
		ad->read_result = true;
		retval = count;

		memset(ad->output, 0x00, sizeof(ad->output));
		ad->output_length = ad->length;

		printk("data read with command 0x%02x:\n", (unsigned char)input);
		for (ii = 0; ii < ad->length; ii++) {
			printk("0x%02x\n", ad->buffer[ii]);
			sprintf(&ad->output[ii * 2], "%02x", ad->buffer[ii]);
		}
	}

	ad->length = 0;
	ad->index = 0;

	return retval;
}

static ssize_t td4291_store_write_reg(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int retval;
	unsigned long input;
	unsigned char cmd;
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	retval = strict_strtoul(buf, 16, &input);
	if (retval)
		return retval;

	if (!ad->ready_for_access) {
		dev_err(&dssdev->dev, "Not ready to do read/write yet\n");
		return -ENODEV;
	}

	mutex_lock(&ad->lock);
	dsi_bus_lock(dssdev);

	if (ad->enabled) {
		dsi_video_mode_disable(dssdev);
		msleep(ad->delay_time);
	}

	cmd = 0xde;
	retval = dsi_vc_dcs_write_nosync(dssdev, ad->config_channel, &cmd, 1);
	if (retval < 0) {
		dev_err(&dssdev->dev, "Failed to enter register access mode\n");
		goto exit;
	}

	msleep(ad->delay_time);

	ad->buffer[0] = (unsigned char)input;
	ad->length++;

	if (ad->use_generic_access)
		retval = td4291_dcs_write(dssdev, ad->buffer, ad->length, false,
				true);
	else
		retval = td4291_dcs_write(dssdev, ad->buffer, ad->length, false,
				false);
	if (retval < 0) {
		dev_err(&dssdev->dev, "Failed to write (register 0x%02x)\n",
				(unsigned char)input);
		goto exit;
	}

	msleep(ad->delay_time);

	cmd = 0xdf;
	retval = dsi_vc_dcs_write_nosync(dssdev, ad->config_channel, &cmd, 1);
	if (retval < 0) {
		dev_err(&dssdev->dev, "Failed to exit register access mode\n");
		goto exit;
	}

	retval = count;

exit:
	if (ad->enabled) {
		msleep(ad->delay_time);
		dsi_video_mode_enable(dssdev, MIPI_DSI_PACKED_PIXEL_STREAM_24);
	}

	dsi_bus_unlock(dssdev);
	mutex_unlock(&ad->lock);

	ad->length = 0;
	ad->index = 0;

	return retval;
}

static ssize_t td4291_show_read_reg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	if (!ad->read_result)
		return -EINVAL;

	memcpy(buf, ad->output, ad->output_length * 2);
	buf[ad->output_length * 2] = '\n';
	buf[ad->output_length * 2 + 1] = '\0';

	return (ad->output_length * 2 + 1);
}

static ssize_t td4291_store_read_reg(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int retval;
	unsigned char ii;
	unsigned long input;
	unsigned char cmd;
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	retval = strict_strtoul(buf, 16, &input);
	if (retval)
		return retval;

	if (!ad->ready_for_access) {
		dev_err(&dssdev->dev, "Not ready to do read/write yet\n");
		return -ENODEV;
	}

	if (ad->length == 0)
		ad->length = 1;

	mutex_lock(&ad->lock);
	dsi_bus_lock(dssdev);

	if (ad->enabled) {
		dsi_video_mode_disable(dssdev);
		msleep(ad->delay_time);
	}

	cmd = 0xde;
	retval = dsi_vc_dcs_write_nosync(dssdev, ad->config_channel, &cmd, 1);
	if (retval < 0) {
		ad->read_result = false;
		dev_err(&dssdev->dev, "Failed to enter register access mode\n");
		goto exit;
	} else {
		ad->read_result = true;
	}

	msleep(ad->delay_time);

	if (ad->use_generic_access)
		retval = dsi_vc_dcs_generic_read(ad->dssdev, ad->config_channel,
				input, ad->buffer, ad->length);
	else
		retval = dsi_vc_dcs_read(ad->dssdev, ad->config_channel,
				input, ad->buffer, ad->length);

	if (retval < 0) {
		dev_err(&dssdev->dev, "Failed to read (register 0x%02x)\n",
				(unsigned char)input);
		goto exit;
	}

	msleep(ad->delay_time);

	cmd = 0xdf;
	retval = dsi_vc_dcs_write_nosync(dssdev, ad->config_channel, &cmd, 1);
	if (retval < 0) {
		dev_err(&dssdev->dev, "Failed to exit register access mode\n");
		goto exit;
	}

	retval = count;

	memset(ad->output, 0x00, sizeof(ad->output));
	ad->output_length = ad->length;

	printk("data read from register 0x%02x:\n", (unsigned char)input);
	for (ii = 0; ii < ad->length; ii++) {
		printk("0x%02x\n", ad->buffer[ii]);
		sprintf(&ad->output[ii * 2], "%02x", ad->buffer[ii]);
	}

exit:
	if (ad->enabled) {
		msleep(ad->delay_time);
		dsi_video_mode_enable(dssdev, MIPI_DSI_PACKED_PIXEL_STREAM_24);
	}

	dsi_bus_unlock(dssdev);
	mutex_unlock(&ad->lock);

	ad->length = 0;
	ad->index = 0;

	return retval;
}

static ssize_t td4291_store_config(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	char input[8] = {0};
	unsigned char num_of_regs;
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	if (count > 8)
		return -EINVAL;

	memcpy(input, buf, count - 1);

	mutex_lock(&ad->lock);
	dsi_bus_lock(dssdev);

	if (ad->enabled)
		dsi_video_mode_disable(dssdev);

	if (strcmp(input, "auo") == 0) {
		num_of_regs = ARRAY_SIZE(auo_reg_settings);
		td4291_config(dssdev, auo_reg_settings, num_of_regs);
	} else if (strcmp(input, "yxt") == 0) {
		num_of_regs = ARRAY_SIZE(yxt_reg_settings);
		td4291_config(dssdev, yxt_reg_settings, num_of_regs);
	}

	if (ad->enabled)
		dsi_video_mode_enable(dssdev, MIPI_DSI_PACKED_PIXEL_STREAM_24);

	dsi_bus_unlock(dssdev);
	mutex_unlock(&ad->lock);

	return count;
}

static ssize_t td4291_store_touch(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int retval;
	unsigned long input;

	retval = strict_strtoul(buf, 10, &input);
	if (retval)
		return retval;

	if (input == 1)
		synaptics_rmi4_touch_wake();
	else if (input == 0)
		synaptics_rmi4_touch_sleep();
	else
		return -EINVAL;

	return count;
}

static ssize_t td4291_store_uwait(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int retval;
	unsigned long input;

	retval = strict_strtoul(buf, 10, &input);
	if (retval)
		return retval;

	usleep_range(input, input);

	return count;
}

static ssize_t td4291_show_delay_time_ms(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", ad->delay_time);
}

static ssize_t td4291_store_delay_time_ms(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int retval;
	unsigned long input;
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	retval = strict_strtoul(buf, 10, &input);
	if (retval)
		return retval;

	ad->delay_time = input;

	return count;
}

static ssize_t td4291_show_sleep_out_ms(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",
			td4291_panel_config.sleep.sleep_out);
}

static ssize_t td4291_store_sleep_out_ms(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int retval;
	unsigned long input;

	retval = strict_strtoul(buf, 10, &input);
	if (retval)
		return retval;

	td4291_panel_config.sleep.sleep_out = input;

	return count;
}

static ssize_t td4291_show_display_on_ms(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",
			td4291_panel_config.sleep.display_on);
}

static ssize_t td4291_store_display_on_ms(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int retval;
	unsigned long input;

	retval = strict_strtoul(buf, 10, &input);
	if (retval)
		return retval;

	td4291_panel_config.sleep.display_on = input;

	return count;
}

static ssize_t td4291_show_use_generic(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", ad->use_generic_access);
}

static ssize_t td4291_store_use_generic(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int retval;
	unsigned long input;
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	retval = strict_strtoul(buf, 10, &input);
	if (retval)
		return retval;

	if (input == 1)
		ad->use_generic_access = true;
	else if (input == 0)
		ad->use_generic_access = false;
	else
		return -EINVAL;

	return count;
}

static ssize_t td4291_show_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, TD4291_VERSION"\n");
}

static DEVICE_ATTR(enable_display, S_IWUGO,
		NULL, td4291_store_enable_display);
static DEVICE_ATTR(reset, S_IWUGO,
		NULL, td4291_store_reset);
static DEVICE_ATTR(length, S_IWUGO,
		NULL, td4291_store_length);
static DEVICE_ATTR(buffer, S_IWUGO,
		NULL, td4291_store_buffer);
static DEVICE_ATTR(write, S_IWUGO,
		NULL, td4291_store_write);
static DEVICE_ATTR(read, S_IRUGO | S_IWUGO,
		td4291_show_read, td4291_store_read);
static DEVICE_ATTR(write_reg, S_IWUGO,
		NULL, td4291_store_write_reg);
static DEVICE_ATTR(read_reg, S_IRUGO | S_IWUGO,
		td4291_show_read_reg, td4291_store_read_reg);
static DEVICE_ATTR(config, S_IWUGO,
		NULL, td4291_store_config);
static DEVICE_ATTR(touch, S_IWUGO,
		NULL, td4291_store_touch);
static DEVICE_ATTR(uwait, S_IWUGO,
		NULL, td4291_store_uwait);
static DEVICE_ATTR(delay_time_ms, S_IRUGO | S_IWUGO,
		td4291_show_delay_time_ms, td4291_store_delay_time_ms);
static DEVICE_ATTR(sleep_out_ms, S_IRUGO | S_IWUGO,
		td4291_show_sleep_out_ms, td4291_store_sleep_out_ms);
static DEVICE_ATTR(display_on_ms, S_IRUGO | S_IWUGO,
		td4291_show_display_on_ms, td4291_store_display_on_ms);
static DEVICE_ATTR(use_generic, S_IRUGO | S_IWUGO,
		td4291_show_use_generic, td4291_store_use_generic);
static DEVICE_ATTR(version, S_IRUGO,
		td4291_show_version, NULL);

static struct attribute *td4291_attrs[] = {
	&dev_attr_enable_display.attr,
	&dev_attr_reset.attr,
	&dev_attr_length.attr,
	&dev_attr_buffer.attr,
	&dev_attr_write.attr,
	&dev_attr_read.attr,
	&dev_attr_write_reg.attr,
	&dev_attr_read_reg.attr,
	&dev_attr_config.attr,
	&dev_attr_touch.attr,
	&dev_attr_uwait.attr,
	&dev_attr_delay_time_ms.attr,
	&dev_attr_sleep_out_ms.attr,
	&dev_attr_display_on_ms.attr,
	&dev_attr_use_generic.attr,
	&dev_attr_version.attr,
	NULL,
};

static struct attribute_group td4291_attr_group = {
	.attrs = td4291_attrs,
};

static int td4291_dcs_write(struct omap_dss_device *dssdev, unsigned char *buf,
		int len, bool sync, bool generic)
{
	int retval = 0;
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	if (sync == false) {
		if (generic)
			retval = dsi_vc_dcs_generic_write_nosync(dssdev,
					ad->config_channel, buf, len);
		else
			retval = dsi_vc_dcs_write_nosync(dssdev,
					ad->config_channel, buf, len);
		if (retval < 0)
			dev_err(&dssdev->dev, "Failed to complete nosync write (0x%02x)\n",
					buf[0]);
	} else {
		if (generic)
			retval = dsi_vc_dcs_generic_write(dssdev,
					ad->config_channel, buf, len);
		else
			retval = dsi_vc_dcs_write(dssdev,
					ad->config_channel, buf, len);
		if (retval < 0)
			dev_err(&dssdev->dev, "Failed to complete write (0x%02x)\n",
					buf[0]);
	}

	mdelay(10);

	return retval;
}

static int td4291_dcs_write_sequence(struct omap_dss_device *dssdev,
		struct td4291_reg *seq, int len)
{
	int retval = 0;
	int ii;

	for (ii = 0; ii < len; ii++) {
		retval = td4291_dcs_write(dssdev, seq[ii].data, seq[ii].len,
				false, false);
		if (retval < 0) {
			dev_err(&dssdev->dev, "Failed to write sequence %d\n",
					ii);
			return -EINVAL;
		}
	}

	return 0;
}

static int td4291_hw_reset(struct omap_dss_device *dssdev)
{
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	gpio_set_value(ad->pdata->reset_gpio, 1);
	msleep(td4291_panel_config.reset_sequence.high);
	gpio_set_value(ad->pdata->reset_gpio, 0);
	msleep(td4291_panel_config.reset_sequence.low);
	gpio_set_value(ad->pdata->reset_gpio, 1);
	msleep(td4291_panel_config.sleep.hw_reset);

	return 0;
}

static int td4291_update_brightness(struct omap_dss_device *dssdev)
{
	return 0;
}

static int td4291_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static int td4291_set_brightness(struct backlight_device *bd)
{
	int retval = 0;
	int bl = bd->props.brightness;
	struct omap_dss_device *dssdev = dev_get_drvdata(&bd->dev);
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	if (bl == ad->bl)
		return 0;

	ad->bl = bl;

	mutex_lock(&ad->lock);
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dsi_bus_lock(dssdev);
		retval = td4291_update_brightness(dssdev);
		dsi_bus_unlock(dssdev);
	}
	mutex_unlock(&ad->lock);

	return retval;
}

static const struct backlight_ops td4291_backlight_ops = {
	.get_brightness = td4291_get_brightness,
	.update_status = td4291_set_brightness,
};

static int td4291_set_update_mode(struct omap_dss_device *dssdev,
		enum omap_dss_update_mode mode)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	if (mode != OMAP_DSS_UPDATE_AUTO)
		return -EINVAL;

	return 0;
}

static enum omap_dss_update_mode td4291_get_update_mode(
		struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	return OMAP_DSS_UPDATE_AUTO;
}

static int td4291_sync(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	return 0;
}

static void td4291_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "%s\n", __func__);

	if (ad->rotate == 0 || ad->rotate == 2) {
		*xres = dssdev->panel.timings.x_res;
		*yres = dssdev->panel.timings.y_res;
	} else {
		*yres = dssdev->panel.timings.x_res;
		*xres = dssdev->panel.timings.y_res;
	}

	return;
}

static int td4291_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	return 0;
}

static int td4291_set_rotate(struct omap_dss_device *dssdev,
		unsigned char rotate)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	return 0;
}

static unsigned char td4291_get_rotate(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	return 0;
}

static int td4291_set_mirror(struct omap_dss_device *dssdev, bool enable)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	return 0;
}

static bool td4291_get_mirror(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	return false;
}

static void td4291_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	*timings = dssdev->panel.timings;

	return;
}

static void td4291_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	dssdev->panel.timings.x_res = timings->x_res;
	dssdev->panel.timings.y_res = timings->y_res;
	dssdev->panel.timings.pixel_clock = timings->pixel_clock;
	dssdev->panel.timings.hfp = timings->hfp;
	dssdev->panel.timings.hsw = timings->hsw;
	dssdev->panel.timings.hbp = timings->hbp;
	dssdev->panel.timings.vfp = timings->vfp;
	dssdev->panel.timings.vsw = timings->vsw;
	dssdev->panel.timings.vbp = timings->vbp;

	return;
}

static int td4291_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	return 0;
}

static void td4291_set_reg(struct omap_dss_device *dssdev, unsigned char addr,
		unsigned char value)
{
	int retval;
	unsigned char cmd[2];
//	unsigned char readback;
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);
/*
	retval = dsi_vc_dcs_read(dssdev, ad->config_channel, addr, &readback,
			1);
	if (retval < 0)
		dev_err(&dssdev->dev, "Failed to read register 0x%02x\n",
				addr);
	dev_info(&dssdev->dev, "Register 0x%02x before = 0x%02x\n",
			addr, readback);
*/
	cmd[0] = addr;
	cmd[1] = value;

	if (ad->use_generic_access)
		retval = dsi_vc_dcs_generic_write_nosync(dssdev,
				ad->config_channel, cmd, 2);
	else
		retval = dsi_vc_dcs_write_nosync(dssdev,
				ad->config_channel, cmd, 2);

	if (retval < 0)
		dev_err(&dssdev->dev, "Failed to write register 0x%02x\n",
				addr);
/*
	retval = dsi_vc_dcs_read(dssdev, ad->config_channel, addr, &readback,
			1);
	if (retval < 0)
		dev_err(&dssdev->dev, "Failed to read register 0x%02x\n",
				addr);
	dev_info(&dssdev->dev, "Register 0x%02x after = 0x%02x\n",
			addr, readback);
*/
	msleep(ad->delay_time);

	return;
}

static void td4291_config(struct omap_dss_device *dssdev,
		struct td_4291_register_setting *reg_settings,
		unsigned char num_of_regs)
{
	int retval = 0;
	unsigned char ii;
	unsigned char cmd;
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	cmd = 0xde;
	retval = dsi_vc_dcs_write_nosync(dssdev, ad->config_channel, &cmd, 1);
	if (retval < 0)
		dev_err(&dssdev->dev, "Failed to enter register access mode\n");

	msleep(ad->delay_time);

	for (ii = 0; ii < num_of_regs; ii++) {
		td4291_set_reg(dssdev, reg_settings[ii].address,
				reg_settings[ii].value);
	}

	msleep(ad->delay_time);

	cmd = 0xdf;
	retval = dsi_vc_dcs_write_nosync(dssdev, ad->config_channel, &cmd, 1);
	if (retval < 0)
		dev_err(&dssdev->dev, "Failed to exit register access mode\n");

	return;
}

static int td4291_power_on(struct omap_dss_device *dssdev)
{
	int retval = 0;
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	if (!ad->enabled) {
		if (ad->pdata->set_power != NULL)
			ad->pdata->set_power(true);

		if (ad->pdata->set_backlight != NULL)
			ad->pdata->set_backlight(255);

		retval = omapdss_dsi_display_enable(dssdev);
		if (retval) {
			dev_err(&dssdev->dev, "Failed to enable DSI\n");
			goto exit;
		}

		td4291_hw_reset(dssdev);

		omapdss_dsi_vc_enable_hs(dssdev, ad->pixel_channel, true);

		dsi_videomode_panel_preinit(dssdev);

		ad->ready_for_access = true;

		td4291_dcs_write_sequence(dssdev, sleep_out,
				ARRAY_SIZE(sleep_out));
		msleep(td4291_panel_config.sleep.sleep_out);

		td4291_dcs_write_sequence(dssdev, display_on,
				ARRAY_SIZE(display_on));
		msleep(td4291_panel_config.sleep.display_on);

//		td4291_config(dssdev);

		retval = dsi_vc_set_max_rx_packet_size(dssdev,
				ad->config_channel, 0xff);
		if (retval < 0) {
			dev_err(&dssdev->dev, "Failed to set max Rx packet size\n");
			goto exit;
		}

		td4291_update_brightness(dssdev);

		dsi_video_mode_enable(dssdev, MIPI_DSI_PACKED_PIXEL_STREAM_24);

		ad->enabled = true;
	}

	synaptics_rmi4_touch_wake();

exit:
	return retval;
}

static void td4291_power_off(struct omap_dss_device *dssdev)
{
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	gpio_set_value(ad->pdata->reset_gpio, 0);

	ad->enabled = false;
	omapdss_dsi_display_disable(dssdev, false, false);

	if (ad->pdata->set_power != NULL)
		ad->pdata->set_power(false);

	return;
}

static int td4291_start(struct omap_dss_device *dssdev)
{
	int retval = 0;
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	mutex_lock(&ad->lock);

	dsi_bus_lock(dssdev);

	retval = td4291_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (retval) {
		dev_err(&dssdev->dev, "Failed to power on panel\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		dssdev->manager->enable(dssdev->manager);
	}

	mutex_unlock(&ad->lock);

	return retval;
}

static void td4291_stop(struct omap_dss_device *dssdev)
{
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	mutex_lock(&ad->lock);

	dssdev->manager->disable(dssdev->manager);

	dsi_bus_lock(dssdev);

	td4291_power_off(dssdev);

	dsi_bus_unlock(dssdev);

	mutex_unlock(&ad->lock);

	return;
}

static int td4291_enable(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	return td4291_start(dssdev);
}

static void td4291_disable(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		td4291_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	return;
}

static int td4291_suspend(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return -EINVAL;

	td4291_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	return 0;
}

static int td4291_resume(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED)
		return -EINVAL;

	return td4291_start(dssdev);
}

static int td4291_probe(struct omap_dss_device *dssdev)
{
	int retval = 0;
	struct td4291_data *ad = NULL;
	struct backlight_properties props = {
		.brightness = 255,
		.max_brightness = 255,
		.type = BACKLIGHT_RAW,
	};

	dev_dbg(&dssdev->dev, "%s\n", __func__);

	if (dssdev->data == NULL) {
		dev_err(&dssdev->dev, "No platform data\n");
		return -EINVAL;
	}

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = *td4291_panel_config.timings;
//	dssdev->panel.dsi_vm_data = vm_data; //check

	dssdev->ctrl.pixel_size = 24;
//	dssdev->panel.acbi = 0; //check
//	dssdev->panel.acb = 40; //check

	ad = kzalloc(sizeof(*ad), GFP_KERNEL);
	if (!ad)
		return -ENOMEM;

	ad->dssdev = dssdev;
	ad->pdata = dssdev->data;
	ad->delay_time = DELAY_TIME_MS;
	ad->use_generic_access = USE_GENERIC_ACCESS;

	if (!ad->pdata) {
		dev_err(&dssdev->dev, "Invalid platform data\n");
		retval = -EINVAL;
		goto err_pdata;
	}

	mutex_init(&ad->lock);

	dev_set_drvdata(&dssdev->dev, ad);

	ad->bldev = backlight_device_register("td4291", &dssdev->dev, dssdev,
		&td4291_backlight_ops, &props);
	if (IS_ERR(ad->bldev)) {
		retval = PTR_ERR(ad->bldev);
		goto err_backlight_device_register;
	}

	ad->debug_dir = debugfs_create_dir("td4291", NULL);
	if (!ad->debug_dir) {
		dev_err(&dssdev->dev, "Failed to create debug directory\n");
		goto err_debugfs;
	}

	retval = omap_dsi_request_vc(dssdev, &ad->pixel_channel);
	if (retval) {
		dev_err(&dssdev->dev, "Failed to request pixel channel\n");
		goto err_request_pixel_vc;
	}
	printk("Pixel channel = %d\n", ad->pixel_channel);

	retval = omap_dsi_set_vc_id(dssdev, ad->pixel_channel, 0);
	if (retval) {
		dev_err(&dssdev->dev, "Failed to set ID for pixel channel\n");
		goto err_set_pixel_vc_id;
	}

	retval = omap_dsi_request_vc(dssdev, &ad->config_channel);
	if (retval) {
		dev_err(&dssdev->dev, "Failed to request config channel\n");
		goto err_request_config_vc;
	}
	printk("Config channel = %d\n", ad->config_channel);

	retval = omap_dsi_set_vc_id(dssdev, ad->config_channel, 0);
	if (retval) {
		dev_err(&dssdev->dev, "Failed to set ID for config channel\n");
		goto err_set_config_vc_id;
	}

	retval = sysfs_create_group(&dssdev->dev.kobj, &td4291_attr_group);
	if (retval) {
		dev_err(&dssdev->dev, "Failed to create sysfs files\n");
		goto err_set_config_vc_id;
	}

	retval = gpio_export(ad->pdata->debug_mode_gpio, false);
	if (retval < 0)
		dev_err(&dssdev->dev, "Failed to export debug mode gpio\n");

	retval = gpio_export(ad->pdata->reset_gpio, false);
	if (retval < 0)
		dev_err(&dssdev->dev, "Failed to export attention gpio\n");

	return retval;

err_set_config_vc_id:
	omap_dsi_release_vc(dssdev, ad->config_channel);
err_request_config_vc:
err_set_pixel_vc_id:
	omap_dsi_release_vc(dssdev, ad->pixel_channel);
err_request_pixel_vc:
err_debugfs:
	backlight_device_unregister(ad->bldev);
err_backlight_device_register:
	mutex_destroy(&ad->lock);
	gpio_free(ad->pdata->reset_gpio);
err_pdata:
	kfree(ad);

	return retval;
}

static void td4291_remove(struct omap_dss_device *dssdev)
{
	struct td4291_data *ad = dev_get_drvdata(&dssdev->dev);

	debugfs_remove_recursive(ad->debug_dir);
	backlight_device_unregister(ad->bldev);
	mutex_destroy(&ad->lock);
	gpio_free(ad->pdata->reset_gpio);
	kfree(ad);

	return;
}

static struct omap_dss_driver td4291_driver = {
	.probe = td4291_probe,
	.remove = td4291_remove,

	.enable = td4291_enable,
	.disable = td4291_disable,
	.suspend = td4291_suspend,
	.resume = td4291_resume,

	.set_update_mode = td4291_set_update_mode,
	.get_update_mode = td4291_get_update_mode,

	.sync = td4291_sync,

	.get_resolution = td4291_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	/* dummy start */
	.enable_te = td4291_enable_te,
	.set_rotate = td4291_set_rotate,
	.get_rotate = td4291_get_rotate,
	.set_mirror = td4291_set_mirror,
	.get_mirror = td4291_get_mirror,
	/* dummy end */

	.get_timings = td4291_get_timings,
	.set_timings = td4291_set_timings,
	.check_timings = td4291_check_timings,

	.driver = {
		.name = "td4291",
		.owner = THIS_MODULE,
	},
};

static int __init td4291_init(void)
{
	omap_dss_register_driver(&td4291_driver);

	return 0;
}

static void __exit td4291_exit(void)
{
	omap_dss_unregister_driver(&td4291_driver);

	return;
}

module_init(td4291_init);
module_exit(td4291_exit);
