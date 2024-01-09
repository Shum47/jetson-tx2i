#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/tegra-v4l2-camera.h>
#include <media/tegracam_core.h>
#include <media/ov5693.h>
#include "ov5693_mode_tbls.h"
#define CREATE_TRACE_POINTS
#include <trace/events/ov5693.h>

#define OV5693_MAX_COARSE_DIFF		6
#define OV5693_MAX_FRAME_LENGTH	(0x7fff)
#define OV5693_MIN_EXPOSURE_COARSE	(0x0002)
#define OV5693_MAX_EXPOSURE_COARSE	\
	(OV5693_MAX_FRAME_LENGTH-OV5693_MAX_COARSE_DIFF)
#define OV5693_DEFAULT_LINE_LENGTH	(0xA80)
#define OV5693_DEFAULT_PIXEL_CLOCK	(160)
#define OV5693_DEFAULT_FRAME_LENGTH	(0x07C0)
#define OV5693_DEFAULT_EXPOSURE_COARSE	\
	(OV5693_DEFAULT_FRAME_LENGTH-OV5693_MAX_COARSE_DIFF)

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_EXPOSURE_SHORT,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_GROUP_HOLD,
	TEGRA_CAMERA_CID_HDR_EN,
};

struct ov5693 {
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	const char			*devname;
	struct mutex			streaming_lock;
	bool				streaming;

	s32				group_hold_prev;
	u32				frame_length;
	bool				group_hold_en;
	struct camera_common_i2c	i2c_dev;
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
};

static struct regmap_config ov5693_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

static inline void ov5693_get_frame_length_regs(ov5693_reg *regs,
				u32 frame_length)
{
	regs->addr = OV5693_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = OV5693_FRAME_LENGTH_ADDR_LSB;
	(regs + 1)->val = (frame_length) & 0xff;
}

static inline void ov5693_get_coarse_time_regs(ov5693_reg *regs,
				u32 coarse_time)
{
	regs->addr = OV5693_COARSE_TIME_ADDR_1;
	regs->val = (coarse_time >> 12) & 0xff;
	(regs + 1)->addr = OV5693_COARSE_TIME_ADDR_2;
	(regs + 1)->val = (coarse_time >> 4) & 0xff;
	(regs + 2)->addr = OV5693_COARSE_TIME_ADDR_3;
	(regs + 2)->val = (coarse_time & 0xf) << 4;
}

static inline void ov5693_get_coarse_time_short_regs(ov5693_reg *regs,
				u32 coarse_time)
{
	regs->addr = OV5693_COARSE_TIME_SHORT_ADDR_1;
	regs->val = (coarse_time >> 12) & 0xff;
	(regs + 1)->addr = OV5693_COARSE_TIME_SHORT_ADDR_2;
	(regs + 1)->val = (coarse_time >> 4) & 0xff;
	(regs + 2)->addr = OV5693_COARSE_TIME_SHORT_ADDR_3;
	(regs + 2)->val = (coarse_time & 0xf) << 4;
}

static inline void ov5693_get_gain_regs(ov5693_reg *regs,
				u16 gain)
{
	regs->addr = OV5693_GAIN_ADDR_MSB;
	regs->val = (gain >> 8) & 0xff;

	(regs + 1)->addr = OV5693_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xff;
}

static int test_mode;
module_param(test_mode, int, 0644);

static int ov5693_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	return 0;
}

static int ov5693_write_table(struct ov5693 *priv,
			      const ov5693_reg table[])
{
	return 0;
}

static int ov5693_power_on(struct camera_common_data *s_data)
{
	struct camera_common_power_rail *pw = s_data->power;
	pw->state = SWITCH_ON;
	return 0;
}

static int ov5693_power_off(struct camera_common_data *s_data)
{
	struct camera_common_power_rail *pw = s_data->power;
	pw->state = SWITCH_OFF;
	return 0;
}

static int ov5693_power_put(struct tegracam_device *tc_dev)
{
	return 0;
}

static int ov5693_power_get(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	pw->state = SWITCH_OFF;
	return 0;
}

static int ov5693_set_gain(struct tegracam_device *tc_dev, s64 val);
static int ov5693_set_frame_rate(struct tegracam_device *tc_dev, s64 val);
static int ov5693_set_exposure(struct tegracam_device *tc_dev, s64 val);
static int ov5693_set_exposure_short(struct tegracam_device *tc_dev, s64 val);

static const struct of_device_id ov5693_of_match[] = {
	{
		.compatible = "nvidia,ov5693",
	},
	{ },
};

static int ov5693_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	int err;
	struct ov5693 *priv = tc_dev->priv;
	int gh_prev = switch_ctrl_qmenu[priv->group_hold_prev];
	struct device *dev = tc_dev->dev;

	if (priv->group_hold_en == true && gh_prev == SWITCH_OFF) {
		camera_common_i2c_aggregate(&priv->i2c_dev, true);
		/* enter group hold */
		err = ov5693_write_reg(priv->s_data,
				       OV5693_GROUP_HOLD_ADDR, val);
		if (err)
			goto fail;

		priv->group_hold_prev = 1;

		dev_info(dev, "%s: enter group hold\n", __func__);
	} else if (priv->group_hold_en == false && gh_prev == SWITCH_ON) {
		/* leave group hold */
		err = ov5693_write_reg(priv->s_data,
				       OV5693_GROUP_HOLD_ADDR, 0x11);
		if (err)
			goto fail;

		err = ov5693_write_reg(priv->s_data,
				       OV5693_GROUP_HOLD_ADDR, 0x61);
		if (err)
			goto fail;

		priv->group_hold_prev = 0;

		dev_info(dev, "%s: leave group hold\n", __func__);
	}

	return 0;

fail:
	dev_info(dev, "%s: Group hold control error\n", __func__);
	return err;
}

static int ov5693_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct ov5693 *priv = (struct ov5693 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	ov5693_reg reg_list[2];
	int err;
	u16 gain;
	int i;

	if (!priv->group_hold_prev)
		ov5693_set_group_hold(tc_dev, 1);

	/* translate value */
	gain = (u16) (((val * 16) +
			(mode->control_properties.gain_factor / 2)) /
			mode->control_properties.gain_factor);
	ov5693_get_gain_regs(reg_list, gain);
	dev_info(dev, "%s: gain %d val: %lld\n", __func__, gain, val);

	for (i = 0; i < 2; i++) {
		err = ov5693_write_reg(s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_info(dev, "%s: GAIN control error\n", __func__);
	return err;
}

static int ov5693_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct ov5693 *priv = tc_dev->priv;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	ov5693_reg reg_list[2];
	int err;
	u32 frame_length;
	int i;

	if (!priv->group_hold_prev)
		ov5693_set_group_hold(tc_dev, 1);

	frame_length =  mode->signal_properties.pixel_clock.val *
		mode->control_properties.framerate_factor /
		mode->image_properties.line_length / val;

	ov5693_get_frame_length_regs(reg_list, frame_length);
	dev_info(dev, "%s: val: %d\n", __func__, frame_length);

	for (i = 0; i < 2; i++) {
		err = ov5693_write_reg(s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	priv->frame_length = frame_length;

	return 0;

fail:
	dev_info(dev, "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

static int ov5693_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct ov5693 *priv = tc_dev->priv;
	const s32 max_coarse_time = priv->frame_length - OV5693_MAX_COARSE_DIFF;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	ov5693_reg reg_list[3];
	int err;
	u32 coarse_time;
	int i;

	if (!priv->group_hold_prev)
		ov5693_set_group_hold(tc_dev, 1);

	coarse_time = (u32)(((mode->signal_properties.pixel_clock.val*val)
			/mode->image_properties.line_length)/
			mode->control_properties.exposure_factor);
	if (coarse_time < OV5693_MIN_EXPOSURE_COARSE)
		coarse_time = OV5693_MIN_EXPOSURE_COARSE;
	else if (coarse_time > max_coarse_time)
		coarse_time = max_coarse_time;
	ov5693_get_coarse_time_regs(reg_list, coarse_time);
	dev_info(dev, "%s: val: %d\n", __func__, coarse_time);

	for (i = 0; i < 3; i++) {
		err = ov5693_write_reg(s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_info(dev, "%s: COARSE_TIME control error\n", __func__);
	return err;
}

static int ov5693_set_exposure_short(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct ov5693 *priv = tc_dev->priv;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	ov5693_reg reg_list[3];
	int err;
	struct v4l2_control hdr_control;
	int hdr_en;
	u32 coarse_time_short;
	int i;
	if (!priv->group_hold_prev)
		ov5693_set_group_hold(tc_dev, 1);

	/* check hdr enable ctrl */
	hdr_control.id = TEGRA_CAMERA_CID_HDR_EN;

	err = camera_common_g_ctrl(s_data, &hdr_control);
	if (err < 0) {
		dev_err(dev, "could not find device ctrl.\n");
		return err;
	}

	hdr_en = switch_ctrl_qmenu[hdr_control.value];
	if (hdr_en == SWITCH_OFF)
		return 0;

	coarse_time_short = (u32)(((mode->signal_properties.pixel_clock.val*val)
				/mode->image_properties.line_length)
				/mode->control_properties.exposure_factor);

	ov5693_get_coarse_time_short_regs(reg_list, coarse_time_short);
	dev_info(dev, "%s: val: %d\n", __func__, coarse_time_short);

	for (i = 0; i < 3; i++) {
		err = ov5693_write_reg(s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_info(dev, "%s: COARSE_TIME_SHORT control error\n", __func__);
	return err;
}

MODULE_DEVICE_TABLE(of, ov5693_of_match);

static struct camera_common_pdata *ov5693_parse_dt(struct tegracam_device
							*tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *node = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!node)
		return NULL;

	match = of_match_device(ov5693_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = camera_common_parse_clocks(dev,
					 board_priv_pdata);
	if (err) {
		dev_err(dev, "Failed to find clocks\n");
	}

	return board_priv_pdata;
}

static int ov5693_set_mode(struct tegracam_device *tc_dev)
{
	struct ov5693 *priv = (struct ov5693 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	int err;

	err = ov5693_write_table(priv, mode_table[s_data->mode_prop_idx]);
	if (err)
		return err;

	return 0;
}

static int ov5693_start_streaming(struct tegracam_device *tc_dev)
{
	struct ov5693 *priv = (struct ov5693 *)tegracam_get_privdata(tc_dev);

	mutex_lock(&priv->streaming_lock);
	priv->streaming = true;
	mutex_unlock(&priv->streaming_lock);

	return 0;
}

static int ov5693_stop_streaming(struct tegracam_device *tc_dev)
{
	struct ov5693 *priv = (struct ov5693 *)tegracam_get_privdata(tc_dev);
	u32 frame_time;

	mutex_lock(&priv->streaming_lock);
	priv->streaming = false;
	mutex_unlock(&priv->streaming_lock);

	/*
	 * Wait for one frame to make sure sensor is set to
	 * software standby in V-blank
	 *
	 * frame_time = frame length rows * Tline
	 * Tline = line length / pixel clock (in MHz)
	 */
	frame_time = priv->frame_length *
		OV5693_DEFAULT_LINE_LENGTH / OV5693_DEFAULT_PIXEL_CLOCK;

	usleep_range(frame_time, frame_time + 1000);

	return 0;
}

static struct camera_common_sensor_ops ov5693_common_ops = {
	.numfrmfmts = ARRAY_SIZE(ov5693_frmfmt),
	.frmfmt_table = ov5693_frmfmt,
	.power_on = ov5693_power_on,
	.power_off = ov5693_power_off,
	.write_reg = ov5693_write_reg,
	.parse_dt = ov5693_parse_dt,
	.power_get = ov5693_power_get,
	.power_put = ov5693_power_put,
	.set_mode = ov5693_set_mode,
	.start_streaming = ov5693_start_streaming,
	.stop_streaming = ov5693_stop_streaming,
};

static struct tegracam_ctrl_ops ov5693_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = ov5693_set_gain,
	.set_exposure = ov5693_set_exposure,
	.set_exposure_short = ov5693_set_exposure_short,
	.set_frame_rate = ov5693_set_frame_rate,
	.set_group_hold = ov5693_set_group_hold,
};

static int ov5693_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_info(&client->dev, "%s:\n", __func__);
	return 0;
}

static const struct v4l2_subdev_internal_ops ov5693_subdev_internal_ops = {
	.open = ov5693_open,
};

static int ov5693_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = client->dev.of_node;
	struct tegracam_device *tc_dev;
	struct ov5693 *priv;
	int err;
	const struct of_device_id *match;

	dev_info(dev, "probing v4l2 sensor no i2c.\n");

	match = of_match_device(ov5693_of_match, dev);
	if (!match) {
		dev_err(dev, "No device match found\n");
		return -ENODEV;
	}

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			    sizeof(struct ov5693), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			    sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "ov5693", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &ov5693_regmap_config;
	tc_dev->sensor_ops = &ov5693_common_ops;
	tc_dev->v4l2sd_internal_ops = &ov5693_subdev_internal_ops;
	tc_dev->tcctrl_ops = &ov5693_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}

	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);
	mutex_init(&priv->streaming_lock);

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_info(dev, "Detected OV5693 sensor\n");

	return 0;
}

static int
ov5693_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ov5693 *priv = (struct ov5693 *)s_data->priv;

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	ov5693_power_put(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	mutex_destroy(&priv->streaming_lock);

	return 0;
}

static const struct i2c_device_id ov5693_id[] = {
	{ "ov5693", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov5693_id);

static struct i2c_driver ov5693_i2c_driver = {
	.driver = {
		.name = "ov5693",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ov5693_of_match),
	},
	.probe = ov5693_probe,
	.remove = ov5693_remove,
	.id_table = ov5693_id,
};
module_i2c_driver(ov5693_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for OmniVision OV5693");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");