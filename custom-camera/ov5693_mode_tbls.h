#ifndef __OV5693_TABLES__
#define __OV5693_TABLES__

#include <media/camera_common.h>

#define OV5693_TABLE_WAIT_MS	0
#define OV5693_TABLE_END	1
#define OV5693_MAX_RETRIES	3
#define OV5693_WAIT_MS		10

#define ov5693_reg struct reg_8

static const ov5693_reg ov5693_start[] = {
	{0x0100, 0x01}, /* mode select streaming on */
	{OV5693_TABLE_END, 0x00}
};

static const ov5693_reg ov5693_stop[] = {
	{0x0100, 0x00}, /* mode select streaming on */
	{OV5693_TABLE_END, 0x00}
};

static const ov5693_reg tp_colorbars[] = {
	{0x0600, 0x00},
	{0x0601, 0x02},

	{OV5693_TABLE_WAIT_MS, OV5693_WAIT_MS},
	{OV5693_TABLE_END, 0x00}
};

static const ov5693_reg mode_3264x2464[] = {
	{OV5693_TABLE_WAIT_MS, OV5693_WAIT_MS},
	{OV5693_TABLE_END, 0x0000}
};

enum {
	OV5693_MODE_3264x2464,

	OV5693_MODE_START_STREAM,
	OV5693_MODE_STOP_STREAM,
	OV5693_MODE_TEST_PATTERN
};

static const ov5693_reg *mode_table[] = {
	[OV5693_MODE_3264x2464]		= mode_3264x2464,

	[OV5693_MODE_START_STREAM]		= ov5693_start,
	[OV5693_MODE_STOP_STREAM]		= ov5693_stop,
	[OV5693_MODE_TEST_PATTERN]		= tp_colorbars,
};

static const int ov5693_21fps[] = {
	21,
};

static const struct camera_common_frmfmt ov5693_frmfmt[] = {
	{{3264, 2464},	ov5693_21fps,	1, 0,	OV5693_MODE_3264x2464},
};
#endif  /* __OV5693_TABLES__ */