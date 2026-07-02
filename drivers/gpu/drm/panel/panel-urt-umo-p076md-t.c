// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Mediatek
 * Author: Andrew Perepech <andrew.perepech@mediatek.com>
 *
 * Based on panel-truly-r63350a driver.
 */

#include <linux/backlight.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

struct lcm_init_cmd {
	u8 len;
	u8 data[64];
};

static const struct lcm_init_cmd urt_umo_p076md_t_init[] = {
	{ 2, { 0xB0, 0x00 } },
	{ 4, { 0xB9, 0xFF, 0x83, 0x94 } },
	{ 16, { 0xB1, 0x64, 0x10, 0x30, 0x44, 0x34, 0x11, 0xF1,
		     0x81, 0x70, 0xD9, 0x34, 0x80, 0xC0, 0xD2, 0x1F } },
	{ 13, { 0xB2, 0x45, 0x64, 0x0F, 0x09, 0x40, 0x1C, 0x08,
		     0x08, 0x1C, 0x4D, 0x00, 0x00 } },
	{ 23, { 0xB4, 0x00, 0xFF, 0x18, 0x60, 0x60, 0x60, 0x00,
		     0x00, 0x01, 0x30, 0x04, 0x68, 0x18, 0x60, 0x60,
		     0x60, 0x00, 0x00, 0x01, 0x30, 0x04, 0x68 } },
	{ 3, { 0xB6, 0x73, 0x73 } },
	{ 2, { 0xCC, 0x09 } },
	{ 33, { 0xD3, 0x00, 0x08, 0x00, 0x01, 0x07, 0x00, 0x08,
		     0x32, 0x10, 0x0A, 0x00, 0x05, 0x00, 0x20, 0x0A,
		     0x05, 0x09, 0x00, 0x32, 0x10, 0x08, 0x00, 0x11,
		     0x11, 0x0D, 0x07, 0x23, 0x0D, 0x07, 0x47, 0x0D,
		     0x08 } },
	{ 45, { 0xD5, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
		     0x00, 0x03, 0x03, 0x03, 0x03, 0x02, 0x02, 0x02,
		     0x02, 0x20, 0x20, 0x18, 0x18, 0x18, 0x18, 0x18,
		     0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x21,
		     0x21, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
		     0x18, 0x18, 0x18, 0x18, 0x18 } },
	{ 43, { 0xE0, 0x01, 0x09, 0x0B, 0x26, 0x2A, 0x2E, 0x14,
		     0x34, 0x05, 0x09, 0x0B, 0x16, 0x0E, 0x12, 0x14,
		     0x12, 0x14, 0x07, 0x13, 0x15, 0x17, 0x01, 0x09,
		     0x0B, 0x26, 0x2A, 0x2E, 0x14, 0x34, 0x05, 0x09,
		     0x0B, 0x16, 0x0E, 0x11, 0x14, 0x12, 0x14, 0x07,
		     0x13, 0x15, 0x17 } },
	{ 6, { 0xD9, 0x00, 0x01, 0x02, 0x07, 0x0C } },
};

struct urt_umo_p076md_t {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	struct regulator *power;
	bool prepared;
	bool enabled;
};

static inline struct urt_umo_p076md_t *to_urt_umo_p076md_t(struct drm_panel *panel)
{
	return container_of(panel, struct urt_umo_p076md_t, panel);
}

static int urt_umo_p076md_t_push_cmds(struct mipi_dsi_device *dsi,
				      const struct lcm_init_cmd *cmds,
				      size_t count)
{
	int ret = 0;
	size_t i;

	for (i = 0; i < count; i++) {
		ret = mipi_dsi_dcs_write_buffer(dsi, cmds[i].data, cmds[i].len);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int urt_umo_p076md_t_on(struct urt_umo_p076md_t *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = urt_umo_p076md_t_push_cmds(dsi, urt_umo_p076md_t_init,
					 ARRAY_SIZE(urt_umo_p076md_t_init));
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		return ret;

	msleep(120);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0)
		return ret;

	return 0;
}

static int urt_umo_p076md_t_off(struct urt_umo_p076md_t *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0)
		return ret;

	return 0;
}

static int urt_umo_p076md_t_disable(struct drm_panel *panel)
{
	struct urt_umo_p076md_t *ctx = to_urt_umo_p076md_t(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->panel.backlight)
		backlight_disable(ctx->panel.backlight);

	ctx->enabled = false;
	return 0;
}

static int urt_umo_p076md_t_unprepare(struct drm_panel *panel)
{
	struct urt_umo_p076md_t *ctx = to_urt_umo_p076md_t(panel);
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = urt_umo_p076md_t_off(ctx);
	if (ret < 0)
		return ret;

	regulator_disable(ctx->power);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	ctx->prepared = false;

	return 0;
}

static int urt_umo_p076md_t_prepare(struct drm_panel *panel)
{
	struct urt_umo_p076md_t *ctx = to_urt_umo_p076md_t(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	ret = regulator_enable(ctx->power);
	if (ret < 0)
		return ret;

	msleep(10);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);

	ret = urt_umo_p076md_t_on(ctx);
	if (ret < 0)
		goto poweroff;

	ctx->prepared = true;
	return 0;

poweroff:
	regulator_disable(ctx->power);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	return ret;
}

static int urt_umo_p076md_t_enable(struct drm_panel *panel)
{
	struct urt_umo_p076md_t *ctx = to_urt_umo_p076md_t(panel);

	if (!ctx->enabled && ctx->panel.backlight)
		backlight_enable(ctx->panel.backlight);

	ctx->enabled = true;
	return 0;
}

static const struct drm_display_mode urt_umo_p076md_t_default_mode = {
	.clock = 73000,
	.hdisplay = 800,
	.hsync_start = 800 + 77,
	.hsync_end = 800 + 77 + 24,
	.htotal = 800 + 77 + 24 + 24,
	.vdisplay = 1280,
	.vsync_start = 1280 + 12,
	.vsync_end = 1280 + 12 + 4,
	.vtotal = 1280 + 12 + 4 + 18,
	.width_mm = 94,
	.height_mm = 150,
};

static int urt_umo_p076md_t_get_modes(struct drm_panel *panel,
				      struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &urt_umo_p076md_t_default_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs urt_umo_p076md_t_funcs = {
	.disable = urt_umo_p076md_t_disable,
	.unprepare = urt_umo_p076md_t_unprepare,
	.prepare = urt_umo_p076md_t_prepare,
	.enable = urt_umo_p076md_t_enable,
	.get_modes = urt_umo_p076md_t_get_modes,
};

static int urt_umo_p076md_t_probe(struct mipi_dsi_device *dsi)
{
	struct urt_umo_p076md_t *ctx;
	int ret;

	ctx = devm_kzalloc(&dsi->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dsi = dsi;

	drm_panel_init(&ctx->panel, &dsi->dev, &urt_umo_p076md_t_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ctx->reset_gpio = devm_gpiod_get(&dsi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(&dsi->dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset GPIO\n");

	ctx->power = devm_regulator_get(&dsi->dev, "power");
	if (IS_ERR(ctx->power))
		return dev_err_probe(&dsi->dev, PTR_ERR(ctx->power),
				     "failed to get power regulator\n");


	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	drm_panel_add(&ctx->panel);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void urt_umo_p076md_t_remove(struct mipi_dsi_device *dsi)
{
	struct urt_umo_p076md_t *ctx = mipi_dsi_get_drvdata(dsi);

	urt_umo_p076md_t_disable(&ctx->panel);
	urt_umo_p076md_t_unprepare(&ctx->panel);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static void urt_umo_p076md_t_shutdown(struct mipi_dsi_device *dsi)
{
	struct urt_umo_p076md_t *ctx = mipi_dsi_get_drvdata(dsi);

	urt_umo_p076md_t_disable(&ctx->panel);
	urt_umo_p076md_t_unprepare(&ctx->panel);
}

static const struct of_device_id urt_umo_p076md_t_of_match[] = {
	{ .compatible = "urt,umo-p076md-t" },
	{ }
};
MODULE_DEVICE_TABLE(of, urt_umo_p076md_t_of_match);

static struct mipi_dsi_driver urt_umo_p076md_t_driver = {
	.driver = {
		.name = "panel-urt-umo-p076md-t",
		.of_match_table = urt_umo_p076md_t_of_match,
	},
	.probe = urt_umo_p076md_t_probe,
	.remove = urt_umo_p076md_t_remove,
	.shutdown = urt_umo_p076md_t_shutdown,
};
module_mipi_dsi_driver(urt_umo_p076md_t_driver);

MODULE_AUTHOR("Andrew Perepech <andrew.perepech@mediatek.com>");
MODULE_DESCRIPTION("U.R.T. UMO-P076MD-T DSI Panel Driver");
MODULE_LICENSE("GPL v2");
