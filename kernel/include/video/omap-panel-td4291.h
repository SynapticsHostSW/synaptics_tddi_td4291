#ifndef __OMAP_PANEL_TD4291_H
#define __OMAP_PANEL_TD4291_H

struct omap_dss_device;

struct panel_td4291_data {
	const char *name;
	int debug_mode_gpio;
	int reset_gpio;
	int vid_if_sel0_gpio;
	int vid_if_sel1_gpio;
	int (*set_backlight)(int level);
	int (*set_power)(bool on_off);
};

#endif /* __OMAP_PANEL_TD4291_H */
