// SPDX-License-Identifier: GPL-2.0-or-later
//
// Actios Semi Owl SoCs Reset Driver
//
// Copyright (c) 2018 Linaro Ltd.
// Author: Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

#include "owl-reset.h"

static int owl_reset_assert(struct reset_controller_dev *rcdev,
			    unsigned long id)
{
	struct owl_reset *reset = to_owl_reset(rcdev);
	const struct owl_reset_map *map = &reset->reset_map[id];
	u32 reg;

	regmap_read(reset->regmap, map->reg, &reg);
	regmap_write(reset->regmap, map->reg, reg & ~map->bit);

	return 0;
}

static int owl_reset_deassert(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct owl_reset *reset = to_owl_reset(rcdev);
	const struct owl_reset_map *map = &reset->reset_map[id];
	u32 reg;

	regmap_read(reset->regmap, map->reg, &reg);
	regmap_write(reset->regmap, map->reg, reg | map->bit);

	return 0;
}

static int owl_reset_reset(struct reset_controller_dev *rcdev,
			   unsigned long id)
{
	owl_reset_assert(rcdev, id);
	udelay(1);
	owl_reset_deassert(rcdev, id);

	return 0;
}

static int owl_reset_status(struct reset_controller_dev *rcdev,
			    unsigned long id)
{
	struct owl_reset *reset = to_owl_reset(rcdev);
	const struct owl_reset_map *map = &reset->reset_map[id];
	u32 reg;

	regmap_read(reset->regmap, map->reg, &reg);

	/*
	 * The reset control API expects 0 if reset is not asserted,
	 * which is the opposite of what our hardware uses.
	 */
	return !(map->bit & reg);
}

const struct reset_control_ops owl_reset_ops = {
	.assert		= owl_reset_assert,
	.deassert	= owl_reset_deassert,
	.reset		= owl_reset_reset,
	.status		= owl_reset_status,
};
