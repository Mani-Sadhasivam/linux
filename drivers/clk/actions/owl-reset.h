#ifndef _OWL_RESET_H_
#define _OWL_RESET_H_

#include <linux/reset-controller.h>
#include <linux/spinlock.h>

struct owl_reset_map {
	u16	reg;
	u32	bit;
};

struct owl_reset {
	struct reset_controller_dev	rcdev;
	struct owl_reset_map		*reset_map;
	struct regmap			*regmap;
};

static inline struct owl_reset *to_owl_reset(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct owl_reset, rcdev);
}

extern const struct reset_control_ops owl_reset_ops;

#endif /* _OWL_RESET_H_ */
