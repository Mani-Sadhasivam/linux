/*
 * adv7626.c  --  ADV7626 ALSA driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "adv7626.h"

static struct snd_soc_dai_driver adv7626_dai = {
        .name = "adv7626",
        .capture = {
                .stream_name = "Capture",
                .channels_min = 1,
                .channels_max = 2,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.formats = SNDRV_PCM_FMTBIT_S24_LE,
	},
};

static const struct snd_soc_codec_driver soc_codec_dev_adv7626;

static const struct regmap_config adv7626_regmap_cnf[] = {
	{
		.name			= "main",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "dpll_a",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "dpll_b",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "cp_lite_a",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "cp_lite_b",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "osd",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "hdmi_rx1",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "rx1_rpt",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "rx1_iframe",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "rx_edid",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "hdmi_rx2",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "rx2_rpt",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "rx2_iframe",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "txa_main",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "txa_edid",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "txa_test",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "txa_pmem",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "txa_cec",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "txb_main",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "txb_edid",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "txb_test",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "txb_pmem",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "txb_cec",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "adi_req",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
};

static int configure_regmap(struct adv7626_priv *adv7626, int idx)
{
	int ret;

	if (!adv7626->i2c_clients[idx])
		return -ENODEV;

	adv7626->regmap[idx] =
		devm_regmap_init_i2c(adv7626->i2c_clients[idx],
				     &adv7626_regmap_cnf[idx]);
	if (IS_ERR(adv7626->regmap[idx])) {
		ret = PTR_ERR(adv7626->regmap[idx]);
		pr_err("Error initializing regmap %d with error %d\n", idx, ret);
		return -EINVAL;
	}

	return 0;
}

static int adv7626_info(struct adv7626_priv *adv7626)
{
	unsigned int tmp1, tmp2;
	int ret;

	ret = regmap_read(adv7626->regmap[ADV7626_REG_MAIN],
				ADV7626_IOMAP_RD_INFO_1, &tmp1);
	if (ret) {
		pr_err("Error %d reading IO Regmap\n", ret);
		return -ENODEV;
	}

	ret = regmap_read(adv7626->regmap[ADV7626_REG_MAIN],
				ADV7626_IOMAP_RD_INFO_2, &tmp2);
	if (ret) {
		pr_err("Error %d reading IO Regmap\n", ret);
		return -ENODEV;
	}

	pr_info("ADV7626 ID: 0x%x\n", ((tmp1 << 8) | tmp2));

	return 0;
}

static void config_transceiver_mode(struct adv7626_priv *adv7626)
{
	int i;

	/* Power Up DPLL_A, DPLL_B, xtal */
	regmap_write(adv7626->regmap[ADV7626_REG_MAIN], 0x00, 0xC0);

	/* Power up TxA */
	regmap_write(adv7626->regmap[ADV7626_REG_TxA_MAIN], 0x41, 0x10);

	/* Power up TxB */
	regmap_write(adv7626->regmap[ADV7626_REG_TxB_MAIN], 0x41, 0x10);

	/* DPLL_A Set 128Fs MCLK Out */
	regmap_write(adv7626->regmap[ADV7626_REG_DPLL_A], 0xB5, 0x00);

	/* DPLL_B Set 128Fs MCLK Out */
	regmap_write(adv7626->regmap[ADV7626_REG_DPLL_B], 0xB5, 0x00);

	/* Configure MAIN reg */
	for (i = 0; i < 10; i++)
		regmap_write(adv7626->regmap[ADV7626_REG_MAIN], main_seq[i][0], main_seq[i][1]);

	/* Disable Auto Parameter Buffering */
	regmap_write(adv7626->regmap[ADV7626_REG_CP_LITE_A], 0xC9, 0x2D);

	/* Disable Auto Parameter Buffering */
	regmap_write(adv7626->regmap[ADV7626_REG_CP_LITE_B], 0xC9, 0x2D);

	/* HDMI Rx1 Configuration - input TMDS Clock equal to 297MHz */
	for (i = 0; i < 17; i++)
		regmap_write(adv7626->regmap[ADV7626_REG_HDMI_RX1], hdmi_rx1_seq[i][0], hdmi_rx1_seq[i][1]);

	/* HDMI Rx2 Configuration - input TMDS Clock equal to 297MHz */
	for (i = 0; i < 17; i++)
		regmap_write(adv7626->regmap[ADV7626_REG_HDMI_RX2], hdmi_rx2_seq[i][0], hdmi_rx2_seq[i][1]);

	/* Power Up Rx1 and Rx2 after Rx Configs */
	regmap_write(adv7626->regmap[ADV7626_REG_MAIN], 0x00, 0x00);

	/* Enable all EDIDs */
	regmap_write(adv7626->regmap[ADV7626_REG_Rx1_RPT], 0x74, 0x1F);

	/* HDMI TxA Config */
	regmap_write(adv7626->regmap[ADV7626_REG_TxA_TEST], 0x24, 0x42);
	for (i = 0; i < 42; i++)
		regmap_write(adv7626->regmap[ADV7626_REG_TxA_MAIN], hdmi_txa_seq[i][0], hdmi_txa_seq[i][1]);

	/* HDMI TxB Config */
	regmap_write(adv7626->regmap[ADV7626_REG_TxB_TEST], 0x24, 0x42);
	for (i = 0; i < 42; i++)
		regmap_write(adv7626->regmap[ADV7626_REG_TxB_MAIN], hdmi_txb_seq[i][0], hdmi_txb_seq[i][1]);

	/* Clear SPI Line Tristate */
	regmap_write(adv7626->regmap[ADV7626_REG_ADI_REQ], 0x1A ,0x00);
}

static int adv7626_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct adv7626_priv *adv7626;
	int ret, i;
	
	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	adv7626 = devm_kzalloc(&i2c->dev, sizeof(struct adv7626_priv),
			      GFP_KERNEL);
	if (!adv7626)
		return -ENOMEM;

	adv7626->i2c_clients[ADV7626_REG_MAIN] = i2c;
        i2c_set_clientdata(i2c, adv7626);

	/* Configure IO Regmap region */
	ret = configure_regmap(adv7626, ADV7626_REG_MAIN);
	if (ret) {
		dev_err(&i2c->dev, "Error configuring IO regmap region\n");
		return ret;
	}

	/* Do a global reset */
	regmap_write(adv7626->regmap[ADV7626_REG_MAIN], ADV7626_IOMAP_RST, 0xFF);

	ret = adv7626_info(adv7626);
	if (ret) {
		dev_err(&i2c->dev, "Error reading chip info\n");
		return ret;
	}

	for (i = ADV7626_REG_DPLL_A; i < ADV7626_REG_MAX; i++) {
		/* Program address map */
		regmap_write(adv7626->regmap[ADV7626_REG_MAIN],
					adv7626_addr[i].addr_loc,
					(adv7626_addr[i].addr << 1));
		
		adv7626->i2c_clients[i] = i2c_new_secondary_device(i2c, adv7626_addr[i].name,
							adv7626_addr[i].addr);
		if (!adv7626->i2c_clients[i]) {
			dev_err(&i2c->dev, "Failed to create i2c client %u\n", i);
			return -EINVAL;
		}
	}

	for (i = ADV7626_REG_DPLL_A; i < ADV7626_REG_MAX; i++) {
		ret = configure_regmap(adv7626, i);
		if (ret) {
			dev_err(&i2c->dev, "Error configuring IO regmap region\n");
			return ret;
		}
	}

	config_transceiver_mode(adv7626);

        return snd_soc_register_codec(&i2c->dev,
                        &soc_codec_dev_adv7626, &adv7626_dai, 1);

}

static int adv7626_i2c_remove(struct i2c_client *i2c)
{
	struct adv7626_priv *adv7626 = i2c_get_clientdata(i2c);
	unsigned int i;

	for (i = 1; i < ARRAY_SIZE(adv7626->i2c_clients); ++i)
		i2c_unregister_device(adv7626->i2c_clients[i]);

	snd_soc_unregister_codec(&i2c->dev);

	return 0;
}

static const struct of_device_id adv7626_of_match[] = {
	{ .compatible = "adi,adv7626" },
	{ },
};
MODULE_DEVICE_TABLE(of, adv7626_of_match);

static const struct i2c_device_id adv7626_i2c_id[] = {
	{ "adv7626", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adv7626_i2c_id);

static struct i2c_driver adv7626_i2c_driver = {
	.driver = {
		.name = "adv7626",
		.of_match_table = adv7626_of_match,
	},
	.probe = adv7626_i2c_probe,
	.remove = adv7626_i2c_remove,
	.id_table = adv7626_i2c_id
};

module_i2c_driver(adv7626_i2c_driver);

MODULE_DESCRIPTION("ADV7626 Codec driver");
MODULE_LICENSE("GPL");
