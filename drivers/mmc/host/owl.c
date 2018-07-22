/*
 * owl_mmc.c - owl SD/MMC driver
 *
 * Copyright (C) 2015, Actions Semiconductor Co. LTD.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
  * Author: ZhengFeng
 *       	  16 August 2015
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/cpufreq.h>
#include <linux/genhd.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/reset.h>
#include "owl_mmc.h"
#include "owl_wlan_plat_data.h"
#include "owl_wlan_device.h"

static int owl_check_trs_date_status(struct owl_mmc_host *host,
				     struct mmc_request *mrq);
static void owl_dump_reg(struct owl_mmc_host *host);

static int owl_switch_sd_pinctr(struct mmc_host *mmc);
static int owl_switch_uartsd_pinctr(struct mmc_host *mmc);

void owl_dump_mfp(struct owl_mmc_host *host);

static int owl_switch_uartsd_pinctr(struct mmc_host *mmc)
{

	struct owl_mmc_host *host;
	host = mmc_priv(mmc);

	if (mmc_card_expected_mem(host->type_expected) &&
	    (host->sdio_uart_supported)) {

		if (host->switch_pin_flag == UART_PIN) {

			host->pcl = pinctrl_get_select(host->mmc->parent,
						       "share_uart2_5");
			if (IS_ERR(host->pcl)) {
				pr_err
				    ("swtich SDC%d get default pinctrl failed, %ld\n",
				     host->id, PTR_ERR(host->pcl));
				return (long)PTR_ERR(host->pcl);
			}
			host->switch_pin_flag = UART_SD_PIN;

		} else if (host->switch_pin_flag == SD_PIN) {
			/* release sd pin */
			if ((host->pcl) && (!IS_ERR(host->pcl))) {
				pinctrl_put(host->pcl);
			}

			host->pcl = pinctrl_get_select(host->mmc->parent,
						       "share_uart2_5");
			if (IS_ERR(host->pcl)) {
				pr_err
				    ("swtich uart SDC%d get default pinctrl failed, %ld\n",
				     host->id, PTR_ERR(host->pcl));
				return (long)PTR_ERR(host->pcl);
			}

			host->switch_pin_flag = UART_SD_PIN;
		}
	}

	return 0;
}

static int owl_switch_sd_pinctr(struct mmc_host *mmc)
{

	struct owl_mmc_host *host;
	host = mmc_priv(mmc);

	if (mmc_card_expected_mem(host->type_expected) &&
	    (host->sdio_uart_supported)) {

		if (host->switch_pin_flag == UART_SD_PIN) {
			/* release sd pin */
			if ((host->pcl) && (!IS_ERR(host->pcl))) {
				pinctrl_put(host->pcl);
			}

			host->pcl = pinctrl_get_select(host->mmc->parent,
						       host->pinctrname);
			if (IS_ERR(host->pcl)) {
				pr_err
				    ("swtich sd SDC%d get default pinctrl failed, %ld\n",
				     host->id, PTR_ERR(host->pcl));
				return (long)PTR_ERR(host->pcl);
			}
			host->switch_pin_flag = SD_PIN;

		} else if (host->switch_pin_flag == UART_PIN) {

			host->pcl = pinctrl_get_select(host->mmc->parent,
						       host->pinctrname);
			if (IS_ERR(host->pcl)) {
				pr_err
				    ("swtich sd SDC%d get default pinctrl failed, %ld\n",
				     host->id, PTR_ERR(host->pcl));
				return (long)PTR_ERR(host->pcl);
			}
			host->switch_pin_flag = SD_PIN;
		}

	}

	return 0;
}

#undef pr_debug
/*#define OWL_MMC_DEBUG*/
#ifdef OWL_MMC_DEBUG
#define pr_debug(format, arg...)	\
	printk(KERN_INFO format, ##arg)
#else
#define pr_debug(format, args...)
#endif

/*
 * SD Controller Linked Card type, one of those:
 * MMC_CARD_DISABLE | MMC_CARD_MEMORY | MMC_CARD_WIFI
 */
static const char *const card_types[] = {
	[MMC_CARD_DISABLE] = "none",
	[MMC_CARD_MEMORY] = "memory",
	[MMC_CARD_EMMC] = "emmc",
	[MMC_CARD_WIFI] = "wifi",
};

/*
 * Method to detect card Insert/Extract:
 */
static const char *const card_detect_modes[] = {
	[SIRQ_DETECT_CARD] = "sirq",
	[GPIO_DETECT_CARD] = "gpio",
	[COMMAND_DETECT_CARD] = "command",
};

static const char *const wifi_card_voltage[] = {
	[DETECT_CARD_LOW_VOLTAGE] = "1.8v",
	[DETECT_CARD_NORMAL_VOLTAGE] = "3.1v",
};

static int detect_use_gpio;
extern int owl_wlan_set_power(struct wlan_plat_data *pdata, int on,
			      unsigned long msec);
extern struct wlan_plat_data *owl_get_wlan_plat_data(void);

static int owl_clk_set_rate(struct owl_mmc_host *host, unsigned long freq)
{
	unsigned long rate;
	int ret;
	freq = freq << 1;
	rate = clk_round_rate(host->clk, freq);
	if (rate < 0) {
		pr_err("SDC%d cannot get suitable rate:%lu\n", host->id, rate);
		return -ENXIO;
	}

	ret = clk_set_rate(host->clk, rate);
	if (ret < 0) {
		pr_err("SDC%d Cannot set rate %ld: %d\n", host->id, rate, ret);
		return ret;
	}

	return 0;
}

static inline int owl_enable_clock(struct owl_mmc_host *host)
{
	int ret;

	if (!host->clk_on) {
		ret = clk_prepare_enable(host->clk);

		if (ret) {
			pr_err("SDC[%d] enable module clock error\n", host->id);
			return ret;
		}
		host->clk_on = 1;
	}
	return 0;
}

static inline void owl_disable_clock(struct owl_mmc_host *host)
{
	if (host->clk_on) {
		clk_disable_unprepare(host->clk);
		host->clk_on = 0;
	}
}

static inline int owl_enable_nand_clock(struct owl_mmc_host *host)
{
	int ret;

	if (!host->nandclk_on) {
		ret = clk_prepare_enable(host->nandclk);

		if (ret) {
			pr_err("nand%d enable module clock error\n",
			       (host->id - 2));
			return ret;
		}
		host->nandclk_on = 1;
	}
	return 0;
}

static inline void owl_disable_nand_clock(struct owl_mmc_host *host)
{
	if (host->nandclk_on) {
		clk_disable_unprepare(host->nandclk);
		host->nandclk_on = 0;
	}
}

static int owl_mmc_send_init_clk(struct owl_mmc_host *host)
{
	u32 mode;
	int ret = 0;

	init_completion(&host->sdc_complete);

	mode = SD_CTL_TS | SD_CTL_TCN(5) | SD_CTL_TM(8);
	mode |= (readl(HOST_CTL(host)) & (0xff << 16));
	writel(mode, HOST_CTL(host));

	if (!wait_for_completion_timeout(&host->sdc_complete, HZ)) {
		pr_err("*SDC%d send 80 init clock timeout error\n", host->id);
		owl_dump_reg(host);
		ret = -1;
	}
	return ret;
}

static void owl_mmc_set_clk(struct owl_mmc_host *host, int rate)
{
	if (0 == rate) {
		pr_err("SDC%d set clock error\n", host->id);
		return;
	}

	/*
	 * Set the RDELAY and WDELAY based on the sd clk.
	 */
	if (rate <= 1000000) {

		writel((readl(HOST_CTL(host)) & (~(0xff << 16))) |
		       SD_CTL_RDELAY(host->rdelay.delay_lowclk) |
		       SD_CTL_WDELAY(host->wdelay.delay_lowclk),
		       HOST_CTL(host));

	} else if ((rate > 1000000) && (rate <= 26000000)) {
		writel((readl(HOST_CTL(host)) & (~(0xff << 16))) |
		       SD_CTL_RDELAY(host->rdelay.delay_midclk) |
		       SD_CTL_WDELAY(host->wdelay.delay_midclk),
		       HOST_CTL(host));

	} else if ((rate > 26000000) && (rate <= 52000000)) {
		if ((readl(HOST_EN(host)) & (1 << 2))
		    && (SDC0_SLOT == host->id)) {
			/*for sd0 ddr50 mode,specil delay chain */
			writel((readl(HOST_CTL(host)) & (~(0xff << 16))) |
			       SD_CTL_RDELAY(SDC0_RDELAY_DDR50) |
			       SD_CTL_WDELAY(SDC0_WDELAY_DDR50),
			       HOST_CTL(host));
		} else {
			writel((readl(HOST_CTL(host)) & (~(0xff << 16))) |
			       SD_CTL_RDELAY(host->rdelay.delay_highclk) |
			       SD_CTL_WDELAY(host->wdelay.delay_highclk),
			       HOST_CTL(host));
		}

	} else if ((rate > 52000000) && (rate <= 100000000)) {

		writel((readl(HOST_CTL(host)) & (~(0xff << 16))) |
		       SD_CTL_RDELAY(6) | SD_CTL_WDELAY(4), HOST_CTL(host));

	} else {
		pr_err("SD3.0 max clock should not > 100Mhz\n");

	}

	host->read_delay_chain = (readl(HOST_CTL(host)) & (0xf << 20)) >> 20;
	host->write_delay_chain = (readl(HOST_CTL(host)) & (0xf << 16)) >> 16;
	host->write_delay_chain_bak = host->write_delay_chain;
	host->read_delay_chain_bak = host->read_delay_chain;

	owl_clk_set_rate(host, rate);

}

static int owl_mmc_opt_regulator(struct owl_mmc_host *host, bool enable)
{
	int ret = 0;
	if (!(IS_ERR(host->reg) || NULL == host->reg)) {
		if ((enable == REG_ENABLE) &&
		    (OWL_REGULATOR_OFF == host->regulator_status)) {
			ret = regulator_enable(host->reg);
			host->regulator_status = OWL_REGULATOR_ON;
		} else if ((enable == REG_DISENABLE) &&
			   (OWL_REGULATOR_ON == host->regulator_status)) {
			ret = regulator_disable(host->reg);
			host->regulator_status = OWL_REGULATOR_OFF;
		} else {
			WARN_ON(1);
		}

	}
	return ret;
}

void owl_mmc_ctr_reset(struct owl_mmc_host *host)
{
	/* Reset the sd controller to clear all previous status. */
	reset_control_assert(host->rst);
	udelay(20);
	reset_control_deassert(host->rst);
}

static int owl_enable_powergate_clk(struct owl_mmc_host *host)
{

	unsigned int nand_alg_ctr;
	int ret = 0;

	if (host->nand_powergate == POWER_GATE_OFF) {

		pm_runtime_get_sync(host->mmc->parent);
		owl_enable_nand_clock(host);

		nand_alg_ctr = readl(host->nand_io_base + NAND_ALOG_CTR);
		nand_alg_ctr &= ~((1 << 21) | (1 << 20));
		nand_alg_ctr |= (1 << 21);
		/*set pad driver */
		nand_alg_ctr &= ~0xffff;
		nand_alg_ctr |= 0xaaaa;

		/* set nand power pin */
		if (nand_alg_ctr & EN_V18_R) {
			nand_alg_ctr |= EN_V18_W;
		} else {
			nand_alg_ctr &= ~EN_V18_W;
		}

		writel(nand_alg_ctr, (host->nand_io_base + NAND_ALOG_CTR));
		/*set powerpin for host2 and host3 */

		host->nand_powergate = POWER_GATE_ON;

	}

	return ret;
}

static inline void owl_disable_powergate_clk(struct owl_mmc_host *host)
{

	if (host->nandclk_on) {
		clk_disable_unprepare(host->nandclk);
		host->nandclk_on = 0;
	}
	if (host->nand_powergate == POWER_GATE_ON) {
		pm_runtime_put_sync(host->mmc->parent);
		host->nand_powergate = POWER_GATE_OFF;
	}
}

static void owl_mmc_power_up(struct owl_mmc_host *host)
{

	u32 pad_ctr;
	/* enable gl5302 power for card */
	/* power on reset */
	owl_mmc_ctr_reset(host);
	/* open nand powr gateing and operare nand ctr */

	if (host->id == SDC2_SLOT || host->id == SDC3_SLOT) {
		if (owl_enable_powergate_clk(host)) {
			pr_err("err:host%d power_up\n", host->id);
			return;
		}
	}

	if (host->id == SDC1_SLOT) {

		pad_ctr = readl(PAD_CTL + host->mfp_base);

		if (DETECT_CARD_LOW_VOLTAGE == host->wifi_detect_voltage) {
			pad_ctr |= SD1_PAD_POWER;
		} else {
			pad_ctr &= ~SD1_PAD_POWER;
		}

		writel(pad_ctr, PAD_CTL + host->mfp_base);
	}

	if (host->id == SDC2_SLOT) {

	}

	owl_enable_clock(host);
	writel(SD_ENABLE | SD_EN_RESE, HOST_EN(host));
}

/*
* only when sd0 used for sdcard ,
* we set uart ux rx vaild,sd0 clk cmd vaild,
* so it rescan sdcard and uart pin is vaild
*/

static int owl_mmc_request_pinctrl(struct owl_mmc_host *host)
{

	/* host0 usd for sdcard need not free,
	   it will free when resan fail */
	if (mmc_card_expected_mem(host->type_expected) &&
	    (host->sdio_uart_supported)) {

		goto out;

	} else {

		if (NULL == host->pcl) {
			host->pcl = pinctrl_get_select(host->mmc->parent,
						       host->pinctrname);
		}

		if (IS_ERR(host->pcl)) {
			pr_err("SDC%d get default pinctrl failed, %ld\n",
			       host->id, PTR_ERR(host->pcl));
			return (long)PTR_ERR(host->pcl);
		}
	}

out:
	return 0;
}

static int owl_mmc_free_pinctrl(struct owl_mmc_host *host)
{

	/* host0 usd for sdcard need not free,
	   it will free when resan fail */
	if (mmc_card_expected_mem(host->type_expected) &&
	    (host->sdio_uart_supported)) {

		goto out;

	} else {
		if ((host->pcl) && (!IS_ERR(host->pcl))) {
			pinctrl_put(host->pcl);
			host->pcl = NULL;
		}
	}

out:
	return 0;

}

static void owl_mmc_power_on(struct owl_mmc_host *host)
{

	mutex_lock(&host->pin_mutex);
	if (owl_mmc_request_pinctrl(host) < 0)
		pr_err("SDC%d request pinctrl failed\n", host->id);
	mutex_unlock(&host->pin_mutex);

	/* clocks is provided to eliminate power-up synchronization problems */
	/* enabel cmd irq */
	writel(readl(HOST_STATE(host)) | SD_STATE_TEIE, HOST_STATE(host));

	/* module function enable */
	if (mmc_card_expected_wifi(host->type_expected)) {
		writel(readl(HOST_EN(host)) | SD_EN_SDIOEN, HOST_EN(host));
	}

	owl_mmc_send_init_clk(host);

}

static void owl_mmc_power_off(struct owl_mmc_host *host)
{
	int ret = 0;
	mutex_lock(&host->pin_mutex);
	if (owl_mmc_free_pinctrl(host) < 0)
		pr_err("SDC%d free pinctrl failed\n", host->id);
	mutex_unlock(&host->pin_mutex);
	/* close power */
	if ((OWL_REGULATOR_ON == host->regulator_status) &&
	    (!mmc_card_expected_mem(host->type_expected))) {
		ret = owl_mmc_opt_regulator(host, REG_DISENABLE);
		if (ret) {
			pr_err("%s:%d\n", __FUNCTION__, __LINE__);
			WARN_ON(1);
		}
	}
	owl_disable_clock(host);
	/*close nand power gate and clk */
	if (SDC2_SLOT == host->id || SDC3_SLOT == host->id) {
		owl_disable_powergate_clk(host);
	}

}

static void owl_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct owl_mmc_host *host = mmc_priv(mmc);
	u32 ctrl_reg;

	if (ios->power_mode != host->power_state) {
		host->power_state = ios->power_mode;

		switch (ios->power_mode) {
		case MMC_POWER_UP:

			pr_debug("\tMMC_POWER_UP\n");
			owl_mmc_power_up(host);

			break;
		case MMC_POWER_ON:

			pr_debug("\tMMC_POWER_ON\n");
			owl_mmc_power_on(host);

			break;
		case MMC_POWER_OFF:

			pr_debug("\tMMC_POWER_OFF\n");
			/*
			   disable sd clk,so msut return
			   not rw sd reg
			 */

			owl_mmc_power_off(host);

			return;
		default:

			pr_debug("Power mode not supported\n");
		}
	}

	if (ios->clock && ios->clock != host->clock) {
		host->clock = ios->clock;

		owl_mmc_set_clk(host, ios->clock);
	}

	ctrl_reg = readl(HOST_EN(host));
	{
		host->bus_width = ios->bus_width;
		switch (ios->bus_width) {
		case MMC_BUS_WIDTH_8:

			ctrl_reg &= ~0x3;
			ctrl_reg |= 0x2;
			break;
		case MMC_BUS_WIDTH_4:

			ctrl_reg &= ~0x3;
			ctrl_reg |= 0x1;
			break;
		case MMC_BUS_WIDTH_1:

			ctrl_reg &= ~0x3;
			break;
		}
	}

	if (ios->chip_select != host->chip_select) {
		host->chip_select = ios->chip_select;
		switch (ios->chip_select) {
		case MMC_CS_DONTCARE:
			break;
		case MMC_CS_HIGH:
			ctrl_reg &= ~0x3;
			ctrl_reg |= 0x1;
			break;
		case MMC_CS_LOW:
			ctrl_reg &= ~0x3;
			break;
		}
	}

	if (ios->timing != host->timing) {
		host->timing = ios->timing;
		if (ios->timing == MMC_TIMING_UHS_DDR50) {
			ctrl_reg |= (1 << 2);
		}
	}

	switch (ctrl_reg & 0x3) {
	case 0x2:
		pr_debug("\tMMC_BUS_WIDTH_8\n");
		break;
	case 0x1:
		pr_debug("\tMMC_BUS_WIDTH_4\n");
		break;
	case 0x0:
		pr_debug("\tMMC_BUS_WIDTH_1\n");
		break;
	default:
		pr_debug("\tMMC_BUS_WIDTH NOT known\n");
	}

	writel(ctrl_reg, HOST_EN(host));

}

static void owl_mmc_gpio_check_status(unsigned long data)
{
	struct owl_mmc_host *host = (struct owl_mmc_host *)data;
	int card_present;

	if (host->card_detect_reverse)
		card_present = !!(gpio_get_value(host->detect_pin));
	else
		card_present = !(gpio_get_value(host->detect_pin));

	if (card_present ^ host->present) {
		/* debouncer */
		mdelay(20);

		if (host->card_detect_reverse)
			card_present = !!(gpio_get_value(host->detect_pin));
		else
			card_present = !(gpio_get_value(host->detect_pin));

		if (card_present ^ host->present) {
			pr_info("%s: Slot status change detected (%d -> %d)\n",
				mmc_hostname(host->mmc), host->present,
				card_present);

			mmc_detect_change(host->mmc, 0);
			host->present = card_present;
		}
	}
	mod_timer(&host->timer, jiffies + HZ / 5);
}

static void owl_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct owl_mmc_host *host = mmc_priv(mmc);
	unsigned long flags;
	u32 state;

	spin_lock_irqsave(&host->lock, flags);
	state = readl(HOST_STATE(host));
	if (enable) {
		state |= SD_STATE_SDIOA_EN;
		/* default SDIOA,  protect irq throw away */
		state &= ~SD_STATE_SDIOA_P;
		state &= ~SD_STATE_TEI;

	} else {
		state |= SD_STATE_SDIOA_P;
		state &= ~SD_STATE_SDIOA_EN;
		state &= ~SD_STATE_TEI;
	}
	writel(state, HOST_STATE(host));
	spin_unlock_irqrestore(&host->lock, flags);
}

static irqreturn_t owl_sdc_irq_handler(int irq, void *devid)
{
	struct owl_mmc_host *host = devid;
	struct mmc_host *mmc = host->mmc;
	unsigned long flags;
	u32 state;
	u32 temp;

	spin_lock_irqsave(&host->lock, flags);
	state = readl(HOST_STATE(host));
		/* check cmd irq */
	if (state & SD_STATE_TEI) {
		temp = readl(HOST_STATE(host));
		temp = temp & (~SD_STATE_SDIOA_P);
		temp |= SD_STATE_TEI;
		writel(temp, HOST_STATE(host));
		spin_unlock_irqrestore(&host->lock, flags);
		complete(&host->sdc_complete);
		goto tag;
	}

	spin_unlock_irqrestore(&host->lock, flags);
tag:

	/*check sdio date0 irq */
	if (mmc->caps & MMC_CAP_SDIO_IRQ) {
		if ((state & SD_STATE_SDIOA_P) && (state & SD_STATE_SDIOA_EN)) {
			mmc_signal_sdio_irq(host->mmc);
		}
	}

	return IRQ_HANDLED;

}


static void owl_mmc_finish_request(struct owl_mmc_host *host)
{
	struct mmc_request *mrq;
	struct mmc_data *data;

	WARN_ON(!host->mrq);

	mrq = host->mrq;
	host->mrq = NULL;

	if (mrq->data) {
		data = mrq->data;
		/* Finally finished */
		dma_unmap_sg(host->dma->device->dev, data->sg, data->sg_len,
			     host->dma_dir);
	}

	mmc_request_done(host->mmc, mrq);

}

/*
 * Since send_command can be called by data_complete,
 * so it should not "finish the request".
 * owl_mmc_send_command May sleep!
 */

static int owl_mmc_send_command(struct owl_mmc_host *host,
				struct mmc_command *cmd, struct mmc_data *data)
{

	u32 mode;
	u32 rsp[2];
	unsigned int cmd_rsp_mask = 0;
	u32 status;

	cmd->error = 0;
	init_completion(&host->sdc_complete);

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		mode = SD_CTL_TM(0);
		break;

	case MMC_RSP_R1:
		if (data) {
			if (data->flags & MMC_DATA_READ)
				mode = SD_CTL_TM(4);
			else
				mode = SD_CTL_TM(5);
		} else {
			mode = SD_CTL_TM(1);
		}
		cmd_rsp_mask = SD_STATE_CLNR | SD_STATE_CRC7ER;

		break;

	case MMC_RSP_R1B:
		mode = SD_CTL_TM(3);
		cmd_rsp_mask = SD_STATE_CLNR | SD_STATE_CRC7ER;
		break;

	case MMC_RSP_R2:
		mode = SD_CTL_TM(2);
		cmd_rsp_mask = SD_STATE_CLNR | SD_STATE_CRC7ER;
		break;

	case MMC_RSP_R3:
		mode = SD_CTL_TM(1);
		cmd_rsp_mask = SD_STATE_CLNR;
		break;

	default:
		pr_err("no math command RSP flag %x\n", cmd->flags);
		cmd->error = -1;
		return MMC_CMD_COMPLETE;
	}

	/* keep current RDELAY & WDELAY value */
	mode |= (readl(HOST_CTL(host)) & (0xff << 16));

	/* start to send corresponding command type */
	writel(cmd->arg, HOST_ARG(host));
	writel(cmd->opcode, HOST_CMD(host));
	pr_debug("host%d Transfer mode:0x%x\n\tArg:0x%x\n\tCmd:%u\n",
		 host->id, mode, cmd->arg, cmd->opcode);

	/*set lbe to send clk after busy */
	if (data) {
		if (mmc_card_expected_emmc(host->type_expected)) {
			pr_debug("mmcid:0x%x\n",host->mmc ->mmcid >> 24);
			if( (host->mmc ->mmcid >> 24) == FDI_EMMC_ID){
				mode |= (SD_CTL_TS | 0x64000000);
			}else{
				mode |= (SD_CTL_TS | SD_CTL_LBE | 0x64000000);
			}
		}else{
			/* not enable HW tiemout, use sw timeout */
			mode |= (SD_CTL_TS | SD_CTL_LBE | 0x64000000);
		}
	} else {
		/*pure cmd disable hw timeout and SD_CTL_LBE */
		mode &= ~(SD_CTL_TOUTEN | SD_CTL_LBE);
		mode |= SD_CTL_TS;
	}

	/* start transfer */
	writel(mode, HOST_CTL(host));

	pr_debug("SDC%d send CMD%d, SD_CTL=0x%x\n", host->id,
		 cmd->opcode, readl(HOST_CTL(host)));

	/* date cmd return */
	if (data) {
		return DATA_CMD;
	}

	/*
	 *wait for cmd finish
	 * Some bad card dose need more time to complete
	 * data transmission and programming.
	 */
	if (!wait_for_completion_timeout(&host->sdc_complete, 30 * HZ)) {
		pr_err("!!!host%d:cmd wait ts interrupt timeout\n", host->id);
		cmd->error = -ETIMEDOUT;
		owl_dump_reg(host);
		goto out;
	}

	status = readl(HOST_STATE(host));

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd_rsp_mask & status) {
			if (status & SD_STATE_CLNR) {
				cmd->error = -EILSEQ;
#if 0
				pr_err("SDC%d send CMD%d: CMD_NO_RSP...\n",
				       host->id, cmd->opcode);
#endif
				goto out;
			}

			if (status & SD_STATE_CRC7ER) {
				cmd->error = -EILSEQ;
				pr_err("SDC%d send CMD%d, CMD_RSP_CRC_ERR\n",
				       host->id, cmd->opcode);
				goto out;
			}

		}

		if (cmd->flags & MMC_RSP_136) {
			/*TODO: MSB first */
			cmd->resp[3] = readl(HOST_RSPBUF0(host));
			cmd->resp[2] = readl(HOST_RSPBUF1(host));
			cmd->resp[1] = readl(HOST_RSPBUF2(host));
			cmd->resp[0] = readl(HOST_RSPBUF3(host));
		} else {
			rsp[0] = readl(HOST_RSPBUF0(host));
			rsp[1] = readl(HOST_RSPBUF1(host));
			cmd->resp[0] = rsp[1] << 24 | rsp[0] >> 8;
			cmd->resp[1] = rsp[1] >> 8;
		}
	}

out:
	return PURE_CMD;

}

static void owl_mmc_dma_complete(struct owl_mmc_host *host)
{
	BUG_ON(!host->mrq->data);

	if (host->mrq->data) {
		complete(&host->dma_complete);
	}

}

static int owl_mmc_prepare_data(struct owl_mmc_host *host,
				struct mmc_data *data)
{
	struct scatterlist *sg;
	enum dma_transfer_direction slave_dirn;
	int i, sglen;
	unsigned total;

	writel(readl(HOST_EN(host)) | SD_EN_BSEL, HOST_EN(host));

	writel(data->blocks, HOST_BLK_NUM(host));
	writel(data->blksz, HOST_BLK_SIZE(host));

	total = data->blksz * data->blocks;

	if (total < 512) {
		writel(total, HOST_BUF_SIZE(host));
	} else {
		if (mmc_card_expected_wifi(host->type_expected)) {

			if (total % 512 == 0) {
				writel(512, HOST_BUF_SIZE(host));
			} else if (data->blocks % 2 == 0) {
				writel(data->blksz * 2, HOST_BUF_SIZE(host));
			} else if (data->blocks % 3 == 0) {
				writel(data->blksz * 3, HOST_BUF_SIZE(host));
			} else {
				writel(data->blksz, HOST_BUF_SIZE(host));
			}
		} else {
			writel(512, HOST_BUF_SIZE(host));
		}
	}

	/*
	 * We don't do DMA on "complex" transfers, i.e. with
	 * non-word-aligned buffers or lengths.
	 */
	for_each_sg(data->sg, sg, data->sg_len, i) {
		if (sg->offset & 3 || sg->length & 3)
			pr_err
			    ("SD tag: non-word-aligned buffers or lengths.\n");
	}

	if (data->flags & MMC_DATA_READ) {
		host->dma_dir = DMA_FROM_DEVICE;
		host->dma_conf.direction = slave_dirn = DMA_DEV_TO_MEM;
	} else if (data->flags & MMC_DATA_WRITE) {
		host->dma_dir = DMA_TO_DEVICE;
		host->dma_conf.direction = slave_dirn = DMA_MEM_TO_DEV;
	} else {
		BUG_ON(1);
	}

	sglen = dma_map_sg(host->dma->device->dev, data->sg,
			   data->sg_len, host->dma_dir);

	if (dmaengine_slave_config(host->dma, &host->dma_conf))
		pr_err("Failed to config DMA channel\n");

	host->desc = dmaengine_prep_slave_sg(host->dma,
					     data->sg, sglen, slave_dirn,
					     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!host->desc) {
		pr_err("dmaengine_prep_slave_sg() fail\n");
		return -EBUSY;
	}

	host->desc->callback = owl_mmc_dma_complete;
	host->desc->callback_param = host;
	/*
	 *init for adjust delay chain
	 */
	data->error = 0;

	return 0;
}

static int owl_mmc_card_exist(struct mmc_host *mmc)
{
	struct owl_mmc_host *host = mmc_priv(mmc);
	int present;

	if (mmc_card_expected_mem(host->type_expected)) {
		if (detect_use_gpio) {
			if (host->card_detect_reverse) {
				present = !!(gpio_get_value(host->detect_pin));
			} else {
				present = !(gpio_get_value(host->detect_pin));
			}

			return present;
		}
	}

	if (mmc_card_expected_wifi(host->type_expected))
		return host->sdio_present;

	return -ENOSYS;
}

static void owl_mmc_err_reset(struct owl_mmc_host *host)
{
	u32 reg_en, reg_ctr, reg_state;

	reg_en = readl(HOST_EN(host));
	reg_ctr = readl(HOST_CTL(host));
	reg_state = readl(HOST_STATE(host));

	owl_mmc_ctr_reset(host);

	writel(SD_ENABLE, HOST_EN(host));
	writel(reg_en, HOST_EN(host));
	reg_ctr &= ~SD_CTL_TS;
	writel(reg_ctr, HOST_CTL(host));
	writel(reg_state, HOST_STATE(host));
}

/*#define OWL_RETRY_DEBUG 1*/
static void owl_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct owl_mmc_host *host = mmc_priv(mmc);
	int ret = 0;

	/* check card is removed */
	if ((owl_mmc_card_exist(mmc) == 0) &&
	    (!(mmc->caps & MMC_CAP_NONREMOVABLE))) {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	host->mrq = mrq;

	if (mrq->data) {
		ret = owl_mmc_prepare_data(host, mrq->data);
		if (ret != 0) {
			pr_err("SD DMA transfer: prepare data error\n");
			mrq->data->error = ret;
			owl_mmc_finish_request(host);
			return;
		} else {
			init_completion(&host->dma_complete);
			dmaengine_submit(host->desc);
			dma_async_issue_pending(host->dma);
		}
	}

	ret = owl_mmc_send_command(host, mrq->cmd, mrq->data);

	if (ret == DATA_CMD) {

		if (!wait_for_completion_timeout(&host->sdc_complete, 10 * HZ)) {
			pr_err
			    ("!!!host%d:wait date transfer ts intrupt timeout\n",
			     host->id);
		}

		if (owl_check_trs_date_status(host, mrq)) {
			pr_err("!!!host%d err:owl_check_trs_date_status\n",
			       host->id);
			owl_dump_reg(host);
			pr_err("Entry SD/MMC module error reset\n");
			dmaengine_terminate_all(host->dma);
			owl_mmc_err_reset(host);
			pr_err("Exit SD/MMC module error reset\n");
			goto finish;
		}

		if (!wait_for_completion_timeout(&host->dma_complete, 5 * HZ)) {
			pr_err("!!!host%d:dma transfer completion timeout\n",
			       host->id);
			mrq->data->error = CMD_DATA_TIMEOUT;
			mrq->cmd->error = -ETIMEDOUT;
			owl_dump_reg(host);
			pr_err("Entry SD/MMC module error reset\n");
			dmaengine_terminate_all(host->dma);
			owl_mmc_err_reset(host);
			pr_err("Exit SD/MMC module error reset\n");
			goto finish;

		}

		if (mrq->data->stop) {
			/* send stop cmd */
			owl_mmc_send_command(host, mrq->data->stop, NULL);
			if (mrq->data->stop->error) {
				owl_dump_reg(host);
				pr_err("Entry SD/MMC module error reset\n");
				owl_mmc_err_reset(host);
				pr_err("Exit SD/MMC module error reset\n");
				goto finish;
			}
		}

		mrq->data->bytes_xfered = mrq->data->blocks * mrq->data->blksz;

	}

finish:
	owl_mmc_finish_request(host);
}

static void owl_dump_reg(struct owl_mmc_host *host)
{
	owl_dump_mfp(host);
	owl_dump_sdc(host);
	owl_dump_dmac(host);
}

/* check status reg  is ok */
static int owl_check_trs_date_status(struct owl_mmc_host *host,
				     struct mmc_request *mrq)
{

	struct mmc_data *data = mrq->data;
	struct mmc_command *cmd = mrq->cmd;
	u32 status = readl(HOST_STATE(host));
	u32 check_status = 0;
	u32 cmd_rsp_mask = 0;

	if (!host || !host->mrq || !host->mrq->data) {
		pr_err("SDC%d when DMA finish, request is NULL\n", host->id);
		BUG_ON(1);
		return -1;
	}

	cmd_rsp_mask = SD_STATE_TOUTE
	    | SD_STATE_CLNR
	    | SD_STATE_WC16ER | SD_STATE_RC16ER | SD_STATE_CRC7ER;

	check_status = status & cmd_rsp_mask;

	if (check_status) {
		if (check_status & SD_STATE_TOUTE) {
			pr_err("data:card HW  timeout error\n");
			data->error = HW_TIMEOUT;
			goto out;

		}
		if (check_status & SD_STATE_CLNR) {
			pr_err("data:card cmd line no respond error\n");
			data->error = CMD_RSP_ERR;
			goto out;
		}
		if (check_status & SD_STATE_WC16ER) {

			pr_err("data:card write:crc error\n");
			data->error = DATA_WR_CRC_ERR;
			cmd->error = -EILSEQ;
			goto out;
		}
		if (check_status & SD_STATE_RC16ER) {
			pr_err("data:card read:crc error\n");
			data->error = DATA_RD_CRC_ERR;
			cmd->error = -EILSEQ;
			goto out;
		}
		if (check_status & SD_STATE_CRC7ER) {
			pr_err("data: cmd  CMD_RSP_CRC_ERR\n");
			data->error = CMD_RSP_CRC_ERR;
			goto out;
		}
	}

out:
	if (data->error == DATA_RD_CRC_ERR) {
		host->read_delay_chain--;
		if (host->read_delay_chain < 0) {
			host->read_delay_chain = 0xf;
		}

		printk("try read delay chain:%d\n", host->read_delay_chain);

	} else if (data->error == DATA_WR_CRC_ERR) {

		host->write_delay_chain--;
		if (host->write_delay_chain < 0) {
			host->write_delay_chain = 0xf;
		}

		printk("try write delay chain:%d\n", host->write_delay_chain);
	}

	if (data->error == DATA_WR_CRC_ERR || data->error == DATA_RD_CRC_ERR) {
		writel((readl(HOST_CTL(host)) & (~(0xff << 16))) |
		       SD_CTL_RDELAY(host->read_delay_chain) |
		       SD_CTL_WDELAY(host->write_delay_chain), HOST_CTL(host));
	}

	return data->error;
}

static int owl_mmc_signal_voltage_switch(struct mmc_host *mmc,
					 struct mmc_ios *ios)
{
	struct owl_mmc_host *host = mmc_priv(mmc);
	u32 state;
	int i = 0;
	int ret = 0;
	void __iomem *sps_ld0_ctl;

	if (host->id == SDC0_SLOT) {

		sps_ld0_ctl = ioremap(SPS_LDO_CTL, 4);
		if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
			goto out;
		} else if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
			/* stop send clk until 5ms later */
			mdelay(10);
			/* switch host voltage to 1.8v and delay 10ms */
			printk("%s: start swith 3.3v -> 1.8V \n", __FUNCTION__);
			/* down vol */
			writel(readl(sps_ld0_ctl) | 0x400, sps_ld0_ctl);
			/* rise vol */
			writel(readl(PAD_CTL + host->mfp_base) | 0x100,
			       (PAD_CTL + host->mfp_base));
			printk("%s: end swith 3.3v -> 1.8V \n", __FUNCTION__);
			mdelay(5);

			/* send continuous clock */
			writel(readl(HOST_CTL(host)) | SD_CTL_SCC,
			       HOST_CTL(host));

			for (i = 0; i < 100; i++) {
				state = readl(HOST_STATE(host));
				if ((state & SD_STATE_CMDS)
				    && (state & SD_STATE_DAT0S))
					break;
				udelay(100);
			}
			if (i >= 100) {	/* max 10ms */
				pr_err
				    ("SDC%d level error, voltage switch failed\n",
				     host->id);
				ret = -EINVAL;
			}
			/* stop to send continuous clock */
			writel(readl(HOST_CTL(host)) & ~SD_CTL_SCC,
			       HOST_CTL(host));
		}

		if (sps_ld0_ctl)
			iounmap(sps_ld0_ctl);
	}

out:
	return ret;
}

static int owl_mmc_get_ro(struct mmc_host *mmc)
{
	struct owl_mmc_host *host = mmc_priv(mmc);
	int status = -ENOSYS;
	int read_only = -ENOSYS;

	if (gpio_is_valid(host->wpswitch_gpio)) {
		status = gpio_request(host->wpswitch_gpio, "wp_switch");
		if (status) {
			pr_err("%s: %s: Failed to request GPIO %d\n",
			       mmc_hostname(mmc), __func__,
			       host->wpswitch_gpio);
		} else {
			status = gpio_direction_input(host->wpswitch_gpio);
			if (!status) {
				/*
				 * Wait for atleast 300ms as debounce
				 * time for GPIO input to stabilize.
				 */
				msleep(300);

				/*
				 * SD card write protect switch on, high level.
				 */
				read_only =
				    gpio_get_value_cansleep
				    (host->wpswitch_gpio);

			}
			gpio_free(host->wpswitch_gpio);
		}
	}

	pr_info("%s: Card read-only status %d\n", __func__, read_only);

	return read_only;
}

static const struct mmc_host_ops owl_mmc_ops = {
	.get_cd = owl_mmc_card_exist,
	.request = owl_mmc_request,
	.set_ios = owl_mmc_set_ios,
	.enable_sdio_irq = owl_mmc_enable_sdio_irq,
	.start_signal_voltage_switch = owl_mmc_signal_voltage_switch,
	.get_ro = owl_mmc_get_ro,
	.switch_sd_pinctr = owl_switch_sd_pinctr,
	.switch_uart_pinctr = owl_switch_uartsd_pinctr,
};

static int owl_mmc_clkfreq_notify(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct clk_notifier_data *clk_nd = data;
	unsigned long old_rate, new_rate;
	struct owl_mmc_host *host;

	host = container_of(nb, struct owl_mmc_host, nblock);
	old_rate = clk_nd->old_rate;
	new_rate = clk_nd->new_rate;

	if (action == PRE_RATE_CHANGE) {
		/* pause host dma transfer */
	} else if (action == POST_RATE_CHANGE) {
		/* owl_mmc_set_clk(host, new_rate); */
		/* resume to start dma transfer */
	}

	return NOTIFY_OK;
}

static inline void owl_mmc_sdc_config(struct owl_mmc_host *host)
{
	if (host->start == SDC1_BASE) {
		host->id = SDC1_SLOT;
		host->pad_drv = SDC1_PAD_DRV;
		host->wdelay.delay_lowclk = SDC1_WDELAY_LOW_CLK;
		host->wdelay.delay_midclk = SDC1_WDELAY_MID_CLK;
		host->wdelay.delay_highclk = SDC1_WDELAY_HIGH_CLK;
		host->rdelay.delay_lowclk = SDC1_RDELAY_LOW_CLK;
		host->rdelay.delay_midclk = SDC1_RDELAY_MID_CLK;
		host->rdelay.delay_highclk = SDC1_RDELAY_HIGH_CLK;
	} else if (host->start == SDC2_BASE) {
		host->id = SDC2_SLOT;
		host->pad_drv = SDC2_PAD_DRV;
		host->wdelay.delay_lowclk = SDC2_WDELAY_LOW_CLK;
		host->wdelay.delay_midclk = SDC2_WDELAY_MID_CLK;
		host->wdelay.delay_highclk = SDC2_WDELAY_HIGH_CLK;
		host->rdelay.delay_lowclk = SDC2_RDELAY_LOW_CLK;
		host->rdelay.delay_midclk = SDC2_RDELAY_MID_CLK;
		host->rdelay.delay_highclk = SDC2_RDELAY_HIGH_CLK;
	} else if (host->start == SDC3_BASE) {
		host->id = SDC3_SLOT;
		host->pad_drv = SDC3_PAD_DRV;
		host->wdelay.delay_lowclk = SDC3_WDELAY_LOW_CLK;
		host->wdelay.delay_midclk = SDC3_WDELAY_MID_CLK;
		host->wdelay.delay_highclk = SDC3_WDELAY_HIGH_CLK;
		host->rdelay.delay_lowclk = SDC3_RDELAY_LOW_CLK;
		host->rdelay.delay_midclk = SDC3_RDELAY_MID_CLK;
		host->rdelay.delay_highclk = SDC3_RDELAY_HIGH_CLK;

	} else {
		host->id = SDC0_SLOT;
		host->pad_drv = SDC0_PAD_DRV;
		host->wdelay.delay_lowclk = SDC0_WDELAY_LOW_CLK;
		host->wdelay.delay_midclk = SDC0_WDELAY_MID_CLK;
		host->wdelay.delay_highclk = SDC0_WDELAY_HIGH_CLK;
		host->rdelay.delay_lowclk = SDC0_RDELAY_LOW_CLK;
		host->rdelay.delay_midclk = SDC0_RDELAY_MID_CLK;
		host->rdelay.delay_highclk = SDC0_RDELAY_HIGH_CLK;
	}
}

const int owl_of_get_card_detect_mode(struct device_node *np)
{
	const char *pm;
	int err, i;

	err = of_property_read_string(np, "card-detect-mode", &pm);
	if (err < 0)
		return err;
	for (i = 0; i < ARRAY_SIZE(card_detect_modes); i++)
		if (!strcasecmp(pm, card_detect_modes[i]))
			return i;
	pr_err("error: please chose card detect method\n");
	return -ENODEV;
}

const int owl_of_get_wifi_card_volate_mode(struct device_node *np)
{
	const char *pm;
	int err, i;

	err = of_property_read_string(np, "wifi-card-voltage", &pm);
	if (err < 0)
		return err;

	for (i = 0; i < ARRAY_SIZE(wifi_card_voltage); i++)
		if (!strcasecmp(pm, wifi_card_voltage[i]))
			return i;

	return -ENODEV;
}

const int owl_of_get_card_type(struct device_node *np)
{
	const char *pm;
	int err, i;

	err = of_property_read_string(np, "card-type", &pm);
	if (err < 0)
		return err;
	for (i = 0; i < ARRAY_SIZE(card_types); i++)
		if (!strcasecmp(pm, card_types[i]))
			return i;
	pr_err("error: please make sure card type is exist\n");
	return -ENODEV;
}

int owl_mmc_get_power(struct owl_mmc_host *host, struct device_node *np)
{
	const char *pm;
	int err;

	if (of_find_property(np, "sd_vcc", NULL)) {
		err = of_property_read_string(np, "sd_vcc", &pm);
		if (err < 0) {
			pr_err("SDC[%u] can not read SD_VCC power source\n",
			       host->id);
			return -1;
		}

		host->reg = regulator_get(NULL, pm);
		if (IS_ERR(host->reg)) {
			pr_err("SDC[%u] failed to get regulator %s\n",
			       host->id, "sd_vcc");
			return -1;
		}

		err = owl_mmc_opt_regulator(host, REG_ENABLE);
		if (err) {
			printk("%s:%d\n", __FUNCTION__, __LINE__);
			WARN_ON(1);
		}

	}

	return 0;
}

/*
static int owl_upgrade_flag = OWL_NORMAL_BOOT;
static int __init owl_check_upgrade(char *__unused)
{
	owl_upgrade_flag = OWL_UPGRADE;
	printk("%s:owl_upgrade_flag is OWL_UPGRADE\n", __FUNCTION__);
	return 0 ;
}

__setup("OWL_PRODUCE", owl_check_upgrade);

static int owl_mmc_resan(struct owl_mmc_host * host)
{
	int bootdev;

	bootdev = owl_get_boot_dev();
	if(mmc_card_expected_mem(host->type_expected)||
		mmc_card_expected_wifi(host->type_expected)	){
		printk("host%d:sure rescan mmc\n",host->id);
		mmc_add_host(host->mmc);
	}else if(mmc_card_expected_emmc(host->type_expected)){
		if((owl_upgrade_flag == OWL_UPGRADE)||((bootdev !=OWL_BOOTDEV_NAND)&&\
			(bootdev !=OWL_BOOTDEV_SD02NAND))){
			mmc_add_host(host->mmc);
			printk("host%d:owl_mmc_resan\n",host->id);
		}else{
			printk("host%d:there is no need to resan emmc\n",host->id);
		}
	}else{
		pr_err("host%d : error type:%d :%s \n",host->id,\
				host->type_expected, __FUNCTION__);
		return -1;
	}
	return 0;
}
*/

static int owl_alloc_nand_resource(struct owl_mmc_host *host,
				   struct platform_device *pdev)
{
	struct resource *res = NULL;

	host->nandclk = devm_clk_get(&pdev->dev, "nand");
	if (IS_ERR(host->nandclk)) {
		return IS_ERR(host->nandclk);

	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		return -ENODEV;
	}

	host->nand_io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->nand_io_base)) {
		pr_err("err:%s:%d\n", __FUNCTION__, __LINE__);
		return PTR_ERR(host->nand_io_base);
	}

	pm_runtime_enable(host->mmc->parent);

	host->nand_powergate = POWER_GATE_OFF;

	return 0;

}

static int owl_alloc_sys_resource(struct owl_mmc_host *host,
				  struct platform_device *pdev)
{
	struct resource *res = NULL;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		pr_err("err:%s:%d\n", __FUNCTION__, __LINE__);
		return -ENODEV;
	}

	ioremap(res->start, (res->end - res->start));
	host->mfp_base = ioremap(res->start, res->end - res->start);

	if (!host->mfp_base) {
		pr_err("err:%s:%d\n", __FUNCTION__, __LINE__);
		return PTR_ERR(host->mfp_base);
	}
	pr_debug("id:%d start:0x%llx end:0x%llx map_mfp:0x%llx\n",
		 host->id, res->start, res->end, host->mfp_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (!res) {
		pr_err("err:%s:%d\n", __FUNCTION__, __LINE__);
		return -ENODEV;
	}
	host->cmu_base = ioremap(res->start, res->end - res->start);

	if (!host->cmu_base) {
		pr_err("err:%s:%d\n", __FUNCTION__, __LINE__);
		return PTR_ERR(host->cmu_base);
	}
	pr_debug("id:%d start:0x%llx end:0x%llx map_cmu:0x%llx\n",
		 host->id, res->start, res->end, host->cmu_base);
	return 0;

}

static int __init owl_mmc_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct owl_mmc_host *host;
	struct resource *res;
	struct device_node *dn = pdev->dev.of_node;
	struct wlan_plat_data *pdata;
	int ret = 0;

	mmc = mmc_alloc_host(sizeof(struct owl_mmc_host), &pdev->dev);
	if (!mmc) {
		dev_err(&pdev->dev, "require memory for mmc_host failed\n");
		ret = -ENOMEM;
		goto out;
	}

	host = mmc_priv(mmc);
	spin_lock_init(&host->lock);
	mutex_init(&host->pin_mutex);
	host->mmc = mmc;
	host->power_state = host->bus_width = host->chip_select = -1;
	host->clock = 0;
	host->mrq = NULL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no memory resource\n");
		ret = -ENODEV;
		goto out_free_host;
	}

	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "Unable to request register region\n");
		return -EBUSY;
	}

	host->iobase = ioremap(res->start, resource_size(res));
	if (host->iobase == NULL) {
		dev_err(&pdev->dev, "Unable to ioremap register region\n");
		return -ENXIO;
	}

	host->start = res->start;

	host->type_expected = owl_of_get_card_type(dn);
	if (host->type_expected < 0)
		goto out_free_host;

	owl_mmc_sdc_config(host);

	/*config pinctr name,because mmc not want
	   to use platform default pinctr */
	sprintf((char *)(host->pinctrname), "pinctrl_mmc%d", host->id);

	host->clk = devm_clk_get(&pdev->dev, "mmc");
	if (IS_ERR(host->clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		ret = PTR_ERR(host->clk);
		goto out_free_host;
	}

	host->dev_clk = devm_clk_get(&pdev->dev, "dev_clk");
	if (IS_ERR(host->dev_clk)) {
		dev_err(&pdev->dev, "dev_clk no clock defined\n");
		ret = PTR_ERR(host->dev_clk);
		goto out_free_host;
	}
	host->nandpll_clk = devm_clk_get(&pdev->dev, "nand_pll");
	if (IS_ERR(host->nandpll_clk)) {
		dev_err(&pdev->dev, "nandpll_clk no clock defined\n");
		ret = PTR_ERR(host->nandpll_clk);
		goto out_free_host;
	}

	clk_set_parent(host->clk, host->dev_clk);
	clk_set_parent(host->clk, host->nandpll_clk);
	clk_set_parent(host->clk, host->dev_clk);

	host->rst = devm_reset_control_get(&pdev->dev, "mmc");
	if (IS_ERR(host->rst)) {
		dev_err(&pdev->dev, "Couldn't get the reset\n");
		return PTR_ERR(host->rst);
	}

	memset(&host->nblock, 0, sizeof(host->nblock));
	host->nblock.notifier_call = owl_mmc_clkfreq_notify;
	/* clk_notifier_register(host->clk, &host->nblock); */

	host->regulator_status = OWL_REGULATOR_OFF;
	ret = owl_mmc_get_power(host, dn);
	if (ret < 0)
		goto out_put_clk;

	if (of_find_property(dn, "sdio_uart_supported", NULL)) {
		host->sdio_uart_supported = 1;
		pr_info("SDC%d use sdio uart conversion\n", host->id);
	}

	if (of_find_property(dn, "card_detect_reverse", NULL)) {
		host->card_detect_reverse = 1;
		host->present = 0;
		pr_info("SDC%d detect sd card use reverse power-level\n",
			host->id);
	} else {
		host->card_detect_reverse = 0;
		host->present = 0;
	}

	/* MT5931 SDIO WiFi need to send continuous clock */
	if (mmc_card_expected_wifi(host->type_expected)) {
		if (of_find_property(dn, "send_continuous_clock", NULL)) {
			host->send_continuous_clock = 1;
			pr_info("SDC%d wifi send continuous clock\n", host->id);
		}
	}

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	/* Request DMA channel */

	host->dma = dma_request_slave_channel(&pdev->dev, "mmc");

	if (!host->dma) {

		dev_err(&pdev->dev, "Failed to request DMA channel\n");
		ret = -1;
		goto out_free_sdc_irq;

	}

	dev_info(&pdev->dev, "using %s for DMA transfers\n",
		 dma_chan_name(host->dma));

	host->dma_conf.src_addr = HOST_DAT_DMA(host);
	host->dma_conf.dst_addr = HOST_DAT_DMA(host);
	host->dma_conf.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	host->dma_conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	host->dma_conf.device_fc = false;

	mmc->ops = &owl_mmc_ops;

	mmc->f_min = 100000;
	mmc->f_max = 52000000;

	mmc->max_seg_size = 256 * 512 * 2;
	mmc->max_segs = 128 * 2;
	mmc->max_req_size = 512 * 256 * 2;
	mmc->max_blk_size = 512;
	mmc->max_blk_count = 256 * 2;

	mmc->ocr_avail = ACTS_MMC_OCR;
	mmc->caps = MMC_CAP_NEEDS_POLL | MMC_CAP_MMC_HIGHSPEED |
	    MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA;

	mmc->caps2 = (MMC_CAP2_BOOTPART_NOACC | MMC_CAP2_DETECT_ON_ERR);

	if (of_find_property(dn, "one_bit_width", NULL))

		mmc->caps &= ~MMC_CAP_4_BIT_DATA;

	if (mmc_card_expected_mem(host->type_expected)) {

		mmc->f_max = 50000000;
		mmc->caps |= MMC_CAP_ERASE;
	}

	/*sdio 3.0 support */
	if (mmc_card_expected_wifi(host->type_expected)) {
		mmc->f_max = 100000000;
		mmc->caps |= (MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25
			      | MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_DDR50);
	}
	if (mmc_card_expected_emmc(host->type_expected)) {
		mmc->caps |=
		    MMC_CAP_1_8V_DDR | MMC_CAP_UHS_DDR50 | MMC_CAP_8_BIT_DATA;
		/*emmc and sd card support earse (discard,trim,sediscard) */
		mmc->caps |= MMC_CAP_ERASE;
	}

	if (owl_alloc_sys_resource(host, pdev)) {
		dev_err(&pdev->dev, "can not owl_alloc_sys_resource\n");
		ret = -ENODEV;
		goto out_free_dma;
	}

	if (SDC2_SLOT == host->id || SDC3_SLOT == host->id) {
		if (owl_alloc_nand_resource(host, pdev)) {
			dev_err(&pdev->dev,
				"can not getowl_alloc_nand_resource\n");
			ret = -ENODEV;
			goto out_free_dma;
		}

	}

	if (mmc_card_expected_mem(host->type_expected)) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
		if (!res) {
			dev_err(&pdev->dev,
				"can not get sdc transfer end irq resource\n");
			ret = -ENODEV;
			goto out_free_dma;
		}
		host->sdc_irq = res->start;
		ret = request_irq(host->sdc_irq,
				  (irq_handler_t) owl_sdc_irq_handler, 0,
				  "sdcard", host);

		if (ret < 0) {
			dev_err(&pdev->dev,
				"request SDC transfer end interrupt failed\n");
			goto out_free_dma;
		}

		host->card_detect_mode = owl_of_get_card_detect_mode(dn);

		if (host->card_detect_mode == GPIO_DETECT_CARD) {
			pr_info("use GPIO to detect SD/MMC card\n");
			detect_use_gpio = 1;
			mmc->caps &= ~MMC_CAP_NEEDS_POLL;

			/* card write protecte switch gpio */
			host->wpswitch_gpio = -ENOENT;

			if (of_find_property(dn, "wp_switch_gpio", NULL)) {
				host->wpswitch_gpio = of_get_named_gpio(dn,
									"wp_switch_gpio",
									0);
			}

			/* card detect gpio */
			host->detect_pin = of_get_named_gpio(dn, "cd-gpios", 0);
			if (gpio_is_valid(host->detect_pin)) {
				ret = gpio_request(host->detect_pin,
						   "card_detect_gpio");
				if (ret < 0) {
					dev_err(&pdev->dev,
						"couldn't claim card detect gpio pin\n");
					goto out_free_sdc_irq;
				}
				gpio_direction_input(host->detect_pin);

			} else {
				dev_err(&pdev->dev, "card detect poll mode\n");
				goto out_free_sdc_irq;
			}

			host->present = 0;

			init_timer(&host->timer);
			host->timer.data = (unsigned long)host;
			host->timer.function = owl_mmc_gpio_check_status;
			host->timer.expires = jiffies + 2 * HZ;
			add_timer(&host->timer);
		} else if (host->card_detect_mode == COMMAND_DETECT_CARD) {

			pr_info("use COMMAND to detect SD/MMC card\n");
		} else {
			pr_err("please choose card detect method\n");
		}
	} else if (mmc_card_expected_wifi(host->type_expected)) {

		host->wifi_detect_voltage =
		    owl_of_get_wifi_card_volate_mode(dn);

		if (DETECT_CARD_LOW_VOLTAGE == host->wifi_detect_voltage) {
			printk("wifi card use 1.8v to scan\n");
		} else {
			printk("wifi card use 3.3v to scan\n");
		}

		mmc->caps &= ~MMC_CAP_NEEDS_POLL;
		res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
		if (!res) {
			dev_err(&pdev->dev, "no SDIO irq resource\n");
			ret = -ENODEV;
			goto out_free_dma;
		}

		mmc->caps |= MMC_CAP_SDIO_IRQ;
		host->sdio_irq = res->start;
		ret = request_irq(host->sdio_irq,
				  (irq_handler_t) owl_sdc_irq_handler, 0,
				  "sdio", host);
		if (ret < 0) {
			dev_err(&pdev->dev, "request SDIO interrupt failed\n");
			goto out_free_dma;
		}

		/* dummy device for power control */
		owl_wlan_status_check_register(host);

		pdata = owl_wlan_device.dev.platform_data;
		pdata->parent = pdev;
		ret = platform_device_register(&owl_wlan_device);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"failed to register dummy wifi device\n");
			goto out_free_sdio_irq;
		}

		/* (wifi & bt) power control init */
		owl_wlan_bt_power_init(pdata);

	} else if (mmc_card_expected_emmc(host->type_expected)) {
		mmc->caps &= ~MMC_CAP_NEEDS_POLL;
		mmc->caps |= MMC_CAP_NONREMOVABLE;
		res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
		if (!res) {
			dev_err(&pdev->dev,
				"can not get sdc transfer end irq resource\n");
			ret = -ENODEV;
			goto out_free_dma;
		}
		host->sdc_irq = res->start;
		ret = request_irq(host->sdc_irq,
				  (irq_handler_t) owl_sdc_irq_handler, 0,
				  "emmc", host);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"request eMMC SDC transfer end interrupt failed\n");
			goto out_free_dma;
		}
	} else {
		dev_err(&pdev->dev, "SDC%d not supported %d\n",
			host->id, host->type_expected);
		ret = -ENXIO;
		goto out_free_dma;
	}

	mmc_add_host(host->mmc);
	platform_set_drvdata(pdev, host);

	return 0;

out_free_sdio_irq:
	/* SDIO WiFi card */
	if (mmc_card_expected_wifi(host->type_expected))
		free_irq(host->sdio_irq, host);

	/* memory card */
	if (mmc_card_expected_mem(host->type_expected)) {
		if (gpio_is_valid(host->detect_pin)) {
			del_timer_sync(&host->timer);
			gpio_free(host->detect_pin);
		}

		if (host->detect_irq_registered)
			free_irq(host->detect, host);
	}

out_free_sdc_irq:
	if (mmc_card_expected_mem(host->type_expected))
		free_irq(host->sdc_irq, host);

	if (mmc_card_expected_emmc(host->type_expected))
		free_irq(host->sdc_irq, host);

out_free_dma:
	if (host->dma)
		dma_release_channel(host->dma);

	if (host->reg) {
		regulator_disable(host->reg);
		regulator_put(host->reg);
	}

out_put_clk:
	if (host->clk) {
		/* clk_notifier_unregister(host->clk, &host->nblock); */
		clk_put(host->clk);
	}

out_free_host:
	mmc_free_host(mmc);

out:
	return ret;
}

static int owl_mmc_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct owl_mmc_host *host = platform_get_drvdata(pdev);

	if (host) {
		mmc_remove_host(host->mmc);

		if (mmc_card_expected_wifi(host->type_expected)) {
			owl_wlan_bt_power_release();
			platform_device_unregister(&owl_wlan_device);
			free_irq(host->sdio_irq, host);
		}

		if (mmc_card_expected_mem(host->type_expected)) {
			if (gpio_is_valid(host->detect_pin)) {
				del_timer_sync(&host->timer);
				gpio_free(host->detect_pin);
			}
			if (host->detect_irq_registered) {
				free_irq(host->detect, host);
				host->detect_irq_registered = 0;
			}
			free_irq(host->sdc_irq, host);
		}

		if (mmc_card_expected_emmc(host->type_expected))
			free_irq(host->sdc_irq, host);

		if (host->dma)
			dma_release_channel(host->dma);

		/* when stop host, power is off */
		ret = owl_mmc_opt_regulator(host, REG_DISENABLE);
		if (ret) {
			pr_err("%s:%d\n", __FUNCTION__, __LINE__);
			WARN_ON(1);
		}

		if (host->clk) {
			/* clk_notifier_unregister(host->clk, &host->nblock); */
			clk_put(host->clk);
		}

		mmc_free_host(host->mmc);
		platform_set_drvdata(pdev, NULL);
	}

	return 0;
}

static void owl_suspend_wait_data_finish(struct owl_mmc_host *host, int timeout)
{

	while ((readl(HOST_CTL(host)) & SD_CTL_TS) && (--timeout)) {
		udelay(1);
	}

	if (timeout <= 0) {
		pr_err("SDC%d mmc suspend wait card finish data timeout\n",
		       host->id);
	} else {
		printk("SDC%d mmc card finish data then enter suspend\n",
		       host->id);
	}

}

#ifdef CONFIG_PM
static int owl_mmc_suspend(struct device *dev)
{

	int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct owl_mmc_host *host = platform_get_drvdata(pdev);
	struct mmc_host *mmc = host->mmc;

	printk("SD%d host controller driver Enter suspend\n", host->id);

	cancel_delayed_work(&host->dma_work);
	if (host->dma_wq) {
		flush_workqueue(host->dma_wq);
	}

	if (host->clk_on) {
		owl_suspend_wait_data_finish(host, 2000000);
	}

	ret = mmc_suspend_host(host->mmc);
	if (mmc && (mmc->card) && (mmc->card->type == MMC_TYPE_SDIO))
		owl_wlan_set_power(owl_get_wlan_plat_data(), 0, 0);

	ret = owl_mmc_opt_regulator(host, REG_DISENABLE);
	if (ret) {
		pr_err("%s:%d\n", __FUNCTION__, __LINE__);
		WARN_ON(1);
	}

	return ret;
}

static int owl_mmc_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct owl_mmc_host *host = platform_get_drvdata(pdev);
	struct mmc_host *mmc = host->mmc;
	int ret = 0;

	printk("SD%d host controller driver Enter resume \n", host->id);

	ret = owl_mmc_opt_regulator(host, REG_ENABLE);
	if (ret) {
		pr_err("%s:%d\n", __FUNCTION__, __LINE__);
		WARN_ON(1);
	}

	if ((SDC2_SLOT == host->id || SDC3_SLOT == host->id) &&
	    mmc && (mmc->card)) {
		owl_enable_nand_clock(host);
	}

	if (mmc && (mmc->card) && (mmc->card->type == MMC_TYPE_SDIO)) {
		owl_wlan_set_power(owl_get_wlan_plat_data(), 1, 0);
	}

	mmc_resume_host(host->mmc);

	return 0;
}
#else
#define owl_mmc_suspend NULL
#define owl_mmc_resume NULL
#endif

static const struct of_device_id owl_mmc_dt_match[] = {
	{.compatible = "actions,s900-mmc",},
	{}
};

static struct dev_pm_ops mmc_pm = {
	.suspend = owl_mmc_suspend,
	.resume = owl_mmc_resume,
};

static struct platform_driver owl_mmc_driver = {
	.probe = owl_mmc_probe,
	.remove = owl_mmc_remove,
	.driver = {
		   .name = "s900-mmc",
		   .owner = THIS_MODULE,
		   .of_match_table = owl_mmc_dt_match,
		   .pm = &mmc_pm,
		   },

};

static int __init owl_mmc_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&owl_mmc_driver);
	if (ret) {
		pr_err("SD/MMC controller driver register failed\n");
		ret = -ENOMEM;
	}

	return ret;
}

static void owl_mmc_exit(void)
{
	platform_driver_unregister(&owl_mmc_driver);

}

module_init(owl_mmc_init);
module_exit(owl_mmc_exit);

MODULE_AUTHOR("Actions");
MODULE_DESCRIPTION("MMC/SD host controller driver");
MODULE_LICENSE("GPL");
