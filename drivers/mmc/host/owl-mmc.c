#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dma-direction.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

// REMOVE
#include <linux/clk-provider.h>

/*
 * SDC registers
 */
#define OWL_REG_SDx_EN			0x0000
#define OWL_REG_SDx_CTL			0x0004
#define OWL_REG_SDx_STATE		0x0008
#define OWL_REG_SDx_CMD			0x000c
#define OWL_REG_SDx_ARG			0x0010
#define OWL_REG_SDx_RSPBUF0		0x0014
#define OWL_REG_SDx_RSPBUF1		0x0018
#define OWL_REG_SDx_RSPBUF2		0x001c
#define OWL_REG_SDx_RSPBUF3		0x0020
#define OWL_REG_SDx_RSPBUF4		0x0024
#define OWL_REG_SDx_DAT			0x0028
#define OWL_REG_SDx_BLK_SIZE		0x002c
#define OWL_REG_SDx_BLK_NUM		0x0030
#define OWL_REG_SDx_BUF_SIZE		0x0034

/* SDx_EN Bits */
#define OWL_SD_EN_RANE			BIT(31)
#define OWL_SD_EN_RAN_SEED(x)		(((x) & 0x3f) << 24)
#define OWL_SD_EN_S18EN			BIT(12)
#define OWL_SD_EN_RESE			BIT(10)
#define OWL_SD_EN_DAT1_S		BIT(9)
#define OWL_SD_EN_CLK_S			BIT(8)
#define OWL_SD_ENABLE			BIT(7)
#define OWL_SD_EN_BSEL			BIT(6)
#define OWL_SD_EN_SDIOEN		BIT(3)
#define OWL_SD_EN_DDREN			BIT(2)
#define OWL_SD_EN_DATAWID(x)		(((x) & 0x3) << 0)

/* SDx_CTL Bits */
#define OWL_SD_CTL_TOUTEN		BIT(31)
#define OWL_SD_CTL_TOUTCNT(x)		(((x) & 0x7f) << 24)
#define OWL_SD_CTL_DELAY_MSK		GENMASK(23, 16)
#define OWL_SD_CTL_RDELAY(x)		(((x) & 0xf) << 20)
#define OWL_SD_CTL_WDELAY(x)		(((x) & 0xf) << 16)
#define OWL_SD_CTL_CMDLEN		BIT(13)
#define OWL_SD_CTL_SCC			BIT(12)
#define OWL_SD_CTL_TCN(x)		(((x) & 0xf) << 8)
#define OWL_SD_CTL_TS			BIT(7)
#define OWL_SD_CTL_LBE			BIT(6)
#define OWL_SD_CTL_C7EN			BIT(5)
#define OWL_SD_CTL_TM(x)		(((x) & 0xf) << 0)

#define OWL_SD_DELAY_LOW_CLK		0x0f
#define OWL_SD_DELAY_MID_CLK		0x0a
#define OWL_SD_DELAY_HIGH_CLK		0x09
#define OWL_SD_RDELAY_DDR50		0x0a
#define OWL_SD_WDELAY_DDR50		0x08

/* SDx_STATE Bits */
#define OWL_SD_STATE_DAT1BS		BIT(18)
#define OWL_SD_STATE_SDIOB_P		BIT(17)
#define OWL_SD_STATE_SDIOB_EN		BIT(16)
#define OWL_SD_STATE_TOUTE		BIT(15)
#define OWL_SD_STATE_BAEP		BIT(14)
#define OWL_SD_STATE_MEMRDY		BIT(12)
#define OWL_SD_STATE_CMDS		BIT(11)
#define OWL_SD_STATE_DAT1AS		BIT(10)
#define OWL_SD_STATE_SDIOA_P		BIT(9)
#define OWL_SD_STATE_SDIOA_EN		BIT(8)
#define OWL_SD_STATE_DAT0S		BIT(7)
#define OWL_SD_STATE_TEIE		BIT(6)
#define OWL_SD_STATE_TEI		BIT(5)
#define OWL_SD_STATE_CLNR		BIT(4)
#define OWL_SD_STATE_CLC		BIT(3)
#define OWL_SD_STATE_WC16ER		BIT(2)
#define OWL_SD_STATE_RC16ER		BIT(1)
#define OWL_SD_STATE_CRC7ER		BIT(0)

struct owl_mmc_host {
	struct device *dev;
	struct reset_control *reset;
	void __iomem	*base;
	struct clk	*clk;
	spinlock_t	lock;
	int		irq;
	u32		int_sum;
	u32 		clock;
	struct completion sdc_complete;

        enum dma_data_direction dma_dir;
        struct dma_chan *dma;
        struct dma_async_tx_descriptor *desc;
        struct dma_slave_config dma_cfg;
        struct completion dma_complete;

	struct mmc_host			*mmc;
	struct mmc_request		*mrq;
	struct mmc_command		*cmd;
	struct mmc_data			*data;
};

static void owl_mmc_send_cmd(struct owl_mmc_host *owl_host,
			     struct mmc_command *cmd,
			     struct mmc_data *data);

static void mmc_writel(struct owl_mmc_host *owl_host, u32 reg, u32 data)
{
	writel(data, owl_host->base + reg);
}

static u32 mmc_readl(struct owl_mmc_host *owl_host, u32 reg)
{
	return readl(owl_host->base + reg);
}

static void mmc_update_reg(void __iomem *reg, unsigned int val, bool state)
{
	unsigned int regval;

	regval = readl(reg);

	if (state)
		regval |= val;
	else
		regval &= ~val;

	writel(regval, reg);
}

static irqreturn_t owl_irq_handler(int irq, void *devid)
{
        struct owl_mmc_host *owl_host = devid;
        struct mmc_command *cmd = owl_host->cmd;
	unsigned long flags;
	u32 resp[2], state;

	pr_info("*************IRQ for Transfer: Arg:0x%x\n\tCmd:%u\n",
		 cmd->arg, cmd->opcode);
	spin_lock_irqsave(&owl_host->lock, flags);
	
	state = mmc_readl(owl_host, OWL_REG_SDx_STATE);
	if (state & OWL_SD_STATE_TEI) {
		pr_info("%s: Transfer End IRQ\n", __func__);
		state = mmc_readl(owl_host, OWL_REG_SDx_STATE);
		state |= OWL_SD_STATE_TEI;
		mmc_writel(owl_host, OWL_REG_SDx_STATE, state);

		if (mmc_resp_type(cmd) & MMC_RSP_PRESENT) {
			if (mmc_resp_type(cmd) & MMC_RSP_136) {
				cmd->resp[3] = mmc_readl(owl_host, OWL_REG_SDx_RSPBUF0);
				cmd->resp[2] = mmc_readl(owl_host, OWL_REG_SDx_RSPBUF1);
				cmd->resp[1] = mmc_readl(owl_host, OWL_REG_SDx_RSPBUF2);
				cmd->resp[0] = mmc_readl(owl_host, OWL_REG_SDx_RSPBUF3);
			} else {
				resp[0] = mmc_readl(owl_host, OWL_REG_SDx_RSPBUF0);
				resp[1] = mmc_readl(owl_host, OWL_REG_SDx_RSPBUF1);
				cmd->resp[0] = resp[1] << 24 | resp[0] >> 8;
				cmd->resp[1] = resp[1] >> 8;
			}
		}

		mmc_request_done(owl_host->mmc, owl_host->mrq);
		owl_host->mrq = NULL;
	}

	spin_unlock_irqrestore(&owl_host->lock, flags);

	return IRQ_HANDLED;
}

static void owl_mmc_send_cmd(struct owl_mmc_host *owl_host,
			     struct mmc_command *cmd,
			     struct mmc_data *data)
{
	u32 mode;

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		pr_info("%s: MMC_RSP_NONE\n", __func__);
		mode = OWL_SD_CTL_TM(0);
		break;

	case MMC_RSP_R1:
		pr_info("%s: MMC_RSP_R1\n", __func__);
		if (data) {
			if (data->flags & MMC_DATA_READ)
				mode = OWL_SD_CTL_TM(4);
			else
				mode = OWL_SD_CTL_TM(5);
		} else {
			mode = OWL_SD_CTL_TM(1);
		}

		break;

	case MMC_RSP_R1B:
		pr_info("%s: MMC_RSP_R1B\n", __func__);
		mode = OWL_SD_CTL_TM(3);
		break;

	case MMC_RSP_R2:
		pr_info("%s: MMC_RSP_R2\n", __func__);
		mode = OWL_SD_CTL_TM(2);
		break;

	case MMC_RSP_R3:
		pr_info("%s: MMC_RSP_R3\n", __func__);
		mode = OWL_SD_CTL_TM(1);
		break;

	default:
		pr_info("%s: Unknown MMC Command\n", __func__);
		dev_warn(mmc_dev(owl_host->mmc),
			 "Unknown MMC command\n");
		return;
	}

	/* Keep current WDELAY and RDELAY */
	mode |= (mmc_readl(owl_host, OWL_REG_SDx_CTL) & (0xff << 16));

	/* Start to send corresponding command type */
	mmc_writel(owl_host, OWL_REG_SDx_ARG, cmd->arg);
	mmc_writel(owl_host, OWL_REG_SDx_CMD, cmd->opcode);

	/* Set LBE to send clk at the end of last read block */
	if (data) {
		mode &= ~OWL_SD_CTL_TOUTEN;
		mode |= OWL_SD_CTL_TS | OWL_SD_CTL_LBE;
	} else {
		pr_info("%s: %d\n", __func__,__LINE__);
		mode &= ~(OWL_SD_CTL_TOUTEN | OWL_SD_CTL_LBE);
		mode |= OWL_SD_CTL_TS;
	}

	owl_host->cmd = cmd;

	pr_info("Host Transfer mode:0x%x\n\tArg:0x%x\n\tCmd:%u\n",
		 mode, cmd->arg, cmd->opcode);

	/* Start transfer */
	mmc_writel(owl_host, OWL_REG_SDx_CTL, mode);
	mdelay(30);
	pr_info("SDC: Sent CMD: %d\n", cmd->opcode);
}

static void owl_mmc_dma_complete(void *param)
{
	struct owl_mmc_host *owl_host = param;
	struct mmc_command *cmd = owl_host->cmd;
	struct mmc_data *data = owl_host->data;
	struct mmc_request *mrq = owl_host->mrq;

	pr_info("%s: %d\n", __func__,__LINE__);
	if (data) {
		dma_unmap_sg(mmc_dev(owl_host->mmc), data->sg,
		     data->sg_len, owl_host->dma_dir);
		/*
		 * If there was an error on any block, we mark all
	 	* data blocks as being in error.
	 	*/
		if (!data->error)
			data->bytes_xfered = data->blocks * data->blksz;
		else
			data->bytes_xfered = 0;

		owl_host->data = NULL;
		if (mrq->data->stop) {
			owl_mmc_send_cmd(owl_host, mrq->data->stop, NULL);
			return;
		}
	}

	owl_host->mrq = NULL;
	mmc_request_done(owl_host->mmc, mrq);
}

static void owl_mmc_start_dma(struct owl_mmc_host *owl_host,
			      struct mmc_data *data)
{
	int sglen;

	pr_info("%s: %d\n", __func__,__LINE__);
	if (data->flags & MMC_DATA_WRITE) {
		owl_host->dma_dir = DMA_TO_DEVICE;
		owl_host->dma_cfg.direction = DMA_MEM_TO_DEV;
	} else {
		owl_host->dma_dir = DMA_FROM_DEVICE;
		owl_host->dma_cfg.direction = DMA_DEV_TO_MEM;
	}

	sglen = dma_map_sg(owl_host->dma->device->dev, data->sg,
			   data->sg_len, owl_host->dma_dir);

	dmaengine_slave_config(owl_host->dma, &owl_host->dma_cfg);
	owl_host->desc = dmaengine_prep_slave_sg(owl_host->dma, data->sg,
				       		 sglen,
						 owl_host->dma_cfg.direction,
				       		 DMA_PREP_INTERRUPT |
						 DMA_CTRL_ACK);
	if (!owl_host->desc) {
		dev_err(owl_host->dev, "Can't prepare slave sg\n");
		goto err_out;
	}
	
	owl_host->data = data;

	owl_host->desc->callback = owl_mmc_dma_complete;
	owl_host->desc->callback_param = (void *)owl_host;
	dmaengine_submit(owl_host->desc);

	dma_async_issue_pending(owl_host->dma);

err_out:
	dma_unmap_sg(owl_host->dma->device->dev, data->sg, data->sg_len,
		     owl_host->dma_dir);
}

static void owl_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct owl_mmc_host *owl_host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;
	
	pr_info("%s: %d\n", __func__,__LINE__);
	if (mrq->data) {
		mmc_update_reg(owl_host->base + OWL_REG_SDx_EN, OWL_SD_EN_BSEL, true);
		writel(data->blocks, owl_host->base + OWL_REG_SDx_BLK_NUM);
		writel(data->blksz, owl_host->base + OWL_REG_SDx_BLK_SIZE);
		writel(data->blocks * data->blocks, owl_host->base + OWL_REG_SDx_BUF_SIZE);
	}

	owl_host->mrq = mrq;
	owl_mmc_send_cmd(owl_host, mrq->cmd, data);
	
	if (mrq->data)
		owl_mmc_start_dma(owl_host, data);
}

static void owl_mmc_set_clk_rate(struct owl_mmc_host *owl_host,
				 unsigned int rate)
{
	unsigned long clk_rate;
	int ret;
	u32 reg;

	reg = mmc_readl(owl_host, OWL_REG_SDx_CTL);
	reg &= ~OWL_SD_CTL_DELAY_MSK;

	/* Set RDELAY and WDELAY based on the clock */
	if (rate <= 1000000) {
		mmc_writel(owl_host, OWL_REG_SDx_CTL, reg |
		       OWL_SD_CTL_RDELAY(OWL_SD_DELAY_LOW_CLK) |
		       OWL_SD_CTL_WDELAY(OWL_SD_DELAY_LOW_CLK));
	} else if ((rate > 1000000) && (rate <= 26000000)) {
		mmc_writel(owl_host, OWL_REG_SDx_CTL, reg |
		       OWL_SD_CTL_RDELAY(OWL_SD_DELAY_MID_CLK) |
		       OWL_SD_CTL_WDELAY(OWL_SD_DELAY_MID_CLK));
	} else if ((rate > 26000000) && (rate <= 52000000)) {
		mmc_writel(owl_host, OWL_REG_SDx_CTL, reg |
		       OWL_SD_CTL_RDELAY(OWL_SD_RDELAY_DDR50) |
		       OWL_SD_CTL_WDELAY(OWL_SD_WDELAY_DDR50));
	} else {
		pr_err("SD3.0 max clock should not > 100Mhz\n");
		return;
	}

	pr_info("%s: Rate: %u from: %u\n", __func__, rate << 1, rate);
	clk_rate = clk_round_rate(owl_host->clk, rate << 1);
	if (rate < 0) {
		pr_err("Cannot get suitable rate: %d\n", rate);
		return;
	}

	ret = clk_set_rate(owl_host->clk, clk_rate);
	if (ret < 0)
		pr_err("Cannot set rate %ld\n", clk_rate);
}

static void owl_mmc_set_clk(struct owl_mmc_host *owl_host, struct mmc_ios *ios)
{
	if (!ios->clock)
		return;

	owl_host->clock = ios->clock;
	pr_info("%s: %d\n", __func__, __LINE__);
	owl_mmc_set_clk_rate(owl_host, ios->clock);	
}

static void owl_mmc_set_bus_width(struct owl_mmc_host *owl_host,
				  struct mmc_ios *ios)
{
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		mmc_writel(owl_host, OWL_REG_SDx_EN, OWL_SD_EN_DATAWID(1));
		break;
	case MMC_BUS_WIDTH_4:
		mmc_writel(owl_host, OWL_REG_SDx_EN, OWL_SD_EN_DATAWID(4));
		break;
	case MMC_BUS_WIDTH_8:
		mmc_writel(owl_host, OWL_REG_SDx_EN, OWL_SD_EN_DATAWID(8));
		break;
	}
}

static void owl_mmc_ctr_reset(struct owl_mmc_host *owl_host)
{
	reset_control_assert(owl_host->reset);
	udelay(20);
	reset_control_deassert(owl_host->reset);
}

static void owl_mmc_power_on(struct owl_mmc_host *owl_host)
{
	u32 mode;

	/* Enable transfer end IRQ */
	mmc_update_reg(owl_host->base + OWL_REG_SDx_STATE,
		       OWL_SD_STATE_TEIE, true);

	/* Send init clk */
	mode = (mmc_readl(owl_host, OWL_REG_SDx_CTL) & (0xff << 16));
	mode |= OWL_SD_CTL_TS | OWL_SD_CTL_TCN(5) | OWL_SD_CTL_TM(8);
	mmc_update_reg(owl_host->base + OWL_REG_SDx_CTL, mode, true);
}

static void owl_mmc_card_power(struct owl_mmc_host *owl_host,
			       struct mmc_ios *ios)
{
	struct mmc_host *mmc = owl_host->mmc;

	switch (ios->power_mode) {
	case MMC_POWER_UP:
		pr_info("%s: %d\n", __func__, __LINE__);
		dev_dbg(mmc_dev(mmc), "Powering card up\n");

		/* Reset the SDC controller to clear all previous states */
		owl_mmc_ctr_reset(owl_host);

		/* TODO: Regulator enable */	
		int ret = clk_prepare_enable(owl_host->clk);
		if (ret)
			pr_info("%s: clk enable failed\n", __func__);

		mmc_writel(owl_host, OWL_REG_SDx_EN, OWL_SD_ENABLE |
			   OWL_SD_EN_RESE);

		break;

	case MMC_POWER_ON:
		pr_info("%s: %d\n", __func__, __LINE__);
		dev_dbg(mmc_dev(mmc), "Powering card on\n");
		owl_mmc_power_on(owl_host);
		break;

	case MMC_POWER_OFF:
		dev_dbg(mmc_dev(mmc), "Powering card off\n");

		/* TODO: Regulator disable */	
		clk_disable_unprepare(owl_host->clk);
		break;

	default:
		dev_dbg(mmc_dev(mmc), "Ignoring unknown card power state\n");
		break;
	}
}

static void owl_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct owl_mmc_host *owl_host = mmc_priv(mmc);

	owl_mmc_card_power(owl_host, ios);
	if (ios->clock != owl_host->clock)
		owl_mmc_set_clk(owl_host, ios);
	owl_mmc_set_bus_width(owl_host, ios);

	/* Enable DDR mode if requested */
	if (ios->timing == MMC_TIMING_UHS_DDR50)
		mmc_update_reg(owl_host->base + OWL_REG_SDx_EN,
			       OWL_SD_EN_DDREN, true);

	return;
}

static void owl_mmc_hw_reset(struct mmc_host *mmc)
{
	pr_info("%s: %d\n", __func__, __LINE__);
	return;
}

static const struct mmc_host_ops owl_mmc_ops = {
	.request	 = owl_mmc_request,
	.set_ios	 = owl_mmc_set_ios,
	.get_ro		 = mmc_gpio_get_ro,
	.get_cd		 = mmc_gpio_get_cd,
	.hw_reset	 = owl_mmc_hw_reset,
};

static int owl_mmc_probe(struct platform_device *pdev)
{
	struct owl_mmc_host *owl_host;
	struct mmc_host *mmc;
	struct resource *res;
	int ret;

	mmc = mmc_alloc_host(sizeof(struct owl_mmc_host), &pdev->dev);
	if (!mmc) {
		dev_err(&pdev->dev, "mmc alloc host failed\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, mmc);

	owl_host = mmc_priv(mmc);
	owl_host->dev = &pdev->dev;
	owl_host->mmc = mmc;
	spin_lock_init(&owl_host->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no memory resource\n");
		ret = -ENODEV;
		goto err_free_host;
	}

        owl_host->base = devm_ioremap_resource(&pdev->dev, res);
        if (IS_ERR(owl_host->base)) {
                dev_err(&pdev->dev, "Failed to remap registers\n");
                ret = PTR_ERR(owl_host->base);
                goto err_free_host;
        }

	owl_host->clk = devm_clk_get(&pdev->dev, "mmc");
	if (IS_ERR(owl_host->clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		ret = PTR_ERR(owl_host->clk);
		goto err_free_host;
	}

	pr_info("CLK Name: %s\n", __clk_get_name(owl_host->clk));
	owl_host->reset = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(owl_host->reset)) {
		dev_err(&pdev->dev, "Could not get reset control\n");
		ret = PTR_ERR(owl_host->reset);
		goto err_free_host;
	}

	mmc->ops		= &owl_mmc_ops;
	mmc->max_blk_count	= 512;
	mmc->max_blk_size	= 512;
	mmc->max_segs		= 256;
	mmc->max_seg_size	= 262144;
	mmc->max_req_size	= 262144;
	/* 100kHz ~ 52MHz */
	mmc->f_min		= 100000;
	mmc->f_max		= 52000000;
	mmc->caps	       |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
				  MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 |
				  MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_DDR50 |
				  MMC_CAP_ERASE | MMC_CAP_8_BIT_DATA |
				  MMC_CAP_1_8V_DDR | MMC_CAP_NEEDS_POLL;
	mmc->ocr_avail 		= MMC_VDD_32_33 | MMC_VDD_33_34 |
				  MMC_VDD_34_35 | MMC_VDD_35_36;

	ret = mmc_of_parse(mmc);
	if (ret)
		goto err_free_host;

	owl_host->dma = dma_request_slave_channel(&pdev->dev, "mmc");

	dev_info(&pdev->dev, "using %s for DMA transfers\n",
		 dma_chan_name(owl_host->dma));

	owl_host->dma_cfg.src_addr = res->start + OWL_REG_SDx_DAT;
	owl_host->dma_cfg.dst_addr = res->start + OWL_REG_SDx_DAT;
	owl_host->dma_cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	owl_host->dma_cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	owl_host->irq = platform_get_irq(pdev, 0);
	if (owl_host->irq < 0) {
		ret = -EINVAL;
		goto err_free_host;
	}

	ret = devm_request_irq(&pdev->dev, owl_host->irq, owl_irq_handler,
			       0, dev_name(&pdev->dev), owl_host);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq %d\n",
			owl_host->irq);
		goto err_free_host;
	}

	ret = mmc_add_host(mmc);
	if (ret) {
		dev_err(&pdev->dev, "failed to add host\n");
		goto err_free_host;
	}

	pr_info("OWL MMC Controller Initialized\n");

	return 0;

err_free_host:
	mmc_free_host(mmc);

	return ret;
}

static int owl_mmc_remove(struct platform_device *pdev)
{
	struct mmc_host	*mmc = platform_get_drvdata(pdev);
	struct owl_mmc_host *owl_host = mmc_priv(mmc);

	mmc_remove_host(mmc);
	disable_irq(owl_host->irq);
	mmc_free_host(mmc);

	return 0;
}

static const struct of_device_id owl_mmc_of_match[] = {
	{.compatible = "actions,s900-mmc",},
	{ /* sentinel */ }
};

static struct platform_driver owl_mmc_driver = {
	.driver = {
		.name	= "owl_mmc",
		.of_match_table = of_match_ptr(owl_mmc_of_match),
	},
	.probe		= owl_mmc_probe,
	.remove		= owl_mmc_remove,
};
module_platform_driver(owl_mmc_driver);

MODULE_DESCRIPTION("Actions Semi Owl SoCs SD/MMC/SDIO Controller Driver");
MODULE_AUTHOR("Actions Semi");
MODULE_AUTHOR("Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>");
MODULE_LICENSE("GPL");
