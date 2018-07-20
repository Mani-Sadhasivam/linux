struct sunxi_mmc_host {
	struct device *dev;
	struct mmc_host	*mmc;
	struct reset_control *reset;
	const struct sunxi_mmc_cfg *cfg;

	void __iomem	*reg_base;

	struct clk	*clk_ahb;
	struct clk	*clk_mmc;
	struct clk	*clk_sample;
	struct clk	*clk_output;

	spinlock_t	lock;
	
	int		irq;
	u32		int_sum;
	u32		sdio_imask;

	/* dma */
	dma_addr_t	sg_dma;
	void		*sg_cpu;
	bool		wait_dma;

	struct mmc_request *mrq;
	struct mmc_request *manual_stop_mrq;
};

static int owl_mmc_probe(struct platform_device *pdev)
{
	struct owl_mmc_host *host;
	struct mmc_host *mmc;
	int ret;

	mmc = mmc_alloc_host(sizeof(struct owl_mmc_host), &pdev->dev);
	if (!mmc) {
		dev_err(&pdev->dev, "mmc alloc host failed\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, mmc);

	host = mmc_priv(mmc);
	host->dev = &pdev->dev;
	host->mmc = mmc;
	spin_lock_init(&host->lock);

	ret = owl_mmc_resource_request(host, pdev);
	if (ret)
		goto error_free_host;

	host->sg_cpu = dma_alloc_coherent(&pdev->dev, PAGE_SIZE,
					  &host->sg_dma, GFP_KERNEL);
	if (!host->sg_cpu) {
		dev_err(&pdev->dev, "Failed to allocate DMA descriptor mem\n");
		ret = -ENOMEM;
		goto error_free_host;
	}

	mmc->ops		= &owl_mmc_ops;
	mmc->max_blk_count	= 8192;
	mmc->max_blk_size	= 4096;
	mmc->max_segs		= PAGE_SIZE / sizeof(struct owl_idma_des);
	mmc->max_seg_size	= (1 << host->cfg->idma_des_size_bits);
	mmc->max_req_size	= mmc->max_seg_size * mmc->max_segs;
	/* 400kHz ~ 52MHz */
	mmc->f_min		= 400000;
	mmc->f_max		= 52000000;
	mmc->caps	       |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
				  MMC_CAP_ERASE | MMC_CAP_SDIO_IRQ;

	if (host->cfg->clk_delays || host->use_new_timings)
		mmc->caps      |= MMC_CAP_1_8V_DDR;

	ret = mmc_of_parse(mmc);
	if (ret)
		goto error_free_dma;

	ret = owl_mmc_init_host(host);
	if (ret)
		goto error_free_dma;

	ret = mmc_add_host(mmc);
	if (ret)
		goto error_free_dma;

	dev_info(&pdev->dev, "base:0x%p irq:%u\n", host->reg_base, host->irq);
	return 0;

error_free_dma:
	dma_free_coherent(&pdev->dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);
error_free_host:
	mmc_free_host(mmc);
	return ret;
}

static int owl_mmc_remove(struct platform_device *pdev)
{
	struct mmc_host	*mmc = platform_get_drvdata(pdev);
	struct owl_mmc_host *host = mmc_priv(mmc);

	mmc_remove_host(mmc);
	disable_irq(host->irq);
	owl_mmc_disable(host);
	dma_free_coherent(&pdev->dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);
	mmc_free_host(mmc);

	return 0;
}

static struct platform_driver owl_mmc_driver = {
	.driver = {
		.name	= "owl-mmc",
		.of_match_table = of_match_ptr(owl_mmc_of_match),
	},
	.probe		= owl_mmc_probe,
	.remove		= owl_mmc_remove,
};
module_platform_driver(owl_mmc_driver);

MODULE_DESCRIPTION("Actions Semi Owl SoCs SD/MMC Card Controller Driver");
MODULE_LICENSE("GPL");
