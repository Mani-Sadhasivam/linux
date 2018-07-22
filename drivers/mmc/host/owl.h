#ifndef _OWL_MMC_H_
#define _OWL_MMC_H_

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dma-direction.h>
#include <linux/clk.h>



#define FDI_EMMC_ID			0x14
#define POWER_GATE_ON 		1
#define POWER_GATE_OFF   		0

#define OWL_UPGRADE			0
#define OWL_NORMAL_BOOT		1
#define OWL_REGULATOR_OFF		0
#define OWL_REGULATOR_ON		1

#define REG_ENABLE 				1
#define REG_DISENABLE 			0
#define MMC_CMD_COMPLETE		1

/*
 * command response code
 */
#define CMD_OK						BIT(0)
#define CMD_RSP_ERR				BIT(1)
#define CMD_RSP_BUSY				BIT(2)
#define CMD_RSP_CRC_ERR			BIT(3)
#define CMD_TS_TIMEOUT			BIT(4)
#define CMD_DATA_TIMEOUT			BIT(5)
#define HW_TIMEOUT					BIT(6)
#define DATA_WR_CRC_ERR			BIT(7)
#define DATA_RD_CRC_ERR			BIT(8)
#define DATA0_BUSY_ERR				BIT(9)

enum {
	PURE_CMD,
	DATA_CMD,
};

/*
 * card type
 */
enum {
	MMC_CARD_DISABLE,
	MMC_CARD_MEMORY,
	MMC_CARD_EMMC,
	MMC_CARD_WIFI,
};

/* * wifi card detect voltage */
enum { DETECT_CARD_LOW_VOLTAGE,
	DETECT_CARD_NORMAL_VOLTAGE,
};

#define mmc_card_expected_mem(type)	((type) == MMC_CARD_MEMORY)
#define mmc_card_expected_emmc(type)	((type) == MMC_CARD_EMMC)
#define mmc_card_expected_wifi(type)	((type) == MMC_CARD_WIFI)

/*
 * card detect method
 */
enum {
	SIRQ_DETECT_CARD,
	GPIO_DETECT_CARD,
	COMMAND_DETECT_CARD,
};

typedef enum {
	UART_PIN,		/*uart2 0r uart5 rx tx */
	UART_SD_PIN,		/* sd0 cmd clk , uart2 0r uart5 rx,tx */
	SD_PIN,			/*sd0 cmd clk sddate0-3 */
} PIN_STAT;

#define SDC0_SLOT	0
#define SDC1_SLOT	1
#define SDC2_SLOT	2
#define SDC3_SLOT	3

#define SDC0_BASE	0xe0330000
#define SDC1_BASE	0xe0334000
#define SDC2_BASE	0xe0338000
#define SDC3_BASE	0xe033c000

#define ACTS_MMC_OCR (MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30  | \
	MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33  | \
	MMC_VDD_33_34 | MMC_VDD_34_35 | MMC_VDD_35_36)

struct mmc_con_delay {
	unsigned char delay_lowclk;
	unsigned char delay_midclk;
	unsigned char delay_highclk;
};

struct owl_mmc_host {
	spinlock_t lock;
	struct mutex pin_mutex;
	PIN_STAT switch_pin_flag;	/*UART_PIN: uart mode and host0 cmd sd0 clk vail
					 * SD_PIN: cmd clk sd0-sd3
					 * ERR_PIN: init status*/
	u8 id;			/* SD Controller number */
	u32 module_id;		/* global module ID */
	void __iomem *iobase;
	void __iomem *nand_io_base;
	void __iomem *mfp_base;
	void __iomem *cmu_base;

	bool nand_powergate;
	u32 start;
	u32 type_expected;	/* MEMORY Card or SDIO Card */

	int card_detect_mode;	/* which method used to detect card */
	char pinctrname[14];
	u32 detect;		/* irq line for mmc/sd card detect */
	u32 detect_sirq;	/* Which SIRQx used to detect card */
	int detect_irq_registered;	/* card detect irq is registered */

	u32 sdc_irq;		/* irq line for SDC transfer end */
	int wifi_detect_voltage;
	struct completion sdc_complete;

	u32 sdio_irq;		/* irq for SDIO wifi data transfer */
	u32 eject;		/* card status */

	int power_state;	/* card status */
	int bus_width;		/* data bus width */
	int chip_select;
	int timing;
	u32 clock;		/* current clock frequency */
	u32 clk_on;		/* card module clock status */
	u32 nandclk_on;		/* card module clock status */
	struct clk *dev_clk;		/* devpll clock source */
	struct clk *nandpll_clk;		/* devpll clock source */
	struct clk *clk;	/* SDC clock source */
	struct clk *nandclk;	/* SDC clock source */
	struct notifier_block nblock;	/* clkfreq notifier block */
	struct regulator *reg;	/* supply regulator */
	u32 regulator_status;	/* card module clock status */

	struct timer_list timer;	/* used for gpio card detect */
	u32 detect_pin;		/* gpio card detect pin number */
	int wpswitch_gpio;	/* card write protect gpio */
	int present;		/* card is inserted or extracted ? */
	int sdio_present;	/* Wi-Fi is open or not ? */

	struct pinctrl *pcl;

	char write_delay_chain;
	char read_delay_chain;
	char write_delay_chain_bak;
	char read_delay_chain_bak;
	char adjust_write_delay_chain;
	char adjust_read_delay_chain;
	int sdio_uart_supported;
	int card_detect_reverse;
	int send_continuous_clock;	/* WiFi need to send continuous clock */

	struct mmc_host *mmc;
	struct mmc_request *mrq;

	enum dma_data_direction dma_dir;
	struct dma_chan *dma;
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config dma_conf;
	struct completion dma_complete;
	struct workqueue_struct *dma_wq;
	struct workqueue_struct *add_host_wq;
	struct delayed_work dma_work;
	struct delayed_work host_add_work;

	struct mmc_con_delay wdelay;
	struct mmc_con_delay rdelay;
	unsigned char pad_drv;
	struct reset_control *rst;
};

/** pard voltage*/
#define SD0_PAD_POWER 			(1<<8)
#define SD1_PAD_POWER 			(1<<7)

/*
 * PAD Drive Capacity config
 */
#define PAD_DRV_LOW			(0)
#define PAD_DRV_MID			(1)
#define PAD_DRV_HIGH			(3)

#define SDC0_WDELAY_LOW_CLK	(0xf)
#define SDC0_WDELAY_MID_CLK	(0xa)
#define SDC0_WDELAY_HIGH_CLK	(0x9)

#define SDC0_RDELAY_LOW_CLK	(0xf)
#define SDC0_RDELAY_MID_CLK	(0xa)
#define SDC0_RDELAY_HIGH_CLK	(0x8)
#define SDC0_RDELAY_DDR50		(0xa)
#define SDC0_WDELAY_DDR50		(0x8)

#define SDC0_PAD_DRV			PAD_DRV_MID

#define SDC1_WDELAY_LOW_CLK	(0xf)
#define SDC1_WDELAY_MID_CLK	(0xa)
#define SDC1_WDELAY_HIGH_CLK	(0x8)

#define SDC1_RDELAY_LOW_CLK	(0xf)
#define SDC1_RDELAY_MID_CLK	(0xa)
#define SDC1_RDELAY_HIGH_CLK	(0x8)

#define SDC1_PAD_DRV			PAD_DRV_MID

#define SDC2_WDELAY_LOW_CLK	(0xf)
#define SDC2_WDELAY_MID_CLK	(0xa)
#define SDC2_WDELAY_HIGH_CLK	(0x8)

#define SDC2_RDELAY_LOW_CLK	(0xf)
#define SDC2_RDELAY_MID_CLK	(0xa)
#define SDC2_RDELAY_HIGH_CLK	(0x8)

#define SDC2_PAD_DRV			PAD_DRV_MID

#define SDC3_WDELAY_LOW_CLK	(0xf)
#define SDC3_WDELAY_MID_CLK	(0xa)
#define SDC3_WDELAY_HIGH_CLK	(0x8)

#define SDC3_RDELAY_LOW_CLK	(0xf)
#define SDC3_RDELAY_MID_CLK	(0xa)
#define SDC3_RDELAY_HIGH_CLK	(0x8)

#define SDC3_PAD_DRV			PAD_DRV_MID

/*
 * SDC registers
 */
#define SD_EN_OFFSET			0x0000
#define SD_CTL_OFFSET			0x0004
#define SD_STATE_OFFSET		0x0008
#define SD_CMD_OFFSET			0x000c
#define SD_ARG_OFFSET			0x0010
#define SD_RSPBUF0_OFFSET		0x0014
#define SD_RSPBUF1_OFFSET		0x0018
#define SD_RSPBUF2_OFFSET		0x001c
#define SD_RSPBUF3_OFFSET		0x0020
#define SD_RSPBUF4_OFFSET		0x0024
#define SD_DAT_OFFSET			0x0028
#define SD_BLK_SIZE_OFFSET		0x002c
#define SD_BLK_NUM_OFFSET		0x0030
#define SD_BUF_SIZE_OFFSET		0x0034

#define HOST_EN(h)				((h)->iobase + SD_EN_OFFSET)
#define HOST_CTL(h)				((h)->iobase + SD_CTL_OFFSET)
#define HOST_STATE(h)			((h)->iobase + SD_STATE_OFFSET)
#define HOST_CMD(h)				((h)->iobase + SD_CMD_OFFSET)
#define HOST_ARG(h)				((h)->iobase + SD_ARG_OFFSET)
#define HOST_RSPBUF0(h)			((h)->iobase + SD_RSPBUF0_OFFSET)
#define HOST_RSPBUF1(h)			((h)->iobase + SD_RSPBUF1_OFFSET)
#define HOST_RSPBUF2(h)			((h)->iobase + SD_RSPBUF2_OFFSET)
#define HOST_RSPBUF3(h)			((h)->iobase + SD_RSPBUF3_OFFSET)
#define HOST_RSPBUF4(h)			((h)->iobase + SD_RSPBUF4_OFFSET)
#define HOST_DAT(h)				((h)->iobase + SD_DAT_OFFSET)
#define HOST_DAT_DMA(h)		((h)->start + SD_DAT_OFFSET)
#define HOST_BLK_SIZE(h)		((h)->iobase + SD_BLK_SIZE_OFFSET)
#define HOST_BLK_NUM(h)		((h)->iobase + SD_BLK_NUM_OFFSET)
#define HOST_BUF_SIZE(h)		((h)->iobase + SD_BUF_SIZE_OFFSET)

/*
 * Register Bit defines
 */

/*
 * Register SD_EN
 */
#define SD_EN_RANE				(1 << 31)
/* bit 30 reserved */
#define SD_EN_RAN_SEED(x)		(((x) & 0x3f) << 24)
/* bit 23~13 reserved */
#define SD_EN_S18EN				(1 << 12)
/* bit 11 reserved */
#define SD_EN_RESE				(1 << 10)
#define SD_EN_DAT1_S			(1 << 9)
#define SD_EN_CLK_S				(1 << 8)
#define SD_ENABLE				(1 << 7)
#define SD_EN_BSEL				(1 << 6)
/* bit 5~4 reserved */
#define SD_EN_SDIOEN			(1 << 3)
#define SD_EN_DDREN			(1 << 2)
#define SD_EN_DATAWID(x)		(((x) & 0x3) << 0)

/*
 * Register SD_CTL
 */
#define SD_CTL_TOUTEN			(1 << 31)
#define SD_CTL_TOUTCNT(x)		(((x) & 0x7f) << 24)
#define SD_CTL_RDELAY(x)		(((x) & 0xf) << 20)
#define SD_CTL_WDELAY(x)		(((x) & 0xf) << 16)
/* bit 15~14 reserved */
#define SD_CTL_CMDLEN			(1 << 13)
#define SD_CTL_SCC				(1 << 12)
#define SD_CTL_TCN(x)			(((x) & 0xf) << 8)
#define SD_CTL_TS				(1 << 7)
#define SD_CTL_LBE				(1 << 6)
#define SD_CTL_C7EN				(1 << 5)
/* bit 4 reserved */
#define SD_CTL_TM(x)			(((x) & 0xf) << 0)

/*
 * Register SD_STATE
 */
/* bit 31~19 reserved */
#define SD_STATE_DAT1BS		(1 << 18)
#define SD_STATE_SDIOB_P		(1 << 17)
#define SD_STATE_SDIOB_EN		(1 << 16)
#define SD_STATE_TOUTE			(1 << 15)
#define SD_STATE_BAEP			(1 << 14)
/* bit 13 reserved */
#define SD_STATE_MEMRDY		(1 << 12)
#define SD_STATE_CMDS			(1 << 11)
#define SD_STATE_DAT1AS		(1 << 10)
#define SD_STATE_SDIOA_P		(1 << 9)
#define SD_STATE_SDIOA_EN		(1 << 8)
#define SD_STATE_DAT0S			(1 << 7)
#define SD_STATE_TEIE			(1 << 6)
#define SD_STATE_TEI			(1 << 5)
#define SD_STATE_CLNR			(1 << 4)
#define SD_STATE_CLC			(1 << 3)
#define SD_STATE_WC16ER		(1 << 2)
#define SD_STATE_RC16ER		(1 << 1)
#define SD_STATE_CRC7ER		(1 << 0)

#define S900

#ifdef S900

#define NAND_ALOG_CTR         				 0xa8
#define EN_V18_R							(1<<25)
#define EN_V18_W							(1<<24)
/*CMU_BASE*/
#define CMU_DEVCLKEN0                              	 0x00A0
#define CMU_DEVCLKEN1                              	 0x00A4
#define CMU_DEVPLL                                    	 0x0004
#define CMU_NANDPLL                                  	 0x000C
#define CMU_SD0CLK                                    	 0x0050
#define CMU_SD1CLK                                    	 0x0054
#define CMU_SD2CLK                                  	  	 0x0058

/* Pinctrl registers offset */
#define MFP_CTL0							0x0040
#define MFP_CTL1							0x0044
#define MFP_CTL2							0x0048
#define MFP_CTL3							0x004C
#define PAD_PULLCTL0						0x0060
#define PAD_PULLCTL1						0x0064
#define PAD_PULLCTL2						0x0068
#define PAD_ST0								0x006C
#define PAD_ST1								0x0070
#define PAD_CTL								0x0074
#define PAD_DRV0							0x0080
#define PAD_DRV1							0x0084
#define PAD_DRV2							0x0088
#define GPIO_COUTEN                                 	0x0018
#define GPIO_CINEN                                  		0x001C

 /*CMU*/
#define DUMP_CMU_DEVCLKEN0(mapbase) 		(mapbase+CMU_DEVCLKEN0)
#define DUMP_CMU_DEVCLKEN1(mapbase)	 	(mapbase+CMU_DEVCLKEN1)
#define DUMP_CMU_DEVPLL(mapbase) 			(mapbase+CMU_DEVPLL)
#define DUMP_CMU_NANDPLL(mapbase) 		(mapbase+CMU_NANDPLL)
#define DUMP_CMU_CMU_SD0CLK(mapbase) 	(mapbase+CMU_SD0CLK)
#define DUMP_CMU_CMU_SD1CLK(mapbase) 	(mapbase+CMU_SD1CLK)
#define DUMP_CMU_CMU_SD2CLK(mapbase) 	(mapbase+CMU_SD2CLK)
/* Pinctrl registers  */
#define DUMP_MFP_CTL0(mapbase) 			(mapbase+MFP_CTL0)
#define DUMP_MFP_CTL1(mapbase) 			(mapbase+MFP_CTL1)
#define DUMP_MFP_CTL2(mapbase) 			(mapbase+MFP_CTL2)
#define DUMP_MFP_CTL3(mapbase) 			(mapbase+MFP_CTL3)
#define DUMP_PAD_DVR0(mapbase) 			(mapbase+PAD_DRV0)
#define DUMP_PAD_DVR1(mapbase) 			(mapbase+PAD_DRV1)
#define DUMP_PAD_DVR2(mapbase) 			(mapbase+PAD_DRV2)
#define DUMP_PAD_PULLCTL0(mapbase)  		(mapbase+PAD_PULLCTL0)
#define DUMP_PAD_PULLCTL1(mapbase)  		(mapbase+PAD_PULLCTL1)
#define DUMP_PAD_PULLCTL2(mapbase)  		(mapbase+PAD_PULLCTL2)
#define DUMP_GPIO_CINEN(mapbase) 			(mapbase+GPIO_CINEN)
#define DUMP_GPIO_COUTEN(mapbase) 		(mapbase+GPIO_COUTEN)
#define     SPS_PG_BASE                                     0xE012e000
#define     SPS_LDO_CTL                                     (SPS_PG_BASE+0x0014)

#endif
extern void owl_mmc_ctr_reset(struct owl_mmc_host *host);
void owl_dma_debug_dump(void);
#endif /* end of _OWL_MMC_H_ */
