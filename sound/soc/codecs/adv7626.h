#ifndef _ADV7626_H_
#define _ADV7626_H_

enum adv7626_reg {
	ADV7626_REG_MAIN = 0,
	ADV7626_REG_DPLL_A,
	ADV7626_REG_DPLL_B,
	ADV7626_REG_CP_LITE_A,
	ADV7626_REG_CP_LITE_B,
	ADV7626_REG_OSD,
	ADV7626_REG_HDMI_RX1,
	ADV7626_REG_Rx1_RPT,
	ADV7626_REG_Rx1_IFRAME,
	ADV7626_REG_RX_EDID,
	ADV7626_REG_HDMI_RX2,
	ADV7626_REG_Rx2_RPT,
	ADV7626_REG_Rx2_IFRAME,
	ADV7626_REG_TxA_MAIN,
	ADV7626_REG_TxA_EDID,
	ADV7626_REG_TxA_TEST,
	ADV7626_REG_TxA_PMEM,
	ADV7626_REG_TxA_CEC,
	ADV7626_REG_TxB_MAIN,
	ADV7626_REG_TxB_EDID,
	ADV7626_REG_TxB_TEST,
	ADV7626_REG_TxB_PMEM,
	ADV7626_REG_TxB_CEC,
	ADV7626_REG_ADI_REQ,
	ADV7626_REG_MAX
};

struct adv7626_reg_map {
	const char *name;
	u8 addr;
	u8 addr_loc;
};

static const struct adv7626_reg_map adv7626_addr[] = {
	[ADV7626_REG_MAIN] = {"main", 0xB0, 0x00},
	[ADV7626_REG_DPLL_A] = {"dpll_a", 0x08, 0xE6},
	[ADV7626_REG_DPLL_B] = {"dpll_b", 0x0C, 0xE7},
	[ADV7626_REG_CP_LITE_A] = {"cp_lite_a", 0x0A, 0xE8},
	[ADV7626_REG_CP_LITE_B] = {"cp_lite_b", 0x09, 0xE9},
	[ADV7626_REG_OSD] = {"osd", 0x10, 0xEA},
	[ADV7626_REG_HDMI_RX1] = {"hdmi_rx1", 0x14, 0xEC},
	[ADV7626_REG_Rx1_RPT] = {"rx1_rpt", 0x12, 0xEE},
	[ADV7626_REG_Rx1_IFRAME] = {"rx1_iframe", 0x11, 0xEF},
	[ADV7626_REG_RX_EDID] = {"rx_edid", 0x1D, 0xED},
	[ADV7626_REG_HDMI_RX2] = {"hdmi_rx2", 0x18, 0xF0},
	[ADV7626_REG_Rx2_RPT] = {"rx2_rpt", 0x1C, 0xF2},
	[ADV7626_REG_Rx2_IFRAME] = {"rx2_iframe", 0x1A, 0xF3},
	[ADV7626_REG_TxA_MAIN] = {"txa_main", 0x19, 0xF4},
	[ADV7626_REG_TxA_EDID] = {"txa_edid", 0x20, 0xF5},
	[ADV7626_REG_TxA_TEST] = {"txa_test", 0x24, 0xF6},
	[ADV7626_REG_TxA_PMEM] = {"txa_pmem", 0x22, 0xF7},
	[ADV7626_REG_TxA_CEC] = {"txa_cec", 0x21, 0xF8},
	[ADV7626_REG_TxB_MAIN] = {"txb_main", 0x0F, 0xF9},
	[ADV7626_REG_TxB_EDID] = {"txb_edid", 0x0E, 0xFA},
	[ADV7626_REG_TxB_TEST] = {"txb_test", 0x0D, 0xFB},
	[ADV7626_REG_TxB_PMEM] = {"txb_pmem", 0x17, 0xFC},
	[ADV7626_REG_TxB_CEC] = {"txb_cec", 0x16, 0xFD},
	[ADV7626_REG_ADI_REQ] = {"adi_req", 0x15, 0xE5}
};

struct adv7626_priv {
	/* i2c clients */
	struct i2c_client *i2c_clients[ADV7626_REG_MAX];

	/* regmaps */
	struct regmap *regmap[ADV7626_REG_MAX];
};

/* IO Map registers */
#define ADV7626_IOMAP_RD_INFO_1	0xDF
#define ADV7626_IOMAP_RD_INFO_2	0xE0
#define ADV7626_IOMAP_RST	0xFF

static const struct reg_sequence main_seq[] = {
	{0x01, 0x89}, // default=0x01, HDMI Receivers Rx1 & Rx2 enabled, RxA-Rx1, RxB-Rx2
	{0x08, 0x12},
	{0x09, 0x11},
	{0x0A, 0x12},
	{0x0F, 0x0C},
	{0xB1, 0x1A},
	{0xB2, 0x1F},
	{0xB3, 0xDF},
	{0x9D, 0x00},
	{0x9E, 0x00}
};

static const struct reg_sequence hdmi_rx1_seq[] = {
	{0x01, 0x04}, // default=0x00, Rx1 HBR Setting
	{0x3E, 0x79}, // default=0x79, ADI Required Write - HDMI Rx1 Config
	{0x3F, 0x04}, // default=0x63, ADI Required Write - HDMI Rx1 Config
	{0x4E, 0xFE}, // default=0x7B, ADI Required Write - HDMI Rx1 Config
	{0x4F, 0x18}, // default=0x63, ADI Required Write - HDMI Rx1 Config
	{0x57, 0xF5}, // default=0x30, ADI Required Write - HDMI Rx1 Config
	{0x58, 0x1C}, // default=0x01, ADI Required Write - HDMI Rx1 Config
	{0x6F, 0x04}, // default=0x40, ADI Required Write - HDMI Rx1 Config
	{0x75, 0x00}, // default=0x00, ADI Required Write - HDMI Rx1 Config
	{0x85, 0x10}, // default=0x16, ADI Required Write - HDMI Rx1 Config
	{0x97, 0xC0}, // default=0x80, ADI Required Write - HDMI Rx1 Config
	{0x98, 0x3F}, // default=0xFF, ADI Required Write - HDMI Rx1 Config - for input TMDS Clock 297MHz
	{0x99, 0xE3}, // default=0xA3, ADI Required Write - HDMI Rx1 Config - for input TMDS Clock 297MHz
	{0x9A, 0x9F}, // default=0xFF, ADI Required Write - HDMI Rx1 Config - for input TMDS Clock 297MHz
	{0x9B, 0x01}, // default=0x03, ADI Required Write - HDMI Rx1 Config - for input TMDS Clock 297MHz
	{0x9C, 0x10}, // default=0x08, ADI Required Write - HDMI Rx1 Config
	{0xCB, 0x01} // default=0x00, ADI Required Write - HDMI Rx1 Config
};

static const struct reg_sequence hdmi_rx2_seq[] = {
	{0x01, 0x04}, // default=0x00, Rx2 HBR Setting
	{0x3E, 0x79}, // default=0x79, ADI Required Write - HDMI Rx2 Config
	{0x3F, 0x04}, // default=0x63, ADI Required Write - HDMI Rx2 Config
	{0x4E, 0xFE}, // default=0x7B, ADI Required Write - HDMI Rx2 Config
	{0x4F, 0x18}, // default=0x63, ADI Required Write - HDMI Rx2 Config
	{0x57, 0xF5}, // default=0x30, ADI Required Write - HDMI Rx2 Config
	{0x58, 0x1C}, // default=0x01, ADI Required Write - HDMI Rx2 Config
	{0x6F, 0x04}, // default=0x40, ADI Required Write - HDMI Rx2 Config
	{0x75, 0x00}, // default=0x00, ADI Required Write - HDMI Rx2 Config
	{0x85, 0x10}, // default=0x16, ADI Required Write - HDMI Rx2 Config
	{0x97, 0xC0}, // default=0x80, ADI Required Write - HDMI Rx2 Config
	{0x98, 0x3F}, // default=0xFF, ADI Required Write - HDMI Rx2 Config - for input TMDS Clock 297MHz
	{0x99, 0xE3}, // default=0xA3, ADI Required Write - HDMI Rx2 Config - for input TMDS Clock 297MHz
	{0x9A, 0x9F}, // default=0xFF, ADI Required Write - HDMI Rx2 Config - for input TMDS Clock 297MHz
	{0x9B, 0x01}, // default=0x03, ADI Required Write - HDMI Rx2 Config - for input TMDS Clock 297MHz
	{0x9C, 0x10}, // default=0x08, ADI Required Write - HDMI Rx2 Config
	{0xCB, 0x01} // default=0x00, ADI Required Write - HDMI Rx2 Config
};

static const struct reg_sequence hdmi_txa_seq[] = {
	{0x01, 0x00}, // default=0x00, Set N Value(6144) used for Audio 48kHz
	{0x02, 0x18}, // default=0x00, Set N Value(6144) used for Audio 48kHz
	{0x03, 0x00}, // default=0x00, Set N Value(6144) used for Audio 48kHz
	{0x13, 0xFF}, // default=0x00, Set Tx Category Code
	{0x15, 0x20}, // default 0x00, CS I2S Fs = 48kHz, Video Input Format = RGB/YCbCr 444
	{0x16, 0x61}, // default=0x00, Output video format YCbCr 444, Input Video YCbCr
	{0x40, 0x80}, // default=0x00, Enable Global Control Packet
	{0x4C, 0x04}, // default=0x00, 8-bit Deep Colour Mode
	{0x55, 0x40}, // default=0x00, AVI Infoframe - Y1Y0 = YCbCr 444 
	{0x56, 0x08}, // default=0x00, AVI Infoframe - R[3:0] = Active Format Apsect Ratio - Same as
	{0x73, 0x01}, // default 0x00, Audio IF CC = 001, 2 channels
	{0x96, 0x20}, // default=0x00, Configure TxA Interrupts
	{0xAF, 0x16}, // default=0x14, Set HDMI Mode Output
	{0xBA, 0x70}, // default=0x00, ADI Required Write - HDMI TxA Config
	{0xD0, 0x44}, // default=0x4C, ADI Required Write - HDMI TxA Config
	{0xD1, 0x3C}, // default=0x38, ADI Required Write - HDMI TxA Config
	{0xD3, 0x07}, // default=0x06, ADI Required Write - HDMI TxA Config
	{0xD6, 0x02}, // default=0x06, ADI Required Write - HDMI TxA Config
	{0xDB, 0x0B}, // default=0x0A, ADI Required Write - HDMI TxA Config
	{0xE0, 0x90}, // default=0x00, ADI Required Write - HDMI TxA Config
	{0xE1, 0xFC}, // default=0x0C, ADI Required Write - HDMI TxA Config
	{0xE3, 0xD0}, // default=0x00, ADI Required Write - HDMI TxA Config
	{0xE8, 0xF0}, // default=0x10, ADI Required Write - HDMI TxA Config
	{0xF3, 0x01}, // default=0x00, ADI Required Write - HDMI TxA Config
	{0xF5, 0xCC}, // default=0xCC, ADI Required Write - HDMI TxA Config
	{0xF6, 0x08}, // default=0x00, ADI Required Write - HDMI TxA Config
	{0xF7, 0xF0}, // default=0xFF, ADI Required Write - HDMI TxA Config
	{0xDA, 0x40}, // default=0x00, ADI Required Write - HDMI TxA Config
	{0xF5, 0xD4}, // default=0xCC, ADI Required Write - HDMI TxA Config
	{0x81, 0x33}, // default=0x88, TxA Charge Injection Ch0 = 3, TxA Charge Injection Ch1 = 3
	{0x82, 0x33}, // default=0x88, TxA Charge Injection Ch2 = 3, TxA Charge Injection Clk = 3
	{0x83, 0x03}, // default=0x00, ADI Required Write - HDMI TxA Source Termination ON
	{0x84, 0x03}, // default=0x00, ADI Required Write - HDMI TxA Source Termination ON
	{0x85, 0x03}, // default=0x00, ADI Required Write - HDMI TxA Source Termination ON
	{0x86, 0x03}, // default=0x00, ADI Required Write - HDMI TxA Source Termination ON
	{0xEA, 0x3D}, // default=0x00, ADI Required Write - HDMI TxA Source Termination ON
	{0xED, 0x38}, // default=0x80, ADI Required Write - HDMI TxA Source Termination ON
	{0xEE, 0x38}, // default=0x80, ADI Required Write - HDMI TxA Source Termination ON
	{0xEF, 0x39}, // default=0x82, ADI Required Write - HDMI TxA Source Termination ON
	{0xFC, 0x55}, // default=0x05, ADI Required Write - HDMI TxA Config
	{0x41, 0x30}, // default=0x50, ADI Required Write - HDMI TxA Config
	{0x41, 0x10} // default=0x50, ADI Required Write - HDMI TxA Config
};

static const struct reg_sequence hdmi_txb_seq[] = {
	{0x01, 0x00}, // default=0x00, Set N Value(6144) used for Audio 48kHz
	{0x02, 0x18}, // default=0x00, Set N Value(6144) used for Audio 48kHz
	{0x03, 0x00}, // default=0x00, Set N Value(6144) used for Audio 48kHz
	{0x13, 0xFF}, // default=0x00, Set Tx Category Code
	{0x15, 0x20}, // default 0x00, CS I2S Fs = 48kHz, Video Input Format = RGB/YCbCr 444
	{0x16, 0x61}, // default=0x00, Output video format YCbCr 444, Input Video YCbCr
	{0x40, 0x80}, // default=0x00, Enable Global Control Packet
	{0x4C, 0x04}, // default=0x00, 8-bit Deep Colour Mode
	{0x55, 0x40}, // default=0x00, AVI Infoframe - Y1Y0 = YCbCr 444 
	{0x56, 0x08}, // default=0x00, AVI Infoframe - R[3:0] = Active Format Apsect Ratio
	{0x73, 0x01}, // default 0x00, Audio IF CC = 001, 2 channels
	{0x96, 0x20}, // default=0x00, Configure TxA Interrupts
	{0xAF, 0x16}, // default=0x14, Set HDMI Mode Output
	{0xBA, 0x70}, // default=0x00, ADI Required Write - HDMI TxA Config
	{0xD0, 0x44}, // default=0x4C, ADI Required Write - HDMI TxA Config
	{0xD1, 0x3C}, // default=0x38, ADI Required Write - HDMI TxA Config
	{0xD3, 0x07}, // default=0x06, ADI Required Write - HDMI TxA Config
	{0xD6, 0x02}, // default=0x06, ADI Required Write - HDMI TxA Config
	{0xDB, 0x0B}, // default=0x0A, ADI Required Write - HDMI TxA Config
	{0xE0, 0x90}, // default=0x00, ADI Required Write - HDMI TxA Config
	{0xE1, 0xFC}, // default=0x0C, ADI Required Write - HDMI TxA Config
	{0xE3, 0xD0}, // default=0x00, ADI Required Write - HDMI TxA Config
	{0xE8, 0xF0}, // default=0x10, ADI Required Write - HDMI TxA Config
	{0xF3, 0x01}, // default=0x00, ADI Required Write - HDMI TxA Config
	{0xF5, 0xCC}, // default=0xCC, ADI Required Write - HDMI TxA Config
	{0xF6, 0x08}, // default=0x00, ADI Required Write - HDMI TxA Config
	{0xF7, 0xF0}, // default=0xFF, ADI Required Write - HDMI TxA Config
	{0xDA, 0x40}, // default=0x00, ADI Required Write - HDMI TxA Config
	{0xF5, 0xD4}, // default=0xCC, ADI Required Write - HDMI TxA Config
	{0x81, 0x33}, // default=0x88, TxA Charge Injection Ch0 = 3, TxA Charge Injection Ch1 = 3
	{0x82, 0x33}, // default=0x88, TxA Charge Injection Ch2 = 3, TxA Charge Injection Clk = 3
	{0x83, 0x03}, // default=0x00, ADI Required Write - HDMI TxA Source Termination ON
	{0x84, 0x03}, // default=0x00, ADI Required Write - HDMI TxA Source Termination ON
	{0x85, 0x03}, // default=0x00, ADI Required Write - HDMI TxA Source Termination ON
	{0x86, 0x03}, // default=0x00, ADI Required Write - HDMI TxA Source Termination ON
	{0xEA, 0x3D}, // default=0x00, ADI Required Write - HDMI TxA Source Termination ON
	{0xED, 0x38}, // default=0x80, ADI Required Write - HDMI TxA Source Termination ON
	{0xEE, 0x38}, // default=0x80, ADI Required Write - HDMI TxA Source Termination ON
	{0xEF, 0x39}, // default=0x82, ADI Required Write - HDMI TxA Source Termination ON
	{0xFC, 0x55}, // default=0x05, ADI Required Write - HDMI TxA Config
	{0x41, 0x30}, // default=0x50, ADI Required Write - HDMI TxA Config
	{0x41, 0x10} // default=0x50, ADI Required Write - HDMI TxA Config
};

#endif /* _ADV7626_H_ */
