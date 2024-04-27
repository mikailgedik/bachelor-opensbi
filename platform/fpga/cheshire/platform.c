/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019 FORTH-ICS/CARV
 *				Panagiotis Peristerakis <perister@ics.forth.gr>
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_encoding.h>
#include <sbi/riscv_io.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_const.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_platform.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/fdt/fdt_fixup.h>
#include <sbi_utils/ipi/aclint_mswi.h>
#include <sbi_utils/irqchip/plic.h>
#include <sbi_utils/serial/uart8250.h>
#include <sbi_utils/timer/aclint_mtimer.h>

#define CHESHIRE_UART_ADDR	      0x03002000
#define CHESHIRE_UART_FREQ	      50000000
#define CHESHIRE_UART_BAUDRATE	      115200
#define CHESHIRE_UART_REG_SHIFT	      2
#define CHESHIRE_UART_REG_WIDTH	      4
#define CHESHIRE_PLIC_ADDR	      0x04000000
#define CHESHIRE_PLIC_NUM_SOURCES     20
#define CHESHIRE_HART_COUNT	      1
#define CHESHIRE_CLINT_ADDR	      0x02040000
#define CHESHIRE_ACLINT_MTIMER_FREQ   1000000
#define CHESHIRE_ACLINT_MSWI_ADDR     (CHESHIRE_CLINT_ADDR + 0x0)
#define CHESHIRE_ACLINT_MTIMER_ADDR   (CHESHIRE_CLINT_ADDR + 0xbff8)
#define CHESHIRE_ACLINT_MTIMECMP_ADDR (CHESHIRE_CLINT_ADDR + 0x4000)

#define CHESHIRE_VGA_ADDR             0x03008000
#define CHESHIRE_FB_ADDR              0xA0000000

//If you change these: remembere to also change the clock speed, as well as the front porch/sync times!
#define CHESHIRE_PIXTOT_W			  1056
#define CHESHIRE_PIXTOT_H			  628
#define CHESHIRE_FB_HEIGHT            600
#define CHESHIRE_FB_WIDTH  			  800

//The pixtot values are
#define PIXTOT ((CHESHIRE_PIXTOT_W<<16) + CHESHIRE_PIXTOT_H)
#define PIXACT ((CHESHIRE_FB_WIDTH<<16) + CHESHIRE_FB_HEIGHT)
#define FRONT_PORCH ((40<<16) + 1)
#define SYNC_TIMES (((128<<16) + 4) | (1<<31) | (1<<15))
//Take some cool looking values
#define COLS 90
#define ROWS 35

#define ENABLE_TEXT_MODE 0

//Internal registers offsets
#define AXI2HDMI_CMD_IF_OFFSET     0x00000008  // Paper's command interface's register size
#define AXI2HDMI_POINTERQ          ( 0 * AXI2HDMI_CMD_IF_OFFSET)
#define AXI2HDMI_H_VTOT            ( 1 * AXI2HDMI_CMD_IF_OFFSET)
#define AXI2HDMI_H_VACTIVE         ( 2 * AXI2HDMI_CMD_IF_OFFSET)
#define AXI2HDMI_H_VFRONT          ( 3 * AXI2HDMI_CMD_IF_OFFSET)
#define AXI2HDMI_H_VSYNC           ( 4 * AXI2HDMI_CMD_IF_OFFSET)
#define AXI2HDMI_POWERREG          ( 5 * AXI2HDMI_CMD_IF_OFFSET)
#define AXI2HDMI_CURRENT_PTR       ( 6 * AXI2HDMI_CMD_IF_OFFSET)
#define AXI2HDMI_TEXT_BUFF_PARA    ( 7 * AXI2HDMI_CMD_IF_OFFSET)
#define AXI2HDMI_CURSOR_FONT_PARA  ( 8 * AXI2HDMI_CMD_IF_OFFSET)

static struct platform_uart_data uart = {
	CHESHIRE_UART_ADDR,
	CHESHIRE_UART_FREQ,
	CHESHIRE_UART_BAUDRATE,
};

static struct plic_data plic = {
	.addr = CHESHIRE_PLIC_ADDR,
	.num_src = CHESHIRE_PLIC_NUM_SOURCES,
};

static struct aclint_mswi_data mswi = {
	.addr = CHESHIRE_ACLINT_MSWI_ADDR,
	.size = ACLINT_MSWI_SIZE,
	.first_hartid = 0,
	.hart_count = CHESHIRE_HART_COUNT,
};

static struct aclint_mtimer_data mtimer = {
	.mtime_freq = CHESHIRE_ACLINT_MTIMER_FREQ,
	.mtime_addr = CHESHIRE_ACLINT_MTIMER_ADDR,
	.mtime_size = 8,
	.mtimecmp_addr = CHESHIRE_ACLINT_MTIMECMP_ADDR,
	.mtimecmp_size = 16,
	.first_hartid = 0,
	.hart_count = CHESHIRE_HART_COUNT,
	.has_64bit_mmio = FALSE,
};

/*
 * Cheshire platform early initialization.
 */
static int cheshire_early_init(bool cold_boot)
{
	void *fdt;
	struct platform_uart_data uart_data;
	int rc;

	if (!cold_boot)
		return 0;
	fdt = fdt_get_address();

	rc = fdt_parse_uart8250(fdt, &uart_data, "ns16550a");
	if (!rc)
		uart = uart_data;

	return 0;
}

static void set_axi2hdmi_testpattern() {
	// Generate test pattern for screen
	uint32_t RGB[8] = {
		0xffffff, //White
		0xffff00, //Yellow
		0x00ffff, //Cyan
		0x00ff00, //Green
		0xff00ff, //Magenta
		0xff0000, //Red
		0x0000ff, //Blue
		0x000000, //Black
	};
	int col_width = CHESHIRE_FB_WIDTH / 8;

    volatile uint8_t *fb = (volatile uint16_t*)(void*)(uintptr_t) CHESHIRE_FB_ADDR;

    for (int y=0; y < CHESHIRE_FB_HEIGHT; y++) {
        for (int x=0; x < CHESHIRE_FB_WIDTH; x++) {
			uint32_t rgb = RGB[x / col_width];
			//TODO use pixel packed format
			fb[0] = rgb;
			fb[1] = rgb >> 8;
			fb[2] = rgb >> 16;

			fb += 3;
        }
    }
	//Make sure that changes are written to memory
	fence();
}

static volatile uint32_t * reg32(uint32_t base_addr, uint32_t byte_offset) {
	return (volatile uint32_t *) (base_addr + byte_offset);
}

//Sets a starting image, inits the peripheral and starts the peripheral
static void init_axi2hdmi() {
    *reg32(CHESHIRE_VGA_ADDR, AXI2HDMI_H_VTOT) = PIXTOT;

    *reg32(CHESHIRE_VGA_ADDR, AXI2HDMI_H_VACTIVE) = PIXACT;

    *reg32(CHESHIRE_VGA_ADDR, AXI2HDMI_H_VFRONT) = FRONT_PORCH;

    *reg32(CHESHIRE_VGA_ADDR, AXI2HDMI_H_VSYNC) = SYNC_TIMES;
    
    *reg32(CHESHIRE_VGA_ADDR, AXI2HDMI_TEXT_BUFF_PARA) = (COLS << 16) | ROWS;

	//Do NOT set the colors for text mode; there is a default color palette, which is perfectly fine

    //Set next frame as text mode, if in text mode
    *reg32(CHESHIRE_VGA_ADDR, AXI2HDMI_POWERREG) = (0 | (ENABLE_TEXT_MODE << 16));

    //Bitmask, so that the framebuffer is used over and over again and not discarded!
    *reg32(CHESHIRE_VGA_ADDR, AXI2HDMI_POINTERQ) = CHESHIRE_FB_ADDR | 0b010;

    *reg32(CHESHIRE_VGA_ADDR, AXI2HDMI_POWERREG) = (1 | (ENABLE_TEXT_MODE << 16));
}

/*
 * Cheshire platform final initialization.
 */
static int cheshire_final_init(bool cold_boot)
{
	void *fdt;

	if (!cold_boot)
		return 0;

	fdt = fdt_get_address();
	fdt_fixups(fdt);

	set_axi2hdmi_testpattern();

	init_axi2hdmi();
	
	return 0;
}

/*
 * Initialize the cheshire console.
 */
static int cheshire_console_init(void)
{
	return uart8250_init(uart.addr,
			     uart.freq,
			     uart.baud,
			     CHESHIRE_UART_REG_SHIFT,
			     CHESHIRE_UART_REG_WIDTH);
}

static int plic_cheshire_warm_irqchip_init(int m_cntx_id, int s_cntx_id)
{
//  size_t i, ie_words = CHESHIRE_PLIC_NUM_SOURCES / 32 + 1;

	/* By default, enable all IRQs for M-mode of target HART */
//  if (m_cntx_id > -1) {
//  	for (i = 0; i < ie_words; i++)
//  		plic_set_ie(&plic, m_cntx_id, i, 1);
//  }
//  /* Enable all IRQs for S-mode of target HART */
//  if (s_cntx_id > -1) {
//  	for (i = 0; i < ie_words; i++)
//  		plic_set_ie(&plic, s_cntx_id, i, 1);
//  }
//  /* By default, enable M-mode threshold */
//  if (m_cntx_id > -1)
//  	plic_set_thresh(&plic, m_cntx_id, 1);
//  /* By default, disable S-mode threshold */
//  if (s_cntx_id > -1)
//  	plic_set_thresh(&plic, s_cntx_id, 0);

	return plic_warm_irqchip_init(&plic, m_cntx_id, s_cntx_id);
}

/*
 * Initialize the cheshire interrupt controller for current HART.
 */
static int cheshire_irqchip_init(bool cold_boot)
{
	u32 hartid = current_hartid();
	int ret;

	if (cold_boot) {
		ret = plic_cold_irqchip_init(&plic);
		if (ret)
			return ret;
	}
	return plic_cheshire_warm_irqchip_init(2 * hartid, 2 * hartid + 1);
}

/*
 * Initialize IPI for current HART.
 */
static int cheshire_ipi_init(bool cold_boot)
{
	int ret;

	if (cold_boot) {
		ret = aclint_mswi_cold_init(&mswi);
		if (ret)
			return ret;
	}

	return aclint_mswi_warm_init();
}

/*
 * Initialize cheshire timer for current HART.
 */
static int cheshire_timer_init(bool cold_boot)
{
	int ret;

	if (cold_boot) {
		ret = aclint_mtimer_cold_init(&mtimer, NULL);
		if (ret)
			return ret;
	}

	return aclint_mtimer_warm_init();
}

/*
 * Platform descriptor.
 */
const struct sbi_platform_operations platform_ops = {
	.early_init = cheshire_early_init,
	.final_init = cheshire_final_init,
	.console_init = cheshire_console_init,
	.irqchip_init = cheshire_irqchip_init,
	.ipi_init = cheshire_ipi_init,
	.timer_init = cheshire_timer_init,
};

const struct sbi_platform platform = {
	.opensbi_version = OPENSBI_VERSION,
	.platform_version = SBI_PLATFORM_VERSION(0x0, 0x01),
	.name = "CHESHIRE RISC-V",
	.features = SBI_PLATFORM_DEFAULT_FEATURES,
	.hart_count = CHESHIRE_HART_COUNT,
	.hart_stack_size = SBI_PLATFORM_DEFAULT_HART_STACK_SIZE,
	.platform_ops_addr = (unsigned long)&platform_ops
};
