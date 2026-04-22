// SPDX-License-Identifier: GPL-2.0
/*
 * Anlogic Display Driver (Dynclk / Fixed 1080p)
 *
 * This driver supports both dynamic clock configuration (based on device tree
 * display-timings) and a fixed 1080p mode. The mode is selected by the Kconfig
 * symbol CONFIG_ANLOGIC_DYNCLK.
 *
 * Supported resolutions in dynclk mode: 1920x1080, 1280x720, 800x600, 800x480
 * Fixed mode: 1920x1080 @ 150 MHz pixel clock (50 MHz refclk assumed)
 *
 * Copyright (c) 2026 anlogic Technologies
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/of_dma.h>
#include <linux/dma/anlogic_dma.h>
#include <video/videomode.h>
#include <video/of_videomode.h>
#include <video/of_display_timing.h>

/* ========== VTC register definitions ========== */
#define VTC_RESET               0x20
#define VTC_HFRONT              0x00
#define VTC_HACTIVE             0x04
#define VTC_HSYNC               0x08
#define VTC_HBACK               0x0c
#define VTC_VFRONT              0x10
#define VTC_VACTIVE             0x14
#define VTC_VSYNC               0x18
#define VTC_VBACK               0x1c

/* ========== PLL register definitions ========== */
#define PLL_N_ADDR              0x04
#define PLL_M_ADDR              0x12
#define PLL_M_FRAC_LOW_ADDR     0x10
#define PLL_M_FRAC_HIGH_ADDR    0x11
#define PLL_C0_DIV_ADDR         0x27
#define PLL_C0_DUTY_ADDR        0x24

#define PLL_BASE_ADDR           0x00
#define PLL_OUTPUT_CTRL_ADDR    0x0C
#define PLL_LOCK_STATUS_ADDR    0x08

/* PLL VCO limits */
#define PLL_VCO_MIN_HZ          800000000UL
#define PLL_VCO_MAX_HZ          2000000000UL
#define PLL_VCO_PREFERRED_MIN_HZ 1600000000UL
#define PLL_N_MIN               1
#define PLL_N_MAX               127
#define PLL_M_INT_MIN           3
#define PLL_M_INT_MAX           126
#define PLL_C_MIN               1
#define PLL_C_MAX               128
#define FRAC_PRECISION          4096
#define MIN_DUTY_CYCLES         4

/* Fixed 1080p parameters (50 MHz refclk -> 150 MHz pixel clock) */
#define FIXED_PLL_N             1
#define FIXED_PLL_M_INT         18
#define FIXED_PLL_M_FRAC        0
#define FIXED_PLL_C             6
#define FIXED_DUTY              (FIXED_PLL_C / 2)

/* Register write macro (with optional debug) */
#define pll_write_reg(dev, base, reg, val) do { \
	dev_dbg(dev, "PLL write: reg=0x%02x, value=0x%02x\n", (reg), (val)); \
	writel(((val) | ((reg<<8)&0xff00)), (base)); \
} while (0)

#define DDEV_FLG_FIXEDMODE		BIT(0)
#define DDEV_FLG_FIXEDPLL		BIT(1)
/* ========== Device structure ========== */
struct display_dev {
	struct platform_device *pdev;
	void __iomem *vtc_base;         /* VTC register base */
#ifdef CONFIG_ANLOGIC_DYNCLK
	void __iomem *pll_base;         /* PLL register base */
	uint32_t refclk;                /* Input clock frequency (Hz) */
	uint32_t expclk;                /* Desired output clock frequency (Hz) */
#endif
	struct fb_info *info;
	struct dma_chan *vdma;
	uint32_t num_buffers;
	uint32_t flags;
	uint8_t bits_per_pixel;
	uint8_t bytes_per_pixel;
	uint8_t fmt_idx;
	atomic_t done;
	wait_queue_head_t wait;
	struct videomode vmode;
	struct dma_interleaved_template xt;
	struct data_chunk sgl;
};

static const struct fb_bitfield bitfields[][4] = {
	{{16, 8, 0}, {8, 8, 0}, {0, 8, 0}, {24, 8, 0}},		//ARGB8888
	{{0, 8, 0}, {8, 8, 0}, {16, 8, 0}, {24, 8, 0}},		//ABGR8888
	{{16, 8, 0}, {8, 8, 0}, {0, 8, 0}, {0, 0, 0}},		//RGB888
	{{0, 8, 0}, {8, 8, 0}, {16, 8, 0}, {0, 0, 0}}		//BGR888
};

/* ========== PLL parameter calculation (dynclk only) ========== */
#ifdef CONFIG_ANLOGIC_DYNCLK
struct pll_config_t {
	unsigned long clk_in_hz;
	unsigned long clk_out_hz;
	int n_div;
	int m_div_int;
	int m_div_frac;
	int c_div;
	unsigned long vco_freq_hz;
	unsigned long actual_clk_out_hz;
	int error_ppm;
	int duty_ratio_percent;
};

static int calculate_pll_params_simplified(struct pll_config_t *config, struct device *dev)
{
	int n, c;
	int best_n = 0, best_c = 0, best_m_scaled = 0;
	int best_error = INT_MAX;
	unsigned long long m_total_scaled;

	if (!config)
		return -EINVAL;

	dev_dbg(dev, "Simplified PLL calculation: input=%lu Hz, target=%lu Hz\n",
		config->clk_in_hz, config->clk_out_hz);

	for (n = PLL_N_MIN; n <= PLL_N_MAX; n++) {
		for (c = PLL_C_MIN; c <= PLL_C_MAX; c++) {
			unsigned long long vco_scaled;
			unsigned long long m_scaled;
			int m_int;
			unsigned long long actual_vco_hz;
			unsigned long long actual_out_hz;
			unsigned long long diff;
			int error_ppm;
			int quality_score;
			bool is_better;

			vco_scaled = (unsigned long long)config->clk_out_hz *
							c * FRAC_PRECISION;
			m_scaled = (vco_scaled * n) / config->clk_in_hz;
			m_int = (int)(m_scaled / FRAC_PRECISION);

			if (m_int < PLL_M_INT_MIN || m_int > PLL_M_INT_MAX)
				continue;

			actual_vco_hz = (config->clk_in_hz * m_scaled) /
						       ((unsigned long long)n * FRAC_PRECISION);
			if (actual_vco_hz < PLL_VCO_MIN_HZ || actual_vco_hz > PLL_VCO_MAX_HZ)
				continue;

			actual_out_hz = actual_vco_hz / c;
			diff = (actual_out_hz > config->clk_out_hz) ?
					      (actual_out_hz - config->clk_out_hz) :
					      (config->clk_out_hz - actual_out_hz);
			error_ppm = (int)((diff * 1000000ULL) / config->clk_out_hz);
			quality_score = 0;

			if (actual_vco_hz > PLL_VCO_PREFERRED_MIN_HZ)
				quality_score += 100;
			if (c >= MIN_DUTY_CYCLES)
				quality_score += 50;
			if (c % 2 == 0)
				quality_score += 25;

			is_better = false;

			if (error_ppm < best_error)
				is_better = true;
			else if (error_ppm == best_error && quality_score > 0)
				is_better = true;

			if (is_better) {
				best_error = error_ppm;
				best_n = n;
				best_c = c;
				best_m_scaled = (int)m_scaled;

				if (error_ppm <= 10)
					break;
			}
		}
		if (best_error <= 10)
			break;
	}

	if (best_n == 0 || best_c == 0 || best_m_scaled == 0) {
		dev_err(dev, "Failed to find valid PLL parameters\n");
		return -EINVAL;
	}

	config->n_div = best_n;
	config->c_div = best_c;
	config->m_div_int = best_m_scaled / FRAC_PRECISION;
	config->m_div_frac = best_m_scaled % FRAC_PRECISION;

	m_total_scaled = (unsigned long long)config->m_div_int * FRAC_PRECISION +
					    config->m_div_frac;
	config->vco_freq_hz = (config->clk_in_hz * m_total_scaled) /
			((unsigned long long)config->n_div * FRAC_PRECISION);
	config->actual_clk_out_hz = config->vco_freq_hz / config->c_div;
	config->error_ppm = best_error;
	config->duty_ratio_percent = (config->c_div % 2 == 0) ? 50 :
			 ((config->c_div + 1) * 50) / config->c_div;

	return 0;
}

/* ========== PLL configuration (common) ========== */
static int pll_configure(struct display_dev *ddev)
{
	void __iomem *base = ddev->pll_base;
	struct device *dev = &ddev->pdev->dev;
	int i;

	if (ddev->flags & DDEV_FLG_FIXEDPLL) {
		dev_info(dev, "Configuring PLL with fixed parameters\n");

		/* Disable output before configuration */
		writel(0x0, base + PLL_OUTPUT_CTRL_ADDR);

		/* Write fixed PLL registers */
		pll_write_reg(dev, base, PLL_N_ADDR, (FIXED_PLL_N - 1) & 0xff);
		pll_write_reg(dev, base, PLL_M_ADDR, ((FIXED_PLL_M_INT - 1) | 0x80) & 0xff);
		pll_write_reg(dev, base, PLL_M_FRAC_LOW_ADDR, FIXED_PLL_M_FRAC & 0xFF);
		pll_write_reg(dev, base, PLL_M_FRAC_HIGH_ADDR, (FIXED_PLL_M_FRAC >> 8) & 0x0F);
		pll_write_reg(dev, base, PLL_C0_DIV_ADDR, (FIXED_PLL_C - 1) & 0xff);
		pll_write_reg(dev, base, PLL_C0_DUTY_ADDR, FIXED_DUTY & 0xff);
	} else {
		struct pll_config_t pll_config;
		unsigned int best_duty;
		unsigned int frac_low, frac_high;

		memset(&pll_config, 0, sizeof(pll_config));
		pll_config.clk_in_hz = ddev->refclk;
		pll_config.clk_out_hz = ddev->expclk;

		if (calculate_pll_params_simplified(&pll_config, dev)) {
			dev_err(dev, "Unable to find suitable PLL configuration\n");
			return -EINVAL;
		}

		dev_info(dev, "PLL config: VCO=%lu Hz, N=%d, M=%d.%04d, C=%d\n",
			pll_config.vco_freq_hz, pll_config.n_div, pll_config.m_div_int,
			(pll_config.m_div_frac * 10000) / FRAC_PRECISION, pll_config.c_div);
		dev_info(dev, "Output: expected=%u Hz, actual=%lu Hz, error=%d ppm, duty=%d%%\n",
			ddev->expclk, pll_config.actual_clk_out_hz, pll_config.error_ppm,
			pll_config.duty_ratio_percent);

		/* Disable output before configuration */
		writel(0x0, base + PLL_OUTPUT_CTRL_ADDR);

		/* Write PLL registers */
		pll_write_reg(dev, base, PLL_N_ADDR, (pll_config.n_div - 1) & 0xff);
		pll_write_reg(dev, base, PLL_M_ADDR, ((pll_config.m_div_int - 1) | 0x80) & 0xff);

		frac_low = pll_config.m_div_frac & 0xFF;
		frac_high = (pll_config.m_div_frac >> 8) & 0x0F;
		pll_write_reg(dev, base, PLL_M_FRAC_LOW_ADDR, frac_low);
		pll_write_reg(dev, base, PLL_M_FRAC_HIGH_ADDR, frac_high);

		pll_write_reg(dev, base, PLL_C0_DIV_ADDR, (pll_config.c_div - 1) & 0xff);

		if (pll_config.c_div % 2 == 0)
			best_duty = pll_config.c_div / 2;
		else
			best_duty = (pll_config.c_div + 1) / 2;
		pll_write_reg(dev, base, PLL_C0_DUTY_ADDR, best_duty & 0xff);
	}

	/* Wait for PLL lock */
	for (i = 0; i < 100; i++) {
		udelay(10);
		if (readl(base + PLL_LOCK_STATUS_ADDR) & 0x1)
			break;
	}

	/* Enable output */
	writel(0x1, base + PLL_OUTPUT_CTRL_ADDR);
	dev_info(dev, "PLL configured and locked\n");

	return 0;
}
#endif /* CONFIG_ANLOGIC_DYNCLK */

/* ========== VTC initialization ========== */
static int vtc_init(struct display_dev *ddev, struct videomode *vmode)
{
	void __iomem *vtc_base = ddev->vtc_base;
	struct device *dev = &ddev->pdev->dev;

	writel(0x0, vtc_base + VTC_RESET);
	udelay(10);
	writel(vmode->hfront_porch, vtc_base + VTC_HFRONT);
	writel(vmode->hactive, vtc_base + VTC_HACTIVE);
	writel(vmode->hsync_len, vtc_base + VTC_HSYNC);
	writel(vmode->hback_porch, vtc_base + VTC_HBACK);
	writel(vmode->vfront_porch, vtc_base + VTC_VFRONT);
	writel(vmode->vactive, vtc_base + VTC_VACTIVE);
	writel(vmode->vsync_len, vtc_base + VTC_VSYNC);
	writel(vmode->vback_porch, vtc_base + VTC_VBACK);
	writel(0x1, vtc_base + VTC_RESET);

	dev_info(dev, "VTC registers: HACTIVE=%u (exp %u), VACTIVE=%u (exp %u)\n",
		readl(vtc_base + VTC_HACTIVE), vmode->hactive,
		readl(vtc_base + VTC_VACTIVE), vmode->vactive);
	return 0;
}

static void vdmafb_callback(void *data)
{
	struct display_dev *ddev = data;

	atomic_set(&ddev->done, 1);
	wake_up_interruptible(&ddev->wait);
}

static int vdmafb_wait_for_completion(struct display_dev *ddev, unsigned long timeout_ms)
{
	int ret = wait_event_interruptible_timeout(ddev->wait,
			atomic_read(&ddev->done) != 0,
			msecs_to_jiffies(timeout_ms));
	if (ret == 0) {
		dev_err(&ddev->pdev->dev, "Timeout waiting for VDMA completion\n");
		return -ETIMEDOUT;
	}
	return 0;
}

/* ========== VDMA initialization ========== */
static int vdma_init(struct display_dev *ddev)
{
	struct device *dev = &ddev->pdev->dev;
	struct fb_info *info = ddev->info;
	struct dma_async_tx_descriptor *tx;
	struct anlogic_vdma_config config;
	dma_cookie_t cookie;
	enum dma_status status;
	int ret;

	ddev->vdma = of_dma_request_slave_channel(dev->of_node, "vdma");
	if (IS_ERR(ddev->vdma)) {
		dev_err(dev, "Failed to request vdma channel: %ld\n", PTR_ERR(ddev->vdma));
		return PTR_ERR(ddev->vdma);
	}
	if (!ddev->vdma) {
		dev_err(dev, "VDMA channel is NULL\n");
		return -ENODEV;
	}
	dev_info(dev, "VDMA channel obtained\n");

	dmaengine_terminate_all(ddev->vdma);

	memset(&ddev->xt, 0, sizeof(ddev->xt));
	ddev->xt.dir = DMA_MEM_TO_DEV;
	ddev->xt.numf = info->var.yres;
	ddev->xt.sgl[0].size = info->fix.line_length;
	ddev->xt.sgl[0].icg = 0;
	ddev->xt.frame_size = 1;
	ddev->xt.src_sgl = 1;
	ddev->xt.dst_sgl = 0;
	ddev->xt.src_inc = true;

	ddev->xt.src_start = info->fix.smem_start;

	init_waitqueue_head(&ddev->wait);

	tx = dmaengine_prep_interleaved_dma(ddev->vdma, &ddev->xt,
					DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!tx) {
		dev_err(dev, "Failed to prepare DMA descriptor\n");
		ret = -ENOMEM;
		goto err_release_chan;
	}

	tx->callback = vdmafb_callback;
	tx->callback_param = ddev;

	cookie = dmaengine_submit(tx);
	ret = dma_submit_error(cookie);
	if (ret) {
		dev_err(dev, "Failed to submit DMA descriptor\n");
		dmaengine_desc_free(tx);
		goto err_release_chan;
	}

	/* Configure VDMA (assuming anlogic_vdma_channel_set_config exists) */
	memset(&config, 0, sizeof(config));
	config.park = 1;
	config.park_frm = 0;
	anlogic_vdma_channel_set_config(ddev->vdma, &config);

	atomic_set(&ddev->done, 0);

	/* Start DMA */
	dma_async_issue_pending(ddev->vdma);

	/* Wait a bit for DMA to start */
	msleep(100);

	/* Check DMA status for the last submitted cookie */
	status = dma_async_is_tx_complete(ddev->vdma, cookie, NULL, NULL);

	dev_info(dev, "DMA status after 100ms: %d (0=idle,1=in_progress,2=complete,3=error)\n",
		status);

	if (status != DMA_COMPLETE && status != DMA_IN_PROGRESS) {
		dev_err(dev, "DMA not started properly, status=%d\n", status);
		/* Continue anyway, the DMA might still be okay */
	} else {
		dev_info(dev, "VDMA started successfully\n");
	}

	return 0;

err_release_chan:
	dma_release_channel(ddev->vdma);
	return ret;
}

/* ========== Framebuffer operations ========== */
static int vdmafb_setcolreg(unsigned int regno, unsigned int red,
			    unsigned int green, unsigned int blue,
			    unsigned int transp, struct fb_info *info)
{
	u32 tmp;

	if (regno >= 16)
		return -EINVAL;

	red >>= 8; green >>= 8; blue >>= 8;
	tmp = (red << 16) | (green << 8) | blue;
	((u32 *)(info->pseudo_palette))[regno] = tmp;
	return 0;
}

static int vdmafb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	dev_dbg(info->dev, "check_var: xres: %u->%u, yres: %u->%u, ...\n",
	    info->var.xres, var->xres, info->var.yres, var->yres);

	if (var->xres != info->var.xres ||
	    var->yres != info->var.yres ||
	    var->xres_virtual != info->var.xres_virtual ||
	    var->bits_per_pixel != info->var.bits_per_pixel) {
		dev_err(info->dev, "mode change not supported\n");
		return -EINVAL;
	}

	if (var->xoffset) {
		dev_err(info->dev, "xoffset not supported\n");
		return -EINVAL;
	}

	return 0;
}

static int vdmafb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct display_dev *ddev = info->par;
	unsigned int buffer_index;
	struct dma_async_tx_descriptor *tx;
	dma_cookie_t cookie;
	int ret = 0;

	dev_dbg(info->dev, "pan_display: yoffset=%u\n", var->yoffset);

	if (var->yoffset % info->var.yres) {
		dev_err(info->dev, "yoffset %% yres is not zero\n");
		return -EINVAL;
	}

	buffer_index = var->yoffset / info->var.yres;
	if (buffer_index >= ddev->num_buffers) {
		dev_err(info->dev, "buffer_index %u >= num_buffers %u\n",
			buffer_index, ddev->num_buffers);
		return -EINVAL;
	}

	ddev->xt.src_start = info->fix.smem_start +
		   buffer_index * info->var.yres * info->fix.line_length;

	tx = dmaengine_prep_interleaved_dma(ddev->vdma, &ddev->xt,
					DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!tx) {
		dev_err(info->dev, "Failed to prepare DMA descriptor for buffer %d\n",
			buffer_index);
		return -ENOMEM;
	}

	tx->callback = vdmafb_callback;
	tx->callback_param = ddev;

	cookie = dmaengine_submit(tx);
	ret = dma_submit_error(cookie);
	if (ret) {
		dev_err(info->dev, "Failed to submit DMA descriptor for buffer %d\n",
			buffer_index);
		dmaengine_desc_free(tx);
		return ret;
	}

	atomic_set(&ddev->done, 0);

	dma_async_issue_pending(ddev->vdma);
	info->var.yoffset = var->yoffset;
	vdmafb_wait_for_completion(ddev, 40);
	return 0;
}

static const struct fb_ops vdmafb_ops = {
	.owner = THIS_MODULE,
	.fb_pan_display = vdmafb_pan_display,
	.fb_setcolreg = vdmafb_setcolreg,
	.fb_check_var = vdmafb_check_var,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

static void video_fixed_mode(struct display_dev *ddev)
{
	struct device *dev = &ddev->pdev->dev;

	/* Fixed 1080p video mode */
	ddev->vmode.hactive = 1920;
	ddev->vmode.vactive = 1080;
	ddev->vmode.hfront_porch = 88;
	ddev->vmode.hback_porch = 148;
	ddev->vmode.hsync_len = 44;
	ddev->vmode.vfront_porch = 4;
	ddev->vmode.vback_porch = 36;
	ddev->vmode.vsync_len = 5;
	ddev->vmode.pixelclock = 150000000; /* 150 MHz */
	ddev->vmode.flags = 0;
	ddev->flags |= DDEV_FLG_FIXEDMODE;
	dev_info(dev, "Using fixed video mode: %dx%d, pixel clock=%lu Hz\n",
		ddev->vmode.hactive, ddev->vmode.vactive, ddev->vmode.pixelclock);
}

static void video_bitfield_setup(struct display_dev *ddev, const char *name)
{
	struct device *dev = &ddev->pdev->dev;

	if (!name)
		goto default_bitfield;
	else if (strcmp(name, "ARGB8888") == 0) {
		ddev->bits_per_pixel = 32;
		ddev->bytes_per_pixel = 4;
		ddev->fmt_idx = 0;
	} else if (strcmp(name, "ABGR8888") == 0) {
		ddev->bits_per_pixel = 32;
		ddev->bytes_per_pixel = 4;
		ddev->fmt_idx = 1;
	} else if (strcmp(name, "RGB888") == 0) {
		ddev->bits_per_pixel = 24;
		ddev->bytes_per_pixel = 3;
		ddev->fmt_idx = 2;
	} else if (strcmp(name, "BGR888") == 0) {
		ddev->bits_per_pixel = 24;
		ddev->bytes_per_pixel = 3;
		ddev->fmt_idx = 3;
	} else {
default_bitfield:
		dev_info(dev, "defaulting to RGB888\n");
		ddev->bits_per_pixel = 24;
		ddev->bytes_per_pixel = 3;
		ddev->fmt_idx = 2;
	}
}

/* ========== Framebuffer info initialization ========== */
static int fbinfo_init(struct display_dev *ddev)
{
	struct device *dev = &ddev->pdev->dev;
	struct fb_info *info;
	unsigned int fb_size;
	dma_addr_t fb_phy;
	void *fb_virt;
	int ret;

	info = framebuffer_alloc(sizeof(struct display_dev *), dev);
	if (!info) {
		dev_err(dev, "Failed to allocate framebuffer info\n");
		return -ENOMEM;
	}
	ddev->info = info;
	info->par = ddev;

	/* Video mode already obtained from device tree in probe */
	dev_info(dev, "Video mode: %dx%d, hfp=%d, hbp=%d, hsw=%d, vfp=%d, vbp=%d, vsw=%d",
		ddev->vmode.hactive, ddev->vmode.vactive,
		ddev->vmode.hfront_porch, ddev->vmode.hback_porch, ddev->vmode.hsync_len,
		ddev->vmode.vfront_porch, ddev->vmode.vback_porch, ddev->vmode.vsync_len);

	fb_size = ddev->vmode.hactive * ddev->vmode.vactive *
		ddev->bytes_per_pixel * ddev->num_buffers;
	fb_virt = dma_alloc_wc(dev, PAGE_ALIGN(fb_size), &fb_phy, GFP_KERNEL);
	if (!fb_virt) {
		dev_err(dev, "Failed to allocate framebuffer memory\n");
		ret = -ENOMEM;
		goto err_free_info;
	}

	dev_info(dev, "Framebuffer: %dx%d, %d buffers, total size=%u, phys=0x%llx",
		ddev->vmode.hactive, ddev->vmode.vactive, ddev->num_buffers,
		fb_size, (unsigned long long)fb_phy);

	memset(fb_virt, 0x0, fb_size);

	strcpy(info->fix.id, "anlogic");
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.accel = FB_ACCEL_NONE;
	info->fix.line_length = ddev->vmode.hactive * ddev->bytes_per_pixel;
	info->fix.ypanstep = 1;
	info->fix.smem_start = fb_phy;
	info->fix.smem_len = fb_size;

	info->fbops = &vdmafb_ops;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->screen_base = fb_virt;
	info->screen_size = fb_size;

	/* Set var based on videomode */
	info->var.xres = ddev->vmode.hactive;
	info->var.yres = ddev->vmode.vactive;
	info->var.xres_virtual = ddev->vmode.hactive;
	info->var.yres_virtual = ddev->vmode.vactive * ddev->num_buffers;
	info->var.xoffset = 0;
	info->var.yoffset = 0;
	info->var.grayscale = 0;
	info->var.nonstd = 0;
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.accel_flags = FB_ACCEL_NONE;
	info->var.bits_per_pixel = ddev->bits_per_pixel;
	info->var.red = bitfields[ddev->fmt_idx][0];
	info->var.green = bitfields[ddev->fmt_idx][1];
	info->var.blue = bitfields[ddev->fmt_idx][2];
	info->var.transp = bitfields[ddev->fmt_idx][3];

	info->var.pixclock = KHZ2PICOS(ddev->vmode.pixelclock / 1000);
	info->var.hsync_len = ddev->vmode.hsync_len;
	info->var.vsync_len = ddev->vmode.vsync_len;
	info->var.left_margin = ddev->vmode.hback_porch;
	info->var.right_margin = ddev->vmode.hfront_porch;
	info->var.upper_margin = ddev->vmode.vback_porch;
	info->var.lower_margin = ddev->vmode.vfront_porch;
	info->var.sync = 0;
	if (ddev->vmode.flags & DISPLAY_FLAGS_HSYNC_HIGH)
		info->var.sync |= FB_SYNC_HOR_HIGH_ACT;
	if (ddev->vmode.flags & DISPLAY_FLAGS_VSYNC_HIGH)
		info->var.sync |= FB_SYNC_VERT_HIGH_ACT;

	ret = fb_alloc_cmap(&info->cmap, 16, 0);
	if (ret) {
		dev_err(dev, "Failed to allocate color map\n");
		goto err_free_dma;
	}

	info->pseudo_palette = devm_kzalloc(dev, sizeof(u32) * 16, GFP_KERNEL);
	if (!info->pseudo_palette) {
		ret = -ENOMEM;
		goto err_free_cmap;
	}

	return 0;

err_free_cmap:
	fb_dealloc_cmap(&info->cmap);
err_free_dma:
	dma_free_wc(dev, info->screen_size, info->screen_base, info->fix.smem_start);
err_free_info:
	framebuffer_release(info);
	ddev->info = NULL;
	return ret;
}

/* ========== Platform driver probe/remove ========== */
static int display_probe(struct platform_device *pdev)
{
	struct display_dev *ddev;
	const char *str = NULL;
	int ret;

	ddev = devm_kzalloc(&pdev->dev, sizeof(*ddev), GFP_KERNEL);
	if (!ddev)
		return -ENOMEM;

	ddev->pdev = pdev;
	platform_set_drvdata(pdev, ddev);

	/* Get VTC register region (index 1) */
	ddev->vtc_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ddev->vtc_base))
		return PTR_ERR(ddev->vtc_base);

	/* Obtain video mode from device tree */
	ret = of_get_videomode(pdev->dev.of_node, &ddev->vmode, OF_USE_NATIVE_MODE);
	if (ret)
		video_fixed_mode(ddev);

	if (of_property_read_u32(pdev->dev.of_node, "buf-num", &ddev->num_buffers))
		ddev->num_buffers = 2;

	of_property_read_string(pdev->dev.of_node, "pixel-format", &str);
	video_bitfield_setup(ddev, str);

#ifdef CONFIG_ANLOGIC_DYNCLK
	/* Get PLL register region (index 0) */
	ddev->pll_base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(ddev->pll_base)) {
		dev_err(&pdev->dev, "pll resource not found\n");
		return PTR_ERR(ddev->pll_base);
	}

	/* Read refclk from device tree */
	ret = of_property_read_u32(pdev->dev.of_node, "refclk", &ddev->refclk);
	if (ret) {
		if (!(ddev->flags & DDEV_FLG_FIXEDMODE)) {
			dev_err(&pdev->dev, "refclk property not found\n");
			return -EINVAL;
		}
		ddev->flags |= DDEV_FLG_FIXEDPLL;
	} else {
		dev_info(&pdev->dev, "Display driver: refclk=%u Hz\n", ddev->refclk);

		/* Determine desired output clock frequency:
		 * first try 'expclk' property, else use pixelclock
		 */
		if (of_property_read_u32(pdev->dev.of_node, "expclk", &ddev->expclk))
			ddev->expclk = ddev->vmode.pixelclock;  /* pixelclock is in Hz */
		dev_info(&pdev->dev, "Target pixel clock: %u Hz\n", ddev->expclk);
	}

	/* Configure PLL (dynamic or fixed) */
	ret = pll_configure(ddev);
	if (ret)
		return ret;
#endif

	/* Initialize framebuffer info */
	ret = fbinfo_init(ddev);
	if (ret)
		return ret;

	/* Initialize VTC */
	ret = vtc_init(ddev, &ddev->vmode);
	if (ret)
		goto err_free_fb;

	/* Initialize VDMA */
	ret = vdma_init(ddev);
	if (ret)
		goto err_free_fb;

	/* Register framebuffer */
	ret = register_framebuffer(ddev->info);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register framebuffer\n");
		goto err_release_dma;
	}

	return 0;

err_release_dma:
	if (!IS_ERR(ddev->vdma)) {
		dmaengine_terminate_all(ddev->vdma);
		dma_release_channel(ddev->vdma);
	}
err_free_fb:
	if (ddev->info) {
		fb_dealloc_cmap(&ddev->info->cmap);
		dma_free_wc(&pdev->dev, ddev->info->screen_size,
			    ddev->info->screen_base, ddev->info->fix.smem_start);
		framebuffer_release(ddev->info);
	}
	return ret;
}

static int display_remove(struct platform_device *pdev)
{
	struct display_dev *ddev = platform_get_drvdata(pdev);
	struct fb_info *info = ddev->info;

	unregister_framebuffer(info);
	if (!IS_ERR(ddev->vdma)) {
		dmaengine_terminate_all(ddev->vdma);
		dma_release_channel(ddev->vdma);
	}
	fb_dealloc_cmap(&info->cmap);
	dma_free_wc(&pdev->dev, info->screen_size,
		info->screen_base, info->fix.smem_start);
	framebuffer_release(info);
	return 0;
}

static void display_shutdown(struct platform_device *pdev)
{
	/* Optional shutdown handling */
}

static const struct of_device_id display_of_match[] = {
	{ .compatible = "anlogic,vdmafb", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, display_of_match);

static struct platform_driver display_driver = {
	.probe = display_probe,
	.remove = display_remove,
	.shutdown = display_shutdown,
	.driver = {
		.name = "anlogic-vdmafb-display",
		.of_match_table = display_of_match,
	},
};

module_platform_driver(display_driver);

MODULE_DESCRIPTION("Anlogic Display Driver (Dynclk / Fixed 1080p)");
MODULE_AUTHOR("minghao.lv");
MODULE_LICENSE("GPL");
