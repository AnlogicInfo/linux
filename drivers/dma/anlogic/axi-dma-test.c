// SPDX-License-Identifier: GPL-2.0
/*
 * anlogic axi dma Test Driver
 *
 * Copyright (c) 2024 anlogic Technologies Co., Ltd.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>

#define GP0_Master                                   0x80000000UL
#define STREAM_DATA_GEN_OFFSET                       0x100000
#define STREAM_DATA_CHECK_OFFSET                     0x200000

#define AL_AXI_DMA0_MM2S_STREAM_DATA_WIDTH           32
#define AL_AXI_DMA0_S2MM_STREAM_DATA_WIDTH           32

#define AL_AXI_DMA0_MM2S_MEMORY_MAP_DATA_WIDTH       64
#define AL_AXI_DMA0_S2MM_MEMORY_MAP_DATA_WIDTH       64

#define BUFFER_SIZE                                  0x1000
#define TEST_START_VALUE                             0x0
#define BD_NUM                                       0x2

struct dma_test_device {
	struct dma_chan *tx_chan;
	struct dma_chan *rx_chan;
	dma_addr_t dma_handle[BD_NUM];
	unsigned char *buffer[BD_NUM];
	size_t buffer_length[BD_NUM];
	struct completion tx_completion;
	struct completion rx_completion;
	void __iomem *base_addr;
	struct device *dev;
};

static void GenrateData(struct dma_test_device *dev)
{
	iowrite32((BUFFER_SIZE * BD_NUM) /
		  (AL_AXI_DMA0_S2MM_STREAM_DATA_WIDTH / 8),
		  dev->base_addr + STREAM_DATA_GEN_OFFSET + 0x04);
	iowrite32(TEST_START_VALUE,
		  dev->base_addr + STREAM_DATA_GEN_OFFSET + 0x00);
}

static void PrepareDataCheck(struct dma_test_device *dev)
{
	iowrite32((BUFFER_SIZE * BD_NUM) /
		  (AL_AXI_DMA0_MM2S_STREAM_DATA_WIDTH / 8),
		  dev->base_addr + STREAM_DATA_CHECK_OFFSET + 0x04);
	iowrite32(TEST_START_VALUE,
		  dev->base_addr + STREAM_DATA_CHECK_OFFSET + 0x00);
}

static void DmaTransferCheck(struct dma_test_device *dev)
{
	if (ioread32(dev->base_addr + STREAM_DATA_CHECK_OFFSET + 0x08)) {
		dev_info(dev->dev, "axi dma loopback test fail!\n");
	} else if (ioread32(dev->base_addr + STREAM_DATA_CHECK_OFFSET + 0x0C) &&
		   ioread32(dev->base_addr + STREAM_DATA_CHECK_OFFSET + 0x10)) {
		dev_info(dev->dev, "axi dma loopback test success!\n");
	} else {
		dev_info(dev->dev, "axi dma loopback test invalid!\n");
	}
}

static void tx_dma_callback(void *param)
{
	struct dma_test_device *dev = (struct dma_test_device *)param;

	DmaTransferCheck(dev);
	complete(&dev->tx_completion);
}

static void rx_dma_callback(void *param)
{
	struct dma_test_device *dev = (struct dma_test_device *)param;

	PrepareDataCheck(dev);
	complete(&dev->rx_completion);
}

static int anlogic_axidmatest_probe(struct platform_device *pdev)
{
	struct dma_test_device *dev;
	struct dma_async_tx_descriptor *tx;
	struct dma_async_tx_descriptor *rx;
	dma_cookie_t tx_cookie;
	dma_cookie_t rx_cookie;
	struct scatterlist sg[BD_NUM];
	int ret = 0;
	int i = 0;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = &pdev->dev;

	platform_set_drvdata(pdev, dev);

	dev->base_addr = ioremap(GP0_Master, 0x201000);
	if (!dev->base_addr) {
		dev_info(dev->dev, "Failed to map device registers\n");
		ret = -ENOMEM;
		goto free_dev;
	}

	init_completion(&dev->tx_completion);
	init_completion(&dev->rx_completion);

	dev->tx_chan = dma_request_chan(&pdev->dev, "axidma_mm2s");
	if (IS_ERR(dev->tx_chan)) {
		dev_info(dev->dev, "DMA TX channel request failed\n");
		ret = PTR_ERR(dev->tx_chan);
		goto unmap_registers;
	}

	dev->rx_chan = dma_request_chan(&pdev->dev, "axidma_s2mm");
	if (IS_ERR(dev->rx_chan)) {
		dev_info(dev->dev, "DMA RX channel request failed\n");
		ret = PTR_ERR(dev->rx_chan);
		goto release_tx_channel;
	}

	for (i = 0; i < BD_NUM; i++) {
		dev->buffer_length[i] = BUFFER_SIZE;

		dev->buffer[i] = dma_alloc_coherent(&pdev->dev,
						dev->buffer_length[i],
						&dev->dma_handle[i],
						GFP_KERNEL);
		if (!dev->buffer[i]) {
			ret = -ENOMEM;
			goto release_rx_channel;
		}

		if (!IS_ALIGNED((unsigned long)dev->buffer[i],
				AL_AXI_DMA0_S2MM_MEMORY_MAP_DATA_WIDTH / 8)) {
			ret = -EINVAL;
			goto free_buffer;
		}
	}

	sg_init_table(sg, BD_NUM);

	for (i = 0; i < BD_NUM; i++) {
		sg_dma_address(&sg[i]) = dev->dma_handle[i];
		sg_dma_len(&sg[i]) = dev->buffer_length[i];
		dev_info(dev->dev, "scatterlist[%d] address: %pad, length: %u\n",
			 i, &sg_dma_address(&sg[i]), sg_dma_len(&sg[i]));
	}

	rx = dmaengine_prep_slave_sg(dev->rx_chan, sg, BD_NUM, DMA_DEV_TO_MEM,
				     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!rx) {
		dev_info(dev->dev, "Failed to prepare DMA rx transfer\n");
		ret = -EINVAL;
		goto free_buffer;
	}
	rx->callback = rx_dma_callback;
	rx->callback_param = dev;

	tx = dmaengine_prep_slave_sg(dev->tx_chan, sg, BD_NUM, DMA_MEM_TO_DEV,
				     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx) {
		dev_info(dev->dev, "Failed to prepare DMA tx transfer\n");
		ret = -EINVAL;
		goto free_buffer;
	}
	tx->callback = tx_dma_callback;
	tx->callback_param = dev;

	rx_cookie = rx->tx_submit(rx);
	if (dma_submit_error(rx_cookie)) {
		dev_info(dev->dev, "DMA rx_submit failed\n");
		ret = -EIO;
		goto free_buffer;
	}

	tx_cookie = tx->tx_submit(tx);
	if (dma_submit_error(tx_cookie)) {
		dev_info(dev->dev, "DMA tx_submit failed\n");
		ret = -EIO;
		goto free_buffer;
	}

	// Start s2mm (RX) transfer
	GenrateData(dev);
	dma_async_issue_pending(dev->rx_chan);

	// Wait for s2mm (RX) transfer to complete
	wait_for_completion(&dev->rx_completion);

	// Start mm2s (TX) transfer
	dma_async_issue_pending(dev->tx_chan);

	// Wait for mm2s (TX) transfer to complete
	wait_for_completion(&dev->tx_completion);

	return 0;

free_buffer:
	for (i = 0; i < BD_NUM; i++) {
		if (dev->buffer[i])
			dma_free_coherent(&pdev->dev, dev->buffer_length[i],
					  dev->buffer[i], dev->dma_handle[i]);
	}
release_rx_channel:
	if (dev->rx_chan) {
		dmaengine_terminate_all(dev->rx_chan);
		dma_release_channel(dev->rx_chan);
	}
release_tx_channel:
	if (dev->tx_chan) {
		dmaengine_terminate_all(dev->tx_chan);
		dma_release_channel(dev->tx_chan);
	}
unmap_registers:
	if (dev->base_addr)
		iounmap(dev->base_addr);
free_dev:
	devm_kfree(&pdev->dev, dev);
	return ret;
}

static int anlogic_axidmatest_remove(struct platform_device *pdev)
{
	struct dma_test_device *dev = platform_get_drvdata(pdev);
	int i;

	if (!dev)
		return -EINVAL;

	if (dev->tx_chan) {
		dmaengine_terminate_all(dev->tx_chan);
		dma_release_channel(dev->tx_chan);
	}

	if (dev->rx_chan) {
		dmaengine_terminate_all(dev->rx_chan);
		dma_release_channel(dev->rx_chan);
	}

	if (dev->base_addr)
		iounmap(dev->base_addr);

	for (i = 0; i < BD_NUM; i++)
		dma_free_coherent(&pdev->dev, dev->buffer_length[i],
				  dev->buffer[i], dev->dma_handle[i]);

	return 0;
}

static const struct of_device_id anlogic_axidmatest_of_ids[] = {
	{ .compatible = "anlogic,axi-dma-test", },
	{}
};

static struct platform_driver anlogic_axidmatest_driver = {
	.driver = {
		.name = "anlogic_axidma_test",
		.of_match_table = anlogic_axidmatest_of_ids,
	},
	.probe = anlogic_axidmatest_probe,
	.remove = anlogic_axidmatest_remove,
};

static int __init axidma_init(void)
{
	return platform_driver_register(&anlogic_axidmatest_driver);
}
late_initcall(axidma_init);

static void __exit axidma_exit(void)
{
	platform_driver_unregister(&anlogic_axidmatest_driver);
}
module_exit(axidma_exit);

MODULE_AUTHOR("Naiyuan Que <naiyuan.que@anlogic.com>");
MODULE_DESCRIPTION("anlogic axi dma Test Driver");
MODULE_LICENSE("GPL");
