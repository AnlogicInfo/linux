/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023 Anlogic, Inc.
*/
#ifndef AL_QSPI_HEADER_H
#define AL_QSPI_HEADER_H

#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/irqreturn.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/scatterlist.h>
#include <linux/spi/spi-mem.h>

/* Register offset definitions */
#define AL_QSPI_CTRLR0_OFFSET 0x0UL
#define AL_QSPI_CTRLR1_OFFSET 0x4UL
#define AL_QSPI_SSIENR_OFFSET 0x8UL
#define AL_QSPI_MWCR_OFFSET 0xCUL
#define AL_QSPI_SER_OFFSET 0x10UL
#define AL_QSPI_BAUDR_OFFSET 0x14UL
#define AL_QSPI_TXFTLR_OFFSET 0x18UL
#define AL_QSPI_RXFTLR_OFFSET 0x1CUL
#define AL_QSPI_TXFLR_OFFSET 0x20UL
#define AL_QSPI_RXFLR_OFFSET 0x24UL
#define AL_QSPI_SR_OFFSET 0x28UL
#define AL_QSPI_IMR_OFFSET 0x2CUL
#define AL_QSPI_ISR_OFFSET 0x30UL
#define AL_QSPI_RISR_OFFSET 0x34UL
#define AL_QSPI_TXOICR_OFFSET 0x38UL
#define AL_QSPI_RXOICR_OFFSET 0x3CUL
#define AL_QSPI_RXUICR_OFFSET 0x40UL
#define AL_QSPI_MSTICR_OFFSET 0x44UL
#define AL_QSPI_ICR_OFFSET 0x48UL
#define AL_QSPI_DMACR_OFFSET 0x4CUL
#define AL_QSPI_DMATDLR_OFFSET 0x50UL
#define AL_QSPI_DMARDLR_OFFSET 0x54UL
#define AL_QSPI_IDR_OFFSET 0x58UL
#define AL_QSPI_SSIC_VERSION_ID_OFFSET 0x5CUL
#define AL_QSPI_DR0_OFFSET 0x60UL
#define AL_QSPI_RX_SAMPLE_DELAY_OFFSET 0xF0UL
#define AL_QSPI_SPI_CTRLR0_OFFSET 0xF4UL
#define AL_QSPI_XIP_MODE_BITS_OFFSET 0xFCUL
#define AL_QSPI_XIP_INCR_INST_OFFSET 0x100UL
#define AL_QSPI_XIP_WRAP_INST_OFFSET 0x104UL
#define AL_QSPI_XIP_CNT_TIME_OUT_OFFSET 0x114UL


/* Register: CTRLR0
 * This register controls the serial data transfer. It is impossible to write to this register when the qspi is enabled.
 */
#define QSPI_CTRLR0_DFS_SHIFT	0
#define QSPI_CTRLR0_FRF_SHIFT	6
#define QSPI_CTRLR0_SCPH_SHIFT	8
#define QSPI_CTRLR0_SCPOL_SHIFT	9
#define QSPI_CTRLR0_TMOD_SHIFT	10
#define QSPI_CTRLR0_SLV_OE_SHIFT	12
#define QSPI_CTRLR0_SRL_SHIFT	13
#define QSPI_CTRLR0_SSTE_SHIFT	14
#define QSPI_CTRLR0_CFS_SHIFT	16
#define QSPI_CTRLR0_SPI_FRF_SHIFT	22
#define QSPI_CTRLR0_SPI_HYPERBUS_EN_SHIFT	24
#define QSPI_CTRLR0_SSI_IS_MST_SHIFT	31

#define QSPI_CTRLR0_DFS_SIZE 5
#define QSPI_CTRLR0_FRF_SIZE 2
#define QSPI_CTRLR0_SCPH_SIZE 1
#define QSPI_CTRLR0_SCPOL_SIZE 1
#define QSPI_CTRLR0_TMOD_SIZE 2
#define QSPI_CTRLR0_SLV_OE_SIZE 1
#define QSPI_CTRLR0_SRL_SIZE 1
#define QSPI_CTRLR0_SSTE_SIZE 1
#define QSPI_CTRLR0_CFS_SIZE 4
#define QSPI_CTRLR0_SPI_FRF_SIZE 2
#define QSPI_CTRLR0_SPI_HYPERBUS_EN_SIZE 1
#define QSPI_CTRLR0_SSI_IS_MST_SIZE 1

/* Register: CTRLR1
 * Control register 1 controls the end of serial transfers when in receive-only mode.
 * It is impossible to write to this register when the qspi is enabled.
 */
#define QSPI_CTRLR1_NDF_SHIFT	0
#define QSPI_CTRLR1_NDF_SIZE 16


/* Register: SSIENR
 * This register enables and disables the qspi
 */
#define QSPI_SSIENR_SSIC_EN_SHIFT	0
#define QSPI_SSIENR_SSIC_EN_SIZE 1

/* Register: MWCR
 * This register controls the direction of the data word for the half-duplex Microwire serial protocol.
 * It is impossible to write to this register when the qspi is enabled.
 */
#define QSPI_MWCR_MWMOD_SHIFT	0
#define QSPI_MWCR_MDD_SHIFT	1
#define QSPI_MWCR_MHS_SHIFT	2

#define QSPI_MWCR_MWMOD_SIZE 1
#define QSPI_MWCR_MDD_SIZE 1
#define QSPI_MWCR_MHS_SIZE 1

/* Register: SER
 * The register enables the individual slave select output lines from the qspi master.
 * You cannot write to this register when qspi is busy and when SSIC_EN = 1.
 */
#define QSPI_SER_SER_SHIFT	0
#define QSPI_SER_SER_SIZE 2

/* Register: BAUDR
 * The register derives the frequency of the serial clock that regulates the data transfer.
 * It is impossible to write to this register when the qspi is enabled.
 */
#define QSPI_BAUDR_SCKDV_SHIFT	1

#define QSPI_BAUDR_SCKDV_SIZE 15

/* Register: TXFTLR
 * This register controls the threshold value for the transmit FIFO memory.
 */
#define QSPI_TXFTLR_TFT_SHIFT	0
#define QSPI_TXFTLR_TXFTHR_SHIFT	16

#define QSPI_TXFTLR_TFT_SIZE 8
#define QSPI_TXFTLR_TXFTHR_SIZE 8

/* Register: RXFTLR
 * This register controls the threshold value for the receive FIFO memory.
 */
#define QSPI_RXFTLR_RFT_SHIFT	0

#define QSPI_RXFTLR_RFT_SIZE 8

/* Register: TXFLR
 * This register contains the number of valid data entries in the transmit FIFO memory.
 */
#define QSPI_TXFLR_TXTFL_SHIFT	0
#define QSPI_TXFLR_TXTFL_SIZE 9

/* Register: RXFLR
 * This register contains the number of valid data entries in the receive FIFO memory.
 */
#define QSPI_RXFLR_RXTFL_SHIFT	0

#define QSPI_RXFLR_RXTFL_SIZE 9

/* Register: SR
 * This is a read-only register used to indicate the current transfer status,
 * FIFO status, and any transmission/reception errors that may have occurred.
 */
#define QSPI_SR_BUSY_SHIFT	0
#define QSPI_SR_TFNF_SHIFT	1
#define QSPI_SR_TFE_SHIFT	2
#define QSPI_SR_RFNE_SHIFT	3
#define QSPI_SR_RFF_SHIFT	4
#define QSPI_SR_TXE_SHIFT	5
#define QSPI_SR_DCOL_SHIFT	6

#define QSPI_SR_BUSY_SIZE 1
#define QSPI_SR_TFNF_SIZE 1
#define QSPI_SR_TFE_SIZE 1
#define QSPI_SR_RFNE_SIZE 1
#define QSPI_SR_RFF_SIZE 1
#define QSPI_SR_TXE_SIZE 1
#define QSPI_SR_DCOL_SIZE 1

/* Register: IMR
 * This read/write register masks or enables all interrupts generated by the qspi.
 */
#define QSPI_IMR_TXEIM_SHIFT	0
#define QSPI_IMR_TXOIM_SHIFT	1
#define QSPI_IMR_RXUIM_SHIFT	2
#define QSPI_IMR_RXOIM_SHIFT	3
#define QSPI_IMR_RXFIM_SHIFT	4
#define QSPI_IMR_MSTIM_SHIFT	5
#define QSPI_IMR_XRXOIM_SHIFT	6

#define QSPI_IMR_TXEIM_SIZE 1
#define QSPI_IMR_TXOIM_SIZE 1
#define QSPI_IMR_RXUIM_SIZE 1
#define QSPI_IMR_RXOIM_SIZE 1
#define QSPI_IMR_RXFIM_SIZE 1
#define QSPI_IMR_MSTIM_SIZE 1
#define QSPI_IMR_XRXOIM_SIZE 1

/* Register: ISR
 * This register reports the status of the qspi interrupts after they have been masked.
 */
#define QSPI_ISR_TXEIS_SHIFT	0
#define QSPI_ISR_TXOIS_SHIFT	1
#define QSPI_ISR_RXUIS_SHIFT	2
#define QSPI_ISR_RXOIS_SHIFT	3
#define QSPI_ISR_RXFIS_SHIFT	4
#define QSPI_ISR_MSTIS_SHIFT	5
#define QSPI_ISR_XRXOIS_SHIFT	6

#define QSPI_ISR_TXEIS_SIZE 1
#define QSPI_ISR_TXOIS_SIZE 1
#define QSPI_ISR_RXUIS_SIZE 1
#define QSPI_ISR_RXOIS_SIZE 1
#define QSPI_ISR_RXFIS_SIZE 1
#define QSPI_ISR_MSTIS_SIZE 1
#define QSPI_ISR_XRXOIS_SIZE 1

/* Register: RISR
 * Raw Interrupt Status Register
 */
#define QSPI_RISR_TXEIR_SHIFT	0
#define QSPI_RISR_TXOIR_SHIFT	1
#define QSPI_RISR_RXUIR_SHIFT	2
#define QSPI_RISR_RXOIR_SHIFT	3
#define QSPI_RISR_RXFIR_SHIFT	4
#define QSPI_RISR_MSTIR_SHIFT	5
#define QSPI_RISR_XRXOIR_SHIFT	6

#define QSPI_RISR_TXEIR_SIZE 1
#define QSPI_RISR_TXOIR_SIZE 1
#define QSPI_RISR_RXUIR_SIZE 1
#define QSPI_RISR_RXOIR_SIZE 1
#define QSPI_RISR_RXFIR_SIZE 1
#define QSPI_RISR_MSTIR_SIZE 1
#define QSPI_RISR_XRXOIR_SIZE 1

/* Register: TXOICR
 * Transmit FIFO Overflow Interrupt Clear Register
 */
#define QSPI_TXOICR_TXOICR_SHIFT	0

#define QSPI_TXOICR_TXOICR_SIZE 1

/* Register: RXOICR
 * Receive FIFO Overflow Interrupt Clear Register
 */
#define QSPI_RXOICR_RXOICR_SHIFT	0

#define QSPI_RXOICR_RXOICR_SIZE 1

/* Register: RXUICR
 * Receive FIFO Underflow Interrupt Clear Register
 */
#define QSPI_RXUICR_RXUICR_SHIFT	0

#define QSPI_RXUICR_RXUICR_SIZE 1

/* Register: MSTICR
 * Multi-Master Interrupt Clear Register
 */
#define QSPI_MSTICR_MSTICR_SHIFT	0

#define QSPI_MSTICR_MSTICR_SIZE 1

/* Register: ICR
 * Interrupt Clear Register
 */
#define QSPI_ICR_ICR_SHIFT	0
#define QSPI_ICR_ICR_SIZE 1

/* Register: DMACR
 * DMA Control Register
 */
#define QSPI_DMACR_RDMAE_SHIFT	0
#define QSPI_DMACR_TDMAE_SHIFT	1

#define QSPI_DMACR_RDMAE_SIZE 1
#define QSPI_DMACR_TDMAE_SIZE 1

/* Register: DMATDLR
 * DMA Transmit Data Level
 */
#define QSPI_DMATDLR_DMATDL_SHIFT	0

#define QSPI_DMATDLR_DMATDL_SIZE 8

/* Register: DMARDLR
 * DMA Receive Data Level
 */
#define QSPI_DMARDLR_DMARDL_SHIFT	0

#define QSPI_DMARDLR_DMARDL_SIZE 8

/* Register: IDR
 * This register contains the peripherals identification code
 */
#define QSPI_IDR_IDCODE_SHIFT	0
#define QSPI_IDR_IDCODE_SIZE 32

/* Register: SSIC_VERSION_ID
 * This read-only register stores the specific qspi component version.
 */
#define QSPI_SSIC_VERSION_ID_SSIC_COMP_VERSION_SHIFT	0

#define QSPI_SSIC_VERSION_ID_SSIC_COMP_VERSION_SIZE 32


/* Register: DR0
 * Data Register 1. When the register is read, data in the receive FIFO buffer is accessed.
 * When it is written to, data are moved into the transmit FIFO buffer
 */
#define QSPI_DR0_DR_SHIFT	0
#define QSPI_DR0_DR_SIZE 32

/* Register: RX_SAMPLE_DELAY
 * This register control the number of ssi_clk cycles that are delayed (from the default sample time)
 * before the actual sample of the rxd input occurs
 */
#define QSPI_RX_SAMPLE_DELAY_RSD_SHIFT	0
#define QSPI_RX_SAMPLE_DELAY_SE_SHIFT	16

#define QSPI_RX_SAMPLE_DELAY_RSD_SIZE 8
#define QSPI_RX_SAMPLE_DELAY_SE_SIZE 1

/* Register: SPI_CTRLR0
 * This register is used to control the serial data transfer in enhanced SPI mode of operation
 */
#define QSPI_SPI_CTRLR0_TRANS_TYPE_SHIFT	0
#define QSPI_SPI_CTRLR0_ADDR_L_SHIFT	2
#define QSPI_SPI_CTRLR0_XIP_MD_BIT_EN_SHIFT	7
#define QSPI_SPI_CTRLR0_INST_L_SHIFT	8
#define QSPI_SPI_CTRLR0_WAIT_CYCLES_SHIFT	11
#define QSPI_SPI_CTRLR0_SPI_DDR_EN_SHIFT	16
#define QSPI_SPI_CTRLR0_INST_DDR_EN_SHIFT	17
#define QSPI_SPI_CTRLR0_SPI_RXDS_EN_SHIFT	18
#define QSPI_SPI_CTRLR0_XIP_DFS_HC_SHIFT	19
#define QSPI_SPI_CTRLR0_XIP_INST_EN_SHIFT	20
#define QSPI_SPI_CTRLR0_SSIC_XIP_CONT_XFER_EN_SHIFT	21
#define QSPI_SPI_CTRLR0_SPI_DM_EN_SHIFT	24
#define QSPI_SPI_CTRLR0_SPI_RXDS_SIG_EN_SHIFT	25
#define QSPI_SPI_CTRLR0_XIP_MBL_SHIFT	26
#define QSPI_SPI_CTRLR0_XIP_PREFETCH_EN_SHIFT	29
#define QSPI_SPI_CTRLR0_CLK_STRETCH_EN_SHIFT	30

#define QSPI_SPI_CTRLR0_TRANS_TYPE_SIZE 2
#define QSPI_SPI_CTRLR0_ADDR_L_SIZE 4
#define QSPI_SPI_CTRLR0_XIP_MD_BIT_EN_SIZE 1
#define QSPI_SPI_CTRLR0_INST_L_SIZE 2
#define QSPI_SPI_CTRLR0_WAIT_CYCLES_SIZE 5
#define QSPI_SPI_CTRLR0_SPI_DDR_EN_SIZE 1
#define QSPI_SPI_CTRLR0_INST_DDR_EN_SIZE 1
#define QSPI_SPI_CTRLR0_SPI_RXDS_EN_SIZE 1
#define QSPI_SPI_CTRLR0_XIP_DFS_HC_SIZE 1
#define QSPI_SPI_CTRLR0_XIP_INST_EN_SIZE 1
#define QSPI_SPI_CTRLR0_SSIC_XIP_CONT_XFER_EN_SIZE 1
#define QSPI_SPI_CTRLR0_SPI_DM_EN_SIZE 1
#define QSPI_SPI_CTRLR0_SPI_RXDS_SIG_EN_SIZE 1
#define QSPI_SPI_CTRLR0_XIP_MBL_SIZE 2
#define QSPI_SPI_CTRLR0_XIP_PREFETCH_EN_SIZE 1
#define QSPI_SPI_CTRLR0_CLK_STRETCH_EN_SIZE 1

/* Register: XIP_MODE_BITS
 * This register carries the mode bits which are sent in the XIP mode of operation after address phase
 */
#define QSPI_XIP_MODE_BITS_XIP_MD_BITS_SHIFT	0

#define QSPI_XIP_MODE_BITS_XIP_MD_BITS_SIZE 16

/* Register: XIP_INCR_INST
 * This register is used to store the instruction op-code to be used in INCR transactions
 * when the same is requested on AHB interface
 */
#define QSPI_XIP_INCR_INST_INCR_INST_SHIFT	0

#define QSPI_XIP_INCR_INST_INCR_INST_SIZE 16

/* Register: XIP_WRAP_INST
 * This register is used to store the instruction op-code to be used in WRAP transactions
 * when the same is requested on AHB interface
 */
#define QSPI_XIP_WRAP_INST_WRAP_INST_SHIFT	0

#define QSPI_XIP_WRAP_INST_WRAP_INST_SIZE 16

/* Register: XIP_CNT_TIME_OUT
 * XIP count down register for continuous mode.
 * The counter is used to de-select the slave during continuous transfer mode.
 */
#define QSPI_XIP_CNT_TIME_OUT_XTOC_SHIFT	0

#define QSPI_XIP_CNT_TIME_OUT_XTOC_SIZE 8

/* Bit fields in CTRLR0 */
#define SPI_DFS_OFFSET			0

#define SPI_FRF_OFFSET			4
#define SPI_FRF_SPI			0x0
#define SPI_FRF_SSP			0x1
#define SPI_FRF_MICROWIRE		0x2
#define SPI_FRF_RESV			0x3

#define SPI_MODE_OFFSET			6
#define SPI_SCPH_OFFSET			6
#define SPI_SCOL_OFFSET			7

#define SPI_TMOD_OFFSET			8
#define SPI_TMOD_MASK			(0x3 << SPI_TMOD_OFFSET)
#define	SPI_TMOD_TR			0x0		/* xmit & recv */
#define SPI_TMOD_TO			0x1		/* xmit only */
#define SPI_TMOD_RO			0x2		/* recv only */
#define SPI_TMOD_EPROMREAD		0x3		/* eeprom read mode */

/* Bit fields in CTRLR1 */
#define SPI_NDF_MASK			GENMASK(15, 0)

/* Bit fields in SR, 7 bits */
#define SR_MASK				0x7f		/* cover 7 bits */
#define SR_BUSY				(1 << 0)
#define SR_TF_NOT_FULL			(1 << 1)
#define SR_TF_EMPT			(1 << 2)
#define SR_RF_NOT_EMPT			(1 << 3)
#define SR_RF_FULL			(1 << 4)
#define SR_TX_ERR			(1 << 5)
#define SR_DCOL				(1 << 6)

/* Bit fields in ISR, IMR, RISR, 7 bits */
#define SPI_INT_TXEI			(1 << 0)
#define SPI_INT_TXOI			(1 << 1)
#define SPI_INT_RXUI			(1 << 2)
#define SPI_INT_RXOI			(1 << 3)
#define SPI_INT_RXFI			(1 << 4)
#define SPI_INT_MSTI			(1 << 5)

/* Bit fields in DMACR */
#define QSPI_DMA_RDMAE			(1 << 0)
#define QSPI_DMA_TDMAE			(1 << 1)

#define SPI_STANDARD_FORMAT 0
#define SPI_DUAL_FORMAT	 1
#define SPI_QUAD_FORMAT	 2

/* Slave Select Toggle Enable */
#define QSPI_TOGGLE_DISABLE	0
#define QSPI_TOGGLE_EN		1

/* Shift Register Loop */
#define QSPI_NORMAL_MODE	0
#define QSPI_TEST_MODE		1

#define QSPI_SLV_OE_ENABLE	0
#define QSPI_SLV_OE_DISABLE 1

#define QSPI_TX_RX		0
#define QSPI_TX_ONLY	1
#define QSPI_RX_ONLY	2
#define QSPI_EEPROM		3

#define QSPI_DFS_8BITS	 0x7
#define QSPI_DFS_16BITS	 0xf
#define QSPI_DFS_32BITS	 0x1f

#define QSPI_SER_SS0_EN		(1 << 0)
#define QSPI_SER_SS1_EN		(1 << 1)
#define QSPI_SER_SS2_EN		(1 << 2)

#define QSPI_SLV_TOGGLE_DISABLE	0
#define QSPI_SLV_TOGGLE_ENABLE	1

#define TXFIFO_NOTEMPTY	0
#define TXFIFO_EMPTY		1

#define TXFIFO_FULL		0
#define TXFIFO_NOTFULL	1

#define IDLE	 0
#define BUSY	 1

#define RXFIFO_EMPTY		0
#define RXFIFO_NOTEMPTY	1

#define RXFIFO_NOTFULL	0
#define RXFIFO_FULL		1

#define QSPI_BUSY 0x1		/* SSI Busy Flag */
#define QSPI_TFNF 0x2		/* Transmit FIFO Not Full */
#define QSPI_TFE	0x4		/* Transmit FIFO Empty */
#define QSPI_RFNE 0x8		/* Receive FIFO Not Empty */
#define QSPI_RFF	0x10		/* Receive FIFO Full */
#define QSPI_TXE	0x20		/* Transmission Error */
#define QSPI_DCOL 0x40		/* Data Collision Error */

#define QSPI_TXEIM	1 << 0	/* Transmit FIFO Empty Interrupt */
#define QSPI_TXOIM	1 << 1	/* Transmit FIFO Overflow Interrupt	*/
#define QSPI_RXUIM	1 << 2	/* Receive FIFO Underflow Interrupt */
#define QSPI_RXOIM	1 << 3	/* Receive FIFO Overflow Interrupt */
#define QSPI_RXFIM	1 << 4	/* Receive FIFO Full Interrupt */
// QSPI_MSTIM	1 << 5	/* Multi-Master Contention Interrupt */

#define QSPI_XRXOIM	1 << 6	/* XIP Receive FIFO Overflow Interrupt Mask */
#define QSPI_TXUIM	1 << 7	/* Transmit FIFO Underflow Interrupt Mask */
#define QSPI_AXIEM	1 << 8	/* AXI Error Interrupt Mask */
#define QSPI_SPITEM	1 << 10	/* SPI Transmit Error Interrupt Mask */
#define QSPI_DONEM	1 << 11	/* SSI Done Interrupt Mask */

#define TXEIS			0x1		/* Transmit FIFO Empty Interrupt Status */
#define TXOIS			0x2		/* Transmit FIFO Overflow Interrupt Status */
#define RXUIS			0x4		/* Receive FIFO Underflow Interrupt Status */
#define RXOIS			0x8		/* Receive FIFO Overflow Interrupt Status */
#define RXFIS			0x10		/* Receive FIFO Full Interrupt Status */
// MSTIS			0x20		/* Multi-Master Contention Interrupt Status */

#define XRXOIS			0x40		/* XIP Receive FIFO Overflow Interrupt Status */
#define TXUIS			0x80		/* Transmit FIFO Underflow Interrupt Status */
#define AXIES			0x100	 /* AXI Error Interrupt Status */
#define SPITES			0x400	 /* SPI Transmit Error Interrupt */
#define DONES			0x800	 /* SSI Done Interrupt Status */

#define QSPI_PostiveEdgeSampling 0
#define QSPI_NegativeEdgeSampling 1

#define QSPI_DisableClockStretch 0
#define QSPI_EnableClockStretch	1

#define QSPI_DisableXipPrefetch 0
#define QSPI_EnableXipPrefetch	1

#define QSPI_MBL_2 0
#define QSPI_MBL_4 1
#define QSPI_MBL_8 2
#define QSPI_MBL_16 3

#define QSPI_DisableXipContTrans 0
#define QSPI_EnableXipContTrans	1

#define QSPI_XipInstPhaseDisable 0
#define QSPI_XipInstPhaseEnable	1

#define QSPI_XipDfsChange 0
#define QSPI_XipDfsFix	1

#define QSPI_InstDdrDisable 0
#define QSPI_InstDdrEnable	1

#define QSPI_SpiDdrDisable 0
#define QSPI_SpiDdrEnable	1

#define QSPI_INST_L0 0
#define QSPI_INST_L4 1
#define QSPI_INST_L8 2
#define QSPI_INST_L16 3

#define QSPI_XipModeBitDisable 0
#define QSPI_XipModeBitEnable	1

#define QSPI_ADDR_L0 0x0
#define QSPI_ADDR_L8 0x2
#define QSPI_ADDR_L32 0x8
#define QSPI_ADDR_L48 0xc
#define QSPI_ADDR_L60 0xf

#define QSPI_TT0 0	/* Instruction and Address will be sent in Standard SPI Mode */
#define QSPI_TT1 1	/* Instruction will be sent in Standard SPI Mode and Address will be sent in the mode specified by CTRLR0.SPI_FRF */
#define QSPI_TT2 2	/* Both Instruction and Address will be sent in the mode specified by SPI_FRF */

#define	QSPI_DFS_8BITS	0x7
#define	QSPI_DFS_16BITS	0xf
#define	QSPI_DFS_32BITS	0x1f

// Register: CTRLR0
#define DFS_SHIFT	0
#define TMOD_SHIFT	10
#define SLV_OE_SHIFT	12
#define SRL_SHIFT	13
#define SSTE_SHIFT	14
#define CFS_SHIFT	16
#define SPI_FRF_SHIFT	22

#define DFS_MASK	GENMASK(7, 0)
#define TMOD_MASK	GENMASK(7, 0)
#define SLV_OE_MASK	GENMASK(7, 0)
#define SRL_MASK	GENMASK(7, 0)
#define SSTE_MASK	GENMASK(7, 0)
#define CFS_MASK	GENMASK(7, 0)
#define SPI_FRF_MASK	GENMASK(7, 0)

// Register: MWCR
#define MWMOD_SHIFT	0
#define MDD_SHIFT	1
#define MHS_SHIFT	2

#define MWMOD_MASK	0
#define MDD_MASK	1
#define MHS_MASK	2

// Register: BAUDR
#define SCKDV_SHIFT	1

#define SCKDV_MASK	1

// Register: TXFTLR
#define TFT_SHIFT	0
#define TXFTHR_SHIFT	16

#define TFT_MASK	0
#define TXFTHR_MASK	16

// Register: SR
#define BUSY_SHIFT	0
#define TFNF_SHIFT	1
#define TFE_SHIFT	2
#define RFNE_SHIFT	3
#define RFF_SHIFT	4
#define TXE_SHIFT	5
#define DCOL_SHIFT	6

#define BUSY_MASK	0
#define TFNF_MASK	1
#define TFE_MASK	2
#define RFNE_MASK	3
#define RFF_MASK	4
#define TXE_MASK	5
#define DCOL_MASK	6

// Register: IMR
#define TXEIM_SHIFT	0
#define TXOIM_SHIFT	1
#define RXUIM_SHIFT	2
#define RXOIM_SHIFT	3
#define RXFIM_SHIFT	4
#define MSTIM_SHIFT	5
#define XRXOIM_SHIFT	6

#define TXEIM_MASK	0
#define TXOIM_MASK	1
#define RXUIM_MASK	2
#define RXOIM_MASK	3
#define RXFIM_MASK	4
#define MSTIM_MASK	5
#define XRXOIM_MASK	6

// Register: ISR
#define TXEIS_SHIFT	0
#define TXOIS_SHIFT	1
#define RXUIS_SHIFT	2
#define RXOIS_SHIFT	3
#define RXFIS_SHIFT	4
#define MSTIS_SHIFT	5
#define XRXOIS_SHIFT	6

#define TXEIS_MASK	0
#define TXOIS_MASK	1
#define RXUIS_MASK	2
#define RXOIS_MASK	3
#define RXFIS_MASK	4
#define MSTIS_MASK	5
#define XRXOIS_MASK	6

// Register: RISR
#define TXEIR_SHIFT	0
#define TXOIR_SHIFT	1
#define RXUIR_SHIFT	2
#define RXOIR_SHIFT	3
#define RXFIR_SHIFT	4
#define MSTIR_SHIFT	5
#define XRXOIR_SHIFT	6

#define TXEIR_MASK	0
#define TXOIR_MASK	1
#define RXUIR_MASK	2
#define RXOIR_MASK	3
#define RXFIR_MASK	4
#define MSTIR_MASK	5
#define XRXOIR_MASK	6

// Register: DMACR
#define RDMAE_SHIFT	0
#define TDMAE_SHIFT	1

#define RDMAE_MASK	0
#define TDMAE_MASK	1

// Register: RX_SAMPLE_DELAY
#define RSD_SHIFT	0
#define SE_SHIFT	16

#define RSD_MASK	0
#define SE_MASK	16

// Register: SPI_CTRLR0
#define TRANS_TYPE_SHIFT	0
#define ADDR_L_SHIFT	2
#define XIP_MD_BIT_EN_SHIFT	7
#define INST_L_SHIFT	8
#define WAIT_CYCLES_SHIFT	11
#define SPI_DDR_EN_SHIFT	16
#define INST_DDR_EN_SHIFT	17
#define SPI_RXDS_EN_SHIFT	18
#define XIP_DFS_HC_SHIFT	19
#define XIP_INST_EN_SHIFT	20
#define SSIC_XIP_CONT_XFER_EN_SHIFT	21
#define SPI_DM_EN_SHIFT	24
#define SPI_RXDS_SIG_EN_SHIFT	25
#define XIP_MBL_SHIFT	26
#define RESERVED_28_SHIFT	28
#define XIP_PREFETCH_EN_SHIFT	29
#define CLK_STRETCH_EN_SHIFT	30


#define TRANS_TYPE_MASK	GENMASK(7, 0)
#define ADDR_L_MASK	GENMASK(7, 0)
#define XIP_MD_BIT_EN_MASK	GENMASK(7, 0)
#define INST_L_MASK	GENMASK(7, 0)
#define WAIT_CYCLES_MASK	GENMASK(7, 0)
#define SPI_DDR_EN_MASK	GENMASK(7, 0)
#define INST_DDR_EN_MASK	GENMASK(7, 0)
#define SPI_RXDS_EN_MASK	GENMASK(7, 0)
#define XIP_DFS_HC_MASK	GENMASK(7, 0)
#define XIP_INST_EN_MASK	GENMASK(7, 0)
#define SSIC_XIP_CONT_XFER_EN_MASK	GENMASK(7, 0)
#define SPI_DM_EN_MASK	GENMASK(7, 0)
#define SPI_RXDS_SIG_EN_MASK	GENMASK(7, 0)
#define XIP_MBL_MASK	GENMASK(7, 0)
#define RESERVED_28_MASK	GENMASK(7, 0)
#define XIP_PREFETCH_EN_MASK	GENMASK(7, 0)
#define CLK_STRETCH_EN_MASK	GENMASK(7, 0)

#define SPI_DDR_EN 1
#define SPI_DDR_DISABLE 0

#define INST_DDR_EN 1
#define INST_DDR_DISABLE 0

#define SPI_WAIT_RETRIES		50
#define SPI_BUF_SIZE \
	(sizeof_field(struct spi_mem_op, cmd.opcode) + \
	 sizeof_field(struct spi_mem_op, addr.val) + 256)

struct al_qspi_info {
	u8 tmod;
	u8 dfs;
	u32 ndf;
	u32 freq;
	u8 addr_length;
	u8 inst_length;
	u8 trans_type;
	u16 wait_cycles;
	u8 spi_frame_format;
	u8 clock_stretch;
	u8 slv_toggleenum;
	u8 slv_sel;
};

struct al_qspi_cfg {
	u32 ctrlr0;
	u32 mwcr;
	u32 baudr;
	u32 txftlr;
	u32 sr;
	u32 imr;
	u32 isr;
	u32 risr;
	u32 dmacr;
	u32 rx_sample_delay;
	u32 spi_ctrlr0;
};

struct al_qspi {
	void __iomem *regs;
	struct al_qspi_cfg cfg;
	struct spi_controller	*master;
	struct clk		*clk;
	struct clk		*pclk;
	struct reset_control *rstc;

	/* Current message transfer state info */
	void			*tx;
	unsigned int	tx_len;
	void			*rx;
	unsigned int	rx_len;
	struct al_qspi_info info;

	u8			buf[SPI_BUF_SIZE];
	// int			dma_mapped;
	u8			n_bytes;	/* current is a 1/2 bytes op */
	irqreturn_t		(*transfer_handler)(struct al_qspi *qspi);
	u32			current_freq;	/* frequency in hz */
	u32			cur_rx_sample_dly;
	u32			def_rx_sample_dly_ns;

	/* Custom memory operations */
	struct spi_controller_mem_ops mem_ops;

	u32			fifo_len;	/* depth of the FIFO buffer */
	u32			max_mem_freq;	/* max mem-ops bus freq */
	u32			max_freq;	/* max bus freq supported */

	// u32			reg_io_width;	/* DR I/O width in bytes */
	// u16			bus_num;
	u16			num_cs;		/* supported slave numbers */

};

static inline u32 al_readl(struct al_qspi *qspi, u32 offset)
{
	//return __raw_readl(qspi->regs + offset);
	return readl_relaxed(qspi->regs + offset);
}

static inline void al_writel(struct al_qspi *qspi, u32 offset, u32 val)
{
	//__raw_writel(val, qspi->regs + offset);
	writel_relaxed(val, qspi->regs + offset);
}

#define al_reg32_get_bits(qspi, reg_addr, shift, size)	(((al_readl(qspi, reg_addr)) & GENMASK((size+shift),(shift))) >> (shift))
#define al_reg32_get_bit(reg_addr, shift)			(bool)(((al_readl(qspi, reg_addr)) & (BIT(shift))) >> (shift))

#define al_reg32_set_bits(qspi, reg_addr, shift, size, value)	do {											 \
	al_writel(qspi, reg_addr, (al_readl(qspi, reg_addr) & (~GENMASK((size+shift), (shift)))) | ((value) << (shift))); \
} while (0)

#define al_reg32_set_bit(qspi,reg_addr, shift, value)		 do {										\
	al_writel(qspi, reg_addr, (al_readl(qspi, reg_addr) & (~(BIT(shift)))) | ((value) << (shift))); \
} while (0)

static inline void al_read_data(struct al_qspi *qspi, void *data)
{
	switch (qspi->n_bytes) {
	case 1:
		*(u8 *)data = (u8)al_readl(qspi, AL_QSPI_DR0_OFFSET);
		break;
	case 2:
		*(u16 *)data = (u16)al_readl(qspi, AL_QSPI_DR0_OFFSET);
		break;
	case 4:
		*(u32 *)data = al_readl(qspi, AL_QSPI_DR0_OFFSET);
		break;
	default:
		break;
	}
}

static inline void al_write_data(struct al_qspi *qspi, void *data)
{
	switch (qspi->n_bytes) {
	case 1:
		return al_writel(qspi, AL_QSPI_DR0_OFFSET, *(u8 *)data);
		break;
	case 2:
		return al_writel(qspi, AL_QSPI_DR0_OFFSET, *(u16 *)data);
		break;
	case 4:
		return al_writel(qspi, AL_QSPI_DR0_OFFSET, *(u32 *)data);
		break;
	default:
		break;
	}
}

static inline void qspi_enable_chip(struct al_qspi *qspi, int enable)
{
	al_writel(qspi, AL_QSPI_SSIENR_OFFSET, (enable ? 1 : 0));
}

static inline void qspi_set_clk(struct al_qspi *qspi, u16 div)
{
	al_writel(qspi, AL_QSPI_BAUDR_OFFSET, div);
}

/* Disable IRQ bits */
static inline void qspi_mask_intr(struct al_qspi *qspi, u32 mask)
{
	u32 new_mask;

	new_mask = al_readl(qspi, AL_QSPI_IMR_OFFSET) & ~mask;
	al_writel(qspi, AL_QSPI_IMR_OFFSET, new_mask);
}

/* Enable IRQ bits */
static inline void qspi_umask_intr(struct al_qspi *qspi, u32 mask)
{
	u32 new_mask;

	new_mask = al_readl(qspi, AL_QSPI_IMR_OFFSET) | mask;
	al_writel(qspi, AL_QSPI_IMR_OFFSET, new_mask);
}

/*
 * This disables the SPI controller, interrupts, clears the interrupts status
 * and CS, then re-enables the controller back. Transmit and receive FIFO
 * buffers are cleared when the device is disabled.
 */
static inline void qspi_reset_chip(struct al_qspi *qspi)
{
	qspi_enable_chip(qspi, 0);
	qspi_mask_intr(qspi, 0xff);
	al_readl(qspi, AL_QSPI_ICR_OFFSET);
	al_writel(qspi, AL_QSPI_SER_OFFSET, 0);
	qspi_enable_chip(qspi, 1);
}

static inline void qspi_shutdown_chip(struct al_qspi *qspi)
{
	qspi_enable_chip(qspi, 0);
	qspi_set_clk(qspi, 0);
}

#endif /* AL_QSPI_HEADER_H */
