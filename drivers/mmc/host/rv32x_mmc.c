// SPDX-License-Identifier: GPL-2.0-only
/*
 *  rv32x/mmc.c
 *
 *  Copyright by Michał Mirosław, 2008-2009
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include "rv32x-mmc.h"

#define RV32X_SPI_DATA_BASE    0x0
#define RV32X_SPI_STATUS    0x200
#define RV32X_SPI_OPERATION    0x210
#define RV32X_SPI_ADDR    0x214

static int rv32x_mmc_receive(struct rv32x_slot *slot, struct mmc_data *data)
{
	struct sg_mapping_iter miter;
	size_t len, blocks = data->blocks;
	int err = 0;

	/* TODO: I don't know how/if the hardware handles non-16B-boundary blocks
	 * except single 8B block */
	if (unlikely(data->blksz & 15 && (data->blocks != 1 || data->blksz != 8)))
		return -EINVAL;

	sg_miter_start(&miter, data->sg, data->sg_len, SG_MITER_TO_SG);

	rv32x_modify_port_8(slot, rv32x_MMC_CONFIG2_PORT,
		15, rv32x_MMC_C2_READ_PIO_SIZE_MASK);

	rv32x_mmc_fifo_hack(slot);

	while (blocks-- > 0) {
		len = data->blksz;

		while (len >= 16) {
			err = rv32x_mmc_receive_pio(slot, &miter, 4);
			if (err)
				goto out;
			len -= 16;
		}

		if (!len)
			continue;

		rv32x_modify_port_8(slot, rv32x_MMC_CONFIG2_PORT,
			len - 1, rv32x_MMC_C2_READ_PIO_SIZE_MASK);

		len = (len >= 8) ? 4 : 2;
		err = rv32x_mmc_receive_pio(slot, &miter, len);
		if (err)
			goto out;
	}
out:
	sg_miter_stop(&miter);
	return err;
}

static int rv32x_mmc_send(struct rv32x_slot *slot, struct mmc_data *data)
{
	struct sg_mapping_iter miter;
	size_t len, blocks = data->blocks;
	int err = 0;

	/* TODO: I don't know how/if the hardware handles multiple
	 * non-16B-boundary blocks */
	if (unlikely(data->blocks > 1 && data->blksz & 15))
		return -EINVAL;

	sg_miter_start(&miter, data->sg, data->sg_len, SG_MITER_FROM_SG);

	rv32x_modify_port_8(slot, rv32x_MMC_CONFIG2_PORT,
		0, rv32x_MMC_C2_READ_PIO_SIZE_MASK);

	while (blocks-- > 0) {
		len = (data->blksz + 15) >> 4;
		do {
			if (!(rv32x_read_port_8(slot, rv32x_MMC_STATUS2_PORT)
			    & rv32x_MMC_S2_FIFO_EMPTY)) {
				err = rv32x_wait_for_event(slot,
					rv32x_MMC_S1_PIO_TRANSFER_DONE);
				if (err)
					goto out;
			}
			rv32x_sg_dwiter_read_to_io(&miter,
				slot->iobase + rv32x_MMC_DATA_PORT, 4);
		} while (--len);
	}
out:
	sg_miter_stop(&miter);
	return err;
}

static u16 rv32x_encode_cmd_flags(struct rv32x_mmc_reader *reader,
	struct mmc_command *cmd)
{
	unsigned int flags = cmd->flags;
	u16 cb_flags = 0;

	/* Windows driver returned 0 for commands for which no response
	 * is expected. It happened that there were only two such commands
	 * used: MMC_GO_IDLE_STATE and MMC_GO_INACTIVE_STATE so it might
	 * as well be a bug in that driver.
	 *
	 * Original driver set bit 14 for MMC/SD application
	 * commands. There's no difference 'on the wire' and
	 * it apparently works without it anyway.
	 */

	switch (flags & MMC_CMD_MASK) {
	case MMC_CMD_AC:	cb_flags = rv32x_MMC_CMD_AC;	break;
	case MMC_CMD_ADTC:	cb_flags = rv32x_MMC_CMD_ADTC;	break;
	case MMC_CMD_BC:	cb_flags = rv32x_MMC_CMD_BC;	break;
	case MMC_CMD_BCR:	cb_flags = rv32x_MMC_CMD_BCR;	break;
	}

	if (flags & MMC_RSP_BUSY)
		cb_flags |= rv32x_MMC_RSP_BUSY;

	cb_flags |= cmd->opcode << rv32x_MMC_CMD_CODE_SHIFT;

	if (cmd->data && (cmd->data->flags & MMC_DATA_READ))
		cb_flags |= rv32x_MMC_DATA_READ;

	if (flags & MMC_RSP_PRESENT) {
		/* Windows driver set 01 at bits 4,3 except for
		 * MMC_SET_BLOCKLEN where it set 10. Maybe the
		 * hardware can do something special about this
		 * command? The original driver looks buggy/incomplete
		 * anyway so we ignore this for now.
		 *
		 * I assume that 00 here means no response is expected.
		 */
		cb_flags |= rv32x_MMC_RSP_PRESENT;

		if (flags & MMC_RSP_136)
			cb_flags |= rv32x_MMC_RSP_136;
		if (!(flags & MMC_RSP_CRC))
			cb_flags |= rv32x_MMC_RSP_NO_CRC;
	}

	return cb_flags;
}

static void rv32x_receive_response(struct rv32x_slot *slot,
	struct mmc_command *cmd)
{
	unsigned rsp_opcode, wanted_opcode;

	/* Looks like final byte with CRC is always stripped (same as SDHCI) */
	if (cmd->flags & MMC_RSP_136) {
		u32 resp[4];

		resp[0] = rv32x_read_port_32(slot, rv32x_MMC_RESPONSE3_PORT);
		resp[1] = rv32x_read_port_32(slot, rv32x_MMC_RESPONSE2_PORT);
		resp[2] = rv32x_read_port_32(slot, rv32x_MMC_RESPONSE1_PORT);
		resp[3] = rv32x_read_port_32(slot, rv32x_MMC_RESPONSE0_PORT);
		rsp_opcode = resp[0] >> 24;

		cmd->resp[0] = (resp[0] << 8)|(resp[1] >> 24);
		cmd->resp[1] = (resp[1] << 8)|(resp[2] >> 24);
		cmd->resp[2] = (resp[2] << 8)|(resp[3] >> 24);
		cmd->resp[3] = (resp[3] << 8);
	} else {
		rsp_opcode = rv32x_read_port_32(slot, rv32x_MMC_RESPONSE1_PORT) & 0x3F;
		cmd->resp[0] = rv32x_read_port_32(slot, rv32x_MMC_RESPONSE0_PORT);
	}

	wanted_opcode = (cmd->flags & MMC_RSP_OPCODE) ? cmd->opcode : 0x3F;
	if (rsp_opcode != wanted_opcode)
		cmd->error = -EILSEQ;
}

static int rv32x_mmc_transfer_data(struct rv32x_slot *slot,
	struct mmc_data *data)
{
	int error, to;

	if (data->flags & MMC_DATA_READ)
		error = rv32x_mmc_receive(slot, data);
	else
		error = rv32x_mmc_send(slot, data);

	to = rv32x_wait_for_event(slot, rv32x_MMC_S1_DATA_TRANSFER_DONE);
	if (!error)
		error = to;

	if (!error)
		data->bytes_xfered = data->blksz * data->blocks;
	return error;
}

static int rv32x_mmc_write_block(struct rv32x_mmc* rv32x, struct mmc_data* data){
	struct scatterlist	*sg;
	unsigned		i;
	iowrite32(data->blk_addr,rv32x->regs+RV32X_SPI_ADDR);
	enum dma_data_direction	direction;
	int			multiple = (data->blocks > 1);

	direction = mmc_get_dma_dir(data);

	for_each_sg(data->sg, sg, data->sg_len, i){
		int			status = 0;
		dma_addr_t		dma_addr = 0;
		void			*kmap_addr;
		unsigned		length = sg->length;
		unsigned		size = 0;
		enum dma_data_direction	dir = direction;

		/* allow pio too; we don't allow highmem */
		kmap_addr = kmap(sg_page(sg));
		if (direction == DMA_TO_DEVICE)
			t->tx_buf = kmap_addr + sg->offset;
		else
			t->rx_buf = kmap_addr + sg->offset;

		/* transfer each block, and update request status */
		while (length) {
			size = min(length, data->blksz);

			if (direction == DMA_TO_DEVICE)
				status = mmc_spi_writeblock(host, t, timeout);
			else
				status = mmc_spi_readblock(host, t, timeout);
			if (status < 0)
				break;

			data->bytes_xfered += size;
			length -= size;

			if (!multiple)
				break;
		}

		/* discard mappings */
		if (direction == DMA_FROM_DEVICE)
			flush_kernel_dcache_page(sg_page(sg));
		kunmap(sg_page(sg));

		if (status < 0) {
			data->error = status;
			break;
		}
	}

	for(i=0;i<RV32X_SPI_STATUS;i+=4){
		iowrite32(data->,rv32x->regs+RV32X_SPI_ADDR);
	}
	iowrite32(data->blk_addr,rv32x->regs+RV32X_SPI_ADDR);
	iowrite32(data->blocks,rv32x->regs+RV32X_SPI_OPERATION);
}

static int rv32x_mmc_command(struct mmc_host *mmc, struct mmc_command *cmd)
{
	struct mmc_data *data = cmd->data;
	rv32x = mmc_priv(mmc);
	switch(cmd->opcode){
		case MMC_READ_SINGLE_BLOCK:
			rv32x_mmc_write_block();
		case MMC_WRITE_BLOCK:
		case MMC_READ_MULTIPLE_BLOCK:
		case MMC_WRITE_MULTIPLE_BLOCK:
		default:	 
	}
	

	if (data) {
		if (!rv32x_is_transfer_size_supported(data)) {
			data->error = -EINVAL;
			return -1;
		}
		rv32x_mmc_set_transfer_size(slot, data->blocks, data->blksz);
	}

	rv32x_wait_while_busy(slot, rv32x_MMC_S2_BUSY_20|rv32x_MMC_S2_BUSY_10);
	rv32x_write_port_16(slot, rv32x_MMC_CMD_TYPE_PORT, cb_cmd);
	rv32x_wait_while_busy(slot, rv32x_MMC_S2_BUSY_20);
	rv32x_write_port_32(slot, rv32x_MMC_CMD_PARAM_PORT, cmd->arg);
	rv32x_mmc_reset_events(slot);
	rv32x_wait_while_busy(slot, rv32x_MMC_S2_BUSY_20);
	rv32x_modify_port_8(slot, rv32x_MMC_CONFIG0_PORT, 0x01, 0);

	cmd->error = rv32x_wait_for_event(slot, rv32x_MMC_S1_COMMAND_SENT);
	if (cmd->error)
		return -1;

	if (cmd->flags & MMC_RSP_PRESENT) {
		rv32x_receive_response(slot, cmd);
		if (cmd->error)
			return -1;
	}

	if (data)
		data->error = rv32x_mmc_transfer_data(slot, data);
	return 0;
}

static void rv32x_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	if (!rv32x_mmc_command(mmc, mrq->cmd) && mrq->stop)
		rv32x_mmc_command(mmc, mrq->stop);
}

static void rv32x_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	//no io set CRs(like fmax, clock)
}

static int rv32x_mmc_get_ro(struct mmc_host *mmc)
{
	rerurn 0; //	read/write card
}

static int rv32x_mmc_get_cd(struct mmc_host *mmc)
{
	return 1;	//always present(TODO: add card existing cr)
}

static const struct mmc_host_ops rv32x_mmc_host = {
	.request = rv32x_mmc_request,
	.set_ios = rv32x_mmc_set_ios,
	.get_ro = rv32x_mmc_get_ro,
	.get_cd = rv32x_mmc_get_cd,
};

struct rv32x_mmc {
	void __iomem      *regs;
};

static int rv32x_mmc_init(struct platform_device *pdev)
{
	struct rv32x_mmc *rv32x;
	struct mmc_host *mmc;
	int err;
	u32 val;
	mmc = mmc_alloc_host(sizeof(*rv32x), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	platform_set_drvdata(pdev, mmc);
	mmc->ops = &rv32x_mmc_host;
	mmc->caps = MMC_CAP_SPI
	rv32x = mmc_priv(mmc);
	rv32x->regs = devm_platform_ioremap_resource(pdev, 0);

	err = mmc_add_host(mmc);
	if (unlikely(err))
		goto err_free_mmc;

	dev_dbg(rv32x_slot_dev(slot), "mmc_hostname is %s\n",
		mmc_hostname(mmc));
	return 0;

err_free_mmc:
	dev_dbg(rv32x_slot_dev(slot), "mmc_add_host() failed: %d\n", err);
	mmc_free_host(mmc);
	return err;
}

static int rv32x_mmc_exit(struct platform_device *pdev)
{
	struct rv32x_slot *slot = rv32x_pdev_to_slot(pdev);
	struct mmc_host *mmc = rv32x_slot_to_mmc(slot);
	mmc_free_host(mmc);
	return 0;
}

static struct platform_driver rv32x_mmc_driver = {
	.driver.name = "rv32x-mmc",
	.probe = rv32x_mmc_init,
	.remove = rv32x_mmc_exit,
};

module_platform_driver(rv32x_mmc_driver);

MODULE_AUTHOR("Michał Mirosław <mirq-linux@rere.qmqm.pl>");
MODULE_DESCRIPTION("ENE rv32x memory card reader driver - MMC/SD part");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rv32x-mmc");
