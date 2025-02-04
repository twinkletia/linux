
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>

#include <linux/bio.h>
#include <linux/dma-mapping.h>
#include <linux/printk.h>
#include <linux/io.h>
#include <linux/of.h>

#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h> /* for R1_SPI_* bit values */

#include <linux/platform_device.h>

#define RV32X_MMC_DATA_BASE 0x0
#define RV32X_MMC_STATUS 0x200
#define RV32X_MMC_R1STAT 0x204
#define RV32X_MMC_RESP 0x208
#define RV32X_MMC_CMD 0x20C
#define RV32X_MMC_ARG 0x210
#define RV32X_MMC_CMDINFO 0x214
#define RV32X_MMC_DATA_RESP 0x218
#define INITED 0x1
#define IDLE 0x2
#define EXEC 0x4
#define INTR_EN 0x8
#define OK (INITED | IDLE)
#define MMC_RECIEVE_BYTES(u) (u << 16)

struct rv32x_mmc {
	void __iomem *regbase;
	void *kmap_addr;
	unsigned blk_cnt;
	unsigned transfer_len;
};

static inline struct mmc_command *data_to_cmd(struct mmc_data **);
static void rv32x_mmc_read_block(struct rv32x_mmc *, struct mmc_request *);
static int rv32x_mmc_write_block(struct rv32x_mmc *, struct mmc_request *);
static int rv32x_mmc_transfer_data(struct rv32x_mmc *, struct mmc_request *);
static void rv32x_mmc_do_command(struct rv32x_mmc *, struct mmc_command *, u32);

static inline struct mmc_command *data_to_cmd(struct mmc_data **data_ptr)
{
	return container_of(data_ptr, struct mmc_command, data);
}

static void rv32x_mmc_read_block(struct rv32x_mmc *rv32x,
				 struct mmc_request *mrq)
{
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	unsigned rw_flag =
		((data->flags & (MMC_DATA_READ | MMC_DATA_WRITE)) >> 8);
	unsigned i = 0;
	unsigned *memaddr = rv32x->kmap_addr;
	rv32x_mmc_do_command(rv32x, cmd,
			     (MMC_RECIEVE_BYTES(data->blksz) |
			      mmc_spi_resp_type(cmd) | mmc_cmd_type(cmd) |
			      rw_flag));
	while (i < rv32x->transfer_len) {
		*memaddr++ = ioread32(rv32x->regbase + RV32X_MMC_DATA_BASE + i);
		i += 4;
	}
}

static int rv32x_mmc_write_block(struct rv32x_mmc *rv32x,
				 struct mmc_request *mrq)
{
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	unsigned rw_flag =
		((data->flags & (MMC_DATA_READ | MMC_DATA_WRITE)) >> 8);
	unsigned i = 0;
	unsigned *memaddr = rv32x->kmap_addr;
	unsigned error = 0;
	while (i < rv32x->transfer_len) {
		iowrite32(*memaddr++, rv32x->regbase + RV32X_MMC_DATA_BASE + i);
		i += 4;
	}
	rv32x_mmc_do_command(rv32x, cmd,
			     (mmc_spi_resp_type(cmd) | mmc_cmd_type(cmd) |
			      rw_flag));
	error = ioread32(rv32x->regbase + RV32X_MMC_DATA_RESP);
	if ((error & 0xF) == 0x5) {
		return 0;
	} else {
		pr_debug("dataError: %08x", error);
		return -EILSEQ;
	}
}

static int rv32x_mmc_transfer_data(struct rv32x_mmc *rv32x,
				   struct mmc_request *mrq)
{
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	struct scatterlist *sg;
	unsigned i;
	int multiple = (data->blocks > 1);
	unsigned length = 0;
	unsigned size = 0;

	//	pr_debug("blocks:%d", data->blocks);
	//	pr_debug("mrq->data->blk_addr:%d", data->blk_addr);
	//	pr_debug("mrq->cmd->arg:%d", cmd->arg);
	rv32x->blk_cnt = 0;

	for_each_sg (data->sg, sg, data->sg_len, i) {
		length = sg->length;
		size = 0;
		//	pr_debug("sg length:%d", length);
		//	pr_debug("sg offset:%08x", sg->offset);
		/* allow pio too; we don't allow highmem */
		rv32x->kmap_addr = sg_virt(sg);

		/* transfer each block, and update request status */
		while (length) {
			rv32x->kmap_addr += size;
			size = min(length, data->blksz);
			rv32x->transfer_len = size;
			//	pr_debug("kmap addr:%x", rv32x->kmap_addr);
			//	pr_debug("size:%d", size);
			//pr_debug("cmd:%u", cmd->opcode);
			//pr_debug("arg:%x", cmd->arg);

			if (data->flags & MMC_DATA_READ) {
				cmd->opcode =
					(cmd->opcode != 18 ? cmd->opcode : 17);
				rv32x_mmc_read_block(rv32x, mrq);
			} else {
				cmd->opcode =
					(cmd->opcode != 25 ? cmd->opcode : 24);
				data->error = rv32x_mmc_write_block(rv32x, mrq);
				if (data->error || cmd->error) {
					pr_debug("cmd:%u", cmd->opcode);
					pr_debug("arg:%x", cmd->arg);
					pr_debug("R1resp: %08x", cmd->resp[0]);
					break;
				}
			}
			rv32x->blk_cnt++;
			data->bytes_xfered += size;
			cmd->arg +=
				data->blksz; //シングルコマンドでマルチ転送を実現するために必要
			length -= size;
			if (!multiple)
				break;
		}
	}

	return 0;
}

static void rv32x_mmc_do_command(struct rv32x_mmc *rv32x,
				 struct mmc_command *cmd, u32 cmdinfo)
{
	//	pr_debug("cmd:%u", cmd->opcode);
	//	pr_debug("arg:%x", cmd->arg);
	while (ioread32(rv32x->regbase + RV32X_MMC_STATUS) != OK) {
		asm volatile("nop");
	}
	iowrite32(cmd->opcode, rv32x->regbase + RV32X_MMC_CMD);
	iowrite32(cmd->arg, rv32x->regbase + RV32X_MMC_ARG);
	iowrite32(cmdinfo, rv32x->regbase + RV32X_MMC_CMDINFO);
	iowrite32((ioread32(rv32x->regbase + RV32X_MMC_STATUS) | EXEC),
		  rv32x->regbase + RV32X_MMC_STATUS);
	while (ioread32(rv32x->regbase + RV32X_MMC_STATUS) != OK) {
		asm volatile("nop");
	}
	if (mmc_spi_resp_type(cmd) == MMC_RSP_SPI_R2) {
		cmd->resp[0] = ioread32(rv32x->regbase + RV32X_MMC_RESP);
		cmd->resp[0] <<= 8;
		cmd->resp[0] |= ioread32(rv32x->regbase + RV32X_MMC_R1STAT);
	} else {
		cmd->resp[0] = ioread32(rv32x->regbase + RV32X_MMC_R1STAT);
		cmd->resp[1] = ioread32(rv32x->regbase + RV32X_MMC_RESP);
	}
	//	pr_debug("R1resp: %08x", cmd->resp[0]);
	//pr_debug("ocr:%x", ioread32(rv32x->regbase + RV32X_MMC_RESP));

	if (cmd->resp[0] != 0) {
		if ((R1_SPI_PARAMETER | R1_SPI_ADDRESS) & cmd->resp[0])
			cmd->error = -EFAULT; /* Bad address */
		else if (R1_SPI_ILLEGAL_COMMAND & cmd->resp[0])
			cmd->error = -ENOSYS; /* Function not implemented */
		else if (R1_SPI_COM_CRC & cmd->resp[0])
			cmd->error = -EILSEQ; /* Illegal byte sequence */
		else if ((R1_SPI_ERASE_SEQ | R1_SPI_ERASE_RESET) & cmd->resp[0])
			cmd->error = -EIO;
	}
}

static void rv32x_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	mrq->cap_cmd_during_tfr = true; //no irq wait

	struct mmc_command *cmd = mrq->cmd;
	struct rv32x_mmc *rv32x = mmc_priv(mmc);

	//pr_debug("cmd ptr:%x",mrq->cmd);
	//pr_debug("cmd addr:%x",&(mrq->cmd));
	//pr_debug("data addr:%x",&(cmd->data));
	//pr_debug("container cmd ptr:%x",data_to_cmd(&(cmd->data)));
	if (mrq->data) {
		rv32x_mmc_transfer_data(rv32x, mrq);
	} else if (mrq->sbc) { //マルチ転送用コマンドをskip
		mrq->sbc->resp[0] = 0;
	} else if (mrq->stop) { //マルチ転送用コマンドをskip
		mrq->stop->resp[0] = 0;
	} else {
		rv32x_mmc_do_command(rv32x, cmd,
				     (mmc_spi_resp_type(cmd) |
				      mmc_cmd_type(cmd)));
	}
done:
	mmc_request_done(mmc, mrq);
}

static void rv32x_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	//no io set CRs(like fmax, clock)
}

static int rv32x_mmc_get_ro(struct mmc_host *mmc)
{
	return 0; //	read/write card
}

static int rv32x_mmc_get_cd(struct mmc_host *mmc)
{
	return 1; //always present(TODO: add card existing cr)
}

static const struct mmc_host_ops rv32x_mmc_host = {
	.request = rv32x_mmc_request,
	.set_ios = rv32x_mmc_set_ios,
	.get_ro = rv32x_mmc_get_ro,
	.get_cd = rv32x_mmc_get_cd,
};

static int rv32x_mmc_init(struct platform_device *pdev)
{
	struct rv32x_mmc *rv32x;
	struct mmc_host *mmc;
	struct resource *res;
	int err;
	mmc = mmc_alloc_host(sizeof(*rv32x), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	platform_set_drvdata(pdev, mmc);
	mmc->ops = &rv32x_mmc_host;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_SPI | MMC_CAP_NONREMOVABLE;
	mmc->caps2 = MMC_CAP2_NO_SDIO | MMC_CAP2_NO_WRITE_PROTECT;
	rv32x = mmc_priv(mmc);
	rv32x->regbase = devm_platform_ioremap_resource(pdev, 0);
	//pr_debug("rv32x->regbase:%x",rv32x->regbase);

	err = mmc_add_host(mmc);
	if (unlikely(err))
		goto err_free_mmc;

	return 0;

err_free_mmc:
	mmc_free_host(mmc);
	return err;
}

static int rv32x_mmc_exit(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	;
	mmc_free_host(mmc);
	return 0;
}

static const struct of_device_id rv32x_mmc_of_match_table[] = {
	{
		.compatible = "rv32x-mmcspi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, rv32x_mmc_of_match_table);

static struct platform_driver rv32x_mmc_driver = {
	.driver = {
		.name =		"rv32x_mmc",
		.of_match_table = rv32x_mmc_of_match_table,
	},
	.probe = rv32x_mmc_init,
	.remove = rv32x_mmc_exit,
};

module_platform_driver(rv32x_mmc_driver);

MODULE_AUTHOR("twinkletia");
MODULE_DESCRIPTION("rv32x MMC host driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rv32x-mmc");
