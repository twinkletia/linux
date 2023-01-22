
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
#include <linux/mmc/mmc.h>		/* for R1_SPI_* bit values */

#include <linux/platform_device.h>

#define RV32X_MMC_DATA_BASE    0x0
#define RV32X_MMC_STATUS    0x200
#define RV32X_MMC_R1STAT	0x204
#define RV32X_MMC_RESP		0x208
#define RV32X_MMC_CMD		0x20C
#define RV32X_MMC_ARG		0x210
#define RV32X_MMC_CSD_CID_BASE	0x214
#define INITED 0x1
#define IDLE 0x2
#define EXEC 0x4
#define INTR_EN 0x8
#define OK (INITED | IDLE)


struct rv32x_mmc {
	void __iomem      *regbase;
	void* kmap_addr;
	unsigned blk_cnt;
};


static inline struct mmc_command* data_to_cmd(struct mmc_data**);
static void rv32x_mmc_read_block(struct rv32x_mmc*, struct mmc_data*);
static void rv32x_mmc_write_block(struct rv32x_mmc*, struct mmc_data*);
static int rv32x_mmc_transfer_data(struct rv32x_mmc*, struct mmc_data *);
static int rv32x_mmc_do_command(struct rv32x_mmc*, u32, u32, u32*);

static inline struct mmc_command* data_to_cmd(struct mmc_data** data_ptr){
	return container_of(data_ptr, struct mmc_command, data);
}

static void rv32x_mmc_read_block(struct rv32x_mmc* rv32x, struct mmc_data* data){
	unsigned* data_ptr;
	unsigned i = 0;
	struct mmc_command* cmd = data_to_cmd(&data);
	rv32x_mmc_do_command(rv32x, 17, cmd->arg + (rv32x->blk_cnt*512), cmd->resp);	//argはdata->blk_addrでいいかも
	rv32x->blk_cnt++;
	data_ptr = rv32x->kmap_addr;
	while(i < RV32X_MMC_STATUS){
		*data_ptr++ = ioread32(rv32x->regbase+RV32X_MMC_DATA_BASE+i);
		i+=4;
	}
}

static void rv32x_mmc_write_block(struct rv32x_mmc* rv32x, struct mmc_data* data){
	unsigned* data_ptr;
	unsigned i = 0;
	struct mmc_command* cmd = data_to_cmd(&data);
	data_ptr = rv32x->kmap_addr;
	while(i < RV32X_MMC_STATUS){
		iowrite32(*data_ptr++,rv32x->regbase+RV32X_MMC_DATA_BASE+i);
		i+=4;
	}
	rv32x_mmc_do_command(rv32x, 24, cmd->arg + (rv32x->blk_cnt*512), cmd->resp);	//argはdata->blk_addrでいいかも
	rv32x->blk_cnt++;
}

static int rv32x_mmc_transfer_data(struct rv32x_mmc* rv32x,
	struct mmc_data *data)
{
	struct scatterlist	*sg;
	unsigned		i;
	int			multiple = (data->blocks > 1);
	unsigned		length = 0;
	unsigned		size = 0;

	rv32x->blk_cnt = 0;
	
	for_each_sg(data->sg, sg, data->sg_len, i){
		length = sg->length;
		size = 0;

		/* allow pio too; we don't allow highmem */
		rv32x->kmap_addr = kmap(sg_page(sg)) + sg->offset;

		/* transfer each block, and update request status */
		while (length) {
			rv32x->kmap_addr += size;
			size = min(length, data->blksz);
			if(data->flags & MMC_DATA_READ){
				rv32x_mmc_read_block(rv32x, data);
			}else{
				rv32x_mmc_write_block(rv32x, data);
			}
			data->bytes_xfered += size;
			length -= size;
			if (!multiple)
				break;
		}
	}
	for_each_sg(data->sg, sg, data->sg_len, i){
		kunmap(sg_page(sg));
	}

	return 0;
}

static int rv32x_mmc_do_command(struct rv32x_mmc* rv32x, u32 command, u32 arg, u32* response)
{
	//pr_debug("cmd:%u",command);
	//pr_debug("cmd:%x",arg);
	while(ioread32(rv32x->regbase+RV32X_MMC_STATUS) != OK){
		asm volatile("nop");
	}
	iowrite32(command,rv32x->regbase+RV32X_MMC_CMD);
	iowrite32(arg,rv32x->regbase+RV32X_MMC_ARG);
	iowrite32(EXEC,rv32x->regbase+RV32X_MMC_STATUS);
	while(ioread32(rv32x->regbase+RV32X_MMC_STATUS) != OK){
		asm volatile("nop");
	}
	if(command == 13){
		response[0] = ioread32(rv32x->regbase+RV32X_MMC_RESP);
		response[0] <<= 8;
		response[0] |= ioread32(rv32x->regbase+RV32X_MMC_R1STAT);
	}else{
		response[0] = ioread32(rv32x->regbase+RV32X_MMC_R1STAT);
		response[1] = ioread32(rv32x->regbase+RV32X_MMC_RESP);
	}

	return 0;
}

static void rv32x_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	mrq->cap_cmd_during_tfr = true;	//no irq wait

	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->cmd->data;
	struct rv32x_mmc* rv32x = mmc_priv(mmc);

	if(data){
		rv32x_mmc_transfer_data(rv32x, data);
	}else if(mrq->sbc){
		mrq->sbc->resp[0] = 0;
	}else if(mrq->stop){
		mrq->stop->resp[0] = 0;
	}else{
		rv32x_mmc_do_command(rv32x, cmd->opcode, cmd->arg, cmd->resp);
	}
	/*
	pr_debug("mmc_request mrq->cmd:%x",mrq->cmd);
	pr_debug("mmc_request mrq->cmd->opcode:%x",mrq->cmd->opcode);
	pr_debug("mmc_request mrq->cmd->arg:%x",mrq->cmd->arg);
	pr_debug("mmc_request mrq->sbc:%x",mrq->sbc);
	pr_debug("mmc_request mrq->stop:%x",mrq->stop);
	pr_debug("mmc_request mrq->data:%x",mrq->data);
	*/
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
	return 1;	//always present(TODO: add card existing cr)
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
	struct resource* res;
	int err;
	mmc = mmc_alloc_host(sizeof(*rv32x), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	platform_set_drvdata(pdev, mmc);
	mmc->ops = &rv32x_mmc_host;
	mmc->ocr_avail = MMC_VDD_32_33|MMC_VDD_33_34;
	mmc->caps = MMC_CAP_SPI;
	mmc->caps2 = MMC_CAP2_NO_SDIO | MMC_CAP2_NO_SD | MMC_CAP2_NO_WRITE_PROTECT;
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
	struct mmc_host *mmc = platform_get_drvdata(pdev);;
	mmc_free_host(mmc);
	return 0;
}

static const struct of_device_id rv32x_mmc_of_match_table[] = {
	{ .compatible = "rv32x-mmcspi", },
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
