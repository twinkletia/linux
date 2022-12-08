
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

#define RV32X_SPI_DATA_BASE    0x0
#define RV32X_SPI_STATUS    0x200
#define RV32X_SPI_OPERATION    0x210
#define RV32X_SPI_ADDR    0x214


struct rv32x_mmc {
	void __iomem      *regs;
	unsigned size;
	void* kmap_addr;
	unsigned blk_cnt;
};

static void rv32x_mmc_read_block(struct rv32x_mmc* rv32x, struct mmc_data* data){
	iowrite32(data->blk_addr + rv32x->blk_cnt,rv32x->regs+RV32X_SPI_ADDR);
	iowrite32(0x00000001,rv32x->regs+RV32X_SPI_OPERATION);
	while((ioread8(rv32x->regs+RV32X_SPI_OPERATION) != 0x0) || (ioread8(rv32x->regs+RV32X_SPI_STATUS) != 0x3)) {
		asm volatile("nop");
	}
}

static void rv32x_mmc_read_block_prep(struct rv32x_mmc* rv32x, struct mmc_data* data){
	unsigned size;
	unsigned char* data_ptr;
	static unsigned i = 0;
	size = rv32x->size;
	data_ptr = rv32x->kmap_addr;
	rv32x_mmc_read_block(rv32x, data); //TODO
	while(i<size || i<RV32X_SPI_STATUS){
		*data_ptr++ = ioread8(rv32x->regs+RV32X_SPI_DATA_BASE+i);
		i++;
	}
	if(i == RV32X_SPI_STATUS){
		i=0;
		rv32x->blk_cnt++;
	}
	if(i != size){
		rv32x->size -= i;
		rv32x_mmc_read_block_prep(rv32x,data);
	}
}


static void rv32x_mmc_write_block(struct rv32x_mmc* rv32x, struct mmc_data* data){
	iowrite32(data->blk_addr + rv32x->blk_cnt,rv32x->regs+RV32X_SPI_ADDR);
	iowrite32(0x00000002,rv32x->regs+RV32X_SPI_OPERATION);
	while((ioread8(rv32x->regs+RV32X_SPI_OPERATION) != 0x0) || (ioread8(rv32x->regs+RV32X_SPI_STATUS) != 0x3)) {
		asm volatile("nop");
	}
}

static void rv32x_mmc_write_block_prep(struct rv32x_mmc* rv32x, struct mmc_data* data){
	unsigned size;
	unsigned char* data_ptr;
	static unsigned i = 0;
	size = rv32x->size;
	data_ptr = rv32x->kmap_addr;
	while(i<size || i<RV32X_SPI_STATUS){
		iowrite8(*data_ptr++,rv32x->regs+RV32X_SPI_DATA_BASE+i);
		i++;
	}
	if(i == RV32X_SPI_STATUS){
		rv32x_mmc_write_block(rv32x, data);
		i=0;
		rv32x->blk_cnt++;
	}
	if(i != size){
		rv32x->size -= i;
		rv32x_mmc_write_block_prep(rv32x,data);
	}
}

static int rv32x_mmc_command(struct mmc_host *mmc, struct mmc_command *cmd)
{
	pr_debug("mmc_data data:%x",cmd->data);
	struct mmc_data *data = cmd->data;
	struct rv32x_mmc* rv32x = mmc_priv(mmc);
	struct scatterlist	*sg;
	unsigned		i;
	int			multiple = (data->blocks > 1);

	rv32x->blk_cnt = 0;
	
	for_each_sg(data->sg, sg, data->sg_len, i){
		unsigned		length = sg->length;
		unsigned		size = 0;

		/* allow pio too; we don't allow highmem */
		rv32x->kmap_addr = kmap(sg_page(sg)) + sg->offset;

		/* transfer each block, and update request status */
		while (length) {
			size = min(length, data->blksz);
			rv32x->size = size;
			if(data->flags & MMC_DATA_READ){
				rv32x_mmc_read_block_prep(rv32x, data);
			}else{
				rv32x_mmc_write_block_prep(rv32x, data);
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

static void rv32x_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	pr_debug("mmc_host mmc:%x",mmc);
	pr_debug("mmc_request mrq:%x",mrq);
	pr_debug("mmc_request mrq->cmd:%x",mrq->cmd);
	pr_debug("mmc_request mrq->cmd->opcode:%x",mrq->cmd->opcode);
	pr_debug("mmc_request mrq->cmd->arg:%x",mrq->cmd->arg);
	pr_debug("mmc_request mrq->sbc:%x",mrq->sbc);
	pr_debug("mmc_request mrq->data:%x",mrq->data);
	pr_debug("mmc_request mrq->stop:%x",mrq->stop);
	if (!rv32x_mmc_command(mmc, mrq->cmd) && mrq->stop)
		rv32x_mmc_command(mmc, mrq->stop);
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
	int err;
	mmc = mmc_alloc_host(sizeof(*rv32x), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	platform_set_drvdata(pdev, mmc);
	mmc->ops = &rv32x_mmc_host;
	mmc->caps = MMC_CAP_SPI;
	rv32x = mmc_priv(mmc);
	rv32x->regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pr_debug("rv32x->regs:%x",rv32x->regs);
	pr_debug("rv32x->regs:%x",*(int *)(rv32x->regs));

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
