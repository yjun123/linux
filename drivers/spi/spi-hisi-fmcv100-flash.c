// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
/*
 * HiSilicon FMCv100 serial flash controller driver
 *
 * The FMCv100 flash controller of the HiSilicon Hi3519DV500 talks to
 * one flash die at a time in one of three mutually exclusive modes:
 * SPI NOR, SPI NAND or raw NAND.  This driver drives the serial
 * (SPI NOR and SPI NAND) modes as a spi-mem controller: the spi-nor
 * core claims "jedec,spi-nor" children and the spinand core claims
 * "spi-nand" children, and both stacks run through the same register
 * interface.
 *
 * A regular command is assembled from the FMC_CMD, FMC_ADDRL,
 * FMC_ADDRH, FMC_OP_CFG, FMC_DATA_NUM and FMC_OP registers; FMC_OP
 * carries the enable bit of every phase of the sequence (command,
 * address, dummy, data in/out) and starts it.  Data goes through
 * the beginning of the memory window.  Addresses are written as a
 * bit plane combination in the SPI NAND mode: column addresses
 * occupy FMC_ADDRL bits 15:0 while row addresses (page numbers)
 * start at bit 16 and spill into FMC_ADDRH; the SPI NOR mode takes
 * the linear flash address in FMC_ADDRL instead.
 *
 * The controller also has a hardware page engine with DMA and a
 * hardware BCH ECC engine.  Both are left unused: with
 * FMC_CFG[ecc_type] zero the controller passes data through as-is,
 * and the on-die ECC of the NAND die does the error correction, as
 * in every other spinand setup.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

#define FMC_CFG					0x00
#define  FMC_CFG_FLASH_TYPE_SPI_NOR		0
#define  FMC_CFG_FLASH_TYPE_SPI_NAND		1
#define  FMC_CFG_FLASH_TYPE_MASK		GENMASK(2, 1)
#define  FMC_CFG_OP_MODE_NORMAL			1
#define  FMC_CFG_OP_MODE_MASK			BIT(0)

#define FMC_GLOBAL_CFG				0x04
#define  FMC_GLOBAL_CFG_WP			BIT(6)
#define  FMC_GLOBAL_CFG_DTR			BIT(11)

#define FMC_SPI_TIMING_CFG			0x08
#define  FMC_SPI_TIMING_CFG_TCSH(x)		(((x) & 0xf) << 8)
#define  FMC_SPI_TIMING_CFG_TCSS(x)		(((x) & 0xf) << 4)
#define  FMC_SPI_TIMING_CFG_TSHSL(x)		((x) & 0xf)
#define FMC_SPI_TIMING_CFG_DEFAULT		(FMC_SPI_TIMING_CFG_TCSH(6) | \
						 FMC_SPI_TIMING_CFG_TCSS(6) | \
						 FMC_SPI_TIMING_CFG_TSHSL(0xf))

#define FMC_INT_CLR				0x20
#define  FMC_INT_CLR_ALL			0xff

#define FMC_CMD					0x24
#define  FMC_CMD_CMD1(x)			((x) & 0xff)

#define FMC_ADDRH				0x28
#define FMC_ADDRL				0x2c

#define FMC_OP_CFG				0x30
#define  FMC_OP_CFG_OEN				BIT(13)
#define  FMC_OP_CFG_CS(x)			(((x) & 0x3) << 11)
#define  FMC_OP_CFG_IF_TYPE(x)			(((x) & 0x7) << 7)
#define  FMC_OP_CFG_ADDR_NUM(x)			(((x) & 0x7) << 4)
#define  FMC_OP_CFG_DUMMY(x)			((x) & 0xf)

#define FMC_DATA_NUM				0x38
#define  FMC_DATA_NUM_MAX			0x3fff

#define FMC_OP					0x3c
#define  FMC_OP_DUMMY_EN			BIT(8)
#define  FMC_OP_CMD1_EN				BIT(7)
#define  FMC_OP_ADDR_EN				BIT(6)
#define  FMC_OP_WRITE_DATA_EN			BIT(5)
#define  FMC_OP_READ_DATA_EN			BIT(2)
#define  FMC_OP_REG_OP_START			BIT(0)

#define FMC_OP_TIMEOUT_MS			1000

enum fmcv100_if_type {
	FMCV100_IF_TYPE_STD,
	FMCV100_IF_TYPE_DUAL,
	FMCV100_IF_TYPE_DIO,
	FMCV100_IF_TYPE_QUAD,
	FMCV100_IF_TYPE_QIO,
};

struct fmcv100_flash {
	struct device *dev;
	void __iomem *regbase;
	void __iomem *iobase;
	struct clk *clk;
	bool nand;
};

static void fmcv100_flash_select_mode(struct fmcv100_flash *host)
{
	u32 reg;

	/*
	 * Keep the controller in the normal (not boot) operation mode
	 * and in the flash type of the populated die, with the ECC type
	 * cleared: a zero ECC type makes the controller carry data
	 * as-is, which is what the on-die ECC of the NAND die requires.
	 */
	reg = readl(host->regbase + FMC_CFG);
	reg &= ~(FMC_CFG_FLASH_TYPE_MASK | FMC_CFG_OP_MODE_MASK);
	reg |= FIELD_PREP(FMC_CFG_FLASH_TYPE_MASK,
			  host->nand ? FMC_CFG_FLASH_TYPE_SPI_NAND
				     : FMC_CFG_FLASH_TYPE_SPI_NOR);
	reg |= FMC_CFG_OP_MODE_NORMAL;
	writel(reg, host->regbase + FMC_CFG);
}

static bool fmcv100_flash_supports_op(struct spi_mem *mem,
				      const struct spi_mem_op *op)
{
	if (op->cmd.nbytes != 1 || op->cmd.buswidth != 1)
		return false;

	/* SPI NOR addresses are up to four bytes, SPI NAND ones to three. */
	if (op->addr.nbytes > 4)
		return false;

	if (op->addr.buswidth > 4)
		return false;

	/*
	 * Extended I/O protocols send the address lines with the same
	 * width as the data lines.
	 */
	if (op->addr.buswidth > 1 && op->addr.buswidth != op->data.buswidth)
		return false;

	if (op->dummy.nbytes > 15)
		return false;

	if (op->data.nbytes > FMC_DATA_NUM_MAX)
		return false;

	if (op->data.buswidth > 4)
		return false;

	return true;
}

static int fmcv100_flash_adjust_op_size(struct spi_mem *mem,
					struct spi_mem_op *op)
{
	op->data.nbytes = min_t(u32, op->data.nbytes, FMC_DATA_NUM_MAX);

	return 0;
}

static int fmcv100_flash_get_if_type(const struct spi_mem_op *op)
{
	if (op->data.buswidth == 4)
		return op->addr.buswidth == 4 ?
			FMCV100_IF_TYPE_QIO : FMCV100_IF_TYPE_QUAD;

	if (op->data.buswidth == 2)
		return op->addr.buswidth == 2 ?
			FMCV100_IF_TYPE_DIO : FMCV100_IF_TYPE_DUAL;

	return FMCV100_IF_TYPE_STD;
}

static int fmcv100_flash_exec_op(struct spi_mem *mem,
				 const struct spi_mem_op *op)
{
	struct fmcv100_flash *host = spi_controller_get_devdata(mem->spi->controller);
	u32 op_cfg, op_val = 0;
	u32 cs = spi_get_chipselect(mem->spi, 0);
	u32 addr_num = op->addr.nbytes;
	u32 reg;
	int ret;

	fmcv100_flash_select_mode(host);

	writel(FMC_INT_CLR_ALL, host->regbase + FMC_INT_CLR);
	writel(FMC_CMD_CMD1(op->cmd.opcode), host->regbase + FMC_CMD);

	if (addr_num) {
		if (host->nand) {
			/*
			 * Column addresses (one or two bytes) occupy the
			 * low half of FMC_ADDRL.  Row addresses (three
			 * bytes, page numbers) start at bit 16: the low
			 * half of FMC_ADDRL carries the low half of the
			 * page number and the rest goes into FMC_ADDRH.
			 */
			if (addr_num <= 2)
				writel(op->addr.val, host->regbase + FMC_ADDRL);
			else
				writel((u32)(op->addr.val & 0xffff) << 16,
				       host->regbase + FMC_ADDRL);

			writel(op->addr.val >> 16, host->regbase + FMC_ADDRH);
		} else {
			writel(op->addr.val, host->regbase + FMC_ADDRL);
		}

		op_val |= FMC_OP_ADDR_EN;
	}

	op_cfg = FMC_OP_CFG_OEN |
		 FMC_OP_CFG_CS(cs) |
		 FMC_OP_CFG_IF_TYPE(fmcv100_flash_get_if_type(op)) |
		 FMC_OP_CFG_ADDR_NUM(addr_num) |
		 FMC_OP_CFG_DUMMY(op->dummy.nbytes);
	writel(op_cfg, host->regbase + FMC_OP_CFG);

	if (op->data.nbytes) {
		writel(op->data.nbytes, host->regbase + FMC_DATA_NUM);

		if (op->data.dir == SPI_MEM_DATA_OUT) {
			memcpy_toio(host->iobase, op->data.buf.out,
				    op->data.nbytes);
			op_val |= FMC_OP_WRITE_DATA_EN;
		} else {
			op_val |= FMC_OP_READ_DATA_EN;
		}
	}

	if (op->dummy.nbytes)
		op_val |= FMC_OP_DUMMY_EN;

	op_val |= FMC_OP_CMD1_EN | FMC_OP_REG_OP_START;
	writel(op_val, host->regbase + FMC_OP);

	ret = readl_poll_timeout(host->regbase + FMC_OP, reg,
				 !(reg & FMC_OP_REG_OP_START),
				 10, FMC_OP_TIMEOUT_MS);
	if (ret)
		return ret;

	if (op->data.nbytes && op->data.dir == SPI_MEM_DATA_IN)
		memcpy_fromio(op->data.buf.in, host->iobase,
			      op->data.nbytes);

	return 0;
}

static const struct spi_controller_mem_ops fmcv100_flash_mem_ops = {
	.supports_op = fmcv100_flash_supports_op,
	.adjust_op_size = fmcv100_flash_adjust_op_size,
	.exec_op = fmcv100_flash_exec_op,
};

static void fmcv100_flash_hw_init(struct fmcv100_flash *host)
{
	u32 reg;

	fmcv100_flash_select_mode(host);

	/* Stay in single transfer rate mode and drop write protection. */
	reg = readl(host->regbase + FMC_GLOBAL_CFG);
	reg &= ~(FMC_GLOBAL_CFG_DTR | FMC_GLOBAL_CFG_WP);
	writel(reg, host->regbase + FMC_GLOBAL_CFG);

	writel(FMC_SPI_TIMING_CFG_DEFAULT,
	       host->regbase + FMC_SPI_TIMING_CFG);
}

static int fmcv100_flash_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *ctlr;
	struct fmcv100_flash *host;
	int ret;

	ctlr = devm_spi_alloc_host(dev, sizeof(*host));
	if (!ctlr)
		return -ENOMEM;

	host = spi_controller_get_devdata(ctlr);
	host->dev = dev;

	host->regbase = devm_platform_ioremap_resource_byname(pdev, "control");
	if (IS_ERR(host->regbase))
		return PTR_ERR(host->regbase);

	host->iobase = devm_platform_ioremap_resource_byname(pdev, "memory");
	if (IS_ERR(host->iobase))
		return PTR_ERR(host->iobase);

	host->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(host->clk))
		return dev_err_probe(dev, PTR_ERR(host->clk),
				     "failed to get and enable clock\n");

	platform_set_drvdata(pdev, host);

	/*
	 * The controller drives the single die wired in one of its serial
	 * modes; pick the flash type from the compatible of the child
	 * node before the SPI core claims the child.
	 */
	for_each_available_child_of_node_scoped(dev->of_node, child)
		if (of_device_is_compatible(child, "spi-nand")) {
			host->nand = true;
			break;
		}

	fmcv100_flash_hw_init(host);

	ctlr->mode_bits = SPI_RX_DUAL | SPI_RX_QUAD |
			  SPI_TX_DUAL | SPI_TX_QUAD;
	ctlr->bus_num = -1;
	ctlr->num_chipselect = 2;
	ctlr->mem_ops = &fmcv100_flash_mem_ops;

	ret = devm_spi_register_controller(dev, ctlr);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register controller\n");

	return 0;
}

static int fmcv100_flash_suspend(struct device *dev)
{
	struct fmcv100_flash *host = dev_get_drvdata(dev);

	clk_disable_unprepare(host->clk);

	return 0;
}

static int fmcv100_flash_resume(struct device *dev)
{
	struct fmcv100_flash *host = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(host->clk);
	if (ret)
		return ret;

	fmcv100_flash_hw_init(host);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(fmcv100_flash_pm_ops,
				fmcv100_flash_suspend, fmcv100_flash_resume);

static const struct of_device_id fmcv100_flash_match[] = {
	{ .compatible = "hisilicon,fmcv100-spi-flash" },
	{}
};
MODULE_DEVICE_TABLE(of, fmcv100_flash_match);

static struct platform_driver fmcv100_flash_driver = {
	.probe = fmcv100_flash_probe,
	.driver = {
		.name = "fmcv100-spi-flash",
		.of_match_table = fmcv100_flash_match,
		.pm = pm_ptr(&fmcv100_flash_pm_ops),
	},
};
module_platform_driver(fmcv100_flash_driver);

MODULE_DESCRIPTION("HiSilicon FMCv100 serial flash controller driver");
MODULE_AUTHOR("Jun Yan <jerrysteve1101@gmail.com>");
MODULE_LICENSE("GPL");
