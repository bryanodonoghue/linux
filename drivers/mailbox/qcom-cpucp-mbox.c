// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024, The Linux Foundation. All rights reserved.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/platform_device.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>

#define APSS_CPUCP_IPC_CHAN_SUPPORTED		3
#define APSS_CPUCP_MBOX_CMD_OFF			0x4

/* Tx Registers */
#define APSS_CPUCP_TX_MBOX_IDR			0
#define APSS_CPUCP_TX_MBOX_CMD			0x100

/* Rx Registers */
#define APSS_CPUCP_RX_MBOX_IDR			0
#define APSS_CPUCP_RX_MBOX_CMD			0x100
#define APSS_CPUCP_RX_MBOX_MAP			0x4000
#define APSS_CPUCP_RX_MBOX_STAT			0x4400
#define APSS_CPUCP_RX_MBOX_CLEAR		0x4800
#define APSS_CPUCP_RX_MBOX_EN			0x4C00
#define APSS_CPUCP_RX_MBOX_CMD_MASK		0xFFFFFFFFFFFFFFFF

/**
 * struct qcom_cpucp_mbox - Holder for the mailbox driver
 * @chans:			The mailbox channel
 * @mbox:			The mailbox controller
 * @tx_base:			Base address of the CPUCP tx registers
 * @rx_base:			Base address of the CPUCP rx registers
 * @dev:			Device associated with this instance
 * @irq:			CPUCP to AP irq
 */
struct qcom_cpucp_mbox {
	struct mbox_chan chans[APSS_CPUCP_IPC_CHAN_SUPPORTED];
	struct mbox_controller mbox;
	void __iomem *tx_base;
	void __iomem *rx_base;
	struct device *dev;
	int irq;
	int num_chan;
};

static irqreturn_t qcom_cpucp_mbox_irq_fn(int irq, void *data)
{
	struct qcom_cpucp_mbox *cpucp = data;
	u64 status;
	u32 val;
	int i;

	status = readq(cpucp->rx_base + APSS_CPUCP_RX_MBOX_STAT);

	for (i = 0; i < cpucp->num_chan; i++) {
		val = 0;
		if (status & ((u64)1 << i)) {
			val = readl(cpucp->rx_base + APSS_CPUCP_RX_MBOX_CMD + (i * 8) + APSS_CPUCP_MBOX_CMD_OFF);
			if (!IS_ERR(cpucp->chans[i].con_priv))
				mbox_chan_received_data(&cpucp->chans[i], &val);
			writeq(status, cpucp->rx_base + APSS_CPUCP_RX_MBOX_CLEAR);
		}
	}

	return IRQ_HANDLED;
}

static int qcom_cpucp_mbox_startup(struct mbox_chan *chan)
{
	struct qcom_cpucp_mbox *cpucp = container_of(chan->mbox, struct qcom_cpucp_mbox, mbox);
	unsigned long chan_id = (unsigned long)chan->con_priv;
	u64 val;

	val = readq(cpucp->rx_base + APSS_CPUCP_RX_MBOX_EN);
	val |= ((u64)1 << chan_id);
	writeq(val, cpucp->rx_base + APSS_CPUCP_RX_MBOX_EN);

	return 0;
}

static void qcom_cpucp_mbox_shutdown(struct mbox_chan *chan)
{
	struct qcom_cpucp_mbox *cpucp = container_of(chan->mbox, struct qcom_cpucp_mbox, mbox);
	unsigned long chan_id = (unsigned long)chan->con_priv;
	u64 val;

	val = readq(cpucp->rx_base + APSS_CPUCP_RX_MBOX_EN);
	val &= ~((u64)1 << chan_id);
	writeq(val, cpucp->rx_base + APSS_CPUCP_RX_MBOX_EN);

	chan->con_priv = ERR_PTR(-EINVAL);
}

static int qcom_cpucp_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct qcom_cpucp_mbox *cpucp = container_of(chan->mbox, struct qcom_cpucp_mbox, mbox);
	unsigned long chan_id = (unsigned long)chan->con_priv;
	u32 val = (unsigned long)data;

	writel(val, cpucp->tx_base + APSS_CPUCP_TX_MBOX_CMD + (chan_id * 8) + APSS_CPUCP_MBOX_CMD_OFF);

	return 0;
}

static struct mbox_chan *qcom_cpucp_mbox_xlate(struct mbox_controller *mbox,
					       const struct of_phandle_args *sp)
{
	unsigned long ind = sp->args[0];

	if (sp->args_count != 1)
		return ERR_PTR(-EINVAL);

	if (ind >= mbox->num_chans)
		return ERR_PTR(-EINVAL);

	if (!IS_ERR(mbox->chans[ind].con_priv))
		return ERR_PTR(-EBUSY);

	mbox->chans[ind].con_priv = (void *)ind;

	return &mbox->chans[ind];
}

static const struct mbox_chan_ops qcom_cpucp_mbox_chan_ops = {
	.startup = qcom_cpucp_mbox_startup,
	.send_data = qcom_cpucp_mbox_send_data,
	.shutdown = qcom_cpucp_mbox_shutdown
};

static int qcom_cpucp_setup_mbox(struct qcom_cpucp_mbox *cpucp)
{
	struct device *dev = cpucp->dev;
	struct mbox_controller *mbox;
	unsigned long i;

	/* Initialize channel identifiers */
	for (i = 0; i < ARRAY_SIZE(cpucp->chans); i++)
		cpucp->chans[i].con_priv = ERR_PTR(-EINVAL);

	mbox = &cpucp->mbox;
	mbox->dev = dev;
	mbox->num_chans = cpucp->num_chan;
	mbox->chans = cpucp->chans;
	mbox->ops = &qcom_cpucp_mbox_chan_ops;
	mbox->of_xlate = qcom_cpucp_mbox_xlate;
	mbox->txdone_irq = false;
	mbox->txdone_poll = false;

	return mbox_controller_register(mbox);
}

static int qcom_cpucp_mbox_probe(struct platform_device *pdev)
{
	struct qcom_cpucp_mbox *cpucp;
	struct resource *res;
	int ret;

	cpucp = devm_kzalloc(&pdev->dev, sizeof(*cpucp), GFP_KERNEL);
	if (!cpucp)
		return -ENOMEM;

	cpucp->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get the cpucp rx base address\n");
		return -ENODEV;
	}

	cpucp->rx_base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!cpucp->rx_base) {
		dev_err(&pdev->dev, "Failed to ioremap cpucp tx base\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get the cpucp tx base address\n");
		return -ENODEV;
	}

	cpucp->tx_base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!cpucp->tx_base) {
		dev_err(&pdev->dev, "Failed to ioremap cpucp tx base\n");
		return -ENOMEM;
	}

	writeq(0, cpucp->rx_base + APSS_CPUCP_RX_MBOX_EN);
	writeq(0, cpucp->rx_base + APSS_CPUCP_RX_MBOX_CLEAR);
	writeq(0, cpucp->rx_base + APSS_CPUCP_RX_MBOX_MAP);

	cpucp->irq = platform_get_irq(pdev, 0);
	if (cpucp->irq < 0) {
		dev_err(&pdev->dev, "Failed to get the IRQ\n");
		return cpucp->irq;
	}

	ret = devm_request_irq(&pdev->dev, cpucp->irq, qcom_cpucp_mbox_irq_fn,
			       IRQF_TRIGGER_HIGH, "apss_cpucp_mbox", cpucp);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register the irq: %d\n", ret);
		return ret;
	}

	writeq(APSS_CPUCP_RX_MBOX_CMD_MASK, cpucp->rx_base + APSS_CPUCP_RX_MBOX_MAP);

	cpucp->num_chan = APSS_CPUCP_IPC_CHAN_SUPPORTED;
	ret = qcom_cpucp_setup_mbox(cpucp);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create mailbox\n");
		return ret;
	}

	platform_set_drvdata(pdev, cpucp);

	return 0;
}

static int qcom_cpucp_mbox_remove(struct platform_device *pdev)
{
	struct qcom_cpucp_mbox *cpucp = platform_get_drvdata(pdev);

	mbox_controller_unregister(&cpucp->mbox);

	return 0;
}

static const struct of_device_id qcom_cpucp_mbox_of_match[] = {
	{ .compatible = "qcom,cpucp-mbox"},
	{}
};
MODULE_DEVICE_TABLE(of, qcom_cpucp_mbox_of_match);

static struct platform_driver qcom_cpucp_mbox_driver = {
	.probe = qcom_cpucp_mbox_probe,
	.remove = qcom_cpucp_mbox_remove,
	.driver = {
		.name = "qcom_cpucp_mbox",
		.of_match_table = qcom_cpucp_mbox_of_match,
		.suppress_bind_attrs = true,
	},
};

static int __init qcom_cpucp_mbox_init(void)
{
	int ret;

	ret = platform_driver_register(&qcom_cpucp_mbox_driver);
	if (ret)
		pr_err("%s: qcom_cpucp_mbox register failed %d\n", __func__, ret);

	return ret;
}
module_init(qcom_cpucp_mbox_init);

static __exit void qcom_cpucp_mbox_exit(void)
{
	platform_driver_unregister(&qcom_cpucp_mbox_driver);
}
module_exit(qcom_cpucp_mbox_exit);

MODULE_DESCRIPTION("QTI CPUCP MBOX Driver");
MODULE_LICENSE("GPL");
