#include "LD2450.h"

static int ld2450_probe(struct serdev_device *serdev)
{
    struct ld2450_device *ld2450;
    struct device *dev = &serdev->dev;

    ld2450 = devm_kzalloc(dev, sizeof(*ld2450), GFP_KERNEL);
    if (!ld2450)
        return -ENOMEM;

    ld2450->serdev = serdev;
    ld2450->dev = dev;
    serdev_device_set_drvdata(serdev, ld2450);

    /* Configuração da UART */
    serdev_device_set_baudrate(serdev, LD2450_BAUDRATE);
    serdev_device_set_flow_control(serdev, false);

    dev_info(dev, "HLK-LD2450 radar initialized\n");
    return 0;
}

static void ld2450_remove(struct serdev_device *serdev)
{
    dev_info(&serdev->dev, "HLK-LD2450 radar disconnected\n");
}

/* Tabela de compatibilidade para Device Tree */
static const struct of_device_id ld2450_of_match[] = {
    { .compatible = "hlk,ld2450" },
    { }
};
MODULE_DEVICE_TABLE(of, ld2450_of_match);

/* Estrutura do driver serdev */
static struct serdev_device_driver ld2450_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = ld2450_of_match,
    },
    .probe = ld2450_probe,
    .remove = ld2450_remove,
};

module_serdev_device_driver(ld2450_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Seu Nome");
MODULE_DESCRIPTION("Linux device driver for HLK-LD2450 millimeter-wave radar");