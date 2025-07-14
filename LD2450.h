#ifndef LD2450_H
#define LD2450_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/serdev.h>
#include <linux/device.h>
#include <linux/of.h>

#define DRIVER_NAME "ld2450"
#define LD2450_BAUDRATE 115200

/* Estrutura de dados do dispositivo */
struct ld2450_device {
    struct serdev_device *serdev;
    struct device *dev;
    char buffer[256];
    int buffer_len;
};

#endif // LD2450_H