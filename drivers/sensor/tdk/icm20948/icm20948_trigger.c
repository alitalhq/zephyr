/*
 * Copyright (c) 2026 T3 Gemstone
 *	T3 Gemstone Developer Team <support@t3gemstone.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "icm20948.h"
#include "icm20948_reg.h"

LOG_MODULE_DECLARE(ICM20948, CONFIG_SENSOR_LOG_LEVEL);

int icm20948_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	const struct icm20948_config *cfg = dev->config;
	struct icm20948_data *data = dev->data;
	int ret;

	if (trig->type != SENSOR_TRIG_DATA_READY) {
		return -ENOTSUP;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_DISABLE);
	if (ret < 0) {
		LOG_ERR("failed to mask the interrupt: %d", ret);
		return ret;
	}

	data->drdy_handler = handler;
	if (handler == NULL) {
		return 0;
	}

	data->drdy_trigger = trig;

	ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("failed to unmask the interrupt: %d", ret);
		return ret;
	}

	return 0;
}

static void icm20948_irq_callback(const struct device *port, struct gpio_callback *cb,
				  uint32_t pins)
{
	struct icm20948_data *data = CONTAINER_OF(cb, struct icm20948_data, irq_cb);
	const struct icm20948_config *cfg = data->dev->config;
	int ret;

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	/* Masked until the handler has run, so that it cannot be re-entered */
	ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_DISABLE);
	if (ret < 0) {
		LOG_ERR("failed to mask the interrupt: %d", ret);
		return;
	}

#if defined(CONFIG_ICM20948_TRIGGER_OWN_THREAD)
	k_sem_give(&data->irq_sem);
#elif defined(CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->work);
#endif
}

static void icm20948_handle_irq(const struct device *dev)
{
	const struct icm20948_config *cfg = dev->config;
	struct icm20948_data *data = dev->data;
	int ret;

	if (data->drdy_handler != NULL) {
		data->drdy_handler(dev, data->drdy_trigger);
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("failed to unmask the interrupt: %d", ret);
	}
}

#ifdef CONFIG_ICM20948_TRIGGER_OWN_THREAD
static void icm20948_thread(void *p1, void *p2, void *p3)
{
	struct icm20948_data *data = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_sem_take(&data->irq_sem, K_FOREVER);
		icm20948_handle_irq(data->dev);
	}
}
#endif

#ifdef CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD
static void icm20948_work_cb(struct k_work *work)
{
	struct icm20948_data *data = CONTAINER_OF(work, struct icm20948_data, work);

	icm20948_handle_irq(data->dev);
}
#endif

int icm20948_trigger_init(const struct device *dev)
{
	const struct icm20948_config *cfg = dev->config;
	struct icm20948_data *data = dev->data;
	int ret;

	if (!gpio_is_ready_dt(&cfg->irq)) {
		LOG_ERR("interrupt line is not ready");
		return -ENODEV;
	}

	data->dev = dev;

	ret = gpio_pin_configure_dt(&cfg->irq, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("failed to configure the interrupt line: %d", ret);
		return ret;
	}

	gpio_init_callback(&data->irq_cb, icm20948_irq_callback, BIT(cfg->irq.pin));

	ret = gpio_add_callback(cfg->irq.port, &data->irq_cb);
	if (ret < 0) {
		LOG_ERR("failed to add the interrupt callback: %d", ret);
		return ret;
	}

	/*
	 * Out of reset the pin is active high, push-pull and pulsed, which is
	 * what the edge the callback is armed for expects. Say so anyway, as
	 * the device is not necessarily coming out of a power cycle.
	 */
	ret = icm20948_write(dev, REG_INT_PIN_CFG, 0);
	if (ret < 0) {
		LOG_ERR("failed to configure the interrupt pin: %d", ret);
		return ret;
	}

	ret = icm20948_write(dev, REG_INT_ENABLE_1, BIT_RAW_DATA_0_RDY_EN);
	if (ret < 0) {
		LOG_ERR("failed to enable the data ready interrupt: %d", ret);
		return ret;
	}

#if defined(CONFIG_ICM20948_TRIGGER_OWN_THREAD)
	k_sem_init(&data->irq_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&data->thread, data->thread_stack, CONFIG_ICM20948_THREAD_STACK_SIZE,
			icm20948_thread, data, NULL, NULL,
			K_PRIO_COOP(CONFIG_ICM20948_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&data->thread, "icm20948");
#elif defined(CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD)
	data->work.handler = icm20948_work_cb;
#endif

	return 0;
}
