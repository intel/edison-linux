/*
 * intel_mid_thermal.c - Intel MID platform thermal driver
 *
 * Copyright (C) 2010 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Ananth Krishna <ananth.krishna.r@intel.com>
 * Author: Durgadoss <durgadoss.r@intel.com>
 */

#define pr_fmt(fmt)  "intel_mid_thermal: " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/param.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>

#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/thermal.h>

#include <asm/intel_scu_ipc.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/intel_mid_thermal.h>
#include <asm/intel_mid_rpmsg.h>

#define DRIVER_NAME "msic_thermal"

/* Cooling device attributes */
#define SOC_IPC_COMMAND		0xCF

enum {
	NORMAL = 0,
	WARNING,
	ALERT,
	CRITICAL
} thermal_state;

enum {
	SOC_SKIN_NORMAL = 0,
	SOC_SKIN_WARM = 2,
	SOC_SKIN_PROCHOT,
	SOC_MAX_STATES
} soc_skin_state;

/* MSIC die attributes */
#define MSIC_DIE_ADC_MIN	488
#define MSIC_DIE_ADC_MAX	1004

#define TABLE_LENGTH 24
/*
 * ADC code vs Temperature table
 * This table will be different for different thermistors
 * Row 0: ADC code
 * Row 1: Temperature (in degree celsius)
 */
static const int adc_code[2][TABLE_LENGTH] = {
	{977, 961, 941, 917, 887, 853, 813, 769, 720, 669, 615, 561, 508, 456,
		407, 357, 315, 277, 243, 212, 186, 162, 140, 107},
	{-20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60,
		65, 70, 75, 80, 85, 90, 100},
	};

struct ts_cache_info {
	bool is_cached_data_initialized;
	struct mutex lock;
	int *cached_values;
	unsigned long last_updated;
};

struct soc_cooling_device_info {
	unsigned long soc_state;
	struct mutex lock_cool_state;
};

static struct soc_cooling_device_info soc_cdev_info;

struct platform_info {
	struct platform_device *pdev;
	struct thermal_zone_device **tzd;
	struct ts_cache_info cacheinfo;
	/* ADC handle used to read sensor temperature values */
	void *therm_adc_handle;
	struct thermal_cooling_device *soc_cdev;
	int num_sensors;
	int soc_cooling;
	struct intel_mid_thermal_sensor *sensors;
};

static struct platform_info *platforminfo;

struct thermal_device_info {
	struct intel_mid_thermal_sensor *sensor;
};

/* SoC cooling device callbacks */
static int soc_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	/* SoC has 4 levels of throttling from 0 to 3 */
	*state = SOC_MAX_STATES - 1;
	return 0;
}

static int soc_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	mutex_lock(&soc_cdev_info.lock_cool_state);
	*state = soc_cdev_info.soc_state;
	mutex_unlock(&soc_cdev_info.lock_cool_state);
	return 0;
}

static int soc_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long state)
{
	int ret;
	if (state > SOC_MAX_STATES - 1) {
		pr_err("Invalid SoC throttle state:%ld\n", state);
		return -EINVAL;
	}

	switch (state) {
	/* SoC De-Throttle */
	case NORMAL:
		state = SOC_SKIN_NORMAL;
		break;
	case WARNING:
		/*
		 * New state is assigned based on present state.
		 * State 1 can be reached from state 0 or 2.
		 * State 0 to 1 means skin WARM.
		 * state 2 to 1 means skin no longer PROCHOT but WARM
		 */
		state = SOC_SKIN_WARM;
		break;
	/* SoC Throttle, PROCHOT */
	case ALERT:
	case CRITICAL:
		state = SOC_SKIN_PROCHOT;
		break;
	}
	/* Send IPC command to throttle SoC */
	mutex_lock(&soc_cdev_info.lock_cool_state);
	ret = rpmsg_send_generic_command(SOC_IPC_COMMAND, 0,
			(u8 *) &state, 4, NULL, 0);
	if (ret)
		pr_err("IPC_COMMAND failed: %d\n", ret);
	else
		soc_cdev_info.soc_state = state;

	mutex_unlock(&soc_cdev_info.lock_cool_state);
	return ret;
}

static struct thermal_cooling_device_ops soc_cooling_ops = {
	.get_max_state = soc_get_max_state,
	.get_cur_state = soc_get_cur_state,
	.set_cur_state = soc_set_cur_state,
};

static int register_soc_as_cdev(void)
{
	int ret = 0;
	platforminfo->soc_cdev = thermal_cooling_device_register("SoC", NULL,
						&soc_cooling_ops);
	if (IS_ERR(platforminfo->soc_cdev)) {
		ret = PTR_ERR(platforminfo->soc_cdev);
		platforminfo->soc_cdev = NULL;
	}
	return ret;
}

static void unregister_soc_as_cdev(void)
{
	thermal_cooling_device_unregister(platforminfo->soc_cdev);
}

/**
 * is_valid_adc - checks whether the adc code is within the defined range
 * @min: minimum value for the sensor
 * @max: maximum value for the sensor
 *
 * Can sleep
 */
static int is_valid_adc(uint16_t adc_val, uint16_t min, uint16_t max)
{
	return (adc_val >= min) && (adc_val <= max);
}

/**
 * find_adc_code - searches the ADC code using binary search
 * @val: value to find in the array
 *
 * This function does binary search on an array sorted in 'descending' order
 * Can sleep
 */
static int find_adc_code(uint16_t val)
{
	int left = 0;
	int right = TABLE_LENGTH - 1;
	int mid;
	while (left <= right) {
		mid = (left + right)/2;
		if (val == adc_code[0][mid] ||
			(mid > 0 &&
			val > adc_code[0][mid] && val < adc_code[0][mid-1]))
			return mid;
		else if (val > adc_code[0][mid])
			right = mid - 1;
		else if (val < adc_code[0][mid])
			left = mid + 1;
	}
	return -1;
}

/**
 * linear_interpolate - does interpolation to find temperature
 * Returns the temperature in milli degree celsius
 * @adc_val: ADC code(x) at which temperature(y) should be found
 * @indx: index of the minimum(x0) of the two ADC codes
 *
 * Can sleep
 */
static int linear_interpolate(int indx, uint16_t adc_val)
{
	int x = adc_val;
	int x0 = adc_code[0][indx];
	int x1 = adc_code[0][indx - 1];
	int y0 = adc_code[1][indx];
	int y1 = adc_code[1][indx - 1];

	/*
	 * Find y:
	 * Of course, we can avoid these variables, but keep them
	 * for readability and maintainability.
	 */
	int numerator = (x-x0)*y1 + (x1-x)*y0;
	int denominator = x1-x0;

	/*
	 * We have to report the temperature in milli degree celsius.
	 * So, to reduce the loss of precision, do (Nr*1000)/Dr, instead
	 * of (Nr/Dr)*1000.
	 */
	 return (numerator * 1000)/denominator;
}

/**
 * adc_to_temp - converts the ADC code to temperature in C
 * @direct: true if ths channel is direct index
 * @adc_val: the adc_val that needs to be converted
 * @tp: temperature return value
 *
 * Can sleep
 */
static int adc_to_temp(struct intel_mid_thermal_sensor *sensor,
		uint16_t adc_val, long *tp)
{
	int indx;

	/* Direct conversion for msic die temperature */
	if (sensor->direct) {
		if (is_valid_adc(adc_val, MSIC_DIE_ADC_MIN, MSIC_DIE_ADC_MAX)) {
			*tp = sensor->slope * adc_val - sensor->intercept;
			return 0;
		}
		return -ERANGE;
	}

	indx = find_adc_code(adc_val);
	if (indx < 0)
		return -ERANGE;

	if (adc_code[0][indx] == adc_val) {
		/* Convert temperature in celsius to milli degree celsius */
		*tp = adc_code[1][indx] * 1000;
		return 0;
	}

	/*
	 * The ADC code is in between two values directly defined in the
	 * table. So, do linear interpolation to calculate the temperature.
	 */
	*tp = linear_interpolate(indx, adc_val);
	return 0;
}

int skin0_temp_correlation(void *info, long temp, long *res)
{
	struct intel_mid_thermal_sensor *sensor = info;

	*res = ((temp * sensor->slope) / 1000) + sensor->intercept;

	return 0;
}

int bptherm_temp_correlation(void *info, long temp, long *res)
{
	struct intel_mid_thermal_sensor *sensor = info;

	*res = ((temp * sensor->slope) / 1000) + sensor->intercept;

	return 0;
}

int skin1_temp_correlation(void *info, long temp, long *res)
{
	struct intel_mid_thermal_sensor *sensor = info;
	struct intel_mid_thermal_sensor *dsensor; /* dependent sensor */
	struct skin1_private_info *skin_info;
	long sensor_temp = 0, curr_temp;
	int ret, index;

	skin_info = sensor->priv;

	*res = ((temp * sensor->slope) / 1000) + sensor->intercept;

	/* If we do not have dependent sensors, just return. Not an error */
	if (!skin_info || !skin_info->dependent || !skin_info->sensors)
		return 0;

	for (index = 0; index < skin_info->dependent; index++) {
		if (!skin_info->sensors[index])
			continue;

		dsensor = skin_info->sensors[index];

		ret = adc_to_temp(dsensor,
			platforminfo->cacheinfo.cached_values[dsensor->index],
			&curr_temp);
		if (ret)
			return ret;

		if (dsensor->temp_correlation)
			dsensor->temp_correlation(dsensor, curr_temp,
						&sensor_temp);

		if (sensor_temp > *res)
			*res = sensor_temp;
	}

	return 0;
}

/**
 * mid_read_temp - read sensors for temperature
 * @temp: holds the current temperature for the sensor after reading
 *
 * reads the adc_code from the channel and converts it to real
 * temperature. The converted value is stored in temp.
 *
 * Can sleep
 */
static int mid_read_temp(struct thermal_zone_device *tzd, long *temp)
{
	struct thermal_device_info *td_info = tzd->devdata;
	int ret;
	long curr_temp;
	int indx = td_info->sensor->index; /* Required Index */

	mutex_lock(&platforminfo->cacheinfo.lock);

	if (!platforminfo->cacheinfo.is_cached_data_initialized ||
	time_after(jiffies, platforminfo->cacheinfo.last_updated + HZ)) {
		ret = get_gpadc_sample(platforminfo->therm_adc_handle, 1,
					platforminfo->cacheinfo.cached_values);
		if (ret)
			goto exit;
		platforminfo->cacheinfo.last_updated = jiffies;
		platforminfo->cacheinfo.is_cached_data_initialized = true;
	}

	/* Convert ADC value to temperature */
	ret = adc_to_temp(td_info->sensor,
		platforminfo->cacheinfo.cached_values[indx], &curr_temp);
	if (ret)
		goto exit;

	if (td_info->sensor->temp_correlation)
		ret = td_info->sensor->temp_correlation(td_info->sensor,
							curr_temp, temp);
	else
		*temp = curr_temp;

exit:
	mutex_unlock(&platforminfo->cacheinfo.lock);
	return ret;
}

/**
 * initialize_sensor - Initializes ADC information for each sensor.
 * @index: index of the sensor
 *
 * Context: can sleep
 */
static struct thermal_device_info *initialize_sensor(
			struct intel_mid_thermal_sensor *sensor)
{
	struct thermal_device_info *td_info =
		kzalloc(sizeof(struct thermal_device_info), GFP_KERNEL);

	if (!td_info)
		return NULL;

	td_info->sensor = sensor;

	return td_info;
}

/**
 * mid_thermal_resume - resume routine
 * @dev: device structure
 */
static int mid_thermal_resume(struct device *dev)
{
	return 0;
}

/**
 * mid_thermal_suspend - suspend routine
 * @dev: device structure
 */
static int mid_thermal_suspend(struct device *dev)
{
	return 0;
}

#ifdef CONFIG_DEBUG_THERMAL
static int read_slope(struct thermal_zone_device *tzd, long *slope)
{
	struct thermal_device_info *td_info = tzd->devdata;

	*slope = td_info->sensor->slope;

	return 0;
}

static int update_slope(struct thermal_zone_device *tzd, long slope)
{
	struct thermal_device_info *td_info = tzd->devdata;

	td_info->sensor->slope = slope;

	return 0;
}

static int read_intercept(struct thermal_zone_device *tzd, long *intercept)
{
	struct thermal_device_info *td_info = tzd->devdata;

	*intercept = td_info->sensor->intercept;

	return 0;
}

static int update_intercept(struct thermal_zone_device *tzd, long intercept)
{
	struct thermal_device_info *td_info = tzd->devdata;

	td_info->sensor->intercept = intercept;

	return 0;
}
#endif

/**
 * read_curr_temp - reads the current temperature and stores in temp
 * @temp: holds the current temperature value after reading
 *
 * Can sleep
 */
static int read_curr_temp(struct thermal_zone_device *tzd, long *temp)
{
	return (tzd) ? mid_read_temp(tzd, temp) : -EINVAL;
}

/* Can't be const */
static struct thermal_zone_device_ops tzd_ops = {
	.get_temp = read_curr_temp,
#ifdef CONFIG_DEBUG_THERMAL
	.get_slope = read_slope,
	.set_slope = update_slope,
	.get_intercept = read_intercept,
	.set_intercept = update_intercept,
#endif
};

/**
 * mid_thermal_probe - mfld thermal initialize
 * @pdev: platform device structure
 *
 * mid thermal probe initializes the hardware and registers
 * all the sensors with the generic thermal framework. Can sleep.
 */
static int mid_thermal_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	int *adc_channel_info;
	struct intel_mid_thermal_platform_data *pdata;

	pdata = pdev->dev.platform_data;

	if (!pdata)
		return -EINVAL;

	platforminfo = kzalloc(sizeof(struct platform_info), GFP_KERNEL);

	if (!platforminfo)
		return -ENOMEM;

	platforminfo->num_sensors = pdata->num_sensors;
	platforminfo->soc_cooling = pdata->soc_cooling;
	platforminfo->sensors = pdata->sensors;

	platforminfo->tzd = kzalloc(
	(sizeof(struct thermal_zone_device *) * platforminfo->num_sensors),
		 GFP_KERNEL);

	if (!platforminfo->tzd)
		goto platforminfo_alloc_fail;

	platforminfo->cacheinfo.cached_values =
		kzalloc((sizeof(int) * platforminfo->num_sensors), GFP_KERNEL);

	if (!platforminfo->cacheinfo.cached_values)
		goto tzd_alloc_fail;

	adc_channel_info = kzalloc((sizeof(int) * platforminfo->num_sensors),
			GFP_KERNEL);

	if (!adc_channel_info)
		goto cachedinfo_alloc_fail;

	/* initialize mutex locks */
	mutex_init(&platforminfo->cacheinfo.lock);


	if (platforminfo->soc_cooling)
		mutex_init(&soc_cdev_info.lock_cool_state);

	for (i = 0; i < platforminfo->num_sensors; i++)
		adc_channel_info[i] = platforminfo->sensors[i].adc_channel;

	/* Allocate ADC channels for all sensors */
	platforminfo->therm_adc_handle = gpadc_alloc_channels(
				platforminfo->num_sensors, adc_channel_info);

	if (!platforminfo->therm_adc_handle) {
		ret = -ENOMEM;
		goto adc_channel_alloc_fail;
	}

	/* Register each sensor with the generic thermal framework*/
	for (i = 0; i < platforminfo->num_sensors; i++) {
		platforminfo->tzd[i] = thermal_zone_device_register(
				platforminfo->sensors[i].name, 0, 0,
				initialize_sensor(&platforminfo->sensors[i]),
				&tzd_ops, NULL, 0, 0);
		if (IS_ERR(platforminfo->tzd[i]))
			goto reg_fail;
	}

	platforminfo->pdev = pdev;

	platform_set_drvdata(pdev, platforminfo);

	/* Register SoC as a cooling device */
	if (platforminfo->soc_cooling) {
		ret = register_soc_as_cdev();
		/* Log this, but keep the driver loaded */
		if (ret) {
			dev_err(&pdev->dev,
				"register_soc_as_cdev failed:%d\n", ret);
		}
	}

	kfree(adc_channel_info);

	return 0;

reg_fail:
	ret = PTR_ERR(platforminfo->tzd[i]);
	while (--i >= 0)
		thermal_zone_device_unregister(platforminfo->tzd[i]);
adc_channel_alloc_fail:
	kfree(adc_channel_info);
cachedinfo_alloc_fail:
	kfree(platforminfo->cacheinfo.cached_values);
tzd_alloc_fail:
	kfree(platforminfo->tzd);
platforminfo_alloc_fail:
	kfree(platforminfo);
	return ret;
}

/**
 * mid_thermal_remove - mfld thermal finalize
 * @dev: platform device structure
 *
 * MLFD thermal remove unregisters all the sensors from the generic
 * thermal framework. Can sleep.
 */
static int mid_thermal_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < platforminfo->num_sensors; i++)
		thermal_zone_device_unregister(platforminfo->tzd[i]);

	/* Unregister SoC as cooling device */
	if (platforminfo->soc_cooling)
		unregister_soc_as_cdev();

	/* Free the allocated ADC channels */
	if (platforminfo->therm_adc_handle)
		intel_mid_gpadc_free(platforminfo->therm_adc_handle);

	kfree(platforminfo->cacheinfo.cached_values);
	kfree(platforminfo->tzd);
	kfree(platforminfo);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

/*********************************************************************
 *		Driver initialisation and finalization
 *********************************************************************/


/* Platfrom device functionality */

static const struct dev_pm_ops msic_thermal_pm_ops = {
	.suspend = mid_thermal_suspend,
	.resume = mid_thermal_resume,
};

static const struct platform_device_id mid_therm_table[] = {
	{ DRIVER_NAME, 1 },
};

static struct platform_driver mid_therm_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &msic_thermal_pm_ops,
	},
	.probe = mid_thermal_probe,
	.remove = mid_thermal_remove,
	.id_table = mid_therm_table,
};

static int mid_therm_module_init(void)
{
	return platform_driver_register(&mid_therm_driver);
}

static void mid_therm_module_exit(void)
{
	platform_driver_unregister(&mid_therm_driver);
}


/* RPMSG related functionality */

static int mid_therm_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;
	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed mid_therm rpmsg device\n");

	ret = mid_therm_module_init();
out:
	return ret;
}

static void mid_therm_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	mid_therm_module_exit();
	dev_info(&rpdev->dev, "Removed mid_therm rpmsg device\n");
}

static void mid_therm_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
			int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
				data, len, true);
}

static struct rpmsg_device_id mid_therm_id_table[] = {
	{ .name = "rpmsg_mid_thermal" },
	{ },
};

MODULE_DEVICE_TABLE(rpmsg, mid_therm_id_table);

static struct rpmsg_driver mid_therm_rpmsg_driver = {
	.drv.name	= DRIVER_NAME,
	.drv.owner	= THIS_MODULE,
	.probe		= mid_therm_rpmsg_probe,
	.callback	= mid_therm_rpmsg_cb,
	.remove		= mid_therm_rpmsg_remove,
	.id_table	= mid_therm_id_table,
};

static int __init mid_therm_rpmsg_init(void)
{
	return register_rpmsg_driver(&mid_therm_rpmsg_driver);
}

static void __exit mid_therm_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&mid_therm_rpmsg_driver);
}


/* Changing _init call to make the thermal driver
 * load _after_ the GPADC driver
 * module_init(mid_therm_rpmsg_init);
 */
late_initcall(mid_therm_rpmsg_init);
module_exit(mid_therm_rpmsg_exit);

MODULE_AUTHOR("Durgadoss R <durgadoss.r@intel.com>");
MODULE_DESCRIPTION("Intel Medfield Platform Thermal Driver");
MODULE_LICENSE("GPL");
