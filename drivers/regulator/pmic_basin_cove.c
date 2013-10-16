/*
 * pmic_basin_cove.c - Merrifield regulator driver
 * Copyright (c) 2013, Intel Corporation.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/intel_basin_cove_pmic.h>
#include <linux/regulator/machine.h>

#include <asm/intel_scu_pmic.h>

/* Intel Voltage cntrl register parameters*/
#define REG_ENA_STATUS_MASK	0x01
#define REG_VSEL_MASK		0xc0
#define VSEL_SHIFT		6

#define REG_ON			0x01
#define REG_OFF			0xfe

const u16 reg_addr_offset[] = { VPROG1CNT_ADDR, VPROG2CNT_ADDR,
					 VPROG3CNT_ADDR };

/**
* intel_pmic_reg_is_enabled - To check if the regulator is enabled
* @rdev:    regulator_dev structure
* @return value : 1 - Regulator is ON
*			  :0 - Regulator is OFF
*/
static int intel_pmic_reg_is_enabled(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	u8 reg;
	int ret;

	/*FIXME: Is it ok to use the following IPC API*/
	ret = intel_scu_ipc_ioread8(pmic_info->pmic_reg, &reg);
	if (ret) {
		dev_err(&rdev->dev,
			"intel_scu_ipc_ioread8 returns error %08x\n", ret);
		return ret;
	}

	return reg & REG_ENA_STATUS_MASK;
}
/**
* intel_pmic_reg_enable - To enable the regulator
* @rdev:    regulator_dev structure
* @return value : 0 - Regulator enabling success
*			:1 - Regulator enabling failed
*/
static int intel_pmic_reg_enable(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	u8  reg;
	int ret;

	ret = intel_scu_ipc_ioread8(pmic_info->pmic_reg, &reg);
	if (ret) {
		dev_err(&rdev->dev,
			"intel_scu_ipc_ioread8 returns error %08x\n", ret);
		return ret;
	}
	return intel_scu_ipc_iowrite8(pmic_info->pmic_reg, (reg | REG_ON));
}
/**
* intel_pmic_reg_disable - To disable the regulator
* @rdev:    regulator_dev structure
* @return value :0 - Regulator disabling success
*			:1 - Regulator disabling failed
*/
static int intel_pmic_reg_disable(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	u8  reg;
	int ret;

	ret = intel_scu_ipc_ioread8(pmic_info->pmic_reg, &reg);
	if (ret) {
		dev_err(&rdev->dev,
			"intel_scu_ipc_ioread8 returns error %08x\n", ret);
		return ret;
	}
	return intel_scu_ipc_iowrite8(pmic_info->pmic_reg,
		(reg & REG_OFF));
}
/**
* intel_pmic_reg_listvoltage - Return the voltage value,this is called
*                                   from core framework
* @rdev: regulator source
* @index : passed on from core
* @return value : Returns the value in micro volts.
 */
static int intel_pmic_reg_listvoltage(struct regulator_dev *rdev,
								unsigned index)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);

	if (index >= pmic_info->table_len) {
		dev_err(&rdev->dev, "Index out of range in listvoltage\n");
		return -EINVAL;
	}
	return pmic_info->table[index] * 1000;
}
/**
* intel_pmic_reg_getvoltage - Return the current voltage value in  uV
* @rdev:    regulator_dev structure
*  @return value : Returns the voltage value.
*/
static int intel_pmic_reg_getvoltage(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	u8  reg, vsel;
	int ret;

	ret = intel_scu_ipc_ioread8(pmic_info->pmic_reg, &reg);
	if (ret) {
		dev_err(&rdev->dev,
			"intel_scu_ipc_ioread8 returns error %08x\n", ret);
		return ret;
	}
	vsel = (reg & REG_VSEL_MASK) >> VSEL_SHIFT;
	if (vsel >= pmic_info->table_len) {
		dev_err(&rdev->dev, "vsel value is out of range\n");
		return -EINVAL;
	}
	dev_dbg(&rdev->dev, "Voltage value is %d mV\n",
		pmic_info->table[vsel]);
	return pmic_info->table[vsel] * 1000;
}

/**
* intel_pmic_reg_setvoltage - Set voltage to the regulator
* @rdev:    regulator_dev structure
* @min_uV: Minimum required voltage in uV
* @max_uV: Maximum acceptable voltage in uV
* @selector: Voltage value passed back to core layer
* Sets a voltage regulator to the desired output voltage
* @return value : Returns 0 if success
*			: Return error value on failure
*/
static int intel_pmic_reg_setvoltage(struct regulator_dev *rdev, int min_uV,
					int max_uV, unsigned *selector)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	int ret;
	u8 reg, vsel;

	for (vsel = 0; vsel < pmic_info->table_len; vsel++) {
		int mV = pmic_info->table[vsel];
		int uV = mV * 1000;
		if (min_uV > uV || uV > max_uV)
			continue;

		*selector = vsel;
		ret = intel_scu_ipc_ioread8(pmic_info->pmic_reg, &reg);
		if (ret) {
			dev_err(&rdev->dev,
				"intel_scu_ipc_ioread8 error %08x\n", ret);
			return ret;
		}
		reg &= ~REG_VSEL_MASK;
		reg |= vsel << VSEL_SHIFT;
		dev_dbg(&rdev->dev,
			"intel_pmic_reg_setvoltage voltage: %u uV\n", uV);
		return intel_scu_ipc_iowrite8(pmic_info->pmic_reg, reg);
	}
	return -EINVAL;
}

/* regulator_ops registration */
static struct regulator_ops intel_pmic_ops = {
	.is_enabled = intel_pmic_reg_is_enabled,
	.enable = intel_pmic_reg_enable,
	.disable = intel_pmic_reg_disable,
	.get_voltage = intel_pmic_reg_getvoltage,
	.set_voltage = intel_pmic_reg_setvoltage,
	.list_voltage = intel_pmic_reg_listvoltage,
};
/**
* struct regulator_desc - Regulator descriptor
* Each regulator registered with the core is described with a structure of
* this type.
* @name: Identifying name for the regulator.
* @id: Numerical identifier for the regulator.
* @n_voltages: Number of selectors available for ops.list_voltage().
* @ops: Regulator operations table.
* @irq: Interrupt number for the regulator.
* @type: Indicates if the regulator is a voltage or current regulator.
* @owner: Module providing the regulator, used for refcounting.
*/
static struct regulator_desc intel_pmic_desc[] = {
	{
		.name = "vprog1",
		.id = VPROG1,
		.ops = &intel_pmic_ops,
		.n_voltages = ARRAY_SIZE(VPROG1_VSEL_table),
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "vprog2",
		.id = VPROG2,
		.ops = &intel_pmic_ops,
		.n_voltages = ARRAY_SIZE(VPROG2_VSEL_table),
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "vprog3",
		.id = VPROG3,
		.ops = &intel_pmic_ops,
		.n_voltages = ARRAY_SIZE(VPROG3_VSEL_table),
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

static int basin_cove_pmic_probe(struct platform_device *pdev)
{
	struct intel_pmic_info *pdata = dev_get_platdata(&pdev->dev);
	struct regulator_config config = { };
	unsigned int i;

	if (!pdata || !pdata->pmic_reg)
		return -EINVAL;

	config.dev = &pdev->dev;
	config.init_data = pdata->init_data;
	config.driver_data = pdata;

	for (i = 0; i < ARRAY_SIZE(reg_addr_offset); i++) {
		if (reg_addr_offset[i] == pdata->pmic_reg)
			break;
	}
	if (i == (ARRAY_SIZE(reg_addr_offset)))
		return -EINVAL;

	pdata->intel_pmic_rdev =
	regulator_register(&intel_pmic_desc[i], &config);
	if (IS_ERR(pdata->intel_pmic_rdev)) {
		dev_err(&pdev->dev, "can't register regulator..error %ld\n",
				PTR_ERR(pdata->intel_pmic_rdev));
		return PTR_ERR(pdata->intel_pmic_rdev);
	}
	platform_set_drvdata(pdev, pdata->intel_pmic_rdev);
	dev_dbg(&pdev->dev, "registered regulator\n");
	return 0;
}

static int basin_cove_pmic_remove(struct platform_device *pdev)
{
	regulator_unregister(platform_get_drvdata(pdev));
	return 0;
}

static const struct platform_device_id basin_cove_id_table[] = {
	{ "intel_regulator", 0 },
	{ },
};

MODULE_DEVICE_TABLE(platform, basin_cove_id_table);

static struct platform_driver basin_cove_pmic_driver = {
	.driver		= {
		.name = "intel_regulator",
		.owner = THIS_MODULE,
	},
	.probe = basin_cove_pmic_probe,
	.remove = basin_cove_pmic_remove,
	.id_table = basin_cove_id_table,
};
static int __init basin_cove_pmic_init(void)
{
	return platform_driver_register(&basin_cove_pmic_driver);
}
subsys_initcall(basin_cove_pmic_init);

static void __exit basin_cove_pmic_exit(void)
{
	platform_driver_unregister(&basin_cove_pmic_driver);
}
module_exit(basin_cove_pmic_exit);

MODULE_DESCRIPTION("Basin Cove voltage regulator driver");
MODULE_AUTHOR("Vishwesh/Mahesh/Sudarshan");
MODULE_LICENSE("GPL v2");
