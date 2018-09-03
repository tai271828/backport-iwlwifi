/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2007 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * Copyright(c) 2016 Intel Deutschland GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2005 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/pci.h>
#include <linux/pci-aspm.h>
#include <linux/acpi.h>

#include "iwl-trans.h"
#include "iwl-drv.h"
#include "internal.h"

#define IWL_PCI_DEVICE(dev, subdev, cfg) \
	.vendor = PCI_VENDOR_ID_INTEL,  .device = (dev), \
	.subvendor = PCI_ANY_ID, .subdevice = (subdev), \
	.driver_data = (kernel_ulong_t)&(cfg)

/* Hardware specific file defines the PCI IDs table for that hardware module */
static const struct pci_device_id iwl_hw_card_ids[] = {

#if IS_ENABLED(CPTCFG_IWLMVM)
/* 8000 Series */
	{IWL_PCI_DEVICE(0x24FD, 0x0050, iwl8265_2ac_cfg)},
#endif /* CPTCFG_IWLMVM */

	{0}
};
MODULE_DEVICE_TABLE(pci, iwl_hw_card_ids);

#ifdef CONFIG_ACPI
#define SPL_METHOD		"SPLC"
#define SPL_DOMAINTYPE_MODULE	BIT(0)
#define SPL_DOMAINTYPE_WIFI	BIT(1)
#define SPL_DOMAINTYPE_WIGIG	BIT(2)
#define SPL_DOMAINTYPE_RFEM	BIT(3)

static u64 splx_get_pwr_limit(struct iwl_trans *trans, union acpi_object *splx)
{
	union acpi_object *limits, *domain_type, *power_limit;

	if (splx->type != ACPI_TYPE_PACKAGE ||
	    splx->package.count != 2 ||
	    splx->package.elements[0].type != ACPI_TYPE_INTEGER ||
	    splx->package.elements[0].integer.value != 0) {
		IWL_ERR(trans, "Unsupported splx structure\n");
		return 0;
	}

	limits = &splx->package.elements[1];
	if (limits->type != ACPI_TYPE_PACKAGE ||
	    limits->package.count < 2 ||
	    limits->package.elements[0].type != ACPI_TYPE_INTEGER ||
	    limits->package.elements[1].type != ACPI_TYPE_INTEGER) {
		IWL_ERR(trans, "Invalid limits element\n");
		return 0;
	}

	domain_type = &limits->package.elements[0];
	power_limit = &limits->package.elements[1];
	if (!(domain_type->integer.value & SPL_DOMAINTYPE_WIFI)) {
		IWL_DEBUG_INFO(trans, "WiFi power is not limited\n");
		return 0;
	}

	static char *tai_message = "world";
	printk(KERN_INFO "TAI INFO: %s from splx_get_pwr_limit\n", tai_message);
	printk(KERN_INFO "TAI INFO: CONFIG_ACPI is defined\n");

	return power_limit->integer.value;
}

static void set_dflt_pwr_limit(struct iwl_trans *trans, struct pci_dev *pdev)
{
	acpi_handle pxsx_handle;
	acpi_handle handle;
	struct acpi_buffer splx = {ACPI_ALLOCATE_BUFFER, NULL};
	acpi_status status;

	pxsx_handle = ACPI_HANDLE(&pdev->dev);
	if (!pxsx_handle) {
		IWL_DEBUG_INFO(trans,
			       "Could not retrieve root port ACPI handle\n");
		return;
	}

	/* Get the method's handle */
	status = acpi_get_handle(pxsx_handle, (acpi_string)SPL_METHOD, &handle);
	if (ACPI_FAILURE(status)) {
		IWL_DEBUG_INFO(trans, "SPL method not found\n");
		return;
	}

	/* Call SPLC with no arguments */
	status = acpi_evaluate_object(handle, NULL, NULL, &splx);
	if (ACPI_FAILURE(status)) {
		IWL_ERR(trans, "SPLC invocation failed (0x%x)\n", status);
		return;
	}

	trans->dflt_pwr_limit = splx_get_pwr_limit(trans, splx.pointer);
	IWL_DEBUG_INFO(trans, "Default power limit set to %lld\n",
		       trans->dflt_pwr_limit);
	kfree(splx.pointer);
}

#else /* CONFIG_ACPI */
static void set_dflt_pwr_limit(struct iwl_trans *trans, struct pci_dev *pdev) {

	static char *tai_message = "world";
	printk(KERN_INFO "TAI INFO: %s from splx_get_pwr_limit\n", tai_message);
	printk(KERN_INFO "TAI INFO: CONFIG_ACPI is not defined\n");

}
#endif

/* PCI registers */
#define PCI_CFG_RETRY_TIMEOUT	0x041

static int iwl_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	const struct iwl_cfg *cfg = (struct iwl_cfg *)(ent->driver_data);
	struct iwl_trans *iwl_trans;
	struct iwl_trans_pcie *trans_pcie;
	int ret;

	iwl_trans = iwl_trans_pcie_alloc(pdev, ent, cfg);
	if (IS_ERR(iwl_trans))
		return PTR_ERR(iwl_trans);

	pci_set_drvdata(pdev, iwl_trans);

	trans_pcie = IWL_TRANS_GET_PCIE_TRANS(iwl_trans);
	trans_pcie->drv = iwl_drv_start(iwl_trans, cfg);

	if (IS_ERR(trans_pcie->drv)) {
		ret = PTR_ERR(trans_pcie->drv);
		goto out_free_trans;
	}

	set_dflt_pwr_limit(iwl_trans, pdev);

	/* register transport layer debugfs here */
	ret = iwl_trans_pcie_dbgfs_register(iwl_trans);
	if (ret)
		goto out_free_drv;

	/* if RTPM is in use, enable it in our device */
	if (iwl_trans->runtime_pm_mode != IWL_PLAT_PM_MODE_DISABLED) {
		/* We explicitly set the device to active here to
		 * clear contingent errors.
		 */
		pm_runtime_set_active(&pdev->dev);

		pm_runtime_set_autosuspend_delay(&pdev->dev,
					 iwlwifi_mod_params.d0i3_entry_delay);
		pm_runtime_use_autosuspend(&pdev->dev);

		/* We are not supposed to call pm_runtime_allow() by
		 * ourselves, but let userspace enable runtime PM via
		 * sysfs.  However, since we don't enable this from
		 * userspace yet, we need to allow/forbid() ourselves.
		*/
		pm_runtime_allow(&pdev->dev);
	}

	/* The PCI device starts with a reference taken and we are
	 * supposed to release it here.  But to simplify the
	 * interaction with the opmode, we don't do it now, but let
	 * the opmode release it when it's ready.
	 */
#ifdef CPTCFG_IWLMVM_WAKELOCK
	/* We always unref before ref'ing at the beginning, to free
	 * the RTPM initial reference.  So set the wakelock reference
	 * count to 1 here, to acoount for that.
	 */
	iwl_trans->wakelock_count = 1;
#endif

	static char *tai_message = "world";
	printk(KERN_INFO "TAI INFO: %s from iwl_pci_probe!\n", tai_message);

	return 0;

out_free_drv:
	iwl_drv_stop(trans_pcie->drv);
out_free_trans:
	iwl_trans_pcie_free(iwl_trans);
	return ret;
}

static void iwl_pci_remove(struct pci_dev *pdev)
{
	struct iwl_trans *trans = pci_get_drvdata(pdev);
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);

	/* if RTPM was in use, restore it to the state before probe */
	if (trans->runtime_pm_mode != IWL_PLAT_PM_MODE_DISABLED) {
		/* We should not call forbid here, but we do for now.
		 * Check the comment to pm_runtime_allow() in
		 * iwl_pci_probe().
		 */
		pm_runtime_forbid(trans->dev);
	}

	iwl_drv_stop(trans_pcie->drv);

	iwl_trans_pcie_free(trans);

	static char *tai_message = "world";
	printk(KERN_INFO "TAI INFO: %s from iwl_pci_remove!\n", tai_message);
}

#ifdef CONFIG_PM_SLEEP

static int iwl_pci_suspend(struct device *device)
{
	/* Before you put code here, think about WoWLAN. You cannot check here
	 * whether WoWLAN is enabled or not, and your code will run even if
	 * WoWLAN is enabled - don't kill the NIC, someone may need it in Sx.
	 */

	return 0;
}

static int iwl_pci_resume(struct device *device)
{
	struct pci_dev *pdev = to_pci_dev(device);
	struct iwl_trans *trans = pci_get_drvdata(pdev);
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	bool hw_rfkill;

	/* Before you put code here, think about WoWLAN. You cannot check here
	 * whether WoWLAN is enabled or not, and your code will run even if
	 * WoWLAN is enabled - the NIC may be alive.
	 */

	/*
	 * We disable the RETRY_TIMEOUT register (0x41) to keep
	 * PCI Tx retries from interfering with C3 CPU state.
	 */
	pci_write_config_byte(pdev, PCI_CFG_RETRY_TIMEOUT, 0x00);

	if (!trans->op_mode)
		return 0;

	/*
	 * Enable rfkill interrupt (in order to keep track of
	 * the rfkill status)
	 */
	iwl_enable_rfkill_int(trans);

	hw_rfkill = iwl_is_rfkill_set(trans);

	mutex_lock(&trans_pcie->mutex);
	iwl_trans_pcie_rf_kill(trans, hw_rfkill);
	mutex_unlock(&trans_pcie->mutex);

	return 0;
}

int iwl_pci_fw_enter_d0i3(struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	int ret;

	if (test_bit(STATUS_FW_ERROR, &trans->status))
		return 0;

	set_bit(STATUS_TRANS_GOING_IDLE, &trans->status);

	/* config the fw */
	ret = iwl_op_mode_enter_d0i3(trans->op_mode);
	if (ret == 1) {
		IWL_DEBUG_RPM(trans, "aborting d0i3 entrance\n");
		clear_bit(STATUS_TRANS_GOING_IDLE, &trans->status);
		return -EBUSY;
	}
	if (ret)
		goto err;

	ret = wait_event_timeout(trans_pcie->d0i3_waitq,
				 test_bit(STATUS_TRANS_IDLE, &trans->status),
				 msecs_to_jiffies(IWL_TRANS_IDLE_TIMEOUT));
	if (!ret) {
		IWL_ERR(trans, "Timeout entering D0i3\n");
		ret = -ETIMEDOUT;
		goto err;
	}

	clear_bit(STATUS_TRANS_GOING_IDLE, &trans->status);

	return 0;
err:
	clear_bit(STATUS_TRANS_GOING_IDLE, &trans->status);
	iwl_trans_fw_error(trans);
	return ret;
}

int iwl_pci_fw_exit_d0i3(struct iwl_trans *trans)
{
	struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
	int ret;

	/* sometimes a D0i3 entry is not followed through */
	if (!test_bit(STATUS_TRANS_IDLE, &trans->status))
		return 0;

	/* config the fw */
	ret = iwl_op_mode_exit_d0i3(trans->op_mode);
	if (ret)
		goto err;

	/* we clear STATUS_TRANS_IDLE only when D0I3_END command is completed */

	ret = wait_event_timeout(trans_pcie->d0i3_waitq,
				 !test_bit(STATUS_TRANS_IDLE, &trans->status),
				 msecs_to_jiffies(IWL_TRANS_IDLE_TIMEOUT));
	if (!ret) {
		IWL_ERR(trans, "Timeout exiting D0i3\n");
		ret = -ETIMEDOUT;
		goto err;
	}

	return 0;
err:
	clear_bit(STATUS_TRANS_IDLE, &trans->status);
	iwl_trans_fw_error(trans);
	return ret;
}

#ifdef CPTCFG_IWLWIFI_PCIE_RTPM
static int iwl_pci_runtime_suspend(struct device *device)
{
	struct pci_dev *pdev = to_pci_dev(device);
	struct iwl_trans *trans = pci_get_drvdata(pdev);
	int ret;

	IWL_DEBUG_RPM(trans, "entering runtime suspend\n");

	if (test_bit(STATUS_DEVICE_ENABLED, &trans->status)) {
		ret = iwl_pci_fw_enter_d0i3(trans);
		if (ret < 0)
			return ret;
	}

	trans->system_pm_mode = IWL_PLAT_PM_MODE_D0I3;

	iwl_trans_d3_suspend(trans, false, false);

	return 0;
}

static int iwl_pci_runtime_resume(struct device *device)
{
	struct pci_dev *pdev = to_pci_dev(device);
	struct iwl_trans *trans = pci_get_drvdata(pdev);
	enum iwl_d3_status d3_status;

	IWL_DEBUG_RPM(trans, "exiting runtime suspend (resume)\n");

	iwl_trans_d3_resume(trans, &d3_status, false, false);

	if (test_bit(STATUS_DEVICE_ENABLED, &trans->status))
		return iwl_pci_fw_exit_d0i3(trans);

	return 0;
}

static int iwl_pci_system_prepare(struct device *device)
{
	struct pci_dev *pdev = to_pci_dev(device);
	struct iwl_trans *trans = pci_get_drvdata(pdev);

	IWL_DEBUG_RPM(trans, "preparing for system suspend\n");

	/* This is called before entering system suspend and before
	 * the runtime resume is called.  Set the suspending flag to
	 * prevent the wakelock from being taken.
	 */
	trans->suspending = true;

	/* Wake the device up from runtime suspend before going to
	 * platform suspend.  This is needed because we don't know
	 * whether wowlan any is set and, if it's not, mac80211 will
	 * disconnect (in which case, we can't be in D0i3).
	 */
	pm_runtime_resume(device);

	return 0;
}

static void iwl_pci_system_complete(struct device *device)
{
	struct pci_dev *pdev = to_pci_dev(device);
	struct iwl_trans *trans = pci_get_drvdata(pdev);

	IWL_DEBUG_RPM(trans, "completing system suspend\n");

	/* This is called as a counterpart to the prepare op.  It is
	 * called either when suspending fails or when suspend
	 * completed successfully.  Now there's no risk of grabbing
	 * the wakelock anymore, so we can release the suspending
	 * flag.
	 */
	trans->suspending = false;
}
#endif /* CPTCFG_IWLWIFI_PCIE_RTPM */

static const struct dev_pm_ops iwl_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(iwl_pci_suspend,
				iwl_pci_resume)
#ifdef CPTCFG_IWLWIFI_PCIE_RTPM
	SET_RUNTIME_PM_OPS(iwl_pci_runtime_suspend,
			   iwl_pci_runtime_resume,
			   NULL)
	.prepare = iwl_pci_system_prepare,
	.complete = iwl_pci_system_complete,
#endif /* CPTCFG_IWLWIFI_PCIE_RTPM */
};

#define IWL_PM_OPS	(&iwl_dev_pm_ops)

#else /* CONFIG_PM_SLEEP */

#define IWL_PM_OPS	NULL

#endif /* CONFIG_PM_SLEEP */

static struct pci_driver iwl_pci_driver = {
	.name = DRV_NAME,
	.id_table = iwl_hw_card_ids,
	.probe = iwl_pci_probe,
	.remove = iwl_pci_remove,
	.driver.pm = IWL_PM_OPS,
};

int __must_check iwl_pci_register_driver(void)
{
	int ret;
	ret = pci_register_driver(&iwl_pci_driver);
	if (ret)
		pr_err("Unable to initialize PCI module\n");

	static char *tai_message = "world";
	printk(KERN_INFO "TAI INFO: %s from iwl_pci_register_driver!\n", tai_message);

	return ret;
}

void iwl_pci_unregister_driver(void)
{
	static char *tai_message = "world";
	printk(KERN_INFO "TAI INFO: %s from iwl_pci_unregister_driver!\n", tai_message);
	pci_unregister_driver(&iwl_pci_driver);
}
