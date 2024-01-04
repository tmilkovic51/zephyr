/*
 * Copyright (c) 2023 Byte Lab Grupa d.o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <soc.h>

#ifndef _ASMLANGUAGE
#include <stm32h7xx.h>
#endif /* !_ASMLANGUAGE */

#include <clock_control/clock_stm32_ll_common.h>
#include <stm32h7xx_ll_bus.h>
#include <stm32h7xx_ll_cortex.h>
#include <stm32h7xx_ll_pwr.h>
#include <stm32h7xx_ll_system.h>
#include <stm32h7xx_ll_utils.h>
#include <stm32h7xx_ll_rcc.h>

LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

/* Invoke Low Power/System Off specific Tasks */
__weak void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	if (state != PM_STATE_SUSPEND_TO_IDLE) {
		LOG_ERR("Unsupported power state %u", state);
		return;
	}

	switch (substate_id) {
		case 1: /* this corresponds to the CDSTOP mode: */
			/* You can comment some of these out for lower exit latency, but higher power consumption: */
			LL_PWR_EnableFlashPowerDown();
			LL_PWR_SetStopModeRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SVOS_SCALE5);
			LL_PWR_SetRegulModeDS(LL_PWR_REGU_DSMODE_LOW_POWER);
			LL_PWR_CPU_DisableSRDRunInLowPowerMode();
			LL_PWR_CPU_SetCDPowerMode(LL_PWR_CPU_MODE_CDSTOP);
			LL_PWR_CPU_SetSRDPowerMode(LL_PWR_CPU_MODE_SRDSTOP);
			LL_LPM_DisableSleepOnExit();
			LL_LPM_EnableDeepSleep();

			/* enter low power mode: execute WFE or WFI instruction */
			k_cpu_idle();
			break;
		default:
			LOG_ERR("Unsupported power state %u substate %u", state, substate_id);
			break;
	}
}

/* Handle SOC specific activity after Low Power Mode Exit */
__weak void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	if (state != PM_STATE_SUSPEND_TO_IDLE) {
		LOG_ERR("Unsupported power state %u", state);
	} else {
		switch (substate_id) {
		case 1: /* CDSTOP */
			LL_PWR_ClearFlag_CPU();
			LL_LPM_DisableSleepOnExit();
			LL_LPM_EnableSleep();
			break;
		default:
			LOG_ERR("Unsupported power state %u substate %u", state, substate_id);
			break;
		}

		/* Need to restore the clock */
		stm32_clock_control_init(NULL);

		/* Set lowest voltage scale bacause MCU is working at 64 MHz */
		LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE3);
		while (LL_PWR_IsActiveFlag_VOS() == 0) {
		}
	}

	/*
	 * System is now in active mode.
	 * Reenable interrupts which were disabled
	 * when OS started idling code.
	 */
	irq_unlock(0);
}

/* Initialize STM32 Power */
static int stm32_power_init(void)
{
	/* Keep in mind that debugging draws a lot of power */
#ifdef CONFIG_DEBUG
	LL_DBGMCU_EnableD1DebugInSleepMode();
	LL_DBGMCU_EnableD1DebugInStopMode();
	LL_DBGMCU_EnableD1DebugInStandbyMode();

	LL_DBGMCU_EnableD3DebugInStopMode();
	LL_DBGMCU_EnableD3DebugInStandbyMode();
#else
	LL_DBGMCU_DisableD1DebugInSleepMode();
	LL_DBGMCU_DisableD1DebugInStopMode();
	LL_DBGMCU_DisableD1DebugInStandbyMode();

	LL_DBGMCU_DisableD3DebugInStopMode();
	LL_DBGMCU_DisableD3DebugInStandbyMode();
#endif

	return 0;
}

SYS_INIT(stm32_power_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
