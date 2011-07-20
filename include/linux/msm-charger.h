/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, and the entire permission notice in its entirety,
 *    including the disclaimer of warranties.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * ALTERNATIVELY, this product may be distributed under the terms of
 * the GNU General Public License, version 2, in which case the provisions
 * of the GPL version 2 are required INSTEAD OF the BSD license.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ALL OF
 * WHICH ARE HEREBY DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF NOT ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */
#ifndef __MSM_CHARGER_H__
#define __MSM_CHARGER_H__

#include <linux/power_supply.h>

enum {
	CHG_TYPE_USB,
	CHG_TYPE_AC
};

enum msm_hardware_charger_event {
	CHG_INSERTED_EVENT,
	CHG_ENUMERATED_EVENT,
	CHG_REMOVED_EVENT,
	CHG_DONE_EVENT,
	CHG_BATT_BEGIN_FAST_CHARGING,
	CHG_BATT_CHG_RESUME,
	CHG_BATT_TEMP_OUTOFRANGE,
	CHG_BATT_TEMP_INRANGE,
	CHG_BATT_INSERTED,
	CHG_BATT_REMOVED,
	CHG_BATT_STATUS_CHANGE,
	CHG_BATT_NEEDS_RECHARGING,
};

/**
 * enum hardware_charger_state
 * @CHG_ABSENT_STATE: charger cable is unplugged
 * @CHG_PRESENT_STATE: charger cable is plugged but charge current isnt drawn
 * @CHG_READY_STATE: charger cable is plugged and kernel knows how much current
 *			it can draw
 * @CHG_CHARGING_STATE: charger cable is plugged and current is drawn for
 *			charging
 */
enum msm_hardware_charger_state {
	CHG_ABSENT_STATE,
	CHG_PRESENT_STATE,
	CHG_READY_STATE,
	CHG_CHARGING_STATE,
};

struct msm_hardware_charger {
	int type;
	int rating;
	const char *name;
	int (*start_charging) (struct msm_hardware_charger *hw_chg,
			       int chg_voltage, int chg_current);
	int (*stop_charging) (struct msm_hardware_charger *hw_chg);
	int (*charging_switched) (struct msm_hardware_charger *hw_chg);
	void (*start_system_current) (struct msm_hardware_charger *hw_chg,
							int chg_current);
	void (*stop_system_current) (struct msm_hardware_charger *hw_chg);

	void *charger_private;	/* used by the msm_charger.c */
};

struct msm_battery_gauge {
	int (*get_battery_mvolts) (void);
	int (*get_battery_temperature) (void);
	int (*is_battery_present) (void);
	int (*is_battery_temp_within_range) (void);
	int (*is_battery_id_valid) (void);
	int (*get_battery_status)(void);
	int (*get_batt_remaining_capacity) (void);
	int (*monitor_for_recharging) (void);
};
/**
 * struct msm_charger_platform_data
 * @safety_time: max charging time in minutes
 * @update_time: how often the userland be updated of the charging progress
 * @max_voltage: the max voltage the battery should be charged upto
 * @min_voltage: the voltage where charging method switches from trickle to fast
 * @get_batt_capacity_percent: a board specific function to return battery
 *			capacity. Can be null - a default one will be used
 */
struct msm_charger_platform_data {
	unsigned int safety_time;
	unsigned int update_time;
	unsigned int max_voltage;
	unsigned int min_voltage;
	unsigned int (*get_batt_capacity_percent) (void);
};

typedef void (*notify_vbus_state) (int);
#if defined(CONFIG_BATTERY_MSM8X60) || defined(CONFIG_BATTERY_MSM8X60_MODULE)
void msm_battery_gauge_register(struct msm_battery_gauge *batt_gauge);
void msm_battery_gauge_unregister(struct msm_battery_gauge *batt_gauge);
int msm_charger_register(struct msm_hardware_charger *hw_chg);
int msm_charger_unregister(struct msm_hardware_charger *hw_chg);
int msm_charger_notify_event(struct msm_hardware_charger *hw_chg,
			     enum msm_hardware_charger_event event);
void msm_charger_vbus_draw(unsigned int mA);

int msm_charger_register_vbus_sn(void (*callback)(int));
void msm_charger_unregister_vbus_sn(void (*callback)(int));
#else
static inline void msm_battery_gauge_register(struct msm_battery_gauge *gauge)
{
}
static inline void msm_battery_gauge_unregister(struct msm_battery_gauge *gauge)
{
}
static inline int msm_charger_register(struct msm_hardware_charger *hw_chg)
{
	return -ENXIO;
}
static inline int msm_charger_unregister(struct msm_hardware_charger *hw_chg)
{
	return -ENXIO;
}
static inline int msm_charger_notify_event(struct msm_hardware_charger *hw_chg,
			     enum msm_hardware_charger_event event)
{
	return -ENXIO;
}
static inline void msm_charger_vbus_draw(unsigned int mA)
{
}
static inline int msm_charger_register_vbus_sn(void (*callback)(int))
{
	return -ENXIO;
}
static inline void msm_charger_unregister_vbus_sn(void (*callback)(int))
{
}
#endif
#endif /* __MSM_CHARGER_H__ */
