#ifndef _LINUX_POWER_LIBS_BATTERY_H_
#define _LINUX_POWER_LIBS_BATTERY_H_

int libs_bat_get_voltage_now(void);
int libs_bat_get_capacity(void);
int libs_bat_poll_charge_source(void);

struct libs_bat_platform_data {
	unsigned int	voltage;
	unsigned int	capacity;
};

#endif

