#ifndef _LINUX_MFD_LIBS_PWR_H_
#define _LINUX_MFD_LIBS_PWR_H_

int libs_bat_get_voltage_now(void);
int libs_bat_get_capacity(void);
int libs_bat_poll_charge_source(void);

#endif

