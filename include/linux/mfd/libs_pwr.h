#ifndef _LINUX_MFD_LIBS_DRIVER_H_
#define _LINUX_MFD_LIBS_DRIVER_H_

int libs_batt_get_voltage_now(void);
int libs_batt_get_capacity(void);
int libs_batt_poll_charge_source(void);

#endif

