
#ifndef _BATT_INTERFACE_H__
#define _BATT_INTERFACE_H__

#include <stdint.h>
#include <stdbool.h>

//电池电压配置
//#define BATT_NO_BOOT_VOLT           1850

#define BATT_LOW_WARNNING_PERCENT           10
#define BATT_CRITICAL_PERCENT               1

typedef enum
{
    batt_work_no_charging = 0,
    batt_charge_proessing,
    batt_charge_full,
    batt_charge_overheat,
}batt_status;
    
typedef enum
{
    batt_interface_evt_none = 0x00,
    batt_interface_evt_volt_update,
    batt_interface_evt_percent_update,
    batt_interface_evt_temperature_update,
    batt_interface_evt_charge_start,
    batt_interface_evt_charge_stop,
    batt_interface_evt_charge_full,
    batt_interface_evt_batt_volt_low,
    batt_interface_evt_batt_volt_critical,  //have to shutdown
}batt_intereface_evt_type_t;

typedef struct
{
    batt_intereface_evt_type_t type;
    union
    {
        uint32_t percent;
        uint32_t volt_mv;
        float temperature_degree;
    }data;
}batt_intereface_evt_t;

typedef void (*pfbatt_callback_t)(batt_intereface_evt_t event);

void batt_interface_init(void);
void batt_interface_event_registe(pfbatt_callback_t callback);
batt_status batt_interface_get_charge_status_now(void);
uint8_t batt_interface_get_batt_percent_last(void);
uint32_t batt_interface_set_batt_percent_last(uint8_t percent);
uint32_t batt_interface_get_batt_volt_last(void);

//force stop charging
void batt_interface_force_stop_charge(bool isStopNow);

//charge mainloop
void boot_just_charge_main_loop(void);

#endif
