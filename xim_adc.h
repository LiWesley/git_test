
#ifndef __XIM_ADC_H__
#define __XIM_ADC_H__

    #include <stdint.h>
    
    typedef void (*pfAdcValueCallback)(uint32_t volt_mv);

    void xim_batt_volt_update_registe(pfAdcValueCallback callback);
    void xim_batt_ntc_volt_update_registe(pfAdcValueCallback callback);

    void xim_adc_init(void);
    void xim_adc_start(void);
    void xim_adc_uninit(void);

    uint32_t get_trigger_percent(void);
    uint32_t get_batt_value(void);
    uint32_t get_odm_revision(uint8_t** p_str, uint32_t* p_length);
#endif
