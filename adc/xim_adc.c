
#define _FILE_TAG_  "adc"
#include "debug/xim_log.h"


#include "adc/xim_adc.h"
#include "info/xim_info.h"

#include "nrf_delay.h"
#include "app_scheduler.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "boards.h"
#include "nrf_drv_saadc.h"



#define XIM_ADC_CH_BATTERY      1


#define XIM_ADC_PIN_BATTERY             COBRA_ADC_BATT

static uint32_t odm_version_cnt = 0;

static uint8_t odm_version_str[64];
static uint32_t odm_version_str_len = 0;

static bool isReady = false;

#define SAADC_OVER_SAMPLING             32
#define TRIGGER_ADC_INTERVAL            16              //
#define SAMPLES_IN_BUFFER               1
volatile static uint32_t last_batt_adc_volt = 0;
volatile static uint32_t last_batt_ntc_volt = 0;

static pfAdcValueCallback m_callback_batt_update = NULL;
//static pfAdcValueCallback m_callback_batt_ntc_update = NULL;

APP_TIMER_DEF(m_batt_update_timer);
#define TIMER_INTERVAL_BATT_UPDATE      APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) //  ms intervals


static inline uint32_t convert_from_adc_14bit_to_volt_mv(int32_t in)
{
    const uint32_t max_input = (1 << 14);
    const uint32_t max_output = 3600;
    in = in > 0 ? in : 0;
    in = in < max_input ? in : max_input;
    return in * max_output / max_input;
}

static uint32_t convert_from_pin_to_batt(uint32_t mv)
{
    return mv;//*(715+680)/715;
}


#if 1   //region saadc

static void saadc_event_handler(nrf_drv_saadc_evt_t const* p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {

    }

}
static void private_xim_saadc_block_init(void)
{
    ret_code_t err_code;

    static const nrf_drv_saadc_config_t saadc_config =
    {
        .resolution         = NRF_SAADC_RESOLUTION_14BIT,
        .oversample         = NRF_SAADC_OVERSAMPLE_DISABLED,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW,
    };

    err_code = nrf_drv_saadc_init(&saadc_config, saadc_event_handler);
    APP_ERROR_CHECK(err_code);
}


static void private_xim_saadc_uninit(void)
{
    nrf_drv_saadc_uninit();
}

#endif


#if 1   //region adc channel config and sample

static void battery_adc_init(void)
{
    ret_code_t err_code;

    nrf_saadc_channel_config_t channel_config =
    {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain       = NRF_SAADC_GAIN1_6,
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,
        .acq_time   = NRF_SAADC_ACQTIME_40US,
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .pin_p      = XIM_ADC_PIN_BATTERY,
        .pin_n      = NRF_SAADC_INPUT_DISABLED,
    };

    err_code = nrf_drv_saadc_channel_init(XIM_ADC_CH_BATTERY, &channel_config);
    APP_ERROR_CHECK(err_code);
}

//static void battery_ntc_adc_init(void)
//{
//    ret_code_t err_code;

//    nrf_saadc_channel_config_t channel_config =
//    {
//        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
//        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
//        .gain       = NRF_SAADC_GAIN1_6,
//        .reference  = NRF_SAADC_REFERENCE_INTERNAL,
//        .acq_time   = NRF_SAADC_ACQTIME_40US,
//        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
//        .pin_p      = XIM_ADC_PIN_BAT_TEMPERATURE,
//        .pin_n      = NRF_SAADC_INPUT_DISABLED,
//    };

//    err_code = nrf_drv_saadc_channel_init(XIM_ADC_CH_BAT_NTC, &channel_config);
//    APP_ERROR_CHECK(err_code);
//}

//static void odm_adc_init(void)
//{
//    ret_code_t err_code;

//    nrf_saadc_channel_config_t channel_config =
//    {
//        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
//        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
//        .gain       = NRF_SAADC_GAIN1_6,
//        .reference  = NRF_SAADC_REFERENCE_INTERNAL,
//        .acq_time   = NRF_SAADC_ACQTIME_40US,
//        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
//        .pin_p      = XIM_ADC_PIN_ODM_1,
//        .pin_n      = NRF_SAADC_INPUT_DISABLED,
//    };

//    err_code = nrf_drv_saadc_channel_init(XIM_ADC_CH_ODM_1, &channel_config);
//    APP_ERROR_CHECK(err_code);

//    channel_config.pin_p = XIM_ADC_PIN_ODM_2;
//    err_code = nrf_drv_saadc_channel_init(XIM_ADC_CH_ODM_2, &channel_config);
//    APP_ERROR_CHECK(err_code);
//    
//    channel_config.pin_p = XIM_ADC_PIN_BAT_ID;
//    err_code = nrf_drv_saadc_channel_init(XIM_ADC_CH_BAT_ID, &channel_config);
//    APP_ERROR_CHECK(err_code);
//}

static uint32_t adc_convert_by_channel(uint8_t channel, int16_t* p_value, int times)
{
    uint32_t err_code = 0;
    int16_t min = INT16_MAX;
    int16_t max = INT16_MIN;
    int16_t current = 0;
    int32_t sum = 0;
    
    if(times < 3)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    for(int i = 0; i < times; i++)
    {
        err_code |= nrf_drv_saadc_sample_convert(channel, &current);
        min = min < current ? min : current;
        max = max > current ? max : current;
        sum += current;
    }
    sum -= min;
    sum -= max;
    sum = sum>0?sum:0;
    sum /= (times - 2);
    
    *p_value = sum;
    return err_code;
}

static void battery_adc_sample(void)
{
    uint32_t err_code = 0;
    int16_t value;


    private_xim_saadc_block_init();
    
    battery_adc_init();
    err_code |= adc_convert_by_channel(XIM_ADC_CH_BATTERY, &value, 6);
    last_batt_adc_volt = convert_from_pin_to_batt(convert_from_adc_14bit_to_volt_mv(value));

//    battery_ntc_adc_init();
//    err_code |= adc_convert_by_channel(XIM_ADC_CH_BAT_NTC, &value, 3);
//    last_batt_ntc_volt = convert_from_adc_14bit_to_volt_mv(value);
//    
    APP_ERROR_CHECK(err_code);

    private_xim_saadc_uninit();

    if(m_callback_batt_update != NULL)
    {
        m_callback_batt_update(last_batt_adc_volt);
    }
//    if(m_callback_batt_ntc_update != NULL)
//    {
//        m_callback_batt_ntc_update(last_batt_ntc_volt);
//    }
}

static void batt_update_timeout_handler(void* p_context)
{
    uint32_t err_code;
    err_code = app_sched_event_put(0, 0, (app_sched_event_handler_t)battery_adc_sample);
    APP_ERROR_CHECK(err_code);
}

static void batt_update_timer_init(void)
{
    uint32_t err_code;
    err_code = app_timer_create(&m_batt_update_timer, APP_TIMER_MODE_REPEATED,
                                batt_update_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_batt_update_timer, TIMER_INTERVAL_BATT_UPDATE, 0);
    APP_ERROR_CHECK(err_code);
}

static void odm_adc_value_init(void)
{
//    uint32_t err_code = 0;
//    int16_t sum_odm1 = 0;
//    int16_t sum_odm2 = 0;
//    int16_t sum_bat_id = 0;
//    const uint32_t cnt_odm_cfg = 10;

//    nrf_delay_ms(1);

//    err_code |= adc_convert_by_channel(XIM_ADC_CH_ODM_1, &sum_odm1, cnt_odm_cfg);
//    err_code |= adc_convert_by_channel(XIM_ADC_CH_ODM_2, &sum_odm2, cnt_odm_cfg);
//    err_code |= adc_convert_by_channel(XIM_ADC_CH_BAT_ID, &sum_bat_id, cnt_odm_cfg);
//    APP_ERROR_CHECK(err_code);

//    uint32_t odm1_volt = convert_from_adc_14bit_to_volt_mv(sum_odm1);
//    uint32_t odm2_volt = convert_from_adc_14bit_to_volt_mv(sum_odm2);
//    uint32_t bat_id_volt = convert_from_adc_14bit_to_volt_mv(sum_bat_id);
//    //根据实际测试sample batt_id 安装电池2940mV左右 未安装电池 3.3V
//    LOGPF("ODM1:%5d mv,  ODM2:%5d mv,    Batid:%5d mv", odm1_volt, odm2_volt, bat_id_volt);
//    

//    if((p_usr_info->hw_version.times > 0)
//        &&(p_usr_info->hw_version.value[0] != 0x0)
//        &&(p_usr_info->hw_version.value[0] != 0xFF))
//    {
//        odm_version_str_len = sprintf((char*)odm_version_str, "Ver%s", p_usr_info->hw_version.value); // 1.65V 1.65V
//    }
//    else
//    {
//        odm_version_str_len = sprintf((char*)odm_version_str, "%s", "VerDVT2");
//    }
//    
    odm_version_str_len = sprintf((char*)odm_version_str, "%s", "VerB");
}
#endif

//region sample_event_generate
#if 0


static void trigger_adc_sample(void);

static void saadc_trigger_timer_handler(void* p_context)
{
    uint32_t err_code;
    err_code = app_sched_event_put(0, 0, (app_sched_event_handler_t)trigger_adc_sample);
    APP_ERROR_CHECK(err_code);
}

static void saadc_sampling_generate_init(void)
{
    uint32_t err_code;
    err_code = app_timer_create(&m_trigger_update_timer, APP_TIMER_MODE_SINGLE_SHOT,
                                saadc_trigger_timer_handler);
    APP_ERROR_CHECK(err_code);
}

static void saadc_trigger_sampling_event_enable(void)
{
    uint32_t err_code;
    err_code = app_timer_start(m_trigger_update_timer, trigger_time_interval_busy, 0);
    APP_ERROR_CHECK(err_code);
}

static void saadc_trigger_sampling_event_disable(void)
{
    uint32_t err_code;
    err_code = app_timer_stop(m_trigger_update_timer);
    APP_ERROR_CHECK(err_code);
}

static uint32_t trigger_volt_filter_rc(uint32_t volt_in)
{
    static float f_recorder = 0.0f;
    static uint32_t volt_gate_min = 10;
    static uint32_t volt_gate_max = 1500;    
    static const float new_value_weight_low = 1.0;
    static const float new_value_weight_high = 0.8;
    
    if(volt_in <= volt_gate_min)
    {
        f_recorder = 0;
    }
    else if(volt_in >= volt_gate_max)
    {
        f_recorder = f_recorder*(1-new_value_weight_high) + ((float)volt_in)*new_value_weight_high;
    }
    else
    {
        f_recorder = f_recorder*(1-new_value_weight_low) + ((float)volt_in)*new_value_weight_low;
    }
    
    return f_recorder;
}

#endif

void xim_adc_init(void)
{
    if(isReady)
    {
        return;
    }

    private_xim_saadc_block_init();
//    odm_adc_init();
    odm_adc_value_init();

    private_xim_saadc_uninit();
    
    battery_adc_sample();
    isReady = true;
}

void xim_adc_start(void)
{
    batt_update_timer_init();
}

void xim_adc_uninit(void)
{
    private_xim_saadc_uninit();
}

uint32_t get_trigger_percent(void)
{
    if(nrf_gpio_pin_read(BSP_PIN_BUTTON_TRIG) == 0)
    {
        return 0x0;
    }
    else
    {
        return 0xff;
    }
}

uint32_t get_batt_value(void)
{
    return last_batt_adc_volt;
}
void xim_batt_volt_update_registe(pfAdcValueCallback callback)
{
    m_callback_batt_update = callback;

    if(m_callback_batt_update != NULL)
    {
        m_callback_batt_update(last_batt_adc_volt);
    }
}

void xim_batt_ntc_volt_update_registe(pfAdcValueCallback callback)
{
//    m_callback_batt_ntc_update = callback;
}

uint32_t get_odm_revision(uint8_t** p_str, uint32_t* p_length)
{
    *p_str = odm_version_str;
    *p_length = odm_version_str_len;
    return odm_version_cnt;
}
