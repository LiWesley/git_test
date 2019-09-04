
#define _FILE_TAG_  "batt_if"
#include "debug/xim_log.h"

#include "adc/battery_interface.h"

#include "app_scheduler.h"

#include "adc/xim_adc.h"
#include "pwr/xim_boot_manager.h"
#include "pwm/led_interface.h"
#include "bsp/xim_bsp.h"
#include "pwr/m_pwr_mgmt.h"
#include "wdt/xim_wdt.h"

#include "adc/xim_ntc_table.h"

#include "nrf_delay.h"

#define BAT_INTERFACE_CTRL_PIN  BSP_PIN_CHARGE_EN

#define BATT_PERCENT_0_VOLT     1800
#define BATT_PERCENT_100_VOLT   3006


#if DEBUG_EN
#define BATT_CHARGE_TEMP_OVERHEAT   39.0f
#define BATT_CHARGE_TEMP_NORMAL     35.0f
#else
#define BATT_CHARGE_TEMP_OVERHEAT   50.0f
#define BATT_CHARGE_TEMP_NORMAL     45.0f
#endif

#define BATT_CONVERTER_COUNTS       1000

#define BATT_USE_3DOF_CFG           0
#define BATT_USE_0W3_CFG            1

#if BATT_USE_0W3_CFG
static const uint8_t batt_converter[BATT_CONVERTER_COUNTS] = {
0,0,0,0,0,0,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,2,2,
2,2,2,2,2,2,2,2,2,2,
2,2,2,2,2,2,2,2,2,2,
2,2,2,2,2,2,2,2,2,2,
2,2,2,2,3,3,3,3,3,3,
3,3,3,3,3,3,3,3,3,3,
3,3,3,3,3,3,3,3,3,4,
4,4,4,4,4,4,4,4,4,4,
4,4,4,4,4,4,4,4,5,5,
5,5,5,5,5,5,5,5,5,5,
5,5,5,5,5,5,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
7,7,7,7,7,7,7,7,7,7,
7,7,7,8,8,8,8,8,8,8,
8,8,8,8,9,9,9,9,9,9,
9,9,9,9,9,9,10,10,10,10,
10,10,10,10,10,10,11,11,11,11,
11,11,11,11,11,12,12,12,12,12,
12,12,12,12,12,12,13,13,13,13,
13,13,13,13,14,14,14,14,14,14,
14,14,14,14,15,15,15,15,15,15,
15,15,16,16,16,16,16,16,16,16,
17,17,17,17,17,17,18,18,18,18,
18,19,19,19,19,19,19,19,19,20,
20,20,20,20,20,20,21,21,21,21,
21,21,21,22,22,22,22,22,22,23,
23,23,23,23,23,24,24,24,24,24,
24,25,25,25,25,25,25,25,26,26,
26,26,26,27,27,27,27,27,27,28,
28,28,28,28,28,29,29,29,29,29,
30,30,30,30,30,31,31,31,31,31,
32,32,32,32,32,33,33,33,33,34,
34,34,34,34,35,35,35,35,35,36,
36,36,36,37,37,37,37,37,38,38,
38,38,39,39,39,40,40,40,40,40,
41,41,41,41,42,42,42,42,43,43,
43,43,44,44,44,44,44,45,45,45,
45,46,46,46,46,46,47,47,47,47,
48,48,48,48,49,49,49,49,50,50,
50,50,50,51,51,51,51,52,52,52,
52,52,53,53,53,53,53,54,54,54,
54,54,54,55,55,55,55,55,55,56,
56,56,56,56,57,57,57,57,57,57,
58,58,58,58,58,58,59,59,59,59,
59,59,60,60,60,60,60,60,60,61,
61,61,61,61,61,62,62,62,62,62,
62,63,63,63,63,63,63,63,64,64,
64,64,64,64,64,65,65,65,65,65,
65,66,66,66,66,66,66,66,67,67,
67,67,67,67,67,67,68,68,68,68,
68,68,68,68,69,69,69,69,69,69,
69,69,70,70,70,70,70,70,70,70,
71,71,71,71,71,71,71,72,72,72,
72,72,72,72,72,73,73,73,73,73,
73,73,73,74,74,74,74,74,74,74,
74,75,75,75,75,75,75,75,75,76,
76,76,76,76,76,76,76,77,77,77,
77,77,77,77,77,77,78,78,78,78,
78,78,78,78,79,79,79,79,79,79,
79,79,80,80,80,80,80,80,80,80,
80,81,81,81,81,81,81,81,81,81,
82,82,82,82,82,82,82,82,83,83,
83,83,83,83,83,83,83,83,84,84,
84,84,84,84,84,84,84,85,85,85,
85,85,85,85,85,85,85,86,86,86,
86,86,86,86,86,86,86,87,87,87,
87,87,87,87,87,87,87,87,88,88,
88,88,88,88,88,88,88,88,88,88,
88,89,89,89,89,89,89,89,89,89,
89,89,89,89,89,89,90,90,90,90,
90,90,90,90,90,90,90,90,90,90,
90,90,91,91,91,91,91,91,91,91,
91,91,91,91,91,91,91,91,91,92,
92,92,92,92,92,92,92,92,92,92,
92,92,92,92,92,92,92,93,93,93,
93,93,93,93,93,93,93,93,93,93,
93,93,93,93,93,93,94,94,94,94,
94,94,94,94,94,94,94,94,94,94,
94,94,94,94,94,94,94,95,95,95,
95,95,95,95,95,95,95,95,95,95,
95,95,95,95,95,95,95,95,95,95,
96,96,96,96,96,96,96,96,96,96,
96,96,96,96,96,96,96,96,96,96,
96,96,96,96,96,97,97,97,97,97,
97,97,97,97,97,97,97,97,97,97,
97,97,97,97,97,97,97,97,97,97,
97,97,97,97,97,98,98,98,98,98,
98,98,98,98,98,98,98,98,98,98,
98,98,98,98,98,98,98,98,98,98,
98,98,98,98,98,98,98,98,98,98,
99,99,99,99,99,99,99,99,99,99,
99,99,99,100,100,100,100,100,100,100,
};
#elif BATT_USE_3DOF_CFG
static const uint8_t batt_converter[BATT_CONVERTER_COUNTS] = {
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1, 
  1,   2,   2,   2,   2,   2,   2,   2,   2,   2, 
  2,   2,   2,   2,   2,   2,   2,   2,   2,   2, 
  2,   2,   2,   2,   2,   2,   2,   2,   2,   2, 
  2,   2,   2,   2,   2,   2,   2,   2,   2,   2, 
  2,   2,   2,   2,   2,   2,   2,   2,   2,   2, 
  2,   2,   2,   2,   2,   2,   2,   2,   2,   2, 
  2,   3,   3,   3,   3,   3,   3,   3,   3,   3, 
  3,   3,   3,   3,   3,   3,   3,   3,   3,   3, 
  3,   3,   3,   3,   3,   3,   3,   3,   3,   3, 
  3,   3,   3,   3,   3,   3,   3,   3,   3,   3, 
  3,   4,   4,   4,   4,   4,   4,   4,   4,   4, 
  4,   4,   4,   4,   4,   4,   4,   4,   4,   4, 
  4,   4,   4,   4,   4,   4,   4,   4,   4,   4, 
  4,   5,   5,   5,   5,   5,   5,   5,   5,   5, 
  5,   5,   5,   5,   5,   5,   5,   5,   5,   5, 
  5,   5,   5,   5,   5,   5,   5,   5,   5,   5, 
  5,   6,   6,   6,   6,   6,   6,   6,   6,   6, 
  6,   6,   6,   6,   6,   6,   6,   6,   6,   6, 
  6,   7,   7,   7,   7,   7,   7,   7,   7,   7, 
  7,   7,   7,   7,   7,   7,   7,   7,   7,   7, 
  7,   7,   7,   7,   7,   7,   7,   7,   7,   7, 
  7,   8,   8,   8,   8,   8,   8,   8,   8,   8, 
  8,   8,   8,   8,   8,   8,   8,   8,   8,   8, 
  8,   9,   9,   9,   9,   9,   9,   9,   9,   9, 
  9,   9,   9,   9,   9,   9,   9,   9,   9,   9, 
  9,  10,  10,  10,  10,  10,  10,  10,  10,  10, 
 10,  10,  10,  10,  10,  10,  10,  10,  10,  10, 
 10,  11,  11,  11,  11,  11,  11,  11,  11,  11, 
 11,  12,  12,  12,  12,  12,  12,  12,  12,  12, 
 12,  12,  12,  12,  12,  12,  12,  12,  12,  12, 
 12,  13,  13,  13,  13,  13,  13,  13,  13,  13, 
 13,  13,  13,  13,  13,  13,  13,  13,  13,  13, 
 13,  14,  14,  14,  14,  14,  14,  14,  14,  14, 
 14,  14,  14,  14,  14,  14,  14,  14,  14,  14, 
 14,  15,  15,  15,  15,  15,  15,  15,  15,  15, 
 15,  16,  16,  16,  16,  16,  16,  16,  16,  16, 
 16,  17,  17,  17,  17,  17,  17,  17,  17,  17, 
 17,  18,  18,  18,  18,  18,  18,  18,  18,  18, 
 18,  18,  18,  18,  18,  18,  18,  18,  18,  18, 
 18,  19,  19,  19,  19,  19,  19,  19,  19,  19, 
 19,  20,  20,  20,  20,  20,  20,  20,  20,  20, 
 20,  21,  21,  21,  21,  21,  21,  21,  21,  21, 
 21,  22,  22,  22,  22,  22,  22,  22,  22,  22, 
 22,  22,  22,  22,  22,  22,  22,  22,  22,  22, 
 22,  23,  23,  23,  23,  23,  23,  23,  23,  23, 
 23,  24,  24,  24,  24,  24,  24,  24,  24,  24, 
 24,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  26,  26,  26,  26,  26,  27,  27,  27,  27, 
 27,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  29,  29,  29,  29,  29,  30,  30,  30,  30, 
 30,  31,  31,  31,  31,  31,  31,  31,  31,  31, 
 31,  32,  32,  32,  32,  32,  33,  33,  33,  33, 
 33,  34,  34,  34,  34,  34,  34,  34,  34,  34, 
 34,  35,  35,  35,  35,  35,  36,  36,  36,  36, 
 36,  37,  37,  37,  37,  37,  38,  38,  38,  38, 
 38,  39,  39,  39,  40,  40,  40,  40,  40,  41, 
 41,  42,  42,  42,  42,  42,  42,  42,  42,  42, 
 42,  43,  43,  43,  44,  44,  44,  45,  45,  45, 
 46,  46,  47,  47,  47,  47,  47,  48,  48,  48, 
 49,  50,  50,  51,  51,  51,  51,  52,  52,  53, 
 53,  54,  54,  55,  55,  56,  56,  56,  56,  57, 
 57,  58,  58,  58,  59,  59,  59,  59,  59,  60, 
 60,  61,  61,  61,  62,  62,  62,  62,  62,  63, 
 63,  64,  64,  64,  64,  64,  65,  65,  65,  65, 
 65,  66,  66,  66,  66,  66,  67,  67,  67,  67, 
 67,  68,  68,  68,  68,  68,  69,  69,  69,  69, 
 69,  70,  70,  70,  71,  71,  71,  71,  71,  72, 
 72,  73,  73,  73,  74,  74,  74,  74,  74,  75, 
 75,  76,  76,  76,  77,  77,  77,  77,  77,  78, 
 78,  79,  79,  79,  80,  80,  80,  80,  80,  81, 
 81,  82,  82,  82,  82,  82,  83,  83,  83,  83, 
 83,  84,  84,  84,  84,  84,  84,  84,  84,  84, 
 84,  85,  85,  85,  85,  85,  86,  86,  86,  86, 
 86,  87,  87,  87,  87,  87,  88,  88,  88,  88, 
 88,  89,  89,  89,  89,  89,  90,  90,  90,  90, 
 90,  91,  91,  91,  91,  91,  92,  92,  92,  92, 
 92,  93,  93,  93,  93,  93,  93,  93,  93,  94, 
 94,  95,  95,  95,  95,  95,  95,  95,  95,  96, 
 96,  97,  97,  97,  97,  97,  97,  97,  97,  97, 
 97,  98,  98,  98,  98,  98,  98,  98,  98,  99, 
 99,  99,  99,  99,  99,  99,  99,  99,  99,  99, 
 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 
100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 
100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 
};
#else
#error SHOULD NOT USE CV01 LINE
static const uint8_t batt_converter[BATT_CONVERTER_COUNTS] = {
};
#endif


static batt_status staus_now = batt_work_no_charging;

static float m_last_charge_temperature = 0.0f;

static pfbatt_callback_t m_batt_callback = NULL;
static uint32_t m_last_batt_volt = 0;
static uint32_t m_last_batt_percent = 100;
static const uint32_t batt_percent_charge_default = 100;
static const uint32_t batt_percent_charging = 101;
static const uint32_t batt_percent_charg_full = 102;


static void batt_event_exec(void* p_context, uint16_t size)
{
    if(m_batt_callback != NULL)
    {
        batt_intereface_evt_t event = *(batt_intereface_evt_t*)p_context;
        m_batt_callback(event);
    }
}

static void on_batt_event(batt_intereface_evt_type_t event_type)
{
    batt_intereface_evt_t event;
    event.type = event_type;
    
    switch(event_type)
    {
        case batt_interface_evt_volt_update:
            event.data.volt_mv = m_last_batt_volt;
            break;
        case batt_interface_evt_percent_update:
        {
            if(staus_now == batt_charge_proessing)
            {
                event.data.percent = batt_percent_charging;
                m_last_batt_percent = batt_percent_charge_default;
            }
            else if(staus_now == batt_charge_full)
            {
                event.data.percent = batt_percent_charg_full;
                m_last_batt_percent = batt_percent_charge_default;
            }
            else
            {
                event.data.percent = m_last_batt_percent;
            }
        }
            break;
        case batt_interface_evt_temperature_update:
            event.data.temperature_degree = m_last_charge_temperature;
        default:
            break;
    }
    
    if(m_batt_callback != NULL)
    {
        app_sched_event_put(&event, sizeof(batt_intereface_evt_t), batt_event_exec);
    }
    else
    {
        switch(event_type)
        {
            case batt_interface_evt_batt_volt_low:
                event.data.volt_mv = m_last_batt_volt;
                led_command(led_request_batt_low);
                break;
            case batt_interface_evt_batt_volt_critical:
                event.data.percent = m_last_batt_percent;
                LOGPF("low batt shutdown");
                xim_boot_action_force_shutdown();
                break;
            default:
                break;
        }
    }
}

static float batt_rc_filter(float new_weight, uint32_t data_in, float* p_record)
{
    if(*p_record == 0x00)
    {
        *p_record = data_in;
        return data_in;
    }
    
    float old = *p_record;
    *p_record = old*(1-new_weight) + data_in*new_weight;
    return *p_record;
}

static float batt_volt_recorder = 0.0f;
static float batt_volt_filter(uint32_t mv_in)
{
    static const float batt_volt_filter_new_value_weight = 0.01f;    //电池电压检测 新值权重    
    return batt_rc_filter(batt_volt_filter_new_value_weight, mv_in, &batt_volt_recorder);
}

//static float batt_ntc_volt_recorder = 0.0f;
//static float batt_ntc_volt_filter(uint32_t mv_in)
//{
//    static const float batt_volt_filter_new_value_weight = 0.2f;    //电池电压检测 新值权重    
//    return batt_rc_filter(batt_volt_filter_new_value_weight, mv_in, &batt_ntc_volt_recorder);
//}


static uint8_t convert_from_batt_volt_to_percent(uint32_t mv)
{
    const uint32_t min_batt_mv = BATT_PERCENT_0_VOLT;
    const uint32_t max_batt_mv = BATT_PERCENT_100_VOLT;
    
    uint32_t volt_mv = mv;

    volt_mv = volt_mv > min_batt_mv ? volt_mv : min_batt_mv;
    volt_mv = volt_mv < max_batt_mv ? volt_mv : max_batt_mv;

    m_last_batt_volt = volt_mv;
    

    uint32_t before_remap_percent = (volt_mv - min_batt_mv) * (BATT_CONVERTER_COUNTS-1) / (max_batt_mv - min_batt_mv);
    uint32_t percent = batt_converter[before_remap_percent];

//    #if !defined(DEBUG_EN)
    if(percent > m_last_batt_percent)
    {
        return m_last_batt_percent;
    }
//    #endif
    return percent;
}

#if DEBUG_EN
static void private_batt_volt_update_handler(uint32_t volt_mv)
{
    uint32_t origine_mv = volt_mv;
    volt_mv = batt_volt_filter(volt_mv);
    m_last_batt_volt = volt_mv;

    m_last_batt_percent = convert_from_batt_volt_to_percent(volt_mv);
    on_batt_event(batt_interface_evt_percent_update);

    LOGPF("volt: %d mv, %d mv, percent: %d%%", origine_mv, m_last_batt_volt, m_last_batt_percent);
    on_batt_event(batt_interface_evt_percent_update);
    
    if(m_last_batt_percent <= BATT_CRITICAL_PERCENT)
    {
        on_batt_event(batt_interface_evt_batt_volt_critical);
    }
    else if(m_last_batt_percent <= BATT_LOW_WARNNING_PERCENT)
    {
        //interface to led
        on_batt_event(batt_interface_evt_batt_volt_low);
    }
}
#else
static void private_batt_volt_update_handler(uint32_t volt_mv)
{
    volt_mv = batt_volt_filter(volt_mv);
    m_last_batt_volt = volt_mv;
    
    m_last_batt_percent = convert_from_batt_volt_to_percent(volt_mv);
    on_batt_event(batt_interface_evt_percent_update);

    if(m_last_batt_percent <= BATT_CRITICAL_PERCENT)
    {
        on_batt_event(batt_interface_evt_batt_volt_critical);
    }
    else if(m_last_batt_percent <= BATT_LOW_WARNNING_PERCENT)
    {
        //interface to led
        on_batt_event(batt_interface_evt_batt_volt_low);
    }
}
#endif

//static float convert_from_volt_mv_to_ntc_Ohm(float mv)
//{
//    const float target_vcc_volt_mv = 3270.0f;
//    const float target_resitor_10k = 10000.0f;
//    // mv = 3300*(Ohm/(Ohm+10_000))
//    // Ohm = (10_000/(1-mv/3300))-10_000
//    // Ohm = (3300*10_000/(3300-mv))-10_000
//    return (target_vcc_volt_mv * target_resitor_10k / (target_vcc_volt_mv-mv) ) - target_resitor_10k;
//}

static void charge_status_update(void)
{
    staus_now = batt_work_no_charging;
}

static void force_charge_stop(bool isStopNow)
{
////    //没有影响  不考虑该使用场景
////    nrf_gpio_pin_set(BAT_INTERFACE_CTRL_PIN);
////    nrf_gpio_cfg_output(BAT_INTERFACE_CTRL_PIN);
//    if(isStopNow)
//    {
//        // force stop
//        nrf_gpio_pin_clear(BAT_INTERFACE_CTRL_PIN);
//        nrf_gpio_cfg_output(BAT_INTERFACE_CTRL_PIN);
//    }
//    else
//    {
//        //
//        nrf_gpio_cfg_default(BAT_INTERFACE_CTRL_PIN);
//    }
}

static void private_batt_ntc_volt_update_handler(uint32_t volt_mv)
{
//    float volt_ntc = batt_ntc_volt_filter(volt_mv);
//    float ntc_Ohm = convert_from_volt_mv_to_ntc_Ohm(volt_ntc);
//    float ntc_degree_c = convert_from_resistorOhmToTemperature(ntc_Ohm);
////    LOGPF("ntc: %f C, %f Ohm, %d mv, %f mv", ntc_degree_c, ntc_Ohm, volt_mv, volt_ntc);

//    m_last_charge_temperature = ntc_degree_c;
////    on_charge_ctrl_update();
//    on_batt_event(batt_interface_evt_temperature_update);
}

static void modify_status_led(void)
{
//    if(m_batt_callback != NULL)
//    {
//        switch(staus_now)
//        {
//            case batt_work_no_charging:
//                on_batt_event(batt_interface_evt_charge_stop);
//                break;
//            case batt_charge_proessing:
//                on_batt_event(batt_interface_evt_charge_start);
//                break;
//            case batt_charge_full:
//                on_batt_event(batt_interface_evt_charge_full);
//                break;
//        }
//    }else{
//        switch(staus_now)
//        {
//            case batt_work_no_charging:
//                LOGPF("batt_work_no_charging");
//                led_command(led_request_red_close);
//                break;
//            case batt_charge_proessing:
//                LOGPF("batt_charge_processing");
//                led_command(led_request_red_close);
//                led_command(led_request_batt_charging);
//                break;
//            case batt_charge_full:
//                LOGPF("batt_charge_full");
//                led_command(led_request_red_close);
//                led_command(led_request_batt_charge_full);
//                break;
//        }
//    }
}

static void charge_status_change_handler(bsp_event_t evt)
{
//    app_sched_event_put(0, 0, (app_sched_event_handler_t)charge_status_update);
//    app_sched_event_put(0, 0, (app_sched_event_handler_t)modify_status_led);
}


void batt_interface_init(void)
{
    //registe adc and button
    xim_batt_volt_update_registe(private_batt_volt_update_handler);
    xim_batt_ntc_volt_update_registe(private_batt_ntc_volt_update_handler);
    xim_bsp_battery_event_registe(charge_status_change_handler);
    charge_status_update();
    modify_status_led();
}


void batt_interface_event_registe(pfbatt_callback_t callback)
{
    m_batt_callback = callback;
}

batt_status batt_interface_get_charge_status_now(void)
{
    charge_status_update();
    return staus_now;
}

uint8_t batt_interface_get_batt_percent_last(void)
{
    return m_last_batt_percent;
}

uint32_t batt_interface_set_batt_percent_last(uint8_t percent)
{
    uint32_t old_value = m_last_batt_percent;
    
    batt_volt_recorder = 0; //force reload batt filter recoder. when reload adc values
    m_last_batt_percent = percent;
    return old_value;
}


uint32_t batt_interface_get_batt_volt_last(void)
{
    return m_last_batt_volt;
}

//region mainloop charge

static void on_button_event_charge_mainloop(bsp_event_t evt)
{
}

static void on_batt_interface_handler(batt_intereface_evt_t event)
{
    switch(event.type)
    {
        default:
            break;
    }
}

void boot_just_charge_main_loop(void)
{
    LOGPF("boot_charge_main_loop");
    xim_bsp_button_event_registe(on_button_event_charge_mainloop);
    
    xim_wdt_feed();
    batt_interface_event_registe(on_batt_interface_handler);
    batt_interface_init();

    if((staus_now == batt_work_no_charging)
        &&(nrf_gpio_pin_read(BSP_BUTTON_2) != 0x00))
    {
        NVIC_SystemReset();
    }
//    led_command(led_request_batt_charging);
    nrf_delay_ms(100);
    led_command(led_request_red_close);
    charge_status_update();
    modify_status_led();
    
    LOGPF("init finish");
//    
//    while(true)
//    {
//        app_sched_execute();
//        m_pwr_mgmt_run();
//    }
}

void batt_interface_force_stop_charge(bool isStopNow)
{
    force_charge_stop(isStopNow);
}

//end region

