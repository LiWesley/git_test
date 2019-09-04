
#include <stdint.h>
#include "xim_ntc_table.h"

#define ResistorTemperatureTableCount 166

static const int16_t ResistorTempDegreeC[ResistorTemperatureTableCount] = 
{
    -40,-39,-38,-37,-36, -35,-34,-33,-32,-31,
    -30,-29,-28,-27,-26, -25,-24,-23,-22,-21,
    -20,-29,-18,-17,-16, -15,-14,-13,-12,-11,
    -10,-9,-8,-7,-6, -5,-4,-3,-2,-1,
    0,1,2,3,4, 5,6,7,8,9,
    10,11,12,13,14, 15,16,17,18,19,
    20,21,22,23,24, 25,26,27,28,29,
    30,31,32,33,34, 35,36,37,38,39,
    40,41,42,43,44, 45,46,47,48,49,
    50,51,52,53,54, 55,56,57,58,59,
    60,61,62,63,64, 65,66,67,68,69,
    70,71,72,73,74, 75,76,77,78,79,
    80,81,82,83,84, 85,86,87,88,89,
    90,91,92,93,94, 95,96,97,98,99,
    100,101,102,103,104, 105,106,107,108,109,
    110,111,112,113,114, 115,116,117,118,119,
    120,121,122,123,124, 125,
};

static const float ResistorTempTargetOhm[ResistorTemperatureTableCount] = 
{
    195652.0, 184917.1, 174845.2, 165391.0, 156512.5, 
    148171.0, 14330.4, 132957.6, 126021.5, 119493.6, 
    113347.1, 107564.9, 102115.5, 96977.6, 92131.5, 
    87558.8, 83242.4, 79166.3, 75315.7, 71676.8, 
    68236.7, 64990.7, 61919.0, 59011.3, 56257.9, 
    53649.6, 51177.9, 48834.9, 46613.2, 44505.8, 
    42506.2, 4599.7, 38790.5, 37072.9, 35441.7, 
    33892.2, 32419.7, 31020.0, 29689.0, 28423.1, 
    27218.6, 26076.0, 24987.7, 23950.9, 22962.9, 
    22021.1, 21123.0, 2266.6, 19449.5, 18669.8, 
    17925.5, 17213.9, 16534.4, 15885.6, 15265.8, 
    14673.5, 14107.5, 13566.4, 13048.9, 12554.0, 
    12080.5, 11628.1, 11194.7, 10779.5, 10381.5, 
    10000.0, 9634.2, 9283.5, 8947.0, 8624.2, 
    8314.5, 8018.1, 7733.7, 7460.9, 7199.1, 
    6947.9, 6706.7, 6475.1, 6252.6, 6039.0, 
    5833.6, 5635.7, 5445.4, 5262.3, 5086.3, 
    4916.9, 4753.9, 4597.1, 4446.1, 4300.8, 
    4160.9, 4026.2, 3896.4, 3771.4, 3651.0, 
    3535.0, 3423.1, 3315.2, 3211.3, 3111.0, 
    3014.3, 2922.4, 2833.7, 2748.2, 2665.7, 
    2586.1, 2509.3, 2435.1, 2363.5, 2294.3, 
    2227.5, 2162.7, 2100.1, 2039.6, 1981.1, 
    1924.5, 1869.8, 1817.0, 1765.8, 1716.4, 
    1668.5, 1622.4, 1577.7, 1534.5, 1492.7, 
    1452.1, 1412.9, 1374.9, 1338.1, 1302.5, 
    1268.0, 1234.3, 1201.6, 1170.0, 1139.3, 
    1109.6, 1080.7, 1052.8, 1025.6, 999.3, 
    973.8, 949.2, 925.4, 902.2, 879.8, 
    858.0, 836.8, 816.2, 796.3, 776.9, 
    758.0, 739.7, 721.9, 704.6, 687.8, 
    671.5, 655.6, 640.2, 625.2, 610.6, 
    596.4, 582.6, 569.2, 556.2, 543.5, 
    531.1, 
};

float ParseTemperatureWithRange(float input, float max_Ohm, float min_Ohm, float min_Temperature, float max_Temperature)
{
    float percent = (input-max_Ohm)/(min_Ohm-max_Ohm);
    float add_temperature = (max_Temperature-min_Temperature)*percent;
    return min_Temperature + add_temperature;
}

float convert_from_resistorOhmToTemperature(float Ohm)
{
    int16_t index_first = 0;
    int16_t index_last = ResistorTemperatureTableCount - 1;
    int16_t index_mid;
    if(Ohm >= ResistorTempTargetOhm[index_first])
    {
        return ResistorTempDegreeC[index_first];
    }
    
    if(Ohm <= ResistorTempTargetOhm[index_last])
    {
        return ResistorTempDegreeC[index_last];
    }
    
    for(int i = 0;i<10;i++)
    {
        index_mid = (index_first + index_last)/2;
        if((index_mid == index_last - 1)&&(Ohm < ResistorTempTargetOhm[index_mid]))
        {
            return ParseTemperatureWithRange(Ohm, 
                        ResistorTempTargetOhm[index_mid],
                        ResistorTempTargetOhm[index_last], 
                        ResistorTempDegreeC[index_mid],
                        ResistorTempDegreeC[index_last]);
        }

        if((index_mid == index_first + 1)&&(Ohm > ResistorTempTargetOhm[index_mid]))
        {
            return ParseTemperatureWithRange(Ohm, 
                        ResistorTempTargetOhm[index_first],
                        ResistorTempTargetOhm[index_mid], 
                        ResistorTempDegreeC[index_first],
                        ResistorTempDegreeC[index_mid]);
        }

        if(Ohm > ResistorTempTargetOhm[index_mid])
        {
            index_last = index_mid;
        }
        else
        {
            index_first = index_mid;
        }
    }
    return 127;
}


