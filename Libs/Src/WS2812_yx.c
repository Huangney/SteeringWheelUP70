#include "WS2812_yx.h"
/*
    作者：StephenHuang

    本库基于 UTF-8 编写！！
    本库基于 UTF-8 编写！！
    本库基于 UTF-8 编写！！
    
    The Codes Are Encoding By UTF-8  !!!
    The Codes Are Encoding By UTF-8  !!!
    The Codes Are Encoding By UTF-8  !!!

    使用说明：
        （1）可以直接调用本库的函数来控制 WS2812灯带
				
        （2）推荐使用!!!：状态监测器
								1.新建CubeMX Keil工程，照常配置时钟等
								2.选择一路SPI，配置 SPI频率 尽可能靠近 8Mhz ！！
								3.启用SPI DMA，默认设置就行
								4.生成工程
								5.使用WS2812_InitBuffer()初始化
								6.覆写WS2812_Refresh()函数，推荐格式类似：下方
								7.使用WS2812_AddStateLink()向状态检测器中添加状态
								8.在循环中不断调用WS2812_State_Handler()
								9.该函数会自己根据你添加的状态更改灯带状态
								
								
		覆写：
		void WS2812_Refresh()
		{
			HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)WS2812buf2send, 24 * (LED_Nums + 1));
		}
		
*/

int targ_link_id = -1;


WS2812_StateLink WS2812_my_links[64];
uint8_t WS2812_StateNums = 0;
uint8_t WS2812buf2send[LED_Nums][24] = {0};
// int8_t Led_Center_id = 0;
WS2812_StateDescription WS2812_Empty_State =
{
    .R = 0,
    .G = 0,
    .B = 0,
    .priority = 99,
    .state = WS2812_clear
};


///////////////
//int targ_link_id = -1;
//int targ_priority = 0;
int test_int = 0;
int8_t Led_Distan = 0;
////////////////////////


float RGB_Space_Function(float x)
{
    while (x < 0)
    {
        x += 6;
    }
    while (x > 6)
    {
        x -= 6;
    }


    if (x >= 0 && x < 1)
    {
        return x;
    }
    else if (x >= 1 && x < 3)
    {
        return 1;
    }
    else if (x >= 3 && x < 4)
    {
        return 4-x;
    }
    else if (x >= 4 && x <= 6)
    {
        return 0;
    }
    else
        return 0;
}


void RGB_SetOne_Color(int8_t Led_id, uint8_t R, uint8_t G, uint8_t B)
{
    // 不允许 Led_id 大于 实际拥有的灯数
    while(Led_id > (LED_Nums - 1))
    {
        Led_id -= LED_Nums;
    }

    while(Led_id < 0)
    {
        Led_id += LED_Nums;
    }

    // 向 数组 覆写颜色
	int i = 0;
	for(i=0;i<8;i++) WS2812buf2send[Led_id][i]   = ( G & (1 << (7 -i))? (HIGH_WS2812):LOW_WS2812 );//数组某一�??0~7转化存放G
	for(i=8;i<16;i++) WS2812buf2send[Led_id][i]  = ( R & (1 << (15-i))? (HIGH_WS2812):LOW_WS2812 );//数组某一�??8~15转化存放R
	for(i=16;i<24;i++) WS2812buf2send[Led_id][i] = ( B & (1 << (23-i))? (HIGH_WS2812):LOW_WS2812 );//数组某一�??16~23转化存放B
}

void WS2812_InitBuffer()
{
    for(int i = 0; i < LED_Nums ; i++)
    {
        RGB_SetOne_Color(i, 0, 0, 0);
    }
		WS2812_Refresh();
}

void RGB_Clear_Color(int8_t Led_id)
{
    // 不允许 Led_id 大于 实际拥有的灯数
    while(Led_id > (LED_Nums - 1))
    {
        Led_id -= LED_Nums;
    }

    while(Led_id < 0)
    {
        Led_id += LED_Nums;
    }
    
    // 向 数组 覆写颜色
		for(int i = 0; i < 24; i++)
    {
        WS2812buf2send[Led_id][i] = LOW_WS2812;
    } 
}

float max(float a, float b)
{
	if(a > b) return a;
	else return b;
}


/// @brief 单色流动灯 模式
/// @param R 
/// @param G 
/// @param B 
/// @param band_wide
void WS2812_Flows(uint8_t R, uint8_t G, uint8_t B, uint8_t band_wide, float Delit_rate)
{
    static int8_t Led_Center_id = 0;
    
    for (int i = 0; i < Led_Center_id - (band_wide / 2); i++)
    {
        RGB_Clear_Color(i);
    }

    for (int i = LED_Nums - 1; i >= Led_Center_id + (band_wide / 2); i--)
    {
        RGB_Clear_Color(i);
    }
    
    for (int i = Led_Center_id - (band_wide / 2); i < Led_Center_id + (band_wide / 2); i++)
    {
        float rate = 1.0;

        if (Led_Center_id != i)
        {
            rate = max((1 - (abs(Led_Center_id - i) * 1.0 / (band_wide / 2.0)) * Delit_rate), 0.05);
						
        }
        RGB_SetOne_Color(i, R * rate, G * rate, B * rate);
    }

    Led_Center_id++;

    if(Led_Center_id > (LED_Nums - 1))
    {
        Led_Center_id = 0;
    }
}

/// @brief 角度对应模式
/// @param R 
/// @param G 
/// @param B 
/// @param band_wide 
/// @param Delit_rate 亮度强度0-1
/// @param angle 0-360度输入
void WS2812_Angles(uint8_t R, uint8_t G, uint8_t B, uint8_t band_wide, float Delit_rate, float angle, int reverse, int bias)
{
    int8_t Led_Center_id;

    if (reverse)
    {
        Led_Center_id = (1 - (angle / 360.0)) * LED_Nums + bias;
    }
    else
    {
        Led_Center_id = (angle / 360.0) * LED_Nums + bias;
    }
    
    
    // 清除其余部分的灯带
    for (int i = 0; i < Led_Center_id - (band_wide / 2); i++)
    {
        RGB_Clear_Color(i);
    }
    for (int i = LED_Nums - 1; i >= Led_Center_id + (band_wide / 2); i--)
    {
        RGB_Clear_Color(i);
    }

    // 重画灯带
    for (int i = Led_Center_id - (band_wide / 2); i < Led_Center_id + (band_wide / 2); i++)
    {
        float rate = 1.0;

        if (Led_Center_id != i)
        {
            rate = max((1 - (abs(Led_Center_id - i) * 1.0 / (band_wide / 2.0)) * Delit_rate), 0.05);
						
        }
        RGB_SetOne_Color(i, R * rate, G * rate, B * rate);
    }
}

/// @brief 用于追加关于角度的灯带颜色
/// @details 这个函数不会提前清空灯带颜色
/// @param R 
/// @param G 
/// @param B 
/// @param band_wide 
/// @param Delit_rate 亮度强度0-1
/// @param angle 
/// @param reverse 
/// @param bias 0-360度输入
void WS2812_Angles_Add(uint8_t R, uint8_t G, uint8_t B, uint8_t band_wide, float Delit_rate,float angle, int reverse, int bias)
{
    int8_t Led_Center_id;

    if (reverse)
    {
        Led_Center_id = (1 - (angle / 360.0)) * LED_Nums + bias;
    }
    else
    {
        Led_Center_id = (angle / 360.0) * LED_Nums + bias;
    }

    // 重画灯带
    for (int i = Led_Center_id - (band_wide / 2); i < Led_Center_id + (band_wide / 2); i++)
    {
        float rate = 1.0;

        if (Led_Center_id != i)
        {
            rate = max((1 - (abs(Led_Center_id - i) * 1.0 / (band_wide / 2.0)) * Delit_rate), 0.05);
						
        }
        RGB_SetOne_Color(i, R * rate, G * rate, B * rate);
    }
}

/// @brief 
/// @param R 
/// @param G 
/// @param B 
void WS2812_Wonder(float intensity)
{
    int R = 0;
    int G = 0;
    int B = 0;
    static int start_num = 0;

    start_num++;

    if(start_num > LED_Nums) start_num = 0;
    
    for (int i = 0; i < LED_Nums - 1; i++)
    {
        float funcx = (i + 1 + start_num) * 6.0 / LED_Nums;
        R = RGB_Space_Function(funcx) * 255 * intensity;
        G = RGB_Space_Function(funcx + 2) * 255 * intensity;
        B = RGB_Space_Function(funcx + 4) * 255 * intensity;

        RGB_SetOne_Color(i, R, G, B);
    }
    
}

/// @brief 颜色从灯带中间出现，填满整个灯带
/// @param R 
/// @param G 
/// @param B 
void WS2812_Expand_From_Center(uint8_t R, uint8_t G, uint8_t B)
{
    RGB_SetOne_Color((LED_Nums / 2) - Led_Distan, R, G, B);
    RGB_SetOne_Color((LED_Nums / 2) + Led_Distan, R, G, B);

    Led_Distan++;

    if (LED_Nums / 2 - Led_Distan < 0)
    {
        Led_Distan = 0;
    }
    
}

/// @brief 直接填充灯带颜色
/// @param R 
/// @param G 
/// @param B 
void WS2812_Fill(uint8_t R, uint8_t G, uint8_t B)
{
        for(int i = 0; i < LED_Nums ; i++)
    {
        RGB_SetOne_Color(i, R, G, B);
    }
}

/// @brief 将数据发送到WS2812，需要根据用户使用情况覆写
/// @return 
__weak void WS2812_Refresh()
{

}



void WS2812_AddStateLink(int* link_flag, WS2812_StateDescription targ_state)
{
    WS2812_my_links[WS2812_StateNums].link_flag = link_flag;
    WS2812_my_links[WS2812_StateNums].targ_state = targ_state;
    WS2812_StateNums++;
}

void WS2812_State_Handler()
{
  targ_link_id = -1;
	int targ_priority = 0;

    // 遍历所有存在的状态链接
    for (int i = 0; i < WS2812_StateNums; i++)
    {
		test_int = *(WS2812_my_links[i].link_flag);
        if (*(WS2812_my_links[i].link_flag) == 1)
        {
            if (WS2812_my_links[i].targ_state.priority >= targ_priority)
            {
                targ_link_id = i;
                targ_priority = WS2812_my_links[i].targ_state.priority;
            }
        }
    }


    if (targ_link_id != -1)
    {
        WS2812_Set_State(WS2812_my_links[targ_link_id].targ_state);
    }
		else
		{
			WS2812_Set_State(WS2812_Empty_State);
		}
}


void WS2812_Set_State(WS2812_StateDescription targ_state)
{
	static int state_before = 0;
    switch (targ_state.state)
    {
        case WS2812_clear:
        {
            for(int i = 0; i < LED_Nums ; i++)
            {
                RGB_SetOne_Color(i, 0, 0, 0);
            }
            break;
        }

        case WS2812_expand:
        {
            WS2812_Expand_From_Center(targ_state.R, targ_state.G, targ_state.B);
            break;
        }

        case WS2812_wonder:
        {
            WS2812_Wonder(0.5);
            break;
        }
        
        case WS2812_fill:
        {
            WS2812_Fill(targ_state.R, targ_state.G, targ_state.B);
            break;
        }

        case WS2812_flow:
        {
            WS2812_Flows(targ_state.R, targ_state.G, targ_state.B, 6, 1.25);
            break;
        }
        
        default:
        {
            break;
        }
    }
		
		if(state_before != targ_state.state)
		{
			for(int i = 0; i < LED_Nums ; i++)
			{
					RGB_SetOne_Color(i, 0, 0, 0);
			}
			Led_Distan = 0;
		}
		state_before = targ_state.state;
		WS2812_Refresh();
}























//////////////////////////////////////////////////////////

WS2812_StateDescription Yellow_Flow = 
{
.R = 255,
.G = 255,
.B = 0,
.priority = 1,
.state = WS2812_flow,
};

WS2812_StateDescription Yellow_Fill = 
{
.R = 255,
.G = 255,
.B = 0,
.priority = 1,
.state = WS2812_fill,
};

WS2812_StateDescription LightPurple_Expand = 
{
.R = 127,
.G = 0,
.B = 255,
.priority = 1,
.state = WS2812_expand,
};

WS2812_StateDescription LightBlue_Flow = 
{
.R = 0,
.G = 233,
.B = 128,
.priority = 1,
.state = WS2812_flow,
};

WS2812_StateDescription Wonderful = 
{
.R = 0,
.G = 0,
.B = 0,
.priority = 5,
.state = WS2812_wonder,
};

WS2812_StateDescription Light_White = 
{
.R = 64,
.G = 64,
.B = 64,
.priority = 2,
.state = WS2812_expand,
};

WS2812_StateDescription Pure_Blue_HighPriority = 
{
.R = 0,
.G = 0,
.B = 255,
.priority = 99,
.state = WS2812_fill,
};

WS2812_StateDescription Pure_Red_HighPriority = 
{
.R = 255,
.G = 0,
.B = 0,
.priority = 99,
.state = WS2812_fill,
};

WS2812_StateDescription Pure_Green_HighPriority = 
{
.R = 0,
.G = 255,
.B = 0,
.priority = 99,
.state = WS2812_fill,
};

WS2812_StateDescription Pure_Black_HighPriority = 
{
.R = 0,
.G = 0,
.B = 0,
.priority = 99,
.state = WS2812_fill,
};

WS2812_StateDescription Pure_White_HighPriority = 
{
.R = 255,
.G = 255,
.B = 255,
.priority = 99,
.state = WS2812_fill,
};