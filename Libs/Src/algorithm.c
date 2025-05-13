#include "algorithm.h"
#include "math.h"

ALGO_VEC2 unit_x_vec2 =
{
    .x = 1,
};

ALGO_VEC2 unit_y_vec2 =
{
    .y = 1,
};

ALGO_VEC2 unit_45_vec2 =
{
    .x = 0.707106,
    .y = 0.707106,
};

ALGO_VEC2 unit_135_vec2 =
{
    .x = -0.707106,
    .y = 0.707106,
};

ALGO_VEC2 unit_225_vec2 =
{
    .x = -0.707106,
    .y = -0.707106,
};

ALGO_VEC2 unit_315_vec2 =
{
    .x = 0.707106,
    .y = -0.707106,
};


void algo_get_steerBetter_vec(int velo_now, float angleDeg_now, int* velo_targ, float* angleDeg_targ)
{
    // 先将极坐标系下的velo和angle映射到，直角v_theta坐标系下，并生成一组点列
    // 对于v, theta，有(v, theta)和(-v, theta -180)，(-v, theta +180)三种实现方法（点列是周期的）,我们找到这三种中舵向最近的那个点，返回回去

    // 有 基于现实的 和 基于目标的

    // 限制在 0 ~ 360度
    while (*angleDeg_targ < 0)
    {
        *angleDeg_targ += 360;
    }
    while (*angleDeg_targ > 360)
    {
        *angleDeg_targ -= 360;
    }
    
    
    int velo_mid = *velo_targ;
    float angleDeg_mid = *angleDeg_targ;

    int velo_left = -*velo_targ;
    float angleDeg_left = *angleDeg_targ - 180.0;
    int velo_right = -*velo_targ;
    float angleDeg_right = *angleDeg_targ + 180.0;

    float steer_distan_mid = fabs(angleDeg_mid - angleDeg_now);
    float steer_distan_left = fabs(angleDeg_left - angleDeg_now);
    float steer_distan_right = fabs(angleDeg_right - angleDeg_now);

    // 选择舵向最小的方法
    if (steer_distan_left < steer_distan_mid && steer_distan_left < steer_distan_right)
    {
        *velo_targ = velo_left;
        *angleDeg_targ = angleDeg_left;
    }
    if (steer_distan_right < steer_distan_mid && steer_distan_right < steer_distan_left)
    {
        *velo_targ = velo_right;
        *angleDeg_targ = angleDeg_right;
    }
}

void algo_vec2_add(ALGO_VEC2* vec_1, ALGO_VEC2* vec_2)
{
    vec_1->x += vec_2->x;
    vec_1->y += vec_2->y;
}

void algo_vec2_add_xy(ALGO_VEC2* vec_1, float x, float y)
{
    vec_1->x += x;
    vec_1->y += y;
}

void algo_vec2_multiply(ALGO_VEC2* vec, float k)
{
    vec->x *= k;
    vec->y *= k;
}

void algo_vec2_to_polesys(ALGO_VEC2* vec)
{
    float x_temp = vec->x;
    float y_temp = vec->y;

    // 计算极坐标系下的R
    vec->x = sqrt(x_temp * x_temp + y_temp * y_temp);

    // 计算极坐标系下的theta
    if (fabs(x_temp) < 0.003)
    {
        if (y_temp < -0.003)
        {
            vec->y = 270.0;
            return;
        }
        else if (y_temp > 0.003)
        {
            vec->y = 90.0;
            return;
        }
        else
        {
            vec->y = 0.0;
            return;
        }
    }
    
    if (x_temp >= 0 && y_temp >= 0)       // 第一象限
    {
        vec->y = (atan(y_temp / x_temp) / MATH_PI * 180);
    }
    else if (x_temp <= 0 && y_temp >= 0)     // 第二象限
    {
        vec->y = (atan(y_temp / x_temp) / MATH_PI * 180) + 180.0;
    }
    else if (x_temp <= 0 && y_temp <= 0)     // 第三象限
    {
        vec->y = (atan(y_temp / x_temp) / MATH_PI * 180) + 180.0;
    }
    else if (x_temp >= 0 && y_temp <= 0)     // 第四象限
    {
        vec->y = (atan(y_temp / x_temp) / MATH_PI * 180) + 360.0;
    }
    else
    {
        vec->y = (atan(y_temp / x_temp) / MATH_PI * 180);
    }
}


void algo_calc_steer_vecs_4(float V_x, float V_y, float V_w, ALGO_VEC2 Str_Ms[])
{
    // 速度向量叠加
    ALGO_VEC2 StrM_1 = 
    {
        .x = 0,
        .y = 0,
    };
    algo_vec2_add_xy(&StrM_1, V_x, V_y);
    algo_vec2_add_xy(&StrM_1, unit_135_vec2.x * V_w, unit_135_vec2.y * V_w);

    ALGO_VEC2 StrM_2 = 
    {
        .x = 0,
        .y = 0,
    };
    algo_vec2_add_xy(&StrM_2, V_x, V_y);
    algo_vec2_add_xy(&StrM_2, unit_225_vec2.x * V_w, unit_225_vec2.y * V_w);

    ALGO_VEC2 StrM_3 = 
    {
        .x = 0,
        .y = 0,
    };
    algo_vec2_add_xy(&StrM_3, V_x, V_y);
    algo_vec2_add_xy(&StrM_3, unit_315_vec2.x * V_w, unit_315_vec2.y * V_w);

    ALGO_VEC2 StrM_4 = 
    {
        .x = 0,
        .y = 0,
    };
    algo_vec2_add_xy(&StrM_4, V_x, V_y);
    algo_vec2_add_xy(&StrM_4, unit_45_vec2.x * V_w, unit_45_vec2.y * V_w);
    // 目前为止，各轮向量均为m/s单位

    // 将各向量转换到极坐标系下。
    algo_vec2_to_polesys(&StrM_1);
    algo_vec2_to_polesys(&StrM_2);
    algo_vec2_to_polesys(&StrM_3);
    algo_vec2_to_polesys(&StrM_4);

    // 换算成 ERPM
    StrM_1.x *= MeterPerSec_2_ERPM;
    StrM_2.x *= MeterPerSec_2_ERPM;
    StrM_3.x *= MeterPerSec_2_ERPM;
    StrM_4.x *= MeterPerSec_2_ERPM;

    // 填回舵轮解算
    Str_Ms[0] = StrM_1;
    Str_Ms[1] = StrM_2;
    Str_Ms[2] = StrM_3;
    Str_Ms[3] = StrM_4;
}