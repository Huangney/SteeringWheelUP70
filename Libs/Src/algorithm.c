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
    
    // int velo_0 = *velo_targ;
    // float angleDeg_0 = *angleDeg_targ;
    // int velo_1 = -*velo_targ;
    // float angleDeg_1 = *angleDeg_targ - 180.0;
    // int velo_2 = -*velo_targ;
    // float angleDeg_2 = *angleDeg_targ + 180.0;
    // int velo_3 = -*velo_targ;
    // float angleDeg_3 = *angleDeg_targ + 180.0;
    int velos[4];
    float angleDegs[4];

    if (*angleDeg_targ < 180)
    {
        velos[0] = -*velo_targ;
        angleDegs[0] = *angleDeg_targ - 180.0;
        velos[1] = *velo_targ;
        angleDegs[1] = *angleDeg_targ;
        velos[2] = -*velo_targ;
        angleDegs[2] = *angleDeg_targ + 180.0;
        velos[3] = *velo_targ;
        angleDegs[3] = *angleDeg_targ + 360.0;
    }
    else
    {
        velos[0] = *velo_targ;
        angleDegs[0] = *angleDeg_targ - 360.0;
        velos[1] = -*velo_targ;
        angleDegs[1] = *angleDeg_targ - 180.0;
        velos[2] = *velo_targ;
        angleDegs[2] = *angleDeg_targ;
        velos[3] = -*velo_targ;
        angleDegs[3] = *angleDeg_targ + 180.0;
    }
    
    // 角度归一化到0-360度
    while (angleDeg_now < 0) angleDeg_now += 360.0;
    while (angleDeg_now >= 360.0) angleDeg_now -= 360.0;

    float steer_distans[4];
    for (int i = 0; i < 4; i++)
    {
        steer_distans[i] = fabs(angleDegs[i] - angleDeg_now);
    }

    float min_distan = 999;
    for (int i = 0; i < 4; i++)
    {
        if (steer_distans[i] < min_distan)
        {
            min_distan = steer_distans[i];
        }
    }

    // 选择舵向最小的方法
    for (int i = 0; i < 4; i++)
    {
        if (min_distan == steer_distans[i])
        {
            *velo_targ = velos[i];
            
            // 角度归一化到0-360度
            while (angleDegs[i] < 0) angleDegs[i] += 360.0;
            while (angleDegs[i] >= 360.0) angleDegs[i] -= 360.0;

            *angleDeg_targ = angleDegs[i];
            return;
        }
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