#ifndef ALGORITHM_H
#define ALGORITHM_H

#define STEER_WHEEL_RADIO 0.0871
#define STEER_WHEEL_C 0.54636
#define STEER_WHEEL_60RPM_ERPM 1520
#define STEER_WHEEL_POLE_PAIRS 21
#define MATH_PI 3.1415926

#define MeterPerSec_2_ERPM_ (1 / STEER_WHEEL_C) * 60 * STEER_WHEEL_POLE_PAIRS
#define MeterPerSec_2_ERPM 2306.1718

#define REALBASE_STEERCALC 0x00     // 基于现实 的舵轮优化
#define TARGBASE_STEERCALC 0x01     // 基于目标 的舵轮优化
#define NO_STEERCALC 0x02           // 不开启 舵轮优化

typedef struct algo_vec2
{
    float x;
    float y;
}ALGO_VEC2;


void algo_get_steerBetter_vec(int velo_now, float angleDeg_now, int* velo_targ, float* angleDeg_targ);
void algo_calc_steer_vecs_4(float V_x, float V_y, float V_w, ALGO_VEC2 Str_Ms[]);

void algo_vec2_add(ALGO_VEC2* vec_1, ALGO_VEC2* vec_2);
void algo_vec2_add_xy(ALGO_VEC2* vec_1, float x, float y);
void algo_vec2_multiply(ALGO_VEC2* vec, float k);


extern ALGO_VEC2 unit_x_vec2;
extern ALGO_VEC2 unit_y_vec2;
extern ALGO_VEC2 unit_45_vec2;
extern ALGO_VEC2 unit_135_vec2;
extern ALGO_VEC2 unit_225_vec2;
extern ALGO_VEC2 unit_315_vec2;

#endif