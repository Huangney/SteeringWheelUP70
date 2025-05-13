/**
 * 本库必须使用RTOS！！！
 * 本库必须使用RTOS！！！
 * 偷懒还没写不用RTOS的版本
 * 
 * 
 */



#ifndef MOTOR_VESC_YONG
#define MOTOR_VESC_YONG
#include "fdcan.h"

float limit_abs(float targ_num, float limit);
extern int test_rpm_value;

#ifdef __cplusplus
extern "C"
{
#endif
    /// @brief 电调上报的操作数ID类型
    typedef enum
    {
        CAN_PACKET_SET_DUTY = 0,
        CAN_PACKET_SET_CURRENT,
        CAN_PACKET_SET_CURRENT_BRAKE,
        CAN_PACKET_SET_RPM,
        CAN_PACKET_SET_POS,
        CAN_PACKET_FILL_RX_BUFFER,
        CAN_PACKET_FILL_RX_BUFFER_LONG,
        CAN_PACKET_PROCESS_RX_BUFFER,
        CAN_PACKET_PROCESS_SHORT_BUFFER,
        CAN_PACKET_STATUS,
        CAN_PACKET_SET_CURRENT_REL,
        CAN_PACKET_SET_CURRENT_BRAKE_REL,
        CAN_PACKET_SET_CURRENT_HANDBRAKE,
        CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
        CAN_PACKET_STATUS_2,
        CAN_PACKET_STATUS_3,
        CAN_PACKET_STATUS_4,
        CAN_PACKET_PING,
        CAN_PACKET_PONG,
        CAN_PACKET_DETECT_APPLY_ALL_FOC,
        CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
        CAN_PACKET_CONF_CURRENT_LIMITS,
        CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
        CAN_PACKET_CONF_CURRENT_LIMITS_IN,
        CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
        CAN_PACKET_CONF_FOC_ERPMS,
        CAN_PACKET_CONF_STORE_FOC_ERPMS,
        CAN_PACKET_STATUS_5
    } CanPacketType;
    
    typedef struct MotorVescRecvData
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t recv_data[8];
    }MotorVescRecvData;
    


    class MotorVESC
    {
    public:
        MotorVESC(FDCAN_HandleTypeDef* can_n, int motor_id, int motor_can_id); // 构造函数

        int motor_id;                       // 总线上的电机编号，第几个
        int motor_can_id;                   // 总线上的电机的CANID
        float motor_duty_real;              // 电机实际占空比
        int32_t motor_rpm_real;                 // 电机实际每分钟转速
        float motor_duty_set;               // 电机设置占空比
        int motor_rpm_set;                  // 电机设置每分钟转速
        FDCAN_HandleTypeDef* targ_can_n;       // 哪一条CAN总线
        
        void setMotorDuty(float duty);
        void setMotorRPM(int RPM);

    private:
        void sendCanTXBuffer(CanPacketType cmd_type, float values);
    };

    void can_filter_config(FDCAN_HandleTypeDef* can_n);
    void CAN_Interrupt_Enable(FDCAN_HandleTypeDef *can_n);


    /*     *********     以下四个函数可以在外部被调用   *********     */

    void motor_vesc_init(FDCAN_HandleTypeDef *hcan);     // 可以被用户自定义
    int motor_vesc_get_rpm(int motor_id);
    void motor_vesc_handle(MotorVescRecvData vesc_recvs);
    

#ifdef __cplusplus
}
#endif



#endif