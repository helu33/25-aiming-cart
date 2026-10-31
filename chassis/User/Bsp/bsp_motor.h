#ifndef BSP_MOTOR_H_
#define BSP_MOTOR_H_

typedef enum { MOTOR_LEFT = 0, MOTOR_RIGHT, MOTOR_MAX } motor_id_t;
typedef enum { MOTOR_DIR_FORWARD = 0, MOTOR_DIR_BACKWARD, MOTOR_DIR_BRAKE, MOTOR_DIR_COAST } motor_dir_t;

void motor_bsp_init(void);
void motor_bsp_set_speed(motor_id_t motor, float duty_percent);
void motor_bsp_set_dual_pwm(motor_id_t motor, float duty_in2_percent, float duty_in1_percent);
void motor_bsp_stop(motor_id_t motor, motor_dir_t stop_mode);

#endif
