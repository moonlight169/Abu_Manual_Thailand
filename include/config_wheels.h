#define MotorFL_A PB0
#define MotorFL_B PB1

#define MotorFR_A PA6
#define MotorFR_B PA7

#define MotorRL_A PA2
#define MotorRL_B PA3

#define MotorRR_A PA1
#define MotorRR_B PA0

#define EncFL_A PB12
#define EncFL_B PB13

#define EncFR_A PB15
#define EncFR_B PB14

#define EncRL_A PB4
#define EncRL_B PB5

#define EncRR_A PB7
#define EncRR_B PB8

#define MAX_RPM 300
#define PULSE_PER_REV 844.8

#define LR_WHEELS_DISTANCE 0.420
#define FR_WHEELS_DISTANCE 0.430
#define WHEEL_RADIUS 0.153

// PWM range for 8-bit resolution (0-255)
#define PWM_MIN -254
#define PWM_MAX 255

#define stepDelay 10

#define FL_K_P 1.0
#define FL_K_I 0.05
#define FL_K_D 0.005

#define FR_K_P 1.0
#define FR_K_I 0.05
#define FR_K_D 0.005

#define RL_K_P 1.0
#define RL_K_I 0.05
#define RL_K_D 0.005

#define RR_K_P 1.0
#define RR_K_I 0.05
#define RR_K_D 0.005