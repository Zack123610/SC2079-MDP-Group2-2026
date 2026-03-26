/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.a
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ir_sensor.h"
#include "motor.h"
#include "servo.h"
#include "ultrasonic.h"
#include "odometry.h"
#include "pid.h"
#include "ICM20948.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MIN_PWM           (0x1518U)   /* 5400 */
#define UART_RX_DIGITS    (0x04U)
#define IMU_CAL_SAMPLES   (0x80U)     /* 128 */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* --- Robust UART digit collector --- */
static uint8_t rx_buf[4];

int echo = 0;
volatile uint8_t is_first = 1;
volatile uint32_t tc1, tc2;

PIController piA, piD;

volatile uint8_t  g_cmd_dir  = 0;
volatile uint16_t g_cmd_dist = 0;
volatile uint8_t  g_cmd_new  = 0;

volatile float g_ir_evade_distance = 0.0f;
volatile float g_target_cm = 0.0f;
volatile uint8_t  dbg_dir = 0;
volatile uint16_t dbg_dist = 0;
uint8_t dbg_rx[0x04];

static uint8_t ir_debounce_count = 0;

volatile float us_target = 0;

/* --- Robust UART digit collector --- */
static uint8_t rx_idx = 0;

/* --- Smoothing / sensors --- */
static float g_cmd_rps = 0.0f;
static float g_us_filt_cm = 0.0f;
static float g_us_last_good_cm = 0.0f;

/* --- IMU bias --- */
static float g_gyro_bias_z = 0.0f;

/* --- ASYNC STOP FLAG --- */
volatile uint8_t g_force_stop = 0;
// --- Coordinate Tracking ---
float total_y = 0.0f;
float g_prev_dist_cm = 0.0f;

/* --- Triangle parking additions --- */
static float H_BYPASS_OBS1_CM = 75.0f;   /* EDIT: forward/projection contribution to h */
static float H_BYPASS_OBS2_CM = 45.0f;   /* EDIT: forward/projection contribution to h */

static float g_h_forward_obs1_cm = 0.0f;
static float g_h_forward_obs2_cm = 0.0f;
static float g_h_total_cm = 0.0f;
static float g_b_total_cm = 0.0f;
static float g_alpha_deg = 0.0f;
static float g_beta_deg = 0.0f;
static float g_diag_return_cm = 0.0f;

static uint8_t g_h_capture_phase = 0;    /* 0 none, 1 obstacle 1 approach, 2 obstacle 2 approach */
static uint8_t g_h_capture_segment_active = 0;
static float g_h_capture_accum_cm = 0.0f;

static const float FINAL_PARK_BRAKE_DIST_CM = 0.0f;
static const float FINAL_PARK_STOP_DIST_CM  = 30.0f;
static const float FINAL_PARK_SLOW_RPS      = 1.0f;
static const float FINAL_PARK_BASE_RPS      = 2.0f;
/* Motion state */
typedef enum {
  ACT_NONE = 0,
  ACT_MOVE_CM,
  ACT_TURN_HEADING,
  ACT_APPROACH_US,
  ACT_MOVE_UNTIL_IR,
  ACT_MOVE_UNTIL_IR2,
  ACT_WAIT,
  ACT_FINAL_PARK_US
} action_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static odom_state_t g_odom;

char uartBuf[0x80];

/* --- IMU & Filter Variables --- */
float imu_accel[0x03];
float imu_gyro[0x03];

float g_yaw = 0.0f;
float g_pitch = 0.0f;
float g_roll = 0.0f;

float g_target_angle = 0.0f;
float g_target_yaw = 0.0f;

const float ALPHA = 0.98f;
uint32_t t_last_imu = 0;

static uint8_t macro_step = 0;
static uint32_t g_settle_until = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void Read_IR_Sensors(void);
float adc_to_voltage(uint16_t adc);
float voltage_to_distance_cm(float v);

static void uart_send4(const char *s);
static float clampf(float x, float lo, float hi);
static float slew_limit(float current, float target, float rate_per_s, float dt);
static float lpf(float y, float x, float a);

static void motion_hard_stop(void);
static void imu_calibrate_bias(void);
static float get_odom_avg_cm(void);
static void h_capture_begin(uint8_t phase);
static void h_capture_close_segment(void);
static void h_capture_restart_segment(void);
static void h_capture_finish(void);
static void compute_triangle_geometry(void);
static void uart_send_h_total(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 0x01, HAL_MAX_DELAY);
  return ch;
}

static void uart_send4(const char *s)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)s, 0x04, 0x0A);
}
static float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}


static float slew_limit(float current, float target, float rate_per_s, float dt)
{
  float step = rate_per_s * dt;
  float diff = target - current;

  if (diff >  step) diff =  step;
  if (diff < -step) diff = -step;

  return current + diff;
}

static float lpf(float y, float x, float a)
{
  return y + a * (x - y);
}

static void motion_hard_stop(void)
{
  MotorA_Stop();
  MotorD_Stop();
  PI_Reset(&piA);
  PI_Reset(&piD);
  g_cmd_rps = 0.0f;
}

static void imu_calibrate_bias(void)
{
  uint16_t i = 0;
  float sum_z = 0.0f;

  g_gyro_bias_z = 0.0f;
  g_yaw = 0.0f;
  g_pitch = 0.0f;
  g_roll = 0.0f;

  for (i = 0; i < IMU_CAL_SAMPLES; i++)
  {
    ICM_ReadGyro(&hi2c2, imu_gyro);
    sum_z += imu_gyro[0x02];
    HAL_Delay(0x05);
  }

  g_gyro_bias_z = sum_z / (float)IMU_CAL_SAMPLES;
}

static float get_odom_avg_cm(void)
{
  return 0.5f * (g_odom.distance_cm_A + g_odom.distance_cm_D);
}

static void h_capture_begin(uint8_t phase)
{
  g_h_capture_phase = phase;
  g_h_capture_accum_cm = 0.0f;
  g_h_capture_segment_active = 1;
}

static void h_capture_close_segment(void)
{
  if (g_h_capture_segment_active)
  {
    g_h_capture_accum_cm += get_odom_avg_cm();
    g_h_capture_segment_active = 0;
  }
}

static void h_capture_restart_segment(void)
{
  if (g_h_capture_phase != 0)
  {
    g_h_capture_segment_active = 1;
  }
}

static void h_capture_finish(void)
{
  h_capture_close_segment();

  if (g_h_capture_phase == 1)
    g_h_forward_obs1_cm = g_h_capture_accum_cm;
  else if (g_h_capture_phase == 2)
    g_h_forward_obs2_cm = g_h_capture_accum_cm;

  g_h_capture_phase = 0;
  g_h_capture_accum_cm = 0.0f;
  g_h_capture_segment_active = 0;
}

static void compute_triangle_geometry(void)
{
  /* b should come from the full straight encoder travel after the 180-degree turn.
     Only fall back to the legacy IR-derived value if the straight-travel value
     has not been captured yet. */
  if (g_b_total_cm <= 1.0f)
  {
    g_b_total_cm = 2.0f * g_ir_evade_distance;
  }

  g_h_total_cm = g_h_forward_obs1_cm + H_BYPASS_OBS1_CM + g_h_forward_obs2_cm + H_BYPASS_OBS2_CM;

  if (g_b_total_cm < 1.0f) g_b_total_cm = 1.0f;
  if (g_h_total_cm < 1.0f) g_h_total_cm = 1.0f;

  /* alpha = angle between base and diagonal
     beta  = angle between height and diagonal
     After the final 90-degree turn, the robot is aligned with the height axis,
     so the parking turn should use beta. */
  g_alpha_deg = atan2f(g_h_total_cm, 0.5f * g_b_total_cm) * 57.2957795f;
  g_beta_deg  = atan2f(0.5f * g_b_total_cm, g_h_total_cm) * 57.2957795f;
  g_diag_return_cm = sqrtf((g_h_total_cm * g_h_total_cm) +
                           ((0.5f * g_b_total_cm) * (0.5f * g_b_total_cm)));
}

static void uart_send_h_total(void)
{
//  char msg[64];
//  int len = snprintf(msg, sizeof(msg), "H_TOTAL: %.1f cm\r\n", g_h_total_cm);
//  HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 20);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  const uint32_t CONTROL_MS = 0x0A;    /* 10 ms */
  const uint32_t ODOM_MS    = 0x14;    /* 20 ms */
  const uint32_t US_MS      = 30;    /* 50 ms */

  const uint16_t SERVO_CENTER = 148;  /* 148 */
  const uint16_t SERVO_LEFT   = 95;  /* 105 */
  const uint16_t SERVO_RIGHT  = 220;  /* 210 */

  const float ACCEL_RPS_PER_S = 14.0f;

  const float ANGLE_EPS_DEG   = 2.0f;
  const float DIST_EPS_CM     = 0.8f;
  const float US_VALID_MIN    = 2.0f;
  const float US_VALID_MAX    = 300.0f;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  MotorA_Init();
  MotorD_Init();
  Servo_Init();
  ICM_Init(&hi2c2);
  HAL_Delay(0x64);

  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);

  Odom_Init(&htim2, &htim5);
  Odom_Reset(&g_odom);

  rx_idx = 0;
  HAL_UART_Receive_IT(&huart3, rx_buf, 4);
  Servo_SetAngle(SERVO_LEFT);
  HAL_Delay(500);
  Servo_SetAngle(SERVO_RIGHT);
  HAL_Delay(500);
  Servo_SetAngle(SERVO_CENTER);
  HAL_Delay(500);

  PI_Init(&piA, 11500.0f, 1000.0f, 0.0f, 65535.0f);
  PI_Init(&piD, 11500.0f, 1000.0f, 0.0f, 65535.0f);

  g_cmd_rps = 0.0f;
  g_us_filt_cm = 0.0f;
  g_us_last_good_cm = 0.0f;

  static action_t act = ACT_NONE;
  static float target_cm = 0.0f;
  static float base_rps = 0.0f;
  static uint16_t steer_angle = 0x94;
  static float   g_stop_dist = 30.0f;
  static float   g_brake_dist = 55.0f;

  uint32_t t_last_ctl  = HAL_GetTick();
  uint32_t t_last_odom = HAL_GetTick();
  uint32_t t_last_us   = HAL_GetTick();
  uint32_t t_last_oled = HAL_GetTick();
  t_last_imu = HAL_GetTick();

  imu_calibrate_bias();
  HAL_Delay(0x64);
  IR_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    const uint32_t now = HAL_GetTick();

    /* ---------- 0. CHECK FOR ASYNC STOP ---------- */
    if (g_force_stop == 1)
    {
      motion_hard_stop();
      act = ACT_NONE;
      g_cmd_new = 0;
      macro_step = 0;
      g_yaw = 0.0f;
      Servo_SetAngle(SERVO_CENTER);
      g_force_stop = 0;
      uart_send4("STOP");

      HAL_UART_Receive_IT(&huart3, rx_buf, 4);
    }

    /* ---------- 1. IMU UPDATE & FILTER ---------- */
    if ((now - t_last_imu) >= CONTROL_MS)
    {
      float dt = (float)(now - t_last_imu) / 1000.0f;
      t_last_imu = now;

      ICM_ReadAccel(&hi2c2, imu_accel);
      ICM_ReadGyro(&hi2c2, imu_gyro);

      g_yaw += (imu_gyro[0x02] - g_gyro_bias_z) * dt;

      {
        float pitch_acc = atan2f(imu_accel[0x01], imu_accel[0x02]) * 57.296f;
        float roll_acc  = atan2f(imu_accel[0x00], imu_accel[0x02]) * 57.296f;

        g_pitch = ALPHA * (g_pitch + imu_gyro[0x00] * dt) + (1.0f - ALPHA) * pitch_acc;
        g_roll  = ALPHA * (g_roll  + imu_gyro[0x01] * dt) + (1.0f - ALPHA) * roll_acc;
      }
    }

    /* ---------- 2. Ultrasonic non-blocking state machine ---------- */
        static uint8_t us_state = 0;
        static uint32_t t_us_trigger = 0;

        // State 0: Time to fire the trigger (Every 30ms)
        if (us_state == 0 && (now - t_last_us) >= US_MS)
        {
          t_last_us = now;
          t_us_trigger = now;
          US_Trigger();
          us_state = 1; // Move to wait state
        }
        // State 1: Wait 15ms for the sound to bounce back, THEN read
        else if (us_state == 1 && (now - t_us_trigger) >= 15)
        {
          float raw = US_GetDistance();
          us_state = 0; // Reset state for the next 30ms cycle

          if ((raw > US_VALID_MIN) && (raw < US_VALID_MAX))
          {
            if (g_us_filt_cm <= 0.1f) g_us_filt_cm = raw;
            else g_us_filt_cm = lpf(g_us_filt_cm, raw, 0.60f);
            g_us_last_good_cm = g_us_filt_cm;
          }
          else
          {
            if (g_us_last_good_cm > 0.1f) g_us_filt_cm = g_us_last_good_cm;
          }
        }

        /* ---------- 3. IR Sensor periodic update ---------- */
//            static uint32_t t_last_ir = 0;
//            if ((now - t_last_ir) >= 20) // Update every 20ms
//            {
//                t_last_ir = now;
//                IR_Update();
//            }


    /* ---------- 4. Accept new UART command ---------- */
    if (g_cmd_new)
    	  {
    		  g_cmd_new = 0;
    		  dbg_dir  = g_cmd_dir;
    		  dbg_dist = g_cmd_dist;

    		  MotorA_Stop();
    		  MotorD_Stop();
    		  PI_Reset(&piA);
    		  PI_Reset(&piD);
    		  g_cmd_rps = 0.0f;

    		  Odom_Reset(&g_odom);
    		  g_yaw = 0.0f;

              g_b_total_cm = 0.0f;
              g_alpha_deg = 0.0f;
              g_beta_deg = 0.0f;
              g_diag_return_cm = 0.0f;
              if (g_cmd_dir == 1) {
                g_h_forward_obs1_cm = 0.0f;
                g_h_forward_obs2_cm = 0.0f;
                g_h_total_cm = 0.0f;
              }

    		  steer_angle = SERVO_CENTER;
    		  Servo_SetAngle(steer_angle);

    	      switch (g_cmd_dir)
    		  {
    			case 0:
    			  act = ACT_NONE;
    			  break;

    			case 1: // TASK 2: FAST APPROACH
    				act = ACT_APPROACH_US;
    				steer_angle = SERVO_CENTER;
    				base_rps = 4.0f; // Go FAST
					  g_target_angle = 0.0f;
					  us_target = 25;
					  macro_step = 1;
          h_capture_begin(1);

					float initial_us = g_us_filt_cm;
          total_y += initial_us;

					if (initial_us >= 130.0f) {
						g_stop_dist = 33.0f;
						g_brake_dist = 43.0f;
					}
					else if (initial_us >= 80.0f) {
						g_stop_dist = 28.0f;
						g_brake_dist = 37.0f;
						base_rps = 4.5f;
					}
					else {
						g_stop_dist = 25.0f;
						g_brake_dist = 35.0f;
						base_rps = 4.0f;
					}
    				break;

    			case 2: // TASK 2: First "Left Arrow!" //20 - 25
    				act = ACT_TURN_HEADING;
    				steer_angle = SERVO_LEFT;
    				g_target_angle = 35.0f;
    				base_rps = 4.0f;
    				macro_step = 1;
    				break;

    			case 3: // TASK 2: First "Right Arrow!"
    				act = ACT_TURN_HEADING;
    				steer_angle = SERVO_RIGHT;
    				g_target_angle = -33.0f;
    				base_rps = 4.0f;
    				macro_step = 1;
    				break;

    			case 4: // TASK 2: Second "Left Arrow!"
    				act = ACT_TURN_HEADING;
    				steer_angle = SERVO_LEFT;
    				g_target_angle = 90.0f;
    				base_rps = 4.0f;
    				macro_step = 1;
    				break;

    			case 5: // TASK 2: Second "Right Arrow!"
    				act = ACT_TURN_HEADING;
    				steer_angle = SERVO_RIGHT;
    				g_target_angle = -90.0f; // Turn Right 90 deg
    				base_rps = 4.0f;
    				macro_step = 10;
    				break;

    			default:
    			  act = ACT_NONE;
    			  break;
    		  }

    		  Servo_SetAngle(steer_angle);
    	    }


    /* ---------- 6. Odometry update ---------- */
   		 if ((now - t_last_odom) >= ODOM_MS)
   			    {
   			      t_last_odom = now;
   			      Odom_Update_20ms(&g_odom);

   		        // if(act == ACT_APPROACH_US){
   		        // float current_dist_cm = 0.5f * (g_odom.distance_cm_A + g_odom.distance_cm_D);
   		        // float step_dist = current_dist_cm - g_prev_dist_cm;

   		        // g_prev_dist_cm = current_dist_cm;
   		        // float yaw_rad = g_yaw * (M_PI / 180.0f);
   		        // total_y += step_dist * cosf(yaw_rad);
   		        // }
   		}

   	  /* ---------- 6. Control Loop ---------- */
   			if ((now - t_last_ctl) >= CONTROL_MS)
   			{
   			  const float dt = (float)(now - t_last_ctl) / 1000.0f;
   			  t_last_ctl = now;

   			  float desired_rps = 0.0f;
   			  uint8_t done = 0;
   			float dist_cm = 0.5f * (g_odom.distance_cm_A + g_odom.distance_cm_D);
   			  // --- ACTION: APPROACH ULTRASONIC ---
   			// --- ACTION: MOVE CM ---
   			if (act == ACT_MOVE_CM)
   			{
   			    // 1. Distance Control (Move and Stop)
   			    float err_dist = target_cm - fabsf(dist_cm);

   			    if (err_dist <= DIST_EPS_CM)
   			    {
   			    	if (base_rps >= 0.0f){
   			    		MotorA_Reverse(35000);
   			    		MotorD_Reverse(35000);
   			    	}else{
   			    		MotorA_Forward(35000);
   			    		MotorD_Forward(35000);
   			    	}

					  HAL_Delay(10);
   			        done = 1;           // Reached the target distance!
   			        desired_rps = 0.0f; // Stop the motors
   			    }
   			    else
   			    {
   			        // Slow down as we get closer to the target (Proportional control)
   			        float speed_mag = 0.18f * err_dist;
   			        speed_mag = clampf(speed_mag, 0.8f, fabsf(base_rps)); // Don't go faster than base_rps, don't stall below 0.8
   			        desired_rps = (base_rps >= 0.0f) ? speed_mag : -speed_mag;
   			    }

   			    // 2. Heading Control (Drive Straight)
   			    float Kp_steer = 2.0f;
   			    float heading_err = g_target_yaw - g_yaw;

   			    if (g_cmd_rps < 0.0f) {
   			        heading_err = -heading_err; // Reverse steering fix
   			    }

   			    float new_steer = (float)SERVO_CENTER - (Kp_steer * heading_err);

   			    if (new_steer < (float)SERVO_LEFT)  new_steer = (float)SERVO_LEFT;
   			    if (new_steer > (float)SERVO_RIGHT) new_steer = (float)SERVO_RIGHT;

   			    steer_angle = (uint16_t)new_steer;
   			    Servo_SetAngle(steer_angle);
   			}else if (act == ACT_APPROACH_US)
   			  {
   				  float current_dist = g_us_filt_cm;

   	        // Use Servo to maintain heading
   	        float Kp_steer = 2.0f;
   	        float heading_err = g_target_angle - g_yaw;

   	        float new_steer = (float)SERVO_CENTER - (Kp_steer * heading_err);

   	        if (new_steer < (float)SERVO_LEFT)  new_steer = (float)SERVO_LEFT;
   	        if (new_steer > (float)SERVO_RIGHT) new_steer = (float)SERVO_RIGHT;

   	        steer_angle = (uint16_t)new_steer;
   	        Servo_SetAngle(steer_angle);

   				  const float MIN_RPS = 1.0f;
   				// Calculate actual velocity using current RPS and wheel circumference
   				// *NOTE: Change '20.4f' to your actual wheel circumference in cm (e.g., 6.5cm diameter * 3.14)*
   				float wheel_circumference = 20.42f;
   				float actual_rps = 0.5f * (g_odom.rpsA + g_odom.rpsD);
   				float velocity_cm_s = actual_rps * wheel_circumference;

   				// Estimate system lag (30ms loop + ~20ms filter lag with alpha 0.6)
   				float filter_delay_seconds = 0.05f;
   				float projected_dist = current_dist - (velocity_cm_s * filter_delay_seconds);
   				  if (projected_dist <= g_stop_dist && projected_dist > 2.0f)
   				  {
   					  // --- FIRM ACTIVE BRAKING ---
   					  // 1. Instantly punch reverse to kill forward momentum.
   					  // 35000 is a strong PWM punch. 40ms is enough to bite without actually driving backward.
   					  MotorA_Reverse(35000);
   					  MotorD_Reverse(35000);
   					  HAL_Delay(100);

   					  MotorA_Stop();
   					  MotorD_Stop();

   					  g_cmd_rps = 0.0f;
   					  desired_rps = 0.0f;

   					  done = 1;
   				  }
   				  // Phase 1: Far away -> Move as fast as possible
   				  else if (current_dist > g_brake_dist)
   				  {
   					  desired_rps = (base_rps >= 0.0f) ? base_rps : -base_rps;
   				  }
   				  // Phase 2: Crossed the threshold -> Aggressively ramp down speed
   				  else
   				  {
   					  float braking_ratio = (current_dist - g_stop_dist) / (g_brake_dist - g_stop_dist);
   					  float r = MIN_RPS + braking_ratio * (fabsf(base_rps) - MIN_RPS);

   					  desired_rps = (base_rps >= 0.0f) ? r : -r;
   				  }

   			  }
   			else if (act == ACT_FINAL_PARK_US)
   			{
   			    float current_dist = g_us_filt_cm;

   			    // 1. Maintain Heading
   			    float Kp_steer = 2.0f;
   			    float heading_err = g_target_angle - g_yaw;
   			    float new_steer = (float)SERVO_CENTER - (Kp_steer * heading_err);

   			    if (new_steer < (float)SERVO_LEFT)  new_steer = (float)SERVO_LEFT;
   			    if (new_steer > (float)SERVO_RIGHT) new_steer = (float)SERVO_RIGHT;

   			    steer_angle = (uint16_t)new_steer;
   			    Servo_SetAngle(steer_angle);

   			    // 2. Predict Future Position
   			    float wheel_circumference = 20.42f;
   			    float actual_rps = 0.5f * (g_odom.rpsA + g_odom.rpsD);
   			    float velocity_cm_s = actual_rps * wheel_circumference;
   			    float filter_delay_seconds = 0.05f;

   			    // Where the robot ACTUALLY is right now
   			    float projected_dist = current_dist - (velocity_cm_s * filter_delay_seconds);

   			    // 3. Braking and Stopping Logic using Projected Distance
   			    if (projected_dist <= 30 && projected_dist > 2.0f)
   			    {
   			        MotorA_Reverse(30000);
   			        MotorD_Reverse(30000);
   			        HAL_Delay(100);
   			        MotorA_Stop();
   			        MotorD_Stop();
   			        g_cmd_rps = 0.0f;
   			        desired_rps = 0.0f;
   			        done = 1;
   			    }
   			    else if (projected_dist <= 40)
   			    {
   			        desired_rps = 2;
   			    }
   			    else
   			    {
   			        desired_rps = 4; // (Or whatever base_rps was passed in)
   			    }
   			}
   				// --- ACTION: HEADING TURN (For Macro) ---
				else if (act == ACT_TURN_HEADING)
				{
					float err_angle = g_target_angle - g_yaw;

					// Detect Overshoot: If the error sign flips opposite to the target sign, we crossed the line.
					uint8_t overshot = 0;
					if ((g_target_angle > 0.0f && err_angle < 0.0f) ||
						(g_target_angle < 0.0f && err_angle > 0.0f))
					{
						overshot = 1;
					}

					// Stop if we are within tolerance OR if we accidentally overshot
					if (fabsf(err_angle) <= ANGLE_EPS_DEG || overshot) {
						done = 1;
						desired_rps = 0.0f;
					}
					else {
						float speed_mag = 0.10f * fabsf(err_angle);
						if (speed_mag < 1.0f) speed_mag = 1.5f;
						if (speed_mag > 3.0f) speed_mag = 3.0f;
						desired_rps = (base_rps < 0.0f) ? -speed_mag : speed_mag;
					}
				}
   			  // --- ACTION: MOVE UNTIL IR CLEARS (For Unknown Length Obstacles) ---
   				else if (act == ACT_MOVE_UNTIL_IR)
   				{
   					float side_dist = 0.0f;

   					if (g_cmd_dir == 4) side_dist = IR_GetDistance(0);

   					else if (g_cmd_dir == 5) side_dist = IR_GetDistance(1);

   					desired_rps = base_rps; // Keep driving forward

   					if (side_dist > 90.0f)
   					{
   						done = 1;
   						desired_rps = 0.0f;
   					}
   				}
   				else if (act == ACT_MOVE_UNTIL_IR2)
   				{
   				    float side_dist = 100.0f;
   				    if (g_cmd_dir == 4) side_dist = IR_GetDistance(0);
   				    else if (g_cmd_dir == 5) side_dist = IR_GetDistance(1);

   				    desired_rps = base_rps; // Set forward speed

   				    // 1. Evaluate if ANY stop condition is currently met
   				    uint8_t condition_met = 0;

   				    if (g_cmd_dir == 4 && macro_step == 10 && side_dist > 10.0f && side_dist < 500.0f){
   				    	done = 1;
						desired_rps = 0.0f;
						ir_debounce_count = 0; // Reset for next time
   				    }
   				    if (g_cmd_dir == 5 && macro_step == 10 && side_dist > 10.0f && side_dist < 100.0f)  {
   				    	done = 1;
						desired_rps = 0.0f;
						ir_debounce_count = 0; // Reset for next time
   				    }
   				    if (macro_step == 14 && side_dist > 10.0f && side_dist < 50.0f) condition_met = 1;
   				    if (macro_step == 15 && side_dist > 10.0f && side_dist < 50.0f) condition_met = 1;

   				    // 2. The Debounce Filter
   				    if (condition_met)
   				    {
   				        ir_debounce_count++;
   				        if (ir_debounce_count >= 2)
   				        {
   				            done = 1;
   				            desired_rps = 0.0f;
   				            ir_debounce_count = 0; // Reset for next time
   				        }
   				    }
   				    else
   				    {
   				        ir_debounce_count = 0; // Reset if the condition breaks
   				    }

   				    // 3. ACTIVE HEADING CONTROL (Keeps it perfectly straight!)
   				    float Kp_steer = 2.0f;
   				    float heading_err = g_target_yaw - g_yaw; // Ensure g_target_yaw is set in your macro

   				    if (g_cmd_rps < 0.0f) {
   				        heading_err = -heading_err; // Reverse steering fix
   				    }

   				    float new_steer = (float)SERVO_CENTER - (Kp_steer * heading_err);

   				    if (new_steer < (float)SERVO_LEFT)  new_steer = (float)SERVO_LEFT;
   				    if (new_steer > (float)SERVO_RIGHT) new_steer = (float)SERVO_RIGHT;

   				    steer_angle = (uint16_t)new_steer;
   				    Servo_SetAngle(steer_angle);
   				}else if (act == ACT_WAIT)
   		      {
   					Servo_SetAngle(SERVO_CENTER);
   		          desired_rps = 0.0f; // Keep motors stopped
   		          if (now >= g_settle_until)
   		          {
   		              done = 1; // Settle time is over, trigger the next step
   		          }
   		      }
   				else if (act == ACT_NONE) { desired_rps = 0.0f; }

   			  // --- RAMPING ---
   			  g_cmd_rps = slew_limit(g_cmd_rps, desired_rps, ACCEL_RPS_PER_S, dt);

   			  // --- MOTOR OUTPUT ---
   			  if (fabsf(g_cmd_rps) < 0.08f)
   			  {
   				MotorA_Stop(); MotorD_Stop();
   			  }
   			  else
   			  {
   				// 1. Set base targets for both wheels
   				float target_A = fabsf(g_cmd_rps);
   				float target_D = fabsf(g_cmd_rps);

   				// 2. Feed the (possibly modified) targets to the PID controllers
   				float outA = PI_Update(&piA, target_A, g_odom.rpsA, dt);
   				float outD = PI_Update(&piD, target_D, g_odom.rpsD, dt);
   				uint16_t pwmA = (uint16_t)(outA + (float)MIN_PWM);
   				uint16_t pwmD = (uint16_t)(outD + (float)MIN_PWM);

   				if (g_cmd_rps > 0.0f) { MotorA_Forward(pwmA); MotorD_Forward(pwmD); }
   				else                  { MotorA_Reverse(pwmA); MotorD_Reverse(pwmD); }
   			  }
      /* ---------- Finish action / Sequence Logic ---------- */
      if (done)
      		{
      			MotorA_Stop();
      			MotorD_Stop();

	      			if (g_cmd_dir == 1)
      		      {
      		          if (macro_step == 1) {
      		              act = ACT_WAIT;
      		              g_settle_until = HAL_GetTick() + 300;
      		              macro_step = 99;
      		              done = 0;
      		          }
      		          else if (macro_step == 99) {
      		              float current_US = g_us_filt_cm;
      		              float error = current_US - us_target;

      		              char msg_buf[64];
//      		              int msg_len = sprintf(msg_buf, "US: %.1f cm | Err: %.1f cm\r\n", current_US, error);
//      		              HAL_UART_Transmit(&huart3, (uint8_t*)msg_buf, msg_len, 10);

      		              if (fabsf(error) <= 1.0f) {
      		                  h_capture_finish();
      		                  macro_step = 0; done = 0; PI_Reset(&piA); PI_Reset(&piD);
      		                  act = ACT_NONE; g_cmd_dir = 0;
                              HAL_UART_Transmit(&huart3, (uint8_t*)"DONE\n", 5, 10);
      		                  HAL_UART_Receive_IT(&huart3, rx_buf, 4);
      		              }
      		              else {
      		                  h_capture_close_segment();
      		                  macro_step = 2; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);

      		                  act = ACT_MOVE_CM;
      		                  steer_angle = SERVO_CENTER;
      		                  g_yaw = 0.0f;
      		                  g_target_yaw = 0.0f;

      		                  target_cm = fabsf(error);
      		                  base_rps = (error > 0.0f) ? 2.0f : -2.0f;
      		                  h_capture_restart_segment();
      		              }
      		          }
      		          else if (macro_step == 2) {
      		              h_capture_finish();
      		              macro_step = 0; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
      		              act = ACT_NONE; g_cmd_dir = 0;
                          HAL_UART_Transmit(&huart3, (uint8_t*)"DONE\n", 5, 10);
                          HAL_UART_Receive_IT(&huart3, rx_buf, 4);
//      		              HAL_UART_Transmit(&huart3, (uint8_t*)"move\n", 5, 10);
//      		              HAL_UART_Receive_IT(&huart3, rx_buf, 4);
      		          }
      		          Servo_SetAngle(steer_angle);
      		      }
	      			// ==========================================================
					// MACRO: EVADE LEFT S-CURVE (Command 2)
					// ==========================================================
				else if (g_cmd_dir == 2)
					{
						HAL_Delay(100);

						if (macro_step == 1) {
							macro_step = 2; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
							g_yaw = 0.0f;
							act = ACT_MOVE_CM; steer_angle = SERVO_CENTER; target_cm = 10.0f; base_rps = 4.0f; g_target_yaw = 0.0f;
						}
						else if (macro_step == 2) {
							macro_step = 3; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
							g_yaw = 0.0f;
							act = ACT_TURN_HEADING; steer_angle = SERVO_RIGHT; g_target_angle = -70.0f; base_rps = 4.0f;
						}else if (macro_step == 3) {
							macro_step = 4; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
							g_yaw = 0.0f;
							act = ACT_TURN_HEADING; steer_angle = SERVO_LEFT; g_target_angle = 22.0f; base_rps = 4.0f;
						}else if (macro_step == 4) {
								  // 1. Enter the non-blocking wait state to let the US and LPF settle
								  act = ACT_WAIT; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom); g_yaw = 0.0f;
								  g_settle_until = HAL_GetTick() + 300;
								  macro_step = 5;
								  done = 0;
						}else if (macro_step == 5) {
							macro_step = 0; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
							g_stop_dist = 23.0f;
							g_brake_dist = 32.0f;
							total_y += g_us_filt_cm;
							g_yaw = 0.0f;
						  g_target_angle = 0.0f;
						  us_target = 33;
							act = ACT_APPROACH_US;
							steer_angle = SERVO_CENTER;
							base_rps = 4.0f; g_cmd_dir = 1; macro_step = 1;
						}
						Servo_SetAngle(steer_angle);
					}

					// ==========================================================
					// MACRO: EVADE RIGHT S-CURVE (Command 3)
					// ==========================================================
					else if (g_cmd_dir == 3)
					{
						HAL_Delay(100);

						if (macro_step == 1) {
							macro_step = 2; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
							g_yaw = 0.0f;
							act = ACT_MOVE_CM; steer_angle = SERVO_CENTER; target_cm = 10.0f; base_rps = 4.0f; g_target_yaw = 0.0f;
						}
						else if (macro_step == 2) {
							macro_step = 3; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
							g_yaw = 0.0f;
							act = ACT_TURN_HEADING; steer_angle = SERVO_LEFT; g_target_angle = 70.0f; base_rps = 4.0f;
						}
						else if (macro_step == 3) {
							macro_step = 4; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
							g_yaw = 0.0f;
							act = ACT_TURN_HEADING; steer_angle = SERVO_RIGHT; g_target_angle = -23.0f; base_rps = 4.0f;
						}else if (macro_step == 4) {
							  // 1. Enter the non-blocking wait state to let the US and LPF settle
							  act = ACT_WAIT; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom); g_yaw = 0.0f;
							  g_settle_until = HAL_GetTick() + 150;
							  macro_step = 5;
							  done = 0;
						}else if (macro_step == 5) {
							macro_step = 0; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
							g_stop_dist = 23.0f;
							g_brake_dist = 32.0f;
							total_y += g_us_filt_cm;
							g_yaw = 0.0f;
						  g_target_angle = 0.0f;
						  us_target = 33;
							act = ACT_APPROACH_US;
							steer_angle = SERVO_CENTER;
							base_rps = 4.0f; g_cmd_dir = 1; macro_step = 1;
						}
						Servo_SetAngle(steer_angle);
					}
      			// ==========================================================
      			// MACRO: EVADE LEFT - (Command 4)
      			// ==========================================================
                else if (g_cmd_dir == 4)
                {
                    HAL_Delay(100);
                    if (macro_step == 1) {
                        act = ACT_WAIT; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom); g_yaw = 0.0f;
                        g_settle_until = HAL_GetTick() + 300;
                        macro_step = 2;
                        done = 0;
				     }
                    if (macro_step == 2) {
                        macro_step = 3; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
                        act = ACT_MOVE_UNTIL_IR; steer_angle = SERVO_CENTER; base_rps = 4.0f;
                    }
                    else if (macro_step == 3) {
                        macro_step = 4; done = 0; PI_Reset(&piA); PI_Reset(&piD); g_yaw = 0.0f;
                        act = ACT_TURN_HEADING; steer_angle = SERVO_RIGHT; g_target_angle = -180.0f; base_rps = 5.0f;
                    } else if (macro_step == 4) {
                        act = ACT_WAIT; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom); g_yaw = 0.0f;
                        g_settle_until = HAL_GetTick() + 300;
                        macro_step = 5;
                        done = 0;
                    }else if (macro_step == 5) {
                        macro_step = 6; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
                        g_yaw = 0.0f;
                        act = ACT_MOVE_CM; steer_angle = SERVO_CENTER; target_cm = 20.0f; base_rps = 4.0f; g_target_yaw = 0.0f;
                    }
                    else if (macro_step == 6) {
                        macro_step = 7; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
                        g_yaw = 0.0f;
                        act = ACT_MOVE_UNTIL_IR; steer_angle = SERVO_CENTER; base_rps = 3.0f;
                    }
                    else if (macro_step == 7) {
                        macro_step = 77; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
                        act = ACT_TURN_HEADING; steer_angle = SERVO_RIGHT; g_target_angle = -90; base_rps = 4.0f;
                    }else if (macro_step == 77) {
                        act = ACT_WAIT; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom); g_yaw = 0.0f;
                        g_settle_until = HAL_GetTick() + 300;
                        macro_step = 8;
                        done = 0;
                    }
                    else if (macro_step == 8) {
                        macro_step = 99; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
                        g_yaw = 0.0f;
                        act = ACT_MOVE_CM; steer_angle = SERVO_CENTER; target_cm = 30; base_rps = 5.0f; g_target_yaw = 0.0f;
                    }
                    else if (macro_step == 99) {
                    	act = ACT_WAIT; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom); g_yaw = 0.0f;
						g_settle_until = HAL_GetTick() + 300;
						macro_step = 9;
						done = 0;
					}
                    else if (macro_step == 9) {
                        macro_step = 10; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
                        g_yaw = 0.0f;
                        act = ACT_MOVE_UNTIL_IR2; steer_angle = SERVO_CENTER; base_rps = 1.2f;
                    }
//                    else if (macro_step == 10) {
//                        macro_step = 11; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
//                        g_yaw = 0.0f;
//                        act = ACT_MOVE_CM; steer_angle = SERVO_CENTER; target_cm = 10.0f; base_rps = 4.0f; g_target_yaw = 0.0f;
//                    }
                    else if (macro_step == 10) {
						macro_step = 11; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
						g_yaw = 0.0f;
						act = ACT_MOVE_CM; steer_angle = SERVO_CENTER; target_cm = 10.0f; base_rps = 4.0f; g_target_yaw = 0.0f;
					}
                    else if(macro_step == 11) {
                        PI_Reset(&piA); PI_Reset(&piD);
                        g_cmd_rps = 0.0f; done = 0;Odom_Reset(&g_odom);
                        steer_angle = SERVO_RIGHT;
                        act = ACT_TURN_HEADING;
                        macro_step = 12; g_target_angle = -90.0f; base_rps = 3.0f;
                    }
                    else if(macro_step == 12) {
						macro_step = 13; PI_Reset(&piA); PI_Reset(&piD);
						g_cmd_rps = 0.0f; g_yaw=0; done = 0;Odom_Reset(&g_odom);
						act = ACT_MOVE_CM; steer_angle = SERVO_CENTER; target_cm = 10.0f; base_rps = -3.0f; g_target_yaw = 0.0f;
					}
                    else if (macro_step == 13) {
                    	macro_step = 14; done = 0; PI_Reset(&piA); PI_Reset(&piD); Odom_Reset(&g_odom);
						g_yaw = 0.0f;
						act = ACT_MOVE_UNTIL_IR2; steer_angle = SERVO_CENTER; base_rps = 2.0f; g_target_yaw = 0.0f;
                    }
                    else if(macro_step == 14) {
                    	macro_step = 15; PI_Reset(&piA); PI_Reset(&piD);
						g_cmd_rps = 0.0f; done = 0;Odom_Reset(&g_odom);
						act = ACT_MOVE_CM; steer_angle = SERVO_CENTER; target_cm = 10.0f; base_rps = -3.0f; g_target_yaw = 0.0f;
					}
                    else if(macro_step == 15) {
                        PI_Reset(&piA); PI_Reset(&piD);
                        g_cmd_rps = 0.0f; done = 0;Odom_Reset(&g_odom);
                        steer_angle = SERVO_LEFT;
                        act = ACT_TURN_HEADING;
                        macro_step = 16; g_target_angle = 90.0f; base_rps = 4.0f;
                    }
                    else if(macro_step == 16) {
						PI_Reset(&piA); PI_Reset(&piD);
						g_cmd_rps = 0.0f; us_target = 20; g_stop_dist = 20.0f;
                        g_brake_dist = 30.0f; done = 0;Odom_Reset(&g_odom);
						steer_angle = SERVO_CENTER;
						act = ACT_APPROACH_US;
						macro_step = 17; g_target_angle = 90.0f; base_rps = 2.0f;
					}
                    else if (macro_step == 17) {
                        PI_Reset(&piA); PI_Reset(&piD);
                        g_cmd_rps = 0.0f;
                        steer_angle = SERVO_CENTER;
                        act = ACT_NONE;
                        g_cmd_dir = 0;
                        macro_step = 0;
                        uart_send_h_total();
                        HAL_UART_Transmit(&huart3, (uint8_t*)"DONE\n", 5, 10);
                        HAL_UART_Receive_IT(&huart3, rx_buf, 4);
                    }

                    Servo_SetAngle(steer_angle);
                }

	      			// ==========================================================
					// MACRO: EVADE RIGHT - (Command 5)
					// ==========================================================
					else if (g_cmd_dir == 5)
					{
					  HAL_Delay(100);
					  if(macro_step == 10){
						  act = ACT_WAIT;
						  g_settle_until = HAL_GetTick() + 300;
						  macro_step = 1;
						  done = 0;
					  }
					  if (macro_step == 1)
					  {
						macro_step = 2;
						done = 0;
						PI_Reset(&piA);
						PI_Reset(&piD);
						Odom_Reset(&g_odom);

						act = ACT_MOVE_UNTIL_IR;
						steer_angle = SERVO_CENTER;
						base_rps = 4.0f;
					  }
					  else if (macro_step == 2)
					  {
						macro_step = 3;
						done = 0;
						PI_Reset(&piA);
						PI_Reset(&piD);
						g_yaw = 0.0f;

						act = ACT_TURN_HEADING;
						steer_angle = SERVO_LEFT;
						g_target_angle = 185.0f;
						base_rps = 5.0f;
					  }
					  else if (macro_step == 3)
					  {
						macro_step = 4;
						done = 0;
						PI_Reset(&piA);
						PI_Reset(&piD);
						Odom_Reset(&g_odom);
						g_yaw = 0.0f;

						act = ACT_MOVE_CM;
						steer_angle = SERVO_CENTER;
						target_cm = 20.0f;
						base_rps = 4.0f;
						g_target_yaw = 0.0f;
					  }
					  else if (macro_step == 4)
					  {
						macro_step = 5;
						done = 0;
						PI_Reset(&piA);
						PI_Reset(&piD);
						Odom_Reset(&g_odom);
						g_yaw = 0.0f;

						act = ACT_MOVE_UNTIL_IR;
						steer_angle = SERVO_CENTER;
						base_rps = 3.0f;
					  }
					  else if (macro_step == 5)
					  {
						g_b_total_cm = fabsf(0.5f * (g_odom.distance_cm_A + g_odom.distance_cm_D) + 10.0f);
						compute_triangle_geometry();

						macro_step = 6;
						done = 0;
						PI_Reset(&piA);
						PI_Reset(&piD);
						Odom_Reset(&g_odom);

						act = ACT_TURN_HEADING;
						steer_angle = SERVO_LEFT;
						g_target_angle = (g_beta_deg + 90.0f);
						base_rps = 6.0f;
					  }
					  // else if (macro_step == 10)
					  // {
					  //   compute_triangle_geometry();
					  //   macro_step = 11;
					  //   done = 0;
					  //   PI_Reset(&piA);
					  //   PI_Reset(&piD);
					  //   Odom_Reset(&g_odom);
					  //   g_yaw = 0.0f;
					  //
					  //   act = ACT_TURN_HEADING;
					  //   steer_angle = SERVO_LEFT;
					  //   g_target_angle = g_beta_deg;
					  //   base_rps = 3.0f;
					  // }
					  else if (macro_step == 6)
					  {
						macro_step = 7;
						done = 0;
						PI_Reset(&piA);
						PI_Reset(&piD);
						Odom_Reset(&g_odom);
						g_yaw = 0.0f;

						act = ACT_MOVE_CM;
						steer_angle = SERVO_CENTER;
						target_cm = g_diag_return_cm-10;
						base_rps = 5.0f;
						g_target_yaw = 0.0f;
					  }
					  else if (macro_step == 7)
					  {
						macro_step = 8;
						done = 0;
						PI_Reset(&piA);
						PI_Reset(&piD);
						Odom_Reset(&g_odom);
						g_yaw = 0.0f;

						act = ACT_FINAL_PARK_US;
						steer_angle = SERVO_CENTER;
						base_rps = 6.0f;
						g_target_angle = 0.0f;
					  }
					  else if (macro_step == 8)
					  {
						PI_Reset(&piA);
						PI_Reset(&piD);
						g_cmd_rps = 0.0f;
						steer_angle = SERVO_CENTER;
						act = ACT_NONE;
						g_cmd_dir = 0;
						macro_step = 0;

						uart_send_h_total();
						HAL_UART_Transmit(&huart3, (uint8_t *)"DONE\n", 5, 10);
						HAL_UART_Receive_IT(&huart3, rx_buf, 4);
					  }

					  Servo_SetAngle(steer_angle);
					}

            else if (g_cmd_dir == 9)
            {
                PI_Reset(&piA); PI_Reset(&piD);
                g_cmd_rps = 0.0f;
                Servo_SetAngle(SERVO_CENTER);
                act = ACT_NONE;
                g_cmd_dir = 0;

                // Send the final DONE signal back to the PC/Pi
                HAL_UART_Transmit(&huart3, (uint8_t*)"DONE\n", 5, 10);
                HAL_UART_Receive_IT(&huart3, rx_buf, 4);
            }
      			// ==========================================================
      			// STANDARD FINISH
      			// ==========================================================
      			else
      			{
      				PI_Reset(&piA); PI_Reset(&piD);
      				g_cmd_rps = 0.0f;
      				Servo_SetAngle(SERVO_CENTER);
      				act = ACT_NONE;
      				g_cmd_dir = 0;
      				HAL_UART_Transmit(&huart3, (uint8_t*)"DONE\n", 5, 10);
      				HAL_UART_Receive_IT(&huart3, rx_buf, 4);
      			}
      		}
      		}

    /* ---------- 8. OLED debug ---------- */

    if ((now - t_last_oled) >= 250)
    		{
    			t_last_oled = now;
    			char line[32];

    			// 1. Print Ultrasonic distance on the top line
    			sprintf(line, "US: %.1f cm", g_us_filt_cm);
    			OLED_ShowString(0, 0, line);

    			// 2. Fetch IR distances
    			// (If Left and Right are backward on the screen, just swap the 0 and 1)
    			float ir_left = IR_GetDistance(1);
    			float ir_right = IR_GetDistance(0);

    			// 3. Print IR distances on the second line
    			sprintf(line, "L:%.1f R:%.1f", ir_left, ir_right);
    			OLED_ShowString(0, 16, line);

    			sprintf(line, "Y:%.1f B:%.1f", g_yaw, g_beta_deg);
    			 OLED_ShowString(0, 32, line);

    			 //  sprintf(line, "DIR:%.1f", g_cmd_dir);
    			     			// OLED_ShowString(0, 48, line);

    			           sprintf(line, "b:%.1f h:%.1f", g_b_total_cm, g_h_total_cm);
    			           OLED_ShowString(0, 48, line);

    			OLED_Refresh_Gram();
    		}

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 160;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xffffffff;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 16-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DC_Pin|RESET__Pin|GPIO_PIN_13|SCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin RESET__Pin PD13 SCLK_Pin */
  GPIO_InitStruct.Pin = DC_Pin|RESET__Pin|GPIO_PIN_13|SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        // Compare the 4 received bytes.
        // We use strncmp instead of strcmp because our 4-byte buffer doesn't have a null-terminator (\0).
    	if (strncmp((char*)rx_buf, "1000", 4) == 0)
		{
			g_cmd_dir = 1;
			g_cmd_new = 1;
		}
    	else if (strncmp((char*)rx_buf, "3000", 4) == 0)
        {
            g_cmd_dir = 2;
            g_cmd_new = 1;
        }
        else if (strncmp((char*)rx_buf, "4000", 4) == 0)
        {
            g_cmd_dir = 3;
            g_cmd_new = 1;
        }
        else if (strncmp((char*)rx_buf, "3030", 4) == 0)
        {
            g_cmd_dir = 4;
            g_cmd_new = 1;
        }
        else if (strncmp((char*)rx_buf, "4040", 4) == 0)
        {
            g_cmd_dir = 5;
            g_cmd_new = 1;
        }
        else if (strncmp((char*)rx_buf, "0000", 4) == 0)
        {
            g_force_stop = 1;
        }

    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
