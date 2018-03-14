/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * Publication for the desired attitude tracking:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
 */

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
///cxy
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/a_log_att.h>
////
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>
#include <vtol_att_control/vtol_type.h>
#include <systemlib/perf_counter.h>

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define TPA_RATE_LOWER_LIMIT 0.05f
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f
#define ATTITUDE_TC_DEFAULT 0.2f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

#define MAX_GYRO_COUNT 3

class MulticopterAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControl();

	/**
	 * Destructor, also kills the main task
	 */
	~MulticopterAttitudeControl();

	/**
	 * Print vehicle mode status info loop time and loop rate
	 */
	void print_info();

	/**
	 * Start the multicopter attitude control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, task_main() should exit */
	int		_control_task;			/**< task handle */

	int		_ctrl_state_sub;		/**< control state subscription */
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub;	/**< vehicle control mode subscription */
	int		_params_sub;			/**< parameter updates subscription */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
	int		_armed_sub;				/**< arming status subscription */
	int		_vehicle_status_sub;	/**< vehicle status subscription */
	int 	_motor_limits_sub;		/**< motor limits subscription */
	int 	_battery_status_sub;	/**< battery status subscription */
	int	_sensor_gyro_sub[MAX_GYRO_COUNT];	/**< gyro data subscription */
	int	_sensor_correction_sub;	/**< sensor thermal correction subscription */
	int	local_pos_sub;
	int sensor_data_sub;

	unsigned _gyro_count;
	int _selected_gyro;

	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
	orb_advert_t	_actuators_1_pub;		/**< attitude actuator_1 controls publication */
	orb_advert_t	_controller_status_pub;	/**< controller status publication */
	//cxy
	orb_advert_t	a_log_att_pub;

	orb_id_t _rates_sp_id;	/**< pointer to correct rates setpoint uORB metadata structure */
	orb_id_t _actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	struct control_state_s				_ctrl_state;		/**< control state */
	struct vehicle_attitude_setpoint_s	_v_att_sp;			/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s			_actuators;			/**< actuator controls */
	struct actuator_controls_s			_actuators_1;		/**< actuator controls 1*/
	struct actuator_armed_s				_armed;				/**< actuator arming status */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct multirotor_motor_limits_s	_motor_limits;		/**< motor limits */
	struct mc_att_ctrl_status_s 		_controller_status; /**< controller status */
	struct battery_status_s				_battery_status;	/**< battery status */
	struct sensor_gyro_s			_sensor_gyro;		/**< gyro data before thermal correctons and ekf bias estimates are applied */
	struct sensor_correction_s		_sensor_correction;		/**< sensor thermal corrections */
	//cxy
	struct vehicle_local_position_s		local_pos;
	struct sensor_combined_s		sensor_data;
	struct a_log_att_s				a_log_att;
	
	union {
		struct {
			uint16_t motor_pos	: 1; // 0 - true when any motor has saturated in the positive direction
			uint16_t motor_neg	: 1; // 1 - true when any motor has saturated in the negative direction
			uint16_t roll_pos	: 1; // 2 - true when a positive roll demand change will increase saturation
			uint16_t roll_neg	: 1; // 3 - true when a negative roll demand change will increase saturation
			uint16_t pitch_pos	: 1; // 4 - true when a positive pitch demand change will increase saturation
			uint16_t pitch_neg	: 1; // 5 - true when a negative pitch demand change will increase saturation
			uint16_t yaw_pos	: 1; // 6 - true when a positive yaw demand change will increase saturation
			uint16_t yaw_neg	: 1; // 7 - true when a negative yaw demand change will increase saturation
			uint16_t thrust_pos	: 1; // 8 - true when a positive thrust demand change will increase saturation
			uint16_t thrust_neg	: 1; // 9 - true when a negative thrust demand change will increase saturation
		} flags;
		uint16_t value;
	} _saturation_status;

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_controller_latency_perf;
	perf_counter_t  _update_perf;			/**< loop rate */

	math::Vector<3>		_rates_prev;	/**< angular rates on previous step */
	math::Vector<3>		_rates_sp_prev; /**< previous rates setpoint */
	math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
	math::Vector<3>		_rates_int;		/**< angular rates integral error */
	float				_thrust_sp;		/**< thrust setpoint */
	math::Vector<3>		_att_control;	/**< attitude control vector */

	math::Matrix<3, 3>  _I;				/**< identity matrix */


	/********cxy circuit data**********/
	
	math::Matrix<4, 9>  k_lqr;
	math::Matrix<3, 3>  Rxyz_uvw;
	math::Matrix<3, 3> 	Rvxyz_vuvw;
	math::Matrix<3, 3> 	Reuler_body;

	math::Vector<9>  x_current;
	math::Vector<9>  x_nominal;
	math::Vector<9>  x_err;
	math::Vector<9>  x_err_nominal;
	math::Vector<9>  x_err_dot;

	

	math::Vector<4>  v_nominal;
	math::Vector<4>  v_input;
	math::Vector<4>  v_compensation;
	math::Vector<3> euler_current_v;
	math::Vector<3> euler_current_w;
	math::Vector<3> euler_last_v;
	math::Vector<3> euler_last_w;
	math::Vector<3> euler_next_v;
	math::Vector<3> euler_next_w;
	math::Vector<3> eulerrate_current_v;
	math::Vector<3> eulerrate_current_w;
	math::Vector<3> eulerrate_next_v;
	math::Vector<3> eulerrate_next_w;
	math::Vector<3> pos_current_xyz;
	math::Vector<3> pos_current_uvw;
	math::Vector<3> vel_current_xyz;
	math::Vector<3> vel_current_uvw;
	math::Vector<3> euler_angle_v_temp;
	math::Vector<3> euler_angle_w_temp;
	math::Vector<3> euler_input;
	math::Vector<3> anglerate_sp;
	math::Vector<3> rates_log;
	math::Vector<3> rates_sp_original;

	// math::Vector<9> ak;
	// math::Vector<9> bk;
	// math::Vector<9> a0;
	// math::Vector<9> ak_integral;
	// math::Vector<9> bk_integral;
	// math::Vector<9> a0_integral;

	// math::Vector<3> a0;
	// math::Vector<3> a1;
	// math::Vector<3> b1;
	// math::Vector<3> a2;
	// math::Vector<3> b2;
	// math::Vector<3> a3;
	// math::Vector<3> b3;

	// math::Vector<3>  y_err;
	math::Vector<4>  y_err;

	math::Vector<4> a0;
	math::Vector<4> a1;
	math::Vector<4> b1;
	math::Vector<4> a2;
	math::Vector<4> b2;
	math::Vector<4> a3;
	math::Vector<4> b3;
	
	// math::Vector<3> a0_integral;
	// math::Vector<3> a1_integral;
	// math::Vector<3> b1_integral;
	// math::Vector<3> a2_integral;
	// math::Vector<3> b2_integral;
	// math::Vector<3> a3_integral;
	// math::Vector<3> b3_integral;

	math::Vector<4> a0_integral;
	math::Vector<4> a1_integral;
	math::Vector<4> b1_integral;
	math::Vector<4> a2_integral;
	math::Vector<4> b2_integral;
	math::Vector<4> a3_integral;
	math::Vector<4> b3_integral;
	

	math::Vector<4> r0;
	math::Vector<4> r0_check;
	math::Vector<4> r1;
	math::Vector<4> s1;
	math::Vector<4> r2;
	math::Vector<4> s2;
	math::Vector<4> r3;
	math::Vector<4> s3;
	// math::Matrix<18, 18> C_PARAM;
	// math::Matrix<9, 9> C_PARAM_a0;
	math::Matrix<6, 18> C_PARAM;
	math::Matrix<3, 9> C_PARAM_a0;

	// math::Matrix<8, 18> JINV;
	math::Matrix<8, 18> XINV;
	math::Matrix<4, 9> RINV;
	// math::Matrix<4, 9> HINV;

	// math::Matrix<8, 6> JINV_1;
	// math::Matrix<8, 6> JINV_2;
	// math::Matrix<8, 6> JINV_3;
	// math::Matrix<4, 3> HINV;

	math::Matrix<8, 8> JINV_1;
	math::Matrix<8, 8> JINV_2;
	math::Matrix<8, 8> JINV_3;
	math::Matrix<4, 4> HINV;

	math::Matrix<9, 9> A_MAT;
	math::Matrix<9, 4> B_MAT;

	math::Vector<8> vcompensation_param_1;
	math::Vector<8> vcompensation_param_2;
	math::Vector<8> vcompensation_param_3;
	// math::Vector<18> ek_param;
	math::Vector<8> e_param_1;
	math::Vector<8> e_param_2;
	math::Vector<8> e_param_3;

	float a_input;
	float t_original;
	float t_current;
	float t_next;
	float flag_circle;
	float t_delay;
	float a_err;
	float a_err_prev;
	float a_err_last;
	float a_current;
	float d_thrust_sp;
	float thrust_sp_hover;
	int   flag_wrate;
	float flag_t;
	int   flag_cal_k;
	float k_lqr_temp2[4][9];
	int flag_origin;
	float err_u;
	float err_v;
	float flag_t_num;
	int flag_lqr_enable;
	uint64_t last_t;
	float circuit_n_current;
	float t_x;
	float t_v;
	int flag_t_x;
	int flag_t_v;
	int flag_t_v_check;
	int n_iteration;
	// int flag_tdt;
	int flag_pos_poll;
	float flag_dt;
	int flag_t_x_flag;
	int angle_n;
	int _flag_acc_controlstate;
	int _flag_acc_sensor;
	float flag_dt_sensor;
	float last_t_sensor;
	int flag_iteration;
	int flag_first_cal;
	int flag_ILC;

	// float flag_dt_controlstate;
	// uint64_t last_t_controlstate;
	


	/************************/
	math::Matrix<3, 3>	_board_rotation = {};	/**< rotation matrix for the orientation that the board is mounted */

	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_integ_lim;
		param_t roll_rate_d;
		param_t roll_rate_ff;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_integ_lim;
		param_t pitch_rate_d;
		param_t pitch_rate_ff;
		param_t tpa_breakpoint_p;
		param_t tpa_breakpoint_i;
		param_t tpa_breakpoint_d;
		param_t tpa_rate_p;
		param_t tpa_rate_i;
		param_t tpa_rate_d;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_integ_lim;
		param_t yaw_rate_d;
		param_t yaw_rate_ff;
		param_t yaw_ff;
		param_t roll_rate_max;
		param_t pitch_rate_max;
		param_t yaw_rate_max;
		param_t yaw_auto_max;

		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;
		param_t rattitude_thres;

		param_t vtol_type;
		param_t roll_tc;
		param_t pitch_tc;
		param_t vtol_opt_recovery_enabled;
		param_t vtol_wv_yaw_rate_scale;

		param_t bat_scale_en;

		param_t board_rotation;

		param_t board_offset[3];

	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		math::Vector<3> att_p;					/**< P gain for angular error */
		math::Vector<3> rate_p;				/**< P gain for angular rate error */
		math::Vector<3> rate_i;				/**< I gain for angular rate error */
		math::Vector<3> rate_int_lim;			/**< integrator state limit for rate loop */
		math::Vector<3> rate_d;				/**< D gain for angular rate error */
		math::Vector<3>	rate_ff;			/**< Feedforward gain for desired rates */
		float yaw_ff;						/**< yaw control feed-forward */

		float tpa_breakpoint_p;				/**< Throttle PID Attenuation breakpoint */
		float tpa_breakpoint_i;				/**< Throttle PID Attenuation breakpoint */
		float tpa_breakpoint_d;				/**< Throttle PID Attenuation breakpoint */
		float tpa_rate_p;					/**< Throttle PID Attenuation slope */
		float tpa_rate_i;					/**< Throttle PID Attenuation slope */
		float tpa_rate_d;					/**< Throttle PID Attenuation slope */

		float roll_rate_max;
		float pitch_rate_max;
		float yaw_rate_max;
		float yaw_auto_max;
		math::Vector<3> mc_rate_max;		/**< attitude rate limits in stabilized modes */
		math::Vector<3> auto_rate_max;		/**< attitude rate limits in auto modes */
		math::Vector<3> acro_rate_max;		/**< max attitude rates in acro mode */
		float rattitude_thres;
		int vtol_type;						/**< 0 = Tailsitter, 1 = Tiltrotor, 2 = Standard airframe */
		bool vtol_opt_recovery_enabled;
		float vtol_wv_yaw_rate_scale;			/**< Scale value [0, 1] for yaw rate setpoint  */

		int bat_scale_en;

		int board_rotation;

		float board_offset[3];

	}		_params;

	TailsitterRecovery *_ts_opt_recovery;	/**< Computes optimal rates for tailsitter recovery */

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll();

	void 		local_pos_poll();

	void		sensor_data_poll();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for attitude setpoint updates.
	 */
	void		vehicle_attitude_setpoint_poll();

	/**
	 * Check for rates setpoint updates.
	 */
	void		vehicle_rates_setpoint_poll();

	/**
	 * Check for arming status updates.
	 */
	void		arming_status_poll();

	/**
	 * Attitude controller.
	 */
	void		control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

	/**
	 * Throttle PID attenuation.
	 */
	math::Vector<3> pid_attenuations(float tpa_breakpoint, float tpa_rate);

	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for vehicle motor limits status.
	 */
	void		vehicle_motor_limits_poll();

	/**
	 * Check for battery status updates.
	 */
	void		battery_status_poll();

	/**
	 * Check for control state updates.
	 */
	void		control_state_poll();

	/**
	 * Check for sensor thermal correction updates.
	 */
	void		sensor_correction_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();
};

namespace mc_att_control
{

MulticopterAttitudeControl	*g_control;
}

MulticopterAttitudeControl::MulticopterAttitudeControl() :

	_task_should_exit(false),
	_control_task(-1),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),
	_vehicle_status_sub(-1),
	_motor_limits_sub(-1),
	_battery_status_sub(-1),
	_sensor_correction_sub(-1),

	//cxy
	local_pos_sub(-1),
	sensor_data_sub(-1),

	/* gyro selection */
	_gyro_count(1),
	_selected_gyro(0),

	/* publications */
	_v_rates_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_actuators_1_pub(nullptr),
	_controller_status_pub(nullptr),
	//cxy
	// a_log_att_pub(nullptr),

	_rates_sp_id(nullptr),
	_actuators_id(nullptr),

	_actuators_0_circuit_breaker_enabled(false),


	_ctrl_state{},
	_v_att_sp{},
	_v_rates_sp{},
	_manual_control_sp{},
	_v_control_mode{},
	_actuators{},
	_actuators_1{},
	_armed{},
	_vehicle_status{},
	_motor_limits{},
	_controller_status{},
	_battery_status{},
	_sensor_gyro{},
	_sensor_correction{},
	//cxy
	local_pos{},
	sensor_data{},
	a_log_att{},
	//
	_saturation_status{},

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),
	_update_perf(perf_alloc(PC_INTERVAL,"mc_att_control_rate")),
	_ts_opt_recovery(nullptr)
{
	for (uint8_t i = 0; i < MAX_GYRO_COUNT; i++) {
		_sensor_gyro_sub[i] = -1;
	}

	_vehicle_status.is_rotary_wing = true;

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.rate_i.zero();
	_params.rate_int_lim.zero();
	_params.rate_d.zero();
	_params.rate_ff.zero();
	_params.yaw_ff = 0.0f;
	_params.roll_rate_max = 0.0f;
	_params.pitch_rate_max = 0.0f;
	_params.yaw_rate_max = 0.0f;
	_params.mc_rate_max.zero();
	_params.auto_rate_max.zero();
	_params.acro_rate_max.zero();
	_params.rattitude_thres = 1.0f;
	_params.vtol_opt_recovery_enabled = false;
	_params.vtol_wv_yaw_rate_scale = 1.0f;
	_params.bat_scale_en = 0;

	_params.board_rotation = 0;

	_params.board_offset[0] = 0.0f;
	_params.board_offset[1] = 0.0f;
	_params.board_offset[2] = 0.0f;

	_rates_prev.zero();
	_rates_sp.zero();
	_rates_sp_prev.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	///********cxy circuit data************////

	x_current.zero();
	x_nominal.zero();
	x_err.zero();
	x_err_nominal.zero();
	x_err_dot.zero();

	y_err.zero();
	v_nominal.zero();
	v_input.zero();
	v_compensation.zero();
	euler_current_v.zero();
	euler_current_w.zero();
	euler_last_v.zero();
	euler_last_w.zero();
	euler_next_v.zero();
	euler_next_w.zero();
	eulerrate_current_v.zero();
	eulerrate_current_w.zero();
	eulerrate_next_v.zero();
	eulerrate_next_w.zero();
	pos_current_xyz.zero();
	pos_current_uvw.zero();
	vel_current_xyz.zero();
	vel_current_uvw.zero();
	euler_angle_v_temp.zero();
	euler_angle_w_temp.zero();
	euler_input.zero();
	anglerate_sp.zero();
	
	
	rates_log.zero();
	rates_sp_original.zero();
	t_original=0;
	t_current=0;
	t_next=0;
	a_input=0;
	flag_circle=0;
	t_delay=0;
	a_err=0;
	a_err_prev=0;
	a_err_last=0;
	a_current=0;
	d_thrust_sp=0;
	thrust_sp_hover=0;
	flag_wrate=0;
	flag_t=0;
	flag_cal_k=0;
	flag_origin=0;
	flag_t_num=0;
	flag_lqr_enable=0;
	last_t=0;
	circuit_n_current=0;
	t_x=0;
	t_v=0;
	flag_t_x=0;
	flag_t_v=0;
	flag_t_v_check=0;
	flag_first_cal=1;
	flag_ILC=0;

	vcompensation_param_1.zero();
	vcompensation_param_2.zero();
	vcompensation_param_3.zero();

	a0.zero();
	a1.zero();
	b1.zero();
	a2.zero();
	b2.zero();
	a3.zero();
	b3.zero();
	
	a0_integral.zero();
	a1_integral.zero();
	b1_integral.zero();
	a2_integral.zero();
	b2_integral.zero();
	a3_integral.zero();
	b3_integral.zero();
	
	r0.zero();
	r1.zero();
	s1.zero();
	r2.zero();
	s2.zero();
	r3.zero();
	s3.zero();

	e_param_1.zero();
	e_param_2.zero();
	e_param_3.zero();

	n_iteration=0;
	// flag_tdt=0;
	flag_pos_poll=0;
	flag_dt=0;
	flag_t_x_flag=0;
	angle_n=1;
	_flag_acc_controlstate=0;
	_flag_acc_sensor=0;
	flag_dt_sensor=0;
	last_t_sensor=0;
	flag_iteration=1;
	// flag_dt_controlstate=0;
	// last_t_controlstate=0;

	/********************/

	_I.identity();
	_board_rotation.identity();

	_params_handles.roll_p			= 	param_find("MC_ROLL_P");
	_params_handles.roll_rate_p		= 	param_find("MC_ROLLRATE_P");
	_params_handles.roll_rate_i		= 	param_find("MC_ROLLRATE_I");
	_params_handles.roll_rate_integ_lim	= 	param_find("MC_RR_INT_LIM");
	_params_handles.roll_rate_d		= 	param_find("MC_ROLLRATE_D");
	_params_handles.roll_rate_ff	= 	param_find("MC_ROLLRATE_FF");
	_params_handles.pitch_p			= 	param_find("MC_PITCH_P");
	_params_handles.pitch_rate_p	= 	param_find("MC_PITCHRATE_P");
	_params_handles.pitch_rate_i	= 	param_find("MC_PITCHRATE_I");
	_params_handles.pitch_rate_integ_lim	= 	param_find("MC_PR_INT_LIM");
	_params_handles.pitch_rate_d	= 	param_find("MC_PITCHRATE_D");
	_params_handles.pitch_rate_ff 	= 	param_find("MC_PITCHRATE_FF");
	_params_handles.tpa_breakpoint_p 	= 	param_find("MC_TPA_BREAK_P");
	_params_handles.tpa_breakpoint_i 	= 	param_find("MC_TPA_BREAK_I");
	_params_handles.tpa_breakpoint_d 	= 	param_find("MC_TPA_BREAK_D");
	_params_handles.tpa_rate_p	 	= 	param_find("MC_TPA_RATE_P");
	_params_handles.tpa_rate_i	 	= 	param_find("MC_TPA_RATE_I");
	_params_handles.tpa_rate_d	 	= 	param_find("MC_TPA_RATE_D");
	_params_handles.yaw_p			=	param_find("MC_YAW_P");
	_params_handles.yaw_rate_p		= 	param_find("MC_YAWRATE_P");
	_params_handles.yaw_rate_i		= 	param_find("MC_YAWRATE_I");
	_params_handles.yaw_rate_integ_lim	= 	param_find("MC_YR_INT_LIM");
	_params_handles.yaw_rate_d		= 	param_find("MC_YAWRATE_D");
	_params_handles.yaw_rate_ff	 	= 	param_find("MC_YAWRATE_FF");
	_params_handles.yaw_ff			= 	param_find("MC_YAW_FF");
	_params_handles.roll_rate_max	= 	param_find("MC_ROLLRATE_MAX");
	_params_handles.pitch_rate_max	= 	param_find("MC_PITCHRATE_MAX");
	_params_handles.yaw_rate_max	= 	param_find("MC_YAWRATE_MAX");
	_params_handles.yaw_auto_max	= 	param_find("MC_YAWRAUTO_MAX");
	_params_handles.acro_roll_max	= 	param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max	= 	param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max	= 	param_find("MC_ACRO_Y_MAX");
	_params_handles.rattitude_thres = 	param_find("MC_RATT_TH");
	_params_handles.vtol_type 		= 	param_find("VT_TYPE");
	_params_handles.roll_tc			= 	param_find("MC_ROLL_TC");
	_params_handles.pitch_tc		= 	param_find("MC_PITCH_TC");
	_params_handles.vtol_opt_recovery_enabled	= param_find("VT_OPT_RECOV_EN");
	_params_handles.vtol_wv_yaw_rate_scale		= param_find("VT_WV_YAWR_SCL");
	_params_handles.bat_scale_en		= param_find("MC_BAT_SCALE_EN");

	/* rotations */
	_params_handles.board_rotation = param_find("SENS_BOARD_ROT");

	/* rotation offsets */
	_params_handles.board_offset[0] = param_find("SENS_BOARD_X_OFF");
	_params_handles.board_offset[1] = param_find("SENS_BOARD_Y_OFF");
	_params_handles.board_offset[2] = param_find("SENS_BOARD_Z_OFF");

	/* fetch initial parameter values */
	parameters_update();

	if (_params.vtol_type == vtol_type::TAILSITTER && _params.vtol_opt_recovery_enabled) {
		// the vehicle is a tailsitter, use optimal recovery control strategy
		_ts_opt_recovery = new TailsitterRecovery();
	}

	/* initialize thermal corrections as we might not immediately get a topic update (only non-zero values) */
	for (unsigned i = 0; i < 3; i++) {
		// used scale factors to unity
		_sensor_correction.gyro_scale_0[i] = 1.0f;
		_sensor_correction.gyro_scale_1[i] = 1.0f;
		_sensor_correction.gyro_scale_2[i] = 1.0f;
	}
}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	if (_ts_opt_recovery != nullptr) {
		delete _ts_opt_recovery;
	}

	mc_att_control::g_control = nullptr;
	perf_free(_loop_perf);
	perf_free(_update_perf);
	perf_free(_controller_latency_perf);
}

int
MulticopterAttitudeControl::parameters_update()
{
	float v;

	float roll_tc, pitch_tc;

	param_get(_params_handles.roll_tc, &roll_tc);
	param_get(_params_handles.pitch_tc, &pitch_tc);

	/* roll gains */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_i, &v);
	_params.rate_i(0) = v;
	param_get(_params_handles.roll_rate_integ_lim, &v);
	_params.rate_int_lim(0) = v;
	param_get(_params_handles.roll_rate_d, &v);
	_params.rate_d(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_ff, &v);
	_params.rate_ff(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_i, &v);
	_params.rate_i(1) = v;
	param_get(_params_handles.pitch_rate_integ_lim, &v);
	_params.rate_int_lim(1) = v;
	param_get(_params_handles.pitch_rate_d, &v);
	_params.rate_d(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_ff, &v);
	_params.rate_ff(1) = v;

	param_get(_params_handles.tpa_breakpoint_p, &_params.tpa_breakpoint_p);
	param_get(_params_handles.tpa_breakpoint_i, &_params.tpa_breakpoint_i);
	param_get(_params_handles.tpa_breakpoint_d, &_params.tpa_breakpoint_d);
	param_get(_params_handles.tpa_rate_p, &_params.tpa_rate_p);
	param_get(_params_handles.tpa_rate_i, &_params.tpa_rate_i);
	param_get(_params_handles.tpa_rate_d, &_params.tpa_rate_d);

	/* yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	param_get(_params_handles.yaw_rate_i, &v);
	_params.rate_i(2) = v;
	param_get(_params_handles.yaw_rate_integ_lim, &v);
	_params.rate_int_lim(2) = v;
	param_get(_params_handles.yaw_rate_d, &v);
	_params.rate_d(2) = v;
	param_get(_params_handles.yaw_rate_ff, &v);
	_params.rate_ff(2) = v;

	param_get(_params_handles.yaw_ff, &_params.yaw_ff);

	/* angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.mc_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.mc_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
	_params.mc_rate_max(2) = math::radians(_params.yaw_rate_max);

	/* auto angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.auto_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.auto_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_auto_max, &_params.yaw_auto_max);
	_params.auto_rate_max(2) = math::radians(_params.yaw_auto_max);

	/* manual rate control scale and auto mode roll/pitch rate limits */
	param_get(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	param_get(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	param_get(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);

	/* stick deflection needed in rattitude mode to control rates not angles */
	param_get(_params_handles.rattitude_thres, &_params.rattitude_thres);

	param_get(_params_handles.vtol_type, &_params.vtol_type);

	int tmp;
	param_get(_params_handles.vtol_opt_recovery_enabled, &tmp);
	_params.vtol_opt_recovery_enabled = (bool)tmp;

	param_get(_params_handles.vtol_wv_yaw_rate_scale, &_params.vtol_wv_yaw_rate_scale);

	param_get(_params_handles.bat_scale_en, &_params.bat_scale_en);

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	/* rotation of the autopilot relative to the body */
	param_get(_params_handles.board_rotation, &(_params.board_rotation));

	/* fine adjustment of the rotation */
	param_get(_params_handles.board_offset[0], &(_params.board_offset[0]));
	param_get(_params_handles.board_offset[1], &(_params.board_offset[1]));
	param_get(_params_handles.board_offset[2], &(_params.board_offset[2]));

	return OK;
}

void
MulticopterAttitudeControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}
//  cxy   get current local_pos
void
MulticopterAttitudeControl::local_pos_poll()
{
	bool updated;
	orb_check(local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);
		flag_pos_poll=1;
	}
	else
	{
		flag_pos_poll=0;
	}
}

void
MulticopterAttitudeControl::sensor_data_poll()
{
	bool updated;
	orb_check(sensor_data_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_combined), sensor_data_sub, &sensor_data);
		_flag_acc_sensor=1;
	}
	else
	{
		_flag_acc_sensor=0;
	}
}

void
MulticopterAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
MulticopterAttitudeControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

void
MulticopterAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(mc_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void
MulticopterAttitudeControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &_motor_limits);
		_saturation_status.value = _motor_limits.saturation_status;
	}
}

void
MulticopterAttitudeControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

void
MulticopterAttitudeControl::control_state_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_ctrl_state_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
		_flag_acc_controlstate=1;
	}
	else
	{
		_flag_acc_controlstate=0;
	}
}

void
MulticopterAttitudeControl::sensor_correction_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_sensor_correction_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_correction), _sensor_correction_sub, &_sensor_correction);
	}

	/* update the latest gyro selection */
	if (_sensor_correction.selected_gyro_instance < _gyro_count) {
		_selected_gyro = _sensor_correction.selected_gyro_instance;
	}
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
MulticopterAttitudeControl::control_attitude(float dt)
{
	vehicle_attitude_setpoint_poll();

	_thrust_sp = _v_att_sp.thrust;

	math::Quaternion q_sp(_v_att_sp.q_d[0], _v_att_sp.q_d[1], _v_att_sp.q_d[2], _v_att_sp.q_d[3]);
	math::Matrix<3, 3> R_sp = q_sp.to_dcm();
	/* get current rotation matrix from control state quaternions */
	math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
	math::Matrix<3, 3> R = q_att.to_dcm();

	math::Vector<3> e_R;
	e_R.zero();
	// if the vehicle is a tailsitter and in fw mode, we have to rotate the attitude by the pitch offset
	// between multirotor and fixed wing body axes for att_sp, and only control mc body yaw and pitch (i.e fw body roll and pitch)
	if ( _params.vtol_type == vtol_type::TAILSITTER && _vehicle_status.is_vtol 
		&& !_vehicle_status.is_rotary_wing && !_vehicle_status.in_transition_mode ) {

		math::Matrix<3, 3> R_offset;
		R_offset.from_euler(0, -M_PI_2_F, 0);
		// lyu: get current euler angle in fw body axis
		// math::Matrix<3, 3> R_offset_tofw;
		// R_offset_tofw.from_euler(0, M_PI_2_F, 0);
		// math::Matrix<3, 3> R_fw = R * R_offset_tofw;
		// math::Vector<3> euler = R_fw.to_euler();
		// lyu: actually, the roll and yaw angles are same in fw and mc body axis. if we use Tait-Bryan: pry (zxy)
		math::Vector<3> euler = R.to_euler();
		// get fw att_sp ( use fw current yaw)
		R_sp.from_euler(_v_att_sp.roll_body, _v_att_sp.pitch_body, euler(2));
		// convert to mc body axis
		R_sp = R_sp * R_offset;

	} else {

		/* construct attitude setpoint rotation matrix for mc */
		// do noting, use the original R_sp

	}

	/* calculate weight for yaw control */
	float yaw_w = R_sp(2, 2) * R_sp(2, 2);	

	/* use quaternion with linearization for tailsitter */
	if (_params.vtol_type == vtol_type::TAILSITTER) {

		math::Matrix<3, 3> e_R_M = R.transposed() * R_sp;
		math::Quaternion q_e;
		q_e.from_dcm(e_R_M);
		float sign = 1.0f;
		if (q_e(0) > 0.0f) {
			sign = 1.0f;
		} else {
			sign = -1.0f;
		}
		q_e = q_e * sign;
		e_R = q_e.imag();
		// start to linearize: q.imag is sin(theta/2)*w
		float rotate_angle = 2.0f * acosf(q_e(0));
		if (rotate_angle < 0.001f){
			e_R = e_R/(0.5f - 0.020833f * rotate_angle * rotate_angle);
		} else {
			e_R = e_R*rotate_angle/sinf(rotate_angle/2.0f);
		}

	} else {
		/* all input data is ready, run controller itself */
		/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
		math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
		math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));	
		/* axis and sin(angle) of desired rotation */
		e_R = R.transposed() * (R_z % R_sp_z);	

		/* calculate angle error */
		float e_R_z_sin = e_R.length();
		float e_R_z_cos = R_z * R_sp_z;	

		/* calculate rotation matrix after roll/pitch only rotation */
		math::Matrix<3, 3> R_rp;	

		if (e_R_z_sin > 0.0f) {
			/* get axis-angle representation */
			float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
			math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;	

			e_R = e_R_z_axis * e_R_z_angle;	

			/* cross product matrix for e_R_axis */
			math::Matrix<3, 3> e_R_cp;
			e_R_cp.zero();
			e_R_cp(0, 1) = -e_R_z_axis(2);
			e_R_cp(0, 2) = e_R_z_axis(1);
			e_R_cp(1, 0) = e_R_z_axis(2);
			e_R_cp(1, 2) = -e_R_z_axis(0);
			e_R_cp(2, 0) = -e_R_z_axis(1);
			e_R_cp(2, 1) = e_R_z_axis(0);	

			/* rotation matrix for roll/pitch only rotation */
			R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));	

		} else {
			/* zero roll/pitch rotation */
			R_rp = R;
		}	

		/* R_rp and R_sp has the same Z axis, calculate yaw error */
		math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
		math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
		e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;	

		if (e_R_z_cos < 0.0f) {
			/* for large thrust vector rotations use another rotation method:
			 * calculate angle and axis for R -> R_sp rotation directly */
			math::Quaternion q_error;
			q_error.from_dcm(R.transposed() * R_sp);
			math::Vector<3> e_R_d = q_error(0) >= 0.0f ? q_error.imag()  * 2.0f : -q_error.imag() * 2.0f;	
			/* use fusion of Z axis based rotation and direct rotation */
			float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
			e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
		}

	}
	
	/* calculate angular rates setpoint */
	_rates_sp = _params.att_p.emult(e_R);

	/* limit rates */
	for (int i = 0; i < 3; i++) {
		if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
		    !_v_control_mode.flag_control_manual_enabled) {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.auto_rate_max(i), _params.auto_rate_max(i));
		} else {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
		}
	}

	/* feed forward yaw setpoint rate */
	_rates_sp(2) += _v_att_sp.yaw_sp_move_rate * yaw_w * _params.yaw_ff;
	/* weather-vane mode, dampen yaw rate */
	if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
	    _v_att_sp.disable_mc_yaw_control == true && !_v_control_mode.flag_control_manual_enabled) {
		float wv_yaw_rate_max = _params.auto_rate_max(2) * _params.vtol_wv_yaw_rate_scale;
		_rates_sp(2) = math::constrain(_rates_sp(2), -wv_yaw_rate_max, wv_yaw_rate_max);
		// prevent integrator winding up in weathervane mode
		_rates_int(2) = 0.0f;
		// PX4_WARN("WEATHER-VANE MODE");
	}
}

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
math::Vector<3>
MulticopterAttitudeControl::pid_attenuations(float tpa_breakpoint, float tpa_rate)
{
	/* throttle pid attenuation factor */
	float tpa = 1.0f - tpa_rate * (fabsf(_v_rates_sp.thrust) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
	tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

	math::Vector<3> pidAttenuationPerAxis;
	pidAttenuationPerAxis(AXIS_INDEX_ROLL) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_PITCH) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_YAW) = 1.0;

	return pidAttenuationPerAxis;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
MulticopterAttitudeControl::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	if (!_armed.armed || !_vehicle_status.is_rotary_wing) {
		_rates_int.zero();
	}

	// *****LQR cxy********//
	/////cxy circuit data//////
	local_pos_poll();
	vehicle_control_mode_poll();
	// control_state_poll();
	sensor_data_poll();
	vehicle_manual_poll();

	//***circuit param*****//
	float pi=3.1415926;

	float r_nominal;
	float anglerate_nominal;
	float t_circuit;
	float t_circuit_start;
	float n_start_ilc=3;   //////circles before ILC collect data
	float circuit_laps=60;
	float circuit_n_sp=circuit_laps+n_start_ilc;  // circle number
	float tpercircle;
	float n_x_cal=1;     /////collect xerr for n circles
	float n_v_cal=2;     /////wait for compensation
	float r_param_0=0.3;  /// set 0 to shut down this order compesation
	float r_param_1=0.3;
	float r_param_2=0;
	float r_param_3=0;
	float t_delay_sp=10; //////time for hover before start circuit
	float T_n;


	//set ILC status
	bool ILC_enable=true;  

	r_nominal=1;				// circuit radius
	anglerate_nominal=2*pi/2;//2*pi/5;    //circuit speed
	tpercircle=2*pi/anglerate_nominal;
	t_circuit=circuit_n_sp*tpercircle;
	t_circuit_start=n_start_ilc*tpercircle;
	// r_nominal=0;q
	// anglerate_nominal=0;

	T_n=tpercircle*n_x_cal;

	float heiht_nominal=-1;

	float g_nominal= 9.8;
	float a_nominal=-1*sqrtf(g_nominal*g_nominal+anglerate_nominal*anglerate_nominal*r_nominal*anglerate_nominal*anglerate_nominal*r_nominal);
	
	
	

	a_log_att.bodyratesp_x=anglerate_sp(0);
	a_log_att.bodyratecur_x=rates_log(0);
	a_log_att.bodyratesp_x_original=rates_sp_original(0);
	a_log_att.bodyratesp_y=anglerate_sp(1);
	a_log_att.bodyratecur_y=rates_log(1);
	a_log_att.bodyratesp_y_original=rates_sp_original(1);
	a_log_att.bodyratesp_z=anglerate_sp(2);
	a_log_att.bodyratecur_z=rates_log(2);
	a_log_att.bodyratesp_z_original=rates_sp_original(2);
	a_log_att.bodyrateerr_x=anglerate_sp(0)-rates_log(0);
	a_log_att.bodyrateerr_y=anglerate_sp(1)-rates_log(1);
	a_log_att.bodyrateerr_z=anglerate_sp(2)-rates_log(2);
	
	a_log_att.vratesp_x=euler_input(0);
	a_log_att.vratesp_y=euler_input(1);
	a_log_att.vratesp_z=euler_input(2);
	a_log_att.uvwratesp_x=v_input(2);
	a_log_att.uvwratesp_y=v_input(1);
	a_log_att.uvwratesp_z=anglerate_nominal;



		/*get current euler angle  z y x*/
	euler_current_v(0)=atan2(2.0f*(_ctrl_state.q[0]*_ctrl_state.q[3]+_ctrl_state.q[1]*_ctrl_state.q[2]),1.0f-2.0f*(_ctrl_state.q[2]*_ctrl_state.q[2]+_ctrl_state.q[3]*_ctrl_state.q[3]));
	euler_current_v(1)=asin(2.0f*(_ctrl_state.q[0]*_ctrl_state.q[2]-_ctrl_state.q[1]*_ctrl_state.q[3]));
	euler_current_v(2)=atan2(2.0f*(_ctrl_state.q[0]*_ctrl_state.q[1]+_ctrl_state.q[2]*_ctrl_state.q[3]),1.0f-2.0f*(_ctrl_state.q[1]*_ctrl_state.q[1]+_ctrl_state.q[2]*_ctrl_state.q[2]));

	euler_angle_w_temp(0)=sinf(euler_current_v(0))*sinf(euler_current_v(2))+cosf(euler_current_v(0))*sinf(euler_current_v(1))*cosf(euler_current_v(2));
	euler_angle_w_temp(1)=-cosf(euler_current_v(0))*sinf(euler_current_v(2))+sinf(euler_current_v(0))*sinf(euler_current_v(1))*cosf(euler_current_v(2));
	euler_angle_w_temp(2)=cosf(euler_current_v(1))*cosf(euler_current_v(2));
	
	//euler_current_ yita miu niu
	euler_current_w(0)=anglerate_nominal*t_current;
	euler_current_w(1)=atan2(euler_angle_w_temp(0)*cosf(euler_current_w(0))+euler_angle_w_temp(1)*sinf(euler_current_w(0)),euler_angle_w_temp(2));
	euler_current_w(2)=atan2(cosf(euler_current_w(1))*(euler_angle_w_temp(0)*sinf(euler_current_w(0))-euler_angle_w_temp(1)*cosf(euler_current_w(0))),euler_angle_w_temp(2));

	a_log_att.vrate_x=(euler_current_v(2)-euler_last_v(2))/dt;
	a_log_att.vrate_y=(euler_current_v(1)-euler_last_v(1))/dt;
	a_log_att.vrate_z=(euler_current_v(0)-euler_last_v(0))/dt;
	a_log_att.uvwrate_x=(euler_current_w(2)-euler_last_w(2))/dt;
	a_log_att.uvwrate_y=(euler_current_w(1)-euler_last_w(1))/dt;
	a_log_att.uvwrate_z=(euler_current_w(0)-euler_last_w(0))/dt;
	a_log_att.vrateerr_x=a_log_att.vratesp_x-a_log_att.vrate_x;
	a_log_att.vrateerr_y=a_log_att.vratesp_y-a_log_att.vrate_y;
	a_log_att.vrateerr_z=a_log_att.vratesp_z-a_log_att.vrate_z;
	a_log_att.uvwrateerr_x=a_log_att.uvwratesp_x-a_log_att.uvwrate_x;
	a_log_att.uvwrateerr_y=a_log_att.uvwratesp_y-a_log_att.uvwrate_y;
	a_log_att.uvwrateerr_z=a_log_att.uvwratesp_z-a_log_att.uvwrate_z;

	euler_last_v=euler_current_v;
	euler_last_w=euler_current_w;


	pos_current_xyz(0)=local_pos.x;
	pos_current_xyz(1)=local_pos.y;
	pos_current_xyz(2)=local_pos.z;
	vel_current_xyz(0)=local_pos.vx;
	vel_current_xyz(1)=local_pos.vy;
	vel_current_xyz(2)=local_pos.vz;
	a_log_att.pos_x=pos_current_xyz(0);
	a_log_att.pos_y=pos_current_xyz(1);
	a_log_att.pos_z=pos_current_xyz(2);

	if(euler_current_w(0)>(angle_n*2*pi))
	{
		a_log_att.rotateangle_nominal=euler_current_w(0)-angle_n*2*pi;
		angle_n++;
	}
	else
	{
		a_log_att.rotateangle_nominal=euler_current_w(0)-(angle_n-1)*2*pi;
	}
	
	if(pos_current_xyz(1)>=0)
	{
		a_log_att.rotateangle_current=atan2f(pos_current_xyz(1),pos_current_xyz(0));
	}
	if(pos_current_xyz(1)<0)
	{
		a_log_att.rotateangle_current=atan2f(pos_current_xyz(1),pos_current_xyz(0))+2*pi;
	}
	
	Rxyz_uvw(0,0)=cosf(-euler_current_w(0));
	Rxyz_uvw(0,1)=-sinf(-euler_current_w(0));
	Rxyz_uvw(0,2)=0;
	Rxyz_uvw(1,0)=sinf(-euler_current_w(0));
	Rxyz_uvw(1,1)=cosf(-euler_current_w(0));
	Rxyz_uvw(1,2)=0;
	Rxyz_uvw(2,0)=0;
	Rxyz_uvw(2,1)=0;
	Rxyz_uvw(2,2)=1;
	
	Rvxyz_vuvw(0,0)=-anglerate_nominal*(-sinf(-euler_current_w(0)));
	Rvxyz_vuvw(0,1)=-anglerate_nominal*(-cosf(-euler_current_w(0)));
	Rvxyz_vuvw(0,2)=0;
	Rvxyz_vuvw(1,0)=-anglerate_nominal*(cosf(-euler_current_w(0)));
	Rvxyz_vuvw(1,1)=-anglerate_nominal*(-sinf(-euler_current_w(0)));
	Rvxyz_vuvw(1,2)=0;
	Rvxyz_vuvw(2,0)=0;
	Rvxyz_vuvw(2,1)=0;
	Rvxyz_vuvw(2,2)=0;
	
	pos_current_uvw=Rxyz_uvw*pos_current_xyz;
	vel_current_uvw=Rvxyz_vuvw*pos_current_xyz+Rxyz_uvw*vel_current_xyz;


	x_nominal(0)=r_nominal;
	x_nominal(1)=0;
	x_nominal(2)=heiht_nominal;
	x_nominal(3)=0;
	x_nominal(4)=0;
	x_nominal(5)=0;

	x_nominal(6)=pi/2;
	// x_nominal(6)=0;
	x_nominal(7)=atan2(anglerate_nominal*anglerate_nominal*r_nominal,g_nominal);
	x_nominal(8)=0;

	v_nominal(0)=a_nominal;
	v_nominal(1)=0;
	v_nominal(2)=0;
	v_nominal(3)=0;

	x_current(0)=pos_current_uvw(0);
	x_current(1)=pos_current_uvw(1);
	x_current(2)=pos_current_uvw(2);
	x_current(3)=vel_current_uvw(0);
	x_current(4)=vel_current_uvw(1);
	x_current(5)=vel_current_uvw(2);
	x_current(6)=euler_current_v(0);
	x_current(7)=euler_current_w(1);
	x_current(8)=euler_current_w(2);

	// double t_current2=(double)t_current;
	// double posx=(double)pos_current_xyz(0);
	// PX4_WARN("tcurrent %f", t_current2);
	// PX4_WARN("posx %f", posx);
	// double posu=(double)pos_current_uvw(0);
	// double posv=(double)pos_current_uvw(1);
	// double posw=(double)pos_current_uvw(2);
	// PX4_WARN("posu %f", posu);
	// PX4_WARN("posv %f", posv);
	// PX4_WARN("posw %f", posw);

	// x_current(0)=pos_current_xyz(0);
	// x_current(1)=pos_current_xyz(1);
	// x_current(2)=pos_current_xyz(2);
	// x_current(3)=vel_current_xyz(0);
	// x_current(4)=vel_current_xyz(1);
	// x_current(5)=vel_current_xyz(2);
	// x_current(6)=euler_current_v(0);
	// x_current(7)=euler_current_v(1);
	// x_current(8)=euler_current_v(2);

	// double z_pos=(double) (pos_current_xyz(2)-pos_current_uvw(2));
	// double err_out=(double) (euler_current_v(1)-euler_current_w(1));

	// PX4_WARN("err_out %f", err_out);


	///******Circuit K r=5 w=2*pi/10  **********//
	// static float k_lqr_temp[4][9]=
	// 		{-0.1089,0.0944,-3.1598,-0.1627,0.0789,-31.6990,0.0000,-1.6834,0.3324,
	// 		0.4140,-0.1411,-0.0122,0.5524,0.1632,-0.1284,-0.0000,-3.3772,-0.0094,
	// 	   -0.1406,-0.4152,-0.0030,0.1663,-0.5528,-0.0301,-0.0000,-0.0094,-3.3393,
	// 		0.0000,0.0000,0.0000,-0.0000,0.0000,0.0000,-0.3162,-0.0000,-0.0000};


// ///******Circuit K r=10 w=2*pi/10   better R(2-4,2-4)=10 **********//
	// static float k_lqr_temp[4][9]=
	// 		{-0.5735,0.6231,-9.7192,-0.9106,0.6254,-5.4303,0.0000,-2.7933,2.3083,
	// 		0.4000,-0.1493,-0.2254,0.5443,0.1445,-0.2494,-0.0000,-3.6821,-0.0191,
	// 	   -0.1559,-0.4095,-0.0675,0.1491,-0.5550,-0.0352,-0.0000,-0.0191,-3.5600,
	// 		0.0000,0.0000,0.0000,-0.0000,0.0000,0.0000,-1.0000,-0.0000,-0.0000};

// ///******Circuit K r=5 w=2*pi/100   better R(2-4,2-4)=10 R(1)=0.1 Q(3 7 8 9)=10 **********//
	// static float k_lqr_temp[4][9]=
	// 		{-0.0034,0.0003,-10.0000,-0.0040,0.0003,-5.4772,0.0000,-0.0127,0.0012,
	// 		0.3172,-0.0160,-0.0011,0.5703,0.0160,-0.0013,-0.0000,-3.4896,-0.0000,
	// 	   -0.0160,-0.3172,-0.0000,0.0160,-0.5703,-0.0000,-0.0000,-0.000,-3.4896,
	// 		0.0000,0.0000,0.0000,-0.0000,0.0000,0.0000,-1.0000,-0.0000,-0.0000};

// ///******Circuit K r=5 w=2*pi/100    R(2-4,2-4)=10 R(1)=0.1 Q(3 7 8 9)=10 **********//
	// static float k_lqr_temp[4][9]=
	// 		{-0.2718,0.3296,-9.9296,-0.4473,0.3493,-5.4704,0.0000,-1.4616,1.1944,
	// 		0.4122,-0.1569,-0.1122,0.5635,0.1555,-0.1284,-0.0000,-3.5398,-0.0055,
	// 	   -0.1585,-0.4146,-0.0379,0.1568,-0.5660,-0.0194,-0.0000,-0.0055,-3.5073,
	// 		0.0000,0.0000,0.0000,-0.0000,0.0000,0.0000,-1.0000,-0.0000,-0.0000};

///******Circuit K   **********//

	// ///******Circuit K r=3 w=2*pi/10   R(2-4,2-4)=10 R(1,1)=0.1 Q(3 7 8 9)**********//
	// static float k_lqr_temp[4][9]=
	// 		{-0.1608,0.2002,-9.9747,-0.2669,0.2149,-5.4752,0.0000,-0.8856,0.7221,
	// 		0.4150,-0.1586,-0.0672,0.5678,0.1581,-0.0775,-0.0000,-3.5079,-0.0020,
	// 	   -0.1592,-0.4158,-0.0233,0.1586,-0.5687,-0.0119,-0.0000,-0.0020,-3.4960,
	// 		0.0000,0.0000,0.0000,-0.0000,0.0000,0.0000,-1.0000,-0.0000,-0.0000};

///******Hover K   **********//

	// ////hover stable
	// static float k_lqr_temp[4][9]=
	// 		{0,0,-3.1623,0,0,-31.7226,0,0,0.0000,
	// 		0.3162,0,0,0.5613,0,0,0,-3.3318,0,
	// 		0,-0.3162,0,0,-0.5613,0,0,0,-3.3318,
	// 		0,0,0,0,0,0,-0.3162,0,0};

	///////******OK
	// static float k_lqr_temp[4][9]=
	// 		{0,0,-31.6228,0,0,-32.6074,0,0,0.0000,
	// 		1,0,0,1.4518,0,0,0,-5.4272,0,
	// 		0,-1,0,0,-1.4518,0,0,0,-5.4272,
	// 		0,0,0,0,0,0,-1,0,0};

// //// OK test PID
	// static float k_lqr_temp[4][9]=
	// 		{0,0,-1,0,0,-1.7321,0,0,0.0000,
	// 		1,0,0,1.4518,0,0,0,-5.4272,0,
	// 		0,-1,0,0,-1.4518,0,0,0,-5.4272,
	// 		0,0,0,0,0,0,-1,0,0};

	// ///******Circuit K r=5 w=2*pi/10   better R(2-4,2-4)=10 **********//
	// static float k_lqr_temp[4][9]=
	// 		{-0.2511,0.1222,-0.9776,-0.3439,-0.0509,-1.6944,0.0000,-0.0435,0.0991,
	// 		0.4067,-0.1393,-0.0660,0.5427,0.1618,-0.1137,-0.0000,-3.3446,-0.0023,
	// 	   -0.1442,-0.4140,-0.0084,0.1620,-0.5560,-0.0225,-0.0000,-0.0023,-3.3489,
	// 		0.0000,0.0000,0.0000,-0.0000,0.0000,0.0000,-0.3162,-0.0000,-0.0000};

	// // ///******Circuit K r=5 w=2*pi/5   R(2-4,2-4)=10 R(1)=0.1 Q(3 7 8 9)=10 **********//
	// static float k_lqr_temp[4][9]=
	// 		{-0.8070,2.6204,-8.7011,-2.2566,1.7373,-5.1081,0.0000,-5.7876,7.8606,
	// 		0.5793,-0.2559,-0.4664,0.4794,0.2324,-0.4605,-0.0000,-4.1629,-0.0598,
	// 	   -0.2910,-0.6182,-0.1592,0.2309,-0.5023,-0.0851,-0.0000,-0.0598,-3.6087,
	// 		0.0000,0.0000,0.0000,-0.0000,0.0000,0.0000,-1.0000,-0.0000,-0.0000};
	// ///******Circuit K r=1 w=2*pi/10   R(2-4,2-4)=10 R(1)=0.1 Q(3 7 8 9)=10 **********//
	// static float k_lqr_temp[4][9]=
	// 		{-0.0532,0.0672,-9.9972,-0.0887,0.0725,-5.4770,0.0000,-0.2966,0.2416,
	// 		0.4163,-0.1595,-0.0224,0.5700,0.1594,-0.0259,-0.0000,-3.4916,-0.0002,
	// 	   -0.1596,-0.4164,-0.0079,0.1595,-0.5701,-0.0040,-0.0000,-0.0002,-3.4903,
	// 		0.0000,0.0000,0.0000,-0.0000,0.0000,0.0000,-1.0000,-0.0000,-0.0000};
	///******Circuit K r=0.5 w=2*pi/5   better R(2-4,2-4)=10 R1=0.1 Q3789=10**********//
	// static float k_lqr_temp[4][9]=
	// 		{0.0290,0.3661,-9.9835,-0.2368,0.3303,-5.4772,0.0000,-0.8384,1.0181,
	// 		0.7155,-0.3185,-0.0487,0.5692,0.3179,-0.0543,-0.0000,-3.4978,-0.0013,
	// 	   -0.3186,-0.7155,-0.0305,0.3177,-0.5688,-0.0154,-0.0000,-0.0013,-3.4891,
	// 		0.0000,0.0000,0.0000,-0.0000,0.0000,0.0000,-1.0000,-0.0000,-0.0000};

	///******Circuit K r=1 w=2*pi/5   better R(2-4,2-4)=10 R1=0.1 Q3789=10**********//
	// static float k_lqr_temp[4][9]=
	// 		{0.0449,0.7224,-9.9347,-0.4738,0.6435,-5.4760,0.0000,-1.6528,2.0134,
	// 		0.7098,-0.3163,-0.0973,0.5659,0.3139,-0.1079,-0.0000,-3.5221,-0.0049,
	// 	   -0.3166,-0.7102,-0.0595,0.3132,-0.5645,-0.0301,-0.0000,-0.0049,-3.4881,
	// 		0.0000,0.0000,0.0000,-0.0000,0.0000,0.0000,-1.0000,-0.0000,-0.0000};

	///******Circuit K r=1 w=2*pi/3   better R(2-4,2-4)=10 R1=0.1 Q3789=10**********//
	// static float k_lqr_temp[4][9]=
	// 		{1.7975 ,   4.8058 ,  -9.3432 ,  -2.0617  ,  2.7259 ,  -5.4475 ,  -0.0000 ,  -6.0929  ,  8.5885,
 //   	        1.2869 ,  -0.4954 ,  -0.2995 ,   0.5290 ,   0.4632  , -0.3035  ,  0.0000,   -3.6959  , -0.0266,
 //      		 -0.4855,   -1.2394 ,  -0.1932 ,   0.4292 ,  -0.5063  , -0.0980 ,  -0.0000 ,  -0.0266 ,  -3.3367,
 //  			 -0.0000 ,  -0.0000 ,  -0.0000 ,  -0.0000 ,  -0.0000 ,   0.0000 ,  -1.0000 ,   0.0000  , -0.0000};

  	///******Circuit K r=1 w=2*pi/1   better R(2-4,2-4)=10 R1=0.1 Q3789=10**********//
	// static float k_lqr_temp[4][9]=
	// 		{-4.0777,   45.4948,   -2.3954,   -8.1271 ,   2.7068  , -1.2641,   -0.0000 ,  -4.7640,   42.1363,
 //   			 1.0524 ,  -0.3502 ,  -0.9686 ,   0.1164,    0.1439,   -0.6667 ,  -0.0000,   -7.4610 ,  -0.1800,
 //            -1.7976 ,  -3.9089 ,  -0.0669 ,   0.4450 ,  -0.5074 ,  -0.0436 ,   0.0000 ,  -0.1800,   -4.9489,
 //             0.0000 ,   0.0000 ,  -0.0000 ,  -0.0000,    0.0000 ,   0.0000 ,  -1.0000 ,  -0.0000 ,   0.0000};

    ///******Circuit K r=1 w=2*pi/2   better R(2-4,2-4)=10 R1=0.1 Q3789=10**********//
	// static float k_lqr_temp[4][9]=
	// 		{3.4298 ,  15.4510 ,  -7.6327 ,  -4.9667,    4.1544 ,  -4.8836 ,   0.0000,  -10.8416 ,  18.0059,
 //             1.7653,   -0.5491,   -0.6081 ,   0.3992,    0.4586,   -0.5559,  -0.0000,   -4.3152,   -0.0417,
 //            -0.6390,  -1.6323,   -0.2184 ,   0.3675,   -0.4089 ,  -0.1145,    0.0000,   -0.0417 ,  -3.0218,
 //             0.0000 ,  -0.0000,   -0.0000 ,   0.0000 ,   0.0000 ,  -0.0000 ,  -1.0000 ,  -0.0000,    0.0000
 //         	};

    ///******Circuit K r=1 w=2*pi/2   better R(2-4,2-4)=10 R1=1 Q3789=10**********//
	static float k_lqr_temp[4][9]=
			{
			-1.9458 ,   3.5597,   -2.1848 ,  -1.4958 ,  -0.0353 ,  -1.6478 ,   0.0000 ,  -2.0401,    3.4208,
    		1.3756 ,  -0.7150,   -0.7157 ,   0.4173 ,   0.3575 ,  -0.7098 ,   0.0000 ,  -4.7543 ,  -0.4111,
  			 -1.4136 ,  -2.6746,   -0.1021 ,   0.6528 ,  -0.7519 ,  -0.1719 ,  -0.0000 ,  -0.4111,   -4.5360,
  			 -0.0000 ,  -0.0000,    0.0000 ,  -0.0000,   -0.0000,    0.0000 ,  -1.0000 ,   0.0000,   -0.0000
         	};


    static float A_MAT_temp[9][9]=
			{
			 0  ,       0 ,        0,    1.0000 ,        0 ,        0,         0,         0  ,       0,
         	 0 ,        0  ,       0 ,        0  ,  1.0000 ,        0,         0 ,        0 ,        0,
         	 0  ,       0 ,        0 ,        0 ,        0 ,   1.0000 ,        0 ,        0  ,       0,
   			 9.8696 ,        0 ,        0 ,        0 ,   6.2832 ,        0,         0 ,  -9.8000 ,       0,
         	 0 ,   9.8696 ,        0 ,  -6.2832  ,       0 ,        0 ,        0 ,        0 ,  13.9086,
         	 0 ,        0 ,        0 ,        0  ,       0 ,        0 ,        0 ,   9.8696 ,        0,
         	 0 ,        0 ,        0 ,        0 ,        0 ,        0 ,        0 ,        0 ,        0,
         	 0 ,        0 ,        0  ,       0 ,        0  ,       0 ,        0 ,        0 ,        0,
         	 0 ,        0 ,        0  ,       0 ,        0 ,       0 ,       0 ,        0 ,        0
         	};

    static float B_MAT_temp[9][4]=
			{
			0 ,        0 ,        0 ,        0,
         	0 ,        0,        0 ,        0,
         	0 ,        0 ,        0 ,        0,
    		0.7096 ,        0 ,        0 ,        0,
         	0  ,       0  ,       0   ,      0,
    		0.7046 ,        0 ,        0 ,        0,
         	0 ,        0 ,        0 ,   1.0000,
         	0 ,   1.0000 ,        0 ,        0,
         	0  ,       0 ,   1.0000 ,        0
         	};

	
	// PX4_WARN("currentknum!!!!!!!!!!!!!!!!!!!!!!! %i", flag_wrate);
	A_MAT.set(A_MAT_temp);
	B_MAT.set(B_MAT_temp);
	k_lqr.set(k_lqr_temp);

	// if(flag_ILC==1)
	// {
	// 	if(flag_first_cal==1)
	// 	{
	// 		x_err_nominal=x_current-x_nominal;
	// 		flag_first_cal=0;
	// 	}
	// 	else
	// 	{
	// 		x_err_nominal=x_err_nominal+x_err_dot*dt;
	// 	}
	// }

	////****calculate input   LQR and compensation
	if(flag_pos_poll==1)
	{
		if(ILC_enable)
		{
			v_input=k_lqr*(x_current-x_nominal)+v_nominal+v_compensation;
		}
		else
		{
			v_input=k_lqr*(x_current-x_nominal)+v_nominal;
		}
		// flag_dt = (hrt_absolute_time() - last_t) / 1000000.0f;
		// last_t = hrt_absolute_time();
		// a_log_att.dt=flag_dt;
		// a_log_att.hz_attitude=1/flag_dt;
	}
	if(flag_ILC==1)
	{
		x_err_dot=A_MAT*(x_current-x_nominal)+B_MAT*(v_input-v_nominal);
		a_log_att.err_u_dot=x_err_dot(0);
		a_log_att.err_v_dot=x_err_dot(1);
	}
	
	// if(_flag_acc_sensor==1)
	// {
	// 	flag_dt_sensor = (hrt_absolute_time() - last_t_sensor) / 1000000.0f;
	// 	last_t_sensor = hrt_absolute_time();
	// 	a_log_att.dt_sensor=flag_dt_sensor;
	// 	a_log_att.hz_sensor=1/flag_dt_sensor;
	// 	double flag_dt_sensor2=(double)flag_dt_sensor;
	// 	PX4_WARN("flag_dt_sensor %f", flag_dt_sensor2);
	// }
	// a_log_att.flag_sensor=_flag_acc_sensor;
	// a_log_att.flag_controlstate=_flag_acc_controlstate;
	// if(_flag_acc_controlstate==1)
	// {
	// 	// flag_dt_controlstate = (hrt_absolute_time() - last_t_controlstate) / 1000000.0f;
	// 	// last_t_controlstate = hrt_absolute_time();
	// 	// a_log_att.dt_controlstate=flag_dt_controlstate;
	// 	// a_log_att.hz_controlstate=1/flag_dt_controlstate;
	// 	flag_dt_sensor = (hrt_absolute_time() - last_t_sensor) / 1000000.0f;
	// 	last_t_sensor = hrt_absolute_time();
	// 	a_log_att.dt_sensor=flag_dt_sensor;
	// 	a_log_att.hz_sensor=1/flag_dt_sensor;
	// 	// double flag_dt_controlstate2=(double)flag_dt_controlstate;
	// 	// PX4_WARN("flag_dt_controlstate %f", flag_dt_controlstate2);
	// }

	eulerrate_next_w(0)=anglerate_nominal;
	eulerrate_next_w(1)=v_input(1);
	eulerrate_next_w(2)=v_input(2);

	// euler_next_w(0)=anglerate_nominal*t_next;
	euler_next_w(0)=euler_current_w(0)+anglerate_nominal*dt;
	euler_next_w(1)=euler_current_w(1)+eulerrate_next_w(1)*dt;
	euler_next_w(2)=euler_current_w(2)+eulerrate_next_w(2)*dt;


	euler_angle_v_temp(0)=sinf(euler_next_w(0))*sinf(euler_next_w(2))+cosf(euler_next_w(0))*sinf(euler_next_w(1))*cosf(euler_next_w(2));
	euler_angle_v_temp(1)=-cosf(euler_next_w(0))*sinf(euler_next_w(2))+sinf(euler_next_w(0))*sinf(euler_next_w(1))*cosf(euler_next_w(2));
	euler_angle_v_temp(2)=cosf(euler_next_w(1))*cosf(euler_next_w(2));
	
	euler_next_v(0)=euler_current_v(0)+v_input(3)*dt;
	euler_next_v(1)=atan2(euler_angle_v_temp(0)*cosf(euler_next_v(0))+euler_angle_v_temp(1)*sinf(euler_next_v(0)),euler_angle_v_temp(2));
	euler_next_v(2)=atan2(cosf(euler_next_v(1))*(euler_angle_v_temp(0)*sinf(euler_next_v(0))-euler_angle_v_temp(1)*cosf(euler_next_v(0))),euler_angle_v_temp(2));


	eulerrate_next_v(0)=v_input(3);
	eulerrate_next_v(1)=(euler_next_v(1)-euler_current_v(1))/dt;
	eulerrate_next_v(2)=(euler_next_v(2)-euler_current_v(2))/dt;


	euler_input(0)=eulerrate_next_v(2);
	euler_input(1)=eulerrate_next_v(1);
	euler_input(2)=eulerrate_next_v(0);
	

	Reuler_body(0,0)=1;
	Reuler_body(0,1)=0;
	Reuler_body(0,2)=-sinf(euler_next_v(1));
	Reuler_body(1,0)=0;
	Reuler_body(1,1)=cosf(euler_next_v(2));
	Reuler_body(1,2)=cosf(euler_next_v(1))*sinf(euler_next_v(2));
	Reuler_body(2,0)=0;
	Reuler_body(2,1)=-sinf(euler_next_v(2));
	Reuler_body(2,2)=cosf(euler_next_v(1))*cosf(euler_next_v(2));


	// if(flag_lqr_enable==0)
	// {
	// 	a_input=10;
	// }
	// else
	// {
	a_input=-v_input(0);
	// }
	if(a_input<6)
	{
		a_input=6;
	}
	anglerate_sp=Reuler_body*euler_input;
	a_current=_ctrl_state.z_acc;
	// a_current=sensor_data.accelerometer_m_s2[2];
	a_log_att.a_current_before=-sensor_data.accelerometer_m_s2[2];
	a_err=a_input-(-a_current);
	x_err=x_current-x_nominal;
	y_err(0)=x_err(0);
	y_err(1)=x_err(1);
	y_err(2)=x_err(2);
	y_err(3)=x_err(6);

	if(flag_t_x==0)
	{
		t_x=-T_n/2;
		flag_t_x=1;
		// ak.zero();
		// bk.zero();
		// a0.zero();
		// ak_integral.zero();
		// bk_integral.zero();
		// a0_integral.zero();
	}
	// if(flag_t_v==0)
	// {
	// 	t_v=-T_n/2;
	// 	flag_t_v=1;
	// }
	/////k_p=0.05  k_i=0.01  k_d=0.01
	float k_p=0.06;   //0.06
	float k_i=0.006;  //0.006
	float k_d=0.03;   //0.03
	float k_hover=0.49;
	// static float XINV_temp[8][18]=
	// 		{
	// 		0    ,     0 ,        0 ,  -0.1241  ,       0 ,  -0.6159  ,       0  ,       0 ,        0 ,   0.1731 ,  -0.1222 ,   0.9776 ,    	0.3439 ,  -0.1972 ,   1.6944 ,  -0.0000  ,  0.0435 ,  -0.0991,
 //         	0    ,     0  ,       0 ,        0  ,       0 ,        0 ,        0 ,  -0.6283 ,        0 ,  -0.4067 ,   0.1393,   0.0660	,	    -0.5427 ,  -0.1618 ,   0.1137 ,   0.0000 ,   3.3446 ,   0.0023,
 //         	0    ,     0   ,      0  ,       0   ,      0  ,       0 ,        0  ,       0 ,  -0.6283 ,   0.1442 ,   0.4140 ,   0.0084	,	    -0.1620 ,   0.5560 ,   0.0225  ,  0.0000  ,  0.0023  ,  3.3489,
 //        	0    ,     0  ,       0   ,      0   ,      0  ,       0 ,  -0.6283  ,      0 ,        0 ,  -0.0000 ,   0.0000 ,  -0.0000	,	    -0.0000 ,   0.0000 ,  -0.0000 ,   0.3162 ,   0.0000 ,   0.0000,
 //    		0.1731  , -0.1222 ,   0.9776,    0.3439,   -0.1972 ,   1.6944 ,  -0.0000 ,   0.0435 ,  -0.0991 ,        0  ,       0  ,       0,	0.1241 ,        0 ,  0.6159  ,       0 ,        0 ,        0,
 //           -0.4067 ,   0.1393,    0.0660 ,  -0.5427 ,  -0.1618 ,   0.1137 ,   0.0000 ,   3.3446 ,   0.0023 ,        0  ,       0  ,       0,	     0  ,       0  ,       0  ,       0 ,  0.6283 ,        0,
 //            0.1442 ,   0.4140 ,   0.0084 ,  -0.1620 ,   0.5560,    0.0225 ,   0.0000  ,  0.0023 ,   3.3489 ,        0  ,       0  ,       0,	 0   ,      0    ,     0   ,      0     ,    0 ,  0.6283,
 //           -0.0000 ,   0.0000 ,  -0.0000 ,  -0.0000 ,   0.0000 ,  -0.0000 ,   0.3162  ,  0.0000 ,   0.0000  ,       0   ,      0  ,       0 ,	  0   ,      0 ,        0  , 0.6283   ,      0 ,        0

	// 		};
	/////**************r=0.5 ????*****************//
	// static float JINV_temp[8][18]=
	// {
	// -0.2082 ,   0.1205 ,  -1.0647 ,  -0.0000 ,   0.0000 ,   0.0000 ,   0.0000  ,       0  ,       0 ,   0.0985 ,  -0.1144 ,   0.5902 , 0 ,        0 ,        0  ,       0 ,        0 ,        0,
 //    0.2922 ,  -0.1573,   -0.0666,   -0.0000 ,  -0.0000 ,  -0.0000 ,   0.0000 ,        0  ,       0 ,  -0.1478 ,   0.0905 ,   0.0399 ,  0 ,        0 ,        0  ,       0 ,        0 ,        0,
 //   -0.1627 ,  -0.2999 ,  -0.0142 ,  -0.0000 ,   0.0000 ,   0.0000 ,   0.0000  ,       0  ,       0  ,  0.0948 ,   0.1495 ,   0.0084 ,   0 ,        0 ,        0 ,        0 ,        0 ,        0,
 //    -0.0000 ,   0.0000,   -0.0000 ,   0.0000 ,  -0.0000 ,   0.0000 ,  -0.0000  ,       0  ,       0 ,  -0.0000 ,   0.0000 ,  -0.0000 ,   0 ,        0,         0 ,        0 ,        0 ,        0,
 //    0.0985 ,  -0.1144 ,   0.5902 ,  -0.0000 ,   0.0000 ,  -0.0000  ,  0.0000  ,       0  ,       0  ,  0.2082 ,  -0.1205 ,   1.0647  ,  0  ,       0  ,       0 ,        0 ,        0 ,        0,
 //   -0.1478 ,   0.0905 ,   0.0399 ,   0.0000 ,  -0.0000 ,  -0.0000 ,  -0.0000  ,       0  ,       0 ,  -0.2922 ,   0.1573 ,   0.0666 ,   0  ,       0 ,        0 ,        0 ,        0  ,       0,
 //    0.0948 ,   0.1495  ,  0.0084  , -0.0000  , -0.0000  , -0.0000 ,   0.0000  ,       0  ,       0 ,   0.1627  ,  0.2999 ,   0.0142  ,  0  ,       0 ,        0 ,        0 ,        0 ,        0,
 //    -0.0000  ,  0.0000 ,  -0.0000 ,  -0.0000 ,   0.0000  ,  0.0000 ,   0.0000  ,       0  ,       0  ,  0.0000 ,  -0.0000 ,   0.0000 ,   0  ,       0  ,       0  ,       0  ,       0 ,        0
	// };
	// static float RINV_temp[4][9]=
	// {
	// 0.1731 ,  -0.1222 ,   0.9776,    0.3439,   -0.1972 ,   1.6944,   -0.0000,   0.0435,   -0.0991,
 //   -0.4067 ,   0.1393 ,   0.0660 ,  -0.5427,   -0.1618 ,   0.1137,    0.0000,    3.3446,    0.0023,
 //    0.1442 ,   0.4140 ,   0.0084 ,  -0.1620 ,   0.5560 ,   0.0225 ,   0.0000,    0.0023 ,   3.3489,
 //   -0.0000 ,   0.0000 ,  -0.0000 ,  -0.0000 ,   0.0000 ,  -0.0000,    0.3162 ,   0.0000 ,   0.0000
	// };
	/////**************r=1*****************//
	// static float JINV_temp[8][18]=
	// {
	// 2.6995 ,   4.7493 , -11.4093 ,  -0.0000 ,   0.0000 ,   0.0000 ,   0.0000   ,      0  ,       0  , -0.8376 ,   2.2118 ,   4.3226, 0 ,        0 ,        0  ,       0 ,        0 ,        0,
 //    -0.4757,   -1.7863 ,  -0.2862 ,   0.0000 ,  -0.0000 ,  -0.0000 ,  -0.0000  ,       0  ,       0 ,   1.4695 ,  -1.0883 ,  -0.3174,  0 ,        0 ,        0  ,       0 ,        0 ,        0,
 //   -1.8274 ,   0.6311 ,  -0.2053 ,  -0.0000 ,  -0.0000 ,  -0.0000 ,   0.0000   ,      0  ,       0  , -1.2060 ,  -1.4870 ,   0.1888,   0 ,        0 ,        0 ,        0 ,        0 ,        0,
 //    -0.0000 ,   0.0000,    0.0000,   -0.0000 ,  -0.0000  , -0.0000,    0.0000  ,       0 ,        0 ,  -0.0000 ,  -0.0000 ,   0.0000 ,   0 ,        0,         0 ,        0 ,        0 ,        0,
 //    -0.8376,    2.2118,    4.3226 ,  -0.0000 ,  -0.0000 ,   0.0000,    0.0000  ,       0  ,       0,   -2.6995 ,  -4.7493 ,  11.4093,  0  ,       0  ,       0 ,        0 ,        0 ,        0,
 //    1.4695  , -1.0883  , -0.3174  , -0.0000  ,  0.0000  ,  0.0000  , -0.0000   ,      0  ,       0 ,   0.4757 ,   1.7863  ,  0.2862,   0  ,       0 ,        0 ,        0 ,        0  ,       0,
 //    -1.2060 ,  -1.4870 ,   0.1888,    0.0000 ,  -0.0000 ,  -0.0000,   -0.0000  ,       0 ,        0 ,   1.8274 ,  -0.6311 ,   0.2053 ,  0  ,       0 ,        0 ,        0 ,        0 ,        0,
 //    -0.0000  ,  0.0000 ,  -0.0000 ,  -0.0000 ,   0.0000  ,  0.0000 ,   0.0000  ,       0  ,       0  ,  0.0000 ,  -0.0000 ,   0.0000 ,   0  ,       0  ,       0  ,       0  ,       0 ,        0
	// };
	// static float RINV_temp[4][9]=
	// {
	// -3.5895,   -4.8058 ,   9.3432,    2.0617 ,  -4.4372 ,   5.4475 ,   0.0000,    6.0929 ,  -8.5885,
 //   -1.2869 ,   0.4954  ,  0.2995,   -0.5290,   -0.4632 ,   0.3035 ,  -0.0000 ,   3.6959 ,   0.0266,
 //    0.4855 ,   1.2394  ,  0.1932,   -0.4292,    0.5063 ,   0.0980 ,   0.0000 ,   0.0266 ,   3.3367,
 //    0.0000 ,   0.0000  ,  0.0000 ,   0.0000 ,   0.0000 ,  -0.0000 ,   1.0000 ,  -0.0000 ,   0.0000,
	// };
	// static float HINV_temp[4][9]=
	// {
	// -1.3175,   -1.2970 ,   9.3432,  0.0000 ,   0.0000  ,  0.0000 ,   0.0000 ,   0.0000 ,  -0.0000 ,
 //   	0.0913 ,   0.4845  ,  0.2995,   0.0000 ,   0.0000  ,  0.0000 ,   0.0000 ,   0.0000 ,  -0.0000 ,
 //    0.4954 ,   -0.1238  ,  0.1932,   0.0000 ,   0.0000  ,  0.0000 ,   0.0000 ,   0.0000 ,  -0.0000 ,
 //    0.0000 ,   0.0000  ,  0.0000 ,  0.0000 ,   0.0000  ,  0.0000 ,   0.0000 ,   0.0000 ,  -0.0000 ,
	// };
	///******Circuit K r=1 w=2*pi/3   better R(2-4,2-4)=10 R1=0.1 Q3789=10**********//
	// static float JINV_temp_1[8][6]=
	// {
	// 2.6995 ,   4.7493 , -11.4093 ,  -0.8376 ,   2.2118 ,   4.3226,
 //    -0.4757,   -1.7863 ,  -0.2862 ,    1.4695 ,  -1.0883 ,  -0.3174, 
 //   -1.8274 ,   0.6311 ,  -0.2053 , -1.2060 ,  -1.4870 ,   0.1888,  
 //    -0.0000 ,   0.0000,    0.0000,   -0.0000 ,   0.0000 ,   0 ,       
 //    -0.8376,    2.2118,    4.3226 ,    -2.6995 ,  -4.7493 ,  11.4093, 
 //    1.4695  , -1.0883  , -0.3174  ,   0.4757 ,   1.7863  ,  0.2862,   
 //    -1.2060 ,  -1.4870 ,   0.1888,    1.8274 ,  -0.6311 ,   0.2053 , 
 //    -0.0000  ,  0.0000 ,  -0.0000 ,  -0.0000 ,   0.0000  ,  0.0000
	// };
	// static float JINV_temp_2[8][6]=
	// {
	// 5.3990 ,   9.4986,  -22.8186,    0.6022 ,  12.7381,  -10.7395,
 //   -5.6373,   -3.5726,    1.5251,   5.6041,   -5.8068 ,  -2.1680,
 //   -3.6547,    6.3962,   -0.4106 ,  -6.3102 ,  -5.5765 ,   0.1755,
 //   -0.0000 ,   0.0000 ,  -0.0000 ,  -0.0000 ,  -0.0000,    0.0000,
 //    0.6022,  12.7381,  -10.7395,   -5.3990,   -9.4986 ,  22.8186,
 //    5.6041,   -5.8068,   -2.1680,    5.6373,    3.5726,   -1.5251,
 //   -6.3102 ,  -5.5765,    0.1755 ,   3.6547,   -6.3962,    0.4106,
 //   -0.0000,   -0.0000 ,   0.0000 ,   0.0000,   -0.0000,    0.0000
	// };
	// static float JINV_temp_3[8][6]=
	// {
	// 8.0985,   14.2479,  -34.2278,    3.0019,   30.2820 , -35.8428,
 //  -20.1707 ,  -5.3589 ,   7.5311,   12.4951 , -13.6709,   -5.2524,
 //   -5.4821,  22.4291 ,  -0.6160 , -14.8172 , -12.3924 ,   0.1534,
 //   -0.0000 ,   0.0000,   -0.0000,    0.0000 ,  -0.0000 ,   0.0000,
 //    3.0019 ,  30.2820,  -35.8428,   -8.0985,  -14.2479 ,  34.2278,
 //   12.4951 , -13.6709,  -5.2524,   20.1707,    5.3589,   -7.5311,
 //  -14.8172 , -12.3924 ,   0.1534,    5.4821,  -22.4291,    0.6160,
 //   -0.0000 ,  -0.0000,    0.0000,    0.0000,   -0.0000 ,   0.0000

	// };
	// static float HINV_temp[4][3]=
	// {
	// -1.3175,   -1.2970 ,   9.3432, 
 //   	0.0913 ,   0.4845  ,  0.2995,  
 //    0.4954 ,   -0.1238  ,  0.1932, 
 //    0.0000 ,   0.0000  ,  0.0000 
	// };

	///******Circuit K r=1 w=2*pi/1   better R(2-4,2-4)=10 R1=0.1 Q3789=10**********//
	// static float JINV_temp_1[8][6]=
	// {
	// 30.7263 ,  91.4103,   -7.9426,  -70.3254 ,  36.2955,  -11.6035,
 //   -2.5562 ,  -2.5848,    1.7293 ,   2.4367,   -2.9376 ,  -6.0593,
 //   -6.8104 ,   8.9240,   -0.2737 , -10.3144,   -5.6973,   -0.1027,
 //   -0.0000 ,   0.0000,   -0.0000,   -0.0000 ,  -0.0000,    0.0000,
 //  -70.3254 ,  36.2955,  -11.6035,  -30.7263 , -91.4103 ,   7.9426,
 //    2.4367 ,  -2.9376 ,  -6.0593 ,   2.5562,    2.5848,  -1.7293,
 //  -10.3144,   -5.6973 ,  -0.1027 ,   6.8104 ,  -8.9240 ,   0.2737,
 //   -0.0000,   -0.0000 ,   0.0000,    0.0000 ,  -0.0000 ,   0.0000
	// };
	// static float JINV_temp_2[8][6]=
	// {
	// 61.4527 , 182.8207,  -15.8851, -181.9301,  158.9808,  -53.6000,
 //  -13.9275 ,  -5.1697,   38.9694,    7.6705 , -12.2767,  -27.1429,
 //  -13.6209,   54.4366 ,  -0.5474 , -46.7768 , -20.1067,   -0.6112,
 //   -0.0000,    0.0000 ,  -0.0000,    0.0000 ,  -0.0000,    0.0000,
 // -181.9301 , 158.9808 , -53.6000,  -61.4527 ,-182.8207,   15.8851,
 //    7.6705 , -12.2767 , -27.1429 ,  13.9275,    5.1697,  -38.9694,
 //  -46.7768 , -20.1067 ,  -0.6112 ,  13.6209 , -54.4366,    0.5474,
 //   -0.0000 ,  -0.0000 ,   0.0000,    0.0000 ,  -0.0000,    0.0000
	// };
	// static float JINV_temp_3[8][6]=
	// {
	// 92.1790,  274.2310,  -23.8277, -367.9380,  363.4564, -123.5942,
 //  -42.9291 ,  -7.7545 , 147.2313,   16.3934,  -27.8419,  -62.2824,
 //  -20.4313 , 173.1264,   -0.8212, -107.5475,  -44.1222,   -1.4589,
 //   -0.0000,    0.0000,   -0.0000,  -0.0000,  -0.0000 ,   0.0000,
 // -367.9380 , 363.4564, -123.5942,  -92.1790, -274.2310,  23.8277,
 //   16.3934,  -27.8419,  -62.2824,   42.9291,    7.7545,-147.2313,
 // -107.5475 , -44.1222,   -1.4589,   20.4313, -173.1264,    0.8212,
 //   -0.0000,   -0.0000,    0.0000 ,   0.0000,   -0.0000,    0.0000
	// };
	// static float HINV_temp[4][3]=
	// {
	// -33.1239,   -4.5997 ,   2.3954,
 //    0.6922 ,   0.1755,   0.9686,
 //    1.8397 ,  -0.8942 ,   0.0669,
 //    0.0000 ,   0.0000,   -0.0000
	// };

	///******Circuit K r=1 w=2*pi/2   better R(2-4,2-4)=10 R1=0.1 Q3789=10**********//
	// static float JINV_temp_1[8][6]=
	// {
	//  9.9507 ,  16.2172 , -15.3421 ,  -6.5955,   10.1031,   -4.7806,
 //   -1.9465,   -2.8745 ,  -0.1646 ,   2.5498,   -2.6516 ,  -1.5648,
 //   -3.1342 ,   3.1322 ,  -0.3598 ,  -3.7779,   -2.6563,   0.1974,
 //    0.0000 ,  -0.0000 ,   0.0000 ,   0.0000,    0.0000 ,  -0.0000,
 //   -6.5955 ,  10.1031 ,  -4.7806 ,  -9.9507 , -16.2172 ,  15.3421,
 //    2.5498,   -2.6516 ,  -1.5648,    1.9465,    2.8745 ,   0.1646,
 //   -3.7779,   -2.6563 ,   0.1974,    3.1342 ,  -3.1322 ,   0.3598,
 //    0.0000,    0.0000 ,  -0.0000 ,  -0.0000 ,   0.0000 ,   0.0000
	// };
	// static float JINV_temp_2[8][6]=
	// {
	// 19.9015 ,  32.4344,  -30.6843,  -11.3441 ,  48.4344,  -42.0205,
 //  -13.3175 ,  -5.7491,    9.1623 ,   9.0225,  -12.1648 ,  -8.0835,
 //   -6.2683 ,  19.6401 ,  -0.7196 , -17.0911,   -9.0892 ,   0.1344,
 //    0.0000 ,  -0.0000 ,   0.0000 ,   0.0000,    0.0000 ,  -0.0000,
 //  -11.3441 ,  48.4344 , -42.0205 , -19.9015,  -32.4344 ,  30.6843,
 //    9.0225 , -12.1648 ,  -8.0835 ,  13.3175,    5.7491 ,  -9.1623,
 //  -17.0911 ,  -9.0892 ,   0.1344 ,   6.2683,  -19.6401 ,   0.7196,
 //    0.0000 ,   0.0000 ,  -0.0000 ,  -0.0000 ,   0.0000 ,  -0.0000
	// };
	// static float JINV_temp_3[8][6]=
	// {
	// 29.8522,   48.6516,  -46.0264,  -19.2584 , 112.3199, -104.0869,
 //  -43.5376 ,  -8.6236 ,  37.4722 ,  19.8103,  -28.0203 , -18.9479,
 //   -9.4025,   62.8995,   -1.0794 , -39.2798,  -19.8106 ,   0.0295,
 //    0.0000,   -0.0000,    0.0000 ,   0.0000 ,   0.0000 ,  -0.0000,
 //  -19.2584 , 112.3199 ,-104.0869 , -29.8522,  -48.6516 ,  46.0264,
 //   19.8103 , -28.0203,  -18.9479 ,  43.5376,    8.6236 , -37.4722,
 //  -39.2798 , -19.8106 ,   0.0295 ,   9.4025,  -62.8995 ,   1.0794,
 //    0.0000 ,   0.0000 ,  -0.0000 ,  -0.0000 ,   0.0000 ,  -0.0000
	// };
	// static float HINV_temp[4][3]=
	// {
	// -5.0127 ,  -2.6740,    7.6327,
 //    0.3922 ,   0.5195,    0.6081,
 //    0.6598,   -0.5120,    0.2184,
 //    0.0000 ,   0.0000,   -0.0000
	// };

	///******Circuit K r=1 w=2*pi/2   better R(2-4,2-4)=10 R1=1 Q3789=10**********//
	static float JINV_temp_1[8][8]=
	{
	 0.1557 ,  11.8561 ,  -5.1766 ,0.0000, -10.0212 ,   1.2952 ,  -5.7966,0.0000,
   -2.4140,   -3.6310 ,  -0.6479  ,0.0000,  3.3786 ,  -3.0099 ,  -1.6783,0.0000,
   -4.3867 ,   1.6855 ,  -0.5401 , 0.0000, -2.6339 ,  -3.7630 ,  -0.1049,0.0000,
    0.0000 ,   0.0000 ,   0.0000 , -3.1416, -0.0000 ,   0.0000 ,  -0.0000,1.0000,
  -10.0212 ,   1.2952 ,  -5.7966 , 0.0000, -0.1557 , -11.8561 ,   5.1766,0.0000,
    3.3786 ,  -3.0099 ,  -1.6783 ,0.0000,   2.4140 ,   3.6310 ,   0.6479,0.0000,
   -2.6339 ,  -3.7630 ,  -0.1049 ,0.0000,   4.3867,   -1.6855 ,   0.5401,0.0000,
   -0.0000 ,   0.0000 ,  -0.0000 ,1.0000,   -0.0000,   -0.0000 ,  -0.0000,3.1416
	};
	static float JINV_temp_2[8][8]=
	{
	0.3113 ,  23.7123 , -10.3532 ,0.0000, -27.9717,    8.5775 , -29.7408,0.0000,
  -14.2526 ,  -7.2619 ,   8.1957 ,0.0000,  10.5100, -13.3096 ,  -8.8603,0.0000,
   -8.7735 ,  16.7467 ,  -1.0802 ,0.0000, -15.3931,  -13.4193,   -0.7258,0.0000,
    0.0000 ,  -0.0000 ,   0.0000 ,-6.2832,  -0.0000 ,   0.0000 ,  -0.0000,1.0000,
  -27.9717 ,   8.5775 , -29.7408 ,0.0000,  -0.3113 , -23.7123 ,  10.3532,0.0000,
   10.5100 , -13.3096,   -8.8603 ,0.0000,  14.2526,    7.2619 ,  -8.1957,0.0000,
  -15.3931 , -13.4193 ,  -0.7258 , 0.0000,  8.7735 , -16.7467 ,   1.0802,0.0000,
   -0.0000,    0.0000 ,  -0.0000 ,1.0000,  -0.0000 ,   0.0000 ,  -0.0000,6.2832
	};
	static float JINV_temp_3[8][8]=
	{
	0.4670 ,  35.5684,  -15.5299 ,0.0000, -57.8892 ,  20.7147, -69.6477,0.0000,
  -44.9403,  -10.8929 ,  36.0222 ,0.0000,  22.3955 , -30.4757 , -20.8302,0.0000,
  -13.1602 ,  58.5594 ,  -1.6204 ,0.0000, -36.6583 , -29.5131 ,  -1.7608,0.0000,
    0.0000 ,  -0.0000 ,   0.0000 , -9.4248, -0.0000 ,   0.0000 ,  -0.0000,1.0000,
  -57.8892 ,  20.7147 , -69.6477 ,0.0000,  -0.4670,  -35.5684 ,  15.5299,0.0000,
   22.3955 , -30.4757 , -20.8302 ,0.0000,  44.9403,  10.8929,  -36.0222,0.0000,
  -36.6583 , -29.5131 ,  -1.7608 ,0.0000,  13.1602,  -58.5594,    1.6204,0.0000,
   -0.0000 ,   0.0000 ,  -0.0000 ,1.0000,  -0.0000 ,   0.0000 ,  -0.0000,9.4248
	};
	static float HINV_temp[4][4]=
	{
	-4.0377 ,  -1.1322 ,   2.1848,  0.0000,
    1.0015  ,  0.4233  ,  0.7157,  0.0000,
    1.6191  , -0.5442  ,  0.1021,	0.0000,
   -0.0000  , -0.0000  ,  0.0000,	1.0000
	};

	///////////********r=5*******************//
	// static float JINV_temp[8][18]=
	// {
	// -0.8631,    3.0336 ,  -6.4190 ,  -0.0000 ,  -0.0000 ,   0.0000 ,  -0.0000   ,      0  ,       0  , -0.0435 ,  -0.6478,    7.0158, 0 ,        0 ,        0  ,       0 ,        0 ,        0,
 //    0.3418 ,  -0.5215  , -0.4798 ,  -0.0000 ,   0.0000,   -0.0000  , -0.0000  ,       0   ,      0  ,  0.2342 ,  -0.0047,    0.1387 ,  0 ,        0 ,        0  ,       0 ,        0 ,        0,
 //   -0.6155 ,  -0.3276 ,  -0.1069 ,   0.0000 ,   0.0000,   0.0000 ,   0.0000  ,       0   ,      0  , -0.0126 ,  -0.2874 ,   0.1545,   0 ,        0 ,        0 ,        0 ,        0 ,        0,
 //    -0.0000 ,   0.0000,   -0.0000 ,   0.0000 ,  -0.0000 ,   0.0000 ,  -0.0000  ,       0  ,       0 ,  -0.0000 ,   0.0000 ,  -0.0000 ,   0 ,        0,         0 ,        0 ,        0 ,        0,
 //   -0.0435 ,  -0.6478  ,  7.0158 ,   0.0000  ,  0.0000 ,  -0.0000 ,   0.0000  ,       0   ,      0 ,   0.8631 ,  -3.0336 ,   6.4190,  0  ,       0  ,       0 ,        0 ,        0 ,        0,
 //   0.2342  , -0.0047  ,  0.1387  , -0.0000  , -0.0000 ,  -0.0000 ,  -0.0000    ,     0    ,     0  , -0.3418 ,   0.5215  ,  0.4798,   0  ,       0 ,        0 ,        0 ,        0  ,       0,
 //    -0.0126 ,  -0.2874  ,  0.1545 ,  -0.0000 ,   0.0000  , -0.0000,   -0.0000  ,       0  ,       0 ,   0.6155 ,   0.3276 ,   0.1069,  0  ,       0 ,        0 ,        0 ,        0 ,        0,
 //    -0.0000  ,  0.0000 ,  -0.0000 ,  -0.0000 ,   0.0000  ,  0.0000 ,   0.0000  ,       0  ,       0  ,  0.0000 ,  -0.0000 ,   0.0000 ,   0  ,       0  ,       0  ,       0  ,       0 ,        0
	// };
	// static float RINV_temp[4][9]=
	// {
	// -0.1838 ,  -2.6204 ,   8.7011 ,   2.2566 ,  -3.3141 ,   5.1081 ,   0.0000 ,   5.7876 ,  -7.8606,
 //   -0.5793 ,   0.2559 ,   0.4664,   -0.4794 ,  -0.2324  ,  0.4605 ,  -0.0000  ,  4.1629  ,  0.0598,
 //    0.2910 ,   0.6182 ,   0.1592 ,  -0.2309 ,   0.5023 ,   0.0851 ,   0.0000 ,   0.0598  ,  3.6087,
 //    0.0000 ,  -0.0000 ,  -0.0000  ,  0.0000 ,   0.0000  , -0.0000 ,   1.0000 ,  -0.0000  ,  0.0000
	// };

	///////////********r=0.5*******************//
	// static float JINV_temp[8][18]=
	// {
	// 0.0294,   0.4002 ,  -6.8828 ,  -0.0000 ,  -0.0000  ,  0.0000 ,   0.0000 ,        0 ,        0 ,  -0.0142 ,  -0.0390,    8.3986, 0 ,        0 ,        0  ,       0 ,        0 ,        0,
 //     0.3125,   -0.7205,   -0.0520,    0.0000 ,   0.0000 ,  -0.0000 ,   0.0000 ,        0 ,        0 ,   0.4045 ,  -0.0843,    0.0035,  0 ,        0 ,        0  ,       0 ,        0 ,        0,
 //   -0.7216 ,  -0.3115 ,  -0.0193 ,  -0.0000 ,   0.0000  ,  0.0000 ,  -0.0000  ,       0   ,      0  , -0.0847 ,  -0.4053 ,   0.0305,   0 ,        0 ,        0 ,        0 ,        0 ,        0,
 //    -0.0000,    0.0000 ,  -0.0000 ,   0.0000 ,  -0.0000  ,  0.0000 ,   0.0000 ,        0 ,        0 ,  -0.0000 ,   0.0000 ,  -0.0000,   0 ,        0,         0 ,        0 ,        0 ,        0,
 //   -0.0142 ,  -0.0390 ,   8.3986 ,  -0.0000  ,  0.0000  , -0.0000 ,   0.0000  ,       0  ,       0  , -0.0294 ,  -0.4002  ,  6.8828,  0  ,       0  ,       0 ,        0 ,        0 ,        0,
 //   0.4045  , -0.0843  ,  0.0035  ,  0.0000  , -0.0000   , 0.0000  ,  0.0000   ,      0   ,      0  , -0.3125   , 0.7205  ,  0.0520,   0  ,       0 ,        0 ,        0 ,        0  ,       0,
 //    -0.0847 ,  -0.4053 ,   0.0305 ,   0.0000,    0.0000 ,  -0.0000 ,   0.0000 ,        0  ,       0 ,   0.7216 ,   0.3115 ,   0.0193,  0  ,       0 ,        0 ,        0 ,        0 ,        0,
 //    -0.0000 ,   0.0000,    0.0000,   -0.0000 ,  -0.0000 ,  -0.0000 ,   0.0000 ,        0  ,       0 ,   0.0000 ,  -0.0000 ,   0.0000,   0  ,       0  ,       0  ,       0  ,       0 ,        0
	// };
	// static float RINV_temp[4][9]=
	// {
	// -0.1558 ,  -0.3661,    9.9835 ,   0.2368,   -0.5321 ,   5.4772 ,  -0.0000,    0.8384,   -1.0181,
 //   -0.7155 ,   0.3185 ,   0.0487 ,  -0.5692 ,  -0.3179,    0.0543 ,  -0.0000 ,   3.4978 ,   0.0013,
 //    0.3186 ,   0.7155 ,   0.0305 ,  -0.3177 ,   0.5688 ,   0.0154 ,  -0.0000 ,   0.0013 ,   3.4891,
 //   -0.0000 ,  -0.0000 ,  -0.0000 ,   0.0000 ,  -0.0000 ,  -0.0000 ,   1.0000 ,  -0.0000 ,  -0.0000
	// };
	// static float C_temp[18][18];
	// for(int c_i_temp=0;c_i_temp<18;c_i_temp++)
	// {
	// 	for(int c_j_temp=0;c_j_temp<18;c_j_temp++)
	// 	{
	// 		C_temp[c_i_temp][c_j_temp]=0;
	// 	}
	// }
	// C_temp[0][0]=1;
	// C_temp[1][1]=1;
	// C_temp[2][2]=1;
	// C_temp[9][9]=1;
	// C_temp[10][10]=1;
	// C_temp[11][11]=1;
	

	// static float C_temp_a0[9][9];
	// for(int c_i_temp_a0=0;c_i_temp_a0<9;c_i_temp_a0++)
	// {
	// 	for(int c_j_temp_a0=0;c_j_temp_a0<9;c_j_temp_a0++)
	// 	{
	// 		C_temp_a0[c_i_temp_a0][c_j_temp_a0]=0;
	// 	}
	// }
	// C_temp_a0[0][0]=1;
	// C_temp_a0[1][1]=1;
	// C_temp_a0[2][2]=1;

	// static float C_temp[6][18];
	// for(int c_i_temp=0;c_i_temp<6;c_i_temp++)
	// {
	// 	for(int c_j_temp=0;c_j_temp<18;c_j_temp++)
	// 	{
	// 		C_temp[c_i_temp][c_j_temp]=0;
	// 	}
	// }
	// C_temp[0][0]=1;
	// C_temp[1][1]=1;
	// C_temp[2][2]=1;
	// C_temp[3][9]=1;
	// C_temp[4][10]=1;
	// C_temp[5][11]=1;
	

	// static float C_temp_a0[3][9];
	// for(int c_i_temp_a0=0;c_i_temp_a0<3;c_i_temp_a0++)
	// {
	// 	for(int c_j_temp_a0=0;c_j_temp_a0<9;c_j_temp_a0++)
	// 	{
	// 		C_temp_a0[c_i_temp_a0][c_j_temp_a0]=0;
	// 	}
	// }
	// C_temp_a0[0][0]=1;
	// C_temp_a0[1][1]=1;
	// C_temp_a0[2][2]=1;


	// C_PARAM.set(C_temp);
	// C_PARAM_a0.set(C_temp_a0);

	JINV_1.set(JINV_temp_1);
	JINV_2.set(JINV_temp_2);
	JINV_3.set(JINV_temp_3);
	// XINV.set(XINV_temp);
	// RINV.set(RINV_temp);
	HINV.set(HINV_temp);
	/*      cxy        */
	if(flag_t_num<t_circuit && _v_control_mode.flag_control_offboard_enabled && (fabsf(pos_current_xyz(2))>fabsf(heiht_nominal) || flag_circle>1))
	{
		
		flag_circle=2;
		t_delay=t_delay+dt;
		// double t_delayprint=double(t_delay);
		// PX4_WARN("t_delayprint , %f", t_delayprint );
		if(t_delay>t_delay_sp)     //  hover wait for  seconds to start circuit
		{
			flag_t_num=flag_t_num+dt;
			t_current=t_current+dt;
			_rates_sp=anglerate_sp;

			//*****Acceleration PID controller
			// _thrust_sp=k_hover*a_input/g_nominal+k_p*(a_err)+k_d*(a_err_prev-a_err)+k_i*a_err_last;
			_thrust_sp=k_hover+k_p*(a_err)+k_i*a_err_last+k_d*(a_err_prev-a_err);
			if(_thrust_sp>1)
			{
				_thrust_sp=1;
			}
			if(_thrust_sp<0)
			{
				_thrust_sp=0;
			}
			a_err_last=a_err+a_err_last;
			//*****************//

			if(flag_lqr_enable==0)
			{
				// flag_tdt=1;
				flag_lqr_enable=1;   // for position control to stop poserr integral
			}

			/////ILC part
			if( (flag_t_num>t_circuit_start) && (ILC_enable) )
			{
				
				if(flag_t_x==1)
				{
					t_x=t_x+dt;
					if(t_x<=(T_n-T_n/2))
					{
						
						a0_integral=a0_integral+y_err*dt;
						a1_integral=a1_integral+y_err*cosf(anglerate_nominal*t_x)*dt;
						b1_integral=b1_integral+y_err*sinf(anglerate_nominal*t_x)*dt;
						a2_integral=a2_integral+y_err*cosf(2*anglerate_nominal*t_x)*dt;
						b2_integral=b2_integral+y_err*sinf(2*anglerate_nominal*t_x)*dt;
						a3_integral=a3_integral+y_err*cosf(3*anglerate_nominal*t_x)*dt;
						b3_integral=b3_integral+y_err*sinf(3*anglerate_nominal*t_x)*dt;
						v_compensation=r0+r1*cosf(anglerate_nominal*(t_x+n_v_cal*tpercircle))+s1*sinf(anglerate_nominal*(t_x+n_v_cal*tpercircle))+r2*cosf(2*anglerate_nominal*(t_x+n_v_cal*tpercircle))+s2*sinf(2*anglerate_nominal*(t_x+n_v_cal*tpercircle))+r3*cosf(3*anglerate_nominal*(t_x+n_v_cal*tpercircle))+s3*sinf(3*anglerate_nominal*(t_x+n_v_cal*tpercircle));
					}
					if(t_x>(T_n-T_n/2) && flag_iteration==1)
					{
						
						a0=a0_integral/(T_n);
						// double a00=(double) (a0(0));
						// PX4_WARN("a00 %f", a00);
						a1=a1_integral*2/(T_n);
						b1=b1_integral*2/(T_n);
						a2=a2_integral*2/(T_n);
						b2=b2_integral*2/(T_n);
						a3=a3_integral*2/(T_n);
						b3=b3_integral*2/(T_n);
						for(int i_temp=0;i_temp<4;i_temp++)
						{
							e_param_1(i_temp)=a1(i_temp);
							e_param_1(i_temp+4)=b1(i_temp);
							e_param_2(i_temp)=a2(i_temp);
							e_param_2(i_temp+4)=b2(i_temp);
							e_param_3(i_temp)=a3(i_temp);
							e_param_3(i_temp+4)=b3(i_temp);
							// ek_param=C_PARAM*ek_param;
						}

						vcompensation_param_1=vcompensation_param_1-JINV_1*r_param_1*e_param_1;
						vcompensation_param_2=vcompensation_param_2-JINV_2*r_param_2*e_param_2;
						vcompensation_param_3=vcompensation_param_3-JINV_3*r_param_3*e_param_3;
							// r0=r0-HINV*r_param*(C_PARAM_a0*a0);
							// r0_check=HINV*r_param*a0;
						r0=r0-HINV*r_param_0*a0;
							// double rcheck1=(double) r0_check(0);
							// double rcheck2=(double) (HINV_temp[0][0]*a0(0)+HINV_temp[0][1]*a0(1)+HINV_temp[0][2]*a0(2));
							
							// PX4_WARN("rcheck1 %f", rcheck1);
							// PX4_WARN("rcheck2 %f", rcheck2);
							// r0=RINV*a0;
							// double a0warn=(double) a0(0);
							// double r0warn=(double) r0(0);
							// PX4_WARN("A0warn %f", a0warn);
							// PX4_WARN("R0warn %f", r0warn);
						for(int j_temp=0;j_temp<4;j_temp++)
						{
							s1(j_temp)=vcompensation_param_1(j_temp);
							r1(j_temp)=vcompensation_param_1(j_temp+4);
							s2(j_temp)=vcompensation_param_2(j_temp);
							r2(j_temp)=vcompensation_param_2(j_temp+4);
							s3(j_temp)=vcompensation_param_3(j_temp);
							r3(j_temp)=vcompensation_param_3(j_temp+4);
						}
						a0_integral.zero();
						a1_integral.zero();
						b1_integral.zero();
						a2_integral.zero();
						b2_integral.zero();
						a3_integral.zero();
						b3_integral.zero();
						n_iteration=n_iteration+1;
						flag_iteration=0;
					}
					if (t_x>(T_n-T_n/2))
					{
						v_compensation=r0+r1*cosf(anglerate_nominal*(t_x+n_v_cal*tpercircle))+s1*sinf(anglerate_nominal*(t_x+n_v_cal*tpercircle))+r2*cosf(2*anglerate_nominal*(t_x+n_v_cal*tpercircle))+s2*sinf(2*anglerate_nominal*(t_x+n_v_cal*tpercircle))+r3*cosf(3*anglerate_nominal*(t_x+n_v_cal*tpercircle))+s3*sinf(3*anglerate_nominal*(t_x+n_v_cal*tpercircle));
						flag_ILC=1;					
					}
					if (t_x>(((n_v_cal+n_x_cal)*tpercircle)-T_n/2))
					{
						a0.zero();
						a1.zero();
						b1.zero();
						a2.zero();
						b2.zero();
						a3.zero();
						b3.zero();
						flag_iteration=1;
						flag_t_x=0;
					}
				}

				// if(flag_t_x==1)
				// {
				// 	a0_integral=a0_integral+y_err*dt;
				// 	ak_integral=ak_integral+y_err*cosf(anglerate_nominal*t_x)*dt;
				// 	bk_integral=bk_integral+y_err*sinf(anglerate_nominal*t_x)*dt;
				// 	t_x=t_x+dt;
				// 	if(t_x>(T_n-T_n/2))
				// 	{
				// 		flag_t_v=2;
				// 		flag_t_x=2;
				// 		a0=a0_integral/(T_n);
				// 		double a00=(double) (a0(0));
				// 		PX4_WARN("a00 %f", a00);
				// 		ak=ak_integral*2/(T_n);
				// 		bk=bk_integral*2/(T_n);
				// 		flag_t_v_check=1;
				// 		for(int i_temp=0;i_temp<3;i_temp++)
				// 		{
				// 			ek_param(i_temp)=ak(i_temp);
				// 			ek_param(i_temp+3)=bk(i_temp);
				// 			// ek_param=C_PARAM*ek_param;
				// 		}
				// 		if(n_iteration==0)
				// 		{
				// 			vcompensation_param=vcompensation_param-JINV*r_param*ek_param;
				// 			// r0=r0-HINV*r_param*(C_PARAM_a0*a0);
				// 			// r0_check=HINV*r_param*a0;
				// 			r0=r0-HINV*r_param*a0;
				// 			// double rcheck1=(double) r0_check(0);
				// 			// double rcheck2=(double) (HINV_temp[0][0]*a0(0)+HINV_temp[0][1]*a0(1)+HINV_temp[0][2]*a0(2));
							
				// 			// PX4_WARN("rcheck1 %f", rcheck1);
				// 			// PX4_WARN("rcheck2 %f", rcheck2);
				// 			// r0=RINV*a0;
				// 			// double a0warn=(double) a0(0);
				// 			// double r0warn=(double) r0(0);
				// 			// PX4_WARN("A0warn %f", a0warn);
				// 			// PX4_WARN("R0warn %f", r0warn);
				// 			for(int j_temp=0;j_temp<4;j_temp++)
				// 			{
				// 				sk(j_temp)=vcompensation_param(j_temp);
				// 				rk(j_temp)=vcompensation_param(j_temp+4);
				// 			}
				// 			n_iteration=1;
				// 		}
				// 	}
				// }
				// if(flag_t_v==2)
				// {
				
				// 	if(t_v>(n_v_cal*tpercircle-T_n/2) && flag_t_x_flag==0)
				// 	{
				// 		flag_t_x=0;
				// 		flag_t_x_flag=1;
				// 	}
				// 	if(t_v>(((n_v_cal+n_x_cal)*tpercircle)-T_n/2) && flag_t_v_check==1 )
				// 	{
				// 		t_v=-T_n/2;
				// 		flag_t_x_flag=0;
				// 		if(n_iteration>0)
				// 		{
				// 			vcompensation_param=vcompensation_param-JINV*r_param*ek_param;
				// 			for(int k_temp=0;k_temp<4;k_temp++)
				// 			{
				// 				sk(k_temp)=vcompensation_param(k_temp);
				// 				rk(k_temp)=vcompensation_param(k_temp+4);
				// 			}
							
				// 			r0=r0-HINV*r_param*a0;
				// 			flag_t_v_check=0;

				// 			r0_check=HINV*r_param*a0;
				// 			double rcheck1=(double) r0_check(0);
				// 			double rcheck2=(double) (HINV_temp[0][0]*a0(0)+HINV_temp[0][1]*a0(1)+HINV_temp[0][2]*a0(2));
				// 			double rchecka00=(double) (a0(0));
				// 			double rchecka01=(double) (a0(1));
				// 			double rchecka02=(double) (a0(2));
				// 			double rcheck21=(double) (HINV_temp[0][0]*a0(0));
				// 			double rcheck22=(double) (HINV_temp[0][1]*a0(1));
				// 			double rcheck23=(double) (HINV_temp[0][2]*a0(2));
				// 			PX4_WARN("rcheck1 %f", rcheck1);
				// 			PX4_WARN("rcheck2 %f", rcheck2);
				// 			PX4_WARN("rchecka00 %f", rchecka00);
				// 			PX4_WARN("rchecka01 %f", rchecka01);
				// 			PX4_WARN("rchecka02 %f", rchecka02);
				// 			PX4_WARN("rcheck21 %f", rcheck21);
				// 			PX4_WARN("rcheck22 %f", rcheck22);
				// 			PX4_WARN("rcheck23 %f", rcheck23);



				// 			// r0=r0-HINV*r_param*(C_PARAM_a0*a0);
				// 			// r0=RINV*a0;


				// 			n_iteration=n_iteration+1;
				// 			// double n_iterationwarn=(double) n_iteration;
				// 			// PX4_WARN("n_iterationwarn %f", n_iterationwarn);
				// 		}
				// 	}
				// 	v_compensation=r0+rk*cosf(anglerate_nominal*t_v)+sk*sinf(anglerate_nominal*t_v);
				// 	t_v=t_v+dt;

					a_log_att.a01=a0(0);
					a_log_att.a02=a0(1);
					a_log_att.a03=a0(2);
					a_log_att.a04=a0(3);
					// a_log_att.a05=a0(4);
					// a_log_att.a06=a0(5);
					// a_log_att.a07=a0(6);
					// a_log_att.a08=a0(7);
					// a_log_att.a09=a0(8);

					a_log_att.a11=a1(0);
					a_log_att.a12=a1(1);
					a_log_att.a13=a1(2);
					a_log_att.a14=a1(3);

					a_log_att.a21=a2(0);
					a_log_att.a22=a2(1);
					a_log_att.a23=a2(2);
					a_log_att.a24=a2(3);

					a_log_att.a31=a3(0);
					a_log_att.a32=a3(1);
					a_log_att.a33=a3(2);
					a_log_att.a34=a3(3);

					// a_log_att.ak4=ak(3);
					// a_log_att.ak5=ak(4);
					// a_log_att.ak6=ak(5);
					// a_log_att.ak7=ak(6);
					// a_log_att.ak8=ak(7);
					// a_log_att.ak9=ak(8);

					a_log_att.b11=b1(0);
					a_log_att.b12=b1(1);
					a_log_att.b13=b1(2);
					a_log_att.b14=b1(3);

					a_log_att.b21=b2(0);
					a_log_att.b22=b2(1);
					a_log_att.b23=b2(2);
					a_log_att.b24=b2(3);

					a_log_att.b31=b3(0);
					a_log_att.b32=b3(1);
					a_log_att.b33=b3(2);
					a_log_att.b34=b3(3);
					// a_log_att.bk4=bk(3);
					// a_log_att.bk5=bk(4); 
					// a_log_att.bk6=bk(5);
					// a_log_att.bk7=bk(6);
					// a_log_att.bk8=bk(7);
					// a_log_att.bk9=bk(8);

					a_log_att.norma0=sqrtf(a0(0)*a0(0)+a0(1)*a0(1)+a0(2)*a0(2));
					a_log_att.norma1=sqrtf(a1(0)*a1(0)+a1(1)*a1(1)+a1(2)*a1(2));
					a_log_att.normb1=sqrtf(b1(0)*b1(0)+b1(1)*b1(1)+b1(2)*b1(2));
					a_log_att.norma2=sqrtf(a2(0)*a2(0)+a2(1)*a2(1)+a2(2)*a2(2));
					a_log_att.normb2=sqrtf(b2(0)*b2(0)+b2(1)*b2(1)+b2(2)*b2(2));
					a_log_att.norma3=sqrtf(a3(0)*a3(0)+a3(1)*a3(1)+a3(2)*a3(2));
					a_log_att.normb3=sqrtf(b3(0)*b3(0)+b3(1)*b3(1)+b3(2)*b3(2));

					a_log_att.r01=r0(0);
					a_log_att.r02=r0(1);
					a_log_att.r03=r0(2);
					a_log_att.r04=r0(3);

					a_log_att.r0_check_1=r0_check(0);
					a_log_att.r0_check_2=r0_check(1);
					a_log_att.r0_check_3=r0_check(2);
					a_log_att.r0_check_4=r0_check(3);

					a_log_att.r11=r1(0);
					a_log_att.r12=r1(1);
					a_log_att.r13=r1(2);
					a_log_att.r14=r1(3);

					a_log_att.r21=r2(0);
					a_log_att.r22=r2(1);
					a_log_att.r23=r2(2);
					a_log_att.r24=r2(3);

					a_log_att.r31=r3(0);
					a_log_att.r32=r3(1);
					a_log_att.r33=r3(2);
					a_log_att.r34=r3(3);

					a_log_att.s11=s1(0);
					a_log_att.s12=s1(1);
					a_log_att.s13=s1(2);
					a_log_att.s14=s1(3);

					a_log_att.s21=s2(0);
					a_log_att.s22=s2(1);
					a_log_att.s23=s2(2);
					a_log_att.s24=s2(3);

					a_log_att.s31=s3(0);
					a_log_att.s32=s3(1);
					a_log_att.s33=s3(2);
					a_log_att.s34=s3(3);

					a_log_att.compensation_1=v_compensation(0);
					a_log_att.compensation_2=v_compensation(1);
					a_log_att.compensation_3=v_compensation(2);
					a_log_att.compensation_4=v_compensation(3);
				// }
			}
		}
	}
	else
	{
		t_current=0;
		angle_n=1;
	}
	if(flag_t_num>t_circuit)
	{
		flag_lqr_enable=0;
		// flag_tdt=0;
	}
	///initialization again
	if(!_v_control_mode.flag_control_offboard_enabled && flag_circle>1)
	{
		flag_circle=0;
		t_delay=0;
		flag_t_num=0;
		flag_lqr_enable=0;
		angle_n=1;

		flag_t_x=0;
		flag_first_cal=1;
		flag_ILC=0;
		// flag_tdt=0;
	}

	a_err_prev=a_err;
	// double aa=(double)a_err;
	// PX4_WARN("a_err %f", aa);
	// double ainput=(double)a_input;
	// PX4_WARN("ainput %f", ainput);
	// double thrust=(double)_thrust_sp;
	// PX4_WARN("thrust %f", thrust);

	// float flag_dt = (hrt_absolute_time() - last_t) / 1000000.0f;
	// last_t = hrt_absolute_time();
	// double flag_dt2=(double)flag_dt;
	// PX4_WARN("dt %f",flag_dt2);

	a_log_att.flag_lqr_enable=flag_lqr_enable;
	a_log_att.a_current=-a_current;
	a_log_att.a_input=a_input;
	a_log_att.a_err=a_err;
	a_log_att.flag_wrate=flag_wrate;
	a_log_att.err_u=x_err(0);
	a_log_att.err_v=x_err(1);
	a_log_att.err_yaw=x_err(6);
	a_log_att.current_yaw=x_current(6);
	a_log_att.nominal_yaw=x_nominal(6);
	// a_log_att.err_u_nominal=x_err_nominal(0);
	// a_log_att.err_v_nominal=x_err_nominal(1);

	a_log_att.current_u=x_current(0);
	a_log_att.nominal_u=x_nominal(0);

	a_log_att.current_v=x_current(1);
	a_log_att.nominal_v=x_nominal(1);

	a_log_att.t_x=t_x;
	a_log_att.t_v=t_v;
	a_log_att.flag_t_x=flag_t_x;
	a_log_att.n_iteration=n_iteration;
	a_log_att.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(a_log_att), a_log_att_pub, &a_log_att);
	// double hz_att=double(a_log_att.hz_positionin);
	// PX4_WARN("HZ_ATT , %f", hz_att );

	// *****LQR cxy********//


	/* get transformation matrix from sensor/board to body frame */
	get_rot_matrix((enum Rotation)_params.board_rotation, &_board_rotation);

	/* fine tune the rotation */
	math::Matrix<3, 3> board_rotation_offset;
	board_rotation_offset.from_euler(M_DEG_TO_RAD_F * _params.board_offset[0],
					 M_DEG_TO_RAD_F * _params.board_offset[1],
					 M_DEG_TO_RAD_F * _params.board_offset[2]);
	_board_rotation = board_rotation_offset * _board_rotation;

	// get the raw gyro data and correct for thermal errors
	math::Vector<3> rates;

	if (_selected_gyro == 0) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_0[0]) * _sensor_correction.gyro_scale_0[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_0[1]) * _sensor_correction.gyro_scale_0[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_0[2]) * _sensor_correction.gyro_scale_0[2];

	} else if (_selected_gyro == 1) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_1[0]) * _sensor_correction.gyro_scale_1[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_1[1]) * _sensor_correction.gyro_scale_1[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_1[2]) * _sensor_correction.gyro_scale_1[2];

	} else if (_selected_gyro == 2) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_2[0]) * _sensor_correction.gyro_scale_2[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_2[1]) * _sensor_correction.gyro_scale_2[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_2[2]) * _sensor_correction.gyro_scale_2[2];

	} else {
		rates(0) = _sensor_gyro.x;
		rates(1) = _sensor_gyro.y;
		rates(2) = _sensor_gyro.z;
	}

	// rotate corrected measurements from sensor to body frame
	rates = _board_rotation * rates;

	// correct for in-run bias errors
	rates(0) -= _ctrl_state.roll_rate_bias;
	rates(1) -= _ctrl_state.pitch_rate_bias;
	rates(2) -= _ctrl_state.yaw_rate_bias;

	rates_log=rates;
	rates_sp_original=_rates_sp;

	math::Vector<3> rates_p_scaled = _params.rate_p.emult(pid_attenuations(_params.tpa_breakpoint_p, _params.tpa_rate_p));
	//math::Vector<3> rates_i_scaled = _params.rate_i.emult(pid_attenuations(_params.tpa_breakpoint_i, _params.tpa_rate_i));
	math::Vector<3> rates_d_scaled = _params.rate_d.emult(pid_attenuations(_params.tpa_breakpoint_d, _params.tpa_rate_d));

	/* angular rates error */
	math::Vector<3> rates_err = _rates_sp - rates;

	

	// if(!_v_control_mode.flag_control_offboard_enabled && flag_circle>1)
	// {
	// 	flag_circle=0;
	// 	_rates_sp(0)=0;
	// 	_rates_sp(1)=0;
	// 	_rates_sp(2)=0;
	// }
	_att_control = rates_p_scaled.emult(rates_err) +
		       _rates_int +
		       rates_d_scaled.emult(_rates_prev - rates) / dt +
		       _params.rate_ff.emult(_rates_sp);

	_rates_sp_prev = _rates_sp;
	_rates_prev = rates;

	/* update integral only if motors are providing enough thrust to be effective */
	if (_thrust_sp > MIN_TAKEOFF_THRUST) {
		for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
			// Check for positive control saturation
			bool positive_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_pos) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_pos) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_pos);

			// Check for negative control saturation
			bool negative_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_neg) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_neg) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_neg);

			// prevent further positive control saturation
			if (positive_saturation) {
				rates_err(i) = math::min(rates_err(i), 0.0f);

			}

			// prevent further negative control saturation
			if (negative_saturation) {
				rates_err(i) = math::max(rates_err(i), 0.0f);

			}

			// Perform the integration using a first order method and do not propaate the result if out of range or invalid
			float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;

			// lyu: do sth to combined, if yaw saturate turn off the integral term
			bool yaw_saturation = ((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_pos) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_neg);
			if (yaw_saturation && _params.vtol_type == vtol_type::STANDARD) {
				rate_i = 0.0f;
				//PX4_WARN("integrator set to zero in combined");
			}

			if (PX4_ISFINITE(rate_i) && rate_i > -_params.rate_int_lim(i) && rate_i < _params.rate_int_lim(i)) {
				_rates_int(i) = rate_i;
				// PX4_WARN("yaw_saturation %d, positive_saturation %d, negative_saturation %d",yaw_saturation,positive_saturation,negative_saturation);
			}
		}
	}

	/* explicitly limit the integrator state */
	for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
		_rates_int(i) = math::constrain(_rates_int(i), -_params.rate_int_lim(i), _params.rate_int_lim(i));

	}
}

void
MulticopterAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	mc_att_control::g_control->task_main();
}

void
MulticopterAttitudeControl::task_main()
{

	/*
	 * do subscriptions
	 */
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	sensor_data_sub = orb_subscribe(ORB_ID(sensor_combined));

	//cxy
	a_log_att_pub = orb_advertise(ORB_ID(a_log_att), &a_log_att);

	_gyro_count = math::min(orb_group_count(ORB_ID(sensor_gyro)), MAX_GYRO_COUNT);

	if (_gyro_count == 0) {
		_gyro_count = 1;
	}

	for (unsigned s = 0; s < _gyro_count; s++) {
		_sensor_gyro_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
	}

	_sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));

	/* initialize parameters cache */
	parameters_update();

	/* wakeup source: gyro data from sensor selected by the sensor app */
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;

	while (!_task_should_exit) {
		perf_count(_update_perf);
		poll_fds.fd = _sensor_gyro_sub[_selected_gyro];

						
		/* wait for up to 100ms for data */
		int pret = px4_poll(&poll_fds, 1, 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("mc att ctrl: poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		/* run controller on gyro changes */
		if (poll_fds.revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			/* copy gyro data */
			orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub[_selected_gyro], &_sensor_gyro);

			/* check for updates in other topics */
			parameter_update_poll();
			vehicle_control_mode_poll();
			arming_status_poll();
			vehicle_manual_poll();
			vehicle_status_poll();
			vehicle_motor_limits_poll();
			battery_status_poll();
			control_state_poll();
			sensor_correction_poll();

			/* Check if we are in rattitude mode and the pilot is above the threshold on pitch
			 * or roll (yaw can rotate 360 in normal att control).  If both are true don't
			 * even bother running the attitude controllers */
			if (_v_control_mode.flag_control_rattitude_enabled) {
				if (fabsf(_manual_control_sp.y) > _params.rattitude_thres ||
				    fabsf(_manual_control_sp.x) > _params.rattitude_thres) {
					_v_control_mode.flag_control_attitude_enabled = false;
				}
			}

			if (_v_control_mode.flag_control_attitude_enabled) {

				if (_ts_opt_recovery == nullptr) {
					// the  tailsitter recovery instance has not been created, thus, the vehicle
					// is not a tailsitter, do normal attitude control
					control_attitude(dt);

				} else {
					vehicle_attitude_setpoint_poll();
					_thrust_sp = _v_att_sp.thrust;
					math::Quaternion q(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
					math::Quaternion q_sp(&_v_att_sp.q_d[0]);
					_ts_opt_recovery->setAttGains(_params.att_p, _params.yaw_ff);
					_ts_opt_recovery->calcOptimalRates(q, q_sp, _v_att_sp.yaw_sp_move_rate, _rates_sp);

					/* limit rates */
					for (int i = 0; i < 3; i++) {
						_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
					}
				}

				/* publish attitude rates setpoint */
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub != nullptr) {
					orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

				} else if (_rates_sp_id) {
					_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
				}

				//}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				if (_v_control_mode.flag_control_manual_enabled) {
					/* manual rates control - ACRO mode */
					_rates_sp = math::Vector<3>(_manual_control_sp.y, -_manual_control_sp.x,
								    _manual_control_sp.r).emult(_params.acro_rate_max);
					_thrust_sp = math::min(_manual_control_sp.z, MANUAL_THROTTLE_MAX_MULTICOPTER);

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
					}

				} else {
					/* attitude controller disabled, poll rates setpoint topic */
					vehicle_rates_setpoint_poll();
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = _v_rates_sp.thrust;
				}
			}

			if (_v_control_mode.flag_control_rates_enabled) {
				control_attitude_rates(dt);

				// //cxy//
				// if(flag_circle>1)
				// {
				// 	_thrust_sp=0.3;
				// }
					
				/* publish actuator controls */
				_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
				_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
				_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
				_actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
				_actuators.control[7] = _v_att_sp.landing_gear;
				_actuators.timestamp = hrt_absolute_time();
				_actuators.timestamp_sample = _ctrl_state.timestamp;


				/* scale effort by battery status */
				if (_params.bat_scale_en && _battery_status.scale > 0.0f) {
					for (int i = 0; i < 4; i++) {
						_actuators.control[i] *= _battery_status.scale;
					}
				}

				_controller_status.roll_rate_integ = _rates_int(0);
				_controller_status.pitch_rate_integ = _rates_int(1);
				_controller_status.yaw_rate_integ = _rates_int(2);
				_controller_status.timestamp = hrt_absolute_time();

				if (!_actuators_0_circuit_breaker_enabled) {
					if (_actuators_0_pub != nullptr) {

						orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
						perf_end(_controller_latency_perf);

					} else if (_actuators_id) {
						_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
					}

				}

				// lyu: use _actuators_1_pub to pub control command
				// _actuators_1.control[0] = -0.8f;
				// _actuators_1.control[1] = 0.8f;
				// _actuators_1.control[2] = 2.0f;
				// _actuators_1.control[4] = -1.0f;
				// _actuators_1.timestamp = hrt_absolute_time();
				// _actuators_1.timestamp_sample = _ctrl_state.timestamp;

				// if (!_actuators_0_circuit_breaker_enabled) {
				// 	if (_actuators_1_pub != nullptr) {
				// 		orb_publish(ORB_ID(actuator_controls_1), _actuators_1_pub, &_actuators_1);
				// 	} else {
				// 		_actuators_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &_actuators_1);
				// 	}
				// }

				/* publish controller status */
				if (_controller_status_pub != nullptr) {
					orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

				} else {
					_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
				}
			}

			if (_v_control_mode.flag_control_termination_enabled) {
				if (!_vehicle_status.is_vtol) {

					_rates_sp.zero();
					_rates_int.zero();
					_thrust_sp = 0.0f;
					_att_control.zero();


					/* publish actuator controls */
					_actuators.control[0] = 0.0f;
					_actuators.control[1] = 0.0f;
					_actuators.control[2] = 0.0f;
					_actuators.control[3] = 0.0f;
					_actuators.timestamp = hrt_absolute_time();
					_actuators.timestamp_sample = _ctrl_state.timestamp;

					if (!_actuators_0_circuit_breaker_enabled) {
						if (_actuators_0_pub != nullptr) {

							orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
							perf_end(_controller_latency_perf);

						} else if (_actuators_id) {
							_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
						}
					}

					_controller_status.roll_rate_integ = _rates_int(0);
					_controller_status.pitch_rate_integ = _rates_int(1);
					_controller_status.yaw_rate_integ = _rates_int(2);
					_controller_status.timestamp = hrt_absolute_time();

					/* publish controller status */
					if (_controller_status_pub != nullptr) {
						orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

					} else {
						_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
					}

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
					}
				}
			}
		}

		perf_end(_loop_perf);
	}

	_control_task = -1;
}

void
MulticopterAttitudeControl::print_info(){
	warnx("flag_control_manual_enabled: %s", _v_control_mode.flag_control_manual_enabled ? "ture" : "false");
	warnx("flag_control_auto_enabled: %s", _v_control_mode.flag_control_auto_enabled ? "ture" : "false");
	warnx("flag_control_rates_enabled: %s", _v_control_mode.flag_control_rates_enabled ? "ture" : "false");
	warnx("flag_control_attitude_enabled: %s", _v_control_mode.flag_control_attitude_enabled ? "ture" : "false");
	warnx("flag_control_rattitude_enabled: %s", _v_control_mode.flag_control_rattitude_enabled ? "ture" : "false");
	warnx("flag_control_altitude_enabled: %s", _v_control_mode.flag_control_altitude_enabled ? "ture" : "false");
	warnx("flag_control_climb_rate_enabled: %s", _v_control_mode.flag_control_climb_rate_enabled ? "ture" : "false");
	warnx("flag_control_position_enabled: %s", _v_control_mode.flag_control_position_enabled ? "ture" : "false");
	warnx("flag_control_velocity_enabled: %s", _v_control_mode.flag_control_velocity_enabled ? "ture" : "false");
	warnx("flag_control_acceleration_enabled: %s", _v_control_mode.flag_control_acceleration_enabled ? "ture" : "false");
	warnx("flag_control_termination_enabled: %s", _v_control_mode.flag_control_termination_enabled ? "ture" : "false");
	perf_print_counter(_loop_perf);
	perf_print_counter(_update_perf);
}


int
MulticopterAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   6000,
					   (px4_main_t)&MulticopterAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_att_control {start|stop|status|info}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (mc_att_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		mc_att_control::g_control = new MulticopterAttitudeControl;

		if (mc_att_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != mc_att_control::g_control->start()) {
			delete mc_att_control::g_control;
			mc_att_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (mc_att_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete mc_att_control::g_control;
		mc_att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (mc_att_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	if (!strcmp(argv[1], "info")) {
		warnx("Print vehicle logic status, loop time, loop rates");
		mc_att_control::g_control->print_info();
		return 0;
	}

	warnx("unrecognized command");
	return 1;
}
