#pragma once

/*
Incremental Nonlinear Dynamic Inversion Controller (INDI)
Microraptor Works
Liuzhenchang 2020.5.21
*/

#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>


class MW_INDI {
public:
	MW_INDI(AP_AHRS_DCM& ahrs, const AP_Vehicle::FixedWing& parms)
		: _ahrs(ahrs)
		, aparm(parms)
	{
		AP_Param::setup_object_defaults(this, var_info);
		T_x = 100;
		aileron = 0;
		elevator = 0;
		rudder = 0;
		aileron_last = 0;
		elevator_last = 0;
		rudder_last = 0;
	}

	/* Do not allow copies */
	MW_INDI(const MW_INDI& other) = delete;
	MW_INDI& operator=(const MW_INDI&) = delete;

	void INDI_state_process_10HZ();
	void INDI_state_process_400HZ();

	void update_AOA_SSA();						//updata angle of attack and side slip angle
	void update_velocity();						//update velocity and acceleration 
	void update_kinematic_azimuth_angle();		//updata chi and chi rate
	void update_flight_path_angle();			//update gamma and gamma rate
	void update_bank_angle();					//update mu

	void INDI_aerodynamic_coefficient();
	void trajectory_control(const struct Location& prev_WP, const struct Location& next_WP);
	void attitude_control();

	float get_thrust();								//get engine thrust in unit N
	float get_aileron();							//get aileron in unit degree
	float get_elevator();							//get elevator in unit degree
	float get_rudder();								//get rudder in unit degree

	float get_V();
	float get_d_V1();
	float get_d_V2();
	float get_d_V3();

	Vector3f get_a_body_1();
	Vector3f get_a_body_2();

	// this supports the MW_INDI_* user settable parameters
	static const struct AP_Param::GroupInfo var_info[];

private:

	//////////////////////////////////////////////////////////////////////////
	/////////////////////////// INDI tuning parameters////////////////////////
	//////////////////////////////////////////////////////////////////////////
	
	AP_Float _k_x;
	AP_Float _k_y;
	AP_Float _k_z;

	AP_Float _k_V;
	AP_Float _k_chi;
	AP_Float _k_gamma;

	AP_Float _k_mu;
	AP_Float _k_alpha;
	AP_Float _k_beta;

	AP_Float _k_p;
	AP_Float _k_q;
	AP_Float _k_r;

	//Definition of control variables in x0
	float X;			//position in the north direction
	float Y;			//position in the east direction
	float Z;			//position in the down direction
	Vector3f x0;		//positon vector
	
	float X_ref;		//reference position in the north direction
	float Y_ref;		//reference position in the east direction
	float Z_ref;		//reference position in the down direction
	Vector3f x0_ref;	//reference position vector

	float d_X_des;		//desired velocity in the north direction
	float d_Y_des;		//desired velocity in the east direction
	float d_Z_des;		//desired velocity in the down direction
	Vector3f d_x0_des;	//desired velocity vector

	DerivativeFilterFloat_Size7		_ax_filter;
	DerivativeFilterFloat_Size7		_ay_filter;
	DerivativeFilterFloat_Size7		_az_filter;

	//Definition of control variables in x1
	float V;			//total velocity of the aircraft
	float d_V1;          //velocity rate calculated by body axis acceleration
	float d_V2;          //velocity rate calculated by 7 point derivative filter
	float d_V3;          //velocity rate calculated by 9 point derivative filter

	Vector3f a_body_1;  //acceleration in body axis calculated by ground axis velocity
	Vector3f a_body_2;  //acceleration in body axis acquired by gyroscope

	float chi;			//kinematic azimuth angle
	float gamma;		//flight path angle
	Vector3f x1;		//flight path vector
	Vector3f velocity;	//velocity vector
	Vector3f a;			//acceleration in body axis

	float d_V;			//the derivation of velocity
	float d_chi;		//the derivation of kinematic azimuth angle
	float d_gamma;		//the derivation of flight path angle
	float d_chi_last;	//avoid the sudden change of d_chi value, when chi stride accross 180 degrees

	// declares a 5point average filter using floats
	AverageFilterFloat_Size5		_vdot1_filter;
	DerivativeFilterFloat_Size7		_vdot2_filter;
	DerivativeFilterFloat_Size9     _vdot3_filter;
	DerivativeFilterFloat_Size9     _chidot_filter;
	DerivativeFilterFloat_Size9     _gammadot_filter;

	float V_ref;		//reference total velocity of the aircraft
	float chi_ref;		//reference kinematic azimuth angle
	float gamma_ref;	//reference flight path angle
	Vector3f x1_ref;	//reference flight path vector

	float d_V_des;		//desired total acceleration of the aircraft
	float d_chi_des;	//desired kinematic azimuth angular velocity
	float d_gamma_des;	//desired flight path angular velocity
	Vector3f d_x1_des;	//desired derivation of flight path vector

	//Definition of control variables in x2
	float mu;			//bank angle
	float alpha;		//attack angle
	float beta;			//sideslip angle
	Vector3f x2;		//vector of aerodynamic control variables

	float mu_ref;		//reference bank angle
	float alpha_ref;	//reference attack angle
	float beta_ref;		//reference sideslip angle
	Vector3f x2_ref;	//reference vector of aerodynamic control variables

	float d_mu_des;		//desired bank angular velocity
	float d_alpha_des;	//desired attack angular velocity
	float d_beta_des;	//desired sideslip angular velocity
	Vector3f d_x2_des;	//desired derivation of aerodynamic control variables vector

	float increm_alpha; //increment of attack angle
	float increm_T_x;	//increment of thrust in body axis
	float T_x;			//thrust in body axis
	float T_x_last;		//last thrust in body axis

	////Definition of control variables in x3
	float p;			//roll rate
	float q;			//pitch rate
	float r;			//yaw rate 
	Vector3f x3;		//angular velocity vector

	float p_ref;		//reference_roll rate
	float q_ref;		//reference_pitch rate
	float r_ref;		//reference_yaw rate 
	Vector3f x3_ref;	//reference_angular velocity vector

	float d_p;		//desired roll acceleration
	float d_q;		//desired pitch acceleration
	float d_r;		//desired yaw acceleration 
	Vector3f d_x3;	//desired angular acceleration vector

	DerivativeFilterFloat_Size9     _pdot_filter;
	DerivativeFilterFloat_Size9     _qdot_filter;
	DerivativeFilterFloat_Size9     _rdot_filter;

	float d_p_des;		//desired roll acceleration
	float d_q_des;		//desired pitch acceleration
	float d_r_des;		//desired yaw acceleration 
	Vector3f d_x3_des;	//desired angular acceleration vector

	//Definition of servos output
	float aileron;		//unit:rad
	float elevator;		//unit:rad
	float rudder;		//unit:rad

	float aileron_last;		//unit:rad
	float elevator_last;		//unit:rad
	float rudder_last;		//unit:rad

	// reference to the AHRS object
	AP_AHRS_DCM& _ahrs; //LZC modified AP_AHRS &_ahrs;

	const AP_Vehicle::FixedWing& aparm;
	bool _data_is_stale = true;

	//Vehicle property
	float m; //vehicle mass
	float CL_alpha;		//the derrivation of lift coefficient due to AOA
	float CD_alpha;		//the derrivation of drag coefficient due to AOA
	float Cl_aileron;	//the derrivation of roll moment coefficient due to aileron
	float Cm_elavator;	//the derrivation of pitch moment coefficient due to elevator
	float Cn_rudder;	//the derrivation of yaw moment coefficient due to rudder
};
