/*
Incremental Nonlinear Dynamic Inversion Controller (INDI)
Microraptor Works
Liuzhenchang 2020.5.21
*/

#define air_density		1.28	//air density, unit kg/m^3
#define reference_area	0.982	//reference wing area, unit m^2
#define reference_c		0.351	//reference chord length, unit m
#define reference_b		2.795	//reference wing span, unit m
#define vehicle_mass    5.897	//Vehicle mass, unit kg
#define T_max           80      //Maximum Thrust
#define T_min           0       //Minimum Thrust

#include "MW_INDI.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo MW_INDI::var_info[] = {

	// @Param: GAIN_KX
	// @DisplayName: Desired x velocity Proportional Gain
	// @Description: This is the gain to calculate desired x directional velocity in tanslational kinematic control loop
	// @Increment: unknown
	// @Range: unknown
	// @User: Standard
	AP_GROUPINFO("GAIN_KX",    1, MW_INDI, _k_x, 0.1f),

	// @Param: GAIN_KY
	// @DisplayName: Desired y velocity Proportional Gain
	// @Description: This is the gain to calculate desired y directional velocity in tanslational kinematic control loop
	// @Increment: unknown
	// @Range: unknown
	// @User: Standard
	AP_GROUPINFO("GAIN_KY",    2, MW_INDI, _k_y, 0.1f),

	// @Param: GAIN_KZ
	// @DisplayName: Desired z velocity Proportional Gain
	// @Description: This is the gain to calculate desired z directional velocity in tanslational kinematic control loop
	// @Increment: unknown
	// @Range: unknown
	// @User: Standard
	AP_GROUPINFO("GAIN_KZ",    3, MW_INDI, _k_z, 0.1f),

	// @Param: GAIN_KV
	// @DisplayName: Desired acceleration scaler Proportional Gain
	// @Description: This is the gain to calculate Desired acceleration in tanslational dynamic control loop
	// @Increment: unknown
	// @Range: unknown
	// @User: Standard
	AP_GROUPINFO("GAIN_KV",    4, MW_INDI, _k_V, 2.0f),

	// @Param: GAIN_KCHI
	// @DisplayName: Desired heading angular velocity Proportional Gain
	// @Description: This is the gain to calculate Desired kinematic azimuth angular velocity in tanslational dynamic control loop
	// @Increment: unknown
	// @Range: unknown
	// @User: Standard
	AP_GROUPINFO("GAIN_KCHI",    5, MW_INDI, _k_chi, 1.0f),

	// @Param: GAIN_KGAM
	// @DisplayName: Desired flight path angular velocity Proportional Gain
	// @Description: This is the gain to calculate Desired flight path angular velocity in tanslational dynamic control loop
	// @Increment: unknown
	// @Range: unknown
	// @User: Standard
	AP_GROUPINFO("GAIN_KGAM",    6, MW_INDI, _k_gamma, 2.0f),

	// @Param: GAIN_KMU
	// @DisplayName: Desired bank angular velocity Proportional Gain
	// @Description: This is the gain to calculate desired bank angular velocity in rotational kinematic control loop
	// @Increment: unknown
	// @Range: unknown
	// @User: Standard
	AP_GROUPINFO("GAIN_KMU",    7, MW_INDI, _k_mu, 2.0f),

	// @Param: GAIN_KALP
	// @DisplayName: Desired attack angular velocity Proportional Gain
	// @Description: This is the gain to calculate desired attack angular velocity in rotational kinematic control loop
	// @Increment: unknown
	// @Range: unknown
	// @User: Standard
	AP_GROUPINFO("GAIN_KALP",    8, MW_INDI, _k_alpha, 10.0f),

	// @Param: GAIN_KBET
	// @DisplayName: Desired sideslip angular velocity Proportional Gain
	// @Description: This is the gain to calculate desired sideslip angular velocity in rotational kinematic control loop
	// @Increment: unknown
	// @Range: unknown
	// @User: Standard
	AP_GROUPINFO("GAIN_KBET",    9, MW_INDI, _k_beta, 2.0f),

	// @Param: GAIN_KP
	// @DisplayName: Desired roll angular acceleration Proportional Gain
	// @Description: This is the gain to calculate desired roll angular acceleration in rotational dynamic control loop
	// @Increment: unknown
	// @Range: unknown
	// @User: Standard
	AP_GROUPINFO("GAIN_KP",    10, MW_INDI, _k_p, 6.0f),

	// @Param: GAIN_KQ
	// @DisplayName: Desired pitch angular acceleration Proportional Gain
	// @Description: This is the gain to calculate desired pitch angular acceleration in rotational dynamic control loop
	// @Increment: unknown
	// @Range: unknown
	// @User: Standard
	AP_GROUPINFO("GAIN_KQ",    11, MW_INDI, _k_q, 6.0f),

	// @Param: GAIN_KR
	// @DisplayName: Desired yaw angular acceleration Proportional Gain
	// @Description: This is the gain to calculate desired yaw angular acceleration in rotational dynamic control loop
	// @Increment: unknown
	// @Range: unknown
	// @User: Standard
	AP_GROUPINFO("GAIN_KR",    12, MW_INDI, _k_r, 3.0f),


	AP_GROUPEND
};

/*******************************updata angle of attack and side slip angle************************************/
void MW_INDI::update_AOA_SSA() 
{
	alpha = radians(_ahrs.getAOA());	//unit: rad
	beta = radians(_ahrs.getSSA());		//unit: rad
}


/************************************update velocity and acceleration****************************************/
void MW_INDI::update_velocity()
{
	uint64_t now = AP_HAL::millis();

	//////////////////////////////////////////// V1 //////////////////////////////////////////////////////////
	if (_ahrs.get_velocity_NED(velocity) == false) {
		//printf("can't get velocity");
		return;}
	float V1 = velocity.length();

	//define V
	V = V1;

	///////////////////////////////////////// a_body_1 ////////////////////////////////////////////////////////
	_ax_filter.update(velocity.x, now);								//the time stamp is in millim second
	float ax = _ax_filter.slope() * 1.0e3;							//take 7 point derivative filter
	_ay_filter.update(velocity.y, now);								//the time stamp is in millim second
	float ay = _ay_filter.slope() * 1.0e3;							//take 7 point derivative filter
	_az_filter.update(velocity.z, now);								//the time stamp is in millim second
	float az = _az_filter.slope() * 1.0e3;							//take 7 point derivative filter
	Vector3f a_ground = Vector3f(ax, ay, az);
    const Matrix3f& rotMat = _ahrs.get_rotation_body_to_ned();      //get DCM
	a_body_1 = rotMat.mul_transpose(a_ground);                      //rotMat is the DCM matrix from body to ground

	///////////////////////////////////////// a_body_2 ////////////////////////////////////////////////////////
	a_body_2= AP::ins().get_accel();

	//define a
	a = a_body_1;

	//////////////////////////////////////////// d_V_1 ////////////////////////////////////////////////////////
	float temp2 = a.x * cosf(alpha) + a.z * sinf(alpha);	//calculate acceleration
	d_V1 = _vdot1_filter.apply(temp2);								//take 5 point moving average


	//////////////////////////////////////////// d_V_2 ////////////////////////////////////////////////////////
	_vdot2_filter.update(velocity.length(), now);					//the time stamp is in millim second
	d_V2 = _vdot2_filter.slope() * 1.0e3;							//take 7 point derivative filter

	//////////////////////////////////////////// d_V_3 ////////////////////////////////////////////////////////
	_vdot3_filter.update(velocity.length(), now);					//the time stamp is in millim second
	d_V3 = _vdot3_filter.slope() * 1.0e3;							//take 9 point derivative filter

	//define d_V
	d_V = d_V1;
}

/************************************************update mu***************************************************/
void MW_INDI::update_bank_angle()
{
	//uint64_t now = AP_HAL::millis();
	// use roll angle to repalce mu for temporary
	float mu_1 = radians(0.01 * wrap_180_cd(_ahrs.roll_sensor));	//unit: rad
	mu = mu_1;
}

/*****************************************updata chi and chi rate********************************************/
void MW_INDI::update_kinematic_azimuth_angle()
{
	uint64_t now = AP_HAL::millis();

    ////////////////////////////////////////////// chi_1 //////////////////////////////////////////////////////
    if (velocity.x < 0 && velocity.y>0) { chi_1 = atanf(velocity.y / velocity.x) + M_PI; }
    else if (velocity.x < 0 && velocity.y < 0) { chi_1 = atanf(velocity.y / velocity.x) - M_PI; }
    else { chi_1 = atanf(velocity.y / velocity.x); }

    ////////////////////////////////////////////// chi_2 //////////////////////////////////////////////////////
    Vector2f groundspeed_vector = _ahrs.groundspeed_vector();
	if (groundspeed_vector.x < 0 && groundspeed_vector.y>0) { chi_2 = atanf(groundspeed_vector.y / groundspeed_vector.x) + M_PI; }
	else if (groundspeed_vector.x < 0 && groundspeed_vector.y < 0) { chi_2 = atanf(groundspeed_vector.y / groundspeed_vector.x) - M_PI; }
	else { chi_2 = atanf(groundspeed_vector.y / groundspeed_vector.x); }

	////////////////////////////////////////////// chi_3 //////////////////////////////////////////////////////
	chi_3 = radians(0.01 * wrap_180_cd(_ahrs.yaw_sensor));		    //unit: rad

	//define chi
	if (velocity.length() > 5) { chi = chi_2; }
	else { chi = chi_3; }

    /////////////////////////////////////////////// d_chi1 /////////////////////////////////////////////////////
	_chidot_filter.update(chi, now);
	d_chi1 = _chidot_filter.slope() * 1.0e3;
	// avoid the sudden change of d_chi value, when chi stride accross 180 degrees
	if ((degrees(chi) > 175 || degrees(chi) < -175) && (abs(d_chi1 - d_chi_last) * 180 / M_PI) > 3) { d_chi1 = d_chi_last; }
	else { d_chi_last = d_chi1; }

	/////////////////////////////////////////////// d_chi2 /////////////////////////////////////////////////////
	d_chi2 = (1 / (V * cosf(gamma))) * (a.x * sinf(alpha) * sinf(mu) + a.y * cosf(mu) - a.z * cosf(alpha) * sinf(mu)); //unit: rad/s

	//define d_chi
	d_chi = d_chi1;
}

//update gamma and gamma rate
void MW_INDI::update_flight_path_angle() 
{
	uint64_t now = AP_HAL::millis();

	////////////////////////////////////////////// gamma_1 /////////////////////////////////////////////////////
	gamma_1 = asinf(-velocity.z / velocity.length());				    //unit: rad

	////////////////////////////////////////////// gamma_2 /////////////////////////////////////////////////////
	Vector2f groundspeed_vector = _ahrs.groundspeed_vector();
	gamma_2 = atanf(-velocity.z / groundspeed_vector.length());	        //unit: rad

	////////////////////////////////////////////// gamma_3 /////////////////////////////////////////////////////
	gamma_3 = radians(0.01 * wrap_180_cd(_ahrs.pitch_sensor));	        //unit: rad

	//define gamma
	if (velocity.length() > 5) { gamma = gamma_1; }                     // When velocity is small using yaw angle to replace chi
	else { gamma = gamma_1; }

	////////////////////////////////////////////// d_gamma1 /////////////////////////////////////////////////////
	_gammadot_filter.update(gamma, now);
	d_gamma1 = _gammadot_filter.slope() * 1.0e3;

	////////////////////////////////////////////// d_gamma2 /////////////////////////////////////////////////////
	d_gamma2 = (1 / V) * (a.x * sinf(alpha) * cosf(mu) - a.y * sinf(mu) - a.z * cosf(alpha) * cosf(mu));	//unit: rad/s

	//Determining which method to be used
	d_gamma = d_gamma1;

}

void MW_INDI::INDI_state_process_10HZ()
{	
	update_AOA_SSA();					//get angle of attack (AOA) from Ahrs
	
	update_velocity();					//update velocity and acceleration
	
	update_bank_angle();				//update mu

	update_kinematic_azimuth_angle();	//updata chi and chi rate

	update_flight_path_angle();			//update gamma and gamma rate

	INDI_aerodynamic_coefficient();		//update aerodynamic coefficient
}

void MW_INDI::INDI_state_process_400HZ()
{	
	uint64_t now = AP_HAL::millis();
	//get attitude anglular rate & acceleration
	x3 = AP::ins().get_gyro();
	p = AP::ins().get_gyro().x;
	q = AP::ins().get_gyro().y;
	r = AP::ins().get_gyro().z;

	_pdot_filter.update(p, now);
	_qdot_filter.update(q, now);
	_rdot_filter.update(r, now);

	d_p = _pdot_filter.slope() * 1.0e3;
	d_q = _qdot_filter.slope() * 1.0e3;
	d_r = _rdot_filter.slope() * 1.0e3;

	d_x3 = Vector3f(d_p, d_q, d_r);

	INDI_aerodynamic_coefficient();
}


void MW_INDI::INDI_aerodynamic_coefficient()
{
	//get angle of attack (AOA) from Ahrs
	alpha = radians(_ahrs.getAOA());	//unit: rad
	beta = radians(_ahrs.getSSA());		//unit: rad

	//get aerodynamic coefficient
	// choose CL_alpha with alpha section
	if (alpha > -1.57 && alpha < -0.2) {
        CL_alpha = 0.454;
    } else if (alpha >= -0.2 && alpha < 0.23) {
        CL_alpha = 2.268;
    } else if (alpha >= 0.23 && alpha < 0.6) {
        CL_alpha = -0.844;
    } else if (alpha >= 0.6 && alpha < 1.57) {
        CL_alpha = -0.331;
    } else {
        CL_alpha = 0;
    }
	// choose CD_alpha with alpha section
    if (alpha > -1.57 && alpha < -0.26) {
        CD_alpha = -0.5;
    } else if (alpha >= -0.26 && alpha < 0) {
        CD_alpha = -0.049;
    } else if (alpha >= 0 && alpha < 0.26) {
        CD_alpha = 0.049;
    } else if (alpha >= 0.26 && alpha < 1.57) {
        CD_alpha = 0.5;
    } else {
        CD_alpha = 0;
    }

	Cl_aileron = 0.018;
	Cm_elavator = 0.0691; //LZC Caution: Multiply -1
	Cn_rudder = 0.0069; //LZC Caution: Multiply -1
}

void MW_INDI::trajectory_control(const struct Location& prev_WP, const struct Location& next_WP)
{
	
	INDI_state_process_10HZ();
	//uint64_t now = AP_HAL::millis();
	struct Location _current_loc;
	// Get current position and velocity
	if (_ahrs.get_position(_current_loc) == false) {
		// if no GPS loc available, maintain last nav/target_bearing
		_data_is_stale = true;
		_current_loc=prev_WP;
		return;
	}

	///////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////Translational kinematic control lopp in NDI/////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////
	
	//get the position vector (x0) value
	//return the distance in meters in North / East / Down vector from current location to home
	x0 = location_3d_diff_NED(_ahrs.get_home(), _current_loc);
	x0_ref = location_3d_diff_NED(_ahrs.get_home(), next_WP);

	//K_x0 is a diagonal matrix, which is the gain matrix of the position linear control
	Matrix3f K_x0 = Matrix3f(	_k_x,	0,		0,
								0,		_k_y,	0,
								0,		0,		_k_z);
	
	//calculate desired velocity vector
	d_x0_des=K_x0* location_3d_diff_NED(_current_loc, next_WP);
	d_X_des = d_x0_des.x;
	d_Y_des = d_x0_des.y;
	d_Z_des = d_x0_des.z;

	//calculate reference flight path vector
	V_ref = 0.01 * aparm.airspeed_cruise_cm; //unit: m
	//limit the range of chi from -90 degree to 90 degree 

	if (d_X_des < 0 && d_Y_des>0) { chi_ref = atanf(d_Y_des / d_X_des) + M_PI; }
	else if (d_X_des < 0 && d_Y_des < 0) { chi_ref = atanf(d_Y_des / d_X_des) - M_PI; }
	else { chi_ref = atanf(d_Y_des / d_X_des); }

	
	//limit the range of gamma from -45 degree to 45 degree 
	gamma_ref = -asinf(constrain_float(d_Z_des / V_ref, -0.707, 0.707));
	
	x1_ref = Vector3f(V_ref, chi_ref, gamma_ref);

	////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////Translational dynamic control lopp in INDI/////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////
	m = vehicle_mass; //mass unit kg
	x1 = Vector3f(V, chi, gamma);
	
	//calculate desired velocity change rate
	d_V_des = _k_V * (V_ref - V);
	//calculate desired chi change rate, keep the difference of chi_ref and chi in [-180,180]
	if (chi_ref - chi > M_PI) { d_chi_des = _k_chi * (chi_ref - chi - M_2PI); }
	else if (chi_ref - chi < -M_PI) { d_chi_des = _k_chi * (chi_ref - chi + M_2PI); }
	else { d_chi_des = _k_chi * (chi_ref - chi); }
	//calculate desired gamma change rate
	d_gamma_des = _k_gamma * (gamma_ref - gamma);

	d_x1_des = Vector3f(d_V_des, d_chi_des, d_gamma_des);


	double a11 = -0.5 * air_density * sq(V) * reference_area * CD_alpha - T_x * sinf(alpha);
	double a12 = cosf(alpha);
	double a21 = -0.5 * air_density * sq(V) * reference_area * CL_alpha * cosf(mu) - T_x * cosf(mu) * cosf(alpha);
	double a22 = -cosf(mu) * sinf(alpha);


	increm_alpha = (m / (a11 * a22 - a21 * a12)) * (a22 * (d_V_des - d_V) + a12 * V * (d_gamma_des - d_gamma));
	increm_T_x = (-m / (a11 * a22 - a21 * a12)) * (a21 * (d_V_des - d_V) + a11 * V * (d_gamma_des - d_gamma));

    increm_alpha = constrain_float((float) increm_alpha, -10, 10);
    increm_T_x = constrain_float((float) increm_T_x, -20, 20);

	alpha_ref = constrain_float(alpha + 0.2*increm_alpha,-M_PI/4,M_PI/4);
	T_x = constrain_float(T_x_last + 0.1*increm_T_x, T_min, T_max);
	T_x_last = T_x;

	watch2=float(a11);
    watch3=float(a11 * a22 - a21 * a12);
    watch4 = float(a21);
    watch5 = float(a22);
    watch6 = float(increm_alpha);

    //calculate reference bank angle
    mu_ref = atanf(
            (d_chi_des * V * cosf(gamma))
                    / (d_gamma * V + GRAVITY_MSS * cosf(gamma))); //LZC Caution
    // Using waypoint fault tolerant guidance method to limit the range of mu_ref
    if (location_3d_diff_NED(_current_loc, next_WP) * velocity > 0)
        mu_ref = constrain_float(mu_ref, -1, 1);
    else
        mu_ref = constrain_float(mu_ref, -0.6, 0.6);

    beta_ref = 0;
    x2_ref = Vector3f(mu_ref, alpha_ref, beta_ref);

    trajectory_flag = true;
}


void MW_INDI::attitude_control()
{
	///////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////Rotational kinematic control loop in NDI//////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////
	INDI_state_process_400HZ();

	//uint64_t now = AP_HAL::millis();
	x2 = Vector3f(mu, alpha, beta);
	//K_x2 is a diagonal matrix, which is the gain matrix of the attitude linear control
	Matrix3f K_x2 = Matrix3f(	_k_mu,		0,			0,
								0,			_k_alpha,	0,
								0,			0,			_k_beta);
	//calculate desired flight path vector
	d_x2_des = K_x2 * (x2_ref - x2);

    watch1 = d_chi2 * 180 / M_PI;

	Matrix3f M_rot_kine_1 = Matrix3f(	cosf(alpha) * cosf(beta),			0,				sinf(alpha),
										sinf(beta),						    1,				0,
										sinf(alpha) * cosf(beta),			0,				-cosf(alpha));

	Matrix3f M_rot_kine_2 = Matrix3f(	cosf(alpha) * cosf(beta),											sinf(beta),						sinf(alpha) * cosf(beta),
										-cosf(alpha) * sinf(beta) * cosf(mu) + sinf(alpha) * sinf(mu),		cosf(beta) * cosf(mu),			-sinf(alpha) * sinf(beta) * cosf(mu) - cosf(alpha) * sinf(mu),
										-cosf(alpha) * sinf(beta) * sinf(mu) - sinf(alpha) * cosf(mu),		cosf(beta) * sinf(mu),			-sinf(alpha) * sinf(beta) * sinf(mu) + cosf(alpha) * cosf(mu));

	Vector3f V_rot_kine_1 = Vector3f(-d_chi2 * sinf(gamma), d_gamma2, d_chi2 * cosf(gamma));

	x3_ref = M_rot_kine_1 * d_x2_des + M_rot_kine_2.transposed() * V_rot_kine_1;


	///////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////Rotational dynamic control loop in NDI///////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////
	x3 = Vector3f(p, q, r);
	//K_x2 is a diagonal matrix, which is the gain matrix of the attitude linear control
	Matrix3f K_x3 = Matrix3f(	_k_p,		0,			0,
								0,			_k_q,		0,
								0,			0,			_k_r);
	//calculate desired flight path vector
	d_x3_des = K_x3 * (x3_ref - x3);


	Matrix3f M_coefficent = Matrix3f(	Cl_aileron,		0,				0,
										0,				Cm_elavator,	0,
										0,				0,				Cn_rudder);

	Matrix3f M_reference = Matrix3f(	reference_b,	0,				0,
										0,				reference_c,	0,
										0,				0,				reference_b);

	Matrix3f M_inertia = Matrix3f(	2.644,		0,		    0,
									0,		    2.101,		0,
									0,		    0   ,		2.590);
	M_coefficent.invert();
	M_reference.invert();
	
	Vector3f delta_x4;
	delta_x4 = M_coefficent* M_reference * M_inertia * (d_x3_des- d_x3)* (0.015 / (0.5 * air_density * sq(V) * reference_area));

	aileron = aileron_last + constrain_float(delta_x4.x, -0.1, 0.1); //LZC: Caution: the time factor is determined by the  control frequency
	aileron = constrain_float(aileron, radians(-30), radians(30));
	if (!(isnan(aileron))&& !(isnan(delta_x4.x)))
	{ aileron_last = aileron; }

	elevator = elevator_last + constrain_float(delta_x4.y, -0.1, 0.1);
	elevator = constrain_float(elevator, radians(-30), radians(30));
	if (!(isnan(elevator)) && !(isnan(delta_x4.y)))
	{ elevator_last = elevator; }
	
	rudder = rudder_last + constrain_float(delta_x4.z, -0.1, 0.1);
	rudder = 0*constrain_float(rudder, radians(-30), radians(30));
	if (!(isnan(rudder)) && !(isnan(delta_x4.z)))
	{ rudder_last = rudder; }
}


/************************* Velocity and acceleration Log***************************/
float MW_INDI::get_V() {return V;}
float MW_INDI::get_d_V1() {return d_V1;}
float MW_INDI::get_d_V2() {return d_V2;}
float MW_INDI::get_d_V3() {return d_V3;}

Vector3f MW_INDI::get_a_body_1() {return a_body_1;}
Vector3f MW_INDI::get_a_body_2() {return a_body_2;}

/********************** Kinematic azimuth angle and rate Log************************/
float MW_INDI::get_chi_1() {return degrees(chi_1);}
float MW_INDI::get_chi_2() {return degrees(chi_2);}
float MW_INDI::get_chi_3() {return degrees(chi_3);}
float MW_INDI::get_d_chi1() {return degrees(d_chi1);}
float MW_INDI::get_d_chi2() {return degrees(d_chi2);}

/************************** Flight path angle and rate Log***************************/
float MW_INDI::get_gamma_1() {return degrees(gamma_1);}
float MW_INDI::get_gamma_2() {return degrees(gamma_2);}
float MW_INDI::get_gamma_3() {return degrees(gamma_3);}
float MW_INDI::get_d_gamma1() {return degrees(d_gamma1);}
float MW_INDI::get_d_gamma2() {return degrees(d_gamma2);}

/********************* Translational kinematic control Loop Log**********************/
Vector3f MW_INDI::get_x0() {return x0;}
Vector3f MW_INDI::get_x0_ref() {return x0_ref;}

Vector3f MW_INDI::get_x1() {return x1;}
Vector3f MW_INDI::get_x1_ref() {return x1_ref;}

Vector3f MW_INDI::get_x2() {return x2;}
Vector3f MW_INDI::get_x2_ref() {return x2_ref;}

float MW_INDI::get_inc_alpha() {return degrees(increm_alpha);}
float MW_INDI::get_inc_T() {return increm_T_x;}

Vector3f MW_INDI::get_x3() {return x3;}
Vector3f MW_INDI::get_x3_ref() {return x3_ref;}

float MW_INDI::get_thrust() {return T_x;}
float MW_INDI::get_aileron() { return degrees(aileron); }
float MW_INDI::get_elevator() { return degrees(elevator); }
float MW_INDI::get_rudder() { return degrees(rudder); }
bool  MW_INDI::get_trajectory_flag() { return trajectory_flag; }

/****************************** control output signal *******************************/
int32_t MW_INDI::get_aileron_out(){return int32_t(degrees(aileron)*100);}
int32_t MW_INDI::get_elevator_out(){return int32_t(degrees(elevator)*100);}
int16_t MW_INDI::get_rudder_out(){return int16_t(degrees(rudder)*100);}
int32_t MW_INDI::get_throttle_demand(){return int32_t((T_x/T_max)*100);}

/****************************** INDI watched variables ******************************/
float MW_INDI::get_watch1() {return watch1;}
float MW_INDI::get_watch2() {return watch2;}
float MW_INDI::get_watch3() {return watch3;}
float MW_INDI::get_watch4() {return watch4;}
float MW_INDI::get_watch5() {return watch5;}
float MW_INDI::get_watch6() {return watch6;}
