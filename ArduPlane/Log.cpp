#include "Plane.h"

#if LOGGING_ENABLED == ENABLED

// Write an attitude packet
void Plane::Log_Write_Attitude(void)
{
    Vector3f targets;       // Package up the targets into a vector for commonality with Copter usage of Log_Wrote_Attitude
    targets.x = nav_roll_cd;
    targets.y = nav_pitch_cd;

    if (quadplane.in_vtol_mode() || quadplane.in_assisted_flight()) {
        // when VTOL active log the copter target yaw
        targets.z = wrap_360_cd(quadplane.attitude_control->get_att_target_euler_cd().z);
    } else {
        //Plane does not have the concept of navyaw. This is a placeholder.
        targets.z = 0;
    }
    
    if (quadplane.tailsitter_active() || quadplane.in_vtol_mode()) {
        // we need the attitude targets from the AC_AttitudeControl controller, as they
        // account for the acceleration limits
        targets = quadplane.attitude_control->get_att_target_euler_cd();
        DataFlash.Log_Write_AttitudeView(*quadplane.ahrs_view, targets);
    } else {
        DataFlash.Log_Write_Attitude(ahrs, targets);
    }
    if (quadplane.in_vtol_mode() || quadplane.in_assisted_flight()) {
        // log quadplane PIDs separately from fixed wing PIDs
        DataFlash.Log_Write_PID(LOG_PIQR_MSG, quadplane.attitude_control->get_rate_roll_pid().get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIQP_MSG, quadplane.attitude_control->get_rate_pitch_pid().get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIQY_MSG, quadplane.attitude_control->get_rate_yaw_pid().get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIQA_MSG, quadplane.pos_control->get_accel_z_pid().get_pid_info() );
    }

    DataFlash.Log_Write_PID(LOG_PIDR_MSG, rollController.get_pid_info());
    DataFlash.Log_Write_PID(LOG_PIDP_MSG, pitchController.get_pid_info());
    DataFlash.Log_Write_PID(LOG_PIDY_MSG, yawController.get_pid_info());
    DataFlash.Log_Write_PID(LOG_PIDS_MSG, steerController.get_pid_info());

#if AP_AHRS_NAVEKF_AVAILABLE
    DataFlash.Log_Write_EKF(ahrs);
    DataFlash.Log_Write_AHRS2(ahrs);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.Log_Write_SIMSTATE(&DataFlash);
#endif
    DataFlash.Log_Write_POS(ahrs);
}

// do logging at loop rate
void Plane::Log_Write_Fast(void)
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }
}


struct PACKED log_Startup {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t startup_type;
    uint16_t command_total;
};

void Plane::Log_Write_Startup(uint8_t type)
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG),
        time_us         : AP_HAL::micros64(),
        startup_type    : type,
        command_total   : mission.num_commands()
    };
    DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t nav_roll_cd;
    int16_t roll;
    int16_t nav_pitch_cd;
    int16_t pitch;
    int16_t throttle_out;
    int16_t rudder_out;
    int16_t throttle_dem;
    float airspeed_estimate;
};

// Write a control tuning packet. Total length : 22 bytes
void Plane::Log_Write_Control_Tuning()
{
    float est_airspeed = 0;
    ahrs.airspeed_estimate(&est_airspeed);
    
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CTUN_MSG),
        time_us         : AP_HAL::micros64(),
        nav_roll_cd     : (int16_t)nav_roll_cd,
        roll            : (int16_t)ahrs.roll_sensor,
        nav_pitch_cd    : (int16_t)nav_pitch_cd,
        pitch           : (int16_t)ahrs.pitch_sensor,
        throttle_out    : (int16_t)SRV_Channels::get_output_scaled(SRV_Channel::k_throttle),
        rudder_out      : (int16_t)SRV_Channels::get_output_scaled(SRV_Channel::k_rudder),
        throttle_dem    : (int16_t)SpdHgt_Controller->get_throttle_demand(),
        airspeed_estimate : est_airspeed
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float wp_distance;
    int16_t target_bearing_cd;
    int16_t nav_bearing_cd;
    int16_t altitude_error_cm;
    float   xtrack_error;
    float   xtrack_error_i;
    float   airspeed_error;
    int32_t target_lat;
    int32_t target_lng;
    int32_t target_alt;
};

// Write a navigation tuning packet
void Plane::Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NTUN_MSG),
        time_us             : AP_HAL::micros64(),
        wp_distance         : auto_state.wp_distance,
        target_bearing_cd   : (int16_t)nav_controller->target_bearing_cd(),
        nav_bearing_cd      : (int16_t)nav_controller->nav_bearing_cd(),
        altitude_error_cm   : (int16_t)altitude_error_cm,
        xtrack_error        : nav_controller->crosstrack_error(),
        xtrack_error_i      : nav_controller->crosstrack_error_integrator(),
        airspeed_error      : airspeed_error,
        target_lat          : next_WP_loc.lat,
        target_lng          : next_WP_loc.lng,
        target_alt          : next_WP_loc.alt,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Status {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t is_flying;
    float is_flying_probability;
    uint8_t armed;
    uint8_t safety;
    bool is_crashed;
    bool is_still;
    uint8_t stage;
    bool impact;
};

void Plane::Log_Write_Status()
{
    struct log_Status pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STATUS_MSG)
        ,time_us   : AP_HAL::micros64()
        ,is_flying   : is_flying()
        ,is_flying_probability : isFlyingProbability
        ,armed       : hal.util->get_soft_armed()
        ,safety      : static_cast<uint8_t>(hal.util->safety_switch_state())
        ,is_crashed  : crash_state.is_crashed
        ,is_still    : AP::ins().is_still()
        ,stage       : static_cast<uint8_t>(flight_stage)
        ,impact      : crash_state.impact_detected
        };

    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Sonar {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float distance;
    float voltage;
    uint8_t count;
    float correction;
};

// Write a sonar packet
void Plane::Log_Write_Sonar()
{
    uint16_t distance = 0;
    if (rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::RangeFinder_Good) {
        distance = rangefinder.distance_cm_orient(ROTATION_PITCH_270);
    }

    struct log_Sonar pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SONAR_MSG),
        time_us     : AP_HAL::micros64(),
        distance    : (float)distance*0.01f,
        voltage     : rangefinder.voltage_mv_orient(ROTATION_PITCH_270)*0.001f,
        count       : rangefinder_state.in_range_count,
        correction  : rangefinder_state.correction
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));

    DataFlash.Log_Write_RFND(rangefinder);
}

struct PACKED log_Optflow {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t surface_quality;
    float flow_x;
    float flow_y;
    float body_x;
    float body_y;
};

#if OPTFLOW == ENABLED
// Write an optical flow packet
void Plane::Log_Write_Optflow()
{
    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }
    const Vector2f &flowRate = optflow.flowRate();
    const Vector2f &bodyRate = optflow.bodyRate();
    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        time_us         : AP_HAL::micros64(),
        surface_quality : optflow.quality(),
        flow_x           : flowRate.x,
        flow_y           : flowRate.y,
        body_x           : bodyRate.x,
        body_y           : bodyRate.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

struct PACKED log_Arm_Disarm {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  arm_state;
    uint16_t arm_checks;
};

void Plane::Log_Arm_Disarm() {
    struct log_Arm_Disarm pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ARM_DISARM_MSG),
        time_us                 : AP_HAL::micros64(),
        arm_state               : arming.is_armed(),
        arm_checks              : arming.get_enabled_checks()      
    };
    DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
}


struct PACKED log_AETR {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t aileron;
    int16_t elevator;
    int16_t throttle;
    int16_t rudder;
    int16_t flap;
};

void Plane::Log_Write_AETR()
{
    struct log_AETR pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AETR_MSG)
        ,time_us  : AP_HAL::micros64()
        ,aileron  : SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)
        ,elevator : SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)
        ,throttle : SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)
        ,rudder   : SRV_Channels::get_output_scaled(SRV_Channel::k_rudder)
        ,flap     : SRV_Channels::get_output_scaled(SRV_Channel::k_flap_auto)
        };

    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

void Plane::Log_Write_RC(void)
{
    DataFlash.Log_Write_RCIN();
    DataFlash.Log_Write_RCOUT();
    if (rssi.enabled()) {
        DataFlash.Log_Write_RSSI(rssi);
    }
    Log_Write_AETR();
}

/////////////////////////////////// INDI Log //////////////////////////////////////

/************************* Velocity and acceleration Log***************************/
struct PACKED log_INDI_V {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float V;
    float d_V1;
    float d_V2;
    float d_V3;
    float d_V4;
    float ab1_x;
    float ab1_y;
    float ab1_z;
    float ab2_x;
    float ab2_y;
    float ab2_z;
};

void Plane::Log_Write_INDI_V(void)
{
    struct log_INDI_V pkt = {
            LOG_PACKET_HEADER_INIT(LOG_INDIV_MSG),
            time_us             : AP_HAL::micros64(),
            V                   : INDI_controller.get_V(),
            d_V1                :INDI_controller.get_d_V1(),
            d_V2                :INDI_controller.get_d_V2(),
            d_V3                :INDI_controller.get_d_V3(),
            d_V4                :INDI_controller.get_d_V4(),
            ab1_x               :INDI_controller.get_a_body_1().x,
            ab1_y               :INDI_controller.get_a_body_1().y,
            ab1_z               :INDI_controller.get_a_body_1().z,
            ab2_x               :INDI_controller.get_a_body_2().x,
            ab2_y               :INDI_controller.get_a_body_2().y,
            ab2_z               :INDI_controller.get_a_body_2().z,
        };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

/*********************** Kinematic azimuth angle and rate Log************************/
struct PACKED log_INDI_CHI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float chi_1;
    float chi_2;
    float chi_3;
    float d_chi1;
    float d_chi2;
};

void Plane::Log_Write_INDI_CHI(void)
{
    struct log_INDI_CHI pkt = {
            LOG_PACKET_HEADER_INIT(LOG_INDICHI_MSG),
            time_us             : AP_HAL::micros64(),
            chi_1               :INDI_controller.get_chi_1(),
            chi_2               :INDI_controller.get_chi_2(),
            chi_3               :INDI_controller.get_chi_3(),
            d_chi1              :INDI_controller.get_d_chi1(),
            d_chi2              :INDI_controller.get_d_chi2(),

        };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

/************************** Flight path angle and rate Log***************************/
struct PACKED log_INDI_GAM {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float gamma_1;
    float gamma_2;
    float gamma_3;
    float d_gamma1;
    float d_gamma2;
    float d_gamma3;

};


void Plane::Log_Write_INDI_GAM(void)
{
    struct log_INDI_GAM pkt = {
            LOG_PACKET_HEADER_INIT(LOG_INDIGAM_MSG),
            time_us             : AP_HAL::micros64(),
            gamma_1             :INDI_controller.get_gamma_1(),
            gamma_2             :INDI_controller.get_gamma_2(),
            gamma_3             :INDI_controller.get_gamma_3(),
            d_gamma1            :INDI_controller.get_d_gamma1(),
            d_gamma2            :INDI_controller.get_d_gamma2(),
            d_gamma3            :INDI_controller.get_d_gamma3(),
        };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

/********************* Translational kinematic control Loop Log**********************/
struct PACKED log_INDI_X0 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float X;
    float Y;
    float Z;
    float X_ref;
    float Y_ref;
    float Z_ref;
};


void Plane::Log_Write_INDI_X0(void)
{
    struct log_INDI_X0 pkt = {
            LOG_PACKET_HEADER_INIT(LOG_INDIX0_MSG),
            time_us             : AP_HAL::micros64(),
            X                   :INDI_controller.get_x0().x,
            Y                   :INDI_controller.get_x0().y,
            Z                   :INDI_controller.get_x0().z,
            X_ref               :INDI_controller.get_x0_ref().x,
            Y_ref               :INDI_controller.get_x0_ref().y,
            Z_ref               :INDI_controller.get_x0_ref().z,
        };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

/********************* Translational Dynamic control Loop Log**********************/
struct PACKED log_INDI_X1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float V;
    float chi;
    float gamma;
    float V_ref;
    float chi_ref;
    float gamma_ref;
};


void Plane::Log_Write_INDI_X1(void)
{
    struct log_INDI_X1 pkt = {
            LOG_PACKET_HEADER_INIT(LOG_INDIX1_MSG),
            time_us             : AP_HAL::micros64(),
            V                   :INDI_controller.get_x1().x,
            chi                 :degrees(INDI_controller.get_x1().y),
            gamma               :degrees(INDI_controller.get_x1().z),
            V_ref               :INDI_controller.get_x1_ref().x,
            chi_ref             :degrees(INDI_controller.get_x1_ref().y),
            gamma_ref           :degrees(INDI_controller.get_x1_ref().z),
        };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

/********************* Rotational kinematic control Loop Log**********************/
struct PACKED log_INDI_X2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float mu;
    float alpha;
    float beta;
    float mu_ref;
    float alpha_ref;
    float beta_ref;
};


void Plane::Log_Write_INDI_X2(void)
{
    struct log_INDI_X2 pkt = {
            LOG_PACKET_HEADER_INIT(LOG_INDIX2_MSG),
            time_us             : AP_HAL::micros64(),
            mu                  :degrees(INDI_controller.get_x2().x),
            alpha               :degrees(INDI_controller.get_x2().y),
            beta                :degrees(INDI_controller.get_x2().z),
            mu_ref              :degrees(INDI_controller.get_x2_ref().x),
            alpha_ref           :degrees(INDI_controller.get_x2_ref().y),
            beta_ref            :degrees(INDI_controller.get_x2_ref().z),
        };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

/********************* Rotational dynamic control Loop Log**********************/
struct PACKED log_INDI_X3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float p;
    float q;
    float r;
    float p_ref;
    float q_ref;
    float r_ref;
};


void Plane::Log_Write_INDI_X3(void)
{
    struct log_INDI_X3 pkt = {
            LOG_PACKET_HEADER_INIT(LOG_INDIX3_MSG),
            time_us             : AP_HAL::micros64(),
            p                   :degrees(INDI_controller.get_x3().x),
            q                   :degrees(INDI_controller.get_x3().y),
            r                   :degrees(INDI_controller.get_x3().z),
            p_ref               :degrees(INDI_controller.get_x3_ref().x),
            q_ref               :degrees(INDI_controller.get_x3_ref().y),
            r_ref               :degrees(INDI_controller.get_x3_ref().z),
        };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

/*************************** Desired control output ****************************/
struct PACKED log_INDI_X4 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float aileron;
    float elevator;
    float rudder;
    float thrust;
};


void Plane::Log_Write_INDI_X4(void)
{
    struct log_INDI_X4 pkt = {
            LOG_PACKET_HEADER_INIT(LOG_INDIX4_MSG),
            time_us             : AP_HAL::micros64(),
            aileron             :INDI_controller.get_aileron()*100,
            elevator            :INDI_controller.get_elevator()*100,
            rudder              :INDI_controller.get_rudder()*100,
            thrust              :INDI_controller.get_thrust(),
        };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

/*************************** INDI turning watch variables ****************************/
struct PACKED log_INDI_WATCH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float watch1;
    float watch2;
    float watch3;
    float watch4;
    float watch5;
    float watch6;
};


void Plane::Log_Write_INDI_WATCH(void)
{
    struct log_INDI_WATCH pkt = {
            LOG_PACKET_HEADER_INIT(LOG_INDIWTH_MSG),
            time_us             : AP_HAL::micros64(),
            watch1             :INDI_controller.get_watch1(),
            watch2             :INDI_controller.get_watch2(),
            watch3             :INDI_controller.get_watch3(),
            watch4             :INDI_controller.get_watch4(),
            watch5             :INDI_controller.get_watch5(),
            watch6             :INDI_controller.get_watch6(),
        };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
/////////////////////////////////////////////////////////////////////////////////////

// type and unit information can be found in
// libraries/DataFlash/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Plane::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "QBH",         "TimeUS,SType,CTot", "s--", "F--" },
    { LOG_CTUN_MSG, sizeof(log_Control_Tuning),     
      "CTUN", "Qcccchhhf",    "TimeUS,NavRoll,Roll,NavPitch,Pitch,ThrOut,RdrOut,ThrDem,Aspd", "sdddd---n", "FBBBB---0" },
    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),         
      "NTUN", "QfcccfffLLi",  "TimeUS,WpDist,TBrg,NavBrg,AltErr,XT,XTi,ArspdErr,TLat,TLng,TAlt", "smddmmmnDUm", "F0BBB0B0GGB" },
    { LOG_SONAR_MSG, sizeof(log_Sonar),             
      "SONR", "QffBf",   "TimeUS,Dist,Volt,Cnt,Corr", "smv--", "FB0--" },
    { LOG_ARM_DISARM_MSG, sizeof(log_Arm_Disarm),
      "ARM", "QBH", "TimeUS,ArmState,ArmChecks", "s--", "F--" },
    { LOG_ATRP_MSG, sizeof(AP_AutoTune::log_ATRP),
      "ATRP", "QBBcfff",  "TimeUS,Type,State,Servo,Demanded,Achieved,P", "s---dd-", "F---00-" },
    { LOG_STATUS_MSG, sizeof(log_Status),
      "STAT", "QBfBBBBBB",  "TimeUS,isFlying,isFlyProb,Armed,Safety,Crash,Still,Stage,Hit", "s--------", "F--------" },
    { LOG_QTUN_MSG, sizeof(QuadPlane::log_QControl_Tuning),
      "QTUN", "Qffffhhfffff", "TimeUS,AngBst,ThrOut,DAlt,Alt,DCRt,CRt,DVx,DVy,DAx,DAy,TMix", "s--mmnnnnoo-", "F--BBBB0000-" },
    { LOG_AOA_SSA_MSG, sizeof(log_AOA_SSA),
      "AOA", "Qff", "TimeUS,AOA,SSA", "sdd", "F00" },
#if OPTFLOW == ENABLED
    { LOG_OPTFLOW_MSG, sizeof(log_Optflow),
      "OF",   "QBffff",   "TimeUS,Qual,flowX,flowY,bodyX,bodyY", "s-EEEE", "F-0000" },
#endif
    { LOG_PIQR_MSG, sizeof(log_PID), \
      "PIQR", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS },  \
    { LOG_PIQP_MSG, sizeof(log_PID), \
      "PIQP", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS }, \
    { LOG_PIQY_MSG, sizeof(log_PID), \
      "PIQY", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS }, \
    { LOG_PIQA_MSG, sizeof(log_PID), \
      "PIQA", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS }, \
    { LOG_AETR_MSG, sizeof(log_AETR), \
      "AETR", "Qhhhhh",  "TimeUS,Ail,Elev,Thr,Rudd,Flap", "s-----", "F-----" },  \

      //INDI Log
    { LOG_INDIV_MSG, sizeof(log_INDI_V),
      "INDV", "Qfffffffffff",  "TimeUS,V,d_V1,d_V2,d_V3,d_V4,ab1_x,ab1_y,ab1_z,ab2_x,ab2_y,ab2_z", "snoooooooooo", "F00000000000" },
    { LOG_INDICHI_MSG, sizeof(log_INDI_CHI),
      "INDC", "Qfffff",  "TimeUS,chi_1,chi_2,chi_3,d_chi1,d_chi2", "sdddkk", "F00000" },
    { LOG_INDIGAM_MSG, sizeof(log_INDI_GAM),
      "INDG", "Qffffff",  "TimeUS,gam_1,gam_2,gam_3,d_gam1,d_gam2,d_gam3", "sdddkkk", "F000000" },
    { LOG_INDIX0_MSG, sizeof(log_INDI_X0),
      "INX0", "Qffffff",  "TimeUS,X,Y,Z,X_ref,Y_ref,Z_ref", "smmmmmm", "F000000" },
    { LOG_INDIX1_MSG, sizeof(log_INDI_X1),
      "INX1", "Qffffff",  "TimeUS,V,chi,gam,V_ref,chi_ref,gam_ref", "snddndd", "F000000" },
    { LOG_INDIX2_MSG, sizeof(log_INDI_X2),
      "INX2", "Qffffff",  "TimeUS,mu,alpha,beta,mu_ref,alpha_ref,beta_ref", "sdddddd", "F000000" },
    { LOG_INDIX3_MSG, sizeof(log_INDI_X3),
      "INX3", "Qffffff",  "TimeUS,p,q,r,p_ref,q_ref,r_ref", "skkkkkk", "F000000" },
    { LOG_INDIX4_MSG, sizeof(log_INDI_X4),
      "INX4", "Qffff",  "TimeUS,aileron,elevator,rudder,thrust", "sddd-", "F0000" },
    { LOG_INDIWTH_MSG, sizeof(log_INDI_WATCH),
      "INWH", "Qffffff",  "TimeUS,watch1,watch2,watch3,watch4,watch5,watch6", "s------", "F000000" },
};

void Plane::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by DataFlash
    Log_Write_Startup(TYPE_GROUNDSTART_MSG);
    DataFlash.Log_Write_Mode(control_mode, control_mode_reason);
    DataFlash.Log_Write_Rally(rally);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_DataFlash_Log_Startup_messages();
}

/*
  initialise logging subsystem
 */
void Plane::log_init(void)
{
    DataFlash.Init(log_structure, ARRAY_SIZE(log_structure));
}

#else // LOGGING_ENABLED

void Plane::Log_Write_Attitude(void) {}
void Plane::Log_Write_Fast(void) {}
void Plane::Log_Write_Performance() {}
void Plane::Log_Write_Startup(uint8_t type) {}
void Plane::Log_Write_Control_Tuning() {}
void Plane::Log_Write_Nav_Tuning() {}
void Plane::Log_Write_Status() {}
void Plane::Log_Write_Sonar() {}

 #if OPTFLOW == ENABLED
void Plane::Log_Write_Optflow() {}
 #endif

void Plane::Log_Arm_Disarm() {}
void Plane::Log_Write_RC(void) {}
void Plane::Log_Write_Vehicle_Startup_Messages() {}

void Plane::log_init(void) {}

#endif // LOGGING_ENABLED
