#ifndef _FDCL_COMMON_TYPE_H
#define _FDCL_COMMON_TYPE_H


typedef Eigen::Matrix<double, 3, 3> Matrix3;
typedef Eigen::Matrix<double, 3, 1> Vector3;
typedef Eigen::Matrix<double, 4, 1> Vector4;
typedef Eigen::Matrix<double, 6, 6> Matrix6;
typedef Eigen::Matrix<double, 6, 1> Vector6;
typedef Eigen::Matrix<double, 9, 9> Matrix9;
typedef Eigen::Matrix<double, 12, 12> Matrix12;
typedef Eigen::Matrix<double, 15, 15> Matrix15;

struct UAV_state_type {
// copied from fdcl_EKF.h
	double t, t_IMU, t_VICON;
	double t_pre, t_IMU_pre, t_VICON_pre;
	double dt, dt_IMU, dt_VICON;

	// UAV state
	Vector3	x, v, a, a_pre, W, W_pre;
	Matrix3 R, R_pre;
	Vector3 b_a, b_a_pre, b_W, b_W_pre;  // acceleration, gyro bias
	Matrix15 P; // Cov([x,v,eta,b_a,b_W])

	// IMU measurements
	Vector3 a_i, W_i, a_i_pre, W_i_pre;
	Matrix3 R_ni;
	Vector3 a_f_IMU; // a_i coordinate transform to the f-frame, a
	Vector3 W_b_IMU; // W_i coordinate transform to the b-frame, W
	Matrix3 R_fb_IMU; // R_ni coordinate transform to R_fb
	// IMU covariance
	Matrix3 Q_W;  // IMU covarirance for W_i noise
	Matrix3 Q_a;  // IMU covariance for a_i noise
	Matrix3 V_R_ni;  // IMU covariance for R_ni (eta)
	Matrix3 Q_b_a;  // covariance for b_a_dot
	Matrix3 Q_b_W; // covariance for b_W_dot

	// VICON measurements
	Vector3 x_v;
	Vector4 q_v;
	Matrix3 R_vm;
	Vector3 x_f_VICON; // x_v coordinate transform to the f-frame, x
	Matrix3 R_fb_VICON; // R_vm coordinate transformed to R_fb
	// VICON covariance
	Matrix3	V_x; // VICON covariance for x_v noise
	Matrix3 V_R_vm; // VICON cobariance for R_vm noise

	// GPS measurements
	Vector3 llh; // GPS location in lat, lon, h
	Vector3 llh_prev; // prev GPS location
	float S_llh_h; // horizontal position estimated standard deviation [m]
	float S_llh_v; // vertical position estimated standard deviation [m]

	Vector3 rtk_x; // GPS position in meters
	float S_rtk_x_h; // baseline horizontal estimated standard deviation [m]
  float S_rtk_x_v;  // baseline vertical estimated standard deviation [m]

	Matrix3 rtk_v; // GPS velocity
	float S_rtk_v_h; // velocity horizontal estimated standard deviation [m]
  float S_rtk_v_v; // velocity vertical estimated standard deviation [m]

	float utc; // UTC time
	int status; //RTK status
	int sats; // No. of available sattelites

	// Calibration data
	Matrix3 R_nv, R_mi, R_fv, R_bi;
	double g_ave;
};

struct COMMAND_type {
	double t;
	Vector3 Wd, Wd_dot, Wd_2dot;
	Matrix3 Rd;
	Vector3 xd, xd_dot, xd_2dot;
};


#endif
