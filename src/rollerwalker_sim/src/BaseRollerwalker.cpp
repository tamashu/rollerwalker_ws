#include <cmath>
#include <iostream>
#include "rollerwalker_sim/BaseRollerwalker.hpp"


#define PI 3.1415926535897932384626433832795

BaseRollerwalker::BaseRollerwalker(double phi,double l2, double l3, double l4, double wheel_radius, double wheel_thickenss)
					:phi_(phi),l2_(l2),l3_(l3),l4_(l4),WHEEL_THICKNESS(wheel_thickenss),WHEEL_RADIUS(wheel_radius) 
{

}

double BaseRollerwalker::calD(double t, double d_0, double omega, double phi_fr){
	double ret = d_ofset + d_0 * (sin(omega * t + 3 * PI / 2+phi_fr) + 1);
	return ret;
}
double BaseRollerwalker::calTheta(double t, double theta_0, double omega, double phi_fr, double steering_ofset){
	double ret = -theta_0 * sin(omega * t + 3 * PI / 2 + phi_+ phi_fr) + steering_ofset;
	return ret;
};

//θ適応則
double  BaseRollerwalker::thetaAdaption(double v_d, double theta_0) {
	double tol = TOL;
	double theta_target_0;

	if (abs(v_d) > tol) {//加速が必要なときはθ_0は大きくなる
		theta_target_0 = THETA_MAX_0;
	}
	else {
		theta_target_0 = THETA_MIN_0;
	}

	double ret = theta_0 - K_THETA_0 * (theta_0 - theta_target_0);
	return ret;
}

double  BaseRollerwalker::empricalFormula(double theta_0) {//g(θ)
	double ret = 0;
	const double a_0 = 1.256;
	const double a_1 = -7.349;
	const double a_2 = 18.556;
	const double a_3 = -16.947;

	const double a_k[] = { a_0,a_1,a_2,a_3 };
	for (int k = 0; k <= 3; k++) {
		ret += a_k[k] * pow(theta_0, k);
	}

	return ret;
}

//ω適応則
// void BaseRollerwalker::nominalOmegaAdaption(double v_d, double theta_0) {
// 	double omega_d = v_d / empricalFormula(theta_0);	//ω_d = V_d/g(θ_0)
// 	double dot_omega = - K_OMEGA_d * (omega - omega_d);
	
// 	omega += dot_omega * DELTA_T; 
// }

// void BaseRollerwalker::ActualOmegaAdaption(double v_d, double current_v,double theta_0) {
// 	double omega_d = v_d / empricalFormula(theta_0);	//ω_d = V_d/g(θ_0)
// 	double dot_omega = -K_OMEGA_d * (omega - omega_d) - K_V * (current_v - v_d);

// 	omega += dot_omega * DELTA_T; //K_V=0ならノミナル適応速
// }

double BaseRollerwalker::calTheta2(double target_d, double center_z, bool is_rollerwalk) {
	double target_z;
	double x_3;	//第3リンクの位置（正味のd)
	if (is_rollerwalk) {//ローラウォーク時
		target_z = WHEEL_RADIUS;
		x_3 = target_d - l4_ - WHEEL_THICKNESS/2;
	}
	else {//歩行時
		target_z =l4_ +  WHEEL_THICKNESS;
		x_3 = target_d;
	}

	double numerator = pow(x_3, 2) + pow((target_z - center_z), 2) + pow(l2_, 2) - pow(l3_, 2);
	double denominator = 2 * l2_ * sqrt(pow(x_3, 2) + pow(target_z - center_z, 2));

	double ret = acos(numerator / denominator) + atan((target_z - center_z) /x_3 ); //atan要チェック (acos2は[0,pi])

	return ret;
}

double BaseRollerwalker::calTheta3(double target_d, double theta2,double center_z, bool is_rollerwalk) {
	double target_z;
	double x_3;	//第3リンクの位置（正味のd)
	if (is_rollerwalk) {//ローラウォーク時
		target_z = WHEEL_RADIUS;
		x_3 = target_d - l4_ - WHEEL_THICKNESS/2;
	}
	else {//歩行時
		target_z =l4_ +  WHEEL_THICKNESS;
		x_3 = target_d;
	}
	double numerator = (target_z - center_z) - l2_ * sin(theta2);
	double denominator = x_3 - l2_ * cos(theta2);
	double ret = atan(numerator/denominator) - theta2 ; //atan要チェック

	return ret;
}

double BaseRollerwalker::calTheta4(double theta2,double theta3,bool is_rollerwalk) {
	double ret;
	if (is_rollerwalk) { //ローラウォーク時
		ret = - theta2 - theta3+ PI/2;
	}
	else {	//歩行時
		ret = - theta2 - theta3 ;
	}
	
	return ret;
}



