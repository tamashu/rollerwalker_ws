#include <cmath>
#include <iostream>
#include "rollerwalker_sim/BaseRollerwalker.hpp"


#define PI 3.1415926535897932384626433832795

BaseRollerwalker::BaseRollerwalker(double d_0, double theta_0, double omega, double phi,double phi_fr,double center_z, bool is_rollerWalk) {
	this->d_0_ = d_0;

	theta_0_lf_ = theta_0;
	theta_0_lr_ = theta_0;
	theta_0_rr_ = theta_0;
	theta_0_rf_ = theta_0;

	steering_ofset_lf_ = 0.0;
	steering_ofset_lr_ = 0.0;
	steering_ofset_rr_ = 0.0;
	steering_ofset_rf_ = 0.0;

	this->omega = omega;
	this->phi = phi;
	this->phi_fr = phi_fr;
	
	this->v_front = 0;
	this->v_side = 0;

	this->mu_t = VINYL_MU_t;
	this->mu_n = VINYL_MU_n;
	/*this->mu_t = CARPET_MU_t;
	this->mu_n = CARPET_MU_n;*/
	/*this->mu_t = 0.001;
	this->mu_n = 10;*/
	this->t = 0;
    this->is_rollerWalk = is_rollerWalk;


	this->front_center_z_ = center_z;
	this->back_center_z_ = center_z;
}
double BaseRollerwalker::getOmega(){	//omegaのゲッタ
	return omega;
}
//各脚のゲッター
//左前脚
double BaseRollerwalker::getTheta1LF_(){
	return theta_1_lf_;
}
double BaseRollerwalker::getTheta2LF_(){
	return theta_2_lf_;
}
double BaseRollerwalker::getTheta3LF_(){
	return theta_3_lf_;
}
double BaseRollerwalker::getTheta4LF_(){
	return theta_4_lf_;
}
//左後ろ脚
double BaseRollerwalker::getTheta1LR_(){
	return theta_1_lr_;
}
double BaseRollerwalker::getTheta2LR_(){
	return theta_2_lr_;
}
double BaseRollerwalker::getTheta3LR_(){
	return theta_3_lr_;
}
double BaseRollerwalker::getTheta4LR_(){
	return theta_4_lr_;
}
//右後ろ脚
double BaseRollerwalker::getTheta1RR_(){
	return theta_1_rr_;
}
double BaseRollerwalker::getTheta2RR_(){
	return theta_2_rr_;
}
double BaseRollerwalker::getTheta3RR_(){
	return theta_3_rr_;
}
double BaseRollerwalker::getTheta4RR_(){
	return theta_4_rr_;
}
//右前脚
double BaseRollerwalker::getTheta1RF_(){
	return theta_1_rf_;
}
double BaseRollerwalker::getTheta2RF_(){
	return theta_2_rf_;
}
double BaseRollerwalker::getTheta3RF_(){
	return theta_3_rf_;
}
double BaseRollerwalker::getTheta4RF_(){
	return theta_4_rf_;
}
//重心位置のセッター
void BaseRollerwalker::setFrontCenterZ(double center_z){
	front_center_z_ = center_z;
}
void BaseRollerwalker::setBackCenterZ(double center_z){
	back_center_z_ = center_z;
}
// theta_0のセッター
void BaseRollerwalker::setTheta_0_lf_(double theta_0_lf){
	theta_0_lf_ = theta_0_lf;
}
void BaseRollerwalker::setTheta_0_lr_(double theta_0_lr){
	theta_0_lr_ = theta_0_lr;
}
void BaseRollerwalker::setTheta_0_rr_(double theta_0_lf){
	theta_0_rr_ = theta_0_lf;
}
void BaseRollerwalker::setTheta_0_rf_(double theta_0_lf){
	theta_0_rf_ = theta_0_lf;
}
//steering_ofsetのセッター
void BaseRollerwalker::setSteering_ofset_lf_(double steering_ofset){
	steering_ofset_lf_ = steering_ofset;
}
void BaseRollerwalker::setSteering_ofset_lr_(double steering_ofset){
	steering_ofset_lr_ = steering_ofset;
}
void BaseRollerwalker::setSteering_ofset_rr_(double steering_ofset){
	steering_ofset_rr_ = steering_ofset;
}
void BaseRollerwalker::setSteering_ofset_rf_(double steering_ofset){
	steering_ofset_rf_ = steering_ofset;
}

void BaseRollerwalker::calAndSetTheta(double t){
	this->t = t;
	//脚軌道に必要なパラメータの計算
	double d_lf  =d_front(t,d_0_,omega);   //左前脚の長さ  
	double d_lr  =d_rear(t,d_0_,omega);   //左後ろ脚の長さ  
	double d_rr  =d_rear(t,d_0_,omega);   //右後ろ脚の長さ  
	double d_rf  =d_front(t,d_0_,omega);   //右前脚の長さ

	if(is_rollerWalk){	//タイヤの中心がdとなるための計算
		d_lf = d_lf - l4_ - WHEEL_THICKNESS/2;
		d_lr = d_lr - l4_ - WHEEL_THICKNESS/2;
		d_rr = d_rr - l4_ - WHEEL_THICKNESS/2;
		d_rf = d_rf - l4_ - WHEEL_THICKNESS/2;
	}

	// 左前脚
	theta_1_lf_ = theta_front(t,theta_0_lf_,omega,steering_ofset_lf_);
	theta_2_lf_ = calTheta2(d_lf,front_center_z_);
	theta_3_lf_ = calTheta3(d_lf,theta_2_lf_,front_center_z_) + PI/2;
	theta_4_lf_ = calTheta4(theta_2_lf_,theta_3_lf_,is_rollerWalk);
	//左後ろ脚
	theta_1_lr_ = theta_front(t,theta_0_lr_,omega,steering_ofset_lf_);
	theta_2_lr_ = calTheta2(d_lr,back_center_z_);
	theta_3_lr_ = calTheta3(d_lr,theta_2_lr_,back_center_z_)+ PI/2;
	theta_4_lr_ = calTheta4(theta_2_lr_,theta_3_lr_,is_rollerWalk);
	//右後ろ脚
	theta_1_rr_ = theta_front(t,theta_0_rr_,omega,steering_ofset_lf_) ;
	theta_2_rr_ = calTheta2(d_rr,back_center_z_);
	theta_3_rr_ = calTheta3(d_rr,theta_2_rr_,back_center_z_)+ PI/2;
	theta_4_rr_ = calTheta4(theta_2_rr_,theta_3_rr_,is_rollerWalk);
	//右前脚
	theta_1_rf_ = theta_front(t,theta_0_rf_,omega,steering_ofset_lf_);
	theta_2_rf_ = calTheta2(d_rf,front_center_z_);
	theta_3_rf_ = calTheta3(d_rf,theta_2_rf_,front_center_z_)+ PI/2;
	theta_4_rf_ = calTheta4(theta_2_rf_,theta_3_rf_,is_rollerWalk);
	// theta_1_lf = theta_front(t,theta_0,omega);
	// theta_2_lf = calTheta2(d_lf);
	// theta_3_lf = calTheta3(d_lf,theta_2_lf) + PI/2;
	// theta_4_lf = calTheta4(theta_2_lf,theta_3_lf,is_rollerWalk);
	// //左後ろ脚
	// theta_1_lr_ = theta_front(t,theta_0,omega);
	// theta_2_lr_ = calTheta2(d_lr);
	// theta_3_lr_ = calTheta3(d_lr,theta_2_lr_)+ PI/2;
	// theta_4_lr_ = calTheta4(theta_2_lr_,theta_3_lr_,is_rollerWalk);
	// //右後ろ脚
	// theta_1_rr_ = theta_front(t,theta_0,omega) ;
	// theta_2_rr_ = calTheta2(d_rr);
	// theta_3_rr_ = calTheta3(d_rr,theta_2_rr_)+ PI/2;
	// theta_4_rr_ = calTheta4(theta_2_rr_,theta_3_rr_,is_rollerWalk);
	// //右前脚
	// theta_1_rf_ = theta_front(t,theta_0,omega);
	// theta_2_rf_ = calTheta2(d_rf);
	// theta_3_rf_ = calTheta3(d_rf,theta_2_rf_)+ PI/2;
	// theta_4_rf_ = calTheta4(theta_2_rf_,theta_3_rf_,is_rollerWalk);

	//std::cout << "時刻t: " << t << "  d_lf: " << d_lf << std::endl;
	//std::cout << "時刻t: " << t << " omega: "<<omega<<"  d_f:" << d_0_ * (sin(omega * t + 3 * PI / 2) + 1) << std::endl;
	
}

// 前脚の脚軌道関数
double BaseRollerwalker::d_front(double t, double d_0, double omega) {
	double ret = d_ofset + d_0 * (sin(omega * t + 3 * PI / 2) + 1);
	//double ret = d_ofset + d_0 * (sin(omega * t) + 1);
	return ret;
}
double BaseRollerwalker::theta_front(double t, double theta_0, double omega,double steering_ofset) {
	double ret = -theta_0 * sin(omega * t + 3 * PI / 2 + phi) + steering_ofset;
	//double ret = -theta_0 * sin(omega * t+phi);
	return ret;
}
double BaseRollerwalker::dotD_front(double t,double d_0,double omega) {
	double ret = d_0 * omega * cos(omega * t + 3 * PI / 2);
	//double ret = d_0 * omega * cos(omega * t);
	return ret;
}
double BaseRollerwalker::dotTheta_front(double t,double theta_0,double omega) {
	double ret = -theta_0 * omega * cos(omega * t + 3 * PI / 2 + phi);
	//double ret = -theta_0 * omega * cos(omega * t + phi);
	return ret;
}

// 後ろ脚の脚軌道関数
double BaseRollerwalker::d_rear(double t, double d_0, double omega) {
	double ret = d_ofset + d_0 * (sin(omega * t + 3 * PI / 2+phi_fr) + 1);
	//double ret = d_ofset + d_0 * (sin(omega * t  + phi_fr) + 1);
	return ret;
}
double BaseRollerwalker::theta_rear(double t, double theta_0, double omega, double steering_ofset) {
	double ret = -theta_0 * sin(omega * t + 3 * PI / 2 + phi+ phi_fr) + steering_ofset;
	//double ret = -theta_0 * sin(omega * t  + phi+ phi_fr);
	return ret;
}
double BaseRollerwalker::dotD_rear(double t, double d_0, double omega) {
	double ret = d_0 * omega * cos(omega * t + 3 * PI / 2+ phi_fr);
	//double ret = d_0 * omega * cos(omega * t  + phi_fr);
	return ret;
}
double BaseRollerwalker::dotTheta_rear(double t, double theta_0, double omega) {
	double ret = -theta_0 * omega * cos(omega * t + 3 * PI / 2 + phi+ phi_fr);
	//double ret = -theta_0 * omega * cos(omega * t +  phi + phi_fr);
	return ret;
}

//θ適応則
void BaseRollerwalker::thetaAdaption(double v_d) {
	double tol = TOL;
	double theta_target_0;

	if (abs(v_d) > tol) {//加速が必要なときはθ_0は大きくなる
		theta_target_0 = THETA_MAX_0;
	}
	else {
		theta_target_0 = THETA_MIN_0;
	}

	theta_0_lf_ = theta_0_lf_ - K_THETA_0 * (theta_0_lf_ - theta_target_0);
	theta_0_lr_ = theta_0_lr_ - K_THETA_0 * (theta_0_lr_ - theta_target_0);
	theta_0_rr_ = theta_0_rr_ - K_THETA_0 * (theta_0_rr_ - theta_target_0);
	theta_0_rf_ = theta_0_rf_ - K_THETA_0 * (theta_0_rf_ - theta_target_0);
}

double  BaseRollerwalker::empricalFormula() {//g(θ)
	double ret = 0;
	const double a_0 = 1.256;
	const double a_1 = -7.349;
	const double a_2 = 18.556;
	const double a_3 = -16.947;

	const double a_k[] = { a_0,a_1,a_2,a_3 };
	for (int k = 0; k <= 3; k++) {
		ret += a_k[k] * pow(theta_0_lf_, k);
	}

	return ret;
}

//ω適応則
void BaseRollerwalker::nominalOmegaAdaption(double v_d) {
	double omega_d = v_d / empricalFormula();	//ω_d = V_d/g(θ_0)
	double dot_omega = - K_OMEGA_d * (omega - omega_d);
	
	omega += dot_omega * DELTA_T; 
}

void BaseRollerwalker::ActualOmegaAdaption(double v_d) {
	double omega_d = v_d / empricalFormula();	//ω_d = V_d/g(θ_0)
	double dot_omega = -K_OMEGA_d * (omega - omega_d) - K_V * (v_front - v_d);

	omega += dot_omega * DELTA_T; //K_V=0ならノミナル適応速
}

//符号関数
double  BaseRollerwalker::sign(double input){
	if (input > 0) return 1;
	else if (input < 0) return -1;
	else return 0;
}

double BaseRollerwalker::calTheta2(double target_d, double center_z) {
	double target_z;
	if (is_rollerWalk) {//ローラウォーク時
		target_z = WHEEL_RADIUS;
	}
	else {//歩行時
		target_z =l4_ +  WHEEL_THICKNESS;
	}

	double numerator = pow(target_d, 2) + pow((target_z - center_z), 2) + pow(l2_, 2) - pow(l3_, 2);
	double denominator = 2 * l2_ * sqrt(pow(target_d, 2) + pow(target_z - center_z, 2));

	double ret = acos(numerator / denominator) + atan((target_z - center_z) /target_d ); //atan要チェック (acos2は[0,pi])

	return ret;
}

double BaseRollerwalker::calTheta3(double target_d, double theta2,double center_z) {
	double target_z;
	if (is_rollerWalk) {//ローラウォーク時
		target_z = WHEEL_RADIUS;
	}
	else {//歩行時
		target_z =l4_ +  WHEEL_THICKNESS;
	}
	double numerator = (target_z - center_z) - l2_ * sin(theta2);
	double denominator = target_d - l2_ * cos(theta2);
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



