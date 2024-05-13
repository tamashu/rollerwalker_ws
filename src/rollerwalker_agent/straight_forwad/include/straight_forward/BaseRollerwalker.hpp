#pragma once

#include <vector>
#include <fstream>
#include <string>


//摩擦設定
#define VINYL_MU_t 0.0264	//ビニール床の接線方向の摩擦係数
#define VINYL_MU_n 0.417	//ビニール床の法線方向の摩擦係数
#define CARPET_MU_t 0.057	//カーペット上の接線方向の摩擦系数
#define CARPET_MU_n 0.61	//カーペット上の法線方向の摩擦系数

#define NUM_OF_LEG 4		//制御する脚の数


//パラメータ適応則の定数
#define TOL 0.1				//目標速度の許容値
#define THETA_MAX_0 0.4		//θ_0の最大値
#define THETA_MIN_0 0.2		//θ_0の最小値
#define K_THETA_0	1.0		//θ_0のゲイン
#define K_OMEGA_d	10.0	//ωのゲイン
#define	K_V			50.0		//Vのゲイン
#define DELTA_T		0.01	//時間の刻み幅
#define TARGET_V	2.0		//目標速度


/// <summary>
///		脚の番号の割り振り
///		lf				rf
///		1---------------0
///		|				|
///		|				|	
/// 	|				|
/// 	|				|
///		|				|	
/// 	|				|
/// 	|				|
///		2---------------3
///		lr				rr	
/// 
/// </summary>

class BaseRollerwalker
{
public:
	BaseRollerwalker(double phi, double l2, double l3, double l4, double wheel_radius, double wheel_thickenss);

	//脚軌道関数(前はphi_fr=0, 後ろはphi_fr=pi/2)
	double calD(double t, double d_0, double omega, double phi_fr);			
	double calTheta(double t, double theta_0, double omega, double phi_fr, double steering_ofset);

	//関節角の計算    
    double calTheta2(double target_d, double center_z, bool is_rollerwalk);
    double calTheta3(double target_d,double theta2, double center_z, bool is_rollerwalk);
    double calTheta4(double theta2,double theta3,bool is_rollerwalk);

	//ローラウォーカの定数
    const double l2_ = 0.20;        //第二関節から第三関節まで
    const double l3_ = 0.188;        //第三関節から足先まで
    const double l4_ = 0.031;        //リンク4の長さ(円の半径+ホイール側面まで)
    const double WHEEL_THICKNESS = 0.024;   //タイヤの厚み
    const double WHEEL_RADIUS = 0.035;       //タイヤの半径
private:
	//定数
	const double d_ofset = 0.235; //[m]
	const double phi_;	//法線方向と接線方向の位相差

	//パラメータ適応則
	double thetaAdaption(double v_d, double theta_0);	//θ_0適応則
	double empricalFormula(double theta_0);			//実験式g(θ)
	void nominalOmegaAdaption(double v_d,double theta_0);	//ノミナルω適応則
	void ActualOmegaAdaption(double v_d, double theta_0, double current_v);	//実測ω適応則

};