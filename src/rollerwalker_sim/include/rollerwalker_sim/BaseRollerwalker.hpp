#pragma once

#include <vector>
#include <fstream>
#include <string>


//摩擦設定
#define VINYL_MU_t 0.0264	//ビニール床の接線方向の摩擦係数
#define VINYL_MU_n 0.417	//ビニール床の法線方向の摩擦係数
#define CARPET_MU_t 0.057	//カーペット上の接線方向の摩擦系数
#define CARPET_MU_n 0.61	//カーペット上の法線方向の摩擦系数

#define NUM_OF_LEG 4
//制御する脚の数

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
	BaseRollerwalker(double d_0, double theta_0, double omega, double phi,  double phi_fr, double center_z, bool is_rollerWalk);
    void calAndSetTheta(double t);	//各関節の値の計算、set
	//各脚の関節の値のゲッター
	//左前脚
	double getTheta1LF();	//関節1
	double getTheta2LF();	//関節2
	double getTheta3LF();	//関節3
	double getTheta4LF();	//関節4
	//左後ろ脚
	double getTheta1LR();	//関節1
	double getTheta2LR();	//関節2
	double getTheta3LR();	//関節3
	double getTheta4LR();	//関節4
	//右後ろ脚
	double getTheta1RR();	//関節1
	double getTheta2RR();	//関節2
	double getTheta3RR();	//関節3
	double getTheta4RR();	//関節4
	//右前脚
	double getTheta1RF();	//関節1
	double getTheta2RF();	//関節2
	double getTheta3RF();	//関節3
	double getTheta4RF();	//関節4

private:
	//定数
	const double d_ofset = 0.235; //[m]

	//前脚の脚軌道関数
    double d_front(double t, double d_0, double omega);				//法線方向の脚軌道関数(パラメータ可
	double theta_front(double t, double theta_0, double omega);		//接線方向の脚軌道関数(パラメータ可変)
	double dotD_front(double t, double d_0, double omega);			//接線方向の脚軌道関数の一階微分(パラメータ可変)
	double dotTheta_front(double t, double theta_0,double omega );	//接線方向の脚軌道関数の一階微分(パラメータ可変)

	//後ろ足の脚軌道関数
	double d_rear(double t, double d_0, double omega);				//法線方向の脚軌道関数(パラメータ可変)
	double theta_rear(double t, double theta_0, double omega);		//接線方向の脚軌道関数(パラメータ可変)
	double dotD_rear(double t, double d_0, double omega);			//接線方向の脚軌道関数の一階微分(パラメータ可変)
	double dotTheta_rear(double t, double theta_0, double omega);	//接線方向の脚軌道関数の一階微分(パラメータ可変)

	//パラメータ適応則
	void thetaAdaption(double v_d);	//θ_0適応則
	double empricalFormula();			//実験式g(θ)
	void nominalOmegaAdaption(double v_d);	//ノミナルω適応則
	void ActualOmegaAdaption(double v_d);	//実測ω適応則

	double frontAcceleration(double t);//前方向の加速度
	double sideAcceleration(double t);	//横方向の加速度

	
    //関節角の計算    
    double calTheta2(double target_d);
    double calTheta3(double target_d,double theta2);
    double calTheta4(double theta2,double theta3,bool is_rollerwalk);
	double sign(double input);		//符号関数
    
	//ローラウォーカの4つのパラメータ
	double d_0;					//法線方向の振幅
	double theta_0;				//接線方向の振幅
	double omega;				//脚軌道の周期関数の角速度
	double phi;					//接線方向と法線方向の正弦波の位相差
	double phi_fr;				//前後の脚の位相差(後ろ足の周期関数に使用)
	
    //ローラウォーカの姿勢
    double center_z;                   //ローラウォーカー本体の高さ
    double target_z;            //ローラウォーカの第四関節の終端の高さ(モードによってホイールの高さ（厚み)が変わるため）

    //その他パラメータ
	double v_front;					//ローラウォーカの前方速度
	double v_side;					//ローラウォーカの前方速度
	
	double mu_t;					//受動車輪に働く接線方向の摩擦係数
	double mu_n;					//受動車輪に働く法線方向の摩擦係数
	
	double t;						//時刻
	bool is_rollerWalk;				//ローラウォーク(true)か歩行か(false)

    //ローラウォーカの定数
    const double l1 = 0.045;        //第一関節から第二関節まで
    const double l2 = 0.155;        //第二関節から第三関節まで
    const double l3 = 0.243;        //第三関節から足先まで
    const double l4 = 0.031;        //リンク4の長さ(円の半径+ホイール側面まで)
    const double WHEEL_THICKNESS = 0.024;   //タイヤの厚み
    const double WHEEL_RADIUS = 0.035;       //タイヤの半径
    const double LINK_2_LENGTH = l1+l2;                             //第二リンクの長さ
    const double LINK_3_LENGTH = l3  -WHEEL_THICKNESS - l4;         //第三リンクの長さ(Y方向)

	//各脚の関節の関数
	//左前脚
	double theta_1_lf;
	double theta_2_lf;
	double theta_3_lf;
	double theta_4_lf;
	//左後ろ脚
	double theta_1_lr;
	double theta_2_lr;
	double theta_3_lr;
	double theta_4_lr;
	//右後ろ脚
	double theta_1_rr;
	double theta_2_rr;
	double theta_3_rr;
	double theta_4_rr;
	//右前脚
	double theta_1_rf;
	double theta_2_rf;
	double theta_3_rf;
	double theta_4_rf;
	
};