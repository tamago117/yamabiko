#pragma once

class PID{
    public:

        //コンストラクタ
        //kp:比例定数
        //ki:積分定数
        //kd:微分定数
        PID(double kp_, double ki_, double kd_);
        PID(){};

        void set(double kp_, double ki_, double kd_);
        double update(double local_val, double target_val, double dt);
        double result_val();
        void reset_i();

    private:
        double kp, ki, kd;
        double dt;
        double res_p;
        double res_i = 0;
        double iii = 0;
        double res_d;
        double res_prep = 0;
        double result_value;
        unsigned long pretime;

};

PID::PID(double kp_, double ki_, double kd_) : kp(kp_), ki(ki_), kd(kd_)
{

}

void PID::set(double kp_, double ki_, double kd_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

double PID::update(double local_val, double target_val, double dt)
{
    res_p = local_val - target_val;
    //res_i = res_i + (res_p*dt);
    res_d = (res_p-res_prep)/dt;

    res_prep = res_p;

    //result_value = kp*res_p + ki*res_i + kd*res_d;
    result_value = kp*res_p + kd*res_d;
    return result_value;
}

void PID::reset_i()
{
    res_i = 0;
}

double PID::result_val()
{
    return result_value;
}
