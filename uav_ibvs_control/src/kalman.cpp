// DKF卡尔曼实现
#include "iostream"
#include "Eigen/Dense"
#include "random"

using namespace std;
class KF
{ 
    public:
    KF(Eigen::MatrixXd A,Eigen::MatrixXd B,Eigen::MatrixXd C,Eigen::MatrixXd Q,Eigen::MatrixXd R,Eigen::MatrixXd P,Eigen::MatrixXd X)
    {
        
    }
    void kf_init();
    void kf_update();
    void kf_predict();
    private:
    {
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::MatrixXd C;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        Eigen::MatrixXd P;
        Eigen::MatrixXd X;
    }
};

class DKF
{

};