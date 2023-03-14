#include "robot_planning/robot_planning.h" //注意替换头文件位置



string arm_joint_names[7]={"joint1","joint2","jointl3","jointl4","joint5","joint6","joint7"};


//手臂关节驱动函数
void robotPositionController(ros::Publisher &pub, VectorXd qd)
{

  baxter_core_msgs::JointCommand msg;
  msg.mode = 5;
  msg.command.resize(7);
  for(int i=0;i<7;i++)
  {
    msg.command[i] = qd(i);
  }
  pub.publish(msg);
}



//三次多项式函数
double triple(double x0,double x1,double v0,double v1,double T,double t)
{
    double a,b,c,d,y;
    if(t<=T)
    {
        a=(2*x0-2*x1+v0*T+v1*T)/T/T/T;
        b=(-3*x0+3*x1-2*v0*T-v1*T)/T/T;
        c=v0;
        d=x0;
        y=a*t*t*t+b*t*t+c*t+d;
    }
    else
    {
        y=x1;
    }
    return y;
}

//Hank：2点带速度运动规划函数
VectorXd motionP2P_v2v(VectorXd q_init, VectorXd q_goal, VectorXd v_init, VectorXd v_goal, double duration,  double t)
{
    VectorXd q_t(q_init.size()); //对应t时间的关节角
    // triple 路径规划
    for(int i=0;i<q_init.size();i++)
    {
        q_t(i) = triple(q_init(i), q_goal(i), v_init(i), v_goal(i), duration, t);
    }

    return q_t;
}



// Hank 20200115
// 多点速度连续轨迹规划，起点、终点速度为0
// waypoints(N,7)是路径点，N*7矩阵, N表示第几个路径点，7表示7个关节
// t_是各段路径段时间，是N-1列向量(N个点只对应N-1段路径，只需要N-1个时间)
VectorXd multipoints_trajectory_plan(MatrixXd waypoints, VectorXd t_, double t)
{
    VectorXd q_t(7);  //存储最终计算出的t时刻关节角度的内部变量
    //首先判断输入矩阵是否合法：路径点个数应比路径段时间个数多1
    //注意：VectorXd是列向量
    int n = waypoints.rows(); //n为路径点个数
    int m = t_.rows();        //m为路径段时间个数
    if(m != n-1)
    {
        std::cout << "路径点个数和路径时间段数不匹配！" << std::endl;
        //summer20211029异常退出程序
        exit(EXIT_FAILURE);
        exit(1);
        //return 1;
    }
    else  //
    {
        //求总的时间长度
        double T = 0;
        T = t_.sum();  //T是各段时间之和

        // 判断t是否超过总时间（存在测量误差）
        if(t<T) //没超过正常计算
        {
            // 计算每一段路径的斜率k（也就是路径平均速度）
            MatrixXd k(m,7);
            for(int i=0; i<m; i++) //针对路径段的迭代
            {
                k.row(i) = (waypoints.row(i+1)-waypoints.row(i)) / t_(i);
            }

            // 计算路径点处的速度v
            MatrixXd v(n,7);
            v.row(0) << 0,0,0,0,0,0,0;
            v.row(n-1) << 0,0,0,0,0,0,0; //起点和终点速度为0
            for(int i=1; i<n-1; i++) //针对路径点的迭代，排除掉第一点和最后一点
            {
                for(int j=0; j<7; j++) //针对7个关节的迭代
                {
                    if(k(i-1,j)*k(i,j) > 0) //%当路径点前后两端斜率方向一致时(相乘同号)取平均速度
                    {
                        v(i,j) = (k(i-1,j)+k(i,j)) / 2;
                    }
                    else  //当路径点前后两端斜率方向不一致时取0
                    {
                        v(i,j) = 0;
                    }
                }
            }

            // 判断当前t属于哪一段区间
            double T_sum, last_T_sum = 0;
            for(int i=0; i<m; i++) //针对路径段的迭代
            {
                last_T_sum = T_sum; //用来被t减，确定当前总t在当前这一段时间内的部分t
                T_sum = T_sum + t_(i); //时间段依次累计和

                if(t < T_sum) //说明t在第i段内，若不在则继续循环判断
                {
                    //                       起始点             终止点         起始v      终止v  该段总时间 t在该段中的位置
                    q_t = motionP2P_v2v(waypoints.row(i), waypoints.row(i+1), v.row(i), v.row(i+1), t_(i), t-last_T_sum);
                    break;
                }
            }
        }

        else //超过了则取路径点最后最后一行的关节角
        {
            //注意：虽然q_t是列向量，但是并不需要把waypoints的行先转置再赋值，这是Eigen的特性
            q_t = waypoints.row(n-1);
        }

        return q_t;
    }
}

