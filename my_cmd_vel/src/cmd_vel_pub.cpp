#include "my_cmd_vel/cmd_vel_pub.h"


controller::controller(ros::NodeHandle *nh_):nh(nh_){

    odom_subscriber = nh->subscribe("/odom", 1, &controller::messageCallback, this);
    lidar_subscriber=nh->subscribe("/lidar_object",10,&controller::lidarCallback,this);
    cmd_vel_publisher = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);

    lstate_pub = nh->advertise<std_msgs::String>("/led_state",10);

    gain_x = 0.4;
    gain_y =0.4;
    c = 0.214;
    flag = false;
    is_goal=false;
    velocity = 0;
    steer_angle = 0;
    distance = 0;

    pre_time = 0;
    pre_x = 0;
    pre_y =0;

    lstate.data="RED";

}

// lidar call back
void controller::lidarCallback(const sensor_msgs::PointCloud2 &msg)
{
    pcl::PointCloud<pcl::PointXYZ> output_pointcloud;
    pcl::fromROSMsg(msg,output_pointcloud);
    if(!(output_pointcloud.points.size()==0)){

     goal_x =  output_pointcloud.points.at(0).x;
     goal_y = output_pointcloud.points.at(0).y;
     flag =true;



    distance = sqrt((goal_x*goal_x)+(goal_y*goal_y));
    // distance more than 1.3 -> stop
    if (distance <= 1.3)
     {
         is_goal = true;
         std::cout<<"distance to goal"<<distance<<std::endl;
         lstate.data="RED";
         lstate_pub.publish(lstate);

     }
     else
     {
         is_goal=false;
         std::cout<<"distance to goal"<<distance<<std::endl;
         lstate.data="GREEN";
         lstate_pub.publish(lstate);
     }
    }
}

void controller::messageCallback(const nav_msgs::Odometry &msg){

    ang_z = 0;
    pos_x = 0;
    pos_y =0;


    if ((flag == true &&is_goal==false) )
    {
        geometry_msgs::Twist cmd_vel; // 속도 파라미터


        error_x = goal_x - pos_x;
        error_y = goal_y - pos_y;

        if(distance > 3.0)
        {
            gain_x = 0.6;
            gain_y = 0.6;

            velocity = cos(ang_z) * gain_x * error_x + sin(ang_z) * gain_y * error_y;
            steer_angle = 1/c*(-sin(ang_z) * gain_x * error_x + cos(ang_z) * gain_y * error_y);


            if(velocity>1.0)
            {
                velocity = 1.0;
            }
            else if(velocity<0)
            {
                velocity = 0;
            }
            if(steer_angle>0.2)
            {
                steer_angle =0.2;
            }
            else if(steer_angle<-0.2)
            {
                steer_angle = -0.2;
            }
            // lstate.data="BLUE";
            // lstate_pub.publish(lstate);
        }

        else if(distance > 2.5)
        {
            gain_x = 0.5;
            gain_y = 0.25;

            velocity = cos(ang_z) * gain_x * error_x + sin(ang_z) * gain_y * error_y;
            steer_angle = 1/c*(-sin(ang_z) * gain_x * error_x + cos(ang_z) * gain_y * error_y);


            if(velocity>1.0)
            {
                velocity = 1.0;
            }
            else if(velocity<0)
            {
                velocity = 0;
            }
            if(steer_angle>0.2)
            {
                steer_angle =0.2;
            }
            else if(steer_angle<-0.2)
            {
                steer_angle = -0.2;
            }
            // lstate.data="BLUE";
            // lstate_pub.publish(lstate);
        }
        else
        {
            gain_x = 0.4;
            gain_y = 0.05;


            velocity = cos(ang_z) * gain_x * error_x + sin(ang_z) * gain_y * error_y;
            steer_angle = 1/c*(-sin(ang_z) * gain_x * error_x + cos(ang_z) * gain_y * error_y);

            if(velocity>0.5)
            {
                velocity = 0.5;
            }
            else if(velocity<0)
            {
                velocity = 0;
            }
            if(steer_angle>0.15)
            {
                steer_angle =0.15;
            }
            else if(steer_angle<-0.15)
            {
                steer_angle = -0.15;
            }

            //object 의 각도에 따라 gain 튜닝을 하였음 object가 30도 일때, 50도 일때, 30도 이하일때
            //를 나눠 각각 gain 튜닝 함.
            if(atan2(error_x,error_y)*180/3.14>30 || atan2(error_x,error_y)*180/3.14>330)
            {
                gain_x = 0.4;
                gain_y = 0.22;

                velocity = cos(ang_z) * gain_x * error_x + sin(ang_z) * gain_y * error_y;
                steer_angle = 1/c*(-sin(ang_z) * gain_x * error_x + cos(ang_z) * gain_y * error_y);


                if(velocity>0.5)
                {
                    velocity = 0.5;
                }
                else if(velocity<0)
                {
                    velocity = 0;
                }
                if(steer_angle>0.34)
                {
                    steer_angle = 0.34;
                }
                else if(steer_angle<-0.34)
                {
                    steer_angle = -0.34;
                }


            }
            else if(atan2(error_x,error_y)*180/3.14>50 || atan2(error_x,error_y)*180/3.14>310)
            {
                gain_x = 0.4;
                gain_y = 0.35;

                velocity = cos(ang_z) * gain_x * error_x + sin(ang_z) * gain_y * error_y;
                steer_angle = 1/c*(-sin(ang_z) * gain_x * error_x + cos(ang_z) * gain_y * error_y);


                if(velocity>0.5)
                {
                    velocity = 0.5;
                }
                else if(velocity<0)
                {
                    velocity = 0;
                }
                if(steer_angle>0.35)
                {
                    steer_angle = 0.35;
                }
                else if(steer_angle<-0.35)
                {
                    steer_angle = -0.35;
                }
            }

        }
        
        cmd_vel.linear.x = velocity;
        cmd_vel.angular.z = steer_angle;
        ROS_INFO("vel : %2f", cmd_vel.linear.x);
        ROS_INFO("ang_vel : %f\n", cmd_vel.angular.z);

        
        cmd_vel_publisher.publish(cmd_vel);
        flag =false;
    }
    else
    {
       if(flag == false)
       {
           std::cout<<"no goal point data"<<std::endl;
          
       }
       if(is_goal == true)
       {
           std::cout<<"to close"<<std::endl;
       }

        
    }
}
controller::~controller()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_pub");
    ros::NodeHandle nh;

    controller kine(&nh);
    ros::spin();

    return 0;
}

