#include <cmath>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <tf/tf.h>

#define L 0.13           //Length from center of body to wheel? 
#define KP 70.0          //Proportional Gain   (max 150)    (70/5)    70
#define KI 0.0           //Integral Gain
#define KD 0.4            //Derivative Gain                            0.4

//x = 8, y = 4, KP = 20 
geometry_msgs::Vector3 curr_position;
geometry_msgs::Vector3 goal_position;

ros::Publisher publ, pubb, pubr;
ros::Subscriber sub, keysub;

using namespace std; 


fstream file;

struct state {
    double x, y, theta;
    double x_dot, y_dot, w; 
};

void setGoal();
void inv_kinematics_vehicle(double, double, double);
void inv_kinematics_world(double , double, double );
void controller(const geometry_msgs::Pose::ConstPtr& );
void keyper_callback(const std_msgs::Int16& );

double V1, V2, V3;
double VelX_v, VelY_v;
double VelX_w, VelY_w;
double w;

double xdes, ydes, thetades;

double start_time;
double curr_t, prev_t, dt;
double error_x, error_y, error_theta;
double prop_x, prop_y, prop_theta; 
double integral_tot_x, integral_tot_y, integral_tot_theta;
double derivative_x, derivative_y, derivative_theta; 
double prev_error_x, prev_error_y, prev_error_theta; 

double x_prev, y_prev, yaw_prev; 

void setGoal(){
    xdes = 0;
    ydes = 0;
    thetades = 1.5;
}

void keyper_callback(const std_msgs::Int16& keyper_msg) {

	int code = keyper_msg.data;
	switch(code)
	{		
		case 1:
			thetades = thetades + 0.1745;
			puts("Turning left");
			break;
		case 2:
			thetades = thetades - 0.1745;				
			puts("Turning right");
			break;
		case 5:
			xdes = xdes + 0.1;
			puts("Going X forward");
			break;
		case 6:
			xdes = xdes - 0.1;
			puts("Going X backward");
			break;
		case 7:
			ydes = ydes + 0.1;
			puts("Going Y forward");
			break;
		case 8:
			ydes = ydes - 0.1;
			puts("Going Y backward");
			break;
	}
}

void inv_kinematics_vehicle(double VelX_v, double  VelY_v, double u_t) {

    V1 = -(VelX_v/2) - (sqrt(3)*VelY_v)/2 + L*u_t; 
    V2 = VelX_v + (L*u_t);
    V3 = -(VelX_v/2) + (sqrt(3)*VelY_v)/2 + L*u_t;

    std_msgs::Float64 msg;
    msg.data = V1;
    publ.publish(msg);
	
    msg.data = V2;
    pubb.publish(msg);

    msg.data = V3;
    pubr.publish(msg);
}

void inv_kinematics_world(double VelX_w, double VelY_w, double yaw, double u_t) {

    VelX_v = (cos(yaw) * VelX_w) + (sin(yaw) * VelY_w);
    VelY_v = -(sin(yaw) * VelX_w) + (cos(yaw) * VelY_w);

    inv_kinematics_vehicle(VelX_v, VelY_v, u_t);
}

void controller(const geometry_msgs::Pose::ConstPtr& msg)
{    
    curr_t = ros::Time::now().toSec();
    if(prev_t == 0)
    {
	    prev_t = curr_t - 0.01;
    }
    dt = curr_t - prev_t;
	
    //getting current position (x and y)
    double x = msg->position.x;
    double y = msg->position.y;

    //getting current orientation (yaw)
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 R(q);
    double roll, pitch, yaw ;
    R.getRPY(roll, pitch, yaw); 

    //derivatives (velocity and acceleration)
    double x_dot = (x - x_prev)/dt;
	double y_dot = (y - y_prev)/dt;
	double w  = (yaw - yaw_prev)/dt;

    xdes = sin(curr_t); 
    ydes = 0; 
    thetades = 0; 

    double x_dest_dot = cos(curr_t); 
    double y_dest_dot = 0; 
    
    double x_dest_double_dot = -sin(curr_t); 
    double y_dest_double_dot = 0; 
    
    //curr to prev
    x_prev = x; 
    y_prev = y;
    yaw_prev = yaw; 

    //errors
    error_x = xdes - x;
    error_y = ydes - y;
    error_theta = thetades - yaw;

    if (error_theta > M_PI)
    {
        error_theta -= 2*M_PI; 
    }
    
    else if (error_theta < -M_PI)
    {
        error_theta += 2*M_PI; 
    }

    double u_x = - KP * ( -error_x ) - KD * (x_dot - x_dest_dot) + x_dest_double_dot;
	double u_y = - KP * ( -error_y ) - KD * (y_dot - y_dest_dot) + y_dest_double_dot;
	double u_t =  KP * error_theta;

    prev_t = curr_t; 

    file << curr_t << " , " << xdes << " , " << x_prev << " , " << ydes << " , " << y_prev << " , " << thetades << " , " << yaw_prev << endl; 
    inv_kinematics_world(u_x, u_y, yaw, u_t);
}

int main(int argc, char **argv)
{
    //make time start 
    //start_time = ros::Time::now().toSec();
    prev_t = 0; 
    
    file.open("controller_accuracy.txt",ios::out);
    if(!file)
   {
       cout<<"Error in creating file!!!";
       return 0;
   }
    ros::init(argc, argv, "vel_Publisher");
    ros::NodeHandle n;

    //setGoal();
  
    //pub
    ros::init(argc, argv, "vel_Publisher");
    publ = n.advertise<std_msgs::Float64>("/Omni_vehicle/left_controller/command", 1);
    pubb = n.advertise<std_msgs::Float64>("/Omni_vehicle/back_controller/command", 1);
    pubr = n.advertise<std_msgs::Float64>("/Omni_vehicle/right_controller/command", 1);
    ros::Rate loop_rate(100);

    //sub 
    keysub = n.subscribe("/Omni_wheel/teleoperator", 1, keyper_callback);
    sub = n.subscribe("Omni_vehicle/vehicle_pose", 1, controller);
    
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    file.close();
    return (0); 
}
