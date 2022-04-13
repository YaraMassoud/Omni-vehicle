#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <string>
#include <time.h>
#include <Eigen/Dense>
#include <tf/tf.h>

#define PI 3.1416

namespace gazebo
{
class OmniVehiclePlugin : public ModelPlugin
{

public: ros::NodeHandle nh;
public: std::string namespc;
public: ros::Publisher  vehicle_state_pub ;
private: physics::ModelPtr model;
private: event::ConnectionPtr updateConnection;

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
    	this->model = _parent;
		if(_sdf->HasElement("nameSpace"))
			namespc  = _sdf->GetElement("nameSpace")->Get<std::string>();
			
		std::string topicName = namespc + "/vehicle_pose" ;
		vehicle_state_pub = nh.advertise<geometry_msgs::Pose>(topicName, 1); 
	
    	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&OmniVehiclePlugin::onUpdate, this));		
	}  


public:	void onUpdate()
	{
		
		geometry_msgs::Pose msg;
		
		// Get pointers to relevant joints and iterate //
		physics::LinkPtr vehicle;
		std::string vehicle_name;
		vehicle_name = namespc + "::" + "origin_link" ;

		vehicle = this->model->GetLink(vehicle_name);

		ignition::math::Pose3d pose = vehicle->WorldCoGPose() ;
		ignition::math::Vector3 position = pose.Pos() ;
		ignition::math::Quaternion quatern = pose.Rot() ;		
		
		geometry_msgs::Point point;
		point.x = position.X();	point.y = position.Y();	point.z = position.Z();		
		geometry_msgs::Quaternion quaternion;
		quaternion.x = quatern.X();	quaternion.y = quatern.Y();	quaternion.z = quatern.Z();	quaternion.w = quatern.W();	
		
/*		tf::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
		tf::Matrix3x3 R(q);
		double roll, pitch, yaw ;
		R.getRPY(roll, pitch, yaw) ; 
*/
		
		msg.position = point ;
		msg.orientation = quaternion ;
		
		vehicle_state_pub.publish(msg);

	}

};
GZ_REGISTER_MODEL_PLUGIN(OmniVehiclePlugin)
}

