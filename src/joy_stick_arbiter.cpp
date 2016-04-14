#include <iostream>
#include <cstdio>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <algorithm>

// ros and opencv includes for using opencv and ros
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <homog_track/ImageProcessingMsg.h>
#include <homog_track/DecompMsg.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <bebop_msgs/Ardrone3CameraStateOrientation.h>
#include <geometry_msgs/PoseStamped.h>

class Joycall
{
	public:
		ros::NodeHandle nh;
		ros::Subscriber joy_sub, velCmdHomogSub, velCmdMocapSub, camera_state_sub, body_vel_sub, gimbal_sub, body_mocap_sub, body_world_vel_sub;
		ros::Publisher takeoff_pub, land_pub, reset_pub, cmd_vel_pub, camera_state_pub, error_pub, errorDot_pub;
		ros::Timer watchdogTimer;
		tf::TransformListener listener;
		
		geometry_msgs::Twist error_gm, errorDot_gm;
		
		/********** output command choice and xbox controller **********/
		geometry_msgs::Twist command_from_xbox;
		geometry_msgs::Twist velCmdHomog;
        geometry_msgs::Twist velCmdMocap;
		geometry_msgs::Twist command_out;
		
		geometry_msgs::Twist gimbal_state_current;
		geometry_msgs::Twist gimbal_state_desired;
		int a_button = 0, x_button = 0, b_button = 0, y_button = 0, left_bumper = 0, right_bumper = 0, start_button = 0, dpad_l = 0, dpad_r = 0, dpad_u = 0, dpad_d = 0;
		double right_stick_vert = 0, right_stick_horz = 0, left_stick_vert = 0, left_stick_horz = 0;
		bool start_autonomous = false;
		bool send_0 = false;
		bool recievedVelCmdHomog = false;
        bool recievedVelCmdMocap = false;
		bool recieved_command_from_xbox = false;
		bool first_run = true;
		
		double max_vel = 1;// max velocity
		
		double joy_gain = 1.0;// gain on xbox controller
		double step_gain = 0.0;
		
		double x_bounds[2] = {-1.75, 1.75};// x world bounds meters
		double y_bounds[2] = {-1.75, 1.75};// y world bounds meters
		double z_bounds[2] = {0,2.5};// z position meters
		
		tf::Quaternion bound_push_back_out_temp;// holds the quaternion result output after transforming
		tf::Quaternion bound_push_back_temp;// holds the quaternion form for the push back velocities so it can be transformed
		geometry_msgs::Twist bound_push_back_cmd;// push back twist message, to be created when the boundary is crossed
		tf::StampedTransform body_wrt_world;
		bool x_bounds_exceeded[2] = {false, false};// boolean to tell if the x boundary has been exceeded
		bool y_bounds_exceeded[2] = {false, false};// boolean to tell if the y boundary has been exceeded
		bool z_bounds_exceeded[2] = {false, false};// boolean to tell if the z boundary has been exceeded
		bool boundary_exceeded = false;// tells if any boundary exceeded
		double kx = 1.0/17.46;// x map
		double ky = 1.0/13.75;// y map
		double kz = 1.0/1.02;// z map
		double kyaw = 1.0/1.72;// yaw map 

		double kp_yaw = 1;//original 2.5
		double kd_yaw = 0.005;//original 0.075
		
		tf::Matrix3x3 kp = tf::Matrix3x3(1.75, 0, 0,
										 0, 1.75, 0,
										 0, 0, 1);
		tf::Matrix3x3 kd = tf::Matrix3x3(0.01, 0, 0,
										 0, 0.01, 0,
										 0, 0, 0.01);
		
		geometry_msgs::Twist body_vel;// body velocity from the mocap
		ros::Time last_body_vel_time;// last time a body velocity was recieved
		ros::Time curr_body_vel_time;// current time for a recieved body velocity
		ros::Time start_body_vel_time;//start time for the body velocity
		tf::Vector3 error;//error for the pd controller
		tf::Vector3 last_error;//last error for the pd controller
		tf::Vector3 errorDot;//derivative of error for the pd controller
		double error_yaw = 0;
		double errorDot_yaw = 0;
		double last_error_yaw;
		bool first_body_vel = true;
		
		geometry_msgs::Pose body_pose;
		geometry_msgs::Twist body_world_vel;
		
		double wait_time = 1;//wait time of 1 second for the watchdog timer
		
		Joycall(double step_gain_des)
		{
			step_gain = step_gain_des;
			velCmdHomogSub = nh.subscribe("desVelHomog",1,&Joycall::velCmdHomogCB,this);
            velCmdMocapSub = nh.subscribe("desVelMocap",1,&Joycall::velCmdMocapCB,this);
			cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1);// publisher for the decomposed stuff
			takeoff_pub = nh.advertise<std_msgs::Empty>("bebop/takeoff",1);
			land_pub = nh.advertise<std_msgs::Empty>("bebop/land",1);
			reset_pub = nh.advertise<std_msgs::Empty>("bebop/reset",1);
			camera_state_pub = nh.advertise<geometry_msgs::Twist>("/bebop/camera_control",1);// set the gimbal desired center
			joy_sub = nh.subscribe("joy",1,&Joycall::joy_callback,this);
			body_vel_sub = nh.subscribe("/bebop/body_vel",1,&Joycall::body_vel_callback,this);// callback for the body vel
			gimbal_sub = nh.subscribe("bebop/states/ARDrone3/CameraState/Orientation",1,&Joycall::gimbal_state_callback,this);
            watchdogTimer = nh.createTimer(ros::Duration(wait_time),&Joycall::timeout,this,false);// Initialize watchdog timer
            error_pub = nh.advertise<geometry_msgs::Twist>("error_twist",1);
            errorDot_pub = nh.advertise<geometry_msgs::Twist>("errorDot_twist",1);
			body_mocap_sub = nh.subscribe("/bebop/pose", 1, &Joycall::body_pose_callback, this);// subscribing to the p
			body_world_vel_sub = nh.subscribe("/bebop/vel", 1, &Joycall::body_world_vel_callback, this);// subscribing to the body velocity publisher
			
			cmd_vel_pub.publish(geometry_msgs::Twist());// initially sending it a desired command of 0
            gimbal_state_desired.angular.y = -60;// adjust the gimbal
            camera_state_pub.publish(gimbal_state_desired);// update the angle
		}
		
        // homog velocity command callback
        void velCmdHomogCB(const geometry_msgs::Twist& msg)
        {
            watchdogTimer.stop();
            error.setValue(msg.linear.x - body_vel.linear.x, msg.linear.y - body_vel.linear.y, msg.linear.z - body_vel.linear.z);
            error_yaw = msg.angular.z - body_vel.angular.z;

            // if some time has passed between the last body velocity time and the current body velocity time then will calculate the (feed forward PD)
            if (std::abs(curr_body_vel_time.toSec() - last_body_vel_time.toSec()) > 0.00001)
            {   
                errorDot = (1/(curr_body_vel_time - last_body_vel_time).toSec()) * (error - last_error);
                //std::cout << "errordot x: " << errorDot.getX() << " y: " << errorDot.getY() << " z: " << errorDot.getZ() << std::endl;
                
                errorDot_yaw = (1/(curr_body_vel_time - last_body_vel_time).toSec()) * (error_yaw - last_error_yaw);
                //std::cout << "error dot yaw " << errorDot_yaw << std::endl;
                velCmdHomog.linear.x = cap_vel_auton(kx*msg.linear.x + (kp*error).getX() + (kd*errorDot).getX());
                velCmdHomog.linear.y = cap_vel_auton(ky*msg.linear.y + (kp*error).getY() + (kd*errorDot).getY());
                velCmdHomog.linear.z = cap_vel_auton(kz*msg.linear.z + (kp*error).getZ() + (kd*errorDot).getZ());
                velCmdHomog.angular.z = -1*cap_vel_auton(kyaw*msg.angular.z + kp_yaw*error_yaw + kd_yaw*errorDot_yaw); // z must be switched because bebop driver http://bebop-autonomy.readthedocs.org/en/latest/piloting.html
            }
            
            if (left_bumper > 0)
            {
                recievedVelCmdHomog = true;
            }
            
            watchdogTimer.start();
        }
        
        // mocap velocity command callback
        void velCmdMocapCB(const geometry_msgs::Twist& msg)
        {
            watchdogTimer.stop();
            error.setValue(msg.linear.x - body_vel.linear.x, msg.linear.y - body_vel.linear.y, msg.linear.z - body_vel.linear.z);
            error_yaw = msg.angular.z - body_vel.angular.z;
            // if some time has passed between the last body velocity time and the current body velocity time then will calculate the (feed forward PD)
            if (std::abs(curr_body_vel_time.toSec() - last_body_vel_time.toSec()) > 0.00001)
            {   
                errorDot = (1/(curr_body_vel_time - last_body_vel_time).toSec()) * (error - last_error);
                //std::cout << "errordot x: " << errorDot.getX() << " y: " << errorDot.getY() << " z: " << errorDot.getZ() << std::endl;
                
                errorDot_yaw = (1/(curr_body_vel_time - last_body_vel_time).toSec()) * (error_yaw - last_error_yaw);
                //std::cout << "error dot yaw " << errorDot_yaw << std::endl;
                velCmdMocap.linear.x = cap_vel_auton(kx*msg.linear.x + (kp*error).getX() + (kd*errorDot).getX());
                velCmdMocap.linear.y = cap_vel_auton(ky*msg.linear.y + (kp*error).getY() + (kd*errorDot).getY());
                velCmdMocap.linear.z = cap_vel_auton(kz*msg.linear.z + (kp*error).getZ() + (kd*errorDot).getZ());
                velCmdMocap.angular.z = -1*cap_vel_auton(kyaw*msg.angular.z + kp_yaw*error_yaw + kd_yaw*errorDot_yaw); // z must be switched because bebop driver http://bebop-autonomy.readthedocs.org/en/latest/piloting.html
            }
            
            last_body_vel_time = curr_body_vel_time;// update last time body velocity was recieved
            last_error = error;
            last_error_yaw = error_yaw;
            
            if (right_bumper > 0)
            {
                recievedVelCmdMocap = true;
            }
            watchdogTimer.start();
        }
        
		/********** callback for the watchdog timer **********/
		void timeout(const ros::TimerEvent& event)
	    {
	        cmd_vel_pub.publish(geometry_msgs::Twist());// publish 0s
	        //std::cout << "\ntimer callback" << std::endl;
	    }
		
		/********** callback for the gimbal **********/
		void gimbal_state_callback(const bebop_msgs::Ardrone3CameraStateOrientation::ConstPtr& msg)
		{
			double pan = msg->pan;
			double tilt = msg->tilt;
			
            if (std::abs(gimbal_state_current.angular.y - tilt) > 0.1)
            {
                gimbal_state_current.angular.y = tilt;
                gimbal_state_current.angular.z = pan;
                std::cout << "\ngimbal tilt angle: " << gimbal_state_current.angular.y << " pan angle: " << gimbal_state_current.angular.z << std::endl;
            }
		}
		
		/********** callback for the body pose **********/
		void body_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
		{
			body_pose = msg->pose;
		}
		
		/********** callback for the body world vel **********/
		void body_world_vel_callback(const geometry_msgs::TwistStamped& msg)
		{
			body_world_vel = msg.twist;
		}
		
		/********** callback for the body velocity **********/
		void body_vel_callback(const geometry_msgs::TwistStamped& msg)
		{			
			body_vel = msg.twist;
			curr_body_vel_time  = msg.header.stamp;
			if (first_body_vel)
			{
				start_body_vel_time = curr_body_vel_time;
				first_body_vel = false;
			}
		}
		
		/********** callback for the controller **********/
		void joy_callback(const sensor_msgs::Joy& msg)
		{
			watchdogTimer.stop();
			
			command_from_xbox = geometry_msgs::Twist();// cleaning the xbox twist message
			a_button = msg.buttons[0];// a button lands
			if (a_button > 0)
			{
				land_pub.publish(std_msgs::Empty());
			}
			x_button = msg.buttons[2];// x button sends constant
			if (x_button > 0)
			{
				send_0 = true;
			}
			
			b_button = msg.buttons[1];// b button kills motors
			if (b_button > 0)
			{
				reset_pub.publish(std_msgs::Empty());
			}
			
			y_button = msg.buttons[3];// y button takes off
			if (y_button > 0)
			{
				takeoff_pub.publish(std_msgs::Empty());
			}
				
			dpad_l = msg.buttons[11];//dpad left value, fine adjustment of tilt angle negatively
			dpad_r = msg.buttons[12];//dpad right value, fine adjustment of tilt angle positively
			dpad_u = msg.buttons[13];//dpad up value, coarse adjustment of tilt angle positively
			dpad_d = msg.buttons[14];//dpad down value, coarse adjustment of tilt angle negatively
			
			int tilt_neg = -1*dpad_l - 10*dpad_d;// tilt negatively
			int tilt_pos = 1*dpad_r + 10*dpad_u;// tilt positively
			gimbal_state_desired.angular.y = gimbal_state_desired.angular.y + tilt_neg + tilt_pos;// adjust the gimbal
			camera_state_pub.publish(gimbal_state_desired);// update the angle
			
			left_bumper = msg.buttons[4];// left bumper for using the tracking as the controller output
			right_bumper = msg.buttons[5];// right bumper for using the mocap as the controller output
			
			start_autonomous = (left_bumper > 0) || (right_bumper > 0);// start the autonomous if either are greater than 0
			
			right_stick_vert = joy_deadband(msg.axes[4]);// right thumbstick vert controls linear x
			command_from_xbox.linear.x = joy_gain*right_stick_vert;
			
			right_stick_horz = joy_deadband(msg.axes[3]);// right thumbstick horz controls linear y
			command_from_xbox.linear.y = joy_gain*right_stick_horz;
			
			left_stick_vert = joy_deadband(msg.axes[1]);// left thumbstick vert controls linear z
			command_from_xbox.linear.z = joy_gain*left_stick_vert;
			
			left_stick_horz = joy_deadband(msg.axes[0]);// left thumbstick horz controls angular z
			command_from_xbox.angular.z = -1*joy_gain*left_stick_horz;
			
			if (!start_autonomous)// if not using the autonomous velocities as output will indicate recieved a new control input
			{
				if (send_0)
				{
                    command_from_xbox = geometry_msgs::Twist();// cleaning the xbox twist message
					//command_from_xbox.linear.x = 0;
					//command_from_xbox.linear.y = 0;
					//command_from_xbox.linear.z = 0;
					//command_from_xbox.angular.z = 0;
					send_0 = false;
				}
				recieved_command_from_xbox = true;
			}
			
			watchdogTimer.start();
		}
		
		/********** deadband for the xbox controller **********/
		double joy_deadband(double input_value)
		{
			double filtered_value = 0;
			if (std::abs(input_value) > 0.15)
			{
				filtered_value = input_value;
			}
			return filtered_value;
		}
		
		/********** saturation for tracking controller commands **********/
		double cap_vel_auton(double input_value)
		{
			double filtered_value = input_value;
			if (std::abs(input_value) > max_vel)
			{
				if (input_value >= 0)
				{
					filtered_value = max_vel;
				}
				else
				{
					filtered_value = -1.0*max_vel;
				}
			}
			return filtered_value;
		}
};
// main
int main(int argc, char** argv)
{   
	ros::init(argc,argv,"joy_stick_control_node");
	
	bool write_to_file = true;
	double step_gain_ = 1.0;
	std::string output_file_name = "/home/ncr/ncr_ws/src/homog_track/testing_files/body_world_vel_test_5.txt";
	std::fstream output_file;
	
	if( (std::remove( output_file_name.c_str() ) != 0) && write_to_file)
	{
		std::cout << "file does not exist" << std::endl;
	}
	else
	{
		std::cout << "file deleted" << std::endl;
	}
	
	/******************** Writing headers to file ********************/
	if (write_to_file)
	{
		output_file.open(output_file_name, std::fstream::out | std::fstream::app);
	
		if (output_file.is_open())
		{
			output_file << "time,"
						<< "right bumper,"
						<< "linear vel x," << "linear vel y," << "linear vel z,"
						<< "linear pose x," << "linear pose y," << "linear pose z,"
						<< "\n";
			output_file.close();
		}
	}
	
	Joycall joycall(step_gain_);
	ros::Rate loop_rate(300);
    
    bool markers_found = false;
    tf::StampedTransform red_wrt_world, green_wrt_world, cyan_wrt_world, purple_wrt_world;
    red_wrt_world.setIdentity(); green_wrt_world.setIdentity(); cyan_wrt_world.setIdentity(); purple_wrt_world.setIdentity();
    try
    {
        // read turtle bot poses
        joycall.listener.waitForTransform("world", "ugv1", ros::Time(0), ros::Duration(5));
        joycall.listener.lookupTransform("world", "ugv1", ros::Time(0), red_wrt_world);
        joycall.listener.waitForTransform("world", "ugv2", ros::Time(0), ros::Duration(5));
        joycall.listener.lookupTransform("world", "ugv2", ros::Time(0), green_wrt_world);
        joycall.listener.waitForTransform("world", "ugv3", ros::Time(0), ros::Duration(5));
        joycall.listener.lookupTransform("world", "ugv3", ros::Time(0), cyan_wrt_world);
        joycall.listener.waitForTransform("world", "ugv4", ros::Time(0), ros::Duration(5));
        joycall.listener.lookupTransform("world", "ugv4", ros::Time(0), purple_wrt_world);
        markers_found = true;
        
    }
    catch (tf::TransformException ex)
    {
        std::cout << "failed to get marker positions" << std::endl;
        markers_found = false;
    }
    
    double marker_center[2] = {(red_wrt_world.getOrigin().getX() + green_wrt_world.getOrigin().getX() + cyan_wrt_world.getOrigin().getX() + purple_wrt_world.getOrigin().getX())/4.0,
                               (red_wrt_world.getOrigin().getY() + green_wrt_world.getOrigin().getY() + cyan_wrt_world.getOrigin().getY() + purple_wrt_world.getOrigin().getY())/4.0};// use the center of the markers as the center of the circle
	
    std::cout << "\n\nmarker center " << marker_center[0] << ", " << marker_center[1] << std::endl << std::endl;
	ros::Time start_time = ros::Time::now();
	
	while (ros::ok() && markers_found)
	{
		bool send_command = false;
		
		try
		{
			// getting the body in world frame
			joycall.listener.waitForTransform("world", "bebop", ros::Time(0), ros::Duration(1.0));
			joycall.listener.lookupTransform("world", "bebop", ros::Time(0), joycall.body_wrt_world);
		}
		catch (tf::TransformException ex)
		{
			std::cout << "faild to get body wrt world in joycall" << std::endl;
		}
		
        double currentOrigin[3] = {joycall.body_wrt_world.getOrigin().getX(),joycall.body_wrt_world.getOrigin().getY(),joycall.body_wrt_world.getOrigin().getZ(),};
        double currentBound[6] = {marker_center[0]+joycall.x_bounds[0],//lower X
                                 marker_center[0]+joycall.x_bounds[1],//upper X
                                 marker_center[1]+joycall.y_bounds[0],//lower y
                                 marker_center[1]+joycall.y_bounds[1],//upper y
                                 joycall.z_bounds[0],//lower z
                                 joycall.z_bounds[1]};//upper z
        double distances[6] = {currentOrigin[0]-currentBound[0],//lower x difference
                               currentOrigin[0]-currentBound[1],//upper x difference
                               currentOrigin[1]-currentBound[2],//lower y difference
                               currentOrigin[1]-currentBound[3],//upper y difference
                               currentOrigin[2]-currentBound[4],//lower z difference
                               currentOrigin[2]-currentBound[5]};//upper z difference
        bool boundExceed[6] = {distances[0] < 0,//lower negative implies exceeded
                               distances[1] > 0,//upper positive implies exceeded
                               distances[2] < 0,//lower negative implies exceeded
                               distances[3] > 0,//upper positive implies exceeded
                               distances[4] < 0,//lower negative implies exceeded
                               distances[5] > 0};//upper positive implies exceeded
		
        // will saturate command at a quarter meter outside bound
        double invSatDistance = 8.0; // inverse of saturation distance
        // lower x boundary exceeded
		if (boundExceed[0])
		{
			joycall.bound_push_back_temp.setX(-1*std::tanh(invSatDistance*distances[0]));
            std::cout << "\nlower x bound exceeded by " << distances[0] << " meters" << std::endl;
		}
		
		// upper x lower boundary exceeded
		if (boundExceed[1])
		{
			joycall.bound_push_back_temp.setX(-1*std::tanh(invSatDistance*distances[1]));
            std::cout << "upper x bound exceeded by " << distances[1] << " meters" << std::endl;
		}
		
		// lower y boundary exceeded
		if (boundExceed[2])
		{
			joycall.bound_push_back_temp.setY(-1*std::tanh(invSatDistance*distances[2]));
            std::cout << "lower y bound exceeded by " << distances[2] << " meters" << std::endl;
		}
		
		// upper y boundary exceeded
		if (boundExceed[3])
		{
			joycall.bound_push_back_temp.setY(-1*std::tanh(invSatDistance*distances[3]));
            std::cout << "upper y bound exceeded by " << distances[3] << " meters" << std::endl;
		}
        
        // lower z boundary exceeded
		if (boundExceed[4])
		{
			joycall.bound_push_back_temp.setZ(-1*std::tanh(invSatDistance*distances[4]));
            std::cout << "lower z bound exceeded by " << distances[4] << " meters" << std::endl;
		}

        // upper z boundary exceeded
		if (boundExceed[5])
		{
			joycall.bound_push_back_temp.setZ(-1*std::tanh(invSatDistance*distances[5]));
            std::cout << "upper z bound exceeded by " << distances[5] << " meters" << std::endl;
		}
        
		joycall.boundary_exceeded = false;
        for (int ii = 0; ii<6;ii++)
        {
            joycall.boundary_exceeded |= boundExceed[ii];
        }
		
		// if boundary exceeded will transform to body frame and set command
		if (joycall.boundary_exceeded)
		{
			// transform velocity to body frame and send zero rotation
			joycall.bound_push_back_out_temp = (joycall.body_wrt_world.getRotation().inverse()*joycall.bound_push_back_temp)*joycall.body_wrt_world.getRotation();
			joycall.bound_push_back_cmd.linear.x = joycall.bound_push_back_out_temp.getX(); joycall.bound_push_back_cmd.linear.y = joycall.bound_push_back_out_temp.getY(); joycall.bound_push_back_cmd.linear.z = joycall.bound_push_back_out_temp.getZ();
			joycall.bound_push_back_cmd.angular.x = 0; joycall.bound_push_back_cmd.angular.y = 0; joycall.bound_push_back_cmd.angular.z = 0;
		}
		
		if ((joycall.recievedVelCmdMocap || joycall.recievedVelCmdHomog || joycall.recieved_command_from_xbox) && !joycall.boundary_exceeded)// if command from xbox or controller and bound not exceeded
        {
            if (joycall.recieved_command_from_xbox)
            {
                //joycall.cmd_vel_pub.publish(joycall.command_from_xbox);// publish command;
                joycall.command_out = joycall.command_from_xbox;
                //joycall.command_from_xbox = geometry_msgs::Twist();
                joycall.recieved_command_from_xbox = false;
                send_command = true;
            }
            
            if (joycall.recievedVelCmdHomog || joycall.recievedVelCmdMocap)
            {
                if (joycall.recievedVelCmdMocap)
                {
                    joycall.command_out = joycall.velCmdMocap;
                    joycall.recievedVelCmdMocap = false;
                    //std::cout << "Mocap" << std::endl;
                    //std::cout << joycall.command_out << std::endl;
                }
                
                if (joycall.recievedVelCmdHomog)
                {
                    joycall.command_out = joycall.velCmdHomog;
                    joycall.recievedVelCmdHomog = false;
                    //std::cout << "Homog" << std::endl;
                    //std::cout << joycall.command_out << std::endl;
                }
                send_command = true;
                joycall.last_body_vel_time = joycall.curr_body_vel_time;// update last time body velocity was recieved
                joycall.last_error = joycall.error;
                joycall.last_error_yaw = joycall.error_yaw;
            }
        }
		
		if (joycall.boundary_exceeded)// set command to boundary command if boundary exceeded
		{
			//joycall.cmd_vel_pub.publish(joycall.bound_push_back_cmd);// publish command;
			joycall.command_out = joycall.bound_push_back_cmd;// output is the boundary command
			// reset stuff
			std::cout << "\nboundary exceeded" << std::endl;
			std::cout << "linear x: " << joycall.command_out.linear.x << std::endl;
			std::cout << "linear y: " << joycall.command_out.linear.y << std::endl;
			std::cout << "linear z: " << joycall.command_out.linear.z << std::endl;
			std::cout << "angular z: " << joycall.command_out.angular.z << std::endl;
			joycall.boundary_exceeded = false;
			joycall.recieved_command_from_xbox = false;
			joycall.recievedVelCmdHomog = false;
            joycall.recievedVelCmdMocap = false;
			send_command = true;
			joycall.bound_push_back_out_temp = tf::Quaternion(0,0,0,0);
			joycall.bound_push_back_temp = tf::Quaternion(0,0,0,0);
			joycall.bound_push_back_cmd = geometry_msgs::Twist();
		}
		
		if (send_command)
		{
			joycall.cmd_vel_pub.publish(joycall.command_out);// publish command;
			std::cout << "\npublish command" << std::endl;
			std::cout << "linear x: " << joycall.command_out.linear.x << std::endl;
			std::cout << "linear y: " << joycall.command_out.linear.y << std::endl;
			std::cout << "linear z: " << joycall.command_out.linear.z << std::endl;
			std::cout << "angular z: " << joycall.command_out.angular.z << std::endl << std::endl;
		}
		
		if (write_to_file)
		{
			output_file.open(output_file_name, std::fstream::out | std::fstream::app);
		}
		if (output_file.is_open())
		{
			output_file  << ros::Time::now().toSec() - start_time.toSec() << "," 
			<< joycall.right_bumper << "," 
			<< joycall.body_world_vel.linear.x << "," << joycall.body_world_vel.linear.y << "," << joycall.body_world_vel.linear.z << ","
			<< joycall.body_pose.position.x << "," << joycall.body_pose.position.y << "," << joycall.body_pose.position.z << ","
			<< "\n";
			output_file.close();
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}
