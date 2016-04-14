#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <homography_vsc_cl/HomogDecompSolns.h>
#include <image_geometry/pinhole_camera_model.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>

Eigen::Matrix<double,4,3> diffMat(const Eigen::Quaterniond);

class Sim
{
    // ROS stuff
    ros::NodeHandle nh;
    ros::Subscriber velSubMocap;
    ros::Subscriber velSubHomog;
    ros::Subscriber joySub;
    ros::Publisher camInfoPub;
    ros::Publisher velPub;
    ros::Publisher homogPub;
    tf::TransformBroadcaster tfbr;
    tf::TransformListener tfl;
    ros::Timer integrateTimer;
    
    // Parameters
    double intTime;
    cv::Mat camMat;
    cv::Mat distCoeffs;
    std::string cameraName;
    double startTime;
    bool useHomog;
    bool useMocap;
    double lastTime;
    
    // States
    Eigen::Vector3d camPos;
    Eigen::Quaterniond camOrient;
    Eigen::Vector3d linVel;
    Eigen::Vector3d angVel;
    
public:
    Sim()
    {
        // Get Parameters
        ros::NodeHandle nhp("~");
        nhp.param<std::string>("cameraName",cameraName,"bebop_image");
        
        // Publishers
        camInfoPub = nh.advertise<sensor_msgs::CameraInfo>("bebop/camera_info",10,true); // latched
        velPub = nh.advertise<geometry_msgs::TwistStamped>(cameraName+"/body_vel",10);
        velSubMocap = nh.subscribe<geometry_msgs::Twist>("desVelMocap",1,&Sim::mocapVelCB,this);
        velSubHomog = nh.subscribe<geometry_msgs::Twist>("desVelHomog",1,&Sim::homogVelCB,this);
        homogPub = nh.advertise<homography_vsc_cl::HomogDecompSolns>("homogDecompSoln",10);
        
        // Initialize states
        camPos << 0,0,3;
        camOrient = Eigen::Quaterniond(0,1,1,0);
        camOrient.normalize();
        linVel = Eigen::Vector3d::Zero();
        angVel = Eigen::Vector3d::Zero();
        useHomog = false;
        useMocap = false;
        
        // Initialize Parameters
        intTime = 1.0/300.0;
        startTime = ros::Time::now().toSec();
        
        // Initialize and Publish camera info
        sensor_msgs::CameraInfo camInfoMsg;
        double K[] = {1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0};
        double D[] = {0.0,0.0,0.0,0.0,0.0};
        std::vector<double> Dvec(D,D + sizeof(D)/sizeof(D[0]));
        for (int i = 0; i < 9; i++)
        {
            camInfoMsg.K[i] = K[i];
        }
        camInfoMsg.D = Dvec;
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(camInfoMsg);
        camMat = cv::Mat(cam_model.fullIntrinsicMatrix());
        camMat.convertTo(camMat,CV_64FC1);
        cam_model.distortionCoeffs().convertTo(distCoeffs,CV_64FC1);
        camInfoPub.publish(camInfoMsg); // latched
        
        // Integrator
        joySub = nh.subscribe("joy",1,&Sim::joyCB,this);
        integrateTimer = nh.createTimer(ros::Duration(intTime),&Sim::integrateCB,this,false);
    }
    
    void integrateCB(const ros::TimerEvent& event)
    {
        
        // Integrate camera pose
        camPos += camOrient*linVel*intTime; // convert from body to world and integrate
        Eigen::Vector4d camOrientTemp(camOrient.w(),camOrient.x(),camOrient.y(),camOrient.z());
        camOrientTemp += 0.5*diffMat(camOrient)*angVel*intTime;
        camOrient = Eigen::Quaterniond(camOrientTemp(0),camOrientTemp(1),camOrientTemp(2),camOrientTemp(3));
        camOrient.normalize();
        
        // Publish camera tf
        tf::Transform tfCam;
        tfCam.setOrigin(tf::Vector3(camPos(0),camPos(1),camPos(2)));
        tfCam.setRotation(tf::Quaternion(camOrient.x(),camOrient.y(),camOrient.z(),camOrient.w()));
        tfbr.sendTransform(tf::StampedTransform(tfCam,event.current_real,"world",cameraName));
        
        // Publish red marker tf
        tf::Transform tfMarker;
        tfMarker.setOrigin(tf::Vector3(0.2,0.2,0));
        tfMarker.setRotation(tf::Quaternion(0,0,0,1));
        tfbr.sendTransform(tf::StampedTransform(tfMarker,event.current_real+ros::Duration(0.01),"world","ugv1"));
        
        // publish velocity
        geometry_msgs::TwistStamped twistMsg;
        twistMsg.header.stamp = event.current_real;
        twistMsg.twist.linear.x = linVel.x();
        twistMsg.twist.linear.y = linVel.y();
        twistMsg.twist.linear.z = linVel.z();
        twistMsg.twist.angular.x = angVel.x();
        twistMsg.twist.angular.y = angVel.y();
        twistMsg.twist.angular.z = angVel.z();
        velPub.publish(twistMsg);
        
        try
        {
            tf::StampedTransform tfRef;
            tfl.waitForTransform("world",cameraName+"_ref",ros::Time(0),ros::Duration(0.01));
            tfl.lookupTransform("world",cameraName+"_ref",ros::Time(0),tfRef);
            tf::Vector3 nStarVec = tf::quatRotate(tfRef.getRotation(),tf::Vector3(0,0,-1));
            
            tf::Transform tfIm2Ref = tfRef.inverse()*tfCam;
            tf::Transform tfMarker2Im = tfCam.inverse()*tfMarker;
            tf::Transform tfMarker2Ref = tfIm2Ref*tfMarker2Im;
            
            cv::Mat m1(cv::Size(1,3),CV_64FC1);
            tf::Vector3 marker2ImT = tfMarker2Im.getOrigin();
            m1.at<double>(0,0) = marker2ImT.getX()/marker2ImT.getZ();
            m1.at<double>(1,0) = marker2ImT.getY()/marker2ImT.getZ();
            m1.at<double>(2,0) = 1;
            cv::Mat pixels = camMat*m1;
            tf::Vector3 ref2ImT = -1*tfIm2Ref.getOrigin();
            tf::Quaternion ref2ImQ = tfIm2Ref.getRotation().inverse();
            
            // Construct homog message and publish
            homography_vsc_cl::HomogDecompSolns homogMsg;
            homogMsg.header.stamp = ros::Time::now();
            homogMsg.pose1.position.x = ref2ImT.getX();
            homogMsg.pose1.position.y = ref2ImT.getY();
            homogMsg.pose1.position.z = ref2ImT.getZ();
            homogMsg.pose1.orientation.x = ref2ImQ.getX();
            homogMsg.pose1.orientation.y = ref2ImQ.getY();
            homogMsg.pose1.orientation.z = ref2ImQ.getZ();
            homogMsg.pose1.orientation.w = ref2ImQ.getW();
            homogMsg.n1.x = nStarVec.getX();
            homogMsg.n1.y = nStarVec.getY();
            homogMsg.n1.z = nStarVec.getZ();
            homogMsg.alphar = tfMarker2Ref.getOrigin().getZ()/tfMarker2Im.getOrigin().getZ();
            homogMsg.newPixels.pr.x = pixels.at<double>(0,0);
            homogMsg.newPixels.pr.y = pixels.at<double>(1,0);
            homogMsg.pose2 = homogMsg.pose1;
            homogMsg.n2 = homogMsg.n1;
            homogMsg.decomp_successful = true;
            homogPub.publish(homogMsg);
        }
        catch(tf::TransformException ex)
        {
            return;
        }
    }
    
    void homogVelCB(const geometry_msgs::TwistConstPtr& twist)
    {
        if (useHomog)
        {
            linVel << twist->linear.x,twist->linear.y,twist->linear.z;
            angVel << twist->angular.x,twist->angular.y,twist->angular.z;
        }
    }
    
    void mocapVelCB(const geometry_msgs::TwistConstPtr& twist)
    {
        if (useMocap)
        {
            linVel << twist->linear.x,twist->linear.y,twist->linear.z;
            angVel << twist->angular.x,twist->angular.y,twist->angular.z;
        }
    }
    
    void joyCB(const sensor_msgs::JoyConstPtr& joyMsg)
    {
        if (joyMsg->buttons[5]) // RB - use homog
        {
            useHomog = true;
            useMocap = false;
        }
        else if (joyMsg->buttons[4]) // LB - use mocap
        {
            useHomog = false;
            useMocap = true;
        }
        else if (joyMsg->buttons[1]) // b - reset target
        {
            useHomog = false;
            useMocap = false;
            camPos << 0,0,3;
            camOrient = Eigen::Quaterniond(0,1,1,0);
        }
        else
        {
            useHomog = false;
            useMocap = false;
            linVel = 3*Eigen::Vector3d(-1*joy_deadband(joyMsg->axes[0]), joy_deadband(-1*joyMsg->axes[1]), joy_deadband(joyMsg->axes[7]));
            angVel = 2*Eigen::Vector3d(-1*joy_deadband(joyMsg->axes[4]), joy_deadband(joyMsg->axes[3]), joy_deadband(joyMsg->axes[2]-joyMsg->axes[5]));
        }
    }
    
    double joy_deadband(double input_value)
    {
        double filtered_value = 0;
        if (std::abs(input_value) > 0.15)
        {
            filtered_value = input_value;
        }
        return filtered_value;
    }
}; // End Sim class

// Calculate differential matrix for relationship between quaternion derivative and angular velocity.
// qDot = 1/2*B*omega 
// See strapdown inertial book. If quaternion is orientation of frame 
// B w.r.t N in the sense that nP = q*bP*q', omega is ang. vel of frame B w.r.t. N,
// i.e. N_w_B, expressed in the B coordinate system
// q = [w,x,y,z]
Eigen::Matrix<double,4,3> diffMat(const Eigen::Quaterniond q)
{
    Eigen::Matrix<double,4,3> B;
    B << -q.x(), -q.y(), -q.z(), q.w(), -q.z(), q.y(), q.z(), q.w(), -q.x(), -q.y(), q.x(), q.w();
    return B;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sim_node");
    
    Sim obj;
    
    ros::spin();
    return 0;
}