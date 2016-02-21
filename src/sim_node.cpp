#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Twist.h>
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
    
    // States
    Eigen::Vector3d camPosHomog;
    Eigen::Quaterniond camOrientHomog;
    Eigen::Vector3d camPosMocap;
    Eigen::Quaterniond camOrientMocap;
    Eigen::Vector3d linVelHomog;
    Eigen::Vector3d angVelHomog;
    Eigen::Vector3d linVelMocap;
    Eigen::Vector3d angVelMocap;
    
public:
    Sim()
    {
        // Get Parameters
        ros::NodeHandle nhp("~");
        nhp.param<std::string>("cameraName",cameraName,"bebop_image");
        
        // Publishers
        camInfoPub = nh.advertise<sensor_msgs::CameraInfo>(cameraName+"/camera_info",10,true); // latched
        velPub = nh.advertise<geometry_msgs::Twist>(cameraName+"/body_vel",10);
        velSubMocap = nh.subscribe<geometry_msgs::Twist>("desVelMocap",1,&Sim::mocapVelCB,this);
        velSubHomog = nh.subscribe<geometry_msgs::Twist>("desVelHomog",1,&Sim::homogVelCB,this);
        homogPub = nh.advertise<geometry_msgs::Twist>("homogDecompSoln",10);
        
        // Initialize states
        camPosHomog << 0,0,-2;
        camOrientHomog = Eigen::Quaterniond::Identity();
        camPosMocap << 0,0,-2;
        camOrientMocap = Eigen::Quaterniond::Identity();
        linVelHomog = Eigen::Vector3d::Zero();
        angVelHomog = Eigen::Vector3d::Zero();
        linVelMocap = Eigen::Vector3d::Zero();
        angVelMocap = Eigen::Vector3d::Zero();
        
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
        camMat.convertTo(camMat,CV_32FC1);
        cam_model.distortionCoeffs().convertTo(distCoeffs,CV_32FC1);
        camInfoPub.publish(camInfoMsg); // latched
        
        // Integrator
        integrateTimer = nh.createTimer(ros::Duration(intTime),&Sim::integrateCB,this,false);
    }
    
    void integrateCB(const ros::TimerEvent& event)
    {
        // Integrate homog camera pose
        camPosHomog += camOrientHomog*linVelHomog*intTime; // convert from body to world and integrate
        Eigen::Vector4d camOrientHomogTemp(camOrientHomog.w(),camOrientHomog.x(),camOrientHomog.y(),camOrientHomog.z());
        camOrientHomogTemp += 0.5*diffMat(camOrientHomog)*angVelHomog*intTime;
        camOrientHomog = Eigen::Quaterniond(camOrientHomogTemp(0),camOrientHomogTemp(1),camOrientHomogTemp(2),camOrientHomogTemp(3));
        camOrientHomog.normalize();
        
        // Integrate mocap camera pose
        camPosMocap += camOrientMocap*linVelMocap*intTime; // convert from body to world and integrate
        Eigen::Vector4d camOrientMocapTemp(camOrientMocap.w(),camOrientMocap.x(),camOrientMocap.y(),camOrientMocap.z());
        camOrientMocapTemp += 0.5*diffMat(camOrientMocap)*angVelMocap*intTime;
        camOrientMocap = Eigen::Quaterniond(camOrientMocapTemp(0),camOrientMocapTemp(1),camOrientMocapTemp(2),camOrientMocapTemp(3));
        camOrientMocap.normalize();
        
        // Publish homog camera tf
        tf::Transform tfHomog;
        tfHomog.setOrigin(tf::Vector3(camPosHomog(0),camPosHomog(1),camPosHomog(2)));
        tfHomog.setRotation(tf::Quaternion(camOrientHomog.x(),camOrientHomog.y(),camOrientHomog.z(),camOrientHomog.w()));
        tfbr.sendTransform(tf::StampedTransform(tfHomog,ros::Time::now(),"world",cameraName+"_homog"));
        
        // Publish mocap camera tf
        tf::Transform tfMocap;
        tfMocap.setOrigin(tf::Vector3(camPosMocap(0),camPosMocap(1),camPosMocap(2)));
        tfMocap.setRotation(tf::Quaternion(camOrientMocap.x(),camOrientMocap.y(),camOrientMocap.z(),camOrientMocap.w()));
        tfbr.sendTransform(tf::StampedTransform(tfMocap,ros::Time::now(),"world",cameraName));
        
        // Publish red marker tf
        tf::Transform tfMarker;
        tfMarker.setOrigin(tf::Vector3(0.2,0.2,0));
        tfMarker.setRotation(tf::Quaternion(0,0,0,1));
        tfbr.sendTransform(tf::StampedTransform(tfMarker,ros::Time::now(),"world","ugv1"));
        
        // homog solution
        tf::StampedTransform tfIm2Ref;
        tfl.waitForTransform(cameraName+"_ref",cameraName,event.current_real,ros::Duration(0.01));
        tfl.lookupTransform(cameraName+"_ref",cameraName,event.current_real,tfIm2Ref);
        tf::StampedTransform tfRef;
        tfl.waitForTransform("world",cameraName+"_ref",event.current_real,ros::Duration(0.01));
        tfl.lookupTransform("world",cameraName+"_ref",event.current_real,tfRef);
        tf::Vector3 nStarVec = tfRef.getRotation()*tf::Vector3(0,0,-1);
        homography_vsc_cl::HomogDecompSolns homogMsg;
        homogMsg.header.stamp = ros::Time::now();
        
    }
    
    void velPubCB(const ros::TimerEvent& event)
    {
        geometry_msgs::TwistStamped twistMsg;
        twistMsg.header.stamp = ros::Time::now();
        
        // Publish camera velocities
        twistMsg.twist.linear.x = camLinVel(0);
        twistMsg.twist.linear.y = camLinVel(1);
        twistMsg.twist.linear.z = camLinVel(2);
        twistMsg.twist.angular.x = camAngVel(0);
        twistMsg.twist.angular.y = camAngVel(0);
        twistMsg.twist.angular.z = camAngVel(0);
        camVelPub.publish(twistMsg);
        
        // Publish target velocities
        if (useVelocityMap) {get_target_velocity_from_map();}
        if (driveCircle)
        {
            double timeNow = twistMsg.header.stamp.toSec();
            targetLinVel << std::sin(3*(timeNow - startTime)), std::cos(3*(timeNow - startTime)), 0;
            targetAngVel << 0,0,0;
        }
        twistMsg.twist.linear.x = targetLinVel(0);
        twistMsg.twist.linear.y = targetLinVel(1);
        twistMsg.twist.linear.z = targetLinVel(2);
        twistMsg.twist.angular.x = targetAngVel(0);
        twistMsg.twist.angular.y = targetAngVel(0);
        twistMsg.twist.angular.z = targetAngVel(0);
        targetVelPub.publish(twistMsg);
        
        // Publish target Odometry
        nav_msgs::Odometry odomMsg;
        odomMsg.header.stamp = ros::Time::now();
        odomMsg.header.frame_id = targetName+"/odom";
        odomMsg.child_frame_id = targetName+"/base_footprint";
        odomMsg.twist.twist = twistMsg.twist;
        targetOdomPub.publish(odomMsg);
        
        /*
        std::cout << "VelPubCB:" << std::endl;
        std::cout << "time: " << twistMsg.header.stamp.toSec() << std::endl;
        std::cout << "targetLinVel: " << targetLinVel.transpose() << std::endl;
        std::cout << "t2cPos: " << (camOrient.inverse()*(targetPos - camPos)).transpose() << std::endl;
        std::cout << "useVelocityMap: " << useVelocityMap << std::endl;
        */
    }
    
    void imagePubCB(const ros::TimerEvent& event)
    {
        // Target w.r.t. camera
        Eigen::Vector3d t2cPos = camOrient.inverse()*(targetPos - camPos);
        
        // Convert to OpenCV
        cv::Mat t2cPosCV;
        cv::eigen2cv((Eigen::MatrixXd) t2cPos.transpose(),t2cPosCV);
        
        // Project points to determine pixel coordinates
        cv::Mat imagePoint;
        cv::projectPoints(t2cPosCV,cv::Mat::zeros(1,3,CV_32F),cv::Mat::zeros(1,3,CV_32F),camMat,distCoeffs,imagePoint);
        
        // Publish image point
        aruco_ros::Center msg;
        msg.header.stamp = ros::Time::now();
        char buf[10];
        std::sprintf(buf,"%d",markerID);
        msg.header.frame_id = buf;
        msg.x = imagePoint.at<double>(0);
        msg.y = imagePoint.at<double>(1);
        featurePub.publish(msg);
    }
    
    void get_target_velocity_from_map()
    {
        // service msg handle
        switch_vis_exp::MapVel srv;
        
        // Construct request
        srv.request.pose.position.x = targetPos(0);
        srv.request.pose.position.y = targetPos(1);
        srv.request.pose.position.z = targetPos(2);
        if (velocityMapClient.call(srv))
        {
            // get velocity
            Eigen::Vector3d des_lin_vel;
            Eigen::Vector3d des_ang_vel;
            des_lin_vel << srv.response.twist.linear.x, srv.response.twist.linear.y, srv.response.twist.linear.z;
            des_ang_vel << srv.response.twist.angular.x, srv.response.twist.angular.y, srv.response.twist.angular.z;
            
            // rotate velocity into target body frame
            targetLinVel = targetOrient.inverse()*des_lin_vel;
            targetAngVel = targetOrient.inverse()*des_ang_vel;
        }
    }
    
    void joyCB(const sensor_msgs::JoyConstPtr& joyMsg)
    {
        if (joyMsg->buttons[2]) // x - drive along velocity map
        {
            useVelocityMap = true;
            driveCircle = false;
            /*
            radius += 0.1*(joyMsg->buttons[12]-joyMsg->buttons[11]);
            period -= 10*(joyMsg->buttons[13]-joyMsg->buttons[14]);
            targetAngVel << 0, 0, 2*M_PI/period;
            targetLinVel << 2*M_PI*radius/period, 0, 0;
            */
        }
        else if (joyMsg->buttons[0]) // a - drive in circle
        {
            driveCircle = true;
            useVelocityMap = false;
        }
        else if (joyMsg->buttons[1]) // b - reset target
        {
            useVelocityMap = false;
            targetPos << 0, -radius, 0;
            targetOrient.setIdentity();
        }
        else
        {
            useVelocityMap = false;
            driveCircle = false;
            Eigen::Vector3d linVel(-1*joyMsg->axes[0], -1*joyMsg->axes[1], 0);
            Eigen::Vector3d angVel(-1*joyMsg->axes[4], joyMsg->axes[3], joyMsg->axes[2]-joyMsg->axes[5]);
            if (joyMsg->buttons[5]) // Right bumper, control camera
            {
                camLinVel = linVel;
                camAngVel = angVel;
            }
            else // control target
            {
                targetLinVel = linVel;
                targetAngVel = angVel;
                camLinVel << 0,0,0;
                camAngVel << 0,0,0;
            }
        }
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
