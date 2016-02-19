#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <homography_vsc_cl/HomogDecompSolns.h>
#include <geometry_msgs/Twist.h>

class Controller
{
    // ROS stuff
    ros::NodeHandle nh;
    ros::Subscriber camInfoSub;
    ros::Subscriber joySub;
    ros::Subscriber homogSolnSub;
    ros::ServiceClient setHomogReferenceClient;
    ros::Publisher velPubHomog;
    ros::Publisher velPubMocap;
    
    // Camera parameters
    cv::Mat camMat;
    bool gotCamParam;
    Eigen::Matrix3d camMatFactor; // factor used in Lv
    
    // Controller parameters
    double kv;
    double kw;
    Eigen::Matrix3d Kv;
    Eigen::Matrix3d Kw;
    
public:
    Controller()
    {
        // Parameters
        ros::NodeHandle nhp("~"); // "private" nodehandle, used to access private parameters
        nhp.param<double>("kv",kv,1.0);
        nhp.param<double>("kw",kw,1.0);
        
        // Initialize other parameters
        Kv = kv*Eigen::Matrix3d::Identity()
        Kw = kw*Eigen::Matrix3d::Identity()
        
        // Get camera parameters
        std::cout << "Getting camera parameters on topic: "+cameraName+"/camera_info" << std::endl;
        gotCamParam = false;
        camInfoSub = nh.subscribe(cameraName+"/camera_info",1,&Controller::camInfoCB,this);
        ROS_DEBUG("Waiting for camera parameters ...");
        do {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        } while (!(ros::isShuttingDown()) and !gotCamParam);
        ROS_DEBUG("Got camera parameters");
        
        // Get service handle for setting homography reference
        setHomogReferenceClient = nh.serviceClient<homography_vsc_cl::ReferencePoint>("set_reference");
        
        // publishers
        velPubHomog = nh.advertise<geometry_msgs::Twist>("desVelHomog",10);
        velPubMocap = nh.advertise<geometry_msgs::Twist>("desVelMocap",10);
        
        // Subscribers
        joySub = nh.subscribe("joy",1,&Controller::joyCB,this);
        homogSolnSub = nh.subscribe("homogDecompSoln",1,&Controller::homogCB,this);
        
        // Timers
        //mocap
        //desired
        //zhats
    }
    
    void homogCB(const homography_vsc_cl::HomogDecompSolns& soln)
    {
        Eigen::Matrix3d temp1;
        Eigen::Matrix3d temp2 = Eigen::Identity();
        temp2.rightCols<1>() = m1;
        Eigen::Matrix3d Lv = temp1*temp2;
        
        Eigen::Vector3d wc = -Kw*qTilde + qTilde.inverse()*wcd*qTilde;
        Eigen::Vector3d phi = Lv*m1.cross(wc) - pedDot;
        Eigen::Vector3d vc = (1.0/alpha1)*Lv.inverse()*(Kv*ev + phi*zhat)
    }
    
    // xbox controller callback
    void joyCB(const sensor_msgs::Joy& msg)
    {
        
    }
    
    // callback for getting camera intrinsic parameters
    void camInfoCB(const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
    {
        //get camera info
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(camInfoMsg);
        camMat = cv::Mat(cam_model.fullIntrinsicMatrix());
        camMat.convertTo(camMat,CV_32FC1);
        
        // Calculate factor used in Lv
        cv::cv2eigen(camMat,camMatFactor);
        camMatFactor.block<2,1>(0,2) << 0, 0;
        
        //unregister subscriber
        camInfoSub.shutdown();
        gotCamParam = true;
    }
    
}; // end Controller class

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_processing_node");
    
    Controller obj;
    
    ros::spin();
    return 0;
}
