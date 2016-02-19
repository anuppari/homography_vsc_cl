#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <homography_vsc_cl/ImagePoints.h>
#include <homography_vsc_cl/ReferencePoint.h>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

class HomogDecomp
{
    // ROS stuff
    ros::NodeHandle nh;
    ros::ServiceServer service;
    ros::Subscriber imagePtsSub;
    ros::Subscriber camInfoSub;
    
    // Parameters
    bool referenceSet;
    std::vector<cv::Point2d> refPoints(4);
    
    // Camera parameters
    cv::Mat camMat;
    
public:
    HomogDecomp()
    {
        // Parameters
        ros::NodeHandle nhp("~"); // "private" nodehandle, used to access private parameters
        std::string cameraName;
        nhp.param<std::string>("cameraName", cameraName, "bebop");
        
        referenceSet = false;
        
        // Start service for setting reference
        service = nh.advertiseService("set_reference", &HomogDecomp::set_reference,this);
        
        // Get camera parameters
        std::cout << "Getting camera parameters on topic: "+cameraName+"/camera_info" << std::endl;
        gotCamParam = false;
        camInfoSub = nh.subscribe(cameraName+"/camera_info",1,&ImageProcessing::camInfoCB,this);
        ROS_DEBUG("Waiting for camera parameters ...");
        do {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        } while (!(ros::isShuttingDown()) and !gotCamParam);
        ROS_DEBUG("Got camera parameters");
        
        // Wait for reference to be set
        do {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        } while (!(ros::isShuttingDown()) and !referenceSet);
        
        // Subscribe to image points
        imagePtsSub = nh.subscribe("markerPoints",1,&HomogDecomp::pointCB,this);
    }
    
    // callback for getting camera intrinsic parameters
    void camInfoCB(const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
    {
        //get camera info
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(camInfoMsg);
        camMat = cv::Mat(cam_model.fullIntrinsicMatrix());
        camMat.convertTo(camMat,CV_32FC1);
        cam_model.distortionCoeffs().convertTo(distCoeffs,CV_32FC1);
        
        //unregister subscriber
        camInfoSub.shutdown();
        gotCamParam = true;
    }
    
    bool set_reference(homography_vsc_cl::ReferencePoint::Request &req, homography_vsc_cl::ReferencePoint::Response &resp)
    {
        refPoints[0] = cv::Point2d(req->points.pr.x,req->points.pr.y);
        refPoints[1] = cv::Point2d(req->points.pg.x,req->points.pg.y);
        refPoints[2] = cv::Point2d(req->points.pc.x,req->points.pc.y);
        refPoints[3] = cv::Point2d(req->points.pp.x,req->points.pp.y);
        referenceSet = true;
        
        return true;
    }
    
    void pointCB(const homography_vsc_cl::ImagePointsConstPtr& points)
    {
        // get points
        std::vector<cv::Point2d> newPoints(4);
        newPoints[0] = cv::Point2d(points->pr.x,points->pr.y);
        newPoints[1] = cv::Point2d(points->pg.x,points->pg.y);
        newPoints[2] = cv::Point2d(points->pc.x,points->pc.y);
        newPoints[3] = cv::Point2d(points->pp.x,points->pp.y);
        
        // Calculate homography
        cv::Mat G = cv::findHomography(refPoints,newPoints,0);
        cv::Mat H_hat = (camMat.inv(cv::DECOMP_LU)*G)*camMat;
        cv::Mat svd = cv::SVD(H_hat,cv::SVD::NO_UV);
        double* svds[3] = {svd.w.at<double>(0,0), svd.w.at<double>(1,0), svd.w.at<double>(2,0)};
        std::sort(svds,svds+3);
        double gamma = svds[1];
        cv::Mat H = (1.0/gamma)*H_hat;
        
        // Decompose
        std::vector<cv::Mat> R, T, n;
        successful_decomp = cv::decomposeHomographyMat(G,camMat,R,T,n);
        
        // Reduce to two solutions, and flip sign if necessary
        for (int ii = 0; ii < successful_decomp; ii++)
        {
            // check if possible solution
            
            
            // check if rotation is improper
            
        }
    }
    
}; // end HomogDecomp class
int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_processing_node");
    
    HomogDecomp obj;
    
    ros::spin();
    return 0;
}
