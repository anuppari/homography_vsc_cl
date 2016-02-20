#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <homography_vsc_cl/ImagePoints.h>
#include <homography_vsc_cl/ReferencePoint.h>
#include <homography_vsc_cl/HomogDecompSolns.h>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class HomogDecomp
{
    // ROS stuff
    ros::NodeHandle nh;
    ros::ServiceServer service;
    ros::Subscriber imagePtsSub;
    ros::Subscriber camInfoSub;
    ros::Publisher solnPub;
    
    // Parameters
    bool referenceSet;
    std::vector<cv::Point2d> refPoints;
    cv::Mat mRef;
    homography_vsc_cl::ImagePoints refPointsMsg;
    
    // Camera parameters
    cv::Mat camMat;
    bool gotCamParam;
    
public:
    HomogDecomp()
    {
        // Parameters
        ros::NodeHandle nhp("~"); // "private" nodehandle, used to access private parameters
        std::string cameraName;
        nhp.param<std::string>("cameraName", cameraName, "bebop");
        
        referenceSet = false;
        
        // Get camera parameters
        std::cout << "Getting camera parameters on topic: "+cameraName+"/camera_info" << std::endl;
        gotCamParam = false;
        camInfoSub = nh.subscribe(cameraName+"/camera_info",1,&HomogDecomp::camInfoCB,this);
        ROS_DEBUG("Waiting for camera parameters ...");
        do {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        } while (!(ros::isShuttingDown()) and !gotCamParam);
        ROS_DEBUG("Got camera parameters");
        
        // Start service for setting reference
        service = nh.advertiseService("set_reference", &HomogDecomp::set_reference,this);
        
        // Wait for reference to be set
        do {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        } while (!(ros::isShuttingDown()) and !referenceSet);
        
        // Solution publisher and Subscribe to image points
        solnPub = nh.advertise<homography_vsc_cl::HomogDecompSolns>("homogDecompSoln",10);
        imagePtsSub = nh.subscribe("markerPoints",1,&HomogDecomp::pointCB,this);
    }
    
    // callback for getting camera intrinsic parameters
    void camInfoCB(const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
    {
        //get camera info
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(camInfoMsg);
        camMat = cv::Mat(cam_model.fullIntrinsicMatrix());
        camMat.convertTo(camMat,CV_64F);
        
        //unregister subscriber
        camInfoSub.shutdown();
        gotCamParam = true;
    }
    
    bool set_reference(homography_vsc_cl::ReferencePoint::Request &req, homography_vsc_cl::ReferencePoint::Response &resp)
    {
        // pixels
        refPoints.push_back(cv::Point2d(req.points.pr.x,req.points.pr.y));
        refPoints.push_back(cv::Point2d(req.points.pg.x,req.points.pg.y));
        refPoints.push_back(cv::Point2d(req.points.pc.x,req.points.pc.y));
        refPoints.push_back(cv::Point2d(req.points.pp.x,req.points.pp.y));
        
        // normalized euclidean
        std::vector<cv::Point2d> temp;
        cv::undistortPoints(refPoints,temp,camMat,cv::Mat());
        mRef = cv::Mat::ones(4,3,CV_64F);
        mRef.at<double>(0,0) = temp[0].x; mRef.at<double>(0,1) = temp[0].y; // red
        mRef.at<double>(1,0) = temp[1].x; mRef.at<double>(1,1) = temp[1].y; // green
        mRef.at<double>(2,0) = temp[2].x; mRef.at<double>(2,1) = temp[2].y; // cyan
        mRef.at<double>(3,0) = temp[3].x; mRef.at<double>(3,1) = temp[3].y; // purple
        
        // Reference points msg
        refPointsMsg = req.points;
        //refPointsMsg.header.stamp = ros::Time::now();
        //refPointsMsg.pr.x = temp[0].x; refPointsMsg.pr.y = temp[0].y; // red
        //refPointsMsg.pg.x = temp[1].x; refPointsMsg.pg.y = temp[1].y; // green
        //refPointsMsg.pc.x = temp[2].x; refPointsMsg.pc.y = temp[2].y; // cyan
        //refPointsMsg.pp.x = temp[3].x; refPointsMsg.pp.y = temp[3].y; // purple
        //refPointsMsg.features_found = true;
        
        referenceSet = true;
        
        return true;
    }
    
    void pointCB(const homography_vsc_cl::ImagePoints& points)
    {
        if (points.features_found)
        {
            // get points
            std::vector<cv::Point2d> newPoints(4);
            newPoints[0] = cv::Point2d(points.pr.x,points.pr.y);
            newPoints[1] = cv::Point2d(points.pg.x,points.pg.y);
            newPoints[2] = cv::Point2d(points.pc.x,points.pc.y);
            newPoints[3] = cv::Point2d(points.pp.x,points.pp.y);
            
            // Calculate homography
            cv::Mat G = cv::findHomography(refPoints,newPoints,0);
            cv::Mat H_hat = (camMat.inv(cv::DECOMP_LU)*G)*camMat;
            cv::SVD svd = cv::SVD(H_hat,cv::SVD::NO_UV);
            double svds[3] = {svd.w.at<double>(0,0), svd.w.at<double>(1,0), svd.w.at<double>(2,0)};
            std::sort(svds,svds+3);
            double gamma = svds[1];
            cv::Mat H = (1.0/gamma)*H_hat;
            
            // Decompose
            std::vector<cv::Mat> R, T, n;
            int successful_decomp = cv::decomposeHomographyMat(G,camMat,R,T,n);
            
            // Reduce to two solutions, and flip sign if necessary
            std::vector<cv::Point2d> temp;
            cv::undistortPoints(newPoints,temp,camMat,cv::Mat());
            cv::Mat m = cv::Mat::ones(4,3,CV_64F);
            m.at<double>(0,0) = temp[0].x; m.at<double>(0,1) = temp[0].y; // red
            m.at<double>(1,0) = temp[1].x; m.at<double>(1,1) = temp[1].y; // green
            m.at<double>(2,0) = temp[2].x; m.at<double>(2,1) = temp[2].y; // cyan
            m.at<double>(3,0) = temp[3].x; m.at<double>(3,1) = temp[3].y; // purple
            std::vector<int> goodSolutionIndex;
            for (int ii = 0; ii < successful_decomp; ii++)
            {
                // check if possible solution
                if (!cv::countNonZero(m*R[ii]*n[ii] < 0))
                {
                    goodSolutionIndex.push_back(ii);
                }
                
                // check if rotation is improper
                if (cv::determinant(R[ii]) < 0)
                {
                    R[ii] = -1*R[ii];
                    T[ii] = -1*T[ii];
                }
            }
            
            // Get alpha
            double alphaRed = m.at<double>(0,2)/(H.row(2).dot(mRef.row(0)));
            double alphaGreen = m.at<double>(1,2)/(H.row(2).dot(mRef.row(1)));
            double alphaCyan = m.at<double>(2,2)/(H.row(2).dot(mRef.row(2)));
            double alphaPurple = m.at<double>(3,2)/(H.row(2).dot(mRef.row(3)));
            
            // Construct message
            homography_vsc_cl::HomogDecompSolns msg;
            msg.header.stamp = points.header.stamp;
            
            if (goodSolutionIndex.size() > 0)
            {
                // Convert rotation matrix to quaternion
                std::vector<Eigen::Quaterniond> q(goodSolutionIndex.size());
                for (int ii = 0; ii < goodSolutionIndex.size(); ii++)
                {
                    Eigen::Matrix3d eigR;
                    cv::cv2eigen(R[goodSolutionIndex[ii]],eigR);
                    q[ii] = Eigen::Quaterniond(eigR);
                }
                
                // Construct message
                msg.newPixels = points;
                msg.refPixels = refPointsMsg;
                msg.pose1.position.x = T[goodSolutionIndex[0]].at<double>(0,0);
                msg.pose1.position.y = T[goodSolutionIndex[0]].at<double>(1,0);
                msg.pose1.position.z = T[goodSolutionIndex[0]].at<double>(2,0);
                msg.pose1.orientation.x = q[0].x();
                msg.pose1.orientation.y = q[0].y();
                msg.pose1.orientation.z = q[0].z();
                msg.pose1.orientation.w = q[0].w();
                msg.alphar = alphaRed;
                msg.alphag = alphaGreen;
                msg.alphac = alphaCyan;
                msg.alphap = alphaPurple;
                
                if (goodSolutionIndex.size() > 1)
                {
                    msg.pose2.position.x = T[goodSolutionIndex[1]].at<double>(0,0);
                    msg.pose2.position.y = T[goodSolutionIndex[1]].at<double>(1,0);
                    msg.pose2.position.z = T[goodSolutionIndex[1]].at<double>(2,0);
                    msg.pose2.orientation.x = q[1].x();
                    msg.pose2.orientation.y = q[1].y();
                    msg.pose2.orientation.z = q[1].z();
                    msg.pose2.orientation.w = q[1].w();
                }
            }
            else
            {
                msg.decomp_successful = false;
            }
            
            // publish
            solnPub.publish(msg);
        }
    }
    
}; // end HomogDecomp class

int main(int argc, char** argv)
{
    ros::init(argc, argv, "homography_decomposition_node");
    
    HomogDecomp obj;
    
    ros::spin();
    return 0;
}
