#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <homography_vsc_cl/ImagePoints.h>

#include <string>
#include <opencv2/imgproc/imgproc.hpp>

class ImageProcessing
{
    // ROS stuff
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber imageSub;
    image_transport::Publisher imagePub;
    ros::Subscriber camInfoSub;
    ros::Publisher pixelPub;
    
    // Camera Parameters
    bool gotCamParam;
    cv::Mat camMat;
    cv::Mat distCoeffs;
    
    // Image Processing parameters
    int blurKernalSize;
    int erodeKernalSize;
    int dilateKernalSize;
    
public:
    ImageProcessing() : it(nh)
    {
        // Get parameters
        ros::NodeHandle nhp("~"); // "private" nodehandle, used to access private parameters
        std::string cameraName;
        nhp.param<std::string>("cameraName", cameraName, "bebop");
        nhp.param<int>("blurKernalSize", blurKernalSize, 5);
        nhp.param<int>("erodeKernalSize", erodeKernalSize, 5);
        nhp.param<int>("dilateKernalSize", dilateKernalSize, 3);
        
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
        
        // Publisher and subscriber
        imagePub = it.advertise(cameraName+"/image_processed",10);
        pixelPub = nh.advertise<homography_vsc_cl::ImagePoints>("markerPoints",10);
        imageSub = it.subscribe(cameraName+"/image_raw", 1, &ImageProcessing::imageCB,this);
    }
    
    // callback for getting camera intrinsic parameters
    void camInfoCB(const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
    {
        //get camera info
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(camInfoMsg);
        camMat = cv::Mat(cam_model.fullIntrinsicMatrix());
        camMat.convertTo(camMat,CV_64F);
        cam_model.distortionCoeffs().convertTo(distCoeffs,CV_64F);
        
        //unregister subscriber
        camInfoSub.shutdown();
        gotCamParam = true;
    }
    
    // callback for finding markers
    void imageCB(const sensor_msgs::ImageConstPtr& msg)
    {
        // convert to opencv image
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat image;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            image = cv_ptr->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        // basic processing
        cv::Mat imageHSV;
        cv::undistort(image.clone(),image,camMat,distCoeffs);
        cv::GaussianBlur(image,imageHSV,cv::Size(blurKernalSize,blurKernalSize),blurKernalSize,blurKernalSize);
        cv::cvtColor(imageHSV,imageHSV,CV_BGR2HSV);
        
        // Threshold color
        // order is {lower hue, upper hue, lower saturation, upper saturation, lower value, upper value}
        int redThresh[6] = {155, 180, 25, 255, 80, 255};
        int greenThresh[6] = {65, 90, 40, 255, 40, 255};
        int cyanThresh[6] = {95, 105, 60, 255, 150, 255};
        int purpleThresh[6] = {110, 135, 30, 255, 50, 255};
        cv::Mat redMask;
        cv::Mat greenMask;
        cv::Mat cyanMask;
        cv::Mat purpleMask;
        thresholdColor(imageHSV,redMask,redThresh);
        thresholdColor(imageHSV,greenMask,greenThresh);
        thresholdColor(imageHSV,cyanMask,cyanThresh);
        thresholdColor(imageHSV,purpleMask,purpleThresh);
        
        // Find circles
        cv::Point2d redCenter;
        cv::Point2d greenCenter;
        cv::Point2d cyanCenter;
        cv::Point2d purpleCenter;
        double redRadius;
        double greenRadius;
        double cyanRadius;
        double purpleRadius;
        bool foundRed = getCenter(redMask, redCenter, redRadius);
        bool foundGreen = getCenter(greenMask, greenCenter, greenRadius);
        bool foundCyan = getCenter(cyanMask, cyanCenter, cyanRadius);
        bool foundPurple = getCenter(purpleMask, purpleCenter, purpleRadius);
        
        // Draw circles
        if (foundRed) { cv::circle(image,redCenter,redRadius,cv::Scalar(0, 0, 255),2); }
        if (foundGreen) { cv::circle(image,greenCenter,greenRadius,cv::Scalar(0, 255, 0),2); }
        if (foundCyan) { cv::circle(image,cyanCenter,cyanRadius,cv::Scalar(255, 255, 0),2); }
        if (foundPurple) { cv::circle(image,purpleCenter,purpleRadius,cv::Scalar(255, 0, 255),2); }
        
        // Publish image
        cv_ptr->image = image;
        imagePub.publish(cv_ptr->toImageMsg());
        
        // Publish points
        homography_vsc_cl::ImagePoints pointsMsg;
        pointsMsg.header.stamp = msg->header.stamp;
        pointsMsg.pr.x = redCenter.x;       pointsMsg.pr.y = redCenter.y;
        pointsMsg.pg.x = greenCenter.x;     pointsMsg.pg.y = greenCenter.y;
        pointsMsg.pc.x = cyanCenter.x;      pointsMsg.pc.y = cyanCenter.y;
        pointsMsg.pp.x = purpleCenter.x;    pointsMsg.pp.y = purpleCenter.y;
        pointsMsg.features_found = foundRed && foundGreen && foundCyan && foundPurple;
        pixelPub.publish(pointsMsg);
    }
    
    bool getCenter(cv::Mat mask, cv::Point2d& point, double& radius)
    {
        // find contours
        std::vector< std::vector<cv::Point> > contours;
        cv::findContours(mask.clone(), contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
        
        // find bounding circles
        if (contours.size() > 0)
        {
            std::vector<cv::Point2f> center(contours.size());
            std::vector<float> radii(contours.size());
            for (int ii = 0; ii < contours.size(); ii++)
            {
                cv::minEnclosingCircle(contours[ii], center[ii], radii[ii]);
            }
            int index = std::distance(radii.begin(),std::max_element(radii.begin(),radii.end()));
            radius = radii[index];
            point = center[index];
            
            // Check if any on the edge
            cv::Mat contourMask = cv::Mat::zeros(mask.size(),mask.type());
            cv::Mat edgeMask = cv::Mat::zeros(mask.size(),mask.type());
            cv::Mat finalMask = cv::Mat::zeros(mask.size(),mask.type());
            cv::drawContours(contourMask,contours,index,cv::Scalar(255),-1);
            cv::rectangle(edgeMask, cv::Point(0,0), cv::Point(mask.size())-cv::Point(1,1),cv::Scalar(255),4);
            cv::bitwise_and(contourMask,edgeMask,finalMask);
            int pointsOnEdge = cv::countNonZero(finalMask);
            return !pointsOnEdge;
        }
    }
    
    void thresholdColor(cv::Mat& image, cv::Mat& mask, int* thresh)
    {
        // deal with the case where hue wraps (e.g., red)
        if (thresh[0] > thresh[1])
        {
            cv::Mat lowerMask;
            cv::Mat upperMask;
            cv::inRange(image,cv::Scalar(0,thresh[2],thresh[4]),cv::Scalar(thresh[1],thresh[3],thresh[5]),lowerMask);
            cv::inRange(image,cv::Scalar(thresh[0],thresh[2],thresh[4]),cv::Scalar(180,thresh[3],thresh[5]),upperMask);
            cv::bitwise_or(lowerMask,upperMask,mask);
        }
        else
        {
            cv::inRange(image,cv::Scalar(thresh[0],thresh[2],thresh[4]),cv::Scalar(thresh[1],thresh[3],thresh[5]),mask);
        }
        
        // erode and dilate
        cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(erodeKernalSize,erodeKernalSize),cv::Point(-1,-1)));
        cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(dilateKernalSize,dilateKernalSize),cv::Point(-1,-1)));
    }
    
}; // End ImageProcessing class

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_processing_node");
    
    ImageProcessing obj;
    
    ros::spin();
    return 0;
}
