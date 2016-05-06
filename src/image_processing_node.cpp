#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
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
    tf::TransformListener tfl;
    image_transport::ImageTransport it;
    image_transport::Subscriber imageSub;
    image_transport::Publisher imagePub, imageThesholdPub;
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
    
    bool useMocap;
    
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
        nhp.param<bool>("useMocap", useMocap, false);
        
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
        imageThesholdPub = it.advertise(cameraName+"/image_threshold",10);
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
        int redThresh[6] = {170, 5, 50, 255, 150, 255};
        int greenThresh[6] = {70, 89, 50, 255, 100, 255};
        int cyanThresh[6] = {90, 114, 50, 255, 150, 255};
        int purpleThresh[6] = {110, 145, 30, 255, 150, 255};
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
        if (foundRed) { cv::circle(image,redCenter,redRadius,cv::Scalar(0, 0, 255),2); cv::circle(image,redCenter,2,cv::Scalar(0, 0, 255),2);}
        if (foundGreen) { cv::circle(image,greenCenter,greenRadius,cv::Scalar(0, 255, 0),2); cv::circle(image,greenCenter,2,cv::Scalar(0, 255, 0),2); }
        if (foundCyan) { cv::circle(image,cyanCenter,cyanRadius,cv::Scalar(255, 255, 0),2); cv::circle(image,cyanCenter,2,cv::Scalar(255, 255, 0),2); }
        if (foundPurple) { cv::circle(image,purpleCenter,purpleRadius,cv::Scalar(255, 0, 255),2); cv::circle(image,purpleCenter,2,cv::Scalar(255, 0, 255),2); }
        
        cv::Mat p1, p2, p3, p4;
        
        try
        {
            tf::StampedTransform redWrtCamera, greenWrtCamera, cyanWrtCamera, purpleWrtCamera;
            tfl.waitForTransform("bebop_image", "ugv1", ros::Time(0), ros::Duration(0.1));
            tfl.lookupTransform("bebop_image", "ugv1", ros::Time(0), redWrtCamera);
            tfl.waitForTransform("bebop_image", "ugv2", ros::Time(0), ros::Duration(0.1));
            tfl.lookupTransform("bebop_image", "ugv2", ros::Time(0), greenWrtCamera);
            tfl.waitForTransform("bebop_image", "ugv3", ros::Time(0), ros::Duration(0.1));
            tfl.lookupTransform("bebop_image", "ugv3", ros::Time(0), cyanWrtCamera);
            tfl.waitForTransform("bebop_image", "ugv4", ros::Time(0), ros::Duration(0.1));
            tfl.lookupTransform("bebop_image", "ugv4", ros::Time(0), purpleWrtCamera);
            cv::Mat m1Bar = cv::Mat::zeros(3,1,CV_64F), m2Bar = cv::Mat::zeros(3,1,CV_64F), m3Bar = cv::Mat::zeros(3,1,CV_64F), m4Bar = cv::Mat::zeros(3,1,CV_64F);
            m1Bar.at<double>(0,0) = redWrtCamera.getOrigin().getX(); m1Bar.at<double>(1,0) = redWrtCamera.getOrigin().getY(); m1Bar.at<double>(2,0) = redWrtCamera.getOrigin().getZ();
            m2Bar.at<double>(0,0) = greenWrtCamera.getOrigin().getX(); m2Bar.at<double>(1,0) = greenWrtCamera.getOrigin().getY(); m2Bar.at<double>(2,0) = greenWrtCamera.getOrigin().getZ();
            m3Bar.at<double>(0,0) = cyanWrtCamera.getOrigin().getX(); m3Bar.at<double>(1,0) = cyanWrtCamera.getOrigin().getY(); m3Bar.at<double>(2,0) = cyanWrtCamera.getOrigin().getZ();
            m4Bar.at<double>(0,0) = purpleWrtCamera.getOrigin().getX(); m4Bar.at<double>(1,0) = purpleWrtCamera.getOrigin().getY(); m4Bar.at<double>(2,0) = purpleWrtCamera.getOrigin().getZ();
            p1 = camMat*((1/m1Bar.at<double>(2,0))*m1Bar);
            p2 = camMat*((1/m2Bar.at<double>(2,0))*m2Bar);
            p3 = camMat*((1/m3Bar.at<double>(2,0))*m3Bar);
            p4 = camMat*((1/m4Bar.at<double>(2,0))*m4Bar);
            cv::circle(image,cv::Point2d(p1.at<double>(0,0),p1.at<double>(1,0)),2,cv::Scalar(0,0,100),2);
            cv::circle(image,cv::Point2d(p2.at<double>(0,0),p2.at<double>(1,0)),2,cv::Scalar(0,100,0),2);
            cv::circle(image,cv::Point2d(p3.at<double>(0,0),p3.at<double>(1,0)),2,cv::Scalar(100,100,0),2);
            cv::circle(image,cv::Point2d(p4.at<double>(0,0),p4.at<double>(1,0)),2,cv::Scalar(100,0,100),2);
        }
        catch (tf::TransformException ex)
        {
            std::cout << "failed to set desired" << std::endl;
        }
        
        
        // Publish image
        cv_ptr->image = image;
        imagePub.publish(cv_ptr->toImageMsg());
        //std::cout << "hi" << std::endl;
        cv::Mat imageBinary;
        cv::cvtColor(redMask + greenMask + cyanMask + purpleMask,imageBinary,CV_GRAY2BGR);
        cv_ptr->image = imageBinary;
        imageThesholdPub.publish(cv_ptr->toImageMsg());
        
        // Publish points
        homography_vsc_cl::ImagePoints pointsMsg;
        pointsMsg.header.stamp = msg->header.stamp;
        if (!useMocap)
        {
            pointsMsg.pr.x = redCenter.x;       pointsMsg.pr.y = redCenter.y;
            pointsMsg.pg.x = greenCenter.x;     pointsMsg.pg.y = greenCenter.y;
            pointsMsg.pc.x = cyanCenter.x;      pointsMsg.pc.y = cyanCenter.y;
            pointsMsg.pp.x = purpleCenter.x;    pointsMsg.pp.y = purpleCenter.y;
        }
        else
        {
            pointsMsg.pr.x = p1.at<double>(0,0);    pointsMsg.pr.y = p1.at<double>(1,0);
            pointsMsg.pg.x = p2.at<double>(0,0);    pointsMsg.pg.y = p2.at<double>(1,0);
            pointsMsg.pc.x = p3.at<double>(0,0);    pointsMsg.pc.y = p3.at<double>(1,0);
            pointsMsg.pp.x = p4.at<double>(0,0);    pointsMsg.pp.y = p4.at<double>(1,0);
        }
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
            std::vector<double> area(contours.size());
            for (int ii = 0; ii < contours.size(); ii++)
            {
                cv::Moments moments = cv::moments(contours[ii],false);
                cv::minEnclosingCircle(contours[ii], center[ii], radii[ii]);
                center[ii] = cv::Point2d(moments.m10/moments.m00, moments.m01/moments.m00);
                area[ii] = moments.m00;
            }
            int index = std::distance(area.begin(),std::max_element(area.begin(),area.end()));
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
