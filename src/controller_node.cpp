#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Joy.h>
#include <homography_vsc_cl/HomogDecompSolns.h>
#include <homography_vsc_cl/SetReference.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <bebop_msgs/Ardrone3CameraStateOrientation.h>

#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <cmath>

Eigen::Matrix<double,4,3> diffMat(const Eigen::Quaterniond);

class Controller
{
    // ROS stuff
    ros::NodeHandle nh;
    ros::Subscriber camInfoSub;
    ros::Subscriber joySub;
    ros::Subscriber panTiltSub;
    ros::Subscriber homogSolnSub;
    ros::ServiceClient setHomogReferenceClient;
    ros::Publisher velPubHomog;
    ros::Publisher velPubMocap;
    tf::TransformListener tfl;
    tf::TransformBroadcaster tfbr;
    ros::Timer mocapControlTimer;
    ros::Timer zhatUpdateTimer;
    ros::Timer refPubTimer;
    ros::Timer desUpdateTimer;
    
    // Camera parameters
    Eigen::Matrix3d camMat;
    bool gotCamParam;
    
    // Controller parameters
    double kv;
    double kw;
    double gamma1;
    double gamma2;
    Eigen::Matrix3d Kv;
    Eigen::Matrix3d Kw;
    int stackSize;
    double intWindowTime;
    double mocapUpdateRate;
    double zhatUpdateRate;
    double desUpdateRate;
    double zLastTime;
    
    // Mocap based control parameters
    std::string imageTFframe;
    std::string redTFframe;
    
    // Homography buffers
    double zhatHomog;
    int homogBuffSize;
    Eigen::VectorXd tBuffHomog;
    Eigen::MatrixXd evBuffHomog;
    Eigen::MatrixXd phiBuffHomog;
    Eigen::MatrixXd uBuffHomog;
    Eigen::MatrixXd YstackHomog;
    Eigen::MatrixXd UstackHomog;
    
    // Mocap buffers
    double zhatMocap;
    int mocapBuffSize;
    Eigen::VectorXd tBuffMocap;
    Eigen::MatrixXd evBuffMocap;
    Eigen::MatrixXd phiBuffMocap;
    Eigen::MatrixXd uBuffMocap;
    Eigen::MatrixXd YstackMocap;
    Eigen::MatrixXd UstackMocap;
    
    // Reference
    Eigen::Vector3d nStar;
    tf::StampedTransform tfRef;
    
    // Desired states/parameters
    double desRadius;
    double desPeriod;
    double desHeight;
    double desLastTime;
    Eigen::Quaterniond qPanTilt;
    Eigen::Quaterniond qd;
    Eigen::Vector3d desCamPos;
    Eigen::Quaterniond desCamOrient;
    Eigen::Vector3d ped;
    Eigen::Vector3d pedLast;
    Eigen::Vector3d pedDot;
    Eigen::Vector3d wcd;
    
public:
    Controller()
    {
        // Parameters
        ros::NodeHandle nhp("~"); // "private" nodehandle, used to access private parameters
        std::string cameraName;
        nhp.param<std::string>("cameraName", cameraName, "bebop");
        nhp.param<double>("kv",kv,1.0);
        nhp.param<double>("kw",kw,1.0);
        nhp.param<double>("gamma1",gamma1,1.0);
        nhp.param<double>("gamma2",gamma2,1.0);
        nhp.param<double>("intWindowTime",intWindowTime,1.0);
        nhp.param<int>("stackSize",stackSize,20);
        nhp.param<double>("mocapUpdateRate",mocapUpdateRate,300.0);
        nhp.param<double>("zhatUpdateRate",zhatUpdateRate,300.0);
        nhp.param<double>("desUpdateRate",desUpdateRate,300.0);
        nhp.param<double>("desRadius",desRadius,1.0);
        nhp.param<double>("desPeriod",desPeriod,45.0);
        nhp.param<double>("desHeight",desHeight,1.0);
        
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
        
        // Initialize other parameters
        qPanTilt.setIdentity();
        Kv = kv*Eigen::Matrix3d::Identity();
        Kw = kw*Eigen::Matrix3d::Identity();
        zhatHomog = 0;
        zhatMocap = zhatHomog;
        
        // Get service handle for setting homography reference
        setHomogReferenceClient = nh.serviceClient<homography_vsc_cl::SetReference>("set_reference");
        
        // publishers
        velPubHomog = nh.advertise<geometry_msgs::Twist>("desVelHomog",10);
        velPubMocap = nh.advertise<geometry_msgs::Twist>("desVelMocap",10);
        
        // some subscribers
        joySub = nh.subscribe("joy",1,&Controller::joyCB,this);
        panTiltSub = nh.subscribe("bebop/states/ARDrone3/CameraState/Orientation",1,&Controller::panTiltCB,this);
    }
    
    void initializeStates()
    {
        // Initialize buffers
        homogBuffSize = (int) intWindowTime*30.0;
        tBuffHomog = Eigen::VectorXd::Zero(homogBuffSize);
        evBuffHomog = Eigen::MatrixXd::Zero(3,homogBuffSize);
        phiBuffHomog = Eigen::MatrixXd::Zero(3,homogBuffSize);
        uBuffHomog = Eigen::MatrixXd::Zero(3,homogBuffSize);
        YstackHomog = Eigen::MatrixXd::Zero(3,stackSize);
        UstackHomog = Eigen::MatrixXd::Zero(3,stackSize);
        
        mocapBuffSize = (int) intWindowTime*mocapUpdateRate;
        tBuffMocap = Eigen::VectorXd::Zero(mocapBuffSize);
        evBuffMocap = Eigen::MatrixXd::Zero(3,mocapBuffSize);
        phiBuffMocap = Eigen::MatrixXd::Zero(3,mocapBuffSize);
        uBuffMocap = Eigen::MatrixXd::Zero(3,mocapBuffSize);
        YstackMocap = Eigen::MatrixXd::Zero(3,stackSize);
        UstackMocap = Eigen::MatrixXd::Zero(3,stackSize);
        
        // Initialize desired
        desCamPos << -1*desRadius, 0, desHeight;
        Eigen::Matrix3d tempRot; 
        tempRot << 0,0,1,-1,0,0,0,-1,0;
        desCamOrient = Eigen::Quaterniond(tempRot);
        desUpdateCB(ros::TimerEvent()); // call once to update signals
        pedDot << 0,0,0; // reset
        
        // zhat update time
        zLastTime = ros::Time::now().toSec();
        
        // Subscribers
        homogSolnSub = nh.subscribe("homogDecompSoln",1,&Controller::homogCB,this);
        
        // Timers
        desUpdateTimer = nh.createTimer(ros::Duration(1.0/desUpdateRate),&Controller::desUpdateCB,this,true);
        mocapControlTimer = nh.createTimer(ros::Duration(1.0/mocapUpdateRate),&Controller::mocapCB,this,true);
        zhatUpdateTimer = nh.createTimer(ros::Duration(1.0/zhatUpdateRate),&Controller::zhatUpdateCB,this,true);
        refPubTimer = nh.createTimer(ros::Duration(1.0/10),&Controller::refPubCB,this,true);
    }
    
    void homogCB(const homography_vsc_cl::HomogDecompSolns& soln)
    {
        // Find right solution
        Eigen::Vector3d n1(soln.n1.x, soln.n1.y, soln.n1.z);
        Eigen::Vector3d n2(soln.n2.x, soln.n2.y, soln.n2.z);
        Eigen::Quaterniond q;
        if ((n1-nStar).squaredNorm() < (n2-nStar).squaredNorm())
        {
            q = (Eigen::Quaterniond(soln.pose1.orientation.w, soln.pose1.orientation.x, soln.pose1.orientation.y, soln.pose1.orientation.z)).inverse();
        }
        else
        {
            q = (Eigen::Quaterniond(soln.pose2.orientation.w, soln.pose2.orientation.x, soln.pose2.orientation.y, soln.pose2.orientation.z)).inverse();
        }
        
        // Other stuff
        Eigen::Vector3d pixels(soln.newPixels.pr.x, soln.newPixels.pr.y, 1);
        double alpha1 = soln.alphar;
        
        // Calculate control and publish
        calculateControl(pixels, q, alpha1, false);
    }
    
    void mocapCB(const ros::TimerEvent& event)
    {
        // Red marker w.r.t. image, and reference w.r.t. image
        ros::Time timeStamp = ros::Time::now();
        tf::StampedTransform tfMarker2Im;
        tf::StampedTransform tfIm2Ref;
        try
        {
            tfl.waitForTransform(imageTFframe,redTFframe,timeStamp,ros::Duration(0.01));
            tfl.lookupTransform(imageTFframe,redTFframe,timeStamp,tfMarker2Im);
            //tfl.waitForTransform(imageTFframe+"_ref",redTFframe,timeStamp,ros::Duration(0.01));
            //tfl.lookupTransform(imageTFframe+"_ref",redTFframe,timeStamp,tfMarker2Ref);
            tfl.waitForTransform(imageTFframe,imageTFframe+"_ref",timeStamp,ros::Duration(0.01));
            tfl.lookupTransform(imageTFframe,imageTFframe+"_ref",timeStamp,tfIm2Ref);
            
        }
        catch(tf::TransformException ex)
        {
            return;
        }
        
        tf::Transform tfMarker2Ref = tfIm2Ref*tfMarker2Im;
        
        // Pixels
        Eigen::Vector3d pixels = camMat*Eigen::Vector3d(tfMarker2Im.getOrigin().getX()/tfMarker2Im.getOrigin().getZ(), tfMarker2Im.getOrigin().getY()/tfMarker2Im.getOrigin().getZ(), 1);
        
        // Orientation
        Eigen::Quaterniond q(tfIm2Ref.getRotation().getW(),tfIm2Ref.getRotation().getX(),tfIm2Ref.getRotation().getY(),tfIm2Ref.getRotation().getZ());
        
        // alpha
        double alpha1 = tfMarker2Ref.getOrigin().getZ()/tfMarker2Im.getOrigin().getZ();
        
        // Calculate control and publish
        calculateControl(pixels, q, alpha1, true);
    }
    
    void zhatUpdateCB(const ros::TimerEvent& event)
    {
        // Time
        double timeNow = ros::Time::now().toSec();
        double delT = timeNow - zLastTime;
        zLastTime = timeNow;
        
        // homography
        double term1 = gamma1*evBuffHomog.rightCols<1>().transpose()*phiBuffHomog.rightCols<1>();
        double term2 = gamma1*gamma2*(YstackHomog.cwiseProduct((-1*YstackHomog*zhatHomog - UstackHomog).eval()).sum());
        double zhatDotHomog = term1 + term2;
        
        // Mocap
        term1 = gamma1*evBuffMocap.rightCols<1>().transpose()*phiBuffMocap.rightCols<1>();
        term2 = gamma1*gamma2*(YstackMocap.cwiseProduct((-1*YstackMocap*zhatMocap - UstackMocap).eval()).sum());
        double zhatDotMocap = term1 + term2;
        
        // Update
        zhatHomog += zhatDotHomog*delT;
        zhatMocap += zhatDotMocap*delT;
    }
    
    void calculateControl(Eigen::Vector3d pixels, Eigen::Quaterniond q, double alpha1, bool forMocap)
    {
        // Signals
        Eigen::Vector3d m1 = camMat.inverse()*pixels;
        Eigen::Vector3d pe(pixels(0),pixels(1),-1*std::log(alpha1));
        
        // errors
        Eigen::Vector3d ev = pe - ped;
        Eigen::Quaterniond qTilde = qd.inverse()*q;
        
        // Lv
        Eigen::Matrix3d camMatFactor = camMat;
        camMatFactor.block<2,1>(0,2) << 0, 0;
        Eigen::Matrix3d temp1 = Eigen::Matrix3d::Identity();
        temp1.topRightCorner<2,1>() = -1*m1.head<2>();
        Eigen::Matrix3d Lv = camMatFactor*temp1;
        
        // Parameter estimate
        double zhat = (forMocap ? zhatMocap : zhatHomog);
        
        // control
        Eigen::Vector3d wc = -Kw*Eigen::Vector3d(qTilde.vec()) + qTilde.inverse()*wcd; // qTilde.inverse()*wcd rotates wcd to current camera frame, equivalent to qTilde^-1*wcd*qTilde in paper
        Eigen::Vector3d phi = Lv*m1.cross(wc) - pedDot;
        Eigen::Vector3d vc = (1.0/alpha1)*Lv.inverse()*(Kv*ev + phi*zhat);
        
        // Construct message
        geometry_msgs::Twist twistMsg;
        twistMsg.linear.x = vc(0);  twistMsg.linear.y = vc(1);  twistMsg.linear.z = vc(2);
        twistMsg.angular.x = wc(0); twistMsg.angular.y = wc(1); twistMsg.angular.z = wc(2);
        
        // Update buffers and publish
        if (forMocap)
        {
            // Update Integration buffers
            tBuffMocap << tBuffMocap.rightCols(mocapBuffSize-1).eval(), ros::Time::now().toSec();
            evBuffMocap << evBuffMocap.rightCols(mocapBuffSize-1).eval(), ev;
            phiBuffMocap << phiBuffMocap.rightCols(mocapBuffSize-1).eval(), phi;
            uBuffMocap << uBuffMocap.rightCols(mocapBuffSize-1).eval(), alpha1*Lv*vc;
            
            // Integrate
            Eigen::VectorXd delTbuff = (tBuffMocap.rightCols(mocapBuffSize-1) - tBuffMocap.leftCols(mocapBuffSize-1));
            Eigen::Vector3d PHI = 0.5*((phiBuffMocap.leftCols(mocapBuffSize-1) + phiBuffMocap.rightCols(mocapBuffSize-1))*delTbuff.asDiagonal()).rowwise().sum();
            Eigen::Vector3d scriptY = evBuffMocap.rightCols<1>() - evBuffMocap.leftCols<1>() - PHI;
            Eigen::Vector3d scriptU = 0.5*((uBuffMocap.leftCols(mocapBuffSize-1) + uBuffMocap.rightCols(mocapBuffSize-1))*delTbuff.asDiagonal()).rowwise().sum();
            
            // Add data to stack
            int index;
            double minVal = YstackMocap.colwise().squaredNorm().minCoeff(&index);
            if (scriptY.squaredNorm() > minVal)
            {
                YstackMocap.col(index) = scriptY;
                UstackMocap.col(index) = scriptU;
            }
            
            // publish
            velPubMocap.publish(twistMsg);
        }
        else
        {
            // Update Integration buffers
            tBuffHomog << tBuffHomog.rightCols(homogBuffSize-1).eval(), ros::Time::now().toSec();
            evBuffHomog << evBuffHomog.rightCols(homogBuffSize-1).eval(), ev;
            phiBuffHomog << phiBuffHomog.rightCols(homogBuffSize-1).eval(), phi;
            uBuffHomog << uBuffHomog.rightCols(homogBuffSize-1).eval(), alpha1*Lv*vc;
            
            // Integrate
            Eigen::VectorXd delTbuff = (tBuffHomog.rightCols(homogBuffSize-1) - tBuffHomog.leftCols(homogBuffSize-1));
            Eigen::Vector3d PHI = 0.5*((phiBuffHomog.leftCols(homogBuffSize-1) + phiBuffHomog.rightCols(homogBuffSize-1))*delTbuff.asDiagonal()).rowwise().sum();
            Eigen::Vector3d scriptY = evBuffHomog.rightCols<1>() - evBuffHomog.leftCols<1>() - PHI;
            Eigen::Vector3d scriptU = 0.5*((uBuffHomog.leftCols(homogBuffSize-1) + uBuffHomog.rightCols(homogBuffSize-1))*delTbuff.asDiagonal()).rowwise().sum();
            
            // Add data to stack
            int index;
            double minVal = YstackHomog.colwise().squaredNorm().minCoeff(&index);
            if (scriptY.squaredNorm() > minVal)
            {
                YstackHomog.col(index) = scriptY;
                UstackHomog.col(index) = scriptU;
            }
            
            // publish
            velPubHomog.publish(twistMsg);
        }
    }
    
    void desUpdateCB(const ros::TimerEvent& event)
    {
        // time
        ros::Time timestamp = ros::Time::now();
        double delT = timestamp.toSec() - desLastTime;
        desLastTime = timestamp.toSec();
        
        // update velocities
        Eigen::Vector3d vcd = qPanTilt*Eigen::Vector3d(2*M_PI*desRadius/desPeriod, 0, 0);
        wcd = qPanTilt*Eigen::Vector3d(0,2*M_PI/desPeriod,0);
        
        // Integrate
        desCamPos += desCamOrient*vcd*delT;
        Eigen::Vector4d camOrientTemp(desCamOrient.w(),desCamOrient.x(),desCamOrient.y(),desCamOrient.z());
        camOrientTemp += 0.5*diffMat(desCamOrient)*wcd*delT;
        desCamOrient = Eigen::Quaterniond(camOrientTemp(0),camOrientTemp(1),camOrientTemp(2),camOrientTemp(3));
        desCamOrient.normalize();
        
        // Publish tf
        tfbr.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(desCamOrient.x(),desCamOrient.y(),desCamOrient.z(),desCamOrient.w()),tf::Vector3(desCamPos(0),desCamPos(1),desCamPos(2))), timestamp, "world", imageTFframe+"_des"));
        
        // Calculate extra signals
        tf::StampedTransform tfDes2Ref;
        tf::StampedTransform tfMarker2Des;
        try
        {
            tfl.waitForTransform(imageTFframe+"_ref",imageTFframe+"_des",timestamp,ros::Duration(0.01));
            tfl.lookupTransform(imageTFframe+"_ref",imageTFframe+"_des",timestamp,tfDes2Ref);
            tfl.waitForTransform(imageTFframe+"_des",redTFframe,timestamp,ros::Duration(0.01));
            tfl.lookupTransform(imageTFframe+"_des",redTFframe,timestamp,tfMarker2Des);
        }
        catch(tf::TransformException ex)
        {
            return;
        }
        tf::Transform tfMarker2Ref = tfDes2Ref*tfMarker2Des;
        
        qd = Eigen::Quaterniond(tfDes2Ref.getRotation().getW(),tfDes2Ref.getRotation().getX(),tfDes2Ref.getRotation().getY(),tfDes2Ref.getRotation().getZ());
        Eigen::Vector3d m1d(tfMarker2Des.getOrigin().getX()/tfMarker2Des.getOrigin().getZ(), tfMarker2Des.getOrigin().getY()/tfMarker2Des.getOrigin().getZ(), 1);
        Eigen::Vector3d p1d = camMat*m1d;
        double alpha1d = tfMarker2Ref.getOrigin().getZ()/tfMarker2Des.getOrigin().getZ();
        ped << p1d(0), p1d(1), -1*std::log(alpha1d);
        pedDot = (ped - pedLast)/delT;
        pedLast = ped;
    }
    
    // xbox controller callback, for setting reference
    void joyCB(const sensor_msgs::Joy& msg)
    {
        if (msg.buttons[0]) // button for (re)setting reference
        {
            // stop subscribers/timers
            homogSolnSub.shutdown();
            desUpdateTimer.stop();
            mocapControlTimer.stop();
            zhatUpdateTimer.stop();
            refPubTimer.stop();
            
            // Get reference pose
            try
            {
                tfl.waitForTransform("world",imageTFframe,msg.header.stamp,ros::Duration(0.01));
                tfl.lookupTransform("world",imageTFframe,msg.header.stamp,tfRef);
                tfRef.child_frame_id_ = imageTFframe+"_ref";
                refPubCB(ros::TimerEvent());
            }
            catch(tf::TransformException ex)
            {
                return;
            }
            
            // Set reference in homography node
            homography_vsc_cl::SetReference srv;
            setHomogReferenceClient.call(srv);
            
            // (re)initialize buffers and subscribers/timers
            initializeStates();
        }
    }
    
    void refPubCB(const ros::TimerEvent& event)
    {
        tfRef.stamp_ = ros::Time::now();
        tfbr.sendTransform(tfRef);
    }
    
    // callback for getting camera intrinsic parameters
    void camInfoCB(const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
    {
        //get camera info
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(camInfoMsg);
        cv::Mat camMatCV = cv::Mat(cam_model.fullIntrinsicMatrix());
        camMatCV.convertTo(camMatCV,CV_64F);
        
        // Calculate factor used in Lv
        cv::cv2eigen(camMatCV,camMat);
        
        //unregister subscriber
        camInfoSub.shutdown();
        gotCamParam = true;
    }
    
    void panTiltCB(const bebop_msgs::Ardrone3CameraStateOrientation::ConstPtr& msg)
    {
        // Get new pan tilt
        double pan = -1*msg->pan*M_PI/180.0; // yaw, in radians
        double tilt = -1*msg->tilt*M_PI/180.0; // pitch, in radians
        Eigen::Quaterniond qPanTiltNew((Eigen::AngleAxisd(pan,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(tilt,Eigen::Vector3d::UnitY())).inverse());
        
        // Adjust desired
        desCamOrient *= qPanTilt*qPanTiltNew.inverse();
        qPanTilt = qPanTiltNew;
    }
}; // end Controller class

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
    ros::init(argc, argv, "controller_node");
    
    Controller obj;
    
    ros::spin();
    return 0;
}