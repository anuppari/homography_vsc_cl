#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Joy.h>
#include <homography_vsc_cl/HomogDecompSolns.h>
#include <homography_vsc_cl/Output.h>
#include <homography_vsc_cl/Debug.h>
#include <homography_vsc_cl/SetReference.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
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
    ros::Subscriber actualVelSub;
    ros::ServiceClient setHomogReferenceClient;
    ros::Publisher velPubHomog;
    ros::Publisher velPubMocap;
    ros::Publisher outputPub;
    ros::Publisher debugMocapPub;
    ros::Publisher debugHomogPub;
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
    double kvXY;
    double kvZ;
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
    bool useActualVelocities;
    Eigen::Vector3d vcActual;
    Eigen::Vector3d wcActual;
    double zStar;
    double filterAlpha; // in range [0,1]
    
    // Mocap based control parameters
    std::string cameraName;
    std::string imageTFframe;
    std::string redTFframe;
    
    // Homography buffers
    double zhatHomog;
    int homogBuffSize;
    Eigen::Quaterniond qTildeLastHomog;
    Eigen::MatrixXd tBuffHomog;
    Eigen::MatrixXd evBuffHomog;
    Eigen::MatrixXd phiBuffHomog;
    Eigen::MatrixXd uBuffHomog;
    Eigen::MatrixXd YstackHomog;
    Eigen::MatrixXd UstackHomog;
    
    // Mocap buffers
    double zhatMocap;
    int mocapBuffSize;
    Eigen::Quaterniond qTildeLastMocap;
    Eigen::MatrixXd tBuffMocap;
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
        //std::string cameraName;
        nhp.param<std::string>("cameraName", cameraName, "bebop");
        nhp.param<std::string>("imageTFframe", imageTFframe, "bebop_image");
        nhp.param<std::string>("redTFframe", redTFframe, "ugv1");
        nhp.param<double>("kvXY",kvXY,5.0);
        nhp.param<double>("kvZ",kvZ,5.0);
        nhp.param<double>("kw",kw,5.0);
        nhp.param<double>("gamma1",gamma1,0.002);
        nhp.param<double>("gamma2",gamma2,50.0);
        nhp.param<double>("intWindowTime",intWindowTime,1.0);
        nhp.param<int>("stackSize",stackSize,20);
        nhp.param<double>("mocapUpdateRate",mocapUpdateRate,300.0);
        nhp.param<double>("zhatUpdateRate",zhatUpdateRate,500.0);
        nhp.param<double>("desUpdateRate",desUpdateRate,100.0);
        nhp.param<bool>("useActualVelocities",useActualVelocities,true);
        nhp.param<double>("desRadius",desRadius,2.0);
        nhp.param<double>("desPeriod",desPeriod,45.0);
        nhp.param<double>("desHeight",desHeight,1.0);
        nhp.param<double>("filterAlpha",filterAlpha,0.2);
        
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
        Kv = (Eigen::Vector3d(kvXY, kvXY, kvZ)).asDiagonal();
        Kw = kw*Eigen::Matrix3d::Identity();
        zStar = 0;
        vcActual = Eigen::Vector3d::Zero();
        wcActual = Eigen::Vector3d::Zero();
        
        // Get service handle for setting homography reference
        setHomogReferenceClient = nh.serviceClient<homography_vsc_cl::SetReference>("set_reference");
        
        // publishers
        velPubHomog = nh.advertise<geometry_msgs::Twist>("desVelHomog",10);
        velPubMocap = nh.advertise<geometry_msgs::Twist>("desVelMocap",10);
        outputPub = nh.advertise<homography_vsc_cl::Output>("controller_output",10);
        debugMocapPub = nh.advertise<homography_vsc_cl::Debug>("controller_debug_mocap",10);
        debugHomogPub = nh.advertise<homography_vsc_cl::Debug>("controller_debug_homog",10);
        
        // some subscribers
        actualVelSub = nh.subscribe(imageTFframe+"/body_vel",1,&Controller::actualVelCB,this);
        joySub = nh.subscribe("joy",1,&Controller::joyCB,this);
        panTiltSub = nh.subscribe("bebop/states/ARDrone3/CameraState/Orientation",1,&Controller::panTiltCB,this);
    }
    
    void initializeStates()
    {
        // Initialize buffers
        homogBuffSize = (int) intWindowTime*30.0;
        tBuffHomog.resize(1,0);
        evBuffHomog.resize(3,0);
        phiBuffHomog.resize(3,0);
        uBuffHomog.resize(3,0);
        YstackHomog = Eigen::MatrixXd::Zero(3,stackSize);
        UstackHomog = Eigen::MatrixXd::Zero(3,stackSize);
        
        mocapBuffSize = (int) intWindowTime*mocapUpdateRate;
        tBuffMocap.resize(1,0);
        evBuffMocap.resize(3,0);
        phiBuffMocap.resize(3,0);
        uBuffMocap.resize(3,0);
        YstackMocap = Eigen::MatrixXd::Zero(3,stackSize);
        UstackMocap = Eigen::MatrixXd::Zero(3,stackSize);
        
        // Intialize parameter estimates
        zhatHomog = 0.01;
        zhatMocap = zhatHomog;
        
        // set last time
        desLastTime = ros::Time::now().toSec();
        zLastTime = desLastTime;
        
        // initialize desired
        double desOrig[2] = {0.0,0.0};
        bool desiredSet = false;
        while (!desiredSet)
        {
            desOrig[0] = 0.0;
            desOrig[1] = 0.0;
            try
            {
                tf::StampedTransform turtleWrtWorldTemp;
                tfl.waitForTransform("world", "ugv1", ros::Time(0), ros::Duration(0.1));
                tfl.lookupTransform("world", "ugv1", ros::Time(0), turtleWrtWorldTemp);
                desOrig[0] = desOrig[0] + turtleWrtWorldTemp.getOrigin().getX();
                desOrig[1] = desOrig[1] + turtleWrtWorldTemp.getOrigin().getY();
                tfl.waitForTransform("world", "ugv2", ros::Time(0), ros::Duration(0.1));
                tfl.lookupTransform("world", "ugv2", ros::Time(0), turtleWrtWorldTemp);
                desOrig[0] = desOrig[0] + turtleWrtWorldTemp.getOrigin().getX();
                desOrig[1] = desOrig[1] + turtleWrtWorldTemp.getOrigin().getY();
                tfl.waitForTransform("world", "ugv3", ros::Time(0), ros::Duration(0.1));
                tfl.lookupTransform("world", "ugv3", ros::Time(0), turtleWrtWorldTemp);
                desOrig[0] = desOrig[0] + turtleWrtWorldTemp.getOrigin().getX();
                desOrig[1] = desOrig[1] + turtleWrtWorldTemp.getOrigin().getY();
                tfl.waitForTransform("world", "ugv4", ros::Time(0), ros::Duration(0.1));
                tfl.lookupTransform("world", "ugv4", ros::Time(0), turtleWrtWorldTemp);
                desOrig[0] = desOrig[0] + turtleWrtWorldTemp.getOrigin().getX();
                desOrig[1] = desOrig[1] + turtleWrtWorldTemp.getOrigin().getY();
                desiredSet = true;
            }
            catch (tf::TransformException ex)
            {
                std::cout << "failed to set desired" << std::endl;
            }
        }
        desCamPos << -1*desRadius + desOrig[0]/4.0, desOrig[1]/4.0, desHeight;
        Eigen::Matrix3d tempRot; 
        tempRot << 0,0,1,-1,0,0,0,-1,0;
        qPanTilt.setIdentity();
        desCamOrient = Eigen::Quaterniond(tempRot);
        desUpdateCB(ros::TimerEvent()); // call once to update signals
        pedDot << 0,0,0; // reset
        
        // Subscribers
        homogSolnSub = nh.subscribe("homogDecompSoln",1,&Controller::homogCB,this);
        
        // Timers
        desUpdateTimer = nh.createTimer(ros::Duration(1.0/desUpdateRate),&Controller::desUpdateCB,this,false);
        mocapControlTimer = nh.createTimer(ros::Duration(1.0/mocapUpdateRate),&Controller::mocapCB,this,false);
        zhatUpdateTimer = nh.createTimer(ros::Duration(1.0/zhatUpdateRate),&Controller::zhatUpdateCB,this,false);
        refPubTimer = nh.createTimer(ros::Duration(1.0/10),&Controller::refPubCB,this,false);
    }
    
    void homogCB(const homography_vsc_cl::HomogDecompSolns& soln)
    {
        if (soln.decomp_successful)
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
            
            //debug
            homography_vsc_cl::Debug msg;
            msg.header.stamp = ros::Time::now();
            msg.q.x = q.x();
            msg.q.y = q.y();
            msg.q.z = q.z();
            msg.q.w = q.w();
            msg.alpha = alpha1;
            msg.zStar = zStar;
            msg.newPixels.pr.x = pixels.x();
            msg.newPixels.pr.y = pixels.y();
            debugHomogPub.publish(msg);
            
            if (alpha1 > 0)
            {
                // Calculate control and publish
                calculateControl(pixels, q, alpha1, false);
            }
        }
    }
    
    void mocapCB(const ros::TimerEvent& event)
    {
        // Red marker w.r.t. image, and reference w.r.t. image
        tf::StampedTransform tfMarker2Im;
        tf::StampedTransform tfIm2Ref;
        try
        {
            tfl.waitForTransform(imageTFframe,redTFframe,ros::Time(0),ros::Duration(0.01));
            tfl.lookupTransform(imageTFframe,redTFframe,ros::Time(0),tfMarker2Im);
            tfl.waitForTransform(imageTFframe+"_ref",imageTFframe,ros::Time(0),ros::Duration(0.01));
            tfl.lookupTransform(imageTFframe+"_ref",imageTFframe,ros::Time(0),tfIm2Ref);
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
        
        //debug
        homography_vsc_cl::Debug msg;
        msg.header.stamp = ros::Time::now();
        msg.q.x = q.x();
        msg.q.y = q.y();
        msg.q.z = q.z();
        msg.q.w = q.w();
        msg.alpha = alpha1;
        msg.zStar = zStar;
        msg.newPixels.pr.x = pixels.x();
        msg.newPixels.pr.y = pixels.y();
        debugMocapPub.publish(msg);
    
        if (alpha1 > 0)
        {
            // Calculate control and publish
            calculateControl(pixels, q, alpha1, true);       
        }
    }
    
    void zhatUpdateCB(const ros::TimerEvent& event)
    {
        // Time
        ros::Time timestamp = ros::Time::now();
        double timeNow = timestamp.toSec();
        double delT = timeNow - zLastTime;
        zLastTime = timeNow;
        
        //std::cout << delT << std::endl;
        
        // homography
        double zhatDotHomog = 0;
        if (evBuffHomog.cols() > 0)
        {
            double term1 = gamma1*evBuffHomog.rightCols<1>().transpose()*phiBuffHomog.rightCols<1>();
            double term2 = gamma1*gamma2*(YstackHomog.cwiseProduct((-1*YstackHomog*zhatHomog - UstackHomog).eval()).sum());
            zhatDotHomog = term1 + term2;
        }
        
        // Mocap
        double zhatDotMocap = 0;
        if (evBuffMocap.cols() > 0)
        {
            double term1 = gamma1*evBuffMocap.rightCols<1>().transpose()*phiBuffMocap.rightCols<1>();
            double term2 = gamma1*gamma2*(YstackMocap.cwiseProduct((-1*YstackMocap*zhatMocap - UstackMocap).eval()).sum());
            zhatDotMocap = term1 + term2;
            //std::cout << "YstackMocap: \n" << YstackMocap << std::endl;
            //std::cout << "UstackMocap: \n" << UstackMocap << std::endl;
            //std::cout << "-1*YstackMocap*zhatMocap: \n" << -1*YstackMocap*zhatMocap << std::endl;
            //std::cout << "((-1*YstackMocap*zhatMocap - UstackMocap).eval()): \n" << ((-1*YstackMocap*zhatMocap - UstackMocap).eval()) << std::endl;
            //std::cout << "YstackMocap.cwiseProduct((-1*YstackMocap*zhatMocap - UstackMocap).eval()): \n" << YstackMocap.cwiseProduct((-1*YstackMocap*zhatMocap - UstackMocap).eval()) << std::endl;
            //std::cout << "((-1*YstackMocap*zhatMocap - UstackMocap).eval()): \n" << ((-1*YstackMocap*zhatMocap - UstackMocap).eval()) << std::endl;
            //std::cout << "term2: " << term2 << std::endl;
        }
        
        // Update
        zhatHomog += zhatDotHomog*delT;
        zhatMocap += zhatDotMocap*delT;
        
        // Output
        homography_vsc_cl::Output outputMsg;
        outputMsg.header.stamp = timestamp;
        outputMsg.zTildeHomog = zStar - zhatHomog;
        outputMsg.zTildeMocap = zStar - zhatMocap;
        outputMsg.zTildePercentHomog = (zStar - zhatHomog)/zStar;
        outputMsg.zTildePercentMocap = (zStar - zhatMocap)/zStar;
        outputPub.publish(outputMsg);
    }
    
    void calculateControl(Eigen::Vector3d pixels, Eigen::Quaterniond q, double alpha1, bool forMocap)
    {
        // Signals
        Eigen::Vector3d m1 = camMat.inverse()*pixels;
        Eigen::Vector3d pe(pixels(0),pixels(1),-1*std::log(alpha1));
        
        // errors
        Eigen::Vector3d ev = pe - ped;
        Eigen::Quaterniond qTilde = qd.inverse()*q;
        
        // maintain continuity of q, and filter
        Eigen::Quaterniond qTildeLast = (forMocap ? qTildeLastMocap : qTildeLastHomog);
        if ((qTildeLast.coeffs() - -1*qTilde.coeffs()).squaredNorm() < (qTildeLast.coeffs() - qTilde.coeffs()).squaredNorm()) { qTilde = Eigen::Quaterniond(-1*qTilde.coeffs()); }
        if (forMocap) { qTildeLastMocap = qTilde; }
        else
        {
            qTilde = Eigen::Quaterniond((1-filterAlpha)*qTilde.coeffs() + filterAlpha*qTildeLast.coeffs());
            qTildeLastHomog = qTilde;
        }
        
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
        Eigen::Vector3d vc = (1.0/alpha1)*Lv.inverse()*(Kv*ev + phi*zStar);
        
        // transforming velocities to bebop body frame
        tf::StampedTransform tfCamera2Body;
        tfl.waitForTransform(cameraName,imageTFframe,ros::Time(0),ros::Duration(0.01));
        tfl.lookupTransform(cameraName,imageTFframe,ros::Time(0),tfCamera2Body);
        Eigen::Quaterniond qCamera2Body(tfCamera2Body.getRotation().getW(),tfCamera2Body.getRotation().getX(),tfCamera2Body.getRotation().getY(),tfCamera2Body.getRotation().getZ());
        Eigen::Vector3d vcBody = qCamera2Body*vc;
        Eigen::Vector3d wcBody = qCamera2Body*wc;
        
        // Construct message
        geometry_msgs::Twist twistMsg;
        twistMsg.linear.x = vcBody(0);  twistMsg.linear.y = vcBody(1);  twistMsg.linear.z = vcBody(2);
        twistMsg.angular.x = wcBody(0); twistMsg.angular.y = wcBody(1); twistMsg.angular.z = wcBody(2);
        
        // Get velocity data for learning from mocap
        Eigen::Vector3d phi2;
        Eigen::Vector3d vc2;
        if (useActualVelocities)
        {
            phi2 = Lv*m1.cross(wcActual) - pedDot;
            vc2 = vcActual;
        }
        else
        {
            phi2 = phi;
            vc2 = vc;
        }
        
        // nan check
        bool nanCheckPass = !std::isnan(vc2.x()) && !std::isnan(vc2.y()) && !std::isnan(vc2.z()) && !isnan(phi2.x()) && !isnan(phi2.y()) && !isnan(phi2.z()) && !std::isnan(wc.x()) && !isnan(wc.y()) && !isnan(wc.z());
        bool infCheckPass = !std::isinf(vc2.x()) && !std::isinf(vc2.y()) && !std::isinf(vc2.z()) && !isinf(phi2.x()) && !isinf(phi2.y()) && !isinf(phi2.z()) && !std::isinf(wc.x()) && !isinf(wc.y()) && !isinf(wc.z());
        
        // error check before push to stacks and buffers
        if (nanCheckPass && infCheckPass)
        {
            // Update buffers and publish
            if (forMocap)
            {
                int numCols = tBuffMocap.cols();
                if (numCols < mocapBuffSize)
                {
                    // Update Integration buffers
                    tBuffMocap.conservativeResize(Eigen::NoChange, numCols+1);
                    tBuffMocap.rightCols<1>() << ros::Time::now().toSec();
                    evBuffMocap.conservativeResize(Eigen::NoChange, numCols+1);
                    evBuffMocap.rightCols<1>() << ev;
                    phiBuffMocap.conservativeResize(Eigen::NoChange, numCols+1);
                    phiBuffMocap.rightCols<1>() << phi2;
                    uBuffMocap.conservativeResize(Eigen::NoChange, numCols+1);
                    uBuffMocap.rightCols<1>() << alpha1*Lv*vc2;
                }
                else
                {
                    // Update Integration buffers
                    tBuffMocap << tBuffMocap.rightCols(mocapBuffSize-1).eval(), ros::Time::now().toSec();
                    evBuffMocap << evBuffMocap.rightCols(mocapBuffSize-1).eval(), ev;
                    phiBuffMocap << phiBuffMocap.rightCols(mocapBuffSize-1).eval(), phi2;
                    uBuffMocap << uBuffMocap.rightCols(mocapBuffSize-1).eval(), alpha1*Lv*vc2;
                    
                    // Integrate
                    Eigen::MatrixXd delTbuff = (tBuffMocap.rightCols(mocapBuffSize-1) - tBuffMocap.leftCols(mocapBuffSize-1));
                    Eigen::Vector3d PHI = 0.5*((phiBuffMocap.leftCols(mocapBuffSize-1) + phiBuffMocap.rightCols(mocapBuffSize-1))*delTbuff.asDiagonal()).rowwise().sum();
                    Eigen::Vector3d scriptY = evBuffMocap.rightCols<1>() - evBuffMocap.leftCols<1>() - PHI;
                    Eigen::Vector3d scriptU = 0.5*((uBuffMocap.leftCols(mocapBuffSize-1) + uBuffMocap.rightCols(mocapBuffSize-1))*delTbuff.asDiagonal()).rowwise().sum();
                    
                    // Add data to stack
                    int index;
                    double minVal = YstackMocap.colwise().squaredNorm().minCoeff(&index);
                    //std::cout << "minVal: " << minVal << std::endl;
                    if (scriptY.squaredNorm() > minVal)
                    {
                        YstackMocap.col(index) = scriptY;
                        UstackMocap.col(index) = scriptU;
                        //std::cout << "index: " << index << std::endl;
                    }
                }
                
                // publish
                velPubMocap.publish(twistMsg);
            }
            else
            {
                int numCols = tBuffHomog.cols();
                if (numCols < homogBuffSize)
                {
                    // Update Integration buffers
                    tBuffHomog.conservativeResize(Eigen::NoChange, numCols+1);
                    tBuffHomog.rightCols<1>() << ros::Time::now().toSec();
                    evBuffHomog.conservativeResize(Eigen::NoChange, numCols+1);
                    evBuffHomog.rightCols<1>() << ev;
                    phiBuffHomog.conservativeResize(Eigen::NoChange, numCols+1);
                    phiBuffHomog.rightCols<1>() << phi2;
                    uBuffHomog.conservativeResize(Eigen::NoChange, numCols+1);
                    uBuffHomog.rightCols<1>() << alpha1*Lv*vc2;
                }
                else
                {
                    // Update Integration buffers
                    tBuffHomog << tBuffHomog.rightCols(homogBuffSize-1).eval(), ros::Time::now().toSec();
                    evBuffHomog << evBuffHomog.rightCols(homogBuffSize-1).eval(), ev;
                    phiBuffHomog << phiBuffHomog.rightCols(homogBuffSize-1).eval(), phi2;
                    uBuffHomog << uBuffHomog.rightCols(homogBuffSize-1).eval(), alpha1*Lv*vc2;
                    
                    // Integrate
                    Eigen::MatrixXd delTbuff = (tBuffHomog.rightCols(homogBuffSize-1) - tBuffHomog.leftCols(homogBuffSize-1));
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
                }
                
                // publish
                velPubHomog.publish(twistMsg);
            }
        }
        else
        {
            // send zeros
            if (forMocap)
            {
                // publish
                velPubMocap.publish(geometry_msgs::Twist());
            }
            else
            {
                // publish
                velPubHomog.publish(geometry_msgs::Twist());
            }
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
        wcd = qPanTilt*Eigen::Vector3d(0,-2*M_PI/desPeriod,0);
        
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
            tfl.waitForTransform(imageTFframe+"_ref",imageTFframe+"_des",ros::Time(0),ros::Duration(0.01));
            tfl.lookupTransform(imageTFframe+"_ref",imageTFframe+"_des",ros::Time(0),tfDes2Ref);
            tfl.waitForTransform(imageTFframe+"_des",redTFframe,ros::Time(0),ros::Duration(0.01));
            tfl.lookupTransform(imageTFframe+"_des",redTFframe,ros::Time(0),tfMarker2Des);
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
        if (msg.buttons[7]) // start button for (re)setting reference
        {
            // stop subscribers/timers
            homogSolnSub.shutdown();
            desUpdateTimer.stop();
            mocapControlTimer.stop();
            zhatUpdateTimer.stop();
            refPubTimer.stop();
            
            try
            {
                // Get reference pose
                tfl.waitForTransform("world",imageTFframe,ros::Time(0),ros::Duration(0.01));
                tfl.lookupTransform("world",imageTFframe,ros::Time(0),tfRef);
                tfRef.child_frame_id_ = imageTFframe+"_ref";
                refPubCB(ros::TimerEvent());
                
                // Get zStar
                tf::StampedTransform tfMarker2Ref;
                tfl.waitForTransform(imageTFframe,redTFframe,ros::Time(0),ros::Duration(0.01));
                tfl.lookupTransform(imageTFframe,redTFframe,ros::Time(0),tfMarker2Ref);
                zStar = tfMarker2Ref.getOrigin().getZ();
                
                // Get nStar
                tf::Vector3 nStarVec = tf::quatRotate(tfRef.getRotation(),tf::Vector3(0,0,-1));
                nStar << nStarVec.getX(), nStarVec.getY(), nStarVec.getZ();
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
        tfRef.stamp_ = ros::Time::now() + ros::Duration(0.1);
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
        // rotation to compensate for the fact that neutral camera has z out, x right, while neutral rpy has x out, y left
        Eigen::Matrix3d tempRot; 
        tempRot << 0,0,1,-1,0,0,0,-1,0;
        Eigen::Quaterniond tempQuat(tempRot);
        
        // Get new pan tilt
        double pan = -1*msg->pan*M_PI/180.0; // yaw, in radians
        double tilt = -1*msg->tilt*M_PI/180.0; // pitch, in radians
        Eigen::Quaterniond qPanTiltNew((tempQuat.inverse()*Eigen::AngleAxisd(pan,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(tilt,Eigen::Vector3d::UnitY())*tempQuat).inverse());
        
        // Adjust desired
        desCamOrient *= qPanTilt*(qPanTiltNew.inverse());
        qPanTilt = qPanTiltNew;
    }
    
    void actualVelCB(const geometry_msgs::TwistStampedConstPtr& twist)
    {
        vcActual << twist->twist.linear.x,twist->twist.linear.y,twist->twist.linear.z;
        wcActual << twist->twist.angular.x,twist->twist.angular.y,twist->twist.angular.z;
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
