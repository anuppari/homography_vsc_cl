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
    ros::Subscriber bebopVelSub;
    ros::ServiceClient setHomogReferenceClient;
    ros::Publisher velPubHomog;
    ros::Publisher velPubMocap;
    ros::Publisher outputPub;
    ros::Publisher debugMocapPub;
    ros::Publisher debugHomogPub;
    ros::Publisher debugDesPub;
    ros::Publisher debugZhatPub;
    tf::TransformListener tfl;
    tf::TransformBroadcaster tfbr;
    ros::Timer mocapControlTimer;
    ros::Timer zhatUpdateTimer;
    ros::Timer refPubTimer;
    ros::Timer desUpdateTimer;
    ros::Timer resetHomogBuffersWatchdogTimer;
    ros::Timer predictorTimer;
    
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
    double homogUpdateRate;
    double mocapUpdateRate;
    double zhatUpdateRate;
    double desUpdateRate;
    double zLastTime;
    bool useActualVelocities;
    Eigen::Vector3d vcActual;
    Eigen::Vector3d wcActual;
    Eigen::Vector3d vBebopActual;
    Eigen::Vector3d wBebopActual;
    double zStar;
    double filterAlpha; // in range [0,1]
    bool useZstar;
    bool controlHistoryStack;
    bool startHistoryStack;
    bool fillHistoryStackBegin;
    
    // predictor
    std::deque<ros::Time> velActualTimeBuffer;
    std::deque<double> velActualTimeDiffBuffer;
    std::deque<Eigen::Vector3d> vcActualBuffer;
    std::deque<Eigen::Vector3d> wcActualBuffer;
    std::deque<ros::Time> velTimeBuffer;
    std::deque<double> velTimeDiffBuffer;
    std::deque<Eigen::Vector3d> vcBuffer;
    std::deque<Eigen::Vector3d> wcBuffer;
    bool usePredictor;
    double predictorTimeWindow;
    double predictorUpdateRate;
    Eigen::Vector3d homogPixelNew;
    Eigen::Quaterniond homogQNew;
    double homogAlphaNew;
    Eigen::Vector3d pixelPredict;
    Eigen::Quaterniond qPredict;
    double alphaPredict;
        
    // Mocap based control parameters
    std::string cameraName;
    std::string imageTFframe;
    std::string redTFframe;
    
    // Homography buffers
    double zhatHomog;
    int homogBuffSize;
    Eigen::Quaterniond qTildeLastHomog;
    Eigen::Quaterniond qLastHomog;
    Eigen::MatrixXd tBuffHomog;
    Eigen::MatrixXd evBuffHomog;
    Eigen::MatrixXd phiBuffHomog;
    Eigen::MatrixXd uBuffHomog;
    Eigen::MatrixXd YstackHomog;
    Eigen::MatrixXd UstackHomog;
    Eigen::Vector3d vcLastHomog;
    Eigen::Vector3d wcLastHomog;
    bool firstHomog;
    
    // Mocap buffers
    double zhatMocap;
    int mocapBuffSize;
    Eigen::Quaterniond qTildeLastMocap;
    Eigen::Quaterniond qLastMocap;
    Eigen::MatrixXd tBuffMocap;
    Eigen::MatrixXd evBuffMocap;
    Eigen::MatrixXd phiBuffMocap;
    Eigen::MatrixXd uBuffMocap;
    Eigen::MatrixXd YstackMocap;
    Eigen::MatrixXd UstackMocap;
    Eigen::Vector3d vcLastMocap;
    Eigen::Vector3d wcLastMocap;
    bool firstMocap;
    
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
    Eigen::Quaterniond qdLast;
    Eigen::Vector3d desCamPos;
    Eigen::Quaterniond desCamOrient;
    Eigen::Vector3d ped;
    Eigen::Vector3d pedLast;
    Eigen::Vector3d pedDot;
    Eigen::Vector3d pedDotLast;
    Eigen::Vector3d wcd;
    bool firstDes;
    bool secondDes;
    double filterVc;
    ros::Time startTimeDes;
    
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
        nhp.param<double>("homogUpdateRate",homogUpdateRate,30.0);
        nhp.param<double>("zhatUpdateRate",zhatUpdateRate,500.0);
        nhp.param<double>("desUpdateRate",desUpdateRate,100.0);
        nhp.param<bool>("useActualVelocities",useActualVelocities,true);
        nhp.param<double>("desRadius",desRadius,2.0);
        nhp.param<double>("desPeriod",desPeriod,45.0);
        nhp.param<double>("desHeight",desHeight,1.0);
        nhp.param<double>("filterAlpha",filterAlpha,0.2);
        nhp.param<bool>("useZstar",useZstar,false);
        nhp.param<bool>("controlHistoryStack",controlHistoryStack,false);
        nhp.param<bool>("fillHistoryStackBegin",fillHistoryStackBegin,false);
        nhp.param<double>("filterVc",filterVc,0.0);
        nhp.param<bool>("usePredictor",usePredictor,false);
        nhp.param<double>("predictorTimeWindow",predictorTimeWindow,0.25);
        nhp.param<double>("predictorUpdateRate",predictorUpdateRate,100.0);
        
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
        std::cout << "Got camera parameters" << std::endl;
        
        // Initialize other parameters
        Kv = (Eigen::Vector3d(kvXY, kvXY, kvZ)).asDiagonal();
        Kw = kw*Eigen::Matrix3d::Identity();
        zStar = 0;
        vcActual = Eigen::Vector3d::Zero();
        wcActual = Eigen::Vector3d::Zero();
        vBebopActual = Eigen::Vector3d::Zero();
        wBebopActual = Eigen::Vector3d::Zero();
        
        // Get service handle for setting homography reference
        setHomogReferenceClient = nh.serviceClient<homography_vsc_cl::SetReference>("set_reference");
        
        // publishers
        velPubHomog = nh.advertise<geometry_msgs::Twist>("desVelHomog",1);
        velPubMocap = nh.advertise<geometry_msgs::Twist>("desVelMocap",1);
        outputPub = nh.advertise<homography_vsc_cl::Output>("controller_output",1);
        debugMocapPub = nh.advertise<homography_vsc_cl::Debug>("controller_debug_mocap",1);
        debugHomogPub = nh.advertise<homography_vsc_cl::Debug>("controller_debug_homog",1);
        debugDesPub = nh.advertise<homography_vsc_cl::Debug>("controller_debug_des",1);
        debugZhatPub = nh.advertise<homography_vsc_cl::Debug>("controller_debug_zhat",1);
        
        // subscribers
        actualVelSub = nh.subscribe(imageTFframe+"/body_vel",1,&Controller::actualVelCB,this);
        bebopVelSub = nh.subscribe(cameraName+"/body_vel",1,&Controller::bebopVelCB,this);
        joySub = nh.subscribe("joy",1,&Controller::joyCB,this);
        panTiltSub = nh.subscribe("bebop/states/ARDrone3/CameraState/Orientation",1,&Controller::panTiltCB,this);
    }
    
    void initializeStates()
    {
        // Initialize buffers
        homogBuffSize = (int) (intWindowTime*homogUpdateRate);
        std::cout << "homogBuffSize: " << homogBuffSize << std::endl;
        tBuffHomog.resize(1,0);
        evBuffHomog.resize(3,0);
        phiBuffHomog.resize(3,0);
        uBuffHomog.resize(3,0);
        YstackHomog = Eigen::MatrixXd::Zero(3,stackSize);
        UstackHomog = Eigen::MatrixXd::Zero(3,stackSize);
        vcLastHomog << 0,0,0;
        firstHomog = true;
        
        mocapBuffSize = (int) (intWindowTime*mocapUpdateRate);
        std::cout << "mocapBuffSize: " << mocapBuffSize << std::endl;
        tBuffMocap.resize(1,0);
        evBuffMocap.resize(3,0);
        phiBuffMocap.resize(3,0);
        uBuffMocap.resize(3,0);
        YstackMocap = Eigen::MatrixXd::Zero(3,stackSize);
        UstackMocap = Eigen::MatrixXd::Zero(3,stackSize);
        vcLastMocap << 0,0,0;
        firstMocap = true;
        
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
        
        desCamPos << desOrig[0]/4.0,  desRadius + desOrig[1]/4.0, desHeight;
        Eigen::Matrix3d tempRot;
        tempRot << -1,0,0,0,0,-1,0,-1,0;
        qPanTilt.setIdentity();
        desCamOrient = Eigen::Quaterniond(tempRot);
        desUpdateCB(ros::TimerEvent()); // call once to update signals
        pedDot << 0,0,0; // reset
        firstDes = true;
        secondDes = true;
        startHistoryStack = false;
        
        // Subscribers
        homogSolnSub = nh.subscribe("homogDecompSoln",1,&Controller::homogCB,this);
        
        qdLast.setIdentity();
        qLastHomog.setIdentity();
        qLastMocap.setIdentity();
        
        // Timers
        desUpdateTimer = nh.createTimer(ros::Duration(1.0/desUpdateRate),&Controller::desUpdateCB,this,false);
        mocapControlTimer = nh.createTimer(ros::Duration(1.0/mocapUpdateRate),&Controller::mocapCB,this,false);
        zhatUpdateTimer = nh.createTimer(ros::Duration(1.0/zhatUpdateRate),&Controller::zhatUpdateCB,this,false);
        refPubTimer = nh.createTimer(ros::Duration(1.0/10),&Controller::refPubCB,this,false);
        resetHomogBuffersWatchdogTimer = nh.createTimer(ros::Duration(0.1),&Controller::resetBuffersCB,this,false);
        predictorTimer = nh.createTimer(ros::Duration(1.0/predictorUpdateRate),&Controller::predictorUpdateCB,this,false);
    }
    void predictorUpdateCB(const ros::TimerEvent& event)
    {
        
    }
    void resetBuffersCB(const ros::TimerEvent& event)
    {
        // reset buffers
        tBuffHomog.resize(1,0);
        evBuffHomog.resize(3,0);
        phiBuffHomog.resize(3,0);
        uBuffHomog.resize(3,0);
        vcLastHomog << 0,0,0;
        firstHomog = true;
        startHistoryStack = false;
    }
    
    void homogCB(const homography_vsc_cl::HomogDecompSolns& soln)
    {
        if (soln.decomp_successful)
        {
            resetHomogBuffersWatchdogTimer.stop();
            homography_vsc_cl::Debug msg;//debug
            // Find right solution
            Eigen::Vector3d n1(soln.n1.x, soln.n1.y, soln.n1.z);
            Eigen::Vector3d n2(soln.n2.x, soln.n2.y, soln.n2.z);
            Eigen::Quaterniond q1,q2,q,qNeg;
            q1 = Eigen::Quaterniond(soln.pose1.orientation.w, soln.pose1.orientation.x, soln.pose1.orientation.y, soln.pose1.orientation.z);
            q2 = Eigen::Quaterniond(soln.pose2.orientation.w, soln.pose2.orientation.x, soln.pose2.orientation.y, soln.pose2.orientation.z);
            if ((q2.norm() > 0) && ((n1-nStar).squaredNorm() > (n2-nStar).squaredNorm()))
            {
                q = q2.inverse();
                msg.n = soln.n2;
                msg.t.x = soln.pose2.position.x; msg.t.y = soln.pose2.position.y; msg.t.z = soln.pose2.position.z;
            }
            else
            {
                q = q1.inverse();
                msg.n = soln.n1;
                msg.t.x = soln.pose1.position.x; msg.t.y = soln.pose1.position.y; msg.t.z = soln.pose1.position.z;
            }
            
            // compare to last solution
            if ((qLastHomog.coeffs() - -1*q.coeffs()).squaredNorm() < (qLastHomog.coeffs() - q.coeffs()).squaredNorm()) 
            { 
                q = Eigen::Quaterniond(-1*q.coeffs()); 
            }
            
            // Other stuff
            Eigen::Vector3d pixels(soln.newPixels.pr.x, soln.newPixels.pr.y, 1);
            double alpha1 = soln.alphar;
            
            //debug
            msg.header.stamp = ros::Time::now();
            msg.q.x = q.x();
            msg.q.y = q.y();
            msg.q.z = q.z();
            msg.q.w = q.w();
            msg.nStar.x = nStar.x();
            msg.nStar.y = nStar.y();
            msg.nStar.z = nStar.z();
            msg.alpha = alpha1;
            msg.zStar = zStar;
            msg.newPixels.pr.x = pixels.x();
            msg.newPixels.pr.y = pixels.y();
            msg.H = soln.H;
            
            // Signals
            Eigen::Vector3d pe(pixels(0),pixels(1),-1*std::log(alpha1));
            Eigen::Vector3d ev = pe - ped;
            
            if ((alpha1 > 0))
            {               
                if (usePredictor && (zhatHomog > 0) && (!std::isnan(zhatHomog)) && (!std::isinf(zhatHomog)) && (YstackHomog.cols() > 0) && (std::abs(ev(0)) < 50) && (std::abs(ev(1)) < 50))
                {
                    // predictor
                    predictorUpdate(pixels, q, alpha1, pixelPredict, qPredict, alphaPredict);
                    msg.pixelPredict.x = pixelPredict.x(); msg.pixelPredict.y = pixelPredict.y(); msg.pixelPredict.z = pixelPredict.z();
                    msg.qPredict.w = qPredict.w(); msg.qPredict.x = qPredict.x(); msg.qPredict.y = qPredict.y(); msg.qPredict.z = qPredict.z();
                    msg.alphaPredict = alphaPredict;
                    
                    // Calculate control and publish
                    calculateControl(pixelPredict, qPredict, alphaPredict, msg, false);
                    qLastHomog = qPredict;
                }
                else
                {
                    // Calculate control and publish
                    calculateControl(pixels, q, alpha1, msg, false);
                    qLastHomog = q;
                }

            }
            if (!std::isnan(msg.vc.x) && !std::isnan(msg.vc.y) && !std::isnan(msg.vc.z) && !std::isnan(msg.wc.x) && !std::isnan(msg.wc.y) && !std::isnan(msg.wc.z))
            {
                msg.vcLast.x = vcLastHomog.x(); msg.vcLast.y = vcLastHomog.y(); msg.vcLast.z = vcLastHomog.z(); 
                vcLastHomog << msg.vc.x, msg.vc.y, msg.vc.z;
                msg.wcLast.x = wcLastHomog.x(); msg.wcLast.y = wcLastHomog.y(); msg.wcLast.z = wcLastHomog.z(); 
                wcLastHomog << msg.wc.x, msg.wc.y, msg.wc.z;
                firstHomog = false;
            }
            msg.firstRun = firstHomog;
            debugHomogPub.publish(msg);
            resetHomogBuffersWatchdogTimer.start();
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
        
        // compare to last solution
        if ((qLastMocap.coeffs() - -1*q.coeffs()).squaredNorm() < (qLastMocap.coeffs() - q.coeffs()).squaredNorm()) 
        { 
            q = Eigen::Quaterniond(-1*q.coeffs()); 
        }
        
        // alpha
        double alpha1 = tfMarker2Ref.getOrigin().getZ()/tfMarker2Im.getOrigin().getZ();
        
        //debug
        homography_vsc_cl::Debug msg;
        msg.header.stamp = ros::Time::now();
        msg.q.x = q.inverse().x();
        msg.q.y = q.inverse().y();
        msg.q.z = q.inverse().z();
        msg.q.w = q.inverse().w();
        msg.alpha = alpha1;
        msg.zStar = zStar;
        msg.newPixels.pr.x = pixels.x();
        msg.newPixels.pr.y = pixels.y();
        
        
        if (alpha1 > 0)
        {
            // Calculate control and publish
            calculateControl(pixels, q, alpha1, msg, true);
            qLastMocap = q;
        }
        if (!std::isnan(msg.vc.x) && !std::isnan(msg.vc.y) && !std::isnan(msg.vc.z) && !std::isnan(msg.wc.x) && !std::isnan(msg.wc.y) && !std::isnan(msg.wc.z))
        {
            msg.vcLast.x = vcLastMocap.x(); msg.vcLast.y = vcLastMocap.y(); msg.vcLast.z = vcLastMocap.z(); 
            vcLastMocap << msg.vc.x, msg.vc.y, msg.vc.z;
            msg.wcLast.x = wcLastMocap.x(); msg.wcLast.y = wcLastMocap.y(); msg.wcLast.z = wcLastMocap.z(); 
            wcLastMocap << msg.wc.x, msg.wc.y, msg.wc.z;
            firstMocap = false;
        }
        msg.firstRun = firstMocap;
        debugMocapPub.publish(msg);
    }
    
    void zhatUpdateCB(const ros::TimerEvent& event)
    {
        
        // Time
        ros::Time timestamp = ros::Time::now();
        double timeNow = timestamp.toSec();
        double delT = timeNow - zLastTime;
        zLastTime = timeNow;
        homography_vsc_cl::Debug msg;
        msg.header.stamp = ros::Time::now();
        //std::cout << delT << std::endl;
        
        // homography
        double zhatDotHomog = 0;
        if ((evBuffHomog.cols() > 0 && !controlHistoryStack) || (startHistoryStack && controlHistoryStack))
        {
            double term1 = gamma1*evBuffHomog.rightCols<1>().transpose()*phiBuffHomog.rightCols<1>();
            double term2 = gamma1*gamma2*(YstackHomog.cwiseProduct((-1*YstackHomog*zhatHomog - UstackHomog).eval()).sum());
            zhatDotHomog = term2;
            if (!useZstar)
            {
                zhatDotHomog += term1;
            }
        }
        
        // Mocap
        double zhatDotMocap = 0;
        if ((evBuffMocap.cols() > 0 && !controlHistoryStack) || (startHistoryStack && controlHistoryStack))
        {
            double term1 = gamma1*evBuffMocap.rightCols<1>().transpose()*phiBuffMocap.rightCols<1>();
            double term2 = gamma1*gamma2*(YstackMocap.cwiseProduct((-1*YstackMocap*zhatMocap - UstackMocap).eval()).sum());
            zhatDotMocap = term2;
            if (!useZstar)
            {
                zhatDotMocap += term1;
            }
            //std::cout << "YstackMocap: \n" << YstackMocap << std::endl;
            //std::cout << "UstackMocap: \n" << UstackMocap << std::endl;
            //std::cout << "-1*YstackMocap*zhatMocap: \n" << -1*YstackMocap*zhatMocap << std::endl;
            //std::cout << "((-1*YstackMocap*zhatMocap - UstackMocap).eval()): \n" << ((-1*YstackMocap*zhatMocap - UstackMocap).eval()) << std::endl;
            //std::cout << "YstackMocap.cwiseProduct((-1*YstackMocap*zhatMocap - UstackMocap).eval()): \n" << YstackMocap.cwiseProduct((-1*YstackMocap*zhatMocap - UstackMocap).eval()) << std::endl;
            //std::cout << "((-1*YstackMocap*zhatMocap - UstackMocap).eval()): \n" << ((-1*YstackMocap*zhatMocap - UstackMocap).eval()) << std::endl;
            //std::cout << "term2: " << term2 << std::endl;
            msg.zHatDotTerm1 = term1;
            msg.zHatDotTerm2 = term2;
            debugZhatPub.publish(msg);
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
        outputMsg.zStar = zStar;
        outputMsg.zHatHomog = zhatHomog;
        outputMsg.zHatMocap = zhatMocap;
        outputMsg.zHatDotHomog = zhatDotHomog;
        outputMsg.zHatDotMocap = zhatDotMocap;
        outputPub.publish(outputMsg);
    }
    
    void calculateControl(Eigen::Vector3d pixels, Eigen::Quaterniond q, double alpha1, homography_vsc_cl::Debug& msg, bool forMocap)
    {
        // get position error expressed in world
        try
        {
            tf::StampedTransform desImageWrtImage;
            tfl.waitForTransform("bebop_image", "bebop_image_des", ros::Time(0), ros::Duration(0.1));
            tfl.lookupTransform("bebop_image", "bebop_image_des", ros::Time(0), desImageWrtImage);
            msg.desImageWrtImage.translation.x = desImageWrtImage.getOrigin().getX();
            msg.desImageWrtImage.translation.y = desImageWrtImage.getOrigin().getY();
            msg.desImageWrtImage.translation.z = desImageWrtImage.getOrigin().getZ();
            msg.desImageWrtImage.rotation.x = desImageWrtImage.getRotation().getX();
            msg.desImageWrtImage.rotation.y = desImageWrtImage.getRotation().getY();
            msg.desImageWrtImage.rotation.z = desImageWrtImage.getRotation().getZ();
            msg.desImageWrtImage.rotation.w = desImageWrtImage.getRotation().getW();
            
        }
        catch(tf::TransformException ex)
        {
            return;
        }
        
        // Signals
        Eigen::Vector3d m1 = camMat.inverse()*pixels;
        Eigen::Vector3d pe(pixels(0),pixels(1),-1*std::log(alpha1));
        msg.m1.x = m1.x(); msg.m1.y = m1.y(); msg.m1.z = m1.z(); 
        
        // errors
        Eigen::Vector3d ev = pe - ped;
        //std::cout << "pe\n" << pe <<std::endl;
        //std::cout << "ped\n" << ped <<std::endl;
        Eigen::Quaterniond qTilde = qd.inverse()*q;
        msg.qd.x = qd.x(); msg.qd.y = qd.y(); msg.qd.z = qd.z(); msg.qd.w = qd.w(); 
        msg.ped.x = ped.x(); msg.ped.y = ped.y(); msg.ped.z = ped.z();
        
        bool firstRun = (forMocap ? firstMocap : firstHomog);
        
        // maintain continuity of q, and filter
        Eigen::Quaterniond qTildeLast = (forMocap ? qTildeLastMocap : qTildeLastHomog);
        if ((qTildeLast.coeffs() - -1*qTilde.coeffs()).squaredNorm() < (qTildeLast.coeffs() - qTilde.coeffs()).squaredNorm()) 
        { 
            qTilde = Eigen::Quaterniond(-1*qTilde.coeffs()); 
        }
        
        if (!firstRun)
        {
            qTilde = Eigen::Quaterniond((1-filterAlpha)*qTilde.coeffs() + filterAlpha*qTildeLast.coeffs());
        }
        
        if (forMocap) 
        { 
            qTildeLastMocap = qTilde;
        }
        else
        {
            qTildeLastHomog = qTilde;
        }
        
        msg.ev.x = ev.x(); msg.ev.y = ev.y(); msg.ev.z = ev.z();
        msg.qTilde.x = qTilde.x(); msg.qTilde.y = qTilde.y(); msg.qTilde.z = qTilde.z(); msg.qTilde.w = qTilde.w(); 
        
        // Lv
        Eigen::Matrix3d camMatFactor = camMat;
        camMatFactor.block<2,1>(0,2) << 0, 0;
        Eigen::Matrix3d temp1 = Eigen::Matrix3d::Identity();
        temp1.topRightCorner<2,1>() = -1*m1.head<2>();
        Eigen::Matrix3d Lv = camMatFactor*temp1;
        msg.LvRow1.x = Lv(0,0); msg.LvRow1.y = Lv(0,1); msg.LvRow1.z = Lv(0,2);
        msg.LvRow2.x = Lv(1,0); msg.LvRow2.y = Lv(1,1); msg.LvRow2.z = Lv(1,2);
        msg.LvRow3.x = Lv(2,0); msg.LvRow3.y = Lv(2,1); msg.LvRow3.z = Lv(2,2); 
        
        // Parameter estimate
        double zhat = (forMocap ? zhatMocap : zhatHomog);
        
        // check the position error of the homography using Zhat
        if (!forMocap)
        {
            // calculate m* from m
            Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
            for (int ii = 0; ii<9; ii++)
            {
                H(ii/3,ii%3) = msg.H[ii];
            }
            Eigen::Vector3d m1Star(0.0,0.0,0.0);
            if (alpha1 > 0)
            {
                m1Star = (1/alpha1)*(H.inverse()*m1);
                Eigen::Vector3d m1StarBar = zhat*m1Star;
                Eigen::Vector3d nStar(msg.n.x,msg.n.y,msg.n.z);
                double dStar = nStar.transpose()*m1StarBar;
                Eigen::Vector3d xfNorm(msg.t.x,msg.t.y,msg.t.z);
                Eigen::Vector3d xf = dStar*xfNorm;
                tf::Transform referenceWrtImageZhat;
                referenceWrtImageZhat.setOrigin(tf::Vector3(xf.x(),xf.y(),xf.z()));
                Eigen::Quaterniond qInv = q.inverse();
                referenceWrtImageZhat.setRotation(tf::Quaternion(qInv.x(),qInv.y(),qInv.z(),qInv.w()));
                
                // get position error expressed in world using zhat
                try
                {
                    tfbr.sendTransform(tf::StampedTransform(referenceWrtImageZhat.inverse(), msg.header.stamp, "bebop_image_ref", "bebop_image_zhat"));
                    tf::StampedTransform desImageWrtImageZhat;
                    tfl.waitForTransform("bebop_image_zhat", "bebop_image_des", ros::Time(0), ros::Duration(0.1));
                    tfl.lookupTransform("bebop_image_zhat", "bebop_image_des", ros::Time(0), desImageWrtImageZhat);
                    msg.desImageWrtImageZhat.translation.x = desImageWrtImageZhat.getOrigin().getX();
                    msg.desImageWrtImageZhat.translation.y = desImageWrtImageZhat.getOrigin().getY();
                    msg.desImageWrtImageZhat.translation.z = desImageWrtImageZhat.getOrigin().getZ();
                    msg.desImageWrtImageZhat.rotation.x = desImageWrtImageZhat.getRotation().getX();
                    msg.desImageWrtImageZhat.rotation.y = desImageWrtImageZhat.getRotation().getY();
                    msg.desImageWrtImageZhat.rotation.z = desImageWrtImageZhat.getRotation().getZ();
                    msg.desImageWrtImageZhat.rotation.w = desImageWrtImageZhat.getRotation().getW();
                }
                catch(tf::TransformException ex)
                {
                    return;
                }
                
            }
        }
        
        // control
        Eigen::Vector3d vcLast = (forMocap ? vcLastMocap : vcLastHomog);
        Eigen::Vector3d wcLast = (forMocap ? wcLastMocap : wcLastHomog);
        Eigen::Vector3d wc = -Kw*Eigen::Vector3d(qTilde.vec()) + qTilde.inverse()*wcd; // qTilde.inverse()*wcd rotates wcd to current camera frame, equivalent to qTilde^-1*wcd*qTilde in paper
        if (!firstRun && !std::isnan(wc.x()) && !std::isnan(wc.y()) && !std::isnan(wc.z()))
        {
            wc = (1-filterVc)*wc + filterVc*wcLast;
        }
        msg.wc.x = wc.x(); msg.wc.y = wc.y(); msg.wc.z = wc.z(); 
        Eigen::Vector3d phiTerm1 = Lv*m1.cross(wc);
        Eigen::Vector3d phi = phiTerm1 - pedDot;
        Eigen::Vector3d vcterm1 = (1.0/alpha1)*Lv.inverse()*(Kv*ev);
        Eigen::Vector3d vcterm2 = (1.0/alpha1)*Lv.inverse()*(phi*zhat);
        
        if (useZstar)
        {
            vcterm2 = (1.0/alpha1)*Lv.inverse()*(phi*zStar);
            //std::cout << "vc\n" << vc <<std::endl;
            //std::cout << "1/alpha\n" << 1/alpha1 <<std::endl;
            //std::cout << "Lv inverse\n" << Lv.inverse() <<std::endl;
            //std::cout << "Kv\n" << Kv <<std::endl;
            //std::cout << "ev\n" << ev <<std::endl;
            //std::cout << "Kv*ev\n" << Kv*ev <<std::endl;
            //std::cout << "phi*zStar\n" << phi*zStar <<std::endl;
            //std::cout << std::endl << std::endl;
        }
        Eigen::Vector3d vc = vcterm1 + vcterm2;
        msg.vcterm1.x = vcterm1.x(); msg.vcterm1.y = vcterm1.y(); msg.vcterm1.z = vcterm1.z(); 
        msg.vcterm2.x = vcterm2.x(); msg.vcterm2.y = vcterm2.y(); msg.vcterm2.z = vcterm2.z();
        
        if (!forMocap)
        {
            // Save velocities for predictor
            // update buffers
            velTimeBuffer.push_back(ros::Time::now());
            vcBuffer.push_back(vc);
            wcBuffer.push_back(wc);
            if (velTimeBuffer.size() > 1)
            {
                // get time diff
                std::deque<ros::Time>::iterator timeIt = velTimeBuffer.end();
                double velTimeBufferDiffNew = (*(timeIt-1) - *(timeIt-2)).toSec();
                velTimeDiffBuffer.push_back(velTimeBufferDiffNew);
                //std::cout << "new time diff: " << velActualTimeBufferDiffNew << std::endl;
                double velTimeBufferDiffTotal = (*(timeIt-1)-*(velTimeBuffer.begin())).toSec();
                //std::cout << "total time diff: " << velActualTimeBufferDiffTotal << std::endl;
                
                // check if buffer time window has exceeded the desired time window
                if (velTimeBufferDiffTotal > predictorTimeWindow)
                {
                    // if the buffer time window is larger than the time window pop off the oldest values until it is under the time window
                    while (((*(timeIt-1)-*velTimeBuffer.begin()).toSec()) > predictorTimeWindow)
                    {
                        // remove old data
                        velTimeBuffer.pop_front();
                        vcBuffer.pop_front();
                        wcBuffer.pop_front();
                        velTimeDiffBuffer.pop_front();
                    }
                }
            }
            else
            {
                velTimeDiffBuffer.push_back(0.0);
            }
        }
        
        
        // filter
        if (!firstRun && !std::isnan(vc.x()) && !std::isnan(vc.y()) && !std::isnan(vc.z()))
        {
            vc = (1-filterVc)*vc + filterVc*vcLast;
        }
        
        msg.vc.x = vc.x(); msg.vc.y = vc.y(); msg.vc.z = vc.z();
        msg.phi.x = phi.x(); msg.phi.y = phi.y(); msg.phi.z = phi.z(); 
        msg.zHat = zhat;
        msg.pedDot.x = pedDot.x(); msg.pedDot.y = pedDot.y(); msg.pedDot.z = pedDot.z();
        msg.phiTerm1.x = phiTerm1.x(); msg.phiTerm1.y = phiTerm1.y(); msg.phiTerm1.z = phiTerm1.z(); 
        
        // transforming velocities to bebop body frame
        tf::StampedTransform tfCamera2Body;
        tfl.waitForTransform(cameraName,imageTFframe,ros::Time(0),ros::Duration(0.01));
        tfl.lookupTransform(cameraName,imageTFframe,ros::Time(0),tfCamera2Body);
        Eigen::Vector3d pCamera2Body(tfCamera2Body.getOrigin().getX(),tfCamera2Body.getOrigin().getY(),tfCamera2Body.getOrigin().getZ());
        Eigen::Quaterniond qCamera2Body(tfCamera2Body.getRotation().getW(),tfCamera2Body.getRotation().getX(),tfCamera2Body.getRotation().getY(),tfCamera2Body.getRotation().getZ());
        Eigen::Vector3d vcBody = qCamera2Body*vc - wBebopActual.cross(pCamera2Body);// linear velocity in body = q_cam_wrt_body * vc * q_cam_wrt_body.inverse() - w_body_wrt_world X p_cam_wrt_body
        Eigen::Vector3d wcBody = qCamera2Body*wc;// angular velocity in body = q_cam_wrt_body * wc * q_cam_wrt_body.inverse()
        
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
            msg.vcActual.x = vcActual.x(); msg.vcActual.y = vcActual.y(); msg.vcActual.z = vcActual.z(); 
            msg.wcActual.x = wcActual.x(); msg.wcActual.y = wcActual.y(); msg.wcActual.z = wcActual.z(); 
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
                    msg.PHI.x = PHI.x(); msg.PHI.y = PHI.y(); msg.PHI.z = PHI.z(); 
                    msg.scriptY.x = scriptY.x(); msg.scriptY.y = scriptY.y(); msg.scriptY.z = scriptY.z(); 
                    msg.scriptU.x = scriptU.x(); msg.scriptU.y = scriptU.y(); msg.scriptU.z = scriptU.z(); 
                    
                    // Add data to stack
                    int index;
                    double minVal = YstackMocap.colwise().squaredNorm().minCoeff(&index);
                    msg.minValOld = minVal;
                    msg.newVal = scriptY.squaredNorm();
                    //std::cout << "minVal: " << minVal << std::endl;
                    if (((scriptY.squaredNorm() > minVal) && !controlHistoryStack) || ((scriptY.squaredNorm() > minVal) && startHistoryStack && controlHistoryStack) || ((minVal <= 0.001) && fillHistoryStackBegin && !controlHistoryStack))
                    {
                        YstackMocap.col(index) = scriptY;
                        UstackMocap.col(index) = scriptU;
                    }
                    minVal = YstackMocap.colwise().squaredNorm().minCoeff(&index);
                    msg.minVal = minVal;
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
                    msg.minValOld = minVal;
                    msg.newVal = scriptY.squaredNorm();
                    if (((scriptY.squaredNorm() > minVal) && !controlHistoryStack) || ((scriptY.squaredNorm() > minVal) && startHistoryStack && controlHistoryStack) || ((minVal <= 0.001) && fillHistoryStackBegin && !controlHistoryStack))
                    {
                        YstackHomog.col(index) = scriptY;
                        UstackHomog.col(index) = scriptU;
                    }
                    minVal = YstackHomog.colwise().squaredNorm().minCoeff(&index);
                    msg.minVal = minVal;
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
        if (firstDes)
        {
            startTimeDes = ros::Time::now();
        }
        // time
        ros::Time timestamp = ros::Time::now();
        //double delT = timestamp.toSec() - desLastTime;
        double delT = 1.0/desUpdateRate;
        desLastTime = timestamp.toSec();
        //std::cout << "des time diff " << delT << std::endl;
        
        // update velocities
        Eigen::Vector3d vcd = qPanTilt*Eigen::Vector3d(2*M_PI*desRadius/desPeriod, 0, 0);
        wcd = qPanTilt*Eigen::Vector3d(0,-2*M_PI/desPeriod,0);
        //Eigen::Vector3d vcd = qPanTilt*Eigen::Vector3d(2*M_PIl*0.1*0.5*std::cos(2*M_PIl*0.1*(timestamp-startTimeDes).toSec()),0,0);
        //Eigen::Vector3d vcd;
        //vcd << 0,0,0;
        //wcd << 0,0,0;
        
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
        
        // compare to last solution
        if ((qdLast.coeffs() - -1*qd.coeffs()).squaredNorm() < (qdLast.coeffs() - qd.coeffs()).squaredNorm()) 
        { 
            qd = Eigen::Quaterniond(-1*qd.coeffs()); 
        }
        
        Eigen::Vector3d m1d(tfMarker2Des.getOrigin().getX()/tfMarker2Des.getOrigin().getZ(), tfMarker2Des.getOrigin().getY()/tfMarker2Des.getOrigin().getZ(), 1);
        Eigen::Vector3d p1d = camMat*m1d;
        double alpha1d = tfMarker2Ref.getOrigin().getZ()/tfMarker2Des.getOrigin().getZ();
        //std::cout << "alphad\n" << alpha1d <<std::endl;
        ped << p1d(0), p1d(1), -1*std::log(alpha1d);
        homography_vsc_cl::Debug msg;
        if (!firstDes)
        {
            // Lvd
            Eigen::Matrix3d camMatFactor = camMat;
            camMatFactor.block<2,1>(0,2) << 0, 0;
            Eigen::Matrix3d temp1 = Eigen::Matrix3d::Identity();
            temp1.topRightCorner<2,1>() = -1*m1d.head<2>();
            Eigen::Matrix3d Lvd = camMatFactor*temp1;
            pedDot = -1*alpha1d/tfMarker2Ref.getOrigin().getZ()*Lvd*vcd + Lvd*m1d.cross(wcd);
            //pedDot = (ped - pedLast)/delT;
            msg.delPed.x = (ped - pedLast).x(); msg.delPed.y = (ped - pedLast).y(); msg.delPed.z = (ped - pedLast).z(); 
            if (!secondDes)
            {
                pedDot = (1-filterAlpha)*pedDot + filterAlpha*pedDotLast;
            }
            pedDotLast = pedDot;
            secondDes = false;
        }
        firstDes = false;
        pedLast = ped;
        qdLast = qd;
        
        //debug
        
        msg.header.stamp = timestamp;
        msg.q.x = qd.x(); msg.q.y = qd.y(); msg.q.z = qd.z(); msg.q.w = qd.w();
        msg.alpha = alpha1d;
        msg.newPixels.pr.x = p1d.x(); msg.newPixels.pr.y = p1d.y();
        msg.m1.x = m1d.x(); msg.m1.y = m1d.y(); msg.m1.z = m1d.z(); 
        msg.vc.x = vcd.x(); msg.vc.y = vcd.y(); msg.vc.z = vcd.z();
        msg.wc.x = wcd.x(); msg.wc.y = wcd.y(); msg.wc.z = wcd.z();
        msg.pedLast.x = pedLast.x(); msg.pedLast.y = pedLast.y(); msg.pedLast.z = pedLast.z();
        msg.delT = delT;
        
        debugDesPub.publish(msg);

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
                tf::Vector3 nStarVec = tf::quatRotate(tfRef.getRotation().inverse(),tf::Vector3(0,0,-1));
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
        if (msg.buttons[6])//back button starts the stack
        {
            startHistoryStack = true;
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
        
        // update buffers
        velActualTimeBuffer.push_back(ros::Time::now());
        vcActualBuffer.push_back(vcActual);
        wcActualBuffer.push_back(wcActual);
        if (velActualTimeBuffer.size() > 1)
        {
            // get time diff
            std::deque<ros::Time>::iterator timeIt = velActualTimeBuffer.end();
            double velActualTimeBufferDiffNew = (*(timeIt-1) - *(timeIt-2)).toSec();
            velActualTimeDiffBuffer.push_back(velActualTimeBufferDiffNew);
            //std::cout << "new time diff: " << velActualTimeBufferDiffNew << std::endl;
            double velActualTimeBufferDiffTotal = (*(timeIt-1)-*(velActualTimeBuffer.begin())).toSec();
            
            //std::cout << "total time diff: " << velActualTimeBufferDiffTotal << std::endl;
            
            // check if buffer time window has exceeded the desired time window
            if (velActualTimeBufferDiffTotal > predictorTimeWindow)
            {
                // if the buffer time window is larger than the time window pop off the oldest values until it is under the time window
                while (((*(timeIt-1)-*velActualTimeBuffer.begin()).toSec()) > predictorTimeWindow)
                {
                    // remove old data
                    velActualTimeBuffer.pop_front();
                    vcActualBuffer.pop_front();
                    wcActualBuffer.pop_front();
                    velActualTimeDiffBuffer.pop_front();
                }
            }
        }
        else
        {
            velActualTimeDiffBuffer.push_back(0.0);
        }
    }
    
    void bebopVelCB(const geometry_msgs::TwistStampedConstPtr& twist)
    {
        vBebopActual << twist->twist.linear.x,twist->twist.linear.y,twist->twist.linear.z;
        wBebopActual << twist->twist.angular.x,twist->twist.angular.y,twist->twist.angular.z;
    }
    
    void predictorUpdate(Eigen::Vector3d pixel0, Eigen::Quaterniond q0, double alpha0, Eigen::Vector3d& pixelPredict, Eigen::Quaterniond& qPredict, double& alphaPredict)
    {
        Eigen::Vector3d peHat(pixel0.x(),pixel0.y(),std::log(alpha0));
        Eigen::Quaterniond qHat(q0.w(),q0.x(),q0.y(),q0.z());
        //std::cout << "time buffer size: " << velActualTimeBuffer.size() << std::endl;
        int velBufferSize = useActualVelocities ? velActualTimeBuffer.size() : velTimeBuffer.size();
        for (int ii = 0; ii < velBufferSize; ii++)
        {
            double uHat = peHat.x();
            double vHat = peHat.y();
            double alphaHat = std::exp(peHat.z());
            Eigen::Vector3d pHat(uHat,vHat,1.0);
            //std::cout << "pHat: x: " << pHat.x() << " y: " << pHat.y() << " z: " << pHat.z() << std::endl;
            Eigen::Vector3d mHat = camMat.inverse()*pHat;
            //std::cout << "mHat: x: " << mHat.x() << " y: " << mHat.y() << " z: " << mHat.z() << std::endl;
            Eigen::Matrix3d camMatFactor = camMat;
            camMatFactor.block<2,1>(0,2) << 0, 0;
            Eigen::Matrix3d tempHat = Eigen::Matrix3d::Identity();
            tempHat.topRightCorner<2,1>() = -1*mHat.head<2>();
            Eigen::Matrix3d LvHat = camMatFactor*tempHat;
            Eigen::Vector3d vcHat = useActualVelocities ? vcActualBuffer.at(ii) : vcBuffer.at(ii);
            //std::cout << "vcHat: x: " << vcHat.x() << " y: " << vcHat.y() << " z: " << vcHat.z() << std::endl;
            Eigen::Vector3d wcHat = useActualVelocities ? wcActualBuffer.at(ii) : wcBuffer.at(ii);
            //std::cout << "wcHat: x: " << wcHat.x() << " y: " << wcHat.y() << " z: " << wcHat.z() << std::endl;
            double timeHat = useActualVelocities ? velActualTimeDiffBuffer.at(ii) : velTimeDiffBuffer.at(ii);
            //std::cout << "timeHat: " << timeHat << std::endl;
            Eigen::Vector3d peHatDot;
            if (useZstar)
            {
                peHatDot = -1*(alphaHat/zStar)*(LvHat*vcHat) + LvHat*(mHat.cross(wcHat));
            }
            else
            {
                peHatDot = -1*(alphaHat/zhatHomog)*(LvHat*vcHat) + LvHat*(mHat.cross(wcHat));
            }
            //std::cout << "peHatDot: x: " << peHatDot.x() << " y: " << peHatDot.y() << " z: " << peHatDot.z() << std::endl;
            peHat += peHatDot*timeHat;
            
            Eigen::Vector4d qHatTemp(qHat.w(),qHat.x(),qHat.y(),qHat.z());
            qHatTemp += 0.5*diffMat(qHat)*wcHat*timeHat;
            qHat = Eigen::Quaterniond(qHatTemp(0),qHatTemp(1),qHatTemp(2),qHatTemp(3));
            qHat.normalize();
        }
        
        //std::cout << "pixel0: x: " << pixel0.x() << " y: " << pixel0.y() << " z: " << pixel0.z() << std::endl;
        pixelPredict = Eigen::Vector3d(peHat.x(), peHat.y(), 1.0);
        //std::cout << "pixelPredict: x: " << pixelPredict.x() << " y: " << pixelPredict.y() << " z: " << pixelPredict.z() << std::endl;
        qPredict = Eigen::Quaterniond(qHat.w(), qHat.x(), qHat.y(), qHat.z());
        alphaPredict = std::exp(peHat.z());
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
