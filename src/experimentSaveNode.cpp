#include <iostream>
#include <cstdio>
#include <string>
#include <fstream>

#include <ros/ros.h>
#include <ros/console.h>
#include <homography_vsc_cl/Output.h>
#include <homography_vsc_cl/Debug.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

class ExperimentSaver
{
    ros::NodeHandle nh;
    
    // file
    bool saveExperiment;
    std::fstream saveExperimentFile;
    std::string saveExperimentFilename;
    
    //homog
    homography_vsc_cl::Debug homogDebugMsg;
    ros::Subscriber homogDebugSub;
    
    //mocap
    homography_vsc_cl::Debug mocapDebugMsg;
    ros::Subscriber mocapDebugSub;
    
    //output
    homography_vsc_cl::Output outputMsg;
    ros::Subscriber outputSub;
    
    //time
    ros::Time startTime;

public:
    ExperimentSaver()
    {
        // Parameters
        ros::NodeHandle nhp("~"); // "private" nodehandle, used to access private parameters
        nhp.param<bool>("saveExperiment",saveExperiment,false);
        nhp.param<std::string>("saveExperimentFilename",saveExperimentFilename,"/home/ncr/ncr_ws/src/homography_vsc_cl/experiments/experiment.txt");
        
        //file header
        if (saveExperiment)
        {
            std::remove( saveExperimentFilename.c_str() );
            saveExperimentFile.open(saveExperimentFilename, std::fstream::out | std::fstream::app);
        
            if (saveExperimentFile.is_open())
            {
                saveExperimentFile << "time,"
                                << "evXh," << "evYh," << "evZh,"
                                << "evXm," << "evYm," << "evZm,"
                                << "qTildeWh," << "qTildeXh," << "qTildeYh," << "qTildeZh,"
                                << "qTildeWm," << "qTildeXm," << "qTildeYm," << "qTildeZm,"
                                << "zStar,"
                                << "zStarHath,"
                                << "zStarHatm,"
                                << "zTildeh,"
                                << "zTildem,"
                                << "desImageWrtImageZhatPX," << "desImageWrtImageZhatPY," << "desImageWrtImageZhatPZ,"
                                << "desImageWrtImagePX," << "desImageWrtImagePY," << "desImageWrtImagePZ,"
                                << "desImageWrtImageZhatQW," << "desImageWrtImageZhatQX," << "desImageWrtImageZhatQY," << "desImageWrtImageZhatQZ,"
                                << "desImageWrtImageQW," << "desImageWrtImageQX," << "desImageWrtImageQY," << "desImageWrtImageQZ,"
                                << "vcXh,"<< "vcYh,"<< "vcZh,"
                                << "vcXm,"<< "vcYm,"<< "vcZm,"
                                << "wcXh,"<< "wcYh,"<< "wcZh,"
                                << "wcXm,"<< "wcYm,"<< "wcZm,"
                                << "\n";
                saveExperimentFile.close();
            }
        }
        
        outputSub = nh.subscribe("controller_output",1,&ExperimentSaver::outputCB,this);
        homogDebugSub = nh.subscribe("controller_debug_homog",1,&ExperimentSaver::homogDebugCB,this);
        mocapDebugSub = nh.subscribe("controller_debug_mocap",1,&ExperimentSaver::mocapDebugCB,this);
        
        startTime = ros::Time::now();
    }
    
    void outputCB(const homography_vsc_cl::Output& msg)
    {
        outputMsg = msg;
    }
    
    void homogDebugCB(const homography_vsc_cl::Debug& msg)
    {
        homogDebugMsg = msg;
        if (saveExperiment)
        {
            saveExperimentFile.open(saveExperimentFilename, std::fstream::out | std::fstream::app);
        
            if (saveExperimentFile.is_open())
            {
                saveExperimentFile << (ros::Time::now()-startTime).toSec() << ","
                                << homogDebugMsg.ev.x << "," << homogDebugMsg.ev.y << "," << homogDebugMsg.ev.z << ","
                                << mocapDebugMsg.ev.x << "," << mocapDebugMsg.ev.y << "," << mocapDebugMsg.ev.z << ","
                                << homogDebugMsg.qTilde.w << "," << homogDebugMsg.qTilde.x << "," << homogDebugMsg.qTilde.y << "," << homogDebugMsg.qTilde.z << ","
                                << mocapDebugMsg.qTilde.w << "," << mocapDebugMsg.qTilde.x << "," << mocapDebugMsg.qTilde.y << "," << mocapDebugMsg.qTilde.z << ","
                                << outputMsg.zStar << ","
                                << outputMsg.zHatHomog << ","
                                << outputMsg.zHatMocap << ","
                                << outputMsg.zTildeHomog << ","
                                << outputMsg.zTildeMocap << ","
                                << homogDebugMsg.desImageWrtImageZhat.translation.x << "," << homogDebugMsg.desImageWrtImageZhat.translation.y << "," << homogDebugMsg.desImageWrtImageZhat.translation.z << ","
                                << mocapDebugMsg.desImageWrtImage.translation.x << "," << mocapDebugMsg.desImageWrtImage.translation.y << "," << mocapDebugMsg.desImageWrtImage.translation.z << ","
                                << homogDebugMsg.desImageWrtImageZhat.rotation.w << "," << homogDebugMsg.desImageWrtImageZhat.rotation.x << "," << homogDebugMsg.desImageWrtImageZhat.rotation.y << "," << homogDebugMsg.desImageWrtImageZhat.rotation.z << ","
                                << mocapDebugMsg.desImageWrtImage.rotation.w << "," << mocapDebugMsg.desImageWrtImage.rotation.x << "," << mocapDebugMsg.desImageWrtImage.rotation.y << "," << mocapDebugMsg.desImageWrtImage.rotation.z << ","
                                << homogDebugMsg.vc.x << "," << homogDebugMsg.vc.y << "," << homogDebugMsg.vc.z << ","
                                << mocapDebugMsg.vc.x << "," << mocapDebugMsg.vc.y << "," << mocapDebugMsg.vc.z << ","
                                << homogDebugMsg.wc.x << "," << homogDebugMsg.wc.y << "," << homogDebugMsg.wc.z << ","
                                << mocapDebugMsg.wc.x << "," << mocapDebugMsg.wc.y << "," << mocapDebugMsg.wc.z << ","
                                << "\n";
                saveExperimentFile.close();
            }
        }
    }
    
    void mocapDebugCB(const homography_vsc_cl::Debug& msg)
    {
        mocapDebugMsg = msg;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "experimentSaveNode");
    
    ExperimentSaver obj;
    
    ros::spin();
    return 0;
}
