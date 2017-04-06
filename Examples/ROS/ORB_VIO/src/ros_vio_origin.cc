/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

//#include<ros/ros.h>
//#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include"include/System.h"

//#include "MsgSync/MsgSynchronizer.h"

#include "src/IMU/imudata.h"
#include "src/IMU/configparam.h"
//#include <rosbag/bag.h>
//#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <fstream>
#include <time.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <memory>
#include <functional>
#include <atomic>
using namespace std;

/*class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

   // void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};
*/

typedef struct ImageList
{
	unsigned long timeStamp;
	string imgName;
}ICell;

void loadImageList(char * imagePath,std::vector<ICell> &iListData)
{
    ifstream inf;
    inf.open(imagePath, ifstream::in);
    const int cnt = 2;          // 你要输出的个数

    string line;
    //int i = 0;
    int j = 0;
    size_t comma = 0;
    size_t comma2 = 0;
    ICell temp;
    getline(inf,line);
    while (!inf.eof())
    {
        getline(inf,line);

        comma = line.find(',',0);
        //string temp1 = line.substr(0,comma).substr(0,10);
        temp.timeStamp = (unsigned long)atoi(line.substr(0,comma).c_str());
        
        //cout<<line.substr(0,comma).c_str()<<' ';
        //memcpy(imuTimeStamp,line.substr(0,comma).c_str(),line.substr(0,comma).length);
        while (comma < line.size() && j != cnt-1)
        {
	   
            comma2 = line.find(',',comma + 1);
            //i = atof(line.substr(comma + 1,comma2-comma-1).c_str());
            temp.imgName = line.substr(comma + 1,comma2-comma-1).c_str();
            ++j;
            comma = comma2;
        }
        iListData.push_back(temp);
        cout<<endl;
        j = 0;
    }

    inf.close();
  //  return 0;
}


void loadIMUFile(char * imuPath,std::vector<ORB_SLAM2::IMUData> &vimuData)
{
    ifstream inf;
    inf.open(imuPath, ifstream::in);
    const int cnt = 7;          // 你要输出的个数

    string line;
    //int i = 0;
    int j = 0;
    size_t comma = 0;
    size_t comma2 = 0;

    char imuTime[14] = {0};
    double acc[3] = {0.0};
    double grad[3] = {0.0};
    unsigned long imuTimeStamp = 0;

    getline(inf,line);
    while (!inf.eof())
    {
        getline(inf,line);

        comma = line.find(',',0);
	string temp = line.substr(0,comma);
        imuTimeStamp = (unsigned long)atoi(line.substr(0,comma).c_str());
        
        //cout<<line.substr(0,comma).c_str()<<' ';
        //memcpy(imuTimeStamp,line.substr(0,comma).c_str(),line.substr(0,comma).length);
        while (comma < line.size() && j != cnt-1)
        {
	   
            comma2 = line.find(',',comma + 1);
            switch(j)
	    {
              case 0:
		grad[0] = atof(line.substr(comma + 1,comma2-comma-1).c_str());
		break;
              case 1:
		grad[1] = atof(line.substr(comma + 1,comma2-comma-1).c_str());
		break;
              case 2:
		grad[2] = atof(line.substr(comma + 1,comma2-comma-1).c_str());
		break;
              case 3:
		acc[0] = atof(line.substr(comma + 1,comma2-comma-1).c_str());
		break;
              case 4:
		acc[1] = atof(line.substr(comma + 1,comma2-comma-1).c_str());
		break;
              case 5:
		acc[2] = atof(line.substr(comma + 1,comma2-comma-1).c_str());
		break;
            }
            //cout<<line.substr(comma + 1,comma2-comma-1).c_str()<<' ';
            ++j;
            comma = comma2;
        }
	ORB_SLAM2::IMUData tempImu(grad[0],grad[1],grad[2],acc[0],acc[1],acc[2],imuTimeStamp);
        vimuData.push_back(tempImu);
        cout<<endl;
        j = 0;
    }

inf.close();

//return 0;
}

int main(int argc, char **argv)
{
    //ros::init(argc, argv, "Mono");
    //ros::start();

    if(argc != 6)
    {
       // cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
  //后面三个参数分别为：imu的数据（包括角速度，加速度，共6维）；cam0时间戳和文件名的联系文件(包括timestamp [ns],filename。共二维)；cma0的图片        
   cerr << endl << "Usage: ./project path_to_ORBVOC.TXT path_to_euroc.yaml path_to_imu/data.csv path_to_cam0/data.csv path_to_cam0/data" << endl;
       // ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ORB_SLAM2::ConfigParam config(argv[2]);

    /**
     * @brief added data sync
     */
    double imageMsgDelaySec = config.GetImageDelayToIMU();

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();
    //cout<<"-----------------------------------------------------------------------------"<<endl;
    char *fullPath = new char[500];// = {0};
    memset(fullPath,0,500);
    //imgData>>imageTimeStamp>>imageName;
    //imuDataFile>>imuTimeStamp>>grad[0]>>grad[1]>>grad[2]>>acc[0]>>acc[1]>>acc[2];
    std::vector<ORB_SLAM2::IMUData> allimuData;
    std::vector<ICell> iListData;
    //loadIMUFile("/home/fyj/Code/C++/LearnVIORB/Examples/ROS/ORB_VIO/v2_03_diff/V2_03_difficult/mav0/imu0/data.csv",allimuData);
    loadIMUFile(argv[3],allimuData);
    //cout<<"loading imu finished"<<endl;
    //loadImageList("/home/fyj/Code/C++/LearnVIORB/Examples/ROS/ORB_VIO/v2_03_diff/V2_03_difficult/mav0/cam0/data.csv",iListData);
    loadImageList(argv[4],iListData);
    //cout<<"loading image finished"<<endl;
    double e = pow(10.0,-9);
        
    //cout<<iListData.size()<<"------------"<<allimuData.size()<<endl;
    //cv::waitKey(0);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(iListData.size());

    int nImages=iListData.size();
    if(nImages<1)
    {
         cerr << endl <<"There is no enough images."<<endl;
    }




    for(int j=0;j<iListData.size();j++)
    {
            std::vector<ORB_SLAM2::IMUData> vimuData;
	    /*
	    *imu 的频率是200HZ 图像帧率是20HZ 所以简单的认为每一帧图像对应10个imu数据
	    */
            for(unsigned int i=0;i<10;i++)
            {
//                cout<<"*************************************************************************"<<endl;
                char temp[10] = {0};
		
                //substring(temp,imuTime,0,10);
  //              cout<<"=========================================================================="<<endl;
                if(bAccMultiply98)
                {
                    allimuData[j]._a(0) *= g3dm;
                    allimuData[j]._a(1) *= g3dm;
                    allimuData[j]._a(2) *= g3dm;
                }
                //cout<<allimuData[j]._g(0)<<","<<allimuData[j]._g(1)<<","<<allimuData[j]._g(2)<<endl;
		//cout<<allimuData[j]._a(0)<<","<<allimuData[j]._a(1)<<","<<allimuData[j]._a(2)<<endl;
		//printf("imutimestamp,%0.10f\n",(double)allimuData[j]._t);
                allimuData[j]._t = allimuData[j]._t*e;
		/*
		*这里将时间戳×上e-9后的结果，程序可以正常运行，但是显示出来的时间和ros环境下的时间不同，且运行速度缓慢。
		ORB_SLAM2::IMUData imudata(allimuData[j]._g(0),allimuData[j]._g(1),allimuData[j]._g(2),
                                allimuData[j]._a(0),allimuData[j]._a(1),allimuData[j]._a(2),(double)allimuData[j]._t);
		*/
		//时间戳按这个给程序也可以正常运行，速度基本和ros环境下一样，问题在于当按照正常的0.005设置时候程序会挂，后来尝试后发现这个数据给的越小程序越容易运行成功。
		ORB_SLAM2::IMUData imudata(allimuData[j]._g(0),allimuData[j]._g(1),allimuData[j]._g(2),
                                allimuData[j]._a(0),allimuData[j]._a(1),allimuData[j]._a(2),j*0.0005+i*0.00005);
                vimuData.push_back(imudata);
            }
	   
            //cout<<"IMU FINISHED READING"<<endl;
	    //发现读取txt时，图像文件名后多了一个‘/r’，因此需要截掉这个字符。
	    string temp = iListData[j].imgName.substr(0,iListData[j].imgName.size()-1);
	    //sprintf(fullPath,"%s/%s","/home/fyj/Code/C++/LearnVIORB/Examples/ROS/ORB_VIO/v2_03_diff/V2_03_difficult/mav0/cam0/data",temp.c_str());
	    sprintf(fullPath,"%s/%s",argv[5],temp.c_str());
	    cv::Mat im = cv::imread(fullPath,0);
	    cout<<fullPath<<endl;
	    memset(fullPath,0,100);
	  //  cout<<"-----------------------FYJ----------------------"<<iListData[j].timeStamp<<endl;
	    iListData[j].timeStamp = iListData[j].timeStamp*e;
	    //printf("imagetimestamp,%0.10f\n",(double)iListData[j].timeStamp);
	    //SLAM.TrackMonoVI(im, vimuData, (double)iListData[j].timeStamp);
	    SLAM.TrackMonoVI(im, vimuData, j*0.00005);
		//if(j == 6)
		//{
		//	usleep(20);			
			//cv::waitKey(0);
		//}
            // Wait local mapping end.
            bool bstop = false;
	//cout<<"----------------------------------"<<j<<"----------------------------------------"<<endl;
            while(!SLAM.bLocalMapAcceptKF())
            {
               bstop=true;
            };
            //if(bstop)
              //  break;
     }
    delete [] fullPath;
    SLAM.SaveKeyFrameTrajectoryNavState(config._tmpFilePath+"KeyFrameNavStateTrajectory.txt");
    cout<<endl<<endl<<"press any key to shutdown"<<endl;
    getchar();

    // Stop all threads
    SLAM.Shutdown();

    //ros::shutdown();

    return 0;
}


