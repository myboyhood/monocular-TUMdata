// -------------- test the visual odometry -------------
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp> 
#include "opencv2/opencv.hpp"
#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include "myslam/frame.h"
#define _DEBUG
#define name2str(name) (#name)




int main ( int argc, char** argv )
{
//    if ( argc != 2 )
//    {
//        cout<<"usage: run_vo parameter_file"<<endl;
//        return 1;
//    }
//
    myslam::Config::setParameterFile ( argv[1] );

//
    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
//    ifstream fin_1 ( dataset_dir+"/associate.txt" );
//    if ( !fin_1 )
//    {
//        cout<<"please generate the associate file called associate.txt!"<<endl;
//        return 1;
//    }
//
//    vector<string> rgb_files, depth_files;
//    vector<double> rgb_times, depth_times;
//    while ( !fin_1.eof() )
//    {
//        string rgb_time, rgb_file, depth_time, depth_file;
//        fin_1>>rgb_time>>rgb_file>>depth_time>>depth_file;
//        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
//        depth_times.push_back ( atof ( depth_time.c_str() ) );
//        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
//        depth_files.push_back ( dataset_dir+"/"+depth_file );
//
//        if ( fin_1.good() == false )
//            break;
//    }
    
    vector<cv::Mat> rgb_images ;
    string s1;

    s1 = "/home/wzy/slambook/0.2/rgb";
   Mat rgb_image;

   // cout<<"rgb_dataset"<<ss<<endl;
    for (int j = 1; j < 8 ; ++j)
    {

        string s2;
        stringstream ss2;
        ss2 << j;
        ss2 >> s2;
        rgb_image = imread(s1 + "/" + s2 + ".png");
        rgb_images.push_back(rgb_image);
        cout<<"name"<<" "<<s2<<endl;
//        imshow(s2,rgb_image);

    }

    //myslam::Camera::Ptr camera ( new myslam::Camera );



    // visualization
    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    cv::Point3d cam_pos( 0, -1.0, -1.0 ), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose( cam_pose );
    
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget( "World", world_coor );
    vis.showWidget( "Camera", camera_coor );

    cout<<"read total "<<rgb_images.size() <<" entries"<<endl;


    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );

    for ( int i=0; i<rgb_images.size(); i++ )
    {

        Mat color =  rgb_images[i];

        //我自己加的检测2幅图像不同
        double avg = 0.0;
        double stddev = 0.0;
        Mat src;
        cvtColor(rgb_images[i],src,CV_BGR2GRAY);
        Mat mean;//平均灰度
        Mat stdDev;//灰度方差
        meanStdDev(src,mean,stdDev);
        avg = mean.ptr<double>(0)[0];
        stdDev = stdDev.ptr<double>(0)[0];
        cout<<"avg = "<< avg<<"stdDev = "<<stdDev<<endl;


        if ( color.data==nullptr )
            break;
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        //pFrame->camera_ = camera;
        pFrame->color_ = color;

        cout<<"receive a frame"<<endl;
        boost::timer timer;
        vo->addFrame ( pFrame );
        cout<<"VO costs time: "<<timer.elapsed()<<endl;

        if ( vo->state_ == myslam::VisualOdometry::LOST )
            break;
//        //SE3 Tcw = pFrame->T_c_w_.inverse();//Tcw就是变换矩阵T，可以由T得出旋转矩阵R和位移t。
//


//        // show the map and the camera pose
//cv::Affine3d M(
//        cv::Affine3d::Mat3 (
//        pFrame->R.at<float>(0,0),pFrame->R.at<float>(0,1),pFrame->R.at<float>(0,2),
//        pFrame->R.at<float>(1,0),pFrame->R.at<float>(1,1),pFrame->R.at<float>(1,2),
//        pFrame->R.at<float>(2,0),pFrame->R.at<float>(2,1),pFrame->R.at<float>(2,2)
//        ),
//        cv::Affine3d::Vec3(
//        pFrame->t(0),pFrame->t(1),pFrame->t(2)
//        )
//        );


        //convert from Mat to Matx
//        float* dataR = reinterpret_cast<float*>(pFrame->R.data);
//        Matx33f converted(dataR);


//cv::Affine3f T(pFrame->R,pFrame->t);

cv::Affine3f T(
        cv::Affine3f::Mat3(pFrame->R),cv::Affine3f::Vec3(pFrame->t)

        );
//
        cv::imshow("image", color );
        cv::waitKey(1);
        vis.setWidgetPose( "Camera",T);
        vis.spinOnce(1, false);
      }

    return 0;
}
