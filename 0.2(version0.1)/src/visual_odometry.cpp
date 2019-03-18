/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"

namespace myslam
{

VisualOdometry::VisualOdometry() :
    state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), num_lost_ ( 0 )//对类函数初始化
{
//    num_of_features_    = Config::get<int> ( "number_of_features" );
//    scale_factor_       = Config::get<double> ( "scale_factor" );
//    level_pyramid_      = Config::get<int> ( "level_pyramid" );
//    match_ratio_        = Config::get<float> ( "match_ratio" );
//    max_num_lost_       = Config::get<float> ( "max_num_lost" );
//    min_match_size      = Config::get<int> ( "min_match_size" );
//    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
//    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );

    num_of_features_ = 500;
    scale_factor_ = 1.2;
    level_pyramid_ = 8;
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
}

VisualOdometry::~VisualOdometry()
{

}

bool VisualOdometry::addFrame ( Frame::Ptr frame )
{
    cout<<"state is "<<state_<<endl;
    switch ( state_ )
    {

//    case INITIALIZING:
        case -1:
    {

        state_ = OK;

        cout<<"state_now is "<<state_<<endl;
        curr_ = ref_ = frame;
        ref_->T = (Mat_<float> (3,4) <<
                1,0,0,0,
                0,1,0,0,
                0,0,1,0);
//        map_->insertKeyFrame ( frame );
        // extract features from first frame 
        extractKeyPoints();
        computeDescriptors();

        // compute the 3d position of features in ref frame 
        //setRef3DPoints();
        break;
    }
//    case OK:
        case 0:
    {
        keypoints_ref_ = keypoints_curr_;
        descriptors_ref_ = descriptors_curr_;
        curr_ = frame;
        extractKeyPoints();
        computeDescriptors();
        featureMatching();
        pose_estimation_2d2d();
        if ( checkEstimatedPose() == true ) // a good estimation
        {
            //curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;  // T_c_w = T_c_r*T_r_w
            ref_ = curr_;
            //setRef3DPoints();
            //keypoints_ref_ = keypoints_curr_;
            num_lost_ = 0;
//            if ( checkKeyFrame() == true ) // is a key-frame
//            {
//                addKeyFrame();
//            }
        }
        else // bad estimation due to various reasons
        {

            max_num_lost_ = 2;

            num_lost_++;
            if ( num_lost_ > max_num_lost_ )
            {
                state_ = LOST;
            }
            return false;
        }
        break;
    }
    case LOST:
    {
        cout<<"vo has lost."<<endl;
        break;
    }
    }

    return true;
}

void VisualOdometry::extractKeyPoints()
{
    orb_->detect ( curr_->color_, keypoints_curr_ );
    cout<<"keypoints_curr_size = "<<keypoints_curr_.size()<<endl;
}

void VisualOdometry::computeDescriptors()
{
    orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
}

void VisualOdometry::featureMatching()
{
  // match desp_ref and desp_curr, use OpenCV's brute force match
      vector<cv::DMatch> matches;
      cv::BFMatcher matcher ( cv::NORM_HAMMING );
      cout<<"desp_ref"<<descriptors_ref_.size()<<"desp_curr"<<descriptors_curr_.size()<<endl;
      matcher.match ( descriptors_ref_, descriptors_curr_, matches );
    // select the best matches
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;
    //cout<<"min_dis = "<<min_dis<<endl;
    cout <<"feature_matches_" << matches.size()<<endl;
   feature_matches_.clear();
//当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    match_ratio_=2.0;
    for ( cv::DMatch& m : matches )
    {
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            //cout<<"m.distance"<<m.distance<<endl;
            feature_matches_.push_back(m);
        }
    }
    cout<<"good matches: "<<feature_matches_.size()<<endl;
}

    bool VisualOdometry::checkEstimatedPose()
    {

        min_match_size = 10;
        // check if the estimated pose is good
        if ( feature_matches_.size() < min_match_size )
        {
            cout<<"reject because inlier is too small: "<<min_match_size<<endl;
            return false;
        }
        // if the motion is too large, it is probably wrong
//        Sophus::Vector6d d = T_c_r_estimated_.log();
//        if ( d.norm() > 5.0 )
//        {
//            cout<<"reject because motion is too large: "<<d.norm()<<endl;
//            return false;
//        }
//        return true;
    }




void VisualOdometry::pose_estimation_2d2d ()
    {




        //-- 把匹配点转换为vector<Point2f>的形式


        for ( cv::DMatch m:feature_matches_ )
        {
            points1.push_back ( keypoints_ref_[m.queryIdx].pt );
            points2.push_back ( keypoints_curr_[m.trainIdx].pt );
        }

        //-- 计算基础矩阵
        Mat fundamental_matrix;
        fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
        cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

        //-- 计算本质矩阵
        Point2d principal_point ( 325.1, 249.7 );	//相机光心, TUM dataset标定值
        double focal_length = 517;			//相机焦距, TUM dataset标定值
        Mat essential_matrix;
        essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
        cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

        //-- 计算单应矩阵
        Mat homography_matrix;
        homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
        cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

        //-- 从本质矩阵中恢复旋转和平移信息.
        recoverPose ( essential_matrix, points1, points2, curr_->R, curr_->t, focal_length, principal_point );
        cout<<"RR is "<<endl<<curr_->R<<endl;
        cout<<"tt is "<<endl<<curr_->t<<endl;






    }

void VisualOdometry::triangulation ()

    {

        vector< Point3d > points;

        Mat T1 = (Mat_<float> (3,4) <<
                ref_->R.at<double>(0,0), ref_->R.at<double>(0,1), ref_->R.at<double>(0,2), ref_->t(0),
                ref_->R.at<double>(1,0), ref_->R.at<double>(1,1), ref_->R.at<double>(1,2), ref_->t(1),
                ref_->R.at<double>(2,0), ref_->R.at<double>(2,1), ref_->R.at<double>(2,2), ref_->t(2)
        );

        Mat T2 = (Mat_<float> (3,4) <<
                curr_->R.at<double>(0,0), curr_->R.at<double>(0,1), curr_->R.at<double>(0,2), curr_->t(0),
                curr_->R.at<double>(1,0), curr_->R.at<double>(1,1), curr_->R.at<double>(1,2), curr_->t(1),
                curr_->R.at<double>(2,0), curr_->R.at<double>(2,1), curr_->R.at<double>(2,2), curr_->t(2)
        );


//        Mat T1 = (Mat_<float> (3,4) <<
//                ref_->R(0,0), ref_->R(0,1), ref_->R(0,2), ref_->t(0),
//                ref_->R(1,0), ref_->R(1,1), ref_->R(1,2), ref_->t(1),
//                ref_->R(2,0), ref_->R(2,1), ref_->R(2,2), ref_->t(2)
//        );
//
//        Mat T2 = (Mat_<float> (3,4) <<
//                curr_->R(0,0), curr_->R(0,1), curr_->R(0,2), curr_->t(0),
//                curr_->R(1,0), curr_->R(1,1), curr_->R(1,2), curr_->t(1),
//                curr_->R(2,0), curr_->R(2,1), curr_->R(2,2), curr_->t(2)
//        );



//        for ( DMatch m: feature_matches_ )
//        {
//            // 将像素坐标转换至相机坐标
////            pts_1.push_back ( pixel2cam( keypoint_1[m.queryIdx].pt, K) );
////            pts_2.push_back ( pixel2cam( keypoint_2[m.trainIdx].pt, K) );
//        }

        Mat pts_4d;
        cv::triangulatePoints( T1, T2, points1, points2, pts_4d );

        // 转换成非齐次坐标
        for ( int i=0; i<pts_4d.cols; i++ )
        {
            Mat x = pts_4d.col(i);
            x /= x.at<float>(3,0); // 归一化
            Point3d p (
                    x.at<float>(0,0),
                    x.at<float>(1,0),
                    x.at<float>(2,0)
            );
            points.push_back( p );
        }
    }




//bool VisualOdometry::checkKeyFrame()
//{
//    Sophus::Vector6d d = T_c_r_estimated_.log();
//    Vector3d trans = d.head<3>();
//    Vector3d rot = d.tail<3>();
//    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
//        return true;
//    return false;
//}
//
//void VisualOdometry::addKeyFrame()
//{
//    cout<<"adding a key-frame"<<endl;
//    map_->insertKeyFrame ( curr_ );
//}

}