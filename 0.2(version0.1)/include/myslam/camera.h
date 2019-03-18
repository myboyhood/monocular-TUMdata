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

#ifndef CAMERA_H
#define CAMERA_H

#include "myslam/common_include.h"

namespace myslam
{

    class  Camera
    {
    public:
        typedef std::shared_ptr<Camera> Ptr;
        float fx_, fy_, cx_, cy_;  // Camera intrinsics

        Camera();

        Camera(float fx, float fy, float cx, float cy) :
                fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}
        ~Camera();

    };





//// Pinhole momocular camera model
//Point2d pixel2cam ( const Point2d& p, const Mat& K )
//{
//    return Point2d
//    (
//            ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
//            ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
//    );
//}



};


#endif // CAMERA_H
