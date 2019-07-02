/*
* Ubitrack - Library for Ubiquitous Tracking
* Copyright 2006, Technische Universitaet Muenchen, and individual
        * contributors as indicated by the @authors tag. See the
* copyright.txt in the distribution for a full listing of individual
        * contributors.
*
* This is free software; you can redistribute it and/or modify it
* under the terms of the GNU Lesser General Public License as
        * published by the Free Software Foundation; either version 2.1 of
        * the License, or (at your option) any later version.
*
* This software is distributed in the hope that it will be useful,
        * but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
        * Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this software; if not, write to the Free
* Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA, or see the FSF site: http://www.fsf.org.
*/


/**
 * @ingroup vision_components_aruci
 * @file
 * Camera Calibration with Charuco Board
 *
 * @author Ulrich Eck <ulrich.eck@tum.de>
 */


#include "CharucoMarkerTracker.h"

#include <utVision/Util/OpenCV.h>


#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.CharucoMarkerTracker" ) );
namespace Ubitrack { namespace Vision {

CharucoMarkerTracker::CharucoMarkerTracker( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
        : ArucoTrackerBase( sName, pCfg )
        , m_squaresX(10)
        , m_squaresY(7)
        , m_squareLength(0.026)
        , m_markerLength(0.02)
        , m_dictionaryId(0)
        , m_refindStrategy(false)
        , m_useUndistoredImage(true)
        , m_inPort( "Image", *this, boost::bind( &CharucoMarkerTracker::pushImage, this, _1 ) )
        , m_inIntrinsics( "CameraIntrinsics", *this )
        , m_debugPort( "DebugImage", *this )
        , m_outPort( "Output", *this )
{
    // Parse Charuco Board
    pCfg->m_DataflowAttributes.getAttributeData( "squaresX", m_squaresX);
    pCfg->m_DataflowAttributes.getAttributeData( "squaresY", m_squaresY);

    pCfg->m_DataflowAttributes.getAttributeData( "squareLength", m_squareLength);
    pCfg->m_DataflowAttributes.getAttributeData( "markerLength", m_markerLength);

    if ( pCfg->m_DataflowAttributes.hasAttribute( "dictionary" ) ) // choose the type of the calibration grid
    {
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_4X4_50")
            m_dictionaryId = 0;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_4X4_100")
            m_dictionaryId = 1;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_4X4_250")
            m_dictionaryId = 2;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_4X4_1000")
            m_dictionaryId = 3;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_5X5_50")
            m_dictionaryId = 4;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_5X5_100")
            m_dictionaryId = 5;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_5X5_250")
            m_dictionaryId = 6;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_5X5_1000")
            m_dictionaryId = 7;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_6X6_50")
            m_dictionaryId = 8;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_6X6_100")
            m_dictionaryId = 9;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_6X6_250")
            m_dictionaryId = 10;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_6X6_1000")
            m_dictionaryId = 11;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_7X7_50")
            m_dictionaryId = 12;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_7X7_100")
            m_dictionaryId = 13;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_7X7_250")
            m_dictionaryId = 14;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_7X7_1000")
            m_dictionaryId = 15;
        if( pCfg->m_DataflowAttributes.getAttributeString( "dictionary" ) == "DICT_ARUCO_ORIGINAL")
            m_dictionaryId = 16;
    }

    // Refind Stragegy enabled
    if ( pCfg->m_DataflowAttributes.hasAttribute( "enableRefindStrategy" ) ) {
        m_refindStrategy = pCfg->m_DataflowAttributes.getAttributeString("enableRefindStrategy" ) == "true";
    }

    // Refind Stragegy enabled
    if ( pCfg->m_DataflowAttributes.hasAttribute( "useUndistortedImage" ) ) {
        m_useUndistoredImage = pCfg->m_DataflowAttributes.getAttributeString("useUndistortedImage" ) == "true";
    }


    // setup datastructures
    m_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(m_dictionaryId));
    m_charucoboard = cv::aruco::CharucoBoard::create(m_squaresX, m_squaresY, m_squareLength, m_markerLength, m_dictionary);

}


/** Method to detect the marker and corner points. */
void CharucoMarkerTracker::pushImage( const Measurement::ImageMeasurement& img )
{
    LOG4CPP_DEBUG( logger, "Acruco Board Calibration Received ImageMeasurement." );
    cv::Ptr<cv::aruco::Board> board = m_charucoboard.staticCast<cv::aruco::Board>();
    cv::Mat image;

    if (img->origin() == 1) {
        LOG4CPP_WARN(logger, "Image with orign lower-left corner are currently not handled correclty. Image is being flipped!!!");
        cv::flip(img->Mat(),image,0);
    } else {
        image = img->Mat();
    }

    Math::CameraIntrinsics<double> camera_intrinsics = *m_inIntrinsics.get(img.time());


   camera_intrinsics = Vision::Util::cv2::correctForOrigin(0, camera_intrinsics);


    cv::Mat camMatrix;
    cv::Mat distCoeffs;
   
    Vision::Util::cv2::assignAndConvertToOpenCV( camera_intrinsics, distCoeffs, camMatrix );

    if(m_useUndistoredImage) {
        distCoeffs = cv::Mat::zeros(4, 1, CV_64F);;
    }



    std::vector< int > markerIds, charucoIds;
    std::vector< std::vector< cv::Point2f > > markerCorners, rejectedMarkers;
    std::vector< cv::Point2f > charucoCorners;
    cv::Vec3d rvec, tvec;

    // detect markers
    cv::aruco::detectMarkers(image, m_dictionary, markerCorners, markerIds, m_arucoDetectorParameters, rejectedMarkers);

    // refind strategy to detect more markers
    if(m_refindStrategy)
        cv::aruco::refineDetectedMarkers(image, board, markerCorners, markerIds, rejectedMarkers, camMatrix, distCoeffs);

    // interpolate charuco corners
    int interpolatedCorners = 0;
    if(markerIds.size() > 0)
        interpolatedCorners = cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, m_charucoboard,
                charucoCorners, charucoIds, camMatrix, distCoeffs);

    // estimate charuco board pose
    bool validPose = false;
    if(camMatrix.total() != 0)
        validPose = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, m_charucoboard, camMatrix, distCoeffs, rvec, tvec);

    // draw results
    if (m_debugPort.isConnected()) {
        boost::shared_ptr< Image > pDebugImg = img->CvtColor(CV_GRAY2RGB, 3);
        cv::Mat imageCopy = pDebugImg->Mat();

        if(markerIds.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, markerCorners);        }

        if(interpolatedCorners > 0) {
            cv::Scalar color;
            color = cv::Scalar(255, 0, 0);
            cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
        }
        if(validPose) {
            float axisLength = 0.5f * ((float)cv::min(m_squaresX, m_squaresY) * (m_squareLength));
            cv::aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);
        }
        m_debugPort.send( Measurement::ImageMeasurement( img.time(), pDebugImg ) );
    }

    if (validPose) {
        // convert tracked pose to Ubitrack::Measurement::Pose
        cv::Mat rotMatOpenCV(3, 3, CV_64FC1);
        cv::Rodrigues(rvec, rotMatOpenCV);

        // convert to right-handed coordinate system
        
        Math::Matrix<double, 3, 3> rotMat;
        rotMat(0, 0) = rotMatOpenCV.at<double>(0, 0);
        rotMat(0, 1) = rotMatOpenCV.at<double>(0, 1);
        rotMat(0, 2) = rotMatOpenCV.at<double>(0, 2);
        rotMat(1, 0) = rotMatOpenCV.at<double>(1, 0);
        rotMat(1, 1) = rotMatOpenCV.at<double>(1, 1);
        rotMat(1, 2) = rotMatOpenCV.at<double>(1, 2);
        rotMat(2, 0) = rotMatOpenCV.at<double>(2, 0);
        rotMat(2, 1) = rotMatOpenCV.at<double>(2, 1);
        rotMat(2, 2) = rotMatOpenCV.at<double>(2, 2);
        

        Math::Quaternion rotTmp = Math::Quaternion(rotMat);
        Math::Quaternion rot = Math::Quaternion(rotTmp.x(), -rotTmp.y(), -rotTmp.z(), rotTmp.w());

   
        Math::Pose pose(rot, Math::Vector<double, 3>(tvec(0), -tvec(1), -tvec(2)));

        // rotate target pose to align with other ubitrack tracker, X to the right, Y up on the marker, Z out of the plane
        Math::Quaternion q1 = Math::Quaternion(1, 0, 0, 0);
        Math::Pose rotpose = Math::Pose(q1, Math::Vector3d(0, 0, 0));
        Math::Pose result = pose*rotpose;


        m_outPort.send(Measurement::Pose(img.time(), result));
    }
}




}} // Ubitrack::Vision
