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

#include "CharucoCameraCalibration.h"


#include <log4cpp/Category.hh>
#include <utVision/Util/OpenCV.h>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.CharucoCameraCalibration" ) );
namespace Ubitrack { namespace Vision {

CharucoCameraCalibration::CharucoCameraCalibration( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
        : ArucoTrackerBase( sName, pCfg )
        , m_squaresX(10)
        , m_squaresY(7)
        , m_squareLength(0.026)
        , m_markerLength(0.02)
        , m_dictionaryId(0)
        , m_calibrationFlags(0)
        , m_aspectRatio(0)
        , m_inPort( "Image", *this, boost::bind( &CharucoCameraCalibration::pushImage, this, _1 ) )
        , m_SignalPort( "Signal", *this, boost::bind( &CharucoCameraCalibration::controlInput, this, _1 ) )
        , m_intrPort( "Output", *this )
        , m_debugPort( "DebugImage", *this )
        , m_mutexThread( )
        , m_origin(1)
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

    // Use Rational Model
    if (pCfg->m_DataflowAttributes.hasAttribute("useRationalModel"))
        if (pCfg->m_DataflowAttributes.getAttributeString("useRationalModel") == "true"){
            m_calibrationFlags |= CV_CALIB_RATIONAL_MODEL;
        }

    // Assume zero tangential distortion
    if ( pCfg->m_DataflowAttributes.hasAttribute( "assumeZeroTangentialDistortion" ) ) {
        if (pCfg->m_DataflowAttributes.getAttributeString("assumeZeroTangentialDistortion" ) == "true") {
            m_calibrationFlags |= cv::CALIB_ZERO_TANGENT_DIST;
        }
    }

    // Fix aspect ratio (fx/fy) to this value (0.0 means not fix)
    if ( pCfg->m_DataflowAttributes.hasAttribute( "fixAspectRatio" ) ) {
        pCfg->m_DataflowAttributes.getAttributeData("fixAspectRatio", m_aspectRatio);
        if (m_aspectRatio > 0.0) {
            m_calibrationFlags |= cv::CALIB_FIX_ASPECT_RATIO;
        }
    }

    // Fix the principal point at the center
    if ( pCfg->m_DataflowAttributes.hasAttribute( "fixPrincipalPointAtCenter" ) ) {
        if (pCfg->m_DataflowAttributes.getAttributeString("fixPrincipalPointAtCenter" ) == "true") {
            m_calibrationFlags |= cv::CALIB_FIX_PRINCIPAL_POINT;
        }
    }

    // Refind Stragegy enabled
    if ( pCfg->m_DataflowAttributes.hasAttribute( "enableRefindStrategy" ) ) {
        m_refindStrategy = pCfg->m_DataflowAttributes.getAttributeString("enableRefindStrategy" ) == "true";
    }

    // reset button config
    std::string buttonReset( " " );
    if ( pCfg->m_DataflowAttributes.hasAttribute( "buttonReset" ) ) {
        buttonReset = pCfg->m_DataflowAttributes.getAttributeString( "buttonReset" );
    }

    if ( buttonReset.empty() ) {
        m_buttonReset = Math::Scalar< int >( -1 );
        LOG4CPP_WARN( logger, "No Button value defined - will react to ANY signal." );
    } else {
        LOG4CPP_INFO( logger, "Calibration Reset value: " << buttonReset[ 0 ] );
        m_buttonReset = Math::Scalar< int >( buttonReset[ 0 ] );
    }

    // setup datastructures
    m_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(m_dictionaryId));
    m_charucoboard = cv::aruco::CharucoBoard::create(m_squaresX, m_squaresY, m_squareLength, m_markerLength, m_dictionary);


}

void CharucoCameraCalibration::reset() {
    LOG4CPP_INFO( logger, "Reset Camera Calibration Data." );
    m_allCorners.clear();
    m_allIds.clear();
}

void CharucoCameraCalibration::controlInput( const Measurement::Button& b )
{
    if ( m_buttonReset < 0 || *b == m_buttonReset ) {
        // do we need to take care of a currently running thread?
        reset();
    }
}

void CharucoCameraCalibration::start() {
    reset();
}

void CharucoCameraCalibration::stop() {
    if (m_pThread) {
        if (m_pThread->joinable()) {
            m_pThread->join();
            m_pThread.reset();
        }
    }
}


void CharucoCameraCalibration::computeIntrinsic( const vector_2d_points allCorners, const vector_marker_ids allIds )
{
    boost::mutex::scoped_lock lock( m_mutexThread );

    cv::Ptr<cv::aruco::Board> board = m_charucoboard.staticCast<cv::aruco::Board>();

    const std::size_t m_values = allCorners.size();

    if( m_values < 2 )
    {
        LOG4CPP_ERROR( logger, "Cannot perform camera calibration, need at lest two different views." );
        return;
    }

    LOG4CPP_INFO( logger, "Acruco Board Calibration Start Calibration." );

    cv::Mat cameraMatrix, distCoeffs;
    std::vector< cv::Mat > rvecs, tvecs;
    double repError;

    if(m_calibrationFlags & cv::CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cameraMatrix.at< double >(0, 0) = m_aspectRatio;
    }

    // prepare data for calibration
    std::vector< std::vector< cv::Point2f > > allCornersConcatenated;
    std::vector< int > allIdsConcatenated;
    std::vector< int > markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners.size());
    for(unsigned int i = 0; i < allCorners.size(); i++) {
        markerCounterPerFrame.push_back((int)allCorners[i].size());
        for(unsigned int j = 0; j < allCorners[i].size(); j++) {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }

    try
    {
        // calibrate camera
        repError = cv::aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                markerCounterPerFrame, board, m_imgSize, cameraMatrix,
                distCoeffs, rvecs, tvecs, m_calibrationFlags);
    }
    catch( const std::exception & e )
    {
        LOG4CPP_ERROR( logger, "Cannot perform camera calibration, error in OpenCV function call.\n" << e.what() );
        return;
    }

    Math::CameraIntrinsics<double> m_camIntrinsics;

    ///@todo check if the paramrers should not be flipped, as it is done at some other places.
    Math::Matrix< double, 3, 3 > intrinsic;
    intrinsic(0, 0) = cameraMatrix.at<double>(0, 0);
    intrinsic(0, 1) = cameraMatrix.at<double>(0, 1);
    intrinsic(0, 2) = -cameraMatrix.at<double>(0, 2);
    intrinsic(1, 0) = cameraMatrix.at<double>(1, 0);
    intrinsic(1, 1) = cameraMatrix.at<double>(1, 1);
    intrinsic(1, 2) = -cameraMatrix.at<double>(1, 2);
    intrinsic(2, 0) = cameraMatrix.at<double>(2, 0);
    intrinsic(2, 1) = cameraMatrix.at<double>(2, 1);
    intrinsic(2, 2) = -cameraMatrix.at<double>(2, 2);

    if (m_calibrationFlags & CV_CALIB_RATIONAL_MODEL) {
        Math::Vector< double, 6 > radial;
        radial(0) = distCoeffs.at<double>(0);
        radial(1) = distCoeffs.at<double>(1);
        radial(2) = distCoeffs.at<double>(4);
        radial(3) = distCoeffs.at<double>(5);
        radial(4) = distCoeffs.at<double>(6);
        radial(5) = distCoeffs.at<double>(7);

        const Math::Vector< double, 2 > tangential(distCoeffs.at<double>(2), distCoeffs.at<double>(3));

        m_camIntrinsics = Math::CameraIntrinsics< double >(intrinsic, radial, tangential, (std::size_t)m_imgSize.width, (std::size_t)m_imgSize.height);
    } else {
        Math::Vector< double, 2 > radial;
        radial(0) = distCoeffs.at<double>(0);
        radial(1) = distCoeffs.at<double>(1);

        const Math::Vector< double, 2 > tangential(distCoeffs.at<double>(2), distCoeffs.at<double>(3));

        m_camIntrinsics = Math::CameraIntrinsics< double >(intrinsic, radial, tangential, (std::size_t)m_imgSize.width, (std::size_t)m_imgSize.height);
    }
    
    // check origin of images used, since everything is done internally by aruco we need to flip the result of the image origin is 0, ubitrack default is 1
    m_camIntrinsics = Vision::Util::cv2::correctForOrigin(m_origin, m_camIntrinsics);

    m_intrPort.send( Measurement::CameraIntrinsics ( Measurement::now(), m_camIntrinsics ) );
    LOG4CPP_INFO( logger, "Finished camera calibration using " << m_values << " views." );
}


/** Method to detect the marker and corner points. */
void CharucoCameraCalibration::pushImage( const Measurement::ImageMeasurement& img )
{
    LOG4CPP_DEBUG( logger, "Acruco Board Calibration Received ImageMeasurement." );
    cv::Ptr<cv::aruco::Board> board = m_charucoboard.staticCast<cv::aruco::Board>();
    cv::Mat image = img->Mat();
    m_origin = img->origin();

    std::vector< int > ids;
    std::vector< std::vector< cv::Point2f > > corners, rejected;

    // detect markers
    cv::aruco::detectMarkers(image, m_dictionary, corners, ids, m_arucoDetectorParameters, rejected);

    // refind strategy to detect more markers
    if(m_refindStrategy)
        cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

    // interpolate charuco corners
    cv::Mat currentCharucoCorners, currentCharucoIds;
    if(ids.size() > 0) {
        cv::aruco::interpolateCornersCharuco(corners, ids, image, m_charucoboard, currentCharucoCorners, currentCharucoIds);
    } else {
        LOG4CPP_INFO( logger, "No markers were detected." );
        return;
    }

    // draw results
    if (m_debugPort.isConnected()) {
        boost::shared_ptr< Image > pDebugImg = img->CvtColor(CV_GRAY2RGB, 3);
        cv::Mat imageCopy = pDebugImg->Mat();

        if(ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners);
        }
        if(currentCharucoCorners.total() > 0) {
            cv::aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
        }
        m_debugPort.send( Measurement::ImageMeasurement( img.time(), pDebugImg ) );
    }

    m_allCorners.push_back(corners);
    m_allIds.push_back(ids);
    m_imgSize = image.size();

    LOG4CPP_INFO( logger, "Size of Calibrations: " << m_allIds.size() );

    if (m_allIds.size() > 2) {
        if( !boost::mutex::scoped_try_lock ( m_mutexThread ) )
        {
            LOG4CPP_WARN( logger, "Cannot perform camera calibration, thread is still busy." );
            return;
        }
        else
        {
            // copy data for threaded calibration computation
            vector_2d_points allCorners = m_allCorners;
            vector_marker_ids allIds = m_allIds;
            m_pThread.reset( new boost::thread( boost::bind( &CharucoCameraCalibration::computeIntrinsic, this, allCorners, allIds)));
        }
    }
}




}} // Ubitrack::Vision
