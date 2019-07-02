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

#ifndef UBITRACK_COMPONENT_VISION_ARUCO_CHARUCOCAMERACALIBRATION_H
#define UBITRACK_COMPONENT_VISION_ARUCO_CHARUCOCAMERACALIBRATION_H

#include <vector>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "ArucoTrackerBase.h"

#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushSupplier.h>

#include <opencv2/calib3d.hpp>


namespace Ubitrack { namespace Vision {



class CharucoCameraCalibration
        : public ArucoTrackerBase
{
protected:
    typedef std::vector< std::vector< std::vector< cv::Point2f > > > vector_2d_points;
    typedef std::vector< std::vector< int > > vector_marker_ids;

public:
    CharucoCameraCalibration( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pCfg );

    void reset();
    void controlInput( const Measurement::Button& b );
    void pushImage( const Measurement::ImageMeasurement& img );

    virtual void start();
    virtual void stop();

    /** Method that computes the result. */
    void computeIntrinsic( const vector_2d_points allCorners, const vector_marker_ids allIds );

protected:

    // charuco board config
    int    m_squaresX;
    int    m_squaresY;
    float  m_squareLength;
    float  m_markerLength;
    int    m_dictionaryId;
    int    m_calibrationFlags;
    bool   m_refindStrategy;
    float  m_aspectRatio;

    // charuco board data
    cv::Ptr<cv::aruco::Dictionary> m_dictionary;
    cv::Ptr<cv::aruco::CharucoBoard> m_charucoboard;

    // working data
    vector_2d_points m_allCorners;
    vector_marker_ids m_allIds;
    cv::Size m_imgSize;

    // origin flag of image to correctly convert calibration to ubitrack format
    int m_origin;

    // reset button
    Math::Scalar<int> m_buttonReset;

    /** signs if the thread is running. */
    boost::mutex m_mutexThread;

    /** thread performing camera calibration in the background. */
    boost::scoped_ptr< boost::thread > m_pThread;


    /** Input Image. */
    Dataflow::PushConsumer< Measurement::ImageMeasurement > m_inPort;

    /** Signal Port. */
    Dataflow::PushConsumer< Measurement::Button > m_SignalPort;

    /** Camera Intrinsics */
    Dataflow::PushSupplier< Measurement::CameraIntrinsics > m_intrPort;

    /** Image for debugging purposes */
    Dataflow::PushSupplier< Measurement::ImageMeasurement > m_debugPort;

};

}} // Ubitrack::Vision

#endif //UBITRACK_COMPONENT_VISION_ARUCO_CHARUCOCAMERACALIBRATION_H
