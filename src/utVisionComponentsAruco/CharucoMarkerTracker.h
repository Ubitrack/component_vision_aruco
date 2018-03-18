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

#include <vector>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "ArucoTrackerBase.h"

#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushSupplier.h>

#include <opencv2/calib3d.hpp>


namespace Ubitrack { namespace Vision {



class CharucoMarkerTracker
        : public ArucoTrackerBase
{
protected:

public:
    CharucoMarkerTracker( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pCfg );

    void pushImage( const Measurement::ImageMeasurement& img );

protected:

    // charuco board config
    int    m_squaresX;
    int    m_squaresY;
    float  m_squareLength;
    float  m_markerLength;
    int    m_dictionaryId;
    bool   m_refindStrategy;

    bool   m_useUndistoredImage;

    // charuco board data
    cv::Ptr<cv::aruco::Dictionary> m_dictionary;
    cv::Ptr<cv::aruco::CharucoBoard> m_charucoboard;

    /** Input Image. */
    Dataflow::PushConsumer< Measurement::ImageMeasurement > m_inPort;

    /** Camera Intrinsics */
    Dataflow::PullConsumer< Measurement::CameraIntrinsics > m_inIntrinsics;

    /** Image for debugging purposes */
    Dataflow::PushSupplier< Measurement::ImageMeasurement > m_debugPort;

    /** Tracked Pose */
    Dataflow::PushSupplier< Measurement::Pose > m_outPort;

};

}} // Ubitrack::Vision

#ifndef UBITRACK_COMPONENT_VISION_ARUCO_CHARUCOMARKERTRACKER_H
#define UBITRACK_COMPONENT_VISION_ARUCO_CHARUCOMARKERTRACKER_H

class CharucoMarkerTracker {

};

#endif //UBITRACK_COMPONENT_VISION_ARUCO_CHARUCOMARKERTRACKER_H
