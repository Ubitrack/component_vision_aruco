//
// Created by Ulrich Eck on 17.03.18.
//

#ifndef UBITRACK_COMPONENT_VISION_ARUCO_ARUCOTRACKERBASE_H
#define UBITRACK_COMPONENT_VISION_ARUCO_ARUCOTRACKERBASE_H

#include <string>

#include <boost/shared_ptr.hpp>

#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMath/Vector.h>
#include <utUtil/Exception.h>
#include <utVision/Image.h>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace Ubitrack { namespace Vision {

class ArucoTrackerBase
        : public Dataflow::Component
{
public:
    ArucoTrackerBase( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
            : Dataflow::Component( sName )
            , m_arucoDetectorParameters(cv::aruco::DetectorParameters::create())
    {
        // read aruco detector parameters
        pCfg->m_DataflowAttributes.getAttributeData("adaptiveThreshConstant", m_arucoDetectorParameters->adaptiveThreshConstant);
        pCfg->m_DataflowAttributes.getAttributeData("adaptiveThreshWinSizeMax", m_arucoDetectorParameters->adaptiveThreshWinSizeMax);
        pCfg->m_DataflowAttributes.getAttributeData("adaptiveThreshWinSizeMin", m_arucoDetectorParameters->adaptiveThreshWinSizeMin);
        pCfg->m_DataflowAttributes.getAttributeData("adaptiveThreshWinSizeStep", m_arucoDetectorParameters->adaptiveThreshWinSizeStep);
        pCfg->m_DataflowAttributes.getAttributeData("cornerRefinementMaxIterations", m_arucoDetectorParameters->cornerRefinementMaxIterations);
        pCfg->m_DataflowAttributes.getAttributeData("cornerRefinementMethod", m_arucoDetectorParameters->cornerRefinementMethod);
        pCfg->m_DataflowAttributes.getAttributeData("cornerRefinementMinAccuracy", m_arucoDetectorParameters->cornerRefinementMinAccuracy);
        pCfg->m_DataflowAttributes.getAttributeData("cornerRefinementWinSize", m_arucoDetectorParameters->cornerRefinementWinSize);
        pCfg->m_DataflowAttributes.getAttributeData("errorCorrectionRate", m_arucoDetectorParameters->errorCorrectionRate);
        pCfg->m_DataflowAttributes.getAttributeData("markerBorderBits", m_arucoDetectorParameters->markerBorderBits);
        pCfg->m_DataflowAttributes.getAttributeData("maxErroneousBitsInBorderRate", m_arucoDetectorParameters->maxErroneousBitsInBorderRate);
        pCfg->m_DataflowAttributes.getAttributeData("maxMarkerPerimeterRate", m_arucoDetectorParameters->maxMarkerPerimeterRate);
        pCfg->m_DataflowAttributes.getAttributeData("minCornerDistanceRate", m_arucoDetectorParameters->minCornerDistanceRate);
        pCfg->m_DataflowAttributes.getAttributeData("minDistanceToBorder", m_arucoDetectorParameters->minDistanceToBorder);
        pCfg->m_DataflowAttributes.getAttributeData("minMarkerDistanceRate", m_arucoDetectorParameters->minMarkerDistanceRate);
        pCfg->m_DataflowAttributes.getAttributeData("minMarkerPerimeterRate", m_arucoDetectorParameters->minMarkerPerimeterRate);
        pCfg->m_DataflowAttributes.getAttributeData("minOtsuStdDev", m_arucoDetectorParameters->minOtsuStdDev);
        pCfg->m_DataflowAttributes.getAttributeData("perspectiveRemoveIgnoredMarginPerCell", m_arucoDetectorParameters->perspectiveRemoveIgnoredMarginPerCell);
        pCfg->m_DataflowAttributes.getAttributeData("perspectiveRemovePixelPerCell", m_arucoDetectorParameters->perspectiveRemovePixelPerCell);
        pCfg->m_DataflowAttributes.getAttributeData("polygonalApproxAccuracyRate", m_arucoDetectorParameters->polygonalApproxAccuracyRate);

    }


protected:

    cv::Ptr<cv::aruco::DetectorParameters> m_arucoDetectorParameters;

};



}} // Ubitrack::Vision

#endif //UBITRACK_COMPONENT_VISION_ARUCO_ARUCOTRACKERBASE_H
