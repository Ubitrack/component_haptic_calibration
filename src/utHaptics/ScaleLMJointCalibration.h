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
 * @ingroup haptic_algorithms
 * @file
 * Functions for Workspace Calibration of Scale Haptic Devices
 *
 * @author Ulrich Eck <ulrich.eck@magicvisionlab.com>
 */ 

#ifndef __UBITRACK_HAPTICS_PHANTOMLMCALIBRATION_H_INCLUDED__
#define __UBITRACK_HAPTICS_PHANTOMLMCALIBRATION_H_INCLUDED__

#include <utHaptics.h>
#include <utCore.h>
#include <utMath/Vector.h>
#include <utMath/Matrix.h>

namespace Ubitrack { namespace Haptics {

#ifdef HAVE_LAPACK
/**
 * @ingroup haptics_algorithms
 * Calculates correction factors for non-uniform workspace calibration of phantom haptics devices using forward kinematics
 * and externally measured reference points.
 *
 * The result is a 3x4 Matrix representing a correction factor (k) and an offset (m) for each of the 6 angles of 
 * the device. Currently only the first 3 angles for the joints are computed.
 * The matrix is structured as [ k01, m01, k02, m02; k03, m03, k04, m04; k05, m05, k06, m06]
 *
 *
 * @param jointangles the list of 3-Vectors representing the joint angles of the phantom
 * // future: gimbalangles the list of 3-Vectors representing the gimbal angles of the phantom
 * @param points the list of 3-Vectors representing the measured points corresponding to the joint angles
 * @param l1 length of the first joint
 * @param l2 length of the second joint
 * @return correctionFactors k_i, m_i for each of the 6 (currently 3) angles
 */
UTHAPTICS_EXPORT Math::Matrix< float, 3, 3 > computeScaleLMCalibration( const std::vector< Math::Vector< float, 3 > > & platformsensors, const std::vector< Math::Vector< float, 3 > > & jointangles, const std::vector< Math::Vector< float, 3 > > & points, const float l1, const float l2, const float optimizationStepSize, const float optimizationStepFactor );

UTHAPTICS_EXPORT Math::Matrix< double, 3, 3 > computeScaleLMCalibration( const std::vector< Math::Vector< double, 3 > > & platformsensors, const std::vector< Math::Vector< double, 3 > > & jointangles, const std::vector< Math::Vector< double, 3 > > & points, const double l1, const double l2, const double optimizationStepSize, const double optimizationStepFactor );

#endif

} } // namespace Ubitrack::Calibration

#endif
