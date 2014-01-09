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
 * Functions Estimate the Joint Lengths of Phantom Haptic Devices
 *
 * @author Ulrich Eck <ulrich.eck@magicvisionlab.com>
 */ 

#ifndef __UBITRACK_HAPTICS_PHANTOMLMJOINTLENGTH_H_INCLUDED__
#define __UBITRACK_HAPTICS_PHANTOMLMJOINTLENGTH_H_INCLUDED__

#include <utHaptics.h>
#include <utCore.h>
#include <utMath/Vector.h>
#include <utMath/Matrix.h>

namespace Ubitrack { namespace Haptics {

#ifdef HAVE_LAPACK
/**
 * @ingroup haptics_algorithms
 * Estimates the lengths of the two joints of a phantom device usint levenberg-marquardt.
 *
 *
 * @param jointangles the list of 3-Vectors representing the joint angles of the phantom
 * @param points the list of 3-Vectors representing the measured points corresponding to the joint angles from the haptic device
 * @param l1_est, l2_est .. initial guess for parameters
 * @return correctionFactors l1, l2
 */
UTHAPTICS_EXPORT Math::Vector< float, 5 > computePhantomLMJointLength( const std::vector< Math::Vector< float, 3 > > & jointangles, const std::vector< Math::Vector< float, 3 > > & points, const float l1_est, const float l2_est, const Math::Vector< float, 3 > & origin_est );

UTHAPTICS_EXPORT Math::Vector< double, 5 > computePhantomLMJointLength( const std::vector< Math::Vector< double, 3 > > & jointangles, const std::vector< Math::Vector< double, 3 > > & points, const double l1_est, const double l2_est, const Math::Vector< double, 3 > & origin_est );

#endif

} } // namespace Ubitrack::Calibration

#endif
