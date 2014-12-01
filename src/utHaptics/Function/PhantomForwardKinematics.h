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
* @ingroup dataflow_components
* @file
* phantom forward kinematics component.
*
* @author Ulrich Eck <ulrich.eck@magicvisionlab.com>
*/

#include <utMath/MatrixOperations.h>

namespace Ubitrack { namespace Haptics { namespace Function {

/**
* @ingroup dataflow_components
* Phantom Forward Kinematics Component.
* Given a list of measurements, containing the joint angles O1,O2,O3 and a 3x3 Matrices containing the correction functions, the
* component calculates the pose of the haptic stylus.
*
* An Identity Calibration equals to:
* 0, 1, 0
* 0, 1, 0
* 0, 1, 0
*
* @par Operation
* The component uses correction factors for joint angle sensors of a phantom haptic device to calculate the corrected 6D-Pose.
* The calibration method is based on Harders et al., Calibration, Registration, and Synchronization for High Precision Augmented Reality Haptics,
* IEEE Transactions on Visualization and Computer Graphics, 2009.
*
*/
Math::Pose computePhantomForwardKinematicsPose( const Math::Vector< double, 3 > &joint_angles, const Math::Vector< double, 3 > &gimbal_angles,
                    const Math::Matrix< double, 3, 3 > &ja_correction, const Math::Matrix< double, 3, 3 > &ga_correction,
                    const Math::Vector< double, 2 > &joint_lengths, const Math::Vector< double, 3 > &origin_calib)
    {

        const double l1 = joint_lengths( 0 );
        const double l2 = joint_lengths( 1 );

        const double calx = origin_calib( 0 );
        const double caly = origin_calib( 1 );
        const double calz = origin_calib( 2 );

        const double k1 = ja_correction( 0 , 1 );
        const double m1 = ja_correction( 0 , 2 );
        const double k2 = ja_correction( 1 , 1 );
        const double m2 = ja_correction( 1 , 2 );
        const double k3 = ja_correction( 2 , 1 );
        const double m3 = ja_correction( 2 , 2 );

        const double k4 = ga_correction( 0 , 1 );
        const double m4 = ga_correction( 0 , 2 );
        const double k5 = ga_correction( 1 , 1 );
        const double m5 = ga_correction( 1 , 2 );
        const double k6 = ga_correction( 2 , 1 );
        const double m6 = ga_correction( 2 , 2 );

        const double O1(k1 * joint_angles(0) + m1);
        const double O2(k2 * joint_angles(1) + m2);
        const double O3(k3 * joint_angles(2) + m3);
        const double O4(k4 * gimbal_angles(0) + m4);
        const double O5(k5 * gimbal_angles(1) + m5);
        const double O6(k6 * gimbal_angles(2) + m6);

        const double sO1 = sin(O1);
        const double cO1 = cos(O1);
        const double sO2 = sin(O2);
        const double cO2 = cos(O2);
        const double sO3 = sin(O3);
        const double cO3 = cos(O3);
        const double sO4 = sin(O4);
        const double cO4 = cos(O4);
        const double sO5 = sin(O5);
        const double cO5 = cos(O5);
        const double sO6 = sin(O6);
        const double cO6 = cos(O6);

        // calculate translation
        Math::Vector< double, 3 > trans( calx - (l1*cO2 + l2*sO3)*sO1,
                caly + l1*sO2 - l2*cO3 + l2,
                calz - l1 + (l1*cO2 + l2*sO3)*cO1 );

        // calculate rotation of stylus (6DOF)
        double m[9];

        // sympy 6DOF rotation
        m[0] =  (-(sO1*cO3*cO4 - sO4*cO1)*sO5 - sO1*sO3*cO5)*sO6 + (sO1*sO4*cO3 + cO1*cO4)*cO6;
        m[1] =  ((sO1*cO3*cO4 - sO4*cO1)*sO5 + sO1*sO3*cO5)*cO6 + (sO1*sO4*cO3 + cO1*cO4)*sO6;
        m[2] =  (-sO1*cO3*cO4 + sO4*cO1)*cO5 + sO1*sO3*sO5;
        m[3] =  (sO3*sO5*cO4 - cO3*cO5)*sO6 - sO3*sO4*cO6;
        m[4] =  (-sO3*sO5*cO4 + cO3*cO5)*cO6 - sO3*sO4*sO6;
        m[5] =  sO3*cO4*cO5 + sO5*cO3;
        m[6] =  (-(-sO1*sO4 - cO1*cO3*cO4)*sO5 + sO3*cO1*cO5)*sO6 + (sO1*cO4 - sO4*cO1*cO3)*cO6;
        m[7] =  ((-sO1*sO4 - cO1*cO3*cO4)*sO5 - sO3*cO1*cO5)*cO6 + (sO1*cO4 - sO4*cO1*cO3)*sO6;
        m[8] =  (sO1*sO4 + cO1*cO3*cO4)*cO5 - sO3*sO5*cO1;

        Math::Matrix< double, 3, 3 > rot(m);
        Math::Quaternion q = Math::Quaternion(rot);
        return Math::Pose( q.normalize(), trans );
    }

} } } // namespace Ubitrack::Haptics::Function
