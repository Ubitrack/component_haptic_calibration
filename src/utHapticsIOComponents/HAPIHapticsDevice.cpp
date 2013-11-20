/*
 * HAPIHapticsDevice.cpp
 *
 *  Created on: 12/11/2013
 *      Author: mvl
 */
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

#include "HAPIHapticsDevice.h"

#include <vector>
#include <sstream>
#include <iostream>

#include <utUtil/OS.h>

#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

// have a logger..
#include <log4cpp/Category.hh>
static log4cpp::Category& logger(
		log4cpp::Category::getInstance("Drivers.HAPIHapticsDevice"));


namespace Ubitrack {
namespace Drivers {

class Sensor3DOFForceEffect : public HAPI::HAPIForceEffect {
public:
	/// Constructor
	Sensor3DOFForceEffect(HAPIDeviceSensor3DOF* _receiver)
		: receiver(_receiver) {
		// no initialization needed for now
	}


	/// The force of the EffectOutput will be a force from the position of
	/// the haptics device to the position of the HapticSpring.
	HAPI::HAPIForceEffect::EffectOutput calculateForces( const HAPI::HAPIForceEffect::EffectInput &input ) {

		unsigned long long tstamp = Measurement::now();

		HAPI::Vec3 device_pos = input.hd->getPosition();
		Math::Vector< 3 > pos(device_pos.x, device_pos.y, device_pos.z);
		receiver->sendPosition(tstamp, pos);

		// return no force - this is only a sensor for now
		return HAPI::HAPIForceEffect::EffectOutput();
	}

protected:
	HAPIDeviceSensor3DOF* receiver;
};

class Sensor6DOFForceEffect : public HAPI::HAPIForceEffect {
public:
	/// Constructor
	Sensor6DOFForceEffect(HAPIDeviceSensor6DOF* _receiver)
		: receiver(_receiver) {
		// no initialization needed for now
	}


	/// The force of the EffectOutput will be a force from the position of
	/// the haptics device to the position of the HapticSpring.
	HAPI::HAPIForceEffect::EffectOutput calculateForces( const HAPI::HAPIForceEffect::EffectInput &input ) {

		unsigned long long tstamp = Measurement::now();

		HAPI::Vec3 device_pos = input.hd->getPosition();
		HAPI::Quaternion device_orn = HAPI::Quaternion(input.hd->getOrientation());
		Math::Pose pose(
				Math::Quaternion(device_orn.v.x, device_orn.v.y, device_orn.v.z, device_orn.w),
				Math::Vector< 3 >(device_pos.x,device_pos.y, device_pos.z)
				);
		receiver->sendPose(tstamp, pose);

		// return no force - this is only a sensor for now
		return HAPI::HAPIForceEffect::EffectOutput();
	}

protected:
	HAPIDeviceSensor6DOF* receiver;
};


class SensorPhantomForceEffect : public HAPI::HAPIForceEffect {
public:
	/// Constructor
	SensorPhantomForceEffect(HAPIDeviceSensorPhantom* _receiver)
		: receiver(_receiver) {
		// no initialization needed for now
	}


	/// The force of the EffectOutput will be a force from the position of
	/// the haptics device to the position of the HapticSpring.
	HAPI::HAPIForceEffect::EffectOutput calculateForces( const HAPI::HAPIForceEffect::EffectInput &input ) {

		LOG4CPP_TRACE(logger, "SensorPhantomForceEffect calculateForces.");

		unsigned long long tstamp = Measurement::now();
		// retrieve information from haptic device state
		HAPI::PhantomHapticsDevice* hd = static_cast<HAPI::PhantomHapticsDevice*>(input.hd);

		HAPI::Vec3 device_pos = hd->getPosition();
		HAPI::Quaternion device_orn = HAPI::Quaternion(hd->getOrientation());
		Math::Pose pose(
				Math::Quaternion(device_orn.v.x, device_orn.v.y, device_orn.v.z, device_orn.w),
				Math::Vector< 3 >(device_pos.x,device_pos.y, device_pos.z)
				);
		receiver->sendPose(tstamp, pose);

		HAPI::Vec3 jA = hd->getJointAngles();
		Math::Vector< 3 > joint_angles(jA.x, jA.y, jA.z);
		// send pose to DFG
		receiver->sendJointAngles(tstamp, joint_angles);

		HAPI::Vec3 gA = hd->getGimbalAngles();
		Math::Vector< 3 > gimbal_angles(gA.x, gA.y, gA.z);
		// send pose to DFG
		receiver->sendGimbalAngles(tstamp, gimbal_angles);

		// return no force - this is only a sensor for now
		return HAPI::HAPIForceEffect::EffectOutput();
	}

protected:
	HAPIDeviceSensorPhantom* receiver;
};





HAPIDeviceModule::HAPIDeviceModule(const HAPIDeviceModuleKey& moduleKey,
		boost::shared_ptr<Graph::UTQLSubgraph> subgraph, FactoryHelper* pFactory) :
		HAPIDeviceModuleBase(moduleKey, pFactory)
	, m_stop(true)
	, m_counter(0)
	, m_lastTimestamp(0)
	, m_hdev(NULL)
	, m_deviceType(HAPI_ANYDEVICE)
	, m_deviceName("")
	, m_threadFrequency(1024)
{

	LOG4CPP_INFO(logger, "Initializing Haptic device " << moduleKey.get() << ".");

	std::string key = moduleKey.get();
	int cpos = key.find(':');
	if (cpos > 0 ) {
		std::string m_deviceName = key.substr(cpos+1, key.length());
		std::string devType = key.substr(0, cpos);

		if (devType.compare("phantom") == 0) {
			m_deviceType = HAPI_PHANTOMDEVICE;
		}
	}

	Graph::UTQLSubgraph::NodePtr config;
	if ( subgraph->hasNode( "HAPIDevice" ) )
	  config = subgraph->getNode( "HAPIDevice" );

	if ( !config )
	{
	  UBITRACK_THROW( "HAPIDeviceModule Pattern has no \"HAPIDevice\" node");
	}

	config->getAttributeData("threadFrequency", m_threadFrequency );


}
;
void HAPIDeviceModule::stopModule() {

	// stop components
	ComponentList allComponents( getAllComponents() );
	for ( ComponentList::iterator it = allComponents.begin(); it != allComponents.end(); it++ ) {
		it->get()->stopComponent(m_hdev);
	}

	// cleanup
	m_hdev->disableDevice();
	m_hdev->releaseDevice();
	delete m_hdev;
}


void HAPIDeviceModule::startModule() {

	// init device
	if (m_deviceType == HAPI_PHANTOMDEVICE) {
		m_deviceType = HAPI_PHANTOMDEVICE;
		m_hdev = new HAPI::PhantomHapticsDevice(m_deviceName);
	} else {
		m_hdev = new HAPI::AnyHapticsDevice();
	}

	if (m_hdev->initDevice(m_threadFrequency) != HAPI::HAPIHapticsDevice::SUCCESS) {
		LOG4CPP_ERROR(logger,
						"Cannot initialize haptic device: " << m_deviceName << ".");
				LOG4CPP_ERROR(logger, m_hdev->getLastErrorMsg());
				return;
	}


	m_hdev->enableDevice();

	if (m_deviceType == HAPI_PHANTOMDEVICE) {
		HAPI::PhantomHapticsDevice* phdev = static_cast<HAPI::PhantomHapticsDevice*>(m_hdev);
		if (phdev->needsCalibration()){
			LOG4CPP_INFO(logger, "AutoCalibrating Haptic Device - Please put device into calibration position (e.g. inkwell).");
			do {
				if (!phdev->calibrateDevice()) {
					LOG4CPP_ERROR(logger, "AutoCalibration failed.");
					break;
				}
			} while (!phdev->needsCalibration());
			LOG4CPP_INFO(logger, "AutoCalibration finished.");
		}
	}

	m_hdev->clearEffects();
	m_hdev->transferObjects();

	// start components
	ComponentList allComponents( getAllComponents() );
	for ( ComponentList::iterator it = allComponents.begin(); it != allComponents.end(); it++ ) {
		it->get()->startComponent(m_hdev);
	}

}

boost::shared_ptr< HAPIDeviceModule::ComponentClass > HAPIDeviceModule::createComponent( const std::string&, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph,
	const ComponentKey& key, ModuleClass* pModule )
{
	boost::shared_ptr< HAPIDeviceModule::ComponentClass > component;
	std::string ck = key.get();

	if (ck.compare("3dof_sensor") == 0) {
		component.reset( new HAPIDeviceSensor3DOF(name, subgraph, key, pModule));
	} else if (ck.compare("6dof_sensor") == 0) {
		if (m_deviceType == HAPI_PHANTOMDEVICE) {
			component.reset( new HAPIDeviceSensorPhantom(name, subgraph, key, pModule));
		} else {
			component.reset( new HAPIDeviceSensor6DOF(name, subgraph, key, pModule));
		}
	}
//	case '3dof_actuator':
//		component.reset( new HAPIDeviceSensor3DOF(name, subgraph, key, pModule));
//		break;
//	case '6dof_actuator':
//		component.reset( new HAPIDeviceSensor3DOF(name, subgraph, key, pModule));
//		break;

	return component;
}

HAPIDeviceModuleComponent::HAPIDeviceModuleComponent(const std::string &name,
		boost::shared_ptr<Graph::UTQLSubgraph> subgraph,
		const HAPIDeviceComponentKey &componentKey,
		HAPIDeviceModule *pModule)
	: HAPIDeviceModule::Component(name, componentKey, pModule)
	, m_effect(NULL)
	, m_lastTimestamp(0)
	, m_counter(0)
{
};


void HAPIDeviceModuleComponent::startComponent(HAPI::HAPIHapticsDevice* dev) {
	// add force effect
	m_effect = _createForceEffect();
	dev->addEffect(m_effect);
	dev->transferObjects();
}

void HAPIDeviceModuleComponent::stopComponent(HAPI::HAPIHapticsDevice* dev) {
	// remove force effect
	dev->removeEffect(m_effect, 0);
	dev->transferObjects();
	delete m_effect;
}

HAPI::HAPIForceEffect* HAPIDeviceModuleComponent::_createForceEffect() {
	UBITRACK_THROW("_createForceEffect needs to be specialized");
	return NULL;
}


HAPIDeviceSensor3DOF::HAPIDeviceSensor3DOF(const std::string &name,
		boost::shared_ptr<Graph::UTQLSubgraph> subgraph,
		const HAPIDeviceComponentKey &componentKey, HAPIDeviceModule *pModule)
	: HAPIDeviceModuleComponent(name, subgraph, componentKey, pModule)
	, m_outPort("Position", *this)
{
	LOG4CPP_DEBUG(logger,
			"HAPIDeviceSensor3DOF created.");
};

HAPI::HAPIForceEffect* HAPIDeviceSensor3DOF::_createForceEffect() {
	LOG4CPP_DEBUG(logger, "Create HAPIDeviceSensor3DOF ForceEffect.");
	return new Sensor3DOFForceEffect(this);
}

void HAPIDeviceSensor3DOF::sendPosition(const Measurement::Timestamp t, const Math::Vector< 3 >& v) {
	Measurement::Position MeasPos(t, v);
	m_outPort.send(MeasPos);
}



HAPIDeviceSensor6DOF::HAPIDeviceSensor6DOF(const std::string &name,
		boost::shared_ptr<Graph::UTQLSubgraph> subgraph,
		const HAPIDeviceComponentKey &componentKey, HAPIDeviceModule *pModule)
	: HAPIDeviceModuleComponent(name, subgraph, componentKey, pModule)
	, m_outPort("Pose", *this)
{
	LOG4CPP_DEBUG(logger,
			"HAPIDeviceSensor6DOF created.");
};

HAPI::HAPIForceEffect* HAPIDeviceSensor6DOF::_createForceEffect() {
	LOG4CPP_DEBUG(logger, "Create HAPIDeviceSensor6DOF ForceEffect.");
	return new Sensor6DOFForceEffect(this);
}

void HAPIDeviceSensor6DOF::sendPose(const Measurement::Timestamp t, const Math::Pose& p) {
	Measurement::Pose MeasPose(t, p);
	m_outPort.send(MeasPose);
}


HAPIDeviceSensorPhantom::HAPIDeviceSensorPhantom(const std::string &name,
		boost::shared_ptr<Graph::UTQLSubgraph> subgraph,
		const HAPIDeviceComponentKey &componentKey, HAPIDeviceModule *pModule)
	: HAPIDeviceModuleComponent(name, subgraph, componentKey, pModule)
	, m_outPort("Pose", *this)
	, m_outJointAnglesPort("JointAngles", *this)
	, m_outGimbalAnglesPort("GimbalAngles", *this)
{
	LOG4CPP_DEBUG(logger,
			"HAPIDeviceSensorPhantom created.");
};

HAPI::HAPIForceEffect* HAPIDeviceSensorPhantom::_createForceEffect() {
	LOG4CPP_DEBUG(logger, "Create HAPIDeviceSensorPhantom ForceEffect.");
	return new SensorPhantomForceEffect(this);
}

void HAPIDeviceSensorPhantom::sendPose(const Measurement::Timestamp t, const Math::Pose& p) {
	Measurement::Pose MeasPose(t, p);
	m_outPort.send(MeasPose);
}

void HAPIDeviceSensorPhantom::sendJointAngles(const Measurement::Timestamp t, const Math::Vector< 3 >& v) {
	Measurement::Position MeasPos(t, v);
	m_outJointAnglesPort.send(MeasPos);
}

void HAPIDeviceSensorPhantom::sendGimbalAngles(const Measurement::Timestamp t, const Math::Vector< 3 >& v) {
	Measurement::Position MeasPos(t, v);
	m_outGimbalAnglesPort.send(MeasPos);
}





// register module at factory
UBITRACK_REGISTER_COMPONENT (Dataflow::ComponentFactory* const cf) {

// create list of supported types
	std::vector<std::string> HAPIHapticsDeviceComponents;
	HAPIHapticsDeviceComponents.push_back("HAPIHapticsDevice");

	cf->registerModule<HAPIDeviceModule>(HAPIHapticsDeviceComponents);
}

}
} // namespace Ubitrack::Drivers

