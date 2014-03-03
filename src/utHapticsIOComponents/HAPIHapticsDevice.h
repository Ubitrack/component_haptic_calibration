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
 * @ingroup driver_components
 * @file
 * HAPIHapticsDevice
 * This file contains the driver component to
 * receive measurements and send forces to/from haptic devices.
 *
 * The driver is built as a module to handle
 * communication with multiple devices.
 *
 * @author Ulrich Eck <ulrich.eck@magicvisionlab.com>
 */

#ifndef HAPIHAPTICSDEVICE_H_
#define HAPIHAPTICSDEVICE_H_

//std
#include <string>
#include <cstdlib>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

// HAPI
#include <HAPI/HAPIForceEffect.h>
#include <HAPI/HapticSpring.h>
#include <HAPI/AnyHapticsDevice.h>
#include <HAPI/PhantomHapticsDevice.h>

//Ubitrack
#include <utMath/Pose.h>
#include <utMath/Scalar.h>
#include <utDataflow/Module.h>
#include <utDataflow/Component.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullConsumer.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>

using namespace Ubitrack::Measurement;
using namespace Ubitrack::Dataflow;
using namespace Ubitrack;

namespace Ubitrack {
namespace Drivers {

enum HAPI_DEVICE_TYPE {
	HAPI_ANYDEVICE    = 0x00,
	HAPI_PHANTOMDEVICE    = 0x01
};


/**
 * deviceName for module identification.
 * Represents the haptic device
 */
class HAPIDeviceModuleKey: public NodeAttributeKey<std::string> {
public:
	HAPIDeviceModuleKey(boost::shared_ptr<Graph::UTQLSubgraph> subgraph) :
			NodeAttributeKey<std::string>(subgraph, "HAPIDevice", "deviceName", "") // default value is empty
	{
	}
};

/**
 * Tracker key for component identification.
 * Represents the tracker number within the shared memory.
 */
class HAPIDeviceComponentKey: public DataflowConfigurationAttributeKey<std::string> {
public:
	HAPIDeviceComponentKey(boost::shared_ptr<Graph::UTQLSubgraph> subgraph) :
			DataflowConfigurationAttributeKey<std::string>(subgraph, "componentType", "") // default value is sensor
	{
	}
};

//forward declaration for the module
class HAPIDeviceModule;
class HAPIDeviceModuleComponent;

//Definition of the Module to exchange shared information between several Ubitrack components of the same type
typedef Module<HAPIDeviceModuleKey, HAPIDeviceComponentKey, HAPIDeviceModule,
		HAPIDeviceModuleComponent> HAPIDeviceModuleBase;

class HAPIDeviceModule: public HAPIDeviceModuleBase {

public:

	//  Constructor
	HAPIDeviceModule(const HAPIDeviceModuleKey& moduleKey,
			boost::shared_ptr<Graph::UTQLSubgraph>, FactoryHelper* pFactory);

	// Function to create the right component
	boost::shared_ptr<HAPIDeviceModuleComponent> createComponent(
			const std::string& type, const std::string& name,
			boost::shared_ptr<Graph::UTQLSubgraph> subgraph,
			const ComponentKey& componentKey, ModuleClass* pModule);

	//  Destructor
	~HAPIDeviceModule() {
		m_hdev->releaseDevice();
		m_hdev.reset();
	}
	;
	/** stops the module */
	virtual void stopModule();
	/** starts the module */
	virtual void startModule();

protected:

	/** signs if the whole module should be stopped. */
	bool m_stop;

	/** measurements received */
	std::size_t m_counter;

	/** Timestamp of the last received measurement */
	Ubitrack::Measurement::Timestamp m_lastTimestamp;

	/** pointer to the haptic device instance */
	boost::shared_ptr<HAPI::HAPIHapticsDevice> m_hdev;

	/** device type **/
	HAPI_DEVICE_TYPE m_deviceType;

	/** device name **/
	std::string m_deviceName;

	/** update interval **/
	int m_threadFrequency;

	/** calibrate device on startup **/
	bool m_calibrateOnStartup;

};

class HAPIDeviceModuleComponent: public HAPIDeviceModule::Component {

public:
	//  Constructor
	HAPIDeviceModuleComponent(const std::string &name,
			boost::shared_ptr<Graph::UTQLSubgraph> subgraph,
			const HAPIDeviceComponentKey &componentKey,
			HAPIDeviceModule* pModule);

	/** stops the component */
	virtual void stopComponent(boost::shared_ptr<HAPI::HAPIHapticsDevice> dev);
	/** starts the module */
	virtual void startComponent(boost::shared_ptr<HAPI::HAPIHapticsDevice> dev);

protected:

	virtual boost::shared_ptr<HAPI::HAPIForceEffect> _createForceEffect();

	/** measurments received */
	std::size_t m_counter;

	/** Timestamp of the last received measurement */
	Ubitrack::Measurement::Timestamp m_lastTimestamp;

	/** pointer to the force effect that is called from the haptics loop */
	boost::shared_ptr<HAPI::HAPIForceEffect> m_effect;

};


class HAPIDeviceSensor3DOF: public HAPIDeviceModuleComponent, public boost::enable_shared_from_this<HAPIDeviceSensor3DOF> {

public:
	//  Constructor
	HAPIDeviceSensor3DOF(const std::string &name,
			boost::shared_ptr<Graph::UTQLSubgraph> subgraph,
			const HAPIDeviceComponentKey &componentKey,
			HAPIDeviceModule* pModule);

	/** method to convert the raw data and send it to the port */
	void sendPosition(const Measurement::Timestamp, const Math::Vector< double, 3 >& pos);

protected:

	virtual boost::shared_ptr<HAPI::HAPIForceEffect> _createForceEffect();

	/** Pose output port of the component */
	Dataflow::PushSupplier<Measurement::Position> m_outPort;

};

class HAPIDeviceSensor6DOF: public HAPIDeviceModuleComponent, public boost::enable_shared_from_this<HAPIDeviceSensor6DOF> {

public:
    //  Constructor
	HAPIDeviceSensor6DOF(const std::string &name,
			boost::shared_ptr<Graph::UTQLSubgraph> subgraph,
			const HAPIDeviceComponentKey &componentKey,
			HAPIDeviceModule* pModule);

	/** method to convert the raw data and send it to the port */
	void sendPose(const Measurement::Timestamp, const Math::Pose& pose);

protected:

	virtual boost::shared_ptr<HAPI::HAPIForceEffect> _createForceEffect();

	/** Pose output port of the component */
	Dataflow::PushSupplier<Measurement::Pose> m_outPort;

};



class HAPIDeviceSensorPhantom: public HAPIDeviceModuleComponent, public boost::enable_shared_from_this<HAPIDeviceSensorPhantom> {

public:
    //  Constructor
	HAPIDeviceSensorPhantom(const std::string &name,
			boost::shared_ptr<Graph::UTQLSubgraph> subgraph,
			const HAPIDeviceComponentKey &componentKey,
			HAPIDeviceModule* pModule);

	/** method to convert the raw data and send it to the port */
	void sendPose(const Measurement::Timestamp, const Math::Pose& pose);

	/** method to convert the raw data and send it to the port */
	void sendJointAngles(const Measurement::Timestamp, const Math::Vector< double, 3 >& angles);

	/** method to convert the raw data and send it to the port */
	void sendGimbalAngles(const Measurement::Timestamp, const Math::Vector< double, 3 >& angles);

protected:

	virtual boost::shared_ptr<HAPI::HAPIForceEffect> _createForceEffect();

	/** Pose output port of the component */
	Dataflow::PushSupplier<Measurement::Pose> m_outPort;

	/** Pose output port of the component */
	Dataflow::PushSupplier<Measurement::Position> m_outJointAnglesPort;

	/** Pose output port of the component */
	Dataflow::PushSupplier<Measurement::Position> m_outGimbalAnglesPort;

};


}
} // namespace Ubitrack::Drivers

#endif /* HAPIHAPTICSDEVICE_H_ */
