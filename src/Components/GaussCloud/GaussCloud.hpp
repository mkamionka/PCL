/*!
 * \file
 * \brief 
 * \author Mort
 */

#ifndef GAUSSCLOUD_HPP_
#define GAUSSCLOUD_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>



namespace Processors {
namespace GaussCloud {

/*!
 * \class GaussCloud
 * \brief GaussCloud processor class.
 *
 * 
 */
class GaussCloud: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	GaussCloud(const std::string & name = "GaussCloud");

	/*!
	 * Destructor
	 */
	virtual ~GaussCloud();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	// Input data streams

	// Output data streams

	// Handlers

	// Properties

	
	// Handlers

};

} //: namespace GaussCloud
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("GaussCloud", Processors::GaussCloud::GaussCloud)

#endif /* GAUSSCLOUD_HPP_ */
