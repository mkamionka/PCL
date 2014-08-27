/*!
 * \file
 * \brief 
 * \author Mort
 */

#ifndef TRANSFORMCLOUD_HPP_
#define TRANSFORMCLOUD_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Logger.hpp"

#include "EventHandler2.hpp"
#include "Property.hpp"
#include "Types/HomogMatrix.hpp"
#include <pcl/visualization/pcl_visualizer.h>


namespace Processors {
namespace TransformCloud {

/*!
 * \class TranformCloud
 * \brief TranformCloud processor class.
 *
 * 
 */
class TransformCloud: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	TransformCloud(const std::string & name = "TransformCloud");

	/*!
	 * Destructor
	 */
	virtual ~TransformCloud();

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

	Base::DataStreamIn< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > in_cloud_xyzrgb;
	Base::DataStreamIn< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > in_cloud_xyzrgb_normals;

	Base::DataStreamOut< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > out_cloud_xyzrgb;
	Base::DataStreamOut< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > out_cloud_xyzrgb_normals;
	// Output data streams

	// Handlers
private:
	// Properties
	Base::Property<double> prop_x;
	Base::Property<double> prop_y;
	Base::Property<double> prop_z;
	Base::Property<double> prop_roll;
	Base::Property<double> prop_pitch;
	Base::Property<double> prop_yaw;

	Base::EventHandler2 h_transformCloud;
	// Handlers
	void transformCloud();

};

} //: namespace TransformCloud
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("TransformCloud", Processors::TransformCloud::TransformCloud)

#endif /* TRANSFORMCLOUD_HPP_ */
