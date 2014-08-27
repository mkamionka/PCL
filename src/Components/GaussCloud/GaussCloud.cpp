/*!
 * \file
 * \brief
 * \author Mort
 */

#include <memory>
#include <string>

#include "GaussCloud.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace GaussCloud {

GaussCloud::GaussCloud(const std::string & name) :
		Base::Component(name)  {

}

GaussCloud::~GaussCloud() {
}

void GaussCloud::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	// Register handlers

}

bool GaussCloud::onInit() {

	return true;
}

bool GaussCloud::onFinish() {
	return true;
}

bool GaussCloud::onStop() {
	return true;
}

bool GaussCloud::onStart() {
	return true;
}



} //: namespace GaussCloud
} //: namespace Processors
