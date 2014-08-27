/*!
 * \file
 * \brief
 * \author Mort
 */

#include <memory>
#include <string>

#include "TransformCloud.hpp"
#include "Component_Aux.hpp"
#include "Component.hpp"
#include "Common/Logger.hpp"
#include "EventHandler.hpp"
#include "DataStream.hpp"
#include "Types/Objects3D/Object3D.hpp"
#include "Types/HomogMatrix.hpp"
#include "Types/CameraInfo.hpp"
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/bind.hpp>
#include "Property.hpp"
#include <math.h>

namespace Processors {
namespace TransformCloud {


TransformCloud::TransformCloud(const std::string & name) :
		Base::Component(name),
		prop_x("offset.x", 0),
		prop_y("offset.y", 0),
		prop_z("offset.z", 0),
		prop_roll("offset.roll", 0),
		prop_pitch("offset.pitch", 0),
		prop_yaw("offset.yaw", 0)
		{
			registerProperty(prop_x);
			registerProperty(prop_y);
			registerProperty(prop_z);
			registerProperty(prop_roll);
			registerProperty(prop_pitch);
			registerProperty(prop_yaw);
}

TransformCloud::~TransformCloud() {
}

void TransformCloud::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzrgb_normals", &in_cloud_xyzrgb_normals);

	// Register handlers
	h_transformCloud.setup(boost::bind(&TransformCloud::transformCloud, this));
	registerHandler("transformCloud", &h_transformCloud);
	//addDependency("transformCloud", &in_cloud_xyzrgb);

}

bool TransformCloud::onInit() {

	return true;
}

bool TransformCloud::onFinish() {
	return true;
}

bool TransformCloud::onStop() {
	return true;
}

bool TransformCloud::onStart() {
	return true;
}

void TransformCloud::transformCloud(){

	if(!in_cloud_xyzrgb.empty()){

		Types::HomogMatrix homogMatrix;
		cv::Mat_<double> wsp(1,4);

		CLOG(LDEBUG) << "in transformCloud() \n";
		cv::Mat_<double> rvec;
		cv::Mat_<double> tvec;
		cv::Mat_<double> outputMatrix(4,4);

		// Create homogenous matrix.
		cv::Mat_<double> rotationMatrix;

		// Roll - rotation around the X (blue) axis.
		cv::Mat roll = (cv::Mat_<double>(4, 4) <<
		              1,          0,           0, 0,
		              0, cos(prop_roll), -sin(prop_roll), 0,
		              0, sin(prop_roll),  cos(prop_roll), 0,
		              0, 0, 0, 1 );


		// Pitch - rotation around the Y (green) axis.
		cv::Mat pitch = (cv::Mat_<double>(4, 4) <<
	            cos(prop_pitch), 0, sin(prop_pitch), 0,
	            0, 1, 0, 0,
		        -sin(prop_pitch),  0,	cos(prop_pitch), 0,
		        0, 0, 0, 1 );

		// Yaw - rotation around the Z (red) axis.
		cv::Mat yaw = (cv::Mat_<double>(4, 4) <<
	            cos(prop_yaw), -sin(prop_yaw), 0, 0,
		        sin(prop_yaw),  cos(prop_yaw), 0, 0,
	            0, 0, 1, 0,
		        0, 0, 0, 1 );

		// translation
		cv::Mat t = (cv::Mat_<double>(4, 4) <<
				            0, 0, 0, prop_x,
					        0, 0, 0, prop_y,
				            0, 0, 0, prop_z,
					        0, 0, 0, 0 );

		outputMatrix = t + yaw * pitch * roll;

		Types::HomogMatrix hm;

		stringstream ss;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				hm.elements[i][j] = outputMatrix.at<double>(i,j);
				ss << hm.elements[i][j] << "  ";
			}
		}
		CLOG(LDEBUG) << "HomogMatrix:\n" << ss.str() << endl;

		pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_cloud_xyzrgb.read();

		for(int index=0; index< cloud_xyzrgb->size(); index++){

			wsp(0,0)=cloud_xyzrgb->at(index).x;
			wsp(0,1)=cloud_xyzrgb->at(index).y;
			wsp(0,2)=cloud_xyzrgb->at(index).z;
			wsp(0,3)=0;

			int m=1;
			int n=4;
			int p=4;
			cv::Mat_<double> C= cv::Mat::eye(1,4,1);

			  for(int i = 0; i < m; i++)
			    for(int j = 0; j < p; j++)
			    {
			      double s = 0;
			      for(int k = 0; k < n; k++)
			    	  s += wsp(i,k) * hm.elements[k][j];
			      C[i][j] = s;
			    }

			//	CLOG(LINFO) << "przed: "<< cloud_xyzrgb->at(index).x <<"\n";
			//	CLOG(LINFO) << "przed: "<< cloud_xyzrgb->at(index).y <<"\n";
			//	CLOG(LINFO) << "przed: "<< cloud_xyzrgb->at(index).z <<"\n";


			  cloud_xyzrgb->at(index).x = C[0][0];
			  cloud_xyzrgb->at(index).y = C[0][1];
			  cloud_xyzrgb->at(index).z = C[0][2];

			//CLOG(LINFO) << "po: "<< C[0][0] <<"\n";
			//CLOG(LINFO) << "po: "<< C[0][1] <<"\n";
			//CLOG(LINFO) << "po: "<< C[0][2] <<"\n";

		}
		out_cloud_xyzrgb.write(cloud_xyzrgb);
	}
}


} //: namespace TransformCloud
} //: namespace Processors
