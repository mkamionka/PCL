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
#include <cstdlib>
#include <ctime>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
//
#include "pcl/impl/instantiate.hpp"
#include "pcl/search/kdtree.h"
#include "pcl/search/impl/kdtree.hpp"
#include <pcl/registration/correspondence_estimation.h>
#include "pcl/registration/correspondence_rejection_sample_consensus.h"


#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#define ranf() ((float) rand() / (float) RAND_MAX)

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
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);

	// Register handlers
	h_makeNoisyCloud.setup(boost::bind(&GaussCloud::makeNoisyCloud, this));
	registerHandler("makeNoisyCloud", &h_makeNoisyCloud);

}

bool GaussCloud::onInit() {
	srand(time(0));
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

void GaussCloud::makeNoisyCloud(){
	if(!in_cloud_xyzrgb.empty()){
		pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_cloud_xyzrgb.read();
		std::vector<int> indices;
			cloud_xyzrgb->is_dense = false;
			pcl::removeNaNFromPointCloud(*cloud_xyzrgb, *cloud_xyzrgb, indices);
		CLOG(LDEBUG) << "in makeNoisyCloud() \n";
		cv::Mat_<double> wsp = cv::Mat::zeros(4,1,1);

		// Create homogenous matrix.
		cv::Mat_<double> rotationMatrix;

		for(int index=0; index< cloud_xyzrgb->size(); index++){
			float prop_x = GaussCloud::generateNumber(0.01, 0.001);
			float prop_y = GaussCloud::generateNumber(0.01, 0.001);
			float prop_z = GaussCloud::generateNumber(0.01, 0.001);

			cloud_xyzrgb->at(index).r+=GaussCloud::generateNumber(1, 10);
			cloud_xyzrgb->at(index).g+=GaussCloud::generateNumber(1, 10);
			cloud_xyzrgb->at(index).b+=GaussCloud::generateNumber(1, 10);

			// translation
			cv::Mat t = (cv::Mat_<double>(4, 4) <<
					            1, 0, 0, prop_x,
						        0, 1, 0, prop_y,
					            0, 0, 1, prop_z,
						        0, 0, 0, 1 );

			Types::HomogMatrix hm;

			for (int i = 0; i < 4; ++i) {
				for (int j = 0; j < 4; ++j) {
					hm.setElement(i, j, t.at<double>(i,j));
					CLOG(LINFO) << "hm element: " << hm.getElement(i, j);
				}
			}
					wsp(0,0)=cloud_xyzrgb->at(index).x;
					wsp(1,0)=cloud_xyzrgb->at(index).y;
					wsp(2,0)=cloud_xyzrgb->at(index).z;
					wsp(3,0)=1;

					int m=4;
					int n=4;
					int p=1;

					cv::Mat_<double> C = cv::Mat::eye(4,1,1);

					  for(int i = 0; i < m; i++)
					    for(int j = 0; j < p; j++)
					    {
					      double s = 0;
					      for(int k = 0; k < n; k++){
					    	  CLOG(LINFO) <<"wsp: "<<k<<j<<" " << wsp(k,j);
					    	  CLOG(LINFO) <<"hm.elements: " <<  hm.getElement(i, j);
					    	  s += hm.getElement(i, j) * wsp(k,j);
					    	  CLOG(LINFO) <<"result: " <<s;
					      }
					      C[i][j] = s;
					      CLOG(LINFO) << "C:: " << C[i][j];
					    }

					  cloud_xyzrgb->at(index).x = C[0][0];
					  cloud_xyzrgb->at(index).y = C[0][1];
					  cloud_xyzrgb->at(index).z = C[0][2];

				}
				out_cloud_xyzrgb.write(cloud_xyzrgb);
	}
}

float GaussCloud::generateNumber(float m, float s){
	   static int pass = 0;
	   static float y2;
	   float x1, x2, w, y1;

	   if (pass)
	   {
	      y1 = y2;
	   } else  {
	      do {
	         x1 = 2.0f * ranf () - 1.0f;
	         x2 = 2.0f * ranf () - 1.0f;
	         w = x1 * x1 + x2 * x2;
	      } while (w >= 1.0f);

	      w = (float)sqrt(-2.0 * log (w) / w);
	      y1 = x1 * w;
	      y2 = x2 * w;
	   }
	   pass = !pass;
	   CLOG(LINFO) << "number randoM is: " << (y1 * s + (float) m) ;
	   return ( (y1 * s + (float) m));
}

} //: namespace GaussCloud
} //: namespace Processors
