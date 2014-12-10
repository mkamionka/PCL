/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "CloudTransformer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace CloudTransformer {

CloudTransformer::CloudTransformer(const std::string & name) :
		Base::Component(name), 
		filter("filter", 0),
	    	count_xyz(0), 
		count_xyzrgb(0),
		count_xyzsift(0),
		count_xyzshot(0) {
	registerProperty(filter);
}

CloudTransformer::~CloudTransformer() {
}

void CloudTransformer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
    registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
    registerStream("in_cloud_xyzshot", &in_cloud_xyzshot);
    registerStream("in_hm", &in_hm);
    registerStream("in_eigen", &in_eigen);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
    registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
    registerStream("out_cloud_transformedsift", &out_cloud_transformedsift);
    registerStream("out_cloud_xyzshot", &out_cloud_xyzshot);
	// Register handlers
    registerHandler("transform_clouds", boost::bind(&CloudTransformer::transform_clouds, this));
    addDependency("transform_clouds", &in_hm);
    registerHandler("transform_clouds_eigen", boost::bind(&CloudTransformer::transform_clouds_eigen, this));
    addDependency("transform_clouds_eigen", &in_eigen);

}

bool CloudTransformer::onInit() {

    saved_cloud = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
	return true;
}

bool CloudTransformer::onFinish() {
	return true;
}

bool CloudTransformer::onStop() {
	return true;
}

bool CloudTransformer::onStart() {
	return true;
}

void CloudTransformer::transform_clouds() {
    CLOG(LTRACE) << "CloudTransformer::transform_clouds()";

    // Read hmomogenous matrix.
    Types::HomogMatrix hm = in_hm.read();

    // Try to transform XYZ.
    if(!in_cloud_xyz.empty())
        transform_xyz(hm);

    // Try to transform XYZRGB.
    if(!in_cloud_xyzrgb.empty())
        transform_xyzrgb(hm);

    // Try to transform XYZSIFT.
    if(!in_cloud_xyzsift.empty())
        transform_xyzsift(hm);

    // Try to transform XYZSHOT.
    if(!in_cloud_xyzshot.empty())
    	transform_xyzshot(hm);
}

void CloudTransformer::transform_clouds_eigen() {
    CLOG(LTRACE) << "CloudTransformer::transform_clouds()";

    // Read hmomogenous matrix.
    Eigen::Matrix4f eigen = in_eigen.read();
    Types::HomogMatrix hm;
    hm.setElements(eigen);

    // Try to transform XYZ.
    if(!in_cloud_xyz.empty())
        transform_xyz(hm);

    // Try to transform XYZRGB.
    if(!in_cloud_xyzrgb.empty())
        transform_xyzrgb(hm);

    // Try to transform XYZSIFT.
    if(!in_cloud_xyzsift.empty())
        transform_xyzsift(hm);

    // Try to transform XYZSHOT.
    if(!in_cloud_xyzshot.empty())
        transform_xyzshot(hm);
}


void CloudTransformer::transform_xyz(Types::HomogMatrix hm_) {
    CLOG(LTRACE) << "CloudTransformer::transform_xyz()";

    count_xyz++;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    if(!filter || filter == count_xyz)
    trans = hm_.getElements();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud, *cloud2, trans) ;
    out_cloud_xyz.write(cloud2);
}

void CloudTransformer::transform_xyzrgb(Types::HomogMatrix hm_) {
    CLOG(LTRACE) << "CloudTransformer::transform_xyzrgb()";

    count_xyzrgb++;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    if(!filter || filter == count_xyzrgb)
	trans = hm_.getElements();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*cloud, *cloud2, trans) ;
    out_cloud_xyzrgb.write(cloud2);
}

void CloudTransformer::transform_xyzsift(Types::HomogMatrix hm_) {
    CLOG(LTRACE) << "CloudTransformer::transform_xyzsift()";

    count_xyzsift++;

    pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    if(!filter || filter == count_xyzsift)
    trans = hm_.getElements();
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud2(new pcl::PointCloud<PointXYZSIFT>());
    pcl::transformPointCloud(*cloud, *cloud2, trans) ;
    if(!filter || filter >= count_xyzsift) {
        CLOG(LNOTICE) << "CloudTransformer::transform_xyzsift() saved\n";
        *saved_cloud = *cloud;
    }
    out_cloud_transformedsift.write(saved_cloud);
    out_cloud_xyzsift.write(cloud2);
}

void CloudTransformer::transform_xyzshot(Types::HomogMatrix hm_) {
    CLOG(LTRACE) << "CloudTransformer::transform_xyzshot()";

    count_xyzshot++;

    pcl::PointCloud<PointXYZSHOT>::Ptr cloud = in_cloud_xyzshot.read();
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    if(!filter || filter == count_xyzshot)
	trans = hm_.getElements();
    pcl::PointCloud<PointXYZSHOT>::Ptr cloud2(new pcl::PointCloud<PointXYZSHOT>());
    pcl::transformPointCloud(*cloud, *cloud2, trans) ;
    out_cloud_xyzshot.write(cloud2);
}




} //: namespace CloudTransformer
} //: namespace Processors
