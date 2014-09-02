/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "S2ObjectMatcher.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Types/Features.hpp"
#include <Types/MergeUtils.hpp>

namespace Processors {
namespace S2ObjectMatcher {

class SHOTonlyXYZRepresentation: public pcl::DefaultFeatureRepresentation<PointXYZSHOT> {
	/// Templatiated number of SIFT descriptor dimensions.
	using pcl::PointRepresentation<PointXYZSHOT>::nr_dimensions_;

public:
	SHOTonlyXYZRepresentation() {
		// Define the number of dimensions.
		nr_dimensions_ = 3;
		trivial_ = false;
	}

	/// Overrides the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PointXYZSHOT &p, float * out) const {
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
	}
};

class SHOTonlyDescriptorRepresentation: public pcl::DefaultFeatureRepresentation<PointXYZSHOT> {
	/// Templatiated number of SIFT descriptor dimensions.
	using pcl::PointRepresentation<PointXYZSHOT>::nr_dimensions_;

public:
	SHOTonlyDescriptorRepresentation() {
		// Define the number of dimensions.
		nr_dimensions_ = 352;
		trivial_ = false;
	}

	/// Overrides the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PointXYZSHOT &p, float * out) const {
		for (int i = 0; i < 352; ++i) {
			out[i] = p.descriptor[i];
		}
	}
};

void computeCorrespondences(const pcl::PointCloud<PointXYZSHOT>::ConstPtr &cloud_src,
		const pcl::PointCloud<PointXYZSHOT>::ConstPtr &cloud_trg, const pcl::CorrespondencesPtr& correspondences) {
	pcl::registration::CorrespondenceEstimation<PointXYZSHOT, PointXYZSHOT> correst;
	SHOTonlyDescriptorRepresentation::Ptr point_representation(new SHOTonlyDescriptorRepresentation());
	correst.setPointRepresentation(point_representation);
	correst.setInputSource(cloud_src);
	correst.setInputTarget(cloud_trg);
	// Find correspondences.
	correst.determineReciprocalCorrespondences(*correspondences);

}

S2ObjectMatcher::S2ObjectMatcher(const std::string & name) :
		Base::Component(name) {

}

S2ObjectMatcher::~S2ObjectMatcher() {
}

void S2ObjectMatcher::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_models", &in_models);
	registerStream("in_cloud_xyzshot", &in_cloud_xyzshot);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_source_keypoints", &out_source_keypoints);
	registerStream("out_source_keypoints", &out_source_keypoints);
	registerStream("out_source", &out_source);
	registerStream("out_target", &out_target);
	registerStream("out_correspondences", &out_correspondences);
	// Register handlers
	h_readModels.setup(boost::bind(&S2ObjectMatcher::readModels, this));
	registerHandler("readModels", &h_readModels);
	addDependency("readModels", &in_models);

	h_matchShots.setup(boost::bind(&S2ObjectMatcher::matchShots, this));
	registerHandler("h_matchShots", &h_matchShots);
	addDependency("h_matchShots", &in_cloud_xyzshot);
	addDependency("h_matchShots", &in_cloud_xyzrgb);

	h_matchSifts.setup(boost::bind(&S2ObjectMatcher::matchSifts, this));
	registerHandler("h_matchSitfs", &h_matchSifts);
	addDependency("h_matchSitfs", &in_cloud_xyzsift);
	addDependency("h_matchSitfs", &in_cloud_xyzrgb);
}

bool S2ObjectMatcher::onInit() {

	return true;
}

bool S2ObjectMatcher::onFinish() {
	return true;
}

bool S2ObjectMatcher::onStop() {
	return true;
}

bool S2ObjectMatcher::onStart() {
	return true;
}

void S2ObjectMatcher::readModels() {
	CLOG(LNOTICE)<<"SIFTObjectMatcher::readModels()";
	for( int i = 0; i<models.size(); i++) {
		delete models[i];
	}
	models.clear();
	std::vector<AbstractObject*> abstractObjects = in_models.read();
	for( int i = 0; i<abstractObjects.size(); i++) {
		cout<<"Name: "<<abstractObjects[i]->name<<endl;
		S2ObjectModel *model = dynamic_cast<S2ObjectModel*>(abstractObjects[i]);
		if(model!=NULL)
		models.push_back(model);
		else
		cout<<"niepoprawny model"<<endl;
	}
	cout<<models.size()<<" modeli"<<endl;
}

void S2ObjectMatcher::matchSifts() {
	CLOG(LNOTICE)<< "SIFTObjectMatcher::matchSifts()";
	if(models.empty()) {
		cout<<"No models available" <<endl;
		return;
	}

	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift = in_cloud_xyzsift.read();
//	pcl::PointCloud<PointXYZSHOT>::Ptr cloud_xyzshot = in_cloud_xyzshot.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_cloud_xyzrgb.read();

	CLOG(LTRACE) << "liczba cech instancji (SIFT): " << cloud_xyzsift->size();
//	CLOG(LTRACE) << "liczba cech instancji (SHOT): " << cloud_xyzshot->size();

	for (int i = 0; i < models.size(); ++i) {

		CLOG(LTRACE) << "liczba cech modelu (SIFT)"<<i<<" "<<models[i]->name<<": " << models[i]->cloud_xyzsift->size();
		//	CLOG(LTRACE) << "liczba cech modelu (SHOT)"<<i<<" "<<models[i]->name<<": " << models[i]->cloud_xyzshot->size();

		pcl::CorrespondencesPtr correspondences_sift(new pcl::Correspondences());
		MergeUtils::computeCorrespondences(cloud_xyzsift, models[i]->cloud_xyzsift, correspondences_sift);

		//	pcl::CorrespondencesPtr correspondences_shot(new pcl::Correspondences());
		//	computeCorrespondences(in_cloud_xyzshot, models[i]->cloud_xyzsift, correspondences_shot);

		CLOG(LERROR) << "MODEL " << i << "\n\tSIFT: " << correspondences_sift->size();
		//	<< "\n\tSHOT: " << correspondences_shot->size();

	}
}

void S2ObjectMatcher::matchShots() {
	CLOG(LNOTICE)<< "SIFTObjectMatcher::matchShots()";
	if(models.empty()) {
		cout<<"No models available" <<endl;
		return;
	}

//	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift = in_cloud_xyzsift.read();
	pcl::PointCloud<PointXYZSHOT>::Ptr cloud_xyzshot = in_cloud_xyzshot.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_cloud_xyzrgb.read();

//	CLOG(LTRACE) << "liczba cech instancji (SIFT): " << cloud_xyzsift->size();
	CLOG(LTRACE) << "liczba cech instancji (SHOT): " << cloud_xyzshot->size();

	for (int i = 0; i < models.size(); ++i) {

		//	CLOG(LTRACE) << "liczba cech modelu (SIFT)"<<i<<" "<<models[i]->name<<": " << models[i]->cloud_xyzsift->size();
		CLOG(LTRACE) << "liczba cech modelu (SHOT)"<<i<<" "<<models[i]->name<<": " << models[i]->cloud_xyzshot->size();

		//	pcl::CorrespondencesPtr correspondences_sift(new pcl::Correspondences());
		//	MergeUtils::computeCorrespondences(cloud_xyzsift, models[i]->cloud_xyzsift, correspondences_sift);

		pcl::CorrespondencesPtr correspondences_shot(new pcl::Correspondences());
		computeCorrespondences(cloud_xyzshot, models[i]->cloud_xyzshot, correspondences_shot);

		CLOG(LERROR) << "MODEL " << i << "\n\tSHOT: " << correspondences_shot->size();
		//	<< "\n\tSHOT: " << correspondences_shot->size();

	}
}

/*
 * TODO printowanie

 /////////////////////////////
 out_cloud_xyzrgb.write(cloud_xyzrgb);
 out_cloud_xyzrgb_model.write(models[i]->cloud_xyzrgb);
 out_cloud_xyzsift.write(cloud_xyzsift);
 out_cloud_xyzsift_model.write(models[i]->cloud_xyzsift);
 out_correspondences.write(correspondences);//wszystkie dopasowania
 */
//TODO dopasowania poprawne

} //: namespace S2ObjectMatcher
} //: namespace Processors
