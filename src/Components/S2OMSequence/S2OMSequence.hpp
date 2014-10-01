/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef S2OMSEQUENCE_HPP_
#define S2OMSEQUENCE_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/PointCloudObject.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZSHOT.hpp>

#include <Types/S2ObjectModelFactory.hpp>

#include <vector>
#include <string>

#include <opencv2/core/core.hpp>



namespace Processors {
namespace S2OMSequence {

/*!
 * \class S2OMSequence
 * \brief S2OMSequence processor class.
 *
 * S2OMSequence processor.
 */
class S2OMSequence: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	S2OMSequence(const std::string & name = "S2OMSequence");

	/*!
	 * Destructor
	 */
	virtual ~S2OMSequence();

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

    /// Trigger - used for loading next image in case of several sequences present.
    Base::DataStreamIn<Base::UnitType> in_trigger;

	// Output data streams
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;
	Base::DataStreamOut<pcl::PointCloud<PointXYZSHOT>::Ptr> out_cloud_xyzshot;
	Base::DataStreamOut<std::string> out_name;

	Base::Property<bool> read_on_init;

    /*!
     * Event handler function - moves image index to the next frame of the sequence.
     */
    void onLoadNextImage();


    /*!
     * Event handler function - moves image index to the next frame of the sequence, externally triggered version.
     */
    void onTriggeredLoadNextImage();


    /*!
	 * Event handler function - loads image from the sequence.
	 */
	void onLoadImage();

	/*!
	 * Event handler function - reload the sequence.
	 */
	void onSequenceReload();

private:
	/**
	 * Fill list of files according to pattern
	 *
	 * \return true, if there is at least one file found, false otherwise
	 */
	bool findFiles();
	void readModel(std::string filename);

	/// list of files in sequence
	std::vector<std::string> files;

	/// Index of current frame.
	int frame;

    /// Flag indicating whether the next image should loaded or not.
	bool next_image_flag;

    /// Flag indicating whether the sequence should be reloaded or not.
	bool reload_flag;


	/// Directory containing the images sequence.
	Base::Property<std::string> prop_directory;

	/// Next image loading mode: iterative vs triggered.
	Base::Property<bool> prop_auto_trigger;

	/// Loading mode: images loaded in the loop.
	Base::Property<bool> prop_loop;

	/// Sort image sequence by their names.
	Base::Property<bool> prop_sort;
};

} //: namespace S2OMSequence
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("S2OMSequence", Processors::S2OMSequence::S2OMSequence)

#endif /* S2OMSEQUENCE_HPP_ */
