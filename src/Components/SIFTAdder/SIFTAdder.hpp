/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef SIFTADDER_HPP_
#define SIFTADDER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <Types/PointXYZSIFT.hpp> 
//#include "Types/Features.hpp"

namespace Processors {
namespace SIFTAdder {

/*!
 * \class SIFTAdder
 * \brief SIFTAdder processor class.
 *
 * SIFTAdder processor.
 */
class SIFTAdder: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SIFTAdder(const std::string & name = "SIFTAdder");

	/*!
	 * Destructor
	 */
	virtual ~SIFTAdder();

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
	
	bool matIsEqual(const cv::Mat mat1, const cv::Mat mat2);


// Input data streams

		Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud;

// Output data streams
		Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud;
		//Base::DataStreamOut<vector<vector<int> > > out_descriptors;
		//Base::DataStreamOut<vector<int> > out_times;
	// Handlers
	Base::EventHandler2 h_add;

	
	// Handlers
	void add();
	
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud;
	
	//vector<vector<int> > descriptors;
	//vector<int> times;

};

} //: namespace SIFTAdder
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SIFTAdder", Processors::SIFTAdder::SIFTAdder)

#endif /* SIFTADDER_HPP_ */
