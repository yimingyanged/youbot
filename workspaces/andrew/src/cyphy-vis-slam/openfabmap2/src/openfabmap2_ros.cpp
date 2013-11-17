//
//  openfabmap2_ros.cpp
//  
//
//  Wrapper by Timothy Morris on 15/04/12.
//

#include "openfabmap2_ros.h"
#include <iostream>
#include <cyphy_vslam_msgs/Match.h>
namespace enc = sensor_msgs::image_encodings;

namespace openfabmap2_ros 
{
  /////////////////////////////////
	//// *** OpenFABMap2 ROS BASE ***
	/////////////////////////////////
	//// Constructor
	// Pre:
	// Post: --Load parameters
	OpenFABMap2::OpenFABMap2(ros::NodeHandle nh) : 
	nh_(nh), it_(nh),
	firstFrame_(true), visualise_(false), working_(true), saveQuit_(false)
	{
		// TODO: finish implementing parameter server
		// Read private parameters
		ros::NodeHandle local_nh_("~");
		local_nh_.param<std::string>("vocab", vocabPath_, "vocab.yml");
		local_nh_.param<std::string>("clTree", clTreePath_, "clTree.yml");
		local_nh_.param<std::string>("trainbows", trainbowsPath_, "trainbows.yml");
		local_nh_.param<std::string>("transport", transport_, "raw");
		local_nh_.param<bool>("visualise", visualise_, false);
		local_nh_.param<int>("MinDescriptorCount", minDescriptorCount_, 50);
		
		// Read node parameters
		imgTopic_ = nh_.resolveName("image");
		
		// Initialise feature method

		//////////
		// Surf parameters that may be used for both 'detector' and 'extractor'
		int surf_hessian_threshold, surf_num_octaves, surf_num_octave_layers, surf_upright, surf_extended;
		
		local_nh_.param<int>("HessianThreshold", surf_hessian_threshold, 1000);
		local_nh_.param<int>("NumOctaves", surf_num_octaves, 4);
		local_nh_.param<int>("NumOctaveLayers", surf_num_octave_layers, 2);
		local_nh_.param<int>("Extended", surf_extended, 0);
		local_nh_.param<int>("Upright", surf_upright, 1);
		
		//////////
		//create common feature detector
		std::string detectorType;
		local_nh_.param<std::string>("DetectorType", detectorType, "FAST");
		if(detectorType == "STAR") {	
			int star_max_size, star_response, star_line_threshold, star_line_binarized, star_suppression;
			local_nh_.param<int>("MaxSize", star_max_size, 32);
			local_nh_.param<int>("Response", star_response, 10);
			local_nh_.param<int>("LineThreshold", star_line_threshold, 18);
			local_nh_.param<int>("LineBinarized", star_line_binarized, 18);
			local_nh_.param<int>("Suppression", star_suppression, 20);			
			detector = new cv::StarFeatureDetector(star_max_size,
																						 star_response, 
																						 star_line_threshold,
																						 star_line_binarized,
																						 star_suppression);
			
		} else if(detectorType == "FAST") {
			int fast_threshold, fast_non_max_suppression;
			local_nh_.param<int>("Threshold", fast_threshold, 50);
			local_nh_.param<int>("NonMaxSuppression", fast_non_max_suppression, 1);													 
			detector = new cv::FastFeatureDetector(fast_threshold,
																						 fast_non_max_suppression > 0);
			
		} else if(detectorType == "SURF") {
			detector = new cv::SURF(surf_hessian_threshold, 
																						 surf_num_octaves, 
																						 surf_num_octave_layers, 
																						 surf_extended > 0,
																						 surf_upright > 0);
			
		} else if(detectorType == "SIFT") {
			int sift_nfeatures, sift_num_octave_layers;
			double sift_threshold, sift_edge_threshold, sift_sigma;			
			local_nh_.param<int>("NumFeatures", sift_nfeatures, 0);
			local_nh_.param<int>("NumOctaveLayers", sift_num_octave_layers, 3);
			local_nh_.param<double>("Threshold", sift_threshold, 0.04);
			local_nh_.param<double>("EdgeThreshold", sift_edge_threshold, 10);
			local_nh_.param<double>("Sigma", sift_sigma, 1.6);
			detector = new cv::SIFT(sift_nfeatures,
															sift_num_octave_layers,
															sift_threshold,
															sift_edge_threshold,
															sift_sigma);
			
		} else {
			int mser_delta, mser_min_area, mser_max_area, mser_max_evolution, mser_edge_blur_size;
			double mser_max_variation, mser_min_diversity, mser_area_threshold, mser_min_margin;
			local_nh_.param<int>("Delta", mser_delta, 5);
			local_nh_.param<int>("MinArea", mser_min_area, 60);
			local_nh_.param<int>("MaxArea", mser_max_area, 14400);
			local_nh_.param<double>("MaxVariation", mser_max_variation, 0.25);
			local_nh_.param<double>("MinDiversity", mser_min_diversity, 0.2);
			local_nh_.param<int>("MaxEvolution", mser_max_evolution, 200);
			local_nh_.param<double>("AreaThreshold", mser_area_threshold, 1.01);
			local_nh_.param<double>("MinMargin", mser_min_margin, 0.003);
			local_nh_.param<int>("EdgeBlurSize", mser_edge_blur_size, 5);
			detector = new cv::MSER(mser_delta,
																						 mser_min_area,
																						 mser_max_area,
																						 mser_max_variation,
																						 mser_min_diversity,
																						 mser_max_evolution,
																						 mser_area_threshold,
																						 mser_min_margin,
																						 mser_edge_blur_size);
		}
		
		///////////
		//create common descriptor extractor
		if(detectorType == "SIFT") {
			extractor = new cv::SIFT();
		} else {
			extractor = new cv::SURF(surf_hessian_threshold, 
															 surf_num_octaves, 
																									surf_num_octave_layers,
																									surf_extended > 0,
																									surf_upright > 0);
		}
		
		matcher = new cv::FlannBasedMatcher();
		bide = new cv::BOWImgDescriptorExtractor(extractor, matcher);
	}
	
	// Destructor
	OpenFABMap2::~OpenFABMap2()
	{
	}
	
	//// Set Callback
	// Pre: --Valid 'imgTopic_' exists
	// Post: --Subscribes for Images with 'processImgCallback'
	void OpenFABMap2::subscribeToImages()
	{
		// Subscribe to images
		ROS_INFO("Subscribing to:\n\t* %s", 
						 imgTopic_.c_str());
		
		sub_ = it_.subscribe(imgTopic_, 1, &OpenFABMap2::processImgCallback,
												 this, transport_);
	}
	
	//// Running Check
	// Pre: none
	// Post: none
	bool OpenFABMap2::isWorking() const
	{
		return working_;
	}
	// end class implemtation OpenFABMap2

	//////////////////
	//// *** LEARN ***
	//////////////////
	//// Constructor
	// Pre: Valid NodeHandle provided
	// Post: --Calls 'subscribeToImages'
	FABMapLearn::FABMapLearn(ros::NodeHandle nh) : 
	OpenFABMap2(nh), trainCount_(0)
	{
		// Read private parameters
		ros::NodeHandle local_nh_("~");
		local_nh_.param<int>("maxImages", maxImages_, 10);
		local_nh_.param<double>("clusterSize", clusterSize_, 0.6);
		local_nh_.param<double>("LowerInformationBound", lowerInformationBound_, 0);
		
		trainer = of2::BOWMSCTrainer(clusterSize_);
		
		subscribeToImages();
	}
	
	//// Destructor
	FABMapLearn::~FABMapLearn()
	{
	}
	
	//// Image Callback
	// Pre: 
	// Post: --Calls 'shutdown'
	void FABMapLearn::processImgCallback(const sensor_msgs::ImageConstPtr& image_msg)
	{
		ROS_INFO_STREAM("Learning image sequence number: " << image_msg->header.seq);
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(image_msg, enc::MONO8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		ROS_DEBUG("Received %d by %d image, depth %d, channels %d", cv_ptr->image.cols,cv_ptr->image.rows, cv_ptr->image.depth(), cv_ptr->image.channels());
		
		ROS_INFO("--Detect");
		detector->detect(cv_ptr->image, kpts);
		ROS_INFO("--Extract");
		extractor->compute(cv_ptr->image, kpts, descriptors);
		
		// Check if frame was useful
		if (!descriptors.empty() && kpts.size() > minDescriptorCount_)
		{
			trainer.add(descriptors);
			trainCount_++;
			ROS_INFO_STREAM("--Added to trainer" << " (" << trainCount_ << " / " << maxImages_ << ")");
			
			// Add the frame to the sample pile
			// cv_bridge::CvImagePtr are smart pointers
			framesSampled.push_back(cv_ptr);
			
			if (visualise_)
			{
				ROS_DEBUG("Attempting to visualise key points.");
				cv::Mat feats;
				cv::drawKeypoints(cv_ptr->image, kpts, feats);
				
				cv::imshow("KeyPoints", feats);
				char c = cv::waitKey(10);
				// TODO: nicer exit
				if(c == 27) 
				{
					working_ = false;
					saveQuit_ = true;
		}
		}
		}
		else
		{
			ROS_WARN("--Image not descriptive enough, ignoring.");
		}
		
		// TODO: cv::waitKey(10) // Console triggered save&close
		if ((!(trainCount_ < maxImages_) && maxImages_ > 0) || saveQuit_)
		{
			shutdown();			
		}
	}
	
	//// Find words
	// Pre: 
	// Post:
	void FABMapLearn::findWords()
	{
		cv::Mat bow;
		
		for (std::vector<cv_bridge::CvImagePtr>::iterator frameIter = framesSampled.begin();
				 frameIter != framesSampled.end();
				 ++frameIter)
		{
			detector->detect((*frameIter)->image, kpts);
			bide->compute((*frameIter)->image, kpts, bow);
			bows.push_back(bow);
		}
	}
	
	//// File saver
	// Pre: Application has write premission to path provided
	// Post: YML files are written for 'vocab' 'clTree' and 'bows'
	void FABMapLearn::saveCodebook()
	{
		ROS_INFO("Saving codebook...");
		cv::FileStorage fs;
		
		ROS_INFO_STREAM("--Saving Vocabulary to " << vocabPath_);		
		fs.open(vocabPath_,
		  			cv::FileStorage::WRITE);
		fs << "Vocabulary" << vocab;
		fs.release();
		
		ROS_INFO_STREAM("--Saving Chow Liu Tree to " << clTreePath_);
		fs.open(clTreePath_,
		 				cv::FileStorage::WRITE);
		fs << "Tree" << clTree;
		fs.release();
		
		ROS_INFO_STREAM("--Saving Trained Bag of Words to " << trainbowsPath_);
		fs.open(trainbowsPath_,
		 				cv::FileStorage::WRITE);
		fs << "Trainbows" << bows;
		fs.release();
	}
	
	//// Unlink Callback
	// Pre:
	// Post: --Calls 'saveCodebook' --Cleanup
	void FABMapLearn::shutdown()
	{
		ROS_INFO("Clustering to produce vocabulary");
		vocab = trainer.cluster();
		ROS_INFO("Vocabulary contains %d words, %d dims",vocab.rows,vocab.cols);
		
		ROS_INFO("Setting vocabulary...");
		bide->setVocabulary(vocab);
		
		ROS_INFO("Gathering BoW's...");
		findWords();
		
		ROS_INFO("Making the Chow Liu tree...");
		tree.add(bows);
		clTree = tree.make(lowerInformationBound_);
		
		ROS_INFO("Saving work completed...");
		saveCodebook();
		
		// Flag this worker as complete
		working_ = false;
		
		if (sub_.getNumPublishers() > 0)
		{
			// Un-subscribe to Images
			ROS_WARN_STREAM("Shutting down " << sub_.getNumPublishers() << " subscriptions...");
			sub_.shutdown();
			nh_.shutdown();
		}
		else
		{
			ROS_ERROR("Shutdown called with no existing subscriptions...");
		}
	}
	// end class implementation FABMapLearn

	////////////////
	//// *** RUN ***
	////////////////
	//// Constructor
	// Pre: nh.ok() == true
	// Post: --Calls 'loadCodebook' --Calls 'subscribeToImages'
	FABMapRun::FABMapRun(ros::NodeHandle nh) : OpenFABMap2(nh)
	{
		// Load trained data
		bool goodLoad = loadCodebook();
		
		if (goodLoad)
		{
			ROS_INFO("--Codebook successfully loaded!--");
		// Read private parameters
		ros::NodeHandle local_nh_("~");
		local_nh_.param<int>("maxMatches", maxMatches_, 0);
		local_nh_.param<double>("minMatchValue", minMatchValue_, 0.0);
		local_nh_.param<bool>("DisableSelfMatch", disable_self_match_, false);
			local_nh_.param<int>("SelfMatchWindow", self_match_window_, 1);
		local_nh_.param<bool>("DisableUnknownMatch", disable_unknown_match_, false);
			local_nh_.param<bool>("AddOnlyNewPlaces", only_new_places_, false);
		
		// Setup publisher
		pub_ = nh_.advertise<cyphy_vslam_msgs::Match>("appearance_matches",1000);
		
		// Initialise for the first to contain
		// - Match to current
		// - Match to nothing
		confusionMat = cv::Mat::zeros(2,2,CV_64FC1);
		
		// Set callback
		subscribeToImages();
	}
		else
		{
			shutdown();
		}
	}
	
	//// Destructor
	FABMapRun::~FABMapRun()
	{
	}

	//// Image Callback
	// Pre: image_msg->encoding == end::MONO8
	// Post: -Matches to 'image_msg' published on pub_
	//			 -'firstFrame_' blocks initial nonsensical self match case
	void FABMapRun::processImgCallback(const sensor_msgs::ImageConstPtr& image_msg)
	{
		ROS_DEBUG_STREAM("OpenFABMap2-> Processing image sequence number: " << image_msg->header.seq);
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			// TODO: toCvShare should be used for 'FABMapRun'
			cv_ptr = cv_bridge::toCvCopy(image_msg, enc::MONO8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		ROS_DEBUG("Received %d by %d image, depth %d, channels %d", cv_ptr->image.cols,cv_ptr->image.rows, cv_ptr->image.depth(), cv_ptr->image.channels());
		
		cv::Mat bow;
		ROS_DEBUG("Detector.....");
		detector->detect(cv_ptr->image, kpts);
		ROS_DEBUG("Compute discriptors...");
		bide->compute(cv_ptr->image, kpts, bow);
		
		// Check if the frame could be described
		if (!bow.empty() && kpts.size() > minDescriptorCount_)
		{
		// IF NOT the first frame processed
		if (!firstFrame_)
		{
			ROS_DEBUG("Compare bag of words...");
			std::vector<of2::IMatch> matches;
			
				// Find match likelyhoods for this 'bow'
				fabMap->compare(bow,matches,!only_new_places_);
				
				// Sort matches with oveloaded '<' into
				// Accending 'match' order
				std::sort(matches.begin(), matches.end());
				
				// Add BOW
				if (only_new_places_)
				{
					// Check if fabMap believes this to be a new place
					if (matches.back().imgIdx == -1)
					{
						ROS_WARN_STREAM("Adding bow of new place...");
						fabMap->add(bow);
						
						// store the mapping from 'seq' to match ID
						toImgSeq.push_back(image_msg->header.seq);
					}					
				}
				else
				{
				// store the mapping from 'seq' to match ID
				toImgSeq.push_back(image_msg->header.seq);
				}
				
				// Build message
				cyphy_vslam_msgs::Match matched;
				matched.fromImgSeq = image_msg->header.seq;
				
				// IMAGE seq number
				int matchImgSeq;				
				// Prepare message in Decending match likelihood order
				for (std::vector<of2::IMatch>::reverse_iterator matchIter = matches.rbegin();
						 matchIter != matches.rend();
						 ++matchIter) 
				{
					// Limit the number of matches published (by 'maxMatches_' OR 'minMatchValue_')
					if ( (matched.toImgSeq.size() == maxMatches_ && maxMatches_ != 0)
							|| matchIter->match < minMatchValue_)
					{
						break;
					}
					
				ROS_DEBUG_STREAM("QueryIdx " << matchIter->queryIdx <<
												 " ImgIdx " << matchIter->imgIdx <<
												 " Likelihood " << matchIter->likelihood <<
												 " Match " << matchIter->match);
				
				// Lookup IMAGE seq number from MATCH seq number
				matchImgSeq = matchIter->imgIdx > -1 ? toImgSeq.at(matchIter->imgIdx) : -1;
				
					// Additionally if required, 
					// --do NOT return matches below self matches OR new places ('-1') 
					if ((matchImgSeq >= matched.fromImgSeq-self_match_window_ && disable_self_match_)
							|| (matchImgSeq == -1 && disable_unknown_match_))
					{
						break;
					}
					
				// Add the Image seq number and its match likelihood
				matched.toImgSeq.push_back(matchImgSeq);
				matched.toImgMatch.push_back(matchIter->match);
			}
			
				// IF filtered matches were found
				if (matched.toImgSeq.size() > 0)
				{
			// Publish current matches
			pub_.publish(matched);
					
					if (visualise_)
					{
						visualiseMatches(matches);
		}
				}
			}
		else
		{
			// First frame processed
				fabMap->add(bow);
				
				// store the mapping from 'seq' to match ID
				toImgSeq.push_back(image_msg->header.seq);
				
			firstFrame_ = false;
		}
	}
		else
		{
			ROS_WARN("--Image not descriptive enough, ignoring.");
		}
	}
	
	//// Visualise Matches
	// Pre:
	// Post:
	void FABMapRun::visualiseMatches(std::vector<of2::IMatch> &matches)
	{
		int numMatches = matches.size();
		
		cv::Mat newConfu = cv::Mat::zeros(numMatches,numMatches, CV_64FC1);
		ROS_DEBUG_STREAM("'newConfu -> rows: " << newConfu.rows
										<< " cols: " << newConfu.cols);
		cv::Mat roi(newConfu, cv::Rect(0,0,confusionMat.cols,confusionMat.rows));
		ROS_DEBUG_STREAM("'ROI -> rows: " << roi.rows
										<< " cols: " << roi.cols);
		confusionMat.copyTo(roi);
		
		for (std::vector<of2::IMatch>::reverse_iterator matchIter = matches.rbegin();
				 matchIter != matches.rend();
				 ++matchIter) 
		{
			// Skip null match
			if (matchIter->imgIdx == -1)
			{
				continue;
			}
			
			ROS_DEBUG_STREAM("QueryIdx " << matchIter->queryIdx <<
											 " ImgIdx " << matchIter->imgIdx <<
											 " Likelihood " << matchIter->likelihood <<
											 " Match " << matchIter->match);
			
			ROS_DEBUG_STREAM("--About to multi " << 255 << " by " << (double)matchIter->match);
			ROS_DEBUG_STREAM("---Result " << floor(255*((double)matchIter->match)));
			newConfu.at<double>(numMatches-1, matchIter->imgIdx) = 255*(double)matchIter->match;
			ROS_DEBUG_STREAM("-Uchar: " << newConfu.at<double>(numMatches-1, matchIter->imgIdx)
											<< " at (" << numMatches << ", " << matchIter->imgIdx << ")");
		}
		newConfu.at<double>(numMatches-1, numMatches-1) = 255.0;
		ROS_DEBUG_STREAM("-Value: " << newConfu.at<double>(numMatches-1,numMatches-1)
										<< " at (" << numMatches << ", " << numMatches << ")");
		
		confusionMat = newConfu.clone();
		ROS_DEBUG_STREAM("'confusionMat -> rows: " << confusionMat.rows
										<< " cols: " << confusionMat.cols);
		
		cv::imshow("Confusion Matrix", newConfu);
		cv::waitKey(10);
	}
	
	//// File loader
	// Pre:
	// Post:
	bool FABMapRun::loadCodebook()
	{
		ROS_INFO("Loading codebook...");
		
		cv::FileStorage fs;
		
		fs.open(vocabPath_,
						cv::FileStorage::READ);
		fs["Vocabulary"] >> vocab;
		fs.release();
		ROS_INFO("Vocabulary with %d words, %d dims loaded",vocab.rows,vocab.cols);
		
		fs.open(clTreePath_,
						cv::FileStorage::READ);
		fs["Tree"] >> clTree;
		fs.release();
		ROS_INFO("Chow Liu Tree loaded");
		
		fs.open(trainbowsPath_,
						cv::FileStorage::READ);
		fs["Trainbows"] >> trainbows;
		fs.release();
		ROS_INFO("Trainbows loaded");
		
		ROS_INFO("Setting the Vocabulary...");
		bide->setVocabulary(vocab);
		
		ROS_INFO("Initialising FabMap2 with Chow Liu tree...");
		
		// Get additional parameters
		ros::NodeHandle local_nh_("~");
		
		//create options flags
		std::string new_place_method, bayes_method;
		int simple_motion;
		local_nh_.param<std::string>("NewPlaceMethod", new_place_method, "Meanfield");
		local_nh_.param<std::string>("BayesMethod", bayes_method, "ChowLiu");
		local_nh_.param<int>("SimpleMotion", simple_motion, 0);
		
		int options = 0;
		if(new_place_method == "Sampled") {
			options |= of2::FabMap::SAMPLED;
		} else {
			options |= of2::FabMap::MEAN_FIELD;
		}
		if(bayes_method == "ChowLiu") {
			options |= of2::FabMap::CHOW_LIU;
		} else {
			options |= of2::FabMap::NAIVE_BAYES;
		}
		if(simple_motion) {
			options |= of2::FabMap::MOTION_MODEL;
		}
		
		//create an instance of the desired type of FabMap
		std::string fabMapVersion;
		double pzge, pzgne;
		int num_samples;
		local_nh_.param<std::string>("FabMapVersion", fabMapVersion, "FABMAPFBO");
		local_nh_.param<double>("PzGe", pzge, 0.39);
		local_nh_.param<double>("PzGne", pzgne, 0);
		local_nh_.param<int>("NumSamples", num_samples, 3000);
		
		if(fabMapVersion == "FABMAP1") {			
			fabMap = new of2::FabMap1(clTree, 
																pzge,
																pzgne,
																options,
																num_samples);
			
		} else if(fabMapVersion == "FABMAPLUT") {
			int lut_precision;
			local_nh_.param<int>("Precision", lut_precision, 6);
			fabMap = new of2::FabMapLUT(clTree, 
																	pzge,
																	pzgne,
																	options,
																	num_samples,
																	lut_precision);
			
		} else if(fabMapVersion == "FABMAPFBO") {
			double fbo_rejection_threshold, fbo_psgd;
			int fbo_bisection_start, fbo_bisection_its;
			local_nh_.param<double>("RejectionThreshold", fbo_rejection_threshold, 1e-6);
			local_nh_.param<double>("PsGd", fbo_psgd, 1e-6);
			local_nh_.param<int>("BisectionStart", fbo_bisection_start, 512);
			local_nh_.param<int>("BisectionIts", fbo_bisection_its, 9);
			fabMap = new of2::FabMapFBO(clTree, 
																	pzge,
																	pzgne,
																	options,
																	num_samples,
																	fbo_rejection_threshold,
																	fbo_psgd,
																	fbo_bisection_start,
																	fbo_bisection_its);
			
		} else if(fabMapVersion == "FABMAP2") {
			fabMap = new of2::FabMap2(clTree, 
																pzge,
																pzgne,
																options);
		} else {
			ROS_ERROR("Could not identify openFABMAPVersion from node params");
			return false;
		}
		
		ROS_INFO("Adding the trained bag of words...");
		fabMap->addTraining(trainbows);
		
		return true;
	}
	
	//// Unlink Callback
	// Pre:
	// Post: --Cleanup
	void FABMapRun::shutdown()
	{
		// Flag this worker as complete
		working_ = false;
		
		if (sub_.getNumPublishers() > 0)
		{
			ROS_WARN_STREAM("Shutting down " << sub_.getNumPublishers() << " subscriptions...");
			sub_.shutdown();
			nh_.shutdown();
		}
		else
		{
			ROS_ERROR_STREAM("-> " << sub_.getNumPublishers() << " subscriptions when shutting down..");
		}
	}
	// end class implementation FABMapRun
} // end namespace openfabmap2_ros
