//
//  openfabmap2_ros.h
//  
//
//  Wrapper by Timothy Morris on 15/04/12.
//

#ifndef _openfabmap2_ros_h
#define _openfabmap2_ros_h

#include "openfabmap.hpp"
#include <cyphy_vslam_msgs/Match.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

namespace openfabmap2_ros 
{	
  class OpenFABMap2
	{
	public:
		OpenFABMap2(ros::NodeHandle nh);
		
		virtual ~OpenFABMap2();
		
		void subscribeToImages();
		bool isWorking() const;
		
		virtual void shutdown() = 0;
		virtual void processImgCallback(const sensor_msgs::ImageConstPtr& image_msg) = 0;
		
	protected:
		ros::NodeHandle nh_;
		
		// Image transport
		image_transport::Subscriber sub_;
		
		// OpenFABMap2
		of2::FabMap *fabMap;
		cv::Ptr<cv::FeatureDetector> detector;
		cv::Ptr<cv::DescriptorExtractor>  extractor;
		cv::Ptr<cv::DescriptorMatcher> matcher;
		cv::Ptr<cv::BOWImgDescriptorExtractor> bide;
		std::vector<cv::KeyPoint> kpts;
		
		bool firstFrame_;
		bool visualise_;
		bool working_;
		bool saveQuit_;
		std::string vocabPath_;
		std::string clTreePath_;
		std::string trainbowsPath_;	
		int minDescriptorCount_;
		
		// Data
		cv::Mat vocab;
		cv::Mat clTree;
		cv::Mat trainbows;
		
	private:	
		image_transport::ImageTransport it_;
		
		std::string imgTopic_;
		std::string transport_;
	};
	
	//// Running OpenFABMap2
	class FABMapRun : public OpenFABMap2
	{
	public:
		FABMapRun(ros::NodeHandle nh);
		~FABMapRun();
		
		void processImgCallback(const sensor_msgs::ImageConstPtr& image_msg);
		void visualiseMatches(std::vector<of2::IMatch> &matches);
		bool loadCodebook();
		void shutdown();
		
	private:
		int maxMatches_;
		double minMatchValue_;
		bool disable_self_match_;
		int self_match_window_;
		bool disable_unknown_match_;
		bool only_new_places_;
		
		ros::Publisher pub_;
		std::vector<int> toImgSeq;
		cv::Mat confusionMat;
	};
	
	//// Learning OpenFABMap2
	class FABMapLearn : public OpenFABMap2
	{
	public:
		FABMapLearn(ros::NodeHandle nh);
		~FABMapLearn();
		
		void processImgCallback(const sensor_msgs::ImageConstPtr& image_msg);
		void findWords();
		void saveCodebook();
		void shutdown();
		
	private:
		int trainCount_;
		int maxImages_;
		double clusterSize_;
		double lowerInformationBound_;
		
		std::vector<cv_bridge::CvImagePtr> framesSampled;
		
		cv::Mat descriptors;
		cv::Mat bows;
		of2::BOWMSCTrainer trainer;
		of2::ChowLiuTree tree;
	};

}

#endif
