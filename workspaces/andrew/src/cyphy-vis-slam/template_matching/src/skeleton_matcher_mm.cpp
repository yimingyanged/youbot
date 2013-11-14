#include "skeleton_matcher.h"
#include <opencv2/highgui/highgui.hpp>
#include "cv.h"
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
using std::string;

SkeletonMatcher::SkeletonMatcher(string image_topic, string match_topic, int max_templates, double match_threshold, double acceptable_match_threshold, int rstep, int max_matches, int window) : image_topic_(image_topic), match_topic_(match_topic), max_templates_(max_templates), match_threshold_(match_threshold), acceptable_match_threshold_(acceptable_match_threshold), rstep_(rstep), max_matches_(max_matches), window_(window)
{
    it_.reset(new image_transport::ImageTransport(n_));
    image_transport::TransportHints hints("raw", ros::TransportHints(), n_);
    sub_ = it_->subscribe(image_topic, 10, &SkeletonMatcher::processImageCB, this);
    pub_ = n_.advertise<cyphy_vslam_msgs::Match>(match_topic, 1);
    gs_templates_.reserve(max_templates_);  // reserve in advance for faster execution
    image_ids_.reserve(max_templates_);
    ROS_INFO("Sequence Matcher up, incoming image is on %s and publishing to %s", image_topic_.c_str(), match_topic_.c_str());
	ROS_ERROR_STREAM("Thresholds " << match_threshold_ << " " << acceptable_match_threshold_);    
num_templates_=0;
total_frames_ = 0;
total_match_msgs_ = 0;
}

void SkeletonMatcher::processImageCB(const sensor_msgs::ImageConstPtr& image_msg)
{
    total_frames_++;
    std::vector<double> tempdiffs(num_templates_+1,0.0);

    cv::Mat SmlImageGS(TYRES, TXRES, CV_8UC1);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);   // We want a copy as we'll downsize and convert this image
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

	// Resize original image to smaller image, we know it's a MONO8 already thanks to cv_bridge
    cv::resize(cv_ptr->image, SmlImageGS, SmlImageGS.size(), 0, 0, cv::INTER_AREA);
	
    MatrixTPlate current_template;
    cv::cv2eigen(SmlImageGS, current_template); // convert to a matrix we can do math on

	// Learn a template straight away if none have been learnt yet
	if (num_templates_ == 0) {
        gs_templates_.push_back(current_template);
        image_ids_.push_back(image_msg->header.seq);
		num_templates_++;		
        ROS_INFO("Added template %d", num_templates_);
        return;
	}

	int bxo, byo; // The x and y offsets for the best matching template image, returned by compareFrame function but not currently used.

	int best_match_id = 0;

	// Go through all the templates and compare them to the current visual scene
    std::vector<score_record> match_scores(num_templates_);    // holds the scores for this frame vs all previous templates
    double mindiff=compareFrame(current_template, bxo, byo, best_match_id,match_scores);    // best score over all previous templates, low score indicates good match			

	// If none matched closely enough, learn a new template
	if (mindiff > match_threshold_) {
	    gs_templates_.push_back(current_template);
        image_ids_.push_back(image_msg->header.seq);
		num_templates_++;	
        ROS_INFO("Added template %d", num_templates_);
        return;
	}

    if(best_match_id == num_templates_-1)
        ROS_DEBUG("Best match is current template");
    else
	{
        ROS_DEBUG("Best match is template: %d out of %d templates", best_match_id, num_templates_);
        ROS_DEBUG("Mindiff: %f acceptable threshold: %f", mindiff, acceptable_match_threshold_);
	}

    if(mindiff < acceptable_match_threshold_)
    {
        cyphy_vslam_msgs::Match msg;
        msg.header.stamp = ros::Time::now();
        msg.fromImgSeq = image_msg->header.seq;

        sort(match_scores.begin(), match_scores.end(), score_comparator); // sort the whole lot
        int count = 0;
	ROS_DEBUG("Best match -> %f", match_scores[0].first);
        for(std::vector<score_record>::const_iterator it=match_scores.begin(); it!=match_scores.end(); ++it)
        {
            if(count > max_matches_)
                break;      // got enough already

            if(it->first < acceptable_match_threshold_)
            {
                if(image_msg->header.seq - image_ids_[it->second] < window_)
                    continue;       // Don't accept local matches

                msg.toImgSeq.push_back(image_ids_[it->second]);
                msg.toImgMatch.push_back(it->first);
		//ROS_DEBUG("Got Match!");
		count++;
            }
            else
                break;  // we've exhausted useful matches
        }
        if(!msg.toImgSeq.empty())
	{
	    total_match_msgs_++;
	    ROS_DEBUG_STREAM("Match ratio -> " << (double)total_match_msgs_/total_frames_ 
						<< " (" << total_match_msgs_ 
						<< ", " << total_frames_ << ")");
            pub_.publish(msg);
	}
    }
    match_scores.clear();   // be nice to the memory gods for they have bitten before
}

// Compares two frames but optionally at a range of offsets. Returns by reference the x and y offsets for the best match. bestratio not used currently.
double SkeletonMatcher::compareFrame(MatrixTPlate &current_template, int &xoffset, int &yoffset, int &best_match_id, std::vector<score_record> &match_scores) {
	int ct_i, ct_j, gt_i, gt_j, i=0; 
	double locdiff, minlocdiff, seqminlocdiff;
    int best_x, best_y;
      
    seqminlocdiff = std::numeric_limits<double>::max();
    
    for(std::vector<MatrixTPlate, Eigen::aligned_allocator<MatrixTPlate> >::iterator it=gs_templates_.begin(); it!= gs_templates_.end(); ++it, i++)
    {
        minlocdiff = std::numeric_limits<double>::max();    // reset, we use it to find the best offset within a template

        // Step through all the offsets to check images at
        for (int yo = -YOFFSET; yo <= YOFFSET; yo+=rstep_) {
            for (int xo = -XOFFSET; xo <= XOFFSET; xo+=rstep_) {			

                ct_i = (yo < 0 ) ? -yo : 0;
                gt_i = (yo < 0 ) ? 0 : yo;
                ct_j = (xo < 0 ) ? -xo : 0;
                gt_j = (xo < 0 ) ? 0 : xo;

                // Compute the score for the overlapping matrices at this offset
                MatrixTScore res = current_template.block(ct_i,ct_j,TYRES-YOFFSET,TXRES-XOFFSET).cast<double>()-it->block(gt_i,gt_j,TYRES-YOFFSET,TXRES-XOFFSET).cast<double>();
                locdiff = res.array().abs().sum();

                if (locdiff < minlocdiff) {     // offset best match?
                    minlocdiff = locdiff;
                    best_x = xo;
                    best_y = yo;
                }
            }
        }

        minlocdiff = minlocdiff / 256.0 / double( (TXRES-2*XOFFSET) * (TYRES-2*YOFFSET) );

		if (minlocdiff < seqminlocdiff) {   // global best match ?
			seqminlocdiff = minlocdiff;
			best_match_id = i;
            xoffset = best_x;
            yoffset = best_y;
		}
        score_record new_score(minlocdiff,i);
        match_scores[i]=new_score;   // record a score for every frame
    }
	return(seqminlocdiff);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "template_matcher");

    ros::NodeHandle nh("~");
    std::string image_topic, match_topic;
    int max_templates, rstep, window, max_matches;
    double match_threshold, acceptable_match_threshold;
    nh.param<string>("image_topic",image_topic,"/throttled/image_raw");
    nh.param<string>("match_topic",match_topic,"/appearance_matches");
    nh.param<int>("max_templates",max_templates,10000);
    nh.param<int>("rstep",rstep,1);     // how fast to step through xoffset/yoffset
    nh.param<double>("new_template_threshold",match_threshold,0.05);       // level at which we generate a new template
    nh.param<double>("acceptable_match_threshold",acceptable_match_threshold,0.05); // slightly more generous level at which we accept a match
    nh.param<int>("window",window,150);     // how far back in frame time to start looking
    nh.param<int>("max_matches",max_matches,10);    // how many to throw up for geometric verification

    SkeletonMatcher *sk_match = new SkeletonMatcher(image_topic, match_topic, max_templates, match_threshold, acceptable_match_threshold, rstep, max_matches, window);

    ros::spin();

    delete sk_match;

    return 0;


}
