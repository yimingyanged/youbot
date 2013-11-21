#ifndef SKELETON_MATCHER_H
#define SKELETON_MATCHER_H

#include <cyphy_vslam_msgs/Match.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "cv.h"
#include <Eigen/StdVector>

#define TXRES 16
#define TYRES 12
#define XOFFSET 1   // The range of offsets over which to match templates in x 
#define YOFFSET 1   // Range of offsets to match in y

using std::string;

typedef Eigen::Matrix<unsigned char, TYRES, TXRES> MatrixTPlate;
typedef Eigen::Matrix<double, TYRES-YOFFSET, TXRES-XOFFSET> MatrixTScore;

typedef std::pair<double, int> score_record;
bool score_comparator(const score_record &l, const score_record &r)
{ return l.first < r.first;};
class SkeletonMatcher
{
public:
    
    SkeletonMatcher(string image_topic, string match_topic, int max_templates, double match_threshold, double acceptable_match_threshold, int rstep, int max_matches, int window);
    void processImageCB(const sensor_msgs::ImageConstPtr& image_msg);
    double compareFrame(MatrixTPlate &current_template, int &xoffset, int &yoffset, int &best_match_id, std::vector<score_record> &match_scores);
protected:

    ros::NodeHandle n_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber sub_;
    ros::Publisher pub_;
private:

    string image_topic_;
    string match_topic_;
    int num_templates_;
    int max_templates_;
    int rstep_;
    double match_threshold_;
    double acceptable_match_threshold_;
    int max_matches_;
    int window_;
    std::vector<MatrixTPlate, Eigen::aligned_allocator<MatrixTPlate> > gs_templates_;
    std::vector<int> image_ids_;  // stores the incident sequence numbers which caused the template creation
    int total_frames_;
    int total_match_msgs_;
};
#endif
