#include "ros/ros.h"
#include <ros/callback_queue.h>

#include <image_cache/GetImage.h>
#include <image_cache/GetInfo.h>
#include <image_cache/GetNeighbours.h>

#include <cyphy_vslam_msgs/Match.h>

#include <image_geometry/stereo_camera_model.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <camera_calibration_parsers/parse.h>

#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <viso_stereo.h>

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <queue>

#include <boost/thread.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <slam_backend/AddLoopClosure.h>

#include <geometric_verification/Debug.h>

#include <cmath>

using namespace sensor_msgs;
using std::string;

// some arbitrary values (0.1m linear cov. 10deg. angular cov.)
static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =
{ { 0.1, 0, 0, 0, 0, 0,
    0, 0.1, 0, 0, 0, 0,
    0, 0, 0.1, 0, 0, 0,
    0, 0, 0, 0.17, 0, 0,
    0, 0, 0, 0, 0.17, 0,
    0, 0, 0, 0, 0, 0.17 } };

class GeometricVerifier
{
private:
    ros::NodeHandle n_;
    VisualOdometryStereo::parameters visual_odometer_params_;
    boost::mutex m_mutex_;
    std::queue<image_cache::GetImage> srv_queue_;
    ros::ServiceClient image_client_;
    ros::ServiceClient info_client_;
    ros::ServiceClient neighbour_client_;
    ros::Subscriber match_sub_;
    ros::Publisher add_loop_closure_pub_;
    ros::Publisher debug_pub_;
    boost::array<double, 36> pose_covariance_;
    tf::TransformListener tf_listener_;
    string base_link_frame_id_; 
    string sensor_frame_id_;
    int window_frame_size_;
    int m_proposed_, m_accepted_;
    double transx_, transy_, transz_;
public:
    GeometricVerifier(string base_link_frame_id, string sensor_frame_id, int window_frame_size, double transx, double transy, double transz) :
        base_link_frame_id_(base_link_frame_id),sensor_frame_id_(sensor_frame_id), window_frame_size_(window_frame_size), transx_(transx), transy_(transy), transz_(transz)
    {
        m_proposed_=0;
        m_accepted_=0;
        image_client_ = n_.serviceClient<image_cache::GetImage>("image_service");       // attach to image_cache
        info_client_ = n_.serviceClient<image_cache::GetInfo>("info_service");       // attach to image_cache
        neighbour_client_ = n_.serviceClient<image_cache::GetNeighbours>("neighbour_service");       // attach to image_cache
        match_sub_ = n_.subscribe("/appearance_matches",10,&GeometricVerifier::match_callback,this);    // subscribe to fab-map matches
        add_loop_closure_pub_ = n_.advertise<slam_backend::AddLoopClosure>("add_loop_closure",1);
        debug_pub_ = n_.advertise<geometric_verification::Debug>("verification_debug",1);
        image_cache::GetInfo left_call, right_call;
        left_call.request.name = "left";
        right_call.request.name = "right";
       
        int left_failed=0;
        int right_failed=0;
        
        bool got_left = 0;
        bool got_right = 0;
         
        while(!got_left && left_failed < 5)
        {
            if(!info_client_.call(left_call))
            {
                ROS_INFO("Can't get left camera parameters, retrying in 5 seconds");
                ros::Duration(5.0).sleep();
            }
            else
                got_left=1;
        }

        if(!got_left)
        {
            ROS_ERROR("Can't get left camera info parameters from image cache.  Is the camera up and running?");
            exit(0);
        }
        
        while(!got_right && right_failed < 5)
        {
            if(!info_client_.call(right_call))
            {
                ROS_INFO("Can't get right camera parameters, retrying");
            }
            else
                got_right = 1;
        }

        if(!got_right)
        {
            ROS_ERROR("Can't get right camera info parameters from image cache.  Is the camera up and running?");
            exit(0);
        }

        // read camera calibration from image_cache 
        image_geometry::StereoCameraModel model;
        model.fromCameraInfo(left_call.response.info_msg, right_call.response.info_msg);
        visual_odometer_params_.base = model.baseline();
        visual_odometer_params_.calib.f = model.left().fx();
        visual_odometer_params_.calib.cu = model.left().cx();
        visual_odometer_params_.calib.cv = model.left().cy();
        ROS_INFO("Camera Params are: ");
        ROS_INFO("baseline: %f", visual_odometer_params_.base);
        ROS_INFO("f: %f", visual_odometer_params_.calib.f);
        ROS_INFO("cu: %f", visual_odometer_params_.calib.cu);
        ROS_INFO("cv: %f", visual_odometer_params_.calib.cv);
        ROS_INFO("Base link frame id is %s, sensor frame id is %s, window size is %d subsampled frames",base_link_frame_id_.c_str(), sensor_frame_id_.c_str(), window_frame_size_);
        boost::thread service_thread(boost::bind(&GeometricVerifier::serviceThread,this));
        pose_covariance_.assign(0.0);

    }

    void match_callback(const cyphy_vslam_msgs::MatchConstPtr &msg)
    {
        image_cache::GetImage srv;
        srv.request.from = msg->fromImgSeq;
        srv.request.to.clear();
        for(unsigned int i=0; i < msg->toImgSeq.size(); i++)
        { 
            m_proposed_++;
            ROS_INFO("Verifying FAB-MAP match %d to %d (republished index) ...",msg->fromImgSeq,msg->toImgSeq[i]);
            if( (msg->fromImgSeq-msg->toImgSeq[i]) < window_frame_size_)
            {
                ROS_INFO("Failed, match is within local window");
                continue;
            }
            srv.request.to.push_back(msg->toImgSeq[i]);
        }
        srv.request.rectified = 1;
        boost::mutex::scoped_lock lock(m_mutex_);  // Don't touch the service when I'm adding to it
        if(!srv.request.to.empty())
            srv_queue_.push(srv);
    }
    
    void workerFunc(image_cache::GetImage &srv)
    { 
        if(image_client_.call(srv))
        {
            uint8_t *l_image_data, *r_image_data;
            int l_step, r_step;
            cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
            if(srv.response.from_l.height == 0 || srv.response.from_r.height == 0)
            {
                ROS_INFO("Database returned image with height zero, verification failed.");
                return;
            }
            if (srv.response.from_l.encoding == sensor_msgs::image_encodings::MONO8)
            {
                l_image_data = const_cast<uint8_t*>(&(srv.response.from_l.data[0]));
                l_step = srv.response.from_l.step;
            }
            else
            {
                l_cv_ptr = cv_bridge::toCvShare(srv.response.from_l, ros::VoidPtr(), sensor_msgs::image_encodings::MONO8);
                l_image_data = l_cv_ptr->image.data;
                l_step = l_cv_ptr->image.step[0];
            }
            if (srv.response.from_r.encoding == sensor_msgs::image_encodings::MONO8)
            {
                r_image_data = const_cast<uint8_t*>(&(srv.response.from_r.data[0]));
                r_step = srv.response.from_r.step;
            }
            else
            {
                r_cv_ptr = cv_bridge::toCvShare(srv.response.from_r, ros::VoidPtr(), sensor_msgs::image_encodings::MONO8);
                r_image_data = r_cv_ptr->image.data;
                r_step = r_cv_ptr->image.step[0];
            }
            ROS_ASSERT(l_step == r_step);
            ROS_ASSERT(srv.response.from_l.width == srv.response.from_r.width);
            ROS_ASSERT(srv.response.from_l.height == srv.response.from_r.height); 
         
            int32_t dims[] = {srv.response.from_l.width, srv.response.from_l.height, l_step};
            
            // Reset the visual odometer
            boost::shared_ptr<VisualOdometryStereo> visual_odometer_;
            visual_odometer_.reset(new VisualOdometryStereo(visual_odometer_params_));
            visual_odometer_->process(l_image_data, r_image_data, dims);
           
            for(int i=0; i<srv.response.to_l.size(); i++)
            {
                // Process each image pair and try to compute odometry
                uint8_t *l_image_data, *r_image_data;
                int l_step, r_step;
                cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
                if(srv.response.to_l[i].height == 0 || srv.response.to_r[i].height == 0)
                    continue;
                if (srv.response.to_l[i].encoding == sensor_msgs::image_encodings::MONO8)
                {
                    l_image_data = const_cast<uint8_t*>(&(srv.response.to_l[i].data[0]));
                    l_step = srv.response.to_l[i].step;
                }
                else
                {
                    l_cv_ptr = cv_bridge::toCvShare(srv.response.to_l[i], ros::VoidPtr(), sensor_msgs::image_encodings::MONO8);
                    l_image_data = l_cv_ptr->image.data;
                    l_step = l_cv_ptr->image.step[0];
                }
                if (srv.response.to_r[i].encoding == sensor_msgs::image_encodings::MONO8)
                {
                    r_image_data = const_cast<uint8_t*>(&(srv.response.to_r[i].data[0]));
                    r_step = srv.response.to_r[i].step;
                }
                else
                {
                    r_cv_ptr = cv_bridge::toCvShare(srv.response.to_r[i], ros::VoidPtr(), sensor_msgs::image_encodings::MONO8);
                    r_image_data = r_cv_ptr->image.data;
                    r_step = r_cv_ptr->image.step[0];
                }
                ROS_ASSERT(l_step == r_step);
                ROS_ASSERT(srv.response.to_l[i].width == srv.response.to_r[i].width);
                ROS_ASSERT(srv.response.to_l[i].height == srv.response.to_r[i].height); 
         
                int32_t dims[] = {srv.response.to_l[i].width, srv.response.to_l[i].height, l_step};
                bool replace = false;
                if(i > 0)
                    replace = true;
                // Debug message
                geometric_verification::Debug debug_msg;
                debug_msg.to = srv.response.to_id[i];
                debug_msg.from = srv.response.from_id;

                if(visual_odometer_->process(l_image_data, r_image_data, dims, replace))    // for i > 0 replace current images with the ones we've just added
                {
                    Matrix camera_motion = Matrix::inv(visual_odometer_->getMotion()); 
                    double num_matches = visual_odometer_->getNumberOfMatches();
                    double num_inliers = visual_odometer_->getNumberOfInliers();
                    std::cout << "Comparing " << srv.response.from_id << " and " << srv.response.to_id[i] << "\n";
                    std::cout << "Matches: " << num_matches << " Inliers: " << 100.0*num_inliers/num_matches << " %";

                    // Check that the pose is OK
 
                    btMatrix3x3 rot_mat(
                        camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
                        camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
                    camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
                    btVector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
                    tf::Transform delta_transform(rot_mat, t);

                    // transform pose to base frame
                    tf::StampedTransform base_to_sensor;

                    std::string error_msg;
                    if (tf_listener_.canTransform(base_link_frame_id_, sensor_frame_id_, ros::Time(0), &error_msg))
                    {
                      tf_listener_.lookupTransform(
                          base_link_frame_id_,
                          sensor_frame_id_,
                          ros::Time(0), base_to_sensor);
                    }
                    else
                    {
                      ROS_INFO("The tf from '%s' to '%s' does not seem to be available, "
                                              "will assume it as identity!",
                                              base_link_frame_id_.c_str(),
                                              sensor_frame_id_.c_str());
                                              base_to_sensor.setIdentity();
                    }

                    tf::Transform base_transform = base_to_sensor * delta_transform * base_to_sensor.inverse();
                    geometry_msgs::PoseWithCovarianceStamped transform_msg;
                    transform_msg.header.stamp = ros::Time::now();
                    transform_msg.header.frame_id = "/g2o_map";
                    tf::poseTFToMsg(base_transform, transform_msg.pose.pose);
                    tf::poseTFToMsg(base_transform, debug_msg.transform);
                    transform_msg.pose.covariance = STANDARD_POSE_COVARIANCE;

                    std::cout << "Translation is " << transform_msg.pose.pose.position.x << ", " << transform_msg.pose.pose.position.y << "\n";
                    
                    if(abs(transform_msg.pose.pose.position.z) > transz_)
                    {
                        ROS_INFO("Got a z transform > %fm, this is unlikely, validation failed.", transz_);
                        continue;
                    }

                    if(abs(transform_msg.pose.pose.position.x) > transx_)                     
                    {
                        ROS_INFO("Got an x translation of more than %fm.  Unlikely.  Validation failed.", transx_);
                        continue;
                    }
                    
                    if(abs(transform_msg.pose.pose.position.y) > transy_)                     {
                        ROS_INFO("Got a y translation of more than %fm.  Unlikely.  Validation failed.", transy_);
                        continue;
                    }

                    image_cache::GetNeighbours getNeighboursSrv;
                    getNeighboursSrv.request.nodeID = srv.response.to_id[i];  // these are the ones earlier in the map
                    if(neighbour_client_.call(getNeighboursSrv))
                    {
                        ROS_INFO("Neighbours are %d and %d, fraction %f",getNeighboursSrv.response.node1, getNeighboursSrv.response.node2, getNeighboursSrv.response.frac);
                        slam_backend::AddLoopClosure msg;
                        msg.header.stamp = ros::Time::now();
                        msg.node_id1 = getNeighboursSrv.response.node1;
                        msg.node_id2 = getNeighboursSrv.response.node2;
                        msg.interpolated_time= getNeighboursSrv.response.inter_time;
                        msg.frac = getNeighboursSrv.response.frac;
                        msg.transform = transform_msg;
                        add_loop_closure_pub_.publish(msg);
                        m_accepted_++;
                        ROS_INFO("So far accepted %d matches out of proposed %d",m_accepted_,m_proposed_);
                    }
                    else
                    {
                        ROS_INFO("Failed to get neighbours");
                    }
                }
                else
                    ROS_INFO("Failed to process ...");
                
                debug_pub_.publish(debug_msg);
            }
        }
        else
        {
                ROS_INFO("Failed to call image cache ...");
                return;
        }
            return; 
    }

    void serviceThread()
    {
        ros::NodeHandle n;
        ros::Rate loop_rate(10);
        while(n.ok())
        {
            ros::spinOnce();
            boost::mutex::scoped_lock lock(m_mutex_);  // Don't touch the service queue when I'm clearing it

            while(!srv_queue_.empty())
            {
                boost::thread workerThread(boost::bind(&GeometricVerifier::workerFunc,this,srv_queue_.front()));
                workerThread.detach();
                srv_queue_.pop();
            }
            loop_rate.sleep();
        }

    }


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "geometric_verification");
    ros::NodeHandle nh;

    string base_link_frame_id,sensor_frame_id;
    int window_frame_size;
    double transx, transy, transz;
    nh.param<string>("base_link_frame_id",base_link_frame_id, "base_link");
    nh.param<string>("sensor_frame_id",sensor_frame_id, "camera_0");
    nh.param<int>("window_size_in_frames",window_frame_size, 150);
    nh.param<double>("allowed_trans_x",transx, 10.0);
    nh.param<double>("allowed_trans_y",transy, 10.0);
    nh.param<double>("allowed_trans_z",transz, 10.0);

    GeometricVerifier *gVer = new GeometricVerifier(base_link_frame_id,sensor_frame_id,window_frame_size, transx, transy, transz);
    ros::spin();
    
    delete gVer;

    return 0;

}
