#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/publisher.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <sensor_msgs/image_encodings.h>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv/highgui.h>

#include <ros/callback_queue.h>

#include <image_cache/GetImage.h>
#include <image_cache/GetInfo.h>
#include <image_cache/GetNeighbours.h>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>

#include <slam_backend/NodeAdded.h>

#include <image_geometry/pinhole_camera_model.h>


using std::string;
using namespace boost::filesystem;
using namespace sensor_msgs;
using namespace message_filters::sync_policies;
using namespace boost::multi_index;

namespace enc = sensor_msgs::image_encodings;

typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

struct IDDatabase{
       IDDatabase(ros::Time ts, int original, int republished) : ts_(ts), original_(original), republished_(republished) {}
       ros::Time ts_;
       int original_;
       int republished_;
};

struct NodeDatabase{
       NodeDatabase(ros::Time ts, int NodeID) : ts_(ts), id_(NodeID) {}
       ros::Time ts_;
       int id_;
};


struct time_stamp{};  // tag for an index
struct original{};  // tag for an index
struct republished{};  // tag for an index
struct id{};

typedef boost::multi_index_container< 
        IDDatabase,
        indexed_by <
            ordered_unique<tag<time_stamp>, member<IDDatabase, ros::Time, &IDDatabase::ts_> >,
            ordered_unique<tag<original>, member<IDDatabase, int, &IDDatabase::original_> >,
            ordered_unique<tag<republished>, member<IDDatabase, int, &IDDatabase::republished_> >
        >
    > MsgDB;

typedef boost::multi_index_container<
        NodeDatabase,
        indexed_by <
            ordered_unique<tag<time_stamp>, member<NodeDatabase, ros::Time, &NodeDatabase::ts_> >,
            ordered_unique<tag<id>, member<NodeDatabase, int, &NodeDatabase::id_> >
            >
        > NodeDB;

typedef MsgDB::index<time_stamp>::type TimeIndex;
typedef MsgDB::index<original>::type OriginalIndex;
typedef MsgDB::index<republished>::type RepublishedIndex;
typedef TimeIndex::iterator TimeIterator;
typedef OriginalIndex::iterator OriginalIterator;
typedef RepublishedIndex::iterator RepublishedIterator;

typedef NodeDB::index<time_stamp>::type NodeTimeIndex;
typedef NodeDB::index<id>::type NodeIDIndex;
typedef NodeTimeIndex::iterator NodeTimeIterator;
typedef NodeIDIndex::iterator NodeIDIterator;


class CacheServer
{
private:
    ros::NodeHandle n_;
    ros::Subscriber add_node_sub_;
    ros::ServiceServer image_service_;
    ros::ServiceServer info_service_;
    ros::ServiceServer neighbour_service_;
    ros::CallbackQueue service_queue_;      // separate callback queue for the service (better performance?)
    path cache_path_;
    ros::AdvertiseServiceOptions image_ops_;
    ros::AdvertiseServiceOptions info_ops_;
    ros::AdvertiseServiceOptions neighbour_ops_;
    boost::format l_format_, r_format_, info_format_;

    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<ExactSync> exact_sync_;
    image_transport::Publisher left_image_pub_;  // at this stage only rebroadcast the throttled left image
    image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
    message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
    string image_topic_, left_calib_file_,right_calib_file_;
    bool received_left_cam_info_, received_right_cam_info_;
    CameraInfo left_cam_info_;
    int msg_count_, msg_div_;

    boost::mutex m_mutex_;

    MsgDB database;
    NodeDB node_database_;

    int pub_count_;
    image_geometry::PinholeCameraModel left_model_, right_model_;
    bool use_original_;

public:
    CacheServer(string image_topic, string out_topic, int div, string cache_path, bool use_original)
    {
        it_.reset(new image_transport::ImageTransport(n_));
        image_transport::TransportHints hints("raw", ros::TransportHints(), n_);
        sub_l_image_.subscribe(*it_, image_topic + string("/left/image_raw"), 100, hints);
        sub_r_image_.subscribe(*it_, image_topic + string("/right/image_raw"), 100, hints);
        sub_l_info_.subscribe(n_, image_topic + string("/left/camera_info"), 100);
        sub_r_info_.subscribe(n_, image_topic + string("/right/camera_info"), 100);
        exact_sync_.reset(new ExactSync(ExactPolicy(10), sub_l_image_, sub_l_info_, sub_r_image_, sub_r_info_));
        add_node_sub_ = n_.subscribe("node_added",10, &CacheServer::addNodeCB, this);
        image_topic_ = image_topic;
        msg_div_ = div;
        msg_count_=0;
        pub_count_=0;
        received_left_cam_info_ = 0;
        received_right_cam_info_ = 0;
        cache_path_=cache_path;
        info_format_.parse("%s/%s.ini");
        use_original_=use_original;
        
        if(!is_directory(cache_path_))
        {
            cache_path_  = current_path();  
            ROS_INFO("Supplied path is not a directory, cache will be stored locally in %s", cache_path_.c_str());
            left_calib_file_ = (info_format_ % cache_path_.c_str() % "left").str();
            right_calib_file_ = (info_format_ % cache_path_.c_str() % "right").str();
        }
        else
        {
            ROS_INFO("Will be writing cache to %s", cache_path_.c_str());
            left_calib_file_ = (info_format_ % cache_path_.c_str() % "left" ).str();
            right_calib_file_ = (info_format_ % cache_path_.c_str() % "right").str();
        }


       image_ops_ = ros::AdvertiseServiceOptions::create<image_cache::GetImage>("image_service", boost::bind(&CacheServer::getImageCB, this, _1, _2), ros::VoidPtr(), &service_queue_);
       info_ops_ = ros::AdvertiseServiceOptions::create<image_cache::GetInfo>("info_service", boost::bind(&CacheServer::getInfoCB, this, _1, _2), ros::VoidPtr(), &service_queue_);
       neighbour_ops_ = ros::AdvertiseServiceOptions::create<image_cache::GetNeighbours>("neighbour_service", boost::bind(&CacheServer::getNeighbourCB, this, _1, _2), ros::VoidPtr(), &service_queue_);
       image_service_ = n_.advertiseService(image_ops_);
       info_service_ = n_.advertiseService(info_ops_);
       neighbour_service_ = n_.advertiseService(neighbour_ops_);

       boost::thread service_thread(boost::bind(&CacheServer::serviceThread,this));
   
       l_format_.parse("%s/left%05i.%s"); 
       r_format_.parse("%s/right%05i.%s"); 

       exact_sync_->registerCallback(boost::bind(&CacheServer::stereo_cb, this, _1, _2, _3, _4));

       // broadcast the throttled left image out on out_topic
       left_image_pub_ = it_->advertise(out_topic + string("/image_raw"), 10);
       
       ROS_INFO("Cache Server is up........... "); 

       if(use_original_)
           ROS_INFO("Using original image IDs from matcher");

    }
	
    void addNodeCB(const slam_backend::NodeAddedPtr &msg)
    {
        node_database_.insert(NodeDatabase(msg->header.stamp,msg->node_id));
    }

    void stereo_cb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg, const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg)
    {
        msg_count_++;
        if(!(msg_count_%msg_div_))
        {
            if(!received_left_cam_info_)
            {
                left_cam_info_ = *l_info_msg;
                received_left_cam_info_ = 1;
                camera_calibration_parsers::writeCalibration(left_calib_file_, l_info_msg->header.frame_id, *l_info_msg);  // should be params
                left_model_.fromCameraInfo(l_info_msg);

                ROS_INFO("Got left_cam_info for %s", image_topic_.c_str());
            }

            if(!received_right_cam_info_)
            {
                camera_calibration_parsers::writeCalibration(right_calib_file_, r_info_msg->header.frame_id, *r_info_msg); 
                right_model_.fromCameraInfo(r_info_msg);
                received_right_cam_info_ = 1;
            }
            // Save left and right
            assert(l_image_msg->encoding == enc::MONO8);
            assert(r_image_msg->encoding == enc::MONO8);

            // copied from image_view save_image
            cv_bridge::CvImagePtr image_l;
            try
            {
							image_l = cv_bridge::toCvCopy(l_image_msg, enc::BGR8);
						}
					catch (cv_bridge::Exception& e)
					{
							ROS_ERROR("Unable to convert %s image to bgr8: %s", l_image_msg->encoding.c_str(), e.what());
					}
                
					if (image_l)
					{
							std::string filename = (l_format_ % cache_path_.c_str() % l_image_msg->header.seq % "png").str();
							cv::imwrite(filename.c_str(), image_l->image);
					}
					else
					{
							ROS_WARN("Couldn't save image, no data!");
					}

					// copied from image_view save_image
					cv_bridge::CvImagePtr image_r;
					try
					{
							image_r = cv_bridge::toCvCopy(r_image_msg, enc::BGR8);
					}
					catch (cv_bridge::Exception& e)
					{
							ROS_ERROR("Unable to convert %s image to bgr8: %s", r_image_msg->encoding.c_str(), e.what());
					}

					if (image_r)
					{
							std::string filename = (r_format_ % cache_path_.c_str() % l_image_msg->header.seq % "png").str();
							cv::imwrite(filename.c_str(), image_r->image);
					}
					else
					{
							ROS_WARN("Couldn't save image, no data!");
					}


					// Republish (LEFT IMAGE ONLY)
					left_image_pub_.publish(l_image_msg);
					pub_count_++;
					database.insert(IDDatabase(l_image_msg->header.stamp,l_image_msg->header.seq,pub_count_));
        }
    }

    bool getInfoCB(image_cache::GetInfo::Request &req, image_cache::GetInfo::Response &res)
    {
        string fname = (info_format_ % cache_path_.c_str() % req.name).str();
        string camera;
        sensor_msgs::CameraInfo cam_info;
	
        if(camera_calibration_parsers::readCalibration(fname, camera, cam_info))
	{
        	res.info_msg = cam_info;
	        return true;
	}
	else 
	{
		ROS_ERROR("Calibration files are not available");
		return false;
	}
    }

    bool getNeighbourCB(image_cache::GetNeighbours::Request &req, image_cache::GetNeighbours::Response &res)
    {
        OriginalIterator it = database.get<original>().find(req.nodeID);
        if(it == database.get<original>().end())
        {
            OriginalIndex& orig_ind = database.get<original>();
            it = orig_ind.upper_bound(req.nodeID);
            if(it == database.get<original>().end())
                return false;
            else
                ROS_INFO("Couldn't find neighbour of %d, using %d instead",req.nodeID, it->original_);
        }
        
        ros::Time ts = it->ts_;
      
        boost::mutex::scoped_lock lock(m_mutex_);  // stop changing the node map!
        NodeTimeIndex& node_time_ind = node_database_.get<time_stamp>();
      
        NodeTimeIterator itup = node_time_ind.upper_bound(ts); // first element in db greater than time stamp
         
        if(itup == node_database_.get<time_stamp>().end()) // at the end
        {
            res.node1 = (--itup)->id_; 
            res.node2=-1;
            res.frac=-1;
            return true;
        }
        if(itup == node_database_.get<time_stamp>().begin()) // at the end
        {
            res.node1 = -1; 
            res.node2=(++itup)->id_;
            res.frac=-1;
            return true;
        }
        NodeTimeIterator itlow = itup;
        itlow--;
        if(itlow == node_database_.get<time_stamp>().begin())  // at or before the beginning
        {
            res.node1 = -1;
            res.node2= itup->id_;
            res.frac = -1;
            return true;
        }
        
        res.node1 = itlow->id_; 
        res.node2 = itup->id_; 

        res.frac = (ts - itlow->ts_).toSec()/(itup->ts_ - itlow->ts_).toSec();
        double time_diff = res.frac*(itup->ts_-itlow->ts_).toSec();
        double starttime = itlow->ts_.toSec();
        ros::Time inter(starttime+time_diff);
        res.inter_time = inter;
        return true;

    }

    bool getImageCB(image_cache::GetImage::Request &req, image_cache::GetImage::Response &res)
    {
        if(use_original_)
            getImageOriginal(req, res);
        else
            getImageRepublished(req, res);
    }

    bool getImageOriginal(image_cache::GetImage::Request &req, image_cache::GetImage::Response &res)
    {
        ROS_DEBUG("Requested from image is %d", req.from);
        OriginalIterator it = database.get<original>().find(req.from);

        if(it == database.get<original>().end())
        {
            OriginalIndex& original_ind = database.get<original>();
            it = original_ind.upper_bound(req.from);
            if(it == database.get<original>().end())
                it = original_ind.lower_bound(req.from);
            if(it == database.get<original>().begin())
                return false;
            else
                ROS_DEBUG("Couldn't find it, using %d instead",it->original_);
        }        
        string fname_l = (l_format_ % cache_path_.c_str() % it->original_ % "png").str();
        cv::Mat l_image_raw = cv::imread(fname_l);

        cv_bridge::CvImage img_msg_l;
        cv_bridge::CvImage img_msg_r;
        
        sensor_msgs::Image left_msg;
        if(!l_image_raw.data==NULL)
        {
            img_msg_l.header.stamp = it->ts_;      
            img_msg_l.header.frame_id = "camera_0";
            img_msg_l.encoding = enc::MONO8;
            if(!req.rectified)
                img_msg_l.image = l_image_raw;
            else
                left_model_.rectifyImage(l_image_raw,img_msg_l.image);
            img_msg_l.toImageMsg(left_msg);
        }
        else
        {
            ROS_DEBUG("Error, couldn't read image from %s",fname_l.c_str());
            return false;   // not much good if we can't read the match image
        }
        string fname_r = (r_format_ % cache_path_.c_str() % it->original_ % "png").str();
        cv::Mat r_image_raw = cv::imread(fname_r.c_str());
        if(!r_image_raw.data==NULL)
        {
            img_msg_r.header.stamp = ros::Time::now();
            img_msg_r.header.frame_id = "camera_1";
            img_msg_r.encoding = enc::MONO8;
            if(!req.rectified)
                img_msg_r.image = r_image_raw;
            else
                left_model_.rectifyImage(r_image_raw,img_msg_r.image);
        }
        else
        {
            ROS_DEBUG("Error, couldn't read image from %s",fname_r.c_str());
            return false;
        }

        res.from_id = it->original_;
        //res.from_id = req.from;
        img_msg_l.toImageMsg(res.from_l);
        img_msg_r.toImageMsg(res.from_r);

        for(int i=0; i < req.to.size(); i++)
        {
            ROS_DEBUG("Requested to image is %d", req.to[i]);
            OriginalIterator it = database.get<original>().find(req.to[i]);
            if(it == database.get<original>().end())
            {
                OriginalIndex& original_ind = database.get<original>();
                it = original_ind.upper_bound(req.to[i]);
                if(it == database.get<original>().end())
                    it = original_ind.lower_bound(req.to[i]);
                
                if(it == database.get<original>().begin())
                    return false;
                else
                    ROS_DEBUG("Couldn't find it, using %d instead",it->original_);
            }                
            string fname_l = (l_format_ % cache_path_.c_str() % it->original_ % "png").str();
            ROS_DEBUG("To image is %s",fname_l.c_str());
            cv::Mat l_image_raw = cv::imread(fname_l.c_str());
            sensor_msgs::Image left_msg;
            if(!l_image_raw.data==NULL)
            {
                cv_bridge::CvImage img_msg_l;
                img_msg_r.header.stamp = it->ts_;
                img_msg_l.header.frame_id = "camera_0";
                img_msg_l.encoding = enc::MONO8;
                if(!req.rectified)
                    img_msg_l.image = l_image_raw;
                else
                    left_model_.rectifyImage(l_image_raw,img_msg_l.image);
                img_msg_l.toImageMsg(left_msg);
            }
            else
            {
                ROS_DEBUG("Error, couldn't read image from %s",fname_l.c_str());
                continue;
            }

            std::string fname_r = (r_format_ % cache_path_.c_str() % it->original_ % "png").str();
            cv::Mat r_image_raw = cv::imread(fname_r.c_str());
            if(!r_image_raw.data==NULL)
            {
                cv_bridge::CvImage img_msg_r;
                img_msg_r.header.stamp = it->ts_;
                img_msg_r.header.frame_id = "camera_1";
                img_msg_r.encoding = enc::MONO8;
                if(!req.rectified)
                    img_msg_r.image = r_image_raw;
                else
                    left_model_.rectifyImage(r_image_raw,img_msg_r.image);
                sensor_msgs::Image right_msg;
                img_msg_r.toImageMsg(right_msg);
                res.to_id.push_back(it->original_);
                res.to_l.push_back(left_msg);
                res.to_r.push_back(right_msg);
            }
            else
            {
                ROS_DEBUG("Error, couldn't read image from %s",fname_r.c_str());
            }
        }
        return true;
    }

    bool getImageRepublished(image_cache::GetImage::Request &req, image_cache::GetImage::Response &res)
    {
        ROS_DEBUG("In republished callback, Requested from image is %d", req.from);
        RepublishedIterator it = database.get<republished>().find(req.from);

        if(it == database.get<republished>().end())
        {
            RepublishedIndex& repub_ind = database.get<republished>();
            it = repub_ind.upper_bound(req.from);
            if(it == database.get<republished>().end())
                it = repub_ind.lower_bound(req.from);
            if(it == database.get<republished>().begin())
                return false;
            else
                ROS_DEBUG("Couldn't find it, using %d instead",it->original_);
        }        
        string fname_l = (l_format_ % cache_path_.c_str() % it->original_ % "png").str();
        cv::Mat l_image_raw = cv::imread(fname_l);

        cv_bridge::CvImage img_msg_l;
        cv_bridge::CvImage img_msg_r;
        
        sensor_msgs::Image left_msg;
        if(!l_image_raw.data==NULL)
        {
            img_msg_l.header.stamp = it->ts_;      
            img_msg_l.header.frame_id = "camera_0";
            img_msg_l.encoding = enc::MONO8;
            if(!req.rectified)
                img_msg_l.image = l_image_raw;
            else
                left_model_.rectifyImage(l_image_raw,img_msg_l.image);
            img_msg_l.toImageMsg(left_msg);
        }
        else
        {
            ROS_DEBUG("Error, couldn't read image from %s",fname_l.c_str());
            return false;   // not much good if we can't read the match image
        }
        string fname_r = (r_format_ % cache_path_.c_str() % it->original_ % "png").str();
        cv::Mat r_image_raw = cv::imread(fname_r.c_str());
        if(!r_image_raw.data==NULL)
        {
            img_msg_r.header.stamp = ros::Time::now();
            img_msg_r.header.frame_id = "camera_1";
            img_msg_r.encoding = enc::MONO8;
            if(!req.rectified)
                img_msg_r.image = r_image_raw;
            else
                left_model_.rectifyImage(r_image_raw,img_msg_r.image);
        }
        else
        {
            ROS_DEBUG("Error, couldn't read image from %s",fname_r.c_str());
            return false;
        }

        res.from_id = it->original_;
        //res.from_id = req.from;
        img_msg_l.toImageMsg(res.from_l);
        img_msg_r.toImageMsg(res.from_r);

        for(int i=0; i < req.to.size(); i++)
        {
            ROS_DEBUG("Requested to image is %d", req.to[i]);
            RepublishedIterator it = database.get<republished>().find(req.to[i]);
            if(it == database.get<republished>().end())
            {
                RepublishedIndex& repub_ind = database.get<republished>();
                it = repub_ind.upper_bound(req.to[i]);
                if(it == database.get<republished>().end())
                    it = repub_ind.lower_bound(req.to[i]);
                
                if(it == database.get<republished>().begin())
                    return false;
                else
                    ROS_DEBUG("Couldn't find it, using %d instead",it->original_);
            }                
            string fname_l = (l_format_ % cache_path_.c_str() % it->original_ % "png").str();
            ROS_DEBUG("To image is %s",fname_l.c_str());
            cv::Mat l_image_raw = cv::imread(fname_l.c_str());
            sensor_msgs::Image left_msg;
            if(!l_image_raw.data==NULL)
            {
                cv_bridge::CvImage img_msg_l;
                img_msg_r.header.stamp = it->ts_;
                img_msg_l.header.frame_id = "camera_0";
                img_msg_l.encoding = enc::MONO8;
                if(!req.rectified)
                    img_msg_l.image = l_image_raw;
                else
                    left_model_.rectifyImage(l_image_raw,img_msg_l.image);
                img_msg_l.toImageMsg(left_msg);
            }
            else
            {
                ROS_DEBUG("Error, couldn't read image from %s",fname_l.c_str());
                continue;
            }

            std::string fname_r = (r_format_ % cache_path_.c_str() % it->original_ % "png").str();
            cv::Mat r_image_raw = cv::imread(fname_r.c_str());
            if(!r_image_raw.data==NULL)
            {
                cv_bridge::CvImage img_msg_r;
                img_msg_r.header.stamp = it->ts_;
                img_msg_r.header.frame_id = "camera_1";
                img_msg_r.encoding = enc::MONO8;
                if(!req.rectified)
                    img_msg_r.image = r_image_raw;
                else
                    left_model_.rectifyImage(r_image_raw,img_msg_r.image);
                sensor_msgs::Image right_msg;
                img_msg_r.toImageMsg(right_msg);
                res.to_id.push_back(it->original_);
                res.to_l.push_back(left_msg);
                res.to_r.push_back(right_msg);
            }
            else
            {
                ROS_DEBUG("Error, couldn't read image from %s",fname_r.c_str());
            }
        }
        return true;
    }

    void serviceThread()
    {
        ros::NodeHandle n;
        while(n.ok())
        {
            service_queue_.callAvailable(ros::WallDuration(0.01));
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cache_server");
    ros::NodeHandle nh("~");

    std::string image_topic, republished_topic, cache_path;
    int div;
    bool use_original;
    nh.param<string>("stereo_topic",image_topic,"/stereo");
    nh.param<string>("republished_topic",republished_topic,"/throttled");
    nh.param<int>("div",div,4);
    nh.param<string>("cache_path",cache_path,"./");
    nh.param<bool>("use_original",use_original,false);
    ROS_INFO("Original param set to %s",use_original ? "true":"false");
    CacheServer *cache_server = new CacheServer(image_topic,republished_topic,div,cache_path,use_original);

    ros::spin();

    delete cache_server;

    return 0;
}
