#include <iostream>
#include <map>
#include <vector>

#include <g2o/math_groups/se2.h>
#include <g2o/core/hyper_graph.h>

#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/block_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include <ros/callback_queue.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <slam_backend/NodeAdded.h>
#include <slam_backend/AddLoopClosure.h>
#include <slam_backend/Graph.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

#include "edge_se2_vis.h"
#include "vertex_se2_vis.h"

using namespace std;
using namespace g2o;
using std::string;
using namespace boost;

typedef BlockSolver< BlockSolverTraits<-1, -1> > SlamBlockSolver;
typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

SE2 Zerotf(0.0,0.0,0.0);
Matrix3d ident_cov = Matrix3d::Identity(); 

class SlamBackend
{
private:
    ros::NodeHandle n_;
    ros::Publisher add_pub_, vertex_vis_pub_, edge_vis_pub_, loop_closure_vis_pub_, graph_pub_;
    ros::Subscriber vo_sub_, add_loop_closure_sub_;
    SparseOptimizer optimizer_;
    SlamLinearSolver* linearSolver_;
    double node_sep_;
    string g2o_frame_;
    double graph_publish_rate_;
    int prevID_;
    ros::Time prev_time_;
    SE2 prev_, curr_, lastOdom_;
    Matrix3d cumulative_covariance;
    int loop_closure_id_;
    int node_count_;
    bool first_run_;
    boost::mutex  m_mutex_;
    int loop_closure_count_, min_closures_;
    ros::Time most_recent_;
    std::map<int,ros::Time> vertex_debug_;
public:

    SlamBackend(string vo_topic, double node_sep, string g2o_frame, double graph_publish_rate, int min_closures) : node_sep_(node_sep), g2o_frame_(g2o_frame), graph_publish_rate_(graph_publish_rate), prevID_(-1), min_closures_(min_closures) 
    {
        first_run_ = true;
        add_pub_ = n_.advertise<slam_backend::NodeAdded>("node_added",1);
        vertex_vis_pub_ = n_.advertise<visualization_msgs::Marker>("vertex_vis",1);
        edge_vis_pub_ = n_.advertise<visualization_msgs::Marker>("edge_vis",1);
        loop_closure_vis_pub_ = n_.advertise<visualization_msgs::Marker>("loop_closure_vis",1);
        graph_pub_ = n_.advertise<slam_backend::Graph>("graph",1);
        vo_sub_ = n_.subscribe(vo_topic, 10, &SlamBackend::vOdomCallback, this);
        add_loop_closure_sub_ = n_.subscribe("add_loop_closure", 100, &SlamBackend::addLoopClosureCallback, this);
        linearSolver_ = new SlamLinearSolver();
        linearSolver_->setBlockOrdering(false);
        SlamBlockSolver *solver = new SlamBlockSolver(&optimizer_, linearSolver_);
        optimizer_.setSolver(solver);
        boost::thread vis_thread(boost::bind(&SlamBackend::visualizeThread, this));
        cumulative_covariance = Matrix3d::Zero();  // read in from msg
        node_count_=0;
        loop_closure_count_=0;
        ROS_INFO("SLAM Backend initialized with \n \t Node Sep = %fm \n \t g2o frame = %s", node_sep_, g2o_frame.c_str());
    }

    void addLoopClosureCallback(const slam_backend::AddLoopClosureConstPtr &msg)
    {
        boost::mutex::scoped_lock l(m_mutex_);
        // find the current edge and delete it
        ROS_INFO("Adding interpolated edge between %d and %d", msg->node_id1, msg->node_id2);
        ROS_INFO("With transform x=%f, y=%f, z=%f", msg->transform.pose.pose.position.x,msg->transform.pose.pose.position.y, msg->transform.pose.pose.position.z);

        if(msg->node_id1 < 1 || msg->node_id2 < 1)
        {
            ROS_INFO("Aborting ...");
            return;
        }
        VertexSE2Vis* v1 = dynamic_cast<VertexSE2Vis*>(optimizer_.vertex(msg->node_id1)); 
        VertexSE2Vis* v2; 
        SparseOptimizer::EdgeSet edge_set(v1->edges()); 
        EdgeSE2Vis* e;

	    bool found = false;
        // Get the ID of the edge we need to delete
        for(SparseOptimizer::EdgeSet::iterator edge_it = edge_set.begin(); edge_it!=edge_set.end(); edge_it++)
        {
            e = dynamic_cast<EdgeSE2Vis*>(*edge_it);
            HyperGraph::VertexVector v_vec = e->vertices();
            for(HyperGraph::VertexVector::iterator vec_it = v_vec.begin(); vec_it != v_vec.end(); vec_it++)
            {
                if((*vec_it)->id() == msg->node_id2)
		{
                    found=true;  // Edge to delete is currently stored in e, assuming only 2 vertices per edge
		    v2 = dynamic_cast<VertexSE2Vis*>(*vec_it);
		}
            }
	    if(found)
		break;
        } 
	if(!found)
	{
	    ROS_INFO("These are not neighbouring edges, returning");
	    return;
	}

        // Get the current edge pose transform and covariance
        SE2 pose = v1->estimate();
        SE2 transf1, transf2;
        double k = msg->frac; 
        double transf[3];
        e->getMeasurementData(transf);
        Vector3d transfv;
        transfv[0] = transf[0];
        transfv[1] = transf[1];
        transfv[2] = transf[2];
        transf1.fromVector(k*transfv);
        
        SE2 interPose = pose * transf1;  // NOT SURE!!!!!!!!!!!!!!!!!!!!!!
        transf2.fromVector((1.0-k)*transfv); 

        Matrix3d information = e->information();
        //Matrix3d cov1 = 0.5*information.inverse();
        //Matrix3d cov2 = (1.0-msg->frac)*(1.0-msg->frac)*information.inverse();

        // Add robot poses
        VertexSE2Vis* interNode = new VertexSE2Vis;
        interNode->setId(node_count_++);
        interNode->setEstimate(interPose);
        interNode->setLoopClosure(); 
        interNode->setMarginalized(false);
        optimizer_.addVertex(interNode); 
       
        // remove old edge
        optimizer_.removeEdge(e);  
        
        // add 2 new edges in its place    
        EdgeSE2Vis* odometry1 = new EdgeSE2Vis();
        EdgeSE2Vis* odometry2 = new EdgeSE2Vis();
        odometry1->vertices()[0] = optimizer_.vertex(v1->id());
        odometry1->vertices()[1] = optimizer_.vertex(interNode->id());
        odometry1->setMeasurement(transf1);
        odometry1->setInverseMeasurement(transf1.inverse());
        //odometry1->setInformation(ident_cov.inverse());
        odometry1->setInformation(information); // keep same as previous
        //ROS_INFO("Added interpolated covariance with diagonal %f, %f, %f", cov1(0,0), cov1(1,1), cov1(2,2));
        optimizer_.addEdge(odometry1);

        odometry2->vertices()[0] = optimizer_.vertex(interNode->id());
        odometry2->vertices()[1] = optimizer_.vertex(v2->id());
        odometry2->setMeasurement(transf2);
        odometry2->setInverseMeasurement(transf2.inverse());
        //odometry2->setInformation(ident_cov.inverse());
        odometry2->setInformation(information);
        //ROS_INFO("Added interpolated covariance with diagonal %f, %f, %f", cov1(0,0), cov1(1,1), cov1(2,2));
        optimizer_.addEdge(odometry2);
             
        slam_backend::NodeAdded added_msg;
        added_msg.header.stamp = msg->interpolated_time; 
        added_msg.node_id = interNode->id(); 
        vertex_debug_[interNode->id()]=msg->interpolated_time;
        add_pub_.publish(added_msg);

        // Now add a new node at the current position and and edge to it
        // Add current pose 
        if(cumulative_covariance(0,0) == 0)
            return;     // Don't do things that will stuff up the graph
        
        VertexSE2Vis* newNode = new VertexSE2Vis;
        newNode->setId(node_count_++);
        newNode->setLoopClosure(); 
        newNode->setEstimate(curr_);  // use global last-known point
        newNode->setMarginalized(false);
        optimizer_.addVertex(newNode); 

        SE2 newtransf = prev_.inverse() * curr_; 
             
        EdgeSE2Vis* new_odometry = new EdgeSE2Vis();
        new_odometry->vertices()[0] = optimizer_.vertex(prevID_);
        new_odometry->vertices()[1] = optimizer_.vertex(newNode->id());
        new_odometry->setMeasurement(newtransf);
        new_odometry->setInverseMeasurement(newtransf.inverse());
        new_odometry->setInformation(ident_cov);
        optimizer_.addEdge(new_odometry);
               
        prevID_ = newNode->id();
        prev_ = curr_;

        // Reset the cumulative covariance
        cumulative_covariance = Matrix3d::Zero();
             
        added_msg.header.stamp = most_recent_;
        added_msg.node_id = newNode->id(); 
        vertex_debug_[newNode->id()]=most_recent_;
        add_pub_.publish(added_msg);

        // Now add an edge between the interpolated pose and the new pose
        SE2 looptransf(msg->transform.pose.pose.position.x,msg->transform.pose.pose.position.y,2*atan2(msg->transform.pose.pose.orientation.z,msg->transform.pose.pose.orientation.w));

        SE2 edgetransf = looptransf; 
	edgetransf[0]=0.0;
	edgetransf[1]=0.0;
	edgetransf[2]=0.0;

        EdgeSE2Vis* loop_odometry = new EdgeSE2Vis();
        loop_odometry->vertices()[0] = optimizer_.vertex(newNode->id());
        loop_odometry->vertices()[1] = optimizer_.vertex(interNode->id());
        loop_odometry->setMeasurement(edgetransf);
        loop_odometry->setInverseMeasurement(edgetransf.inverse());
        Matrix3d loop_covariance = Matrix3d::Zero();
        loop_covariance(0,0) = msg->transform.pose.covariance[0];
        loop_covariance(1,1) = msg->transform.pose.covariance[7];
        loop_covariance(2,2) = msg->transform.pose.covariance[21];
        loop_odometry->setInformation(loop_covariance.inverse());
        ROS_INFO("Added loop closure covariance with diagonal %f, %f, %f", loop_covariance(0,0), loop_covariance(1,1), loop_covariance(2,2));
        //loop_odometry->setInformation(ident_cov.inverse());
	    loop_odometry->setLoopClosure();
        //ROS_INFO("Added LOOP closure edge with %f,%f,%f",edgetransf[0],edgetransf[1],edgetransf[2]);
        optimizer_.addEdge(loop_odometry);

	// prepare and run the optimization
	//optimizer_.computeInitialGuess();
    loop_closure_count_++;
    if(loop_closure_count_ == min_closures_)
    {
	    ROS_INFO("Optimizing");
	    optimizer_.initializeOptimization();
        optimizer_.optimize(10);
        ROS_INFO("Optimizer DONE");
        loop_closure_count_ = 0;

        // Reset previous and current following graph reorganization
        prev_ = newNode->estimate();
        curr_ = prev_;

    }
            
    }

    void vOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        boost::mutex::scoped_lock l(m_mutex_);
        SE2 curr_pose(msg->pose.pose.position.x,msg->pose.pose.position.y,2*atan2(msg->pose.pose.orientation.z,msg->pose.pose.orientation.w));

        if(first_run_)
        {
            VertexSE2Vis* robot = new VertexSE2Vis;
            SE2 pose(0,0,0);
            robot->setId(node_count_++);
            robot->setEstimate(pose);
	        robot->setFixed(true);
            robot->setMarginalized(false);
            optimizer_.addVertex(robot); 
            slam_backend::NodeAdded added_msg;
            added_msg.header.stamp = msg->header.stamp; 
            added_msg.node_id = robot->id(); 
            prevID_ = robot->id();
            prev_ = pose;
            prev_time_ = msg->header.stamp;
            vertex_debug_[robot->id()]=msg->header.stamp;
            add_pub_.publish(added_msg);
            first_run_ = false;
            lastOdom_ = curr_pose;
            return;
        }
        SE2 tf = lastOdom_.inverse() * curr_pose; 
        lastOdom_ = curr_pose;
       
        // should be identical to transform case 
        most_recent_ = msg->header.stamp;  // log most recent time stamp for when we need to add nodes in loop closure callback
        curr_ *= tf;     // Instead of taking viso's pose estimate, maintain an internal pose estimate in curr_
        double dist_travelled = (curr_[0]-prev_[0])*(curr_[0]-prev_[0])+(curr_[1]-prev_[1])*(curr_[1]-prev_[1]);
      
        bool bad_edge = false; 
        if(msg->pose.covariance[0] != 9999 && cumulative_covariance(0,0) < 9999)
        {     
            cumulative_covariance(0,0) = cumulative_covariance(0,0)+msg->pose.covariance[0];
            cumulative_covariance(1,1) = cumulative_covariance(1,1)+msg->pose.covariance[7];
            cumulative_covariance(2,2) = cumulative_covariance(2,2)+msg->pose.covariance[21];  // Yaw?
        }
        else
        {   
            ROS_INFO("Inserting weak edge"); 
            cumulative_covariance(0,0) = 9999;
            cumulative_covariance(1,1) = 9999;
            cumulative_covariance(2,2) = 9999;  // Yaw?
            bad_edge = true;
        }
            
        if(dist_travelled > node_sep_*node_sep_)
        {
            // Add robot poses
            Matrix3d information;
            VertexSE2Vis* robot = new VertexSE2Vis;
            robot->setId(node_count_++);

            robot->setEstimate(curr_);
           // if(bad_edge)
            //    robot->setMarginalized(true);
            //else
                robot->setMarginalized(false);

            optimizer_.addVertex(robot); 

            SE2 transf = prev_.inverse() * curr_; 
             
            EdgeSE2Vis* odometry = new EdgeSE2Vis();
            odometry->vertices()[0] = optimizer_.vertex(prevID_);
            odometry->vertices()[1] = optimizer_.vertex(robot->id());
            odometry->setMeasurement(transf);
            //ROS_INFO("Added ege with %f,%f,%f",transf[0],transf[1],transf[2]);
            odometry->setInverseMeasurement(transf.inverse());
            // Temporary fix
         /*   cumulative_covariance = Matrix3d::Zero();
            cumulative_covariance(0,0) = (0.3*node_sep_)*(0.3*node_sep_);
          cumulative_covariance(1,1) = (0.3*node_sep_)*(0.3*node_sep_);
            cumulative_covariance(2,2) = (60*3.1415/180)*(60*3.1415/180);
            odometry->setInformation(cumulative_covariance.inverse());*/
            //odometry->setInformation(ident_cov.inverse());
            odometry->setInformation(cumulative_covariance.inverse());
            ROS_INFO("Added covariance with diagonal %f, %f, %f", cumulative_covariance(0,0), cumulative_covariance(1,1), cumulative_covariance(2,2));
            optimizer_.addEdge(odometry);
               
            prevID_ = robot->id();
            prev_ = curr_;
            prev_time_ = msg->header.stamp;

            cumulative_covariance = Matrix3d::Zero();
             
            slam_backend::NodeAdded added_msg;
            added_msg.header.stamp = msg->header.stamp; 
            added_msg.node_id = robot->id(); 
            vertex_debug_[robot->id()]=msg->header.stamp;
            add_pub_.publish(added_msg);

	    }
        
    }

/*    void vOdomCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
    {
        boost::mutex::scoped_lock l(m_mutex_);
        if(first_run_)
        {
            VertexSE2Vis* robot = new VertexSE2Vis;
            SE2 pose(0,0,0);
            robot->setId(node_count_++);
            robot->setEstimate(pose);
	        robot->setFixed(true);
            robot->setMarginalized(false);
            optimizer_.addVertex(robot); 
            slam_backend::NodeAdded added_msg;
            added_msg.header.stamp = msg->header.stamp; 
            added_msg.node_id = robot->id(); 
            prevID_ = robot->id();
            prev_ = pose;
            prev_time_ = msg->header.stamp;
            vertex_debug_[robot->id()]=msg->header.stamp;
            add_pub_.publish(added_msg);
            first_run_ = false;
            return;
        }

        SE2 tf;
        tf[0] = msg->pose.pose.position.x;
        tf[1] = msg->pose.pose.position.y;
        tf[2] = 2*atan2(msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

        most_recent_ = msg->header.stamp;  // log most recent time stamp for when we need to add nodes in loop closure callback
        curr_ *= tf;     // Instead of taking viso's pose estimate, maintain an internal pose estimate in curr_
        double dist_travelled = (curr_[0]-prev_[0])*(curr_[0]-prev_[0])+(curr_[1]-prev_[1])*(curr_[1]-prev_[1]);
      
        bool bad_edge = false; 
        if(msg->pose.covariance[0] != 9999 && cumulative_covariance(0,0) < 9999)
        {     
            cumulative_covariance(0,0) = cumulative_covariance(0,0)+msg->pose.covariance[0];
            cumulative_covariance(1,1) = cumulative_covariance(1,1)+msg->pose.covariance[7];
            cumulative_covariance(2,2) = cumulative_covariance(2,2)+msg->pose.covariance[21];  // Yaw?
           // cumulative_covariance(3,3) = cumulative_covariance(3,3)+msg->pose.covariance[21];
            //cumulative_covariance(4,4) = cumulative_covariance(4,4)+msg->pose.covariance[28];
            //cumulative_covariance(5,5) = cumulative_covariance(5,5)+msg->pose.covariance[35];
        }
        else
        {   
            ROS_INFO("Inserting weak edge"); 
            cumulative_covariance(0,0) = 9999;
            cumulative_covariance(1,1) = 9999;
            cumulative_covariance(2,2) = 9999;  // Yaw?
            bad_edge = true;
        }
            
        if(dist_travelled > node_sep_*node_sep_)
        {
            // Add robot poses
            Matrix3d information;
            VertexSE2Vis* robot = new VertexSE2Vis;
            robot->setId(node_count_++);

            robot->setEstimate(curr_);
           // if(bad_edge)
            //    robot->setMarginalized(true);
            //else
                robot->setMarginalized(false);

            optimizer_.addVertex(robot); 

            SE2 transf = prev_.inverse() * curr_; 
             
            EdgeSE2Vis* odometry = new EdgeSE2Vis();
            odometry->vertices()[0] = optimizer_.vertex(prevID_);
            odometry->vertices()[1] = optimizer_.vertex(robot->id());
            odometry->setMeasurement(transf);
            //ROS_INFO("Added ege with %f,%f,%f",transf[0],transf[1],transf[2]);
            odometry->setInverseMeasurement(transf.inverse());
            // Temporary fix
        //    cumulative_covariance = Matrix3d::Zero();
         //   cumulative_covariance(0,0) = (0.3*node_sep_)*(0.3*node_sep_);
          //  cumulative_covariance(1,1) = (0.3*node_sep_)*(0.3*node_sep_);
           // cumulative_covariance(2,2) = (60*3.1415/180)*(60*3.1415/180);
           // odometry->setInformation(cumulative_covariance.inverse());
            //odometry->setInformation(ident_cov.inverse());
            odometry->setInformation(cumulative_covariance.inverse());
            ROS_INFO("Added covariance with diagonal %f, %f, %f", cumulative_covariance(0,0), cumulative_covariance(1,1), cumulative_covariance(2,2));
            optimizer_.addEdge(odometry);
               
            prevID_ = robot->id();
            prev_ = curr_;
            prev_time_ = msg->header.stamp;

            cumulative_covariance = Matrix3d::Zero();
             
            slam_backend::NodeAdded added_msg;
            added_msg.header.stamp = msg->header.stamp; 
            added_msg.node_id = robot->id(); 
            vertex_debug_[robot->id()]=msg->header.stamp;
            add_pub_.publish(added_msg);

	   }
    }*/

    void publishGraph()
    {
        slam_backend::Graph graph_msg;

        for(SparseOptimizer::VertexIDMap::iterator it = optimizer_.vertices().begin(); it != optimizer_.vertices().end(); ++it)
        {
            VertexSE2Vis* v = dynamic_cast<VertexSE2Vis*>(it->second);

            if(v)
            {
                graph_msg.id.push_back(v->id());
                graph_msg.stamp.push_back(vertex_debug_[v->id()]);
                SE2 pose = v->estimate();
                geometry_msgs::Pose2D tmp_msg;
                tmp_msg.x = pose[0];
                tmp_msg.y = pose[1];
                tmp_msg.theta = pose[2];
                graph_msg.pose.push_back(tmp_msg);
            }
        }
        graph_pub_.publish(graph_msg);

    }

    void visualizeNodes()
    {
        visualization_msgs::Marker lines;
        lines.type = visualization_msgs::Marker::LINE_LIST; 
        lines.header.frame_id = g2o_frame_;
        lines.header.stamp = ros::Time::now();
        lines.id = 1;  // constant
        lines.ns = "pose_graph";
        lines.action = visualization_msgs::Marker::ADD;
        lines.scale.x = lines.scale.y = lines.scale.z = 0.1;
        lines.color.r = 1.0;
        lines.color.g = 1.0;
        lines.color.b = 1.0;
        lines.color.a = 1.0;

        visualization_msgs::Marker loop_closures;
        loop_closures.type = visualization_msgs::Marker::LINE_LIST; 
        loop_closures.header.frame_id = g2o_frame_;
        loop_closures.header.stamp = ros::Time::now();
        loop_closures.id = 1;  // constant
        loop_closures.ns = "pose_graph";
        loop_closures.action = visualization_msgs::Marker::ADD;
        loop_closures.scale.x = lines.scale.y = lines.scale.z = 0.1;
        loop_closures.color.r = 1.0;
        loop_closures.color.g = 0.0;
        loop_closures.color.b = 0.0;
        loop_closures.color.a = 1.0;


        for(SparseOptimizer::VertexIDMap::iterator it = optimizer_.vertices().begin(); it != optimizer_.vertices().end(); ++it)
        {
            VertexSE2Vis* v = dynamic_cast<VertexSE2Vis*>(it->second);

            if(v)
            {
                SE2 pose = v->estimate();
                // Publish each message as an individually scaled an oriented sphere
                visualization_msgs::Marker vertex;
                vertex.header.frame_id = g2o_frame_;
                vertex.header.stamp = ros::Time::now();
                vertex.id = v->id();
                vertex.ns = "pose_graph";
                vertex.action = visualization_msgs::Marker::ADD;
                if(v->isLoopClosure())
                {
                    vertex.color.r = 1.0;
                    vertex.color.g = 0.0;
                    vertex.color.b = 0.0;
                    vertex.color.a = 1.0;
                }
                else
                {
                    vertex.color.r = 1.0;
                    vertex.color.g = 1.0;
                    vertex.color.b = 0.0;
                    vertex.color.a = 1.0;
                }

                vertex.type = visualization_msgs::Marker::SPHERE;

                // get ellipsoid representation of covariance
    /*            EigenSolver<Matrix3d> es;
                Matrix3d cov = v->uncertainty();
                Matrix3d use_this = Matrix3d::Zero();
                use_this.block(0,0,2,2) = cov.block(0,0,2,2);
                use_this(2,2)=0.01;
                es.compute(use_this,true);
                
                // compose a rotation matrix
                Matrix3d eigVec = es.pseudoEigenvectors();

                // compute a quaternion from the rotation matrix
                Quaternion<double> quat(eigVec);
                vertex.pose.position.x = pose[0];
                vertex.pose.position.y = pose[1];
                vertex.pose.position.z = 0;

                vertex.pose.orientation.x = quat.x();
                vertex.pose.orientation.y = quat.y();
                vertex.pose.orientation.z = quat.z();
                vertex.pose.orientation.w = quat.w();
               
                double xscale = es.pseudoEigenvalueMatrix()(0,0);
                double yscale = es.pseudoEigenvalueMatrix()(1,1);
                cout << "xscale: " << xscale << "\n";
                if(xscale > 0.1 && !isnan<double>(xscale) && xscale < 20  && isfinite<double>(xscale) )
                    vertex.scale.x = 1*(xscale);
                else
                    vertex.scale.x = 1;
		
                if(yscale > 0.1 && !isnan<double>(yscale) && yscale < 20  && isfinite<double>(yscale) )
                    vertex.scale.y = 1*(yscale);
                else
                    vertex.scale.y = 1;

                vertex.scale.z = 1;
*/
 	// Just for the moment
    vertex.pose.position.x = pose[0];
    vertex.pose.position.y = pose[1];
    vertex.pose.position.z = 0;

    vertex.pose.orientation.x = 0;
    vertex.pose.orientation.y = 0.;
    vertex.pose.orientation.z = sin(pose[2]/2);
    vertex.pose.orientation.w = cos(pose[2]/2);
	vertex.scale.x = 0.5;
	vertex.scale.y = 0.5;
	vertex.scale.z = 0.5;
    vertex_vis_pub_.publish(vertex);
            
                SparseOptimizer::EdgeSet e(v->edges()); 
                for(SparseOptimizer::EdgeSet::iterator edge_it = e.begin(); edge_it!=e.end(); edge_it++)
                {
                    EdgeSE2Vis *e = dynamic_cast<EdgeSE2Vis*>(*edge_it);
                    HyperGraph::VertexVector v_vec = e->vertices();

                    // Assume only 2 vertices per edge!
                    VertexSE2Vis* v0 = dynamic_cast<VertexSE2Vis*>(v_vec[0]); 
                    VertexSE2Vis* v1 = dynamic_cast<VertexSE2Vis*>(v_vec[1]); 
                    SE2 p0 = v0->estimate();
                    SE2 p1 = v1->estimate();

                    geometry_msgs::Point p0_msg;
                    p0_msg.x = p0[0];
                    p0_msg.y = p0[1];
                    p0_msg.z = 0;
                    
                    geometry_msgs::Point p1_msg;
                    p1_msg.x = p1[0];
                    p1_msg.y = p1[1];
                    p1_msg.z = 0;
                    if(e->isLoopClosure())
                    {
                        loop_closures.points.push_back(p0_msg);
                        loop_closures.points.push_back(p1_msg);
                    }
                    else
                    {
                        lines.points.push_back(p0_msg);
                        lines.points.push_back(p1_msg); 
                    }
                }
            }
        }
        edge_vis_pub_.publish(lines);
        loop_closure_vis_pub_.publish(loop_closures);
    }
   
    void visualizeThread()
    {
        ros::Rate loop_rate(graph_publish_rate_);
        while(n_.ok())
        {
            visualizeNodes();
            publishGraph();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"slam_backend");
    ros::NodeHandle n("~");

    string vo_topic, slam_map_frame;
    double vertex_sep, graph_publish_rate;
    int min_closures;

    n.param<string>("vo_topic",vo_topic,"/vo/transform");
    n.param<double>("vertex_sep",vertex_sep,1.0);
    n.param<double>("graph_publish_rate",graph_publish_rate,0.2);
    n.param<string>("slam_map_frame",slam_map_frame,"/slam_map");
    n.param<int>("min_closures_to_optimize",min_closures,1);

    SlamBackend *backend = new SlamBackend(vo_topic,vertex_sep,slam_map_frame,graph_publish_rate,min_closures);

    ros::spin();

    delete backend;

    return 0;

}

