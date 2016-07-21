#include <iostream> 

#include <opencv2/highgui/highgui.hpp>

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/data/robot_laser.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"

#include "g2o/stuff/command_args.h"

#include "frequency_map.h"


//ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/GetMap.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"

//openCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>








using namespace std;
using namespace Eigen;
using namespace g2o;






class ROS_handler
{
	ros::NodeHandle n;
	
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;	
	cv_bridge::CvImagePtr cv_ptr;
		
	std::string mapname_;
	ros::Subscriber map_sub_;	
	ros::Timer timer;
			
	float Decomp_threshold_;

	std::vector <double> clean_time_vector, decomp_time_vector, paint_time_vector, complete_time_vector;

	ros::Subscriber odom_sub_;
	ros::Subscriber pose_sub_;
	
	public:
		ROS_handler(const std::string& mapname, float threshold) : mapname_(mapname),  it_(n), Decomp_threshold_(threshold)
		{
			ROS_INFO("Waiting for the map");
			odom_sub_ = n.subscribe("pose_corrected", 1, &ROS_handler::odomCallback, this);
			pose_sub_ = n.subscribe("/robot_0/trajectory", 1, &ROS_handler::poseCallback, this);
			
			timer = n.createTimer(ros::Duration(0.5), &ROS_handler::metronomeCallback, this);
			image_pub_ = it_.advertise("/image_g2o", 1);
			
			
			cv_ptr.reset (new cv_bridge::CvImage);
			cv_ptr->encoding = "mono8";
		}

		~ROS_handler()	{
		}
		
/////////////////		
		void metronomeCallback(const ros::TimerEvent&)
		{
//		  ROS_INFO("tic tac");
		  publish_Image();
		}


////////////////
		void odomCallback(const nav_msgs::Odometry& msg)
		{
			float x =  msg.pose.pose.position.x;
			float y =  msg.pose.pose.position.y;
			
			cout<< "Im in " << x <<", "<< y <<endl;
		}

////////////////
		void poseCallback(const geometry_msgs::PoseArray& msg)
		{
			read_and_publish();
			cout<< "Im in "<<endl;
		}



////////////////////////
// PUBLISHING METHODS		
////////////////////////////		
		void publish_Image(){
			image_pub_.publish(cv_ptr->toImageMsg());
		}

///////////////////////////////////////////

int read_and_publish() {
  /************************************************************************
   *                          Input handling                              *
   ************************************************************************/
  float rows, cols, gain, square_size;
  float resolution, max_range, usable_range, angle, threshold;
  string g2oFilename, mapFilename;


	resolution = 0.05f;
	threshold =  -1.0f;
	rows, 0;
	cols, 0;
	max_range= -1.0f;
	usable_range, -1.0f;
	gain =  1;
	square_size= 1;
	angle =  0;
	g2oFilename= "robot-0-testmrslam.g2o";
	mapFilename= "no importa";
	




  /************************************************************************
   *                          Loading Graph                               *
   ************************************************************************/
  // Load graph
  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
  SlamLinearSolver *linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton *solverGauss = new OptimizationAlgorithmGaussNewton(blockSolver);
  SparseOptimizer *graph = new SparseOptimizer();
  graph->setAlgorithm(solverGauss);    
  graph->load(g2oFilename.c_str());
  
  // Sort verteces
  vector<int> vertexIds(graph->vertices().size());
  int k = 0;
  for(OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it) {
    vertexIds[k++] = (it->first);
  }  
  sort(vertexIds.begin(), vertexIds.end());
  
  /************************************************************************
   *                          Compute map size                            *
   ************************************************************************/
  // Check the entire graph to find map bounding box
  Eigen::Matrix2d boundingBox = Eigen::Matrix2d::Zero();
  std::vector<RobotLaser*> robotLasers;
  std::vector<SE2> robotPoses;
  double xmin=std::numeric_limits<double>::max();
  double xmax=std::numeric_limits<double>::min();
  double ymin=std::numeric_limits<double>::max();
  double ymax=std::numeric_limits<double>::min();

  SE2 baseTransform(0,0,angle);

  for(size_t i = 0; i < vertexIds.size(); ++i) {
    OptimizableGraph::Vertex *_v = graph->vertex(vertexIds[i]);
    VertexSE2 *v = dynamic_cast<VertexSE2*>(_v);
    if(!v) { continue; }
    v->setEstimate(baseTransform*v->estimate());
    OptimizableGraph::Data *d = v->userData();

    while(d) {
      RobotLaser *robotLaser = dynamic_cast<RobotLaser*>(d);
      if(!robotLaser) {
	d = d->next();
	continue;
      }      
      robotLasers.push_back(robotLaser);
      robotPoses.push_back(v->estimate());
      double x = v->estimate().translation().x();
      double y = v->estimate().translation().y();
      
      xmax = xmax > x+usable_range ? xmax : x+usable_range;
      ymax = ymax > y+usable_range ? ymax : y+usable_range;
      xmin = xmin < x-usable_range ? xmin : x-usable_range;
      ymin = ymin < y-usable_range ? ymin : y-usable_range;
 
      d = d->next();
    }
  }

  boundingBox(0,0)=xmin;
  boundingBox(0,1)=xmax;
  boundingBox(1,0)=ymin;
  boundingBox(1,1)=ymax;

  std::cout << "Found " << robotLasers.size() << " laser scans"<< std::endl;
  std::cout << "Bounding box: " << std::endl << boundingBox << std::endl; 
  if(robotLasers.size() == 0)  {
    std::cout << "No laser scans found ... quitting!" << std::endl;
    return 0;
  }

  /************************************************************************
   *                          Compute the map                             *
   ************************************************************************/
  // Create the map
  Eigen::Vector2i size;
  if(rows != 0 && cols != 0) { size = Eigen::Vector2i(rows, cols); }
  else {
    size = Eigen::Vector2i((boundingBox(0, 1) - boundingBox(0, 0))/ resolution,
			   (boundingBox(1, 1) - boundingBox(1, 0))/ resolution);
    } 
  std::cout << "Map size: " << size.transpose() << std::endl;
  if(size.x() == 0 || size.y() == 0) {
    std::cout << "Zero map size ... quitting!" << std::endl;
    return 0;
  }

	cout<< "justo despues "<<endl;  

  //Eigen::Vector2f offset(-size.x() * resolution / 2.0f, -size.y() * resolution / 2.0f);
  Eigen::Vector2f offset(boundingBox(0, 0),boundingBox(1, 0));
  FrequencyMapCell unknownCell;
  FrequencyMap map = FrequencyMap(resolution, offset, size, unknownCell);

	cout<< "antes de asginar"<<endl;

  for(size_t i = 0; i < vertexIds.size(); ++i) {
    OptimizableGraph::Vertex *_v = graph->vertex(vertexIds[i]);
    VertexSE2 *v = dynamic_cast<VertexSE2*>(_v);
    if(!v) { continue; }
    OptimizableGraph::Data *d = v->userData();
    SE2 robotPose = v->estimate();
    
    while(d) {
      RobotLaser *robotLaser = dynamic_cast<RobotLaser*>(d);
      if(!robotLaser) {
	d = d->next();
	continue;
      }      
      map.integrateScan(robotLaser, robotPose, max_range, usable_range, gain, square_size);
      d = d->next();
    }
  }

  /************************************************************************
   *                          Save map image                              *
   ************************************************************************/
  cv::Mat mapImage(map.rows(), map.cols(), CV_8UC1);
  mapImage.setTo(cv::Scalar(0));
  for(int c = 0; c < map.cols(); c++) {
    for(int r = 0; r < map.rows(); r++) {
      if(map(r, c).misses() == 0 && map(r, c).hits() == 0) {
	mapImage.at<unsigned char>(r, c) = 127;
      } else {
	float fraction = (float)map(r, c).hits()/(float)(map(r, c).hits()+map(r, c).misses());
	
	if (threshold > 0 && fraction > threshold)
	  mapImage.at<unsigned char>(r, c) = 0;
	else if (threshold > 0 && fraction <= threshold)
	  mapImage.at<unsigned char>(r, c) = 255;
	else {
	  float val = 255*(1-fraction);
	  mapImage.at<unsigned char>(r, c) = (unsigned char)val;
	}

      }
      // else if(map(r, c).hits() > threshold) {
      // 	mapImage.at<unsigned char>(r, c) = 255;
      // }
      // else {
      // 	mapImage.at<unsigned char>(r, c) = 0;
      // }
    }
  }

	cout<< "despues de asginar "<<endl;

	cv::Mat grad = mapImage;
	
	cv_ptr->encoding = "32FC1";
	grad.convertTo(grad, CV_32F);
	grad.copyTo(cv_ptr->image);////most important


  return 0;
}








/////////////////////////////////////////////////
		
		

};


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "g2o2occ");
	
	std::string mapname = "map";
	
	float decomp_th=3;
	if (argc ==2){ decomp_th = atof(argv[1]); }	
	
	ROS_handler mg(mapname, decomp_th);
	ros::spin();
	
	return 0;
}











int read_and_publish() {
  /************************************************************************
   *                          Input handling                              *
   ************************************************************************/
  float rows, cols, gain, square_size;
  float resolution, max_range, usable_range, angle, threshold;
  string g2oFilename, mapFilename;


	resolution = 0.05f;
	threshold =  -1.0f;
	rows, 0;
	cols, 0;
	max_range= -1.0f;
	usable_range, -1.0f;
	gain =  1;
	square_size= 1;
	angle =  0;
	g2oFilename= "pepito.g2o";
	mapFilename= "no importa";
	




  /************************************************************************
   *                          Loading Graph                               *
   ************************************************************************/
  // Load graph
  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
  SlamLinearSolver *linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton *solverGauss = new OptimizationAlgorithmGaussNewton(blockSolver);
  SparseOptimizer *graph = new SparseOptimizer();
  graph->setAlgorithm(solverGauss);    
  graph->load(g2oFilename.c_str());
  
  // Sort verteces
  vector<int> vertexIds(graph->vertices().size());
  int k = 0;
  for(OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it) {
    vertexIds[k++] = (it->first);
  }  
  sort(vertexIds.begin(), vertexIds.end());
  
  /************************************************************************
   *                          Compute map size                            *
   ************************************************************************/
  // Check the entire graph to find map bounding box
  Eigen::Matrix2d boundingBox = Eigen::Matrix2d::Zero();
  std::vector<RobotLaser*> robotLasers;
  std::vector<SE2> robotPoses;
  double xmin=std::numeric_limits<double>::max();
  double xmax=std::numeric_limits<double>::min();
  double ymin=std::numeric_limits<double>::max();
  double ymax=std::numeric_limits<double>::min();

  SE2 baseTransform(0,0,angle);

  for(size_t i = 0; i < vertexIds.size(); ++i) {
    OptimizableGraph::Vertex *_v = graph->vertex(vertexIds[i]);
    VertexSE2 *v = dynamic_cast<VertexSE2*>(_v);
    if(!v) { continue; }
    v->setEstimate(baseTransform*v->estimate());
    OptimizableGraph::Data *d = v->userData();

    while(d) {
      RobotLaser *robotLaser = dynamic_cast<RobotLaser*>(d);
      if(!robotLaser) {
	d = d->next();
	continue;
      }      
      robotLasers.push_back(robotLaser);
      robotPoses.push_back(v->estimate());
      double x = v->estimate().translation().x();
      double y = v->estimate().translation().y();
      
      xmax = xmax > x+usable_range ? xmax : x+usable_range;
      ymax = ymax > y+usable_range ? ymax : y+usable_range;
      xmin = xmin < x-usable_range ? xmin : x-usable_range;
      ymin = ymin < y-usable_range ? ymin : y-usable_range;
 
      d = d->next();
    }
  }

  boundingBox(0,0)=xmin;
  boundingBox(0,1)=xmax;
  boundingBox(1,0)=ymin;
  boundingBox(1,1)=ymax;

  std::cout << "Found " << robotLasers.size() << " laser scans"<< std::endl;
  std::cout << "Bounding box: " << std::endl << boundingBox << std::endl; 
  if(robotLasers.size() == 0)  {
    std::cout << "No laser scans found ... quitting!" << std::endl;
    return 0;
  }

  /************************************************************************
   *                          Compute the map                             *
   ************************************************************************/
  // Create the map
  Eigen::Vector2i size;
  if(rows != 0 && cols != 0) { size = Eigen::Vector2i(rows, cols); }
  else {
    size = Eigen::Vector2i((boundingBox(0, 1) - boundingBox(0, 0))/ resolution,
			   (boundingBox(1, 1) - boundingBox(1, 0))/ resolution);
    } 
  std::cout << "Map size: " << size.transpose() << std::endl;
  if(size.x() == 0 || size.y() == 0) {
    std::cout << "Zero map size ... quitting!" << std::endl;
    return 0;
  }

  

  //Eigen::Vector2f offset(-size.x() * resolution / 2.0f, -size.y() * resolution / 2.0f);
  Eigen::Vector2f offset(boundingBox(0, 0),boundingBox(1, 0));
  FrequencyMapCell unknownCell;
  FrequencyMap map = FrequencyMap(resolution, offset, size, unknownCell);

  for(size_t i = 0; i < vertexIds.size(); ++i) {
    OptimizableGraph::Vertex *_v = graph->vertex(vertexIds[i]);
    VertexSE2 *v = dynamic_cast<VertexSE2*>(_v);
    if(!v) { continue; }
    OptimizableGraph::Data *d = v->userData();
    SE2 robotPose = v->estimate();
    
    while(d) {
      RobotLaser *robotLaser = dynamic_cast<RobotLaser*>(d);
      if(!robotLaser) {
	d = d->next();
	continue;
      }      
      map.integrateScan(robotLaser, robotPose, max_range, usable_range, gain, square_size);
      d = d->next();
    }
  }

  /************************************************************************
   *                          Save map image                              *
   ************************************************************************/
  cv::Mat mapImage(map.rows(), map.cols(), CV_8UC1);
  mapImage.setTo(cv::Scalar(0));
  for(int c = 0; c < map.cols(); c++) {
    for(int r = 0; r < map.rows(); r++) {
      if(map(r, c).misses() == 0 && map(r, c).hits() == 0) {
	mapImage.at<unsigned char>(r, c) = 127;
      } else {
	float fraction = (float)map(r, c).hits()/(float)(map(r, c).hits()+map(r, c).misses());
	
	if (threshold > 0 && fraction > threshold)
	  mapImage.at<unsigned char>(r, c) = 0;
	else if (threshold > 0 && fraction <= threshold)
	  mapImage.at<unsigned char>(r, c) = 255;
	else {
	  float val = 255*(1-fraction);
	  mapImage.at<unsigned char>(r, c) = (unsigned char)val;
	}

      }
      // else if(map(r, c).hits() > threshold) {
      // 	mapImage.at<unsigned char>(r, c) = 255;
      // }
      // else {
      // 	mapImage.at<unsigned char>(r, c) = 0;
      // }
    }
  }
  cv::imwrite(mapFilename + ".png", mapImage);

  /************************************************************************
   *                          Write yaml file                             *
   ************************************************************************/
  std::ofstream ofs(string(mapFilename + ".yaml").c_str());
  Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);
  ofs << "image: " << mapFilename << ".png" << std::endl
      << "resolution: " << resolution << std::endl
      << "origin: [" << origin.x() << ", " << origin.y() << ", " << origin.z() << "]" << std::endl
      << "negate: 0" << std::endl
      << "occupied_thresh: " << 0.65f << std::endl
      << "free_thresh: " << 0.2f << std::endl;
  return 0;
}