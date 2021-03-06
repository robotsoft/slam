/**
 * Visual Range Odometry Server
 *
 * Created on : 10/30/2012
 * Author : Soonhac Hong (sxhong1@ualr.edu)
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <rgbd_vro/vroAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <swiss_ranger/sr4kAction.h>
#include <rgbd_kinect/kinectAction.h>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>  //for SURF and SIFT
#include <opencv2/legacy/legacy.hpp>  // for BruteForceMatcher
#include <opencv2/highgui/highgui.hpp> 
#include <Eigen/Geometry>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/eigen.h>
#include <math.h>
#include "../include/brisk/brisk.h"

class vroAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<rgbd_vro::vroAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  rgbd_vro::vroFeedback feedback_;
  rgbd_vro::vroResult result_;
  //swiss_ranger::sr4kResult* sr4k_result;
  //swiss_ranger::sr4kResultConstPtr sr4k_result;
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor; 

public:

  vroAction(std::string name) :
    as_(nh_, name, boost::bind(&vroAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();  
  }

  ~vroAction(void)
  {
  }

  void initialize()
  {
    //descriptorExtractor = new cv::BriskDescriptorExtractor();
    descriptorExtractor = new cv::SiftDescriptorExtractor();  // In order to save time for initialize a descriptors at every frame.
  }

  /**
   * @brief Measure computational time 
   * @param
   * @return
  */
  double compute_time(double t)
  {
    return 1000.0 * ((double)cv::getTickCount() - t)/cv::getTickFrequency();
  }

  /**
   * @brief Extract visual features by feature extractor with sr4k data
   * @param
   * @return
  */
  bool extract_visual_feature_kinect(std::string data_name, int data_index, int data_file_index, std::string vf_detector_name, std::string vf_descriptor_name, cv::Mat& img, cv::Mat& x, cv::Mat& y, cv::Mat& z, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, float& ct_load, float& ct_detector, float& ct_descriptor, double& depth_file_name_number)
  {

    bool success = true;
    double start_time;

    // Get depth and image data from swiss_ranger 
    start_time = (double)cv::getTickCount();
    
    actionlib::SimpleActionClient<rgbd_kinect::kinectAction> ac_kinect("ms_kinect",true);
    ac_kinect.waitForServer(); 
    

    rgbd_kinect::kinectGoal kinect_goal;
    kinect_goal.data_name = data_name; //goal->data_name;  // data set name
    kinect_goal.dir_index = data_index; //goal->data_index; // directory index (i.e. dx)
    kinect_goal.file_index = data_file_index; //1;   //file index (i.e. dx_0001);
    ac_kinect.sendGoal(kinect_goal);    

    //wait for the action to return
    bool finished_before_timeout = ac_kinect.waitForResult(ros::Duration(30.0));
    rgbd_kinect::kinectResultConstPtr kinect_result;
    //boost::shared_ptr<const swiss_ranger::kinectResult> kinect_result;

    if (finished_before_timeout)
      {
	actionlib::SimpleClientGoalState state = ac_kinect.getState();
	//ROS_INFO("Action finished: %s",state.toString().c_str());
	kinect_result = ac_kinect.getResult();
	depth_file_name_number = kinect_result->depth_file_name_number;
      }
    else
      {
	ROS_INFO("Action did not finish before the time out.");
	success = false;
      }

    // Extract features
    if(success)
      {

	//std::cout << "Image data : " << kinect_result->image.size() << std::endl;
      
	//Convert vector to cv::Mat
	cv::Size image_size(640,480);  //Size(with, height)
	//cv::Mat img(image_size, CV_8UC1);
	//cv::Mat x(image_size, CV_32FC1);
	//cv::Mat y(image_size, CV_32FC1);
	//cv::Mat z(image_size, CV_32FC1);
	//cv::Mat c(image_size, CV_8UC1);

	for(int i = 0; i < image_size.height; i++)
	  {
	    for(int j = 0; j < image_size.width; j++)
	      {
		img.at<unsigned char>(i,j)=kinect_result->image[i*image_size.width+j];
		x.at<float>(i,j)=kinect_result->x[i*image_size.width+j];
		y.at<float>(i,j)=kinect_result->y[i*image_size.width+j];
		z.at<float>(i,j)=kinect_result->z[i*image_size.width+j];
	      }
	  }
	//kinect_result->image.clear();
	//kinect_result->x.clear();
	//kinect_result->y.clear();
	//kinect_result->z.clear();
	//kinect_result->c.clear();

	ct_load = (float)compute_time(start_time);
	//std::cout<<"Load image time : " << ct_load <<" [msec]" << std::endl;

	
	//debug
	//std::cout << z.at<float>(240,320) << std::endl;
	//std::cout << x.at<float>(0,0) << std::endl;
	//std::cout << y.at<float>(0,0) << std::endl;
	//std::cout << img.at<unsigned char>(0,0) << std::endl;
	//std::cout << c.at<unsigned char>(0,0) << std::endl;
	//ROS_INFO("%d\n",img.at<unsigned char>(88,72));
	//ROS_INFO("%d\n",c.at<unsigned char>(0,0));

	//cv::Mat temp_img;
	//cv::Mat temp_img_color;
	//temp_img_color = cv::imread("/home/viki/soonhac/soonhac_windows/images/matlab_pitch_1_1.png");
	//cv::cvtColor(temp_img_color, img, CV_BGR2GRAY);

	// Show data
	//std::cout << "Show data" << std::endl;
	//cv::namedWindow("Intensity image",CV_WINDOW_AUTOSIZE);
	//cv::imshow("Intensity image", img);
	//cv::waitKey();
	//cv::destroyWindow("Intensity image");
	//char temp_name[1024];
	//sprintf(temp_name,"Intensity_%s_%d_%d.png",data_name.c_str(),data_index,data_file_index);
	//cv::imwrite(temp_name,img);
	//sprintf(temp_name,"dense_%s_%d_%d.png",data_name.c_str(),data_index,data_file_index);
	//cv::imwrite(temp_name,z);
	//cv::imwrite("x.png",x);
	//cv::imwrite("y.png",y);
	//cv::imwrite("z.png",z);
	//cv::imwrite("c.png",c);

	//Call feature extractor
	cv::Ptr<cv::FeatureDetector> detector;
	//std::vector<cv::KeyPoint> keypoints, keypoints2;

	//std::cout << "Feature Detector : " << vf_detector_name << std::endl;

	start_time = (double)cv::getTickCount();

	if(vf_detector_name.compare("SURF") == 0)
	  {
	    int threshold = 400;
	    detector = new cv::SurfFeatureDetector(threshold);
	  }
	else if(vf_detector_name.compare("BRISK") == 0)
	  {
	    int threshold = 60;
	    detector = new cv::BriskFeatureDetector(threshold,4);
	  }
	else  // default : SIFT
	  {
	    float threshold = 0.01; // / 3 / 2.0;
	    float edgeThreshold=10.0; //default : 10.0 //4.0; //atof(argv[3]+4);
	    detector = new cv::SiftFeatureDetector(threshold,edgeThreshold);
	  }

	// debug
	//std::vector<cv::KeyPoint> temp_keypoints;
	//cv::Mat imgGray;
	//cv::cvtColor(img,imgGray,CV_BGR2GRAY);
	detector->detect(img,keypoints);
      
	//t = 1000 * ((double)cv::getTickCount() - t)/cv::getTickFrequency();
	ct_detector = (float)compute_time(start_time);
	//std::cout<<"Detection time : " << ct_detector <<" [msec]" << std::endl;
	//std::cout << "Key point : " <<keypoints.size() << std::endl;
      
	// Descriptor
	//cv::Ptr<cv::DescriptorExtractor> descriptorExtractor; 
	//cv::Mat descriptors, descriptors2;
	//std::cout << "Feature Descriptor : " << vf_descriptor_name << std::endl;

	start_time = (double)cv::getTickCount();

	if(vf_descriptor_name.compare("SURF") == 0)
	  {
	    descriptorExtractor = new cv::SurfDescriptorExtractor();
	  }
	else if(vf_descriptor_name.compare("BRISK") == 0)
	  {
	    //descriptorExtractor = new cv::BriskDescriptorExtractor();
	  }
	else  // default : SIFT
	  {
	    //descriptorExtractor = new cv::SiftDescriptorExtractor();
	  }
	
	//cv::Mat temp_descriptors;
	descriptorExtractor->compute(img,keypoints,descriptors);

	//t = 1000 * ((double)cv::getTickCount() - t)/cv::getTickFrequency();
	ct_descriptor = compute_time(start_time);
	//std::cout<<"Descriptors time : " << ct_descriptor <<" [msec]" << std::endl;
	
	//imgGray.release();
      }

    return success;
  }


  /**
   * @brief Extract visual features by feature extractor with sr4k data
   * @param
   * @return
  */
  bool extract_visual_feature_sr4k(std::string data_name, int data_index, int data_file_index, std::string vf_detector_name, std::string vf_descriptor_name, cv::Mat& img, cv::Mat& x, cv::Mat& y, cv::Mat& z, cv::Mat& c, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, float& ct_load, float& ct_detector, float& ct_descriptor)
  {

    bool success = true;
    double start_time;

    // Get depth and image data from swiss_ranger 
    start_time = (double)cv::getTickCount();
    
    actionlib::SimpleActionClient<swiss_ranger::sr4kAction> ac_sr4k("swiss_ranger",true);
    ac_sr4k.waitForServer(); 
    

    swiss_ranger::sr4kGoal sr4k_goal;
    sr4k_goal.data_name = data_name; //goal->data_name;  // data set name
    sr4k_goal.dir_index = data_index; //goal->data_index; // directory index (i.e. dx)
    sr4k_goal.file_index = data_file_index; //1;   //file index (i.e. dx_0001);
    ac_sr4k.sendGoal(sr4k_goal);    

    //wait for the action to return
    bool finished_before_timeout = ac_sr4k.waitForResult(ros::Duration(30.0));
    swiss_ranger::sr4kResultConstPtr sr4k_result;
    //boost::shared_ptr<const swiss_ranger::sr4kResult> sr4k_result;

    if (finished_before_timeout)
      {
	actionlib::SimpleClientGoalState state = ac_sr4k.getState();
	//ROS_INFO("Action finished: %s",state.toString().c_str());
	sr4k_result = ac_sr4k.getResult();
      }
    else
      {
	ROS_INFO("Action did not finish before the time out.");
	success = false;
      }

    // Extract features
    if(success)
      {

	//std::cout << "Image data : " << sr4k_result->image.size() << std::endl;
      
	//Convert vector to cv::Mat
	cv::Size image_size(176,144);  //Size(with, height)
	//cv::Mat img(image_size, CV_8UC1);
	//cv::Mat x(image_size, CV_32FC1);
	//cv::Mat y(image_size, CV_32FC1);
	//cv::Mat z(image_size, CV_32FC1);
	//cv::Mat c(image_size, CV_8UC1);

	for(int i = 0; i < image_size.height; i++)
	  {
	    for(int j = 0; j < image_size.width; j++)
	      {
		img.at<unsigned char>(i,j)=sr4k_result->image[i*image_size.width+j];
		x.at<float>(i,j)=sr4k_result->x[i*image_size.width+j];
		y.at<float>(i,j)=sr4k_result->y[i*image_size.width+j];
		z.at<float>(i,j)=sr4k_result->z[i*image_size.width+j];
		c.at<unsigned char>(i,j)=sr4k_result->c[i*image_size.width+j];
	      }
	  }
	//sr4k_result->image.clear();
	//sr4k_result->x.clear();
	//sr4k_result->y.clear();
	//sr4k_result->z.clear();
	//sr4k_result->c.clear();

	ct_load = (float)compute_time(start_time);
	//std::cout<<"Load image time : " << ct_load <<" [msec]" << std::endl;

	
	//debug
	//std::cout << z.at<float>(0,0) << std::endl;
	//std::cout << x.at<float>(0,0) << std::endl;
	//std::cout << y.at<float>(0,0) << std::endl;
	//std::cout << img.at<unsigned char>(0,0) << std::endl;
	//std::cout << c.at<unsigned char>(0,0) << std::endl;
	//ROS_INFO("%d\n",img.at<unsigned char>(88,72));
	//ROS_INFO("%d\n",c.at<unsigned char>(0,0));

	//cv::Mat temp_img;
	//cv::Mat temp_img_color;
	//temp_img_color = cv::imread("/home/viki/soonhac/soonhac_windows/images/matlab_pitch_1_1.png");
	//cv::cvtColor(temp_img_color, img, CV_BGR2GRAY);

	// Show data
	//std::cout << "Show data" << std::endl;
	//cv::namedWindow("Intensity image",CV_WINDOW_AUTOSIZE);
	//cv::imshow("Intensity image", img);
	//cv::waitKey();
	//cv::destroyWindow("Intensity image");
	//char temp_name[1024];
	//sprintf(temp_name,"Intensity_%s_%d_%d.png",data_name.c_str(),data_index,data_file_index);
	//cv::imwrite(temp_name,img);
	//cv::imwrite("x.png",x);
	//cv::imwrite("y.png",y);
	//cv::imwrite("z.png",z);
	//cv::imwrite("c.png",c);

	//Call feature extractor
	cv::Ptr<cv::FeatureDetector> detector;
	//std::vector<cv::KeyPoint> keypoints, keypoints2;

	//std::cout << "Feature Detector : " << vf_detector_name << std::endl;

	start_time = (double)cv::getTickCount();

	if(vf_detector_name.compare("SURF") == 0)
	  {
	    int threshold = 400;
	    detector = new cv::SurfFeatureDetector(threshold);
	  }
	else if(vf_detector_name.compare("BRISK") == 0)
	  {
	    int threshold = 60;
	    detector = new cv::BriskFeatureDetector(threshold,4);
	  }
	else  // default : SIFT
	  {
	    float threshold = 0.01; // / 3 / 2.0;
	    float edgeThreshold = 10.0; //10.0; //default : 10.0 //4.0; //atof(argv[3]+4);
	    detector = new cv::SiftFeatureDetector(threshold,edgeThreshold);
	  }

	// debug
	//std::vector<cv::KeyPoint> temp_keypoints;
	//cv::Mat imgGray;
	//cv::cvtColor(img,imgGray,CV_BGR2GRAY);
	detector->detect(img,keypoints);
      
	//t = 1000 * ((double)cv::getTickCount() - t)/cv::getTickFrequency();
	ct_detector = (float)compute_time(start_time);
	//std::cout<<"Detection time : " << ct_detector <<" [msec]" << std::endl;
	std::cout << "Key point : " <<keypoints.size() << std::endl;
      
	// Descriptor
	//cv::Ptr<cv::DescriptorExtractor> descriptorExtractor; 
	//cv::Mat descriptors, descriptors2;
	//std::cout << "Feature Descriptor : " << vf_descriptor_name << std::endl;

	start_time = (double)cv::getTickCount();

	if(vf_descriptor_name.compare("SURF") == 0)
	  {
	    descriptorExtractor = new cv::SurfDescriptorExtractor();
	  }
	else if(vf_descriptor_name.compare("BRISK") == 0)
	  {
	    //descriptorExtractor = new cv::BriskDescriptorExtractor();
	  }
	else  // default : SIFT
	  {
	    //descriptorExtractor = new cv::SiftDescriptorExtractor();
	  }
	
	//cv::Mat temp_descriptors;
	descriptorExtractor->compute(img,keypoints,descriptors);

	//t = 1000 * ((double)cv::getTickCount() - t)/cv::getTickFrequency();
	ct_descriptor = compute_time(start_time);
	//std::cout<<"Descriptors time : " << ct_descriptor <<" [msec]" << std::endl;
	
	//imgGray.release();
      }

    return success;
  }

  /**
   * @brief Run RANSAC with two 3D point sets
   * @param
   * @return
  */
  void do_ransac(std::vector<Eigen::Vector3f> keypoint1_3d, std::vector<Eigen::Vector3f>keypoint2_3d, const unsigned int sample_size, Eigen::Matrix4f& h_transform, unsigned int& inlier_size)
  {
    // Select initial points
    std::set<unsigned int> sample_matches;
    //std::vector<unsigned int> sample_matches_vector;
    while (sample_matches.size() < sample_size)
    {
      int id = rand() % keypoint1_3d.size();
      sample_matches.insert(id);
      //sample_matches_vector.push_back (initial_matches->at (id));
    }
    //std::cout << "Sample matches : " << sample_matches << std::endl;
    
    // Compute transformation matrix from sample data
    pcl::TransformationFromCorrespondences tfc;
    //std::vector<Eigen::Vector3f> t, f;
    double distance_min = 10.0;  // [m]
    double error_threshold = 0.0;
    std::set<unsigned int>::iterator index;
    const double error_threshold_factor = 0.0001;

    //for (int i = 0 ; i < sample_size; i++)
    for(index = sample_matches.begin(); index != sample_matches.end(); index++)
      {	
	//std::set<unsigned int>::iterator index = sample_matches.find(i);
	//std::cout << "Sample index : " << *index << std::endl;

	Eigen::Vector3f kp1 (keypoint1_3d[*index][0], keypoint1_3d[*index][1], keypoint1_3d[*index][2]);
	Eigen::Vector3f kp2 (keypoint2_3d[*index][0], keypoint2_3d[*index][1], keypoint2_3d[*index][2]);

	//f.push_back (from);
	//t.push_back (to);

	tfc.add (kp2, kp1, 1.0); // / kp2[2]); //use weight according to distance
	//tfc.add (kp1, kp2, 1.0); // / kp2[2]); //use weight according to distance

	if(kp2[2] < distance_min)
	  {
	    distance_min = kp2[2];
	    //error_threshold = kp2.norm()*0.001;
	    error_threshold = kp2.squaredNorm()*error_threshold_factor;
	  }
      }

    h_transform = tfc.getTransformation().matrix();
    //std::cout << "Current transform : " << h_transform << std::endl;
    //std::cout << "Ransac threshold : " << error_threshold << std::endl;

    // Find inliers
    std::vector<unsigned int> inlier;

    for(int i = 0 ; i < keypoint1_3d.size(); i++)
      {
	Eigen::Vector4f point1 (keypoint1_3d[i][0], keypoint1_3d[i][1], keypoint1_3d[i][2], 1);
	Eigen::Vector4f point2 (keypoint2_3d[i][0], keypoint2_3d[i][1], keypoint2_3d[i][2], 1);
	Eigen::Vector4f diff = (h_transform * point2) - point1;  
	Eigen::Vector3f diff_3d(diff[0], diff[1], diff[3]);
	double error = diff_3d.squaredNorm();
	//double error = diff_3d.norm();

	//std::cout << "Error distance : " << error << std::endl;
	if(error < error_threshold)
	  {
	    inlier.push_back(i);
	  }
      }

    inlier_size = inlier.size();

    inlier.clear();
    //std::cout << "Inlier size : " << inlier_size << std::endl;
    //std::cout << "Error threshold : "  << error_threshold << std::endl;

  }
  
  /**
   * @brief Compute relative transformation by RANSAC
   * @param
   * @return
  */
  void relative_transform_by_ransac(std::vector<Eigen::Vector3f> keypoint1_3d, std::vector<Eigen::Vector3f>keypoint2_3d, Eigen::Matrix4f& h_transform_optimal, unsigned int& ransac_match_size, unsigned int& ransac_error)
  {
    //bool converged = false;
    unsigned int iteration_count = 0;
    const float target_outlier_tolerance = 0.01;  // 99% confidence of outlier-free
    const unsigned int sample_size = 4;
    const unsigned int data_size = keypoint1_3d.size();
    float current_outlier_tolerance = 1;   // (1 - (sample_num/data_size)^sample_num)^iteration_count
    //std::vector<unsigned int> inlier_size_list;
    //std::vector<Eigen::Matrix4f> h_transform_list;
    unsigned int inlier_size_max = 0;
    const unsigned int iteration_count_max = 120000;

    while(current_outlier_tolerance > target_outlier_tolerance && iteration_count < iteration_count_max)
      //while(iteration_count < iteration_count_max)
      {
	unsigned int inlier_size=0;
	Eigen::Matrix4f h_transform;
	
	//std::cout << "------------- RANSAC Iteration : " << iteration_count << std::endl;
	do_ransac(keypoint1_3d, keypoint2_3d, sample_size, h_transform, inlier_size);
	//inlier_size_list.push_back(inlier_size);
	//h_transform_list.push_back(h_transform);
	if(inlier_size >= inlier_size_max)
	  {
	    inlier_size_max = inlier_size;
	    h_transform_optimal = h_transform;
	  }

	iteration_count++;
	current_outlier_tolerance = pow((double)(1 - pow((double)inlier_size/(double)data_size, sample_size)),iteration_count);
	//std::cout << "RANSAC current outlier tolerance : " << current_outlier_tolerance << std::endl;
	//std::cout << "Current Transformation : " << h_transform << std::endl;
      }

    if(inlier_size_max <= sample_size)
      {
	ransac_error = 1;
      }
    else if(iteration_count >= iteration_count_max)
      {
	ransac_error = 2;
      }
    ransac_match_size = inlier_size_max;
    //std::cout << "RANSAC iteration : " << iteration_count << std::endl;
    //std::cout << "Inliers : " << inlier_size_max << std::endl;
  }
  
  /**
   * @brief Main service of VRO server
   * @param
   * @return
  */
  void executeCB(const rgbd_vro::vroGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;
    double start_time;
    double first_depth_file_name_number = 0.0;
    double second_depth_file_name_number = 0.0;

    // push_back the seeds for the vro sequence
    //feedback_.sequence.clear();
    //feedback_.sequence.push_back(0);
    //feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("---------------------------------------------");
    //ROS_INFO("goal::data_name : %s",goal->data_name.c_str());
    ROS_INFO("goal::camera_name : %s", goal->camera_name.c_str());
    ROS_INFO("goal::data_index : %d", goal->data_index);
    ROS_INFO("goal::data_interval : %d", goal->data_interval);
    ROS_INFO("goal::first_file_index : %d", goal->first_file_index);
    ROS_INFO("goal::second_file_index : %d", goal->second_file_index);
    //ROS_INFO("%s: Executing, creating vro sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);    

    // Get visual features from first image;
    //unsigned int data_file_index = 1;
    //std::string vf_detector_name("SIFT");   //Options : SIFT, SURF, BRISK
    //std::string vf_descriptor_name("SIFT");   // Options : SIFT, SURF, BRISK
    
    unsigned int image_width;
    unsigned int image_height;

    if(goal->camera_name.compare("sr4k") == 0)
      {
	image_width = 176;
	image_height = 144;
      }
    else
      {
	image_width = 640;
	image_height = 480;
      }

    cv::Size image_size(image_width,image_height);  //Size(with, height)
    cv::Mat img1(image_size, CV_8UC1);
    cv::Mat x1(image_size, CV_32FC1);
    cv::Mat y1(image_size, CV_32FC1);
    cv::Mat z1(image_size, CV_32FC1);
    cv::Mat c1(image_size, CV_8UC1);
    std::vector<cv::KeyPoint> keypoints1;
    cv::Mat descriptors1;
    float ct_load1,ct_detector1,ct_descriptor1;

    if(goal->camera_name.compare("sr4k") == 0)
      {
	success = extract_visual_feature_sr4k(goal->data_name, goal->data_index, goal->first_file_index, goal->vf_detector_name, goal->vf_descriptor_name, img1, x1, y1, z1, c1, keypoints1, descriptors1, ct_load1, ct_detector1, ct_descriptor1);
      }
    else    //kinect
      {
	success = extract_visual_feature_kinect(goal->data_name, goal->data_index, goal->first_file_index, goal->vf_detector_name, goal->vf_descriptor_name, img1, x1, y1, z1, keypoints1, descriptors1, ct_load1, ct_detector1, ct_descriptor1, first_depth_file_name_number);
      }


    // Get visual features from second image;
    //data_file_index = 1;
    //unsigned int data_index_step = 1;

    cv::Mat img2(image_size, CV_8UC1);
    cv::Mat x2(image_size, CV_32FC1);
    cv::Mat y2(image_size, CV_32FC1);
    cv::Mat z2(image_size, CV_32FC1);
    cv::Mat c2(image_size, CV_8UC1);
    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors2;
    float ct_load2,ct_detector2,ct_descriptor2;

    if(goal->camera_name.compare("sr4k") == 0)
      {
	success = extract_visual_feature_sr4k(goal->data_name, goal->data_index + goal->data_interval, goal->second_file_index, goal->vf_detector_name, goal->vf_descriptor_name, img2, x2, y2, z2, c2, keypoints2, descriptors2, ct_load2, ct_detector2, ct_descriptor2);
      }
    else
      {
	success = extract_visual_feature_kinect(goal->data_name, goal->data_index, goal->second_file_index, goal->vf_detector_name, goal->vf_descriptor_name, img2, x2, y2, z2, keypoints2, descriptors2, ct_load2, ct_detector2, ct_descriptor2, second_depth_file_name_number);
      }

    //std::cout << "Descriptor 1 : " << descriptors1.size().width <<","<< descriptors1.size().height << std::endl;
    //std::cout << "Descriptor 2 : " << descriptors2.size().width <<","<< descriptors1.size().height << std::endl;
    ////////////////////////////////////////////////////////
    // Match visual features
    start_time = (double)cv::getTickCount();
    cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
    std::vector<std::vector<cv::DMatch> > matches;
    //std::vector<cv::DMatch> matches;
    float ct_match;
    
    if(goal->vf_descriptor_name.compare("BRISK") == 0)
      {
	descriptorMatcher = new cv::BruteForceMatcher<cv::Hamming>();
      }
    else  // default : SIFT, SURF
      {
	descriptorMatcher = new cv::BruteForceMatcher<cv::L2<float> >();
      }

    //descriptorMatcher->knnMatch(descriptors1,descriptors2,matches,2);
    descriptorMatcher->knnMatch(descriptors2,descriptors1,matches,2);
    //descriptorMatcher->match(descriptors1,descriptors2,matches);
    
    //Check distance ratio
    std::vector<cv::DMatch> good_matches;
    float distance_ratio_threshold = 1.0;
    for(int i = 0; i < matches.size(); i++)
      {
	cv::DMatch first_match = matches[i][0];
	cv::DMatch second_match = matches[i][1];
	
	float distance_ratio = first_match.distance / second_match.distance;
	if(distance_ratio < distance_ratio_threshold)
	  {
	    good_matches.push_back(first_match);
	  }
      }

    ct_match = (float)compute_time(start_time); 
    //std::cout<<"Match time : " << ct_match <<" [msec]" << std::endl;
    //std::cout<<"Match points : " << matches.size() << std::endl;
    //std::cout<<"Good Match points : " << good_matches.size() << std::endl;
   
    
    /*
    // drawing matches
    cv::Mat outimg;
    cv::drawMatches(img2, keypoints2, img1, keypoints1, matches, outimg,
		cv::Scalar(0,255,0), cv::Scalar(0,0,255),
		std::vector<std::vector<char> >(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	
    std::cout << "Show result image" << std::endl;
    cv::namedWindow("Matches");
    cv::imshow("Matches", outimg);
    cv::waitKey();
    */

    ////////////////////////////////////////////////////////
    // Compute relative transformation by RANSAC
    std::vector<Eigen::Vector3f> keypoint1_3d, keypoint2_3d;
    //Eigen::Vector3f unit_point;
    int row1, col1, row2, col2;
    std::vector<cv::DMatch> robust_matches;
    const unsigned int intensity_threshold = 50; //100;
    Eigen::Matrix4f h_transform;
    float ct_ransac;
    unsigned int ransac_match_size;
    unsigned int ransac_error = 0 ;
    unsigned int match_size_min = 4;
    //pcl::TransformationFromCorrespondences tfc;

    start_time = (double)cv::getTickCount();
    // Make 3d point sets by qualifying with intensity threshold b/c low intensity has unstable measurement.
    for(int i = 0 ; i < good_matches.size() ; i++)
      {
	//col1 = floor(keypoints1[matches[i][0].queryIdx].pt.x + 0.5); // round()
	//row1 = floor(keypoints1[matches[i][0].queryIdx].pt.y + 0.5);  // round()
	//col2 = floor(keypoints2[matches[i][0].trainIdx].pt.x + 0.5); // round()
	//row2 = floor(keypoints2[matches[i][0].trainIdx].pt.y + 0.5);  // round()
	//col1 = floor(keypoints1[matches[i][0].trainIdx].pt.x + 0.5); // round()
	//row1 = floor(keypoints1[matches[i][0].trainIdx].pt.y + 0.5);  // round()
	//col2 = floor(keypoints2[matches[i][0].queryIdx].pt.x + 0.5); // round()
	//row2 = floor(keypoints2[matches[i][0].queryIdx].pt.y + 0.5);  // round()
	col1 = floor(keypoints1[good_matches[i].trainIdx].pt.x + 0.5); // round()
	row1 = floor(keypoints1[good_matches[i].trainIdx].pt.y + 0.5);  // round()
	col2 = floor(keypoints2[good_matches[i].queryIdx].pt.x + 0.5); // round()
	row2 = floor(keypoints2[good_matches[i].queryIdx].pt.y + 0.5);  // round()
	//col1 = floor(keypoints1[matches[i].queryIdx].pt.x + 0.5); // round()
	//row1 = floor(keypoints1[matches[i].queryIdx].pt.y + 0.5);  // round()
	//col2 = floor(keypoints2[matches[i].trainIdx].pt.x + 0.5); // round()
	//row2 = floor(keypoints2[matches[i].trainIdx].pt.y + 0.5);  // round()

	//debug
	//std::cout << "point 1 = [" << row1 << ", " << col1 << "], point2 = [" << row2 << ", " << col2 << "]" << std::endl;

	if(goal->camera_name.compare("sr4k"))
	  {
	    if(img1.at<unsigned char>(row1,col1) >= intensity_threshold 
	       && img2.at<unsigned char>(row2,col2) >= intensity_threshold)
	      {
		Eigen::Vector3f point1(-x1.at<float>(row1,col1), z1.at<float>(row1,col1), y1.at<float>(row1,col1)); //Change coordinates from sr4k to a robot
		keypoint1_3d.push_back(point1);

		Eigen::Vector3f point2(-x2.at<float>(row2,col2), z2.at<float>(row2,col2), y2.at<float>(row2,col2));
		keypoint2_3d.push_back(point2);

		//robust_matches.push_back(matches[i][0]);
		robust_matches.push_back(good_matches[i]);
	      }
	  }
	  else   // kinect
	    {
	      if(z1.at<unsigned char>(row1,col1) > 0 
		 && z2.at<unsigned char>(row2,col2) > 0)
		{
		  Eigen::Vector3f point1(x1.at<float>(row1,col1), z1.at<float>(row1,col1), -y1.at<float>(row1,col1)); //Change coordinates from sr4k to a robot
		  //Eigen::Vector3f point1(x1.at<float>(row1,col1), y1.at<float>(row1,col1), z1.at<float>(row1,col1)); //Change coordinates from sr4k to a robot
		  keypoint1_3d.push_back(point1);

		  Eigen::Vector3f point2(x2.at<float>(row2,col2), z2.at<float>(row2,col2), -y2.at<float>(row2,col2));
		  //Eigen::Vector3f point2(x2.at<float>(row2,col2), y2.at<float>(row2,col2), z2.at<float>(row2,col2));
		  keypoint2_3d.push_back(point2);

		  //robust_matches.push_back(matches[i][0]);
		  robust_matches.push_back(good_matches[i]);
		}
	    }
      }

    //std::cout << "Robust matches : " << robust_matches.size() << std::endl;
    
    float tx = 0, ty = 0, tz = 0, rx = 0, ry = 0, rz = 0;
    float rx_degree = 0, ry_degree = 0, rz_degree = 0;
    const float PI = 3.141592;

    if(robust_matches.size() > match_size_min)
      {
	start_time = (double)cv::getTickCount();

	relative_transform_by_ransac(keypoint1_3d, keypoint2_3d, h_transform, ransac_match_size, ransac_error);

	ct_ransac = compute_time(start_time);
	//std::cout<<"RANSAC time : " << ct_ransac <<" [msec]" << std::endl;
	//std::cout << "Optimal Transformation : \n" << h_transform << std::endl;

	pcl::getTranslationAndEulerAngles(Eigen::Affine3f(h_transform), tx, ty, tz, rx, ry, rz);
	//std::cout << "[rx, ry, rz][radian] : "  << rx <<", " << ry <<", " << rz << std::endl;
	rx_degree = rx * 180 / PI;
	ry_degree = ry * 180 / PI;
	rz_degree = rz * 180 / PI;
      }
    else
      {
	//std::cout << "Number of visual matches, " << robust_matches.size() << ", is too small to run RANSAC." << std::endl;
	ransac_error = 1;
      }

    std::cout << "[Motion Estimation [x,y,z,rx,ry,rz] [m][degree]]" << std::endl;
    std::cout << tx <<", "<< ty << ", " << tz << ", " << rx_degree << ", " << ry_degree << ", " << rz_degree << std::endl;

    //as_.publishFeedback(feedback_);

    // start executing the action
    /*for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
      }*/

    if(success)
    {
      //result_.sequence = feedback_.sequence;
      result_.translation.clear();
      result_.orientation.clear();
      result_.comp_time.clear();
      result_.match_num.clear();
      
      result_.translation.push_back(tx);
      result_.translation.push_back(ty);
      result_.translation.push_back(tz);
      result_.orientation.push_back(rx);
      result_.orientation.push_back(ry);
      result_.orientation.push_back(rz);
      result_.comp_time.push_back(ct_load1);
      result_.comp_time.push_back(ct_detector1);
      result_.comp_time.push_back(ct_descriptor1);
      result_.comp_time.push_back(ct_load2);
      result_.comp_time.push_back(ct_detector2);
      result_.comp_time.push_back(ct_descriptor2);
      result_.comp_time.push_back(ct_match);
      result_.comp_time.push_back(ct_ransac);
      result_.match_num.push_back(robust_matches.size());
      result_.match_num.push_back(ransac_match_size);
      result_.error = ransac_error;
      result_.depth_file_name_number = first_depth_file_name_number;

      //ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }

    // Clear memory
    img1.release();
    x1.release();
    y1.release();
    z1.release();
    c1.release();
    descriptors1.release();
    img2.release();
    x2.release();
    y2.release();
    z2.release();
    c2.release();
    descriptors2.release();
  }  

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd_vro");

  ROS_INFO("VRO server is working.");
  vroAction rgbd_vro(ros::this_node::getName());
  rgbd_vro.initialize();

  ros::spin();

  return 0;
}
