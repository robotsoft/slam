/**
 * SwissRanger Server
 *
 * Created on : 10/30/2012
 * Author : Soonhac Hong (sxhong1@ualr.edu)
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <swiss_ranger/sr4kAction.h>
#include "load_sr4k.h"
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>

class sr4kAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<swiss_ranger::sr4kAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  swiss_ranger::sr4kFeedback feedback_;
  swiss_ranger::sr4kResult result_;

public:

  sr4kAction(std::string name) :
    as_(nh_, name, boost::bind(&sr4kAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~sr4kAction(void)
  {
  }

  // Get full data file name including its path
  const char* get_data_file_name(const std::string data_name, const int dir_index, const int file_index)
  {
    char data_file_name[1024];

    //debug
    //std::cout<< data_name << std::endl;

    if(data_name.compare("pitch") == 0)
      {
	sprintf(data_file_name,"/home/viki/soonhac/data/pitch_3_interval/d%d_%d/d%d_%04d.dat",dir_index,-40+(dir_index-1)*3, dir_index, file_index);
      }
    else if(data_name.compare("pitch4") == 0)
      {
	sprintf(data_file_name,"/home/viki/soonhac/data/pitch4_3_interval_vfless/d%d_%d/d%d_%04d.dat",dir_index,-50+(dir_index-1)*3, dir_index, file_index);
      }
    else if(data_name.compare("pan") == 0)
      {
	sprintf(data_file_name,"/home/viki/soonhac/data/pan_3_interval/d%d_%d/d%d_%04d.dat",dir_index,44-(dir_index-1)*3, dir_index, file_index);
      }
    else if(data_name.compare("pan3") == 0)
      {
	sprintf(data_file_name,"/home/viki/soonhac/data/pan3_3_interval_vfless/d%d_%d/d%d_%04d.dat",dir_index,21-(dir_index-1)*3, dir_index, file_index);
      }
    else if(data_name.compare("etas") == 0)
      {
	char dir_name[6][16]={"3th_straight","3th_swing","4th_straight","4th_swing","5th_straight","5th_swing"};
	sprintf(data_file_name,"/home/viki/soonhac/data/etas/%s/d1_%04d.dat",dir_name[dir_index-1], file_index);
      }
    else
      {
	std::cout << "Data name is not recognized !!!" <<  std::endl;
	sprintf(data_file_name,"none");
      }

    return (const char*) data_file_name;
  }

  /*void get_src(int row, int col, cv::Mat& src, cv::Mat original)
  {
    for(int i = 0; i < 3; i++)
      {
	for(int j = 0; j< 3; j++)
	  {
	    if(row == 0 && col == 0)
	      {
		src.at<float>(i,j) = original.at<float>(row,col+j-1);
	      }
	    else if(col == 0)
	      {
	      }
	    else if(row == original.Size().height)
	      {
	      }
	    else if(col == original.Size().width)
	      {
	      }
	    else
	      {
		src.at<float>(i,j) = original.at<float>(row+i-1,col+j-1);
	      }
	  }
      }
      }*/

  void executeCB(const swiss_ranger::sr4kGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the sr4k sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    //ROS_INFO("goal::data_name : %s",goal->data_name.c_str());
    //ROS_INFO("goal::dir_index : %d", goal->dir_index);
    //ROS_INFO("goal::file_index : %d", goal->file_index);
    //ROS_INFO("%s: Executing, creating sr4k sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // Get full file name including path
    const char* data_file_name = get_data_file_name(goal->data_name, goal->dir_index, goal->file_index);
    bool is_confidence = true;
    float image_max = 0.0;

    // Get depth and image data

    LoadSR4K sr4k;
    cv::Size image_size(176,144);  //Size(with, height)
    cv::Mat img(image_size, CV_32FC1);
    cv::Mat x(image_size, CV_32FC1);
    cv::Mat y(image_size, CV_32FC1);
    cv::Mat z(image_size, CV_32FC1);
    cv::Mat c(image_size, CV_8UC1);

    if(sr4k.load_sr4k(data_file_name, img, x, y, z, c, is_confidence))
      {

	//debug
	//std::cout << z.at<float>(0,0) << std::endl;
	//std::cout << x.at<float>(0,0) << std::endl;
	//std::cout << y.at<float>(0,0) << std::endl;
	//std::cout << img.at<float>(0,0) << std::endl;
	//std::cout << c.at<unsigned char>(0,0) << std::endl;
	//ROS_INFO("%d\n",c.at<unsigned char>(0,0));

	//std::cout << "Show data" << std::endl;
	//cv::namedWindow("Intensity image",CV_WINDOW_AUTOSIZE);
	//cv::imshow("Intensity image", img);

	// Make medain-filtered data for compesnation of badpixel.
	cv::Mat img_median, x_median, y_median, z_median;

	cv::medianBlur(img,img_median,3);
	cv::medianBlur(x,x_median,3);
	cv::medianBlur(y,y_median,3);
	cv::medianBlur(z,z_median,3);

	// Compensate for badpixel according to confidence
	// Find the maximum value of image
	for(int i = 0; i < image_size.height; i++)
	  {
	    for(int j = 0 ; j < image_size.width; j++)
	      {
		if(c.at<unsigned char>(i,j) < 1)
		  {
		    img.at<float>(i,j) = img_median.at<float>(i,j);
		    x.at<float>(i,j) = x_median.at<float>(i,j);
		    y.at<float>(i,j) = y_median.at<float>(i,j);
		    z.at<float>(i,j) = z_median.at<float>(i,j);
		  }
		if(img.at<float>(i,j) > image_max)
		  {
		    image_max = img.at<float>(i,j);
		  }
	      }
	  }

	//std::cout << "Image maximum : " << image_max << std::endl;
	

	// Scale down image to 0 ~ 255
	cv::Mat img_scale(image_size, CV_8UC1);

	for(int i = 0; i < image_size.height; i++)
	  {
	    for(int j = 0 ; j < image_size.width; j++)
	      {
		img_scale.at<unsigned char>(i,j) = (unsigned char)(sqrt(img.at<float>(i,j))*255.0/sqrt(image_max));
	      }
	  }
	
	// Apply Gaussian filter
	cv::Mat img_gaussian, x_gaussian, y_gaussian, z_gaussian;

	cv::GaussianBlur(img_scale,img_gaussian,cv::Size(3,3),1,1);
	cv::GaussianBlur(x,x_gaussian,cv::Size(3,3),1,1);
	cv::GaussianBlur(y,y_gaussian,cv::Size(3,3),1,1);
	cv::GaussianBlur(z,z_gaussian,cv::Size(3,3),1,1);
	
	// Publish final data
	result_.image.clear();
	result_.x.clear();
	result_.y.clear();
	result_.z.clear();
	result_.c.clear();

	result_.image.reserve(image_size.height*image_size.width);
	result_.x.reserve(image_size.height*image_size.width);
	result_.y.reserve(image_size.height*image_size.width);
	result_.z.reserve(image_size.height*image_size.width);
	result_.c.reserve(image_size.height*image_size.width);

	for(int i = 0; i < image_size.height; i++)
	  {
	    for(int j = 0 ; j < image_size.width; j++)
	      {
		result_.image.push_back(img_gaussian.at<unsigned char>(i,j)); 
		result_.x.push_back(x_gaussian.at<float>(i,j));
		result_.y.push_back(y_gaussian.at<float>(i,j));
		result_.z.push_back(z_gaussian.at<float>(i,j));
		result_.c.push_back(c.at<unsigned char>(i,j));
	      }
	  }

	//std::cout << "Image data size : " << result_.image.size() << std::endl;
	//char temp_name[1024];
	//sprintf(temp_name,"Intensity_%s_%d_%d.png",goal->data_name.c_str(), goal->dir_index, goal->file_index);
	//cv::imwrite(temp_name,img_gaussian);

	img_median.release();
	x_median.release();
	y_median.release();
	z_median.release();
	img_gaussian.release();
	x_gaussian.release();
	y_gaussian.release();
	z_gaussian.release();
	img_scale.release();
	
      }
    else
      {
	std::cout << "Loading data is failed.!!" << std::endl;
	success=false;
      }


    

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
      //ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }

    img.release();
    x.release();
    y.release();
    z.release();
    c.release();
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "swiss_ranger");

  ROS_INFO("SR4K server is working.");
  sr4kAction swiss_ranger(ros::this_node::getName());
  ros::spin();

  return 0;
}
