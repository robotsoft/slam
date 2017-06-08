/**
 * MS Kinect Server
 *
 * Created on : 12/13/2012
 * Author : Soonhac Hong (sxhong1@ualr.edu)
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <rgbd_kinect/kinectAction.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <dirent.h>

class kinectAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<rgbd_kinect::kinectAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  rgbd_kinect::kinectFeedback feedback_;
  rgbd_kinect::kinectResult result_;

public:

  kinectAction(std::string name) :
    as_(nh_, name, boost::bind(&kinectAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~kinectAction(void)
  {
  }


  int getdir(char* dir, std::vector<std::string> &files)
  {
    DIR *dp;
    struct dirent *dirp;

    if((dp  = opendir(dir)) == NULL) 
      {
	std::cout << "Error(" << errno << ") opening " << dir << std::endl;
	return errno;
      }

    while ((dirp = readdir(dp)) != NULL) 
      {
	files.push_back(std::string(dirp->d_name));
      }
    closedir(dp);

    std::sort(files.begin(), files.end());

    return 0;
  }

  // Get full data file name including its path
  void get_depth_file_name(const std::string data_name, const int file_index, double &depth_file_name_number, char* data_file_name)
  {
    char dir_name[1024];
    //char data_file_name[1024];
    //debug
    //std::cout<< data_name << std::endl;

    // Get file list
    ///home/viki/soonhac/sw/rospkgs/rgbd_kinect/data
    sprintf(dir_name,"/home/viki/soonhac/data/kinect_tum/rgbd_dataset_%s/depth",data_name.c_str());
    //sprintf(dir_name,"data/rgbd_dataset_%s/rgb",data_name.c_str());
    std::vector<std::string> file_list;

    getdir(dir_name, file_list);
    std::string file_name(file_list[file_index+1].c_str());

    //std::cout << "Depth file name : " << file_name.c_str() << std::endl;
    sprintf(data_file_name,"%s/%s", dir_name, file_name.c_str());
    
    file_name.erase(file_name.end()-4, file_name.end());
    //std::cout << "Depth file name : " << file_name.c_str() << std::endl;

    depth_file_name_number=atof(file_name.c_str());
    //printf("Depth file number : %f\n",color_file_name_number);

    //debug
    //std::cout << "Color file : "  << data_file_name << std::endl;

    file_list.clear();
    //return data_file_name;
  }


  // Get full data file name including its path
  void get_color_file_name(const std::string data_name, const int file_index, double depth_file_name_number, char* data_file_name)
  {
    char dir_name[1024];
    //char data_file_name[1024];
    double color_file_name_number = 0.0;
    double color_file_name_number_previous = 0.0;
    //unsigned int depth_file_index = 0;
    std::vector<std::string> file_list;

    // Get file list
    sprintf(dir_name,"/home/viki/soonhac/data/kinect_tum/rgbd_dataset_%s/rgb",data_name.c_str());
    //sprintf(dir_name,"data/rgbd_dataset_%s/depth",data_name.c_str());

    //debug
    //std::cout<< data_name << std::endl;
    getdir(dir_name, file_list);

    for(int i = 2; i < file_list.size() ; i++)
      {
	file_list[i].erase(file_list[i].end()-4, file_list[i].end());
	color_file_name_number = atof(file_list[i].c_str());
	if(color_file_name_number > depth_file_name_number){
	  //depth_file_index = i;
	  if(i > 2)
	    {
	      //file_list[i-1].erase(file_list[i-1].end()-4, file_list[i-1].end())
	      color_file_name_number_previous = atof(file_list[i-1].c_str());
	    }
	  else
	    {
	      color_file_name_number_previous = color_file_name_number;
	    }
	  break;
	}
      }
    
    //printf("Depth file number : %f\n", depth_file_name_number);
    //printf("Previous depth file number %f\n", depth_file_name_number_previous);

    if((color_file_name_number - depth_file_name_number) > fabs(color_file_name_number_previous - depth_file_name_number))
      {
	color_file_name_number = color_file_name_number_previous;
      }

    sprintf(data_file_name,"%s/%f.png", dir_name, color_file_name_number);

    //debug
    //std::cout << "Depth file : "  << data_file_name << std::endl;

    file_list.clear();
    //return data_file_name;
  }

  void executeCB(const rgbd_kinect::kinectGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the kinect sequence
    //feedback_.sequence.clear();
    //feedback_.sequence.push_back(0);
    //feedback_.sequence.push_back(1);

    // publish info to the console for the user
    //ROS_INFO("goal::data_name : %s",goal->data_name.c_str());
    //ROS_INFO("goal::dir_index : %d", goal->dir_index);
    //ROS_INFO("goal::file_index : %d", goal->file_index);
    //ROS_INFO("%s: Executing, creating kinect sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // Get full file name including path
    double depth_file_name_number;
    char color_file_name[1024]; 
    char depth_file_name[1024]; 
    bool is_confidence = true;
    float image_max = 0.0;

    // Get depth and image data
    cv::Size image_size(640,480);  //Size(with, height)
    cv::Mat img(image_size, CV_8UC1);
    cv::Mat x(image_size, CV_32FC1);
    cv::Mat y(image_size, CV_32FC1);
    cv::Mat z(image_size, CV_32FC1);
    cv::Mat color_img(image_size, CV_8UC3);
    cv::Mat depth_img;
    //cv::Mat depth_img(image_size, CV_32FC1);
    //cv::Mat depth_img(image_size, CV_16UC1);

    //std::cout << "File index : " << goal->file_index << std::endl;
    get_depth_file_name(goal->data_name, goal->file_index, depth_file_name_number, depth_file_name);
    get_color_file_name(goal->data_name, goal->file_index, depth_file_name_number, color_file_name);
    

    //std::cout << "color file name : " << color_file_name << std::endl;
    //std::cout << "depth file name : " << depth_file_name <<  std::endl;

    color_img = cv::imread(color_file_name);
    //depth_img = cv::imread(depth_file_name, 0); //,CV_LOAD_IMAGE_ANYDEPTH);
    depth_img = cv::imread(depth_file_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR ); // Read the file
    depth_img.convertTo(depth_img, CV_32F); // convert the image data to float type 

    //Covert color image to gray image
    cv::cvtColor(color_img, img, CV_RGB2GRAY);

    //std::cout << "Converting depth into point cloud" << std::endl;
    // Generate point cloud (x, y, z)
    float fx = 525.0;
    float fy = 525.0;
    float cx = 319.5;
    float cy = 239.5;
    float ds = 1.0;
    float factor = 5000.0;
    
    //debug
    //std::cout << "Depth image : " << depth_img.at<float>(240,320) << std::endl;
    //std::cout << x.at<float>(0,0) << std::endl;
    //std::cout << y.at<float>(0,0) << std::endl;
    //std::cout << img.at<float>(0,0) << std::endl;
    //std::cout << c.at<unsigned char>(0,0) << std::endl;
    //ROS_INFO("%d\n",c.at<unsigned char>(0,0));

    //std::cout << "Show data" << std::endl;
    //cv::namedWindow("Intensity image",CV_WINDOW_AUTOSIZE);
    //cv::imshow("Intensity image", img);

    //cv::imwrite("gray_img.png",img);
    //cv::imwrite("color_img.png",color_img);

    
    //std::cout << "Image size : " << image_size.width <<", "<< image_size.height << std::endl;

    for(int i = 0; i < image_size.height; i++)
      {
	for(int j = 0 ; j < image_size.width; j++)
	  {
	    z.at<float>(i,j) = depth_img.at<float>(i,j) * ds /factor;
	    x.at<float>(i,j) = (j - cx) * z.at<float>(i,j) / fx;
	    y.at<float>(i,j) = (i - cy) * z.at<float>(i,j) / fy;
	    //std::cout << z.at<float>(i,j) << std::endl;
	    //std::cout << x.at<float>(i,j) << std::endl;
	    //std::cout << y.at<float>(i,j) << std::endl;
	  }
      }
	
    //std::cout << "Converted depth : " << z.at<float>(240,320) << std::endl;

    //std::cout << "Gaussian filtering" << std::endl;
    // Apply Gaussian filter
    cv::Mat img_gaussian; //, x_gaussian, y_gaussian, z_gaussian;

    cv::GaussianBlur(img,img_gaussian,cv::Size(3,3),1,1);
    //cv::GaussianBlur(x,x_gaussian,cv::Size(3,3),1,1);
    //cv::GaussianBlur(y,y_gaussian,cv::Size(3,3),1,1);
    //cv::GaussianBlur(z,z_gaussian,cv::Size(3,3),1,1);
	
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
	    //result_.image.push_back(img_gaussian.at<unsigned char>(i,j)); 
	    result_.image.push_back(img.at<unsigned char>(i,j)); 
	    result_.x.push_back(x.at<float>(i,j));
	    result_.y.push_back(y.at<float>(i,j));
	    result_.z.push_back(z.at<float>(i,j));
	    //result_.c.push_back(c.at<unsigned char>(i,j));
	  }
      }

    result_.depth_file_name_number = depth_file_name_number;
    //std::cout << "Image data size : " << result_.image.size() << std::endl;
    //char temp_name[1024];
    //sprintf(temp_name,"Intensity_%s_%d_%d.png",goal->data_name.c_str(), goal->dir_index, goal->file_index);
    //cv::imwrite(temp_name,img_gaussian);


    img_gaussian.release();
    //x_gaussian.release();
    //y_gaussian.release();
    //z_gaussian.release();
    //img_scale.release();

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
    //c.release();
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ms_kinect");

  ROS_INFO("KINECT server is working.");
  kinectAction ms_kinect(ros::this_node::getName());
  ros::spin();

  return 0;
}
