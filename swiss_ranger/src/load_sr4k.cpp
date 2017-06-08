/**
 * SwissRanger Server
 *
 * Created on : 10/30/2012
 * Author : Soonhac Hong (sxhong1@ualr.edu)
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include "load_sr4k.h"


bool LoadSR4K::load_sr4k(const char* data_file_name, cv::Mat& img, cv::Mat& x, cv::Mat& y, cv::Mat& z, cv::Mat& c, bool is_confidence)
{
  //std::cout << "Loading sr4k data from " << data_file_name << std::endl;
  
  std::string line;
  std::ifstream data_file;
  float data[144][176];
  int data_row_index = 0;
  int data_index = 0;
  cv::Size image_size(176,144);
  

  //std::cout << "Width, Height : " << image_size.width << "," << image_size.height << std::endl;

  data_file.open(data_file_name, std::ifstream::in);

  //std::cout << "Data file is open : " << data_file.is_open() << std::endl;
  if(data_file.is_open())
    {
      while(data_file.good())//!data_file.eof())
	{
	  std::getline(data_file,line);
	  //data_file >> line;

	  //std::cout << line << std::endl;

	  if(line.find("%") == std::string::npos)
	    {
	      //std::cout << "Row : " << data_row_index << std::endl;
	      std::stringstream ss(line);
	      for(int data_col_index = 0; data_col_index < 176; data_col_index++)
		{
		  ss >> data[data_row_index][data_col_index];
		}
	      data_row_index++;
	    }
	  else
	    {
	      //std::cout << line << std::endl;
	      data_row_index=0;
	  
	      if(data_index >= 1 && data_index <= 5)
		{
		  for(int i = 0 ; i < image_size.height; i++)
		    {
		      for(int j = 0 ; j < image_size.width; j++)
			{
			  if(data_index == 1) // distance
			    {
			      z.at<float>(i,j) = data[i][j];
			      //std::cout << z.at<double>(i,j) << std::endl;
			    }
			  else if(data_index == 2) // x
			    {
			      x.at<float>(i,j) = data[i][j];
			    }
			  else if(data_index == 3) // y
			    {
			      y.at<float>(i,j) = data[i][j]; 
			    }
			  else if(data_index == 4) // intensity image
			    {
			      img.at<float>(i,j) = data[i][j]; 
			      /*if(image_max < data[i][j])
				{
				  image_max = data[i][j];
				  }*/
			    }
			  else if(data_index == 5 && is_confidence) // confidence
			    {
			      c.at<unsigned char>(i,j) = data[i][j]; 
			    }
			}
		    }
		}
	      else if(data_index >= 6)
		{
		  break;
		}
	      data_index++;
	      //std::cout << "Data index : " << data_index << std::endl;
	    }
	}

      data_file.close();
      //std::cout << "Data is loaded safely." << std::endl;

      return true;
    }
  else
    {
      return false;
    }
}
