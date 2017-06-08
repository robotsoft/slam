/**
 * SwissRanger Server
 *
 * Created on : 10/30/2012
 * Author : Soonhac Hong (sxhong1@ualr.edu)
 */

#ifndef LOAD_SR4K_H_
#define LOAD_SR4K_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

class LoadSR4K
{
public:
  bool load_sr4k(const char* file_name, cv::Mat& img, cv::Mat& x, cv::Mat& y, cv::Mat& z, cv::Mat& c, bool confidence);
};

#endif //LOAD_SR4K_H_
