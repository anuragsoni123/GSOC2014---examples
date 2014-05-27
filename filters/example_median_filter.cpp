/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: example_ocl_medianFilter_me.cpp $
 *
 */

// STL
#include <iostream>
#include <limits>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ocl/filters/median_filter.h>
#include <pcl/ocl/filters/impl/median_filter.hpp>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/impl/median_filter.hpp>
typedef pcl::PointXYZ PointT;

#include<string>
#include <time.h>
int main (int argc, char** argv)
{
  std::string incloudfile = argv[1];
  std::string outcloudfile = argv[2];
  std::string outcloudfile1 = argv[3];
  //std::string outcloudfile2 = argv[4];


  clock_t tStart = clock();
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::io::loadPCDFile (incloudfile.c_str (), *cloud);
  //printf("\n\n\n %d %d\n\n\n",cloud->width, cloud->height );
  pcl::PointCloud<PointT> cloud_filtered;
  
  pcl::ocl::MedianFilter<PointT> mf;
  mf.setInputCloud(cloud);
  
  mf.filter(cloud_filtered);
  printf("GPU Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  
  pcl::io::savePCDFile (outcloudfile.c_str (), cloud_filtered);

///////////////CPU////////////////////////////
  tStart = clock();
  pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
  pcl::io::loadPCDFile (incloudfile.c_str (), *cloud1);
  //printf("\n\n\n %d %d\n\n\n",cloud->width, cloud->height );
  pcl::PointCloud<PointT> cloud_filtered1;
  
  pcl::MedianFilter<PointT> mf1;
  mf1.setInputCloud(cloud1);
  
  mf1.filter(cloud_filtered1);
  printf("CPU Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  // Save filtered output
  pcl::io::savePCDFile (outcloudfile1.c_str (), cloud_filtered1);

  /*pcl::PointCloud<pcl::PointXYZ> test;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  test.width  = 1024;
  test.height = 1024;
  test.points.resize (test.width * test.height);

  for (int i = 0; i < cloud->points.size (); ++i)
  {
    test.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    test.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    test.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  pcl::io::savePCDFile (outcloudfile2.c_str(), test);
*/
  
   return (0);
 
 
}
