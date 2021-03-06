/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: openni_viewer.cpp 4360 2012-02-10 01:01:11Z rusu $
 *
 */

#include <boost/thread/thread.hpp>
#define MEASURE_FUNCTION_TIME
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <boost/math/special_functions/fpclassify.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif

boost::mutex cld_mutex, img_mutex;
pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr g_cloud;
boost::shared_ptr<openni_wrapper::Image> g_image;

void
printHelp (int argc, char **argv)
{
  using pcl::console::print_error;
  using pcl::console::print_info;
  print_error ("Syntax is: %s <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -dev device_id           = device to be used\n");
  print_info ("                                                maybe \"#n\", with n being the number of the device in device list.\n");
  print_info ("                                                maybe \"bus@addr\", with bus and addr being the usb bus and address where device is connected.\n");
  print_info ("                                                maybe \"serial\", with serial being the serial number of the device.\n");
  print_info ("\n");
}

// Create the PCLVisualizer object
boost::shared_ptr<pcl::visualization::PCLVisualizer> cld;
#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))
boost::shared_ptr<pcl::visualization::ImageViewer> img;
boost::shared_ptr<pcl::visualization::ImageViewer> img_2d;
#endif

struct EventHelper
{
  void 
  cloud_cb (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud)
  {
    FPS_CALC ("callback");
    cld_mutex.lock ();
    g_cloud = cloud;
    cld_mutex.unlock ();
  }

#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))
  void
  image_callback (const boost::shared_ptr<openni_wrapper::Image>& image)
  {
    FPS_CALC ("image callback");
    img_mutex.lock ();
    g_image = image;
    img_mutex.unlock ();
  }
#endif  
};
// Simple callbacks.
void 
keyboard_callback (const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  std::string* message = (std::string*)cookie;
  cout << (*message) << " :: ";
  if (event.getKeyCode())
    cout << "the key \'" << event.getKeyCode() << "\' (" << (int)event.getKeyCode() << ") was";
  else
    cout << "the special key \'" << event.getKeySym() << "\' was";
  if (event.keyDown())
    cout << " pressed" << endl;
  else
    cout << " released" << endl;
}

void 
mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void* cookie)
{
  std::string* message = (std::string*) cookie;
  if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
  {
    cout << (*message) << " :: " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
  }
}

void
project_points(pcl::ModelCoefficients::Ptr ground_plane, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_cloud) {
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud (*p_cloud, *p_cloud, indices);

  // Project to ground plane
  pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (p_cloud);
  proj.setModelCoefficients (ground_plane);
  proj.filter (*p_cloud);

  // Set coefficients and project to y=0 plane
  pcl::ModelCoefficients::Ptr coefficients_2d (new pcl::ModelCoefficients ());
  coefficients_2d->values.resize (4);
  coefficients_2d->values[0] = 0;
  coefficients_2d->values[1] = 1.0;
  coefficients_2d->values[2] = 0;
  coefficients_2d->values[3] = 0;
  proj.filter (*p_cloud);
}

void
project_points(pcl::ModelCoefficients::Ptr ground_plane, std::vector<pcl::PointXYZRGBA> &points, std::vector<pcl::PointXYZRGBA> &result) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  for (size_t i = 0; i < points.size(); i++) {
    cloud->push_back(points[i]);
  }
  cloud->width = points.size();
  cloud->height = 1;

  project_points(ground_plane, cloud);

  for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator i = cloud->begin(); i != cloud->end(); i++) {
    result.push_back(*i);
  }
}

int
transform_x_coord(float x) {
  return (int) floor((x + 2.5) * 200 + .5);
}

int
transform_z_coord(float z) {
  return (int) floor(z * 100 + .5);
}

void
write_point_to_image(int x, int z, char r, char g, char b, unsigned char* img_2d_rgb, int img_2d_width, int img_2d_height) {
  if (0 <= x && x < img_2d_width && 0 <= z && z < img_2d_height) {
    int pos = img_2d_width * 3 * z + 3 * x;
    img_2d_rgb[pos] = r;
    img_2d_rgb[pos + 1] = g;
    img_2d_rgb[pos + 2] = b;
  }
}

void
write_to_image(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_cloud, unsigned char* img_2d_rgb, int img_2d_width, int img_2d_height) {
  // Write obstacle data to an image.
  memset(img_2d_rgb, 0, 3 * img_2d_width * img_2d_height);
  for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator i = p_cloud->begin(); i != p_cloud->end(); i++) {
    int newX = transform_x_coord(i->x);
    int newZ = transform_z_coord(i->z);

    write_point_to_image(newX, newZ, 255, 255, 255, img_2d_rgb, img_2d_width, img_2d_height);
  }
}

void
add_mark_to_image(pcl::PointXYZRGBA &point, char r, char g, char b, unsigned char* img_2d_rgb, int img_2d_width, int img_2d_height) {
  int pointSize = 5;
  int newX = transform_x_coord(point.x);
  int newZ = transform_z_coord(point.z);
  for (int x = newX - pointSize; x <= newX + pointSize; x++) {
    for (int z = newZ - pointSize; z <= newZ + pointSize; z++) {
      write_point_to_image(x, z, r, g, b, img_2d_rgb, img_2d_width, img_2d_height);
    }
  }
}

float
vector_length(pcl::PointXYZRGBA &point) {
  return sqrt(pow(point.x, 2) + pow(point.z, 2));
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc > 1)
  {
    for (int i = 1; i < argc; i++)
    {
      if (std::string (argv[i]) == "-h")
      {
        printHelp (argc, argv);
        return (-1);
      }
    }
  }

  EventHelper event_helper;
  std::string device_id = "";
  pcl::console::parse_argument (argc, argv, "-dev", device_id);

  pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id);

  cld.reset (new pcl::visualization::PCLVisualizer (argc, argv, "OpenNI Viewer"));

  std::string mouseMsg3D ("Mouse coordinates in PCL Visualizer");
  std::string keyMsg3D ("Key event for PCL Visualizer");
  cld->registerMouseCallback (&mouse_callback, (void*)(&mouseMsg3D));    
  cld->registerKeyboardCallback(&keyboard_callback, (void*)(&keyMsg3D));
  boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) > f = boost::bind (&EventHelper::cloud_cb, &event_helper, _1);
  boost::signals2::connection c1 = interface->registerCallback (f);

#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))
  img.reset (new pcl::visualization::ImageViewer ("OpenNI Viewer"));
  img_2d.reset (new pcl::visualization::ImageViewer ("OpenNI Viewer - 2D"));
  // Register callbacks
  std::string keyMsg2D ("Key event for image viewer");
  std::string mouseMsg2D ("Mouse coordinates in image viewer");
  img->registerMouseCallback (&mouse_callback, (void*)(&mouseMsg2D));
  img->registerKeyboardCallback(&keyboard_callback, (void*)(&keyMsg2D));
  boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&) > image_cb = boost::bind (&EventHelper::image_callback, &event_helper, _1);
  boost::signals2::connection image_connection = interface->registerCallback (image_cb);
  unsigned char* rgb_data = 0;
  unsigned rgb_data_size = 0;
  int img_2d_width = 1000;
  int img_2d_height = 750;
  unsigned char* img_2d_rgb = new unsigned char[3 * img_2d_width * img_2d_height];

#endif 
  
  interface->start ();
  bool cld_init = false;
  // Loop
  while (!cld->wasStopped ())
  {
    // Render and process events in the two interactors
    cld->spinOnce ();
#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))
    img->spinOnce ();
    img_2d->spinOnce ();
#endif
    FPS_CALC ("drawing");

    // Add the cloud
    if (g_cloud && cld_mutex.try_lock ())
    {
      if (!cld_init)
      {
        cld->getRenderWindow ()->SetSize (g_cloud->width, g_cloud->height);
        cld->getRenderWindow ()->SetPosition (g_cloud->width, 0);
        cld_init = !cld_init;
      }

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.05);
      seg.setInputCloud (g_cloud);
      seg.segment (*inliers, *coefficients);

      cout << "plane:" << endl;
      for (std::vector<float>::iterator i = coefficients->values.begin();
           i != coefficients->values.end();
           ++i)
        {
          cout << "\t" << *i << endl;
        }

      float a = coefficients->values[0];
      float b = coefficients->values[1];
      float c = coefficients->values[2];
      float d = coefficients->values[3];

      // Don't process the frame if a plane can't be found.
      if (boost::math::isnan<float>(a) || boost::math::isnan<float>(b) || boost::math::isnan<float>(c) || boost::math::isnan<float>(d)) {
        continue;
      }

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
      extract.setInputCloud (g_cloud);
      extract.setIndices (inliers);
      extract.setNegative (true);
      extract.filter (*p_cloud);

      project_points(coefficients, p_cloud);

      write_to_image(p_cloud, img_2d_rgb, img_2d_width, img_2d_height);

      pcl::PointXYZRGBA closestLeft;
      float closestLeftDistance = -1;
      pcl::PointXYZRGBA closestRight;
      float closestRightDistance = -1;

      for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator i = p_cloud->begin(); i != p_cloud->end(); i++) {
        float distance = vector_length(*i);
        if (i->x < 0) {
          if (closestLeftDistance == -1 || distance < closestLeftDistance) {
            closestLeft = *i;
            closestLeftDistance = distance;
          }
        } else {
          if (closestRightDistance == -1 || distance < closestRightDistance) {
            closestRight = *i;
            closestRightDistance = distance;
          }
        }
      }

      if (closestLeftDistance != -1) {
        cout << "closestLeft: " << closestLeft.x << ", " << closestLeft.z << endl;
        add_mark_to_image(closestLeft, 255, 0, 0, img_2d_rgb, img_2d_width, img_2d_height);
      }
      if (closestRightDistance != -1) {
        cout << "closestRight: " << closestRight.x << ", " << closestRight.z << endl;
        add_mark_to_image(closestRight, 255, 0, 0, img_2d_rgb, img_2d_width, img_2d_height);
      }

      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> handler (p_cloud);
      if (!cld->updatePointCloud (p_cloud, handler, "OpenNICloud"))
      {
        cld->addPointCloud (p_cloud, handler, "OpenNICloud");
        cld->resetCameraViewpoint ("OpenNICloud");
      }
      cld_mutex.unlock ();
    }
    
#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))
    // Add the image
    if (g_image && img_mutex.try_lock ())
    {
      if (g_image->getEncoding() == openni_wrapper::Image::RGB)
        img->showRGBImage (g_image->getMetaData ().Data (), 
                           g_image->getWidth (), g_image->getHeight ());
      else
      {
        if (rgb_data_size < g_image->getWidth () * g_image->getHeight ())
        {
          rgb_data_size = g_image->getWidth () * g_image->getHeight ();
          rgb_data = new unsigned char [rgb_data_size * 3];
        }
        g_image->fillRGB (g_image->getWidth (), g_image->getHeight (), rgb_data);
        img->showRGBImage (rgb_data, g_image->getWidth (), g_image->getHeight ());
      }
      img_2d->showRGBImage (img_2d_rgb, img_2d_width, img_2d_height);
      img_mutex.unlock ();
    }
#endif
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }

  interface->stop ();
}
/* ]--- */
