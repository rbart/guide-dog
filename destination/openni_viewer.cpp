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
#include <cmath> // for sqrt
#include <opencv/cv.h>
#include <cassert>
#include "../../sonic-dog/sonic_dog.h"

#define SHOW_FPS 0
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

int frameSkipCount = 5;
int threshold = 210;
int minCount = 4000;

// Callback sets this to true to indicate that recalibration of
// the color pink should be performed.
bool recalibrate = false;

bool shift = false;

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

  if (event.keyDown() && (event.getKeySym() == "Shift_L" || event.getKeySym() == "Shift_R")) shift = true;   
  if (event.keyUp() && (event.getKeySym() == "Shift_L" || event.getKeySym() == "Shift_R")) shift = false;   

  if (event.keyDown() && event.getKeySym() == "Up") if (shift) threshold += 1; else threshold += 10;
  else if (event.keyDown() && event.getKeySym() == "Down") if (shift) threshold -= 1; else threshold -= 10;

  if (event.keyDown() && event.getKeySym() == "Right") if (shift) minCount += 500; else minCount += 2000;
  else if (event.keyDown() && event.getKeySym() == "Left") if (shift) minCount -= 500; else minCount -= 2000;

  if (event.keyDown() && event.getKeyCode() == 32) {

    recalibrate = true; 
  }
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

  SonicDog dog(1);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  printf("Initializing\n");
  double pinkR = 192.21;
  double pinkG = 96.32;
  double pinkB = 128.36;
  double pinkMag = sqrt(pinkR*pinkR+pinkG*pinkG+pinkB*pinkB);

  double * dvalues;

  pinkR /= pinkMag;
  pinkG /= pinkMag;
  pinkB /= pinkMag;

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
  // Register callbacks
  std::string keyMsg2D ("Key event for image viewer");
  std::string mouseMsg2D ("Mouse coordinates in image viewer");
  img->registerMouseCallback (&mouse_callback, (void*)(&mouseMsg2D));
  img->registerKeyboardCallback(&keyboard_callback, (void*)(&keyMsg2D));
  boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&) > image_cb = boost::bind (&EventHelper::image_callback, &event_helper, _1);
  boost::signals2::connection image_connection = interface->registerCallback (image_cb);
  unsigned char* rgb_data = 0;
  unsigned rgb_data_size = 0;
#endif 
  
  interface->start ();
  int beacon = -1;
  int frameNum = 0;
  bool cld_init = false;
  // Loop
  while (!cld->wasStopped ())
  {
    // Render and process events in the two interactors
    cld->spinOnce ();
#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))
    img->spinOnce ();
#endif
    FPS_CALC ("drawing");
    
    frameNum++;
    
    // Add the cloud
    if (g_cloud && cld_mutex.try_lock ())
    {
      if (!cld_init)
      {
        cld->getRenderWindow ()->SetSize (g_cloud->width, g_cloud->height);
        cld->getRenderWindow ()->SetPosition (g_cloud->width, 0);
	dvalues = new double[g_cloud->width * g_cloud->height];
        cld_init = !cld_init;
      }

      // check out OpenCV blob detection

      // For each RGB value in the cloud,
      // Normalize the magnitude of the RGB vector to one,
      // Compute cosine similarity with pink vector
      // threshold the resulting image
      // Compute the centroid of pixels that make it through the thresholded image.
      // If enough points make it through the filter, we say that the destination object is in the frame at the XYZ centroid.
      int width = g_cloud->width;
      int height = g_cloud->height;

      // compute the average RGB vector for points in the cloud
      for (int x = 0; x < width; x++) {
	for (int y = 0; y < height; y++) {
	  const pcl::PointXYZRGBA p = g_cloud->at(x,y);
	  double r = p.r;
	  double g = p.g;
	  double b = p.b;

	  // compute normalized rgb vector:
	  double mag = sqrt(r*r+g*g+b*b);
	  if (mag < 0.0001) mag = 0.0001;

	  r /= mag;
	  g /= mag;
	  b /= mag;

	  // compute cosine similarity with "pink" vector. 
	  double dotproduct = r*pinkR + g*pinkG + b*pinkB;
	  double dr = fabs(r-pinkR);
	  double dg = fabs(g-pinkG);
	  double db = fabs(b-pinkB);
	  
	  double dmag = sqrt(dr*dr+dg*dg+db*db);
	  double magDiff = fabs(mag - pinkMag);
	  if (magDiff > 192) dmag = 1;
	  
	  dvalues[y*g_cloud->width + x] = dmag;
	}
      }

      pcl::copyPointCloud(*g_cloud, *p_cloud);

      // These values are used to help compute 
      // an average RGB vector for pixels above threshold to help with recalibration.
      float sumR = 0;
      float sumG = 0;
      float sumB = 0;
      int numAvgRgb = 0;
      // These values are used to help compute
      // the centroid for pixels above threshold
      float sumX = 0;
      float sumY = 0;
      float sumZ = 0;

      int numAvg = 0;
      
      // try building this into an OpenCV Mat object.
      // then you can try running blob detection on it. 
      // once that is done, you could move color values
      // back into the cloud for display. 
      // also try looking into the visualization library for ways
      // of showing the cloud.

      for (int x = 0; x < g_cloud->width; x++) {
        for (int y = 0; y < g_cloud->height; y++) {
	  double testColor = (1.0-dvalues[y*g_cloud->width + x]) * 255;

	  const pcl::PointXYZRGBA &g = g_cloud->at(x,y);
	  pcl::PointXYZRGBA &p = p_cloud->at(x,y);

	  if (testColor <= threshold) testColor = 0.0;
	  else {
	    if (p.x == p.x && p.y == p.y && p.z == p.z) {
	      sumX += p.x;
	      sumY += p.y;
	      sumZ += p.z;
	      numAvg++;
	    }
	  }
	  // only compute avg RGB vals for pixels near center of frame
	  bool inCenterX = x >= (7*width)/16 && x <= (9*width)/16;
	  bool inCenterY = y >= (7*height)/16 && y <= (9*height)/16;
	  bool inCenter = inCenterX && inCenterY;
	  if (inCenter && p.r == p.r && p.g == p.g && p.b == p.b) {
	    sumR += p.r;
	    sumG += p.g;
	    sumB += p.b;
	    numAvgRgb++;
	  }
	  

	  p.r = testColor;
	  p.g = testColor;
	  p.b = testColor;
	 }
      }

      double avgX = sumX/numAvg;
      double avgY = sumY/numAvg;
      double avgZ = sumZ/numAvg;
      double avgR = sumR/numAvgRgb;
      double avgG = sumG/numAvgRgb;
      double avgB = sumB/numAvgRgb;

      //      printf("Avg (%d) = (%.2f, %.2f, %.2f)\n", numAvg, avgX, avgY, avgZ);
      
      bool frameSkip = frameNum % frameSkipCount != 0;
      bool haveBeacon = numAvg >= minCount;

      if (!frameSkip) printf("Count: %d, Min Count: %d, Min similarity: %d, Beacon ID: %d\n", numAvg, minCount, threshold, beacon);
      
      if (beacon == -1 && haveBeacon) {
	beacon = dog.addBeacon(12*avgX, 3*avgZ);
	dog.start();
      } 
      else if (beacon != -1 && !haveBeacon) {
        dog.removeObject(beacon);
	beacon = -1;
      } 
      else if (!frameSkip == 0 && haveBeacon && beacon != -1) {
        dog.changeObjectLoc(beacon,12*avgX, 3*avgZ);
      }

// if enough points, etc
      

      if (recalibrate) {
	printf("Recalibrating from %d pts to (R,G,B) of (%.2f, %.2f, %.2f)\n", numAvgRgb, avgR, avgG, avgB);
	recalibrate = false;
	pinkR = avgR; 
        pinkG = avgG;
	pinkB = avgB;
	pinkMag = sqrt(pinkR*pinkR+pinkG*pinkG+pinkB*pinkB);
	pinkR /= pinkMag;
	pinkG /= pinkMag;
	pinkB /= pinkMag;
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
      img_mutex.unlock ();
    }
    #endif
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }

  interface->stop ();
}
