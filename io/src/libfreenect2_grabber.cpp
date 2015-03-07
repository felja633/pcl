/*
 * Software License Agreement (BSD License)
 * 
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 * Copyright (c) 2014, respective authors.
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 */

#include <pcl/pcl_config.h>

#ifdef HAVE_LIBFREENECT2

#include <pcl/io/libfreenect2_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/io/boost.h>
#include <pcl/exceptions.h>
#include <libfreenect2/libfreenect2.hpp>
#include <iostream>

namespace pcl
{
  // Treat color as chars, float32, or uint32
  typedef union
  {
    struct
    {
      unsigned char Blue;
      unsigned char Green;
      unsigned char Red;
      unsigned char Alpha;
    };
    float float_value;
    uint32_t long_value;
  } RGBValue;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::io::Libfreenect2Grabber::Libfreenect2Grabber () :
running_(false)
{
  // initialize driver
  libfreenect2::Freenect2 freenect2;
  device_ = freenect2.openDefaultDevice();
  if (device_ == 0)
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not open the freenect2 device");
  
  // Only listen for depth at this point
  listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Depth);
  
  device_->setIrAndDepthFrameListener(listener_);
  
  createSignal<sig_cb_libfreenect2_depth_image> ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::io::Libfreenect2Grabber::~Libfreenect2Grabber () throw ()
{
  try
  {
    stop ();
    
    disconnect_all_slots<sig_cb_libfreenect2_depth_image> ();
    
    delete listener_;
  }
  catch (...)
  {
    // destructor never throws
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::Libfreenect2Grabber::start ()
{
    if (isRunning ())
        return;
    
    running_ = true;
    
    grabber_thread_ = boost::thread (&pcl::io::Libfreenect2Grabber::processGrabbing, this);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::Libfreenect2Grabber::stop ()
{
  try
  {

    running_ = false;
  }
  catch (IOException& ex)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not stop streams. Reason: " << ex.what ());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::io::Libfreenect2Grabber::isRunning () const
{
  return (running_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::io::Libfreenect2Grabber::getName () const
{
  return (std::string ("Libfreenect2Grabber"));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::io::Libfreenect2Grabber::getFramesPerSecond () const
{
  return 30.0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::Libfreenect2Grabber::processGrabbing ()
{
    bool continue_grabbing = running_;
    
    while (continue_grabbing)
    {     
        listener_->waitForNewFrame(frames_);
        
        if (num_slots<sig_cb_libfreenect2_depth_image> () > 0)
        {
            libfreenect2::Frame *depth = frames_[libfreenect2::Frame::Depth];
            
            // TODO: Wrap depth image and signal callback
            //depth_signal_(depth_);
            
        }
        listener_->release(frames_);
    }
}

#endif // HAVE_OPENNI2
