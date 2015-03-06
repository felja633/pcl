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
pcl::io::Libfreenect2Grabber::Libfreenect2Grabber ()
{
  // initialize driver
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::io::Libfreenect2Grabber::~Libfreenect2Grabber () throw ()
{
  try
  {
    stop ();
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
  try
  {
    // check if we need to start/stop any stream
    if (image_required_ && !device_->isColorStreamStarted () )
    {
      block_signals ();
      device_->startColorStream ();
      startSynchronization ();
    }

    if (depth_required_ && !device_->isDepthStreamStarted ())
    {
      block_signals ();
      if (device_->hasColorSensor () && device_->isImageRegistrationModeSupported () )
      {
        device_->setImageRegistrationMode (true);
      }
      device_->startDepthStream ();
      startSynchronization ();
    }

    if (ir_required_ && !device_->isIRStreamStarted () )
    {
      block_signals ();
      device_->startIRStream ();
    }
    running_ = true;
  }
  catch (IOException& ex)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not start streams. Reason: " << ex.what ());
  }

  // workaround, since the first frame is corrupted
  //boost::this_thread::sleep (boost::posix_time::seconds (1));
  unblock_signals ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::stop ()
{
  try
  {
    if (device_->hasDepthSensor () && device_->isDepthStreamStarted () )
      device_->stopDepthStream ();

    if (device_->hasColorSensor () && device_->isColorStreamStarted () )
      device_->stopColorStream ();

    if (device_->hasIRSensor () && device_->isIRStreamStarted ())
      device_->stopIRStream ();

    running_ = false;
  }
  catch (IOException& ex)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not stop streams. Reason: " << ex.what ());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::io::OpenNI2Grabber::isRunning () const
{
  return (running_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::OpenNI2Grabber::signalsChanged ()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::io::OpenNI2Grabber::getName () const
{
  return (std::string ("Libfreenect2Grabber"));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::io::OpenNI2Grabber::getFramesPerSecond () const
{
  return 30.0;
}

#endif // HAVE_OPENNI2
