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
#ifndef PCL_IO_LIBFREENECT2_GRABBER_H_
#define PCL_IO_LIBFREENECT2_GRABBER_H_

#include <pcl/io/eigen.h>
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <string>
#include <deque>
#include <pcl/common/synchronizer.h>

#include <pcl/io/image.h>
#include <pcl/io/image_rgb24.h>
#include <pcl/io/image_yuv422.h>
#include <pcl/io/image_depth.h>
#include <pcl/io/image_ir.h>

namespace pcl
{
  struct PointXYZ;
  struct PointXYZRGB;
  struct PointXYZRGBA;
  struct PointXYZI;
  template <typename T> class PointCloud;

  namespace io
  {

    /** \brief Grabber for libfreenect2 devices (i.e., Kinect 2)
    * \ingroup io
    */
    class PCL_EXPORTS Libfreenect2Grabber : public Grabber
    {
      public:
        typedef boost::shared_ptr<Libfreenect2Grabber> Ptr;
        typedef boost::shared_ptr<const Libfreenect2Grabber> ConstPtr;

        // Templated images
        typedef pcl::io::DepthImage DepthImage;
        typedef pcl::io::IRImage IRImage;
        typedef pcl::io::Image Image;

        // Callback signatures
        typedef void (sig_cb_libfreenect2_depth_image) (const boost::shared_ptr<DepthImage>&);
        
      public:
        /** \brief Constructor
        */
        Libfreenect2Grabber ();

        /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
        virtual ~Libfreenect2Grabber () throw ();

        /** \brief Start the data acquisition. */
        virtual void
        start ();

        /** \brief Stop the data acquisition. */
        virtual void
        stop ();

        /** \brief Check if the data acquisition is still running. */
        virtual bool
        isRunning () const;

        virtual std::string
        getName () const;

        /** \brief Obtain the number of frames per second (FPS). */
        virtual float
        getFramesPerSecond () const;

      protected:
        bool running_;

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

  } // namespace
}

#endif // PCL_IO_LIBFREENECT2_GRABBER_H_
#endif // HAVE_LIBFREENECT2
