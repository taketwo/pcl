/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *
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
 */

#ifndef PCL_OCTREE_LEAF_DATA_H_
#define PCL_OCTREE_LEAF_DATA_H_

#include <pcl/octree/impl/octree_leaf_data.hpp>

namespace pcl
{
  namespace octree
  {
    
    /** \brief A generic class that computes the average of points fed to it.
     * 
     * After all the points that have to be averaged are input with add(), the
     * user should call compute(). At this point the average is computed, and
     * can be then retrieved using the only public function of the class, cast
     * operator to PointT.
     *
     * \code
     * AveragePoint<pcl::PointXYZ> avg1;
     * avg1.add (pcl::PointXYZ (1, 2, 3);
     * avg1.add (pcl::PointXYZ (5, 6, 7);
     * avg1.compute ();
     * pcl::PointXYZ result = avg1;
     * // result.x == 3, result.y == 4, result.z == 5
     * \endcode
     *
     * The average is field-wise, i.e. it computes average for each of the
     * supported fields separately. The currently supported fields include:
     *
     * - XYZ
     * - Normal+Curvature
     * - RGB/RGBA
     *
     * Note that the template can be successfuly instantiated for *any*
     * point type. Of course, each of the field averages is computed only if
     * the point type has the corresponding field. */
    template <typename PointT>
    class AveragePoint
    {
      
    public:
      
      AveragePoint ()
      : num_points_ (0)
      {
      }
      
      /** Retrieve the computed average point. */
      operator PointT ()
      {
        return average_point_;
      }
      
      /** Add a new point. */
      void add (const PointT& pt)
      {
        ++num_points_;
        xyz_.add (pt);
        normal_.add (pt);
        color_.add (pt);
      }
      
      /** Compute the average of the previously added points. */
      void compute ()
      {
        if (num_points_)
        {
          xyz_.get (average_point_, num_points_);
          normal_.get (average_point_, num_points_);
          color_.get (average_point_, num_points_);
        }
      }
      
    private:
      detail::xyz_accumulator<PointT> xyz_;
      detail::normal_accumulator<PointT> normal_;
      detail::color_accumulator<PointT> color_;
      
      size_t num_points_;
      
      PointT average_point_;
      
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
    };
    
  }
  
  
}

#endif //PCL_OCTREE_LEAF_DATA_H_

