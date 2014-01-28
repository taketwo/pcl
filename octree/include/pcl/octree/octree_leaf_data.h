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
    /** \brief This base accumulator policy does nothing */
    template <typename PointT>
    class NullAccumulator
    {
    public:
      NullAccumulator ()
      {
      }
      
      virtual ~NullAccumulator ()
      {
      }
      
      size_t
      size () const
      {
        return 0;
      }
      
      /** Empty insert, does nothing */
      virtual void 
      insert (int, const PointT&)
      {
      }
      
      /** Reset does nothing */
      virtual void 
      reset ()
      {
      }
      
      virtual int
      index () const
      {
        return -1;
      }
      
      virtual void
      value (PointT&) const
      {
      }
    };
    
    /** \brief The Null accumulator policy simply stores the last point and index given to it */
    template <typename PointT>
    class LastAccumulator : public NullAccumulator<PointT>
    {
    public:
      LastAccumulator ():
      last_index_ (-1)
      {
      }
      
      ~LastAccumulator ()
      {
      }

      /** \brief Retrieve the last point
       */
      void
      value (PointT& point_arg) const
      {
        point_arg = last_value_;
      }
      
      /** \brief Retrieve the last index
       */
      int 
      index () const
      {
        return last_index_;
      }
      
      size_t
      size () const
      {
        return 1;
      }
      
      /** Add a new point or value. */
      void 
      insert (int index_arg, const PointT &point_arg)
      {
        last_value_ = point_arg;
        last_index_ = index_arg;
      }
      
      virtual void 
      reset ()
      {
        last_value_ = PointT ();
        last_index_ = -1;
      }
      
    protected:
      PointT last_value_;
      int last_index_;
    };
    
    /** \brief The counter accumulator policy simply stores the number of values inserted into it */
    template <typename PointT>
    class CounterAccumulator : public NullAccumulator<PointT>
    {
    public:
      CounterAccumulator ()
      {
      }
      
      ~CounterAccumulator ()
      {
      }
      
      size_t
      size () const
      {
        return counter_;
      }
      
      /** Add a new point or value. */
      void 
      insert (int, const PointT &)
      {
        ++counter_;
      }
      
      virtual void 
      reset ()
      {
        counter_ = 0;
      }
      
    protected:
      int counter_;
    };
    
    /** \brief A generic class that computes the average of points fed to it.
     * 
     * After all the points that have to be averaged are input with insert(), the
     * user should call compute(). At this point the average is computed, and
     * can be then retrieved using the cast operator to PointT. If compute has not
     * been invoked, the cast will invoke it.
     *
     * \code
     * AveragingAccumulator<pcl::PointXYZ> avg1;
     * avg1.insert (pcl::PointXYZ (1, 2, 3);
     * avg1.insert (pcl::PointXYZ (5, 6, 7);
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
    class AveragingAccumulator : public NullAccumulator<PointT>
    {
      
    public:
      
      AveragingAccumulator ()
      : num_points_ (0)
      {
      }
      
      ~AveragingAccumulator ()
      {
      }
      
      /** \brief Get the number of points that have been inserted */
      size_t
      size () const
      {
        return num_points_;
      }
      
      /** \brief Retrieve the computed average point.
        * \note By default will invoke compute () if it has not been called yet
        */
      void 
      value (PointT& point_arg) const
      {
        xyz_.get (point_arg, num_points_);
        normal_.get (point_arg, num_points_);
        color_.get (point_arg, num_points_);
      }
      
      /** Add a new point. */
      void 
      insert (int, const PointT& point_arg)
      {
        ++num_points_;
        xyz_.add (point_arg);
        normal_.add (point_arg);
        color_.add (point_arg);
      }
      
      virtual void 
      reset ()
      {
        xyz_ = detail::xyz_accumulator<PointT> ();
        normal_ = detail::normal_accumulator<PointT> ();
        color_ = detail::color_accumulator<PointT> ();
        num_points_ = 0;
      }
      
    protected:
      detail::xyz_accumulator<PointT> xyz_;
      detail::normal_accumulator<PointT> normal_;
      detail::color_accumulator<PointT> color_;
      
      size_t num_points_;
      
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
      
    template <typename PointT>
    class IndexVectorAccumulator : public NullAccumulator<PointT>
    {
    public:
      IndexVectorAccumulator ()
      {
      }
      
      ~IndexVectorAccumulator ()
      {
      }
      
      /** \brief Get the number of elements that have been inserted */
      size_t
      size () const
      {
        return indices_.size ();
      }
          
      /** Add a new point. */
      void 
      insert (int index_arg, const PointT&)
      {
        indices_.push_back (index_arg);
      }
      
      virtual void 
      reset ()
      {
        indices_.clear ();
      }
      
      /** \brief Returns the indices vector
       */
      const std::vector<int>&
      getPointIndicesVector () const
      {
        return indices_;
      }
      
      /** \brief Appends the indices vector to the input vector
        * \param[in] data_vector_arg Vector to append the indices to
        */
      void
      getPointIndices (std::vector<int>& data_vector_arg) const
      {
        data_vector_arg.insert (data_vector_arg.end (), indices_.begin (), indices_.end ());
      }
      
    protected:
      std::vector<int> indices_;
    };
  }
}

#endif //PCL_OCTREE_LEAF_DATA_H_

