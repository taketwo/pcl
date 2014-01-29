/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id: octree_nodes.h 5596 2012-04-17 15:09:31Z jkammerl $
 */

#ifndef PCL_OCTREE_CONTAINER_H
#define PCL_OCTREE_CONTAINER_H

#include <string.h>
#include <vector>
#include <cstddef>
#include <set>

#include <pcl/pcl_macros.h>

#include <pcl/octree/octree_leaf_data.h>
namespace pcl
{
  namespace octree
  {
    /** \brief Empty Data Type */
    class EmptyData {};

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree container class that can serve as a base to construct own leaf node container classes.
     *  The following should work:
     * \code
     * OctreeLeafContainer <AveragingAccumulator<PointXYZ> > leaf_container;
     * leaf_container.insert (PointXYZ (1, 2, 3));
     * leaf_container.insert (PointXYZ (5, 6, 7));
     * PointXYZ result;
     * leaf_container.value (result);
     * // result.x == 3, result.y == 4, result.z == 5
     * \endcode
     */
    template <typename OctreeDataT = EmptyData,
              typename LeafDataT = NullAccumulator,
              typename UserDataT = EmptyData >
    class OctreeLeafContainer
    {

      public:

        /** \brief Empty constructor. */
        OctreeLeafContainer ()
        {
        }

        /** \brief Copy constructor. */
        OctreeLeafContainer (const OctreeLeafContainer&)
        {
          // TODO: do we need this?
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeLeafContainer ()
        {
        }

        template <typename PointT> void
        insert (int index_arg, const PointT& point)
        {
          AccumulatorTraits<LeafDataT>::insert (leaf_data_, index_arg, point);
        }

        /** \brief Deep copy of leaf - copies all internal data */
        virtual OctreeLeafContainer*
        deepCopy () const
        {
          OctreeLeafContainer* new_container = new OctreeLeafContainer;
          new_container->octree_data_ = octree_data_;
          new_container->leaf_data_ = leaf_data_;
          new_container->user_data_ = user_data_;
          return new_container;
        }

        /** \brief Equal comparison operator - compares indices
          * \note value comparison would make more sense, but this maintains compatibility
          * \param[in] other OctreeLeafContainer to compare with
          *
          * TODO: does this make any sense? What about if leaf data does not
          * store indices? What about user data, maybe it makes sense to compare
          * it instead?
          *
          */
        virtual bool
        operator== (const OctreeLeafContainer&) const
        {
          // TODO: commented out because we do not want to strictly require
          // that leaf data has index() function.
          //return (this->index () == other.index ());
          return false;
        }

        /** \brief Inequal comparison operator
          * \param[in] other OctreeLeafContainer to compare with
          */
        bool
        operator!= (const OctreeLeafContainer& other) const
        {
          return (!operator== (other));
        }

        /** \brief Reset the leaf node (sets data and accumulator to DataT and AccumulatorT constructors) */
        virtual void
        reset ()
        {
          // TODO: there used to be a call to reset() function. This puts
          // unnecessary requirement on the leaf data class. Any cons for just
          // default-constructing a new one?
          octree_data_ = OctreeDataT ();
          leaf_data_ = LeafDataT ();
          user_data_ = UserDataT ();
        }

        /** \brief Returns reference to the internal user data member
         */
        UserDataT&
        getUserData ()
        {
          return user_data_;
        }

        OctreeDataT&
        getOctreeData ()
        {
          return octree_data_;
        }

        LeafDataT&
        getLeafData ()
        {
          return leaf_data_;
        }

        /** DEPRECATED
         * \brief This is included to preserve interface
         * \note Use size() instead
         */
        size_t
        getSize () const
        {
          //return this->size ();
          return 0;
        }

      /** DEPRECATED This is maintained because of octree_pointcloud.hpp:521 TODO Investigate...
       * \brief Empty getPointIndices implementation as this leaf node does not store any data. \
       */
      void
      getPointIndices (std::vector<int>&) const
      {
      }

    protected:
      OctreeDataT octree_data_;
      LeafDataT leaf_data_;
      UserDataT user_data_;
    };

      /** \brief @b Octree adjacency leaf container class - stores set of pointers to neighbors, number of points added, and a DataT value.
       * \note This class implements a leaf node that stores pointers to neighboring leaves.
       * \note This class also has a virtual computeData function, which is called by OctreePointCloudAdjacency::addPointsFromInputCloud.
       * \note If you are not happy with the default data type (which is AveragePoint<PointInT>) and its behavior (averaging of XYZ,
       * normal, and RGB fields), then you should implement your own and either:
       *
       * - make sure it has add() and compute() functions (like AveragePoint does)
       *   or
       * - make explicit instantiations of addPoint() and computeData() functions of this class (supervoxel_clustering.hpp for an example).
       */


      class OctreeAdjacencyData
      {
      public:
        typedef std::set<OctreeAdjacencyData*> NeighborSetT;
        /** Iterator for neighbors of this leaf */
        typedef NeighborSetT::iterator iterator;
        typedef NeighborSetT::const_iterator const_iterator;
        inline iterator begin () { return (neighbors_.begin ()); }
        inline iterator end ()   { return (neighbors_.end ()); }
        inline const_iterator begin () const { return (neighbors_.begin ()); }
        inline const_iterator end () const  { return (neighbors_.end ()); }

        /** Returns the number of neighbors this leaf has */
        inline size_t size () const { return neighbors_.size (); }

        /** \brief Deep copy of leaf - copies all internal data */
        virtual OctreeAdjacencyData *
        deepCopy () const
        {
          OctreeAdjacencyData *new_data = new OctreeAdjacencyData;
          new_data->neighbors_ = this->neighbors_;
          return new_data;
        }

        /** \brief Add new neighbor to voxel.
          * \param[in] neighbor the new neighbor to add
          */
        void
        addNeighbor (OctreeAdjacencyData *neighbor)
        {
          neighbors_.insert (neighbor);
        }

        /** \brief Remove neighbor from neighbor set.
          * \param[in] neighbor the neighbor to remove
          */
        void
        removeNeighbor (OctreeAdjacencyData *neighbor)
        {
          neighbors_.erase (neighbor);
        }

        /** \brief Sets the whole neighbor set
         * \param[in] neighbor_arg the new set
         */
        void
        setNeighbors (const NeighborSetT &neighbor_arg)
        {
          neighbors_ = neighbor_arg;
        }

      protected:
        NeighborSetT neighbors_;
      };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Simple wrapper class to maintain compatibility with old container type which didn't template point type
       * But we use a wrapper class instead for compatibility
       */
      template <typename OctreeDataT = EmptyData,
                typename UserDataT = EmptyData>
      class OctreeContainerPointIndices : public OctreeLeafContainer<OctreeDataT, IndexVectorAccumulator, UserDataT>
      {

          typedef OctreeLeafContainer<OctreeDataT, IndexVectorAccumulator, UserDataT> Base;

        public:

          OctreeContainerPointIndices ()
          : Base ()
          {
          }

          void
          getPointIndices (std::vector<int>& indices) const
          {
            Base::leaf_data_.value (indices);
          }

      };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Point Index class wrapper maintained for compatibility
       */
      template <typename OctreeDataT = EmptyData,
                typename UserDataT = EmptyData>
      class OctreeContainerPointIndex : public OctreeLeafContainer<OctreeDataT, LastIndexAccumulator, UserDataT>
      {

          typedef OctreeLeafContainer<OctreeDataT, LastIndexAccumulator, UserDataT> Base;

        public:

          OctreeContainerPointIndex ()
          : Base ()
          {
          }

      };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief @b Octree container class that does not store any information.
       * \note Can be used for occupancy trees that are used for checking only the existence of leaf nodes in the tree
       */
      template <typename OctreeDataT = EmptyData,
                typename UserDataT = EmptyData>
      class OctreeContainerEmpty : public OctreeLeafContainer<OctreeDataT, NullAccumulator, UserDataT>
      {

          typedef OctreeLeafContainer<OctreeDataT, NullAccumulator, UserDataT> Base;

        public:

          OctreeContainerEmpty ()
          : Base ()
          {
          }

      };

  }
}

#endif
