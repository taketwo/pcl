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
    class EmptyDataT {};
    
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
    template<typename PointT, template<class> class AccumulatorPolicy = AveragingAccumulator, typename DataT = EmptyDataT >
    class OctreeLeafContainer : public AccumulatorPolicy<PointT>
    {
    public:
      using typename AccumulatorPolicy<PointT>::insert;
      using typename AccumulatorPolicy<PointT>::size;
      using typename AccumulatorPolicy<PointT>::index;
      using typename AccumulatorPolicy<PointT>::value;
      
      /** \brief Empty constructor. */
      OctreeLeafContainer ()
      {
      }

      /** \brief Empty constructor. */
      OctreeLeafContainer (const OctreeLeafContainer&)
      {
      }

      /** \brief Empty deconstructor. */
      virtual
      ~OctreeLeafContainer ()
      {
      }
      
      /** \brief Deep copy of leaf - copies all internal data */
      virtual OctreeLeafContainer *
      deepCopy () const
      {
        OctreeLeafContainer *new_container = new OctreeLeafContainer;
        new_container->data_ = this->data_;
        return new_container;
      }
      
      /** \brief Equal comparison operator - compares indices
        * \note value comparison would make more sense, but this maintains compatibility
        *  \param[in] other OctreeLeafContainer to compare with
        */
      virtual bool
      operator== (const OctreeLeafContainer& other) const
      {
        return (this->index () == other.index ());
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
        data_ = DataT ();
        AccumulatorPolicy<PointT>::reset ();
      }
      
      /** \brief Returns reference to the internal data member
       */
      DataT &
      getData ()
      {
        return data_;
      }
      
      /** DEPRECATED
       * \brief This is included to preserve interface 
       * \note Use size() instead
       */
      size_t
      getSize () const
      {
        return this->size ();
      }
      
      /** DEPRECATED This is maintained because of octree_pointcloud.hpp:521 TODO Investigate...
       * \brief Empty getPointIndices implementation as this leaf node does not store any data. \
       */
      void
      getPointIndices (std::vector<int>&) const
      {
      }
    protected:
      DataT data_;
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
      template<typename PointT, template<class> class AccumulatorPolicy = AveragingAccumulator, typename DataT = EmptyDataT >
      class OctreeAdjacencyContainer : public OctreeLeafContainer<PointT, AccumulatorPolicy, DataT>
      {
      public:
        typedef std::set<OctreeAdjacencyContainer*> NeighborSetT;
        typedef OctreeLeafContainer<PointT, AccumulatorPolicy, DataT> OctreeLeafContainerT;
        /** Iterator for neighbors of this leaf */
        typedef typename NeighborSetT::iterator iterator;
        typedef typename NeighborSetT::const_iterator const_iterator;
        inline iterator begin () { return (neighbors_.begin ()); }
        inline iterator end ()   { return (neighbors_.end ()); }
        inline const_iterator begin () const { return (neighbors_.begin ()); }
        inline const_iterator end () const  { return (neighbors_.end ()); }
        
        /** Returns the number of neighbors this leaf has */
        inline size_t size () const { return neighbors_.size (); }
        
        /** \brief Class initialization. */
        OctreeAdjacencyContainer ():
        OctreeLeafContainer<PointT, AccumulatorPolicy, DataT> ()
        {   
        }
        
        /** \brief Empty class deconstructor. */
        virtual ~OctreeAdjacencyContainer ()
        {
        }
        
        /** \brief Deep copy of leaf - copies all internal data */
        virtual OctreeAdjacencyContainer *
        deepCopy () const
        {
          OctreeAdjacencyContainer *new_container = new OctreeAdjacencyContainer;
          new_container->neighbors_ = this->neighbors_;
          new_container->data_ = this->data_;
          return new_container;
        }
        
        /** \brief Clear the voxel centroid */
        virtual void 
        reset ()
        {
          OctreeLeafContainerT::reset ();
          neighbors_.clear ();
        }
        
        /** \brief Add new neighbor to voxel.
          * \param[in] neighbor the new neighbor to add  
          */
        void 
        addNeighbor (OctreeAdjacencyContainer *neighbor)
        {
          neighbors_.insert (neighbor);
        }
        
        /** \brief Remove neighbor from neighbor set.
          * \param[in] neighbor the neighbor to remove
          */
        void 
        removeNeighbor (OctreeAdjacencyContainer *neighbor)
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
      class OctreeContainerPointIndices : public OctreeLeafContainer<int, IndexVectorAccumulator>
      {
      public:
        using OctreeLeafContainer<int, IndexVectorAccumulator>::getPointIndices;
        using OctreeLeafContainer<int, IndexVectorAccumulator>::getPointIndicesVector;
        
        OctreeContainerPointIndices ():
        OctreeLeafContainer<int, IndexVectorAccumulator> ()
        {   
        }
        
        /** \brief Template function which drops the templated point parameter */
        template <typename PointT>
        void
        insert (int index_arg, const PointT& temp)
        {
          OctreeLeafContainer<int, IndexVectorAccumulator>::insert (index_arg, 0);
        }
      };
      
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Point Index class wrapper maintained for compatibility
       */
      class OctreeContainerPointIndex : public OctreeLeafContainer<int, LastAccumulator>
      {
      public:
        OctreeContainerPointIndex ():
        OctreeLeafContainer<int, LastAccumulator> ()
        {   
        }
        
        /** \brief Template function which drops the templated point parameter */
        template <typename PointT>
        void
        insert (int index_arg, const PointT&)
        {
          OctreeLeafContainer<int, LastAccumulator>::insert (index_arg, -1);
        }
        
      };
      
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief @b Octree container class that does not store any information.
       * \note Can be used for occupancy trees that are used for checking only the existence of leaf nodes in the tree
       */
      class OctreeContainerEmpty
      {
      public:
        /** \brief Empty constructor. */
        OctreeContainerEmpty ()
        {
        }
        
        /** \brief Empty constructor. */
        OctreeContainerEmpty (const OctreeContainerEmpty&) 
        {
        }
        
        /** \brief Empty deconstructor. */
        virtual
        ~OctreeContainerEmpty ()
        {
        }
        
        /** \brief Octree deep copy method */
        virtual OctreeContainerEmpty *
        deepCopy () const
        {
          return (new OctreeContainerEmpty (*this));
        }
        
        /** \brief Abstract get size of container (number of DataT objects)
         * \return number of DataT elements in leaf node container.
         */
        virtual size_t
        getSize () const
        {
          return 0;
        }
        
        virtual size_t
        size () const
        {
          return 0;
        }
        
        /** \brief Abstract reset leaf node implementation. */
        virtual void
        reset ()
        {
        }
        
        /** \brief Equal comparison operator
         */
        virtual bool
        operator== (const OctreeContainerEmpty&) const
        {
          return false;
        }
        
        /** \brief Inequal comparison operator
         * \param[in] other OctreeContainerBase to compare with
         */
        bool
        operator!= (const OctreeContainerEmpty& other) const
        {
          return (!operator== (other));
        }
        
        /** \brief Empty addPointIndex implementation. This leaf node does not store any point indices.
         */
        void
        addPointIndex (int)
        {
        }
        
        /** \brief Empty getPointIndex implementation as this leaf node does not store any point indices.
         */
        int
        getPointIndex () const
        {
          return -1;
        }
        
        /** \brief Empty getPointIndices implementation as this leaf node does not store any data. \
         */
        void
        getPointIndices (std::vector<int>&) const
        {
        }
        
        template <typename PointT>
        void 
        insert (int, const PointT&)
        {
        }
      };
      
      
  }
}

#endif
