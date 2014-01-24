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

namespace pcl
{
  namespace octree
  {
    
    //! Forward Declaration for DataT classes
    template <typename PointT> class AveragePoint;
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree container class that can serve as a base to construct own leaf node container classes.
     *  \author Julius Kammerl (julius@kammerl.de)
     */
    class OctreeContainerBase
    {
    public:
      /** \brief Empty constructor. */
      OctreeContainerBase ()
      {
      }

      /** \brief Empty constructor. */
      OctreeContainerBase (const OctreeContainerBase&)
      {
      }

      /** \brief Empty deconstructor. */
      virtual
      ~OctreeContainerBase ()
      {
      }

      /** \brief Equal comparison operator
       */
      virtual bool
      operator== (const OctreeContainerBase&) const
      {
        return false;
      }

      /** \brief Inequal comparison operator
       * \param[in] other OctreeContainerBase to compare with
       */
      bool
      operator!= (const OctreeContainerBase& other) const
      {
        return (!operator== (other));
      }

      /** \brief Pure abstract method to get size of container (number of indices)
       * \return number of points/indices stored in leaf node container.
       */
      virtual size_t
      getSize () const
      {
        return 0u;
      }

      /** \brief Pure abstract reset leaf node implementation. */
      virtual void
      reset () = 0;

      /** \brief Empty addPointIndex implementation. This leaf node does not store any point indices.
       */
      void
      addPointIndex (const int&)
      {
      }

      /** \brief Empty getPointIndex implementation as this leaf node does not store any point indices.
       */
      void
      getPointIndex (int&) const
      {
      }

      /** \brief Empty getPointIndices implementation as this leaf node does not store any data. \
            */
      void
      getPointIndices (std::vector<int>&) const
      {
      }

    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree container class that does not store any information.
     * \note Can be used for occupancy trees that are used for checking only the existence of leaf nodes in the tree
     * \author Julius Kammerl (julius@kammerl.de)
     */

    class OctreeContainerEmpty : public OctreeContainerBase
    {
    public:
      /** \brief Empty constructor. */
      OctreeContainerEmpty () :
          OctreeContainerBase ()
      {
      }

      /** \brief Empty constructor. */
      OctreeContainerEmpty (const OctreeContainerEmpty&) :
          OctreeContainerBase ()
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

      /** \brief Abstract reset leaf node implementation. */
      virtual void
      reset ()
      {

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
        assert("getPointIndex: undefined point index");
        return -1;
      }

      /** \brief Empty getPointIndices implementation as this leaf node does not store any data. \
            */
      void
      getPointIndices (std::vector<int>&) const
      {
      }

    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree container class that does store a single point index.
     * \note Enables the octree to store a single DataT element within its leaf nodes.
     * \author Julius Kammerl (julius@kammerl.de)
     */
      class OctreeContainerPointIndex : public OctreeContainerBase
      {
      public:
        /** \brief Empty constructor. */
        OctreeContainerPointIndex () :
            OctreeContainerBase (), data_ ()
        {
          reset ();
        }

        /** \brief Empty constructor. */
        OctreeContainerPointIndex (const OctreeContainerPointIndex& source) :
            OctreeContainerBase (), data_ (source.data_)
        {
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeContainerPointIndex ()
        {
        }

        /** \brief Octree deep copy method */
        virtual OctreeContainerPointIndex*
        deepCopy () const
        {
          return (new OctreeContainerPointIndex (*this));
        }

        /** \brief Equal comparison operator
         * \param[in] other OctreeContainerBase to compare with
         */
        virtual bool
        operator== (const OctreeContainerBase& other) const
        {
          const OctreeContainerPointIndex* otherConDataT = dynamic_cast<const OctreeContainerPointIndex*> (&other);

          return (this->data_ == otherConDataT->data_);
        }

        /** \brief Add point index to container memory. This container stores a only a single point index.
         * \param[in] data_arg index to be stored within leaf node.
         */
        void
        addPointIndex (int data_arg)
        {
          data_ = data_arg;
        }

        /** \brief Retrieve point index from container. This container stores a only a single point index
         * \return index stored within container.
         */
        int
        getPointIndex () const
        {
          return data_;
        }

        /** \brief Retrieve point indices from container. This container stores only a single point index
         * \param[out] data_vector_arg vector of point indices to be stored within data vector
         */
        void
        getPointIndices (std::vector<int>& data_vector_arg) const
        {
          if (data_>=0)
          data_vector_arg.push_back (data_);
        }

        /** \brief Get size of container (number of DataT objects)
         * \return number of DataT elements in leaf node container.
         */
        size_t
        getSize () const
        {
          return data_<0 ? 0 : 1;
        }

        /** \brief Reset leaf node memory to zero. */
        virtual void
        reset ()
        {
          data_ = -1;
        }
      protected:
        /** \brief Point index stored in octree. */
        int data_;
      };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree container class that does store a vector of point indices.
     * \note Enables the octree to store multiple DataT elements within its leaf nodes.
     * \author Julius Kammerl (julius@kammerl.de)
     */
      class OctreeContainerPointIndices : public OctreeContainerBase
      {
      public:
        /** \brief Empty constructor. */
        OctreeContainerPointIndices () :
          OctreeContainerBase (), leafDataTVector_ ()
        {
        }

        /** \brief Empty constructor. */
        OctreeContainerPointIndices (const OctreeContainerPointIndices& source) :
            OctreeContainerBase (), leafDataTVector_ (source.leafDataTVector_)
        {
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeContainerPointIndices ()
        {
        }

        /** \brief Octree deep copy method */
        virtual OctreeContainerPointIndices *
        deepCopy () const
        {
          return (new OctreeContainerPointIndices (*this));
        }

        /** \brief Equal comparison operator
         * \param[in] other OctreeContainerDataTVector to compare with
         */
        virtual bool
        operator== (const OctreeContainerBase& other) const
        {
          const OctreeContainerPointIndices* otherConDataTVec = dynamic_cast<const OctreeContainerPointIndices*> (&other);

          return (this->leafDataTVector_ == otherConDataTVec->leafDataTVector_);
        }

        /** \brief Add point index to container memory. This container stores a vector of point indices.
         * \param[in] data_arg index to be stored within leaf node.
         */
        void
        addPointIndex (int data_arg)
        {
          leafDataTVector_.push_back (data_arg);
        }

        /** \brief Retrieve point index from container. This container stores a vector of point indices.
         * \return index stored within container.
         */
        int
        getPointIndex ( ) const
        {
          return leafDataTVector_.back ();
        }

        /** \brief Retrieve point indices from container. This container stores a vector of point indices.
         * \param[out] data_vector_arg vector of point indices to be stored within data vector
         */
        void
        getPointIndices (std::vector<int>& data_vector_arg) const
        {
          data_vector_arg.insert (data_vector_arg.end (), leafDataTVector_.begin (), leafDataTVector_.end ());
        }

        /** \brief Retrieve reference to point indices vector. This container stores a vector of point indices.
         * \return reference to vector of point indices to be stored within data vector
         */
        std::vector<int>&
        getPointIndicesVector ()
        {
          return leafDataTVector_;
        }

        /** \brief Get size of container (number of indices)
         * \return number of point indices in container.
         */
        size_t
        getSize () const
        {
          return leafDataTVector_.size ();
        }

        /** \brief Reset leaf node. Clear DataT vector.*/
        virtual void
        reset ()
        {
          leafDataTVector_.clear ();
        }

      protected:
        /** \brief Leaf node DataT vector. */
        std::vector<int> leafDataTVector_;
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
      template<typename PointInT, typename DataT = AveragePoint<PointInT> >
      class OctreeAdjacencyContainer : public OctreeContainerBase
      {
      public:
        typedef std::set<OctreeAdjacencyContainer*> NeighborSetT;
        //iterators to neighbors
        typedef typename NeighborSetT::iterator iterator;
        typedef typename NeighborSetT::const_iterator const_iterator;
        inline iterator begin () { return (neighbors_.begin ()); }
        inline iterator end ()   { return (neighbors_.end ()); }
        inline const_iterator begin () const { return (neighbors_.begin ()); }
        inline const_iterator end () const  { return (neighbors_.end ()); }
        //size of neighbors
        inline size_t size () const { return neighbors_.size (); }
        //insert for neighbors
        inline std::pair<iterator, bool> insert (OctreeAdjacencyContainer* neighbor) { return neighbors_.insert (neighbor); }
        
        /** \brief Class initialization. */
        OctreeAdjacencyContainer () :
        OctreeContainerBase ()
        {
          this->reset();       
        }
        
        /** \brief Empty class deconstructor. */
        virtual ~OctreeAdjacencyContainer ()
        {
        }
        
        /** \brief deep copy function */
        virtual OctreeAdjacencyContainer *
        deepCopy () const
        {
          OctreeAdjacencyContainer *new_container = new OctreeAdjacencyContainer;
          new_container->setNeighbors (this->neighbors_);
          new_container->setData (this->data_);
          return new_container;
        }
        
        /** \brief Add new point to container- this just counts points
         * \note To actually store data in the leaves, need to specialize this
         * for your point and data type as in supervoxel_clustering.hpp
         * \param[in] new_point the new point to add  
         */
        void 
        addPoint (const PointInT& new_point)
        {
          data_.add (new_point);
        }
        
        /** \brief Function for working on data added. Base implementation does nothing 
         * */
        void
        computeData ()
        {
          data_.compute ();
        }
        
        /** \brief Clear the voxel centroid */
        virtual void 
        reset ()
        {
          neighbors_.clear ();
          data_ = DataT ();
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
        
        /** \brief Returns the number of neighbors this leaf has
          *  \returns number of neighbors
          */
        size_t 
        getNumNeighbors () const
        {
          return neighbors_.size ();
        }
        
        /** \brief Sets the whole neighbor set
         * \param[in] neighbor_arg the new set
         */
        void
        setNeighbors (const NeighborSetT &neighbor_arg)
        {
          neighbors_ = neighbor_arg;
        }
        
        /** \brief Returns a reference to the data member to access it without copying */
        DataT&
        getData () { return data_; }
        
        /** \brief Sets the data member
         *  \param[in] data_arg New value for data
         */
        void
        setData (const DataT& data_arg) { data_ = data_arg;}
        
      protected:
        NeighborSetT neighbors_;
        DataT data_;
      };
      
  }
}

#endif
