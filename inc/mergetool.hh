#pragma once

#include <algorithm>
#include <vector>

#include "flann/flann.hpp"
#include "flann/util/matrix.h"

#include "types.hh"
#include "utils.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  template <typename T, unsigned int n>
  class MergeTool
  {
  private:
    //--------------------------------------------------------------------------//
    typedef flann::Index<flann::L2<real>> FLANNKdTree;
    static int m_dim;
    static bool m_print_info;
    //--------------------------------------------------------------------------//
  public:
    //--------------------------------------------------------------------------//
    MergeTool<T, n>() { m_dim = n; }
    //--------------------------------------------------------------------------//
    ~MergeTool() {}
    //--------------------------------------------------------------------------//
    /** Gets a bunch of points, that have to be merged, based on their distance.
     * \param points - a container with the positions of the points in nD
     * \param minDist - below this distance two points get merged. */
    static std::vector<T> mergeClosePoints(const std::vector<T> &points,
                                           const real &minDist,
                                           bool print_info = false)
    {
      //--- no points to merge here
      if (1 >= points.size())
        return points;

      m_print_info = print_info;
      std::vector<T> loc_points = points;
      std::vector<bool> keep_points;
      keep_points.resize(loc_points.size());
      for (size_t i = 0; i < keep_points.size(); i++)
      {
        keep_points[i] = true;
      }
      int left_points =
          static_cast<int>(loc_points.size()); // counts how many points are there
      int merge_count = 0;
      // compute the kd tree which is used for nearest neighbour search
      FLANNKdTree *p_kd_tree = computeKDTree(&loc_points);
      //--- we do an iterative merging of close enough points
      bool merged = true;
      do
      {
        bool recompute = loc_points.size() > pow(2, 8);
        merged = false;
        merge_count = 0;
        if (m_print_info)
        {
          std::cout << "compute merge lists. \n";
        }
        auto mergeLists = computeMergeLists(
            loc_points, keep_points, left_points, minDist, *p_kd_tree);
        // compute the average/merged position for each merge list
        std::vector<T> recompute_points;
        for (auto idxList : mergeLists)
        {
          // check if there is something to merge at all
          if (0 == idxList.size())
          {
            continue;
          }
          if (1 == idxList.size())
          {
            if (recompute)
            {
              recompute_points.push_back(loc_points[idxList.front()]);
            }
            continue;
          }
          // merge if necessary
          merged = true;
          auto tmpPnt = T(0.0);
          for (auto idx : idxList)
          {
            tmpPnt += loc_points[idx];
            p_kd_tree->removePoint(idx);
            keep_points[idx] = false;
            left_points--;
            merge_count++;
          }
          tmpPnt /= static_cast<real>(idxList.size());
          loc_points.push_back(tmpPnt);
          keep_points.push_back(true);
          left_points++;
          if (!recompute)
          {
            flann::Matrix<real> new_point(&(loc_points.back()[0]), 1, m_dim);
            p_kd_tree->addPoints(new_point, 2.0);
          }
          else
          {
            recompute_points.push_back(tmpPnt);
          }
        }
        if (recompute)
        { // due to a bug in FLANN we need to recompute by too much
          // points
          loc_points = recompute_points;
          keep_points.resize(loc_points.size());
          for (size_t i = 0; i < keep_points.size(); ++i)
          {
            keep_points[i] = true;
          }
          left_points = static_cast<int>(loc_points.size());
          p_kd_tree = computeKDTree(&loc_points);
        }
        if (m_print_info)
        {
          std::cout << "merged " << merge_count << " points " << left_points
                    << " of " << points.size() << " left\n";
        }
      } while (merged); // repeat the process until nothing was merge recently

      //--- create a vector with all remaining points
      std::vector<T> result;
      result.reserve(2 * points.size() - loc_points.size());
      for (size_t i = 0; i < keep_points.size(); i++)
      {
        if (keep_points[i])
        {
          result.push_back(loc_points[i]);
        }
      }

      delete p_kd_tree;
      return result;
    }
    //--------------------------------------------------------------------------//
  public:
    //--------------------------------------------------------------------------//
    static std::vector<std::vector<int>> computeMergeLists(
        const std::vector<T> &points,
        const std::vector<bool> &handle_pnt_vec,
        const int &left_points,
        const real &minDist,
        const FLANNKdTree &kd_tree)
    {
      //--- create vector with the results
      std::vector<std::vector<int>> mergeLists;
      // contains a flag for each point, if this point gets merged already
      std::vector<bool> idxIsMerged;
      idxIsMerged.resize(points.size());
      for (size_t i = 0; i < idxIsMerged.size(); ++i)
        idxIsMerged[i] = false;

      //--- find cmpCount nearest neighbours
      int cmpMax = std::min(left_points / 2, 300);
      int cmpCount = std::max(cmpMax, 8);
      // if there are less than 8 points available we cannot search for 8
      // neighbors, in this case we search for as many points as are available
      cmpCount = (8 == cmpCount) ? std::min(8, left_points) : cmpCount;

      //--- compute merge list
      // search the n nearest neighbors for each query point
      for (unsigned int i = 0; i < points.size(); ++i)
      {
        // check if there already is a merge partner for this point
        if (idxIsMerged[i] || !handle_pnt_vec[i])
          continue;
        // get a list with neighbours that are merge candidates
        auto closeIndexes =
            computeCloseNeighbours(kd_tree, points[i], cmpCount, minDist);
        // update the merge list
        std::vector<int> pntMergeList;
        for (auto &idx : closeIndexes)
          if (false == idxIsMerged[idx])
          {
            idxIsMerged[idx] = true;
            pntMergeList.push_back(idx);
          }
        mergeLists.push_back(pntMergeList);
      }
      return mergeLists;
    }
    //--------------------------------------------------------------------------//
    static std::vector<int> computeCloseNeighbours(const FLANNKdTree &kd_tree,
                                                   const T &pos,
                                                   const int &count,
                                                   const real &minDist)
    {
      std::vector<int> closeIndexes;
      //--- allocate memory for results
      std::vector<std::vector<int>> idx_vec;
      std::vector<std::vector<real>> dist_vecs;
      T temp_anker = pos;
      flann::Matrix<real> queryPt(&(temp_anker[0]), 1, m_dim);

      //--- get the n neighbours and check their distances
      kd_tree.knnSearch(
          queryPt, idx_vec, dist_vecs, count, flann::SearchParams(256));
      for (auto j = 0; j < count; ++j)
        if (dist_vecs[0][j] > minDist || dist_vecs[0][j] < 0.0)
          continue;
        else
          closeIndexes.push_back(idx_vec[0][j]);

      return closeIndexes;
    }
    //--------------------------------------------------------------------------//
    static FLANNKdTree *computeKDTree(std::vector<T> *p_points)
    {
      // setup up a meshed kd tree with a single point
      unsigned int pntCount = static_cast<unsigned int>(p_points->size());
      flann::Matrix<real> tree_data(&((*p_points)[0][0]), pntCount, m_dim);
      FLANNKdTree *p_kd_tree =
          new FLANNKdTree(tree_data, flann::KDTreeIndexParams(16));
      p_kd_tree->buildIndex();

      return p_kd_tree;
    }
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
  template <typename T, unsigned int n>
  int MergeTool<T, n>::m_dim = n;
  template <typename T, unsigned int n>
  bool MergeTool<T, n>::m_print_info = false;
  //--------------------------------------------------------------------------//
  typedef MergeTool<Vec3r, 3> MergeTool3D;
  typedef MergeTool<Vec4r, 4> MergeTool4D;
  typedef MergeTool<Vec5r, 5> MergeTool5D;
  typedef MergeTool<Vec6r, 6> MergeTool6D;
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//