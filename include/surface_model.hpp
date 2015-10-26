/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald. Ekaterina Potapova
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */

#ifndef SURFACE_SURFACEMODEL_HPP
#define SURFACE_SURFACEMODEL_HPP

#ifndef MODEL_NURBS
#define MODEL_NURBS 20  // Should be higher than pcl::SACMODEL_STICK
#endif

#include <vector>
#include <set>
#include <map>

#include <opencv2/opencv.hpp>

#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>

namespace surface
{

/** @brief Surface model: describing a parametrized surface **/
class SurfaceModel
{
  public:
    typedef std::set<unsigned>::iterator NeighborIter;
    typedef boost::shared_ptr< ::surface::SurfaceModel> Ptr;
    typedef boost::shared_ptr< ::surface::SurfaceModel const> ConstPtr;

  public:
    int idx;                                            ///< for merging in surface modeling
    int type;                                           ///< type of surface model (plane, NURBS, (same than pcl::SACMODEL))
    int label;                                          ///< object assignment label
    int label_ass;                                      ///< object assignment label for assembly level
    bool selected;                                      ///< if surface is used for surface modeling
    bool valid;                                         ///< if surface is not valid, than it should be deleted
    bool initialized;                                    ///< if surface is not initialized
    bool isNew;                                          ///< if surface just added of was modified
    int segmented_number;                                          ///< if surface just added of was modified
    double savings;                                     ///< for surface modeling
    Eigen::Matrix4d pose;                               ///< transformation from View to SurfaceModel coordinate frame

    std::vector<int> indices;                           ///< index list for 2D data
    std::vector<double> error;                          ///< error of each point
    std::vector<double> probs;                          ///< probability of each points
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > normals;

    std::vector<float> coeffs;                          ///< model coefficients
    std::set<unsigned> neighbors2D;                     ///< 2D neighbors of patch (not related to idx => related with view->surfaces[i])
    std::map<unsigned, unsigned> neighbors2DNrPixel;    ///< Number of neighboring pixels for 2D neighbors
    std::set<unsigned> neighbors3D;                     ///< 3D neighbors of patch (not related to idx=> related with view->surfaces[i])
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > nurbs_params;

    float saliency;

    SurfaceModel(): idx(-1), label(-1), label_ass(-1), selected(true), valid(true), initialized(false), isNew(true), segmented_number(-1) {}
    SurfaceModel(int _idx) : idx(_idx), label(-1), label_ass(-1), selected(true), valid(true), initialized(false), isNew(true), segmented_number(-1) {}

    void addTo(SurfaceModel &model)
    {
      for(unsigned int i = 0; i < indices.size(); i++)
      {
        model.indices.push_back(indices.at(i));
        model.normals.push_back(normals.at(i));
      }
      
    }
    
    void Print() {
      printf("[SurfaceModel]\n  ");
      printf("  idx: %d\n  ", idx);
      printf("  type: %d\n  ", type);
      printf("  label: %d\n  ", label);
      printf("  label_ass: %d\n  ", label_ass);
      printf("  selected: %d\n  ", selected);
      printf("  valid: %d\n  ", selected);
      printf("  savings: %f\n  ", savings);
      printf("  indices.size: %lu\n  ", indices.size());
      printf("  errors.size: %lu\n  ", error.size());
      printf("  probs.size: %lu\n  ", probs.size());
      printf("  coeffs.size %lu: ", coeffs.size());
      for(size_t i=0; i<coeffs.size(); i++)
        printf("%f ", coeffs[i]);
      printf("\n  ");
      printf("  neighbors2D [%lu]: ", neighbors2D.size());
      for(NeighborIter i=neighbors2D.begin(); i!=neighbors2D.end(); i++)
        printf("%d ", (int)*i);
      printf("\n  ");
      printf("  neighbors3D [%lu]: ", neighbors3D.size());
      for(NeighborIter i=neighbors3D.begin(); i!=neighbors3D.end(); i++)
        printf("%d ", (int)*i);
      printf("\n  ");
      printf("  saliency: %f\n", saliency);
      printf("\n");
    }
    
};

/* --------------------------- inline --------------------------- */

/** create a deep copy of surface pointers **/
inline std::vector<surface::SurfaceModel::Ptr>
deepCopy(const std::vector<surface::SurfaceModel::Ptr> &_surfaces) {
  std::vector<surface::SurfaceModel::Ptr> surfaces;
  for(unsigned i=0; i<_surfaces.size(); i++) {
    surface::SurfaceModel::Ptr c_surface;
    c_surface.reset(new surface::SurfaceModel());
    (*c_surface) = *_surfaces[i];
    surfaces.push_back(c_surface);
  }
  return surfaces;
}

/** create a deep copy of surface pointers of specific type**/
inline std::vector<surface::SurfaceModel::Ptr>
deepCopyType(const std::vector<surface::SurfaceModel::Ptr> &_surfaces, int type) {
  std::vector<surface::SurfaceModel::Ptr> surfaces;
  for(unsigned i=0; i<_surfaces.size(); i++) {
    if(_surfaces[i]->type == type) {
      surface::SurfaceModel::Ptr c_surface;
      c_surface.reset(new surface::SurfaceModel());
      (*c_surface) = *_surfaces[i];
      surfaces.push_back(c_surface);
    }
  }
  return surfaces;
}


}

#endif

