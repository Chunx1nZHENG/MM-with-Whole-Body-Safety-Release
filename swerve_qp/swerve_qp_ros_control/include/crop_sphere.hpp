#ifndef CROPSPHERE_H
#define CROPSPHERE_H

#include <pcl/point_types.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

template<typename PointT>
class CropSphere : public pcl::FilterIndices<PointT>
{
  using pcl::Filter<PointT>::getClassName;
  typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;
  
public:
  typedef boost::shared_ptr< CropSphere<PointT> > Ptr;
  typedef boost::shared_ptr< const CropSphere<PointT> > ConstPtr;
      
  CropSphere(bool extract_removed_indices = false)
    : pcl::FilterIndices<PointT>::FilterIndices (extract_removed_indices)
    , center_pt_(Eigen::Vector4f(0, 0, 0, 1))
    , radius_(1)
    , transform_ (Eigen::Affine3f::Identity ())
  {
    filter_name_ = "CropSphere";
  }
  
  inline void setCenter(const Eigen::Vector4f &center_pt) { center_pt_ = center_pt; }
  inline Eigen::Vector4f getCenter() const { return center_pt_; }
  
  inline void setRadius(double radius) { radius_ = radius; }  
  inline double getRadius() const { return radius_; }

   /** \brief Set a transformation that should be applied to the cloud before filtering
    * \param[in] transform an affine transformation that needs to be applied to the cloud before filtering
    */
  inline void
  setTransform (const Eigen::Affine3f &transform)
  {
    transform_ = transform;
  }

  /** \brief Get the value of the transformation parameter, as set by the user. */
  inline Eigen::Affine3f
  getTransform () const
  {
    return (transform_);
  }
      
protected:
  using pcl::PCLBase<PointT>::input_;
  using pcl::PCLBase<PointT>::indices_;
  using pcl::Filter<PointT>::filter_name_;
  using pcl::FilterIndices<PointT>::negative_;
  using pcl::FilterIndices<PointT>::keep_organized_;
  using pcl::FilterIndices<PointT>::user_filter_value_;
  using pcl::FilterIndices<PointT>::extract_removed_indices_;
  using pcl::FilterIndices<PointT>::removed_indices_;
      
  void applyFilter(PointCloud &output);
  void applyFilter(std::vector<int> &indices);
      
  Eigen::Vector4f center_pt_;
  double radius_;
  
  /** \brief The affine transform applied to the cloud. */
  Eigen::Affine3f transform_;
};

template<typename PointT> inline void
CropSphere<PointT>::applyFilter(PointCloud &output)
{
  std::vector<int> indices;
  if (keep_organized_)
  {
    bool temp = extract_removed_indices_;
    extract_removed_indices_ = true;
    applyFilter (indices);
    extract_removed_indices_ = temp;

    output = *input_;
    for (int rii = 0; rii < static_cast<int> (removed_indices_->size ()); ++rii)  // rii = removed indices iterator
      output.points[(*removed_indices_)[rii]].x = output.points[(*removed_indices_)[rii]].y = output.points[(*removed_indices_)[rii]].z = user_filter_value_;
    if (!pcl_isfinite (user_filter_value_))
      output.is_dense = false;
  }
  else
  {
    output.is_dense = true;
    applyFilter (indices);
    pcl::copyPointCloud (*input_, indices, output);
  }
}

template<typename PointT> inline void
CropSphere<PointT>::applyFilter(std::vector<int> &indices)
{  
  indices.resize (input_->points.size ());
  removed_indices_->resize (input_->points.size ());
  int indices_count = 0;
  int removed_indices_count = 0;
  
  bool transform_matrix_is_identity = transform_.matrix ().isIdentity ();
  
  const double radius2 = radius_ * radius_;

  for (size_t index = 0; index < indices_->size (); ++index)
  {
    // Check if the point is invalid
    if (!input_->is_dense && !pcl::isFinite (input_->points[index]))
      continue;

    // Get local point
    PointT local_pt = input_->points[(*indices_)[index]];

    // Transform point to world space
    if (!transform_matrix_is_identity)
      local_pt = pcl::transformPoint<PointT> (local_pt, transform_);

    double dx = center_pt_[0] - local_pt.x;
    double dy = center_pt_[1] - local_pt.y;
    double dz = center_pt_[2] - local_pt.z;
    
    dx *= dx;
    dy *= dy;
    dz *= dz;

    // If outside the cropshere
    if (dx + dy + dz > radius2)
    {
      if (negative_)
        indices[indices_count++] = (*indices_)[index];
      else if (extract_removed_indices_)
        (*removed_indices_)[removed_indices_count++] = static_cast<int> (index);
    }
    // If inside the cropsphere
    else
    {
      if (negative_ && extract_removed_indices_)
        (*removed_indices_)[removed_indices_count++] = static_cast<int> (index);
      else if (!negative_) 
        indices[indices_count++] = (*indices_)[index];
    }
  }
  indices.resize (indices_count);
  removed_indices_->resize (removed_indices_count);
}

#endif // CROPSPHERE_H