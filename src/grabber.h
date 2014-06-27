#ifndef GRABBER_H
#define GRABBER_H


#include <pcl/io/openni2_grabber.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

class Grabber
{
/* 
  public functions
*/
public:
  Grabber();
  ~Grabber();
  bool initCamera();
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud();

private:
  /*
   * private funtions
   * 
   */
  void updatePtr(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >& cloud);
  /*
   * private variables
   * 
   */
  boost::shared_ptr<pcl::Grabber> interface;
  boost::mutex mut;
  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> > cloudPtr;
};
#endif // GRABBER_H
