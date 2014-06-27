#include "grabber.h"
#include <pcl/filters/voxel_grid.h>

Grabber::Grabber()
{
    //boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> > cloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  
}

Grabber::~Grabber()
{
    if(interface != 0)
    {
        interface->stop();
        interface.reset();
    }
}
/*
 * Function to initialize the Camera with OpenNI2Grabber
 * camera is always declared as #1, because a single camera operation is assumed with Raspberry Pi
 * camera is started if possible
 * if not possible IOException with error
 */
bool Grabber::initCamera()
{
 try{
   interface = boost::shared_ptr<pcl::io::OpenNI2Grabber>(new pcl::io::OpenNI2Grabber("#1"));
   boost::function<void (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&)> g =
           boost::bind (&Grabber::updatePtr, this, _1);
   interface->registerCallback(g);
   interface->start();
 }
 catch(pcl::IOException e)
 {
    std::cout << "Sensor initialization error: " << e.what() << std::endl;
    interface.reset();
    return false;
 }
    std::cout << "init complete" << std::endl;
 return true;
}
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Grabber::getCloud()
{
/*    updatePtr(cloudPtr);
    mut.lock();
    while(cloudPtr == 0)
    {
        updatePtr(cloudPtr);
        std::cout << "cloudPtr empty" << std::endl;
    }

    for (unsigned yIdx = 0; yIdx < cloudPtr->height; ++yIdx)
    {
      for (unsigned xIdx = 0; xIdx < cloudPtr->width; ++xIdx)
      {
        std::cout << "Point[" << xIdx << "][" << yIdx << "]= x: " <<  cloudPtr->points[yIdx*xIdx + xIdx].x <<
                      " y: " <<  cloudPtr->points[yIdx*xIdx + xIdx].y << " z: " <<  cloudPtr->points[yIdx*xIdx + xIdx].z <<
                     " r: " <<  cloudPtr->points[yIdx*xIdx + xIdx].r << " g: " <<  (cloudPtr->points[yIdx*xIdx + xIdx].g) <<
                     " b: " <<  cloudPtr->points[yIdx*xIdx + xIdx].b << std::endl;
      }
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudData(new pcl::PointCloud<pcl::PointXYZRGBA>());
    std::vector<int> indices;
    *cloudData = *cloudPtr;
    pcl::removeNaNFromPointCloud(*cloudPtr, *cloudData, indices);
    std::cout << "NaN removed" << std::endl;
    int i = 0;
    for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloudData->begin(); it != cloudData->end(); it++)
    {*/
        /*std::cout << "Point[" << xIdx << "][" << yIdx << "]= x: " <<  cloudPtr->points[yIdx*xIdx + xIdx].x <<
                      " y: " <<  cloudPtr->points[yIdx*xIdx + xIdx].y << " z: " <<  cloudPtr->points[yIdx*xIdx + xIdx].z <<
                     " r: " <<  cloudPtr->points[yIdx*xIdx + xIdx].r << " g: " <<  (cloudPtr->points[yIdx*xIdx + xIdx].g) <<
                     " b: " <<  cloudPtr->points[yIdx*xIdx + xIdx].b << std::endl;
        i++;
        it->r = 255;
        it->g = 255;
        it->b = 255;
        std::cout << "Point" << "[" << i << "]" << " x: " << it->x << " y: " << it->y << " z: " << it->z <<  " r: " << it->r << " g: " << it->g << " b: " << it->b << std::endl;

    }
    mut.unlock();
    return cloudData;*/
    updatePtr(cloudPtr);
    while(cloudPtr == 0)
    {
        updatePtr(cloudPtr);
    //    std::cout << "cloudPtr empty" << std::endl;
    }
    mut.lock();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudDataFront(new pcl::PointCloud<pcl::PointXYZRGBA>());
    //*cloudDataFront = *cloudPtr;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloudPtr, *cloudDataFront, indices);
    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
     sor.setInputCloud (cloudDataFront);
     sor.setLeafSize (0.01f, 0.01f, 0.01f);
     sor.filter (*cloudDataFront);
    for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloudDataFront->begin(); it != cloudDataFront->end(); )
    {
        if( it->x < -6.0 || it->x > 6.0 || it->y < -6.0 || it->y > 6.0 || it->z < -6.0 || it->z > 6.0){
            //std::cout << "Filter ";
            it = cloudDataFront->erase(it);
        }
        else
        {
            //std::cout << "iterator ";
            ++it;
        }
    }
    mut.unlock();
    //*cloudDataFront = *cloudPtrFront;
    //pcl::transformPointCloud(*cloudPtrFront, *cloudDataFront, front2World);
    return cloudDataFront;
}

void Grabber::updatePtr(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >& cloud)
{
    mut.lock();
    this->cloudPtr = cloud;
    mut.unlock();
}
