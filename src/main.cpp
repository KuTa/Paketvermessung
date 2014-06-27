#include <iostream>
#include "grabber.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/exceptions.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ctime>
#include <boost/timer.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/common/geometry.h>

using namespace std;

void findMinMaxPoints(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointXYZRGBA min_pt, pcl::PointXYZRGBA max_pt, pcl::PointXYZRGBA *min_x_pt, pcl::PointXYZRGBA *max_x_pt, pcl::PointXYZRGBA *min_y_pt, pcl::PointXYZRGBA *max_y_pt, pcl::PointXYZRGBA *min_z_pt, pcl::PointXYZRGBA *max_z_pt)
{

}

int main()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    Grabber *grabber (new Grabber);
    while (!grabber->initCamera())
    {
        std::cout << "Could not init sensor. Quit." << std::endl;
        sleep(1);
    }
    boost::timer tstart, tend;
    //time_t tstartms, tendms;
    std::cout << "Sensor initalized." << std::endl;
    char loop;
    std::cout << "Press s to stop" << std::endl;
    loop=getchar();
    float w, l, h;
    /*pcl::visualization::PCLVisualizer unViewer ("Unfiltered");
    pcl::visualization::PCLVisualizer voxelViewer ("Voxel Grid");
    pcl::visualization::PCLVisualizer passViewer ("Pass Through");
    pcl::visualization::PCLVisualizer viewer ("Outlier");
    unViewer.addCoordinateSystem(1.0f, "unfiltered");
    voxelViewer.addCoordinateSystem(1.0f, "unfiltered");
    passViewer.addCoordinateSystem(1.0f, "unfiltered");
    viewer.addCoordinateSystem(1.0f, "unfiltered");*/
    //viewer.setBackgroundColor (0, 0, 0);
    int xmin, ymin , zmin, xmax, ymax, zmax;
    while( loop != 115)
    {
        xmin = 0; ymin = 0; xmax = 0;  ymax = 0;
        /*unViewer.removeAllPointClouds();
        voxelViewer.removeAllPointClouds();
        passViewer.removeAllPointClouds();
        viewer.removeAllPointClouds();*/
        //tstart = time(0);
        tstart.restart();
        cloud = grabber->getCloud();
        /*pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
        unViewer.addPointCloud<pcl::PointXYZRGBA>(cloud, rgb,"unfiltered");
        unViewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "unfiltered");
        while (!unViewer.wasStopped ())
        {
          //range_image_widget.spinOnce ();  // process GUI events
          unViewer.spinOnce ();
          pcl_sleep(0.01);
        }*/
        pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
        vg.setInputCloud(cloud);
        float leafSize = 0.01f;
        vg.setLeafSize (leafSize, leafSize, leafSize);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloudData (new pcl::PointCloud<pcl::PointXYZRGBA>);
        vg.filter (*filteredCloudData);

        /*rgb.setInputCloud(filteredCloudData);
        voxelViewer.addPointCloud<pcl::PointXYZRGBA>(filteredCloudData, rgb,"unfiltered");
        voxelViewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "unfiltered");
        while (!voxelViewer.wasStopped ())
        {
          //range_image_widget.spinOnce ();  // process GUI events
          voxelViewer.spinOnce ();
          pcl_sleep(0.01);
        }*/
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr passx(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr passy(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr passz(new pcl::PointCloud<pcl::PointXYZRGBA>());

        pcl::PassThrough<pcl::PointXYZRGBA> pass;
        pass.setInputCloud(filteredCloudData);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-1.0f, 1.0f);
        pass.filter(*passx);
        pass.setInputCloud(passx);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-1.0f, 1.0f);
        pass.filter(*passy);
        pass.setInputCloud(passy);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0f, 1.0f);
        pass.filter(*passz);
        /*rgb.setInputCloud(passz);
        passViewer.addPointCloud<pcl::PointXYZRGBA>(passz, rgb,"unfiltered");
        passViewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "unfiltered");
        while (!passViewer.wasStopped ())
        {
          //range_image_widget.spinOnce ();  // process GUI events
          passViewer.spinOnce ();
          pcl_sleep(0.01);
        }*/

        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
        sor.setInputCloud (passz);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*filteredCloudData);

        /*rgb.setInputCloud(filteredCloudData);
        viewer.addPointCloud<pcl::PointXYZRGBA>(filteredCloudData, rgb,"unfiltered");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "unfiltered");
        while (!viewer.wasStopped ())
        {
          //range_image_widget.spinOnce ();  // process GUI events
          viewer.spinOnce ();
          pcl_sleep(0.01);
        }*/

        pcl::PointXYZRGBA min_pt, max_pt, min_x_pt, max_x_pt, min_y_pt, max_y_pt, min_z_pt, max_z_pt;
        //int x_ind, y_ind, z_ind;
        pcl::getMinMax3D(*filteredCloudData, min_pt, max_pt);
        //findMinMaxPoints(*filteredCloudData, min_pt, max_pt, *min_x_pt, *max_x_pt, *min_y_pt, *max_y_pt, *min_z_pt, *max_z_pt);
        for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = filteredCloudData->begin(); it != filteredCloudData->end(); it++)
        {
            if(it->x - 0.02 <= min_pt.x)
            {
                min_x_pt.x += it->x;
                min_x_pt.y += it->y;
                min_x_pt.z += it->z;
                xmin++;
                //std::cout << "min_x_pt:" << min_x_pt.x<< " " << min_x_pt.y << " " << min_x_pt.z << " " << xmin << std::endl;
            }
            if( it->y - 0.02 <= min_pt.y)
            {
                min_y_pt.x += it->x;
                min_y_pt.y += it->y;
                min_y_pt.z += it->z;
                ymin++;
                //std::cout << "min_y_pt:" << min_y_pt.x<< " " << min_y_pt.y << " " << min_y_pt.z << " " << ymin << std::endl;
            }
            if(min_pt.z == it->z)
            {
                min_z_pt.x = it->x;
                min_z_pt.y = it->y;
                min_z_pt.z = it->z;
                //std::cout << "min_z_pt:" << min_z_pt.x<< " " << min_z_pt.y << " " << min_z_pt.z << std::endl;
            }
            if( it->x + 0.02 >= max_pt.x)
            {
                max_x_pt.x += it->x;
                max_x_pt.y += it->y;
                max_x_pt.z += it->z;
                xmax++;
                //std::cout << "max_x_pt:" << max_x_pt.x<< " " << max_x_pt.y << " " << max_x_pt.z << " " << xmax <<  std::endl;
            }
            if(it->y + 0.02 >= max_pt.y)
            {
                max_y_pt.x += it->x;
                max_y_pt.y += it->y;
                max_y_pt.z += it->z;
                ymax++;
                //std::cout << "max_y_pt:" << max_y_pt.x<< " " << max_y_pt.y << " " << max_y_pt.z << " " << ymax <<  std::endl;
            }
            if(max_pt.z == it->x)
            {
                max_z_pt.x = it->x;
                max_z_pt.y = it->y;
                max_z_pt.z = it->z;
                //std::cout << "max_z_pt:" << max_z_pt.x<< " " << max_z_pt.y << " " << max_z_pt.z << std::endl;
            }
        }
        min_x_pt.x /= xmin;
        min_x_pt.y /= xmin;
        min_x_pt.z /= xmin;
        max_x_pt.x /= xmax;
        max_x_pt.y /= xmax;
        max_x_pt.z /= xmax;
        min_y_pt.x /= ymin;
        min_y_pt.y /= ymin;
        min_y_pt.z /= ymin;
        max_y_pt.x /= ymax;
        max_y_pt.y /= ymax;
        max_y_pt.z /= ymax;

        std::cout << "min_x_pt:" << min_x_pt.x<< " " << min_x_pt.y << " " << min_x_pt.z << std::endl;

        std::cout << "min_y_pt:" << min_y_pt.x<< " " << min_y_pt.y << " " << min_y_pt.z << std::endl;

        std::cout << "max_x_pt:" << max_x_pt.x<< " " << max_x_pt.y << " " << max_x_pt.z << std::endl;

        std::cout << "max_y_pt:" << max_y_pt.x<< " " << max_y_pt.y << " " << max_y_pt.z << std::endl;
        /*
        l = abs(sqrt((min_x_pt.x-max_x_pt.x)*(min_x_pt.x-max_x_pt.x) + (min_x_pt.y-max_x_pt.y)*(min_x_pt.y-max_x_pt.y) + (min_x_pt.z-max_x_pt.z)*(min_x_pt.z-max_x_pt.z)));
        w = abs(cos(min_y_pt.x-max_y_pt.x)*(min_y_pt.y-max_y_pt.y)*cos(min_y_pt.z-max_y_pt.z));
        h = abs(cos(min_z_pt.x-max_z_pt.x)*cos(min_z_pt.y-max_z_pt.y)*(min_z_pt.z-max_z_pt.z));
        */
        l = (max_pt.x-min_pt.x);
        w = (max_pt.y-min_pt.y);
        h = (1.0 - min_pt.z);
        //tend = time(0);


        std::cout<< "It took "<< tstart.elapsed()  <<" second(s)." << std::endl <<" Paketsize width: " << w << " length: " << l << " heigth: " << h <<  " Press s to stop" << std::endl;

        float l1 = sqrt((min_x_pt.x - max_y_pt.x)*(min_x_pt.x - max_y_pt.x) + (min_x_pt.y - max_y_pt.y)*(min_x_pt.y - max_y_pt.y)) + 0.06;
        float w1 = sqrt((max_x_pt.x - max_y_pt.x)*(max_x_pt.x - max_y_pt.x) + (max_x_pt.y - max_y_pt.y)*(max_x_pt.y - max_y_pt.y)) + 0.06;

        //l = pcl::geometry::distance(min_x_pt, max_y_pt);
        //w = pcl::geometry::distance(min_x_pt, min_y_pt);
        std::cout<< "It took "<< tstart.elapsed() <<" second(s)." << std::endl <<" Paketsize width: " << w1 << " length: " << l1 << " heigth: " << h <<  " Press s to stop" << std::endl;

        float l2 = sqrt((max_x_pt.x - min_y_pt.x)*(max_x_pt.x - min_y_pt.x) + (max_x_pt.y - min_y_pt.y)*(max_x_pt.y - min_y_pt.y)) + 0.06;
        float w2 = sqrt((min_x_pt.x - min_y_pt.x)*(min_x_pt.x - min_y_pt.x) + (min_x_pt.y - min_y_pt.y)*(min_x_pt.y - min_y_pt.y)) + 0.06;
        //l = pcl::geometry::distance(max_x_pt, min_y_pt);
        //w = pcl::geometry::distance(max_x_pt, max_y_pt);
        std::cout<< "It took "<< tstart.elapsed() <<" second(s)." << std::endl <<" Paketsize width: " << w2 << " length: " << l2 << " heigth: " << h <<  " Press s to stop" << std::endl;

        float lges = (l1 + l2) /2;
        float wges = (w1 + w2) /2;
        std::cout<< "It took "<< tstart.elapsed() <<" second(s)." << std::endl <<" Paketsize width: " << wges << " length: " << lges << " heigth: " << h <<  " Press s to stop" << std::endl;

        loop = getchar();
    }
    return 0;
}

