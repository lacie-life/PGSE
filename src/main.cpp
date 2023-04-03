#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std::literals::chrono_literals;

int main(int argc, char** argv) {
    std::cout << "Hello, World!" << std::endl;

    // Load the first file
    pcl::PCLPointCloud2 cloud;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    print_highlight ("Loading ");
    print_value ("%s ", argv[1]);

    pcl::PLYReader ply_reader;
    pcl::PCDReader pcd_reader;
    if (ply_reader.read(argv[1], cloud) < 0)
        return (false);

    print_info ("[done, ");
    print_info (" ms : ");
    print_value ("%d", cloud.width * cloud.height);
    print_info (" points]\n");
    print_info ("Available dimensions: ");
    print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

    pcl::fromPCLPointCloud2 (cloud, *point_cloud_ptr);

    // Viewer setup
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("PGSE"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }

    return 0;
}
