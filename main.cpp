#include <iostream>
#include <fstream>
#include <sstream>
#include <random>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

void addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                       PointCloudT &adjacent_supervoxel_centers,
                                       std::string supervoxel_name,
                                       pcl::visualization::PCLVisualizer::Ptr & viewer);

struct RGB {
  uint8_t r;
  uint8_t b;
  uint8_t g;
};

RGB random_color();

RGB random_color()
{
  RGB rgb;
  rgb.r = rand()%256;
  rgb.g = rand()%256;
  rgb.b = rand()%256;
  return rgb;
}

std::vector<RGB> random_colors(PointLCloudT cloud, int n_supervoxels) {
    // make some colors
    std::vector<RGB> colors(cloud.size());
    std::mt19937 random;
    std::vector<RGB> supervoxel_colors(n_supervoxels);
    for (int i = 0; i < n_supervoxels; ++i) {
        supervoxel_colors[i] = random_color();
    }
    for (int i=0; i < cloud.size(); i++) {
        colors[i] = supervoxel_colors[cloud[i].label];
    }
    return colors;
}

bool ReadXYZPoints(const char* filename, pcl::PointCloud<PointT>::Ptr cloud) {
    std::ifstream in(filename);
    if (!in) {
        std::cout << "Cannot open XYZ file '" << filename << "' for reading.";
        return false;
    }

    int n_lines = 0;
    std::string line;
    float x, y, z;
    uint64_t r, g, b;
    while (std::getline(in, line)) {
        std::istringstream is(line);

        if (!(is >> x) || !(is >> y) || !(is >> z) ||
            !(is >> r) || !(is >> g) || !(is >> b)) {
            std::cout << "Invalid XYZ format at line: " << n_lines++ << std::endl;
            in.close();
            return false;
        }
        // std::cout << " " << r << " " << g << " "<< b << std::endl;
        PointT p{x, y, z, (uint8_t) r, (uint8_t) g, (uint8_t)b, 1};
        cloud->push_back(p);
    }

    return true;
}

int
  main ()
{
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  ReadXYZPoints("../test.xyz", cloud);
  //if (pcl::io::loadPCDFile<PointT> ("test_pcd.pcd", *cloud) == -1) //* load the file
  //{
  //  PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  //  return (-1);
  //}
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from testfile."
            << std::endl;

  float voxel_resolution = 0.2f;
  float seed_resolution = 1.0f;
  float color_importance = 0.2f;
  float spatial_importance = 0.4f;
  float normal_importance = 1.0f;

  //////////////////////////////  //////////////////////////////
  ////// This is how to use supervoxels
  //////////////////////////////  //////////////////////////////

  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
  super.setInputCloud (cloud);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);
  super.setUseSingleCameraTransform(false);

  std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

  pcl::console::print_highlight ("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);
  pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());


  PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();

  PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();

  std::vector<RGB> colors = random_colors(*labeled_voxel_cloud, supervoxel_clusters.size());
  PointCloudT colored_cloud;

  for (int i=0; i < labeled_voxel_cloud->size(); i++) {
    PointLT point = labeled_voxel_cloud->points[i];
    RGB rgb = colors[i];
    PointT p{point.x, point.y, point.z, rgb.r, rgb.g, rgb.b, 1};

    colored_cloud.push_back(p);
  }
  pcl::io::savePCDFileASCII ("../vccs_pcl.pcd", colored_cloud);

  if (true) {
    exit(0);
  }

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");
  PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
  //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
  //viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

  pcl::console::print_highlight ("Getting supervoxel adjacency\n");
  std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);
  //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
  for (auto label_itr = supervoxel_adjacency.cbegin (); label_itr != supervoxel_adjacency.cend (); )
  {
    //First get the label
    std::uint32_t supervoxel_label = label_itr->first;
    //Now get the supervoxel corresponding to the label
    pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

    //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
    PointCloudT adjacent_supervoxel_centers;
    for (auto adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
    {
      pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
      adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
    }
    //Now we make a name for this polygon
    std::stringstream ss;
    ss << "supervoxel_" << supervoxel_label;
    //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
    addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
    //Move iterator forward to next label
    label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
  }

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
  }
  return (0);
}

void
addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                  PointCloudT &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  pcl::visualization::PCLVisualizer::Ptr & viewer)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

  //Iterate through all adjacent points, and add a center point to adjacent point pair
  for (auto adjacent_itr = adjacent_supervoxel_centers.begin (); adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
  {
    points->InsertNextPoint (supervoxel_center.data);
    points->InsertNextPoint (adjacent_itr->data);
  }
  // Create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
  // Add the points to the dataset
  polyData->SetPoints (points);
  polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
  for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
    polyLine->GetPointIds ()->SetId (i,i);
  cells->InsertNextCell (polyLine);
  // Add the lines to the dataset
  polyData->SetLines (cells);
  viewer->addModelFromPolyData (polyData,supervoxel_name);
}
