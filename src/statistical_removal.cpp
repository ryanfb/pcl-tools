#include <iostream>
#include <iomanip>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/parse.h>

int
main (int argc, char** argv)
{
  int mean_k = 50;
  double std_dev_thresh = 1.0;

  if(argc < 2) {
    std::cerr << "Usage:" << std::endl;
    std::cerr << argv[0] << " filename.pcd <options>" << std::endl;
    std::cerr << "\twill create filename_inliers.pcd and filename_outliers.pcd" << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "\t-k X\tuse X nearest neighbors around each point (default: " << mean_k << ")" << std::endl;
    std::cerr << "\t-d X\tuse X as the standard deviation multiplier threshold (default: " << std_dev_thresh << ")" << std::endl;
    return (1);
  }

  pcl::console::parse_argument(argc, argv, "-k", mean_k);
  pcl::console::parse_argument(argc, argv, "-d", std_dev_thresh);

  std::string basename(argv[1]);
  basename.erase(basename.find_last_of("."), std::string::npos);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> (argv[1], *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (mean_k);
  sor.setStddevMulThresh (std_dev_thresh);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  std::stringstream settings_prefix;
  settings_prefix << "_" << mean_k << "_" << std::fixed << std::setprecision(2) << std_dev_thresh;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (basename + settings_prefix.str() + "_inliers.pcd", *cloud_filtered, false);

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> (basename + settings_prefix.str() + "_outliers.pcd", *cloud_filtered, false);

  return (0);
}
