#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

int
main (int argc, char** argv)
{
  bool binary = true;

  if(argc < 3) {
    std::cerr << "Usage:" << std::endl;
    std::cerr << argv[0] << " [-a] input.pcd output.ply" << std::endl;
    std::cerr << "\t-a\tASCII output" << std::endl;
    return (1);
  }

  if(argc == 4) {
    if(strncmp(argv[1],"-a",2) != 0) {
      std::cerr << "Error: unknown option!" << std::endl;
      return (1);
    }
    else {
      binary = false;
      argv += 1;
    }
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);

  pcl::PCDReader reader;
  reader.read<pcl::PointNormal> (argv[1], *cloud);

  std::cerr << "Read cloud: " << std::endl;
  std::cerr << *cloud << std::endl;

  pcl::PLYWriter plywriter;
  plywriter.write<pcl::PointNormal> (argv[2], *cloud, binary);

  return (0);
}
