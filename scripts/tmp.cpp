

#include <pcl/range_image/range_image.h>

int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ> pointCloud;
  
  // Generate the data
  for (float y=-0.5f; y<=0.5f; y+=0.01f) {
    for (float z=-0.5f; z<=0.5f; z+=0.01f) {
      pcl::PointXYZ point;
      point.x = 2.0f - y;
      point.y = y;
      point.z = z;
      pointCloud.push_back(point);
    }
  }
  pointCloud.width = pointCloud.size();
  pointCloud.height = 1;
  
  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel=0.00;
  float minRange = 0.0f;
  int borderSize = 1;
  
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  
  std::cout << rangeImage << "\n";
}
Explanation
Lets look at this in parts:

#include <pcl/range_image/range_image.h>

int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ> pointCloud;
  
  // Generate the data
  for (float y=-0.5f; y<=0.5f; y+=0.01f) {
    for (float z=-0.5f; z<=0.5f; z+=0.01f) {
      pcl::PointXYZ point;
      point.x = 2.0f - y;
      point.y = y;
      point.z = z;
      pointCloud.push_back(point);
    }
  }
  pointCloud.width = pointCloud.size();
  pointCloud.height = 1;
This includes the necessary range image header, starts the main and generates a point cloud that represents a rectangle.

  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel=0.00;
  float minRange = 0.0f;
  int borderSize = 1;
This part defines the parameters for the range image we want to create.

The angular resolution is supposed to be 1 degree, meaning the beams represented by neighboring pixels differ by one degree.

maxAngleWidth=360 and maxAngleHeight=180 mean that the range sensor we are simulating has a complete 360 degree view of the surrounding. You can always use this setting, since the range image will be cropped to only the areas where something was observed automatically. Yet you can save some computation by reducing the values. E.g. for a laser scanner with a 180 degree view facing forward, where no points behind the sensor can be observed, maxAngleWidth=180 is enough.

sensorPose defines the 6DOF position of the virtual sensor as the origin with roll=pitch=yaw=0.

coordinate_frame=CAMERA_FRAME tells the system that x is facing right, y downwards and the z axis is forward. An alternative would be LASER_FRAME, with x facing forward, y to the left and z upwards.

For noiseLevel=0 the range image is created using a normal z-buffer. Yet if you want to average over points falling in the same cell you can use a higher value. 0.05 would mean, that all points with a maximum distance of 5cm to the closest point are used to calculate the range.

If minRange is greater than 0, all points that are closer will be ignored.

borderSize greater than 0 will leave a border of unobserved points around the image when cropping it.

  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  
  std::cout << rangeImage << "\n";
The remaining code creates the range image from the point cloud with the given parameters and outputs some information on the terminal.

The range image is derived from the PointCloud class and its points have the members x,y,z and range. There are three kinds of points. Valid points have a real range greater than zero. Unobserved points have x=y=z=NAN and range=-INFINITY. Far range points have x=y=z=NAN and range=INFINITY.

Compiling and running the program
Add the following lines to your CMakeLists.txt file:

 1
 2
 3
 4
 5
 6
 7
 8
 9
10
11
12
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(range_image_creation)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (range_image_creation range_image_creation.cpp)
target_link_libraries (range_image_creation ${PCL_LIBRARIES})
After you have made the executable, you can run it. Simply do:

$ ./range_image_creation
You should see the following:

range image of size 42x36 with angular resolution 1deg/pixel and 1512 points
© Copyright

Built with Sphinx using a theme provided by Read the Docs.