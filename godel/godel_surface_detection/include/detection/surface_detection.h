/*
        Copyright Feb 11, 2014 Southwest Research Institute

        Licensed under the Apache License, Version 2.0 (the "License");
        you may not use this file except in compliance with the License.
        You may obtain a copy of the License at

                http://www.apache.org/licenses/LICENSE-2.0

        Unless required by applicable law or agreed to in writing, software
        distributed under the License is distributed on an "AS IS" BASIS,
        WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
        See the License for the specific language governing permissions and
        limitations under the License.
*/

#ifndef SURFACE_DETECTION_H_
#define SURFACE_DETECTION_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <visualization_msgs/MarkerArray.h>
#include <godel_msgs/SurfaceDetectionParameters.h>

#include <random>

#define LZ4_stream_t LZ4_stream_t_deprecated
#define LZ4_resetStream LZ4_resetStream_deprecated
#define LZ4_createStream LZ4_createStream_deprecated
#define LZ4_freeStream LZ4_freeStream_deprecated
#define LZ4_loadDict LZ4_loadDict_deprecated
#define LZ4_compress_fast_continue LZ4_compress_fast_continue_deprecated
#define LZ4_saveDict LZ4_saveDict_deprecated
#define LZ4_streamDecode_t LZ4_streamDecode_t_deprecated
#define LZ4_compress_continue LZ4_compress_continue_deprecated
#define LZ4_compress_limitedOutput_continue LZ4_compress_limitedOutput_continue_deprecated
#define LZ4_createStreamDecode LZ4_createStreamDecode_deprecated
#define LZ4_freeStreamDecode LZ4_freeStreamDecode_deprecated
#define LZ4_setStreamDecode LZ4_setStreamDecode_deprecated
#define LZ4_decompress_safe_continue LZ4_decompress_safe_continue_deprecated
#define LZ4_decompress_fast_continue LZ4_decompress_fast_continue_deprecated
#include <rosbag/bag.h>
#undef LZ4_stream_t
#undef LZ4_resetStream
#undef LZ4_createStream
#undef LZ4_freeStream
#undef LZ4_loadDict
#undef LZ4_compress_fast_continue
#undef LZ4_saveDict
#undef LZ4_streamDecode_t
#undef LZ4_compress_continue
#undef LZ4_compress_limitedOutput_continue
#undef LZ4_createStreamDecode
#undef LZ4_freeStreamDecode
#undef LZ4_setStreamDecode
#undef LZ4_decompress_safe_continue
#undef LZ4_decompress_fast_continue

namespace godel_surface_detection
{
namespace detection
{

typedef pcl::PointCloud<pcl::PointXYZRGB> CloudRGB;
typedef pcl::PointCloud<pcl::Normal> Normals;

class SurfaceDetection
{
public:
  SurfaceDetection();

public:
  bool init();

  bool load_parameters(const std::string& filename);
  void save_parameters(const std::string& filename);

  bool find_surfaces();

  static void mesh_to_marker(const pcl::PolygonMesh& mesh, visualization_msgs::Marker& marker,
                             std::default_random_engine &random_engine);

  // adds point cloud to the occupancy grid, it performs no frame transformation
  void add_cloud(CloudRGB& cloud);
  int get_acquired_clouds_count();

  void clear_results();

  // retrieve results
  visualization_msgs::MarkerArray get_surface_markers();
  void get_meshes(std::vector<pcl::PolygonMesh>& meshes);
  void get_surface_clouds(std::vector<CloudRGB::Ptr>& surfaces);
  void get_full_cloud(CloudRGB& cloud);
  void get_full_cloud(sensor_msgs::PointCloud2 cloud_msg);
  void get_process_cloud(CloudRGB& cloud);
  void get_process_cloud(sensor_msgs::PointCloud2& cloud_msg);
  void get_region_colored_cloud(CloudRGB& cloud);
  void get_region_colored_cloud(sensor_msgs::PointCloud2& cloud_msg);

  std::string getMeshingPluginName() const;


public:
  // parameters
  godel_msgs::SurfaceDetectionParameters params_;

private:
  // roscpp members
  ros::Subscriber point_cloud_subs_;

  std::default_random_engine random_engine_;

  // pcl members
  CloudRGB::Ptr full_cloud_ptr_;
  CloudRGB::Ptr process_cloud_ptr_;
  CloudRGB::Ptr region_colored_cloud_ptr_;
  std::vector<CloudRGB::Ptr> surface_clouds_;
  visualization_msgs::MarkerArray mesh_markers_;
  std::vector<pcl::PolygonMesh> meshes_;

  // counter
  int acquired_clouds_counter_;

  /**
   * @brief filterFullCloud applies a passthrough and voxelgrid filter to the
   * full cloud.  The result of these filters is the process cloud. The
   * passthrough filter eliminates the table and the voxelgrid downsamples the
   * cloud.
   */
  void filterFullCloud();
};
} /* end namespace detection */
} /* namespace godel_surface_detection */

#endif /* SURFACE_DETECTION_H_ */
