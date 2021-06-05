#ifndef SURFACE_SEGMENTATION_H
#define SURFACE_SEGMENTATION_H

#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>
#include <pcl/geometry/mesh_base.h>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/geometry/quad_mesh.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/geometry/mesh_conversion.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/ear_clipping.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/filter.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

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

static const int MAX_CLUSTER_SIZE = 50000;
static const int MIN_CLUSTER_SIZE = 2500;
static const int NUM_NEIGHBORS = 30;


template <bool IsManifoldT>
struct MeshTraits
{
  typedef pcl::PointXYZRGBNormal                       VertexData;
  typedef pcl::geometry::NoData                        HalfEdgeData;
  typedef pcl::geometry::NoData                        EdgeData;
  typedef pcl::geometry::NoData                        FaceData;
  typedef boost::integral_constant <bool, IsManifoldT> IsManifold;
};

typedef MeshTraits <true >                               ManifoldMeshTraits;
typedef pcl::geometry::PolygonMesh <ManifoldMeshTraits>  Mesh;
typedef typename Mesh::HalfEdgeIndex                     HalfEdgeIndex;
typedef typename Mesh::HalfEdgeIndices                   HalfEdgeIndices;
typedef typename Mesh::InnerHalfEdgeAroundFaceCirculator IHEAFC;


/** @class world_background_subtraction
@brief Maintains record of baseline sensor data to provide method to remove them leaving only new objects in the scene
*/
class SurfaceSegmentation
{
public:

  // mesh results
  pcl::PolygonMesh triangles_;
  std::vector<int> parts_;
  std::vector<int> states_;

  // segmentation results
  std::vector <pcl::PointIndices> clusters_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_downsampled_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree_;
  Mesh HEM_;

  // search terms
  double radius_;


  //-------------------- Constructors/Destructors --------------------//

  SurfaceSegmentation();
  ~SurfaceSegmentation();
  /**
   * @brief constructor that sets the background cloud, also initializes the KdTree for searching
   * @param bg_cloud the set of points defining the background
   */
  SurfaceSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr icloud);


  //-------------------- Clouds --------------------//

  /**
   * @brief sets the background cloud, replaces whatever points exists if any
   * @param background_cloud the cloud representing the background
   */
  void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr icloud);

  /**
   * @brief adds new points to the background, and reinitializes the kd_tree for searching
   * @param bg_cloud additional background points
   */
  void addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr icloud);

  /**
   * @brief creates a cloud from every point estimated to be on the boundary of input_cloud_
   * @return a boundary point cloud
   */
  void getBoundaryCloud(pcl::PointCloud<pcl::Boundary>::Ptr &boundary_cloud);
  void getSurfaceClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &surface_clouds);


  //-------------------- Computations --------------------//

  std::vector <pcl::PointIndices> computeSegments(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud);
  Mesh computeMesh();
  std::pair<int, int> getNextUnused(std::vector< std::pair<int,int> > used);
  int sortBoundary(pcl::IndicesPtr& boundary_indices, std::vector<pcl::IndicesPtr> &sorted_boundaries);
  void setSearchRadius(double radius);
  double getSearchRadius();


  //-------------------- Smoothing --------------------//
  void smoothVector(const std::vector<double> &x_in, std::vector<double> &x_out, const std::vector<double> &coef);

  /**
   * @brief SurfaceSegmentation::smoothPointNormal Uses a running weighted average (look-ahead and look-behind
   *        to smooth a vector of point normals. Default values for position and orientation smoother length were
   *        empirically derived and should be changed to best suit the user's application.
   * @param pts_in Input point vector
   * @param pts_out Destination for smoothed point vector
   * @param p_length Length of position smoother. Increasing this leads to smoother, less accurate edges.
   * @param w_length Length of orienation smoother. Increasing leads to less variation in edge point normal vectors.
   */
  void smoothPointNormal(std::vector<pcl::PointNormal> &pts_in, std::vector<pcl::PointNormal> &pts_out, int p_length,
                         int w_length);
  /**
   * @brief Uses the surrounding surface around the point edges to determine if the adjacent normals are sufficiently
   * consistent so as to generalize them onto the edge points
   * @param boundary_indices
   * @param poses
   * @param
   */

  /**
   * @brief Inspects the surrounding surface around the boundary points to determine if the nearby surface is sufficiently flat
   * such that the plane normal can be used to estimate normal vector at the boundary points.
   * @param boundary_pts                The boundary points
   * @param poses                       Output argument that contains the poses with the computed normals
   * @param eps                         Radius used to search for points in the surface near the boundary
   * @param plane_max_dist              Max distance at which points are considered to be in the plane.
   * @param plane_inlier_threshold      Minimum ratio of points needed to be confident that the surface is flat.
   * @return
   */
  bool regularizeNormals(const std::vector<pcl::PointNormal>& boundary_pts,
                         std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& poses,
                         double eps = 0.01,
                         double plane_max_dist = 0.005,
                         double plane_inlier_threshold = 0.8);


  //-------------------- Misc --------------------//

  void getBoundaryTrajectory(std::vector<pcl::IndicesPtr> &boundaries,
                             int sb,
                             std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &poses);

private:
  /** @brief remove any NAN points, otherwise many algorithms fail */
  void removeNans();

  /** @brief compute the normals and store in normals_, this is requried for both segmentation and meshing*/
  void computeNormals();

  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_;

};
#endif
