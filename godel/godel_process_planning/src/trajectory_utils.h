#ifndef TRAJECTORY_UTILS_H
#define TRAJECTORY_UTILS_H

#include <Eigen/Geometry>
#include <vector>>
namespace godel_process_planning
{
// Cartesian Interpolation
typedef std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > PoseVector;

/**
   * @brief Creates a vector of poses representing linear spatial and rotational interpolation
   *        between starting and ending poses.
   * @param start Beginning pose
   * @param stop Terminating pose
   * @param ds The cartesian distance (m) between intermediate points
   * @return Sequence of poses
   */
PoseVector interpolateCartesian(const Eigen::Isometry3d& start, const Eigen::Isometry3d& stop,
                                double ds);

// Joint Interpolation
typedef std::vector<std::vector<double> > JointVector;

/**
 * @brief Creates a vector of joint poses linearly interpolating from start to stop
 * @param start Initial joint configuration
 * @param stop Final joint configuration
 * @param dtheta Maximum joint step (radians) between intermediate points
 * @return
 */
JointVector interpolateJoint(const std::vector<double>& start, const std::vector<double>& stop,
                             double dtheta);
}

#endif
