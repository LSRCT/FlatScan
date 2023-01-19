# Fast implementaion of frame reprojection using Open3D and numpy.
# Based on TUM script generate_registered_pointcloud.py found here:
# https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools/

import open3d as o3d
import os
from associate import read_file_list, associate
from evaluate_rpe import read_trajectory
from PIL import Image
import numpy as np
import math

def generate_pointcloud(rgb_file, depth_file, intrinsics):
    """
    Generate a colored point cloud 
    :param rgb_file: path to rgb image
    :param depth_file: path to depth image
    :param intrinsics: camera intrinsics (o3d.camera.PinholeCameraIntrinsic)
    """

    rgb_img = Image.open(rgb_file).convert("RGB")
    depth_img = Image.open(depth_file)
    depth = np.asarray(depth_img, dtype=np.uint16)
    rgb = np.asarray(rgb_img, dtype=np.uint8)
    o3d_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(rgb), 
                                                                  o3d.geometry.Image(depth), 
                                                                  depth_scale=5000.0, 
                                                                  depth_trunc=5000.0, 
                                                                  convert_rgb_to_intensity=False)
    o3d_pcl = o3d.geometry.PointCloud.create_from_rgbd_image(o3d_rgbd, intrinsics)
    return o3d_pcl

def process_pointcloud(pcl, transform):
    """
    Perform preprocessing on point cloud: Downsample, estimate normals, transform, and crop ceiling
    :param pcl: point cloud (o3d.geometry.PointCloud)
    :param transform: transformation matrix (4x4 numpy array)
    """
    pcl = pcl.uniform_down_sample(every_k_points=2)
    pcl.estimate_normals()
    pcl = pcl.transform(transform)
    # remove ceiling
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-math.inf, -1.5, -math.inf), max_bound=(math.inf, math.inf, math.inf))
    pcl = pcl.crop(bbox)  # crop point cloud
    return pcl



if __name__ == '__main__':
    rgb_list_file = "KinectData/ANeFull3/rgb.txt"
    depth_list_file = "KinectData/ANeFull3/depth.txt"
    trajectory_file = "orbslam/trajectories/CameraTrajectoryFull3.txt"
    out_file = "out.ply"
    nth_frame = 10
    
    # camera parameters
    width = 512
    height = 424
    focalLength = 366.0
    centerX = 258.5
    centerY = 203.5
    scalingFactor = 5000.0
    cam_intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, focalLength, focalLength, centerX, centerY)

    # read file lists
    rgb_list = read_file_list(rgb_list_file)
    depth_list = read_file_list(depth_list_file)
    pose_list = read_file_list(trajectory_file)

    # match rgb and depth images
    matches_rgb_depth = dict(associate(rgb_list, depth_list, float(
        0), float(0.02)))
    matches_rgb_traj = associate(matches_rgb_depth, pose_list, float(
        0), float(0.01))
    matches_rgb_traj.sort()

    traj = read_trajectory(trajectory_file)


    # process point clouds into one big point cloud
    pcl_full =o3d.geometry.PointCloud()
    list = range(0, len(matches_rgb_traj), int(nth_frame))
    for frame, i in enumerate(list):
        rgb_stamp, traj_stamp = matches_rgb_traj[i]
        base_path = os.path.join(*os.path.split(rgb_list_file)[:-1])

        rgb_file = rgb_list[rgb_stamp][0]
        rgb_file = os.path.join(base_path, rgb_file)

        depth_file = depth_list[matches_rgb_depth[rgb_stamp]][0]
        depth_file = os.path.join(base_path, depth_file)

        pose = traj[traj_stamp]

        pcl = generate_pointcloud(rgb_file, depth_file, cam_intrinsics)
        pcl = process_pointcloud(pcl, pose)

        pcl_full += pcl

        print("Frame %d/%d, number of points so far: %d" %
                (frame+1, len(list), len(pcl_full.points)))

    pcl_full = pcl_full.voxel_down_sample(voxel_size=0.005)
    pcl_full, _ = pcl_full.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.5)
    o3d.visualization.draw_geometries([pcl_full])
    o3d.io.write_point_cloud(out_file, pcl_full)
