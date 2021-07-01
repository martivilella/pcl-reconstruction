# Point Cloud Registration Pipeline
Straight-forward modular scene reconstruction script, that processes a range of input PLY/PCD point cloud files and merges them into a single aligned output file.

1. kNN Filtering
2. Uniform-distribution Downsampling
3. Odometry: ICP Point-to-Point pair-wise point cloud registration
4. Loop closure: MATLAB context descriptors + same odometry algorithm
5. Global alignment of de-noised point clouds (no downsampling)


# Usage
- Input point cloud files exprected to follow the naming convention `0000.pcd`, padding low indices with leading `0`s.
- Add point cloud files in the MATLAB path.
- Set subset of indices from file to use on begin of `main.m`.
- Set `ext` string on begin of `main.m` to `'.ply'` or `'.pcd'` to appropiate input file format.
