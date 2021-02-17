# vislab_dataset

INPUT: Ambarella dataset, OUTPUT: PatchMatchNet dataset

STEPS

1) Load Ambarella dataset as sparse point cloud, pose for each frame and features extracted for each frame
2) Select a subset of keyframes from the input video
3) Compute the depth range (d_min, d_max) for the selected keyframes
4) For each point, build a list of images in which the point can be seen and remove points that can't be seen by any image
5) Build clusters of points and cameras
6) Compute neighbors with view selection methods for each cluster
7) Store the results as required by PatchMatchNet (https://github.com/FangjinhuaWang/PatchmatchNet)

TODO & IMPROVEMENTS

1) View selection can be improved in two ways:
- Computing a better score (e.g. by introducing distance)
- Looking for neighbors in nearby clusters
2) Keyframes can be selected in a smarter way:
- Removing dynamic objects (https://bertabescos.github.io/EmptyCities/)
- Using other criteria, not just pose difference
3) Neighbors for each cluster can be computed in parallel, if speed is needed
4) General code refactoring