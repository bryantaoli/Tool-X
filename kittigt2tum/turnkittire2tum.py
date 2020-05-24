import numpy as np
from scipy.spatial.transform import Rotation as Ro
times = np.loadtxt('times.txt', dtype=float)
poses_kitti = np.loadtxt('00.txt', dtype=float)

poses_tum = np.zeros((times.size, 7), dtype=float)
poses_tum[:,0] = poses_kitti[:,3]
poses_tum[:,1] = poses_kitti[:,7]
poses_tum[:,2] = poses_kitti[:,11]

for i in range(0, times.size):
    R=np.zeros((3,3), dtype=float)
    R[0,0] = poses_kitti[i,0]
    R[0,1] = poses_kitti[i,1]
    R[0,2] = poses_kitti[i,2]
    R[1,0] = poses_kitti[i,4]
    R[1,1] = poses_kitti[i,5]
    R[1,2] = poses_kitti[i,6]
    R[2,0] = poses_kitti[i,8]
    R[2,1] = poses_kitti[i,9]
    R[2,2] = poses_kitti[i,10]
    r = Ro.from_matrix(R)
    q = r.as_quat()
    poses_tum[i,3] = q[0]
    poses_tum[i,4] = q[1]
    poses_tum[i,5] = q[2]
    poses_tum[i,6] = q[3]

times = np.expand_dims(times, axis=1)

times_poses = np.append(times, poses_tum, axis=1)

np.savetxt('times_poses.txt', times_poses, fmt="%f", delimiter=" ", newline ="\n")
