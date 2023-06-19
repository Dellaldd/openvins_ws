import numpy as np
import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def main():
    
    # q = [0, 0, 0,1] # wxyz
    # q = R.from_quat(q)
    # print(q.as_matrix())
    # print(q.as_euler("zyx"))
    
    # a = np.identity(4)
    # b = np.identity(4)
    # a[2,3] = 1
    # b[2,3] = 3
    # print(np.dot(np.linalg.inv(a), b))
    
    
    
    file_name = "record_good.txt"
    save_name = "record_good.png"
    read_path = "/home/ldd/openvins_ws/src/open_vins/path/" + file_name
    save_path = "/home/ldd/openvins_ws/src/open_vins/path/" + save_name
    data = np.loadtxt(read_path, delimiter=',', skiprows=1)
    fig, ax = plt.subplots(2, 3)
    
    first_index = 0
    for i in range(data.shape[0]):
        if data[i,1] and data[i,8]:
            first_index = i
            break
    print(first_index)
    
    T_gt0_w = np.identity(4)
    q = [data[first_index, 11],data[first_index, 12], data[first_index, 13], data[first_index, 14]] # wxyz
    print(q)
    q = R.from_quat(q)
    print(q.as_matrix())
    print(q.as_euler("zyx", degrees=True))
    
    T_gt0_w[:3, :3] = q.as_matrix()
    T_gt0_w[0,3] = data[first_index, 8]
    T_gt0_w[1,3] = data[first_index, 9]
    T_gt0_w[2,3] = data[first_index, 10]
    
    matrix = R.from_matrix(T_gt0_w[:3, :3])
    q_gt = matrix.as_quat()
    print(q_gt)
    print(R.from_quat(q_gt).as_matrix())
    print(R.from_quat(q_gt).as_euler("zyx", degrees=True))
    
    T_gt_w = np.identity(4)
    T_imu_w = np.identity(4)
    T_imu0_w = np.identity(4)
    # T_imu0_gt0 = np.identity(4)
    # T_imu0_gt0[2,2] = -1
    # T_imu0_gt0[1,1] = -1
    
    q = [data[first_index, 4], data[first_index, 5], data[first_index, 6], data[first_index, 7]]
    r = R.from_quat(q)
    T_imu0_w[:3, :3] = r.as_matrix()
    T_imu0_w[0,3] = data[first_index, 1]
    T_imu0_w[1,3] = data[first_index, 2]
    T_imu0_w[2,3] = data[first_index, 3]
    
    gt = []
    imu = []
    for i in range(first_index, data.shape[0]):
        q = [data[i, 11],data[i, 12], data[i, 13], data[i, 14]]
        q = R.from_quat(q)
        T_gt_w[:3, :3] = q.as_matrix()
        T_gt_w[0,3] = data[i, 8]
        T_gt_w[1,3] = data[i, 9]
        T_gt_w[2,3] = data[i, 10]
        T_gt_gt0 = np.dot(np.linalg.inv(T_gt0_w), T_gt_w)
        # T_gt_gt0 = T_gt_w
        
        matrix = R.from_matrix(T_gt_gt0[:3,:3])
        euler_gt = matrix.as_euler('zyx', degrees=True)
        gt.append(np.array([T_gt_gt0[0,3], T_gt_gt0[1,3], T_gt_gt0[2,3], euler_gt[0], euler_gt[1], euler_gt[2]]))
        
        q = [data[i, 4], data[i, 5], data[i, 6], data[i, 7]]
        r = R.from_quat(q)
        T_imu_w[:3, :3] = r.as_matrix()
        T_imu_w[0,3] = data[i, 1]
        T_imu_w[1,3] = data[i, 2]
        T_imu_w[2,3] = data[i, 3]
        T_imu_imu0 = np.dot(np.linalg.inv(T_imu0_w), T_imu_w)
        # T_imu_imu0 = T_imu_w
        
        matrix = R.from_matrix(T_imu_imu0[:3,:3])
        euler_imu = matrix.as_euler('zyx', degrees=True)
        imu.append(np.array([T_imu_imu0[0,3], T_imu_imu0[1,3], T_imu_imu0[2,3], euler_imu[0], euler_imu[1], euler_imu[2]]))
        
    imu = np.array(imu)
    gt = np.array(gt)
    
    ax[0][0].plot(data[first_index:,0], imu[:,0], 'b-', label = "openvins")
    ax[0][1].plot(data[first_index:,0], -imu[:,1], 'b-')
    ax[0][2].plot(data[first_index:,0], -imu[:,2], 'b-')
    # ax[0][1].plot(data[first_index:,0], imu[:,1], 'b-')
    # ax[0][2].plot(data[first_index:,0], imu[:,2], 'b-')

    ax[0][0].plot(data[first_index:,0], gt[:,0], 'r-', label = "opti")
    ax[0][1].plot(data[first_index:,0], gt[:,1], 'r-')
    ax[0][2].plot(data[first_index:,0], gt[:,2], 'r-')
    
    ax[1][0].plot(data[first_index:,0], imu[:,5], 'b-')
    ax[1][1].plot(data[first_index:,0], -imu[:,4], 'b-')
    ax[1][2].plot(data[first_index:,0], -imu[:,3], 'b-')
    # ax[1][1].plot(data[first_index:,0], imu[:,4], 'b-')
    # ax[1][2].plot(data[first_index:,0], imu[:,3], 'b-')

    ax[1][0].plot(data[first_index:,0], gt[:,5], 'r-')
    ax[1][1].plot(data[first_index:,0], gt[:,4], 'r-')
    ax[1][2].plot(data[first_index:,0], gt[:,3], 'r-')
    
    ax[0, 0].set_title("position x(m)")
    ax[0, 1].set_title("position y(m)")
    ax[0, 2].set_title("position z(m)")
    ax[1, 0].set_title("roll(deg)")
    ax[1, 1].set_title("pitch(deg)")
    ax[1, 2].set_title("yaw(deg)")

    fig.legend()
    fig.tight_layout()
    
    plt.savefig(save_path, dpi=300)
    plt.show()
  
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
