import rospy
import rospkg
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from nav_msgs.msg import Path
from mpl_toolkits.mplot3d import Axes3D

rospack = rospkg.RosPack()
PACKAGE_PATH = rospack.get_path('uwb_localization')
print(PACKAGE_PATH)


def calc_rmse(array):
    return np.sqrt(np.mean(array**2))


class TrajectoryAnalyzer:
    def __init__(self):
        self.true_trajectory = None
        self.optimized_trajectory = None
        self.noisy_trajectory = None

        # Subscribe to topics that are publishing the true and optimized trajectories
        rospy.Subscriber('/uav0/ground_truth/path', Path,
                         self.true_trajectory_callback)
        rospy.Subscriber('/uav0/optimized/path', Path,
                         self.optimized_trajectory_callback)
        rospy.Subscriber('/uav0/noisy/path', Path,
                         self.noisy_trajectory_callback)

    def true_trajectory_callback(self, msg):
        self.true_trajectory = msg

    def optimized_trajectory_callback(self, msg):
        self.optimized_trajectory = msg

    def noisy_trajectory_callback(self, msg):
        self.noisy_trajectory = msg

    def analyze_and_plot(self):
        while not rospy.is_shutdown():
            if self.true_trajectory and self.optimized_trajectory and self.noisy_trajectory:
                true_trajectory_poses = self.true_trajectory.poses[-400:]
                optimized_trajectory_poses = self.optimized_trajectory.poses[-400:]
                noisy_trajectory_poses = self.noisy_trajectory.poses[-400:]

                # Extract x, y, and z coordinates of true trajectory
                xyz_true = [[point.pose.position.x, point.pose.position.y, point.pose.position.z]
                            for point in true_trajectory_poses]
                x_true = [point[0] for point in xyz_true]
                y_true = [point[1] for point in xyz_true]
                z_true = [point[2] for point in xyz_true]

                # Extract x, y, and z coordinates of optimized trajectory
                xyz_opt = [[point.pose.position.x, point.pose.position.y, point.pose.position.z]
                           for point in optimized_trajectory_poses]
                x_opt = [point[0] for point in xyz_opt]
                y_opt = [point[1] for point in xyz_opt]
                z_opt = [point[2] for point in xyz_opt]

                xyz_noisy = [[point.pose.position.x, point.pose.position.y, point.pose.position.z]
                           for point in noisy_trajectory_poses]
                x_noisy = [point[0] for point in xyz_noisy]
                y_noisy = [point[1] for point in xyz_noisy]
                z_noisy = [point[2] for point in xyz_noisy]

                # Calculate standard deviation between the two trajectories
                diff_opt_xyz = np.array(xyz_opt) - np.array(xyz_true)
                dist_opt_xyz = np.linalg.norm(diff_opt_xyz, axis=1)
                diff_opt_x = np.array(x_opt) - np.array(x_true)
                diff_opt_y = np.array(y_opt) - np.array(y_true)
                diff_opt_z = np.array(z_opt) - np.array(z_true)

                print(dist_opt_xyz)
                diff_noisy_xyz = np.array(xyz_noisy) - np.array(xyz_true)
                dist_noisy_xyz = np.linalg.norm(diff_noisy_xyz, axis=1)

                std_x = np.std(diff_opt_x)
                std_y = np.std(diff_opt_y)
                std_z = np.std(diff_opt_z)
                std = np.std(dist_opt_xyz)

                rmse_x = calc_rmse(diff_opt_x)
                rmse_y = calc_rmse(diff_opt_y)
                rmse_z = calc_rmse(diff_opt_z)
                rmse = calc_rmse(dist_opt_xyz)

                std_noisy = np.std(dist_noisy_xyz)
                rmse_noisy = calc_rmse(dist_noisy_xyz)

                # Create 3D figure of optimized and true trajectories
                fig1 = plt.figure()
                ax1 = fig1.add_subplot(111, projection='3d')
                ax1.plot(x_true, y_true, z_true, 'b', )
                ax1.plot(x_opt, y_opt, z_opt, 'g')

                # Plot standard deviation
                fig2 = plt.figure()
                ax2 = fig2.add_subplot(111, projection='3d')
                ax2.scatter(x_opt, y_opt, z_opt, c='g', marker='o',
                            s=np.square(dist_opt_xyz*10))

                # Plot trajecotry distance in subsequent timesteps
                fig3 = plt.figure()
                ax3 = fig3.add_subplot()
                ax3.plot(dist_opt_xyz, c='g')
                ax3.set_xlabel('Kolejne odstępy czasu')
                ax3.set_ylabel('Odległości')
                ax3.set_ylim(0, 4)

                df = pd.DataFrame([['STD', std_x, std_y, std_z, std, std_noisy],
                                   ['RMSE', rmse_x, rmse_y, rmse_z, rmse, rmse_noisy]], columns=['TYPE', 'X', 'Y', 'Z', 'XYZ', 'XYZ_noisy'])

                # Create 3D figure of noisy and optimized trajectories
                fig4 = plt.figure()
                ax4 = fig4.add_subplot(111, projection='3d')
                ax4.plot(x_noisy, y_noisy, z_noisy, 'r')
                ax4.plot(x_opt, y_opt, z_opt, 'g')

                # Configure plot
                for ax in [ax1, ax2, ax4]:
                    ax.set_xlabel('Y')
                    ax.set_ylabel('X')
                    ax.set_zlabel('Z')

                df.to_csv(f'{PACKAGE_PATH}/analyse/trajs_table.csv')
                fig1.savefig(f'{PACKAGE_PATH}/analyse/trajs_plot.png')
                fig2.savefig(f'{PACKAGE_PATH}/analyse/trajs_scatter.png')
                fig3.savefig(f'{PACKAGE_PATH}/analyse/traj_dist_plot.png')
                fig4.savefig(f'{PACKAGE_PATH}/analyse/trajs2_plot.png')
                plt.show()
                plt.pause(0.1)
                plt.clf()


if __name__ == '__main__':
    try:
        rospy.init_node('trajectory_analyser', anonymous=True)
        node = TrajectoryAnalyzer()
        node.analyze_and_plot()
    except rospy.ROSInterruptException:
        pass
