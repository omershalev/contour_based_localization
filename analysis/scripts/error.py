import rosbag
import numpy as np
import matplotlib.pyplot as plt
from collections import namedtuple

Measurement = namedtuple('Sample', ['data', 'timestamp'])

BAG_FILES = [r'/home/cear/data/bebop_top_view_1/dynamic_pose_8_800_messages/results_1.bag',
             r'/home/cear/data/bebop_top_view_1/dynamic_pose_8_800_messages/results_2.bag',
             r'/home/cear/data/bebop_top_view_1/dynamic_pose_8_800_messages/results_3.bag',
             r'/home/cear/data/bebop_top_view_1/dynamic_pose_8_800_messages/results_4.bag',
             r'/home/cear/data/bebop_top_view_1/dynamic_pose_8_800_messages/results_5.bag']
RESOLUTION = 0.0125
NUM_OF_SAMPLES = 8


def find_closest_measurement(measurements, timestamp):
    closest = None
    for measurement in measurements:
        if closest is None:
            min_delta = abs(timestamp - measurement.timestamp)
            closest = measurement
        elif abs(timestamp - measurement.timestamp) < min_delta:
            min_delta = abs(timestamp - measurement.timestamp)
            closest = measurement
        else:
            return measurement
    #TODO: verify correctness

if __name__ == '__main__':
    bag_index = 0
    canopies_errors = {}
    trunks_errors = {}

    for BAG_FILE in BAG_FILES:

        bag = rosbag.Bag(BAG_FILE, 'r')
        canopies_errors[bag_index] = []
        trunks_errors[bag_index] = []

        canopies_amcl_poses = [Measurement(msg.pose.pose.position, t) for _, msg, t in bag.read_messages(topics=['/canopies/amcl_pose'])]
        trunks_amcl_poses = [Measurement(msg.pose.pose.position, t) for _, msg, t in bag.read_messages(topics=['/trunks/amcl_pose'])]
        ground_truth_poses = [Measurement(msg, t) for _, msg, t in bag.read_messages(topics=['/pose'])]

        sample_canopies_amcl_poses_indices = [int (item) for item in list(np.floor(np.linspace(0, len(canopies_amcl_poses)-1, num=NUM_OF_SAMPLES)))]
        sample_canopies_amcl_poses = [canopies_amcl_poses[index] for index in sample_canopies_amcl_poses_indices]
        sample_canopies_ground_truth_amcl_poses = [find_closest_measurement(ground_truth_poses, measurement.timestamp) for measurement in sample_canopies_amcl_poses]

        sample_trunks_amcl_poses_indices = [int (item) for item in list(np.floor(np.linspace(0, len(trunks_amcl_poses)-1, num=NUM_OF_SAMPLES)))]
        sample_trunks_amcl_poses = [trunks_amcl_poses[index] for index in sample_trunks_amcl_poses_indices]
        sample_trunks_ground_truth_amcl_poses = [find_closest_measurement(ground_truth_poses, measurement.timestamp) for measurement in sample_trunks_amcl_poses]


        for sample_index in range(NUM_OF_SAMPLES):
            canopies_errors[bag_index].append(np.sqrt((sample_canopies_amcl_poses[sample_index].data.x - sample_canopies_ground_truth_amcl_poses[sample_index].data.x * RESOLUTION) ** 2
                        + (sample_canopies_amcl_poses[sample_index].data.y - sample_canopies_ground_truth_amcl_poses[sample_index].data.y * RESOLUTION) ** 2))
            trunks_errors[bag_index].append(np.sqrt((sample_trunks_amcl_poses[sample_index].data.x - sample_trunks_ground_truth_amcl_poses[sample_index].data.x * RESOLUTION) ** 2
                        + (sample_trunks_amcl_poses[sample_index].data.y - sample_trunks_ground_truth_amcl_poses[sample_index].data.y * RESOLUTION) ** 2))

        bag_index += 1
        continue

    canopies_mean_errors = []
    trunks_mean_errors = []
    canopies_std_errors = []
    trunks_std_errors = []
    for sample_index in range(NUM_OF_SAMPLES):
        canopies_mean_errors.append(np.mean([canopies_errors[bag_index][sample_index] for bag_index in range(len(BAG_FILES))]))
        canopies_std_errors.append(np.std([canopies_errors[bag_index][sample_index] for bag_index in range(len(BAG_FILES))]))
        trunks_mean_errors.append(np.mean([trunks_errors[bag_index][sample_index] for bag_index in range(len(BAG_FILES))]))
        trunks_std_errors.append(np.std([trunks_errors[bag_index][sample_index] for bag_index in range(len(BAG_FILES))]))

    plt.figure()
    plt.errorbar([measurement.timestamp.to_time()-sample_canopies_amcl_poses[0].timestamp.to_time() for measurement in sample_canopies_amcl_poses], canopies_mean_errors, yerr=canopies_std_errors, color='g')
    plt.errorbar([measurement.timestamp.to_time()-sample_trunks_amcl_poses[0].timestamp.to_time() for measurement in sample_trunks_amcl_poses], trunks_mean_errors, yerr=trunks_std_errors, color='r')
    plt.show()

    print 'hi'
    '''
        # last_canopies_amcl_position = canopies_amcl_poses[-1].pose.pose.position
        # last_trunks_amcl_position = trunks_amcl_poses[-1].pose.pose.position
        # last_ground_truth_position = ground_truth_poses[-1]
        # TODO: verify that X and Y shouldn't be switched for the ground truth
        # print np.sqrt((last_canopies_amcl_position.x - last_ground_truth_position.x * RESOLUTION)**2 + (last_canopies_amcl_position.y - last_ground_truth_position.y * RESOLUTION)**2)
        # print np.sqrt((last_trunks_amcl_position.x - last_ground_truth_position.x * RESOLUTION)**2 + (last_trunks_amcl_position.y - last_ground_truth_position.y * RESOLUTION)**2)
        # TODO: define this norm better; is it A*I*inv(A), i.e. sum of squares? is it a product of all eigenvalues? what about the other non zero elements? (theta and correlation between x and y)?
        canopies_cov_norm = canopies_amcl_poses[-1].pose.covariance[0]**2 + canopies_amcl_poses[-1].pose.covariance[7]**2
        trunks_cov_norm = trunks_amcl_poses[-1].pose.covariance[0]**2 + trunks_amcl_poses[-1].pose.covariance[7]**2
        print canopies_cov_norm
        print trunks_cov_norm


        canopies_amcl_particles = [msg for _, msg, _ in bag.read_messages(topics=['/canopies/particlecloud'])]
        trunks_amcl_particles = [msg for _, msg, _ in bag.read_messages(topics=['/trunks/particlecloud'])]
        last_canopies_amcl_particles = canopies_amcl_particles[-1]
        last_trunks_amcl_particles = trunks_amcl_particles[-1]

        rms = 0
        for particle in last_canopies_amcl_particles.poses:
            rms += np.sqrt((particle.position.x - last_ground_truth_position.x * RESOLUTION) ** 2 + (particle.position.y - last_ground_truth_position.y * RESOLUTION) ** 2)
        rms = np.sqrt(rms)
        print rms

        rms = 0
        for particle in last_trunks_amcl_particles.poses:
            rms += np.sqrt((particle.position.x - last_ground_truth_position.x * RESOLUTION) ** 2 + (particle.position.y - last_ground_truth_position.y * RESOLUTION) ** 2)
        rms = np.sqrt(rms)
        print rms


    # (last_canopies_amcl_particles.poses[5].position.x - last_ground_truth_position.x * RESOLUTION)**2
    '''