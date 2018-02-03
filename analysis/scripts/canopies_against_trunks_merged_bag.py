import rosbag
import numpy as np
import matplotlib.pyplot as plt
from collections import namedtuple
from numpy.linalg import norm

Measurement = namedtuple('Measurement', ['data', 'timestamp'])

BAG_FILES_PATHS = [r'/home/cear/data/bebop_top_view_1/dynamic_pose_8_800_messages/results_1.bag',
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
    return closest

def sample(vector):
    indices = [int(item) for item in list(np.floor(np.linspace(0, len(vector) - 1, num=NUM_OF_SAMPLES)))]
    return [vector[index] for index in indices]

def vector_from_topic(bag, topic_name):
    return [Measurement(msg, t) for _, msg, t in bag.read_messages(topics=[topic_name])]

def accuracy(measurement, ground_truth):
    return np.sqrt((measurement.data.pose.pose.position.x - ground_truth.data.x * RESOLUTION) ** 2 + (measurement.data.pose.pose.position.y - ground_truth.data.y * RESOLUTION) ** 2)

def precision(measurement):
    return norm(np.array(measurement.data.pose.covariance).reshape(6, 6))
    # TODO: which norm should we take?
    # TODO: should we ignore theta?

def rms(measurement, ground_truth):
    return sum([np.sqrt((particle.position.x - ground_truth.data.x * RESOLUTION) ** 2 + (particle.position.y - ground_truth.data.y * RESOLUTION) ** 2) for particle in measurement.data.poses])

def aggregate_experiments(results):
    means = []
    stds = []
    num_of_experiments = len(BAG_FILES_PATHS)
    for sample_index in range(NUM_OF_SAMPLES):
        means.append(np.mean([results[bag_index][sample_index] for bag_index in range(num_of_experiments)]))
        stds.append(np.std([results[bag_index][sample_index] for bag_index in range(num_of_experiments)]))
    return means, stds

def plot_canopies_against_trunks(title, ylabel, canopies_means, canopies_stds, trunks_means, trunks_stds):
    fig = plt.figure()
    fig.suptitle(title)
    plt.xlabel('waypoint #')
    plt.ylabel(ylabel)
    plt.errorbar(range(1, NUM_OF_SAMPLES+1), canopies_means, yerr=canopies_stds, color='g')
    plt.errorbar(range(1, NUM_OF_SAMPLES+1), trunks_means, yerr=trunks_stds, color='r')
    plt.xlim((0.8, NUM_OF_SAMPLES+0.2))
    plt.legend(['canopies', 'trunks'], fontsize='small')
    plt.savefig('/home/cear/Pictures/' + title + '.png')


if __name__ == '__main__':
    bag_index = 0
    canopies_accuracy = {}
    trunks_accuracy = {}
    canopies_precision = {}
    trunks_precision = {}
    canopies_rms = {}
    trunks_rms = {}

    for bag_file_path in BAG_FILES_PATHS:

        bag = rosbag.Bag(bag_file_path, 'r')

        canopies_accuracy[bag_index] = []
        trunks_accuracy[bag_index] = []
        canopies_precision[bag_index] = []
        trunks_precision[bag_index] = []
        canopies_rms[bag_index] = []
        trunks_rms[bag_index] = []

        # Create lists of Measurement
        canopies_amcl_poses = vector_from_topic(bag, '/canopies/amcl_pose')
        trunks_amcl_poses = vector_from_topic(bag, '/trunks/amcl_pose')
        ground_truth_poses = vector_from_topic(bag, '/pose')
        canopies_particle_clouds = vector_from_topic(bag, '/canopies/particlecloud')
        trunks_particle_clouds = vector_from_topic(bag, '/trunks/particlecloud')

        # Sample NUM_OF_SAMPLES waypoints
        canopies_amcl_poses_sampled = sample(canopies_amcl_poses)
        trunks_amcl_poses_sampled = sample(trunks_amcl_poses)
        canopies_particle_clouds_sampled = sample(canopies_particle_clouds)
        trunks_particle_clouds_sampled = sample(trunks_particle_clouds)
        ground_truth_poses_sampled_for_canopies_poses = [find_closest_measurement(ground_truth_poses, measurement.timestamp) for measurement in canopies_amcl_poses_sampled]
        ground_truth_poses_sampled_for_trunks_poses = [find_closest_measurement(ground_truth_poses, measurement.timestamp) for measurement in trunks_amcl_poses_sampled]
        ground_truth_poses_sampled_for_canopies_particle_clouds = [find_closest_measurement(ground_truth_poses, measurement.timestamp) for measurement in canopies_particle_clouds_sampled]
        ground_truth_poses_sampled_for_trunks_particle_clouds = [find_closest_measurement(ground_truth_poses, measurement.timestamp) for measurement in trunks_particle_clouds_sampled]

        # Iterate over waypoints (samples)
        for sample_index in range(NUM_OF_SAMPLES):
            canopies_accuracy[bag_index].append(accuracy(canopies_amcl_poses_sampled[sample_index], ground_truth_poses_sampled_for_canopies_poses[sample_index]))
            trunks_accuracy[bag_index].append(accuracy(trunks_amcl_poses_sampled[sample_index], ground_truth_poses_sampled_for_trunks_poses[sample_index]))
            canopies_precision[bag_index].append(precision(canopies_amcl_poses_sampled[sample_index]))
            trunks_precision[bag_index].append(precision(trunks_amcl_poses_sampled[sample_index]))
            canopies_rms[bag_index].append(rms(canopies_particle_clouds_sampled[sample_index], ground_truth_poses_sampled_for_canopies_particle_clouds[sample_index]))
            trunks_rms[bag_index].append(rms(trunks_particle_clouds_sampled[sample_index], ground_truth_poses_sampled_for_trunks_particle_clouds[sample_index]))

        bag_index += 1

    # Aggregate all experiments
    canopies_accuracy_means, canopies_accuracy_stds = aggregate_experiments(canopies_accuracy)
    trunks_accuracy_means, trunks_accuracy_stds = aggregate_experiments(trunks_accuracy)
    canopies_precision_means, canopies_precision_stds = aggregate_experiments(canopies_precision)
    trunks_precision_means, trunks_precision_stds = aggregate_experiments(trunks_precision)
    canopies_rms_means, canopies_rms_stds = aggregate_experiments(canopies_rms)
    trunks_rms_means, trunks_rms_stds = aggregate_experiments(trunks_rms)

    # Plot
    plot_canopies_against_trunks('Accuracy', 'error', canopies_accuracy_means, canopies_accuracy_stds, trunks_accuracy_means, trunks_accuracy_stds)
    plot_canopies_against_trunks('Precision', 'norm of covariance matrix', canopies_precision_means, canopies_precision_stds, trunks_precision_means, trunks_precision_stds)
    plot_canopies_against_trunks('Robustness', 'particle cloud RMS', canopies_rms_means, canopies_rms_stds, trunks_rms_means, trunks_rms_stds)

    print 'End of analysis'
