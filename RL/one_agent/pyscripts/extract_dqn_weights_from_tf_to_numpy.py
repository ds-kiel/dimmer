import argparse
import time
import numpy as np
from baselines import deepq
import tensorflow as tf
import gym
import gym_dimmer
import gym_dimmer.envs.utils.dimmer_nn
from gym_dimmer.envs.utils.Dispatcher import Dispatcher
import gym_dimmer.envs.utils.glossai_utils as glossai_utils
from baselines.common.models import register

# Example of variables in tensorflow session
# We want the weight and biases of our 2 layers
# we do not need the target_q_func for inference
# Nor the Adam optimizer
# names = ["<tf.Variable 'deepq/eps:0' shape=() dtype=float32_ref>",
#         "<tf.Variable 'deepq/q_func/fully_connected/weights:0' shape=(54, 30) dtype=float32_ref>",
#         "<tf.Variable 'deepq/q_func/fully_connected/biases:0' shape=(30,) dtype=float32_ref>",
#         "<tf.Variable 'deepq/q_func/action_value/fully_connected/weights:0' shape=(30, 3) dtype=float32_ref>",
#         "<tf.Variable 'deepq/q_func/action_value/fully_connected/biases:0' shape=(3,) dtype=float32_ref>",
#         "<tf.Variable 'deepq/target_q_func/fully_connected/weights:0' shape=(54, 30) dtype=float32_ref>",
#         "<tf.Variable 'deepq/target_q_func/fully_connected/biases:0' shape=(30,) dtype=float32_ref>",
#         "<tf.Variable 'deepq/target_q_func/action_value/fully_connected/weights:0' shape=(30, 3) dtype=float32_ref>",
#         "<tf.Variable 'deepq/target_q_func/action_value/fully_connected/biases:0' shape=(3,) dtype=float32_ref>",
#         "<tf.Variable 'deepq_1/beta1_power:0' shape=() dtype=float32_ref>",
#         "<tf.Variable 'deepq_1/beta2_power:0' shape=() dtype=float32_ref>",
#         "<tf.Variable 'deepq/deepq/q_func/fully_connected/weights/Adam:0' shape=(54, 30) dtype=float32_ref>",
#         "<tf.Variable 'deepq/deepq/q_func/fully_connected/weights/Adam_1:0' shape=(54, 30) dtype=float32_ref>",
#         "<tf.Variable 'deepq/deepq/q_func/fully_connected/biases/Adam:0' shape=(30,) dtype=float32_ref>",
#         "<tf.Variable 'deepq/deepq/q_func/fully_connected/biases/Adam_1:0' shape=(30,) dtype=float32_ref>",
#         "<tf.Variable 'deepq/deepq/q_func/action_value/fully_connected/weights/Adam:0' shape=(30, 3) dtype=float32_ref>",
#         "<tf.Variable 'deepq/deepq/q_func/action_value/fully_connected/weights/Adam_1:0' shape=(30, 3) dtype=float32_ref>",
#         "<tf.Variable 'deepq/deepq/q_func/action_value/fully_connected/biases/Adam:0' shape=(3,) dtype=float32_ref>",
#         "<tf.Variable 'deepq/deepq/q_func/action_value/fully_connected/biases/Adam_1:0' shape=(3,) dtype=float32_ref>"]

def extract_weights(env_args, model, numpy_filenames):
    try:
        # Create Dimmer Gym Dispatcher
        dispatcher = Dispatcher(testbed,
                                    use_traces = True, # We don't need a real environment
                                    one_agent_per_node = False,
                                    use_randomized_order = False,
                                    use_randomized_value_at_beginning_of_episode = False)
        # ensure that dispatcher is ready
        time.sleep(0.5)
        # Start dispatcher
        dispatcher.start()
        # Create a gym environment
        env = gym.make("CentralizedControl-v0", **env_args)
        # Load the trained DQN
        act = deepq.learn(env,
                            network='dimmer_deepq_network',
                            total_timesteps=0,
                            load_path=model,
                            dueling=False)
        # Create tensorflow session, needed to read the value (in tf 1.x)
        sess = tf.get_default_session()
        # Read the correct tf variables
        for i in range(len(numpy_filenames)):
            np.save(numpy_filenames[i], sess.run(tf.trainable_variables()[i+1]))
        # We can stop the dispatcher
        dispatcher.stop()
    except Exception as e:
        print(e)
        dispatcher.stop()

if __name__ == "__main__":
     # params
    testbed =          "kiel"
    instance_name =    "dqn_eval"
    k_worst_nodes =    10
    history_size =     2
    reward_K = 30
    # check all existing instance IDs, try to get a new one
    instance_id = 2
    env_args = {"testbed":testbed, "k_worst_nodes_len":k_worst_nodes, "history_len":history_size}
    instance = f"{instance_name}_{testbed}_k_{k_worst_nodes}_history_{history_size}_reward_constant_30_instance_{instance_id}"
    numpy_filenames = ["../traces/dimmer_dqn_l0_w",
                       "../traces/dimmer_dqn_l0_b",
                       "../traces/dimmer_dqn_l1_w",
                       "../traces/dimmer_dqn_l1_b",
                      ]
    model = f"../models/evaluation/{instance}.pkl"
    extract_weights(env_args, model, numpy_filenames)