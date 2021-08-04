from baselines.common.models import register
import tensorflow as tf

LAYERS = [30]
ACTIVATIONS = [tf.nn.relu]

@register("dimmer_deepq_network")
def dimmer_deepq_network(**net_kwargs):
    def network_fn(X, nenv=1):
        for i in range(len(LAYERS)):
            X = tf.contrib.layers.fully_connected(X, num_outputs=LAYERS[i], activation_fn=ACTIVATIONS[i])
        return X, None
    return network_fn

@register("empty_network")
def empty_network(**net_kwargs):
    def network_fn(X, nenv=1):
        return X, None
    return network_fn