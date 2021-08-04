import io
import numpy as np

def scale_to_fixed_point(value, fixed_point):
    return int(round(value*fixed_point))

def write_to_file(filename, input_size, size_per_layer, layers_weights, layers_biases, activation_fn_per_layer, fixed_point):
    # try:
        f = open(filename, "x")

        f.write("/* BEGIN AUTOMATICALLY GENERATED NEURAL NETWORK */\n")
        write_fixed_point_scale(f, fixed_point)
        write_struct_beginning(f, size_per_layer)
        for layerID in range(len(layers_weights)):
            write_weights(f, layers_weights[layerID], fixed_point, layerID)
            write_biases(f, layers_biases[layerID], fixed_point, layerID)
        write_activation_function(f, activation_fn_per_layer)
        write_struct_end(f)
        f.write("/* END AUTOMATICALLY GENERATED NEURAL NETWORK */\n")

        f.close()


def write_fixed_point_scale(f, fixed_point):
    f.write("#undef FIXED_POINT_SCALE\n")
    f.write("#define FIXED_POINT_SCALE {}\n".format(fixed_point))


def write_struct_beginning(f, size_per_layer):
    for l in range(len(size_per_layer)):
        f.write("#undef DIMMER_DQN_LAYER{}_SIZE\n".format(l))
        f.write("#define DIMMER_DQN_LAYER{}_SIZE {}\n".format(l, size_per_layer[l]))
    f.write("#undef DIMMER_DQN_OUTPUT_SIZE\n")
    f.write("#define DIMMER_DQN_OUTPUT_SIZE {} \n".format(size_per_layer[-1]))
    f.write("static struct dimmer_dqn_config_struct enn_config = {\n")
    f.write("//layer sizes\n")
    f.write("   {")
    for l in size_per_layer:
        f.write("{},".format(l))
    f.write("}\n")
    f.write("};\n\n")

    f.write("static struct dimmer_dqn_nn_struct enn_dqn = {\n")
    f.write("   {NULL, NULL}, // pointers to weights\n")
    f.write("   {NULL, NULL}, // pointers to biases\n")


def write_weights(f, weights, fixed_point, layerID=-1):
    if layerID >= 0:
        f.write("// layer {} - weights\n".format(layerID))
    f.write("   {")
    weights = np.array(weights)
    weights = weights.transpose()
    for w_ in weights:
        for w in w_:
            f.write("{},".format(scale_to_fixed_point(w, fixed_point)))
    f.write("},\n")


def write_biases(f, biases, fixed_point, layerID=-1):
    if layerID >= 0:
        f.write("// layer {} - biases\n".format(layerID))
    f.write("   {")
    for b in biases:
        f.write("{},".format(scale_to_fixed_point(b, fixed_point)))
    f.write("},\n")


def write_activation_function(f, act_fn_per_layer):
    f.write("// activation function per layer\n")
    f.write("   {")
    for act in act_fn_per_layer:
        f.write("{},".format(act))
    f.write("},\n")


def write_struct_end(f):
    f.write("};\n\n")
    f.write("// Init DQN\n")
    f.write("enn_dqn.weights[0] = enn_dqn.layer0_weights;\n")
    f.write("enn_dqn.biases[0] = enn_dqn.layer0_biases;\n")
    f.write("enn_dqn.weights[1] = enn_dqn.layer1_weights;\n")
    f.write("enn_dqn.biases[1] = enn_dqn.layer1_biases;\n")


def generate_c_dqn():
    target_filename = "dqn_data.h"
    numpy_filenames = ["../traces/dimmer_dqn_l0_w.npy",
                       "../traces/dimmer_dqn_l0_b.npy",
                       "../traces/dimmer_dqn_l1_w.npy",
                       "../traces/dimmer_dqn_l1_b.npy",
                      ]
    input_size = 54
    layer_size = [30,3]
    weights = [np.load(numpy_filenames[0]), np.load(numpy_filenames[2])]
    biases = [np.load(numpy_filenames[1]), np.load(numpy_filenames[3])]
    act_fn = ["DIMMER_ACTIVATION_FN_RELU", "DIMMER_ACTIVATION_FN_RELU"]
    fixed_point = 100
    # start writing file
    write_to_file(target_filename,
                  input_size,
                  layer_size,
                  weights,
                  biases,
                  act_fn,
                  fixed_point)


if __name__ == "__main__":
    generate_c_dqn()
