import tensorflow as tf
import numpy as np

tf.reset_default_graph()
sess = tf.Session()

new_saver = tf.train.import_meta_graph('model/model.ckpt.meta')
new_saver.restore(sess, 'model/model.ckpt')

graph = tf.get_default_graph()

name = [n.name for n in tf.trainable_variables()]
for j in range(6):
    for i in range(4):
        if i == 0:
            mean_tensor_name = "m1/Joint"+str(j)+"Net/batch_normalization/moving_mean:0"
            variance_tensor_name = "m1/Joint"+str(j)+"Net/batch_normalization/moving_variance:0"
        else:
            mean_tensor_name = "m1/Joint"+str(j)+"Net/batch_normalization_"+str(i)+"/moving_mean:0"
            variance_tensor_name = "m1/Joint"+str(j)+"Net/batch_normalization_"+str(i)+"/moving_variance:0"
        name.append(mean_tensor_name)
        name.append(variance_tensor_name)
name.append("m1/batch_normalization/moving_mean:0")
name.append("m1/batch_normalization/moving_variance:0")
name.append("m1/batch_normalization_1/moving_mean:0")
name.append("m1/batch_normalization_1/moving_variance:0")
name.append("m1/batch_normalization_2/moving_mean:0")
name.append("m1/batch_normalization_2/moving_variance:0")
name.append("m1/ConcatenateNet/batch_normalization/moving_mean:0")
name.append("m1/ConcatenateNet/batch_normalization/moving_variance:0")
name.append("m1/ConcatenateNet/batch_normalization_1/moving_mean:0")
name.append("m1/ConcatenateNet/batch_normalization_1/moving_variance:0")

np.savetxt('./name.txt', name, fmt='%s')
savefile_w = open('Weight.txt', 'w')
savefile_b = open('Bias.txt', 'w')
savefile_gm = open('Gamma.txt', 'w')
savefile_bt = open('Beta.txt', 'w')
savefile_bm = open('Mean.txt', 'w')
savefile_bv = open('Variance.txt', 'w')
savefile_data_normal = open('InputMinMax.txt', 'w')

for n in name:
    print_value = graph.get_tensor_by_name(n)
    mat = print_value.eval(session=sess).transpose()

    if 'Net/W' in n:
        for data_slice in mat:
            np.savetxt(savefile_w, [data_slice])
    elif 'W_ee' in n:
        for data_slice in mat:
            np.savetxt(savefile_w, [data_slice])        

    elif 'Net/Variable' in n:
        for data_slice in mat:
            np.savetxt(savefile_b, [data_slice])
    elif 'Variable' in n:
        for data_slice in mat:
            np.savetxt(savefile_b, [data_slice])

    elif 'gamma' in n:
        for data_slice in mat:
            np.savetxt(savefile_gm, [data_slice])

    elif 'beta' in n:
        for data_slice in mat:
            np.savetxt(savefile_bt, [data_slice])

    elif 'moving_mean' in n:
        np.savetxt(savefile_bm, mat)

    elif 'moving_variance' in n:
        np.savetxt(savefile_bv, mat)

Input_data_max = np.array([[144.6886991650000,   166.8623672100000,   97.5043730100000,   36.6971076220000,   38.8747072440000,   15.5852918317100],
                                [1.57,1.57,1.57,1.57,1.57,1.57],
                                [0.012106301100000,   0.009996968500000,   0.008732073000000,   0.010951596900000,   0.009698796749000,   0.007999126000000],
                                [0.030471368000000,   0.032628351300000,   0.032703119000000,   0.055313989900000,   0.041539804000000,   0.023768547000000],
                                [0.001582920000000,   0.002739420600000,   0.002450675600000,   0.003926920000000,   0.000942399100000,   0.001213046000000]])
Input_data_min = np.array([[-170.7753304150000,  -165.3056572400000,  -92.2283913690000,  -42.9723359370000,  -22.2735385400000,  -12.9340748762640],
                                [-1.57,-1.57,-1.57,-1.57,-1.57,-1.57],
                                [-0.011341421000000,  -0.007788007100000,  -0.011375057900000,  -0.011238997400000,  -0.010040846200000,  -0.009056478100000],
                                [-0.037924034800000,  -0.033170233000000,  -0.037959729000000,  -0.044833251500000,  -0.044674244900000,  -0.023149635200000],
                                [-0.002181576000000,  -0.003093831000000,  -0.002274677500000,  -0.001609872000000,  -0.001274405000000,  -0.001873778000000]])
EE_max_min = np.array([[10.0000], [0.0000], [0.0000], [0.0000], [0.0000]])
Input_data_max_min = np.concatenate((Input_data_max.T, Input_data_min.T, EE_max_min.T), axis=0)
np.savetxt(savefile_data_normal, Input_data_max_min, fmt='%f')


savefile_w.close()
savefile_b.close()
savefile_gm.close()
savefile_bt.close()
savefile_bm.close()
savefile_bv.close()
savefile_data_normal.close()