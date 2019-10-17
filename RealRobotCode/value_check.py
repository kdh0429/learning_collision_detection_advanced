import tensorflow as tf
import numpy as np
import pandas as pd

tf.reset_default_graph()
sess = tf.Session()

new_saver = tf.train.import_meta_graph('model/model.ckpt.meta')
new_saver.restore(sess, 'model/model.ckpt')

graph = tf.get_default_graph()

x = graph.get_tensor_by_name("m1/input:0")
y = graph.get_tensor_by_name("m1/output:0")
keep_prob = graph.get_tensor_by_name("m1/keep_prob:0")
is_train = graph.get_tensor_by_name("m1/is_train:0")
#print([n.name for n in tf.get_default_graph().as_graph_def().node])
print_value = graph.get_tensor_by_name("m1/Lee3:0")

Data = pd.read_csv('../build_ws_impact_cut/TestingDivideProcess/Testing_data_3.csv').as_matrix().astype('float32')
X = [Data[0,0:155]]
Y = Data[0,-2:]

hypo = sess.run(print_value, feed_dict={x: X, keep_prob: 1.0, is_train:False})
print(hypo)
