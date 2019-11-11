import tensorflow as tf
import numpy as np
import csv
import matplotlib.pyplot as plt
import argparse

# parameters
parser = argparse.ArgumentParser()
parser.add_argument('--data_type', type=str, default="collision")
args = parser.parse_args()

time_step = 5
num_data_type = 6
num_one_joint_data = time_step * (num_data_type-1)
num_joint = 6
num_input = num_one_joint_data*num_joint  +  time_step # joint data + ee_acc data
num_output = 2
false_negative = 0.0
false_positive = 0.0

tf.reset_default_graph()
sess = tf.Session()

new_saver = tf.train.import_meta_graph('model/model.ckpt.meta')
new_saver.restore(sess, 'model/model.ckpt')

graph = tf.get_default_graph()
x = graph.get_tensor_by_name("m1/input:0")
y = graph.get_tensor_by_name("m1/output:0")
keep_prob = graph.get_tensor_by_name("m1/keep_prob:0")
is_train = graph.get_tensor_by_name("m1/is_train:0")
hypothesis = graph.get_tensor_by_name("m1/ConcatenateNet/hypothesis:0")

accuracy_all = 0.0

path = '../RealRobotCode/TestingDivideProcess/Testing_data_6.csv'
# raw data
f = open(path, 'r', encoding='utf-8')
rdr = csv.reader(f)
t = []
x_data_raw = []
y_data_raw = []
JTS = []
DOB = []

for line in rdr:
    line = [float(i) for i in line]
    x_data_raw.append(line[0:num_input])
    y_data_raw.append(line[-num_output:])
    JTS.append(line[num_input])
    DOB.append(line[num_input+1])
t = range(len(x_data_raw))
t = np.reshape(t,(-1,1))
x_data_raw = np.reshape(x_data_raw, (-1, num_input))
y_data_raw = np.reshape(y_data_raw, (-1, num_output))

hypo = sess.run(hypothesis, feed_dict={x: x_data_raw, keep_prob: 1.0, is_train:False})

fileName = "../RealRobotCode/Result_TF.txt"
savefile = open(fileName, 'w')
np.savetxt(savefile, hypo[:,0])
savefile.close()
