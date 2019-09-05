import tensorflow as tf
import numpy as np
import csv
import matplotlib.pyplot as plt

# parameters
time_step = 5
num_input = 36*time_step
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

#tensor = [n.name for n in tf.get_default_graph().as_graph_def().node]
# f = open("name.txt", 'w')
# for n in tensor:
#     f.write(n)
# f.close()

path = '../data/random/FCModulize/Testing_raw_data_.csv'
# raw data
f = open(path, 'r', encoding='utf-8')
rdr = csv.reader(f)
t = []
x_data_raw = []
y_data_raw = []

for line in rdr:
    line = [float(i) for i in line]
    x_data_raw.append(line[0:num_input])
    y_data_raw.append(line[-num_output:])
t = range(len(x_data_raw))
t = np.reshape(t,(-1,1))
x_data_raw = np.reshape(x_data_raw, (-1, num_input))
y_data_raw = np.reshape(y_data_raw, (-1, num_output))

hypo = sess.run(hypothesis, feed_dict={x: x_data_raw, keep_prob: 1.0, is_train:False})

prediction = np.argmax(hypo, 1)
correct_prediction = np.equal(prediction, np.argmax(y_data_raw, 1))
accuracy = np.mean(correct_prediction)
accuracy_all = accuracy_all + accuracy


print("Accuracy : %f" % accuracy)
false_positive = np.zeros_like(t)
false_negative = np.zeros_like(t)
for i in range(len(x_data_raw)):
    false_positive[i] = np.mean(np.equal(prediction[i], 0) and np.equal(np.argmax(y_data_raw[i,:]), 1))
    false_negative[i] = np.mean(np.equal(prediction[i], 1) and np.equal(np.argmax(y_data_raw[i,:]), 0))

print("False Positive: ",np.mean(false_positive))
print("False Negative: ",np.mean(false_negative))

# fileName = "../FCModulize/result/Result.txt"
# savefile = open(fileName, 'w')
# np.savetxt(savefile, hypo[:,0])
# savefile.close()