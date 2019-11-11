import tensorflow as tf
import numpy as np
import csv
import matplotlib.pyplot as plt
import argparse

# parameters
parser = argparse.ArgumentParser()
parser.add_argument('--data_type', type=str, default="collision")
args = parser.parse_args()

if args.data_type == "collision" :
    total_batch = 220
    file_name = '../data/TestSet/TestingDivide/Testing_data_'
elif args.data_type == "free":
    total_batch = 309
    file_name = '../data/TestSet/TestingDivide/Testing_data_free_'

time_step = 5
num_data_type = 6
num_one_joint_data = time_step * (num_data_type-1)
num_joint = 6
num_input = num_one_joint_data*num_joint  +  time_step # joint data + ee_acc data
num_output = 2

# Model 1
tf.reset_default_graph()
sess1 = tf.Session()

new_saver1 = tf.train.import_meta_graph('model/ensemble1/model.ckpt.meta')
new_saver1.restore(sess1, 'model/ensemble1/model.ckpt')

graph1 = tf.get_default_graph()
x1 = graph1.get_tensor_by_name("m1/input:0")
y1 = graph1.get_tensor_by_name("m1/output:0")
keep_prob1 = graph1.get_tensor_by_name("m1/keep_prob:0")
is_train1 = graph1.get_tensor_by_name("m1/is_train:0")
hypothesis1 = graph1.get_tensor_by_name("m1/ConcatenateNet/hypothesis:0")

# Model 2
tf.reset_default_graph()
sess2 = tf.Session()

new_saver2 = tf.train.import_meta_graph('model/ensemble2/model.ckpt.meta')
new_saver2.restore(sess2, 'model/ensemble2/model.ckpt')

graph2 = tf.get_default_graph()
x2 = graph2.get_tensor_by_name("m1/input:0")
y2 = graph2.get_tensor_by_name("m1/output:0")
keep_prob2 = graph2.get_tensor_by_name("m1/keep_prob:0")
is_train2 = graph2.get_tensor_by_name("m1/is_train:0")
hypothesis2 = graph2.get_tensor_by_name("m1/ConcatenateNet/hypothesis:0")


accuracy_all = 0.0
false_negative = 0.0
false_positive = 0.0

threshold = 0.5

for i in range(total_batch): 
    path = file_name + str(i+1) + '.csv' # Testing_data_free_
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

    hypo1 = sess1.run(hypothesis1, feed_dict={x1: x_data_raw, keep_prob1: 1.0, is_train1:False})
    hypo2 = sess2.run(hypothesis2, feed_dict={x2: x_data_raw, keep_prob2: 1.0, is_train2:False})

    ensemble_prediction = np.zeros_like(t)
    for j in range(len(x_data_raw)):
        if hypo1[j,0] > threshold and hypo2[j,0] > threshold :
            ensemble_prediction[j] = 1.0
        else :
            ensemble_prediction[j] = 0.0

    correct_prediction = np.equal(ensemble_prediction[:], y_data_raw[:,0])
    accuracy = np.mean(correct_prediction)
    accuracy_all = accuracy_all + accuracy
    print("Accuracy : %f" % accuracy)

    false_positive_local_arr = np.zeros_like(t)
    false_negative_local_arr = np.zeros_like(t)

    for j in range(len(x_data_raw)):
        false_positive_local_arr[j] = np.equal(ensemble_prediction[j], 1.0) and np.equal(np.argmax(y_data_raw[j,:]), 1)
        false_negative_local_arr[j] = np.equal(ensemble_prediction[j], 0.0) and np.equal(np.argmax(y_data_raw[j,:]), 0)

    false_positive_local = 0.0
    false_negative_local = 0.0
    if (np.sum(np.equal(np.argmax(y_data_raw,1), 1)) != 0):
        false_positive_local = np.sum(false_positive_local_arr)/np.sum(np.equal(np.argmax(y_data_raw,1), 1))
    if (np.sum(np.equal(np.argmax(y_data_raw,1), 0)) != 0):
        false_negative_local = np.sum(false_negative_local_arr)/np.sum(np.equal(np.argmax(y_data_raw,1), 0))
    false_positive += np.mean(false_positive_local)/total_batch
    false_negative += np.mean(false_negative_local)/total_batch
    print("False Positive Local: ", false_positive_local)
    print("False Negative Local: ", false_negative_local)
    print("False Positive: ",false_positive)
    print("False Negative: ",false_negative)

    if(i < total_batch):
        plt.plot(t,y_data_raw[:,0], color='r', marker="o", label='real')
        plt.plot(t,ensemble_prediction[:], color='b',marker="x", label='prediction')
        plt.plot(t,JTS[:], color='k', marker="x", label='jts')
        plt.plot(t,DOB[:], color='y',marker="x", label='dob')
        plt.xlabel('time')
        plt.ylabel('Collision Probability')
        plt.legend()
        plt.ylim(0,1)
        plt.savefig('../FCModulize/result/Figure_' + str(i)+'.png')
        plt.clf()
        #plt.show()
        fileName = "../FCModulize/result/Result"+str(i)+".txt"
        savefile = open(fileName, 'w')
        np.savetxt(savefile, ensemble_prediction[:])
        savefile.close()

accuracy_all = accuracy_all/total_batch
print(accuracy_all)