import tensorflow as tf
import pandas as pd
import numpy as np


time_step = 5
num_data_type = 6
num_one_joint_data = time_step * (num_data_type-1)
num_joint = 6
num_input = num_one_joint_data*num_joint  + time_step # joint data + ee_acc data
num_output = 2
print(num_input)

def array_to_tfrecords(X, y, data_num, output_file):

    writer = tf.python_io.TFRecordWriter(output_file)
    for index in range(data_num):
        feature = {
        'X': tf.train.Feature(float_list=tf.train.FloatList(value=X[index,:].flatten())),
        'y': tf.train.Feature(float_list=tf.train.FloatList(value=y[index,:].flatten()))
        }
        example = tf.train.Example(features=tf.train.Features(feature=feature))
        serialized = example.SerializeToString()
        writer.write(serialized)
    writer.close()

TrainData = pd.read_csv('../data/TrainingData.csv').as_matrix().astype('float64')
X_train = TrainData[:,0:num_input]
Y_train = TrainData[:,-num_output:]
array_to_tfrecords(X_train,Y_train, Y_train.shape[0], 'TrainingData.tfrecord')

# ValidationData = pd.read_csv('../data/ValidationData.csv').as_matrix().astype('float64')
# X_validation = ValidationData[:,0:num_input]
# Y_validation = ValidationData[:,-num_output:]
# array_to_tfrecords(X_validation,Y_validation, Y_validation.shape[0], 'ValidationData.tfrecord')

# TestData = pd.read_csv('../data/TestingData.csv').as_matrix().astype('float64')
# X_Test = TestData[:,0:num_input]
# Y_Test = TestData[:,-num_output:]
# array_to_tfrecords(X_Test,Y_Test, Y_Test.shape[0], 'TestingData.tfrecord')