import tensorflow as tf
import numpy as np
import csv
import matplotlib.pyplot as plt
import time
import wandb
import os
import time
import pandas as pd

wandb_use = True
start_time = time.time()
if wandb_use == True:
    wandb.init(project="Dusan_2nd_Project", tensorboard=False)

class Model:

    def __init__(self, sess, name):
        self.sess = sess
        self.name = name
        self._build_net()

    def _build_net(self):
        with tf.variable_scope(self.name, reuse = tf.AUTO_REUSE):
            self.X = tf.placeholder(tf.float32, shape=[None, num_input], name = "input")
            self.Y = tf.placeholder(tf.float32, shape=[None, num_output], name= "output")
            self.is_train = tf.placeholder(tf.bool, name = "is_train")
            self.keep_prob = tf.placeholder(tf.float32, name="keep_prob")
            self.cross_entropy_weight = tf.placeholder(tf.float32, name="cross_entropy_weight")
            self.hidden_layers = 0
            self.hidden_neurons = 15

            # Joint Data Layers
            for i in range(6):
                with tf.variable_scope("Joint"+str(i)+"Net"):
                    W1 = tf.get_variable("W1", shape=[num_one_joint_data, self.hidden_neurons], initializer=tf.contrib.layers.xavier_initializer(), regularizer=tf.contrib.layers.l2_regularizer(regul_factor))
                    b1 = tf.Variable(tf.random_normal([self.hidden_neurons]))
                    L1 = tf.matmul(self.X[:, num_one_joint_data*i:num_one_joint_data*(i+1)], W1) +b1
                    L1 = tf.layers.batch_normalization(L1, training=self.is_train)
                    L1 = tf.nn.relu(L1)
                    L1 = tf.nn.dropout(L1, keep_prob=self.keep_prob)

                    W2 = tf.get_variable("W2", shape=[self.hidden_neurons, self.hidden_neurons], initializer=tf.contrib.layers.xavier_initializer(), regularizer=tf.contrib.layers.l2_regularizer(regul_factor))
                    b2 = tf.Variable(tf.random_normal([self.hidden_neurons]))
                    L2 = tf.matmul(L1, W2) +b2
                    L2 = tf.layers.batch_normalization(L2, training=self.is_train)
                    L2 = tf.nn.relu(L2)
                    L2 = tf.nn.dropout(L2, keep_prob=self.keep_prob)
                    self.hidden_layers += 1

                    W3 = tf.get_variable("W3", shape=[self.hidden_neurons, self.hidden_neurons], initializer=tf.contrib.layers.xavier_initializer(), regularizer=tf.contrib.layers.l2_regularizer(regul_factor))
                    b3 = tf.Variable(tf.random_normal([self.hidden_neurons]))
                    L3 = tf.matmul(L2, W3) +b3
                    L3 = tf.layers.batch_normalization(L3, training=self.is_train)
                    L3 = tf.nn.relu(L3)
                    L3 = tf.nn.dropout(L3, keep_prob=self.keep_prob)
                    self.hidden_layers += 1

                    W4 = tf.get_variable("W4", shape=[self.hidden_neurons, 1], initializer=tf.contrib.layers.xavier_initializer(), regularizer=tf.contrib.layers.l2_regularizer(regul_factor))
                    b4 = tf.Variable(tf.random_normal([1]))
                    L4 = tf.matmul(L3, W4) +b4
                    L4 = tf.layers.batch_normalization(L4, training=self.is_train)
                    L4 = tf.nn.relu(L4)
                    L4 = tf.nn.dropout(L4, keep_prob=self.keep_prob)

                    if(i == 0):
                        self.LConcat = L4
                    else:
                        self.LConcat = tf.concat([self.LConcat, L4],1)
                        
            # End Effector Accerlation Data Layers
            W_ee1 = tf.get_variable("W_ee1", shape=[3*time_step, self.hidden_neurons], initializer=tf.contrib.layers.xavier_initializer(), regularizer=tf.contrib.layers.l2_regularizer(regul_factor))
            b_ee1 = tf.Variable(tf.random_normal([self.hidden_neurons]))
            L_ee1 = tf.matmul(self.X[:, num_one_joint_data*6:num_one_joint_data*6+3*time_step], W_ee1) + b_ee1
            L_ee1 = tf.layers.batch_normalization(L_ee1, training=self.is_train)
            L_ee1 = tf.nn.relu(L_ee1)
            L_ee1 = tf.nn.dropout(L_ee1, keep_prob=self.keep_prob)

            W_ee2 = tf.get_variable("W_ee2", shape=[self.hidden_neurons, 1], initializer=tf.contrib.layers.xavier_initializer(), regularizer=tf.contrib.layers.l2_regularizer(regul_factor))
            b_ee2 = tf.Variable(tf.random_normal([1]))
            L_ee2 = tf.matmul(L_ee1, W_ee2) +b_ee2
            L_ee2 = tf.layers.batch_normalization(L_ee2, training=self.is_train)
            L_ee2 = tf.nn.relu(L_ee2)
            L_ee2 = tf.nn.dropout(L_ee2, keep_prob=self.keep_prob)
            self.hidden_layers += 1
            self.LConcat = tf.concat([self.LConcat, L_ee2],1)

            with tf.variable_scope("ConcatenateNet"):
                W5 = tf.get_variable("W5", shape=[7, self.hidden_neurons], initializer=tf.contrib.layers.xavier_initializer(), regularizer=tf.contrib.layers.l2_regularizer(regul_factor))
                b5 = tf.Variable(tf.random_normal([self.hidden_neurons]))
                L5 = tf.matmul(self.LConcat, W5) +b5
                L5 = tf.layers.batch_normalization(L5, training=self.is_train)
                L5 = tf.nn.relu(L5)
                L5 = tf.nn.dropout(L5, keep_prob=self.keep_prob)

                W6 = tf.get_variable("W6", shape=[self.hidden_neurons, num_output], initializer=tf.contrib.layers.xavier_initializer(), regularizer=tf.contrib.layers.l2_regularizer(regul_factor))
                b6 = tf.Variable(tf.random_normal([num_output]))
                self.logits = tf.matmul(L5, W6) + b6
                tf.identity(self.logits, "logits")
                self.hypothesis = tf.nn.softmax(self.logits)
                
                self.hypothesis = tf.identity(self.hypothesis, "hypothesis")

            # define cost/loss & optimizer
            self.l2_reg = sum(tf.get_collection(tf.GraphKeys.REGULARIZATION_LOSSES))
            self.cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits_v2(logits=self.logits, labels=self.Y))
            #self.cost = tf.reduce_mean(tf.nn.weighted_cross_entropy_with_logits(targets=self.Y, logits=self.logits, pos_weight=0.6))

            self.update_ops = tf.get_collection(tf.GraphKeys.UPDATE_OPS)
            with tf.control_dependencies(self.update_ops):
                self.optimizer = tf.train.AdamOptimizer(learning_rate= learning_rate).minimize(self.cost + self.l2_reg)
        
        self.prediction = tf.argmax(self.hypothesis, 1)
        self.correct_prediction = tf.equal(self.prediction, tf.argmax(self.Y, 1))
        self.accuracy = tf.reduce_mean(tf.cast(self.correct_prediction, tf.float32))

    def get_mean_error_hypothesis(self, x_test, y_test, keep_prop=1.0, is_train=False):
        return self.sess.run([self.accuracy,  self.l2_reg, self.cost], feed_dict={self.X: x_test, self.Y: y_test, self.keep_prob: keep_prop, self.is_train: is_train})

    def train(self, x_data, y_data, keep_prop=1.0, is_train=True):
        return self.sess.run([self.accuracy, self.l2_reg, self.cost, self.optimizer], feed_dict={
            self.X: x_data, self.Y: y_data, self.keep_prob: keep_prop, self.is_train: is_train})

    def get_hidden_number(self):
        return [self.hidden_layers, self.hidden_neurons]

# input/output number
time_step = 5
num_data_type = 10
num_one_joint_data = time_step * (num_data_type-1)
num_joint = 6
num_input = num_one_joint_data*num_joint + 3*time_step # joint data + ee_acc data
num_output = 2

# parameters
learning_rate = 0.00002 #0.000001
training_epochs = 150
batch_size = 1000 
total_batch = 1112 # joint : 492, random : 1132/ 705 / 449 / 566
total_batch_val = 123 # joint: 105, random: 242/ 151 / 96/ 121
total_batch_test = 39 # joint: 105, random: 242/ 151 / 96 / 121
drop_out = 1.0
regul_factor = 0.00001#0.032


# initialize
# os.environ['CUDA_VISIBLE_DEVICES'] = '0'
# sess = tf.Session()

os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
tf_config = tf.ConfigProto(
        allow_soft_placement=True,
        inter_op_parallelism_threads=16,
        intra_op_parallelism_threads=16, log_device_placement=False)
    # Prevent tensorflow from taking all the gpu memory
tf_config.gpu_options.allow_growth = False
sess = tf.Session(config=tf_config)

m1 = Model(sess, "m1")
sess.run(tf.global_variables_initializer())


if wandb_use == True:
    wandb.config.epoch = training_epochs
    wandb.config.batch_size = batch_size
    wandb.config.learning_rate = learning_rate
    wandb.config.drop_out = drop_out
    wandb.config.num_input = num_input
    wandb.config.num_output = num_output
    wandb.config.time_step = time_step
    wandb.config.total_batch = total_batch
    wandb.config.activation_function = "ReLU"
    wandb.config.hidden_layers, wandb.config.hidden_neurons = m1.get_hidden_number()
    wandb.config.L2_regularization = regul_factor 

# train my model
train_mse = np.zeros(training_epochs)
validation_mse = np.zeros(training_epochs)

train_cost = np.zeros(training_epochs)
validation_cost = np.zeros(training_epochs)

start_time_tmp = time.time()

TrainData = pd.read_csv('../data/TrainingData.csv')
TrainData = TrainData.as_matrix().astype('float32')
TrainInputData = TrainData[:,0:num_input]
TrainOutputData = TrainData[:,-num_output:]

Traindataset = tf.data.Dataset.from_tensor_slices((TrainInputData, TrainOutputData))
Traindataset = Traindataset.batch(batch_size)
Traindataset = Traindataset.prefetch(buffer_size=1)

Trainiterator = Traindataset.make_initializable_iterator()
train_batch_x, train_batch_y = Trainiterator.get_next()


ValidationData = pd.read_csv('../data/ValidationData.csv')
ValidationData = ValidationData.as_matrix().astype('float32')
ValidationInputData = ValidationData[:,0:num_input]
ValidationOutputData = ValidationData[:,-num_output:]

ValidationDataset = tf.data.Dataset.from_tensor_slices((ValidationInputData, ValidationOutputData))
ValidationDataset = ValidationDataset.batch(batch_size)
ValidationDataset = ValidationDataset.prefetch(buffer_size=1)

Validationiterator = ValidationDataset.make_initializable_iterator()
validation_batch_x, validation_batch_y = Validationiterator.get_next()

for epoch in range(training_epochs):
    accu_train = 0
    accu_val = 0
    reg_train = 0
    reg_val = 0
    cost_train = 0
    cost_val = 0
    
    sess.run(Trainiterator.initializer)

    while True:
        try:
            x, y = sess.run([train_batch_x, train_batch_y])
            c, reg_c, cost,_ = m1.train(x, y, drop_out)
            accu_train += c / total_batch
            reg_train += reg_c / total_batch
            cost_train += cost / total_batch
        except tf.errors.OutOfRangeError:
            break

    sess.run(Validationiterator.initializer)
    while True:
        try:
            x, y = sess.run([validation_batch_x, validation_batch_y])
            c, reg_c, cost = m1.get_mean_error_hypothesis(x, y)
            accu_val += c / total_batch_val
            reg_val += reg_c / total_batch_val
            cost_val += cost / total_batch_val
        except tf.errors.OutOfRangeError:
            break

    print('Epoch:', '%04d' % (epoch + 1))
    print('Train Accuracy =', '{:.9f}'.format(accu_train))
    print('Validation Accuracy =', '{:.9f}'.format(accu_val))
    print('Train Cost =', '{:.9f}'.format(cost_train), 'Train Regul =', '{:.9f}'.format(reg_train))
    print('Validation Cost =', '{:.9f}'.format(cost_val), 'Validation Regul =', '{:.9f}'.format(reg_val))

    train_mse[epoch] = accu_train
    validation_mse[epoch] = accu_val

    train_cost[epoch] = cost_train
    validation_cost[epoch] = cost_val

    if wandb_use == True:
        wandb_dict = dict()
        wandb_dict['Training Accuracy'] = accu_train
        wandb_dict['Validation Accuracy'] = accu_val
        wandb_dict['Training Cost'] = cost_train
        wandb_dict['Training Reg'] = reg_train
        wandb_dict['Validation Cost'] = cost_val
        wandb_dict['Validation Reg'] = reg_val
        
        if epoch % 20 ==0:
            for var in tf.trainable_variables():
                wandb_dict[var.name] =sess.run(var)
        wandb.log(wandb_dict)

elapsed_time = time.time() - start_time_tmp
print(elapsed_time)
print('Learning Finished!')

# Save Model
saver = tf.train.Saver()
saver.save(sess,'model/model.ckpt')

if wandb_use == True:
    saver.save(sess, os.path.join(wandb.run.dir, 'model/model.ckpt'))
    wandb.config.elapsed_time = elapsed_time


# Test Set
TestData = pd.read_csv('../data/TestData.csv')
TestData = TestData.as_matrix().astype('float32')
TestInputData = TestData[:,0:num_input]
TestOutputData = TestData[:,-num_output:]

TestDataset = tf.data.Dataset.from_tensor_slices((TestInputData, TestOutputData))
TestDataset = TestDataset.batch(batch_size)
TestDataset = TestDataset.prefetch(buffer_size=1)

Testiterator = TestDataset.make_initializable_iterator()
test_batch_x, test_batch_y = Testiterator.get_next()

accu_test = 0
reg_test = 0
cost_test = 0

while True:
    try:
        x, y = sess.run([test_batch_x, test_batch_y])
        c, reg, cost  = m1.get_mean_error_hypothesis(x, y)
        accu_test += c / total_batch_test
        reg_test += reg / total_batch_test
        cost_test += cost / total_batch_test
    except tf.errors.OutOfRangeError:
        break
print('Test Accuracy: ', accu_test)
print('Test Cost: ', cost_test)