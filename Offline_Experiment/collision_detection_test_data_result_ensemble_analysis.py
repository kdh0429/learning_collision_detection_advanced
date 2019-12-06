import tensorflow as tf
import numpy as np
import csv
import matplotlib.pyplot as plt
import pandas as pd

# Parameter
time_step = 5
num_data_type = 3
num_one_joint_data = time_step * (num_data_type-1)
num_joint = 6
num_input = num_one_joint_data*num_joint  +  time_step # joint data + ee_acc data
num_output = 2
threshold = 0.99

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

Tool_list = ["0_00kg","2_01kg","3_33kg","5_01kg"]
DataType_list = ["collision", "free"]

for tool_idx in range(len(Tool_list)):
    for data_idx in range(len(DataType_list)):
        file_name = Tool_list[tool_idx] + '/' + DataType_list[data_idx] + '/' + 'DRCL_Data_Process.csv'
        # Collision Data
        if data_idx == 0: 
            TestData = pd.read_csv(file_name).as_matrix().astype('float32')
            X_Test = TestData[:,0:num_input]
            Y_Test = TestData[:,-num_output:]
            JTS = TestData[:,num_input]
            DOB = TestData[:,num_input+1]
            hypo1  =  sess1.run(hypothesis1, feed_dict={x1: X_Test, keep_prob1: 1.0, is_train1:False})
            hypo2  =  sess2.run(hypothesis2, feed_dict={x2: X_Test, keep_prob2: 1.0, is_train2:False})
            t = np.arange(0,0.001*len(JTS),0.001)

            collision_pre = 0
            collision_cnt = 0
            collision_time = 0
            detection_time_NN = []
            detection_time_JTS = []
            detection_time_DoB = []
            collision_status = False
            NN_detection = False
            JTS_detection = False
            DoB_detection = False
            collision_fail_cnt_NN = 0
            collision_fail_cnt_JTS = 0
            collision_fail_cnt_DoB = 0

            for i in range(len(JTS)):
                if (Y_Test[i,0] == 1 and collision_pre == 0):
                    collision_cnt = collision_cnt +1
                    collision_time = t[i]
                    collision_status = True
                    NN_detection = False
                    JTS_detection = False
                    DoB_detection = False
                
                if (collision_status == True and NN_detection == False):
                    if(hypo1[i,0] > threshold and hypo2[i,0] > threshold):
                        NN_detection = True
                        detection_time_NN.append(t[i] - collision_time)

                if (collision_status == True and JTS_detection == False):
                    if(JTS[i] == 1):
                        JTS_detection = True
                        detection_time_JTS.append(t[i] - collision_time)
                
                if (collision_status == True and DoB_detection == False):
                    if(DOB[i] == 1):
                        DoB_detection = True
                        detection_time_DoB.append(t[i] - collision_time)

                if (Y_Test[i,0] == 0 and collision_pre == 1):
                    collision_status = False
                    if(NN_detection == False):
                        detection_time_NN.append(0.0)
                        collision_fail_cnt_NN = collision_fail_cnt_NN+1
                    if(JTS_detection == False):
                        detection_time_JTS.append(0.0)
                        collision_fail_cnt_JTS = collision_fail_cnt_JTS+1
                    if(DoB_detection == False):
                        detection_time_DoB.append(0.0)
                        collision_fail_cnt_DoB = collision_fail_cnt_DoB+1
                collision_pre = Y_Test[i,0]
            print('----------------------------------------')
            print('Tool ', Tool_list[tool_idx],' ', DataType_list[data_idx])
            print('Total collision: ', collision_cnt)
            print('NN Failure: ', collision_fail_cnt_NN)
            print('JTS Failure: ', collision_fail_cnt_JTS)
            print('DOB Failure: ', collision_fail_cnt_DoB)
            print('NN Detection Time: ', sum(detection_time_NN)/(collision_cnt - collision_fail_cnt_NN))
            print('JTS Detection Time: ', sum(detection_time_JTS)/(collision_cnt - collision_fail_cnt_JTS))
            print('DOB Detection Time: ', sum(detection_time_DoB)/(collision_cnt - collision_fail_cnt_DoB))

        # Free Data
        else :
            TestDataFree = pd.read_csv(file_name).as_matrix().astype('float32')
            X_TestFree = TestDataFree[:,0:num_input]
            Y_TestFree = TestDataFree[:,-num_output:]
            JTSFree = TestDataFree[:,num_input]
            DOBFree = TestDataFree[:,num_input+1]
            hypofree1  =  sess1.run(hypothesis1, feed_dict={x1: X_TestFree, keep_prob1: 1.0, is_train1:False})
            hypofree2  =  sess2.run(hypothesis2, feed_dict={x2: X_TestFree, keep_prob2: 1.0, is_train2:False})
            t_free = np.arange(0,0.001*len(JTSFree),0.001)
            NN_FP_time = []
            NN_FP = 0
            JTS_FP_time = []
            JTS_FP = 0
            DOB_FP_time = []
            DOB_FP = 0
            for j in range(len(Y_TestFree)):
                if (hypofree1[j,0] > threshold and hypofree2[j,0] > threshold  and np.equal(np.argmax(Y_TestFree[j,:]), 1)):
                    NN_FP_time.append(t_free[j])
                    NN_FP = NN_FP + 1
                if (JTSFree[j] == 1 and np.equal(np.argmax(Y_TestFree[j,:]), 1)):
                    JTS_FP_time.append(t_free[j])
                    JTS_FP = JTS_FP + 1
                if (DOBFree[j] == 1 and np.equal(np.argmax(Y_TestFree[j,:]), 1)):
                    DOB_FP_time.append(t_free[j])
                    DOB_FP = DOB_FP + 1
            print('----------------------------------------')
            print('Tool ', Tool_list[tool_idx],' ', DataType_list[data_idx])
            print("NN FP Time: ")
            for k in range(NN_FP-1):
                del_time = abs(NN_FP_time[k+1]- NN_FP_time[k])
                if(del_time > 0.5):
                    print(del_time)
            print("JTS FP Time: ")
            for k in range(JTS_FP-1):
                del_time = abs(JTS_FP_time[k+1]- JTS_FP_time[k])
                if(del_time > 0.5):
                    print(del_time)
            print("DOB FP Time: ")
            for k in range(DOB_FP-1):
                del_time = abs(DOB_FP_time[k+1]- DOB_FP_time[k])
                if(del_time > 0.5):
                    print(del_time)