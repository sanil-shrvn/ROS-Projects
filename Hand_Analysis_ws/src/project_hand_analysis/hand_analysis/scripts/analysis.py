#!/usr/bin/env python
import time
import math
import numpy
import rospy
from sklearn.linear_model import LinearRegression
from sklearn.decomposition import PCA
from hand_analysis.msg import GraspInfo
from sklearn.linear_model import LogisticRegression,RidgeClassifier

class Analysis():
     def __init__(self):
         data=self.training_data()
         self.label_emg = LinearRegression()
         self.label_glove = LinearRegression()
         self.glove_emg = LinearRegression()
         self.emg_glove = LinearRegression()
         self.glove_lowdim = PCA(n_components=2)
         self.trained_model(data)
         self.sub=rospy.Subscriber("/grasp_info",GraspInfo,self.predict,queue_size=100)
         self.pub=rospy.Publisher("/labeled_grasp_info",GraspInfo,queue_size=100)

     def predict(msg):
         if len(msg.emg)!=0:
             # If EMG data is available, predict Grasp Label, Glove data and Low dimension glove data
             emg=numpy.array(msg.emg)
             label = self.label_emg.predict(emg.reshape(1,-1))
             label = label[0]
             glove = self.glove_emg.predict(emg.reshape(1,-1))
             glove = glove[0]
             lowdim = self.glove_lowdim.transform(glove.reshape(1,-1))
             lowdim = lowdim[0]

         elif len(msg.glove)!=0:
             # If Glove data is available, predict Grasp Label, EMG data and Low dimension glove data
             glove = numpy.array(msg.glove)
             label = self.label_glove.predict(glove.reshape(1,-1))
             label = label[0]
             emg = self.emg_glove.predict(glove.reshape(1,-1))
             emg = emg[0]
             lowdim = self.glove_lowdim.transform(glove.reshape(1,-1))
             lowdim = lowdim[0]

         elif len(msg.glove_low_dim)!=0:
             # If Low dimension glove data is available, predict Grasp Label, Glove data and EMG data
             lowdim = numpy.array(msg.glove_low_dim)
             glove = self.glove_lowdim.inverse_transform(lowdim.reshape(1,-1))
             glove = glove[0]
             label = self.label_glove.predict(glove.reshape(1,-1))
             label = label[0]
             emg = self.emg_glove.predict(glove.reshape(1,-1))
             emg = emg[0]


         message = GraspInfo()
         message.id = label
  	     message.glove = glove
         message.emg = emg
         message.glove_low_dim = lowdim
         self.pub.publish(message)
																								
         
          
     def training_data(self):
          file = rospy.get_param('~train_filename')
          data = numpy.genfromtxt(fname=file, delimiter = ',', skip_header=1)
          print "data retrieved"
          return data 

     def trained_model(self,data):
         label=data[0]
         emg_data=data[1:9]
         glove_data=data[9:24]
         self.label_emg.fit(emg_data,label)
         self.label_glove.fit(glove_data,label)
         self.glove_emg.fit(emg_data,glove_data)
         self.emg_glove.fit(glove_data,emg_data)
         self.glove_lowdim.fit(glove_data)
         print "model trained"

   
