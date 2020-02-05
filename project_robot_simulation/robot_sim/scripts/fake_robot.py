#!/usr/bin/env python
import numpy as np
import time
import rospy

from robot_sim.srv import RobotAction
from robot_sim.srv import RobotActionRequest
from robot_sim.srv import RobotActionResponse
from robot_sim.msg import RobotState

import torch
import torch.nn as nn
import torch.nn.functional as F

from torch.utils.data.dataset import Dataset
from torch.utils.data import DataLoader

class MyDNN(nn.Module):
    def __init__(self, input_dim):
        super(MyDNN, self).__init__()
        self.fc1 = nn.Linear(input_dim, 32)
        self.fc2 = nn.Linear(32, 32)
        self.fc5 = nn.Linear(32, 6)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc5(x)
        return x

    def predict(self, features):
        self.eval()	#Sets network in eval mode (vs training mode)
        features = torch.from_numpy(features).float()
        return self.forward(features).detach().numpy()


class MyDataset(Dataset):
    def __init__(self, labels, features):
        super(MyDataset, self).__init__()
        self.labels = labels
        self.features = features

    def __len__(self):
        return self.features.shape[0]

    def __getitem__(self, idx):		
        feature = self.features[idx]
        label = self.labels[idx]
        return {'feature': feature, 'label': label}

class MyDNNTrain(object):
    def __init__(self, network):	#Networks is of datatype MyDNN
        self.network = network
        self.learning_rate = 0.295
        self.optimizer = torch.optim.SGD(self.network.parameters(), lr=self.learning_rate, momentum=0.9)
        #,weight_decay=1e-6, momentum=0.9, nesterov=True
        self.criterion = nn.CrossEntropyLoss()
        self.num_epochs = 80
        self.batchsize = 500
        self.shuffle = True

    def train(self, labels, features):
        self.network.train()
        dataset = MyDataset(labels, features)
        loader = DataLoader(dataset, shuffle=self.shuffle, batch_size = self.batchsize)
        for epoch in range(self.num_epochs):
            self.train_epoch(loader)

    def train_epoch(self, loader):
        total_loss = 0.0
        for i, data in enumerate(loader):
            features = data['feature'].float()
            labels = data['label'].float()
            self.optimizer.zero_grad()
            predictions = self.network(features)
            loss = self.criterion(predictions, labels)
            loss.backward()
            total_loss += loss.item()
            self.optimizer.step()
            print 'loss', total_loss/(i+1)

class fk_rbt(object):

    def __init__(self):
        self.network = MyDNN(9)
        self.trainer = MyDNNTrain(self.network)
        features,labels=self.data_acquisition(200,200)
        self.trainer.train(labels, features)
        print "Data Trained"
        self.fake_robot_state_init=[]
        rospy.init_node('fake_robot')
        self.fake_robot_state=rospy.Service('fake_robot', RobotAction ,self.predict_data)
        rospy.spin()

    def data_acquisition(self,num_of_data_set,rep_data):
	print 'Training Started'
	features=[]
        labels=[]
        for i in range(num_of_data_set):
            action = np.random.rand(1, 3)
            action[0, 0] = (2 * action[0, 0] - 1.0) * 1.0
            action[0, 1] = (2 * action[0, 1] - 1.0) * 0.5
            action[0, 2] = (2 * action[0, 2] - 1.0) * 0.25
	    print 'Training for dataset', (i+1)
            real_robot_action = rospy.ServiceProxy('real_robot', RobotAction)
       	    req = RobotActionRequest()
            req.reset = True
            resp =real_robot_action(req)
            for j in range(rep_data):
                #print 'Rep set',(j+1)
                f=np.append(np.array(resp.robot_state),action)
                features.append(f)
                req = RobotActionRequest()
                req.reset = False
                req.action = action.reshape((3))
                resp =  real_robot_action(req)
                labels.append(resp.robot_state)
        features=np.asarray(features)
        labels=np.asarray(labels)
        print features
        print "Acquisition Completed"
        return features,labels

    def predict_data(self,act):
        if act.reset==False:
            print 'a'
            print act
            features=np.array(np.append(np.array(self.fake_robot_state_init),np.array(act.action)))
            robot_state=self.network.predict(features)
            self.fake_robot_state_init=robot_state
            print robot_state
            return RobotActionResponse(robot_state)
        elif act.reset==True:
            print 'b'
            print act
            robot_state=[-1.57, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.fake_robot_state_init=robot_state
            print robot_state
            return RobotActionResponse(robot_state)


if __name__ == '__main__':
	fk_rbt()

