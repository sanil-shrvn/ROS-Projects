#!/usr/bin/env python

import numpy as np
import time
import rospy
import random

from time import time
from robot_sim.msg import RobotState
from robot_sim.srv import RobotAction
from robot_sim.srv import RobotActionRequest
from robot_sim.srv import RobotActionResponse
from robot_sim.srv import RobotPolicy
from robot_sim.srv import RobotPolicyRequest
from robot_sim.srv import RobotPolicyResponse
from collections import namedtuple


import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torchvision.transforms as T



class DQN_Network(nn.Module):
    def __init__(self):
        super(DQN_Network, self).__init__()
        self.fc1 = nn.Linear(4, 32)
        self.fc2 = nn.Linear(32, 32)
        self.fc3 = nn.Linear(32,2)
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x.view(x.size(0), -1)

    def predict(self, state):
        self.eval()
        state = torch.from_numpy(state).float()
        return self.forward(state)

class env(object):
    def __init__(self):
        torch.manual_seed(64)
        self.robot_action = rospy.ServiceProxy('cartpole_robot', RobotAction)

    def state_out_of_bounds(self, state):
        return abs(state[0]) > 1.2 or abs(state[1]) > 6 * np.pi / 180

    def get_random_sign(self):
        return 1.0 if random.random() < 0.5 else -1.0

    def step(self, action):
        request = RobotActionRequest()
        request.action = action
        self.next_state = self.robot_action(request).robot_state
        self.done = self.state_out_of_bounds(self.next_state)
        if self.done:
            self.state = None
            self.reward = 0
        else:
            self.reward = 1
        return self.next_state, self.reward, self.done

    def reset(self):
        request = RobotActionRequest()
        request.reset_robot = True
        sign=self.get_random_sign()
        request.reset_pole_angle = sign*(np.random.rand(1)*3) * np.pi/180
        self.state = self.robot_action(request).robot_state
        return self.state



class ReplayMemory(object):
    def __init__(self, memory_size):
        self.memory = memory_size
        self.samples = []
        self.Transition = namedtuple('Transition', ('state', 'action', 'reward', 'new_state'))

    def push(self, sample):
        try:
            next_state = torch.tensor(np.asarray(sample[3]))
        except TypeError:
            next_state = None
        self.samples.append(self.Transition(torch.tensor(np.asarray(sample[0])), torch.tensor(np.asarray([sample[1]])), torch.tensor(np.asarray([sample[2]]), dtype = torch.float), next_state ))
        if len(self.samples) > self.memory:
            self.samples.pop(0)

    def sample(self, no_samples):
        if no_samples > len(self.samples):
            return random.sample(self.samples, len(self.samples))
        else:
            return random.sample(self.samples, no_samples)


class RunDQN():
    def __init__(self):
        self.no_of_episodes = 500
        self.T = 200

        self.eps = 0.01
        self.gamma = 0.99
        self.update = 100
        self.learning_rate = 0.1
        self.batch_size = 32

        self.memory = ReplayMemory(10000)

        self.policy_net = DQN_Network()
        self.target_net = DQN_Network()
        self.policy_net.load_state_dict(self.target_net.state_dict())
        self.optimizer = torch.optim.Adagrad(self.policy_net.parameters(), lr=self.learning_rate)

        self.env = env()
        self.learn(self.env)

        rospy.init_node('cartpole_policy')
        self.robot_policy = rospy.Service('cartpole_policy', RobotPolicy, self.predict_action)
        rospy.spin()

    def act(self, state, no_of_episode):
        epsilon = max((1 - (float(no_of_episode) / self.no_of_episodes)), self.eps)
        if np.random.rand() < epsilon:
            return torch.tensor([[random.randrange(2)]])
        else:
            with torch.no_grad():
                state = np.array(state)
                return self.policy_net.predict(state).max(0)[1].view(1, 1)

    def learn(self, env):
        for i in range(self.no_of_episodes):
            state = env.reset()
            for t in range(self.T):
                action = self.act(state, i)
                next_state, reward, done = env.step([(action * 20) - 10])
                #print next_state, reward, done
                self.memory.push([state, action, reward, next_state])
                state = next_state
                self.optimize()
                if done:
                    break
            print 'Episode No:', i+1, 'Carpole balanced for', t+1, 'steps'


            if i % self.update == 0:
                self.target_net.load_state_dict(self.policy_net.state_dict())

    def optimize(self):
        if len(self.memory.samples) < self.batch_size:
            return
        samp = self.memory.sample(self.batch_size)
        batch = self.memory.Transition(*zip(*samp))
        mask = torch.tensor(tuple(map(lambda s: s is not None, batch.new_state)), dtype=torch.uint8)
        states = torch.cat(batch.state).view(-1, 4)
        actions = torch.cat(batch.action)
        rewards = torch.cat(batch.reward)
        not_done_new_state = torch.cat([s for s in batch.new_state if s is not None]).view(-1, 4)
        Q_spred = self.policy_net.predict(states.numpy())
        Q_s = Q_spred.gather(1, actions.unsqueeze(1))
        new_V_s = torch.zeros(self.batch_size, dtype=torch.float)
        new_V_s[mask] = self.target_net.predict(not_done_new_state.numpy()).max(1)[0].detach()
        expectedQ_s = new_V_s * self.gamma + rewards
        loss = F.smooth_l1_loss(Q_s, expectedQ_s.unsqueeze(1))
        self.optimizer.zero_grad()
        loss.backward()
        for p in self.policy_net.parameters():
            p.grad.data.clamp_(-1, 1)
        self.optimizer.step()

    def predict_action(self, request):
        req = np.asarray(request.robot_state)
        prediction = self.policy_net.predict(req)
        index = np.argmax(prediction.data)
        if index == 0:
            action = [-10]
        else:
            action = [10]
        return RobotPolicyResponse(action)


if __name__ == '__main__':
    RunDQN()
