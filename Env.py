import random
import os
import random as rd
import win32com.client as com
from ctypes import *
import time

class Envi(object):
    def __init__(self):
        self.action = [-2, -1, 0, 0.5, 1]
        self.n_action = len(self.action)
        # self.Vissim = com.Dispatch('Vissim.Vissim.1000')
        self.Vissim = com.gencache.EnsureDispatch("Vissim.Vissim")
        self.initialize()
        self.inputlane1 = []
        self.inputlane2 = []
        self.inputmap = {}
        self.target_id = -1
        # self.f = open('E:\\Python\\DQN\\results.txt', 'a')
    def initialize(self):
        Vissim = self.Vissim
        network_path = os.getcwd()
        Filename = os.path.join(network_path, 'network_NEM.inpx')
        # flag_read_additionally = False  # you can read network(elements) additionally, in this case set "flag_read_additionally" to true
        Vissim.LoadNet(Filename)
        # Layx = os.path.join(network_path, 'network.layx')
        # Vissim.LoadLayout(Layx)
        Random_seed = rd.randint(1, 100)
        Vissim.Simulation.SetAttValue('RandSeed', Random_seed)
        Vissim.Graphics.CurrentNetworkWindow.SetAttValue('QuickMode', 1)
        Vissim.Simulation.SetAttValue('UseMaxSimSpeed', True)
        Vissim.Simulation.SetAttValue('SimPeriod', 2700)

    def reset(self):
        dll = CDLL("DriverModel.dll")
        sendtoC = dll.sendpython
        sendtoC.argtypes = [c_long, c_double]
        sendtoC(-1, 0)

        Vissim = self.Vissim
        Vissim.Simulation.Stop()
        rd_time = random.randint(200, 300)
        Vissim.Simulation.SetAttValue('SimBreakAt', rd_time)
        Vissim.Simulation.RunContinuous()
        self.target_id = self.findtarget()
        results = self.findinput()

        return results, self.target_id

    def step(self, id, action):
        Vissim = self.Vissim
        dll = CDLL("DriverModel.dll")
        sendtoC = dll.sendpython
        sendtoC.argtypes = [c_long, c_double]
        acc = self.action[action]
        sendtoC(id, acc)
        Vissim.Simulation.RunSingleStep()
        s_ = self.findinput()
        speed, cars, other, toend = s_
        done = False
        reward = 0
        success = False


        if 1 in cars[300:305]:
            reward = -1
            done = True
        elif 1 not in cars[200:230] and 1 not in cars[300:330] and 1 not in cars[70:100] and abs(other[0] - 68/90)<0.05 and other[1] == 0:
            reward = 1
        elif abs(other[0] - 68/90) > 0.12:
            reward = -0.2

        if toend > 1120:
            if reward == 1:
                success = True
            else:
                reward -= 0.15
            done = True

        return s_, reward, done, success



    def findinput(self):
        self.inputlane1.clear()
        self.inputlane2.clear()
        self.inputmap.clear()
        Vissim = self.Vissim
        vehs1 = Vissim.Net.Links.ItemByKey(2).Vehs.GetAll()
        vehs2 = Vissim.Net.Links.ItemByKey(10002).Vehs.GetAll()
        vehs3 = Vissim.Net.Links.ItemByKey(4).Vehs.GetAll()
        vehs = vehs1+vehs2+vehs3
        for item in vehs:
            if item.AttValue('Lane').split('-')[-1] == '1':
                self.inputlane1.append((item.AttValue('CoordRearX'), 'r', item.AttValue('Speed')))
                self.inputlane1.append((item.AttValue('CoordFrontX'), 'f', item.AttValue('Speed')))
            elif item.AttValue('Lane').split('-')[-1] == '2':
                self.inputlane2.append((item.AttValue('CoordRearX'), 'r', item.AttValue('Speed')))
                self.inputlane2.append((item.AttValue('CoordFrontX'), 'f', item.AttValue('Speed')))
            if item.AttValue('No') == self.target_id:
                target_XF = int(item.AttValue('CoordFrontX'))
                target_XR = int(item.AttValue('CoordRearX'))
                target_speed = item.AttValue('Speed')/90
        start = target_XR - 100
        end = target_XF + 101
        neighborlane = [0 for i in range(target_XF-target_XR)]
        speed_output_R = [0 for i in range(200)]
        speed_output_F = [0 for i in range(200)]
        car_output_R = [0 for i in range(200)]
        car_output_F = [0 for i in range(200)]
        temp = 0
        other = []
        tempneighbor = 0
        flag = False
        dirs = 0
        for cord, status, speed in self.inputlane2:
            speed = speed/90
            cord = int(cord)
            if start <= cord < target_XR or target_XF+1 <= cord < end:
                if status == 'r':
                    temp = cord
                else:
                    if start <= cord < target_XR:
                        if temp == 0:
                            temp = start
                        for i in range(temp-start, cord-start+1):
                            speed_output_R[i] = speed
                            car_output_R[i] = 1
                    else:
                        if target_XF+1 > temp:
                            temp = target_XF+1
                        for i in range(temp-target_XF-1, cord-target_XF):
                            speed_output_F[i] = speed
                            car_output_F[i] = 1
            elif target_XR+1 <= cord <= target_XF:
                if status == 'r':
                    flag = True
                    dirs = 1
                    tempneighbor = cord
                else:
                    flag = False
                    dirs = -1
                    if tempneighbor == 0:
                        tempneighbor = target_XR+1
                    for i in range(tempneighbor, cord+1):
                        neighborlane[i-tempneighbor] = 1
        if flag:
            for i in range(tempneighbor-target_XR-1, len(neighborlane)):
                neighborlane[i] = 1
        other.append(target_speed)
        other.append(sum(neighborlane)/len(neighborlane))
        other.append(dirs)


        temp = 0
        for cord, status, speed in self.inputlane1:
            cord = int(cord)
            speed = speed/90
            if start <= cord < target_XR or target_XF+1 <= cord < end:
                if status == 'r':
                    temp = cord
                else:
                    if start <= cord < target_XR:
                        if temp == 0:
                            temp = start
                        for i in range(temp-start, cord-start+1):
                            speed_output_R[i+100] = speed
                            car_output_R[i+100] = 1
                    else:
                        if target_XF+1 > temp:
                            temp = target_XF+1
                        for i in range(temp-target_XF-1, cord-target_XF):
                            speed_output_F[i+100] = speed
                            car_output_F[i+100] = 1
        output = []
        output.append(speed_output_R)
        output[0].extend(speed_output_F)
        output.append(car_output_R)
        output[1].extend(car_output_F)
        output.append(other)
        output.append(target_XF)
        return output






        # print(target_XF, target_XR)

    def findtarget(self):
        Vissim = self.Vissim
        vehicles = Vissim.Net.Links.ItemByKey(10002).Lanes.ItemByKey(1).Vehs.GetAll()
        while len(vehicles) == 0:
            for i in range(5):
                Vissim.Simulation.RunSingleStep()
            vehicles = Vissim.Net.Links.ItemByKey(10002).Lanes.ItemByKey(1).Vehs.GetAll()
        index = random.randrange(0, len(vehicles))
        target_id = vehicles[index].AttValue('No')
        vehicles[index].SetAttValue('Color1', 'FFF7080C')
        return target_id
test = Envi()
test.reset()