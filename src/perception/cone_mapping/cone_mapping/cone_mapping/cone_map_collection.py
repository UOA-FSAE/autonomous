from moa_msgs.msg import Cone
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose

import math

class ConeMapList(list):
    def __init__(self):
        super().__init__(self)
        
    def insert_cone(self, car_pose, cone):
        car_coord = car_pose.pose.position.x, car_pose.pose.position.x
        cone_coord =  cone.pose.pose.position.x, cone.pose.pose.position.y

        if len(self) == 0:
            self.append(cone)
            return
        
        if len(self) == 1:
            coord0 = self[0].pose.pose.position.x, self[0].pose.pose.position.y
            if self.distance(car_coord, cone_coord) < self.distance(car_coord, coord0):
                self.insert(0, cone)
                return
            else:    
                self.append(cone)
                return~
            
        for i in range(len(self) - 1):
            coordi = self[i].pose.pose.position.x, self[i].pose.pose.position.y
            coordi1 = self[i+1].pose.pose.position.x, self[i+1].pose.pose.position.y
            if self.distance(coordi, cone_coord) < self.distance(coordi, coordi1):
                self.insert(i, cone)
                return
        
        self.append(cone)  

    def distance(self, coord1, coord2):
        return  math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)
