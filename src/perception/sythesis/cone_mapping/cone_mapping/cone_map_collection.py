from moa_msgs.msg import Cone
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

import math

class ConeMapList(list):
    def __init__(self):
        super().__init__(self)
        
    def insert_cone(self, car_pose, cone_coord):
        car_coord = car_pose.position.x, car_pose.position.y
        # Insert if first cone detected
        if len(self) == 0:
            self.append(cone_coord)
            return
        
        #
        if len(self) == 1:
            coord0 = self[0].x, self[0].y
            if self.distance(car_coord, cone_coord) < self.distance(car_coord, coord0):
                self.insert(0, cone_coord)
                return
            else:    
                self.append(cone_coord)
                return
            
        for i in range(len(self) - 1):
            coordi = self[i].position.x, self[i].position.y
            coordi1 = self[i+1].position.x, self[i+1].position.y
            if self.distance(coordi, cone_coord) < self.distance(coordi, coordi1):
                self.insert(i, cone_coord)
                return
        
        self.append(cone_coord)  

    def distance(self, coord1, coord2):
        return  math.sqrt((coord1[0].x - coord2[0].x)**2 + (coord1[1].y - coord2[1].y)**2)



class ConeWithId(Cone):
    def __init__(self, id):
        super().__init__(self)
    



cone = ConeWithId(cone, id)