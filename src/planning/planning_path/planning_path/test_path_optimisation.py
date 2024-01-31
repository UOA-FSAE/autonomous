#!/usr/bin/python3
### ------- Testing of the path_planning node ------------ ###
## Notes/Info:

from planning_path.path_optimisation import trajectory_optimization as to
from planning_path.path_optimisation import trajectory_following as tf
import numpy as np

def test_get_control_error():
    csa = 50
    dsa = 60
    assert tf.get_control_error(0, csa,dsa) == -10

def test_trajectory_deletion():
    toI = to.__new__(to)
    # array of 5 trajectories (pose arrays)
    # [[(0,0),(0.1,0),(0.2,0)],[(0.3,0), (0.4,0), (0.5,0)],[(0.6,0),(0.7,0),(0.8,0)],[(0.9,0),(1,0),(1.1,0)],[(1.2,0),(1.3,0),(1.4,0)]]
    # states are randomly generated so we will manually change all of them for this test
    trajectories, states = toI.DEBUG_generate_trajectories(5)
    states = list(states)
    # comment out self.get_logger().info() from trajectory deletion method
    toI.leftbound = [[0,0],[0.5,0.2],[1.1,0],[2,0]]
    toI.rightbound = [[0.1,0],[0.3,0],[3,0],[0.15,0]]
    toI.trajectory_deletion(trajectories, states) 
    # result = [[(0.6,0),(0.7,0),(0.8,0)],[(1.2,0),(1.3,0),(1.4,0)]]
    final_res = []
    for i in range(len(trajectories)):
        for j in range(3):
            final_res.append(trajectories[i].poses[j].position.x)
            final_res.append(trajectories[i].poses[j].position.y)

    tempL = [0.6,0,0.7,0,0.8,0,1.2,0,1.3,0,1.4,0]
    assert np.all([final_res[i]-tempL[i] for i in range(len(final_res))]) <= 1e-2

def test_optimisation():
    toI = to.__new__(to)
    # array of 5 trajectories (pose arrays)
    # [[(0,0),(0.1,0),(0.2,0)],[(0.3,0), (0.4,0), (0.5,0)],[(0.6,0),(0.7,0),(0.8,0)],[(0.9,0),(1,0),(1.1,0)],[(1.2,0),(1.3,0),(1.4,0)]]
    # states are randomly generated so we will manually change all of them for this test
    trajectories, states = toI.DEBUG_generate_trajectories(5)
    states = [0.5, -0.5, 0.3, -0.3, 0.09]
    # comment out self.get_logger().info() code line
    toI.leftbound = [[0,0],[0.5,0.2],[1.1,0],[2,0]]
    toI.rightbound = [[0.1,0],[0.3,0],[3,0],[0.15,0]]
    # result = [[(0.6,0),(0.7,0),(0.8,0)],[(1.2,0),(1.3,0),(1.4,0)]]
    toI.trajectory_deletion(trajectories,states) 
    # pass to optimisation but comment out logger from optimization
    best_state = toI.optimisation(trajectories,states)

    assert best_state == 0.09
