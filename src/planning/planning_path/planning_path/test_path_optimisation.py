#!/usr/bin/python3
### ------- Testing of the path_planning node ------------ ###
## Notes/Info:
# Create a boundary stamped msg type (change cmakelists.txt file as well) for temporary code testing and

from path_optimisation import path_planning as pp
import numpy as np

def test_get_control_error():
    csa = 50
    dsa = 60
    assert pp.get_control_error(0, csa,dsa) == -10

def test_trajectory_deletion():
    ppI = pp.__new__(pp)
    # array of 5 trajectories (pose arrays)
    # [[(0,0),(0.1,0),(0.2,0)],[(0.3,0), (0.4,0), (0.5,0)],[(0.6,0),(0.7,0),(0.8,0)],[(0.9,0),(1,0),(1.1,0)],[(1.2,0),(1.3,0),(1.4,0)]]
    # states are randomly generated so we will manually change all of them for this test
    trajectories, states = ppI.DEBUG_generate_trajectories(5)
    states = list(states)
    # comment out self.get_logger().info() code line
    ppI.leftbound = [[0,0],[0.5,0.2],[1.1,0],[2,0]]
    ppI.rightbound = [[0.1,0],[0.3,0],[3,0],[0.15,0]]
    ppI.trajectory_deletion(trajectories, states) 
    # result = [[(0.6,0),(0.7,0),(0.8,0)],[(1.2,0),(1.3,0),(1.4,0)]]
    final_res = []
    for i in range(len(trajectories)):
        for j in range(3):
            final_res.append(trajectories[i].poses[j].position.x)
            final_res.append(trajectories[i].poses[j].position.y)

    tempL = [0.6,0,0.7,0,0.8,0,1.2,0,1.3,0,1.4,0]
    assert np.all([final_res[i]-tempL[i] for i in range(len(final_res))]) <= 1e-2

def test_optimisation():
    ppI = pp.__new__(pp)
    # array of 5 trajectories (pose arrays)
    # [[(0,0),(0.1,0),(0.2,0)],[(0.3,0), (0.4,0), (0.5,0)],[(0.6,0),(0.7,0),(0.8,0)],[(0.9,0),(1,0),(1.1,0)],[(1.2,0),(1.3,0),(1.4,0)]]
    # states are randomly generated so we will manually change all of them for this test
    trajectories, states = ppI.DEBUG_generate_trajectories(5)
    states = [0.5, -0.5, 0.3, -0.3, 0.09]
    # comment out self.get_logger().info() code line
    ppI.leftbound = [[0,0],[0.5,0.2],[1.1,0],[2,0]]
    ppI.rightbound = [[0.1,0],[0.3,0],[3,0],[0.15,0]]
    # result = [[(0.6,0),(0.7,0),(0.8,0)],[(1.2,0),(1.3,0),(1.4,0)]]
    ppI.trajectory_deletion(trajectories,states) 
    # pass to optimisation but comment out logger code line
    best_state = ppI.optimisation(trajectories,states)

    assert best_state == 0.09
