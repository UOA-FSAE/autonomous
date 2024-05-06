# Core objects for dynamic recursion
import numpy as np

class State():
    def __init__(self, node, entryVector, velocity=0.0, cost=np.inf, previousNode=False, nextState=False, Imax = False, Imin = False) -> None:
        self._node = node
        self._xy = node._xy
        self._entryVector = entryVector
        self._velocity = velocity
        self._cost = cost
        self._previousNode = previousNode
        self._nextState = nextState
        self._max = Imax
        # self._min = Imin

class Node():
    def __init__(self, bid, xy, innerdistance, outerdistance) -> None:
        self._bracketId = bid
        self._xy = xy
        self._innerDistance = innerdistance
        self._outerDistance = outerdistance
        self._stateList = []

    def __repr__(self) -> str:
        print(f"Node at {self._xy} in bracket {self._bracketId}")

class Bracket():
    def __init__(self, bid, innernode, outernode, width, nodelist) -> None:
        self._id = bid 
        self._innerNode = innernode
        self._outerNode = outernode
        self._width = width
        self._nodeList = nodelist

    def __repr__(self) -> str:
        print(f"Bracket {self._id} with width {self._width}units and with {len(self._nodeList)} nodes")