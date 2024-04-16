# Core objects for dynamic recursion

class Node():
    def __init__(self, bid, xy, velocity, innerdistance, outerdistance, nextnode, cost) -> None:
        self._bracketId = bid
        self._xy = xy
        self._velocity = velocity
        self._innerDistance = innerdistance
        self._outerDistance = outerdistance
        self._nextNode = nextnode
        # self._angle = angle
        self._cost = cost

    def __repr__(self) -> str:
        print(f"Node at {self._xy} in bracket {self._bracketId} with state {self._velocity}m/s going to {self._nextNode._xy} node with cost {self._cost}s")


class Bracket():
    def __init__(self, bid, innernode, outernode, width, nodelist) -> None:
        self._id = bid 
        self._innerNode = innernode
        self._outerNode = outernode
        self._width = width
        self._nodeList = nodelist

    def __repr__(self) -> str:
        print(f"Bracket {self._id} with width {self._width}units and with {len(self._nodeList)} nodes")