'''
File: operators_for_x.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
as the file name, here are some operators for the x variable.
----------
Author: 626
Created Date: 2024.01.02
'''


class Relocate():
    def __init__(self, k=1):
        self.k = k # how many points relocate together, k=1:relocate, k>1:Or-Opt

    def run(self, solution):
        """relocate point and the point next to it randomly inter/inner route (capacity not considered)

        Args:
            solution (List[int]): idxs of points of each route (route seperate with idx 0)

        Returns:
            neighbours (List[List[int]]): idxs of points of each route (seperate with idx 0) of each neighbour 
        """
        neighbours = []
        # 1. choose a point to relocate
        for pi in range(1, len(solution)-self.k):
            # 2. choose a position to put
            for li in range(1, len(solution)-self.k): # can't relocate to start/end
                neighbour = solution.copy()
                points = []
                for _ in range(self.k):
                    points.append(neighbour.pop(pi))
                for p in points[::-1]:
                    neighbour.insert(li, p)
                neighbours.append(neighbour)
        return neighbours     

    def get(self, solution):
        pi = np.random.randint(1, len(solution)-self.k)
        li = np.random.randint(1, len(solution)-self.k)
        neighbour = solution.copy()
        points = []
        for _ in range(self.k):
            points.append(neighbour.pop(pi))
        for p in points[::-1]:
            neighbour.insert(li, p)
        assert len(neighbour) == len(solution)
        return neighbour

class Exchange():
    def __init__(self, k=1):
        self.k = k # how many points exchange together

    def run(self, solution):
        """exchange two points randomly inter/inner route (capacity not considered)
        ps: Exchange operator won't change the points number of each vehicle

        Args:
            solution (List[int]): idxs of points of each route (route seperate with idx 0)

        Returns:
            neighbours (List[List[int]]): idxs of points of each route (seperate with idx 0) of each neighbour 
        """
        neighbours = []
        # 1. choose point i
        for pi in range(1, len(solution)-2*self.k-1):
            # 2. choose point j
            for pj in range(pi+self.k+1, len(solution)-self.k): 
                # if math.prod(solution[pi:pi+self.k]) == 0 or math.prod(solution[pj:pj+self.k]) == 0: # don't exchange 0
                #     continue
                neighbour = solution.copy()
                tmp = neighbour[pi:pi+self.k].copy()
                neighbour[pi:pi+self.k] = neighbour[pj:pj+self.k]
                neighbour[pj:pj+self.k] = tmp
                neighbours.append(neighbour)
        return neighbours    

    def get(self, solution):
        pi = np.random.randint(1, len(solution)-2*self.k-1)
        pj = np.random.randint(pi+self.k+1, len(solution)-self.k)
        while math.prod(solution[pi:pi+self.k]) == 0 or math.prod(solution[pj:pj+self.k]) == 0: # don't exchange 0
            pi = np.random.randint(1, len(solution)-2*self.k-1)
            pj = np.random.randint(pi+self.k+1, len(solution)-self.k)
        neighbour = solution.copy()
        tmp = neighbour[pi:pi+self.k].copy()
        neighbour[pi:pi+self.k] = neighbour[pj:pj+self.k]
        neighbour[pj:pj+self.k] = tmp
        assert len(neighbour) == len(solution)
        return neighbour

class Reverse():
    def __init__(self):
        pass

    def run(self, solution):
        """reverse route between two points randomly inter/inner route (capacity not considered)

        Args:
            solution (List[int]): idxs of points of each route (route seperate with idx 0)

        Returns:
            neighbours (List[List[int]]): idxs of points of each route (seperate with idx 0) of each neighbour 
        """
        neighbours = []
        # 1. choose point i
        for pi in range(1, len(solution)-2):
            # 2. choose point j
            for pj in range(pi+1, len(solution)-1): 
                neighbour = solution.copy()
                neighbour[pi:pj+1] = neighbour[pj:pi-1:-1]
                neighbours.append(neighbour)
        return neighbours 
    
    def get(self, solution):
        pi = np.random.randint(1, len(solution)-2)
        pj = np.random.randint(pi+1, len(solution)-1)
        neighbour = solution.copy()
        neighbour[pi:pj+1] = neighbour[pj:pi-1:-1]
        assert len(neighbour) == len(solution)
        return neighbour