import numpy as np
class Transformation :
    def __init__(self,angle,turn_over,dimension = 2):
        """
        1.angle: (rad) means rolling form the asix x(original) to x'(target).
        Considering positive and negative direction
        2.turn_over: (True/False) True means the transformation is a turn over along y axis.
        3.dimension: (int) 2D or 3D transformation. Default is 2D.
        """
        self.angle = angle 
        self.turn_over = turn_over
        self.dimension = dimension
        
    def preparation(self):
        # the transformation matrix is a rotation matrix
        # considering the angle and the turn over
        if self.dimension == 2:
            T = np.matrix([[np.cos(self.angle), -np.sin(self.angle)],
                           [np.sin(self.angle),  np.cos(self.angle)]])
            if self.turn_over == True:
                Q = np.matrix([[1, 0],
                               [0,-1]])
                T = Q * T
            return T
        
    def transformation(self,point,dim):
        if dim ==  2:
            T = self.preparation()
            point = T * point
        return point
    
transformation = Transformation(angle = 5*np.pi/6 ,turn_over = True ,dimension = 2)
point = np.matrix([[np.cos(np.pi/6)],[0.5]])
print(transformation.transformation(point,2),point)