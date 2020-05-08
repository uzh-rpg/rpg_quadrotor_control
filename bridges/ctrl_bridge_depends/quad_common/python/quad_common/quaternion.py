# -*- coding: utf-8 -*-

import numpy as np

def main():
    q1 = Quaternion(np.array([1,0,0,0]) )                        # directly assign the values [w x y z]
    q2 = Quaternion(np.array([1,2,3]), angle = np.deg2rad(0.0) ) # angle axis representation
    q3 = Quaternion(np.deg2rad(np.array([10,20,30])))            # from euler angles
    q4 = Quaternion()
    
    print rad2deg(q4.to_euler_angles())

class Quaternion:
    def __init__(self, input_array=np.array([1,0,0,0]), angle = '' ):
        if len(input_array)==4:
           self.set_from_array(input_array)
        elif len(input_array) == 3:
            if angle != '':
              self.set_from_axis_angle(input_array, angle)  
            else:
              self.set_from_euler_angles(input_array)
        else:
            print 'usage: .....'
            
    def set_from_axis_angle(self, axis, angle):
        axis = axis/np.linalg.norm(axis)
        self.w = np.cos(angle/2.0)
        self.x = axis[0]*np.sin(angle/2.0)
        self.y = axis[1]*np.sin(angle/2.0)
        self.z = axis[2]*np.sin(angle/2.0)

    def set_from_array(self, array):
        self.w = array[0]
        self.x = array[1]
        self.y = array[2]
        self.z = array[3]

    def set_from_euler_angles(self, euler_angles):
        r = euler_angles[0]/2.0;
        p = euler_angles[1]/2.0;
        y = euler_angles[2]/2.0;          
        self.w = np.cos(r)*np.cos(p)*np.cos(y) + np.sin(r)*np.sin(p)*np.sin(y);
        self.x = np.sin(r)*np.cos(p)*np.cos(y) - np.cos(r)*np.sin(p)*np.sin(y);
        self.y = np.cos(r)*np.sin(p)*np.cos(y) + np.sin(r)*np.cos(p)*np.sin(y);
        self.z = np.cos(r)*np.cos(p)*np.sin(y) - np.sin(r)*np.sin(p)*np.cos(y);
        
    def to_rot_matrix(self):
        R_tmp = self.Q_bar().transpose() * self.Q()
        return R_tmp[1:4,1:4]

    def to_euler_angles(self):
        euler_angles = np.matrix(np.zeros((3,1)))
        euler_angles[0] = np.arctan2(2*self.w*self.x + 2*self.y*self.z, self.w*self.w - self.x*self.x - self.y*self.y + self.z*self.z);
        euler_angles[1] = -np.arcsin(2*self.x*self.z - 2*self.w*self.y);
        euler_angles[2] = np.arctan2(2*self.w*self.z + 2*self.x*self.y, self.w*self.w + self.x*self.x - self.y*self.y - self.z*self.z);
        return euler_angles    
    
    def to_geometry_msg(self):
        geometry_q = geometry_msgs.msg.Quaternion()
        geometry_q.w = self.w
        geometry_q.x = self.x
        geometry_q.y = self.y
        geometry_q.z = self.z
        return geometry_q
        
    def Q(self):
        return np.matrix(
    [ [self.w, -self.x, -self.y, -self.z],
      [self.x,  self.w, -self.z,  self.y],
      [self.y,  self.z,  self.w, -self.x],
      [self.z, -self.y,  self.x,  self.w] ])
      
    def Q_bar(self):
        return np.matrix(
        [ [self.w, -self.x, -self.y, -self.z],
          [self.x,  self.w,  self.z, -self.y],
          [self.y, -self.z,  self.w,  self.x],
          [self.z,  self.y, -self.x,  self.w] ])
        
    def multiply_with(self, p):
        result = self.Q() * np.matrix([p.w, p.x, p.y, p.z]).transpose()
        return Quaternion().set_from_array(result)

    def __str__(self):
        return ('[w: %.2f, x: %.2f, y: %.2f, z: %.2f]\'') % (self.w,self.x,self.y,self.z)

    def inverse(self):
        return Quaternion(np.array([self.w,-self.x,-self.y,-self.z]) )

    def rotate_vector(self,v):
        return self.to_rot_matrix()*v

if __name__ == '__main__':
    main()
    
    
    
