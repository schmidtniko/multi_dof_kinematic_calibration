#!/bin/python
import pygame
from pygame.locals import *
from OpenGL.GL import *
import time

import numpy
import math
from OpenGL.GLU import *
from OpenGL.GLUT import *
import json

SCREEN_WIDTH  = 1024
SCREEN_HEIGHT = 768

def quatToRot(q):
    w = q[0]
    x = q[1]
    y = q[2]
    z = q[3]
    
    xx2 = 2 * x * x
    yy2 = 2 * y * y
    zz2 = 2 * z * z
    xy2 = 2 * x * y
    wz2 = 2 * w * z
    zx2 = 2 * z * x
    wy2 = 2 * w * y
    yz2 = 2 * y * z
    wx2 = 2 * w * x
    
    rmat = numpy.eye(3)
    rmat[0,0] = 1. - yy2 - zz2
    rmat[0,1] = xy2 - wz2
    rmat[0,2] = zx2 + wy2
    rmat[1,0] = xy2 + wz2
    rmat[1,1] = 1. - xx2 - zz2
    rmat[1,2] = yz2 - wx2
    rmat[2,0] = zx2 - wy2
    rmat[2,1] = yz2 + wx2
    rmat[2,2] = 1. - xx2 - yy2
    return rmat

###############################################################################
# Utility functions

def render_coordinate_axes(len=1):
    glLineWidth(2.5)
    glBegin(GL_LINES)
    glColor3f(1,0,0)
    glVertex3f(0,0,0)
    glVertex3f(len,0,0)
    glColor3f(0,1,0)
    glVertex3f(0,0,0)
    glVertex3f(0,len,0)
    glColor3f(0,0,1)
    glVertex3f(0,0,0)
    glVertex3f(0,0,len)
    glEnd()
    glColor3f(1,1,1)
    glLineWidth(1)

def render_floor(siz=20):
    glColor3f(0.0, 0.0, 0.3)
    glBegin(GL_QUADS)
    glVertex3f(-siz, -siz, 0)
    glVertex3f(-siz, siz, 0)
    glVertex3f(siz, siz, 0)
    glVertex3f(siz, -siz, 0)
    glEnd()

    glEnable(GL_LINE_SMOOTH)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glLineWidth(2.5)

    glColor3f(0.8, 0.8, 0.8)
    glBegin(GL_LINES)
    for x in range(-siz,siz):
        glVertex3f(x, -siz, 0)
        glVertex3f(x, siz, 0)
        glVertex3f(-siz, x, 0)
        glVertex3f(siz, x, 0)
    glEnd()
    glColor3f(1, 1, 1)

    glLineWidth(1)
    glDisable(GL_BLEND);
    glDisable(GL_LINE_SMOOTH)

def render_sphere(radius=1,pos=[0,0,0]):

    def solid_sphere(r,lats,longs):
        for i in range(0,lats+1):
            lat0 = math.pi * (-0.5 + (i - 1) / float(lats));
            z0  = math.sin(lat0);
            zr0 =  math.cos(lat0);

            lat1 = math.pi * (-0.5 + i / float(lats));
            z1 = math.sin(lat1);
            zr1 = math.cos(lat1);

            glBegin(GL_QUAD_STRIP);
            for j in range(0,longs+1):
                lng = 2 * math.pi * (j - 1) / float(longs)
                x = math.cos(lng);
                y = math.sin(lng);

                glNormal3f(x * zr0, y * zr0, z0);
                glVertex3f(r*x * zr0, r*y * zr0, r*z0);
                glNormal3f(x * zr1, y * zr1, z1);
                glVertex3f(r*x * zr1, r*y * zr1, r*z1);
            glEnd();

    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glTranslatef(pos[0],pos[1],pos[2])
    
    solid_sphere(radius,15,15)

    glPopMatrix()
def render_box(mins,maxs):
    mi=mins
    ma=maxs
    glBegin(GL_QUADS);
    glVertex3f(mi[0], mi[1],mi[2]);glVertex3f(ma[0], mi[1],mi[2]);
    glVertex3f(ma[0], mi[1],ma[2]);glVertex3f(mi[0], mi[1],ma[2]);
    glVertex3f(ma[0], ma[1],ma[2]);glVertex3f(mi[0], ma[1],ma[2]);
    glVertex3f(mi[0], ma[1],mi[2]);glVertex3f(ma[0], ma[1],mi[2]);
    glVertex3f(mi[0], mi[1],mi[2]);glVertex3f(mi[0], ma[1],mi[2]);
    glVertex3f(mi[0], ma[1],ma[2]);glVertex3f(mi[0], mi[1],ma[2]);
    glVertex3f(ma[0], ma[1],ma[2]);glVertex3f(ma[0], mi[1],ma[2]);
    glVertex3f(ma[0], mi[1],mi[2]);glVertex3f(ma[0], ma[1],mi[2]);
    glVertex3f(mi[0], mi[1],mi[2]);glVertex3f(ma[0], mi[1],mi[2]);
    glVertex3f(ma[0], ma[1],mi[2]);glVertex3f(mi[0], ma[1],mi[2]);
    glVertex3f(ma[0], ma[1],ma[2]);glVertex3f(mi[0], ma[1],ma[2]);
    glVertex3f(mi[0], mi[1],ma[2]);glVertex3f(ma[0], mi[1],ma[2]);
    glEnd();

def render_marker(width, height):
    render_coordinate_axes((width+height)*0.5)
    glColor3f(0.3, 0.3, 0.3)
    glBegin(GL_QUADS)
    glVertex3f(-width/2.0, -height/2.0, 0)
    glVertex3f(width/2.0, -height/2.0, 0)
    glVertex3f(width/2.0, height/2.0, 0)
    glVertex3f(-width/2.0, height/2.0, 0)
    glEnd()
    glColor3f(1.0, 1.0, 1.0)

###############################################################################
# Camera class

class Camera(object):
    def __init__(self):
        self.pos=numpy.array([0,0,0])
        self.look_at=numpy.array([0,1,0])
        self.up=numpy.array([0,0,1])
        self.sensitivity=1.0/170.0
    def strafe_camera(self,dist):
        right=numpy.cross(self.look_at-self.pos,self.up)
        right=right/numpy.linalg.norm(right)
        self.pos=self.pos+right*dist*2.0/3.0
        self.look_at=self.look_at+right*dist*2.0/3.0
    def move_camera(self,dist):
        view=self.look_at-self.pos
        view=view/numpy.linalg.norm(view)
        self.pos=self.pos+view*dist
        self.look_at=self.look_at+view*dist
    def set_view_by_mouse(self,dx,dy):
        if (dx==0) and (dy==0):
            return
        view=self.look_at-self.pos
        view=view/numpy.linalg.norm(view)
        cur_rot_x=math.pi*0.5-math.acos(numpy.dot(self.up,view))

        ang_y=-dx*self.sensitivity
        ang_z=-dy*self.sensitivity

        pi_half=math.pi*0.5-0.03125

        if (cur_rot_x+ang_z>pi_half):
            ang_z=pi_half-cur_rot_x;
        elif (cur_rot_x+ang_z<-pi_half):
            ang_z=-pi_half-cur_rot_x;

        axis=numpy.cross(self.look_at-self.pos,self.up)
        axis=axis/numpy.linalg.norm(axis)
        axis[2]=0
        self.rotate_view(axis,ang_z);
        self.rotate_view([0,0,1],ang_y);
    def rotate_view(self,axis,ang):
        def rotation_matrix(axis, theta):
            """
            Return the rotation matrix associated with counterclockwise rotation about
            the given axis by theta radians.
            """
            axis = numpy.asarray(axis)
            theta = numpy.asarray(theta)
            axis = axis/math.sqrt(numpy.dot(axis, axis))
            a = math.cos(theta/2)
            b, c, d = -axis*math.sin(theta/2)
            aa, bb, cc, dd = a*a, b*b, c*c, d*d
            bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
            return numpy.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                             [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                             [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])
        axis=axis/numpy.linalg.norm(axis)
        m=rotation_matrix(axis,ang)

        view=self.look_at-self.pos
        view=view/numpy.linalg.norm(view)
        self.look_at=self.pos+numpy.dot(m,view)

###############################################################################
# Main Game class

class VisMarker(object):
    def __init__(self, R, t, marker_width, marker_height, id):
        self.R=R
        self.t=t
        self.mat = numpy.eye(4,4)
        self.mat[0:3,0:3]=R
        self.mat[0:3,3]=t
        self.width=marker_width
        self.height=marker_height
        self.id=id
        
class VisCamera(object):
    def __init__(self, R, t, id):
        self.R=R
        self.t=t
        self.mat = numpy.eye(4,4)
        self.mat[0:3,0:3]=R
        self.mat[0:3,3]=t
        self.mat=numpy.linalg.inv(self.mat)
        self.id=id
 
class Game(object):
    def __init__(self):
 
        pygame.init()
        flag = OPENGL | DOUBLEBUF | RESIZABLE
        self.surface = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT),flag)
        self.opengl_init()
        pygame.display.set_caption("multi_dof_kinematic_calibration -- Visualization")

        glutInit(1,ctypes.c_char_p(0))

        self.markers=[]
        self.cameras=[]
        self.points=[]
        self.lines=[]
        if len(sys.argv)>=2:
            calib_data_file = sys.argv[1] + "/calibration_data.json"
            reconstruction_file = ""
            with open(calib_data_file) as f:    
                data = json.load(f)
                reconstruction_file = data["world_reference"]
            calibration_results_file = sys.argv[1] + "/calibration_result_visualization.json"
            if os.path.isfile(reconstruction_file):
                self.load_reconstruction_results(reconstruction_file)
            if os.path.isfile(calibration_results_file):
                self.load_calibration_results(calibration_results_file)

        self.dl=-1

        self.last_lbtn=False

        self.stategame=1
 
        self.loop()
    def load_reconstruction_results(self, filename):
        print("Loading reconstruction results from %s" % (filename))
        with open(filename) as f:    
            data = json.load(f)
        for tag in data["reconstructed_tags"]:
            translation = [float(v) for v in tag["translation"]]
            rotation_q = [float(v) for v in tag["rotation"]]
            marker_width = float(tag["width"])
            marker_height = float(tag["height"])
            mid = int(tag["id"])
            m = VisMarker(quatToRot(rotation_q), translation,marker_width,marker_height, mid)
            self.markers.append(m)
        print("Read %d Markers"%(len(self.markers)))
    def load_calibration_results(self, filename):
        print("Loading reconstruction results from %s" % (filename))
        with open(filename) as f:    
            data = json.load(f)
        for tag in data["reconstructed_cameras"]:
            translation = [float(v) for v in tag["translation"]]
            rotation_q = [float(v) for v in tag["rotation"]]
            cid = tag["caption"]
            c = VisCamera(quatToRot(rotation_q), translation, cid)
            self.cameras.append(c)
        self.lines=data["lines"]
        print("Read %d Cameras"%(len(self.cameras)))
        self.points=data["points"]
        print("Read %d Points"%(len(self.points)))
        
 
    def opengl_init(self):
        #init gl
        glClearColor(1.0,0.0,0.0,1.0)
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        #glOrtho(0,SCREEN_WIDTH,SCREEN_HEIGHT,0,0,1)
        gluPerspective(85.0, SCREEN_WIDTH/float(SCREEN_HEIGHT), 0.01, 200.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        self.cam=Camera()
        self.cam.pos=numpy.array([2,0,1])
 
        glDisable(GL_TEXTURE_2D)
        glDisable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)

 
    def draw(self):
        glClearColor(0.0,0.0,0.0,1.0)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(self.cam.pos[0],self.cam.pos[1],self.cam.pos[2],self.cam.look_at[0],self.cam.look_at[1],self.cam.look_at[2],self.cam.up[0],self.cam.up[1],self.cam.up[2]);

        render_floor()
        render_coordinate_axes()
        #glColor3f(1,0,0)
        #self.render_sphere(0.1)
        #self.render_box([-1,-1,-1],[1,1,1])
        if self.dl!=-1:
            glCallList(self.dl)
            return
        self.dl=glGenLists(1)
        glNewList(self.dl, GL_COMPILE_AND_EXECUTE)
        for m in self.markers:
            glPushMatrix();
            glMultMatrixd(m.mat.T)
            render_marker(m.width,m.height)

            h=str.encode(str(m.id))
            glTranslatef(0,0,0.02)
            glScale(m.width,m.width,1)
            glScale(0.002,0.002,0.002)
            glutStrokeString(GLUT_STROKE_ROMAN, ctypes.c_char_p(h))
            glPopMatrix();
        for c in self.cameras:
            glPushMatrix();
            glMultMatrixd(c.mat.T)
            render_coordinate_axes(0.03)#15)

            h=str.encode(str(c.id))
            glTranslatef(0,0,0.02)
            glRotatef(180,0,0,1)
            glScale(0.0005,0.0005,0.0005)
            #glutStrokeString(GLUT_STROKE_ROMAN, ctypes.c_char_p(h))
            glPopMatrix();
        glBegin(GL_POINTS)
        glColor3f(1,1,1)
        for p in self.points:
            glVertex3f(p[0],p[1],p[2])
        glEnd()
        glBegin(GL_LINES);
        for p in self.lines:
            glVertex3f(p["from"][0], p["from"][1], p["from"][2]);
            glVertex3f(p["to"][0], p["to"][1], p["to"][2]);
        glEnd()
        glEndList()


 
    def handle_events(self,dt):
        # Mouse
        (lbtn,rbtn,bla)=pygame.mouse.get_pressed()
        if lbtn:
            (dx,dy)=pygame.mouse.get_rel()
            if self.last_lbtn==True:
                self.cam.set_view_by_mouse(dx,dy)
        self.last_lbtn=lbtn

        # Keyboard
        cam_speed=2.0*dt

        keys = pygame.key.get_pressed();
        if keys[K_SPACE]:
           cam_speed=cam_speed*5
        if keys[K_s]:
            self.cam.move_camera(-cam_speed)
        if keys[K_w]:
            self.cam.move_camera(cam_speed)
        if keys[K_a]:
            self.cam.strafe_camera(-cam_speed)
        if keys[K_d]:
            self.cam.strafe_camera(cam_speed)
 
    def loop(self):
        first=True
        last_time=int(round(time.time() * 1000))
        while self.stategame==1:
            for event in pygame.event.get():
                if event.type == QUIT \
                or (event.type == KEYDOWN and event.key ==  K_ESCAPE):
                    self.stategame = 0
                if event.type == VIDEORESIZE:
                    SCREEN_WIDTH = event.w
                    SCREEN_HEIGHT = event.h
                    flag = OPENGL | DOUBLEBUF | RESIZABLE
                    self.surface = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT),flag)
                    glViewport(0,0,event.w,event.h)
                    self.opengl_init()


            cur_time=int(round(time.time() * 1000))
            dt=(cur_time-last_time)/1000.0
            last_time=cur_time

            if first:
                dt=1.0/60.0
                first=False

 
            self.handle_events(dt)
 
            self.draw()
 
            pygame.display.flip()
 
###############################################################################

if __name__ == '__main__':
    Game()
