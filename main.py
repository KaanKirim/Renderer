import pygame								#controls: wasdrf=forward,right,backward,left,upward,downward motion
import sys								#	   qe    = look right,left
import os								#	   tg    = look up,down
import numpy as np							#	   zx    = roll in positive direction, negative direction

class Vertex:								#a vertex is a point where edges of a 3D shape meet
	def __init__(self,xyz,c=(0,0,0),xy=np.array([0,0])):
		self.xyz_real=xyz
		self.xy_screen=xy
		self.color=c

eye = np.array([-700,0,0], dtype="float64") 				#the cameras inital position
screen= np.array([[1,0,0],[0,1,0],[0,0,1]], dtype="float64")   		#this matrix holds left of camera,top of camera and front of camera as vectors
							 		#first column: front, second column: left, third column: top
distance = 700          						#this is kind of like distance between your eye and the screen used for scaling
window_width,window_height=800,600
offset=+0.5*np.array([window_width,window_height])			#this offset is added to each rendered thing so things camera directly looks at appear in the center



p1=Vertex(np.array([500,500,500],dtype="float64"),(255,0,0))		#these are few example points arranged in a cube
p2=Vertex(np.array([500,-500,500],dtype="float64"),(255,255,0))
p3=Vertex(np.array([500,-500,-500],dtype="float64"),(0,255,0))
p4=Vertex(np.array([500,500,-500],dtype="float64"),(0,255,255))
p5=Vertex(np.array([1500,500,500],dtype="float64"),(0,0,255))
p6=Vertex(np.array([1500,-500,500],dtype="float64"),(255,0,255))
p7=Vertex(np.array([1500,-500,-500],dtype="float64"),(255,0,0))
p8=Vertex(np.array([1500,500,-500],dtype="float64"),(85,85,85))

points=[p1,p2,p3,p4,p5,p6,p7,p8]
lines=[[p1,p2],[p2,p3],[p3,p4],[p4,p1],[p5,p6],[p6,p7],[p7,p8],[p8,p5],[p1,p5],[p2,p6],[p3,p7],[p4,p8]]
									#these pairs of points represent edges
def dot(window,vertex):							#this function draws dots
	if isinstance(vertex,Vertex):
		if vertex.xy_screen.any()==-9999:
			return
		else:
			pygame.draw.circle(window, vertex.color, vertex.xy_screen+offset, 5)
	elif isinstance(vertex,list):					#a recursive definiton to be able to pass lists of vertices in
		for element in vertex:
			dot(window,element)


def render(camera,plane,d,point,window):  				#this function projects a point or list of points onto the screen
	if isinstance(point,Vertex):

		v = point.xyz_real - camera        			#this is the displacement of point relative to the camera
		if  (plane[:,0]@v)/np.linalg.norm(v)<0.34: 		#0.34 is cos(AOV/2), so if a point is out of AOV its coordinates are set -9999
			point.xy_screen=np.array([-9999,-9999])
			return
		od = (v@plane[:,0])	 	  			#magnitude of orthogonal distance of the point to camera
		proj0 = v-od*plane[:,0]	  				#projection of the vector on the plane that is not scaled i.e. fov=0
		proj = ((d/od)*proj0)	          			#scaled projection according to distance between camera and screen
		proj = np.linalg.inv(plane)@proj			#the reason camera is kept as a matrix and not a vector
									#this way any plane can be easily transformed into yz plane
		point.xy_screen=proj[1:]				#x values discarded

	elif isinstance(point,list):	  				#a recursive definition to be able to pass lists in
		for element in point:
			render(camera,plane,d,element,window)

def line(window,vertex1,vertex2=None):					#Fun Fact: You don't have to process lines, they are always between same points, redrawing is sufficent
	if vertex2!=None:
		pygame.draw.line(window, vertex1.color, vertex1.xy_screen+offset, vertex2.xy_screen+offset, width=1)
	if vertex2==None:						#a recursive definition to be able to pass lists in
		for element in vertex1:
			line(window,element[0],element[1])

def inform(camera,plane):						#displays important information about camera on the terminal
	os.system("clear")
	print("Current position: ",camera,"\nCurrent directions:\n", plane,"\nClosest pole:\n",np.round(plane))

def move_point(point,x=0,y=0,z=0,plane=np.eye(3,dtype="float64")): 	#this moves any point but used only on camera in the demo
	point += plane @ np.array([x,y,z])			   	#matrix product scales each column of the plane by x,y,z respectively then this is added to plane


def rotate_vector(rotator,rotatee,theta):
	rotator /= np.linalg.norm(rotator) 				#making sure vector that is rotated around is a unit vector.
	RotCros=np.array([[0, -rotator[2], rotator[1]], 		# we construct a matrix K from a vector k such that k x v = Kv
   		      	  [rotator[2], 0, -rotator[0]],
   			  [-rotator[1], rotator[0], 0]])
	RotMatrix= np.eye(3) + RotCros*np.sin(theta) + (1-np.cos(theta))*(RotCros @ RotCros) 
									#this is a rotation matrix that rotates rotatee around rotator theta radians, see:rodrigue's rotation formula
	rotatee[:] = RotMatrix@rotatee 					#rotatee is updated

pygame.init()								#initialize graphics

window = pygame.display.set_mode((window_width, window_height))
pygame.display.set_caption("RENDERER")
mov_speed=60								#some constants for controls
rot_speed=0.05






running=True								#game loop
while running:
	for event in pygame.event.get():
        	if event.type == pygame.QUIT:
            		running = False

	keys = pygame.key.get_pressed()

	if keys[pygame.K_w]:
        	move_point(eye,10,0,0,screen)
	if keys[pygame.K_s]:
        	move_point(eye,-10,0,0,screen)
	if keys[pygame.K_a]:
        	move_point(eye,0,-10,0,screen)
	if keys[pygame.K_d]:
		move_point(eye,0,10,0,screen)
	if keys[pygame.K_r]:
        	move_point(eye,0,0,-10,screen)
	if keys[pygame.K_f]:
        	move_point(eye,0,0,10,screen)
	if keys[pygame.K_e]:
        	rotate_vector(screen[:,2],screen,rot_speed)
	if keys[pygame.K_q]:
        	rotate_vector(screen[:,2],screen,-rot_speed)
	if keys[pygame.K_t]:
        	rotate_vector(screen[:,1],screen,rot_speed)
	if keys[pygame.K_g]:
        	rotate_vector(screen[:,1],screen,-rot_speed)
	if keys[pygame.K_z]:
	        rotate_vector(screen[:,0],screen,-rot_speed)
	if keys[pygame.K_x]:
		rotate_vector(screen[:,0],screen,rot_speed)


	window.fill((0, 0, 0))
	render(eye,screen,distance,points,window)
	dot(window,points)
	line(window,lines)
	inform(eye,screen)
	pygame.display.flip()
	pygame.time.Clock().tick(60)

pygame.quit()
sys.exit()
