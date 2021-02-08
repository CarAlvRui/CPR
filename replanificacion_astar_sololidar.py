#!/usr/bin/env python
import rospy
import math
from math import pow, atan2, sqrt
from geometry_msgs.msg import TwistStamped, PoseStamped
from geometry_msgs.msg import Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from uav_abstraction_layer.srv import GoToWaypoint, TakeOff
import tf.transformations
import numpy as np
import ros_numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point




fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

class object:
    def __init__(self,name,lim_x,lim_y,lim_z):
        self.name = name
        self.lim_x = lim_x
        self.lim_y = lim_y
        self.lim_z = lim_z


class Planner:

    def __init__(self):
        
        
        self.pose_subscriber = rospy.Subscriber('/ual/odom',
                                                Odometry, self.update_pose)

        self.pose_stamped = PoseStamped() 
	self.pose_cov = PoseWithCovariance()
	self.pose = Pose()

        #me suscribo al nodo del sensor tambien
        self.lidar_subscriber = rospy.Subscriber('octomap_point_cloud_centers', PointCloud2, self.update_points)
        self.points = PointCloud2()

	#Para Rviz
	self.display_publisher = rospy.Publisher('/testcom_topic', Marker, queue_size=10)

    def displayComs(self,path):

		spheres = Marker()
		lines = Marker()


		spheres.header.frame_id = lines.header.frame_id = "odom"
		spheres.header.stamp = lines.header.stamp = rospy.Time.now()
		spheres.header.seq = lines.header.seq = 0
		spheres.ns = lines.ns = "basic_shapes"

		spheres.id = 0
		lines.id = 1

		spheres.type = Marker.SPHERE_LIST
		lines.type = Marker.LINE_STRIP

		spheres.action = lines.action = Marker.ADD

		spheres.pose.position.x = lines.pose.position.x = 0.0
		spheres.pose.position.y = lines.pose.position.y = 0.0
		spheres.pose.position.z = lines.pose.position.z = 0.0

		spheres.pose.orientation.x = lines.pose.orientation.x = 0.0
		spheres.pose.orientation.y = lines.pose.orientation.y = 0.0
		spheres.pose.orientation.z = lines.pose.orientation.z = 0.0
		spheres.pose.orientation.w = lines.pose.orientation.w = 1.0

		spheres.scale.x = 0.1
		spheres.scale.y = 0.1
		spheres.scale.z = 0.1

		lines.scale.x = 0.05
		lines.scale.y = 0.1
		lines.scale.z = 0.1

		spheres.color.r = 1.0
		spheres.color.g = 1.0
		spheres.color.b = 0.0
		spheres.color.a = 1.0

		lines.color.r = 1.0
		lines.color.g = 0.0
		lines.color.b = 0.0
		lines.color.a = 1.0

		spheres.lifetime = lines.lifetime = rospy.Duration() #Tambien vale 0 creo #Duration(10)


		pointList = []

		
		for element in path:
			point = Point()
			point.x = element[2]-20
			point.y = element[1]-20
			point.z = element[0]
			pointList.append(point)

		spheres.points = lines.points = pointList


		self.display_publisher.publish(spheres)
		self.display_publisher.publish(lines)
		#self.rate.sleep()
	
        
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        
        self.pose_stamped.pose = data.pose
	self.pose_cov = self.pose_stamped.pose
	self.pose = self.pose_cov.pose
	self.pose.position.x = round(self.pose.position.x, 4)
	self.pose.position.y = round(self.pose.position.y, 4)
	self.pose.position.z = round(self.pose.position.z, 4)
	
        
        orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]

    def update_points(self,data):	#funcion que se ejecuta cada vez que se recibe un dato del sensor

	self.points = data
        #self.medida = self.points.data

    def obtener_medidas(self,goal,wp,path):
	xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.points)
	puntos_x = np.round(xyz_array[...,0])
	puntos_y = np.round(xyz_array[...,1])
	puntos_z = np.round(xyz_array[...,2])
	puntos = []
	if wp:
		waypoint = [wp[2]-20, wp[1]-20, wp[0]]
	flag = 0

	for element in range(len(puntos_x)):
		if [puntos_x[element],puntos_y[element],puntos_z[element]] not in puntos: 
        		puntos.append([puntos_x[element],puntos_y[element],puntos_z[element]])
			#print("obstaculo en casilla:", [puntos_x[element],puntos_y[element],puntos_z[element]])


	if goal in puntos:			#Vuelvo a mirar si en la meta ha aparecido algun obstaculo
		print("Ha aparecido un obstaculo en la meta, no se puede avanzar mas")
		flag = -1

	elif path:		#miro si hay algun obstaculo nuevo en el siguiente waypoint
		if waypoint in puntos:
			flag=1

	return [flag, puntos]	

    def compute_path(self, start_cell, goal_cell, puntos):
        """Compute path."""
        path = []



	dim_x = 40
	dim_y = 40
	dim_z = 30


	
	gen_map = np.zeros((dim_x, dim_y, dim_z))


	for element in range(len(puntos)):	#anade al mapa evaluado los puntos detectados por sensor ???? preguntar
			punto = puntos[element]
			punto[0] = int(punto[0])+20  #sumar 20??
			punto[1] = int(punto[1])+20
			punto[2] = int(punto[2])
			if (gen_map[punto[0],punto[1],punto[2]] == 0) and (punto[0] in range(dim_x) and punto[1] in range(dim_y) and punto[2] in range(dim_z)) :
				gen_map[punto[0],punto[1],punto[2]] = 1
				#print([element[0],element[1],element[2]])
	

        ######################
        # 		   A*        #
        ######################
	class cell ():
		def __init__(self, x, y, z, gn, hn, fn, parent) :
			self.x = x
			self.y = y
			self.z = z
			self.gn = gn
			self.hn = hn
			self.fn = fn
			self.parent = parent

		def calculafn(self) :
			self.hn = abs(goal_cell[2] - self.x) + abs(goal_cell[1] - self.y) + abs(goal_cell[0] - self.z)	#Manhattan
			#self.hn = sqrt(pow((goal_cell[2] - self.x), 2) + pow((goal_cell[1] - self.y), 2) + pow((goal_cell[0] - self.z), 2))	#Euclidea
			self.fn = self.gn + self.hn

	
	lista_abierta = []
	lista_cerrada = []

	#Paso 0: incluyo celda origen a la lista abierta
	celda = cell(start_cell[2], start_cell[1], start_cell[0], 0, 0, 0, None)
	celda.calculafn()
	lista_abierta.append(celda)
	
	
	#Paso 1: ordeno lista abierta, saco primer elemento y lo meto en la cerrada 

	lista_abierta.sort(key=lambda x: x.fn)

	
	while (lista_abierta != False) :	#solo mientras queden elementos en la lista abierta
		celda = lista_abierta.pop(0)
		lista_cerrada.append(celda)
		if (celda.x == goal_cell[2] and celda.y == goal_cell[1] and celda.z == goal_cell[0]):	#Paso 2: he encontrado la meta, salgo
			break
				
	
		#Paso 3: calculo vecinos
		neighbors_list = []
		neighbors_list.append([celda.z, celda.y + 1, celda.x])		
		neighbors_list.append([celda.z, celda.y, celda.x + 1])
		neighbors_list.append([celda.z, celda.y - 1, celda.x])
		neighbors_list.append([celda.z, celda.y, celda.x - 1])
		neighbors_list.append([celda.z+1, celda.y, celda.x])
		neighbors_list.append([celda.z-1, celda.y, celda.x])

		#Paso 4: compruebo si existe en alguna lista
		for vecino in neighbors_list :		
			if (gen_map[vecino[2],vecino[1],vecino[0]] == 0 and vecino[0]>0) :	#solo busco si la celda esta libre de obstaculos 
				found = 0
				for objeto in lista_abierta : #si esta en la abierta, actualizo gn y reordeno
					if(objeto.x == vecino[2] and objeto.y == vecino[1] and objeto.z == vecino[0]) :
						found = 1
						g_n = celda.gn + 1
						if (g_n < objeto.gn) :
							objeto.gn = g_n
							objeto.calculafn()
							objeto.parent = celda
							lista_abierta.sort(key=lambda x: x.fn) #reordeno la lista
						
						
				for objeto in lista_cerrada : #si esta en la cerrada, hago lo mismo
					if(objeto.x == vecino[2] and objeto.y == vecino[1] and objeto.z == vecino[0]) :
						found = 1
						g_n = celda.gn + 1
						if (g_n < objeto.gn) :
							objeto.gn = g_n
							objeto.calculafn()
							objeto.parent = celda
							lista_abierta.sort(key=lambda x: x.fn) #reordeno la lista
		
				if(found==0) :			#si no esta en ninguna lista, lo incluyo y reordeno
					neighbor = cell(vecino[2],vecino[1], vecino[0], celda.gn + 1, 0, 0, celda)
					neighbor.calculafn()
					lista_abierta.append(neighbor)
					lista_abierta.sort(key=lambda x: x.fn)

	while(celda.parent != None):			#Una vez encontrada la meta, obtengo el camino
		path.append([celda.z, celda.y, celda.x])
		celda = celda.parent

	path.append(start_cell)
	path.reverse()
	
	
        ######################
        # End A*             #
        ######################

        # Print path
	x = []
	y = []
	z = []
        
	for point in path:
		x.append(point[2]-20)
		y.append(point[1]-20)
		z.append(point[0])
		print(point[2]-20, point[1]-20, point[0])

        
        
        return path


    def goto(self):
        """Moves the robot to the goal."""

        goal_pose = Pose()
        	

	while True:
		data = input("Set your x goal: ")
		if data <= 20 and data >= -20:
			goal_pose.position.x = data
			break
		else:
			print("x goal must be between -20, 20")

	while True:
		data = input("Set your y goal: ")
		if data <= 20 and data >= -20:
			goal_pose.position.y = data
			break
		else:
			print("y goal must be between -20, 20")

	while True:
		data = input("Set your z goal: ")
		if data > 0:
			goal_pose.position.z = data
			break
		else:
			print("z goal must be greater than 0")


        current_z= math.trunc(self.pose.position.z)
	current_y= math.trunc(self.pose.position.y)
	current_x= math.trunc(self.pose.position.x)

	current_cell = [current_z, current_y+20, current_x+20]
	fin = 0
	
        while(fin==0):


		goal_z= math.trunc(goal_pose.position.z)  
		goal_y= math.trunc(goal_pose.position.y)
		goal_x= math.trunc(goal_pose.position.x)
		goal = [goal_x,goal_y,goal_z]
		goal_cell = [goal_z, goal_y+20, goal_x+20]

		result = self.obtener_medidas(goal,0,0)
		fin = result[0]
		puntos = result[1]

		if fin == -1:
			break;

        	path = self.compute_path(current_cell,goal_cell, puntos) #le pasa al planificador las nuevas casillas ocupadas detectadas por sensor
        	
		self.displayComs(path)

        	for point in path:
            # TODO: Call service GoTo
		
			try:

				fin = self.obtener_medidas(goal,point,path)						
             			if fin == -1:
					break;
                        	if fin == 1 :
                          		print("encontrado obst no previsto en el camino, me voy a chocar")
					current_z= math.trunc(self.pose.position.z)
					current_y= math.trunc(self.pose.position.y)
					current_x= math.trunc(self.pose.position.x)
					if current_z == 0:
						current_z = 1
					current_cell = [current_z, current_y+20, current_x+20]
					print("Replanificando desde: ", [current_x, current_y, current_z])	
					fin = 0				
                               		break;
				
                                               
				goto_service=rospy.ServiceProxy('/ual/go_to_waypoint',GoToWaypoint)

				waypoint = PoseStamped()		
				waypoint.header.seq = 0 
				waypoint.header.stamp.secs = 0 
				waypoint.header.stamp.nsecs = 0
				waypoint.header.frame_id = ''
				waypoint.pose.position.x = point[2]-20
				waypoint.pose.position.y = point[1]-20
				waypoint.pose.position.z = point[0]
				waypoint.pose.orientation.x = 0
				waypoint.pose.orientation.y = 0
				waypoint.pose.orientation.z = 0
				waypoint.pose.orientation.w = 0
             
	

				goto_service(waypoint,1)
				fin=1
		

			except rospy.ServiceException, e:
				print "Service goto call failed: %s"%e
        		pass

	print("Finalizado. Pulse Ctrl+C para salir") 
       
if __name__ == '__main__':
    try:
        rospy.init_node('robot_planner', anonymous=True)

        x = Planner()
        x.goto()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
