#!/usr/bin/env python
import rospy
import math
from math import pow, atan2, sqrt
from geometry_msgs.msg import TwistStamped, PoseStamped
from geometry_msgs.msg import Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from uav_abstraction_layer.srv import GoToWaypoint, TakeOff
import tf.transformations
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import random

#rviz display
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
#
from sensor_msgs.msg import PointCloud2, PointField
import ros_numpy
#

class object:
	def __init__(self,name,lim_x,lim_y,lim_z):
		self.name = name
		self.lim_x = lim_x
		self.lim_y = lim_y
		self.lim_z = lim_z

class branch:
	def __init__(self,coord,father,cost):
		self.coord = coord
		self.father = father
		self.cost = cost

class Planner:

	def __init__(self):

		#rviz display
		rospy.init_node('robot_planner', anonymous=True)
		self.display_publisher = rospy.Publisher('/testcom_topic', Marker, queue_size=10)
		self.rate = rospy.Rate(10)
		#
		#odometria
		self.pose_subscriber = rospy.Subscriber('/ual/odom', Odometry, self.update_pose)
		self.pose_stamped = PoseStamped() 
		self.pose_cov = PoseWithCovariance()
		self.pose = Pose()
		#
		#octomap
		self.lidar_subscriber = rospy.Subscriber('octomap_point_cloud_centers', PointCloud2, self.update_points)
		self.points = PointCloud2()
		#

	def update_pose(self, data):
		self.pose_stamped.pose = data.pose
		self.pose_cov = self.pose_stamped.pose
		self.pose = self.pose_cov.pose
		self.pose.position.x = round(self.pose.position.x, 4)
		self.pose.position.y = round(self.pose.position.y, 4)
		self.pose.position.z = round(self.pose.position.z, 4)

		self.orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
		self.current_cell = [self.pose.position.x, self.pose.position.y, self.pose.position.z]

	def update_points(self, data):
		self.points = data
		self.octo_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.points)

	def collision_evaluation(self, x, y, z, size):
		result = False
		for center in self.octo_array:
			if (x <= center[0] + size/2 and x >= center[0] - size/2)\
			and (y <= center[1] + size/2 and y >= center[1] - size/2)\
			and (z <= center[2] + size/2 and z >= center[2] - size/2):
				result = True
				break
		
		if self.map_sel != 0:
			for ob_ev in self.object_list:
				if (x <= ob_ev.lim_x[1] and x >= ob_ev.lim_x[0])\
				and (y <= ob_ev.lim_y[1] and y >= ob_ev.lim_y[0])\
				and (z <= ob_ev.lim_z[1] and z >= ob_ev.lim_z[0]):
					result = True
					break

		return result

	def rute_free(self, start, goal, step, size):
		m = math.sqrt((goal[0]-start[0])**2 +(goal[1]-start[1])**2 +(goal[2]-start[2])**2)
		result = True
		for s in np.arange(step, m, step):
			v = [(goal[0]-start[0])*(s/m), (goal[1]-start[1])*(s/m), (goal[2]-start[2])*(s/m)]
			p = [v[0]+start[0], v[1]+start[1], v[2]+start[2]]

			if self.collision_evaluation(p[0], p[1], p[2], size):
				result = False
				break

		return result

	def RRT(self, Ps, Pe, D, Sig, step, size):

		dist_0 = 9999

		# tree es un array de objetos tipo branch(coor,father,cost)
		tree = []
		# Posicion inicial de tree
		tree.append(branch(Ps,Ps,0))
		# Condiciones iniciales para finalizar el arbol y de collision
		endTree = False
		collision = False

		# vectores auxiliares
		tree_res = []
		x_tree = []
		y_tree = []
		z_tree = []
		coord_list = []
		father_list = []

		# variables que representa cada iteracion para tener una idea
		# de como va el programa
		iter = 0

		while (endTree == False) and (not rospy.is_shutdown()): #implementacion chapucera para que pueda pararse
			Pr = [random.gauss(Pe[0], Sig), random.gauss(Pe[1], Sig), random.gauss(Pe[2], Sig)]
			while tree.count(Pr) != 0:
				Pr = [random.gauss(Pe[0], Sig), random.gauss(Pe[1], Sig), random.gauss(Pe[2], Sig)]

			for point in tree:
				dist_1 = math.sqrt((Pr[0]-point.coord[0])**2 +(Pr[1]-point.coord[1])**2 +(Pr[2]-point.coord[2])**2)
				if dist_1 < dist_0:
					Pi = point.coord
					dist_0 = dist_1

			Vd = [(Pr[0]-Pi[0])*(D/dist_0), (Pr[1]-Pi[1])*(D/dist_0), (Pr[2]-Pi[2])*(D/dist_0)]
			Pd = [Vd[0]+Pi[0], Vd[1]+Pi[1], Vd[2]+Pi[2]]

			if self.collision_evaluation(Pd[0], Pd[1], Pd[2], size):
				collision = True
			else:
				if self.rute_free(Pi, Pd, step, size):
					collision = False
				else: 
					collision = True

			if (not collision) and Pd[2] > 0:
				tree.append(branch(Pd, Pi, dist_0))
				dist_2 = math.sqrt((Pe[0]-Pd[0])**2 +(Pe[1]-Pd[1])**2 +(Pe[2]-Pd[2])**2)
				if dist_2 <= D:
					tree.append(branch(Pe, Pd, dist_2))
					endTree = True
			else:
				collision = False
			
			dist_0 = 9999
			iter = iter + 1
			print("ITERACION NUMERO:", iter) #Se puede poner retorno de carro??

		print("FIN DE LA BUSQUEDA")
		# Hemos terminado de recopilar los puntos en tree
		# ahora hace falta ordenarlos

		for point in tree:
			coord_list.append(point.coord)
			father_list.append(point.father)

		s_point = Pe
		while True:
			tree_res.append(s_point)
			if s_point == Ps:
				break
			s_point = father_list[coord_list.index(s_point)]

		tree_res.reverse()

		for point in tree_res:
			x_tree.append(point[0])
			y_tree.append(point[1])
			z_tree.append(point[2])

		print("ARBOL SIN SUAVIZAR:")
		print(tree_res)

		# tree_res representa el arbol tree con solo los puntos de interes
		# y ya organizado de inicio a final

		############## SUAVIZADO ##############
		
		tree_res_s = []
		tree_res_s.append(tree_res[0])
		suav_point = tree_res[0]
		r = D + 2

		print("SUAVIZANDO:")
		while True and (not rospy.is_shutdown()):
			index = tree_res.index(suav_point)
			for n in np.arange(index+1,len(tree_res)):
				prox_point = tree_res[n]
				if self.rute_free(suav_point, prox_point, step, size) and\
					(math.sqrt((suav_point[0]-prox_point[0])**2 +(suav_point[1]-prox_point[1])**2 +(suav_point[2]-prox_point[2])**2) <= r):
					next_point = prox_point

			tree_res_s.append(next_point)
			suav_point = next_point
			print(index, n)
			if next_point == tree_res[-1]:
					break

		print("ARBOL SUAVIZADO")
		print(tree_res_s)
		print("FIN")
		
		return tree_res_s

	def call_service(self, point):
		try:
			goto_service = rospy.ServiceProxy('/ual/go_to_waypoint',GoToWaypoint)

			waypoint = PoseStamped()		
			waypoint.header.seq = 0 
			waypoint.header.stamp.secs = 0 
			waypoint.header.stamp.nsecs = 0
			waypoint.header.frame_id = ''
			waypoint.pose.position.x = point[0]
			waypoint.pose.position.y = point[1]
			waypoint.pose.position.z = point[2]
			waypoint.pose.orientation.x = 0
			waypoint.pose.orientation.y = 0
			waypoint.pose.orientation.z = 0
			waypoint.pose.orientation.w = 0
		
			goto_service(waypoint, 1)

		except rospy.ServiceException as e:
			print ("SERVICE GOTO CALL FAILED: %s"%e)
			pass

	def rviz_display(self, display_list):

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

		spheres.lifetime = lines.lifetime = rospy.Duration()

		pointList = []
		for element in display_list:
				point = Point()
				point.x = element[0]
				point.y = element[1]
				point.z = element[2]
				pointList.append(point)

		spheres.points = lines.points = pointList

		self.display_publisher.publish(spheres)
		self.display_publisher.publish(lines)

	def goto(self):

		# PARAMETROS
		D = 1.0
		Sig = 3.0 #5.0

		step = 0.5
		size = 1.0
		# END PARAMETROS

		while True:
			data = input("Cargar mapas? (0->Nada) (1->House) (2->Square): ")
			if data == 0:
				print("Mapas ignorados")
				self.map_sel = 0
				break
			if data == 1:
				print("Mapa House cargado")
				self.map_sel = 1
				break
			if data == 2:
				print("Mapa Square cargado")
				self.map_sel = 2
				break
			else:
				print("Input invalido")

		if self.map_sel == 1:
			ob1 = object("HOUSE",[-3.5, 11.5],[-15.5, -3.5],[0, 7.5])
			ob2 = object("TABLE",[2.5, 6.5],[-1.5, 1.5],[0, 2])
			ob3 = object("TREE",[-6.5, 6.5],[3.3, 7.7],[0, 5.5])
			ob4 = object("WALL",[-5.5, -3.5],[-3.5, 5.5],[0, 3])
			ob5 = object("LIGHT 1",[-9.5, -7.5],[-0.5, 1.3],[0, 5])
			ob6 = object("LIGHT 2",[-13.7, -7.5],[-0.5, 1.3],[5, 6.5])

			self.object_list = [ob1, ob2, ob3, ob4, ob5, ob6]

		if self.map_sel == 2:
			ob1 = object("AMBULANCE",[1.5, 10.5],[6.5, 10.5],[0, 3.5])
			ob2 = object("AMB BARRIER",[4.5, 9.5],[5.5, 7.5],[0, 1.5])
			ob3 = object("WALL 1",[-1.5, 0.5],[1.5, 10.5],[0, 3.5])
			ob4 = object("WALL 2",[-8.5, 0.5],[1.5, 3.5],[0, 3.5])
			ob5 = object("TREE",[-5.5, -0.5],[-1.5, 3.5],[0, 6.5])
			ob6 = object("HUMMER",[-9.5, -5.5],[-8.5, -1.5],[0, 3.5])
			ob7 = object("FIREMAN CAR",[-3.5, 10.5],[-10.5, -6.5],[0, 4.5])
			ob8 = object("HUMAN 1",[-3.5, -1.5],[-5.5, -3.5],[0, 2.5])
			ob9 = object("HUMAN 2",[1.5, 3.5],[-4.5, -2.5],[0, 2.5])
			ob10 = object("BOX 1",[3.5, 6.5],[-2.5, -0.5],[0, 3.5])
			ob11 = object("BOX 2",[6.5, 10.5],[-3.5, -1.5],[0, 3.5])
			ob12 = object("BOX 3",[5.5, 11.5],[-0.5, 1.5],[0, 2.5])
			ob13 = object("BOX 4",[8.5, 10.5],[0.5, 5.5],[0, 2.5])
			ob14 = object("BOX 5",[5.5, 8.5],[2.5, 4.5],[0, 4.5])
			ob15 = object("FOUNTAIN",[2.5, 5.5],[2.5, 5.5],[0, 3.5])
			ob16 = object("CONTAINER 1",[-10.5, -7.5],[5.5, 9.5],[0, 2.5])
			ob17 = object("CONTAINER 2",[-6.5, -2.5],[6.5, 9.5],[0, 2.5])

			self.object_list = [ob1, ob2, ob3, ob4, ob5, ob6, ob7, ob8, ob9, ob10, ob11, ob12, ob13, ob14, ob15, ob16, ob17]

		goal_pose = Pose()

		while True:
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
			
			if self.collision_evaluation(goal_pose.position.x, goal_pose.position.y, goal_pose.position.z, size):
				print("Conflicto: Posicion invalida")
			else:
				break

		self.goal_cell = [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z]
		path = []

		end = False
		while (not rospy.is_shutdown()) and (not end):
			path = self.RRT(self.current_cell, self.goal_cell, D, Sig, step, size)
			self.rviz_display(path)
			for waypoint in path:
				if self.rute_free(self.current_cell, waypoint, step, size):
					print("WAYPOINT ACTUAL:", waypoint)
					self.call_service(waypoint)
				else:
					print("RUTA OBSTACULIZADA, BUSCANDO ALTERNATIVA ...")
					break
				if waypoint == path[-1]: end = True
		print("TRAYECTORIA TERMINADA")

if __name__ == '__main__':
	try:
		x = Planner()
		x.goto()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
