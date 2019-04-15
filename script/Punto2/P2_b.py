#!/usr/bin/env python

#Importacion de librerias
import rospy
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

#Matriz de obstaculos, cada fila corresponde a un obstaculo, y cada columna a un dato
#Columna 1: PosX, Columna 2: PosY, Columna 3: Diametros
obstaculos = []

#Grilla del mapa.
grillaMapa = []

#Tamano del mapa 100x100 (metros)
tamanoMapa = 50

#Tamano de grilla 1x1 (metros)
tamanoGrilla = 0.5
 
#Funcion callback llamada al haber una actuliazacion en el topico obstaclesPosition
def callbackObstaclesPosition(msg):
	global obstaculos
	Datos = msg.data;
	if len(obstaculos) == 0:
		for i in range(int(Datos[0])):
			obstaculos.append([float(Datos[1+i]),float(Datos[int(Datos[0])+1+i]),float(Datos[2*int(Datos[0])+1+i])])


#def determinarObstaculoGrillaAdaptada(obstaculo,grilla):
#	grillaConObstaculo = False

#	distanciaX = ((grilla[0]+grilla[2]/2.0)-obstaculo[0])
#	distanciaY = ((grilla[1]+grilla[3]/2.0)-obstaculo[1])
#	if distanciaX < ((grilla[2]+obstaculo[2])/2.0) and distanciaY < ((grilla[3]+obstaculo[2])/2.0)
#		grillaConObstaculo = True

#	return grillaConObstaculo

#Funcion que actualiza la grilla de posicion del mapa
#Esta grilla se forma a partir de la descomposicion de celdas aproximadas
#Al final se plotea la grilla resultante
def construirGrillaMapa():
 	global grillaMapa
 	numeroCeldas = int(tamanoMapa/tamanoGrilla)
 	grillaMapa = np.zeros((numeroCeldas,numeroCeldas));

 	for i in range(numeroCeldas):
 		for j in range(numeroCeldas):
 			for k in range(len(obstaculos)):
 				x = i*tamanoGrilla
 				y = j*tamanoGrilla
 				if(determinarObstaculoGrilla(obstaculos[k],[x,y])):
 					grillaMapa[numeroCeldas-1-j][i]=1		
	
	plt.figure()
 	plt.imshow(grillaMapa, cmap='hot')
	plt.show()	

	

#Funcion que determina si un obstaculo [x,y,Diametro] ocupa una grilla [x,y]
def determinarObstaculoGrilla(obstaculo,posicionGrilla):
	grillaConObstaculo = False
	distanciaObsGrilla = np.sqrt((posicionGrilla[0]-obstaculo[0])**2+(posicionGrilla[1]-obstaculo[1])**2) 

	if distanciaObsGrilla < obstaculo[2]/2.0:
		grillaConObstaculo = True

	return grillaConObstaculo

#Funcion que contruye el grafo a partir de la grilla de ocupacion
#Los nodos corresponden al centro de una grilla, y los arcos a las conexines entre ellas
def construirGrafo():		
	G = nx.Graph()
 	numeroCeldas = int(tamanoMapa/tamanoGrilla)

	pos = {}
	for i in range(numeroCeldas):
		for j in range(numeroCeldas):
			pos['{},{}'.format(i,j)]= (i,j)

	vector = []
	G.add_nodes_from(pos)

	for i in range(numeroCeldas):
		for j in range(numeroCeldas):
			if (i > 0 and j > 0) and (i < numeroCeldas-1 and j < numeroCeldas-1):
				G.add_edge("{},{}".format(i,j),"{},{}".format(i+1,j))
				G.add_edge("{},{}".format(i,j),"{},{}".format(i-1,j))
				G.add_edge("{},{}".format(i,j),"{},{}".format(i,j+1))
				G.add_edge("{},{}".format(i,j),"{},{}".format(i,j-1))

				G.add_edge("{},{}".format(i,j),"{},{}".format(i+1,j+1))
				G.add_edge("{},{}".format(i,j),"{},{}".format(i-1,j-1))
				G.add_edge("{},{}".format(i,j),"{},{}".format(i-1,j+1))
				G.add_edge("{},{}".format(i,j),"{},{}".format(i+1,j-1))


	nx.draw(G, with_labels=True, font_weight='bold',pos=pos)
	plt.show()


#Funcion principal de codigo
def main():

	#Iniciacion del nodo antes ROS
	rospy.init_node('P2b_TopicosROS',anonymous=False)

	#Suscripcion al topico de los obstaculos
	rospy.Subscriber('obstaclesPosition',Float32MultiArray,callbackObstaclesPosition)


	try:
		#Mientras el nodo este activo, se realizara:
		while not rospy.is_shutdown():
			if len(grillaMapa) == 0 and len(obstaculos) != 0:
				construirGrillaMapa();
				construirGrafo();
	except Exception as e:
		raise e

if __name__ == '__main__':
	main()
else:
	rospy.loginfo("Fallo al cargar codigo")