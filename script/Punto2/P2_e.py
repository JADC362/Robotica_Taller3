#!/usr/bin/env python

#Importacion de librerias
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import matplotlib.gridspec as gridspec
import networkx as nx
from random import randint
from threading import Thread

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
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
tamanoGrilla = 1
#Grafo del mapa
Grafo = []
#Ruta del punto inicial al punto final
rutaObjetivo = []
#Paso en el que se va en la serie de nodos de la ruta al punto final
pasoRuta = 0

#Variable que representa el umbral de error aceptado en rho
errorRho = 2;
#Variable que representa si ya se sobrepaso el umbral de error
umbralSuperado = False

#Vector que contiene los parametros de la rueda 1 del robot: alpha, beta, r, l
paraRueda1 = [np.pi/2,np.pi,97.65/1000,44.5/200]
paraRueda2 = [-np.pi/2,0,97.65/1000,44.5/200]

#Matriz J1 que describe las restricciones de deslizamiento y rodamiento
J1 = np.array([[np.sin(paraRueda2[0]+paraRueda1[1]), -np.cos(paraRueda2[0]+paraRueda1[1]), -(paraRueda1[3])*np.cos(paraRueda1[1])],
	[np.sin(paraRueda1[0]+paraRueda2[1]), -np.cos(paraRueda1[0]+paraRueda2[1]), -(paraRueda1[3])*np.cos(paraRueda2[1])],
	[np.cos(paraRueda2[0]+paraRueda2[1]), np.sin(paraRueda2[0]+paraRueda2[1]), (paraRueda1[3])*np.sin(paraRueda2[1])]])

#Variables que representan la velocidad de cada motor del robot
velocidadM1 = 0
velocidadM2 = 0

#Variables que indican si se actualizo la informacion
callTime = False
callPos = False

#Variable que indica si el hilo de graficacion esta cerrado
hiloCerrado = False

#Posicion final del robot. Posee un valor por defecto
posicionFinalCar = [40.0,40.0,np.pi/2]
#Posicion actual del robot. Posee un valor por defecto
posicionActualCar = [0.0,0.0,-np.pi]

#Variable que representa el tiempo de ejecucion
simulationTime = 0 #Tiempo actual

# 
def callbackPioneerPosition(msg):

	global posicionActualCar, callPos
	posicionActualCar = [msg.linear.x,msg.linear.y,msg.angular.z]
	callPos = True

#Funcion callback llamada cuando el topico simulationTime es actualizado
def callbackSimulationTime(msg):
	global simulationTime, callTime
	simulationTime = msg.data
	callTime = True

#Funcion callback llamada al haber una actualizacion en el topico obstaclesPosition
def callbackObstaclesPosition(msg):
	global obstaculos
	Datos = msg.data;
	if len(obstaculos) == 0:
		for i in range(int(Datos[0])):
			obstaculos.append([float(Datos[1+i]),float(Datos[int(Datos[0])+1+i]),float(Datos[2*int(Datos[0])+1+i])])
		construirGrillaMapa();
		construirGrafo();
		calcularRutaCorta();

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


#Funcion que contruye el grafo a partir de la grilla de ocupacion
#Los nodos corresponden al centro de una grilla, y los arcos a las conexines entre ellas
def construirGrafo(puntoInicial=[0,0]):	
	global Grafo, posicionFinalCar
	G = nx.Graph()
 	iteraciones = 300

 	numeroCeldas = int(tamanoMapa/tamanoGrilla)

 	pos = {}
	for i in range(numeroCeldas):
		for j in range(numeroCeldas):
			pos['{},{}'.format(i,numeroCeldas-1-j)]= (i,j)

	#Se anade el punto inicial
 	G.add_node("{},{}".format(0,0));

 	for i in range(iteraciones):
 		puntoAleatorio = "{},{}".format(int(randint(0,numeroCeldas-5)*tamanoGrilla),int(randint(0,numeroCeldas-5)*tamanoGrilla))
 		qn = nodoMasCercano(G,puntoAleatorio)
 		qs = nodoDetencionObstaculo(qn,puntoAleatorio)
 		qs = "{},{}".format(int(qs[0]),int(qs[1]))
 		G.add_edge(qn,qs)

	puntoFinalTemp = [int(posicionFinalCar[0]),int(posicionFinalCar[1])]
	obstaculoPresente = False

	for obstaculo in obstaculos:
		if determinarObstaculoGrilla(obstaculo,puntoFinalTemp):
			obstaculoPresente = True

	if obstaculoPresente:
		print("Punto final prohibido: Coincide con un obstaculo. Se dispondra una ruta por defecto.")

		#Posicion final del robot. Posee un valor por defecto
		posicionFinalCar = [40.0,40.0,np.pi/2]

	puntoFinal = "{},{}".format(int(posicionFinalCar[0]),int(posicionFinalCar[1]))
 	qn = nodoMasCercano(G,puntoFinal)
 	G.add_edge(qn,puntoFinal)

	Grafo = G

	hiloGraficar = Thread(target = hiloGraficarMapa,args=(pos, ))
	hiloGraficar.start()

#Funcion que determina si un obstaculo [x,y,Diametro] ocupa una grilla [x,y]
def determinarObstaculoGrilla(obstaculo,posicionGrilla,tasa=1):
	grillaConObstaculo = False
	distanciaObsGrilla = np.sqrt((posicionGrilla[0]-obstaculo[0])**2+(posicionGrilla[1]-obstaculo[1])**2) 

	if distanciaObsGrilla < obstaculo[2]*tasa/2.0:
		grillaConObstaculo = True

	return grillaConObstaculo

#Retorna el nombre del nodo mas cercano del grafo al punto alteatorio 
def nodoMasCercano(Grafo,puntoAleatorio):
	ordenamientoNodos = list(Grafo.nodes)
	for i in range(len(ordenamientoNodos)-1):
		for j in range(i+1,len(ordenamientoNodos)):
			menor = ordenamientoNodos[i]

			nodoI = menor.split(",")
			nodoJ = ordenamientoNodos[j].split(",")
			punto = puntoAleatorio.split(",")

			distanciaMenor = np.sqrt((int(nodoI[0])-int(punto[0]))**2+(int(nodoI[1])-int(punto[1]))**2)
			distanciaNodoJ = np.sqrt((int(nodoJ[0])-int(punto[0]))**2+(int(nodoJ[1])-int(punto[1]))**2)

			if distanciaMenor > distanciaNodoJ:
				temp = ordenamientoNodos[j]
				ordenamientoNodos[j] = ordenamientoNodos[i]
				ordenamientoNodos[i] = temp

	return ordenamientoNodos[0]

#Retorna el nombre del nodo que, en la proyeccion de qn al punto aleatorio, se acerca mas al punto aleatorio sin chocar con un obstaculo
def nodoDetencionObstaculo(qnArg,puntoAleatorioArg):
	puntoDetencion = [int(qnArg.split(",")[0]),int(qnArg.split(",")[1])]
	puntoAleatorio = [int(puntoAleatorioArg.split(",")[0]),int(puntoAleatorioArg.split(",")[1])]

	if puntoAleatorio[0]-puntoDetencion[0] == 0 and puntoAleatorio[1]-puntoDetencion[1] != 0:
		puntoAleatorio[0]=puntoDetencion[0]+1
	elif puntoAleatorio[1]-puntoDetencion[1] != 0 and puntoAleatorio[1]-puntoDetencion[1] == 0:
		puntoAleatorio[1]=puntoDetencion[1]+1
	else:
		pass

	#Obtencion de recta del punto qn a puntoAleatorio
	try:
		pendienteRecta = float(puntoAleatorio[1]-puntoDetencion[1])/float(puntoAleatorio[0]-puntoDetencion[0])
	except Exception as e:
		pendienteRecta = float(puntoAleatorio[1]-puntoDetencion[1])/float(puntoAleatorio[0]-puntoDetencion[0]+1)
	
	obstaculoHallado = False
	distancia = np.sqrt((puntoDetencion[0]-puntoAleatorio[0])**2+(puntoDetencion[1]-puntoAleatorio[1])**2)

	while abs(puntoAleatorio[0]-puntoDetencion[0]) > tamanoGrilla and not obstaculoHallado:

		x = float(tamanoGrilla)/50.0
			
		if puntoAleatorio[0] < puntoDetencion[0]:
			x = -x

		puntoDetencionTemp = [puntoDetencion[0]+x,puntoDetencion[1]+pendienteRecta*x]

		obstaculoHallado = False
		for obstaculo in obstaculos:
			if determinarObstaculoGrilla(obstaculo,puntoDetencionTemp,1.5):
				obstaculoHallado = True

		if not obstaculoHallado:
			puntoDetencion = puntoDetencionTemp[:]

		distancia = np.sqrt((puntoDetencion[0]-puntoAleatorio[0])**2+(puntoDetencion[1]-puntoAleatorio[1])**2)

	return puntoDetencion

#A partir del grafo, calcula la ruta mas corta del punto inicial al punto final.
#Este retorna la serie de nodos en el arbol para llegar al objetivo
def calcularRutaCorta():
	global rutaObjetivo
	posicionInicial = "{},{}".format(0,0)
	posicionFinal = "{},{}".format(int(posicionFinalCar[0]),int(posicionFinalCar[1]))
	rutaObjetivo = nx.shortest_path(Grafo,source=posicionInicial,target=posicionFinal)
	print(rutaObjetivo)

#Funcion creado por un hilo aparte para el proceso de graficacion de la grilla, el grafo y la posicion del robot
def hiloGraficarMapa(pos):

	global hiloCerrado

	hiloCerrado = False
	plt.ion()
	wfig = 10
	hfig = 8
	proporcionTitulo = 0.9
	proporcionLabels = 0.7
	fig = plt.figure(figsize=(wfig,hfig))
	gs = gridspec.GridSpec(1, 2)
	gs.update(hspace=10);

	while not hiloCerrado:
		try:
			plt.clf()
			nx.draw(Grafo, with_labels=True, font_weight='bold',pos=pos)
		 	plt.imshow(grillaMapa, cmap='jet',origin='upper')
		 	#print("{},{}".format(posicionActualCar[1],posicionActualCar[0]))
		 	plt.scatter(posicionActualCar[0],tamanoMapa-posicionActualCar[1],c='w')

			fig.canvas.draw()
			fig.canvas.flush_events()
			plt.pause(0.01)
		except Exception as e:
			plt.close('all')
			hiloCerrado = True
			break

#Obtiene la posicion del robot en coordenadas polares rho, alpha, beta a partir de la entrada en posicion cartesianas
def obtenerPosicionPol(puntoFinal):
	global umbralSuperado, errorRho, posicionActualCar, pasoRuta
	rho = np.sqrt((puntoFinal[0]-posicionActualCar[0])**2+(puntoFinal[1]-posicionActualCar[1])**2)


	if rho <= errorRho:
		pasoRuta = pasoRuta + 1;
		if pasoRuta == len(rutaObjetivo)-1:
			errorRho = 0.05
			print("De {} a ultimo {}".format(rutaObjetivo[pasoRuta-1],rutaObjetivo[pasoRuta]))
		elif pasoRuta == len(rutaObjetivo):
			umbralSuperado = True
			pasoRuta = pasoRuta - 1;
		else:
			print("De {} a {}".format(rutaObjetivo[pasoRuta-1],rutaObjetivo[pasoRuta]))

	if not umbralSuperado:
		alpha = -posicionActualCar[2]+np.arctan2((puntoFinal[1]-posicionActualCar[1]),(puntoFinal[0]-posicionActualCar[0]))
	else:
		alpha = 0;
		rho = 0;
	beta = -alpha-posicionActualCar[2]

	return np.asarray([rho,alpha,beta])

#Funcion encargada de determinar las velocidades de los motores en cinametica inversa a partir de una ley de control
def calcularCinematicaRobot(puntoFinal):
	global posicionActualCar, velocidadM1, velocidadM2, paraRueda1, paraRueda2

	
	#Obtencion del error de posicion en coordenadas polares
	posPol = np.asarray([0,0,puntoFinal[2]])-obtenerPosicionPol(puntoFinal)
	k = [0.02,0.4,0.01]

	#Ley de control aplicada para encontrar v y w
	vecVelLinearAngular = np.asarray([k[0]*posPol[0],k[1]*posPol[1]+k[2]*posPol[2]])
	
	#A partir de v y w se determina el vector [x',y',theta']
	veloCar = [(vecVelLinearAngular[0])*(np.cos(posicionActualCar[2])),(vecVelLinearAngular[0])*(np.sin(posicionActualCar[2])),vecVelLinearAngular[1]]

	#Matriz de transformacion del marco global al local
	matrixR = [[np.cos(posicionActualCar[2]),np.sin(posicionActualCar[2]),0],[-np.sin(posicionActualCar[2]),np.cos(posicionActualCar[2]),0],[0,0,1]]
	#Obtencion de las velocidades aplicadas a cada motor. Cinematica aplicada
	velocidadM1 = np.dot([np.sin(paraRueda1[0]+paraRueda1[1]),-np.cos(paraRueda1[0]+paraRueda1[1]),-(paraRueda1[3])*(np.cos(paraRueda1[1]))],np.dot(matrixR,veloCar))/paraRueda1[2]
	velocidadM2 = np.dot([np.sin(paraRueda2[0]+paraRueda2[1]),-np.cos(paraRueda2[0]+paraRueda2[1]),-(paraRueda2[3])*(np.cos(paraRueda2[1]))],np.dot(matrixR,veloCar))/paraRueda2[2]

#Funcion principal de codigo
def main():

	global posicionFinalCar, velocidadM1, velocidadM2, callTime, callPos, pasoRuta

	#Obtencion del vector de posicion final del robot. Si no se envia se toma por defecto [40,40,pi/2]
	if len(sys.argv) > 1:
		posicionFinalCar = [float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3])]
		
	#Iniciacion del nodo antes ROS
	rospy.init_node('P2e_RRT',anonymous=False)

	#Publicacion al topico de velocidades para los motores del robot pioneer
	pubMotorsVel = rospy.Publisher('motorsVel',Float32MultiArray,queue_size=10)

	#Suscripcion al topico pioneerPosition
	rospy.Subscriber("pioneerPosition",Twist,callbackPioneerPosition)
	#Suscripcion al topico simulationTime
	rospy.Subscriber("simulationTime",Float32,callbackSimulationTime)
	#Suscripcion al topico obstaclesPosition
	rospy.Subscriber('obstaclesPosition',Float32MultiArray,callbackObstaclesPosition)

	#Tiempo durante el cual duerme el nodo
	rate = rospy.Rate(20)
	pasoRuta = 0;
	try:
		#Mientras el nodo este activo, se realizara:
		while not rospy.is_shutdown():

			if callTime and callPos and len(rutaObjetivo) != 0:

				posicionFinal = [0,0]
				if pasoRuta == len(rutaObjetivo)-1:
					posicionFinal = [float(rutaObjetivo[pasoRuta].split(",")[0]),float(rutaObjetivo[pasoRuta].split(",")[1]),posicionFinalCar[2]]
				else:
					posicionFinal = [float(rutaObjetivo[pasoRuta].split(",")[0]),float(rutaObjetivo[pasoRuta].split(",")[1]),0.0]

				calcularCinematicaRobot(posicionFinal)
				callTime = False
				callPos = False

			#Publica la velocidad del robot en el topico motorsVel
			mensaje = Float32MultiArray(data=[velocidadM1,velocidadM2])
			pubMotorsVel.publish(mensaje)

			#Se envia a dormir al nodo
			rate.sleep()
	except Exception as e:
		raise e

if __name__ == '__main__':
	main()
else:
	rospy.loginfo("Fallo al cargar codigo")