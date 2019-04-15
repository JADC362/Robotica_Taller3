#!/usr/bin/env python

#Importacion de librerias
import sys, os, time
import rospy
import numpy as np
import matplotlib.pyplot as plt
import threading
import matplotlib.mlab as mlab
import matplotlib.gridspec as gridspec
import networkx as nx

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

#Matriz de obstaculos, cada fila corresponde a un obstaculo, y cada columna a un dato
#Columna 1: PosX, Columna 2: PosY, Columna 3: Diametros
obstaculos = []

#Grilla del mapa.
grillaMapa = []

#Tamano del mapa 100x100 (metros)
tamanoMapa = 50

#Tamano de grilla 1x1 (metros)
tamanoGrilla = 0.5

#Grafo
G = nx.Graph()

#Variable booleana que mantiene el hilo que grafica activo mientras no se cierre ninguna grafica 
hiloCerrado = False

#Variable que representa al hilo de graficar
hilo=0

#Variable que representa el umbral de error aceptado en rho
errorRho = 0.05

#Variable que representa el umbral de error aceptado en theta
errorMaxTheta = 0.02

#Variable de error que representa el error total de posicion 
errorMax=2*(tamanoGrilla)
#Variable que representa si ya se sobrepaso el umbral de error
umbralSuperado = False
#Vector que contiene los parametros de la rueda 1 del robot: alpha, beta, r, l
paraRueda1 = [np.pi/2,np.pi,97.65/1000,44.5/200]
paraRueda2 = [-np.pi/2,0,97.65/1000,44.5/200]
#Matriz J1 que describe las restricciones de deslizamiento y rodamiento
J1 = np.array([[np.sin(float(paraRueda1[0]+paraRueda1[1])), -np.cos(float(paraRueda1[0]+paraRueda1[1])), -(paraRueda1[3])*np.cos(float(paraRueda1[1]))],
	[np.sin(float(paraRueda2[0]+paraRueda2[1])), -np.cos(float(paraRueda2[0]+paraRueda2[1])), -(paraRueda1[3])*np.cos(float(paraRueda2[1]))],
	[np.cos(float(paraRueda2[0]+paraRueda2[1])), np.sin(float(paraRueda2[0]+paraRueda2[1])), (paraRueda1[3])*np.sin(float(paraRueda2[1]))]])

#Variables que representan la velocidad de cada motor del robot
velocidadM1 = 0
velocidadM2 = 0

#Posicion inicial del robot. Posee un valor por defecto
posicionInicialCar = [0.0,0.0,-np.pi]
#Posicion final del robot. Posee un valor por defecto
posicionFinalCar = [40.0,40.0,np.pi/2]
#Posicion actual del robot. Posee un valor por defecto
posicionActualCar = [0.0,0.0,0.0]

#Variable que representa el tiempo de ejecucion
simulationTimeB = 0 #Tiempo base
simulationTime = 0 #Tiempo actual
simulationTimeAnterior = 0 #Tiempo anterior
pasoDeSimulacion = simulationTime-simulationTimeAnterior #Paso de simulacion utilizado por vrep

#Variables que indican si se actualizo la informacion
callTime = False
callPos = False

#ruta optima
ruta = []

#Variable booleana que indica si ya se calculo la ruta optima
BanderaRutaCalculada=False
 
#Variable que indica a que posicion de la ruta optima debe dirigirse el robot actualmente
contadorPasos=0

#Variable que india si ya se inicializaron  los valores de las posiciones del robot
Inicializar=True

 
#Funcion callback llamada al haber una actuliazacion en el topico obstaclesPosition
def callbackObstaclesPosition(msg):
	global obstaculos
	Datos = msg.data;
	if len(obstaculos) == 0:
		for i in range(int(Datos[0])):
			obstaculos.append([float(Datos[1+i]),float(Datos[int(Datos[0])+1+i]),float(Datos[2*int(Datos[0])+1+i])])


#Funcion callback llamada cuando el topico pionnerPosition es actualizado
#En esta funcion se actualiza la posicion actual del robot simulado y se calcula la posicion teorica del robot
#ademas en caso de que sea la primera vez que se ejecuta la funcion se inicializa la posicion inicial del robot 
def callbackPioneerPosition(msg):
	global posicionActualCar, callPos, Inicializar
	if Inicializar==True:
		posicionInicialCar= [msg.linear.x,msg.linear.y,msg.angular.z]
		Inicializar=False

	posicionActualCar = [msg.linear.x,msg.linear.y,msg.angular.z]
	callPos = True


#Funcion callback llamada cuando el topico simulationTime es actualizado
#La funcion se encarga de actualizar el tiempo actual, el tiempo anterior, el paso de simulacion y el tiempo que lleva la velocidad actual en el robot
def callbackSimulationTime(msg):
	global simulationTimeB, simulationTime, deltaTiempoVelocidades,pasoDeSimulacion, callTime
	if simulationTimeB == 0:
		simulationTimeB = msg.data

	callTime = True
	simulationTimeAnterior=simulationTime
	simulationTime = msg.data
	pasoDeSimulacion = simulationTime-simulationTimeAnterior



#Obtiene la posicion del robot en coordenadas polares rho, alpha, beta
def obtenerPosicionPol(posCar,posicionDeseada):
	global umbralSuperado, errorRho
	rho = np.sqrt((posicionDeseada[0]-posCar[0])**2+(posicionDeseada[1]-posCar[1])**2)
	if rho <= errorRho:
		umbralSuperado = True

	if not umbralSuperado:
		alpha = -posCar[2]+np.arctan2((posicionDeseada[1]-posCar[1]),(posicionDeseada[0]-posCar[0]))
	else:
		alpha = 0;
		rho = 0;

	errorTheta = posicionActualCar[2]-posicionDeseada[2]
	if umbralSuperado and np.absolute(errorTheta)<=errorMaxTheta:
		beta = 0
	else:
		beta = -alpha-posCar[2]

	return np.asarray([rho,alpha,beta])

#Funcion encargada de determinar las velocidades de los motores en cinametica inversa a partir de una ley de control
def calcularCinematicaRobot(posicionDeseada):
	global posicionActualCar, velocidadM1, velocidadM2, paraRueda1, paraRueda2
	cteL = (15/tamanoGrilla)
	cteA = (2/tamanoGrilla)
	k = [0.02*cteL,0.4*cteA,0.01*cteA]

	#Obtencion del error de posicion en coordenadas polares
	posPol = np.asarray([0,0,-posicionDeseada[2]])-obtenerPosicionPol(posicionActualCar,posicionDeseada)
	#rospy.loginfo(obtenerPosicionPol(posicionActualCar,posicionDeseada)) #Se imprime en consola el error

	#Ley de control aplicada para encontrar v y w
	vecVelLinearAngular = np.asarray([k[0]*posPol[0],k[1]*posPol[1]+k[2]*posPol[2]])
	
	#A partir de v y w se determina el vector [x',y',theta']
	veloCar = [(vecVelLinearAngular[0])*(np.cos(posicionActualCar[2])),(vecVelLinearAngular[0])*(np.sin(posicionActualCar[2])),vecVelLinearAngular[1]]

	#Matriz de transformacion del marco global al local
	matrixR = [[np.cos(posicionActualCar[2]),np.sin(posicionActualCar[2]),0],[-np.sin(posicionActualCar[2]),np.cos(posicionActualCar[2]),0],[0,0,1]]
	#Obtencion de las velocidades aplicadas a cada motor. Cinematica aplicada
	velocidadM1 = np.dot([np.sin(paraRueda1[0]+paraRueda1[1]),-np.cos(paraRueda1[0]+paraRueda1[1]),-(paraRueda1[3])*(np.cos(paraRueda1[1]))],np.dot(matrixR,veloCar))/paraRueda1[2]
	velocidadM2 = np.dot([np.sin(paraRueda2[0]+paraRueda2[1]),-np.cos(paraRueda2[0]+paraRueda2[1]),-(paraRueda2[3])*(np.cos(paraRueda2[1]))],np.dot(matrixR,veloCar))/paraRueda2[2]


#Funcion que actualiza la grilla de posicion del mapa
#Esta grilla se forma a partir de la descomposicion de celdas aproximadas
def construirGrillaMapa():
 	global grillaMapa
 	numeroCeldas = int(tamanoMapa/tamanoGrilla)
 	grillaMapa = np.zeros((numeroCeldas,numeroCeldas));

 	for i in range(numeroCeldas):
 		for j in range(numeroCeldas):
 			for k in range(len(obstaculos)):
 				x = (numeroCeldas-1-i)*tamanoGrilla
 				y = (numeroCeldas-1-j)*tamanoGrilla
 				if(determinarObstaculoGrilla(obstaculos[k],[x,y])):
 					grillaMapa[numeroCeldas-1-i][numeroCeldas-1-j]=1		
	

	

#Funcion que determina si un obstaculo [x,y,Diametro] ocupa una grilla [x,y]
def determinarObstaculoGrilla(obstaculo,posicionGrilla):
	l= 1 #Variable de holgura para ajustar los obstaculos y evitar choques
	grillaConObstaculo = False
	distanciaObsGrilla = np.sqrt((posicionGrilla[0]-obstaculo[0])**2+(posicionGrilla[1]-obstaculo[1])**2) 

	if distanciaObsGrilla <= (obstaculo[2]/2.0+l):
		grillaConObstaculo = True

	return grillaConObstaculo

#Funcion que contruye el grafo a partir de la grilla de ocupacion
#Los nodos corresponden al centro de una grilla, y los arcos a las conexines entre ellas
def construirGrafo():	
	global G	
	pesoObstaculo=1000000000
	G = nx.Graph()
 	numeroCeldas = int(tamanoMapa/tamanoGrilla)

	pos = {}
	for i in range(numeroCeldas):
		for j in range(numeroCeldas):
			pos['{},{}'.format(i,j)]= (i,j)

	vector = []
	G.add_nodes_from(pos)

	for m in range(numeroCeldas):
		i=numeroCeldas-1-m
		for n in range(numeroCeldas):
			j=numeroCeldas-1-n
			if (i > 0 and j > 0) and (i < numeroCeldas-1 and j < numeroCeldas-1):

				if grillaMapa[i+1,j]==1:
					pesoI = pesoObstaculo
				else:
					pesoI = tamanoGrilla

				if grillaMapa[i-1,j]==1:
					pesoA = pesoObstaculo
				else:
					pesoA = tamanoGrilla

				if grillaMapa[i,j+1]==1:
					pesoDe = pesoObstaculo
				else:
					pesoDe = tamanoGrilla

				if grillaMapa[i,j-1]==1:
					pesoIz = pesoObstaculo
				else:
					pesoIz = tamanoGrilla

				if grillaMapa[i+1,j+1]==1:
					pesoEsqID = pesoObstaculo
				else:
					pesoEsqID = (2**(0.5))*tamanoGrilla


				if grillaMapa[i-1,j-1]==1:
					pesoEsqSI = pesoObstaculo
				else:
					pesoEsqSI = (2**(0.5))*tamanoGrilla


				if grillaMapa[i-1,j+1]==1:
					pesoEsqSD = pesoObstaculo
				else:
					pesoEsqSD = (2**(0.5))*tamanoGrilla


				if grillaMapa[i+1,j-1]==1:
					pesoEsqII = pesoObstaculo
				else:
					pesoEsqII = (2**(0.5))*tamanoGrilla



				G.add_edge("{},{}".format(i,j),"{},{}".format(i+1,j),weight=pesoI)
				G.add_edge("{},{}".format(i,j),"{},{}".format(i-1,j),weight=pesoA)
				G.add_edge("{},{}".format(i,j),"{},{}".format(i,j+1),weight=pesoDe)
				G.add_edge("{},{}".format(i,j),"{},{}".format(i,j-1),weight=pesoIz)

				G.add_edge("{},{}".format(i,j),"{},{}".format(i+1,j+1),weight=pesoEsqID)
				G.add_edge("{},{}".format(i,j),"{},{}".format(i-1,j-1),weight=pesoEsqSI)
				G.add_edge("{},{}".format(i,j),"{},{}".format(i-1,j+1),weight=pesoEsqSD)
				G.add_edge("{},{}".format(i,j),"{},{}".format(i+1,j-1),weight=pesoEsqII)

#Funcion que grafica la posicion actual del pioneer y la grilla del mapa
def graficar():
	global hiloCerrado
	plt.ion()
	wfig = 8
	hfig = 8
	numeroCeldas = int(tamanoMapa/tamanoGrilla)
	proporcionTitulo = 1.5
	proporcionLabels = 1.5
	fig = plt.figure(figsize=(wfig,hfig))
	gs = gridspec.GridSpec(1, 1)
	gs.update(hspace=10);
	while not hiloCerrado:

		try: 
			x=posicionActualCar[0]
			y=posicionActualCar[1]
			plt.clf()
			ax0 = fig.add_subplot(gs[:, 0])
 			ax0.imshow(np.transpose(grillaMapa), cmap='binary', origin='lower',extent=[0,tamanoMapa,0,tamanoMapa])
			ax0.scatter(x,y, s=5**2, c= 'r', alpha = 0.6 )
			ax0.set_xlabel("Posicion en el eje X del robot con respecto al marco global [m]",fontsize = wfig*proporcionLabels)
			ax0.set_ylabel("Posicion en el eje Y del robot con respecto al marco global [m]", fontsize = wfig*proporcionLabels)
			plt.title("Posicion en tiempo real del robot", fontsize = wfig*proporcionTitulo)
			ax0.grid(True)
			fig.savefig(os.getcwd()+"/src/Taller3/results/GraficaPunto2c.png")
			fig.canvas.draw()
			fig.canvas.flush_events()
			plt.pause(0.01)
			
		except Exception as e:
			plt.close('all')
			hiloCerrado = True
			break
	

	return False

#Funcion que detecta si un punto esta en un obstaculo 
# y en caso de que este aproxima el punto el punto libre(sin obstaculo) mas cercano
# retorna la coordenada x,y del punto en el grafo 
def buscarPosicionMasCercana(x,y):
	xc=0
	yc=0
	distanciaMasCercana = 100000000
	distancia = 0
	for i in range(len(grillaMapa)):
		for j in range(len(grillaMapa)):
			if grillaMapa[i][j]==0:
				distancia =((i-x)**2+(j-y)**2)**(0.5)
				if distancia < distanciaMasCercana:
					distanciaMasCercana=distancia
					xc=i
					yc=j
	return (xc,yc)

#Funcion que calcula la ruta optima al punto final 
#haciendo uso del algoritmo A*
def pathplaning():

	global ruta,contadorPasos
	contadorPasos=0

	xf=int(posicionFinalCar[0]/tamanoGrilla)
	yf=int(posicionFinalCar[1]/tamanoGrilla)

	if(grillaMapa[xf][yf]==1):
		[xf,yf]=buscarPosicionMasCercana(xf,yf) 

	xi=int(posicionInicialCar[0]/tamanoGrilla)
	yi=int(posicionInicialCar[1]/tamanoGrilla)

	nodoInicial = "{},{}".format(xi,yi)
	nodoFinal = "{},{}".format(xf,yf)
	ruta=nx.astar_path(G,nodoInicial,nodoFinal,heuristica)
	return ruta
	
#Funcion que calcula la heuristica entre un nodo de entrada y uno de salida
#utilizando como medida la distancia Manhattan
def heuristica(node1,target):
	D = 3
	[x1,y1]=node1.split(",")
	x1=int(x1)
	y1=int(y1)
	[x2,y2]=target.split(",")
	x2=int(x2)
	y2=int(y2)
	DManhattan=D*(np.absolute(x1-x2)+np.absolute(y1-y2))
	return DManhattan

#Funcion que calcula la posicion x,y,theta a donde se debe mover el robot
def CalcularPosicionDeseada():
	global contadorPasos
	posicionDeseada=[0,0,0]
	[i,j]=ruta[contadorPasos].split(",")
	i=int(i)
	j=int(j)

	x=float(i*tamanoGrilla)
	y=float(j*tamanoGrilla)

	posicionDeseada[0]=x
	posicionDeseada[1]=y

	if contadorPasos==(len(ruta)-1):
		posicionDeseada[2]=posicionFinalCar[2]
	else:
		posicionDeseada[2]=posicionActualCar[2]

	if contadorPasos<(len(ruta)-1):
		contadorPasos+=1
	return posicionDeseada
	
#Funcion principal de codigo
def main():
	global posicionFinalCar, velocidadM1, velocidadM2, callTime, callPos, BanderaRutaCalculada, hiloCerrado
	if len(sys.argv) > 1:
		posicionFinalCar = [float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3])]
	#Iniciacion del nodo antes ROS
	rospy.init_node('P2c_nodoROS',anonymous=False)
	pubMotorsVel = rospy.Publisher('motorsVel',Float32MultiArray,queue_size=10)

	#Suscripcion al topico pioneerPosition
	rospy.Subscriber("pioneerPosition",Twist,callbackPioneerPosition)
	rospy.Subscriber("simulationTime",Float32,callbackSimulationTime)

	#Suscripcion al topico de los obstaculos
	rospy.Subscriber('obstaclesPosition',Float32MultiArray,callbackObstaclesPosition)

	rate = 	rospy.Rate(20)
	try:

		while not rospy.is_shutdown():
			if len(grillaMapa) == 0 and len(obstaculos) != 0:
				construirGrillaMapa();
				construirGrafo();

			if len(grillaMapa)>0 and BanderaRutaCalculada==False:
				pathplaning()
				BanderaRutaCalculada = True
				posicionDeseada=CalcularPosicionDeseada()
				hilo=threading.Thread(target= graficar, name='Graficar')
				hilo.start()
				time.sleep(0.1)

			#Mientras el nodo este activo, se realizara:
			if callTime and callPos:
				error = ((posicionDeseada[0]-posicionActualCar[0])**2+(posicionDeseada[1]-posicionActualCar[1])**2)**(0.5)
				if error<=errorMax:
					posicionDeseada = CalcularPosicionDeseada()
					
				calcularCinematicaRobot(posicionDeseada)
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