#!/usr/bin/env python

#Importacion de librerias
import sys, time
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

#Vector de tiempos de computacion
TiemposCalculo = [[]]

#Vector de magnitud de rutas
rutasMagnitud = [[]]

#Numero indicador del punto final que se esta calculando
iteracionPuntoFinal = 0;

#Vector de posiciones finales para calculo
puntosFinales = [[10,10],[25,25],[40,40],[25,10],[35,40],[45,8],[1,45],[5,5],[24,28]]

#Funcion callback llamada al haber una actuliazacion en el topico obstaclesPosition
def callbackObstaclesPosition(msg):
	global obstaculos,rutasMagnitud,TiemposCalculo,iteracionPuntoFinal

	if len(rutasMagnitud[iteracionPuntoFinal]) < 20 and iteracionPuntoFinal < len(puntosFinales):
		Datos = msg.data;
		for i in range(int(Datos[0])):
			obstaculos.append([float(Datos[1+i]),float(Datos[int(Datos[0])+1+i]),float(Datos[2*int(Datos[0])+1+i])])
		TiemposCalculo[iteracionPuntoFinal].append(time.time())
		construirGrillaMapa();
		construirGrafo([0,0],puntosFinales[iteracionPuntoFinal]);
	else:
		print(iteracionPuntoFinal)
		iteracionPuntoFinal = iteracionPuntoFinal + 1
		rutasMagnitud.append([])
		TiemposCalculo.append([])




#Funcion que actualiza la grilla de posicion del mapa
#Esta grilla se forma a partir de la descomposicion de celdas aproximadas
#Al final se plotea la grilla resultante
def construirGrillaMapa():
 	global grillaMapa
 	grillaMapa = []
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
def construirGrafo(puntoInicial,puntoFinalPar):	
	global Grafo
	Grafo = [];
	G = nx.Graph()
 	iteraciones = 200

 	numeroCeldas = int(tamanoMapa/tamanoGrilla)

 	pos = {}
	for i in range(numeroCeldas):
		for j in range(numeroCeldas):
			pos['{},{}'.format(i,numeroCeldas-1-j)]= (i,j)

	#Se anade el punto inicial
 	G.add_node("{},{}".format(puntoInicial[0],puntoInicial[1]));

 	for i in range(iteraciones):
 		puntoAleatorio = "{},{}".format(int(randint(0,numeroCeldas-5)*tamanoGrilla),int(randint(0,numeroCeldas-5)*tamanoGrilla))
 		qn = nodoMasCercano(G,puntoAleatorio)
 		qs = nodoDetencionObstaculo(qn,puntoAleatorio)
 		qs = "{},{}".format(int(qs[0]),int(qs[1]))
 		G.add_edge(qn,qs)


	for obstaculo in obstaculos:
		if determinarObstaculoGrilla(obstaculo,[int(puntoFinalPar[0]),int(puntoFinalPar[1])]):
			print("Punto final prohibido: "+"({},{})".format(int(puntoFinalPar[0]),int(puntoFinalPar[1]))+" Coincide con un obstaculo. Se dispondra una ruta por defecto.")
			#Posicion final del robot. Posee un valor por defecto
			puntoFinalPar = [40.0,40.0,np.pi/2]
			break

	puntoFinal = "{},{}".format(int(puntoFinalPar[0]),int(puntoFinalPar[1]))
	puntosFinales[iteracionPuntoFinal]=[int(puntoFinalPar[0]),int(puntoFinalPar[1])]
 	qn = nodoMasCercano(G,puntoFinal)
 	G.add_edge(qn,puntoFinal)

	Grafo = G

	calcularRutaCorta(puntosFinales[iteracionPuntoFinal]);

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

def calcularRutaCorta(posicionFinalCar):
	posicionInicial = "{},{}".format(0,0)
	posicionFinal = "{},{}".format(posicionFinalCar[0],posicionFinalCar[1])
	rutaLongitud = nx.shortest_path_length(Grafo,source=posicionInicial,target=posicionFinal)
	rutasMagnitud[iteracionPuntoFinal].append(rutaLongitud)
	TiemposCalculo[iteracionPuntoFinal][-1]=-TiemposCalculo[iteracionPuntoFinal][-1]+time.time()

#Funcion principal de codigo
def main():

	#Obtencion del vector de posicion final del robot. Si no se envia se toma por defecto [40,40,pi/2]
	#if len(sys.argv) > 1:
		#posicionFinalCar = [float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3])]
		
	#Iniciacion del nodo antes ROS
	rospy.init_node('P2g_RRT',anonymous=False)

	#Suscripcion al topico obstaclesPosition
	rospy.Subscriber('obstaclesPosition',Float32MultiArray,callbackObstaclesPosition)

	#Tiempo durante el cual duerme el nodo
	rate = rospy.Rate(20)
	try:

		#Mientras el nodo este activo, se realizara:
		while not rospy.is_shutdown():
			if len(rutasMagnitud) == len(puntosFinales):
				for i in range(len(rutasMagnitud)):

					plt.figure()
					plt.plot(rutasMagnitud[i])
					print("RutaMedia"+str(np.mean(rutasMagnitud[i])))
					print("RutaDesviacion"+str(np.std(rutasMagnitud[i])))
					plt.title("Longitud de las rutas calculadas del grafo a "+str(puntosFinales[i][0])+","+str(puntosFinales[i][1]))
					plt.ylabel("Longitud ruta objetivo")
					plt.xlabel("Iteracion")

					plt.savefig("./src/taller2_12/results/RutaMagnitud{}.pdf".format(i))

					plt.figure()
					plt.plot(TiemposCalculo[i])
					print("TiemposCalculoMedia"+str(np.mean(TiemposCalculo[i])))
					print("TiemposCalculoDesviacion"+str(np.std(TiemposCalculo[i])))
					plt.title("Tiempos de calculo del grafo a "+str(puntosFinales[i][0])+","+str(puntosFinales[i][1]))
					plt.ylabel("Tiempo de calculo [s]")
					plt.xlabel("Iteracion")

					plt.savefig("./src/taller2_12/results/TiemposCalculo{}.pdf".format(i))
				exit()
			rate.sleep()

	except Exception as e:
		raise e

if __name__ == '__main__':
	main()
else:
	rospy.loginfo("Fallo al cargar codigo")