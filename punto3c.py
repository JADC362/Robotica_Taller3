#!/usr/bin/env python
import rospy 
from std_msgs.msg import Int32
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from pynput.keyboard import Key, Listener
import threading
import time
from Tkinter import *
import Tkinter
import numpy as np
import random as rdm
import math
#--------------------------------Declaracion de todas las  variables--------------------------------

mat= Float32MultiArray()#matriz donde se almacenan los datos de la velocidad
velocidad=0 #vcelocidad global del robot
linear=Vector3() #vector donde se almacena la posicion lineal global
angular=Vector3() #vector donde se almacena la posicion angular global
vectorAcumuladoX=[]#vector donde se almacena todo el recorrido de el robot en X
vectorAcumuladoY=[]#vector donde se almacena todo el recorrido de el robot en Y
grupos=[]
estado=0
xPuntos=[]
yPuntos=[]
xCentroides = []
yCentroides = []
ecuaciones = []
SUMA=1
RESTA=2
xant=0
yant=0 
cambiox=0
cambioy=0
robotAngle=0
x=-2.54492855072 #posicion en x del robot
y=0.35003682971 #posicion en y del robot
giro=0
arctan=-180
puntos= []#matriz donde se almacenan los puntos del sensor
xPuntosCumulate= []
yPuntosCumulate= []

#----------------------------Declaracion de funciones-----------------------------------------------
def definirLinea(puntosx, puntosy):
	sumax=0
	sumay=0
	sumaxy=0
	sumax2=0 
	ecuacion=[]
	for i in range(len(puntosx)):
		xy = puntosx[i]*puntosy[i]	 
		x2 = puntosx[i]
		sumax += puntosx[i]
		sumay += puntosy[i]
		sumaxy += xy
		sumax2 += x2
	if len(puntosx)>0:	
		m = (sumaxy-((sumax*sumay)/len(puntosx)))/(sumax2-((sumax2**2)/len(puntosx))) 	
		b = (sumay/len(puntosx))-((m*sumax)/len(puntosx))
		ecuacion.append(m)
		ecuacion.append(b)
	return ecuacion


def kmeans (puntosx, puntosy, k):
	#global centroides
	tamaniopuntos = len(puntosx)
	randomnumber = rdm.randrange(tamaniopuntos)
	randomnumber2= rdm.randrange(tamaniopuntos)	
	gruposF	 = []
	vacio= []
	grupos = []
	centroidesAnt=[]
	centroides=[]
	continua=True
	for i in range(k):
		randomnumber= rdm.randrange(len(puntosx))
		centroides.append([puntosx[randomnumber], puntosy[randomnumber]])

	while continua:
		grupos = []		
		for i in range(k):
			grupos.append([])	
		for i in range(tamaniopuntos):			
			mindistance = 500
			numeroGrupo = 0
			for j in range (k):								
				distanciaE=(math.sqrt(((puntosx[i]-centroides[j][0])**2)+((puntosy[i]-centroides[j][1])**2)))
				
				if distanciaE<mindistance:
					mindistance = distanciaE
					numeroGrupo = j 			
			grupos[numeroGrupo].append(puntosx[i])
			grupos[numeroGrupo].append(puntosy[i])
									
		
		if centroides==centroidesAnt:
			continua=False
			gruposF=grupos
			#print(centroides,centroidesAnt)
		else: 
			centroidesAnt=centroides
			#grupos=vacio
			
			for l in range(k):
				h=0
				newcentx=0
				newcenty=0
				s=len(grupos[l])/2
				while h<len(grupos[l]):
					newcentx += grupos[l][h]/s
					newcenty += grupos[l][h+1]/s
					h += 2
				centroides[l][0]=newcentx
				centroides[l][1]=newcenty
				
	#print(grupos)				
	return grupos

def Ransac():
	global xPuntos, yPuntos, xPuntosCumulate, yPuntosCumulate
	betterModel = [] #mejor modelo
	 #numero de iteraciones
	thresholdDist = 0.0021
	puntosInt = []
	
	k=2
	grupos = kmeans(xPuntosCumulate, yPuntosCumulate, k)  
	gruposInt = []
	
	for i in range(k):
		if len(grupos[i])>=2:
			gruposInt.append(grupos[i])
			betterModel.append([])

	for i in range (len(gruposInt)):
		actual = gruposInt[i] 
		xActual = []
		yActual = []
		puntosInt.append([])
		
		n=len(xPuntos)**2

		h=0
		while h<len(actual):
			xActual.append(gruposInt[i][h])
			yActual.append(gruposInt[i][h+1])
			h+=2 
		iteraciones=0			
		while iteraciones<n:
			inliers = []
			tamaniopuntos = len(xActual)
			randomnumber = rdm.randrange(tamaniopuntos)
			randomnumber2= rdm.randrange(tamaniopuntos)
			mPosibleModelo = (yActual[randomnumber2]-yActual[randomnumber])/(xActual[randomnumber2]-xActual[randomnumber])
			bPosibleModelo = yActual[randomnumber]-(mPosibleModelo*xActual[randomnumber])
			A = -mPosibleModelo
			B =	1
			C = (-yActual[randomnumber])+(mPosibleModelo*xActual[randomnumber])
			for j in range(len(xActual)):
				distancia= np.absolute((A*xActual[j])+(B*yActual[j])+C)/math.sqrt((A**2)+(B**2))
				if distancia<thresholdDist:
					inliers.append(xActual[j])
					inliers.append(yActual[j])
			if 	len(betterModel[i])<=len(inliers):
				betterModel[i] = inliers

			iteraciones+=1
				
	return betterModel			

def lineas():
	global ecuaciones
	puntos= Ransac()
	
	puntosX=[]
	puntosY=[]
	l=0
	ecuaciones= []
	while l<len(puntos):
		puntosX.append([])
		puntosY.append([])
		
		i=0
		while i in range (len(puntos[l])):	
			puntosX[l].append(puntos[l][i])
			puntosY[l].append(puntos[l][i+1])
			i+=2
		ecuaciones.append(definirLinea(puntosX[l],puntosY[l]))	
		l+=1
	print(ecuaciones)		
	return ecuaciones	




def callbackScanner(msg):
	global puntos, xPuntos, yPuntos, x, y, xPuntosCumulate, yPuntosCumulate
	puntos=msg.data
	distancia=[]
	dst=0
	p=0
	i=0
	xPuntos=[]
	yPuntos=[]
	if estado==0:
		while i<len(puntos):
			p=msg.data[i]
			dst=msg.data[i+1]
			xp=x+(np.cos((robotAngle*(np.pi/180))+p)*dst)
			yp=y+(np.sin((robotAngle*(np.pi/180))+p)*dst)
			xPuntos.append(xp)
			yPuntos.append(yp)
			xPuntosCumulate.append(xp)
			yPuntosCumulate.append(yp)
			i += 2

def callbackPioneerPosition(msg): #funcion encargada de obtener la posicion del robot
	global linear, angular, vectorAcumuladoX, vectorAcumuladoY, x, y, xant, yant, cambiox, cambioy, robotAngle, arctan
	xant=x
	yant=y
	linear=msg.linear
	angular=msg.angular
	x = linear.x 
	y = linear.y 
	z = linear.z 
	vectorAcumuladoX.append(x)
	vectorAcumuladoY.append(y)
	cambioxAnt = cambiox
	cambioyAnt = cambioy
	cambiox = x-xant
	cambioy = y-yant
	deltax=0
	deltay=0

	
	if np.absolute(cambiox)>=0.0001:
		deltax = cambiox-cambioxAnt
	if np.absolute(cambioy)>=0.0001:
		deltay = cambioy-cambioyAnt
	if np.absolute(deltax)>=0.0001 or np.absolute(deltay)>=0.0001:
		arctan=(180/np.pi)*np.arctan(cambioy/cambiox)
		if cambiox<0 and cambioy<0:
			robotAngle=(arctan-90)-90
		elif cambiox<0 and cambioy>0:
			robotAngle=(arctan+90)+90
		else:
			robotAngle=arctan	
def funcion(x,ecuaciones):

	f=(-ecuaciones[0]*80)*(x-1.5)+ecuaciones[1]
	return f
	
def ventanaDialogo(): #funcion encargada de lanzar la ventana de dialogo donde se introduce la velocidad
	global velocidad, xPuntos, yPuntos, grupos, xCentroides, yCentroides, xPuntosCumulate, yPuntosCumulate
	k=2
	raiz = Tk()
	raiz.title("Entrada de velocidad")	
	raiz.resizable(1,1)

	entrada=StringVar()

	
	Entrada=""


	frame1=Frame()
	frame1.pack()
	cuadroTexto=Entry(frame1, textvariable=entrada)
	cuadroTexto.grid(row=1, column=2, padx=10, pady=10)
	cuadroTexto.config(justify="right")
	nombreLabel= Label(frame1,text="velocidad:")
	nombreLabel.grid(row=1,column=1, padx=10, pady=10)

	botonEnviar =  Button(frame1, text="enviar velocidad", command=lambda:recibir())
	botonEnviar.grid(row=2, column=1)
	botonKmeans = Button(frame1, text="hacer Ransac", command=lambda:RansacV())
	botonKmeans.grid(row=2, column=2)	
	
	def kmeansV():
		centroides = kmeans(xPuntosCumulate, yPuntosCumulate, k)
		
		#print(centroides)
		

		for i in range(k):
			
			xCentroides.append(centroides[i][0])
			yCentroides.append(centroides[i][1])
			#print(xCentroides)
		# print(yCentroides)
		#print(funcion(1.4))
	def RansacV():
		ecuaciones= lineas()						
	def recibir():
		velocidad=int(entrada.get())
		entrada.set("")
		mat.data=[velocidad, velocidad]	
	
	raiz.mainloop()
			
def on_press(key): #Funcion al detectar una tecla presionada
	
	global velocidad, giro, estado
	try:		
		if key == key.up:
			mat.data=[mat.data[0],mat.data[0]*2]
			giro=RESTA
			estado= 1
		elif key == key.down:
			mat.data=[mat.data[1]*2,mat.data[1]]
			giro=SUMA
			estado= 1
	except Exception as e:
		rospy.loginfo("Presione las teclas arriba y abajo para girar")
def on_release(key): #Funcion al soltar una tecla
	global estado 
	try:
		if key == Key.esc: #Con ESC finaliza este thread
			return False		
		if key == key.up:
			mat.data=[mat.data[0], mat.data[0]]	
			estado=1	
		if key == key.down:
			mat.data=[mat.data[1], mat.data[1]]
			estado=1

	except Exception as e:
		rospy.loginfo("Presione las teclas arriba y abajo para girar")
def graficarCamino():
	ax1.scatter(xPuntos, yPuntos, c='b', edgecolors='b')
	fig.canvas.draw()
	plt.savefig("Ejemplo1.jpg")		

def ThreadInputs(): #Listener de Pynput en otro thread
		with Listener(on_press=on_press, on_release=on_release) as listener:
			listener.join()
def main():
	global iniciar, velocidad, hiloCerrado, x, y, estado, xCentroides, yCentroides, ecuaciones#variables globales
	#ejecucion del hilo donde se crea la ventana	
	hiloVentana=threading.Thread(target=ventanaDialogo)
	hiloVentana.daemon = True
	hiloVentana.start()
	#declaracion de las subscriptions y ublishers del nodo
	rospy.init_node('control', anonymous=True)
	pub1 = rospy.Publisher('motorsVel', Float32MultiArray, queue_size=10)
	pub2 = rospy.Publisher('pauseSimulation', Bool ,queue_size=10)
	pub3 = rospy.Publisher('startSimulation', Bool ,queue_size=10)
	rospy.Subscriber("scanner",Float32MultiArray,callbackScanner)
	rospy.Subscriber("pioneerPosition",Twist,callbackPioneerPosition)
	
	rate = rospy.Rate(10)
	#inicializacion de las datos de la informacion que se le enviara al robot.
	mat.data=[0,0]
	mat.layout.dim.append(MultiArrayDimension())
	mat.layout.dim.append(MultiArrayDimension())
	mat.layout.dim[0].label = "leftMotor"
	mat.layout.dim[1].label = "rigthMotor"
	iniciar=True#inicia la simulacion
	#creacion y abertura de la imagen donde se plotea la posicion global del robot
	fig = plt.figure()
	plt.xlabel('xPos')
	plt.ylabel('yPos')
	plt.title('posicionGlobal')
	ax1 = fig.add_subplot(111)
	plt.grid()
	fig.show()
	#lanzamiendo del hilo encargado del teclado
	hiloTeclado=threading.Thread(target=ThreadInputs)
	hiloTeclado.daemon=True
	hiloTeclado.start()
	xecuaciones=[]
	i=1.3
	while i<1.7:
		if i == 1.3:
			xecuaciones.append(i)
			i+=0.01
		else:
			i+=0.01	
			xecuaciones.append(i)



	def graficar():
		
		j=0
		while j<len(ecuaciones):			
			yecuaciones=[]
			
			for p in range(len(xecuaciones)):

				yecuaciones.append(funcion(xecuaciones[p],ecuaciones[j]))
			
			ax1.scatter(xecuaciones, np.array(yecuaciones).tolist(), c='g', edgecolors='g')	
			j+=1		
		
		ax1.scatter(xPuntos, yPuntos, c='r', edgecolors='r')
		#print(xCentroides)
		#print(yCentroides)
		ax1.scatter(x, y, c='b', edgecolors='b')
		fig.canvas.draw()
		plt.savefig("Ejemplo1.jpg")
		




	
	while not rospy.is_shutdown():
			time.sleep(0.01)
			if estado < 2 and estado != 0: 
				estado += 1
			else:
				estado = 0	
			pub1.publish(mat)#publicacion de las velocidades
			pub3.publish(iniciar)#enviar la orden de iniciar la simlacion		
			rate.sleep()
			if mat.data[0]==mat.data[1]:
				graficar()
						

if __name__ == '__main__':	
	main()
