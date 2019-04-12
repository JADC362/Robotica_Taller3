#!/usr/bin/env python
import rospy, random , threading, time, os

import Queue as queue
import time
import sys


#import Queue


#pacman
from pacman.msg import pacmanPos
from pacman.msg import ghostsPos
from pacman.msg import cookiesPos

from pacman.msg import game
from pacman.msg import performance
from pacman.srv import mapService
from pacman.msg import actions

#var
#nPacmanM = 0
pacmanPosMx = 0
pacmanPosMy = 0
nCookiesM = 1
cookiesPosM = 0


global eatenCookies
eatenCookies = 0
pac_pos_Y = 0
pac_pos_X = 0
global IND
IND = []
global unaVez
unaVez = True
global accion
accion = 7
global optimo
optimo = []

def pacmanPosCallback(msg):
    global nPacmanM, pacmanPosMx, pacmanPosMy
    nPacmanM = msg.nPacman
    pacmanPosMx = msg.pacmanPos.x
    pacmanPosMy = msg.pacmanPos.y

def cookiesPosCallback(msg):
    global nCookiesM, cookiesPosM
    nCookiesM = msg.nCookies
    cookiesPosM = msg.cookiesPos

def mov_valido(pac_map,moves):
    global pac_pos_X
    global pac_pos_Y

    j = pac_pos_X
    i = pac_pos_Y
    #contador = 0

    for m in moves:
        #contador = contador+1
        if m =="L":

            j = j-1
        if m =="R":
            j = j+1
        if m =="U":
            i = i-1
        if m =="D":
            i = i+1

    #print(contador)
    if not(0<=j < len(pac_map[0]) and 0 <= i < len(pac_map)):
		return False

    elif (pac_map[i][j]=='#'):
		return False
    return True

def notVisited(pacman_IND,moves):
    global pac_pos_X
    global pac_pos_Y
    global IND

    j = pac_pos_X
    i = pac_pos_Y
    #contador = 0

    for m in moves:
        #contador = contador+1
        if m =="L":

            j = j-1
        if m =="R":
            j = j+1
        if m =="U":
            i = i-1
        if m =="D":
            i = i+1

    if (pacman_IND[i][j]==1):
        return False, pacman_IND
    else: #(pacman_IND[i][j]==0):
        pacman_IND[i][j]=1
        IND[i][j]=1
        return True, pacman_IND



def finalizo(pac_map,moves):
    global pac_pos_X
    global pac_pos_Y
    global optimo
    global unaVez

    j = pac_pos_X
    i = pac_pos_Y
    #contador = 0

    for m in moves:
        #contador = contador+1
        if m =="L":

            j = j-1
        if m =="R":
            j = j+1
        if m =="U":
            i = i-1
        if m =="D":
            i = i+1

    #print(contador)
    if pac_map[i][j]=="c":
        print("Encontrado")
        print(moves)
        optimo = moves
        unaVez = True
        return True

    return False


def pacman_maper_py():
    ingr = []
    global nPacmanM, pacmanPosMx, pacmanPosMy, optimo, IND, unaVez, pac_pos_Y,pac_pos_X, nPacmanM1, pacmanPosMx1, pacmanPosMy1, nCookiesM, cookiesPosM, eatenCookies
    rospy.init_node('controller_py', anonymous=True)
    rospy.Subscriber('pacmanCoord0', pacmanPos, pacmanPosCallback)
    rospy.Subscriber('cookiesCoord', cookiesPos, cookiesPosCallback)
    #Creacion del publicador
    pub = rospy.Publisher('pacmanActions0', actions, queue_size=10)

    try:
        if(nCookiesM == 0):
            print("FIN DE JUEGO")
            sys.exit()

            
        rate = rospy.Rate(1/0.15)  # 1/(150ms) Hz
        msg = actions()
        mapRequestClient = rospy.ServiceProxy('pacman_world', mapService)
        mapa = mapRequestClient("Santi")
        #matriz
        A=[[' ' for x in range(1 + mapa.maxX - mapa.minX)]for y in range (1 + mapa.maxY - mapa.minY)]
        #obstacles
        for o in range(mapa.nObs):
            A[-mapa.obs[o].y - mapa.minY][mapa.obs[o].x - mapa.minX] = '#'
        #plot matrix 1
        #for i in range(1 + mapa.maxY - mapa.minY):
        #    print(A[i])

        time.sleep(6) #Se da un tiempo de gracia de 6 seg pues la matriz de juego cambia entre
        while not rospy.is_shutdown():
            if(nCookiesM == 0):
                print("FIN DE JUEGO")
                sys.exit()

            print('INICIO while not shut down')

            A=[[' ' for x in range(1 + mapa.maxX - mapa.minX)]for y in range (1 + mapa.maxY - mapa.minY)]
            #obstacles
            for o in range(mapa.nObs):
                A[-mapa.obs[o].y - mapa.minY][mapa.obs[o].x - mapa.minX] = '#'
            #pacman
            A[-pacmanPosMy - mapa.minY][pacmanPosMx - mapa.minX] = 'P' #Pacman
            if (nPacmanM == 2):
                A[-pacmanPosMy1 - mapa.minY][pacmanPosMx1 - mapa.minX] = 'p' #MsPacman
            #cookies
            for c in range(nCookiesM):
                A[-cookiesPosM[c].y - mapa.minY][cookiesPosM[c].x - mapa.minX] = 'c'


            #plot matrix inf
            #for i in range(1 + mapa.maxY - mapa.minY):
            #    print(A[i])
			#Definicion numero de filas y columnas para crear matrices de apoyo
            filas = len(A)
            columnas = len(A[1])
            #Se crea la matriz de indices de tamano filasxcolumnas
            print("Una vez(1):",unaVez)
            if unaVez:
                IND = [[0 for x in range(columnas)] for y in range(filas)]
                unaVez = False
                print("Entro")

            q = queue.Queue()
            pac_pos_Y =  -pacmanPosMy - mapa.minY
            pac_pos_X = pacmanPosMx - mapa.minX
            #line division
            print('')
            q.put("")
            nuevo = ""
            ingr = []

            t0_cookie = time.time()



            while not finalizo(A,nuevo) and nCookiesM !=0:
                nuevo = q.get()
                for j in ["L","R","U","D"]:
                    ingr = nuevo + j
                    #print(ingr)
                    #print("la lista es",list(q.queue))
                    if mov_valido(A,ingr):
                        K =notVisited(IND,ingr)
                        IND = K[1]
                        if K[0]:
                            q.put(ingr)

            tf_cookie = time.time()-t0_cookie
            print("camino optimo es:",optimo)
            print("El tiempo en encontrarlo fue:",tf_cookie)
            comandos = [0 for i in range(len(optimo))]
            cont = 0
            for j in optimo:
                if j == "U":
                    comandos[cont]=0
                if j == "D":
                    comandos[cont]=1
                if j == "R":
                    comandos[cont]=2
                if j == "L":
                    comandos[cont]=3
                cont = cont +1
            #print(comandos)

            print("Conteo Mov")
            t0_mov = time.time()
            for i in comandos:
                msg.action = i
                pub.publish(msg.action)
                time.sleep(0.15)
                msg.action = 4
                pub.publish(msg.action)

            msg.action = 4
            pub.publish(msg.action)
            tf_map = time.time()-t0_mov
            print("El tiempo en recorrer el mapa fue:",tf_map)

            rate.sleep()
    except rospy.ServiceException as e:
        print("Error!! Make sure pacman_world node is running ")
if __name__ == '__main__':
    try:
        pacman_maper_py()
    except rospy.ROSInterruptException:
        pass
