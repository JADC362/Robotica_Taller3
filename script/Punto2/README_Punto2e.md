#Codigo P2_e

CONTEXTO

Código que se encarga de crear una grilla del mapa, crear un arbol a partir de RRT y realizar una ley de control para llevar al robot a lo largo de la ruta más corta dentro del grafo calculado.

DEPENDENCIAS

 - sys
 - rospy
 - numpy
 - matplotlib
 - networkx
 - random
 - threading

INSTALACION LIBRERIAS EXTERNAS

 - pip install numpy
 - pip install matplotlib
 - pip install networkx

PERMISOS

 - Abrir terminal, y dirijirse a la posición del código P2_e.py
 - Dar permisos como: chmod +x P2_e.py

EJECUCION

 - Abrir terminal
 - Correr: roscore
 - Abrir terminal y ejecutar /vrep.sh en la carpeta de este archivo
 - Abrir terminal
 - Correr: rosrun taller3_12 P2_e.py [posicionFinalX] [posicionFinalY] [posicionFinalAngulo]
