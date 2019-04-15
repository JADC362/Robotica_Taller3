#Codigo P2_c

CONTEXTO

Código que se encarga de crear una grilla del mapa, calcular una ruta utilizando A* y realizar una ley de control para llevar al robot a lo largo de la ruta más corta dentro del grafo calculado.

DEPENDENCIAS

 - sys
 - rospy
 - numpy
 - matplotlib
 - networkx
 - threading

INSTALACION LIBRERIAS EXTERNAS

 - pip install numpy
 - pip install matplotlib
 - pip install networkx

PERMISOS

 - Abrir terminal, y dirijirse a la posición del código P2_c.py
 - Dar permisos como: chmod +x P2_c.py

EJECUCION

 - Abrir terminal
 - Correr: roscore
 - Abrir terminal y ejecutar /vrep.sh en la carpeta de este archivo
 - Abrir terminal
 - Correr: rosrun taller3_12 P2_c.py [posicionFinalX] [posicionFinalY] [posicionFinalAngulo]
