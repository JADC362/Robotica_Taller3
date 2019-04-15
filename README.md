# taller3_12
CONTEXTO

Este es un repositorio creado a manera de contener la solución implementada a la tarea 3 del programa ROBÓTICA de la Universidad de los Andes. Esta se enfoca en la creación de un paquete de ROS que solucione 3 puntos diferentes. Estos 3 puntos se dirigen en la solución de retos respecto a la creación de un mapa, de un grafo y de las rutas más optimas a partir de diferentes métodos vistos en clase.

DESCRIPCIÓN

Esta es un nodo que implementa soluciones a retos planteados al planeamiento de rutas.
Especificamente, el primer punto consiste en la implementación de metodos de planeación de rutas sobre un nodo que ejecuta un juego de pacman (https://github.com/carlosquinterop/ros-pacman). El segundo punto consiste en la implementación de estos métodos en el programa VREP (http://www.coppeliarobotics.com/), haciendo uso de leyes de control para mover al robot Pioneer. Por ultimo, se implementan métodos de creación y detección de mapas en el punto 3, sin movilizar al robot, pero teniedno este un sensor LED.

REQUERIMIENTOS DE SISTEMA

	- Ubuntu 16.04 64 bits
	- ROS Kinetic
	- Qt5 Development Libraries
	- OpenGL
	- V-REP PRO EDU

VERSION

	- Rosdistro: kinetic
	- Rosversion: 1.12.14
	- taller3_12: 1.0.0
	- V-rep V: 3.6
	- Ros-Pacman: 0.0.1-0
	
VERSION LIBRERIAS PYTHON

	- rospy: 1.12.14
	- pynput: 1.4
  	- numpy: 1.15.1
	- matplotlib: 2.2.3
 	- networkX: 2.2
	 
LIBRERIAS PYTHON BASE
	
	- time
	- os
	- sys
	- threading
	- random
  
INSTALACIÓN

	1) Instalar primero ROS, siguiendo el tutorial alojado en la pagina http://wiki.ros.org/kinetic/Installation/Ubuntu y crear un workspace
	2) Descargar V-Rep Pro Edu de la pagina http://www.coppeliarobotics.com/downloads.html y alojar este en una carpeta como /Documentos
  	3) Descargar el paquete ros-pacman del siguiente repositorio https://github.com/carlosquinterop/ros-pacman al workspace.
	4) Descargar el paquete taller3_12 del repositorio actual (https://github.com/JADC362/taller3_12) y almacenarlo en el workspace de ROS. 
				
COMPILACIÓN

	- cd ~/catkin_ws (o dirijirse al workspace creado)
	- source devel/setup.bash
	- catkin_make
PERMISOS

	Cada código creado debe darsele la opción de ejecutarse. Para este se implementa el siguiente codigo:
	- cd ~/catkin_ws/src/taller3_12/scripts/
	  - cd Punto1
		- chmod +x *.py 
	  - cd ../Punto2
		- chmod +x *.py 
	  - cd ../Punto3
		- chmod +x *.py 

EJECUCIÓN

	1) Para ejecutar el algoritmo de BFS se debe ejecutar en la terminal el comando roscore para activar el core de ROS.
	En seguida se ejecuta uno de los mapas con galletas y sin fantasmas haciendo uso del comando:
	- rosrun pacman pacman_world --c [opcion de mapa]. 
	Para ejecutar este comando el usuario se debe encontrar en la carpeta catkin_ws y debe haber ejecutado previamente el comando source devel/setup.bash

	Finalmente, una vez ejecutado y a la vista el mapa deseado, se ejecuta el código desde la terminal:
	- rosrun taller3_12 T3_Punto1.py 

	Una vez realizada esta acción, verá como el pacman se mueve de manera automatica buscando el camino más corto entre galletas, mientras en la terminal se imprime el tiempo que le tomó al algoritmo encontrar el camino más corto entre cada galleta y el tiempo que le tomó a Pacman desplazarse a dicha galleta. 
	
	2) Para el punto 2, lo primero es ejecutar el entorno de ros y luego ejecutar vrep en la escena dispuesta en este repositorio (resources/Punto2/Escena.ttt)
	- Abrir una nueva terminal
	- Correr: roscore
	- Abrir una nueva terminal
  	- Dirigirse al directorio de vrep y correr el programa como: ./vrep.sh y disponer en el escena 
  	- Abrir una nueva terminal
	- cd catkin_ws
	- source devel/setup.bash
	- Ejecutar los diferentes puntos:
		1) rosrun taller3_12 P2_b.py
		2) rosrun taller3_12 P2_c.py
		3) rosrun taller3_12 P2_e.py [posicionFinalX] [posicionFinalY] [posicionFinalAngulo]
		4) rosrun taller3_12 P2_g.py
	
CREADORES

	- John Alejandro Duarte Carraco
	- Jonathan Steven Roncancio Pinzon
	- Santiago Devia Valderrama
	- Miguel Angel Mozo Reyes
