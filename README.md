# Control de robots móviles con marcadores ArUco

*Read in [English](READMEen.md)*

En este repositorio se encuentra el escrito y presentación de mi proyecto, los scripts en Python y los sketch para Arduino que desarrollé durante mi proyecto de Estadia en la Universidad Politécnica de Atlacomulco (UPA).

Estos permiten controlar una multitud de robots usando marcadores AruCo sobre ellos para identificarlos mediante una cámara.

El script principal **robotFormation.py** crea una ventana donde puedes seleccionar a un robot con clic izquierdo y con el clic derecho, moverlo a cualquier otra parte de la ventana.
Antes de poder utilizar este script, es necesario crear un archivo con la matriz de transformacion de perspectiva mediante el script **charucoPerspectiveCalibrate.py**, esto requiere de una tabla de calibración ChArUco puesta a la altura de los marcadores de los robots.