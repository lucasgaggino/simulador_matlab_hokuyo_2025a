Simulador MATLAB del curso Robotica Movil, FIUBA 2025.

El script robot_diferencial_con_lidar.m tiene el simulador de robot diferencial.
Es compatible con MATLAB 2016b o posterior. No es compatible con versiones anteriores a R2016b.

El script debe usarse como template para desarrollar el trabajo ya que luego cambiando el valor de use_roomba a True puede usarse con el robot real.

Tener en cuenta que nuevas versiones de este simulador pueden llegar a ser publicadas por lo que se recomienda el uso modular que permita actualizar el simulador facilmente.

IM


ffmpeg -framerate 30 -i "frame_%04d.png" -vf "scale=876:656" -c:v libx264 -pix_fmt yuv420p output.mp4