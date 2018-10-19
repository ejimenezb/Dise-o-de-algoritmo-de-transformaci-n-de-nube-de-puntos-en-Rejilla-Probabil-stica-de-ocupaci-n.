#include "ros/ros.h"
#include <string.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Header.h"
#include <iostream>
#include <vector>
#include <cstdlib>
#include <sstream>
#include <math.h>

#define PI 3.14159274101

using namespace ros;

// using namespace std;

void laserScancallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  sensor_msgs::LaserScan data = *msg; // La funcion callback de ROS guarda la data cruda en la memoria, por lo tanto para extraerla hay que apuntar a la direccion de memoria.
  double angle_rad = data.angle_min; //angulo minimo en radianes del velodyne (desde donde parte el movimiento de escaneo)
  double angle_degrees = 0.0, dist_x_m = 0.0, dist_y_m = 0.0, distancia = 0.0;
  double angle_rad_v[data.ranges.size()], distancia_v[data.ranges.size()], coord_x_v[data.ranges.size()], coord_y_v[data.ranges.size()];
  float resolution = 0.01;

  /*
   *La estructura de datos esta comprendida por los siguientes 12 valores, 
   *divididos entre los tipos string, uint y float.
  
   *uint32 seq
   *time stamp     <------- Segundos, concatenados con nanosegundos 
   *string frame_id

   *float32 angle_min
   *float32 angle_max
   *float32 angle_increment
   *float32 time_increment
   *float32 scan_time
   *float32 range_min
   *float32 range_max
   *float32[] ranges
   *float32[] intensities
   */

  ROS_INFO(" \n\n I'm receiving from LaserScan: \n\n seq: %d \n stamp: %d \n frame_id: %s \n angle_min: %f \n angle_max: %f \n angle_increment: %f \n time_increment: %f \n scan_time: %f \n range_min: %f \n range_max: %f \n ", 
                                                                                       data.header.seq, data.header.stamp, data.header.frame_id.c_str(), data.angle_min, 
                                                                                       data.angle_max, data.angle_increment, data.time_increment, data.scan_time,
                                                                                       data.range_min, data.range_max);
  /*
  ROS_INFO("ranges: \n");

  for(int i = 0; i < data.ranges.size(); i++)  
  {

    std::cout << data.ranges[i] << ", " ; 

  }

  ROS_INFO("intensities: \n");

  for(int i = 0; i < data.intensities.size(); i++)
  {
    std::cout << data.intensities[j] << ", " ;

  }
 
  ROS_INFO("angles: \n");

  for(int i = 0; i < data.ranges.size(); i++)
  {
    angle_rad = angle_rad + data.angle_increment;
    angle_degrees = angle_rad*180/PI;
    std::cout << angle_degrees << ", " ;

  }
  angle_rad = data.angle_min;
  std::cout <<"\n" ;
  //*/

  //ROS_INFO("angles 2: \n");

  for(int i = 0; i < data.ranges.size(); i++)  
  {

    angle_rad = angle_rad + data.angle_increment;
    distancia = data.ranges[i];
    dist_x_m = distancia * cos(angle_rad);
    dist_y_m = distancia * sin(angle_rad);

    angle_rad_v[i] = angle_rad;
    distancia_v[i] = distancia;
    coord_x_v[i] = dist_x_m/resolution;
    coord_y_v[i] = dist_y_m/resolution;

    angle_degrees = angle_rad_v[i]*180/PI;

    std::cout << angle_degrees << ", " ; 
  }
  //std::cout <<"\n" ;




}

int main(int argc, char **argv)//--------------------------------------------
{
  init(argc, argv, "LaserScan_Listener"); // Funcion de ROS que inicializa el nodo que vamos a lanzar

  NodeHandle n; // Declaracion del nodo a utilizar para la comunicacion

  Subscriber sub = n.subscribe("scan", 1000, laserScancallback);

  Rate loop_rate(1);

  spin();

  return 0;
}   