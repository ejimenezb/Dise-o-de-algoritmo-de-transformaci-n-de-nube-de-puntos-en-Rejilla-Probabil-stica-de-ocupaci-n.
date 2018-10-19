#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

#include "std_msgs/String.h"
#include <std_msgs/Int8.h>

#include <vector>
#include <math.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sstream> // for std::stringstream

#define PI 3.14159274101

nav_msgs::OccupancyGrid generate_Grid_Map(nav_msgs::OccupancyGrid gridMap);
void laserScancallback(const sensor_msgs::LaserScan::ConstPtr& msg);
//-------------------------------------------------------------

ros::Publisher map_pub;
ros::Subscriber sub;

nav_msgs::OccupancyGrid gridMap;

class Point{

private:

double angle_rad, distancia, x, y;

public:

Point();
Point(double angle, double dist, double coord_x, double coord_y);

void setAngle_rad(double angle);
void setDistancia(double dist);
void setCoord(double coord_x, double coord_y);
void setPoint(double angle, double dist, double coord_x, double coord_y);

double getAngle_rad();
double getDistancia();
double getX();
double getY();

};

Point::Point(){
  angle_rad = 0.0;
  distancia = 0.0;
  x = 0.0;
  y = 0.0;
}

Point::Point(double angle, double dist, double coord_x, double coord_y){
  angle_rad = angle;
  distancia = dist;
  x = coord_x;
  y = coord_y;
}

void Point::setAngle_rad(double angle){
  angle_rad = angle;
}

void Point::setDistancia(double dist){
  distancia = dist;
}

void Point::setCoord(double coord_x, double coord_y){
  x = coord_x;
  y = coord_y;
}

void Point::setPoint(double angle, double dist, double coord_x, double coord_y){
  Point::setAngle_rad(angle);
  Point::setDistancia(dist);
  Point::setCoord(coord_x,coord_y);
}

double Point::getAngle_rad(){
  return angle_rad;
}

double Point::getDistancia(){
  return distancia;
}

double Point::getX(){
  return x;
}

double Point::getY(){
  return y;
}

void laserScancallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  sensor_msgs::LaserScan data = *msg; // La funcion callback de ROS guarda la data cruda en la memoria, por lo tanto para extraerla hay que apuntar a la direccion de memoria.
  nav_msgs::OccupancyGrid gridMap_pub;
  double angle_rad = data.angle_min; //angulo minimo en radianes del velodyne (desde donde parte el movimiento de escaneo)
  double angle_degrees = 0.0, dist_x_m = 0.0, dist_y_m = 0.0, distancia = 0.0;
  double angle_rad_v[data.ranges.size()], distancia_v[data.ranges.size()], coord_x_v[data.ranges.size()], coord_y_v[data.ranges.size()];
  Point point[data.ranges.size()];
  //float resolution = 0.01; //gridMap.info.resolution = 0.01;


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
    coord_x_v[i] = dist_x_m/gridMap.info.resolution;
    coord_y_v[i] = dist_y_m/gridMap.info.resolution;

    angle_degrees = angle_rad_v[i]*180/PI;

    //std::cout << angle_degrees << ", " ; 
  }
  //Crear objeto para manipular los atributos y hacer los calculos
  //Ejecutar occupancy grid aqui en callback

  gridMap_pub = generate_Grid_Map(gridMap);
  //PUBLICAR
  map_pub.publish(gridMap_pub);

  //std::cout <<"\n" ;

}

//-------------------------------------------------------------

nav_msgs::OccupancyGrid generate_Grid_Map(nav_msgs::OccupancyGrid gridMap)
{

  //ROS_INFO("generate_Grid_Map \n");
  /*
   *int p[]={0, 0, 100, 100};
   *std::vector<signed char>a(p,p+4);
   *gridMap.data = a;*/
  int length = gridMap.info.width*gridMap.info.height;
  int v[length];
  int fin_intervalo_menos_1 = 101;//probabilidad del 0-100
  
  for(int i = 0; i<length; i++) v[i] = rand() % fin_intervalo_menos_1;

  std::vector<signed char>data(v,v+length);//Asignar luego del llenado del arreglo
  gridMap.data = data;
  
  return gridMap;
}


int main(int argc, char *argv[])
{
 
  ros::init(argc, argv, "Occupancy_Grid");

  ROS_INFO("Nodo Occupancy_Grid \n");
  
  gridMap.info.origin.position.x = -5;//ver como se asigna un point
  gridMap.info.origin.position.y = -5;
  gridMap.info.origin.position.z = 0;
  gridMap.info.resolution = 0.01; // resolution is in meters
  gridMap.info.width = 10;
  gridMap.info.height = 10;
  gridMap.header.frame_id = "map";

  std::string arg;

  for (int i = 1; i<argc; i++){
    
    arg = argv[i];
    
    if (arg == "-h") {
      
      std::cout << "Este nodo genera la rejilla probabilistica de ocupacion \n";
      std::cout << "  Parametros: \n";
      std::cout << "\t--r\t'resolution' \n";
      std::cout << "\t--w\t'width' \n";
      std::cout << "\t--h\t'height' \n";
      std::cout << "\t--f\t'frame_id' \n";
      std::cout << "\n \n";
      std::cout << "Por default se genera una rejilla con:\n\n";
      std::cout << "\tresolution\t= 0.01 \n";
      std::cout << "\twidth\t\t= 10 \n";
      std::cout << "\theight\t\t= 10 \n";
      std::cout << "\tframe_id\t= map \n";
      std::cout << "\n\n";
      exit(1);
    
    }else if(argc > (i+1)){ 

        //ROS_INFO("Cadena de Argumentos = %d, iteracion = %d \n",argc,i); 
        
        if (arg == "--r"){

          std::stringstream convert(argv[i+1]);
          if (!(convert >> gridMap.info.resolution)) gridMap.info.resolution = 0;
            ROS_INFO("resolution = %f \n",gridMap.info.resolution);
          
        }else if(arg == "--w"){

            std::stringstream convert(argv[i+1]);
            if (!(convert >> gridMap.info.width)) gridMap.info.width = 0;
            ROS_INFO("width = %u \n", gridMap.info.width);
          
        }else if(arg == "--h"){
            
            std::stringstream convert(argv[i+1]);
            if (!(convert >> gridMap.info.height)) gridMap.info.height = 0; 
            ROS_INFO("height = %u \n", gridMap.info.height); 
        
        }else if(arg == "--f"){
            gridMap.header.frame_id = argv[i+1];
            ROS_INFO("frame_id = %s \n",argv[i+1]); 

        }

      i++;
      } else{
        ROS_INFO("Debe ingresar el valor del argumento \n");
        exit(1);
      }
  }

  ros::NodeHandle n;

  //PUBLICAR
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("map",10);

  //SUBSCRIBIRSE
  sub = n.subscribe("scan", 1000, laserScancallback);

  ros::Rate loop_rate(10.0);

  
  int count = 0;
  while (ros::ok())
  {

    /*//String
     *std_msgs::String msg;

     *std::stringstream ss;
     *ss << "Comunicando " << count;
     *msg.data = ss.str();

     *ROS_INFO("%s", msg.data.c_str());
     */

    /*//Int
     *std_msgs::Int8 msg;
     *msg.data = count;
     *ROS_INFO("%d", msg.data);
     */

    //ROS_INFO("Iteracion Num: [%d] \n",count);
    //gridMap = generate_Grid_Map(gridMap);

    //map_pub.publish(gridMap);
    
    ros::spinOnce(); // Aqui se genera la magia

    loop_rate.sleep();
    //++count;
     
  }

  return 0;
}