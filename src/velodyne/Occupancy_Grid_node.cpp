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

int assign_points(uint width, uint height, int fila, int columna);
nav_msgs::OccupancyGrid generate_Grid_Map(nav_msgs::OccupancyGrid gridMap, std::vector<signed char> dataProb);
void laserScancallback(const sensor_msgs::LaserScan::ConstPtr& msg);
//-------------------------------------------------------------

ros::Publisher map_pub;
ros::Subscriber sub;

nav_msgs::OccupancyGrid gridMap;

class Point{

private:

double angle_rad, angle_degrees, distancia, x_m, y_m, x, y, probability;

public:

Point();
Point(double angle, double angle_degree, double dist, double dist_x_m, double dist_y_m, double coord_x, double coord_y);
~Point();

void setAngle_rad(double angle);
void setAngle_degree(double angle_degree);
void setDistancia(double dist);
void setDistancia_x_m(double dist_x_m);
void setDistancia_y_m(double dist_y_m);
void setCoord(double coord_x, double coord_y);
void setPoint(double angle, double angle_degree, double dist, double dist_x_m, double dist_y_m, double coord_x, double coord_y);
void setProbability(double prob);

double getAngle_rad();
double getAngle_degree();
double getDistancia();
double getDistancia_x_m();
double getDistancia_y_m();
double getX();
double getY();
double getProbability();

};

Point::Point(){
  angle_rad = 0.0;
  angle_degrees = 0.0;
  distancia = 0.0;
  x_m = 0.0;
  y_m = 0.0;
  x = 0.0;
  y = 0.0;
}

Point::Point(double angle, double angle_degree, double dist, double dist_x_m, double dist_y_m, double coord_x, double coord_y){
  angle_rad = angle;
  angle_degrees = angle_degree;
  distancia = dist;
  x_m = dist_x_m;
  y_m = dist_y_m;
  x = coord_x;
  y = coord_y;
}

Point::~Point(){
  //Borra objeto

}

void Point::setAngle_rad(double angle){
  angle_rad = angle;
}

void Point::setAngle_degree(double angle_degree){
  angle_degrees = angle_degree;
}

void Point::setDistancia(double dist){
  distancia = dist;
}

void Point::setDistancia_x_m(double dist_x_m){
  x_m = dist_x_m;
}

void Point::setDistancia_y_m(double dist_y_m){
  y_m = dist_y_m;
}

void Point::setCoord(double coord_x, double coord_y){
  x = coord_x;
  y = coord_y;
}

void Point::setPoint(double angle, double angle_degree, double dist, double dist_x_m, double dist_y_m, double coord_x, double coord_y){
  Point::setAngle_rad(angle);
  Point::setAngle_degree(angle_degree);
  Point::setDistancia(dist);
  Point::setDistancia_x_m(dist_x_m);
  Point::setDistancia_y_m(dist_y_m);
  Point::setCoord(coord_x,coord_y);
}

void Point::setProbability(double prob){
  probability = prob;
}

double Point::getAngle_rad(){
  return angle_rad;
}

double Point::getAngle_degree(){
  return angle_degrees;
}

double Point::getDistancia(){
  return distancia;
}

double Point::getDistancia_x_m(){
  return x_m;
}

double Point::getDistancia_y_m(){
  return y_m;
}

double Point::getX(){
  return x;
}

double Point::getY(){
  return y;
}

double Point::getProbability(){
  return probability;
}

void laserScancallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  sensor_msgs::LaserScan data = *msg; // La funcion callback de ROS guarda la data cruda en la memoria, por lo tanto para extraerla hay que apuntar a la direccion de memoria.
  nav_msgs::OccupancyGrid gridMap_pub;
  double angle_rad = data.angle_min; //angulo minimo en radianes del velodyne (desde donde parte el movimiento de escaneo)
  double angle_degrees = 0.0, dist_x_m = 0.0, dist_y_m = 0.0;
  int gridProb[gridMap.info.height][gridMap.info.width];
  int posX = 0, posY = 0, pos = 0, len = gridMap.info.height*gridMap.info.width, coord_x = 0, coord_y = 0;
  std::vector<signed char> dataProb;
  dataProb.assign(len,50);
  Point point_v[data.ranges.size()];
  Point point;

  //ROS_INFO("longitud = %d \n",len); 
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
  
  /*
  ROS_INFO(" \n\n I'm receiving from LaserScan: \n\n seq: %d \n stamp: %d \n frame_id: %s \n angle_min: %f \n angle_max: %f \n angle_increment: %f \n time_increment: %f \n scan_time: %f \n range_min: %f \n range_max: %f \n ", 
                                                                                       data.header.seq, data.header.stamp, data.header.frame_id.c_str(), data.angle_min, 
                                                                                       data.angle_max, data.angle_increment, data.time_increment, data.scan_time,
                                                                                       data.range_min, data.range_max);
  */

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

    dist_x_m = data.ranges[i] * cos(angle_rad);
    dist_y_m = data.ranges[i] * sin(angle_rad);

    coord_x = dist_x_m/gridMap.info.resolution;
    coord_y = dist_y_m/gridMap.info.resolution;

    angle_degrees = angle_rad*180/PI;

    point.setPoint(angle_rad, angle_degrees, data.ranges[i], dist_x_m, dist_y_m, coord_x, coord_y);
    point_v[i] = point;

    //Analisis del cuadrante
    if (abs(coord_x) <= gridMap.info.width/2 && abs(coord_y) <= gridMap.info.height/2)
    {
      if (coord_x >= 0){//Cuadrante 1 o 4
        if(coord_y >= 0){//Primer Cuadrante 
          /*
          ROS_INFO("Cuadrante 1 \n");
          ROS_INFO("Coord_X = %d \n",coord_x);
          ROS_INFO("Coord_Y = %d \n",coord_y);
          */
          posX = gridMap.info.height/2 - coord_y + 1;
          posY = gridMap.info.width/2 + coord_x + 1; 
          /*
          ROS_INFO("posX = (%d)/2 - %d + 1 = %d \n",gridMap.info.height, coord_y, posX);
          ROS_INFO("posY = (%d)/2 + %d + 1 = %d \n",gridMap.info.width, coord_x, posY);
          */
        }else { //Cuarto cuadrante
          /*
          ROS_INFO("Cuadrante 4 \n");
          ROS_INFO("Coord_X = %d \n",coord_x);
          ROS_INFO("Coord_Y = %d \n",coord_y);
          //*/
          //posX = gridMap.info.height - (gridMap.info.height/2 - coord_y) + 1;
          posX = (gridMap.info.height/2 - coord_y);// + 1;
          posY = gridMap.info.width/2 + coord_x + 1;
          /*
          ROS_INFO("posX = ((%d)/2 - %d) = %d \n",gridMap.info.height, coord_y, posX);
          ROS_INFO("posY = (%d)/2 + %d + 1 = %d \n",gridMap.info.width, coord_x, posY);
          //*/
        }
      }else if(coord_y >= 0){//Cuadrante 2 o 3
        //Segundo Cuadrante
          /*
          ROS_INFO("Cuadrante 2 \n");
          ROS_INFO("Coord_X = %d \n",coord_x);
          ROS_INFO("Coord_Y = %d \n",coord_y);
          //*/
          posX = gridMap.info.height/2 - coord_y + 1; //estatatta
          posY = gridMap.info.width/2 + coord_x ;//+ 1;
          /*
          ROS_INFO("posX = (%d)/2 - %d + 1 = %d \n",gridMap.info.height, coord_y, posX);
          ROS_INFO("posY = (%d)/2 + %d + 1 = %d \n",gridMap.info.width, coord_x, posY);
          //*/
      }else{//Tercer Cuadrante
          /*
          ROS_INFO("Cuadrante 3 \n");
          ROS_INFO("Coord_X = %d \n",coord_x);
          ROS_INFO("Coord_Y = %d \n",coord_y);
          //*/
          //posX = gridMap.info.height - (gridMap.info.height/2 - coord_y) + 1;
          posX = (gridMap.info.height/2 - coord_y);// + 1;
          posY = gridMap.info.width/2 + coord_x ;//+ 1;
          /*
          ROS_INFO("posX = ((%d)/2 - %d) = %d \n",gridMap.info.height, coord_y, posX);
          ROS_INFO("posY = (%d)/2 + %d + 1 = %d \n",gridMap.info.width, coord_x, posY);
          //*/
      }
      //Si es impar nos redondea hacia abajo por lo que debemos sumar una casilla
      //if(gridMap.info.height % 2 != 0) posX = posX + 1; 
      if(gridMap.info.width % 2 != 0) posY = posY + 1;

      gridProb[posX-1][posY-1] = 100;
      pos = assign_points(gridMap.info.width, gridMap.info.height, posX, posY);
      dataProb.at(pos-1) = 100;

    }

    //CORREGIR!! NO DEBEN HABER VALORES MAYOR A 100!!!!!!!! MIENTRAS UN PARCHE
    //VOLVER A ESTUDIAR TODOS LOS CASOS

    //pos = assign_points(gridMap.info.width, gridMap.info.height, posX, posY);

    //gridProb[posX-1][posY-1] = 100;
    /*
    ROS_INFO("Posicion = %d \n",pos); 
    ROS_INFO("posX = %d \n",posX);
    ROS_INFO("posY = %d \n",posY);  
    */
    //dataProb.at(pos) = 100;
    //if (posX <= gridMap.info.height && posY <= gridMap.info.width) dataProb.at(pos) = 100;
    


    //std::cout << angle_degrees << ", " ; 
  }
  //Crear objeto para manipular los atributos y hacer los calculos
  //Ejecutar occupancy grid aqui en callback



  gridMap_pub = generate_Grid_Map(gridMap, dataProb);
  //PUBLICAR
  map_pub.publish(gridMap_pub);

  //std::cout <<"\n" ;

}

//-------------------------------------------------------------

int assign_points(uint width, uint height, int fila, int columna)
{
  //int position = width * (fila-1) + columna; //Posicion en el vector desde arriba a la izquierda en el 2do cuadrante
  int position = width * (height - fila) + columna;//Posicion en el vector desde abajo a la izquierda en el 3er cuadrante
      
  ROS_INFO("position = (%d)*(%d - %d) + %d = %d\n", width, height, fila, columna, position); 
  return position;

}

//-------------------------------------------------------------

nav_msgs::OccupancyGrid generate_Grid_Map(nav_msgs::OccupancyGrid gridMap, std::vector<signed char> dataProb)
{
  //ROS_INFO("frame_id = %s \n",gridMap.header.frame_id.c_str()); 
  //ROS_INFO("generate_Grid_Map \n");
  /*
   *int p[]={0, 0, 100, 100};
   *std::vector<signed char>a(p,p+4);
   *gridMap.data = a;*/
  /*
  int length = gridMap.info.width*gridMap.info.height;
  int v[length];
  int fin_intervalo_menos_1 = 101;//probabilidad del 0-100
  int count = 0;
  for(int i = 0; i<length; i++) 
  { 
    v[i] =rand() % fin_intervalo_menos_1;
    //v[i] = count++;
    //if (count > 99){
      //count = 0;
    //}
  }


  std::vector<signed char>data(v,v+length);//Asignar luego del llenado del arreglo
  */

  gridMap.data = dataProb;
  
  return gridMap;
}


int main(int argc, char *argv[])
{
 
  ros::init(argc, argv, "Occupancy_Grid");

  ROS_INFO("Nodo Occupancy_Grid \n");
  
  gridMap.info.origin.position.x = 0;//ver como se asigna un point
  gridMap.info.origin.position.y = 0;
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
            gridMap.info.origin.position.x = gridMap.info.width*gridMap.info.resolution/2;
            gridMap.info.origin.position.x = -gridMap.info.origin.position.x;
            ROS_INFO("width = %u \n", gridMap.info.width);
          
        }else if(arg == "--h"){
            
            std::stringstream convert(argv[i+1]);
            if (!(convert >> gridMap.info.height)) gridMap.info.height = 0; 
            gridMap.info.origin.position.y = gridMap.info.height*gridMap.info.resolution/2;
            gridMap.info.origin.position.y = -gridMap.info.origin.position.y;
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