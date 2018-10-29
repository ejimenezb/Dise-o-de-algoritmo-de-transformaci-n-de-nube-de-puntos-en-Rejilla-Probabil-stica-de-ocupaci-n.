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


ros::Publisher map_pub;
ros::Subscriber sub;

nav_msgs::OccupancyGrid gridMap;

int assign_points(uint width, uint height, int fila, int columna);
void bresenhamLine(int x0, int y0, int x1, int y1, std::vector<int8_t> &dataProb);
void bresenhamLine_wiki(int x0, int y0, int x1, int y1, std::vector<int8_t> &dataProb);
nav_msgs::OccupancyGrid generate_Grid_Map(nav_msgs::OccupancyGrid gridMap, std::vector<int8_t> dataProb);
void laserScancallback(const sensor_msgs::LaserScan::ConstPtr& msg);
//-------------------------------------------------------------


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
  int posX = 0, posY = 0, pos = 0, len = gridMap.info.height*gridMap.info.width, coord_x = 0, coord_y = 0;
  //std::vector<signed char> dataProb;
  std::vector<int8_t> dataProb;
  dataProb.assign(len,50);
  Point point_v[data.ranges.size()];
  Point point;

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
    *ROS_INFO("ranges: \n");

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
  */


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
    if (abs(coord_x) < gridMap.info.width/2 && abs(coord_y) < gridMap.info.height/2)
    {
      int center_x = gridMap.info.height/2;
      int center_y = gridMap.info.width/2;

      /*
      if (coord_x >= 0){//Cuadrante 1 o 4
        if(coord_y >= 0){//Primer Cuadrante 

          posX = gridMap.info.height/2 - coord_y + 1;
          posY = gridMap.info.width/2 + coord_x;

        }else { //Cuarto cuadrante

          posX = (gridMap.info.height/2 - coord_y);
          posY = gridMap.info.width/2 + coord_x;

        }
      }else if(coord_y >= 0){//Cuadrante 2 o 3
        //Segundo Cuadrante

          posX = gridMap.info.height/2 - coord_y + 1;
          posY = gridMap.info.width/2 + coord_x ;

      }else{//Tercer Cuadrante

          posX = (gridMap.info.height/2 - coord_y);
          posY = gridMap.info.width/2 + coord_x ;

      }
      //*/

      posX = center_x + coord_x;
      posY = center_y + coord_y;

      //bresenhamLine(center_x, center_y, posX, posY, dataProb);
      bresenhamLine_wiki(center_x, center_y, posX, posY, dataProb);

      //bresenhamLine_wiki(center_x, center_y, 75, 75, dataProb);

            //Cuadrado interno de prueba 
      /*
      bresenhamLine_wiki(center_x, center_y, 25, 25, dataProb);
      bresenhamLine_wiki(center_x, center_y, 25, 30, dataProb);
      bresenhamLine_wiki(center_x, center_y, 25, 50, dataProb);
      bresenhamLine_wiki(center_x, center_y, 25, 70, dataProb);
      bresenhamLine_wiki(center_x, center_y, 25, 75, dataProb);
      bresenhamLine_wiki(center_x, center_y, 30, 75, dataProb);
      bresenhamLine_wiki(center_x, center_y, 50, 75, dataProb);
      bresenhamLine_wiki(center_x, center_y, 70, 75, dataProb);
      bresenhamLine_wiki(center_x, center_y, 75, 75, dataProb);
      bresenhamLine_wiki(center_x, center_y, 75, 70, dataProb);
      bresenhamLine_wiki(center_x, center_y, 75, 50, dataProb);
      bresenhamLine_wiki(center_x, center_y, 75, 30, dataProb);
      bresenhamLine_wiki(center_x, center_y, 75, 25, dataProb);
      bresenhamLine_wiki(center_x, center_y, 70, 25, dataProb);
      bresenhamLine_wiki(center_x, center_y, 50, 25, dataProb);
      bresenhamLine_wiki(center_x, center_y, 30, 25, dataProb);
      //*/
      
      pos = assign_points(gridMap.info.width, gridMap.info.height, posX, posY);
      dataProb.at(pos-1) = 100;
    }


  }

  gridMap_pub = generate_Grid_Map(gridMap, dataProb);
  map_pub.publish(gridMap_pub);

}

//-------------------------------------------------------------

int assign_points(uint width, uint height, int columna, int fila)
{
  int position = width * (fila) + columna;
  //int position = width * (height - fila) + columna;//Posicion en el vector desde abajo a la izquierda en el 3er cuadrante  
  //std::cout<<"position = "<<position<<"\n";
  //std::cout<<"coord: x = "<<columna<<", y = "<< fila<<"\n";
  return position;

}

//-------------------------------------------------------------
void bresenhamLine(int x0, int y0, int x1, int y1, std::vector<int8_t> &dataProb){
  int dx = abs(x1-x0), dx2 = 2*dx;
  int dy = abs(y1-y0), dy2 = 2*dy;
  int i=1, pos;
  int pk = 2*dy - dx; //parametro de decision
  int xini, yini, xfin, yfin;
  
  xini = x0;
  yini = y0;
  xfin = x1;
  yfin = y1;

  if(dx != 0){ 
    float m = ((float)y1-(float)y0)/((float)x1-(float)x0);

  pos = assign_points(gridMap.info.width, gridMap.info.height, x0, y0);
  dataProb.at(pos-1) = 0;

  if(!(x0 < x1)){
    std::swap(x0,x1);
    std::swap(y0,y1);
  }

  if(m > 0){

    do{
      //pintar celda
      pos = assign_points(gridMap.info.width, gridMap.info.height, x0, y0);
      if(dataProb.at(pos-1) != 100) dataProb.at(pos-1) = 0;
      
      while(pk >= 0){
        y0++;
        pk -= dx2;
        //pntar celda
        pos = assign_points(gridMap.info.width, gridMap.info.height, x0, y0);
        if(dataProb.at(pos-1) != 100) dataProb.at(pos-1) = 0;
      }
      x0++;
      pk += dy2;
      i++;

    }while(i <= dx);



  }else if(m < 0){

    do{
      
      pos = assign_points(gridMap.info.width, gridMap.info.height, x0, y0);
      if(dataProb.at(pos-1) != 100) dataProb.at(pos-1) = 0;
      while(pk >= 0){
        y0--;
        pk -= dx2;
      
        pos = assign_points(gridMap.info.width, gridMap.info.height, x0, y0);
        if(dataProb.at(pos-1) != 100) dataProb.at(pos-1) = 0;
      }
      x0++;
      pk += dy2;
      i++;



    }while(i <= dx);
  
  }else{
    
    do{
      pos = assign_points(gridMap.info.width, gridMap.info.height, x0, y0);
      if(dataProb.at(pos-1) != 100) dataProb.at(pos-1) = 0;
      x0++;
    }while(x0<=x1);

  }

    }else{

      if(!(y0 < y1)){
        std::swap(x0,x1);
        std::swap(y0,y1);
      }

      do{
        pos = assign_points(gridMap.info.width, gridMap.info.height, x0, y0);
        if(dataProb.at(pos-1) != 100) dataProb.at(pos-1) = 0;
        y0++;
      }while(y0<=y1);    
  }
  pos = assign_points(gridMap.info.width, gridMap.info.height, xini, yini);
  dataProb.at(pos-1) = 100;

  pos = assign_points(gridMap.info.width, gridMap.info.height, xfin, yfin);
  dataProb.at(pos-1) = 100;


}

//-------------------------------------------------------------
void bresenhamLine_wiki(int x0, int y0, int x1, int y1, std::vector<int8_t> &dataProb){
  int dy = y1-y0; 
  int dx = x1-x0;
  int incXi, incYi, incXr, incYr;
  
  int xini = x0, yini = y0, xfin = x1, yfin = y1;
  int pos;

  // 1 - Incrementos para las secciones con avance inclinado
  if(dy >= 0){
    incYi = 1;
  }else {
    incYi = -1;
    dy = -dy;
  }

  if(dx >= 0){
    incXi = 1;
  }else{
    incXi = -1;
    dx = -dx;
  }

  // 2 - Incrementos para las secciones con avance recto:
  if (dx >= dy){
    incYr = 0;
    incXr = incXi;
  }else{
    incYr = incYi;
    incXr = 0;
    // Cuando dy es mayor que dx, se intercambian, para reutilizar el mismo bucle.
    // ver octantes blancos en la imagen encima del código
    int k = dx; dx = dy; dy = k;
  }

  // 3  - Inicializar valores (y de error).
  int x = x0, y = y0;
  int avR = (2 * dy);
  int av = (avR - dx);
  int avI = (av - dx);

  // 4  - Bucle para el trazado de las línea.
  do{
    //std::cout << "\nPunto: "<<x<<", "<<y<<"\n";

    pos = assign_points(gridMap.info.width, gridMap.info.height, x, y);
    if(dataProb.at(pos-1) != 100) dataProb.at(pos-1) = 0;

    if (av >= 0){
      x = (x + incXi);     // X aumenta en inclinado.
      y = (y + incYi);     // Y aumenta en inclinado.
      av = (av + avI);     // Avance Inclinado
    }else{
      x = (x + incXr);     // X aumenta en recto.
      y = (y + incYr);     // Y aumenta en recto.
      av = (av + avR);     // Avance Recto
    }
  }while((x != x1) || (y != y1));

  pos = assign_points(gridMap.info.width, gridMap.info.height, xini, yini);
  dataProb.at(pos-1) = 100;

  pos = assign_points(gridMap.info.width, gridMap.info.height, xfin, yfin);
  dataProb.at(pos-1) = 100;

}

//-------------------------------------------------------------
nav_msgs::OccupancyGrid generate_Grid_Map(nav_msgs::OccupancyGrid gridMap, std::vector<int8_t> dataProb)
{
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

  map_pub = n.advertise<nav_msgs::OccupancyGrid>("map",10); //PUBLICAR

  sub = n.subscribe("scan", 1000, laserScancallback); //SUBSCRIBIRSE

  ros::spin();

  return 0;
}