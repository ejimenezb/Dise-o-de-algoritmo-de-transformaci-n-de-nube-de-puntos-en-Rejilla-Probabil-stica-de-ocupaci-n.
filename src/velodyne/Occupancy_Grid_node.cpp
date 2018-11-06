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
//#define GetSize(array_enteros) (sizeof(array_enteros)/sizeof(*(array_enteros)))

ros::Publisher map_pub;
ros::Publisher map_pub_cluster;
ros::Subscriber sub;

nav_msgs::OccupancyGrid gridMap;

int assign_points(uint width, uint height, int fila, int columna);
std::vector<int> assign_coord(uint width, uint height, int position);
void bresenhamLine(int x0, int y0, int x1, int y1, std::vector<int8_t> &dataProb);
nav_msgs::OccupancyGrid generate_Grid_Map(nav_msgs::OccupancyGrid gridMap, std::vector<int8_t> dataProb);
void laserScancallback(const sensor_msgs::LaserScan::ConstPtr& msg);
//-------------------------------------------------------------

class Point_cluster{

public:

int ind_1, visitado, vecino;

private:

int x, y, id;

public:

Point_cluster();
~Point_cluster();

void setX(int X);
void setY(int Y);
void setId(int ID);
void setPoint_cluster(int X, int Y, int ID);


int getX();
int getY();
int getId();
};

Point_cluster::Point_cluster(){
  x = 0;
  y = 0;
  id = 0;
  ind_1 = 0;
  visitado = 0;
  vecino = 0;
}

Point_cluster::~Point_cluster(){
  //Borra objeto

}

void Point_cluster::setX(int X){
  x = X;
}

void Point_cluster::setY(int Y){
  y = Y;
}

void Point_cluster::setId(int ID){
  id = ID;// id = 0 por default (no ha sido visitado), id = -1 es ruido, id > 0 pertenece a un grupo. Todos los puntos que pertenezcan a un mismo grupo tendran el mismo "id"
}

void Point_cluster::setPoint_cluster(int X, int Y, int ID){
  x = X;
  y = Y;
  id = ID;
}

int Point_cluster::getX(){return x;}

int Point_cluster::getY(){return y;}

int Point_cluster::getId(){return id;}

//DBSCAN necesita del Point_cluster que esta declarado anteiormente
void DBSCAN(std::vector<Point_cluster> point_occupied, float eps, int minPoints, std::vector<int8_t> dataProb);
void expandCluster(std::vector<Point_cluster> &point_occupied, Point_cluster point_core, float eps, int C, int minPoints, std::vector<Point_cluster> &point_neighbors);
std::vector<Point_cluster> regionQuery(std::vector<Point_cluster> &point_occupied, Point_cluster point_core, float eps);


void laserScancallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  sensor_msgs::LaserScan data = *msg; // La funcion callback de ROS guarda la data cruda en la memoria, por lo tanto para extraerla hay que apuntar a la direccion de memoria.
  nav_msgs::OccupancyGrid gridMap_pub;
  double angle_rad = data.angle_min; //angulo minimo en radianes del velodyne (desde donde parte el movimiento de escaneo)
  double angle_degrees = 0.0, dist_x_m = 0.0, dist_y_m = 0.0;
  int posX = 0, posY = 0, pos = 0, len = gridMap.info.height*gridMap.info.width, coord_x = 0, coord_y = 0;
  int center_x = gridMap.info.width/2;
  int center_y = gridMap.info.height/2;

  int ind = 0, minPoints = 3;
  float eps = sqrt(2);

  Point_cluster pointOccupied;
  std::vector<Point_cluster> point_occupied;
  
  //Puntero para arreglo dinamico
  //Point_cluster *point_occupied = NULL;

  std::vector<int8_t> dataProb;
  dataProb.assign(len,50);

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
                                                                                       data.range_min, data.range_max);*/

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
    std::cout <<"\n" ;*/

  for(int i = 0; i < data.ranges.size(); i++)  //Se rellena la rejilla
  {

    if(i>0) angle_rad = angle_rad + data.angle_increment;

    if(data.ranges[i] <= 200) // 200 m es la maxima distancia que nos provee el VLP-16 
    {
    
      dist_x_m = data.ranges[i] * cos(angle_rad);
      dist_y_m = data.ranges[i] * sin(angle_rad);

      coord_x = dist_x_m/gridMap.info.resolution;
      coord_y = dist_y_m/gridMap.info.resolution;
    
    }else{//Si hay una distancia que tiende a inf, esta se asigna a 250 m que esta fuera del rango del velodyne

      dist_x_m = 250 * cos(angle_rad);
      dist_y_m = 250 * sin(angle_rad);
      coord_x = dist_x_m/gridMap.info.resolution;
      coord_y = dist_y_m/gridMap.info.resolution;
    
    }
    
    angle_degrees = angle_rad*180/PI;    

    posX = center_x + coord_x;
    posY = center_y + coord_y;
    
    bresenhamLine(center_x, center_y, posX, posY, dataProb);
    //pos = assign_points(gridMap.info.width, gridMap.info.height, 25, 25);      
    //dataProb.at(pos) = 75;
      
    if ((abs(coord_x) < gridMap.info.width/2) && (abs(coord_y) < gridMap.info.height/2))
    { 
      //pointOccupied.setX(posX);//(gridMap.info.height - posY);
      //pointOccupied.setY(posY);//(posX);
      
      //Vectores
      //pointOccupied.ind_1 = ind;
      //point_occupied.push_back(pointOccupied);
      
      //std::cout << "point_occupied = " << point_occupied.at(ind).ind_1 <<", ind = "<<ind<< "\n\n";
      
      /* 
        //Arreglos dinamicos
        point_occupied = (Point_cluster*)realloc(point_occupied,  (ind + 1) * sizeof(Point_cluster));
        point_occupied[ind] = pointOccupied;
        point_occupied[ind].ind_1 = ind;*/
 
      //ind++;   

      pos = assign_points(gridMap.info.width, gridMap.info.height, posX, posY);      
      dataProb.at(pos) = 100;
    }

  }

  std::vector<int> coord;
  
  for(int pos = 0; pos < dataProb.size(); pos++)
  {
    if(dataProb.at(pos) == 100)
    {
      coord = assign_coord(gridMap.info.width, gridMap.info.height, pos);
      posX = coord.at(0);
      posY = coord.at(1);
      pointOccupied.setX(posX);
      pointOccupied.setY(posY);
      pointOccupied.ind_1 = ind;
      point_occupied.push_back(pointOccupied);
      ind++;
    }
  }

  //------------------      CLUSTER        ---------------------------------------------
  for(int i = 0; i < point_occupied.size(); i++)
  {
    std::cout<<"["<<point_occupied.at(i).getX()<<","<<point_occupied.at(i).getY()<<"],  ";
  }

  DBSCAN(point_occupied, eps, minPoints, dataProb);
  /* 
    int i_, x, y, x1, y1, id, eps = 2, minPoints = 3, distancia, count_cluster = 0, count_neighbors = 0, count_sub_neighbors = 0, visitado = 0;
    
    //Vectores
    std::vector<Point_cluster> point_cluster;
    std::vector<Point_cluster> point_neighbors;
    std::vector<Point_cluster> point_sub_neighbors;

    //*
      //Arreglos dinámicos
      //Point_cluster *point_cluster = NULL;
      //Point_cluster *point_neighbors = NULL;
      //Point_cluster *point_sub_neighbors = NULL;
    
    Point_cluster pointCluster;
    Point_cluster point_core;
    Point_cluster next_cluster;
    
    i_ = rand() % ind;
    point_core = point_occupied.at(i_);

    //Arreglo dinamico
    //point_core = point_occupied[i_]; 
      
    //*
    do{
      if(point_core.getId() == 0)//(next_cluster.getId() == 0)
      {
        //Vectores
        point_neighbors.push_back(point_core);

        //*
          //Arreglos dinamicos
          //point_neighbors = (Point_cluster*)realloc(point_neighbors,  (count_neighbors + 1) * sizeof(Point_cluster));
          //point_neighbors[count_neighbors] = point_core;
        
        x = point_core.getX();
        y = point_core.getY();
        count_neighbors++;
        visitado++;

        for(int i = 0; i < ind; i++){//RegionQuery

          //if(point_occupied[i].getId() == 0)
          //{
            
            //Vectores
            x1 = point_occupied.at(i).getX();
            y1 = point_occupied.at(i).getY();

            //*
              //Arreglos dinamicos
              //x1 = point_occupied[i].getX();
              //y1 = point_occupied[i].getY();

            x = x1 - x;
            y = y1 - y;

            distancia = sqrt(x*x + y*y);

            if(distancia <= eps)
            {
              //Vectores
              point_neighbors.push_back(point_core);
              
              //*
                //Arreglo dinámico
                //point_neighbors = (Point_cluster*)realloc(point_neighbors,  (count_neighbors + 1) * sizeof(Point_cluster));
                //point_neighbors[count_neighbors] = point_occupied[i];

              count_neighbors++;
            }
          
          //}
        
        }//fin RegionQuery
        
        if(count_neighbors >= minPoints)
        {
          //Vectores
          point_cluster.push_back(point_core); 
          
          //*
            //Arreglos dinamicos
            //point_cluster = (Point_cluster*)realloc(point_cluster,  (count_cluster + 1) * sizeof(Point_cluster));
            //point_cluster[count_cluster] = point_core;
          
          count_cluster++;

          //Vector
          point_occupied.at(point_core.ind_1).setId(1);

          //Arreglo dinamico
          //point_occupied[point_core.ind_1].setId(1);


          //Volver a iterar con los vecinos de point_core ---> point_neighbors y asi sucesivamente e ir marcando que fueron visitados
        
          //VAMOS A ITERAR

          for(int k = 0; k < count_neighbors; k++)//Modificar TODOS los contadores porque crecen dinamicamente!!!!!!!!!!!!!!!!!!1
          {
            if(point_neighbors.at(k).getId() == 0)//(point_neighbors[k].getId() == 0)
            {
              
              //Vectores
              next_cluster = point_neighbors.at(k);
              x = next_cluster.getX();
              y = next_cluster.getY();
              point_occupied.at(point_neighbors.at(k).ind_1).vecino = 1;
              
              //*
                //Arreglos dinamicos
                //next_cluster = point_neighbors[k];
                //x = next_cluster.getX();
                //y = next_cluster.getY();

                //point_occupied[point_neighbors[k].ind_1].vecino = 1;

              visitado++;

              for(int j = 0; j < ind; j++){//RegionQuery

                //if(point_occupied[j].getId() == 0)
                //{
                  
                  //Vectores
                  x1 = point_occupied.at(j).getX();
                  y1 = point_occupied.at(j).getY();

                  //*
                    //Arreglos dinamicos
                    //x1 = point_occupied[j].getX();
                    //y1 = point_occupied[j].getY();

                  x = x1 - x;
                  y = y1 - y;

                  distancia = sqrt(x*x + y*y);

                  if(distancia <= eps)
                  {
                    //Vectores
                    point_sub_neighbors.push_back(point_occupied.at(j));

                    //*
                      //Arreglos dinamicos
                      //point_sub_neighbors = (Point_cluster*)realloc(point_sub_neighbors,  (count_sub_neighbors + 1) * sizeof(Point_cluster));
                      //point_sub_neighbors[count_sub_neighbors] = point_occupied[j];
                    
                    count_sub_neighbors++;
                  }

                //}
              }//fin RegionQuery

              if(count_sub_neighbors >= minPoints)
              {
                for(int j = 0; j < count_sub_neighbors; j++) //Union de arrglos de vecinos
                {
                  if(point_sub_neighbors.at(j).vecino != 1)//(point_sub_neighbors[j].vecino != 1)
                  {
                    //Vectores
                    point_neighbors.push_back(point_sub_neighbors.at(j));

                    //*
                      //Arreglos dinamicos
                      //point_neighbors = (Point_cluster*)realloc(point_neighbors,  (count_neighbors + 1) * sizeof(Point_cluster));
                      //point_neighbors[count_neighbors] = point_sub_neighbors[j];
                    
                    count_neighbors++;
                  }
                }
                
                //Vectores
                point_occupied.at(point_neighbors.at(k).ind_1).setId(1);
                point_neighbors.at(k).setId(1);
                point_cluster.push_back(next_cluster);

                //*
                  //Arreglos dinamicos
                  //point_occupied[point_neighbors[k].ind_1].setId(1);
                  //point_neighbors[k].setId(1);
                  //point_cluster = (Point_cluster*)realloc(point_cluster,  (count_cluster + 1) * sizeof(Point_cluster));
                  //point_cluster[count_cluster] = next_cluster;

                count_cluster++;
              
              }//else{//si no tiene minPoints se marca como ruido?... el algoritmo no lo tiene previsto!!!!!!!!!!!!!!!!!!!!!!!!!!
                //*
                  //Vectores
                  //point_occupied.at(point_neighbors.at(k).ind_1).setId(-1);
                  //point_neighbors.at(k).setId(-1);

                  //Arreglos dinamicos
                  //point_occupied[point_neighbors[k].ind_1].setId(-1);
              
              //}            
            
            }
          }


        }else
        { //Se podria declarar y llenar un arreglo/vector de puntos ruido

          //Vectores
          point_occupied.at(point_core.ind_1).setId(-1);

          //*
            //Arreglo dinamico
            //point_occupied[point_core.ind_1].setId(-1); 
        }


      }else
      {
        i_ = rand() % ind;
        point_core = point_occupied[i_];
      }

    }while(visitado < ind);//Mientras que no hayan sido visitados todos los puntos */
  //REJILLA
  gridMap_pub = generate_Grid_Map(gridMap, dataProb);
  map_pub.publish(gridMap_pub); //Se publica la rejilla

}
//-------------------------------------------------------------
/*
  DBSCAN(D, eps, MinPts)
     C = 0
     for each unvisited point P in dataset D
        mark P as visited
        NeighborPts = regionQuery(P, eps)
        if sizeof(NeighborPts) < MinPts
           mark P as NOISE
        else
           C = next cluster
           expandCluster(P, NeighborPts, C, eps, MinPts)

  expandCluster(P, NeighborPts, C, eps, MinPts)
     add P to cluster C
     for each point P' in NeighborPts
        if P' is not visited
           mark P' as visited
           NeighborPts' = regionQuery(P', eps)
           if sizeof(NeighborPts') >= MinPts
              NeighborPts = NeighborPts joined with NeighborPts'
        if P' is not yet member of any cluster
           add P' to cluster C

  regionQuery(P, eps)
     return all points within P's eps-neighborhood (including P)*/
void DBSCAN(std::vector<Point_cluster> point_occupied, float eps, int minPoints, std::vector<int8_t> dataProb)
{ 
  
  //*
    nav_msgs::OccupancyGrid gridMap_pub;//*/
  
  int C = 0, i_ = 0;

  //Vectores 
  std::vector<Point_cluster> point_neighbors;
  Point_cluster point_core;
  //i_ = rand() % point_occupied.size();
  //point_core = point_occupied.at(i_);

  std::cout << "Tamaño ocupados = " << point_occupied.size() << " eps = " << eps << " minPts = " << minPoints << "\n\n";

  for(int i = 0; i < point_occupied.size(); i++)
  {
    if(point_occupied.at(i).visitado == 0)//(point_core.visitado == 0)//(point_core.getId() == 0)
    {
      point_core = point_occupied.at(i);
      point_core.visitado = 1;
      point_occupied.at(i).visitado = 1;

      point_neighbors = regionQuery(point_occupied, point_core, eps);
      //std::cout << "Entrada con ind de occu = " << point_occupied.at(i).ind_1 <<", i = "<< i << "\n\n";
      std::cout << "tamaño vecinos = " << point_neighbors.size() << "\n\n";
      
      if(point_neighbors.size() < minPoints)
      {
        //Ruido
        point_core.setId(-1);
        point_occupied.at(i).setId(-1);
      }else
      {
        C = C + 1;
        
        std::cout << "C = " << C << "\n\n";
        
        expandCluster(point_occupied, point_core, eps, C, minPoints, point_neighbors);
      }

    }
  }

  //*
    //Rellenar la rejilla (CICLAR)
    int cluster = 0, posX, posY, pos;
    for(int i = 0; i < point_occupied.size(); i++)
    {
      if(point_occupied.at(i).getId() == cluster)
      {
        posX = point_occupied.at(i).getX();
        posY = point_occupied.at(i).getY();
        pos = assign_points(gridMap.info.width, gridMap.info.height, posX, posY);      
        dataProb.at(pos) = 75;
      }
    }

    //REJILLA
    gridMap_pub = generate_Grid_Map(gridMap, dataProb);
    map_pub_cluster.publish(gridMap_pub); //Se publica la rejilla*/
}

std::vector<Point_cluster> regionQuery(std::vector<Point_cluster> &point_occupied, Point_cluster point_core, float eps)
{     
  float distancia = 0;
  int x, y, x1, y1;
  //Vectores
  std::vector<Point_cluster> point_neighbors;
  //point_neighbors.push_back(point_core);
      
  x = point_core.getX();
  y = point_core.getY();

  for(int i = 0; i < point_occupied.size(); i++)
  {
    //Vectores
    x1 = point_occupied.at(i).getX();
    y1 = point_occupied.at(i).getY();

    x = x1 - x;
    y = y1 - y;

    distancia = sqrt(x*x + y*y);

    if(distancia <= eps)
    {
      //Vectores
      point_neighbors.push_back(point_occupied.at(i));//estaba point_core lo cual no tiene sentido
      std::cout << "Posiciones = [" << point_occupied.at(i).getX() <<", "<<point_occupied.at(i).getY()<< "] ind = "<< point_occupied.at(i).ind_1 <<"\n\n";
    }

    if(point_occupied.size() == (i+1)) std::cout <<point_occupied.at(point_occupied.size()).visitado << "\n\n";     
  }
  return point_neighbors;
}

void expandCluster(std::vector<Point_cluster> &point_occupied, Point_cluster point_core, float eps, int C, int minPoints, std::vector<Point_cluster> &point_neighbors)
{
  std::vector<Point_cluster> point_sub_neighbors;
  //std::vector<Point_cluster> point_cluster;

  point_core.setId(C);
  point_occupied.at(point_core.ind_1).setId(C);
  //point_cluster.push_back(point_core);

  std::cout << "tamaño vecinos_expand_cluster = " << point_neighbors.size() << "\n\n";

  for(int k = 0; k < point_neighbors.size(); k++)
  { 
    //std::cout <<"Id: " << point_neighbors.at(k).getId() << ", ind = " <<point_neighbors.at(k).ind_1<< ", Visitado "<< k <<" = " << point_neighbors.at(k).visitado << "\n\n";
    //std::cout << "Posiciones = [" << point_neighbors.at(k).getX() <<", "<<point_neighbors.at(k).getY()<< "] \n\n";
    if(point_neighbors.size() == (k+1)) std::cout << "Visitado "<< k <<" = " << point_neighbors.at(point_neighbors.size()).visitado << "\n\n";
    
    if(point_neighbors.at(k).visitado == 0)//(point_neighbors.at(k).getId() == 0)
    { 
      point_neighbors.at(k).visitado = 1;
      point_occupied.at(point_neighbors.at(k).ind_1).visitado = 1;

      point_sub_neighbors = regionQuery(point_occupied, point_neighbors.at(k), eps);
      //std::cout << "tamaño sub vecinos = " << point_sub_neighbors.size() << "\n\n";

      if(point_sub_neighbors.size() >= minPoints)
      {
        for(int j = 0; j < point_sub_neighbors.size(); j++) //Union de arrglos de vecinos
        {
          //Vectores
          point_neighbors.push_back(point_sub_neighbors.at(j));
        }
        //point_cluster.push_back(point_sub_neighbors.at(j));
        
        //point_neighbors.at(k).setId(C);
        //point_occupied.at(point_neighbors.at(k).ind_1).setId(C);
      }
    }

    if(point_neighbors.at(k).getId() == 0)//Cada punto tiene indicado su cluster correpondiente (ruido: -1, cluster 1: 1, cluster 2: 2 y ... etc)
    {
      point_neighbors.at(k).setId(C);
      point_occupied.at(point_neighbors.at(k).ind_1).setId(C);
    }
  }
  //std::cout << "tamaño vecinos_expand_cluster_2 = " << point_neighbors.size() << "\n\n";
}
//-------------------------------------------------------------

int assign_points(uint width, uint height, int columna, int fila)//x,y
{
  int position = width * (fila) + columna;
  return position;

}

std::vector<int> assign_coord(uint width, uint height, int position)
{
  std::vector<int> coord;
  int posX, posY;

  posY = position/height; //posY = (position - columna)/width;
  posX = position - width * (posY);

  coord.push_back(posX);
  coord.push_back(posY);

  return coord;
}

//-------------------------------------------------------------
void bresenhamLine(int x0, int y0, int x1, int y1, std::vector<int8_t> &dataProb){
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

    pos = assign_points(gridMap.info.width, gridMap.info.height, x, y);
    if(dataProb.at(pos) != 100) dataProb.at(pos) = 0;

    if (av >= 0){
      x = (x + incXi);     // X aumenta en inclinado.
      y = (y + incYi);     // Y aumenta en inclinado.
      av = (av + avI);     // Avance Inclinado
    }else{
      x = (x + incXr);     // X aumenta en recto.
      y = (y + incYr);     // Y aumenta en recto.
      av = (av + avR);     // Avance Recto
    }

  }while(((x != x1) || (y != y1)) && (x != gridMap.info.width) && (y != gridMap.info.height) && (x >= 0) && (y >= 0));
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
  map_pub_cluster = n.advertise<nav_msgs::OccupancyGrid>("map_cluster",10); //PUBLICAR

  sub = n.subscribe("scan", 1000, laserScancallback); //SUBSCRIBIRSE

  ros::spin();

  return 0;
}