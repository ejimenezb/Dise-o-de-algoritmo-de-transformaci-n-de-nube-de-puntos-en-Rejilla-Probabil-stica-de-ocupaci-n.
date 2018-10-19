#include "ros/ros.h"
#include <string.h>
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Header.h"

// #include <std_msgs/Int8.h>
// #include <std_msgs/Int16.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/Int64.h>

// #include <std_msgs/UInt8.h>
// #include <std_msgs/Uint16.h>
// #include <std_msgs/Uint32.h>

// #include <std_msgs/FLoat32.h>
// #include <std_msgs/Float64.h>

// #include <std_msgs/UInt8MultiArray.h>

// #include <std_msgs/Time.h>
// #include <std_msgs/Bool.h>
 
using namespace ros;

void pointCloud2callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

  sensor_msgs::PointCloud2 data = *msg; // La funcion callback de ROS guarda la data cruda en la memoria, por lo tanto para extraerla hay que apuntar a la direccion de memoria.

  /*
  La estructura de datos esta comprendida por los siguientes 22 valores, 
  divididos entre los tipos string, uint, int y bool.
  
  uint32 seq
  time stamp       <------- Segundos, concatenados con nanosegundos 
  string frame_id

  uint32 height
  uint32 width
  
  uint8 INT8=1
  uint8 UINT8=2
  uint8 INT16=3
  uint8 UINT16=4
  uint8 INT32=5
  uint8 UINT32=6
  uint8 FLOAT32=7
  uint8 FLOAT64=8
  string name
  uint32 offset
  uint8 datatype
  uint32 count

  bool is_bigendian
  uint32 point_step
  uint32 row_step
  uint8[] data
  bool is_dense
  */

// Ahora imprimimos la data para visualizarla
  ROS_INFO(" \n\n I'm receiving from PointCLoud: \n\n seq: %d \n stamp: %d \n frame_id: %d \n height: %d \n width: %d \n Field1: %d \n Field2: %d \n Field3: %d \n Field4: %d \n Field5: %d \n Field6: %d \n Field7: %d \n Field8: %d \n name: %s \n offset: %d \n datatype: %d \n count: %d \n is_bigendian: %d \n point_step: %d \n row_step: %d \n data: %d \n is_dense: %d \n", 
                                                                                       data.header.seq, data.header.stamp, data.header.frame_id, data.height, data.width,
                                                                                       data.fields[0], data.fields[1], data.fields[2], data.fields[3], data.fields[4], 
                                                                                       data.fields[5], data.fields[6], data.fields[7], data.fields[8], data.fields[9], 
                                                                                       data.fields[10], data.fields[11], data.is_bigendian, data.point_step, data.row_step, 
                                                                                       data.data[0], data.is_dense);
}

void dataRestructuring(const sensor_msgs::PointCloud2::ConstPtr& msg)//---------------------------------------------------
{
  sensor_msgs::PointCloud2 data = *msg;

}

int main(int argc, char **argv)//--------------------------------------------
{
  init(argc, argv, "PointCloud2_Listener"); // Funcion de ROS que inicializa el nodo que vamos a lanzar

  NodeHandle n; // Declaracion del nodo a utilizar para la comunicacion

  Subscriber sub = n.subscribe("velodyne_points", 1000, pointCloud2callback);

  Rate loop_rate(1);

  spin();

  return 0;
}  