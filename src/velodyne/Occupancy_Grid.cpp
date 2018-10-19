#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

#include "std_msgs/String.h"
#include <std_msgs/Int8.h>

#include <sstream>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sstream> // for std::stringstream

#define PI 3.141592


//---Comunicacion bidireccional-------------------------------
void chatterCallback(const std_msgs::Int8::ConstPtr& msg)
{ 
  int n = 0; 
  //ROS_INFO("Recibido: [%s]", msg->data.c_str());
  ROS_INFO("Recibido n: [%d]", msg->data);
  n = msg->data;
  n = n+n; 
  ROS_INFO("n+n: [%d]", n);
}
//-------------------------------------------------------------


//-------------------------------------------------------------

nav_msgs::OccupancyGrid generate_Grid_Map(float resolution, uint width, uint height){

  //ROS_INFO("generate_Grid_Map \n");

  nav_msgs::OccupancyGrid gridMap;// = OccupancyGrid_();
  
  gridMap.header.frame_id = "map";
  
  gridMap.info.resolution = resolution;
  gridMap.info.width = width;
  gridMap.info.height = height;
  
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

  float resolution = 0.01;
  uint width = 10;
  uint height = 10;
  std::string arg;

  for (int i = 1; i<argc; i++){
    
    arg = argv[i];
    
    if (arg == "-h") {
      
      std::cout << "Este nodo genera la rejilla probabilistica de ocupacion \n";
      std::cout << "  Parametros: \n";
      std::cout << "\t--r\t'resolution' \n";
      std::cout << "\t--w\t'width' \n";
      std::cout << "\t--h\t'height' \n";
      std::cout << "\n \n";
      std::cout << "Por default se genera una rejilla con:\n";
      std::cout << "\tresolution\t= 0.01 \n";
      std::cout << "\twidth\t\t= 10 \n";
      std::cout << "\theight\t\t= 10 \n";
      exit(1);
    
    }else if(argc > (i+1)){ 

        //ROS_INFO("Cadena de Argumentos = %d, iteracion = %d \n",argc,i);  
        std::stringstream convert(argv[i+1]); i++;
        
        if (arg == "--r"){
          
          if (!(convert >> resolution)) resolution = 0;
            ROS_INFO("resolution = %f \n",resolution);
          
        }else if(arg == "--w"){

            if (!(convert >> width)) width = 0;
            ROS_INFO("width = %u \n", width);
          
        }else if(arg == "--h"){
            if (!(convert >> height)) height = 0; 
            ROS_INFO("height = %u \n", height); 
        }
      
      } else{
        ROS_INFO("Debe ingresar el valor del argumento \n");
        exit(1);
      }
    

  }
  

  ros::NodeHandle n;

  //PUBLICAR
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  //ros::Publisher chatter_pub = n.advertise<std_msgs::Int8>("chatter", 1000);
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map",10);
  
  
  

  //------------------------
  //SUBSCRIBIRSE
  //ros::Subscriber sub = n.subscribe("chatter_2", 1000, chatterCallback);
  //------------------------

  nav_msgs::OccupancyGrid gridMap;

  ros::Rate loop_rate(10.0);
  
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  
  int count = 0;
  while (ros::ok())
  {
    
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    
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

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

    //ROS_INFO("Iteracion Num: [%d] \n",count);
    gridMap = generate_Grid_Map(resolution, width, height);

    //chatter_pub.publish(msg);
    map_pub.publish(gridMap);
    
    ros::spinOnce(); // Aqui se genera la magia

    loop_rate.sleep();
    ++count;
     
  }

  //ros::spin();//mientras no este ros::spinOnce()
  

  return 0;
}