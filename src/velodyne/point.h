//Clase Point


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