#ifndef _FALLEN_AGENT_H_
#define _FALLEN_AGENT_H_


#include "CrowdObject.h"
#include <math.h>
#include <cmath>
#include <vector>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <cstdlib>
#include "constants.h"
#include "gnuplot-iostream.h"

#define MAXIMPATIENCE 100

class Fallen_Agent : public CrowdObject {


    /*
        left------P1
    A ->|          |
        right-----P2

    where "->" is the norm, or the direction of the agent before it fell
    
    */
 private: 
  //fallen agent left of rectangle
  v2f left; 
  //Fallen Agent right point of rectangle
  v2f right; 
 //norm is found in CrowdObject and is used for calculating the other endpoints

 float rectangleLength = 1.0;
 float rectangleWidth = 0.6;

 public:
  Fallen_Agent();
  //Fallen_Agent constructor taking the start and end vectors
  Fallen_Agent( v2f s, v2f e );

  void getLeft( v2f r) {v2fCopy(left, r);}
  void getRight( v2f r) {v2fCopy(right, r);}
  void getTopLeft( v2f r);
  void getTopRight(v2f r);


  //returns whether the object is visible within the vision rectangle presented
  bool isVisible( v2f pos, v2f dir, float vislength, float viswidth);

  //returns the norm of the object in the return parameter
  void getNorm( v2f ret ); 
  
  //gets the distance from the position argument to the object
  float getDistance( v2f pos );

  //gets the direction vector from the calling position to the object
  void getDirection( v2f pos, v2f res);


  //gets the type of object
  int getType();

  //returns a zero-vector for all non-agent CrowdObjects
  void getVelocity( v2f ret );

  //leaky abstractions
  void getPos( v2f ret) { v2fCopy(left, ret); return; }
  float getRadiu() {return 0.0;}
};

#endif
