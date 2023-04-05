#include "FallenAgent.h"

Fallen_Agent::Fallen_Agent(){
  myType = FALLEN_AGENT;
  
}

Fallen_Agent::Fallen_Agent(v2f s, v2f e ){
  myType = FALLEN_AGENT;
  v2f w;
  v2fCopy(s, left);
  v2fCopy(e, right);
  v2fSub( left, right , w);
  v2fTangent( w, norm );
  //not positive this vector needs to be normalized
  v2fNormalize( norm, norm );


}


  void Fallen_Agent::getTopLeft( v2f r) {
    v2fAdd(left, norm, rectangleLength, r);
  }
  void Fallen_Agent::getTopRight(v2f r) {
    v2fAdd(right, norm, rectangleLength, r);
  }

//checking if any of the rectangles endpoint is in agents vision rectangle
bool Fallen_Agent::isVisible( v2f pos, v2f dir, float vislength, float viswidth){

    v2f topLeft;
    v2f topRight;
    getTopLeft(topLeft);
    getTopRight(topRight);

    //TODO : CONTINUE
}

void Fallen_Agent::getNorm( v2f ret){
  v2fCopy( norm, ret );
}

//find the closest distance from our line to the point
float Fallen_Agent::getDistance( v2f pos ){
  v2f v;
  getDirection( pos, v);
  return v2fLen(v);

}

//similar, but in vector form
void Fallen_Agent::getDirection( v2f pos, v2f res ){
//TODO : CONTINUE
}

int Fallen_Agent::getType(){
  return myType;
}

void Fallen_Agent::getVelocity( v2f ret ){
  v2fMult( ret, 0.0, ret );
}