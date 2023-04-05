#include "Agent.h"

using namespace std;

Agent::Agent(){
  myType = AGENT;
  stopping = false; 
  waiting = false; 
  isColliding = false; 
}

#define MAXIMPATIENCE 100

Agent::Agent( Json::Value a ){
  myType = AGENT;
  stopping = false;
  waiting = false; 
  isColliding = false; 
  
  v2fMult(force, 0.0, force);

  v2fMult(vel, 0.0, vel);
  v2fMult(repelForce, 0.0, repelForce);
  v2fMult(norm, 0.0, norm);
  Beta = 0.0;



  attractorWeight = a["atWeight"].asDouble();
  wallWeight = a["waWeight"].asDouble();
  obstacleWeight = a["obWeight"].asDouble();
  fallenWeight = a["faWeight"].asDouble();
  agentWeight = a["agWeight"].asDouble();
  std::cout << "agent weight: " << agentWeight << std::endl; 
  acceleration = a["accel"].asDouble();
  std::cout << "agent accel: " << acceleration << std::endl; 
  maxVelocity = a["maxVel"].asDouble();
  vislong = a["visDist"].asDouble();
  viswide = a["visWid"].asDouble();

  radius = a["radius"].asDouble();
  
  personalSpace = a["pspace"].asDouble();

  mesh = a["mesh"].asString();

  //not quite sure how to specify attractor
  //for now we'll just have a CrowdObject contained in the agent
  attractor = CrowdObject( a["attractor"] );
  v2fFromJson( a["pos"], pos );
  if(a.isMember("norm")){
    v2fFromJson( a["norm"], norm);
  }

  panic = false;

  if(a.isMember( "liveValues" ) == true && a["liveValues"].asBool() == true){
    //all in-progress data will be reported to JSON object

  }
}

Agent::~Agent(){

}

void Agent::print(){
  // v2fPrint( "position: ", pos);
  return;

}
Json::Value Agent::getJson(){
  Json::Value v;
  v["atWeight"] = attractorWeight;
  v["waWeight"] = wallWeight;
  return v;
}

int Agent::getType(){
  return myType;
}

void Agent::getVelocity( v2f ret ){
  v2fCopy( vel , ret );
}

void Agent::setVelocity( v2f set ){
  v2fCopy( set, vel);
}

float Agent::getSpeed( ){
  return v2fLen( vel );
}

void Agent::getNorm( v2f get ){
  v2f s;
  getDirection( s );
  v2fCopy(s, norm);
  v2fNormalize(norm, norm);
  v2fCopy(norm, get);

}
void Agent::getDirection( v2f get ){
  if( this->getSpeed() >= 0.0 + MY_EPSILON ){
    v2fNormalize( vel, norm );
  }
  v2fCopy(norm, get);
  return;
}

float Agent::getPersonalSpace(){
  return personalSpace;
}

float Agent::getRadius(){
  return radius;
}

void Agent::getPos( v2f ret){ 
  v2fCopy( pos, ret );
  return;
}

void Agent::setPos( v2f set ){
  v2fCopy( set, pos );
  return;
}

bool Agent::isVisible( v2f objPos, v2f objDir, float vislength, float viswidth ){
  //procedure: given the circle that the agent has, find the point closest to the 
  //line described by origin: pos, direction: dir, length: vislength. 
  
  //procedure: take the point at the center of the circle representing an agent
  // check that the distance between it and the line is less than radius + viswidth

  //the technical procedure is out of geometric tools for computer games
  //compute effective pos /radius/ along the normal line (to avoid looking behind oneself 
  float d = ptToLineDist( pos, objPos, objDir, vislength);

  float er = radius + viswidth;
  if( d <= er )
    return true;

  return false;

}

float Agent::getDistance( v2f objPos ){
  v2f diff;
  v2fSub( objPos, pos, diff );
  return v2fLen(diff) - radius;
}

//returns the vector to get from objPos to position of the object
void Agent::getDirection( v2f objPos, v2f res ){
  v2fSub( objPos, pos, res );
  return;
}





void Agent::calculateRepelForce(){

  //repulsion forces are incurred only for those objects which are colliding with the agent
  //this means that for each c in collideObjects, we check what it is, the calc
  //based on the type what the repulsion is. Add them all together and store the result in repelForce (cleared on reset)

  /*overarching formula:
    repelForce = sum( repelForce(Walls)) 
    + obstacles (not implemented) 
    + lambda * sum(repelForce(Agents) ) 

    lambda is set to 0.3 if there are collisions with other obstacles to give preference to avoiding walls and obstacles over agents
   */

  std::cout << "REPELLING" << std::endl;

  float lambda = 1.0;
  v2f forceFromAgents, forceFromWalls, forceFromObjects;
  v2fMult( forceFromAgents, 0.0, forceFromAgents);
  v2fMult( forceFromWalls, 0.0, forceFromWalls);

  //CHECK IF THERES A WALL/OBSTACLE TO SET LAMBDA BEFORE LOOPING 

  for( std::vector<CrowdObject *>::iterator c = collideObjects.begin(); c != collideObjects.end(); c++){
      if ((*c)-> getType()) {
        lambda = 0.3;
        break;
      }
    }

  for( std::vector<CrowdObject *>::iterator c = collideObjects.begin(); c != collideObjects.end(); c++){
    switch( (*c)->getType()){
      case AGENT: {
	/*i is this agent, j is the other agent
	  d_ji is the distance between their centers
	  ep is the person
	  formula for agent: (pos_i - pos_j)*(r_i + ep_i + r_j - d_ji)/ d_ji
	  
	*/
	v2f pos_j; 

  v2f v_j;

  (*c)->getVelocity( v_j ); 
  //If the agents are moving in the same direction, maybe do something else?
  // bool sameDirection = v2fDot(vel, v_j ) > 0.7 ? true : false;
	(*c)->getPos( pos_j );
	v2f jtoi;
	v2fSub( pos, pos_j, jtoi); //Get the distance vector between the 2 agents
	/*getDistance subtracts out the radius of the agent
	  j->getDistance( pos ) = d_ji - r_j
	  => -getDistance_ji = -(d_ji - r_j) = -d_ji + r_j 
	*/
	float k = ( getRadius() + getPersonalSpace() + (*c)->getDistance( pos )) 
	  /
	  ((*c)->getDistance( pos ) + (*c)->getRadius() );
	v2fAdd(forceFromAgents, jtoi, k, forceFromAgents);
      
      }
      case WALL: {
	/* for walls, the formula is
	   n is the normal 
	   n * (r_i + ep_i - d_wi)/ d_wi
	*/
	//compute multiplicative factor, get the walls normal, then mult into wallForce. 
	//set lambda to 0.3 to give precendence to avoiding any agents
	float k = (radius + personalSpace - (*c)->getDistance(pos)) / (*c)->getDistance(pos);
	//k is sometimes memory-corrupt
	std::cout << "cdist: " << (*c)->getDistance(pos) << ", k: " << k << "\n";
	v2f norm;
	v2f currentforce;
	(*c)->getNorm( norm );

	v2fMult( norm, k, currentforce);
	//only add those forces which oppose the agents movement 
	//this is a consequence of the fact that walls are represented as two 
	//back-to-back sections
  //TODO : REMOVING THE IF - statement breaks everything
	if( v2fDot( currentforce, vel ) <= 0.0 ){
	  v2fAdd( forceFromWalls, currentforce, forceFromWalls);
	}
     }

          case OBSTACLE: {
      /*i is this agent, k is the other obstacle
        d_ki is the distance between their centers
        ep is the person
        formula for agent: (pos_i - pos_k)*(r_i + ep_i + r_k - d_ki)/ d_ki
        
      */
      v2f pos_k; 

  //If the agents are moving in the same direction, maybe do something else?
  // bool sameDirection = v2fDot(vel, v_j ) > 0.7 ? true : false;
	(*c)->getPos( pos_k );
	v2f jtoi;
	v2fSub( pos, pos_k, jtoi); //Get the distance vector between the 2 agents
	/*getDistance subtracts out the radius of the agent
	  j->getDistance( pos ) = d_ji - r_j
	  => -getDistance_ji = -(d_ji - r_j) = -d_ji + r_j 
	*/
	float k = ( getRadius() + getPersonalSpace() + (*c)->getRadius() - (*c)->getDistance( pos )) 
	  /
	  ((*c)->getDistance( pos ) );
	v2fAdd(forceFromAgents, jtoi, k, forceFromAgents);
      
      }

	//in the paper's model, there are also obstacles. I have excluded those for now
      default: {
	

      }
      }
  }

  /* carry out the overarching computation */
  //  v2fPrint( "agent forces: ",  forceFromAgents);
  //  v2fPrint( "force from walls: ",  forceFromWalls);
  if( v2fDot(vel, forceFromAgents) < 0 && ! panic ){
    stopping = true;
    stoptime = std::rand() % 25;
    v2fMult(vel, 0.0, vel);
    std::cout << "STOPPING" << std::endl;
  }
  v2fMult(forceFromAgents, lambda,forceFromAgents);
  v2fAdd(forceFromWalls, forceFromAgents, repelForce);
  //  v2fPrint( "repulsion forces: ", repelForce);
}



/* computes crosses for 2d vectors - returns (v1 x v2) x v1 */
void crossAndRecross( v2f v1, v2f v2, v2f ret){
  float firstcross = v2fCross(v1, v2);
  v2fMult(ret, 0.0, ret);
  ret[0] = -firstcross * v1[1];
  ret[1] = firstcross * v1[0];
}




void Agent::calcAgentForce(CrowdObject * a , v2f ret){
  v2f meToYou;
  v2f tforce;
  v2f otherVel;
  a->getDirection( pos, meToYou );
  a->getVelocity(otherVel);

  crossAndRecross( meToYou, vel, tforce);
  v2fNormalize( tforce , tforce );

  float distweight, dirweight;
  distweight = pow( v2fLen(meToYou) - vislong, 2);

  if( v2fDot( vel, otherVel ) > 0 ) {
    dirweight = 1.2;
  } else {
    dirweight = 2.4;
  }
  //add in a slight right-bias if you are headed toward an agent with a direct oncoming or directly same-direction as you
  // std::cout << "ang: " << v2fDot( vel, otherVel );
  //Changed from && to ||
  if( abs( v2fDot(vel, otherVel) ) <= MY_EPSILON * rightHandAngleMultiplier || abs( v2fDot(vel, meToYou)) <= MY_EPSILON * rightHandAngleMultiplier){
    v2f rforce;

    std::cout << "RIGHT HAND RULE APPLIED" << std::endl;

    //CHANGE rightHandAngleMultiplier FOR DIFFERENT CROWD DENSITIES

    v2fTangent( vel, rforce ); //Tangent to the right
    // v2fPrint(rforce);
    //tforce should be zero here
    v2fAdd(tforce, rforce, 2.0, tforce);
  }
  v2f myDir;
  v2f otherDir;
  v2fNormalize(vel, myDir);
  v2fNormalize(otherVel, otherDir);
  //TODO : DONT HARDCODE ANGLE, approx 45 degrees RIGHT NOW
  //WAITING RULES
  if ( v2fDot(myDir, otherDir) >= 0.785 && !panic && v2fLen(meToYou) < waitingRadius && waitTime <= 0) {
    waiting = true;
    waitTime = std::rand() % 10;
    std::cout << "WAITING" << std::endl;
  }



  v2fMult(tforce , distweight * dirweight, ret);
  return;
}

//application of the HiDAC algorithm to an agent, Equation 1 and 2 in paper
void Agent::calculateForces (){
  //calculate perceived density
  //	Density ahead of the agent. Is calculated as the number of agents in the rectangle(not semicircle) ahead of the 
	// agent (dist<R, and vec(agent,others)dotVel > 0) and divided by PIxRxR/2
  //phum->setDensityAhead(num_agents_ahead / AREA);
  //DECREASE VISION RECTANGLE LENGTH AND RIGHT PREFERENCE ANGLE WHEN THIS INCREASES
  float perceivedDensity = 0.0;

  //running total vector, VÄNSTERLEDET
  v2f rt;
  v2fMult(rt, 0.0, rt);
  
  //copy the last force in (term 1 in equation)
  v2fCopy(force, rt);

  //Force towards attractor
  v2f dtoattractor;
  //a problem is that with just the attractor the agent will 'pace' back and forth over it
  attractor.getDirection( pos, dtoattractor );
  v2fMult(dtoattractor, attractorWeight, dtoattractor);
  v2fAdd( rt, dtoattractor, rt);


  //foreach object in visObjects
  std::vector<CrowdObject *>::iterator it;
  v2f tempForce;
  
  //declared for use in switch
  float distweight;
  float dcrossv;
  v2f otherVel;
  v2f n;
  for( it = visObjects.begin() ; it != visObjects.end(); it++ ){
    v2fMult( tempForce, 0.0, tempForce);
    switch( (*it)->getType() ){
    case AGENT : { 
      v2f dtoa;
      (*it)->getDirection(pos, dtoa);
      float dist = v2fLen(dtoa);

      // Get the velocity of the other agent
      v2f otherVel;
      // (*it)->getVelocity(otherVel);

      if (dist < R && v2fDot(dtoa, vel) > 0) {
        num_agents_ahead++;
      }
      //this is called for on page 102, but does not seem to be a part of the algorithm
      /*      (*it)->getDirection( pos, dtoa );
      (*it)->getVelocity( otherVel);
      if( v2fLen(dtoa) < vislength - 1.5 && v2fDot(vel, otherVel) < 0.0){
      } else{*/
      calcAgentForce((*it), tempForce);
      v2fMult( tempForce, agentWeight, tempForce);
	//      }
      break;
    }
    case WALL : {
      //avoidance force for wall is wallnormal cross velocity cross wallnormal, normalized

      (*it)->getNorm(n);
      crossAndRecross(n, vel, tempForce); 
      v2fNormalize(tempForce, tempForce);
      v2fMult( tempForce, wallWeight, tempForce ); //WHATS WALLWEIGHT?
      // v2fPrint(tempForce);
      break; 
    }
    case OBSTACLE : {
      //for now, obstacles work the same as walls, perhaps in the future that will change
      (*it)->getDirection(pos, n);
      crossAndRecross(n, vel, tempForce);
      
      v2fNormalize(tempForce, tempForce);
      v2fMult(tempForce, obstacleWeight, tempForce);
      break;
    }

    case FALLEN_AGENT: {

      break;
    }
      //fallen_agent case not implemented
    default: 
      break;

    }
    
     v2fAdd(rt, tempForce, rt);

  }

  v2fCopy(rt, force);

  //normalize force
  v2fNormalize(force, force);

  // v2fPrint(force);

  
  //calculate repulsionForces (they will be added later)
  if( isColliding ){
    calculateRepelForce();
  } 
  if (waiting)
    v2fMult(vel, 0.0, vel);
}

//stub function
void Agent::computeFallen( v2f ret ){
  v2fMult(ret , 0.0, ret);

  for (int i = 0; i < visObjects.size(); i++) {
    if (visObjects[i]->getType() == FALLEN_AGENT) {
      v2f pos;
      visObjects[i]->getPos(pos);
      if (getDistance(pos) < 2.0 ) {//2.0 meters in the hidac 
        Beta = 0.5;
        return;
      }
    }
  }
  
  //if close enough to a fallen agent, should set the float Beta with the distance to a fallen agent. For now, set to zero
  Beta = 0.0;
}

float Agent::computeAlpha( ){
  if( v2fLen(repelForce) > 0.0 || stopping || waiting )
    return 0.0;
  else 
    return 1.0;
}

float Agent::computeVel( float deltaT ){
  if (v2fLen(vel) > maxVelocity)
    // return getSpeed();
    return maxVelocity;
  else {
    // v2f dir;
    // v2fNormalize(vel, dir);
    // v2f a = {acceleration*dir[0], acceleration*dir[1]};
    
    // // cout << "HERE" << endl;
    // v2fAdd(vel, a, deltaT, vel);
    // v2fPrint(vel);
    return v2fLen(vel) + acceleration*deltaT;
  }
    
}


void Agent::applyForces( float deltaT ){
  //start with the current position = pos
  v2f oldPos; 
  v2fCopy(pos, oldPos);

  //compute normal movement forces
  v2f fallen;
  computeFallen(fallen);
  v2f normalMove, movement;

  float moveFactor = computeAlpha() * computeVel(deltaT) * deltaT; //Equation 3 in paper, sort of
  // std::cout << "velocity: " << v2fLen(vel) << std::endl;
  v2fMult(force, (1.0 - Beta), normalMove); //Force should be enhetsvektor here
  v2fMult(fallen, Beta, fallen);
  v2fAdd(fallen, normalMove, movement);

  v2fMult(movement, moveFactor, movement);


  //add to repulsive Forces
  v2fAdd(movement, repelForce, movement);
  //this is the sum of forces for this move, store it in force for the computation on the next step
  v2fCopy( movement, force );
  v2fAdd(movement, pos, pos);

  //update velocity value after updating position
  v2fSub( pos, oldPos, vel );
  // updateVelocity();
  v2fNormalize( vel, norm);
}

//functions to update visibility and collision vectors
void Agent::checkCollide( CrowdObject * c ){

  //TODO : LOOK AT THIS
  v2f wallVec;
  c->getDirection( pos, wallVec);

  if(c->getDistance( pos ) < radius ){
    collideObjects.push_back( c );
    isColliding = true;
  }
}

void Agent::checkVisible( CrowdObject * c ){
  v2f n; 
  getNorm( n );
  //don't want to look behind ourself, so we'll pass in our position moved forward by our radius
  v2f ep;
  v2fAdd( pos, n, radius, ep);
  if( c->isVisible(ep, n, vislong - radius, viswide) ){
    if (c->getType() == AGENT)
      std::cout << "AGENT SPOTTED" << std::endl;
    visObjects.push_back( c );
  } 

}

//function to 'reset' at the end of a simulation step 
void Agent::reset(){

  isColliding = false;
  stoptime--;
  if(stoptime == 0){
    stopping = false;
  }


  waitTime--;

  // if (waitTime == 0)
    // Always reset waiting, to satisfy: Agent i moves again when its area of influence does not satisfy the conditions for waiting, 
    // or when the timer reaches the value 0 to avoid deadlocks
  waiting = false;

  //Affected by the crowd density ahead of agent
  vislong = 3.0;
  rightHandAngleMultiplier = 1.0;

  visObjects.clear();
  collideObjects.clear();
  num_agents_ahead = 0;
  v2f zero;
  v2fMult( zero, 0.0, zero );
  v2fCopy(zero, repelForce);

}
