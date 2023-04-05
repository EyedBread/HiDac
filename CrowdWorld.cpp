#include "CrowdWorld.h"
#include "gnuplot-iostream.h" // include gnuplot header
#include <chrono>
#include <thread>


CrowdWorld::CrowdWorld(){
  // Render::Render * r = Render::getInstance();
      // Gnuplot gp; // create a gnuplot object 

        gp << "set xrange [-30:30]\n";
        gp << "set yrange [-30:30]\n";
        // Add a dummy plot
        gp << "plot -1 notitle\n";
}

//adds the new CrowdObject(s) to the end of the vector
void CrowdWorld::createNewObject(Json::Value v){
  std::string s = v["type"].asString();
  std::string fa = "fallen agent";
  std::string a = "attractor";
  std::string w = "wall";
  std::string ag = "agent";
  std::string ob = "obstacle";
  if (s.compare(w) == 0){
    Wall* ws = twoWalls(v);

    objectList.push_back( ws );
    objectList.push_back( &(ws[1]) );
    //DRAW WALLS HERE
    v2f start1, end1, start2, end2;
    ws[0].getStart(start1);
    ws[0].getEnd(end1);
    ws[1].getStart(start2);
    ws[1].getEnd(end2);

    gp << "set arrow from " << start1[0] << "," << start1[1] << " to " << end1[0] << "," << end1[1] << " nohead\n";
    gp << "set arrow from " << start2[0] << "," << start2[1] << " to " << end2[0] << "," << end2[1] << " nohead\n";
    gp << "replot\n";

    gp<<"pause 0.1\n"; //pause for 0.1 seconds before continuing
    gp.flush();

    return;
  }
  if (s.compare(ag) == 0){
    
    objectList.push_back( new Agent(v) );
  }
  else 
    objectList.push_back( new CrowdObject(v) );

}

CrowdWorld::CrowdWorld( Json::Value w ){

  // Render::Render * r = Render::getInstance();
        gp << "set xrange [-50:50]\n";
        gp << "set yrange [-50:50]\n";
        // Add a dummy plot
        gp << "plot -5 notitle\n";
  //loading from file
  int numAgents = w["agents"].size();
  for(int i = 0; i < numAgents ; i++ ){
    Agent * a = new Agent(w["agents"][i]);
    agentList.push_back( a );
    // r->drawThis(a, a->getMesh());
  }

  int numObjects = w["objects"].size();
  for(int i = 0; i < numObjects ; i++){

    createNewObject( w["objects"][i] );
  }

  //end loading


}

CrowdWorld::~CrowdWorld(){
}



//updates each agent with visibility and collision information
void CrowdWorld::updateAgents(){
  for( std::vector<Agent *>::iterator a = agentList.begin(); a != agentList.end(); a++ ){
    //contains some inneficiency
    for( std::vector<Agent *>::iterator b = agentList.begin(); b != agentList.end(); b++ ){
      if (a != b) {
	(*a)->checkVisible(*b);
	(*a)->checkCollide(*b);
      }
    }
    for( std::vector<CrowdObject *>::iterator c = objectList.begin();  c != objectList.end(); c++ ){
      
      (* a)->checkVisible(* c);
      (* a)->checkCollide(* c);

    }

  }
}

//calcs forces for each agent
void CrowdWorld::calcForces(){
  for( std::vector<Agent * >::iterator it = agentList.begin();
       it != agentList.end();
       it++ ){
    (*it)->calculateForces();
  }
}
  
//applies forces for each agent
void CrowdWorld::stepWorld( float deltaT ){
  for( std::vector<Agent *>::iterator it = agentList.begin();
       it != agentList.end();
       it++ ){
    (* it)->applyForces(deltaT);
    (* it)->reset();
  }
}

void CrowdWorld::print(){
  std::ofstream tmpfile("data.tmp");
  for( std::vector<Agent *>::iterator it = agentList.begin();
       it != agentList.end();
       it++ ){



    v2f pos;
    (* it)->getPos(pos);
    tmpfile << pos[0] << " " << pos[1] << std::endl;
    (* it)->print();
    // gp<<"pause 0.1\n"; //pause for 0.1 seconds before continuing
    gp.flush();
  }
  tmpfile.close();

    // gp << "set pointtype 6\n"; // Set point type (e.g., 7 for a circle)
    gp << "set pointsize 1\n"; // Set point size
    // gp << "plot '-' with points notitle\n";

    gp << "plot 'data.tmp' with points\n";
    gp<<"pause 0.05\n"; //pause for 0.05 seconds before continuing
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    gp.flush();
  
}

void CrowdWorld::render(){
  // Render::Render * r = Render::getInstance();
  //requires a float, but that shouldn't affect anything
  // r->update(0.1);



}
