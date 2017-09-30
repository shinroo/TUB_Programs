#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include "DiffDrive.h"
#include "SensorPacket.h"
#include <math.h>
#include <float.h>
#include <time.h>
#include <algorithm> 
#include <vector>
#include <climits>
#include "lilac_fundamentals/Cell.h"
#include "lilac_fundamentals/Grid.h"
#include "lilac_fundamentals/Row.h"
#include "lilac_fundamentals/ExecutePlan.h"
#include "lilac_fundamentals/MoveToPosition.h"
#include "lilac_fundamentals/Pose.h"


#define WHITE 0;
#define GRAY 1;
#define BLACK 2;
	

struct point    
  {
    double x;
    double y;
  };

struct model    
  {
    double m;
    double c;
  };

struct MapPoint    
{
	int x;
	int y;
};

struct RobotPerspective
{
	MapPoint m;
	int globalPerspective;

	bool Front;
	bool Left;
	bool Right;

	int visibleWalls;
};

struct Cell
{
	MapPoint m;
	
	bool Top;
	bool Right;
	bool Left;
	bool Bottom;
};

struct MapPositon    
{
	int row;
	int column;
};


class CellNode {

	public: 
		CellNode(int rowN, int columnN);
		int id;

		int row; //row index of cell
		int column; //column index cell
	
		int distanceToGoalEstimate;
		
		std::vector<struct Edge> edges;
   		std::vector<int> walls ; 
   		
        
		int status;
		int distance;

		CellNode* predecesor;

		std::vector<CellNode*> getAdjacentNodes();
		void addEdge(CellNode* end_node);

};

struct Edge
{
	CellNode* endNode;
	int status; //Colour to identify the state of the Edge
};


CellNode::CellNode(int rowN, int columnN) {		
	this->row = rowN;
	this->column = columnN;
	this->id = rowN * 10 + columnN;
	
}

void CellNode::addEdge(CellNode* end_node){
	Edge newEdge = Edge();
	newEdge.endNode = end_node;
	newEdge.status = WHITE;

	this->edges.push_back(newEdge);
}

std::vector<CellNode*> CellNode::getAdjacentNodes(){
	std::vector<CellNode*> adjacentNodes;

	for (int i = 0; i < this->edges.size(); i++)
	{
		adjacentNodes.push_back(edges.at(i).endNode);
	}

	return adjacentNodes;
}



class MoveServer {

	public: 
		MoveServer();

		int actualRow;
		int actualColumn;
		bool listenToPose;
		
		ros::NodeHandle *rosnode;
		ros::ServiceClient planExecuter;
		lilac_fundamentals::ExecutePlan srvExecute;
		ros::Subscriber poseSubscriber;

		
		bool mapCalculated;

		std::vector<CellNode> nodes;
		std::vector<MapPositon> positions;

		void poseCallback(const lilac_fundamentals::Pose::ConstPtr&  msg);

		bool movePlanning(lilac_fundamentals::MoveToPosition::Request &req, lilac_fundamentals::MoveToPosition::Response &res);
		int moveDirection(CellNode* start, CellNode* next);
		std::vector<int> findShortestPath(int row, int column);
		void mapCallback(const lilac_fundamentals::Grid::ConstPtr& grid_msg);
		void mapToGraph();
		CellNode *getNodes(int row,int column);
		void calculatePaths();
		void printDistances();
		void getActualPosition();

};

MoveServer::MoveServer(){
	mapCalculated = false;
	listenToPose = true;
}
CellNode *MoveServer::getNodes(int row,int column)
{   //ROS_INFO("call %d %d ",row,column);
    for (int i = 0 ; i < nodes.size() ; i++)
    {
//	ROS_INFO("nodes (%d %d) \n",nodes.at(i).row,nodes.at(i).column);
        if (nodes.at(i).row == row && nodes.at(i).column == column)
        { 
	   ROS_INFO("Get -- (%d,%d)",nodes.at(i).row,nodes.at(i).column);
	   return &nodes.at(i);
    	}
    }
   // ROS_INFO("not a succes");
  
    
}
 

void MoveServer::mapCallback(const lilac_fundamentals::Grid::ConstPtr& grid_msg){
	//TODO Wissem here you need to implement the map to graph logic see how the mapCallback from robert works on localization.cpp
	if (mapCalculated == true){
		return;
	}
	ROS_INFO(" in ");
	int rowCount = grid_msg->rows.size();
	int cellCount = grid_msg->rows[0].cells.size();
    
    for(int row = 0; row < rowCount; row++){

		for (int cell = 0; cell < cellCount; cell++){
          CellNode newNode = CellNode(row, cell) ; 
          int wallCount = grid_msg->rows[row].cells[cell].walls.size();
          for (int wall = 0; wall < wallCount; wall++){
				newNode.walls.push_back( grid_msg->rows[row].cells[cell].walls[wall]);
            }
            
          nodes.push_back(newNode);
			}
		}
       for (int i = 0 ; i < nodes.size() ; i++)
       {
			ROS_INFO("hergestellte nodes (%d,%d)",nodes.at(i).row,nodes.at(i).column);
		}
        for(int i = 0 ; i < nodes.size() ; i++) {
		ROS_INFO("node mother (%d,%d)-->",nodes.at(i).row,nodes.at(i).column);
            if (!(std::find(nodes.at(i).walls.begin(), nodes.at(i).walls.end(), 0) != nodes.at(i).walls.end()) )
            {			
				ROS_INFO("right (%d,%d)",nodes.at(i).row,nodes.at(i).column+1);
				nodes.at(i).addEdge(getNodes(nodes.at(i).row,nodes.at(i).column+1));
				
				
            }
            
            if (!(std::find(nodes.at(i).walls.begin(), nodes.at(i).walls.end(), 1) != nodes.at(i).walls.end()) )
            {
				ROS_INFO("up (%d,%d)",nodes.at(i).row-1,nodes.at(i).column);
			
					nodes.at(i).addEdge(getNodes(nodes.at(i).row-1,nodes.at(i).column));
                
            }
            if (!(std::find(nodes.at(i).walls.begin(), nodes.at(i).walls.end(), 2) != nodes.at(i).walls.end()) )
            {
				ROS_INFO("left (%d,%d)",nodes.at(i).row,nodes.at(i).column-1);
				
					nodes.at(i).addEdge(getNodes(nodes.at(i).row,nodes.at(i).column-1));
                
            }
            
            if (!(std::find(nodes.at(i).walls.begin(), nodes.at(i).walls.end(), 3) != nodes.at(i).walls.end()) )
            {
				//ROS_INFO("Down %d  next %d",i,getNodes(nodes.at(i).row,nodes.at(i).column));
				ROS_INFO("Down (%d,%d)",nodes.at(i).row+1,nodes.at(i).column);
					nodes.at(i).addEdge(getNodes(nodes.at(i).row+1,nodes.at(i).column));
				//ROS_INFO("after Down %d",i);
                
            }

			for (int j =  0 ; j < nodes.at(i).edges.size() ; j++ ){
				ROS_INFO("(%d,%d) - > (%d,%d)", nodes.at(i).row,nodes.at(i).column,
				nodes.at(i).edges.at(j).endNode->row,nodes.at(i).edges.at(j).endNode->column);
			}
          
        }
	
	
	ROS_INFO("End");
	mapCalculated = true;

}
void MoveServer::mapToGraph(){

	mapCalculated = false;
	ros::Subscriber sub = rosnode->subscribe("map", 1, &MoveServer::mapCallback, this);

	while(!mapCalculated){
		ros::spinOnce();   
	}

	sub.shutdown();

	
	
}



void MoveServer::printDistances(){
	ROS_INFO("\n\n\n");
	for (int i = 0  ; i < nodes.size(); i ++ )
    {
        ROS_INFO("Distance to Node (%d,%d) is %d", nodes.at(i).row, nodes.at(i).column, nodes.at(i).distanceToGoalEstimate);  
    }
}

void MoveServer::calculatePaths()
{
	CellNode * start = getNodes(actualRow,actualColumn);
	
    
    for (int i = 0  ; i < nodes.size(); i ++ )
    {
        nodes.at(i).distanceToGoalEstimate = INT_MAX ; 
        nodes.at(i).predecesor = NULL ; 
        nodes.at(i).status = 0 ; 
        
    }
    
    start->predecesor = NULL ; 
    start->distanceToGoalEstimate = 0 ; 
    
    std::vector<CellNode*> Q ;
    Q.clear() ; 
    Q.push_back(start);
    
    while (!Q.empty() && ros::ok())
    {
        CellNode *n =  Q.back();
        Q.pop_back(); 
        n->status = 2; 
        std::vector<CellNode*> adjacentNodes = n->getAdjacentNodes();
        for (int i = 0 ; i < adjacentNodes.size() ; i ++ )
        {
            if (adjacentNodes.at(i)->status != 1 && adjacentNodes.at(i)->distanceToGoalEstimate > n->distanceToGoalEstimate + 1  )
            {
			
					adjacentNodes.at(i)->distanceToGoalEstimate = n->distanceToGoalEstimate + 1 ;
					adjacentNodes.at(i)->predecesor = n ; 
					//adjacentNodes.at(i)->status = 1;
					Q.push_back(adjacentNodes.at(i));
			
            }
        }
        //printDistances();
        //ROS_INFO("Nodes in queue %lu", Q.size());
        n = NULL ; 
    }
}
std::vector<int> MoveServer::findShortestPath(int row, int column){
	//TODO Wissem Here comes dijkstra the result must be a secuence of directions in the global frame
	//you can use my function moveDirection(CellNode start, CellNode next) to calculate this directions between your nodes
	ROS_INFO("\n\nMoveServer\nFind Path (%d,%d) ----> (%d,%d)\n\n\n", actualRow, actualColumn, row, column);
	std::vector<int> path2;
	calculatePaths();

	CellNode * temp = getNodes(row,column);

	while(temp->distanceToGoalEstimate != 0){
		//path.insert(path.begin(), findSeqBetweenNodes(*temp->predecesor, *temp), 1);
		//path.insert(path.begin(), moveDirection(temp->predecesor, temp), 1);
		path2.push_back(moveDirection(temp->predecesor, temp));		
		temp = temp->predecesor;
	}
	std::vector<int> path;
	ROS_INFO("MoveServer PATH:");
	for (int i = (int) path2.size()-1; i >= 0; i--)
	{
		path.push_back(path2.at(i));
		ROS_INFO("%d", path.at(path2.size()-1 - i));
	}

	
	return path;
}

int MoveServer::moveDirection(CellNode* start, CellNode* next){
	if(start->row == next->row-1 && start->column == next->column){
		return 3;
	} else if(start->row == next->row+1 && start->column == next->column){
		return 1;
	} else if(start->row == next->row && start->column == next->column-1){
		return 0;
	} else if(start->row == next->row && start->column == next->column+1){
		return 2;
	} else{
		return -1;
	}
}

bool MoveServer::movePlanning(lilac_fundamentals::MoveToPosition::Request &req, lilac_fundamentals::MoveToPosition::Response &res){
	getActualPosition();
	ROS_INFO("MoveServer Actual position (%d,%d)", actualRow, actualColumn);
	if(actualRow == req.row && actualColumn == req.column){
		res.success = true;
		return true;
	}
	srvExecute.request.plan = findShortestPath(req.row, req.column);
	bool success = planExecuter.call(srvExecute);
	//TODO Robot recovery 
	res.success = success;
	return success;

}

void MoveServer::getActualPosition(){
	MapPositon actualPosition = positions.back();
	actualRow = actualPosition.row;
	actualColumn = actualPosition.column;
}

void MoveServer::poseCallback(const lilac_fundamentals::Pose::ConstPtr&  msg)
{
	

	ROS_INFO("MoveServer Pose is (%d, %d)",msg->row , msg->column);
	if(msg->row == -2 && msg->column == -2){

		ROS_INFO("I am lost! I will localize one more time");
		

	} else{
		MapPositon newPosition;
		newPosition.row = msg->row;
		newPosition.column = msg->column;
		positions.push_back(newPosition);
		
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "moveServer");
	ros::NodeHandle n;
	MoveServer m1;
	m1.rosnode = &n;
	
	m1.planExecuter = n.serviceClient<lilac_fundamentals::ExecutePlan>("execute_plan");

	m1.poseSubscriber = n.subscribe("pose", 1, &MoveServer::poseCallback, &m1);
	
	ros::ServiceServer service = n.advertiseService("move_to_position", &MoveServer::movePlanning, &m1);
	ROS_INFO("davor ");

	m1.mapToGraph();
	ROS_INFO("Ready Path planning");

	
	ros::spin();
	
	return 0;
}

			
			
