#include "ros/ros.h"
#include <cstdlib>
#include <vector>
#include "lilac_fundamentals/Cell.h"
#include "lilac_fundamentals/Grid.h"
#include "lilac_fundamentals/Row.h"

bool mapCalculated = false;

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

std::vector<RobotPerspective> perspectives;

/**
	row ~ y
	cell ~ x
**/
void mapCallback(const lilac_fundamentals::Grid::ConstPtr& grid_msg){

	if (mapCalculated == true){
		return;
	}

	int rowCount = grid_msg->rows.size();
	int cellCount = grid_msg->rows[0].cells.size();

	for(int row = 0; row < rowCount; row++){

		for (int cell = 0; cell < cellCount; cell++){

			int wallCount = grid_msg->rows[row].cells[cell].walls.size();

			Cell newCell;

			newCell.m.x = row;
			newCell.m.y = cell;

			newCell.Top = false;
			newCell.Right = false;
			newCell.Left = false;
			newCell.Bottom = false;

			for (int wall = 0; wall < wallCount; wall++){

				int walltype = grid_msg->rows[row].cells[cell].walls[wall];
				
				if (walltype == 0){

					//RIGHT
					newCell.Right = true;

				} else if (walltype == 1){

					//TOP
					newCell.Top = true;

				} else if (walltype == 2){

					//LEFT
					newCell.Left = true;

				} else {

					//BOTTOM
					newCell.Bottom = true;

				}

			}

			// Determine perspectives from cell

			RobotPerspective temp;
			temp.m.x = newCell.m.x;
			temp.m.y = newCell.m.y;

			// UP
			temp.globalPerspective = 1;
			
			temp.Front = false;
			temp.Left = false;
			temp.Right = false;

			temp.visibleWalls = 0;

			if (newCell.Top == true){
				temp.Front = true;
				temp.visibleWalls++;
			}	
			if (newCell.Left == true){
				temp.Left = true;
				temp.visibleWalls++;
			}
			if (newCell.Right == true){
				temp.Right = true;
				temp.visibleWalls++;
			}		

			perspectives.push_back(temp);

			// DOWN
			temp.globalPerspective = 3;
			
			temp.Front = false;
			temp.Left = false;
			temp.Right = false;

			temp.visibleWalls = 0;

			if (newCell.Bottom == true){
				temp.Front = true;
				temp.visibleWalls++;
			}	
			if (newCell.Left == true){
				temp.Right = true;
				temp.visibleWalls++;
			}
			if (newCell.Right == true){
				temp.Left = true;
				temp.visibleWalls++;
			}

			perspectives.push_back(temp);

			// LEFT
			temp.globalPerspective = 2;
			
			temp.Front = false;
			temp.Left = false;
			temp.Right = false;

			temp.visibleWalls = 0;

			if (newCell.Top == true){
				temp.Right = true;
				temp.visibleWalls++;
			}	
			if (newCell.Left == true){
				temp.Front = true;
				temp.visibleWalls++;
			}
			if (newCell.Bottom == true){
				temp.Left = true;
				temp.visibleWalls++;
			}

			perspectives.push_back(temp);

			// RIGHT
			temp.globalPerspective = 0;
			
			temp.Front = false;
			temp.Left = false;
			temp.Right = false;

			temp.visibleWalls = 0;

			if (newCell.Top == true){
				temp.Left = true;
				temp.visibleWalls++;
			}	
			if (newCell.Bottom == true){
				temp.Right = true;
				temp.visibleWalls++;
			}
			if (newCell.Right == true){
				temp.Front = true;
				temp.visibleWalls++;
			}

			perspectives.push_back(temp);
		}

	} 

	int size = perspectives.size();

	for(int i = 0; i < size; i++){
	
		ROS_INFO("POINT (%d,%d), Perspective %d, Visisble Walls %d, Front %d, Left %d, Right %d", 
				perspectives.at(i).m.x, 
				perspectives.at(i).m.y, 
				perspectives.at(i).globalPerspective,
				perspectives.at(i).visibleWalls,
				perspectives.at(i).Front,
				perspectives.at(i).Left,
				perspectives.at(i).Right);
	}

	mapCalculated = true;
}


int main(int argc, char **argv){

	ros::init(argc, argv,"Conversion");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("map", 1, mapCallback);

	ros::spin();
	
	return 0;
}

