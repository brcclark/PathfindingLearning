#include "PathFinder.h"
#include <string.h>
#include <iostream>
#include <list>

using namespace PathFinder;

CVector::CVector(int a, int b) {
	X = a;
	Y = b;
}
CVector CVector::operator+ (const CVector& param) {
	CVector temp;
	temp.X = X + param.X;
	temp.Y = Y + param.Y;
	return temp;
}

CVector CVector::operator- (const CVector& param) {
	CVector temp;
	temp.X = X - param.X;
	temp.Y = Y - param.Y;
	return temp;
}

//bool CVector::operator == (const CVector& param) {
//	return (X == param.X) && (Y == param.Y);
//}
//
//bool CVector::operator != (const CVector& param) {
//	return !operator==(param);
//}

MapGrid::MapGrid(Json::Value root) {
	GridWidth = root["width"].asInt();
	GridHeight = root["height"].asInt();
	NodeWidth = root["nodeWidth"].asInt();
	NodeHeight = root["nodeHeight"].asInt();
	NodeSpacing = root["spacing"].asInt();

	for (int i = 0; i < root["obstacles"].size();i++) {
		CVector _position(root["obstacles"][std::to_string(i)]["x"].asInt(), root["obstacles"][std::to_string(i)]["y"].asInt());
		GridNode _node(_position);
		_node.Occupied = true;
		obstacles.push_back(_node);
	}
	for (int y = 0; y < GridHeight; y++) {
		for (int x = 0; x < GridWidth; x++) {
			Grid.push_back(GridNode(CVector(x, y)));
		}
	}
	for (int i = 0; i < obstacles.size(); i++) {
		int index = obstacles[i].GridPosition.Y * GridWidth + obstacles[i].GridPosition.X;
		Grid[index].Occupied = true;
	}
};

std::vector<CVector> MapGrid::GetNeighbors(GridNode Node) {
	std::vector<CVector> neighbors;
	for (int i = -1; i <= 1; i++) {
		for (int j = -1; j <= 1; j++) {
			if (i == 0 && j == 0 || (i*i) == (j*j)) {
				continue;
			}
			int checkX = Node.GridPosition.X + i;
			int checkY = Node.GridPosition.Y + j;
			if (i != j && checkX >= 0 && checkX < GridWidth
				&& checkY >= 0 && checkY < GridHeight ) {
				neighbors.push_back(CVector(checkX, checkY));
			}
		}
	}
	return neighbors;
}

GridNode::GridNode(CVector _WorldPosition) {
	Occupied = false;
	GridPosition = _WorldPosition;
}

int GridNode::FCost() {
	return gCost + hCost;
}

AStar::AStar(MapGrid _Map) {
	Map = _Map;
}

std::list<GridNode> AStar::FindPath(CVector StartPos, CVector GoalPos) {
	//Need an open set
	std::list<GridNode> OpenSet;
	//Closed Set
	std::list<GridNode> ClosedSet;

	OpenSet.push_back(GridNode(StartPos));

	while (OpenSet.size() > 0) {
		GridNode currentNode = OpenSet.front();
		for (std::list<GridNode>::iterator i = OpenSet.begin(); i != OpenSet.end(); ++i) {
			if (i->FCost() < currentNode.FCost() ||
				i->FCost() == currentNode.FCost() &&
				i->hCost < currentNode.hCost) {
				currentNode = *i;
			}
		}
		OpenSet.remove(currentNode);
		ClosedSet.push_back(currentNode);

		if (currentNode.GridPosition == GoalPos) {
			return SortPath(currentNode,GridNode(StartPos));
		}

		std::vector<CVector> neighborsList = Map.GetNeighbors(GridNode(currentNode.GridPosition));
		for (int i = 0; i < neighborsList.size(); i++) {
			GridNode* neighbor = &Map.Grid[neighborsList[i].Y * Map.GridWidth + neighborsList[i].X];
			if (std::find(ClosedSet.begin(), ClosedSet.end(), *neighbor) != ClosedSet.end()
				|| neighbor->Occupied) {
				continue;
			}

			bool neighInOpen = (std::find(OpenSet.begin(), OpenSet.end(), *neighbor) != OpenSet.end());

			int newMovementCostToNeighbor = currentNode.gCost + GetDistance(currentNode, *neighbor);

			if (newMovementCostToNeighbor < neighbor->gCost || !neighInOpen) {
				neighbor->gCost = newMovementCostToNeighbor;
				neighbor->hCost = GetDistance(neighbor->GridPosition, GoalPos);
				neighbor->parentPosition = currentNode.GridPosition;

				if (!neighInOpen) {
					OpenSet.push_back(*neighbor);
				}
			}
		}
		neighborsList.clear();
	}
}

std::list<GridNode> AStar::SortPath(GridNode GoalNode, GridNode StartNode) {
	std::list<GridNode> path;
	GridNode currentNode = GoalNode;
	while (currentNode.GridPosition != StartNode.GridPosition) {
		path.push_back(currentNode);
		currentNode = Map.Grid[currentNode.parentPosition.Y * Map.GridWidth + currentNode.parentPosition.X];
	}
	//Add the time step to the array
	int i = 0;
	for (std::list<GridNode>::iterator nd = path.begin(); nd != path.end(); ++nd) {
		nd->TimeStep = path.size() - i;
		i++;
	}
	path.reverse();
	return path;
}

int AStar::GetDistance(GridNode NodeA, GridNode NodeB) {
	int dstX = abs(NodeA.GridPosition.X - NodeB.GridPosition.X);
	int dstY = abs(NodeA.GridPosition.Y - NodeB.GridPosition.Y);
	return dstX + dstY;
}

Path::Path(CVector _Start, CVector _Goal, std::list<GridNode> _PlannedPath) {
	Start = _Start;
	Goal = _Goal;
	PlannedPath = _PlannedPath;
}

std::string Path::GetFileData() {
	Json::Value outRoot;
	Json::Value agent;
	Json::Value data;
	Json::StreamWriterBuilder writeBuilder;
	

	agent["id"] = "#ID1";
	agent["start"]["x"] = Start.X;
	agent["start"]["y"] = Start.Y;
	agent["goal"]["x"] = Goal.X;
	agent["goal"]["y"] = Goal.Y;
	data["0"]["x"] = Start.X;
	data["0"]["y"] = Start.Y;
	for (auto const& nd : PlannedPath) {
		data[std::to_string(nd.TimeStep)]["x"] = nd.GridPosition.X;
		data[std::to_string(nd.TimeStep)]["y"] = nd.GridPosition.Y;
	}
	agent["path"] = data;
	outRoot["agent"] = agent;
	return Json::writeString(writeBuilder, outRoot);
}