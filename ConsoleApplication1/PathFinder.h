#pragma once
#include "MapGrid.h"
#include <memory>
#include <json/json.h>
#include <list>

namespace PathFinder {
	class CVector {
	public:
		int X = 0, Y = 0;

		CVector operator +(const CVector&);
		CVector operator -(const CVector&);
		bool operator ==(const CVector& c) const { return c.X == X && c.Y == Y; }
		bool operator !=(const CVector& c) const { return !operator==(c); }

		CVector() {};
		CVector(int, int);
	};

	class GridNode {
	public:
		bool Occupied = false;
		CVector GridPosition;
		int gCost = 0, hCost = 0;
		CVector parentPosition;
		int TimeStep = -1;

		bool operator ==(const GridNode& g) const { return g.GridPosition == GridPosition; }
		bool operator !=(const GridNode& g) const { return !operator==(g); }

		int FCost();
		GridNode() = default;
		GridNode(CVector);
	};


	class MapGrid {
	public:
		int GridWidth, GridHeight;
		int NodeWidth, NodeHeight, NodeSpacing;
		std::vector<GridNode> obstacles;
		std::vector<GridNode> Grid;
		std::vector<GridNode> Constraints;

		std::vector<CVector> GetNeighbors(GridNode);
		MapGrid() = default;
		MapGrid(Json::Value root);
	private:

	};

	class AStar {
	public:
		//Find a path
		std::list<GridNode> FindPath(CVector, CVector);
		int GetDistance(GridNode, GridNode);
		MapGrid Map;

		AStar() = default;
		AStar(MapGrid);
	private:
		std::list<GridNode> SortPath(GridNode, GridNode);
	};


	class Path {
	public:
		CVector Start;
		CVector Goal;
		std::list<GridNode> PlannedPath;
		
		std::string GetFileData();

		Path(CVector, CVector,std::list<GridNode>);
	};

	class CBS {
	public:
		CBS();
	};
}