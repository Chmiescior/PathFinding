#include <iostream>
#include <algorithm>
#include <string>
#include "olcConsoleGameEngine.h"

using namespace std;

class PathFinding : public olcConsoleGameEngine
{
public:
	PathFinding()
	{
		m_sAppName = L"Path Finding";
	}

private:
	struct Node
	{
		bool isBlocked = false;
		bool isVisited = false;
		int x, y;
		float global, local;
		Node* parent = nullptr;
		vector<Node*> neighbours;
	};

	Node* G = nullptr;
	int width = 12;
	int height = 12;

	Node* source = nullptr;
	Node* dest = nullptr;
	Node* from = nullptr;
	Node* to = nullptr;

	int distanceMode = 0;

	void prepareGraph()
	{
		for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
			{
				int index = j * width + i;
				G[index].isVisited = false;
				G[index].parent = nullptr;
				G[index].local = INFINITY;
				G[index].global = INFINITY;
			}
	}

	auto euclidianDistance(Node* a, Node* b)
	{
		return sqrtf((float)((a->x - b->x) * (a->y - b->y) + (a->y - b->y) * (a->y - b->y)));
	}

	auto manhattanDistance(Node* a, Node* b)
	{
		return (float)(abs(a->x - b->x) + abs(a->y - b->y));
	}

	auto diagDistance(Node* a, Node* b)
	{
		auto dx = abs(a->x - b->x);
		auto dy = abs(a->y - b->y);

		return (float)((dx + dy) - min(dx, dy));
	}

	auto heuristic(Node* a, Node* b)
	{
		switch (distanceMode)
		{
		case 0:
			return euclidianDistance(a, b);
		case 1:
			return manhattanDistance(a, b);
		case 2:
			return diagDistance(a, b);
		default:
			break;
		}
	}

	bool findPath()
	{
		prepareGraph();
		//prepare source
		source->local = 0.0f;
		source->global = heuristic(source, dest);
		Node* currNode = source;

		list<Node*> nodesToProcess;
		nodesToProcess.push_back(source);

		while (currNode != dest && !nodesToProcess.empty())
		{
			//sort the list of nodes to process 
			nodesToProcess.sort([](const Node* a, const Node* b) {return a->global < b->global; });

			//remove all previously processed nodes
			while (!nodesToProcess.empty() && nodesToProcess.front()->isVisited)
				nodesToProcess.pop_front();
			if (nodesToProcess.empty())
				break;

			//get current node to process
			currNode = nodesToProcess.front();
			currNode->isVisited = true;

			//process all neighbours of current node
			for (auto node : currNode->neighbours)
			{
				if (!node->isBlocked && !node->isVisited)
					nodesToProcess.push_back(node);

				//calculate neighbour's potential lowest parent distance
				float tempLocal = currNode->local + heuristic(currNode, node);

				if (tempLocal < node->local)
				{
					node->local = tempLocal;
					node->global = node->local + heuristic(node, dest);
					node->parent = currNode;
				}
			}
		}
		return true;
	}

	void onlySides()
	{
		for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
			{
				int index = j * width + i;
				G[index].neighbours.clear();

				if (j < height - 1)
					G[index].neighbours.push_back(&G[index + width]); //lower
				if (j > 0)
					G[index].neighbours.push_back(&G[index - width]); //upper
				if (i < width - 1)
					G[index].neighbours.push_back(&G[index + 1]); //right
				if (i > 0)
					G[index].neighbours.push_back(&G[index - 1]); //left
			}
		distanceMode = 1;
		findPath();
	}

	void onlyDiags()
	{
		for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
			{
				int index = j * width + i;
				G[index].neighbours.clear();

				if (i < width - 1 && j < height - 1)
					G[index].neighbours.push_back(&G[index + width + 1]); //lower right
				if (i < width - 1 && j > 0)
					G[index].neighbours.push_back(&G[index - width + 1]); //upper right
				if (j < height - 1 && i > 0)
					G[index].neighbours.push_back(&G[index + width - 1]); //lower left
				if (j > 0 && i > 0)
					G[index].neighbours.push_back(&G[index - width - 1]); //upper left
			}
		distanceMode = 2;
		findPath();
	}

	void sidesAndDiags()
	{
		for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
			{
				int index = j * width + i;
				G[index].neighbours.clear();

				if (j < height - 1)
					G[index].neighbours.push_back(&G[index + width]); //lower
				if (j > 0)
					G[index].neighbours.push_back(&G[index - width]); //upper
				if (i < width - 1)
					G[index].neighbours.push_back(&G[index + 1]); //right
				if (i > 0)
					G[index].neighbours.push_back(&G[index - 1]); //left

				if (i < width - 1 && j < height - 1)
					G[index].neighbours.push_back(&G[index + width + 1]); //lower right
				if (i < width - 1 && j > 0)
					G[index].neighbours.push_back(&G[index - width + 1]); //upper right
				if (j < height - 1 && i > 0)
					G[index].neighbours.push_back(&G[index + width - 1]); //lower left
				if (j > 0 && i > 0)
					G[index].neighbours.push_back(&G[index - width - 1]); //upper left
			}
		distanceMode = 0;
		findPath();
	}

	void removeEdges()
	{
		for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
			{
				int index = j * width + i;
				G[index].neighbours.clear();
			}
		findPath();
	}

	void reverseAll()
	{
		for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
			{
				int index = j * width + i;

				if (&G[index] != source && &G[index] != dest)
					G[index].isBlocked = !G[index].isBlocked;
			}
		findPath();
	}

	void blockAll()
	{
		for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
			{
				int index = j * width + i;

				if (&G[index] != source && &G[index] != dest)
					G[index].isBlocked = true;
			}
		findPath();
	}

	bool anyNumpadKeysPressed()
	{
		if (m_keys[VK_NUMPAD0].bPressed || m_keys[VK_NUMPAD1].bPressed || m_keys[VK_NUMPAD2].bPressed || m_keys[VK_NUMPAD3].bPressed
			|| m_keys[VK_NUMPAD4].bPressed || m_keys[VK_NUMPAD5].bPressed || m_keys[VK_NUMPAD6].bPressed || m_keys[VK_NUMPAD7].bPressed
			|| m_keys[VK_NUMPAD8].bPressed || m_keys[VK_NUMPAD9].bPressed)
			return true;
		else
			return false;
	}

protected:
	virtual bool OnUserCreate()
	{
		//create a proper graph
		G = new Node[width * height];
		for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
			{
				int index = j * width + i;
				G[index].x = i;
				G[index].y = j;
				G[index].parent = nullptr;
				G[index].isBlocked = false;
				G[index].isVisited = false;
			}

		//connect nodes
		for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
			{
				int index = j * width + i;
				if (j < height - 1)
					G[index].neighbours.push_back(&G[index + width]); //lower
				if (j > 0)
					G[index].neighbours.push_back(&G[index - width]); //upper
				if (i < width - 1)
					G[index].neighbours.push_back(&G[index + 1]); //right
				if (i > 0)
					G[index].neighbours.push_back(&G[index - 1]); //left

				if (i < width - 1 && j < height - 1)
					G[index].neighbours.push_back(&G[index + width + 1]); //lower right
				if (i < width - 1 && j > 0)
					G[index].neighbours.push_back(&G[index - width + 1]); //upper right
				if (j < height - 1 && i > 0)
					G[index].neighbours.push_back(&G[index + width - 1]); //lower left
				if (j > 0 && i > 0)
					G[index].neighbours.push_back(&G[index - width - 1]); //upper left
			}

		source = &G[(height - 1) * width];
		dest = &G[width - 1];
		return true;
	}

	virtual bool OnUserUpdate(float fElapsedTime)
	{
		int nodeSize = 13;
		int nodeBorder = 3;

		int xMouse = m_mousePosX / nodeSize;
		int yMouse = m_mousePosY / nodeSize;

		bool nodeSelected = false;

		if (m_mouse[0].bReleased)
		{
			if (xMouse >= 0 && xMouse < width && yMouse >= 0 && yMouse < height)
			{
				int index = yMouse * width + xMouse;
				if (m_keys[VK_SHIFT].bHeld)
					dest = &G[index];
				else if (m_keys[VK_CONTROL].bHeld)
					source = &G[index];
				else
					G[index].isBlocked = !G[index].isBlocked;

				findPath();
			}
		}

		
		if (m_mouse[1].bPressed)
		{
			if (xMouse >= 0 && xMouse < width && yMouse >= 0 && yMouse < height)
			{
				int index = yMouse * width + xMouse;
				from = &G[index];
				nodeSelected = true;
			}
		}

		if (m_mouse[1].bReleased)
		{
			int toIndex = -1;
			int fromIndex = -1;
			int index = yMouse * width + xMouse;
			to = &G[index];

			for (unsigned int i = 0; i < from->neighbours.size(); i++)
			{
				if (from->neighbours[i] == to)
				{
					toIndex = i;
					break;
				}
			}
			if (toIndex == -1)
				from->neighbours.push_back(to);
			else
				from->neighbours.erase(from->neighbours.begin() + toIndex);

			for (unsigned int i = 0; i < to->neighbours.size(); i++)
			{
				if (to->neighbours[i] == from)
				{
					fromIndex = i;
					break;
				}
			}
			if (fromIndex == -1)
				to->neighbours.push_back(from);
			else
				to->neighbours.erase(to->neighbours.begin() + fromIndex);
			from = nullptr;
			to = nullptr;
			findPath();
		}

		if (m_keys[VK_NUMPAD7].bPressed)
			sidesAndDiags();
		else if (m_keys[VK_NUMPAD8].bPressed)
			onlySides();
		else if (m_keys[VK_NUMPAD9].bPressed)
			onlyDiags();
		else if (m_keys[VK_SPACE].bPressed)
			removeEdges();
		else if (m_keys[VK_TAB].bPressed)
			reverseAll();
		else if (m_keys[VK_CAPITAL].bPressed)
			blockAll();

		if (m_keys[VK_NUMPAD0].bPressed)
			distanceMode = 0;
		else if (m_keys[VK_NUMPAD1].bPressed)
			distanceMode = 1;
		else if (m_keys[VK_NUMPAD2].bPressed)
			distanceMode = 2;

		if (anyNumpadKeysPressed())
			findPath();

		//DRAW
		Fill(0, 0, ScreenWidth(), ScreenHeight(), L' ');
		if (from != nullptr)
		{
			xMouse = m_mousePosX / nodeSize;
			yMouse = m_mousePosY / nodeSize;

			DrawLine(from->x * nodeSize + nodeSize / 2, from->y * nodeSize + nodeSize / 2, m_mousePosX, m_mousePosY, PIXEL_SOLID, FG_MAGENTA);
		}
		//EDGES
		for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
				for (auto node : G[j * width + i].neighbours)
				{
					if (!node->isBlocked && !G[j * width + i].isBlocked)
						DrawLine(i * nodeSize + nodeSize / 2, j * nodeSize + nodeSize / 2,
							node->x * nodeSize + nodeSize / 2, node->y * nodeSize + nodeSize / 2,
							PIXEL_SOLID, FG_DARK_BLUE);
				}

		//VERTICES
		for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
			{
				int index = j * width + i;

				if (!G[index].isBlocked)
					FillCircle(i * nodeSize + nodeSize / 2, j * nodeSize + nodeSize / 2, nodeBorder, PIXEL_SOLID, FG_BLUE);

				if (G[index].isVisited)
					FillCircle(i * nodeSize + nodeSize / 2, j * nodeSize + nodeSize / 2, nodeBorder, PIXEL_SOLID, FG_CYAN);

				if (&G[index] == source)
					FillCircle(i * nodeSize + nodeSize / 2, j * nodeSize + nodeSize / 2, nodeBorder, PIXEL_SOLID, FG_GREEN);

				if (&G[index] == dest)
					FillCircle(i * nodeSize + nodeSize / 2, j * nodeSize + nodeSize / 2, nodeBorder, PIXEL_SOLID, FG_RED);
			}

		//PATH
		if (dest != nullptr)
		{
			Node* ptr = dest;
			//cout << "Im here\n";
			while (ptr->parent != nullptr)
			{
				DrawLine(ptr->x * nodeSize + nodeSize / 2, ptr->y * nodeSize + nodeSize / 2,
					ptr->parent->x * nodeSize + nodeSize / 2, ptr->parent->y * nodeSize + nodeSize / 2,
					PIXEL_SOLID, FG_YELLOW);
				ptr = ptr->parent;
			}
		}
		return true;
	}
};


int main()
{
	PathFinding pathFinder;
	pathFinder.ConstructConsole(160, 160, 6, 6);
	pathFinder.Start();

	//CONTROLS:
	//SPACE - remove all edges
	//CAPSLOCK - block all vertices (apart from source and destination)
	//TAB - block unblocked vertices, unblock blocked vertices
	// 
	//NUMPAD0 - euclidian heuristics
	//NUMPAD1 - manhattan heuristics
	//NUMPAD2 - diagonal heuristics
	//NUMPAD7 - connect vertices next to each other horizontally, vertically and diagonally
	//NUMPAD8 - connect vertices next to each other horizontally and vertically
	//NUMPAD9 - connect vertices next to each other diagonally
	//
	//LMB - block / unblock vertice
	//CTRL + LMB - set source
	//SHIFT + LMB - set destination
	//RMB (drag and drop) - connect two vertices

	return 0;
}