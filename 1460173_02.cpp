// 1460173_02.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <queue>
#include "Point2D.h"
#define MAX 1000
using namespace std;

static HWND consoleHandle;

void DrawMap(int, int);
void Drawroute(int, int);
int DrawLine(HWND, CPoint2D, CPoint2D, int = 0, HDC = 0);
int DrawReg(HWND, CPoint2D, int, int = 0, int = 0, HDC = 0);
int DrawCircle(HWND, CPoint2D, int, int = 0, int = 0, HDC = 0);
int n;
int m;
static vector<vector<int>>map;
static int closed_nodes_map[1000][1000]; // map of closed (tried-out) nodes
static int open_nodes_map[1000][1000]; // map of open (not-yet-tried) nodes
static int **dir_map; // map of directions
const int dir = 4;
static int dx[dir] = { 1, 0, -1, 0 };
static int dy[dir]={0, 1, 0, -1};
class node
{
	// current position
	int xPos;
	int yPos;
	// total distance already travelled to reach the node
	int level;
	// priority=level+remaining distance estimate
	int priority;  // smaller: higher priority

public:
	node(int xp, int yp, int d, int p)
	{
		xPos = xp; yPos = yp; level = d; priority = p;
	}

	int getxPos() const { return xPos; }
	int getyPos() const { return yPos; }
	int getLevel() const { return level; }
	int getPriority() const { return priority; }

	void updatePriority(const int & xDest, const int & yDest)
	{
		priority = level + estimate(xDest, yDest) * 10; //A*
	}

	// give better priority to going strait instead of diagonally
	void nextLevel(const int & i) // i: direction
	{
		level += (dir == 8 ? (i % 2 == 0 ? 10 : 14) : 10);
	}

	// Estimation function for the remaining distance to the goal.
	const int & estimate(const int & xDest, const int & yDest) const
	{
		static int xd, yd, d;
		xd = xDest - xPos;
		yd = yDest - yPos;

		// Euclidian Distance
		d = static_cast<int>(sqrt(xd*xd + yd*yd));

		// Manhattan distance
		//d=abs(xd)+abs(yd);

		// Chebyshev distance
		//d=max(abs(xd), abs(yd));

		return(d);
	}
};
bool operator<(const node & a, const node & b)
{
	return a.getPriority() > b.getPriority();
}

void docFile(char* filename, vector<vector<int>> &map)
{
	ifstream fin;
	fin.open(filename);
	while (!fin.eof())
	{
		vector<int> vt;
		string temp;
		getline(fin, temp);
		for (int i = 0; i < temp.length(); i++)
		{
			if (temp[i] == '%')
			{
				vt.push_back(1);
			}
			else if (temp[i] == 'P')
			{
				vt.push_back(2);
			}
			else if (temp[i] == 'G')
			{
				vt.push_back(4);
			}
			else
			{
				vt.push_back(0);
			}
		}
		map.push_back(vt);
	}
	fin.close();
}

string pathFind(const int & xStart, const int & yStart,
	const int & xFinish, const int & yFinish)
{
	dir_map = new int*[n];
	for (int i = 0; i < n; i++)
	{
		dir_map[i] = new int[m];
	}
	static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
	static int pqi; // pq index
	static node* n0;
	static node* m0;
	static int i, j, x, y, xdx, ydy;
	static char c;
	pqi = 0;

	// reset the node maps
	for (y = 0; y<m; y++)
	{
		for (x = 0; x<n; x++)
		{
			closed_nodes_map[x][y] = 0;
			open_nodes_map[x][y] = 0;
		}
	}

	// create the start node and push into list of open nodes
	n0 = new node(xStart, yStart, 0, 0);
	n0->updatePriority(xFinish, yFinish);
	pq[pqi].push(*n0);
	open_nodes_map[x][y] = n0->getPriority(); // mark it on the open nodes map

	// A* search
	while (!pq[pqi].empty())
	{
		// get the current node w/ the highest priority
		// from the list of open nodes
		n0 = new node(pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
			pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

		x = n0->getxPos(); y = n0->getyPos();

		pq[pqi].pop(); // remove the node from the open list
		open_nodes_map[x][y] = 0;
		// mark it on the closed nodes map
		closed_nodes_map[x][y] = 1;

		// quit searching when the goal state is reached
		//if((*n0).estimate(xFinish, yFinish) == 0)
		if (x == xFinish && y == yFinish)
		{
			// generate the path from finish to start
			// by following the directions
			string path = "";
			while (!(x == xStart && y == yStart))
			{
				j = dir_map[x][y];
				c = '0' + (j + dir / 2) % dir;
				path = c + path;
				x += dx[j];
				y += dy[j];
			}

			// garbage collection
			delete n0;
			// empty the leftover nodes
			while (!pq[pqi].empty()) pq[pqi].pop();
			return path;
		}

		// generate moves (child nodes) in all possible directions
		for (i = 0; i<dir; i++)
		{
			xdx = x + dx[i]; ydy = y + dy[i];

			if (!(xdx<0 || xdx>n - 1 || ydy<0 || ydy>m - 1 || map[xdx][ydy] == 1
				|| closed_nodes_map[xdx][ydy] == 1))
			{
				// generate a child node
				m0 = new node(xdx, ydy, n0->getLevel(),
					n0->getPriority());
				m0->nextLevel(i);
				m0->updatePriority(xFinish, yFinish);

				// if it is not in the open list then add into that
				if (open_nodes_map[xdx][ydy] == 0)
				{
					open_nodes_map[xdx][ydy] = m0->getPriority();
					pq[pqi].push(*m0);
					// mark its parent node direction
					dir_map[xdx][ydy] = (i + dir / 2) % dir;
				}
				else if (open_nodes_map[xdx][ydy]>m0->getPriority())
				{
					// update the priority info
					open_nodes_map[xdx][ydy] = m0->getPriority();
					// update the parent direction info
					dir_map[xdx][ydy] = (i + dir / 2) % dir;

					// replace the node
					// by emptying one pq to the other one
					// except the node to be replaced will be ignored
					// and the new node will be pushed in instead
					while (!(pq[pqi].top().getxPos() == xdx &&
						pq[pqi].top().getyPos() == ydy))
					{
						pq[1 - pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pq[pqi].pop(); // remove the wanted node

					// empty the larger size pq to the smaller one
					if (pq[pqi].size()>pq[1 - pqi].size()) pqi = 1 - pqi;
					while (!pq[pqi].empty())
					{
						pq[1 - pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pqi = 1 - pqi;
					pq[pqi].push(*m0); // add the better node instead
				}
				else delete m0; // garbage collection
			}
		}
		delete n0; // garbage collection
	}
	return ""; // no route found
}

void getRowAndColumn(std::vector< std::vector<int> > vec, int &xA, int &xB, int& yA, int &yB)
{
	n = 0;
	m = 0;
	n = vec.size();
	m = vec[0].size();
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < m; j++)
		{
			if (vec[i][j] == 2)
			{
				xA = i;
				yA = j;
			}
			if (vec[i][j] == 4)
			{
				xB = i;
				yB = j;
			}
		}
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	consoleHandle = GetConsoleWindow();
	vector<vector<char>> str;
	char* filename = "layouts/bigCorners.lay";
	int xA, yA, xB, yB;
	docFile(filename, map);
	getRowAndColumn(map, xA, xB, yA, yB);
	string route = pathFind(xA, yA, xB, yB);
	if (route.length()>0)
	{
		int j; char c;
		int x = xA;
		int y = yA;
		map[x][y] = 2;
		for (int i = 0; i<route.length(); i++)
		{
			c = route.at(i);
			j = atoi(&c);
			x = x + dx[j];
			y = y + dy[j];
			map[x][y] = 3;
		}
		map[x][y] = 4;

		// display the map with the route
		for (int x = 0; x<n; x++)
		{
			for (int y = 0; y<m; y++)
			if (map[x][y] == 0)
				cout << " ";
			else if (map[x][y] == 1)
				//DrawMap(y, x); //obstacle
				cout << (char)219;
			else if (map[x][y] == 2)
				cout << (char)248; //start
			else if (map[x][y] == 3)
				cout << ".";
			else if (map[x][y] == 4)
				cout << 'G'; //finish
			cout << endl;
		}
	}
	getchar(); // wait for a (Enter) keypress  
	return(0);
}

void DrawMap(int x, int y)
{
		DrawReg(consoleHandle, CPoint2D(10 * x + 50, 10 * y + 10), 4, RGB(150, 120, 0),RGB(255, 255, 255));
		this_thread::sleep_for(chrono::milliseconds(10));
}

int DrawCircle(HWND windowHandle, CPoint2D center, int radius, int color, int fillcolor, HDC deviceContext)
{
	int a, b = 0;
	if (!deviceContext) deviceContext = GetDC(windowHandle), b = 1;
	HPEN newPen = CreatePen(PS_SOLID, 2, color);
	HPEN oldpen = (HPEN)SelectObject(deviceContext, newPen);
	HBRUSH oldBursh;
	HBRUSH newBursh;

	if (fillcolor)
	{
		newBursh = CreateSolidBrush(color);
		oldBursh = (HBRUSH)SelectObject(deviceContext, newBursh);
	}
	else
	{
		newBursh = (HBRUSH)GetStockObject(NULL_BRUSH);
		oldBursh = (HBRUSH)SelectObject(deviceContext, newBursh);
	}

	a = Ellipse(deviceContext, center.GetX() - radius, center.GetY() + radius, center.GetX() + radius, center.GetY() - radius);
	DeleteObject(SelectObject(deviceContext, oldpen));
	DeleteObject(SelectObject(deviceContext, oldBursh));

	if (b) ReleaseDC(windowHandle, deviceContext);
	return a;
}

void Drawroute(int x, int y)
{
	DrawCircle(consoleHandle, CPoint2D(10 * x + 12, 10 * y), 5, RGB(150, 120, 0), RGB(255, 255, 255));
	this_thread::sleep_for(chrono::milliseconds(50));
}

int DrawLine(HWND windowHandle, CPoint2D start, CPoint2D end, int color, HDC deviceContext)
{
	int a, b = 0;
	HPEN oldPen;
	
	HPEN newPen = CreatePen(PS_SOLID, 2, color);

	if (!deviceContext) deviceContext = GetDC(windowHandle), b = 1;

	oldPen = (HPEN)SelectObject(deviceContext, newPen);
	MoveToEx(deviceContext, start.GetX(), start.GetY(), NULL);
	a = LineTo(deviceContext, end.GetX(), end.GetY());
	DeleteObject(SelectObject(deviceContext, oldPen));

	if (b) ReleaseDC(windowHandle, deviceContext);
	return a;
}

int DrawReg(HWND windowHandle, CPoint2D center, int radius, int color, int fillcolor, HDC deviceContext)
{
	int a, b = 0;
	if (!deviceContext) deviceContext = GetDC(windowHandle), b = 1;
	HPEN newPen = CreatePen(PS_SOLID, 2, color);
	HPEN oldpen = (HPEN)SelectObject(deviceContext, newPen);
	HBRUSH oldBursh;
	HBRUSH newBursh;

	if (fillcolor)
	{
		newBursh = CreateSolidBrush(color);
		oldBursh = (HBRUSH)SelectObject(deviceContext, newBursh);
	}
	else
	{
		newBursh = (HBRUSH)GetStockObject(NULL_BRUSH);
		oldBursh = (HBRUSH)SelectObject(deviceContext, newBursh);
	}

	a = Rectangle(deviceContext, center.GetX() - radius, center.GetY() + radius, center.GetX() + radius, center.GetY() - radius);
	DeleteObject(SelectObject(deviceContext, oldpen));
	DeleteObject(SelectObject(deviceContext, oldBursh));

	if (b) ReleaseDC(windowHandle, deviceContext);
	return a;
}
