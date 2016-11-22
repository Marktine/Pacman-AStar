#pragma once
class CPoint2D
{
private:
	int x;
	int y;
public:

	CPoint2D(void);
	CPoint2D(int x, int y);
	~CPoint2D(void);

	int GetX();
	int GetY();
};



