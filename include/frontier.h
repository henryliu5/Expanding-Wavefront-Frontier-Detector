#ifndef FRONTIER_H
#define FRONTIER_H

class Frontier{
public:
    Frontier(int x, int y, double dis);
    double getDis();
    int getX();
    int getY();
protected:
    int x;
    int y;
    double min_distance;
};

#endif