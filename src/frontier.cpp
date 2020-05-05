#include "../include/frontier.h"

Frontier::Frontier(int xIn, int yIn, double disIn)
    : x(xIn)
    , y(yIn)
    , min_distance(disIn) {};
double Frontier::getDis()
{
    return min_distance;
};
int Frontier::getX()
{
    return x;
};
int Frontier::getY()
{
    return y;
};
