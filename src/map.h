/**
 * A Map is a graph of intersections (nodes) and streets (edges). To save memory, we store
 * the map as a list of "turns". Every time you get to an intersection, you know where you
 * came from and where you want to go. You then calculate the angle at which you need to 
 * turn from the directions of the roads from the current intersection.
 */

#pragma once

#include <Arduino.h>
// #include <TList.h>

// struct Turn
// {
//     Intersection to;
//     int16_t direction; //direction to head from the intersection
// };

// struct Intersection
// {
//     /**
//      * List the reachable intersections and the direction of the street (absolute).
//      */
//     TList<Turn> turns;
// };

// class Map
// {
// private:
//     TList<Intersection> intersections;

// public:
//     int16_t calcAngle(Intersection from, Intersection at, Intersection to);
// };

PROGMEM const uint8_t directions[][4] = 
{
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
};

int16_t CalcTurn(uint8_t from, uint8_t at, uint8_t to)
{
    int16_t currentHeading = directions[at][from] - 180;
    int16_t desiredHeading = directions[at][to];

    int16_t turnAngle = desiredHeading - currentHeading;

    return turnAngle;
}