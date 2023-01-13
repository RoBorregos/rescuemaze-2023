#pragma once

#include <iostream>
#include <vector>
using namespace std;

#include "Maze.h"

void moveForward()
{
}

// Gira a la izquierda y modifica la direccion
void left(int &rDirection)
{
    (rDirection == 0) ? rDirection = 3 : rDirection--;
    cout << "left" << endl;
}

// Gira a la derecha y modifica la direccion
void right(int &rDirection)
{
    (rDirection == 3) ? rDirection = 0 : rDirection++;
    cout << "right" << endl;
}

// Gira a la direccion indicada
void rotateTo(int &rDirection, int newDirection)
{
    if (rDirection + 2 == newDirection or rDirection - 2 == newDirection)
    {
        left(rDirection);
        left(rDirection);
    }
    else if (rDirection + 1 == newDirection or rDirection - 3 == newDirection)
    {
        right(rDirection);
    }
    else if (rDirection - 1 == newDirection or rDirection + 3 == newDirection)
    {
        left(rDirection);
    }
}

// Mueve el robot hacia el norte
void moveNorth(vector<int> &pos, int &yMaze, int &rDirection)
{
    cout << "moveNorth" << endl;

    rotateTo(rDirection, 0);
    moveForward();

    // yMaze--;
}

// Mueve el robot hacia el sur
void moveSouth(vector<int> &pos, int &yMaze, int &rDirection)
{
    cout << "moveSouth" << endl;

    rotateTo(rDirection, 2);
    moveForward();

    // yMaze++;
}

// Mueve el robot hacia el este
void moveEast(vector<int> &pos, int &xMaze, int &rDirection)
{
    cout << "moveEast" << endl;

    rotateTo(rDirection, 1);
    moveForward();

    // xMaze++;
}

// Mueve el robot hacia el oeste
void moveWest(vector<int> &pos, int &xMaze, int &rDirection)
{
    cout << "moveWest" << endl;

    rotateTo(rDirection, 3);
    moveForward();

    // xMaze--;
}


// Calcula la posicion en la que estaria el robot al moverse a la direccion indicada por key
void calcPos(vector<int> &pos, string key)
{
    int rampDir = rampDirection(key);

    if (key == "north")
    {
        pos[1]++;

        if (rampDir != -1)
        {
            if (rampDir == 0)
                pos[2]++;
            else if (rampDir == 2)
                pos[2]--;
        }
    }
    else if (key == "south")
    {
        pos[1]--;

        if (rampDir != -1)
        {
            if (rampDir == 0)
                pos[2]--;
            else if (rampDir == 2)
                pos[2]++;
        }
    }
    else if (key == "east")
    {
        pos[0]++;

        if (rampDir != -1)
        {
            if (rampDir == 1)
                pos[2]++;
            else if (rampDir == 3)
                pos[2]--;
        }
    }
    else if (key == "west")
    {
        pos[0]--;

        if (rampDir != -1)
        {
            if (rampDir == 1)
                pos[2]--;
            else if (rampDir == 3)
                pos[2]++;
        }
    }
}

// Mueve el robot hacia la direccion indicada por key y regresa el tile al que se movio
Tile *move(Tile *tile, string key, vector<int> &pos, int &xMaze, int &yMaze, int &rDirection)
{
    if (tile->adjacentTiles[key])
    {
        calcPos(pos, key);
        moveMaze(key);

        if (key == "north")
        {
            moveNorth(pos, yMaze, rDirection);
        }
        else if (key == "east")
        {
            moveEast(pos, xMaze, rDirection);
        }
        else if (key == "south")
        {
            moveSouth(pos, yMaze, rDirection);
        }
        else if (key == "west")
        {
            moveWest(pos, xMaze, rDirection);
        }

        return tile->adjacentTiles[key];
    }

    return nullptr;
}

