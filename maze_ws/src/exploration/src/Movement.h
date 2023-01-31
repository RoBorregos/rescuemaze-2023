#pragma once

#include <iostream>
#include <vector>
using namespace std;

#include "Maze.h"
#include "InstructionServer.h"

#define DEBUG true

void moveForward(int &rDirection, InstructionServer &server)
{
    cout << "forward" << endl;

    server.setInstruction("forward");

    if (DEBUG)
    {
        printMaze(rDirection);
    }
}

// Gira a la izquierda y modifica la direccion
void left(int &rDirection, InstructionServer &server)
{
    (rDirection == 0) ? rDirection = 3 : rDirection--;
    cout << "left" << endl;

    server.setInstruction("left");

    if (DEBUG)
    {
        printMaze(rDirection);
    }
}

// Gira a la derecha y modifica la direccion
void right(int &rDirection, InstructionServer &server)
{
    (rDirection == 3) ? rDirection = 0 : rDirection++;
    cout << "right" << endl;

    server.setInstruction("right");

    if (DEBUG)
    {
        printMaze(rDirection);
    }
}

// Gira a la direccion indicada
void rotateTo(int &rDirection, int newDirection, InstructionServer &server)
{
    if (rDirection + 2 == newDirection or rDirection - 2 == newDirection)
    {
        left(rDirection, server);
        left(rDirection, server);
    }
    else if (rDirection + 1 == newDirection or rDirection - 3 == newDirection)
    {
        right(rDirection, server);
    }
    else if (rDirection - 1 == newDirection or rDirection + 3 == newDirection)
    {
        left(rDirection, server);
    }
}

// Mueve el robot hacia el norte
void moveNorth(vector<int> &pos, int &yMaze, int &rDirection, InstructionServer &server)
{
    cout << "moveNorth" << endl;

    rotateTo(rDirection, 0, server);
    moveForward(rDirection, server);

    // yMaze--;
}

// Mueve el robot hacia el sur
void moveSouth(vector<int> &pos, int &yMaze, int &rDirection, InstructionServer &server)
{
    cout << "moveSouth" << endl;

    rotateTo(rDirection, 2, server);
    moveForward(rDirection, server);

    // yMaze++;
}

// Mueve el robot hacia el este
void moveEast(vector<int> &pos, int &xMaze, int &rDirection, InstructionServer &server)
{
    cout << "moveEast" << endl;

    rotateTo(rDirection, 1, server);
    moveForward(rDirection, server);

    // xMaze++;
}

// Mueve el robot hacia el oeste
void moveWest(vector<int> &pos, int &xMaze, int &rDirection, InstructionServer &server)
{
    cout << "moveWest" << endl;

    rotateTo(rDirection, 3, server);
    moveForward(rDirection, server);

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
Tile *move(Tile *tile, string key, vector<int> &pos, int &xMaze, int &yMaze, int &rDirection, InstructionServer &server)
{
    if (tile->adjacentTiles[key])
    {
        calcPos(pos, key);
        moveMaze(key);

        if (key == "north")
        {
            moveNorth(pos, yMaze, rDirection, server);
        }
        else if (key == "east")
        {
            moveEast(pos, xMaze, rDirection, server);
        }
        else if (key == "south")
        {
            moveSouth(pos, yMaze, rDirection, server);
        }
        else if (key == "west")
        {
            moveWest(pos, xMaze, rDirection, server);
        }

        return tile->adjacentTiles[key];
    }

    return nullptr;
}

