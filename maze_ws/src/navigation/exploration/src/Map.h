#pragma once

#include <vector>
#include <iostream>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
// #include <nav_msgs/OccupancyGrid.h>

using namespace std;

#include "Tile.h"

class Map
{
private:
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar &xrmaze;
        ar &yrmaze;
        ar &zrmaze;

        ar &unvisited;
        ar &tile;
        ar &tiles;
        ar &recovpos;
    }

public:
    vector<Tile *> unvisited;
    map<string, Tile *> tiles;
    Tile *tile;
    int xMaze;
    int yMaze;
    int zMaze;

    int xrmaze;
    int yrmaze;
    int zrmaze;

    vector<int> pos;
    vector<int> recovpos;

    Map();

    void getMazePos(int &x, int &y, int &z, string key);
    char getDirChar(int &rDirection);
    void moveMaze(string dir);
    void setRecovPos();
    void printMaze(int &rDirection);

    char getNorth();
    char getEast();
    char getSouth();
    char getWest();
    char getChar(string key);

    int rampDirection(string key);

    vector<vector<char>> maze = {
        {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
        {'#', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', '#', '#'},
        {'#', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', ' ', '#', '#', '#', '#', '#', '#', '#', ' ', '#'},
        {'#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', '#', '#', '#', '#', '#', '#', ' ', '#', ' ', '#', ' ', '#'},
        {'#', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', '#'},
        {'#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#'},
        {'#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', '#', ' ', '#', ' ', '#'},
        {'#', '#', '#', '#', '#', '#', '#', ' ', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', ' ', '#', '#', '#'},
        {'#', ' ', ' ', ' ', '#', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', '#'},
        {'#', '#', '#', '#', '#', ' ', '#', ' ', '#', '#', '#', ' ', '#', ' ', '#', ' ', '#', '#', '#', ' ', '#', ' ', '#', '#', '#'},
        {'#', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', '#', ' ', '#'},
        {'#', ' ', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', ' ', '#'},
        {'#', ' ', '#', ' ', '#', '-', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', '#'},
        {'#', ' ', '#', ' ', '#', 'r', '#', '#', '#', ' ', '#', ' ', '#', '#', '#', ' ', '#', ' ', '#', '#', '#', ' ', '#', ' ', '#'},
        {'#', ' ', ' ', ' ', '#', '+', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', '-', ' ', ' ', '#', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', ' ', '#', ' ', '#', '#', '#', '#', '#', '#', '#', '#', '#', ' ', '#', 'r', '#', '#', '#', ' ', '#', '#', '#'},
        {'#', ' ', ' ', '+', 'r', '-', '#', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', '#', '+', ' ', ' ', ' ', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', ' ', '#', ' ', '#', ' ', '#', ' ', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', '#', '#'},
        {'#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', '#'},
        {'#', '#', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', '#', '#', '#', '#', '#', '#', ' ', '#', ' ', '#'},
        {'#', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', ' ', '#', ' ', '#', ' ', '#', ' ', '#', '#', '#'},
        {'#', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', '#', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', ' ', '#'},
        {'#', ' ', '#', ' ', '#', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#'},
        {'#', 'n', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', ' ', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', '#', '#'},
        {'#', '+', 'r', '-', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', ' ', '#', '#', '#'},
        {'#', ' ', '#', '-', 'r', '+', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#'}, //
        {'#', ' ', '#', ' ', '#', ' ', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', ' ', '#'},
        {'#', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', ' ', '#', ' ', '#'},
        {'#', '+', 'r', '-', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', '#'}, //
        {'#', 'n', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#'},
        {'#', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', ' ', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', ' ', '#', ' ', '#', '#', '#'},
        {'#', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', '+', 'r', '-', ' ', ' ', ' ', ' ', '#'},
        {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'}};

    /*
    #: pared
    n: casilla negra
    a: casilla azul
    /: stairs
    b: bumper
    r: rampa
    v: victima
    c: checkpoint
    */
    // vector<vector<char>> maze = {
    //     {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
    //     {'#', ' ', ' ', ' ', '#', '+', ' ', 's', ' ', ' ', '#'},
    //     {'#', ' ', '#', ' ', ' ', 'r', '#', '#', '#', '#', '#'},
    //     {'#', ' ', '#', '#', ' ', '-', ' ', ' ', '#', ' ', '#'},
    //     {'#', ' ', '#', ' ', ' ', 'n', '#', ' ', '#', '2', '#'},
    //     {'#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#'},
    //     {'#', ' ', '#', '#', ' ', ' ', ' ', ' ', ' ', ' ', '#'},
    //     {'#', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', '#'},
    //     {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},

    // };
};

Map::Map()
{
    tile = new Tile(vector<int>{0, 0, 0}, 1, 0);
    xMaze = 3;
    yMaze = 30;
    zMaze = 0;

    xrmaze = 3;
    yrmaze = 30;
    zrmaze = 0;

    pos = {0, 0, 0};
    recovpos = {0, 0, 0};
}

void Map::setRecovPos()
{
    xrmaze = xMaze;
    yrmaze = yMaze;
    zrmaze = zMaze;

    recovpos = pos;
}

void Map::getMazePos(int &x, int &y, int &z, string key)
{
    if (key == "north")
    {
        y--;
    }
    else if (key == "south")
    {
        y++;
    }
    else if (key == "east")
    {
        x++;
    }
    else if (key == "west")
    {
        x--;
    }
}

char Map::getDirChar(int &rDirection)
{
    switch (rDirection)
    {
    case 0:
        return '^';
        break;
    case 1:
        return '>';
        break;
    case 2:
        return 'v';
        break;
    case 3:
        return '<';
        break;
    default:
        return 'o';
        break;
    }
}

void Map::moveMaze(string dir)
{
    if (dir == "north")
    {
        yMaze--;
    }
    else if (dir == "south")
    {
        yMaze++;
    }
    else if (dir == "east")
    {
        xMaze++;
    }
    else if (dir == "west")
    {
        xMaze--;
    }
}

void Map::printMaze(int &rDirection)
{
    for (int j = 0; j < maze.size(); j++)
    {
        for (int i = 0; i < maze[0].size(); i++)
        {
            if (xMaze == i && yMaze == j)
            {
                cout << getDirChar(rDirection);
            }
            else
            {
                cout << maze[j][i];
            }

            cout << " ";
        }
        cout << "\n";
    }
}

// Regresa el caracter en la posicion norte
char Map::getNorth()
{
    if (yMaze - 1 >= 0)
    {
        return maze[yMaze - 1][xMaze];
    }
    else
    {
        return '-';
    }
}

// Regresa el caracter en la posicion sur
char Map::getSouth()
{
    if (yMaze + 1 < maze.size())
    {
        return maze[yMaze + 1][xMaze];
    }
    else
    {
        return '-';
    }
}

// Regresa el caracter en la posicion este
char Map::getEast()
{
    if (xMaze + 1 < maze[0].size())
    {
        return maze[yMaze][xMaze + 1];
    }
    else
    {
        return '-';
    }
}

// Regresa el caracter en la posicion oeste
char Map::getWest()
{
    if (xMaze - 1 >= 0)
    {
        return maze[yMaze][xMaze - 1];
    }
    else
    {
        return '-';
    }
}

// Regresa el caracter en la posicion indicada por key
char Map::getChar(string key)
{
    if (key == "north")
    {
        return getNorth();
    }
    else if (key == "east")
    {
        return getEast();
    }
    else if (key == "south")
    {
        return getSouth();
    }
    else if (key == "west")
    {
        return getWest();
    }

    return '-';
}

// // Regresa el caracter en la posicion actual
// char getCurChar()
// {
//     return maze[yMaze][xMaze];
// }

int Map::rampDirection(string key)
{
    if (getChar(key) != 'r')
        return -1;

    int x = xMaze;
    int y = yMaze;
    int z = zMaze;

    getMazePos(x, y, z, key);

    if (maze[y - 1][x] == '-')
    {
        return 2;
    }
    else if (maze[y + 1][x] == '-')
    {
        return 0;
    }
    else if (maze[y][x - 1] == '-')
    {
        return 1;
    }
    else if (maze[y][x + 1] == '-')
    {
        return 3;
    }

    return -1;
}