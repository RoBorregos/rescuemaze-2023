#include <iostream>
#include <vector>
#include <utility>
#include <stack>
#include <cstdlib>
#include <algorithm>
#include <ctype.h>
#include <map>
using namespace std;

#include "Tile.h"
#include "Movement.h"
#include "Maze.h"
#include "Dijkstra.h"

// 0: north, 1: east, 2: south, 3: west
int rDirection = 0;

/*
#: pared
n: casilla negra
a: casilla azul
/: stairs
b: bumper
r: rampa
v: victima
*/
// vector<vector<char>> maze = {
//     {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
//     {'#', ' ', '3', '-', '#', '+', ' ', 's', ' ', ' ', '#'},
//     {'#', ' ', '#', 'r', '#', 'r', '#', '#', '#', '#', '#'},
//     {'#', ' ', '#', '+', '#', '-', ' ', ' ', '#', ' ', '#'},
//     {'#', 'n', '#', ' ', '#', 'n', '#', ' ', '#', '2', '#'},
//     {'#', ' ', ' ', ' ', '-', 'r', '+', ' ', '/', ' ', '#'},
//     {'#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#'},
//     {'#', ' ', 'b', ' ', ' ', ' ', '#', ' ', 'a', ' ', '#'},
//     {'#', '#', '#', ' ', '#', '1', '#', '#', '#', ' ', '#'},
//     {'#', ' ', '1', ' ', '#', ' ', ' ', ' ', '#', ' ', '#'},
//     {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},

// };

string posvectorToString(vector<int> pos)
{
    return to_string(pos[0]) + "," + to_string(pos[1]) + "," + to_string(pos[2]);
}


// Regresa la distancia en cm a la pared en la direccion dada
int getDistance(string key)
{
    if (key == "north")
        rotateTo(rDirection, 0);
    else if (key == "east")
        rotateTo(rDirection, 1);
    else if (key == "south")
        rotateTo(rDirection, 2);
    else if (key == "west")
        rotateTo(rDirection, 3);

    if (getChar(key) == '#')
        return 5;
    else
        return 30;
}

bool isWall(string key)
{
    if (getDistance(key) > 20)
        return false;
    else
        return true;
}

// Regresa true si el caracter en la posicion establecida por key es una rampa
bool isRamp(string key)
{
    if (getChar(key) == 'r')
        return true;
    else
        return false;
}

// Sigue las instrucciones del stack y regresa el tile al que se movio
Tile *followPath(stack<string> &path, Tile *tile, vector<int> &pos)
{
    while (!path.empty())
    {
        tile = move(tile, path.top(), pos, xMaze, yMaze, rDirection);
        cout << path.top() << "\t" << pos[0] << ", " << pos[1] << ", " << pos[2] << endl;
        path.pop();
    }

    return tile;
}

int checkVictims()
{
    return isdigit(maze[yMaze][xMaze]) ? maze[yMaze][xMaze] : 0;
}

Tile *createTile(vector<int> pos, char c, string key, vector<int> &adjPos)
{
    Tile *newTile = new Tile(pos, 1, 0);

    if (c == 'a') // casilla azul
    {
        newTile->weight = 100;
    }
    else if (c == 'b') // bumper
    {
        newTile->weight = 100;
    }
    else if (c == '/') // escaleras
    {
        newTile->weight = 100;
    }
    else if (c == 'r') // rampa
    {
        newTile->weight = 150;
        newTile->rampa = rampDirection(key);
    }
    else if (checkVictims())
    {
        newTile->victim = checkVictims();
    }

    return newTile;
}

// Funcion principal, se encarga de llamar a las funciones necesarias para explorar el laberinto completamente y regresar al inicio
void explore(Tile *tile, vector<Tile *> &unvisited)
{
    Tile *startTile = tile;
    Tile *newTile = nullptr;
    vector<string> keys = {"north", "east", "south", "west"};

    vector<int> pos = {0, 0, 0};
    vector<int> newPos = pos;

    map<string, Tile *> tiles = {{posvectorToString(pos), startTile}};


    char c;

    stack<string> path;

    int steps = 2;

    do
    {
        tile->visited = true;
        for (auto &&key : keys)
        {
            newPos = pos;

            if (tile->adjacentTiles[key] == nullptr && !tile->walls[key])
            {
                c = getChar(key);

                calcPos(newPos, key);

                if (tiles.count(posvectorToString(newPos)))
                {
                    newTile = tiles.at(posvectorToString(newPos)); // Ya existe una tile para esa casilla

                    tile->appendTile(newTile, key);
                }
                else // no existe tile
                {
                    if (isWall(key))
                    {
                        tile->walls[key] = true;
                    }
                    else if (c == 'n')
                    {
                        tile->walls[key] = true;
                    }
                    else
                    {
                        newTile = createTile(newPos, c, key, pos);
                        tile->appendTile(newTile, key);
                        unvisited.push_back(newTile);

                        tiles.insert({posvectorToString(newPos), newTile});
                        if (newTile->rampa != -1)
                        {
                            newPos[2] = pos[2];
                            tiles.insert({posvectorToString(newPos), newTile});
                        }

                        steps = 2;
                    }
                }
            }
        }

        path = bestUnvisited(tile, unvisited, tiles, keys);
        tile = followPath(path, tile, pos);

        printMaze(rDirection);
        system("pause");

        if (unvisited.empty())
        {
            steps--;
        }
        

    } while (steps > 0);

    unvisited.push_back(startTile);
    path = bestUnvisited(tile, unvisited, tiles, keys);
    tile = followPath(path, tile, pos);

    printMaze(rDirection);
}

int main()
{
    vector<Tile *> unvisited;
    Tile *tile = new Tile(vector<int>{0, 0, 0}, 1, 0);

    printMaze(rDirection);
    explore(tile, unvisited);

    return 0;
}
