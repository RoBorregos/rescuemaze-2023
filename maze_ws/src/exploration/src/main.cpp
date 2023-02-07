#include <iostream>
#include <vector>
#include <utility>
#include <stack>
// #include <cstdlib>
#include <algorithm>
#include <ctype.h>
#include <map>
using namespace std;

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "exploration/Trigger.h"
#include "Tile.h"
// #include "Movement.h"
#include "Maze.h"
#include "Dijkstra.h"

// 0: north, 1: east, 2: south, 3: west
int rDirection = 0;

string curInstruction = "";
ros::CallbackQueue callQueue;

#define DEBUG true

bool sendInstruction(exploration::Trigger::Request &req, exploration::Trigger::Response &res)
{
    res.message = curInstruction;
    // ROS_INFO_STREAM("Sending instruction: " << curInstruction);
    // ROS_INFO(curInstruction.c_str());

    curInstruction = "";

    return true;
}

void setInstruction(string instruction)
{
    // ROS_INFO_STREAM("setInstruction: " << instruction);
    curInstruction = instruction;

    // while (ros::ok() && callQueue.isEmpty())
    // {
    //     ros::Duration(0.2).sleep();
    //     ROS_INFO("Waiting for request");
    // }
    
    // if (callQueue.isEmpty())
    //     ROS_INFO("no service calls on queue");
    
    ros::Duration(1.5).sleep();
    
    // if (callQueue.isEmpty())
    //     ROS_INFO("no service calls on queue");
    
    // callQueue.callOne(ros::WallDuration(5.0));
    ros::spinOnce();
}

void moveForward(int &rDirection)
{
    // cout << "forward" << endl;
    ROS_INFO("moveForward");

    setInstruction("forward");

    if (DEBUG)
    {
        printMaze(rDirection);
    }
}

// Gira a la izquierda y modifica la direccion
void left(int &rDirection)
{
    (rDirection == 0) ? rDirection = 3 : rDirection--;
    // cout << "left" << endl;
    ROS_INFO("left");

    setInstruction("left");

    if (DEBUG)
    {
        printMaze(rDirection);
    }
}

// Gira a la derecha y modifica la direccion
void right(int &rDirection)
{
    (rDirection == 3) ? rDirection = 0 : rDirection++;
    // cout << "right" << endl;
    ROS_INFO("right");

    setInstruction("right");

    if (DEBUG)
    {
        printMaze(rDirection);
    }
}

// Gira a la direccion indicada
void rotateTo(int &rDirection, int newDirection)
{
    // ROS_INFO_STREAM("rotateTo: " << newDirection);
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
    // cout << "moveNorth" << endl;

    rotateTo(rDirection, 0);
    moveForward(rDirection);

    // yMaze--;
}

// Mueve el robot hacia el sur
void moveSouth(vector<int> &pos, int &yMaze, int &rDirection)
{
    // cout << "moveSouth" << endl;

    rotateTo(rDirection, 2);
    moveForward(rDirection);

    // yMaze++;
}

// Mueve el robot hacia el este
void moveEast(vector<int> &pos, int &xMaze, int &rDirection)
{
    // cout << "moveEast" << endl;

    rotateTo(rDirection, 1);
    moveForward(rDirection);

    // xMaze++;
}

// Mueve el robot hacia el oeste
void moveWest(vector<int> &pos, int &xMaze, int &rDirection)
{
    // cout << "moveWest" << endl;

    rotateTo(rDirection, 3);
    moveForward(rDirection);

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

string posvectorToString(vector<int> pos)
{
    return to_string(pos[0]) + "," + to_string(pos[1]) + "," + to_string(pos[2]);
}

// Regresa la distancia en cm a la pared en la direccion dada
int getDistance(string key)
{
    // ROS_INFO_STREAM("getDistance: " << key);
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
    // ROS_INFO_STREAM("isWall: " << key);
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

/* // Funcion principal, se encarga de llamar a las funciones necesarias para explorar el laberinto completamente y regresar al inicio
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
                    if (isWall(key, server))
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
        tile = followPath(path, tile, pos, server);

        printMaze(rDirection);
        // system("pause");
        cin.get();

        if (unvisited.empty())
        {
            steps--;
        }
        

    } while (steps > 0);

    unvisited.push_back(startTile);
    path = bestUnvisited(tile, unvisited, tiles, keys);
    tile = followPath(path, tile, pos, server);

    printMaze(rDirection);
}
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "instructionServer");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("trigger", sendInstruction);
    ROS_INFO("Ready to send instruction.");

    curInstruction = "";

    vector<Tile *> unvisited;
    Tile *tile = new Tile(vector<int>{0, 0, 0}, 1, 0);

    // InstructionServer server(argc, argv);

    printMaze(rDirection);
    // explore(tile, unvisited, server);

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

        if (DEBUG)
            printMaze(rDirection);
        // system("pause");
        // cin.get();

        if (unvisited.empty())
        {
            steps--;
        }
        

    } while (ros::ok() && steps > 0);

    unvisited.push_back(startTile);
    path = bestUnvisited(tile, unvisited, tiles, keys);
    tile = followPath(path, tile, pos);

    printMaze(rDirection);

    return 0;
}
