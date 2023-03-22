#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <utility>
#include <stack>
// #include <cstdlib>
#include <algorithm>
#include <ctype.h>
#include <map>
#include <fstream>
#include <string>
// #include <boost/serialization>

using namespace std;

#include "ros/callback_queue.h"
#include "exploration/Trigger.h"

// Move Base
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

#include "Tile.h"
// #include "Movement.h"
#include "Map.h"
#include "Dijkstra.h"
#include "ROSbridge.h"

// 0: north, 1: east, 2: south, 3: west
int rDirection = 0;

#define mapSimDebug false
#define useros true
#define rosDebug true
#define canMoveBackward false

ROSbridge *bridge;

map<string, int> directions = {
    {"north", 0},
    {"east", 1},
    {"south", 2},
    {"west", 3}};

map<int, string> invDirections = {
    {0, "north"},
    {1, "east"},
    {2, "south"},
    {3, "west"}};

void moveForward(int &rDirection, Map &mapa)
{
    // cout << "forward" << endl;
    ROS_INFO("Move: ");
    ROS_INFO("         forward");

    bridge->sendUnitGoal(0, rDirection);

    if (mapSimDebug)
    {
        mapa.printMaze(rDirection);
        // cin.get();
        ros::Duration(0.1).sleep();
    }
}

// Gira a la derecha y modifica la direccion
void right(int &rDirection, Map &mapa)
{
    (rDirection == 3) ? rDirection = 0 : rDirection++;
    // cout << "right" << endl;
    ROS_INFO("right");

    bridge->sendUnitGoal(1, rDirection);

    if (mapSimDebug)
    {
        mapa.printMaze(rDirection);
        // cin.get();
        ros::Duration(0.1).sleep();
    }
}

void moveBackward(int &rDirection, Map &mapa)
{
    // cout << "forward" << endl;
    ROS_INFO("Move: ");
    ROS_INFO("       backward");

    if (canMoveBackward)
        bridge->sendUnitGoal(2, rDirection);

    if (mapSimDebug)
    {
        mapa.printMaze(rDirection);
        // cin.get();
        ros::Duration(0.1).sleep();
    }
}

// Gira a la izquierda y modifica la direccion
void left(int &rDirection, Map &mapa)
{
    (rDirection == 0) ? rDirection = 3 : rDirection--;
    // cout << "left" << endl;
    ROS_INFO(" Move: ");
    ROS_INFO("      left");

    bridge->sendUnitGoal(3, rDirection);

    if (mapSimDebug)
    {
        mapa.printMaze(rDirection);
        // cin.get();
        ros::Duration(0.1).sleep();
    }
}

// Gira a la direccion indicada
void rotateTo(int &rDirection, int newDirection, Map &mapa)
{
    // ROS_INFO_STREAM("rotateTo: " << newDirection);
    if (rDirection + 2 == newDirection or rDirection - 2 == newDirection)
    {
        left(rDirection, mapa);
        left(rDirection, mapa);
    }
    else if (rDirection + 1 == newDirection or rDirection - 3 == newDirection)
    {
        right(rDirection, mapa);
    }
    else if (rDirection - 1 == newDirection or rDirection + 3 == newDirection)
    {
        left(rDirection, mapa);
    }
}

// Mueve el robot hacia el norte
void moveNorth(int &yMaze, int &rDirection, Map &mapa)
{
    // cout << "moveNorth" << endl;

    if (canMoveBackward && rDirection == 2)
        moveBackward(rDirection, mapa);

    rotateTo(rDirection, 0, mapa);
    moveForward(rDirection, mapa);
}

// Mueve el robot hacia el sur
void moveSouth(int &yMaze, int &rDirection, Map &mapa)
{
    // cout << "moveSouth" << endl;

    if (canMoveBackward && rDirection == 0)
        moveBackward(rDirection, mapa);

    rotateTo(rDirection, 2, mapa);
    moveForward(rDirection, mapa);
}

// Mueve el robot hacia el este
void moveEast(int &xMaze, int &rDirection, Map &mapa)
{
    // cout << "moveEast" << endl;

    if (canMoveBackward && rDirection == 3)
        moveBackward(rDirection, mapa);

    rotateTo(rDirection, 1, mapa);
    moveForward(rDirection, mapa);
}

// Mueve el robot hacia el oeste
void moveWest(int &xMaze, int &rDirection, Map &mapa)
{
    // cout << "moveWest" << endl;

    if (canMoveBackward && rDirection == 1)
        moveBackward(rDirection, mapa);

    rotateTo(rDirection, 3, mapa);
    moveForward(rDirection, mapa);
}

// Calcula la posicion en la que estaria el robot al moverse a la direccion indicada por key
void calcPos(vector<int> &pos, string key, Map &mapa)
{
    int rampDir;
    if (!useros)
        rampDir = mapa.rampDirection(key);
    else
        rampDir = -1;

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
Tile *move(Tile *tile, string key, int &xMaze, int &yMaze, int &rDirection, Map &mapa)
{
    if (tile->adjacentTiles[key])
    {

        if (key == "north")
        {
            moveNorth(yMaze, rDirection, mapa);
        }
        else if (key == "east")
        {
            moveEast(xMaze, rDirection, mapa);
        }
        else if (key == "south")
        {
            moveSouth(yMaze, rDirection, mapa);
        }
        else if (key == "west")
        {
            moveWest(xMaze, rDirection, mapa);
        }

        if (mapa.pos[2] != tile->adjacentTiles[key]->pos[2])
        {
            bridge->clearMap();
            // ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("reset_map");
        }

        calcPos(mapa.pos, key, mapa);

        if (!useros)
            mapa.moveMaze(key);

        return tile->adjacentTiles[key];
    }

    return nullptr;
}

// string posvectorToString(vector<int> pos)
// {
//     return to_string(pos[0]) + "," + to_string(pos[1]) + "," + to_string(pos[2]);
// }

string stackToString(stack<string> s)
{
    string str = "";
    while (!s.empty())
    {
        str += s.top() + " ";
        s.pop();
    }
    return str;
}

/* // Regresa la distancia en cm a la pared en la direccion dada
int getDistance(string key, Map &mapa)
{
    if (mapa.getChar(key) == '#')
        return 5;
    else
        return 30;
} */

bool isWall(string key) // use ros
{
    int index = (directions[key] - rDirection); // Correct according to robot direction

    while (index < 0)
        index += 4;

    if (bridge->getWalls()[index] == 0)
        return true;

    return false;
}

bool isWall(string key, Map &mapa)
{
    if (mapa.getChar(key) == '#')
        return true;

    return false;
}

// Regresa true si el caracter en la posicion establecida por key es una rampa
bool isRamp() // use ros
{
    // TODO: Implement ros ramp detection
    return false;
}
bool isRamp(string key, Map &mapa)
{
    if (mapa.getChar(key) == 'r') // no ros
        return true;

    return false;
}

// Sigue las instrucciones del stack y regresa el tile al que se movio
Tile *followPath(stack<string> &path, Tile *tile, Map &mapa)
{
    while (!path.empty())
    {
        tile = move(tile, path.top(), mapa.xMaze, mapa.yMaze, rDirection, mapa);
        // cout << path.top() << "\t" << mapa.pos[0] << ", " << mapa.pos[1] << ", " << mapa.pos[2] << endl;
        // ROS_INFO("Move to the: %s", path.top().c_str());
        bridge->pubDebug("Move to the: " + path.top());
        path.pop();
    }

    return tile;
}

int checkVictims() // use ros
{
    return bridge->getVictims();
}

int checkVictims(Map &mapa)
{
    if (!useros)
        return isdigit(mapa.maze[mapa.yMaze][mapa.xMaze]) ? mapa.maze[mapa.yMaze][mapa.xMaze] : 0;

    return 0;
}

void defineTile(Tile *tile)
{
    if (bridge->tcsdata == 'a')
    {
        tile->weight = 100;
    }
    else if (false or false) // TODO: Implement bumper and stairs detection
    {
        tile->weight = 100;
    }
    else if (isRamp())
    {
        tile->weight = 150;
    }
    else if (checkVictims())
    {
        tile->victim = checkVictims();
    }
}

// createTile ros
Tile *createTile(vector<int> pos, string key, vector<int> &adjPos, Map &mapa)
{
    Tile *newTile = new Tile();
    newTile->pos = pos;
    newTile->weight = 1;

    /*     else if (useros && isRamp(key) || !useros && isRamp(key, mapa))
        {
            newTile->weight = 150;
        }
        else if (useros && checkVictims())
        {
            newTile->victim = checkVictims();
        }
        else if (!useros && checkVictims(mapa))
        {
            newTile->victim = checkVictims(mapa);
        }
     */
    return newTile;
}

Tile *createTile(vector<int> pos, char c, string key, vector<int> &adjPos, Map &mapa)
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
        newTile->rampa = mapa.rampDirection(key);
    }
    else if (c == 'c') // checkpoint
    {
    }
    else if (checkVictims(mapa))
    {
        newTile->victim = checkVictims(mapa);
    }

    return newTile;
}

int maxX = 0, minX = 0;
int maxY = 0, minY = 0;

void updateLimitsMap(const vector<int> &pos)
{
    if (pos[0] > maxX)
    {
        maxX = pos[0];
    }
    else if (pos[0] < minX)
    {
        minX = pos[0];
    }

    if (pos[1] > maxY)
    {
        maxY = pos[1];
    }
    else if (pos[1] < minY)
    {
        minY = pos[1];
    }
}

void checkOverwritten(const vector<vector<char>> &maze, char unknown, int indexX, int indexY)
{
    if (maze[indexY][indexX] != unknown)
    {
        cout << "Warning: maze[" << indexY << "][" << indexX << "] has been overwritten.\n";
    }
}

char getDir(int rDirection)
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

void printMapRos(const Map &map)
{
    // Create matrix to store data
    vector<vector<char>> maze;

    int width = 20;
    int height = 20;

    int z = 0; // Print only one level of the map.

    vector<char> row(width);

    char unknown = 'U'; // Represents unknown data.

    for (int i = 0; i < row.size(); i++)
        row[i] = unknown; // Character for unknown data.

    for (int i = 0; i < height; i++)
        maze.push_back(row);

    // Find all the visited tiles and mark their walls. Alternative approach: dfs/bfs to print map.
    for (int i = minX; i <= maxX; i++)
    {
        for (int j = minY; j <= maxY; j++)
        {
            vector<int> pos{i, j, z};
            if (map.tiles.count(posvectorToString(pos)))
            {
                int indexX = i + width/2;
                int indexY = height/2 - j;

                checkOverwritten(maze, unknown, indexX, indexY);
                maze[indexY][indexX] = ' '; // Free space

                // Save tile's walls. Assume valid indices because width and height are assumed to be greater.
                Tile *locatedTile = map.tiles.at(posvectorToString(pos));

                if (rosDebug)
                    cout << "Tile counted at: " << locatedTile->pos[0] << ", " << locatedTile->pos[1] << ", " << locatedTile->pos[2] << '\n';

                if (locatedTile->walls["north"])
                {
                    checkOverwritten(maze, unknown, indexX, indexY - 1);
                    maze[indexY - 1][indexX] = '#'; // Wall
                }
                if (locatedTile->walls["south"])
                {
                    checkOverwritten(maze, unknown, indexX, indexY + 1);
                    maze[indexY + 1][indexX] = '#';
                }
                if (locatedTile->walls["east"])
                {
                    checkOverwritten(maze, unknown, indexX + 1, indexY);
                    maze[indexY][indexX + 1] = '#';
                }
                if (locatedTile->walls["west"])
                {
                    checkOverwritten(maze, unknown, indexX - 1, indexY);
                    maze[indexY][indexX - 1] = '#';
                }
            }
        }
    }

    // Mark origin. TODO: Check why origin tile isn't added to map.tiles
    maze[height/2][width/2] = 'o';

    // Add current location
    maze[height/2 - map.pos[1]][map.pos[0] + width/2] = getDir(rDirection);

    // Print map data
    if (rosDebug)
    {
        cout << "Maze of height " << height << " and width " << width << "\n";
        cout << "MinX: " << minX << ", MaxX: " << maxX << '\n';
        cout << "MinY: " << minY << ", MaxY: " << maxY << '\n';
    }

    // Print map
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            cout << maze[i][j];
        }
        cout << '\n';
    }
}

void printTile(Tile *tile)
{
    bridge->pubDebug(" ");
    bridge->pubDebug("Tile: " + posvectorToString(tile->pos) + "       Adjacent Tiles:");

    for (auto i : tile->adjacentTiles)
    {
        if (!i.second)
            continue;

        bridge->pubDebug("    Tile: " + i.first + ", " + to_string(i.second->pos[0]) + ", " + to_string(i.second->pos[1]) + ", " + to_string(i.second->pos[2]));
        // ROS_INFO("Tile: %s, %d, %d, %d", i.first.c_str(), i.second->pos[0], i.second->pos[1], i.second->pos[2]);
    }
}

void printMap(Map &mapa)
{
    ROS_INFO("xrmaze: %d, yrmaze: %d, zrmaze: %d", mapa.xMaze, mapa.yMaze, mapa.zMaze);
    ROS_INFO("unvisited: ");
    for (auto i : mapa.unvisited)
    {
        ROS_INFO("pos: %d, %d, %d", i->pos[0], i->pos[1], i->pos[2]);
    }

    ROS_INFO("Tile Pos: %d, %d, %d", mapa.tile->pos[0], mapa.tile->pos[1], mapa.tile->pos[2]);

    ROS_INFO("recovpos: %d, %d, %d", mapa.recovpos[0], mapa.recovpos[1], mapa.recovpos[2]);
}

// Funcion principal, se encarga de llamar a las funciones necesarias para explorar el laberinto completamente y regresar al inicio
void explore(bool checkpoint, int argc, char **argv)
{
    Map mapa;

    if (checkpoint)
    {
        ROS_INFO("Checkpoint");

        // {
        std::ifstream ifs("./checkpoint.txt");
        boost::archive::text_iarchive ia(ifs);
        ia >> mapa;

        printMap(mapa);
        cin.get();
        //     ia >> mapa;
        // }
    }
    else
    {
        ROS_INFO("No checkpoint");
    }

    // mapa.printMaze(rDirection);

    // ros::Duration(3).sleep();
    Tile *startTile = mapa.tile;
    mapa.tiles.insert({posvectorToString(mapa.tile->pos), mapa.tile});
    Tile *newTile = nullptr;
    vector<string> keys = {"north", "east", "south", "west"};

    vector<int> newPos = mapa.pos;

    ROS_INFO("newPos created");

    char c;

    stack<string> path;

    int steps = 2;

    do
    {
        // ROS_INFO("new do while loop iteration");

        bridge->pubDebug(" ");
        bridge->pubDebug("new do while loop iteration");

        mapa.tile->visited = true;
        for (auto &&key : keys)
        {
            bridge->pubDebug("In position: " + posvectorToString(mapa.pos));
            bridge->pubDebug("Unvisited tiles: " + to_string(mapa.unvisited.size()));
            // bridge->pubDebug("Checking key: " + key);

            if (mapa.tile && rosDebug)
            {
                bridge->pubDebug("Tile: " + posvectorToString(mapa.tile->pos));
                printTile(mapa.tile);

            }

            // ROS_INFO("Checking key: %s", key.c_str());
            bridge->pubDebug("Checking key: " + key);

            newPos = mapa.pos;

            if (mapa.tile->adjacentTiles[key] == nullptr && !mapa.tile->walls[key])
            {
                if (!useros)
                    c = mapa.getChar(key);

                calcPos(newPos, key, mapa);

                updateLimitsMap(newPos); // Expand map to print

                if (mapa.tiles.count(posvectorToString(newPos)))
                {

                    newTile = mapa.tiles.at(posvectorToString(newPos)); // Ya existe una tile para esa casilla

                    mapa.tile->appendTile(newTile, key);
                }
                else // no existe tile
                {

                    if (useros && isWall(key))
                    {
                        // ROS_INFO("Wall detected");
                        bridge->pubDebug("Wall in " + key);
                        mapa.tile->walls[key] = true;
                    }
                    else if (!useros && isWall(key, mapa))
                    {
                        // ROS_INFO("Wall detected");
                        mapa.tile->walls[key] = true;
                    }
                    else if (useros && bridge->tcsdata == 'n' || !useros && c == 'n') // TODO: Check what initial corresponds to the color black
                    {
                        // ROS_INFO("Black tile detected");
                        mapa.tile->walls[key] = true;
                    }
                    else
                    {
                        bridge->pubDebug("Creating new tile in position: " + posvectorToString(newPos) + " with key: " + key);
                        ROS_INFO("Creating new tile");
                        if (useros)
                        {
                            newTile = createTile(newPos, key, mapa.pos, mapa);
                        }
                        else
                        {
                            newTile = createTile(newPos, c, key, mapa.pos, mapa);
                        }
                        mapa.tile->appendTile(newTile, key);
                        mapa.unvisited.push_back(newTile);

                        mapa.tiles.insert({posvectorToString(newPos), newTile});

                        if (newTile->rampa != -1)
                        {
                            newPos[2] = mapa.pos[2];
                            mapa.tiles.insert({posvectorToString(newPos), newTile});
                        }

                        steps = 2;
                    }
                }
            }
        }

        {
            std::ofstream ofs("./checkpoint.txt");
            boost::archive::text_oarchive oa(ofs);
            oa << mapa;

            ROS_INFO("Checkpoint saved");
        }

        bridge->pubDebug("Unvisited tiles: " + to_string(mapa.unvisited.size()));

        bridge->pubDebug("Current tile: " + posvectorToString(mapa.tile->pos));
        printTile(mapa.tile);
        path = bestUnvisited(mapa.tile, mapa.unvisited, mapa.tiles, keys);

        if (path.empty())
        {
            // ROS_INFO("No unvisited tiles");
            bridge->pubDebug("No unvisited tiles");
            // break;
        }
        else
        {
            // ROS_INFO("Unvisited tiles");
            bridge->pubDebug("Unvisited tiles");
            bridge->pubDebug(stackToString(path));
        }

        mapa.tile = followPath(path, mapa.tile, mapa);

        bridge->pubDebug("New tile: " + posvectorToString(mapa.tile->pos));
        printTile(mapa.tile);

        if (mapa.tile->victim != 0)
        {
            ROS_INFO("Victim found");
            bridge->sendKit();
            mapa.tile->victim = 0;
        }

        if (mapSimDebug)
            mapa.printMaze(rDirection);
        // system("pause");
        // cin.get();

        if (useros)
            printMapRos(mapa);

        if (mapa.unvisited.empty())
        {
            steps--;
        }

    } while (ros::ok() && steps > 0);

    mapa.unvisited.push_back(startTile);
    path = bestUnvisited(mapa.tile, mapa.unvisited, mapa.tiles, keys);
    mapa.tile = followPath(path, mapa.tile, mapa);

    // mapa.printMaze(rDirection);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mainAlgorithm");
    ros::NodeHandle *n = new ros::NodeHandle();

    bridge = new ROSbridge(n);

    bridge->pubDebug("Starting main algorithm");

    ros::Duration(1.5).sleep();

    try
    {
        if (bridge->tcsdata == 1) // Checkpoint
        {
            explore(false, argc, argv);
            // explore(true, argc, argv);
        }
        else
        {
            explore(false, argc, argv);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    // ros::ServiceServer service = n.advertiseService("trigger", sendInstruction);
    // ROS_INFO("Ready to send instruction.");

    // curInstruction = "";

    /*     vector<Tile *> unvisited;
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

        printMaze(rDirection); */

    return 0;
}
