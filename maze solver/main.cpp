#include <iostream>
#include <vector>
#include <utility>
#include <stack>
#include <cstdlib>
#include <algorithm>
#include <ctype.h>
using namespace std;

#include "Tile.h"

int xMaze = 3;
int yMaze = 30;
int zMaze = 0;

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
    {'#', 'n', '#', 'n', '#', '#', '#', '#', '#', ' ', '#', ' ', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', '#', '#'},
    {'#', '+', 'r', '-', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#'},
    {'#', ' ', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', ' ', '#', '#', '#'},
    {'#', ' ', '#', '-', 'r', '+', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#'}, //
    {'#', ' ', '#', ' ', '#', ' ', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', ' ', '#'},
    {'#', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#'},
    {'#', ' ', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', ' ', '#', ' ', '#'},
    {'#', '+', 'r', '-', 'n', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', '#'}, //
    {'#', 'n', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', '#', '#', ' ', '#'},
    {'#', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', '#'},
    {'#', ' ', '#', ' ', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', ' ', '#', ' ', '#', ' ', '#', '#', '#'},
    {'#', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', '+', 'r', '-', ' ', ' ', ' ', ' ', '#'},
    {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'}};

char getDirChar()
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

void printMaze()
{
    for (int j = 0; j < maze.size(); j++)
    {
        for (int i = 0; i < maze[0].size(); i++)
        {
            if (xMaze == i && yMaze == j)
            {
                cout << getDirChar();
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
char getNorth()
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
char getSouth()
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
char getEast()
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
char getWest()
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
char getChar(string key)
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

// Regresa el caracter en la posicion actual
char getCurChar()
{
    return maze[yMaze][xMaze];
}

// Gira a la izquierda y modifica la direccion
void left()
{
    (rDirection == 0) ? rDirection = 3 : rDirection--;
    cout << "left" << endl;
}

// Gira a la derecha y modifica la direccion
void right()
{
    (rDirection == 3) ? rDirection = 0 : rDirection++;
    cout << "right" << endl;
}

void moveForward()
{
}

string posvectorToString(vector<int> pos)
{
    return to_string(pos[0]) + "," + to_string(pos[1]) + "," + to_string(pos[2]);
}

// Gira a la direccion indicada
void rotateTo(int newDirection)
{
    if (rDirection + 2 == newDirection or rDirection - 2 == newDirection)
    {
        left();
        left();
    }
    else if (rDirection + 1 == newDirection or rDirection - 3 == newDirection)
    {
        right();
    }
    else if (rDirection - 1 == newDirection or rDirection + 3 == newDirection)
    {
        left();
    }
}

// Regresa la distancia en cm a la pared en la direccion dada
int getDistance(string key)
{
    if (key == "north")
        rotateTo(0);
    else if (key == "east")
        rotateTo(1);
    else if (key == "south")
        rotateTo(2);
    else if (key == "west")
        rotateTo(3);

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

void getMazePos(int &x, int &y, int &z, string key)
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

int rampDirection(string key)
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

// Mueve el robot hacia el norte
void moveNorth(vector<int> &pos)
{
    cout << "moveNorth" << endl;

    rotateTo(0);
    moveForward();

    yMaze--;
}

// Mueve el robot hacia el sur
void moveSouth(vector<int> &pos)
{
    cout << "moveSouth" << endl;

    rotateTo(2);
    moveForward();

    yMaze++;
}

// Mueve el robot hacia el este
void moveEast(vector<int> &pos)
{
    cout << "moveEast" << endl;

    rotateTo(1);
    moveForward();

    xMaze++;
}

// Mueve el robot hacia el oeste
void moveWest(vector<int> &pos)
{
    cout << "moveWest" << endl;

    rotateTo(3);
    moveForward();

    xMaze--;
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
Tile *move(Tile *tile, string key, vector<int> &pos)
{
    if (tile->adjacentTiles[key])
    {
        calcPos(pos, key);
        if (key == "north")
        {
            moveNorth(pos);
        }
        else if (key == "east")
        {
            moveEast(pos);
        }
        else if (key == "south")
        {
            moveSouth(pos);
        }
        else if (key == "west")
        {
            moveWest(pos);
        }

        return tile->adjacentTiles[key];
    }

    return nullptr;
}

// Sigue las instrucciones del stack y regresa el tile al que se movio
Tile *followPath(stack<string> &path, Tile *tile, vector<int> &pos)
{
    while (!path.empty())
    {
        tile = move(tile, path.top(), pos);
        cout << path.top() << "\t" << pos[0] << ", " << pos[1] << ", " << pos[2] << endl;
        path.pop();
    }

    return tile;
}

// Parte del algoritmo de Dijkstra: Regresa el tile con el costo mas bajo que no se ha visitado
Tile *cheapestUnknown(map<string, Tile *> &tiles, map<Tile *, bool> &visited, map<Tile *, int> &cost)
{
    int curCost = INT_MAX;
    Tile *curTile = nullptr;

    for (auto &&p : tiles)
    {
        if (!visited[p.second] && cost[p.second] < curCost)
        {
            curCost = cost[p.second];
            curTile = p.second;
        }
    }

    return curTile;
}

// Algoritmo de Dijkstra: Determina la tile no visitada con el menor costo para ir y regresa el camino mas corto desde el tile de inicio hasta esa tile
stack<string> bestUnvisited(Tile *start, vector<Tile *> &unvisited, map<string, Tile *> tiles, vector<string> &keys)
{
    // Tile *tile = start;
    Tile *targetTile;
    stack<string> bestPath;

    map<Tile *, bool> visited;
    map<Tile *, int> cost;
    map<Tile *, pair<Tile *, string>> paths;

    for (auto &&p : tiles)
    {
        visited[p.second] = false;
        cost[p.second] = INT_MAX;
        paths[p.second] = pair<Tile *, string>{nullptr, ""};
    }

    Tile *source = start;
    vector<int> sourcePos = start->pos;
    cost[start] = 0;

    int lowestCost = INT_MAX;
    Tile *lowTile = nullptr;

    while (source)
    {
        visited[source] = true;

        for (auto &&key : keys)
        {
            targetTile = source->adjacentTiles[key];
            if (targetTile && !visited[targetTile] && (cost[source] + targetTile->weight) < cost[targetTile])
            {
                cost[targetTile] = cost[source] + targetTile->weight;
                paths[targetTile] = pair<Tile *, string>{source, key};

                if (cost[targetTile] < lowestCost && find(unvisited.begin(), unvisited.end(), targetTile) != unvisited.end())
                {
                    lowestCost = cost[targetTile];
                    lowTile = targetTile;
                }
            }
        }

        source = cheapestUnknown(tiles, visited, cost);
    }

    pair<Tile *, string> path = paths[lowTile];
    while (path.first)
    {
        bestPath.push(path.second);
        path = paths[path.first];
    }

    unvisited.erase(remove(unvisited.begin(), unvisited.end(), lowTile), unvisited.end());

    if (lowTile)
    {
        if (lowTile->weight == -10)
        {
            lowTile->weight = 1;
        }
    }
    else
    {
        cout << "No hay casillas sin visitar" << endl;
    }

    return bestPath;
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

    vector<int> pos = {0, 0, 0};
    vector<int> newPos = pos;

    map<string, Tile *> tiles = {{posvectorToString(pos), startTile}};

    vector<string> keys = {"north", "east", "south", "west"};

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

        printMaze();
        system("pause");

        if (unvisited.empty())
        {
            steps--;
        }
        

    } while (steps > 0);

    unvisited.push_back(startTile);
    path = bestUnvisited(tile, unvisited, tiles, keys);
    tile = followPath(path, tile, pos);

    printMaze();
}

int main()
{
    vector<Tile *> unvisited;
    Tile *tile = new Tile(vector<int>{0, 0, 0}, 1, 0);

    printMaze();
    explore(tile, unvisited);

    return 0;
}
