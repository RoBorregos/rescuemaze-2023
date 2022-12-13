#include <iostream>
#include <vector>
#include <utility>
#include <stack>
#include <cstdlib>
#include <algorithm>
using namespace std;

#include "Tile.h"

int xMaze = 1;
int yMaze = 7;
int zMaze = 1;

/*
#: pared
n: casilla negra
a: casilla azul
/: stairs
b: bumper
r: rampa
v: victima
*/
vector<vector<char>> maze = {
    {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
    {'#', ' ', 'v', ' ', '#', ' ', ' ', 's', ' ', ' ', '#'},
    {'#', ' ', '#', ' ', '#', 'r', '#', '#', '#', '#', '#'},
    {'#', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', '#'},
    {'#', 'n', '#', ' ', '#', 'n', '#', ' ', '#', 'v', '#'},
    {'#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '/', ' ', '#'},
    {'#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#'},
    {'#', ' ', 'b', ' ', ' ', ' ', '#', ' ', 'a', ' ', '#'},
    {'#', '#', '#', ' ', '#', 'v', '#', '#', '#', ' ', '#'},
    {'#', ' ', 'v', ' ', '#', ' ', ' ', ' ', '#', ' ', '#'},
    {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},

};

// void printMaze()
// {
//     for (auto &&i : maze)
//     {
//         for (auto &&c : i)
//         {
//             cout << c << " ";
//         }
//         cout << "\n";
//     }
// }

void printMaze()
{
    for (int j = 0; j < maze.size(); j++)
    {
        for (int i = 0; i < maze[0].size(); i++)
        {
            if (xMaze == i && yMaze == j)
            {
                cout << "o";
                // maze[j][i] = 'o';
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

char getCurChar(int xMaze, int yMaze) //, string dir )
{
    return maze[yMaze][xMaze];
    // if (yMaze < maze.size() && xMaze < maze[yMaze].size())
    // {
    // if (dir == "north" && yMaze - 1 >= 0)
    // {
    //     return maze[yMaze + 1][xMaze];
    // }
    // else if (dir == "south" && yMaze + 1 < maze.size())
    // {
    //     return maze[yMaze + 1][xMaze];
    // }
    // else
    // {
    //     return '-';
    // }

    // }
}

char getNorth(int xMaze, int yMaze)
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
char getSouth(int xMaze, int yMaze)
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
char getEast(int xMaze, int yMaze)
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
char getWest(int xMaze, int yMaze)
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

char getChar(int xMaze, int yMaze, string key)
{
    if (key == "north")
    {
        return getNorth(xMaze, yMaze);
    }
    else if (key == "east")
    {
        return getEast(xMaze, yMaze);
    }
    else if (key == "south")
    {
        return getSouth(xMaze, yMaze);
    }
    else if (key == "west")
    {
        return getWest(xMaze, yMaze);
    }

    return '-';
}

void moveNorth(vector<int> &pos)
{
    yMaze--;
    pos[1]++;
}
void moveSouth(vector<int> &pos)
{
    yMaze++;
    pos[1]--;
}
void moveEast(vector<int> &pos)
{
    xMaze++;
    pos[0]++;
}
void moveWest(vector<int> &pos)
{
    xMaze--;
    pos[0]--;
}

// void move(int &xMaze, int &yMaze, Tile *tile, string key)
Tile* move(Tile *tile, string key, vector<int> &pos)
{
    if (tile->adjacentTiles[key])
    {
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

// void goTo(int &xMaze, int &yMaze, Tile *to) // int xTo, int yTo)
// {
//     // yMaze = to->pos[0];
//     // xMaze = to->pos[1];
// }

void calcPos(vector<int> &pos, string key, char c)
{
    if (c == 'r')
    {
        pos[2] = (pos[2] == 1) ? 0 : 1;
    }

    if (key == "north")
    {
        pos[1]++;
    }
    else if (key == "south")
    {
        pos[1]--;
    }
    else if (key == "east")
    {
        pos[0]++;
    }
    else if (key == "west")
    {
        pos[0]--;
    }

    // return pos;
}

/* void checkTile(Tile *tile, char c, string key, vector<Tile *> &unvisited)
{
    vector<int> pos = tile->pos;
    // vector<int> newPos = pos;

    if (c == '#' || c == '-')
    {
        tile->walls[key] = true;
    }
    else if (c == 'n')
    {
        tile->walls[key] = true;
    }
    else if (c == 'a') // casilla azul
    {
        // tile->adjacentTiles[key] = new Tile(4);
        // tile->appendTile(key, 100, c);

        tile->appendTile(new Tile(calcPos(pos, key, c), 100), key);
        unvisited.push_back(tile->adjacentTiles[key]);
    }
    else if (c == 'b') // bumper
    {
        // tile->adjacentTiles[key] = new Tile(3);
        tile->appendTile(new Tile(calcPos(pos, key, c), 100), key);
        unvisited.push_back(tile->adjacentTiles[key]);
    }
    else if (c == '/') // escaleras
    {
        // tile->adjacentTiles[key] = new Tile(2);
        tile->appendTile(new Tile(calcPos(pos, key, c), 100), key);
        unvisited.push_back(tile->adjacentTiles[key]);
    }
    else if (c == 'r') // rampa
    {
        // tile->adjacentTiles[key] = new Tile(1);
        tile->appendTile(new Tile(calcPos(pos, key, c), 100), key);
        unvisited.push_back(tile->adjacentTiles[key]);
    }
    else if (c == ' ') // casilla libre
    {
        // tile->adjacentTiles[key] = new Tile(0);
        tile->appendTile(new Tile(calcPos(pos, key, c), 1), key);
        unvisited.push_back(tile->adjacentTiles[key]);
    }
} */

Tile* followPath(stack<string> &path, Tile *tile, vector<int> &pos)
{
    while (!path.empty())
    {
        tile = move(tile, path.top(), pos);
        cout << path.top() << endl;
        path.pop();
    }

    return tile;
}

Tile *cheapestUnknown(map<vector<int>, Tile *> &tiles, map<Tile *, bool> &visited, map<Tile *, int> &cost)
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

stack<string> bestUnvisited(Tile *start, vector<Tile *> &unvisited, map<vector<int>, Tile *> tiles, vector<string> &keys)
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
            if (targetTile && !visited[targetTile] && (cost[source] + targetTile->val) < cost[targetTile])
            {
                cost[targetTile] = cost[source] + targetTile->val;
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

    // unvisited.erase(find(unvisited.begin(), unvisited.end(), lowTile));
    unvisited.erase(remove(unvisited.begin(), unvisited.end(), lowTile), unvisited.end());
    if (lowTile->val == -10)
    {
        lowTile->val = 1;
    }    

    return bestPath;
}

void explore(Tile *tile, vector<Tile *> &unvisited)
{
    Tile *startTile = tile;
    Tile *newTile = nullptr;

    vector<int> pos = {0, 0, 0};
    vector<int> newPos = pos;

    map<vector<int>, Tile *> tiles = {{pos, startTile}};

    map<string, string> reverse = {
        {"north", "south"},
        {"east", "west"},
        {"south", "north"},
        {"west", "east"},
    };
    vector<string> keys = {"north", "east", "south", "west"};

    char c;
    // string key;
    // vector<Tile *> q;
    // Tile *tmp;

    stack<string> path;

    int steps = 2;

    do
    {
        tile->visited = true;
        for (auto &&key : keys)
        {
            newPos = pos;

            if (tile->adjacentTiles[key] == nullptr && !tile->walls[key]) // || tile->adjacentTiles[key]->visited == false)
            {
                c = getChar(xMaze, yMaze, key);

                calcPos(newPos, key, c);
                try
                {
                    newTile = tiles.at(newPos); // Ya existe una tile para esa casilla

                    tile->appendTile(newTile, key);
                }
                catch (out_of_range &e) // no existe tile
                {
                    if (c == '#' || c == '-')
                    {
                        tile->walls[key] = true;
                    }
                    else if (c == 'n')
                    {
                        tile->walls[key] = true;
                    }
                    else
                    {
                        newTile = new Tile(newPos, 1);

                        if (c == 'a') // casilla azul
                        {
                            // tile->adjacentTiles[key] = new Tile(4);
                            // tile->appendTile(key, 100, c);

                            newTile->val = 100;
                            // tile->appendTile(newTile, key);
                            // unvisited.push_back(tile->adjacentTiles[key]);
                        }
                        else if (c == 'b') // bumper
                        {
                            newTile->val = 100;
                            // tile->appendTile(newTile, key);
                            // unvisited.push_back(tile->adjacentTiles[key]);
                        }
                        else if (c == '/') // escaleras
                        {
                            newTile->val = 100;
                            // tile->appendTile(newTile, key);
                            // unvisited.push_back(tile->adjacentTiles[key]);
                        }
                        else if (c == 'r') // rampa
                        {
                            newTile->val = 100;
                            // tile->appendTile(newTile, key);
                            // unvisited.push_back(tile->adjacentTiles[key]);
                        }
                        else if (c == 'v')
                        {
                            newTile->val = -10;
                        }
                        // else if (c == ' ') // casilla libre
                        // {
                        // }

                        tile->appendTile(newTile, key);
                        unvisited.push_back(newTile);
                        tiles.insert({newPos, newTile});

                        steps = 2;
                        // checkTile(tile, c, key, unvisited);
                    }
                }
            }
        }

        path = bestUnvisited(tile, unvisited, tiles, keys);
        tile = followPath(path, tile, pos);

        // // move(xMaze, yMaze, tmp.first);
        // // goTo(xMaze, yMaze, tmp);
        // // tile = tmp;

        // tile = unvisited[0];
        // unvisited.erase(unvisited.begin());
        // pos =

        printMaze();
        system("pause");

    } while (steps-- > 0);
    // } while (!unvisited.empty());

    /*     key = "north";

        key = "east";
        if (tile->adjacentTiles[key] != nullptr && tile->adjacentTiles[key]->visited == false)
        {
            c = getEast(maze, xMaze, yMaze);
            checkTile(tile, c, key);
        }

        key = "west";
        if (tile->adjacentTiles[key] != nullptr && tile->adjacentTiles[key]->visited == false)
        {
            c = getWest(maze, xMaze, yMaze);
            checkTile(tile, c, key);
        }

        key = "south";
        if (tile->adjacentTiles[key] != nullptr && tile->adjacentTiles[key]->visited == false)
        {
            c = getSouth(maze, xMaze, yMaze);
            checkTile(tile, c, key);
        }
     */
}

int main()
{
    string rDir = "north";

    vector<Tile *> unvisited;

    Tile *tile = new Tile(vector<int>{0, 0, 0}, 1);

    // printMaze(maze);

    printMaze();

    explore(tile, unvisited);

    return 0;
}
