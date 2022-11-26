#include <iostream>
#include <vector>
#include <utility>
using namespace std;

#include "Tile.h"

void printMaze(vector<vector<char>> &maze)
{
    for (auto &&i : maze)
    {
        for (auto &&c : i)
        {
            cout << c << " ";
        }
        cout << "\n";
    }
}
void printMaze(vector<vector<char>> &maze, int xMaze, int yMaze)
{
    for (int j = 0; j < maze.size(); j++)
    {
        for (int i = 0; i < maze[0].size(); i++)
        {
            if (xMaze == i && yMaze == j)
            {
                cout << "o";
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

char getCurChar(vector<vector<char>> &maze, int xMaze, int yMaze) //, string dir )
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

char getNorth(vector<vector<char>> &maze, int xMaze, int yMaze)
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
char getSouth(vector<vector<char>> &maze, int xMaze, int yMaze)
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
char getEast(vector<vector<char>> &maze, int xMaze, int yMaze)
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
char getWest(vector<vector<char>> &maze, int xMaze, int yMaze)
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

char getChar(vector<vector<char>> &maze, int xMaze, int yMaze, string key)
{
    if (key == "north")
    {
        return getNorth(maze, xMaze, yMaze);
    }
    else if (key == "east")
    {
        return getEast(maze, xMaze, yMaze);
    }
    else if (key == "south")
    {
        return getSouth(maze, xMaze, yMaze);
    }
    else if (key == "west")
    {
        return getWest(maze, xMaze, yMaze);
    }

    return '-';
}

void moveNorth(int &yMaze) { yMaze--; }
void moveSouth(int &yMaze) { yMaze++; }
void moveEast(int &xMaze) { xMaze++; }
void moveWest(int &xMaze) { xMaze--; }

void move(int &xMaze, int &yMaze, string key)
{
    if (key == "north")
    {
        moveNorth(yMaze);
    }
    else if (key == "east")
    {
        moveEast(xMaze);
    }
    else if (key == "south")
    {
        moveSouth(yMaze);
    }
    else if (key == "west")
    {
        moveWest(xMaze);
    }
}

void goTo(int &xMaze, int &yMaze, Tile* to)//int xTo, int yTo)
{
    yMaze = to->val[0];
    xMaze = to->val[1];
}

void checkTile(Tile *tile, char c, string key, vector<pair<string, Tile *>> &q)
{
    if (c == '#' || c == '-')
    {
        tile->walls[key] = true;
    }
    else if (c == 'n')
    {
        tile->walls[key] = true;
    }
    else if (c == 'a')
    {
        // tile->adjacentTiles[key] = new Tile(4);
        tile->appendTile(new Tile(), key);
        q.push_back(pair(key, tile->adjacentTiles[key].first));
    }
    else if (c == 'b')
    {
        // tile->adjacentTiles[key] = new Tile(3);
        tile->appendTile(new Tile(3), key);
        q.push_back(pair(key, tile->adjacentTiles[key].first));
    }
    else if (c == 's')
    {
        // tile->adjacentTiles[key] = new Tile(2);
        tile->appendTile(new Tile(2), key);
        q.push_back(pair(key, tile->adjacentTiles[key].first));
    }
    else if (c == 'r')
    {
        // tile->adjacentTiles[key] = new Tile(1);
        tile->appendTile(new Tile(1), key);
        q.push_back(pair(key, tile->adjacentTiles[key].first));
    }
    else if (c == ' ')
    {
        // tile->adjacentTiles[key] = new Tile(0);
        tile->appendTile(new Tile(0), key);
        q.insert(q.begin(), pair(key, tile->adjacentTiles[key].first));
    }
}

void explore(Tile *tile, int xMaze, int yMaze, vector<vector<char>> &maze, vector<string> &keys)
{
    map<string, string> reverse = {
        {"north", "south"},
        {"east", "west"},
        {"south", "north"},
        {"west", "east"},
    };

    tile->visited = true;
    char c;
    // string key;
    // keys = {"north", "east", "south", "west"};
    vector<pair<string, Tile *>> q;
    pair<string, Tile *> tmp;

    do
    {
        for (auto &&key : keys)
        {
            if (tile->adjacentTiles[key].first == nullptr || tile->adjacentTiles[key].first->visited == false)
            {
                c = getChar(maze, xMaze, yMaze, key);
                checkTile(tile, c, key, q);
            }
        }

        tmp = q.front();
        q.erase(q.begin());

        // move(xMaze, yMaze, tmp.first);
        goTo(xMaze, yMaze, tmp.second);
        tile = tmp.second;

        printMaze(maze, xMaze, yMaze);
    } while (!q.empty());

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


    vector<vector<char>> maze = {
        {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
        {'#', ' ', ' ', ' ', '#', ' ', ' ', 's', ' ', ' ', '#'},
        {'#', ' ', '#', ' ', '#', ' ', '#', '#', '#', '#', '#'},
        {'#', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', '#'},
        {'#', 'n', '#', ' ', '#', 'n', '#', ' ', '#', ' ', '#'},
        {'#', ' ', ' ', ' ', 'a', ' ', ' ', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', '#', '#', '#', '#', ' ', '#', '#', '#'},
        {'#', ' ', 'b', ' ', 'r', ' ', '#', ' ', 'a', ' ', '#'},
        {'#', '#', '#', ' ', '#', ' ', '#', '#', '#', ' ', '#'},
        {'#', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', '#'},
        {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},

    };

    int xMaze = 2;
    int yMaze = 1;
    int zMaze = 1;
    string rDir = "north";
    vector<string> keys = {"north", "east", "south", "west"};

    Tile *tile = new Tile(0);

    // printMaze(maze);

    printMaze(maze, xMaze, yMaze);

    explore(tile, xMaze, yMaze, maze, keys);

    return 0;
}
