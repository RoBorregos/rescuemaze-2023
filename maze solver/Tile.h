#pragma once

#include <map>
#include <string>
#include <vector>
using namespace std;

class Tile
{
private:
public:
    // 0: normal tile
    // 1: ramp
    // 2: stairs
    // 3: bumper
    // 4: blue tile
    // -1: black tile
    int val;
    vector<int> pos;

    map<string, pair<Tile *, int>> adjacentTiles;
    // map<string, int> weights;

    map<string, bool> walls;

    bool visited;

    Tile();
    Tile(int val, int x, int y);

    void addNorth(Tile *t);
    void addEast(Tile *t);
    void addSouth(Tile *t);
    void addWest(Tile *t);

    void appendTile(Tile *, string);
};

Tile::Tile()
{
    adjacentTiles = {
        {"north", pair(nullptr, INT_MAX)},
        {"east", pair(nullptr, INT_MAX)},
        {"south", pair(nullptr, INT_MAX)},
        {"west", pair(nullptr, INT_MAX)}};

    // val = 0;

    // weights = {
    //     {"north", 0},
    //     {"east", 0},
    //     {"south", 0},
    //     {"west", 0}};

    walls = {
        {"north", false},
        {"east", false},
        {"south", false},
        {"west", false}};

    visited = false;
}

Tile::Tile(int x, int y)
{
    pos = {x, y};

    adjacentTiles = {
        {"north", pair(nullptr, INT_MAX)},
        {"east", pair(nullptr, INT_MAX)},
        {"south", pair(nullptr, INT_MAX)},
        {"west", pair(nullptr, INT_MAX)}};

    walls = {
        {"north", false},
        {"east", false},
        {"south", false},
        {"west", false}};

    visited = false;

    // weights = {

    // };
}

void Tile::addNorth(Tile *t)
{
    if (adjacentTiles["north"].first == nullptr)
    {
        adjacentTiles["north"].first = t;

        t->addSouth(this);
    }
    if (adjacentTiles["north"].second > t->val)
    {
        adjacentTiles["north"].second = t->val;
    }
}
void Tile::addEast(Tile *t)
{
    if (adjacentTiles["east"].first == nullptr)
    {
        adjacentTiles["east"].first = t;

        t->addWest(this);
    }
    if (adjacentTiles["east"].second > t->val)
    {
        adjacentTiles["east"].second = t->val;
    }
}
void Tile::addSouth(Tile *t)
{
    if (adjacentTiles["south"].first == nullptr)
    {
        adjacentTiles["south"].first = t;

        t->addNorth(this);
    }
    if (adjacentTiles["south"].second > t->val)
    {
        adjacentTiles["south"].second = t->val;
    }
}
void Tile::addWest(Tile *t)
{
    if (adjacentTiles["west"].first == nullptr)
    {
        adjacentTiles["west"].first = t;

        t->addEast(this);
    }
    if (adjacentTiles["west"].second > t->val)
    {
        adjacentTiles["west"].second = t->val;
    }
}

void Tile::appendTile(Tile *t, string key)
{
    if (key == "north")
    {
        addNorth(t);
    }
    else if (key == "east")
    {
        addEast(t);
    }
    else if (key == "south")
    {
        addSouth(t);
    }
    else if (key == "west")
    {
        addWest(t);
    }
}