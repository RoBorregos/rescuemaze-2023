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
    int weight;
    vector<int> pos;

    map<string, Tile *> adjacentTiles;
    // map<string, pair<Tile *, int>> adjacentTiles;
    map<string, int> weights;

    map<string, bool> walls;

    bool visited;

    Tile();
    Tile(vector<int> p, int v);

    void addNorth(Tile *t); //, int weight);
    void addEast(Tile *t); //, int weight);
    void addSouth(Tile *t); //, int weight);
    void addWest(Tile *t); //, int weight);

    void appendTile(Tile *t, string key); //, int weight);
    // void appendTile(string key, int weight, int val);
};

Tile::Tile()
{
    weight = 0;
    pos = {0, 0, 0};

    visited = false;

    adjacentTiles = {
        {"north", nullptr},
        {"east", nullptr},
        {"south", nullptr},
        {"west", nullptr}};

    weights = {
        {"north", 0},
        {"east", 0},
        {"south", 0},
        {"west", 0}};

    walls = {
        {"north", false},
        {"east", false},
        {"south", false},
        {"west", false}};
}

Tile::Tile(vector<int> p, int v)
{
    weight = v;
    pos = p;

    visited = false;

    adjacentTiles = {
        {"north", nullptr},
        {"east", nullptr},
        {"south", nullptr},
        {"west", nullptr}};

    walls = {
        {"north", false},
        {"east", false},
        {"south", false},
        {"west", false}};

    weights = {
        {"north", 0},
        {"east", 0},
        {"south", 0},
        {"west", 0}};

    visited = false;
}

void Tile::addNorth(Tile *t) //, int weight)
{
    if (adjacentTiles["north"] == nullptr)
    {
        adjacentTiles["north"] = t;

        t->addSouth(this); //, this->val);
    }
    /* if (weights["north"] > weight)
    {
        weights["north"] = weight;
    } */
}
void Tile::addEast(Tile *t) //, int weight)
{
    if (adjacentTiles["east"] == nullptr)
    {
        adjacentTiles["east"] = t;

        t->addWest(this); //, this->val);
    }
    /* if (weights["east"] > weight)
    {
        weights["east"] = weight;
    } */
}
void Tile::addSouth(Tile *t) //, int weight)
{
    if (adjacentTiles["south"] == nullptr)
    {
        adjacentTiles["south"] = t;

        t->addNorth(this); //, this->val);
    }
    /* if (weights["south"] > weight)
    {
        weights["south"] = weight;
    } */
}
void Tile::addWest(Tile *t) //, int weight)
{
    if (adjacentTiles["west"] == nullptr)
    {
        adjacentTiles["west"] = t;

        t->addEast(this); //, this->val);
    }
    /* if (weights["west"] > weight)
    {
        weights["west"] = weight;
    } */
}

void Tile::appendTile(Tile *t, string key) //, int weight)
{
    if (key == "north")
    {
        addNorth(t); //, weight);
    }
    else if (key == "east")
    {
        addEast(t); //, weight);
    }
    else if (key == "south")
    {
        addSouth(t); //, weight);
    }
    else if (key == "west")
    {
        addWest(t); //, weight);
    }
}

/* void Tile::appendTile(string key, int weight, int val)
{
    int x2 = this->pos[0];
    int y2 = this->pos[1];
    int z2 = this->pos[2];

    if (val == 'w')
    {
        
    }
    else
    {
        
    }
    
} */