#pragma once

#include <map>
#include <string>
#include <vector>
using namespace std;

class Tile
{
private:
public:
    int weight;
    vector<int> pos;

    int victim;
    
    // Direccion de la rampa
    // -1: no hay rampa
    // 0: rampa hacia el norte
    // 1: rampa hacia el este
    // 2: rampa hacia el sur
    // 3: rampa hacia el oeste
    int rampa;
    int height1;
    int height2;

    map<string, Tile *> adjacentTiles;

    map<string, bool> walls;

    bool visited;

    Tile();
    Tile(vector<int> p, int v, int vict);

    void addNorth(Tile *t);
    void addEast(Tile *t); 
    void addSouth(Tile *t);
    void addWest(Tile *t); 

    void appendTile(Tile *t, string key);
};

Tile::Tile()
{
    weight = 0;
    pos = {0, 0, 0};
    victim = 0;

    rampa = -1;
    height1 = 0;
    height2 = 0;

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
}

Tile::Tile(vector<int> p, int v, int vict)
{
    weight = v;
    pos = p;
    victim = vict;

    rampa = -1;
    height1 = 0;
    height2 = 0;


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

    visited = false;
}

void Tile::addNorth(Tile *t) //, int weight)
{
    if (adjacentTiles["north"] == nullptr)
    {
        adjacentTiles["north"] = t;

        t->addSouth(this); //, this->val);
    }
}
void Tile::addEast(Tile *t) //, int weight)
{
    if (adjacentTiles["east"] == nullptr)
    {
        adjacentTiles["east"] = t;

        t->addWest(this); //, this->val);
    }
}
void Tile::addSouth(Tile *t) //, int weight)
{
    if (adjacentTiles["south"] == nullptr)
    {
        adjacentTiles["south"] = t;

        t->addNorth(this); //, this->val);
    }
}
void Tile::addWest(Tile *t) //, int weight)
{
    if (adjacentTiles["west"] == nullptr)
    {
        adjacentTiles["west"] = t;

        t->addEast(this); //, this->val);
    }

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