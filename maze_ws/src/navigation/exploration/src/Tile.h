#pragma once

#include <map>
#include <string>
#include <vector>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

using namespace std;

class Tile
{
private:
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar &weight;
        ar &pos;
        ar &victim;
        ar &rampa;
        ar &height1;
        ar &height2;
        ar &adjacentTiles;
        ar &walls;
        ar &visited;
        ar &defined;
        ar &black;
        ar &stairs;
    }
public:
    bool defined;
    bool black;

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

    bool stairs;

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
    defined = false;
    weight = 0;
    pos = {0, 0, 0};
    victim = 0;
    black = false;
    stairs = false;

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
    defined = true;
    weight = v;
    pos = p;
    victim = vict;
    black = false;
    stairs = false;

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