#pragma once

#include <map>
#include <stack>
#include <vector>
#include <algorithm>
#include <iostream>
#include <climits>
using namespace std;

#include "Tile.h"

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
