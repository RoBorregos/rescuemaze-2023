#pragma once

#include <map>
#include <stack>
#include <vector>
#include <algorithm>
#include <iostream>
#include <climits>
using namespace std;

#include "Tile.h"

#define debug false

string posvectorToString(vector<int> pos)
{
    return to_string(pos[0]) + "," + to_string(pos[1]) + "," + to_string(pos[2]);
}


// Parte del algoritmo de Dijkstra: Regresa el tile con el costo mas bajo que no se ha visitado
Tile *cheapestUnknown(map<string, Tile *> &tiles, map<Tile *, bool> &visited, map<string, int> &cost)
{
    int curCost = INT_MAX;
    Tile *curTile = nullptr;

    for (auto &&p : tiles)
    {
        if (!visited[p.second] && cost[posvectorToString(p.second->pos)] < curCost)
        {
            curCost = cost[posvectorToString(p.second->pos)];
            curTile = p.second;
        }
    }

    return curTile;
}

// Algoritmo de Dijkstra: Determina la tile no visitada con el menor costo para ir y regresa el camino mas corto desde el tile de inicio hasta esa tile
stack<string> bestUnvisited(Tile *start, vector<Tile *> &unvisited, map<string, Tile *> tiles, vector<string> &keys)
{
    for (auto &&tile : unvisited)
    {
        cout << "Casilla sin visitar: " << tile->pos[0] << ", " << tile->pos[1] << ", " << tile->pos[2] << endl;
    }
    cout << endl;

    // Tile *tile = start;
    Tile *targetTile;
    stack<string> bestPath;

    map<Tile *, bool> visited;
    map<string, int> cost;
    map<Tile *, pair<Tile *, string>> paths;

    for (auto &&p : tiles)
    {
        visited[p.second] = false;
        cost[posvectorToString(p.second->pos)] = INT_MAX;
        paths[p.second] = pair<Tile *, string>{nullptr, ""};
    }

    Tile *source = start;
    vector<int> sourcePos = start->pos;
    cost[posvectorToString(start->pos)] = 0;

    int lowestCost = INT_MAX;
    Tile *lowTile = nullptr;

    while (source)  
    {
        if (debug)
            cout << "Casilla actual: " << source->pos[0] << ", " << source->pos[1] << ", " << source->pos[2] << endl << "Peso: " << source->weight << "    Costo: " << cost[posvectorToString(source->pos)] << endl;

        visited[source] = true;

        for (auto &&key : keys)
        {
            targetTile = source->adjacentTiles[key];

            if (debug && targetTile)
                cout << "Casilla adyacente (" << key << "): " << targetTile->pos[0] << ", " << targetTile->pos[1] << ", " << targetTile->pos[2] << endl << "Peso: " << targetTile->weight << "    Costo: " << cost[posvectorToString(targetTile->pos)] << endl;
            else if (debug)
                cout << "Casilla adyacente: null" << endl;


            if (targetTile && !visited[targetTile] && (cost[posvectorToString(source->pos)] + targetTile->weight) < cost[posvectorToString(targetTile->pos)])
            {
                cost[posvectorToString(targetTile->pos)] = cost[posvectorToString(source->pos)] + targetTile->weight;
                paths[targetTile] = pair<Tile *, string>{source, key};

                if (debug)
                    cout << "Nuevo costo de la casilla adyacente: " << cost[posvectorToString(targetTile->pos)] << endl;

                if (cost[posvectorToString(targetTile->pos)] < lowestCost && find(unvisited.begin(), unvisited.end(), targetTile) != unvisited.end())
                {
                    lowestCost = cost[posvectorToString(targetTile->pos)];
                    lowTile = targetTile;

                    if (debug)
                        cout << "Nuevo costo mas bajo: " << lowestCost << endl << "Nueva casilla de menos costo: " << lowTile->pos[0] << ", " << lowTile->pos[1] << ", " << lowTile->pos[2] << endl;
                }
            }
        }

        source = cheapestUnknown(tiles, visited, cost);
    }

    cout << "El costo mas bajo es: " << lowestCost << endl;
    cout << "La casilla de menos costo es: " << lowTile->pos[0] << ", " << lowTile->pos[1] << ", " << lowTile->pos[2] << endl;

    pair<Tile *, string> path = paths[lowTile];
    while (path.first)
    {
        bestPath.push(path.second);
        path = paths[path.first];
    }

    unvisited.erase(remove(unvisited.begin(), unvisited.end(), lowTile), unvisited.end());

    if (!lowTile)
        cout << "No hay casillas sin visitar" << endl;

/*     if (lowTile)
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
 */
    return bestPath;
}
