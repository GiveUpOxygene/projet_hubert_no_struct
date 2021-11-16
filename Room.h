#include "useful.h"

#define MAPMAXVERTICES 100

typedef struct Room Room;
struct Room
{
    Vector2 vertices[MAPMAXVERTICES];
    int count;
};

Room* InitMap(Room* map)
{
    map->count = 0;
    for (int i = 0; i < MAPMAXVERTICES; i++)
    {
        map->vertices[i].x = 0;
        map->vertices[i].y = 0;
    }
    return map;   
}

void AddVertex(Vector2* newVertex, Room* map)
{
    map->vertices[map->count].x = newVertex->x;
    map->vertices[map->count].y = newVertex->y;
    map->count++;
}
