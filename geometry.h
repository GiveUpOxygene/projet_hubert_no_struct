#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "useful.h"

/*
On travaille dans un repère orthoNormé d'origine le premier sommet de la pièce et de base x = (1, 0), y = (0, 1) selon hubert (donc y va devant le robot et x à sa droite)
De plus les unité utilisé seront le centimètre!
*/

//Renvoie true si les 2 segment sont presque collinéaire, false sinon;
bool IsCollinear(float a[], float b[])
{
    return fabsf((b[0] / a[0])) - fabsf((b[1] / a[1])) < 0.03 * fabsf(b[1] / a[1]);
}

//Renvoie true si les 2 segment sont presque parallèle, false sinon;
bool IsParallele(float seg1[][2], float seg2[][2])
{
    float v1[2]; float v2[];
    float a[2] = SubVector2((seg1[1]), (seg1[0]), v1);
    float b[2] = SubVector2((seg2[1]), (seg2[0]), v2);
    return IsCollinear(&a, &b);
}

// Renvoie le segment passant à peu près par tout les points(coor cartésien) de la liste points, seg doit etre vide au début de la fonction mais pas null
Segment* ToSegment(const float points[][2], int length, float seg[][2])
{
    //On fait l'algorithme de régression linéaire
    float sumX = 0;
    float sumY = 0;
    float sumXX = 0;
    float SumXY = 0;
    int indexXMin = 0;
    int indexXMax = 0;
    for (int i = 0; i < length; i++)
    {
        sumX += points[i][0];
        sumY += points[i][1];
        sumXX = points[i][0] * points[i][0];
        SumXY = points[i][0] * points[i][1];
        if(points[i][0] < indexXMin)
        {
            indexXMin = i;
        }
        else if(points[i][0] > indexXMax)
        {
            indexXMax = i;
        }
    }
    //On calcule la moyenne des coefficients directeurs
    float avgSlope = ((length * SumXY) - (sumX * sumY)) / ((length * sumXX) - (sumX * sumX));
    //On calcule l'ordonnée à l'origine b
    float b = (sumY - (avgSlope * sumX)) / length;
    //On calcule les 2 points d'extrémité du segment
    float A[2] = { points[indexXMin][0], avgSlope * points[indexXMin][0] + b };
    float B[2] = { points[indexXMax][0], avgSlope * points[indexXMax][0] + b };
    seg[0] = A;
    seg[1] = B;
    return seg;
}

//Renvoie l'équation de la droite (y = ax + b ou a = equation[0] et b = equation[1]) dirigé par le segment segment, equation doit etre vide mais non nul <=> malloc a faire avant d'appeler la fonction
void GetSegmentEquation(const float segment[][2], float equation[])
{
    equation[0] = (segment[1][1] - segment[0][1]) / (segment[1][0] - segment[0][0]);
    equation[1] = segment[0][1] - equation[0] * segment[0][0];
}

//Renvoie true si la droite (OP) intersect le segment seg
bool CollideSegmentDroite(const float seg[][2], const float O[2], const float P[])
{
    float OP[2] = SubVector2(P, O, OP);
    float OB[2] = SubVector2(seg[1], O, OB);
    float OA[2] = SubVector2(seg[0], O, OA);
    return (OP[0] * OB[1] - OP[1] * OB[0]) * (OP[0] * OA[1] - OP[1] * OA[0]) < 0;
}

//Renvoie true si les 2 segments s'intersect
bool CollideSegments(const float seg1[][2], const float seg2[][2])
{
    return CollideSegmentDroite(seg1, &seg2[0], &seg2[1]) && CollideSegmentDroite(seg2, &seg1[0], &seg2[1]);
}

//Renvoie true si les 2 segment s'intersectionne en plus des coordonnées du point d'intersection, renvoie false et (0, 0) si les 2 segment de se croise pas
bool Intersect(const float seg1[][2], const float seg2[][2], float intersectionPoint[])
{
    if(!CollideSegments(seg1, seg2))//pas la peine de de continuer si les 2 segments ne se coupe pas
    {
        intersectionPoint[0] = 0;
        intersectionPoint[1] = 0;
        return false;
    }
    //On récupère les équations des 2 segments
    float eq1[2];
    float eq2[2];
    GetSegmentEquation(seg1, eq1);
    GetSegmentEquation(seg2, eq2);
    //On calcule la solution de l'intersection entre 2 droites de vecteur directeur seg1 et seg2
    intersectionPoint[0] = (eq2[1] - eq1[1]) / (eq1[0] - eq2[0]);
    intersectionPoint[1] = ((eq1[0] * intersectionPoint[0] + eq1[1]) + (eq2[0] * intersectionPoint[0] + eq2[1])) / 2;
    return 1;
}
