#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/*
On travaille dans un repère orthoNormé d'origine le premier sommet de la pièce et de base x = (1, 0), y = (0, 1) selon hubert (donc y va devant le robot et x à sa droite)
De plus les unité utilisé seront le centimètre!
*/

float* AddVector2(const float a[], const float b[], float result[]) //tous des tableaux de taille 2 représentant des vecteurs
{
    result[0] = a[0] + b[0];
    result[1] = a[1] + b[1];
    return result;
}
float* SubVector2(const float a[], const float b[], float result[]) //tous des tableaux de taille 2 représentant des vecteurs
{
    result[0] = a[0] - b[0];
    result[1] = a[1] - b[1];
    return result;
}

int RandInt(int a, int b)
{
	return rand()%(b - a + 1) + a;
}

float RandFloat(float a, float b)
{
    return (rand()/RAND_MAX) * (b - a) + a;
}

//coord_pol = [r, téta] est l'entrée et coord_cart = [x, y] est renvoyée. coord_cart est censée etre vide en début de fonction.
void PolaireToCart(float coord_pol[], float coord_cart[]){
    coord_cart[0] = coord_pol[0] * cosf(coord_pol[1]);
    coord_cart[1] = coord_pol[0] * sinf(coord_pol[1]);
}

//coord_cart = [x, y] est l'entrée et coord_pol = [r, téta] est renvoyée. coord_pol est censée etre vide en début de fonction.
void CartToPolaire(float coord_cart[], float coord_pol[])
{
    coord_pol[0] = sqrtf((coord_cart[0] * coord_cart[0]) + (coord_cart[1] * coord_cart[1]));
    coord_pol[1] = 2 * atanf(coord_cart[1]/(coord_cart[0] + (coord_pol[0])));
}

void AddVect(float coord_cart[][2], int longueur, float somme_vect[]){
    //coord_cart est un tableau de vecteurs avec tous les vecteurs à additionner. tous les vecteurs doivent etre en coordonnées cartésiennes.
    //longueur est la longueur de ce tableau
    //somme_vect est le résultat de cette addition, vide au début de la fonction
    somme_vect[0] = 0;
    somme_vect[1] = 0;
    for (int i = 0; i < longueur; i++) {
        somme_vect[0] += coord_cart[i][0];
        somme_vect[1] += coord_cart[i][1];
    }
}

void MesuresToCoord(float mesures[], float coord_mesures[][2], int longueur){//0 :=> 90°, i => 15*i+90 °
    //mesures est un tableau contenant toutes les distance mesurées. l'angle correspondant est 105 - [indice] * 15 en degrés
    //coord_mesures est un tableau qui contient toutes les coordonnées des points du mur où la distance a été mesurées.
    //longueur est la longueur du tableau mesures
    //Le point (0,0) est le capteur
    //premier angle de mesure : 105°
    //dernier angle de mesure : 0°
    for (int i = 0; i < longueur; i++) {
        coord_mesures[i][0] = mesures[i] * cosf((105 - i * 15)*M_PI/180);
        coord_mesures[i][1] = mesures[i] * sinf((105 - i * 15)*M_PI/180);
    }
}

//Renvoie true si les 2 segment sont presque collinéaire, false sinon;
bool IsCollinear(float a[], float b[])
{
    return fabsf((b[0] / a[0])) - fabsf((b[1] / a[1])) < 0.03 * fabsf(b[1] / a[1]);
}

//Renvoie true si les 2 segment sont presque parallèle, false sinon;
bool IsParallele(float seg1[][2], float seg2[][2])
{
    float v1[2];
    float v2[2];
    SubVector2((seg1[1]), (seg1[0]), v1);
    SubVector2((seg2[1]), (seg2[0]), v2);
    return IsCollinear(v1, v2);
}

// Renvoie le segment passant à peu près par tout les points(coor cartésien) de la liste points, seg doit etre vide au début de la fonction mais pas null
void ToSegment(const float points[][2], int length, float seg[][2])
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
        sumXX = points[i][0] * points[i][0]; //à regarder
        SumXY = points[i][0] * points[i][1]; //idem
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
    seg[0][0] = points[indexXMin][0];
    seg[0][1] = avgSlope * points[indexXMin][0] + b;
    seg[1][0] = points[indexXMax][0];
    seg[1][1] = avgSlope * points[indexXMax][0] + b;
}

//Renvoie l'équation de la droite (y = ax + b ou a = equation[0] et b = equation[1]) dirigé par le segment segment, equation doit etre vide mais non nul <=> malloc a faire avant d'appeler la fonction
void GetSegmentEquation(const float segment[][2], float equation[])
{
    equation[0] = (segment[1][1] - segment[0][1]) / (segment[1][0] - segment[0][0]);
    equation[1] = segment[0][1] - equation[0] * segment[0][0];
}

//Renvoie true si la droite (OP) intersect le segment seg
int CollideSegmentDroite(const float seg[][2], const float O[2], const float P[])
{
    float OP[2];
    SubVector2(P, O, OP);
    float OB[2];
    SubVector2(seg[1], O, OB);
    float OA[2];
    SubVector2(seg[0], O, OA);
    return (OP[0] * OB[1] - OP[1] * OB[0]) * (OP[0] * OA[1] - OP[1] * OA[0]) < 0;
}

//Renvoie true si les 2 segments s'intersect
int CollideSegments(const float seg1[][2], const float seg2[][2])
{
    return (CollideSegmentDroite(seg1, seg2[0], seg2[1]) && CollideSegmentDroite(seg2, seg1[0], seg2[1]));
}

//Renvoie true si les 2 segment s'intersectionne en plus des coordonnées du point d'intersection, renvoie false et (0, 0) si les 2 segment de se croise pas
int Intersect(const float seg1[][2], const float seg2[][2], float intersectionPoint[])
{
    if(!CollideSegments(seg1, seg2))//pas la peine de de continuer si les 2 segments ne se coupe pas
    {
        intersectionPoint[0] = 0;
        intersectionPoint[1] = 0;
        return 0;
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

float Angle(float coord_mesures[][2], int longueur){
    //calcule l'angle en fonction des données du capteur.
    //coord_mesures est un tableau qui contient toutes les coordonnées des points du mur où la distance a été mesurée.
    //longueur est la longueur de ce tableau
    float vecteur_1[2];
    float vecteur_2[2];
    int count_gauche = 0;
    int count_droite = longueur;
    //on regarde le plus grand vecteur directeur du mur de gauche (face au robot)
    do {
        vecteur_1[0] = coord_mesures[count_gauche+1][0] - coord_mesures[count_gauche][0];
        vecteur_1[1] = coord_mesures[count_gauche+1][1] - coord_mesures[count_gauche][1];
        vecteur_2[0] = coord_mesures[count_gauche+2][0] - coord_mesures[count_gauche+1][0];
        vecteur_2[1] = coord_mesures[count_gauche+2][1] - coord_mesures[count_gauche+1][1];
        count_gauche ++;
    } while(IsCollinear(vecteur_1, vecteur_2)); //on effectue jusqu'à ce que les vecteurs ne soient plus collinéaires
    //on regarde le plus grand vecteur directeur du mur de droite (celui suivi jusqu'à maintenant)
    do {
        vecteur_1[0] = coord_mesures[count_droite-1][0] - coord_mesures[count_droite][0];
        vecteur_1[1] = coord_mesures[count_droite-1][1] - coord_mesures[count_droite][1];
        vecteur_2[0] = coord_mesures[count_droite-2][0] - coord_mesures[count_droite-1][0];
        vecteur_2[1] = coord_mesures[count_droite-2][1] - coord_mesures[count_droite-1][1];
        count_droite --;
    } while(IsCollinear(vecteur_1, vecteur_2)); //on effectue jusqu'à ce que les vecteurs ne soient plus collinéaires
    vecteur_1[0] = coord_mesures[0][0] - coord_mesures[count_gauche][0];
    vecteur_1[1] = coord_mesures[0][1] - coord_mesures[count_gauche][1];
    vecteur_2[0] = coord_mesures[longueur][0] - coord_mesures[count_droite][0];
    vecteur_2[1] = coord_mesures[longueur][1] - coord_mesures[count_droite][1];
    //on calcule l'angle entre les deux vecteurs directeurs en utilisant les deux formules du produit scalaire
    return (int)acosf((vecteur_1[0] * vecteur_2[0] + vecteur_1[1] * vecteur_2[1])/(sqrt(vecteur_1[0] * vecteur_1[0] + vecteur_1[1] * vecteur_1[1]) * sqrt(vecteur_2[0] * vecteur_2[0] + vecteur_2[1] * vecteur_2[1])));
}
