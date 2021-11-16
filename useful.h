#include <math.h>
#include <stdlib.h>

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
float* MultVector2(const float a[], const float b[], float result[]) //tous des tableaux de taille 2 représentant des vecteurs
{
    result[0] = a[0] * b[0];
    result[1] = a[1] * b[1];
    return result;
}
float* DivVector2(const float a[], const float b[], float result[]) //tous des tableaux de taille 2 représentant des vecteurs
{
    result[0] = a[0] / b[0];
    result[1] = a[1] / b[1];
    return result;
}

int RandInt(int a, int b){
	return rand()%(b - a + 1) + a;
}

float RandFloat(float a, float b)
{
    return (rand()/RAND_MAX) * (b - a) + a;
}

float** RandomVector(float vector[])
{
    float norme = RandFloat((float)0, (float)1);
    float angle = RandFloat((float)0, 2 * M_PI);
    vector[0] = norme * cosf(angle);
    vector[1] = norme * sinf(angle);
    return *vector;
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
    } while(vecteur_1[0] / vecteur_2[0] == vecteur_1[1] / vecteur_2[1]); //condition idéaliste inateignable à cause des erreurs de mesure
    //on regarde le plus grand vecteur directeur du mur de droite (celui suivi jusqu'à maintenant)
    do {
        vecteur_1[0] = coord_mesures[count_droite-1][0] - coord_mesures[count_droite][0];
        vecteur_1[1] = coord_mesures[count_droite-1][1] - coord_mesures[count_droite][1];
        vecteur_2[0] = coord_mesures[count_droite-2][0] - coord_mesures[count_droite-1][0];
        vecteur_2[1] = coord_mesures[count_droite-2][1] - coord_mesures[count_droite-1][1];
        count_droite --;
    } while(vecteur_1[0] / vecteur_2[0] == vecteur_1[1] / vecteur_2[1]);//condition idéaliste inateignable à cause des erreurs de mesure
    vecteur_1[0] = coord_mesures[0][0] - coord_mesures[count_gauche][0];
    vecteur_1[1] = coord_mesures[0][1] - coord_mesures[count_gauche][1];
    vecteur_2[0] = coord_mesures[longueur][0] - coord_mesures[count_droite][0];
    vecteur_2[1] = coord_mesures[longueur][1] - coord_mesures[count_droite][1];
    //on calcule l'angle entre les deux vecteurs directeurs en utilisant les deux formules du produit scalaire
    return acosf((vecteur_1[0] * vecteur_2[0] + vecteur_1[1] * vecteur_2[1])/(sqrt(vecteur_1[0] * vecteur_1[0] + vecteur_1[1] * vecteur_1[1]) * sqrt(vecteur_2[0] * vecteur_2[0] + vecteur_2[1] * vecteur_2[1])));
}
