//    The direction of the car's movement
//  ENA   ENB   IN1   IN2   IN3   IN4   Description
//  HIGH  HIGH  HIGH  LOW   LOW   HIGH  Car is runing forward
//  HIGH  HIGH  LOW   HIGH  HIGH  LOW   Car is runing back
//  HIGH  HIGH  LOW   HIGH  LOW   HIGH  Car is turning left
//  HIGH  HIGH  HIGH  LOW   HIGH  LOW   Car is turning right
//  HIGH  HIGH  LOW   LOW   LOW   LOW   Car is Stoped
//  HIGH  HIGH  HIGH  HIGH  HIGH  HIGH  Car is Stoped
//  LOW   LOW   N/A   N/A   N/A   N/A   Car is Stoped

#include "useful_robot.h"
#include "geometry.h"
#include <math.h>

const float RANGEWALLDETECTION = 40;//au début on détecte le mur a suivre au départ si il est a tant de cm
const float DISTWALL = 30;//la distance entre le robot et le mur.
const float MAXSPEED = 15;//la vitesse du robot en cm/s
const float dt = 16;// le temps en ms entre 2 fonction loop
const int MAPMAXVERTICES = 100;

int currentState = 0; //on initialise l'état du robot
float position[2];//La position du robot
float angle; //L'angle du robot(rad)
int isRepereSet = 0;
float oldWallDist = 0;
int isWallAhead = 0;
float newWallDist = 0;
int vertexIndex;
float room[MAPMAXVERTICES][2];

//time en ms
void Move(float time)
{
    if(isRepereSet)
    {
        position[0] += cosf(angle) * MAXSPEED * (time/1000);
        position[1] += sinf(angle) * MAXSPEED * (time/1000);
    }
}

void AddVertex(float newVertex[], float map[][2], int vertexIndex) //rajoute un vecteur à la carte
{
    map[vertexIndex][0] = newVertex[0];
    map[vertexIndex][1] = newVertex[1];
}

//debut du setup

void setup() {
    //initialisation de la carte
    vertexIndex = 1;
    for (int i = 0; i < MAPMAXVERTICES; i++)
    {
        room[i][0] = 0;
        room[i][1] = 0;
    }
    //fin d'initialisation
    myservo.attach(3); // attach servo on pin 3 to servo object
    Serial.begin(9600);//open serial and set the baudrate
    pinMode(Echo, INPUT);
    pinMode(Trig, OUTPUT);
    pinMode(IN1,OUTPUT);//before using io pin, pin mode must be set first
    pinMode(IN2,OUTPUT);
    pinMode(IN3,OUTPUT);
    pinMode(IN4,OUTPUT);
    pinMode(ENA,OUTPUT); //acceleration
    pinMode(ENB,OUTPUT);
    Stop();
    myservo.write(90);
    position[0] = 0;
    position[1] = 0;
    angle = 90; //le robot
    currentState = 0;
}

//pour les angles de 3° à 177°, utiliser la méthode pour tourner et se mettre parallèle au mur
//pour les angles de 45° à 135°, utiliser la fonction angle définie dans useful_robot.h
//pour les angles de 194° à 270°, utiliser HalfBigAngle
//pour les angles de 270° à 360°, ??????
//les angles de 177° à 194° seront traités comme des erreurs de mesure

void HalfBigAngle(){
    forward(150); //on avance un peu
    myservo.write(0);//capteur vers la droite
    delay(1000);
    Stop();
    if (Distance() > 2.5) { //si le mur s'écarte d'un coup
        Rotate(90); //on se tourne vers le mur
        float mesures_mur[8]; //on mesure l'angle
        MesureDist(105, 0, 8, mesures_mur);
        float mesures_coord[8][2];
        MesuresToCoord(mesures_mur, mesures_coord, 8);
        int new_angle = fabsf(Angle(mesures_coord, 8));
        if ((new_angle > 195) || (new_angle < 175)) //si l'angle est supérieur à ce que l'on considère comme une erreur de mesure
        {
            //calculer posVertex TO DO      //on calcule la position du sommet et on l'ajoute à la carte
            //AddVertex TO DO

            //On récupère les vecteur directeur des 2 murs.
            float vecteur_1[2];
            float vecteur_2[2];
            int count_gauche = 0;
            int count_droite = 8;
            //on regarde le plus grand vecteur directeur du mur de gauche (face au robot)
            do {
                vecteur_1[0] = mesures_coord[count_gauche+1][0] - mesures_coord[count_gauche][0];
                vecteur_1[1] = mesures_coord[count_gauche+1][1] - mesures_coord[count_gauche][1];
                vecteur_2[0] = mesures_coord[count_gauche+2][0] - mesures_coord[count_gauche+1][0];
                vecteur_2[1] = mesures_coord[count_gauche+2][1] - mesures_coord[count_gauche+1][1];
                count_gauche ++;
            } while(IsCollinear(vecteur_1, vecteur_2)); //on effectue jusqu'à ce que les vecteurs ne soient plus collinéaires
            //on regarde le plus grand vecteur directeur du mur de droite (celui suivi jusqu'à maintenant)
            do {
                vecteur_1[0] = mesures_coord[count_droite-1][0] - mesures_coord[count_droite][0];
                vecteur_1[1] = mesures_coord[count_droite-1][1] - mesures_coord[count_droite][1];
                vecteur_2[0] = mesures_coord[count_droite-2][0] - mesures_coord[count_droite-1][0];
                vecteur_2[1] = mesures_coord[count_droite-2][1] - mesures_coord[count_droite-1][1];
                count_droite --;
            } while(IsCollinear(vecteur_1, vecteur_2)); //on effectue jusqu'à ce que les vecteurs ne soient plus collinéaires
            float vertex[2];
            float seg1[2][2];
            float seg2[2][2];
            seg1[0][0] = mesures_coord[0][0];
            seg1[0][0] = mesures_coord[0][1];
            seg1[0][0] = mesures_coord[count_gauche][0];
            seg1[0][0] = mesures_coord[count_gauche][1];
            seg2[0][0] = mesures_coord[8 - 1][0];
            seg2[0][0] = mesures_coord[8 - 1][1];
            seg2[0][0] = mesures_coord[count_droite][0];
            seg2[0][0] = mesures_coord[count_droite][1];
            if(Intersect(seg1, seg2, vertex))
            {
                AddVertex(vertex, room, vertexIndex);
                vertexIndex++;
            }

            new_angle = PutRobotParallele(); //on se place face au nouveau mur à suivre
            Rotate(new_angle - 90);
            float dist = Distance();
            //on se place à la bonne distance du mur (DISTWALL + 10 cm)
            while(fabsf(dist - DISTWALL - 10) > 4)
            {
                if(dist > DISTWALL)
                {
                    forward(100);
                    delay(50); //delay différents pour ne pas être bloqué dans une boucle infinie
                    Stop();
                }
                else
                {
                    back(100);
                    delay(75);
                    Stop();
                }
                dist = Distance();
            }
            Rotate(-90);
        }
    }
}

void loop()
{
    int countCaptor = 0, indexAngleCaptor = 0, nbDistInfToRange = 0;
    int angleCaptor[4] = { 75, 90, 105, 90 };
    switch (currentState)
    {
        case 0: //trouver un mur
            isWallAhead = 0;//mettre dans le booléen si le capteur détecte un mur à moins de RANGEWALLDETECTION
            forward(150);
            countCaptor = 0;//Pour changer l'orientation du capteur
            indexAngleCaptor = 0;
            while (isWallAhead == 0)
            {
                countCaptor++;
                if(countCaptor > 15)
                {
                    indexAngleCaptor = (indexAngleCaptor + 1) % 4;
                    myservo.write(angleCaptor[indexAngleCaptor]);
                    countCaptor = 0;
                }
                Serial.println(Distance());
                if (Distance() < RANGEWALLDETECTION)
                {
                    nbDistInfToRange++;
                    if(nbDistInfToRange >= 3)
                    {
                      isWallAhead = 1;
                      break;
                    }
                }
                else
                {
                  nbDistInfToRange = 0;
                }
                delay(dt);
            }
            myservo.write(90);
            if(isWallAhead == 1)
            {
                //on se replace à DISTWALL +ou- 1 cm du mur
                float dist = Distance();
                //Le vérifie la véracité de la distance mesuré par le capteur, qui des fois "bug" lorsque un objet est trop près,
                // le capteur renvoie une distance de plus de 1500 cm ce qui n'est pas cohérent avec la situation
                if(dist > 1500)
                {
                    //Imposible de faire de mesure, on recule et on recommence
                    back(150);
                    Rotate(10);
                    delay(1000);
                    Stop();
                    break;
                }

                while(fabs(dist - DISTWALL) > 1)
                {
                  if(dist > DISTWALL)
                  {
                    forward(150);
                    delay(20);
                    Stop();
                  }
                  else
                  {
                    back(150);
                    delay(20);
                    Stop();
                  }
                  dist = Distance();
                }
                Stop();
                //On est bien placé en therme de distance, on place le robot // au mur avec lu mur à droite du robot car après on suivra le mur par la droite.
                //On calcule l'angle de rotation pour que le robot soit bien orienté <=> // au mur.
                PutRobotParallele();
                //on avance pour commencer l'étape 2
                forward(150);
                currentState = 1;
            }
            break;
        case 1: //suivre un mur

            isWallAhead = 0;//on effectue la mesure pour voir si il y a un mur devant ou si le mur à gauche/droite s'écarte d'un coup
            if ((Distance() < DISTWALL) || (Distance() - DISTWALL > 5)){ //à cause de l'erreur, ne peut pas "voir un angle entre 180° et 206°"
                isWallAhead = 1;
            }

            if(isWallAhead)
            {
                Stop();
                if (Distance() - DISTWALL > 5){
                    forward(255);
                    delay(10 / DISTWALL);
                    Rotate(90);
                }
                //on mesure la position du sommet (avec les points sur le mur puis régression affine puis calcul d'intersection)
                float mesures_mur[8];
                MesureDist(105, 0, 8, mesures_mur);
                float mesures_coord[8][2];
                MesuresToCoord(mesures_mur, mesures_coord, 8);
                int newAngle = Angle(mesures_coord, 8);
                /*float segment_gauche[2][2];
                float segment_droite[2][2];
                ToSegment();
                ToSegment();*/

                float newVertexPosition[2]; //on mettra les coordonnées dans ce vecteur
                //On regarde si c'est le premier sommet, si oui on définit le Repère orthonormé
                if(vertexIndex == 0)
                {
                    isRepereSet = 1;
                    newVertexPosition[0] = 0;
                    newVertexPosition[1] = 0;
                    //On mesure l'angle pour suivre le mur
                    int newAngle = 0; //Mesurer l'angle a prendre
                    Rotate(newAngle);
                    angle = 0;
                }
                else
                {
                    //On calcule la position du nouveau vertex
                    newVertexPosition[0] = 0;
                    newVertexPosition[1] = 0;
                    //On regarde si on est revenu au départ
                    float distNewVertexOrigin = sqrtf(newVertexPosition[0] * newVertexPosition[0] + newVertexPosition[1] * newVertexPosition[1]);
                    if(distNewVertexOrigin < 20)//on a retrouvé l'origine, in a fini la cartographie
                    {
                        currentState = 2;
                        break;
                    }
                    //On mesure l'angle pour suivre le mur
                    float newAngle = 0;//Mesuré l'angle a prendre
                    Rotate(newAngle);
                }
                AddVertex(newVertexPosition, room, vertexIndex);
                vertexIndex++;
                //On repart
                currentState = 1;
                forward(255);
                delay(100);
                break;
            }

            //on regarde sur le côté si on s'éloigne rapidement du mur
            newWallDist = 0;//faire la mesure
            if(fabsf(newWallDist - oldWallDist) > 2 * DISTWALL)//Constante à droite surement à modifier
            {
                //on a trouvé un nouveau sommet, on calcule sa position
                float newVertexPosition[2];//on mettera les coors dans ce vecteur
                //On regarde si c'est le premier sommet
                if(vertexIndex == 0)
                {
                    isRepereSet = 1;
                    newVertexPosition[0] = 0;
                    newVertexPosition[1] = 0;
                    //On mesure l'angle pour suivre le mur
                    float newAngle = 0;//Mesuré l'angle a prendre
                    Rotate(newAngle);
                    angle = 0;
                }
                else
                {
                    //On regarde si on est revenu au départ
                    float distNewVertexOrigin = sqrtf(newVertexPosition[0] * newVertexPosition[0] + newVertexPosition[1] * newVertexPosition[1]);
                    if(distNewVertexOrigin < 20)//on a retrouvé l'origine, on a fini le taf
                    {
                        currentState = 2;
                        break;
                    }
                    //On mesure l'angle pour suivre le mur
                    float newAngle = 0;//Mesuré l'angle a prendre
                    Rotate(newAngle);
                }
                //On ajoute le sommet
                AddVertex(newVertexPosition, room, vertexIndex);
                //On repart
                vertexIndex++;
                currentState = 1;
                forward(255);
                delay(100);
                break;
            }

            //on a pas détecté de nouveau sommet :(, on se replace juste a la bonne distance du mur (corrigé les imprécisions)

            //TO DO:

            break;
        case 2: //carte finie
            //on tourne a l'infini pour dire que le robot est content
            while (1)
            {
                Rotate(180);
            }
            break;
        default:
            break;
    }
    delay(dt);
    Move(dt);
}
