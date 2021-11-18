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

const float RANGEWALLDETECTION = 15;//au début on détecte le mur a suivre au départ si il est a tant de cm
const float DISTWALL = 20;//la distance entre le robot et le mur.
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

void AddVertex(float newVertex[], float map[][2],int vertexIndex) //rajoute un vecteur à la carte
{
    map[vertexIndex][0] = newVertex[0];
    map[vertexIndex][1] = newVertex[1];
    vertexIndex++;
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
    angle = M_PI / 2; // le robot
    currentState = 0;
}

//debut de la loop
void loop()
{
    switch (currentState)
    {
        case 0: //trouver un mur
            isWallAhead = 0;//mettre dans le booléen si le capteur détecte un mur à moins de RANGEWALLDETECTION
            forward(150);
            while (isWallAhead == 0) {
                //delay(10*100 / MAXSPEED*10); //on mesure tous les 10 cm
                //Stop();
                if (Distance_test() < DISTWALL){
                    isWallAhead = 1;
                }
            }
            if(isWallAhead == 1)
            {
                //on se replace à 15 du mur
                float dist = Distance_test();
                while(fabs(dist - DISTWALL) > 1)
                {
                  if(dist > DISTWALL)
                  {
                    forward(255);
                    delay(5);
                    Stop();
                  }
                  else
                  {
                    back(255);
                    delay(5);
                    Stop();
                  }
                }
                Stop();
                currentState = 1;
            }
            break;
        case 1: //suivre un mur
            Stop();
            break;
            myservo.write(90);
            delay(30);
            isWallAhead = 0;//on effectue la mesure pour voir si il y a un mur devant ou si le mur à gauche s'écarte d'un coup
            if ((Distance_test() < DISTWALL) || (DistDroite() - DISTWALL > 5)){ //à cause de l'erreur, ne peut pas "voir un angle entre 180° et 206°"
                isWallAhead = 1;
            }

            if(isWallAhead)
            {
                Stop();
                if (DistDroite() - DISTWALL > 5){
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
            //TO DO:
            break;
        default:
            break;
    }
    delay(dt);
    Move(dt);
}
