//    The direction of the car's movement
//  ENA   ENB   IN1   IN2   IN3   IN4   Description
//  HIGH  HIGH  HIGH  LOW   LOW   HIGH  Car is runing forward
//  HIGH  HIGH  LOW   HIGH  HIGH  LOW   Car is runing back
//  HIGH  HIGH  LOW   HIGH  LOW   HIGH  Car is turning left
//  HIGH  HIGH  HIGH  LOW   HIGH  LOW   Car is turning right
//  HIGH  HIGH  LOW   LOW   LOW   LOW   Car is stoped
//  HIGH  HIGH  HIGH  HIGH  HIGH  HIGH  Car is stoped
//  LOW   LOW   N/A   N/A   N/A   N/A   Car is stoped

#include "useful_robot.h"
#include "Room.h"

const float RANGEWALLDETECTION = 15;//au début on détecte le mur a suivre au départ si il est a tant de cm
const float DISTWALL = 15;//la distance entre le robot et le mur.
const float MAXSPEED = 15;//la vitesse du robot en cm/s
const float dt = 33;// le temps en ms entre 2 fonction loop

Room room;
enum State currentState;
Vector2 position;//La position du robot
float angle;//L'angle du robot(rad)
bool isRepereSet = false;
float oldWallDist = 0;

void setup() {
    myservo.attach(3); // attach servo on pin 3 to servo object
    Serial.begin(9600);//open serial and set the baudrate
    /*
    pinMode(Echo, INPUT);
    pinMode(Trig, OUTPUT);
    pinMode(IN1,OUTPUT);//before using io pin, pin mode must be set first
    pinMode(IN2,OUTPUT);
    pinMode(IN3,OUTPUT);
    pinMode(IN4,OUTPUT);
    pinMode(ENA,OUTPUT); //acceleration
    pinMode(ENB,OUTPUT);
    stop();
    */
    myservo.write(180);
    position[0] = 0;
    position[1] = 0;
    angle = M_PI / 2; // le robot
    &room = InitMap(&room);
    currentState = FindingWall;
    //on avance
    forward();
}

enum State
{
    FindingWall = 0,
    FollowingSide = 2,
    MappingDone = 3
}

//time en ms
void Move(float time)
{
    if(isRepereSet)
    {
        position[0] += cosf(angle) * MAXSPEED * (time/1000);
        position[1] += sinf(angle) * MAXSPEED * (time/1000);
    }
}

void loop()
{
    switch (currentState)
    {
        case FindingWall:
            bool isWallAhead = false;//mettre dans le booléen si le capteur détecte un mur à moins de RANGEWALLDETECTION
            if(isWallAhead)
            {
                stop();
                //On mesure l'angle à prendre pour être pârallèle au mur;
                float newAngle = 0,//calculer l'angle
                Rotate(newAngle);
                //On replace le capteur a ultrason vers le mur, donc a 0 ou 180°
                myservo.write(180);//180° pour l'exemple
                forward();
                currentState = FollowingSide;
            }
            break;
        case FollowingSide:
            //on vérif si il y a un sommet devant le robot <=> détection d'un mur devant || dist avec le mur augmente d'un coup
            //d'abord le mur devant
            myservo.write(90);
            delay(30);
            bool isWallAhead = false;//on effectue la mesure pour voir si il y a un mur devant
            if(isWallAhead)
            {
                stop();
                //on mesure la position du sommet (avec les points sur le mur puis régression affine puis calcule d'intersection)
                float newVertexPosition[2];//on mettera les coordonnéess dans ce vecteur
                ///On regarde si c'est le premier sommet, si oui on définie le Repère ortho normé
                if(room.count == 0)
                {
                    isRepereSet = true;
                    newVertexPosition[0] = 0;
                    newVertexPosition[1] = 0;
                    //On mesure l'angle pour suivre le mur
                    float newAngle = 0;//Mesuré l'angle a prendre
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
                    if(distNewVertexOrigin < 20)//on a retrouvé l'origine, in a fini le taf
                    {
                        currentState = MappingDone;
                        break;
                    }
                    //On mesure l'angle pour suivre le mur
                    float newAngle = 0;//Mesuré l'angle a prendre
                    Rotate(newAngle);
                }
                AddVertex(&newVertexPosition, &room);
                //On repart
                currentState = FollowingSide;
                forward();
                delay(100);
                break;
            }

            //on regarde sur le côté si on s'éloigne rapidement du mur
            float newWallDist = 0;//faire la mesure
            if(fabsf(newWallDist - oldWallDist) > 2 * DISTWALL)//Constante a droite a surement modifié
            {
                //on a trouvé un nouvelle sommet, on calcule sa position!
                Vector2 newVertexPosition = malloc(sizeof(Vector2));//on mettera les coors dans ce vecteur
                //On regarde si c'est le premier sommet
                if(room.count == 0)
                {
                    isRepereSet = true;
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
                        currentState = MappingDone;
                        break;
                    }
                    //On mesure l'angle pour suivre le mur
                    float newAngle = 0;//Mesuré l'angle a prendre
                    Rotate(newAngle);
                }
                //On ajoute le sommet
                AddVertex(&newVertexPosition, &room);
                //On repart
                currentState = FollowingSide;
                forward();
                delay(100);
                break;
            }

            //on a pas détecté de nouveau sommet :(, on se replace juste a la bonne distance du mur (corrigé les imprécisions)

            //TO DO:

            break;
        case MappingDone:
            //on tourne a l'infini pour dire que le robot est content
            //TO DO:
            break;
        default:
            break;
    }
    delay(dt);
    Move(dt);
}
