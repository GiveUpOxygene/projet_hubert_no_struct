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

const float RANGEWALLDETECTION = 40;//au début on détecte le mur a suivre au départ si il est à tant de cm
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
float room[MAPMAXVERTICES][2]; //tableau contenant les coordonnées des coins de la pièce
int oldDistWall = 0;
float vitesse_robot = 0;
float dist;

//time en ms
void Move(float time)
{
    if(isRepereSet)
    {
        position[0] += cosf(angle) * MAXSPEED * (time/1000);
        position[1] += sinf(angle) * MAXSPEED * (time/1000);
    }
}

void AddVertex(float newVertex[], float room[][2], int vertexIndex) //rajoute un vecteur à la carte
{
    room[vertexIndex][0] = newVertex[0];
    room[vertexIndex][1] = newVertex[1];
}

int CalculateVertexPosition(float vertexPosition[2]){
    Rotate(90); //on se tourne vers le mur
    float mesures_mur[8]; //on mesure l'angle
    MesureDist(105, 0, 8, mesures_mur);
    float mesures_coord[8][2];
    MesuresToCoord(mesures_mur, mesures_coord, 8);
    int new_angle = fabsf(Angle(mesures_coord, 8));
    if ((new_angle > 195) || (new_angle < 175)) //si l'angle est supérieur à ce que l'on considère comme une erreur de mesure
    {
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
        vertexPosition[0] = 0;
        vertexPosition[1] = 0;
        //On se remet a la position initiale avant de retourner si oui ou non on a réussie a calculer le nouveau sommet
        Rotate(-90);
        float seg1[2][2];
        float seg2[2][2];
        seg1[0][0] = mesures_coord[0][0];
        seg1[0][1] = mesures_coord[0][1];
        seg1[1][0] = mesures_coord[count_gauche][0];
        seg1[1][1] = mesures_coord[count_gauche][1];
        seg2[0][0] = mesures_coord[7][0];
        seg2[0][1] = mesures_coord[7][1];
        seg2[1][0] = mesures_coord[count_droite][0];
        seg2[1][1] = mesures_coord[count_droite][1];
        if(Intersect(seg1, seg2, vertexPosition))
        {
            return 1;
        }
        return 0;
    }
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



void loop()
{
    //etat 1
    int countCaptor = 0, indexAngleCaptor = 0, nbDistInfToRange = 0;
    int angleCaptor[4] = { 75, 90, 105, 90 };
    //etat 2

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
                if(countCaptor > 15) //on fait bouger le capteur en permanence pour être sur de ne pas rentrer dans le mur
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
                    //à cause du capteur qui bouge en permanence, les valeurs peuvent être faussées
                    //on regarde donc si trois valeurs d'affilé sont bonnes
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
                if (vitesse_robot == 0)
                {
                    vitesse_robot = mesure_vitesse();
                }
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
                  //on replace le robot à la bonne distance du mur
                  if(dist > DISTWALL)
                  {
                    forward(150);
                    delay(30); //delay différents pour éviter une boucle infinie
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
                myservo.write(0);
                currentState = 1;
            }
            break;
        case 1: //suivre un mur

            isWallAhead = 0;//on effectue la mesure pour voir si il y a un mur devant ou si le mur à gauche/droite s'écarte d'un coup
            myservo.write(0);
            delay(500);
            Move(500);
            dist = Distance();
            if (fabs(dist - DISTWALL) - oldDistWall > 2.5){ //à cause de l'erreur, ne peut pas "voir un angle entre 180° et 206°"
                isWallAhead = 1;
            }
            oldDistWall = fabs(dist - DISTWALL);

            if(isWallAhead)
            {
                Stop();
                //on mesure la position du sommet (avec les points sur le mur puis régression affine puis calcul d'intersection)
                float mesures_mur[8];
                MesureDist(105, 0, 8, mesures_mur);
                float mesures_coord[8][2];
                MesuresToCoord(mesures_mur, mesures_coord, 8);
                int newAngle = Angle(mesures_coord, 8);

                float newVertexPosition[2]; //on mettra les coordonnées dans ce vecteur
                //On regarde si c'est le premier sommet, si oui on définit le Repère orthonormé
                if(vertexIndex == 0)
                {
                    isRepereSet = 1;
                    newVertexPosition[0] = 0;
                    newVertexPosition[1] = 0;
                    //On tourne pour suivre le mur
                    Rotate(newAngle);
                    angle = 0;
                }
                else
                {
                    //On calcule la position du nouveau vertex
                    if(CalculateVertexPosition(newVertexPosition))
                    {
                        //On regarde si on est revenu au départ
                        float distNewVertexOrigin = sqrtf(newVertexPosition[0] * newVertexPosition[0] + newVertexPosition[1] * newVertexPosition[1]);
                        if(distNewVertexOrigin < 20)//on a retrouvé l'origine, in a fini la cartographie
                        {
                            currentState = 2;
                            break;
                        }
                        //On tourne pour suivre le mur
                        Rotate(newAngle);
                        angle += newAngle;
                    }
                }
                AddVertex(newVertexPosition, room, vertexIndex);
                //On repart
                //On se remet a une distance WALLDISTANCE du mur.

                //TODO

                currentState = 1;
                forward(150);
                delay(100);
                Move(100);
                break;
            }

            //on regarde si il y a un mur devant
            myservo.write(90);
            delay(500);
            Move(500);
            if(Distance() < RANGEWALLDETECTION)//Constante à droite surement à modifier
            {
                //On se met à la bonne distance du mur, donc DISTWALL
                dist = Distance();
                //Le vérifie la véracité de la distance mesuré par le capteur, qui des fois "bug" lorsque un objet est trop près,
                // le capteur renvoie une distance de plus de 1500 cm ce qui n'est pas cohérent avec la situation
                if(dist > 1500)
                {
                    //Imposible de faire de mesure, on recule et on recommence
                    back(150);
                    Rotate(10);
                    delay(1000);
                    Move(1000);
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
                    delay(22);
                    Stop();
                  }
                  dist = Distance();
                }
                Stop();

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

                    //On calcule la position du Vertex et on tourne de pourcontinuer la cartographie

                    float mesure[8];
                    //On fait les mesures
                    MesureDist(105, 30, 8, mesure);
                    float vector_1[2];
                    float vector_2[2];
                    int count_left = 0;
                    int count_right = 7;
                    //On cacule les coor des points des murs
                    float coord_mesures[8][2];
                    MesuresToCoord(mesure, coord_mesures, 8);
                    //on regarde le plus grand vecteur directeur du mur de gauche (face au robot)
                    do {
                        vector_1[0] = coord_mesures[count_left+1][0] - coord_mesures[count_left][0];
                        vector_1[1] = coord_mesures[count_left+1][1] - coord_mesures[count_left][1];
                        vector_2[0] = coord_mesures[count_left+2][0] - coord_mesures[count_left+1][0];
                        vector_2[1] = coord_mesures[count_left+2][1] - coord_mesures[count_left+1][1];
                        count_left ++;
                    } while(IsCollinear(vector_1, vector_2)); //on effectue jusqu'à ce que les vecteurs ne soient plus collinéaires
                    //on regarde le plus grand vecteur directeur du mur de droite (celui suivi jusqu'à maintenant)
                    do {
                        vector_1[0] = coord_mesures[count_right-1][0] - coord_mesures[count_right][0];
                        vector_1[1] = coord_mesures[count_right-1][1] - coord_mesures[count_right][1];
                        vector_2[0] = coord_mesures[count_right-2][0] - coord_mesures[count_right-1][0];
                        vector_2[1] = coord_mesures[count_right-2][1] - coord_mesures[count_right-1][1];
                        count_right --;
                    } while(IsCollinear(vector_1, vector_2)); //on effectue jusqu'à ce que les vecteurs ne soient plus collinéaires
                    //On calcule les segment de tendances
                    float pointsForSeg1[8][2];
                    float pointsForSeg2[8][2];
                    for (size_t i = 0; i < count_left; i++) {
                        pointsForSeg1[i][0] = coord_mesures[i][0];
                        pointsForSeg1[i][1] = coord_mesures[i][1];
                    }
                    for (int i = count_right; i < 8; i++) {
                        pointsForSeg2[i][0] = coord_mesures[i][0];
                        pointsForSeg2[i][1] = coord_mesures[i][1];
                    }
                    float seg1[2][2];
                    ToSegment(pointsForSeg1, count_left, seg1);
                    float seg2[2][2];
                    ToSegment(pointsForSeg2, 8 - count_right, seg2);
                    Intersect(seg1, seg2, newVertexPosition);

                    //On mesure l'angle pour suivre le mur
                    float newAngle = 0;//Mesuré l'angle a prendre
                    Rotate(newAngle);
                }
                //On ajoute le sommet
                AddVertex(newVertexPosition, room, vertexIndex);
                //On repart
                vertexIndex++;
                currentState = 1;
                forward(150);
                delay(100);
                Move(100);
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
