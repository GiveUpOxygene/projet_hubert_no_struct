#include <useful_robot.h>
#include <geometry.h>

void Rotate(int angle) //angle négatif : à gauche, angle positif : à droite
{
    //On se ramène dans l'intervalle [-180, 180]
    //des modulos seraient potentiellement plus optimisés mais les résultats pour des nombres négatifs sont incertains
    angle = (angle + 180) % 360 - 180;
    if (angle < 0) {
        left();
        delay((double)angle*1000/VROT_GAUCHE);
        Stop();
    }
    else{
        right();
        delay((double)angle*1000/VROT_DROITE);
        Stop();
    }
}

float PutRobotParallele()
{
    myservo.write(90);
    delay(300);
    float y = Distance();
    myservo.write(180);//capteur vers la gauche
    delay(700);
    float x1 = Distance();
    myservo.write(0);//capteur vers la droite
    delay(700);
    float x2 = Distance();
    myservo.write(90);

    if(x1 > 70 && x2 > 70)
    {
        //Le robot est trop en face du mur pour faire au moins une mesure correct, on tourne pour recommencer les mesure
        Rotate(25);
        PutRobotParallele();
    }
    else
    {
        //Si x1 < x2 le mur est a gauche du robot, sinon il est a droite
        //Pour aller au plus simple on supp qu'on est pile devant.
        if(x1 < x2)
        {
            return(90 + ((atanf(y/x1) * 180)/M_PI));
        }
        else
        {
            return(-90 - ((atanf(y/x2) * 180) / M_PI));
        }
    }
}

//pour les angles de 3° à 177°, utiliser la méthode pour tourner et se mettre parallèle au mur
//pour les angles de 45° à 135°, utiliser la fonction angle définie dans useful_robot.h
//pour les angles de 194° à 270°, utiliser HalfBigAngle
//pour les angles de 270° à 360°, ??????
//les angles de 177° à 194° seront traités comme des erreurs de mesure

void HalfBigAngle(){
    forward(150); //on avance un peu
    delay(1000);
    Stop();
    if (DistDroite() > 2.5) { //si le mur s'écarte d'un coup
        Rotate(90); //on se tourne vers le mur
        float mesures_mur[8]; //on mesure l'angle
        MesureDist(105, 0, 8, mesures_mur);
        float mesures_coord[8][2];
        MesuresToCoord(mesures_mur, mesures_coord, 8);
        int new_angle = fabsf(Angle(mesures_coord, 8))
        if ((new_angle > 195) || (new_angle < 175)) //si l'angle est supérieur à ce que l'on considère comme une erreur de mesure
        {
            //calculer posVertex TO DO      //on calcule la position du sommet et on l'ajoute à la carte
            //AddVertex TO DO
            new_angle = PutRobotParallele(); //on se place face au nouveau mur à suivre
            Rotate(new_angle - 90);
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
                dist = Distance_test();
            }
            Rotate(-90);
        }
    }
}
