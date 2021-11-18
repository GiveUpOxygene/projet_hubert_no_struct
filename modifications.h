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

//pour les angles de 3° à 177°, utiliser la méthode pour tourner et se mettre parallèle au mur
//pour les angles de 45° à 135°, utiliser la fonction angle définie dans useful_robot.h
//pour les angles de 194° à 270°, utiliser le psudo-code
//pour les angles de 270° à 360°, ??????
//les angles de 177° à 194° seront traités comme des erreurs de mesure

void HalfBigAngle(){
    forward(150);
    if (DistDroite() > 2.5) {
        float mesures_mur[8];
        MesureDist(105, 0, 8, mesures_mur);
        float mesures_coord[8][2];
        MesuresToCoord(mesures_mur, mesures_coord, 8);
        if (Angle(mesures_coord, 8)) {
            
        }
    }
}
