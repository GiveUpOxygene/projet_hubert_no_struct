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
