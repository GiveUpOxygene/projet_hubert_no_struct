#include <Servo.h>
Servo myservo;

int Echo = A4;
int Trig = A5;

#define ENB 5
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define ENA 6
#define VROT_DROITE 345 //en degré par seconde, mesurées à la main
#define VROT_GAUCHE 345


void forward(int vit){
    digitalWrite(ENA,vit); //enable L298n A channel
    digitalWrite(ENB,vit); //enable L298n B channel
    digitalWrite(IN1,HIGH); //set IN1 hight level
    digitalWrite(IN2,LOW);    //set IN2 low level
    digitalWrite(IN3,LOW);    //set IN3 low level
    digitalWrite(IN4,HIGH); //set IN4 hight level
    Serial.println("Forward");//send message to serial monitor
}

void back(int vit){ //vit est un entier entre 0 et 255
    digitalWrite(ENA,vit);
    digitalWrite(ENB,vit);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    Serial.println("Back");
}

void left(){
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    Serial.println("Left");
}

void right(){
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    Serial.println("Right");
}

void Stop(){
    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
    Serial.println("Stop!");
}

void Rotate(int angle) //angle négatif : à gauche, angle positif : à droite
{
    //On se ramène dans l'intervalle [-180, 180]
    //des modulos seraient potentiellement plus optimisés mais les résultats pour des nombres négatifs sont incertains
    while(angle < -180)
    {
        angle += 360;
    }
    while(angle > 180)
    {
        angle -= 360;
    }
    if (angle < 0) {
        left();
        delay((angle*100)/VROT_GAUCHE*10); //bizzarement, faire *1000 ne marche pas
        Stop();
    }
    else{
        right();
        delay((angle*100)/VROT_DROITE*10);
        Stop();
    }
}

float Distance_test() { //renvoie la distance en float entre le robot et un obstacle
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(20);
    float Fdistance = pulseIn(Echo, HIGH);
    Fdistance= Fdistance / 58;
    return Fdistance;
}

float DistDroite() {
    myservo.write(0);
    return Distance_test();
}

float MesureVitesse(){ //on suppose la vitesse en fonction du temps est constante
    float d1 = Distance_test();
    back(255);
    delay(2000);
    float d2 = Distance_test();
    return (d2-d1)/2;
}

//angle_deb et angle_fin sont des multiples des angles atteignables par le servo (donc multiples de 15)
void MesureDist(int angle_deb, int angle_fin, int nb_mesures, float mesures[]){
    //mesures[] contiendra l'ensemble des mesures faites pendant la boucle en fin de fonction
    for (int i = angle_deb; i < angle_fin; i += (angle_fin-angle_deb)/nb_mesures){
        myservo.write(i);
        delay(500);
        mesures[(int)(nb_mesures-1 - 1/15 * i)] = Distance_test(); // l'indice est entier car i est un multiple de 15
    }
}
