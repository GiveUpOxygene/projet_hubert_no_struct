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
#define carSpeed 200


void forward(){
    digitalWrite(ENA,HIGH); //enable L298n A channel
    digitalWrite(ENB,HIGH); //enable L298n B channel
    digitalWrite(IN1,HIGH); //set IN1 hight level
    digitalWrite(IN2,LOW);    //set IN2 low level
    digitalWrite(IN3,LOW);    //set IN3 low level
    digitalWrite(IN4,HIGH); //set IN4 hight level
    Serial.println("Forward");//send message to serial monitor
}

void back(){
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
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

void stop(){
    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);
    Serial.println("Stop!");
}

//Permet de tourner le robot d'un certain angle(mesurer la vitesse angulaire supposé constant durant la rotation), si PB demander à Rudio qui a fait la fonction
void Rotate(float angle)
{
    //TO DO:
}

float Distance_test() {
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(20);
    float Fdistance = pulseIn(Echo, HIGH);
    Fdistance= Fdistance / 58;
    return Fdistance;
}

float MesureVitesse(){ //on suppose la vitesse en fonction du temps est constante
    float d1 = Distance_test();
    back();
    delay(2000);
    float d2 = Distance_test();
    return (d2-d1)/2;
}

//angle_deb et angle_fin sont des multiples des angles atteignables par le servo (donc multiples de 15)
void MesureDist(int angle_deb, int angle_fin, int nb_mesures, int mesures[]){ 
    for (int i = angle_deb; i < angle_fin; i += (angle_fin-angle_deb)/nb_mesures){
        myservo.write(i);
        delay(500);
        mesures[(int)(nb_mesures-1 - 1/15 * i)] = Distance_test(); // l'indice est entier car i est un multiple de 15
    }
}
