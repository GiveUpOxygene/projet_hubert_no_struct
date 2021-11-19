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
#define VROT_DROITE 210 //en degré par seconde, mesurées à la main
#define VROT_GAUCHE 210


void forward(int speed)
{
    analogWrite(ENA,speed); //enable L298n A channel
    analogWrite(ENB,speed); //enable L298n B channel
    digitalWrite(IN1,HIGH); //set IN1 hight level
    digitalWrite(IN2,LOW);    //set IN2 low level
    digitalWrite(IN3,LOW);    //set IN3 low level
    digitalWrite(IN4,HIGH); //set IN4 hight level
    //Serial.println("Forward");//send message to serial monitor
}

void back(int speed)
{ //vit est un entier entre 0 et 255
    analogWrite(ENA,speed);
    analogWrite(ENB,speed);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    //Serial.println("Back");
}

void left()
{
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    //Serial.println("Left");
}

void right()
{
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    //Serial.println("Right");
}

void Stop()
{
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
    //Serial.println("Stop!");
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
    analogWrite(ENA, 100);
    analogWrite(ENB, 100);
    if (angle < 0) {
        right();
        delay((-angle*100)/VROT_DROITE*10); //bizzarement, faire *1000 ne marche pas
        Stop();
    }
    else{
        left();
        delay((angle*100)/VROT_GAUCHE*10);
        Stop();
    }
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
}

//Renvoie la distance entre le capteur et l'object devant le capteur.
float Distance()
{
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(20);
    float Fdistance = pulseIn(Echo, HIGH);
    Fdistance = Fdistance / 58;
    return Fdistance;
}

/*void PutRobotParallele()
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
            float angToRot = 90 + ((atanf(y/x1) * 180)/M_PI);
            Rotate(angToRot);
        }
        else
        {
            float angToRot = -90 - ((atanf(y/x2) * 180) / M_PI);
            Rotate(angToRot);
        }
    }
}
*/

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
        return PutRobotParallele() - 25;
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

//angle_deb et angle_fin sont des multiples des angles atteignables par le servo (donc multiples de 15)
void MesureDist(int angle_deb, int angle_fin, int nb_mesures, float mesures[])
{
    //mesures[] contiendra l'ensemble des mesures faites pendant la boucle en fin de fonction
    for (int i = angle_deb; i < angle_fin; i += (angle_fin-angle_deb)/nb_mesures)
    {
        myservo.write(i);
        delay(500);
        mesures[(int)(nb_mesures-1 - 1/15 * i)] = Distance(); // l'indice est entier car i est un multiple de 15
    }
}

float mesure_vitesse()
{
    float mesure1 = Distance();
    back(150);
    delay(3000);
    Stop();
    return ((Distance()-mesure1)/3);
}
