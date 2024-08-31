#include <wiringPi.h>  
#include <stdio.h>  
#include <sys/time.h>
#include <sys/types.h>
#include <sys/signal.h> 

#include <pcf8574.h>
#include <lcd.h>
#include <stdlib.h> 

#include <stdbool.h>

#include <unistd.h>
#include <softPwm.h>

#include <math.h>
#include <time.h>
#include <string.h>
#include <stdint.h>

#include <semaphore.h>

// Variables Globales

// Variables Globales LineFinder

static volatile int isLineD;
static volatile int isLineG;
static volatile int isLineF;

// Variables Globales Capteur Ultrason

static volatile float distance;

// Variables Globales Télécommande

static volatile int remote_controller;
static volatile int mode = 0xff30cf;

// Variables Globales Moteur

static volatile int blocked = 0;
static volatile int speed = 270;

// Variables Globales Main

static volatile int running = 1;

// Sémaphores

sem_t semaphoreLineD;
sem_t semaphoreLineG;
sem_t semaphoreLineF;
sem_t semaphoreRemote;

// Constantes

//Constantes Moteur

// Moteur 1
#define ENA 13 //GPIO 13 PWM1
#define IN1 14 //GPIO 17
#define IN2 5 //GPIO 27

// Motor 2
#define ENB 12 //GPIO 12 PWM0 
#define IN3 15 //GPIO 22
#define IN4 6 //GPIO 10

//Constantes Buzzer
#define BUZZER_PIN 17

//Constantes Line Finder
#define LINEFINDER_PIND 22
#define LINEFINDER_PING 9
#define LINEFINDER_PINF 11

typedef enum {GAUCHE, DROITE, MILIEU} Direction;

//Constantes Capteurs Ultrason

#define  Trig    10
#define  Echo    24

//Constantes LCD

#define        AF_BASE    64
#define        AF_RS                (AF_BASE + 0)
#define        AF_RW                (AF_BASE + 1)
#define        AF_E                 (AF_BASE + 2)
#define        AF_LED               (AF_BASE + 3)

#define        AF_DB4               (AF_BASE + 4)
#define        AF_DB5               (AF_BASE + 5)
#define        AF_DB6               (AF_BASE + 6)
#define        AF_DB7               (AF_BASE + 7)

//Constantes Télécommande

#define IRPIN 26 
#define DATANBR 150.0
#define PERIOD 80 //miliseconds
#define MAXSAMPLES 300 * 1000

#define UP 0xff02fd
#define DOWN 16750695
#define RIGHT 16748655
#define LEFT 16769055
#define STOP 16761405
#define SPEED_MINUS 0xffa25d
#define SPEED_PLUS 0xffe21d

#define MODE_AUTO 0xff6897
#define MODE_MANUEL 0xff30cf
#define KILL 0xff629d

static int lcdHandle;
char data[MAXSAMPLES];
char binary[MAXSAMPLES] = { '\0' };

//------------PARTIE LINE FINDER---------------

void lineFinderInit(){ //on definit les pins en entree
	pinMode(LINEFINDER_PIND, INPUT); 
	pinMode(LINEFINDER_PING, INPUT); 
	pinMode(LINEFINDER_PINF, INPUT); 
}

int lineDetector(int detector){ //on lit la valeur du pin
	  int sensorValue = digitalRead(detector);
        return sensorValue;
}

//-----------PARTIE AFFICHAGE----------------------

void initLcd(){ //initialisation de l'ecran LCD
	int i;

    pcf8574Setup(AF_BASE,0x27); //pcf8574 I2C address
    
    lcdHandle = lcdInit (2, 16, 4, AF_RS, AF_E, AF_DB4,AF_DB5,AF_DB6,AF_DB7, 0,0,0,0) ;
    
    if (lcdHandle < 0) //si l'initialisation a echoue
    {
        fprintf (stderr, "lcdInit failed\n") ;
        exit (EXIT_FAILURE) ;
    }
    
    for(i=0;i<8;i++) 
          pinMode(AF_BASE+i,OUTPUT); 
    digitalWrite(AF_LED,1); 
    digitalWrite(AF_RW,0); /
}

void displayDistance(float distance, int lcdHandle) { //affichage de la distance sur l'ecran LCD
    char buffer[17]; // 16 caractÃ¨res + le caractÃ¨re de fin de chaÃ®ne
	lcdPosition(lcdHandle, 0, 0); // Positionnement du curseur au dÃ©but de la premiÃ¨re ligne
	sem_wait(&semaphoreLineD); // On attend que les autres threads aient fini d'utiliser les variables globales
	sem_wait(&semaphoreLineG); 
	sem_wait(&semaphoreLineF);
    if (isLineD + isLineF + isLineG > 1) { // Si plus d'un capteur est sur la ligne, on est sur une intersection
		snprintf(buffer, sizeof(buffer), "Intersection    ", distance);
        lcdPuts(lcdHandle, buffer);  // Pour effacer le reste de la ligne s'il y avait un texte prÃ©cÃ©dent
    } else {
		snprintf(buffer, sizeof(buffer), "Non Intersection", distance);
        lcdPuts(lcdHandle, buffer);  // Pour effacer le reste de la ligne s'il y avait un texte prÃ©cÃ©dent
    }
	sem_post(&semaphoreLineD); // On libÃ¨re les variables globales
	sem_post(&semaphoreLineG);
	sem_post(&semaphoreLineF);
    lcdPosition(lcdHandle, 0, 1); // Positionnement du curseur au dÃ©but de la deuxiÃ¨me ligne
	snprintf(buffer, sizeof(buffer), "Dist: %.2fcm  ", distance); // On Ã©crit la distance
	lcdPuts(lcdHandle, buffer); // Pour effacer le reste de la ligne s'il y avait un texte prÃ©cÃ©dent
}

//------------PARTIE BUZZER--------------

void buzzerInit(void){ //initialisation du buzzer
	pinMode(BUZZER_PIN, OUTPUT); //dÃ©finit le pin comme une sortie
}

void playStartupSound(void) { //joue une mÃ©lodie au dÃ©marrage
    for (int i = 0; i < 3; i++) { // Joue la note trois fois
        digitalWrite(BUZZER_PIN, HIGH); // Active le buzzer
        delay(200); // Active le buzzer pendant 200 ms
        digitalWrite(BUZZER_PIN, LOW); // DÃ©sactive le buzzer
        delay(100); // Pause de 100 ms
    }
}

void activateBuzzer(void){ //active le buzzer
    digitalWrite(BUZZER_PIN, HIGH); // Active le buzzer
    delay(500); // Active le buzzer pendant 0.5 seconde
    digitalWrite(BUZZER_PIN, LOW); // DÃ©sactive le buzzer
}

// ----------------PARTIE CAPTEUR DISTANCE--------------

void ultraInit(void)  //initialisation du capteur ultrason
{  
	pinMode(Echo, INPUT);  //initialisation du pin Echo en entree
	pinMode(Trig, OUTPUT);  //initialisation du pin Trig en sortie
}  

float disMeasure(void) //mesure de la distance
{
	struct timeval tv1; //struct timeval est une structure qui contient deux champs : tv_sec et tv_usec
	struct timeval tv2;
	
	struct timeval toStart;
	struct timeval toEnd;

	long start, stop; 
	float dis;

	digitalWrite(Trig, LOW);
	delayMicroseconds(2); // delayMicroseconds(2) est une fonction qui permet de faire une pause de 2 microsecondes

	digitalWrite(Trig, HIGH); // produce a pluse
	delayMicroseconds(10); // delayMicroseconds(10) est une fonction qui permet de faire une pause de 10 microsecondes
	digitalWrite(Trig, LOW); 

	gettimeofday(&toStart, NULL); // current time
	gettimeofday(&toEnd, NULL); // current time
	while (!(digitalRead(Echo) == 1) && toEnd.tv_sec - toStart.tv_sec < 5) // tant que le pin Echo est Ã  0 et que le temps Ã©coulÃ© est infÃ©rieur Ã  5 secondes
		gettimeofday(&toEnd, NULL); // current time
	
	gettimeofday(&tv1, NULL); // current time

	gettimeofday(&toStart, NULL);
	gettimeofday(&toEnd, NULL);
	while (!(digitalRead(Echo) == 0) && toEnd.tv_sec - toStart.tv_sec < 5) // tant que le pin Echo est Ã  1 et que le temps Ã©coulÃ© est infÃ©rieur Ã  5 secondes
		gettimeofday(&toEnd, NULL);

	gettimeofday(&tv2, NULL); // current time

	start = tv1.tv_sec * 1000000 + tv1.tv_usec; // count the time
	stop = tv2.tv_sec * 1000000 + tv2.tv_usec; // count the time

	dis = (float)(stop - start) / 1000000 * 34000 / 2; // count the distance

	return dis;	// return the distance
}

//--------------PARTIE MOTOR -----------------------

void motorInit(){ //initialisation des moteurs
  pwmSetMode (PWM_MODE_MS); //mode pwm
 pwmSetRange(1054); //pwm range
 pwmSetClock(1024); //pwm clock
 pinMode (ENA, PWM_OUTPUT); //pin en sortie
 pinMode (ENB, PWM_OUTPUT);  //pin en sortie
 pinMode (IN1, OUTPUT); //pin en sortie
 pinMode (IN2, OUTPUT); //pin en sortie
 pinMode (IN3, OUTPUT); //pin en sortie
 pinMode (IN4, OUTPUT); //pin en sortie
}

void motor_stop () { //arret des moteurs
    //Sens moteur A
   digitalWrite (IN1, LOW); 
   digitalWrite (IN2, LOW);
   pwmWrite (ENA, 0); //Vitesse moteur A
   //Sens moteur B
   digitalWrite (IN3, LOW);
   digitalWrite (IN4, LOW);
   pwmWrite (ENB, 0); //Vitesse moteur B
}
 
void motor_forward(int dutyCycle){ //avance des moteurs
    //sens moteur A
   digitalWrite (IN1, HIGH);
   digitalWrite (IN2, LOW);
   pwmWrite (ENA, dutyCycle); //Vitesse moteur A
   //sens moteur B
   digitalWrite (IN3, HIGH);
   digitalWrite (IN4, LOW);
   pwmWrite (ENB, dutyCycle); //Vitesse moteur B
}

void motor_backward(int dutyCycle){ //recule des moteurs
   //sens moteur A
   digitalWrite (IN1, LOW);
   digitalWrite (IN2, HIGH);
   pwmWrite (ENA, dutyCycle); //Vitesse moteur A
   //sens moteur B
   digitalWrite (IN3, LOW);
   digitalWrite (IN4, HIGH);
   pwmWrite (ENB, dutyCycle); //Vitesse moteur B

}

void motor_left(int dutyCycle){ //tourne a gauche
   if (mode == MODE_AUTO) { //si on est en mode automatique
	   //sens moteur A
		digitalWrite (IN1, HIGH);
		digitalWrite (IN2, LOW);
		pwmWrite (ENA, dutyCycle); //Vitesse moteur A
		//sens moteur B
		digitalWrite (IN3, LOW);
		digitalWrite (IN4, HIGH);
		pwmWrite (ENB, (int)(0.85*dutyCycle)); //Vitesse moteur B
	}
	else { //mode manuel
	   //sens moteur A
		digitalWrite (IN1, HIGH);
		digitalWrite (IN2, LOW);
		pwmWrite (ENA, dutyCycle); //Vitesse moteur A
		//sens moteur B
		digitalWrite (IN3, LOW);
		digitalWrite (IN4, 0);
		pwmWrite (ENB, (int)(0.85*dutyCycle)); //Vitesse moteur B
	}
}


void motor_right(int dutyCycle){ //tourne a droite
   if (mode == MODE_AUTO) { //si on est en mode automatique
		//Sens moteur A
		digitalWrite (IN1, LOW);
		digitalWrite (IN2, HIGH);
		pwmWrite (ENA, 230); //Vitesse moteur A
		//sens moteur B
		digitalWrite (IN3, HIGH);
		digitalWrite (IN4, LOW);
		pwmWrite (ENB, (int)1*dutyCycle); //Vitesse motor B
	}
	else { //mode manuel
		//Sens moteur A
		digitalWrite (IN1, LOW);
		digitalWrite (IN2, 0);
		pwmWrite (ENA, 230); //Vitesse moteur A
		//sens moteur B
		digitalWrite (IN3, HIGH);
		digitalWrite (IN4, LOW);
		pwmWrite (ENB, (int)1*dutyCycle); //Vitesse motor B
	}
}


Direction derniereDir = MILIEU; //variable globale qui permet de savoir la derniere direction prise par le robot
void motor_decision(int detectorD,int detectorG, int detectorF){ //decision des moteurs
	if (detectorF){ //si le capteur du milieu detecte une ligne
	  motor_stop(); //on arrete les moteurs
	  motor_forward(speed); //on avance
	  derniereDir = MILIEU; //on met a jour la derniere direction prise par le robot
	} 
      else if (!detectorD && !detectorG) { //si les capteurs de droite et de gauche ne detectent pas de ligne
        switch (derniereDir){ //on agit en fonction de la derniere direction prise par le robot
            case GAUCHE : //si la derniere direction prise par le robot est la gauche
                motor_stop(); //on arrete les moteurs
		motor_left(speed); //on tourne a gauche
                break;
            case MILIEU : //si la derniere direction prise par le robot est le milieu
                motor_stop(); 
		motor_forward(speed); //on avance
                break;
            case DROITE : //si la derniere direction prise par le robot est la droite
                motor_stop(); 
		motor_right(speed); //on tourne a droite
                break;
            default : printf("erreur");
        }
      } 
      else if (detectorD && !detectorG) { //si le capteur de droite detecte une ligne et pas celui de gauche
        motor_stop();
	motor_right(speed); //on tourne a droite
        derniereDir = DROITE; //on met a jour la derniere direction prise par le robot
      } 
      else if (detectorG && !detectorD) { //si le capteur de gauche detecte une ligne et pas celui de droite
        motor_stop();
	motor_left(speed); //on tourne a gauche
        derniereDir = GAUCHE; //on met a jour la derniere direction prise par le robot
      } 
      else if (detectorG && detectorD) { //si les capteurs de gauche et de droite detectent une ligne
        switch (derniereDir){ //on agit en fonction de la derniere direction prise par le robot
            case GAUCHE : //si la derniere direction prise par le robot est la gauche
                motor_stop();
		motor_left(speed); //on tourne a gauche
                break;
            case MILIEU : //si la derniere direction prise par le robot est le milieu
                motor_stop();
		motor_forward(speed); //on avance
                break;
            case DROITE : //si la derniere direction prise par le robot est la droite
                motor_stop();
		motor_right(speed); //on tourne a droite
                break;
            default : printf("erreur");
        }
      }
}

//------------PARTIE TELECOMMANDE--------------
void remoteControllerInit(){ //initialisation de la telecommande
	pinMode(IRPIN, INPUT); //pin en entree
	pullUpDnControl(IRPIN, PUD_DOWN); //pull down
}

//Get data from IR receiver
int getData() { //recuperation des donnees de la telecommande
  int i = 0;
  unsigned int startTime = millis(), t;
  while ( ((t=millis()) - startTime) < PERIOD && i < MAXSAMPLES) { //tant que le temps ecoule est inferieur a PERIOD et que i est inferieur a MAXSAMPLES
    data[i++] = (char) digitalRead(IRPIN); //on lit la valeur du pin et on l'ajoute au tableau data
  }
  return i - 1; //on retourne la taille du tableau data
}

//Set to \0 the binary array
void resetBinary() { //on met a \0 le tableau binary
  int i;
  for (i = 0; i < 1000; i++)
    binary[i] = '\0';
}

//Decode the signal from IR receiver
void decode() { //decodage du signal de la telecommande
  int i,j=0;
  double rate;
	int datalen = getData(); //on recupere la taille du tableau data
  //printf("Data len: %d\n", datalen);
  rate = (double) datalen / PERIOD; //on calcule le taux
  int pulses[100][100];  //pulses[][0] = value , pulses[][1] = time(us)
  int stop = 0;

  if (datalen < PERIOD) //si la taille du tableau data est inferieur a PERIOD
    return;

  resetBinary(); //on met a \0 le tableau binary

  for (i = 1; i < datalen; i++) { //on parcourt le tableau data
      if ( (data[i] != data[i - 1]) || (i == datalen - 1) ) { //si la valeur du pin change ou si on est a la fin du tableau data
        pulses[j][0] = data[i - 1];             //value
        pulses[j++][1] = (i - stop) * 1000 / (int)rate ; //time im us
        stop = i;
      }
  }
  int pulsesCount = j - 1; //on recupere la taille du tableau pulses

  j = 0;
  for (i = 0; i < pulsesCount; i++) { //on parcourt le tableau pulses
    if (pulses[i][0] != 1) //si la valeur du pin est differente de 1
      continue; //on passe a l'iteration suivante
    if (j != 0 && pulses[i][1] > 2000) //si j est different de 0 et que la valeur du pin est superieure a 2000
      break; //on sort de la boucle
    else if (pulses[i][1] < 1000) //si la valeur du pin est inferieure a 1000
      binary[j++] = '0'; //on ajoute 0 au tableau binary
    else if (pulses[i][1] >= 1000 && pulses[i][1] <= 2000) //si la valeur du pin est comprise entre 1000 et 2000
      binary[j++] = '1'; //on ajoute 1 au tableau binary
  }
  binary[j] = '\0'; //on met a \0 le tableau binary
}

//Reverse string for Binary to decimal conversion
void reverse(char* num) { //on inverse le tableau num
  int i, len = (int) strlen(num), j=0; //on recupere la taille du tableau num
  char tmp[len + 1]; 

  for (i = len-1; i >= 0; i--) //on parcourt le tableau num
    tmp[j++] = num[i]; //on ajoute la valeur du tableau num a tmp
  
  strncpy(num, tmp, (size_t) len); //on copie tmp dans num
}

//Convert binary string to integer
int binaryStr2Dec(char* bin) { //conversion du tableau binaire en entier
  int i, ret = 0;
  reverse(bin); //on inverse le tableau bin
  for (i = 0; i < strlen(bin); i++) //on parcourt le tableau bin
    ret += (int)(bin[i] -'0') * (int) pow(2, i); //on calcule la valeur de ret
  return ret;
}

void traitement(int sig) //traitement du signal
{
	motor_stop(); //on arrete les moteurs
	running = 0;
}

//-----------------MULTITHREADING----------------------------------

PI_THREAD (detectionLignesParallele) //fonction qui permet de detecter les lignes en parallele
{
	while (running) { //tant que le programme est en cours d'execution
		sem_wait(&semaphoreLineD); // On attend que les autres threads aient fini d'utiliser les variables globales
		sem_wait(&semaphoreLineG);
		sem_wait(&semaphoreLineF);
		isLineD = lineDetector(LINEFINDER_PIND); //on lit la valeur du pin
		isLineG = lineDetector(LINEFINDER_PING);
		isLineF = lineDetector(LINEFINDER_PINF);
		sem_post(&semaphoreLineD); // On libÃ¨re les variables globales
		sem_post(&semaphoreLineG);
		sem_post(&semaphoreLineF);
	}
}

PI_THREAD (detectionObstaclesParallele) //fonction qui permet de detecter les obstacles en parallele
{
	while (running) { //tant que le programme est en cours d'execution
		distance = disMeasure(); //on mesure la distance
		displayDistance(distance, lcdHandle); //on affiche la distance sur l'ecran LCD
		if (distance < 25.0 && mode==MODE_AUTO) { // Si la distance est infÃ©rieure Ã  20 cm, activez le buzzer
			motor_stop(); //on arrete les moteurs
			blocked = 1; 
			digitalWrite(BUZZER_PIN, HIGH); // Active le buzzer
			delay(300); // Active le buzzer pendant 300 ms
			digitalWrite(BUZZER_PIN, LOW); // DÃ©sactive le buzzer
			delay(700); // Pause de 700 ms
		}
		else {
			blocked = 0; 
		}
	};
}

PI_THREAD (detectionTelecommandeParallele) //fonction qui permet de detecter la telecommande en parallele
{
	while (running) { //tant que le programme est en cours d'execution
		sem_wait(&semaphoreRemote); // On attend que les autres threads aient fini d'utiliser les variables globales
		if (digitalRead(IRPIN) == 0) { //si la valeur du pin est egale a 0
			decode(); //on decode le signal de la telecommande
			remote_controller = binaryStr2Dec(binary); //on convertit le tableau binaire en entier
			if (mode==MODE_AUTO && remote_controller == MODE_MANUEL){ //si on est en mode automatique et que la telecommande est en mode manuel
				motor_stop(); //on arrete les moteurs
				mode = MODE_MANUEL; 
			};
			if (mode==MODE_MANUEL && remote_controller == MODE_AUTO){ //si on est en mode manuel et que la telecommande est en mode automatique
				mode = MODE_AUTO;
			};
			switch (remote_controller) { //on agit en fonction de la valeur de remote_controller
				case KILL : running = false; break; //on arrete le programme
				case SPEED_PLUS : if (speed < 500) {speed = speed + 20;}; break; //on augmente la vitesse
				case SPEED_MINUS : if (speed > 200) {speed = speed - 20;}; break; //on diminue la vitesse
			}
		};
		sem_post(&semaphoreRemote); // On libÃ¨re les variables globales
	};
}

//-----------------MAIN -------------------------------------------

int main(void)   //main
{
	struct sigaction action; 
	sigset_t masque; 
	sigemptyset(&masque); //Initialise le masque
	action.sa_handler = traitement; //Définit le traitement
	action.sa_mask = masque; //Associe le masque à l'action
	action.sa_flags = 0; //Pas de flags

	sigaction(SIGINT, &action, NULL); //Remplace le CTRL+C pour éteindre proprement

	if (wiringPiSetupGpio()==-1){ //initialisation de la librairie wiringPi
		printf("Erreur GPIO\n"); //si l'initialisation a echoue
	};
	motorInit();	 //initialisation des moteurs
	lineFinderInit(); //initialisation des capteurs de ligne
	ultraInit(); //initialisation du capteur ultrason
	buzzerInit(); //initialisation du buzzer
	initLcd(); //initialisation de l'ecran LCD
	remoteControllerInit(); //initialisation de la telecommande
	playStartupSound(); //joue une mÃ©lodie au dÃ©marrage
	if (sem_init(&semaphoreLineD, 0, 1) != 0 || sem_init(&semaphoreLineG, 0, 1) != 0 || sem_init(&semaphoreLineF, 0, 1) != 0 || sem_init(&semaphoreRemote, 0, 1) != 0 ) { //initialisation des sÃ©maphores
		printf("Erreur lancement sémaphores\n");
		running = 0;
	};
	if (piThreadCreate (detectionObstaclesParallele) != 0 || piThreadCreate (detectionLignesParallele) != 0 || piThreadCreate ( detectionTelecommandeParallele ) != 0 ) { //lancement des threads
		printf("Erreur lancement multithreading\n");
		running = 0;
	};
	while(running){ //tant que le programme est en cours d'execution
		if(mode==MODE_AUTO && !(blocked) ){ //si on est en mode automatique et que le robot n'est pas bloque
			sem_wait(&semaphoreLineD); // On attend que les autres threads aient fini d'utiliser les variables globales
			sem_wait(&semaphoreLineG);
			sem_wait(&semaphoreLineF);
			motor_decision(isLineD, isLineG, isLineF); //on prend une decision
			sem_post(&semaphoreLineD); // On libÃ¨re les variables globales
			sem_post(&semaphoreLineG);
			sem_post(&semaphoreLineF);
		}
		else if (mode==MODE_MANUEL) { //si on est en mode manuel
			while (running && mode == MODE_MANUEL){ 
				sem_wait(&semaphoreRemote); // On attend que les autres threads aient fini d'utiliser les variables globales
				switch (remote_controller) { //on agit en fonction de la valeur de remote_controller
					case UP : motor_forward(speed); break; //on avance
					case DOWN :	motor_backward(speed); break; //on recule
					case RIGHT : motor_right(speed); break; //on tourne a droite
					case LEFT : motor_left(speed); break; //on tourne a gauche
					case STOP : motor_stop(); break; //on arrete les moteurs
				}
				sem_post(&semaphoreRemote); // On libÃ¨re les variables globales
			}
		}
	}
	motor_stop(); //on arrete les moteurs
}

