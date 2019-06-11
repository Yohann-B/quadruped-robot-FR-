/*
 * robot.cpp
 *
 * Created: 6/9/2018 4:48:24 PM
 * Author : Yohann
 */ 

/* This file was creating through an Atmel Studio 7's project.
There are a lot of different improvements to make.
*/

#include <avr/io.h>
#include <avr/interrupt.h>

#define consigne 30 //distance minimale tolérée à un obstacle
int mesure=0;
int valeur=0;
int test=0;
int D=0;
  void config_pwm(void)
  {
    ///////////// FAST PWM POUR LE TIMER 1 ////////////
    DDRB|=(1<<PB5);
    DDRB|=(1<<PB6);
    DDRB|=(1<<PB7);
    // PWM FAST en mode 14 avec ICR
    TCCR1A |=(1<<WGM11);
    TCCR1A &=~(1<<WGM10);
    TCCR1B |= (1<<WGM12)|(1<<WGM13);
    ICR1 = 40000;// ICR1 = Fclk/(Fsignal*prescaler)
    //OCR1A=3000;
    //OCR1B=3000;
    //OCR1C=3000;
    
    // configuration en mode non inversé
    TCCR1A &=~(1<<COM1A0)&~(1<<COM1B0)&~(1<<COM1C0);
    TCCR1A |=(1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1);
    TCNT1=0;
    
    // prescaler de 8 
    TCCR1B|=(1<<CS11);
    TCCR1B&=~(1<<CS10)&~(1<<CS12);
    
    ///////////// FAST PWM POUR LE TIMER 3 /////////////
    DDRE|=(1<<3);
    DDRE|=(1<<4);
    DDRE|=(1<<5);
    // configuration du Fast PWM en mode 14
    TCCR3A |=(1<<WGM31);
    TCCR3A&=~(1<<WGM30);
    TCCR3B |= (1<<WGM32)|(1<<WGM33);
    ICR3=40000;// Fclk/(Fsignal*prescaler)
    //OCR3A=3000;
    //OCR3B=3000;
    //OCR3C=3000;
    
    // configuration en mode clear 
    TCCR3A &=~(1<<COM3A0)&~(1<<COM3B0)&~(1<<COM3C0);
    TCCR3A |=(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);
    TCNT3=0;
    
    // prescaler
    TCCR3B|=(1<<CS31);
    TCCR3B&=~(1<<CS30)&~(1<<CS32);
    
    ///////////// FAST PWM POUR LE TIMER 4 //////////////
    DDRH|=(1<<3);
    DDRH|=(1<<4);
    // configuration du Fast PWM en mode 14
    TCCR4A |=(1<<WGM41);
    TCCR4A&=~(1<<WGM40);
    TCCR4B |= (1<<WGM42)|(1<<WGM43);
    ICR4 = 40000;// Fclk/(Fsignal*prescaler)
    //OCR4A=3000;
    //OCR4B=3000;
    //OCR4C=3000;
    
    // configuration en mode clear
    TCCR4A &=~(1<<COM4A0)&~(1<<COM4B0)&~(1<<COM4C0);
    TCCR4A |=(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
    TCNT4=0;
    
    // prescaler
    TCCR4B|=(1<<CS41);
    TCCR4B&=~(1<<CS40)&~(1<<CS42);
  }
  
  void start_coude_gauche_avant(int x)
  {
    OCR1B=2000*(x/180+1);//x represente l'angle qu on veut pour se deplacer
  }
  
  void start_coude_gauche_arriere(int x)
  {
    OCR3C=2000*(x/180+1);//x represente l'angle qu on veut pour se deplacer
  }
  
  void start_coude_droit_avant(int x)
  {
    OCR3A=2000*(x/180+1);//x represente l'angle qu on veut pour se deplacer
  }
  
  void start_coude_droit_arriere(int x)
  {
    OCR4B=2000*(x/180+1);//x represente l'angle qu on veut pour se deplacer
  }
  
  void start_bras_gauche_avant(int x)
  {
    OCR1A=2000*(x/180+1);//x represente l'angle qu on veut pour se deplacer
  }
  
  void start_bras_gauche_arriere(int x)
  {
    OCR3B=2000*(x/180+1);//x represente l'angle qu on veut pour se deplacer
  }
  
  void start_bras_droit_avant(int x)
  {
    OCR1C=2000*(x/180+1);//x represente l'angle qu on veut pour se deplacer
  }
  
  void start_bras_droit_arriere(int x)
  {
    OCR4A=2000*(x/180+1);//x represente l'angle qu on veut pour se deplacer
  }

  void position_initiale()
  {
    
    start_coude_gauche_avant(90);
    start_coude_gauche_arriere(90);
    start_coude_droit_avant(90);
    start_coude_droit_arriere(90);
    start_bras_gauche_avant(90);
    start_bras_gauche_arriere(90);
    start_bras_droit_avant(90);
    start_bras_droit_arriere(90);
    
  }
  
  //// configuration du timer 0 multiple de 1 ms
  void delai_multiple_cycle_complet(int t)
  {
    // mode normal
    TCCR0A&=~(1<<COM0A1)&~(1<< COM0A0);
    TCCR0A&=~(1<< COM0B1)&~(1<< COM0B0);
    TCCR0A&=~(1<< WGM01) &~(1<< WGM00);
    
    // Time CounterControl Register0 B
    // Divisieur de frequence pour 1 ms avec un prescaller de 64
    TCCR0B|=(1<< CS00)|(1<<CS01);
    TCCR0B&=~(1<< CS02)&~(1<<WGM02);
    
    // Registre de données initialisé à 0
    TCNT0=0x00;
    for(int i=0;i<t;i++)
    {
      // Reset de l’indicateur TOV0
      TIFR0|=(1<<TOV0);
      // Attente fin du cycle (passage  à 1 de TOV0)
      while((TIFR0&0b00000001)==0);
    }
	/*TCCR5A&=~(1<<WGM50)&~(1<<WGM51);
	TCCR5B&=~(1<<WGM52)&~(1<<WGM53);
	// configuration en mode normal
	
	TCCR5A&=~(1<<COM5A1)&~(1<<COM5B1)&~(1<<COM5C1)&~(1<<COM5A0)&~(1<<COM5B0)&~(1<<COM5C0);
	
	// prescaler de 256
	TCCR5B|=(1<<CS52);
	TCCR5B&=~(1<<CS50)&~(1<<CS51);
	TCNT5= 0;     //(4.19-Temps)/16µs
	while((TIFR5 & 1)==0){}
	TIFR5|=(1<<TOV5);*/
  }
  
  void delai_10us()
  {
	
    //utilisation du timer pour faire un délai de 10 microsecondes:
    TCCR0A&=~(1<<WGM01)&~(1<<WGM00);
	TCCR0B&=~(1<<WGM02)&~(1<<CS01)&~(1<<CS02);
	TCCR0B|=(1<<CS00);
	TCNT0=160; //10us*256/16us.
	TIFR0|=(1<<TOV0);
	
  }
  
  void marche_avant()
  {
    // deplacement du bras gauche avant 
    OCR1B=2451;   // start_coude_gauche_avant(124);// angle pour lever de 1cm: 90°+34°
    delai_multiple_cycle_complet(150);
    OCR1A=1651;   //start_bras_gauche_avant(110);// angle de déplacement vers l'avant
    delai_multiple_cycle_complet(150);
    OCR1B=2000;   //start_coude_gauche_avant(90);//angle pour baisser de 1cm: 90°-34°
    delai_multiple_cycle_complet(150);
    
    // deplacement du bras gauche arriere
    OCR3C=1647;//start_coude_gauche_arriere(124);// angle pour lever
    delai_multiple_cycle_complet(150);
    OCR3B=1651;//start_bras_gauche_arriere(110);
    delai_multiple_cycle_complet(150);
    OCR3C=2000;//start_coude_gauche_arriere(90);//angle pour poser
    delai_multiple_cycle_complet(150);

    // on rame 
    OCR1A=2000;//start_bras_gauche_avant(90);//variation opposée à la précédente: 90°-20°
    OCR3B=2000;//start_bras_gauche_arriere(90);
    OCR1C=1651;//start_bras_droit_avant(110);
    OCR4A=1651;//start_bras_droit_arriere(110);
    delai_multiple_cycle_complet(150);
    

    // deplacement du bras doit avant
    OCR3A=1647;//start_coude_droit_avant(124);// angle pour lever
    delai_multiple_cycle_complet(150);
    OCR1C=2347;//start_bras_droit_avant(70);
    delai_multiple_cycle_complet(150);
    OCR3A=2000;//start_coude_droit_avant(90);//angle pour poser
    delai_multiple_cycle_complet(150);

    
    // deplacement du bras doit arriere
    OCR4B=2351;//start_coude_droit_arriere(124);// angle pour lever
    delai_multiple_cycle_complet(150);
    OCR4A=2347;//start_bras_droit_arriere(70);
    delai_multiple_cycle_complet(150);
    OCR4B=2000;//start_coude_droit_arriere(90);//angle pour poser
    delai_multiple_cycle_complet(150);
    
    // on rame
    OCR1A=2347;//start_bras_gauche_avant(70);
    OCR3B=2347;//start_bras_gauche_arriere(70);
    OCR1C=2000;//start_bras_droit_avant(110);
    OCR4A=2000;//start_bras_droit_arriere(110);
    delai_multiple_cycle_complet(150);
  }
  /*
  void marche_arriere()
  {
    
    // deplacement du bras gauche arriere
    start_coude_gauche_arriere(124);// angle pour lever
    delai_multiple_cycle_complet(100);
    start_bras_gauche_arriere(70);
    delai_multiple_cycle_complet(100);
    start_coude_gauche_arriere(56);//angle pour poser
    delai_multiple_cycle_complet(100);

    
    // deplacement du bras gauche avant
    start_coude_gauche_avant(124);// angle pour lever
    delai_multiple_cycle_complet(100);
    start_bras_gauche_avant(70);
    delai_multiple_cycle_complet(100);
    start_coude_gauche_avant(56);//angle pour poser
    delai_multiple_cycle_complet(100);

    // on rame 
    start_bras_gauche_arriere(110);
    start_bras_gauche_avant(110);
    start_bras_droit_arriere(70);
    start_bras_droit_avant(70);
    delai_multiple_cycle_complet(100);
    

    // deplacement du bras doit arriere
    start_coude_droit_arriere(124);// angle pour lever
    delai_multiple_cycle_complet(100);
    start_bras_droit_arriere(110);
    delai_multiple_cycle_complet(100);
    start_coude_droit_arriere(56);//angle pour poser
    delai_multiple_cycle_complet(100);

    
    // deplacement du bras doit avant
    start_coude_droit_avant(124);// angle pour lever
    delai_multiple_cycle_complet(100);
    start_bras_droit_avant(110);
    delai_multiple_cycle_complet(100);
    start_coude_droit_avant(56);//angle pour poser
    delai_multiple_cycle_complet(100);
    
    // on rame
    start_bras_gauche_arriere(110);
    start_bras_gauche_avant(110);
    start_bras_droit_arriere(70);
    start_bras_droit_avant(70);
  }*/

  void pivot()//droite
  {
	  // deplacement du bras gauche avant
	  OCR1B=2451;   // start_coude_gauche_avant(124);// angle pour lever de 1cm: 90°+34°
	  delai_multiple_cycle_complet(150);
	  OCR1A=1651;   //start_bras_gauche_avant(110);// angle de déplacement vers l'avant
	  delai_multiple_cycle_complet(150);
	  OCR1B=2000;   //start_coude_gauche_avant(90);//angle pour baisser de 1cm: 90°-34°
	  delai_multiple_cycle_complet(150);
	  
	  // deplacement du bras doit arriere
	  OCR4B=2451;//start_coude_droit_arriere(124);// angle pour lever
	  delai_multiple_cycle_complet(150);
	  OCR4A=1651;//start_bras_droit_arriere(70);
	  delai_multiple_cycle_complet(150);
	  OCR4B=2000;//start_coude_droit_arriere(90);//angle pour poser
	  delai_multiple_cycle_complet(150);
	  
	  // deplacement du bras gauche arriere
	  OCR3C=1647;//start_coude_gauche_arriere(124);// angle pour lever
	  delai_multiple_cycle_complet(150);
	  OCR3B=1651;//start_bras_gauche_arriere(110);
	  delai_multiple_cycle_complet(150);
	  OCR3C=2000;//start_coude_gauche_arriere(90);//angle pour poser
	  delai_multiple_cycle_complet(150);

	  // deplacement du bras doit avant
      OCR3A=1647;//start_coude_droit avant(124);// angle pour lever
      delai_multiple_cycle_complet(150);
      OCR1C=1647;//start_bras_droit_avant(70);
      delai_multiple_cycle_complet(150);
      OCR3A=2000;//start_coude_droit_avant(90);//angle pour poser
      delai_multiple_cycle_complet(150);

    // on rame les deux bras droite avec un angle x et les deux bras gauche avec un angle opposé pour pouvoir tourner
	OCR1C=2000;
	OCR3B=2000;
	OCR4A=2000;
	OCR1A=2000; 
  } 

  ISR(TIMER5_OVF_vect)
  {
	PORTL|=(1<<PL2); //allumage d'une led
	
	position_initiale();//permet de faire une mesure en étant stable et de repartir correctement ensuite.
	
	TIFR2|=(1<<TOV2);// mise à 0 de TOV2
    //Envoie d'une impulsion de 10 us
    PORTL |=(1<<PL0);
    delai_10us();
	PORTL&=~(1<<PL0);
    
    while((PINL & (1<<PL1)) == 0){}//attente de la réponse
	TCNT2=0; // initialisation du timer 2 à 0.
    while((PINL & (1<<PL1)) == 1){ // la réponse arrive: lecture
		
		mesure=TCNT2;  //On met à jour la mesure à chaque passage dans la boucle while
		
		if ((TIFR2 & 0x01)==1){// si on arrive au max, on sait que l'obstacle, est plus loin qu'1m70, pas intéressant.
			test=1;// permet de savoir si on est arrivé au max.
		}
    }
	
	if (test==1)mesure=255; // on peut directement charger la valeur max, on sait qu'on est arrivé à 0.
	mesure=mesure*1024/16000000;//permet d'avoir une mesure du temps passé à l'état haut sur echo.
    valeur=mesure/0.000010; // Chaque valeur numérique dure 10us àl'état haut sur le pin echo.
	D=17*valeur/100;// On obtient une valeur en cm qui correspond à la distance entre l'objet et le robot.
	
	if(D<=consigne){
		pivot();
		pivot();
    }
    
	TCNT5=18594;
	PORTL&=~(1<<PL2); //on éteint la led
  }
  
    void config_timer5()
    {
	    TCCR5A&=~(1<<WGM50)&~(1<<WGM51);
	    TCCR5B&=~(1<<WGM52)&~(1<<WGM53);
	    // configuration en mode normal
	    
	    TCCR5A&=~(1<<COM5A1)&~(1<<COM5B1)&~(1<<COM5C1)&~(1<<COM5A0)&~(1<<COM5B0)&~(1<<COM5C0);
	    
	    // prescaler de 1024
	    TCCR5B|=(1<<CS50)|(1<<CS52);
	    TCCR5B&=~(1<<CS51);
	    TCNT5= 297;     //(4.19-Temps)/64µs: temps=4sec
    }
  

  void config_capteur_ultrason()
  {
    
    DDRL|=(1<<PL0);//broche pour le trig du HC-sr04 (en sortie de l'arduino)
    DDRL&=~(1<<PL1);//broche pour le echo du HC-sr04 (en entrée de l'arduino)
    DDRL|=(1<<PL2);//broche pour allumage d'une led
    /// mode normal
    TCCR2A&=~(1<<COM2A1)&~(1<< COM2A0);
    TCCR2A&=~(1<< COM2B1)&~(1<< COM2B0);
    TCCR2A&=~(1<< WGM01) &~(1<< WGM00);
    
    // divisieur de frequence  avec un prescaller de 1024
    TCCR2B|=(1<< CS20)|(1<<CS22);
    TCCR2B&=~(1<< CS21)&~(1<<WGM22);
    
    TIFR2=TIFR2|0b00000001;    
	TIMSK5|=(1<<TOIE5);
  }


int main()
{
  cli();
  config_capteur_ultrason();
  config_timer5();
  config_pwm();
  position_initiale();
  delai_multiple_cycle_complet(2000);

  //sei();
  
  while (1)
  {
	 marche_avant();
	 pivot();
	 pivot();
	// pivot();
	 //pivot();
  }
 
  return 0;
}

