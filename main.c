/*
 * Max7219.c
 *
 * Created: 5/23/2022 3:14:02 PM
 * Author : Bodden007
 */ 

#define F_CPU 8000000L
#include <avr/io.h>
#include <avr/delay.h>
#include <math.h>
#include <avr/interrupt.h>

#define INT0_Pin	PD2

#define SPI_DDR		DDRB
#define SPI_PORT	PORTB
#define SPI_SS		PB2
#define SPI_MOSI	PB3
#define SPI_MISO	PB4
#define SPI_SCK		PB5
#define St_ADC		PC1
#define St_Freq		PC2
#define St_PWM		PC3

	volatile int DataFreq = 0;
	volatile int DataTime = 0;

char d[19] ={												//character encoding for a seven-segment indicator
	0x7E,													//0
	0x30,													//1
	0x6D,													//2
	0x79,													//3
	0x33,													//4
	0x5B,													//5
	0x5F,													//6
	0x70,													//7
	0x7F,													//8
	0x7B,													//9
	0x1,													//-		10
	0x77,													//a		11
	0x1F,													//b		12
	0x4E,													//c		13
	0x3D,													//d		14
	0x4F,													//e		15
	0x47,													//f		16
	0x80,													//.		17
	0x00													//NON	18
};

int Kfactor (int DataADC)
{
			
	if ((DataADC>=480) && (DataADC<=611)) {DataADC = (DataADC-480)/0.13;}
		else if ((DataADC>611) && (DataADC<=677)) {DataADC = (DataADC-480)/0.131;}
			else if ((DataADC>677) && (DataADC<=743)) {DataADC = (DataADC-480)/0.1315;}
				else if ((DataADC>743) && (DataADC<=809)) {DataADC = (DataADC-480)/0.1316;}
					else if ((DataADC>809) && (DataADC<=875)) {DataADC = (DataADC-480)/0.1316;}
						else if ((DataADC>875) && (DataADC<=955)) {DataADC = (DataADC-480)/0.1319;}
		
		DataADC = DataADC/10;	
				
	return DataADC;
	
}

void Razryd(int CorADC, int *pRazr2, int *pRazr3, int *pRazr4)
{
	
	*pRazr4 = CorADC%10;
	*pRazr3 = CorADC%100/10;
	*pRazr2 = CorADC%1000/100;
	
}

void INT0_init()
{
	DDRD &=~(1<<INT0_Pin);
	PORTD |=(1<<INT0_Pin);
	GICR |=(1<<INT0);
	MCUCR |=(1<<ISC01) | (1<<ISC00);
}

ISR (INT0_vect)
{
	DataFreq++;
}

void T0_init()
{
	TCCR0 |=(1<<CS01);            //8
	TIMSK |=(1<<TOIE0);							// interrupt ON
}

ISR (TIMER0_OVF_vect)
{
	TCNT0 = 6;
	DataTime++;
}

ISR(ADC_vect)
{
		
	ADCSRA |=(1<<ADSC);
}

void ADC_init()
{
	ADCSRA |=(1<<ADEN)|(1<<ADSC)|(1<<ADFR)|(1<<ADIE)
	|(1<<ADPS2)|(1<<ADPS1);									//ADC - ON, divider - 64
	ADMUX |=(1<<REFS0);										//External ION
}

void SPI_init()
{
	SPI_DDR = (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS);		//setting up MOSI, SCK, SS as outputs
	SPI_PORT |=(1<<SPI_SS);									//setting SS to 1
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);			//through the SPSR register, we configure the hardware SPI
	
	spi(0x0C,0x00);											//Disabling indicators
	spi(0x09,0x00);											//Disabling decoding
	spi(0x0A,0x0A);											//The intensity of the glow of the indicators
	spi(0x0B,0x04);											//Number of indicators starting from 0
	spi(0x0F,0x00);											//Disabling the indicator test
	spi(0x0C,0x01);											//Enabling indicators
	
}

void spi(char cmd,char data)								//The function of transmitting two 8-bit packets over the SPI protocol
{
	
	SPI_PORT &= ~(1<<SPI_SS);								//reset SS to 0
	SPDR = cmd;												//we send the data to the SPI address
	while(!(SPSR&(1<<SPIF)));								//we are waiting for the end of sending
	SPDR = data;											//we send data by SPI data
	while(!(SPSR&(1<<SPIF)));								//we are waiting for the end of sending
	SPI_PORT |= (1<<SPI_SS);								//setting SS to 1
	
}

void Send_SPI(char d[], int Razr1, int Razr2, int Razr3, int Razr4)
{
	int Dp = 0x80;
	
	spi(1,d[Razr1]);
	spi(2,d[Razr2]);
	spi(3,d[Razr3] | Dp);
	spi(4,d[Razr4]);
}

void clrdig ()												//Indicator cleaning function
{
	spi(0x01,d[17]);
	spi(0x02,d[17]);
	spi(0x03,d[17]);
	spi(0x04,d[17]);
	spi(0x05,d[17]);
	
}

int main(void)
{
// 	DDRC &= (~((1<<1) | (1<<2) | (1<<3)));			//pin off
// 	PORTC &= (~((1<<1) | (1<<2) | (1<<3)));
	DDRD |= (1<<7);
	PORTD &=~(1<<7);
	
	int DataADC = 0;
	int CorADC = 0;
	

	
	int Razr1 = 0;
	int Razr2 = 0;
	int Razr3 = 0;
	int Razr4 = 0;
	
	
	/*sei();*/
	SPI_init();
	clrdig();												//Clearing all indicators
	
    while(1){	
 		if (!(PINC & (1<<St_ADC)))	
			{	
			TIMSK &=~(1<<TOIE0);
									
			ADC_init();
									
			while (!(PINC & (1<<St_ADC)))
				{
																
									
				DataADC = ADC;
						
				if (DataADC <= 2 ){Razr1=10; Razr2=10; Razr3=10; Razr4=10;}
					else if ((DataADC >= 0) && (DataADC < 479)){Razr1=10; CorADC = (480-DataADC)/1.323; Razryd(CorADC, &Razr2, &Razr3, &Razr4);}
						else if (DataADC == 479){Razr1=18; CorADC=0; Razryd(CorADC, &Razr2, &Razr3, &Razr4);}
							else if (DataADC > 955){Razr1=10; Razr2=10; Razr3=10; Razr4=10;}
								else if (DataADC >= 480){Razr1=18; CorADC = Kfactor(DataADC); Razryd(CorADC, &Razr2, &Razr3, &Razr4);}
													
				Send_SPI(d, Razr1, Razr2, Razr3, Razr4);												
					    											
				DataADC = 0;
								
						}
			}
				
			 else if (!(PINC & (1<<St_Freq)))
				{
				
				
					
				ADCSRA &=~(1<<ADEN); clrdig();
				
				INT0_init();
				T0_init();
				
				Razr1=10; Razr2=10; Razr3=10; Razr4=10;
				Send_SPI(d, Razr1, Razr2, Razr3, Razr4);
				
				while (!(PINC & (1<<St_Freq)))
					{
					
					
					if (DataTime == 1000)
					{
						DataFreq = DataFreq * 4;
						Razryd(DataFreq, &Razr2, &Razr3, &Razr4);
						Send_SPI(d, Razr1, Razr2, Razr3, Razr4);
						
						DataFreq=0;
						DataTime=0;
					}
					
					
				}
			}
			
			else if ((!(PINC & (1<<St_ADC))) && (!(PINC & (1<<St_Freq))))
			{

				{
					PORTD |=(1<<7);
				}
			}
						
			
			
			}
    
}



