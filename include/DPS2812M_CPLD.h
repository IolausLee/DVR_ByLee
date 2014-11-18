#ifndef _register
#define _register

unsigned int * CPLD_DIDO = (unsigned int *) 0x2C00;
unsigned int * CPLD_SCR = (unsigned int *) 0x2D00;
unsigned int * CPLD_ADCSET = (unsigned int *) 0x2E00;


/*struct DI_BITS
{
	unsigned int DI10:1;
	unsigned int DI11:1;
	unsigned int DI12:1;
	unsigned int DI13:1;
	unsigned int DI14:1;
	unsigned int DI15:1;
	unsigned int DI16:1;
	unsigned int DI17:1;
};

struct DO_BITS
{
	unsigned int DO8:1;
	unsigned int DO9:1;
	unsigned int DO10:1;
	unsigned int DO11:1;
	unsigned int DO12:1;
	unsigned int DO13:1;
	unsigned int DO14:1;
	unsigned int DO15:1;
};



union CPLD_DIDO
{
struct DI_BITS DI;
struct DO_BITS DO;
};

extern volatile union CPLD_DIDO DIDO;
unsigned int * DIDO = (unsigned int *)0x2C00;  */

#endif