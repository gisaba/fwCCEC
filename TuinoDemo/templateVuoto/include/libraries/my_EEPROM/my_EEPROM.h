/*
 * my_EEPROM.h
 *
 * Created: 23/05/2021 15:10:50
 *  Author: Giovanni Barbato
 */ 

#include <EEPROM.h>

typedef struct Erogazioni_struct
{
	String da_memorizzare;
	int startIND;
} Erogazioni_TypeDef;

typedef struct ParametriCCEC_struct
{
	String descrizione;
	String da_memorizzare;
	int startIND;
} ParametriCCEC_TypeDef;

ParametriCCEC_TypeDef *ParametriCCEC; //puntatore  a Map memoria

Erogazioni_TypeDef *ErogazioniEEPROM; //puntatore  a Map memoria

#define numero_parametri 3

ParametriCCEC_TypeDef Parametri[numero_parametri] =
{ {"DNS NAME SERVER\0", "ccec.sa.dipvvf.it\0", 1000},
{"IP CCEC\0", "192.168.0.50\0", 1020},
{"Impulsi al litro\0", "100\0", 1035}
};

#define numero_Erog_salvate 3

Erogazioni_TypeDef Erogazioni[numero_Erog_salvate] =
{	{"DD92743A;28530;D;50.37;1111;000;SA1001\0", 2000},
	{"DD92743A;22530;D;5.37;1111;000;SA1001\0", 2040},
	{"DD92743A;26530;D;180.37;1111;000;SA1001\0", 2080}
};

void clearEEPROM(int ind_from, int ind_to);
String read_eeprom_string_struct(ParametriCCEC_TypeDef dato);
bool write_eeprom_string_struct(ParametriCCEC_TypeDef dato);
bool write_erogazione_eeprom(Erogazioni_TypeDef dato);
String read_erogazione_eeprom(Erogazioni_TypeDef dato);