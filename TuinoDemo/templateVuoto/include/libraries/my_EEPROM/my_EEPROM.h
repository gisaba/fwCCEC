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

#define numero_parametri 4

ParametriCCEC_TypeDef Parametri[numero_parametri] =
{ {"DNS NAME SERVER\0", "ccec.sa.dipvvf.it\0", 1000},
  {"IP CCEC\0", "192.168.0.50\0", 1020},
  {"Ultima Erogazione Salvata\0", "2000\0", 1035},
  {"Codice Sede\0", "SA1001\0", 1040}
};