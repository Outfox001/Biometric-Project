#ifndef __DHT11_H
#define __DHT11_H


// Definirea stării senzorului
#define DHT11_OK         0
#define DHT11_ERROR      -1

// Functii de citire a datelor de la senzor
int DHT11_ReadData(float *temperature, float *humidity);

#endif /* __DHT11_H */
