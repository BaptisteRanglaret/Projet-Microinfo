#ifndef MOUVEMENT_H
#define MOUVEMENT_H

#include <stdint.h>
#include <hal.h>

//Démarre la thread pour les clignotants
void clignotant_start(void);

//Démarre la thread pour le dépassement
void depassement_start(void);

//Démarre la thread pour la manoeuvre
void manoeuvre_start(void);

//Démarre la thread pour le déplacement
void deplacement_start(void);


#endif /* MOUVEMENT_H */


