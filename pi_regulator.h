#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//starts the PI regulator thread
void pi_regulator_start(void);

//Declares the converter function
float convertisseur_value_dist(float value);

//Starts the depassement thread
void depassement_start(void);

#endif /* PI_REGULATOR_H */
