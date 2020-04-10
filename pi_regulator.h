#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//start the PI regulator thread
void pi_regulator_start(void);

//Declares the converter function
float convertisseur_value_dist(float value);

#endif /* PI_REGULATOR_H */
