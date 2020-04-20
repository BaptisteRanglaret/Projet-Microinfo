#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H


//Pi regulator function
int16_t pi_regulator(float angle, float goal);

//Declares the converter function
float convertisseur_value_dist(float value);

//Declares the angle computation function
float calcul_angle (float dist2, float dist4);

#endif /* PI_REGULATOR_H */
