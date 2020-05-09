#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

#define CONST_EQ_DIST_1			2
#define CONST_EQ_DIST_2			6
#define CONST_EQ_DIST_3			120
#define CONST_EQ_DIST_4			13760


//P regulator function
int16_t p_regulator(float angle, float goal);

//Declares the converter function
float convertisseur_value_dist(float value);

//Declares the angle computation function
float calcul_angle (float dist2, float dist4);

#endif /* PI_REGULATOR_H */
