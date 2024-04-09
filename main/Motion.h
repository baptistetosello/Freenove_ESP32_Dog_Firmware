/**
 * @file Motion.h
 * @author suhayl@freenove (support@freenove.com)
 * @brief Code of motion, Including coordinate angle conversion, walking posture control, body length definition and so on.
 * @version v1.0.0
 * @date 2022-04-13
 * 
 * @copyright Copyright (c) 2022. Freenove corporation.
 * 
 */

#ifndef _MOTION_h
#define _MOTION_h

#include "Public.h"
// Remettre la vitesse entre les bornes [min , max]
void setMoveSpeed(int spd);

// Bouger les 4 pieds d'une position de départ (dans leur repère) à une position d'arrivée (dans leur repère) 
// 2 mouvements : P0P,P2P font un mouvement TRIANGULAIRE pdt que P1,P3 poussent vers l'arrière, puis inversement du rôle des pattes 
void move_leg_to_point_directly(float (*startPt)[3], float (*endPt)[3]);

// Recalculer la position des pattes (angles des servos) pour éviter d'avoir des distances [PnM1;PnP] > DR 
void cooToA_All(float (*Pt)[3], float (*ag)[3]);

// Appeler la fonction setBodyHeight qui met le robot debout à la hauteur "BODY_HEIGHT_DEFAULT" si isRobotStanding = false , puis --> isRobotStanding = true
void resumeStanding();

// Inutile (à part peut être pour modifier la variable globale Last_Pt ??)
void standUp();

// Bouger les 4 pieds d'une position de départ (dans leur repère) à une position d'arrivée (dans leur repère) 
// 2 mouvements : P0P,P2P font un mouvement TRAPEZOIDAL pdt que P1,P3 poussent vers l'arrière, puis inversement du rôle des pattes 
// void move_step_by_step(float (*startPt)[3], float (*end__Pt)[3]);
void move_step_by_step(float (*startPt)[3], float (*end__Pt)[3], int spd = 5);

// Calculer la position finale (newPoint) de chaque pied dans son repère à partir d'une direction (angle) et d'une longueur de déplacement (translation pure)
// et d'un angle de Spin (rotation pure autour de Y)
// Appeler move_step_by_step(lastPt, newPt, spd) qui doit réaliser le mouvement souhaité
// void move_any(int alpha, float stepLength, int gama);
void move_any(int alpha, float stepLength, int gama, int spd = 5);
void move_any2(int alpha, float stepLength, int gama, int spd = 5);
void move_any3(int alpha, float stepLength, int gama, int spd = 5);

// Bouger les 4 pieds d'une position de départ (dans leur repère) à une position d'arrivée (dans leur repère) 
// 1 mouvement : les 4 pattes se déplacent simultanéments, les PnP se déplacent en ligne droite 
// void action_twist(float (*startPt)[3], float (*end__Pt)[3], float tickLength);
void action_twist(float (*startPt)[3], float (*end__Pt)[3], float tickLength, bool isContainedOffset = true);

// Calculer une nouvelle position des 4 pieds dans leur repère à partir de 3 angles, puis appeler action_twist pour mvmt simultané des pieds en ligne droite
void twist_any(int alpha, int beta, int gama);

//Mettre le robot à la hauteur h (vertical axe -Y) souhaitée (appel de action_twist --> mvmt en ligne droite des pieds)
void setBodyHeight(int h);


void setBeforeCalibrationHeight(int h);
void setServoToInstallationPosition();
void setServoToBeforeCalibrationPosition();
void moveOneLegToPointDirectly(uint8_t n, bool b, float(*startPt), float(*end__Pt));
void updateLegx(u8 x);
void updateLegxWithoutOffset(u8 n);
void updateServoAngle(bool b = true);
void cooToA_All(float (*Pt)[3], float (*ag)[3]);
void cooToA(float x, float y, float z, float *abc);
void cooToA(float *xyz, float *abc);
void getServoOffsetFromStorage();



void angle_speed_move(float angles_arr [2][2], int role,  int nb_paliers);
void adapt_stride_and_move(float GO_stride [3][20],float pas_temps, int nbr_pas_cycle,int role) ;
void set_servos_angles (float angles[2][3], int role);

#endif
