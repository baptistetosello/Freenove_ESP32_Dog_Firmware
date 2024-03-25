/**
 * @file Motion.cpp
 * @author suhayl@freenove (support@freenove.com)
 * @brief Code of motion, Including coordinate angle conversion, walking posture control, body length definition and so on.
 * @version v1.0.0
 * @date 2022-04-13
 * 
 * @copyright Copyright (c) 2022. Freenove corporation.
 * 
 */

#include "Motion.h"

#define TICK_MS 10 //The time length of each tick, the unit is ms, and the coordinate point is updated every tick.
#define STEP_HEIGHT 15 // 15 MPM
// define leg length (hauteur d'un pas)
// L1: Root
#define L1 23.0
// L2: Thigh length (cuisse)
#define L2 55.0
// L3: Calf length ：55+4 (mollet)
#define L3 59.0
// Define active radius, DR = L2+L3-10 (rayon actif défini) 
#define DR 100.0 // (104 ?)

/**
 * @brief define the body length and width , between adjacent servo.
 * @LEN_BD: body length / 2 = 136.4 / 2.
 * @WID_BD: body width / 2 = 80 / 2.
 */
constexpr float LEN_BD = 68.2, WID_BD = 40;

/**
 * @brief define the hypotenuse
 *
 */
constexpr float v = sqrt(square(LEN_BD) + square(WID_BD));

/**
 * @brief define the theta
 *
 */
constexpr float theta = atan(WID_BD / LEN_BD);

// The movement speed of the leg end point. Too fast speed will damage the servo. Unit: x mm / 10ms
#define SPEED_MIN 1
#define SPEED_MAX 8
int moveSpeed = 5;

// Height
int bodyHeight = BODY_HEIGHT_DEFAULT;

Freenove_PCA9685 pca = Freenove_PCA9685();

bool isRobotStanding = false;
bool isRobotMoving = false;
// last point legs
float lastPt[4][3]; // matrice 4*3 --> 4 pattes, 3 moteurs par pattes

// legs index: 0: Left front, 1: Left rear, 2: Right rear, 3: Right front.
// float calibratePosition[4][3] = {{0, 99, 10}, {0, 99, 10}, {0, 99, -10}, {0, 99, -10}};
// The default coordinate of quadruped is also the calibration coordinate point
// corrdonnées par défaut de PxP dans le repère XxYxZx
float calibratePosition[4][3] = {{10, 99, 10}, {10, 99, 10}, {10, 99, -10}, {10, 99, -10}};
// Used to save the servo offset, in radians
float servoOffset[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
// legAngles ,global variable, Used to save the servo angle ,== abc
float las[4][3];
// The robot origin point position when moving. It will be changed based on the calibration point.
float movingOriginPos[4][3];

void setMoveSpeed(int spd)
{
	moveSpeed = constrain(spd, SPEED_MIN, SPEED_MAX); // remet la vitesse entre les bornes [min , max]
}

/**
 * @brief //Move the leg 0, 2, 1, 3 to the target point without moving the body.
 * 
 * @param startPt 
 * @param end__Pt 
 */


// fonction de mvmt des PnP d'un point de départ vers un point d'arrivée :
void move_leg_to_point_directly(float (*startPt)[3], float (*end__Pt)[3])
	// Mouvement en deux 2 fois 2 temps : 
	// 		- mouvement triangle (2 sous mouvements) des 2 jambes qui avance et mouvement arrière des pattes qui poussent
	// 		- idem en inversant le rôle des paires de jambes (0,2 --> 1,3) (1,3 --> 0,2)
{ 
	//Triangle track. The parameters represent the coordinates of the foot point.
	float lenghtP2P = 0;
	for (int i = 0; i < 4; i++)
	{
		// calcul norme vecteur du déplacement de chacune des 4 pattes  :
		float tmpLen = sqrt(square(end__Pt[i][0] - startPt[i][0]) + square(end__Pt[i][1] - startPt[i][1]) + square(end__Pt[i][2] - startPt[i][2]));
		// si (tmpLenP2P > tmpLen) alors lenghtP2P=lenghthP2P, sinon lenghtP2P=tmpLen :
		// donc on garde le plus grand entre tmpLen et leghtP2P
		lenghtP2P = lenghtP2P > tmpLen ? lenghtP2P : tmpLen;
	}
	// distance_deplacement / vitesse_deplacement --> temps_deplacement
	int stepTicks = round(lenghtP2P / moveSpeed) + 2; // temps_deplacement +2

	// k correspond à 2* la hauteur du pas (montée descente de la patte ?) / la durée du déplacement
	// le signe moins pour monter avant de descendre (axe y vers le bas ?)
	float k = -2 * STEP_HEIGHT / stepTicks; //, v = STEP_LENGTH / TICKS; // H=(-)kT/2, S = vT
	float trackPt[4][3];
	u8 a = 0, c = 2; //, b = 1, d = 3
	for (int i = 0; i < 2; i++)
	{
		if (i == 0)
		{
			a = 0;
			// b = 1;
			c = 2;
			// d = 3;
		}
		else
		{
			a = 1;
			// b = 0;
			c = 3;
			// d = 2;
		}
		// First move leg 0 and 2. Then move leg 1 and 3. // 
		for (int t = 0; t <= stepTicks; t++)
		{
			for (int j = 0; j < 4; j++) //
			{	// on met à jour les TrackPt des 4 pattes 
				// d'abord selon l'axe x
				trackPt[j][0] = startPt[j][0] + t * (end__Pt[j][0] - startPt[j][0]) / stepTicks;
				// puis selon l'axe y (montée et descente)
				if (t < stepTicks / 2)
				{
					trackPt[j][1] = startPt[j][1] + k * t;
				}
				else
				{
					trackPt[j][1] = startPt[j][1] - k * t + k * t; // remet au sol instantanément ?? (ou c'est juste le suivi de de position qui revient au sol ?)ou une erreur ?
				}
				// puis selon l'axe z
				trackPt[j][2] = startPt[j][2] + t * (end__Pt[j][2] - startPt[j][2]) / stepTicks;
			}
			// cooToA semble normaliser la position des pattes pour éviter d'avoir une distance [PxM1;PxP] trop grande et donc trop de couple sur les servos
			// cooToA recalcule donc les angles des servos pour aller à la nouvelle position sans avoir [PxM1;PxP] > DR
			cooToA(trackPt[a], las[a]); //trackPt[a] position PaP (xyz), las[a] angles moteurs Pa
			cooToA(trackPt[c], las[c]);
			updateLegx(a); // Update commande servos avec les nouveaux angles
			updateLegx(c);
			delay(TICK_MS); // délai ajouté entre les mouvements des servos (10ms)
		}
	}
	for (int j = 0; j < 4; j++) // on maj la position finale des 4 pattes avec le point obtenu en sortie de la boucle de déplacement
	{
		lastPt[j][0] = trackPt[j][0];
		lastPt[j][1] = trackPt[j][1];
		lastPt[j][2] = trackPt[j][2];
		// Serial.println(">>>>>");
		// Serial.println(String("lastPt: ") + String(lastPt[j][0]) + String(" ") + String(lastPt[j][1]) + String(" ") + String(lastPt[j][2]));
		// Serial.println("<<<<<");
	}
}

// 
void resumeStanding() 
{
	if (!isRobotStanding) // false par défaut
	{
		setBodyHeight(BODY_HEIGHT_DEFAULT); //commande les servos pour mettre le robot à la hauteur souhaitée
		//standUp(); // semble servir à rien CPM
		isRobotStanding = true;
	}
}
void standUp()
{
	for (int j = 0; j < 4; j++) // set la position finale sur la position de calibrage (pour chaque patte)
	{
		lastPt[j][0] = calibratePosition[j][0];
		lastPt[j][1] = calibratePosition[j][1];
		lastPt[j][2] = calibratePosition[j][2];
	}
	move_leg_to_point_directly(lastPt, calibratePosition); // semble n'avoir aucun intérêt (position de départ erronée, position fin identique au départ et déjà normalisée...)
	// les vecteurs déplacement sont nuls --> stepTicks = 2 --> on monte et descend en y brièvement pdt que les pattes opposés poussent ??
	isRobotStanding = true;
}

/**
 * @brief The body has displacement. The starting point is any position. Leg 0, 2, 1, 3 takes one step.
 * 
 * @param startPt 
 * @param end__Pt 
 * @param spd 
 */

// Fonction de déplacement (deux points d'appuis simultanés) : 
// [cordonnées PxP départ] 
// --> 2 * mouvement trapèze pattes mobiles + recul des deux pattes porteuses (en changeant le role des pattes entre les 2)  
// --> [coordonnées PxP arrivée]
void move_step_by_step(float (*startPt)[3], float (*end__Pt)[3], int spd)
{
	//Trapezoidal quadrilateral track. The parameters represent the coordinates of the foot point. Walking gait.
	// Mouvement en deux 2 fois 3 temps : 
	// 		- mouvement trapèze (3 sous mouvements) des 2 jambes qui avance et mouvement arrière des pattes qui poussent
	// 		- idem en inversant le rôle des paires de jambes (0,2 --> 1,3) (1,3 --> 0,2)
	float lenghtP2P = 0;
	//Judge the two legs.
	for (int i = 0; i < 2; i++)
	{
		float tmpLen = sqrt(square(end__Pt[i][0] - startPt[i][0]) + square(end__Pt[i][1] - startPt[i][1]) + square(end__Pt[i][2] - startPt[i][2]));
		lenghtP2P = lenghtP2P > tmpLen ? lenghtP2P : tmpLen;
	}
	spd = constrain(spd, SPEED_MIN, SPEED_MAX); // on normalise la vitesse
	int stepTicks = round(lenghtP2P / spd) + 3; // durée du mouvement complet calculé à partir du plus grand vecteur déplacement d'un PxP et de la vitesse
	// Serial.println(String("stepTicks: ") + String(stepTicks));
	// constante de vitesse k de chaque composante du mouvement 
	float k = -3 * STEP_HEIGHT / stepTicks; //, v = STEP_LENGTH / stepTicks; //H = kT / 2, S = vT
	float trackPt[4][3];
	float tmpEnd__Pt[4][3]; // tmpStartPt[4][3],
	// float startPt[4][3] = { {-20,99,10},{0,99,10},{-20,99,-10},{0,99,-10} };
	// float endPt[4][3] = { {20,99,10},{0,99,10},{20,99,-10},{0,99,-10} };
	//First move leg 0 and 2, p1->p2, forward. Then move leg 1 and 3, p2->p1, reverse.
	u8 a = 0, b = 1, c = 2, d = 3;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			tmpEnd__Pt[i][j] = end__Pt[i][j]; // on stock les positions finales des 4 pattes (en fin de mouvement)
			// on en a besoin car la variable end_Pt va être modifiée pdt le déplacement
		}
	}
	// Serial.println(">====>");
	// for (u8 e = 0; e < 4; e++)
	// {
	// 	Serial.println(String("startPt: ") + String(startPt[e][0]) + String(" ") + String(startPt[e][1]) + String(" ") + String(startPt[e][2]));
	// }
	// Serial.println("====");
	// for (u8 e = 0; e < 4; e++)
	// {
	// 	Serial.println(String("tmpEnd__Pt: ") + String(tmpEnd__Pt[e][0]) + String(" ") + String(tmpEnd__Pt[e][1]) + String(" ") + String(tmpEnd__Pt[e][2]));
	// }
	// Serial.println("<====<");
	// TICKS;
	for (int i = 0; i < 2; i++) // le mouvement est découpé en deux parties 
	{
		if (i == 0) 
		{
			a = 0; // pattes 0 et 2 pour rôle A (a,c) --> patte qui se soulève et avance
			b = 1; // pattes 1 et 3 pour rôle B (b,d) --> patte qui reste au sol et pousse ??
			c = 2; 
			d = 3;
		}
		else
		{
			a = 1; // pattes 1 et 3 pour rôle A
			b = 0; // pattes 0 et 2 pour rôle B
			c = 3; 
			d = 2;
		}
		for (u8 i = 0; i < 3; i++)
		{
			if (i == 1) 
				continue; // on ignore la loop si on a i==1, donc on set les valeurs de end_Pt (position des pattes) sur les axes X et Z uniquement
			end__Pt[a][i] = tmpEnd__Pt[a][i]; // rôle A
			end__Pt[c][i] = tmpEnd__Pt[c][i]; // rôle A
			// les pattes rôle B (qui poussent) doivent d'abord aller vers l'arrière avant de se diriger vers leur position finale
			end__Pt[b][i] = 2 * movingOriginPos[b][i] - tmpEnd__Pt[b][i]; //Rôle B :  End point of the reverse motion leg.
			end__Pt[d][i] = 2 * movingOriginPos[d][i] - tmpEnd__Pt[d][i]; //Rôle B :  End point of the reverse motion leg.
		}
		// Serial.println(">---->");
		// for (u8 e = 0; e < 4; e++) {
		//	Serial.println(String("startPt: ") + String(startPt[e][0]) + String(" ") + String(startPt[e][1]) + String(" ") + String(startPt[e][2]));
		// }
		// Serial.println("----");
		// for (u8 e = 0; e < 4; e++) {
		//	Serial.println(String("end__Pt: ") + String(end__Pt[e][0]) + String(" ") + String(end__Pt[e][1]) + String(" ") + String(end__Pt[e][2]));
		// }
		// Serial.println("<----<");

		for (int t = 0; t <= stepTicks; t++)
		{
			//First calculate the X-axis coordinates of the four points, of which leg 0 and 2 (rôle A) moves forward, and 1 3 and 0 2 move relatively.
			trackPt[a][0] = startPt[a][0] + (end__Pt[a][0] - startPt[a][0]) * t / stepTicks; //
			trackPt[c][0] = startPt[c][0] + (end__Pt[c][0] - startPt[c][0]) * t / stepTicks; //
			trackPt[b][0] = startPt[b][0] + (end__Pt[b][0] - startPt[b][0]) * t / stepTicks; // Legs 1 and 0 are reversed to each other.
			trackPt[d][0] = startPt[d][0] + (end__Pt[d][0] - startPt[d][0]) * t / stepTicks; // Legs 2 and 3 are reversed to each other.
			//Calculate the Y-axis coordinates, in which leg 0 2 (rôle A) lifts and leg 1 3 (rôle B) does not.
			// on décompose le mouvement des pattes rôle A en 3
			if (t < stepTicks / 3) // la patte monte phase 1/3
			{
				trackPt[a][1] = end__Pt[a][1] + k * t;
				trackPt[c][1] = end__Pt[c][1] + k * t;
			}
			else if (t < stepTicks * 2 / 3) // pas de mouvement vertical dans la phase 2/3
			{
			}
			else
			{
				trackPt[a][1] = end__Pt[a][1] - k * t + k * stepTicks; // la patte descend en phase 3/3
				trackPt[c][1] = end__Pt[c][1] - k * t + k * stepTicks;
			}
			trackPt[b][1] = end__Pt[b][1]; // on met les pattes rôle B à leur coordonnée en Y finale (la patte rôle B reste au solo)
			trackPt[d][1] = end__Pt[d][1];

			// The Z axis calculation method is the same as the X axis.
			trackPt[a][2] = startPt[a][2] + (end__Pt[a][2] - startPt[a][2]) * t / stepTicks; //
			trackPt[c][2] = startPt[c][2] + (end__Pt[c][2] - startPt[c][2]) * t / stepTicks; //
			trackPt[b][2] = startPt[b][2] + (end__Pt[b][2] - startPt[b][2]) * t / stepTicks; // Legs 1 and 0 are reversed to each other.
			trackPt[d][2] = startPt[d][2] + (end__Pt[d][2] - startPt[d][2]) * t / stepTicks; // Legs 2 and 3 are reversed to each other.
			cooToA_All(trackPt, las); // normalise la position des pattes
			updateServoAngle(); // MAJ la commande des servos (4 pattes)
			delay(TICK_MS); // délai entre deux mouvements
			// for (int j = 0; j < 4; j++) {
			//	Serial.println(String("trackPt: ") + String(trackPt[j][0]) + String(" ") + String(trackPt[j][1]) + String(" ") + String(trackPt[j][2]));
			// }
			// Serial.println();
		}
		// delay(1000);

		for (u8 i = 0; i < 4; i++)
		{
			for (u8 j = 0; j < 3; j++)
			{
				// end__Pt[i][j] = -tmpEnd__Pt[i][j];//tmpStartPt[i][j];
				startPt[i][j] = trackPt[i][j]; // on remet toutes les coordonnées des pattes à jour avant d'amorcer la deuxième partie du mouvement
			}
		}

		// Serial.println(">>>>>>");
		// for (u8 e = 0; e < 4; e++) {
		//	Serial.println(String("startPt: ") + String(startPt[e][0]) + String(" ") + String(startPt[e][1]) + String(" ") + String(startPt[e][2]));
		// }
		// Serial.println("----");
		// for (u8 e = 0; e < 4; e++) {
		//	Serial.println(String("end__Pt: ") + String(end__Pt[e][0]) + String(" ") + String(end__Pt[e][1]) + String(" ") + String(end__Pt[e][2]));
		// }
		// Serial.println("<<<<<<");
	}
	// Serial.println(">>>>>");
	for (int j = 0; j < 4; j++)
	{
		lastPt[j][0] = trackPt[j][0]; // on remet toutes les coordonnées des pattes à jour (déjà fait plus haut , semble redondant)
		lastPt[j][1] = trackPt[j][1];
		lastPt[j][2] = trackPt[j][2];
		// Serial.println(String("lastPt: ") + String(lastPt[j][0]) + String(" ") + String(lastPt[j][1]) + String(" ") + String(lastPt[j][2]));
	}
	// Serial.println("<<<<<");
}



// alpha is the moving direction, with the x-axis direction as 0 degrees, counterclockwise as positive, clockwise as negative. The unit is radians.
// StepLength is step distance.
/**
 * @brief Walk command, any direction, any step length, any spin angle, any speed
 *
 * @param alpha is the moving direction, with the x-axis direction as 0 degrees, counterclockwise as positive, clockwise as negative, and the unit is angle, [0-360]. The x direction is the direction of forward movement.
 // alpha direction du mouvement linéaire (translation): (dans le plan XZ)
 // mvmt translation pure --> le centre de gravité se déplace en ligne droite
 * @param stepLength The length of each step (<=20).
 * @param gama Spin angle, in-situ rotation, positive counterclockwise, negative clockwise, in degrees. [0-360].
 * // Angle de rotation (dégré) joystick droit de la commande ? --> positif dans le sens horaire (tourne autour de l'axe x ? dans le plan de THETA)
   // mvmt rotation pure --> le centre de gravité ne bouge pas
 * @param spd Movement speed, unit：mm / 10ms.  [1,8]
 */


void move_any(int alpha, float stepLength, int gama, int spd) 
	// l'argument alpha est un angle en degrés (direction du déplacement linéaire [TRANSLATION]) --> Joystick gauche angle
	// l'agument stepLength semble être la longueur d'un pas [TRANSLATION] --> Joystick droit profondeur
	// l'argument gama correspond au spin [ROTATION]--> joystick droit angle
	// LA FONCTION MOVE ANY CALCULE LA POSITION FINALE DE CHAQUE PATTE DANS LEUR REPERE POUR QUE L'APPEL DE LA FONCTION MOVE_STEP_BY_STEP
	// CORRESPONDE AU DEPLACEMENT SOUHAITE (arguments)
{
	stepLength /= 2; // stepLength = stepLength/2
	float newPt[4][3];
	float delta_x, delta_z;
	delta_x = stepLength * cos(alpha * PI / 180); //Portrait : projection du pas selon l'axe X (on met alpha en radians) [TRANSLATION]
	delta_z = stepLength * sin(alpha * PI / 180); //Horizontal : projection du pas selon l'axe Z (on met alpha en radians) [TRANSLATION]
	float pt[] = {delta_x, 0, delta_z};			  //Target body coordinates. After each action ends, the body coordinates are regarded as the origin.
	// Coordonnées du corps cible (centre de gravité (repère XYZ)). À la fin de chaque action, les coordonnées du corps sont considérées comme l'origine.

	// memcpy(movingOriginPos, calibratePosition, sizeof(calibratePosition));
	// Body translation coefficient (axe X)
	const float kx_t = 0.3, kz_t = 0.15; 
	// Body inclination coefficient (axe Z)
	const float kx_p = 0.34, kz_p = 0.2;

	//Calculate the center point of each foot when moving. 
	//Based on the calibration point, the center of gravity is offset to the moving direction, and the body is inclined to the moving direction. 
	//..., le centre de gravité est décalé par rapport à la direction du mouvement, et le corps est incliné par rapport à la direction du mouvement
	//When moving, the landing point (pt d'attérissage) of the step is symmetrical to the center point.
	
	// translation : [position de calibrage] --> translation  --> [position de calibrage]
	for (int j = 0; j < 4; j++)
	{
		movingOriginPos[j][0] = calibratePosition[j][0] - delta_x * kx_t; // position par défaut de PxP - translation_x*coeff_translation_x (projeté selon X dans le repère XxYxZx de chaque patte) 
		// cela correspond à un recul de la patte dans son repère ?
		movingOriginPos[j][2] = calibratePosition[j][2] - delta_z * kz_t; // idem projeté sur Z
	}
	
	// altitude (Y) des pattes dans leur repère (Yx) (en fonction de l'inclinaison du robot pdt la translation ? pt d'appuis ?)
	movingOriginPos[0][1] = calibratePosition[0][1] - delta_x * kx_p - delta_z * kz_p; 
	movingOriginPos[1][1] = calibratePosition[1][1] + delta_x * kx_p - delta_z * kz_p; 
	movingOriginPos[2][1] = calibratePosition[2][1] + delta_x * kx_p + delta_z * kz_p; 
	movingOriginPos[3][1] = calibratePosition[3][1] - delta_x * kx_p + delta_z * kz_p; 

	// rotation
	float c = gama * PI / 180; // c = gama angle de spin en radian
	float v_x_sin_theta_p_c = v * sin(theta + c); // spin des pattes Px et Pxo projeté sur Xx et Xx+2
	float v_x_sin_theta_m_c = v * sin(theta - c); // mouvement inverse avec les pattes opposé pour que ca tourne
	float v_x_cos_theta_p_c = v * cos(theta + c); // spin des pattes Px+1 et Pxo+1 projeté sur Zx+1 et Zx+3
	float v_x_cos_theta_m_c = v * cos(theta - c); // mouvement inverse avec les pattes opposé pour que ca tourne

	// Position finale : translation + rotation (projeté sur Xn)
	newPt[0][0] = movingOriginPos[0][0] + pt[0] + v_x_cos_theta_p_c - LEN_BD;
	newPt[1][0] = movingOriginPos[1][0] + pt[0] - v_x_cos_theta_m_c + LEN_BD;
	newPt[2][0] = movingOriginPos[2][0] + pt[0] - v_x_cos_theta_p_c + LEN_BD;
	newPt[3][0] = movingOriginPos[3][0] + pt[0] + v_x_cos_theta_m_c - LEN_BD;

	// Position finale : (projeté sur Yn)
	newPt[0][1] = movingOriginPos[0][1];
	newPt[1][1] = movingOriginPos[1][1];
	newPt[2][1] = movingOriginPos[2][1];
	newPt[3][1] = movingOriginPos[3][1];

	// Position finale : translation + rotation (projeté sur Zn)
	newPt[0][2] = movingOriginPos[0][2] + pt[2] + v_x_sin_theta_p_c - WID_BD; // recul_z ? + tranlation_z + (rotation_z - reference) 
	newPt[1][2] = movingOriginPos[1][2] + pt[2] + v_x_sin_theta_m_c - WID_BD;
	newPt[2][2] = movingOriginPos[2][2] + pt[2] - v_x_sin_theta_p_c + WID_BD;
	newPt[3][2] = movingOriginPos[3][2] + pt[2] - v_x_sin_theta_m_c + WID_BD;

	move_step_by_step(lastPt, newPt, spd);

	// memcpy(calibratePosition, tmpCalibrationPos, sizeof(calibratePosition));
}

void move_any2(int alpha, float stepLength, int gama, int spd) 
	// l'argument alpha est un angle en degrés (direction du déplacement linéaire [TRANSLATION]) --> Joystick gauche angle
	// l'agument stepLength semble être la longueur d'un pas [TRANSLATION] --> Joystick droit profondeur
	// l'argument gama correspond au spin [ROTATION]--> joystick droit angle
{
	stepLength = 10; // stepLength = stepLength/2
	//alpha = 0;
	//gama = 0;
	u8 pa1 = 0 , pa2 = 1, pr1 = 2, pr2 = 3;
	pa1 = 0; // patte qui se soulève et avance
	pa2 = 2; // patte qui se soulève et avance
	pr1 = 1; // patte qui reste au sol et pousse 
	pr2 = 3; // patte qui reste au sol et pousse 

// EQUILIBRE 
	if (stepLength==-1) {
		
		pa1 = 1; // patte qui se soulève et avance
		pa2 = 3; // patte qui se soulève et avance
		pr1 = 0; // patte qui reste au sol et pousse 
		pr2 = 2; // patte qui reste au sol et pousse
		
		// PHASE 1.1 : 
		// pa : 135/90 --> 135/135
		// pr : 135/90 --> nbp
		las[pa1][1]=135;
		las[pa1][2]=135;
		las[pa2][1]=135;
		las[pa2][2]=135;
		las[0][0] = 80;
		updateServoAngle(); // MAJ la commande des servos (4 pattes)
		delay(2000*TICK_MS);
		}
		

	// PIETINEMENT (x2)
	if (stepLength<=5 && stepLength<=0 ) {
		for (int i = 0; i < 2; i++) {
			if (i==1)  {
				pa1 = 1; // patte qui se soulève et avance
				pa2 = 3; // patte qui se soulève et avance
				pr1 = 0; // patte qui reste au sol et pousse 
				pr2 = 2; // patte qui reste au sol et pousse
			}
				// PHASE 1.1 : 
				// pa : 135/90 --> 135/135
				// pr : 135/90 --> nbp
				las[pa1][1]=135;
				las[pa1][2]=135;
				las[pa2][1]=135;
				las[pa2][2]=135;
				updateServoAngle(); // MAJ la commande des servos (4 pattes)
				delay(10*TICK_MS);
				// PHASE 1.2 : 
				// pa : 135/135 --> 135/90 
				// pr : 135/90 --> nbp
				las[pa1][1]=135;
				las[pa1][2]=90;
				las[pa2][1]=135;
				las[pa2][2]=90;
				updateServoAngle(); // MAJ la commande des servos (4 pattes)
				delay(20*TICK_MS); 
		}
	}	

	// FOULEE COURTE (x1)
	if (stepLength>5 && stepLength<=10) {
		float angles_arr [2][2];	
		int role = 0;
		// PHASE 1.1 : 
		// pa : 135/90 --> 135/135
		// pr : 135/90 --> 140/89
		las[0][1]=135;
		las[0][2]=90;
		las[1][1]=135;
		las[1][2]=90;
		angles_arr [0][0] = 135;
		angles_arr [0][1] = 135;
		angles_arr [1][0] = 140;
		angles_arr [1][1] = 89;
		angle_speed_move(angles_arr, role, 5);
		// PHASE 1.2 : 
		// pa : 135/135 --> 135/90 
		// pr : 140/89 --> 146/88
		angles_arr [0][0] = 135;
		angles_arr [0][1] = 90;
		angles_arr [1][0] = 146;
		angles_arr [1][1] = 88;
		angle_speed_move(angles_arr, role, 10);
		// PHASE 2.1 : 
		// pa : 135/90 --> NBP
		// pr : 129/89 --> 135/135
		angles_arr [0][0] = 135;
		angles_arr [0][1] = 90;
		angles_arr [1][0] = 135;
		angles_arr [1][1] = 135;
		angle_speed_move(angles_arr, role, 5);
		// PHASE 2.2 : 
		// pa : 135/90 --> NBP
		// pr : 135/135 --> 135/90
		angles_arr [0][0] = 135;
		angles_arr [0][1] = 90;
		angles_arr [1][0] = 135;
		angles_arr [1][1] = 90;
		angle_speed_move(angles_arr, role, 10); 
		delay(100*TICK_MS); 	
	}		
}

void angle_speed_move(float angles_arr [2][2], int role,  int nb_paliers) 
{
	float angles_dep [2][2];
	if (role ==0) {
		angles_dep [0][0] = las [0][1];
		angles_dep [0][1] = las [0][2];
		angles_dep [1][0] = las [1][1];
		angles_dep [1][1] = las [1][2];
	}
	else {
		angles_dep [0][0] = las [1][1];
		angles_dep [0][1] = las [1][2];
		angles_dep [1][0] = las [0][1];
		angles_dep [1][1] = las [0][2];
	}
	float paliers[2][2]; // [pas_b_a, pas_c_a ; pas_b_r, pas_c_r]
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++){
			paliers [i][j] = (angles_arr[i][j]-angles_dep[i][j])/nb_paliers;
		}
	}
	for (int p = 1; p < nb_paliers+1; p++) {
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 2; j++){
				if (role==0){
					las[i][j+1] = las[i][j+1] + paliers[i][j];
					las[i+2][j+1] = las[i+2][j+1] + paliers[i][j];
				}
				else {
					las[i][j+1] = las[i][j+1] + paliers[1-i][j];
					las[i+2][j+1] = las[i+2][j+1] + paliers[1-i][j];
				}
			}
		}
		Serial.println(String(las[0][0])+String(las[0][0])+String(las[0][0])+String(las[0][0]));
		updateServoAngle(); // MAJ la commande des servos (4 pattes)
		delay(TICK_MS); 
	}
}

/**
 * @brief //Twist the body without moving the body. 
 *
 * @param startPt Starting point
 * @param end__Pt Target point
 * @param tickLength The longest distance of each tick (?) The larger the distance, the faster the movement speed. The default is 5 and the unit is mm.
 * @param isContainedOffset Whether to include calibration information, the default is included.
 */

// Cette fonction calcule le temps de mouvement des pattes à partir de la patte qui parcours la plus grande distance (pt depart/arrivee en arguments)
// Puis déplace les pattes à la vitesse demandée en ligne droite (pas de décomposition du mouvement) (tickLength % vitesse)
// PAS DE DEPLACEMENT DU ROBOT
void action_twist(float (*startPt)[3], float (*end__Pt)[3], float tickLength, bool isContainedOffset)
{
	//The parameter represents the foot point coordinates.
	float trackPt[4][3];	//The longest distance of each tick
	float lenghtP2P = 0;
	for (int i = 0; i < 4; i++)
	{
		float tmpLen = sqrt(square(end__Pt[i][0] - startPt[i][0]) + square(end__Pt[i][1] - startPt[i][1]) + square(end__Pt[i][2] - startPt[i][2]));
		lenghtP2P = lenghtP2P > tmpLen ? lenghtP2P : tmpLen;
	}
	int stepTicks = round(lenghtP2P / tickLength) + 2; // plus grand déplacement d'une patte / tickLength +2 (stepTicks = temps donc tickLength devrait correspondre à une vitesse)
	//The four legs move together starting from the calibration position.
	for (int t = 0; t <= stepTicks; t++)
	{
		for (int j = 0; j < 4; j++)
		{
			trackPt[j][0] = startPt[j][0] + t * (end__Pt[j][0] - startPt[j][0]) / stepTicks;
			trackPt[j][1] = startPt[j][1] + t * (end__Pt[j][1] - startPt[j][1]) / stepTicks;
			trackPt[j][2] = startPt[j][2] + t * (end__Pt[j][2] - startPt[j][2]) / stepTicks;
			// Serial.println(">---->");
			// for (int j = 0; j < 4; j++) {
			// Serial.println(String("trackPt: ") + String(trackPt[j][0]) + String(" ") + String(trackPt[j][1]) + String(" ") + String(trackPt[j][2]));
			// }
			// Serial.println();
			// Serial.println("<----<");
		}
		cooToA_All(trackPt, las); // calcule la position normalisée (limiter distance PxM1 PxP)
		updateServoAngle(isContainedOffset); // maj de la commande des servos (arg = avec ou sans offset)
		vTaskDelay(TICK_MS);
	}

	for (int j = 0; j < 4; j++)
	{
		lastPt[j][0] = trackPt[j][0];
		lastPt[j][1] = trackPt[j][1];
		lastPt[j][2] = trackPt[j][2];
		// Serial.println(">>>>>");
		// Serial.println(String("lastPt: ") + String(lastPt[j][0]) + String(" ") + String(lastPt[j][1]) + String(" ") + String(lastPt[j][2]));
		// Serial.println("<<<<<");
	}
}

void twist_any(int alpha, int beta, int gama)
{
	float a = alpha * PI / 180;
	float b = beta * PI / 180;
	float c = gama * PI / 180;

	float v_x_sin_theta_p_c = v * sin(theta + c);
	float v_x_sin_theta_m_c = v * sin(theta - c);
	float v_x_cos_theta_p_c = v * cos(theta + c);
	float v_x_cos_theta_m_c = v * cos(theta - c);

	float l_x_sin_a = LEN_BD * sin(a);
	float w_x_sin_b = WID_BD * sin(b);

	float newPt[4][3];

	// axe X : (dans le cas où c>0 , LEN_BD - vx.cos(theta+c) >0)
	newPt[0][0] = calibratePosition[0][0] - v_x_cos_theta_p_c + LEN_BD; //on avance P0P de (LEN_BD - v.cos(theta+c)) (>0 si c>0)
	newPt[1][0] = calibratePosition[1][0] + v_x_cos_theta_m_c - LEN_BD; //on avance P1P de (v.cos(theta-c) - LEN_BD) (>0 si c>0)
	newPt[2][0] = calibratePosition[2][0] + v_x_cos_theta_p_c - LEN_BD; //on recule P2P de (v.cos(theta+c) - LEN_BD) (<0 si c>0)
	newPt[3][0] = calibratePosition[3][0] - v_x_cos_theta_m_c + LEN_BD; //on recule P3P de (LEN_BD - v.cos(theta-c)) (<0 si c>0)

	newPt[0][1] = calibratePosition[0][1] + l_x_sin_a + w_x_sin_b;
	newPt[1][1] = calibratePosition[1][1] - l_x_sin_a + w_x_sin_b;
	newPt[2][1] = calibratePosition[2][1] - l_x_sin_a - w_x_sin_b;
	newPt[3][1] = calibratePosition[3][1] + l_x_sin_a - w_x_sin_b;

	newPt[0][2] = calibratePosition[0][2] - v_x_sin_theta_p_c + WID_BD;
	newPt[1][2] = calibratePosition[1][2] - v_x_sin_theta_m_c + WID_BD;
	newPt[2][2] = calibratePosition[2][2] + v_x_sin_theta_p_c - WID_BD;
	newPt[3][2] = calibratePosition[3][2] + v_x_sin_theta_m_c - WID_BD;

	// Serial.println(">>>>>");
	// for (int j = 0; j < 4; j++) {
	//	Serial.println(String("newPt: ") + String(newPt[j][0]) + String(" ") + String(newPt[j][1]) + String(" ") + String(newPt[j][2]));
	// }
	// Serial.println("<<<<<");
	action_twist(lastPt, newPt, 5);
	// move_body_step_by_step(lastPt, newPt);
}
void setBodyHeight(int h) //  met le robot à la hauteur h (vertical axe -Y) souhaitée
{
	float newPt[4][3];
	h = constrain(h, BODY_HEIGHT_MIN, BODY_HEIGHT_MAX); // on borne h
	bodyHeight = h;
	newPt[0][0] = calibratePosition[0][0];
	newPt[1][0] = calibratePosition[1][0];
	newPt[2][0] = calibratePosition[2][0];
	newPt[3][0] = calibratePosition[3][0];

	newPt[0][1] = h; // y(P0P) = h
	newPt[1][1] = h; // y(P1P) = h
	newPt[2][1] = h;
	newPt[3][1] = h;

	newPt[0][2] = calibratePosition[0][2];
	newPt[1][2] = calibratePosition[1][2];
	newPt[2][2] = calibratePosition[2][2];
	newPt[3][2] = calibratePosition[3][2];
	action_twist(lastPt, newPt, 5); // commande mouvement pattes (3eme argument % vitesse mouvement)
}

void setBeforeCalibrationHeight(int h)
{
	float newPt[4][3];
	h = constrain(h, BODY_HEIGHT_MIN, BODY_HEIGHT_MAX);
	newPt[0][0] = calibratePosition[0][0];
	newPt[1][0] = calibratePosition[1][0];
	newPt[2][0] = calibratePosition[2][0];
	newPt[3][0] = calibratePosition[3][0];

	newPt[0][1] = h;
	newPt[1][1] = h;
	newPt[2][1] = h;
	newPt[3][1] = h;

	newPt[0][2] = calibratePosition[0][2];
	newPt[1][2] = calibratePosition[1][2];
	newPt[2][2] = calibratePosition[2][2];
	newPt[3][2] = calibratePosition[3][2];
	action_twist(lastPt, newPt, 5, false);
}

void setServoToInstallationPosition()
{
	for (int i = 0; i < 16; i++)
	{
		pca.setServoAngle(i, 90);
	}
}
void setServoToBeforeCalibrationPosition()
{
	float ofs[3];
	for (int n = 0; n < 4; n++)
	{
		cooToA(calibratePosition[n][0], calibratePosition[n][1], calibratePosition[n][2], ofs);
		las[n][0] = ofs[0];
		las[n][1] = ofs[1];
		las[n][2] = ofs[2];
		updateLegxWithoutOffset(n);
	}
}

void moveOneLegToPointDirectly(uint8_t n, bool b, float(*startPt), float(*end__Pt))
{
	//Calculate the distance between two points.
	float lenghtP2P = sqrt(square(end__Pt[0] - startPt[0]) + square(end__Pt[1] - startPt[1]) + square(end__Pt[2] - startPt[2]));
	//Calculate the required motion periods, ticks.
	int stepTicks = round(lenghtP2P / 5) + 2;
	// Serial.println(String("stepTicks: ") + String(stepTicks));
	float trackPt[3];
	//Move according to the track
	for (int t = 0; t <= stepTicks; t++)
	{
		for (int j = 0; j < 3; j++)
		{
			trackPt[j] = startPt[j] + t * (end__Pt[j] - startPt[j]) / stepTicks;
		}
		cooToA(trackPt, las[n]);
		b ? updateLegx(n) : updateLegxWithoutOffset(n);
		delay(TICK_MS);
	}
	//Record end point position.
	for (int j = 0; j < 3; j++)
	{
		lastPt[n][j] = trackPt[j];
		// Serial.println(">>>>>");
		// Serial.println(String("lastPt: ") + String(lastPt[j][0]) + String(" ") + String(lastPt[j][1]) + String(" ") + String(lastPt[j][2]));
		// Serial.println("<<<<<");
	}
}

void moveAllLegToPointDirectly(bool b, float (*startPt)[3], float (*end__Pt)[3])
{
	float lenghtP2P = 0;
	for (int i = 0; i < 4; i++)
	{
		float tmpLen = sqrt(square(end__Pt[i][0] - startPt[i][0]) + square(end__Pt[i][1] - startPt[i][1]) + square(end__Pt[i][2] - startPt[i][2]));
		lenghtP2P = lenghtP2P > tmpLen ? lenghtP2P : tmpLen;
	}
	int stepTicks = round(lenghtP2P / 5) + 2;

	float trackPt[4][3];

	for (int t = 0; t <= stepTicks; t++)
	{
		for (int j = 0; j < 4; j++)
		{
			for (int i = 0; i < 3; i++)
			{
				trackPt[j][i] = startPt[j][i] + t * (end__Pt[j][i] - startPt[j][i]) / stepTicks;
			}
		}
		cooToA_All(trackPt, las);
		updateServoAngle();
		delay(TICK_MS);
	}
	for (int j = 0; j < 4; j++)
	{
		lastPt[j][0] = trackPt[j][0];
		lastPt[j][1] = trackPt[j][1];
		lastPt[j][2] = trackPt[j][2];
		// Serial.println(">>>>>");
		// Serial.println(String("lastPt: ") + String(lastPt[j][0]) + String(" ") + String(lastPt[j][1]) + String(" ") + String(lastPt[j][2]));
		// Serial.println("<<<<<");
	}
}

void updateLegx(u8 x)
{
	switch (x)
	{
	case 0:
		pca.setServoAngle(0, las[0][0] + servoOffset[0][0]);
		pca.setServoAngle(1, las[0][1] + servoOffset[0][1]);
		pca.setServoAngle(2, las[0][2] + servoOffset[0][2]);
		// Serial.println(String("las0: ") + String(las[0][0] + servoOffset[0][0]) + String(" ") + String(las[0][1] + servoOffset[0][1]) + String(" ") + String(las[0][2] + servoOffset[0][2]));
		break;
	case 1:
		pca.setServoAngle(7, las[1][0] + servoOffset[1][0]);
		pca.setServoAngle(6, las[1][1] + servoOffset[1][1]);
		pca.setServoAngle(5, las[1][2] + servoOffset[1][2]);
		// Serial.println(String("las1: ") + String(las[1][0] + servoOffset[1][0]) + String(" ") + String(las[1][1] + servoOffset[1][1]) + String(" ") + String(las[1][2] + servoOffset[1][2]));
		break;
	case 2:
		pca.setServoAngle(8, las[2][0] + servoOffset[2][0]);
		pca.setServoAngle(9, 180 - las[2][1] - servoOffset[2][1]);
		pca.setServoAngle(10, 180 - las[2][2] - servoOffset[2][2]);
		// Serial.println(String("las2: ") + String(las[2][0] + servoOffset[2][0]) + String(" ") + String(180 - las[2][1] - servoOffset[2][1]) + String(" ") + String(180 - las[2][2] - servoOffset[2][2]));
		break;
	case 3:
		pca.setServoAngle(15, las[3][0] + servoOffset[3][0]);
		pca.setServoAngle(14, 180 - las[3][1] - servoOffset[3][1]);
		pca.setServoAngle(13, 180 - las[3][2] - servoOffset[3][2]);
		// Serial.println(String("las3: ") + String(las[3][0] + servoOffset[3][0]) + String(" ") + String(180 - las[3][1] - servoOffset[3][1]) + String(" ") + String(180 - las[3][2] - servoOffset[3][2]));
		break;
	default:
		break;
	}
}

void updateLegxWithoutOffset(u8 n)
{
	switch (n)
	{
	case 0:
		pca.setServoAngle(0, las[0][0]);
		pca.setServoAngle(1, las[0][1]);
		pca.setServoAngle(2, las[0][2]);
		// Serial.println(">>>1:");
		// Serial.println(String(las[0][0]) + String(" ") + String(las[0][1]) + String(" ") + String(las[0][2]));
		// Serial.println(">>>2:");
		// Serial.println(String(las[0][0] + servoOffset[0][0]) + String(" ") + String(las[0][1] + servoOffset[0][1]) + String(" ") + String(las[0][2] + servoOffset[0][2]));
		break;
	case 1:
		pca.setServoAngle(7, las[1][0]);
		pca.setServoAngle(6, las[1][1]);
		pca.setServoAngle(5, las[1][2]);
		// Serial.println(">>>1:");
		// Serial.println(String(las[1][0]) + String(" ") + String(las[1][1]) + String(" ") + String(las[1][2]));
		// Serial.println(">>>2:");
		// Serial.println(String(las[1][0] + servoOffset[1][0]) + String(" ") + String(las[1][1] + servoOffset[1][1]) + String(" ") + String(las[1][2] + servoOffset[1][2]));
		break;
	case 2:
		pca.setServoAngle(8, las[2][0]);
		pca.setServoAngle(9, 180 - las[2][1]);
		pca.setServoAngle(10, 180 - las[2][2]);
		// Serial.println(">>>1:");
		// Serial.println(String(las[2][0]) + String(" ") + String(180 - las[2][1]) + String(" ") + String(180 - las[2][2]));
		// Serial.println(">>>2:");
		// Serial.println(String(las[2][0] + servoOffset[2][0]) + String(" ") + String(180 - las[2][1] - servoOffset[2][1]) + String(" ") + String(180 - las[2][2] - servoOffset[2][2]));
		break;
	case 3:
		pca.setServoAngle(15, las[3][0]);
		pca.setServoAngle(14, 180 - las[3][1]);
		pca.setServoAngle(13, 180 - las[3][2]);
		// Serial.println(">>>1:");
		// Serial.println(String(las[3][0]) + String(" ") + String(180 - las[3][1]) + String(" ") + String(180 - las[3][2]));
		// Serial.println(">>>2:");
		// Serial.println(String(las[3][0] + servoOffset[3][0]) + String(" ") + String(180 - las[3][1] - servoOffset[3][1]) + String(" ") + String(180 - las[3][2] - servoOffset[3][2]));
		break;
	default:
		break;
	}
}

void updateServoAngle(bool b)
{
	if (b)
	{
		updateLegx(0); // si b=true MAJ commande des servos P0
		updateLegx(1);
		updateLegx(2);
		updateLegx(3);
	}
	else
	{
		updateLegxWithoutOffset(0); // si b=true MAJ commande des servos P0 sans le offset (calibrage ?)
		updateLegxWithoutOffset(1);
		updateLegxWithoutOffset(2);
		updateLegxWithoutOffset(3);
	}
}

void cooToA_All(float (*Pt)[3], float (*ag)[3])
{
	// for (u8 i = 0; i < 4; i++) {
	//	cooToA(Pt[i], ag[i]);
	// }
	cooToA(Pt[0], ag[0]);
	cooToA(Pt[1], ag[1]);
	cooToA(Pt[2], ag[2]);
	cooToA(Pt[3], ag[3]);
}

// remet les arguments dans le bon format et appelle cooToA
void cooToA(float x, float y, float z, float *abc)
{
	float xyz[] = {x, y, z};
	cooToA(xyz, abc);
}

// cooToA semble normaliser la position des pattes pour éviter d'avoir une distance [PnM1;PnP] trop grande et donc trop de couple sur les servos
// cooToA recalcule donc les angles des servos pour aller à la nouvelle position sans avoir [PnM1;PnP] > DR
void cooToA(float *xyz, float *abc)
{
	float a, b, c, d, x, y, z, k;

	x = xyz[0];
	y = xyz[1];
	z = xyz[2];

	// Serial.println(String(x) + String(" ") + String(y) + String(" ") + String(z));

	float x_3 = 0, x_4, x_5 = 0, l23 = 0, w = 0, v = 0;

	a = PI / 2 - atan2(z, y); // a= pi/2 - arctan(z/y) --> voir S1/S2

	// position de PxM1 dans le repère de la patte
	x_3 = 0;		   // Intersection point of L1 and L2: x 
	x_4 = L1 * sin(a); // Intersection point of L1 and L2: y ATTENTION ERREUR PROJECTION ?
	x_5 = L1 * cos(a); // Intersection point of L1 and L2: z ATTENTION ERREUR PROJECTION ?

	// calcule la norme de la distance entre les points PxP et PxM1
	d = sqrt(pow((z - x_5), 2) + pow((y - x_4), 2) + pow((x - x_3), 2));
	if (d > DR) // si la distance entre PxP et PxM1 est plus grande que la référence alors on recalcule la position du pied ? 
	{
		k = DR / d; // pas le même k que précédemment (rapport entre taille jambe de référence et écart actuel entre PxP et PxM1)
		x = k * (x - x_3) + x_3; 
		y = k * (y - x_4) + x_4;
		z = k * (z - x_5) + x_5;
	}
	// l23 est une maj du d (si la position du pied a changé)
	l23 = sqrt(pow((z - x_5), 2) + pow((y - x_4), 2) + pow((x - x_3), 2));
	// l23 = constrain(l23, 0, L2 + L3);
	w = (x - x_3) / l23; // w = sin(gamma1) --> voir S1
	v = (L2 * L2 + l23 * l23 - L3 * L3) / (2 * L2 * l23); // v = cos(gamma2) --> voir S1
	b = asin(w) - acos(v); // angle rotation M1 (axe z)
	c = PI - acos((pow(L2, 2) + pow(L3, 2) - pow(l23, 2)) / (2 * L3 * L2)); // c = Pi - gamma3 --> c angle rotation M2 (axe z)
	a = a / PI * 180; // passage en degrés des angles
	b = b / PI * 180;
	c = c / PI * 180;
	b = map(b, -90, 90, 180, 0); // ????
	abc[0] = a;
	abc[1] = b;
	abc[2] = c;
	// printf("abc: %f, %f, %f, %f, %f, %f, %f \n", a, b, c, x, y, z, l23);
	//   Serial.println(String(a) + String(" ") + String(b) + String(" ") + String(c));
}
extern PreferencesPro prefs;
void getServoOffsetFromStorage()
{

	// If there is a problem with the read length, it indicates the board is new and has never been calibrated. Then set all offsets to 0.0.
	// Serial.printf("len : %d, offset size: %d \n\n", prefs.getBytesLength(KEY_SERVO_OFFSET), sizeof(servoOffset));
	if (prefs.getBytesLength(KEY_SERVO_OFFSET) != sizeof(servoOffset))
	{
		prefs.put(KEY_SERVO_OFFSET, servoOffset);
		// Serial.println("new borad! init the servo offset.");
	}
	else
	{
		prefs.get(KEY_SERVO_OFFSET, servoOffset);
		// for (int i = 0; i < 4; i++)
		// {
		// 	for (int j = 0; j < 3; j++)
		// 	{
		// 		Serial.print(" ");
		// 		Serial.print(servoOffset[i][j]);
		// 	}
		// 	Serial.println(" ");
		// }
	}
}
