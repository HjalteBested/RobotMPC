#ifndef ROBOTMPC_C_CONNECTOR_H
#define ROBOTMPC_C_CONNECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

// Construct and Deconstruct
void RPFMPC_create();
void RPFMPC_createWithParams(float T, int clkDiv, float w, float a, float v_des, float a_des, int N, float Qth, float Qdu, float Qu);

void RPFMPC_destroy();

// Wrapper Functions
void RPFMPC_init(float T, int clkDiv, float v_des);
void RPFMPC_initRobot(float w, float a);
void RPFMPC_setConstraints(float vw_min, float vw_max, float omega_min, float omega_max, float v_min, float v_max, float acc_min, float acc_max);
void RPFMPC_setAccConstraints(float acc_min, float acc_max);
void RPFMPC_initMPC(int N);
void RPFMPC_design(float Qth, float Qdu, float Qu);
void RPFMPC_designAll(float Qth, float Qdu, float Qu, float a, int clkDiv, int N, float v_des);

void RPFMPC_setVelAcc(float v, float a);

void RPFMPC_clear();
// Waypoints
void RPFMPC_clearWaypoints();
void RPFMPC_clearWaypointsKeepCurrent();
void RPFMPC_addWaypoint(float x, float y);
void RPFMPC_insertWaypoint(float i, float x, float y);
void RPFMPC_insertWaypointRel(float i, float x, float y);
void RPFMPC_makeLineDefs();

void RPFMPC_setRk(float d_ref, float phi_ref);	//!< Set the reference Rk to a constant value over the whole horizon.
void RPFMPC_setRkToCurrentLine();

// Compute the control
void RPFMPC_compute(float x, float y, float theta);
void RPFMPC_get_ur(float*ur_out);
void RPFMPC_get_uw(float*uw_out);

// Run Simulation
void RPFMPC_runSimulation(float x, float y, float theta, int nstp);

void RPFMPC_printRobot();
void RPFMPC_printSys();
void RPFMPC_printCondenseStateSpace();
void RPFMPC_printControllerGains();
void RPFMPC_printWaypoints();
void RPFMPC_printConstraints();	//!< Print the constraints
void RPFMPC_printParams();		//!< Print the constraints


// Getter / Setter Functions
void RPFMPC_setClkDiv(float clkDiv);
float RPFMPC_getClkDiv();

void RPFMPC_setKs(float ks);
float RPFMPC_getKs();
void RPFMPC_setKa(float ka);
float RPFMPC_getKa();

float RPFMPC_get_s();
float RPFMPC_get_d();
float RPFMPC_get_th_err();
float RPFMPC_get_v_des();
int RPFMPC_get_N();

void RPFMPC_set_a_des(float a_des);
float RPFMPC_get_a_des();

int RPFMPC_isLastLine(int i);
int RPFMPC_allDone();
int RPFMPC_designMPCDone();
int RPFMPC_initMPCDone();

void RPFMPC_setCurrentLine(int currentLine);
int RPFMPC_getCurrentLine();



#ifdef __cplusplus
}
#endif


#endif

