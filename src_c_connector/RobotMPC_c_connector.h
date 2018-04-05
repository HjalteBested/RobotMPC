#ifndef ROBOTMPC_C_CONNECTOR_H
#define ROBOTMPC_C_CONNECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

// Construct and Deconstruct
void RPFMPC_create();
void RPFMPC_destroy();

// Wrapper Functions
void RPFMPC_init(float T, int clkDiv, float v_des);
void RPFMPC_initRobot(float w, float a);
void RPFMPC_setConstraints(float vw_min, float vw_max, float omega_min, float omega_max, float v_min, float v_max, float acc_min, float acc_max);
void RPFMPC_setAccConstraints(float acc_min, float acc_max);
void RPFMPC_initMPC(int N);
void RPFMPC_design(float Qth, float Qdu, float Qu);

// Waypoints
void RPFMPC_clearWaypoints();
void RPFMPC_addWaypoint(float x, float y);
void RPFMPC_insertWaypoint(float i, float x, float y);
void RPFMPC_insertWaypointRel(float i, float x, float y);

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

// Getter / Setter Functions
void RPFMPC_setKs(float ks);
float RPFMPC_getKs();
void RPFMPC_setKv(float kv);
float RPFMPC_getKv();
void RPFMPC_setKa(float ka);
float RPFMPC_getKa();

float RPFMPC_get_s();
float RPFMPC_get_d();
float RPFMPC_get_th_err();
float RPFMPC_get_v_des();

void RPFMPC_set_a_des(float a_des);
float RPFMPC_get_a_des();

int RPFMPC_isLastLine(int i);
int RPFMPC_allDone();
int RPFMPC_designMPCDone();
int RPFMPC_initMPCDone();

#ifdef __cplusplus
}
#endif


#endif

