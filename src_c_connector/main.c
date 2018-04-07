#include <stdio.h>
#include "RobotMPC_c_connector.h"

#define PI 3.141592654

int main() {
	RPFMPC_create();

	// Init MPC
	float T = 0.01;
	int clkDiv = 4;
	float v_des = 0.4;
	float w = 0.26;
	float a = 0.2;
	printf("init = %d, v_des = %f\n",RPFMPC_initMPCDone(),RPFMPC_get_v_des());
	RPFMPC_init(T, clkDiv, v_des);
	RPFMPC_initRobot(w, a);
	RPFMPC_set_a_des(0.2);	

	// Design MPC - Compute controller gains for the horizon
	int N = 100;
	float Qth = 0.02;
	float Qdu = 0.02;
	float Qu = 0.001;
	RPFMPC_initMPC(N);
	RPFMPC_design(Qth, Qdu, Qu);

	printf("init = %d, v_des = %f\n",RPFMPC_initMPCDone(),RPFMPC_get_v_des());

	RPFMPC_printRobot();
	RPFMPC_printSys();
	//RPFMPC_printCondenseStateSpace();
  	//RPFMPC_printControllerGains();

	RPFMPC_addWaypoint(0,		0);
	RPFMPC_addWaypoint(2.8,		0);
	RPFMPC_addWaypoint(2.8,  -0.2);
	RPFMPC_addWaypoint(4,	 -0.2);
	RPFMPC_addWaypoint(4,    -1.5);
	RPFMPC_addWaypoint(2,    -1.5);
	RPFMPC_addWaypoint(2,    -3);
	RPFMPC_addWaypoint(0,    -3);
	RPFMPC_addWaypoint(0,     0);

	// Compute the control signal
	RPFMPC_compute(-0.3,-0.6,0);
	float ur[2];
	RPFMPC_get_ur(ur);
	float uw[2];
	RPFMPC_get_uw(uw);

	printf("The control signals:\n");
	printf("vk = %f, omega = %f\n",ur[0],ur[1]);
	printf("vl = %f, vr = %f\n",uw[0],uw[1]);

    RPFMPC_destroy();
    return 0;
}

// Compile with 
// gcc main.c -L. -laaa_c_connector -o c_aaa
