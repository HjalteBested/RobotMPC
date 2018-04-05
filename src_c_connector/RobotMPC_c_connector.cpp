// #include <cstdlib>
#include "RobotMPC_c_connector.h"
#include "RobotMPC.h"

#ifdef __cplusplus
extern "C" {
#endif

// Inside this "extern C" block, I can define C functions that are able to call C++ code
static RobotPathFollowMPC *rpfmpc = NULL;
void lazyRPFMPC() {
    if (rpfmpc == NULL) {
        rpfmpc = new RobotPathFollowMPC();
    }
}

void RPFMPC_create(){
    if (rpfmpc == NULL){
        rpfmpc = new RobotPathFollowMPC();
    }
}

void RPFMPC_destroy(){
	if(rpfmpc != NULL){ 
		delete rpfmpc;
		rpfmpc = NULL;
	}
}

void RPFMPC_init(float T, int clkDiv, float v_des){
	lazyRPFMPC();
	rpfmpc->clkDiv = clkDiv;
	rpfmpc->init(T*clkDiv,v_des);
}

void RPFMPC_initRobot(float w, float a){
	rpfmpc->initRobot(w,a);
}

void RPFMPC_setConstraints(float vw_min, float vw_max, float omega_min, float omega_max, float v_min, float v_max, float acc_min, float acc_max){
	rpfmpc->setConstraints(vw_min, vw_max, omega_min, omega_max, v_min, v_max, acc_min, acc_max);
}
void RPFMPC_setAccConstraints(float acc_min, float acc_max){
	rpfmpc->setAccConstraints(acc_min, acc_max);
}

void RPFMPC_initMPC(int N){
	rpfmpc->initMPC(N);
};

void RPFMPC_design(float Qth, float Qdu, float Qu){
	rpfmpc->design(Qth,Qdu,Qu);
}

void RPFMPC_clearWaypoints(){
	rpfmpc->clearWaypoints();
}

void RPFMPC_addWaypoint(float x, float y){
	rpfmpc->addWaypoint(x,y);
}

void RPFMPC_insertWaypoint(float i, float x, float y){
	rpfmpc->insertWaypoint(i,x,y);
}

void RPFMPC_insertWaypointRel(float i, float x, float y){
	rpfmpc->insertWaypointRel(i,x,y);
}

void RPFMPC_compute(float x, float y, float theta){
	rpfmpc->compute(x,y,theta);
}

void RPFMPC_get_ur(float*ur_out){
	ur_out[0] = rpfmpc->ur(0);
	ur_out[1] = rpfmpc->ur(1);
}

void RPFMPC_get_uw(float*uw_out){
	uw_out[0] = rpfmpc->uw(0);
	uw_out[1] = rpfmpc->uw(1);
}

void RPFMPC_setKs(float ks){
	rpfmpc->ks = ks;
}
float RPFMPC_getKs(){
	return rpfmpc->ks;
}
void RPFMPC_setKv(float kv){
	rpfmpc->kv = kv;
}
float RPFMPC_getKv(){
	return rpfmpc->kv;
}
void RPFMPC_setKa(float ka){
	rpfmpc->ka = ka;
}
float RPFMPC_getKa(){
	return rpfmpc->ka;
}

float RPFMPC_get_s(){
	return rpfmpc->get_s();
}
float RPFMPC_get_d(){
	return rpfmpc->get_d();
};

float RPFMPC_get_th_err(){
	return rpfmpc->get_th_err();
};

float RPFMPC_get_v_des(){
	return rpfmpc->v_des;
}

void RPFMPC_set_a_des(float a_des){
	rpfmpc->a_des = a_des;
}

float RPFMPC_get_a_des(){
	return rpfmpc->a_des;
}

int RPFMPC_isLastLine(int i){
	return rpfmpc->isLastLine(i);
};
int RPFMPC_allDone(){
	return rpfmpc->allDone;
}

int RPFMPC_initMPCDone(){
	return rpfmpc->mpc.initMPCDone;
}

int RPFMPC_designMPCDone(){
	return rpfmpc->mpc.designMPCDone;
}

void RPFMPC_runSimulation(float x, float y, float theta, int nstp){
	rpfmpc->runSimulation(x, y, theta, nstp);
	cout << "Simulation Complete!" << endl;
	cout << "Here is the simdata:\n" << rpfmpc->simData << endl;
};

// Print Function Wrappers
void RPFMPC_printRobot(){
    rpfmpc->printRobot();
}
void RPFMPC_printSys(){
	rpfmpc->sys.printSys();
}	
void RPFMPC_printCondenseStateSpace(){
	rpfmpc->mpc.printCondenseStateSpace();
}
void RPFMPC_printControllerGains(){
	rpfmpc->mpc.printControllerGains();
}
void RPFMPC_printWaypoints(){
	rpfmpc->printWaypoints();
}


#ifdef __cplusplus
}
#endif


// Compile With
// gcc main.c -L. -lRobotMPC_c_connector -o c_robotMPC
// g++ -fpic -shared RobotMPC.cpp -o libRobotMPC.so -I/usr/local/Cellar/eigen/3.3.4/include/eigen3
// g++ -fpic -shared RobotMPC_c_connector.cpp -L. -lRobotMPC -o libRobotMPC_c_connector.so -I/usr/local/Cellar/eigen/3.3.4/include/eigen3