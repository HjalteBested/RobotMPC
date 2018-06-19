#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include "RobotMPC.h"
using namespace std;
using namespace Eigen;
using namespace boost;

int main(){  
  RobotPathFollowMPC pf = RobotPathFollowMPC();
  // mpc.printRobot();
  pf.clkDiv = 4;

  // "Test"
  
  // Init MPC
  float T = 0.01 * pf.clkDiv;
  float v_des = 0.3;
  pf.a_des = 0.3;
  float w = 0.26;
  float a = 0.1;
  
  pf.initRobot(w,a);
  pf.init(T, v_des);
  pf.currentLine=0;

  // Initialize Constraints
  float vw_max =  0.6;
  float vw_min = -vw_max;
  float omega_max = 2*PI/8;
  float omega_min = -omega_max;
  float v_max =  1;
  float v_min = -v_max;
  float acc_max =  0.5;
  float acc_min = -0.1;
  pf.setConstraints(vw_min,vw_max,omega_min,omega_max,v_min,v_max,acc_min,acc_max);

  // Design MPC - Compute controller gains for the horizon
  int N = 100;
  float Qth = 0.05;
  float Qdu = 0.001;
  float Qu =  0.001;
  pf.initMPC(N);
  pf.design(Qth, Qdu, Qu);
  pf.ks = 0*v_des/omega_max;
  pf.ka = 0;

  pf.useNLScaling = 1;

  // Print That Shit !
  pf.printRobot();
  pf.sys.printSys();
  pf.printConstraints();
  pf.printParams();

  //pf.mpc.printCondenseStateSpace();
  //pf.mpc.printControllerGains();
  // cout << "Matrix P:\n" << pf.P << endl;
  // cout << "Vector q:\n" << pf.q << endl;
  int track = 1;

  switch(track){
    case 0:{
      pf.addWaypoint(0,    0);
      pf.addWaypoint(4,    0);
    }
    break;    
    case 1:{
      int n=2;
      pf.addWaypoint(0,    0);
      pf.addWaypoint(n,    0);
      pf.addWaypoint(n,   -n);
      pf.addWaypoint(0,   -n);
      pf.addWaypoint(0,    0);
    }
    break;
    case 2:{
      pf.addWaypoint(0,    0);
      pf.addWaypoint(1.5,  0);
      pf.addWaypoint(1.5, -0.5);
      pf.addWaypoint(3,   -0.5);
    }
    break;
    case 3:{
      pf.addWaypoint(0,  0 );
      pf.addWaypoint(1.5,  0);
      pf.addWaypoint(1.5, -0.2);
      pf.addWaypoint(3,   -0.2);
    }
    break;
    case 4:{
      float Nstep = 20;
      for(int i=0; i<=Nstep; i++){
        float amp = 0.5;
        float x = i/Nstep;
        float y = amp*sin(2 * M_PI * x);
        pf.addWaypoint(3*x,y);
      }
    }
    break;
    case 5:{
      float len = 2.0;
      float angleDeg = 150;
      float angleRad = angleDeg*M_PI/180.0f;
      pf.addWaypoint(0,    0);
      pf.addWaypoint(len,    0);
      float x = len+len*cos(angleRad);
      float y = len*sin(angleRad);
      pf.addWaypoint(x,y);
    }
    break;
    case 6:{
      pf.addWaypoint(0,    0);
      pf.addWaypoint(2,    0);
      pf.addWaypoint(2,    2);
    }
  }
  pf.makeLineDefs();
  // Define waypoints 
  // n-by-n square

  // pf.insertWaypoint(2, 3.5,0.5);
  cout << "Line Definitions:\n" << pf.lineDefs << endl;
  
  // cout << "Static Kalman Gain:\n" << pf.Kfx << endl;

  //Vector2f ur = pf.compute(-0.3,-0.6,-0.2);
  //cout << "ur:\n" << ur << endl;


  bool runSim = true;
  if(runSim){    
    pf.runSimulation(1,0.5,0,10000);
    // cout << "Here is the simdata:\n\ttime\t      x\t\t  y\t   theta\t  v\t\t  omega\t     vl\t\t vr\t      s\t\t  d\t th_err\n" << mpc.simData << endl;
    cout << "Simulation Complete!" << endl;

    ofstream file("simData/simData.txt");  
    if (file.is_open()){
      file << pf.simData << '\n';
      file.close();
    }

    ofstream file2("simData/simWaypoints.txt");  
    if (file2.is_open()){
      file2 << pf.lineDefs << '\n';
      file2.close();
    }

    ofstream file3("simData/simConstraints.txt");  
    if (file3.is_open()){
      file3 << "T\tv_des\ta_des\tw\ta\tN\tQth\tQdu\tQu\tks\tvw_min\tvw_max\tomega_min\tomega_max\tv_min\tv_max\tacc_min\tacc_max\ttrack\t" << endl;
      file3 << pf.T << '\t' << pf.v_des << '\t' << pf.a_des << '\t' << pf.w << '\t' << pf.a << '\t';
      file3 << pf.mpc.N << '\t' << Qth << '\t' << Qdu << '\t' << Qu  << '\t' << pf.ks << '\t';
      file3 << -pf.q(2) << '\t' << pf.q(0) << '\t' << -pf.q(5) << '\t' << pf.q(4) << '\t' << -pf.q(7) << '\t' << pf.q(6) << '\t' << pf.acc_min << '\t' << pf.acc_max << '\t';
      file3 << track << '\t';
      file3 << endl;
      file3.close();
    }
  }
  
  
  // cout << "Rk:\n" << mpc.Rk << endl;
  // cout << "Uk:\n" << mpc.Uk << endl;
  
}

