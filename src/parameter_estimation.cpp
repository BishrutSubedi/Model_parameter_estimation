#include "R_COM.h"
#include <string>
#include <iostream>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <ctime>
#include <ratio>
#include <chrono>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <fstream>
#include <sys/time.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/NavSatFix.h"
#include <ros/callback_queue.h>

// NOTE: RENAME local_pwm/local_rssi to remote_pwm/remote_rssi to work with remote system

// from local_initial_Scan_main.cpp

using namespace std;
using namespace std::chrono;
double Setpoint=0, Input, Output;
//using namespace exploringBB;
// Set parameters for the LQG cntroller
double lastW = 0;     // NOT USED
double lastE = 0;     // NOT USED
//double Kcp = 4.4721;
//double Kcf = 3.1535;

double kp=1.1;  // THIS VALUE IS HERE TO ROTATE THE MOTOR ONLY, NOT FROM ESTIMATION
double ki=1.1;  // FOR ROTATION, KI GAIN, NOTHING TO DO WITH ESTIMATION


const int sampleTime = 60;
double SampleTimeInSec = ((double)sampleTime)/1000;
steady_clock::time_point t2 = steady_clock::now();
steady_clock::time_point t1 = steady_clock::now();

// initial heading*******************************
double offsetTol=1.5;

// THESE ARE TARGET VALUE OF AZIMUTH TO GET THE ANTENNA TO USING DC MOTOR ROTATION
// THEY ARE USED LATER AS HARDCORED FOR ESTIMATION USING MATLAB

//int Azmuth[] ={ 165,80,155,80,160,95,190,120,200,145,225,165,255,185,255,175,235,140,215,135,195,100,175,85,145,60,140};//exp1
//int Azmuth[] ={ 75,150,90,170,220,165,245,290,200,260,170,100,165,80,155,65,135,45,165,100,195,130,215,145,235,140,220};//exp2
//int Azmuth[] ={150,90,150,220,165,245,290,230usleep(200000)0,180,135,220,135,190,115,165,115,200,155,100,165,90,155,105,185,140,210,150}; //exp4
//int Azmuth[] ={ 145,60,155,230,170,225,150,80,140,65,135,45,105,185,115,175,95,30,125,55,145,200,135,205,125,180,90};//exp5
//int Azmuth[] ={ 125,185,130,215,130,215,135,75,145,95,160,115,185,265,200,275,185,245,180,250,170};//exp6
//int Azmuth[] ={ 135,85,140,90,170,125,195,155,210,165,205,150,200,135,180,125,170,110,165,100,155,85,140,80,135,70,115};//exp7

//Experiment for only one
int Azmuth[] ={120}; //exp4

int Azmuth_index=0;  // IS A GLOBAL VARIABLE
//***********************************************

void setting();     

void PI_Motor() ;
double Get_headingDiff(double Input, double Setpoint);
double PI_Controller(double error);
double integral=0;
readcompass R_compass;  //DECLARING A CLASS I.E. R_compass is object of type readcompass
int Azmuth_num;         // A GLOBAL VARIABLE
//*********************************************

// CREATING STRUCTURE WITH 3 FIELDS,  time, pwm and value heading to write to data files
struct DATA{          

	double during_time;     //TIME STAMP 
  int value_pwm;
  float value_heading;

};

DATA data[10000];   // CREATE ARRAY OF STRUCTURES, I.E. 10000

int data_index=0;   // A GLOBAL VARIABLE

ros::Publisher ros_serial;
ros::Publisher pwm_pub;
ros::Subscriber local_rssi;
std_msgs::Int16   pwm_msg;


int main(int argc, char **argv)
{
	ros::init(argc,argv,"parameter_estimation");
	ros::NodeHandle n;

	pwm_pub = n.advertise<std_msgs::Int16>("local_pwm",1);

	setting();

  Azmuth_num=sizeof(Azmuth)/sizeof(Azmuth[0]); 
	cout<<Azmuth_num<<endl;

  //RECREATE THIS FUNCTION AND DO YOUR OWN WAY
  std::fstream fs;
  fs.open ("test.txt", std::fstream::in | std::fstream::out | std::fstream::app);
	
  if(!fs) {                               // EXIT PROGRAM IF FILE NOT THERE, ADD WARNING ON EXIT
          cout << "1. Cannot open file. \n";
          return 1;
        }
	
	fs<<"In exp4The Azmuth[] ={ 120,55,130,80,155,85,165,100,180,135,220,135,190,115,165,115,200,155,100,165,90,155,105,185,140,210,150} and kp=1.1, ki=1.5;"<<endl;

	sleep(2);

  cout<< "time_count" << "," << "Output_pwm" <<" ,"<< "raw_angle" <<" ," << "target_angle" << endl; //printing to screen as well

	while (Azmuth_index < Azmuth_num)     //NOTE: THIS LOOP ONLY EXITS AFTER ALL ANGLE TARGET REACHED AND DO NOT WRITE ANYTHING AS WRITING IS AFTER WHILE LOOP FINISHES.
	{
		 PI_Motor();  //Azmuth index is incremented in PI_Motor() // fills global variable

//  Can comment this whole section
    if(!fs) {
            cout << "2. Cannot open file.\n"; // here it throws error
            return 1;
          }

  // WRITE TO FILE data.txt 1
  // COMMENT THIS ALL OUT AND TEST FROM TERMINAL FILE ONLY
    // for(int i=1;i<data_index;i++)
    // {
    //   fs<<data[i].during_time<<" "<<data[i].value_pwm<<" "<<data[i].value_heading<<endl;
    //   //this_thread::sleep_for(std::chrono::milliseconds(100)); //sleep 100ms in this thread

    // }

	} //END WHILE LOOP

		pwm_msg.data = 0;    //STOP THE MOTOR
		pwm_pub.publish(pwm_msg);

		fs.close();   

		return 0;
}


void setting()
{
  //Only have to set compass. pwm and gpio as different hardware
	R_compass.setting_compass();//setting compass
	usleep(200000);
}


void PI_Motor()   //motor control
  {
	 t2 = steady_clock::now();
	duration<double> time_span = duration_cast<duration<double>>(t2 - t1);    // For time stamp and freq

    if ( time_span.count()>SampleTimeInSec)
    {
      float headingRaw =R_compass.c_heading();                              //Get current compass
      //cout<<"heading: "<< headingRaw<<endl;
      float targetHeading = Azmuth[Azmuth_index];                           //Get destination direction
      Input = headingRaw;
      Setpoint = targetHeading;
      float headingDiff = Get_headingDiff(Input, Setpoint);

      if (abs(headingDiff) < offsetTol){    //offsetTot = 1.5
        pwm_msg.data = 0;                   //STOP    
	      pwm_pub.publish(pwm_msg);

    	  if(Azmuth_index < Azmuth_num)
    	   {
           Azmuth_index++;
    	   }
        }

      else
       {
        Output = PI_Controller(headingDiff);

        cout<< steady_clock::now() << "," << "," << Output <<" ,"<< Input <<" ," << targetHeading << endl; //printing to screen as well

        if (Output > 0)
        { 
          //Output = (int) Output
        	pwm_msg.data = abs(Output);    //MOTOR ROTATION YAHA MATRA CHANGE GAR H/W
		      pwm_pub.publish(pwm_msg);

        }
        else
        {
          //Output = (int) Output
        	pwm_msg.data = -(abs(Output));    //like double negative is +ve, I don't think this is necessary
		      pwm_pub.publish(pwm_msg);

          }

        }
        t1 = steady_clock::now();
        data[data_index].during_time=time_span.count();
        data[data_index].value_pwm=Output;
        data[data_index].value_heading=Input;       // THIS IS THE COMPASS READING, BETTER TO CALL R_compass.c_heading() for raw
        data_index++;

    }
  }


double Get_headingDiff(double Input, double Setpoint){
    double error1 = Setpoint - Input;
    double error2 = Setpoint + 360 - Input;
    double error3 = Setpoint - Input - 360;
    double error;
    if (abs(error1)  <= abs(error2) && abs(error1) <= abs(error3))
    {
      error = error1;
      }
    else if (abs(error2) <= abs(error1) && abs(error2) <= abs(error3))
    {
      error = error2;
      }
    else
    {
      error = error3;
    }
    return error;
  }


double PI_Controller(double error)
  {

	integral += error * SampleTimeInSec;
	Output = error * kp + integral * ki;
    if (Output > 254){
      Output = 254;
      }
    else if (Output < -254){
        Output = -254;
      }
    return (Output / 254)*100;
  }


