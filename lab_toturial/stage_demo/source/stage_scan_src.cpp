/******************************
*
* Aerotech Stage Control Demo Script
* 	stage_scan_src.cpp
*
* Discription:
*	This script controls the Aerotech Stage do a peroidic scan with predefined stepsize and total distance
*	
******************************/

// including libraries
#include <stdint.h>			//standard C99 include file for integer types
#include <iostream>			//C++ console output
#include <string>			//C++ string class
#include <vector>			//C++ vector (array) class
#include <Windows.h>		//standard windows OS file
#include <sstream>			//string streams
#include <stim/parser/arguments.h>
#include <stim/ui/progressbar.h>
#include "A3200.h"			//Aerotech libraries

// define global variables
A3200Handle hstage = NULL;	//handle for the stage controller

// stage clean up function
void aerotechCleanup(A3200Handle h) {
	A3200ProgramStop(h, TASKID_01);
	A3200MotionDisable(h, TASKID_01, AXISMASK_00);
	A3200Disconnect(h);
}

int main(int argc, char* argv[]) {

			// argument list for the command line tool
			stim::arglist args;
			args.add("help", "prints usage information");
			args.add("stepSize", "step size for the stage", "500", "positive value describing stage motion in microns");
			args.add("totalDistance", "total distance to move forward or backward", "20000", "integer (between 1 and 50000)");
			args.parse(argc, argv);

			// print help menu
			if(args["help"]){
						std::cout<<args.str();
						exit(1);
			}

			// load the parameters
			int stepSize = args["stepSize"].as_int();		
			int totalDistance = args["totalDistance"].as_int();

			// create string commands for stage motion control
			std::stringstream ss;												//create an empty string stream
			float dz = (float)args["stepSize"].as_float() * 0.001f;				//get the z step size in micrometers and convert to millimeters
			ss << "LINEAR Z"<<dz;												//append to the command string
			std::string cmd_step = ss.str();									//store the move command in a string

			std::stringstream bb;
			float bck = (float)args["totalDistance"].as_float() * 0.001f;
			bb << "LINEAR Z-" << bck;											
			std::string cmd_return = bb.str();									

			// calculate total positions
			int positions = totalDistance / stepSize;
			DOUBLE curr_stage;
			
			// perform an imaging pass across the sample
			// connect to the A3200 Aerotech stage controller
			std::cout <<"Connecting to A3200...";
			A3200Connect(&hstage);
			std::cout << "done" << std::endl;

			// enable the axes
			std::cout <<"Enabling axes...";			    							
			A3200MotionEnable(hstage, TASKID_01, AXISMASK_00);
			std::cout << "done" << std::endl;
			
			// Home the axes
			std::cout <<"Home the axes...";			    							
			A3200CommandExecute(hstage, TASKID_01, "HOME Z", &curr_stage);
			std::cout << "done" << std::endl;

			// perform the task
			for (int i = 0; i < positions; i++){

						A3200CommandExecute(hstage, TASKID_01, cmd_step.c_str(), &curr_stage);			// move the stage
						A3200CommandExecute(hstage, TASKID_01, "MOVEDELAY Z, 100", &curr_stage);		// wait

						// display a progress bar
						rtsProgressBar((float)(i + 1) / (float)positions * 100);
			}
			
			std::cout << "\nReset the stage...";
			A3200CommandExecute(hstage, TASKID_01, cmd_return.c_str(), &curr_stage);				// move stage back to origin
			A3200CommandExecute(hstage, TASKID_01, "MOVEDELAY Z, 2000", &curr_stage);				// wait again

			std::cout << "done" << std::endl;
			
			// release the resources
			aerotechCleanup(hstage);
}