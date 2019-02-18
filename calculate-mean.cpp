#include <stdint.h>			//standard C99 include file for integer types
#include <iostream>			//C++ console output
#include <string>			//C++ string class
#include <vector>			//C++ vector (array) class
#include <Windows.h>		//standard windows OS file
#include <sstream>			//string streams
#include <ShlObj.h>			//??????	
#include <Shlwapi.h>		//??????
#include <fstream>			//file stream
#include "ips.h"
#include "vmemory.h"
#include <stdio.h>			//standard input and output
#include <tchar.h>			//Windows Unicode stuff
#include "A3200.h"			//Aerotech libraries
#include <MIRcatSDK.h>		//MIRcat libraries
#include <stim/parser/arguments.h>
#include <sstream>
#include <stim/ui/progressbar.h>
#include <stim/math/matrix.h>
#include <stim/parser/filename.h>
#include <direct.h>
#include <errno.h>
#include <cstdio>
#include <ctime>
#include <algorithm>
#include <iterator>
#include <functional>
#include <chrono>
#include <numeric>

const int fpa_size = 128;
int fpg = 400;				//number of frames per grab

const char * DEFAULT_CONFIG = "<ips_config>"
	"   <winir_inipath>$(IPS_SDK_DATA_DIR)\\Config\\winir.ini</winir_inipath>"
	"   <ccf_path>$(IPS_SDK_DATA_DIR)\\Config\\sbf161_px4full_dv_1tap.ccf</ccf_path>"
	""
	"   <fg_window>"
	"     <xoffset>0</xoffset>"
	"     <yoffset>0</yoffset>"
	"     <xsize>128</xsize>"
	"     <ysize>128</ysize>"
	"   </fg_window>"
	" </ips_config>";

void ipsHandleError(int32_t err, const char* file, int line) {
	if(!IPS_SUCCEEDED(err)){
		std::cout << "IPS ERROR (# " << err << ") in "<<file<< " line "<<line<<std::endl;
		exit(err);
	}
}
#define CHECK_IPS( err ) (ipsHandleError(err, __FILE__, __LINE__))

std::string ipsPrintCaptureSource(uint32_t src) {
	char capture_source[IPS_MAX_CAPTURE_SOURCE_BYTES];						//allocate space for strings
	char capture_source_descr[IPS_MAX_CAPTURE_SOURCE_BYTES];

	CHECK_IPS(IPS_GetCaptureSource(
		IPS_DEVICE_TYPE_CAMERALINK,
		src,
		capture_source, sizeof(capture_source),
		capture_source_descr, sizeof(capture_source_descr)));
	std::cout << "capture source ID: " << src << std::endl;
	std::cout << "name: " << capture_source << std::endl;
	std::cout << "description: " << capture_source_descr << std::endl;
	return std::string(capture_source);
}


//get the string indicating the first available capture source (source 0)
uint32_t GetFirstAVailableCaptureSource() {
	uint32_t nc = 0;																//store the number of sources
	CHECK_IPS(IPS_GetCaptureSourceCount(IPS_DEVICE_TYPE_CAMERALINK, &nc));			//get the number of sources

	if (nc == 0) {																//if there are no capture sources
		std::cout << "No capture sources were found." << std::endl;;			//display an error
		exit(1);																//exit
	}
	return 0;																	//return the number of capture sources
}


//initialize a decommute table
void Initialize_SBF161_Decommute_Table( int grab_num_cols,
									   int grab_num_rows,
									   int display_num_cols,
									   int display_num_rows,
									   std::vector<int> & decommute_table){
										   unsigned num_pixels = grab_num_cols * grab_num_rows;
										   for (unsigned i = 0; i < num_pixels; i++){
											   unsigned display_row = (i % 4) + 4 * (i / grab_num_cols);
											   unsigned display_col = (i / 4) % display_num_cols;
											   unsigned display_pixel = display_row * display_num_cols + display_col;
											   if (display_pixel < decommute_table.size()){
												   decommute_table[display_pixel] = i;
											   }
										   }
}

uint32_t CalculateMean(HANDLE_IPS_ACQ handle_ips, int fpg)
{
	uint32_t frame_width = 128;
	uint32_t frame_height = 128;
	uint32_t mean = 0;
	uint64_t sum = 0;
	int bytes_per_pixel = 2;
	int frame_data_size = frame_width * frame_height * bytes_per_pixel;

	std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
	// Configure the frame acquisition window size
	CHECK_IPS(IPS_SetFrameWindow( handle_ips, 
		0,
		0,
		frame_width,
		frame_height));

	// Start capturing a block of fpg frames
	tsi::ips::VMemory<uint64_t> buffer(frame_data_size*fpg);
	std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

	CHECK_IPS(IPS_StartGrabbing( handle_ips,         
		fpg,            // Capture Fra_Number frames then stop capturing
		buffer.data(), // User allocated buffer
		buffer.size(), // size of user allocated buffer
		false));        // No wrap


	std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
	// Wait for all frames to be acquired
	uint64_t frame_number;
	uint8_t * p_frame = NULL;
	CHECK_IPS(IPS_WaitFrame(handle_ips,
		fpg,                          // Wait until the number of frame has been captured
		IPS_TIMEOUT_WAIT_FOREVER,    // Don't time out, wait for ever
		false,                       // Pause after WaitFrame returns
		&p_frame,                    // Return a pointer to the frame
		&frame_number));              // Return the frame number of the returned frame
	std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();

	// *********** Decommute ********************
	//
	// The data coming from the SBF 161 must be de-commuted before it can be displayed.   
	// The first four pixels go in the first columns of the first four rows, the next 4 pixels 
	// in the second column and so on to the last column.  Then it continues to rows 5-8 and so on.

	// Create a decommute table for recording the pixels for display as an image

	std::vector<int> decommute_table(frame_width*frame_height);
	Initialize_SBF161_Decommute_Table(frame_width * 4,
		frame_height/4,
		frame_width,
		frame_height,
		decommute_table);
	std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
	// Decommute the images.
	std::vector<uint16_t> display_image(frame_width * frame_height);
	std::vector<uint32_t> display_image_all(frame_width * frame_height, 0);
	uint16_t * p_display_image;
	//std::string module_dir = GetModuleDirectory();
	//std::string image_dir = module_dir + "\Frames1800\\";

	for (int frame_index = 0; frame_index < fpg; frame_index++)
	{
		// Get a pointer to the image
		uint16_t * p_image = (uint16_t* ) (buffer.data() + frame_index*frame_data_size);

		// Decommute the image

		for (unsigned i = 0; i < display_image.size(); i++)
		{
			display_image[i] = p_image[decommute_table[i]];

			// Invert the image pixel 
			// uncomment the following line to invert the image
			display_image[i] = display_image[i] ^ 0x3FFF;
		}

		std::transform ( display_image_all.begin(), display_image_all.end(), display_image.begin(), display_image_all.begin(), std::plus<uint32_t>());
			//duration_decommute = ( std::clock() - start_decommute ) / (double) CLOCKS_PER_SEC;
			//std::cout << "\nDuration for decommute 1 frame: " << duration_decommute << " seconds"<< std::endl ;

	}
	sum = std::accumulate( display_image_all.begin(), display_image_all.end(), sum );
	mean = (sum / display_image_all.size()) / fpg;

	// Stop acquiring frames
	CHECK_IPS(IPS_StopGrabbing(handle_ips));
	return mean;
}

void comp_mean(int wn, int QCL_index, int QCL_MaxCur, HANDLE_IPS_ACQ handle, uint32_t threshold){

					  uint32_t ret;												//return value used by MIRcat laser control
					  bool IsOn = false;
					  bool santurate = false;
					  int laserpower_high = 100;
					  int tuning_count = 0;
					  if(!(MIRcatSDK_TuneToWW(wn, MIRcatSDK_UNITS_CM1, QCL_index))){											//tuning to wn

						  bool isTuned = false;

						  while(!isTuned)
						  {
							  ret = MIRcatSDK_IsTuned(&isTuned);
							  ::Sleep(500);
						  }

						  std::cout << "Tuned to " << wn << std::endl;
					  }

					  if(!(MIRcatSDK_IsEmissionOn(&IsOn))){

							if(!(MIRcatSDK_TurnEmissionOn())){
								std::cout << "Laser Emission on." << std::endl;
							}
						}

						  uint32_t mean = CalculateMean(handle, fpg);
						  tuning_count++;
						  std::cout << "mean = " << mean << std::endl;
						 
}

int main(int argc, char* argv[]) {

	stim::arglist args;
	args.add("help", "prints usage information");

	args.add("WN", "the wavenumber of CalculateMean", "1490", "integer (currently between 910 and 1900)");

	args.parse(argc, argv);

	if(args["help"]){
		std::cout<<args.str();
		exit(1);
	}

	int wn = args["WN"].as_int();



	int threshold = 10500;

	//
	//Turn on laser
	//

	uint16_t major, minor, patch;								//MIRcat API version variables
	uint32_t ret;												//return value used by MIRcat laser control
	float minWW, maxWW, pulseRateMax, pulseWidthMax, pulseDutyCycleMax, qclTemp, minTemp, maxTemp, nomTemp;
	uint16_t maxCurPulsed, maxCurCw, tecCur;					//MIRcat variables

	printf( "========================================================\n");
	std::cout <<"Getting API version...\n";						//get software version
	if(!(MIRcatSDK_GetAPIVersion(&major, &minor, &patch))){
		std::cout <<"API version: patch "<< major <<"."<<minor <<"."<< patch << std::endl;
	}															

	std::cout <<"API initializing...";							
	if(!(MIRcatSDK_Initialize())){
		std::cout << "done" << std::endl;
		printf( "********************************************************\n");
		printf( "Getting some system information ... \n" );
		printf( "********************************************************\n");

		char stringData[24];
		if(!(MIRcatSDK_GetModelNumber( stringData, 24 ))){		//get MIRcat hardware model number
			std::cout <<"Model Number:"<< stringData << std::endl;
		}

		if(!(MIRcatSDK_GetSerialNumber( stringData, 24 ))){		//get MIRcat serial number
			std::cout <<"Serial Number:"<< stringData << std::endl;
		}

		uint8_t numQcls;
		if(!(MIRcatSDK_GetNumInstalledQcls(&numQcls))){		//get MIRcat hardware model number
			std::cout <<"Number of installed QCLs: "<< (int)numQcls << std::endl;	
		}


		//
		//arm laser
		//check to see if interlock is on
		//if yes, arm the laser
		//operation
		//loop until arm is done
		//
	}

	//
	//Turn on FPA
	//
	printf( "********************************************************\n");
	printf( "Turning on FPA ... \n" );
	printf( "********************************************************\n");
	int32_t src = GetFirstAVailableCaptureSource();		//get the first available capture source
	std::cout<<"Using the following capture source for imaging----------" << std::endl;
	std::string capture_source = ipsPrintCaptureSource(src);


	HANDLE_IPS_ACQ hcam = NULL;									//declare a handle to the camera
	CHECK_IPS(IPS_InitAcq(CAM_ID_SBF161,  									//Configure the SBF 161 camera
		capture_source.c_str(),							//name collected from the above function
		DEFAULT_CONFIG,
		"$(IPS_SDK_DATA_DIR)\\license.lcx",				//specify the license
		&hcam));											//fill the handle

	//
	//imaging
	//
	static const int badpoints_plus2[] = {1456, 1464, 1490, 1498, 1506, 1516, 1540, 1564, 1616, 1622, 1652, 1674, 1684, 1694, 1696, 1698, 1700, 1714, 1734};
	std::vector<int> badwavenumber_plus2(badpoints_plus2, badpoints_plus2 + sizeof(badpoints_plus2) / sizeof(badpoints_plus2[0]));
	static const int badpoints_minus1[] = {1496, 1522, 1538, 1550, 1568, 1634};
	std::vector<int> badwavenumber_minus1(badpoints_minus1, badpoints_minus1 + sizeof(badpoints_minus1) / sizeof(badpoints_minus1[0]));
	static const int badpoints_minus3[] = {1558, 1646};
	std::vector<int> badwavenumber_minus3(badpoints_minus3, badpoints_minus3 + sizeof(badpoints_minus3) / sizeof(badpoints_minus3[0]));

		//tuning laser
		bool * IsOn;										//get the sub folder for saving different wn images


		printf( "========================================================\n");
		std::cout << "Tuning to WN :" <<wn << std::endl;

		if ( std::find(badwavenumber_plus2.begin(), badwavenumber_plus2.end(), wn) != badwavenumber_plus2.end())
			wn = wn + 2;

		if ( std::find(badwavenumber_minus1.begin(), badwavenumber_minus1.end(), wn) != badwavenumber_minus1.end())
			wn = wn - 1;


		if ( wn >= 910 && wn <= 1170){

			comp_mean(wn, 4, 1400, hcam, 10500);

		}

		if ( wn >= 1172 && wn <= 1420){

			comp_mean(wn, 3, 1000, hcam, 10500);

		}

		if ( wn >= 1422 && wn <= 1690){

			comp_mean(wn, 2, 800, hcam, 10500);

		}

		if ( wn >= 1692 && wn <= 1910){

			comp_mean(wn, 1, 550, hcam, 10500);

		}

	
	if(!(MIRcatSDK_TurnEmissionOff())){
		std::cout << "Laser Emission off." << std::endl;
	}
}