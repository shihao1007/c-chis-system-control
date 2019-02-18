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

std::string dest_path;		//stores the destination path for all output files

A3200Handle hstage = NULL;	//handle for the stage controller

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

void aerotechHandleError(bool err, const char* file, int line) {			//function for handling errors from the Aerotech stage controller
	CHAR data[1024];
	A3200GetLastErrorString(data, 1024);
	std::cout<<"Aerotech ERROR: "<<data<<std::endl;	
	exit(1);
}
#define CHECK_A3200( err ) (ipsHandleError(err, __FILE__, __LINE__))


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

std::string GetPGMFileName(const std::string &  parent_directory,
						   const std::string & base_file_name,
						   size_t grab_index,
						   int frame_index)
{
	stim::filename mask(base_file_name);
	stim::filename newfilename = mask.insert(grab_index, (size_t) 5);
	std::stringstream ss;
	/*ss << parent_directory << newfilename.prefix()<<"_"<< frame_index << ".mat";*/
	ss << parent_directory << newfilename.prefix()<<"_"<< frame_index << ".bin";
	return ss.str();
}

void WriteArray(const char *filename, uint32_t *data, size_t size) {
	FILE *fp;
	//open file for output
	fp = fopen(filename, "wb");
	if (!fp) {
		fprintf(stderr, "Unable to open file '%s'\n", filename);
		exit(1);
	}

	fwrite(data, sizeof(uint32_t), size, fp);
	fclose(fp);
}

void SaveGrayScalePGM(uint16_t * p_pixel_data, int width, int height, const std::string pgm_path) 
{ 
  std::ofstream ofile(pgm_path, std::ios::out|std::ios::ate);
  //ofile << "P2\n" << width << " " << height << "\n16383\n";
  int num_pixels = width*height;
  for (int i = 0; i < num_pixels; i++)
  {
    ofile << p_pixel_data[i] << " ";
  }
  ofile.close();
} 

void CreateDisplayImageExample(HANDLE_IPS_ACQ handle_ips, int position_index, int fpg, std::string dest_sub_path)
{
	uint32_t frame_width = 128;
	uint32_t frame_height = 128;
	int bytes_per_pixel = 2;
	int frame_data_size = frame_width * frame_height * bytes_per_pixel;

	std::chrono::high_resolution_clock::time_point tgrab_start = std::chrono::high_resolution_clock::now();
	// Configure the frame acquisition window size
	CHECK_IPS(IPS_SetFrameWindow( handle_ips, 
		0,
		0,
		frame_width,
		frame_height));

	//std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
	// Start capturing a block of fpg frames
	tsi::ips::VMemory<uint16_t> buffer(frame_data_size*fpg);
	//std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

	CHECK_IPS(IPS_StartGrabbing( handle_ips,         
		fpg,            // Capture Fra_Number frames then stop capturing
		buffer.data(), // User allocated buffer
		buffer.size(), // size of user allocated buffer
		false));        // No wrap


	//std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
	// Wait for all frames to be acquired
	uint64_t frame_number;
	uint8_t * p_frame = NULL;
	CHECK_IPS(IPS_WaitFrame(handle_ips,
		fpg,                          // Wait until the number of frame has been captured
		IPS_TIMEOUT_WAIT_FOREVER,    // Don't time out, wait for ever
		false,                       // Pause after WaitFrame returns
		&p_frame,                    // Return a pointer to the frame
		&frame_number));              // Return the frame number of the returned frame
	std::chrono::high_resolution_clock::time_point tgrab_end = std::chrono::high_resolution_clock::now();


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
	
	// Decommute the images.
	std::vector<uint16_t> display_image(frame_width * frame_height);
	std::vector<uint16_t> display_image_mean(frame_width * frame_height);
	std::vector<uint32_t> display_image_all(frame_width * frame_height, 0);
	//float aver_rate = 1.000000 / fpg;
	uint32_t * p_display_image;
	//std::string module_dir = GetModuleDirectory();
	//std::string image_dir = module_dir + "\Frames1800\\";

	for (int frame_index = 0; frame_index < frame_number; frame_index++)
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

		// add image vectors elemental wise

		std::transform ( display_image_all.begin(), display_image_all.end(), display_image.begin(), display_image_all.begin(), std::plus<uint32_t>());

		rtsProgressBar((float)(frame_index + 1) / (float)fpg * 100);
		
		// concatenating vectors one by one
		
		//display_image_all.insert(display_image_all.end(), display_image.begin(), display_image.end());

	}
	std::cout << "   "<<std::endl;
	std::chrono::high_resolution_clock::time_point tdecommute_end = std::chrono::high_resolution_clock::now();
	// average all captured frames into one vector

	std::transform( display_image_all.begin(), display_image_all.end(), display_image_mean.begin(),std::bind2nd(std::divides<uint32_t>(),fpg));

	//std::transform( display_image_all.begin(), display_image_all.end(), display_image_all.begin(), std::bind2nd(std::multiplies<uint32_t>(), 0.000625));
	//duration_decommute = ( std::clock() - start_decommute ) / (double) CLOCKS_PER_SEC;
	//std::cout << "\nDuration for decommute 1 frame: " << duration_decommute << " seconds"<< std::endl ;

	//p_display_image = display_image.data();
	
	//std::chrono::high_resolution_clock::time_point t5 = std::chrono::high_resolution_clock::now();

	//start_saving_singleframe = std::clock();
	// Save as txt file

	//uint16_t * p_single_frame =  display_image_mean.data();
	//const std::string TXTfilename = GetPGMFileName(dest_sub_path, "sbf161_img_*", position_index, 1600);
	//FILE * picFile;
	//SaveGrayScalePGM( p_single_frame,
	//frame_width,
	//frame_height,
	//TXTfilename);

	//save as a binary file
	uint16_t * p_single_frame =  display_image_mean.data();
	const std::string TXTfilename = GetPGMFileName(dest_sub_path, "sbf161_img_*", position_index, 1250);
	FILE * picFile;
	picFile = fopen (TXTfilename.c_str(), "wb");
	fwrite( p_single_frame, sizeof(uint16_t), 128*128, picFile);
	fclose(picFile);

	std::chrono::high_resolution_clock::time_point tsave_end = std::chrono::high_resolution_clock::now();
	//save individual pics
	
	//for (int frame_index = 0; frame_index < frame_number; frame_index++)
	//{
	//	// Get a pointer to the image
	//	uint16_t * p_single_frame = (uint16_t* ) (display_image_all.data() + frame_index*16384);

	//	const std::string TXTfilename = GetPGMFileName(dest_sub_path, "sbf161_img_*", frame_index, 1600);
	//	std::stringstream grabindex;
	//	grabindex << "s";
	//	stim::save_mat4((char*)p_single_frame, TXTfilename,grabindex.str(), 128, 128, stim::mat4_int16);
	//	
	//	rtsProgressBar((float)(frame_index + 1) / (float)fpg * 100);
	//
	//}
	//	std::cout << "   "<<std::endl;


	//save as a whole

	//uint16_t * p_single_frame =  display_image_mean.data();
	//	const std::string TXTfilename = GetPGMFileName(dest_sub_path, "sbf161_img_*", position_index, 1600);
	//	FILE * picFile;
	//	picFile = fopen (TXTfilename.c_str(), "wb");
	//	fwrite( p_single_frame, sizeof(uint16_t), 128*128*fpg, picFile);
	//	fclose(picFile);

	//WriteArray(TXTfilename.c_str(), p_display_image, frame_height*frame_width);

	//std::chrono::high_resolution_clock::time_point t6 = std::chrono::high_resolution_clock::now();
	//duration_saving_singleframe = ( std::clock() - start_saving_singleframe ) / (double) CLOCKS_PER_SEC;
	//std::cout << "\nDuration for saving 1 frame: " << duration_saving_singleframe << " seconds"<< std::endl ;
	// Stop acquiring frames
	CHECK_IPS(IPS_StopGrabbing(handle_ips));
	//duration_saving = ( std::clock() - start_saving ) / (double) CLOCKS_PER_SEC;
	//std::cout << "\nDuration for saving "<< fpg << " frames: " << duration_saving << " seconds"<< std::endl ;

	
	//std::cout << "Grabbing 1000 frames: " << std::chrono::duration_cast<std::chrono::duration<double>>(tgrab_end - tgrab_start).count()<< "s" <<std::endl;
	//std::cout << "Decommute:    " << std::chrono::duration_cast<std::chrono::duration<double>>(tdecommute_end - tgrab_end).count()<< "s" <<std::endl;
	//std::cout << "Saving: " << std::chrono::duration_cast<std::chrono::duration<double>>(tsave_end - tdecommute_end).count()<< "s" <<std::endl;;
	//std::cout << "   "<<std::endl;

}

void ipsPrintDiagnostics(HANDLE_IPS_ACQ handle) {
	// Display the camera and frame grabber diagnostic data
	std::vector<char> diag_buffer(IPS_MAX_DIAGNOSTIC_STRING_BYTES);		//allocate a character array
	uint32_t diag_buffer_size(0);									//initialize the buffer size to zero (0)
	//get the camera diagnostics - fill diag_buffer with a string describing the camera status								
	CHECK_IPS(IPS_GetCameraDiagnostics(handle,
		diag_buffer.data(),
		(uint32_t)diag_buffer.size(),
		&diag_buffer_size));

	if (strlen(diag_buffer.data()) == diag_buffer_size - 1)			//if the diagnostic string exists
		std::cout << "Camera diagnostics : " << std::string(diag_buffer.data()) << std::endl;	//print it
	//get the frame grabber diagnostics
	CHECK_IPS(IPS_GetFrameGrabberDiagnostics(handle,
		diag_buffer.data(),
		(uint32_t)diag_buffer.size(),
		&diag_buffer_size));

	if (strlen(diag_buffer.data()) == diag_buffer_size - 1)			//if the diagnostic string exists
		std::cout << "Frame grabber diagnostics : " << std::string(diag_buffer.data()) << std::endl;	//print it
}

int32_t IntegrationTimeWriteExample(HANDLE_IPS_ACQ handle_ips,
                                    int row_ticks,
                                    int col_ticks){
  // First write the desired row and col integration ticks
  int32_t result = IPS_SetInt32Param( handle_ips,
                                      IPS_WRITE_INTEGRATION_ROW_TICKS,
                                      row_ticks);
  if (IPS_FAILED(result)) return result;

  result = IPS_SetInt32Param( handle_ips,
                              IPS_WRITE_INTEGRATION_COL_TICKS,
                              col_ticks);
  if (IPS_FAILED(result)) return result;

  // Write the integration time using the last stored row and col ticks
  result = IPS_SetInt32Param( handle_ips,
                              IPS_WRITE_INTEGRATION_TIME,
                              0 /*not used*/);
  if (IPS_FAILED(result)) return result;
  cout << "Integration time successfully written to camera" << endl;
  return result;
}

void aerotechCleanup(A3200Handle h) {
	CHECK_A3200(A3200ProgramStop(h, TASKID_01));
	CHECK_A3200(A3200MotionDisable(h, TASKID_01, AXISMASK_00));
	CHECK_A3200(A3200Disconnect(h));
}

void holo_capture(int wn, int positions, int QCL_index, int QCL_MaxCur, HANDLE_IPS_ACQ handle, DOUBLE result_stage,
				  std::string cmd_step, std::string cmd_return, int frames, int lp, std::string dest_sub_path){

					  uint32_t ret;												//return value used by MIRcat laser control
					  bool IsOn = false;

					  if(!(MIRcatSDK_TuneToWW(wn, MIRcatSDK_UNITS_CM1, QCL_index))){											//tuning to wn

						  bool isTuned = false;

						  while(!isTuned)
						  {
							  ret = MIRcatSDK_IsTuned(&isTuned);
							  ::Sleep(500);
						  }

						  std::cout << "Tuned to " << wn << std::endl;
					  }


					  	  float fCurrentInMilliAmps = QCL_MaxCur * lp / 100;
					  	  if(!(MIRcatSDK_SetQCLParams( QCL_index, 100000, 500, fCurrentInMilliAmps))){
								  std::cout << "Set Laser Current to " << lp << '%' << std::endl;					//set laser current 
							  }

							  if(!(MIRcatSDK_IsEmissionOn(&IsOn))){

								  if(!(MIRcatSDK_TurnEmissionOn())){
									  std::cout << "Laser Emission on." << std::endl;
								  }
							  }

							  //connect to the A3200 Aerotech stage controller
							  std::cout <<"Connecting to A3200...";
							  CHECK_A3200(A3200Connect(&hstage));										//attempt to connect to the controller
							  std::cout << "done" << std::endl;

							  std::cout <<"Enabling axes...";			    							//enable the axes
							  CHECK_A3200(A3200MotionEnable(hstage, TASKID_01, AXISMASK_00));
							  std::cout << "done" << std::endl;



							  //perform an imaging pass across the sample

							  for (int i = 0; i < positions; i++){


								  A3200CommandExecute(hstage, TASKID_01, cmd_step.c_str(), &result_stage);		//move the stage
				//				  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
								  A3200CommandExecute(hstage, TASKID_01, "MOVEDELAY Z, 200", &result_stage);		//wait
								  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
								  std::cout << "   "<<std::endl;
								  std::cout << "capturing images at "<< i+1<<"th position"<< std::endl;				//display the number of images
								  CreateDisplayImageExample(handle, i, frames, dest_sub_path);											//capture images		 
								  std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();

								 // rtsProgressBar((float)(i + 1) / (float)positions * 100);

								 

								  A3200CommandExecute(hstage, TASKID_01, "MOVEDELAY Z, 100", &result_stage);		//wait again

								  
								  //std::cout << "move stage once: " << std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count()<<std::endl;
								  //std::cout << "wait after move: " << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()<<std::endl;
								  //std::cout << "Time spent on this position: " << std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count()<< "s" <<std::endl;

							  }
									
							  A3200CommandExecute(hstage, TASKID_01, cmd_return.c_str(), &result_stage);				//move stage back to origin
							  		A3200CommandExecute(hstage, TASKID_01, "MOVEDELAY Z, 2000", &result_stage);		//wait again
						  }
						


int main(int argc, char* argv[]) {

	std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

	stim::arglist args;
	args.add("help", "prints usage information");
	args.add("zstep", "number of micrometers between images", "1", "positive value describing stage motion in microns");
	args.add("frames", "total number of images to collect", "1000", "integer (currently between 1 and 5000)");
	args.add("WN", "wavenumber to do imaging at", "1250", "integer (currently between 910 and 1900)");
	args.add("laserpower","laser power to do imaging at","90","integer (currently between 1 and 100)");
	args.add("positions","number of different hologram positions","20","integer (currently between 1 and 100)");
	args.add("inte","integration time of grabbing a single frame","19","integer (currently between 15 and 45)");
	args.parse(argc, argv);

	if(args["help"]){
		std::cout<<args.str();
		exit(1);
	}

	if(args.nargs() > 0) dest_path = args.arg(0);						//get the destination path (if specified)
	std::cout<<"\t\t=====>> dest_path "<<dest_path<<std::endl;

	if(dest_path.back() != '\\' || dest_path.back() != '/')
		dest_path += "\\";

	int frames = args["frames"].as_int();									//get the number of images to be acquired		
	int wn = args["WN"].as_int();
	int lp = args["laserpower"].as_int();
	int positions = args["positions"].as_int();
	int inte = args["inte"].as_int();

	DOUBLE result_stage;										//return value used by Aerotech stage control


	std::stringstream ss;												//create an empty string stream
	float dz = (float)args["zstep"].as_float() * 0.001f;						//get the z step size in micrometers and convert to millimeters
	ss << "LINEAR Z"<<dz;												//append to the command string
	std::string cmd_step = ss.str();									//store the move command in a string

	std::stringstream bb;
	float zpass = dz * positions;											//calculate the length of an entire imaging pass
	bb << "LINEAR Z-" << zpass;											//generate the stage return command
	std::string cmd_return = bb.str();									//store in a string	


	
	//Turn on laser
	

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
	}


		//
		//arm laser
		//check to see if interlock is on
		//if yes, arm the laser
		//operation
		//loop until arm is done
		//

		bool bIlockSet = false;
		bool bKeySwitchSet = false;
		bool IsArmed = false;
		bool atTemp = false;

		printf( "********************************************************\n");
		printf( "Turn on Laser ... \n" );
		printf( "********************************************************\n");
		if(!(MIRcatSDK_IsInterlockedStatusSet(&bIlockSet) && !bIlockSet)){
			std::cout <<"Interlock Set"<< std::endl;
			/*if(!(MIRcatSDK_IsLaserArmed(&IsArmed) && !IsArmed)){
				ret = MIRcatSDK_ArmDisarmLaser();
				std::cout <<"Arming Laser...";
				while ( !IsArmed )
				{   
					ret = MIRcatSDK_IsLaserArmed(&IsArmed);
					::Sleep(1000);
				}
				std::cout << "done" << std::endl;
			}*/
		}


		//wait until laser cool down to do tuning

		/*printf( "********************************************************\n");
		printf( "Cooling down Laser ... \n" );
		printf( "********************************************************\n");
		if(!(MIRcatSDK_AreTECsAtSetTemperature(&atTemp) && !atTemp)){
			while ( !atTemp )
			{   
				for( uint8_t i = 1; i <= numQcls; i++ )
				{
					if(!(MIRcatSDK_GetQCLTemperature( i, &qclTemp )) && !(MIRcatSDK_GetTecCurrent( (int)i, &tecCur ))){
						printf(" QCL%u    Temp: %.3f C     TEC%u       Current: %u mA\n", (int)i, qclTemp, (int)i, tecCur );
					}						
				}
				printf( "********************************************************\n");
				printf("   \n");
				ret = MIRcatSDK_AreTECsAtSetTemperature(&atTemp);
				::Sleep(1000);
			}
			std::cout << "done" << std::endl;
		}*/
	

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

	//calculate row ticks and column ticks for specified integration time
	
	int r = inte / 17.25;
	int c = (inte - r * 17.25) / 15 * 120;

	int r_tick = 1 + r;										// R_tick = 1 + 1 means 17.25 microsecond integration time
	int c_tick = 1 + c;                                   // C_tick = 1 + 120 means 15 microsecond integration time
		 
	
	//set the specified integration time to the camera head
	int32_t intewrite = IntegrationTimeWriteExample(hcam, r_tick, c_tick);

	//print out the camera parameters
	ipsPrintDiagnostics(hcam);

	//
	//imaging
	//

	std::stringstream sub_dir;												//create an empty string stream
	sub_dir << dest_path << "\\";												//append to the parent dir string
	std::string dest_sub_path = sub_dir.str();
	int mkdirFlag = mkdir(dest_sub_path.c_str());
	if (mkdirFlag != 0){
	printf ("Error : %s\n", strerror(errno));
	}

		//tuning laser
		
		
		bool * IsOn;										//get the sub folder for saving different wn images


		printf( "========================================================\n");
		std::cout << "Tuning to WN :" << wn << std::endl;

			
		
			if ( wn >= 910 && wn <= 1170){

			holo_capture(wn, positions, 4, 1400, hcam, result_stage, cmd_step, cmd_return, frames, lp, dest_sub_path);

			}

			if ( wn >= 1172 && wn <= 1402){

			holo_capture(wn, positions, 3, 1000, hcam, result_stage, cmd_step, cmd_return, frames, lp, dest_sub_path);

			}

			if ( wn >= 1404 && wn <= 1700){

			holo_capture(wn, positions, 2, 800, hcam, result_stage, cmd_step, cmd_return, frames, lp, dest_sub_path);

			}

			if ( wn >= 1702 && wn <= 1910){

			holo_capture(wn, positions, 1, 550, hcam, result_stage, cmd_step, cmd_return, frames, lp, dest_sub_path);

			}
		

		std::cout << "   "<<std::endl;
		if(!(MIRcatSDK_TurnEmissionOff())){
			std::cout << "Laser Emission off." << std::endl;
		}
		std::cout << "   "<<std::endl;
		std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
		std::cout << "Total Imaging Time: " << std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t1).count()<< "s" <<std::endl;
		//if(!(MIRcatSDK_DisarmLaser())){
			//std::cout << "Laser Disarmed." << std::endl;
		//}
	}