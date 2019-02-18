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

std::string GetPGMFileName(const std::string &  parent_directory,
						   const std::string & base_file_name,
						   size_t grab_index,
						   int frame_index)
{
	stim::filename mask(base_file_name);
	stim::filename newfilename = mask.insert(grab_index, (size_t) 5);
	std::stringstream ss;
	ss << parent_directory << newfilename.prefix()<<"_"<< frame_index;
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

uint32_t CalculateMean(HANDLE_IPS_ACQ handle_ips, int fpg)
{
	uint32_t frame_width = 128;
	uint32_t frame_height = 128;
	uint32_t mean = 0;
	uint64_t sum = 0;
	int bytes_per_pixel = 2;
	int frame_data_size = frame_width * frame_height * bytes_per_pixel;

	//std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
	// Configure the frame acquisition window size
	CHECK_IPS(IPS_SetFrameWindow( handle_ips, 
		0,
		0,
		frame_width,
		frame_height));

	// Start capturing a block of fpg frames
	tsi::ips::VMemory<uint64_t> buffer(frame_data_size*fpg);
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
	//std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();

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
	//std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
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

		std::transform ( display_image_all.begin(), display_image_all.end(), display_image.begin(), display_image_all.begin(), std::plus<uint32_t>());\
			//duration_decommute = ( std::clock() - start_decommute ) / (double) CLOCKS_PER_SEC;
			//std::cout << "\nDuration for decommute 1 frame: " << duration_decommute << " seconds"<< std::endl ;

	}
	sum = std::accumulate( display_image_all.begin(), display_image_all.end(), sum );
	mean = (sum / display_image_all.size()) / fpg;

	// Stop acquiring frames
	CHECK_IPS(IPS_StopGrabbing(handle_ips));
	return mean;
}

void CreateDisplayImageExample(HANDLE_IPS_ACQ handle_ips, int fpg, std::string dest_sub_path)
{
	uint32_t frame_width = 128;
	uint32_t frame_height = 128;
	int bytes_per_pixel = 2;
	int frame_data_size = frame_width * frame_height * bytes_per_pixel;

	//std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
	// Configure the frame acquisition window size
	CHECK_IPS(IPS_SetFrameWindow( handle_ips, 
		0,
		0,
		frame_width,
		frame_height));

	//std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
	// Start capturing a block of fpg frames
	tsi::ips::VMemory<uint64_t> buffer(frame_data_size*fpg);
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
	//std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();


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
	std::vector<uint16_t> display_image_all(frame_width * frame_height, 0);
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

		//std::transform ( display_image_all.begin(), display_image_all.end(), display_image.begin(), display_image_all.begin(), std::plus<uint32_t>());
		display_image_all.insert(display_image_all.end(), display_image.begin(), display_image.end());

	}
	//std::transform( display_image_all.begin(), display_image_all.end(), display_image_all.begin(), std::bind2nd(std::multiplies<uint32_t>(), 0.000625));
	//duration_decommute = ( std::clock() - start_decommute ) / (double) CLOCKS_PER_SEC;
	//std::cout << "\nDuration for decommute 1 frame: " << duration_decommute << " seconds"<< std::endl ;

	//p_display_image = display_image.data();
	
	//std::chrono::high_resolution_clock::time_point t5 = std::chrono::high_resolution_clock::now();

	//start_saving_singleframe = std::clock();
	// Save as txt file
	/*SaveGrayScalePGM( p_display_image,
	frame_width,
	frame_height,
	GetPGMFileName(dest_sub_path, "sbf161_img", grab_index, 1600));*/

	//save individual pics

	//for (int frame_index = 0; frame_index < frame_number; frame_index++)
	//{
	//	// Get a pointer to the image
	//	uint16_t * p_single_frame = (uint16_t* ) (display_image_all.data() + frame_index*16384);

	//	const std::string TXTfilename = GetPGMFileName(dest_sub_path, "sbf161_img_*", frame_index, 1600);
	//	std::stringstream grabindex;
	//	grabindex << "s";
	//	stim::save_mat4((char*)p_single_frame, TXTfilename,grabindex.str(), 128, 128, stim::mat4_int16);
	//	rtsProgressBar((float)(frame_index + 1) / (float)fpg * 100);
	//}

	//save as a whole

	uint16_t * p_single_frame =  display_image_all.data();
		const std::string TXTfilename = GetPGMFileName(dest_sub_path, "sbf161_img_*", 1, 1600);
		FILE * picFile;
		picFile = fopen (TXTfilename.c_str(), "wb");
		fwrite( p_single_frame, sizeof(uint16_t), 128*128*fpg, picFile);
		fclose(picFile);

	//WriteArray(TXTfilename.c_str(), p_display_image, frame_height*frame_width);

	//std::chrono::high_resolution_clock::time_point t6 = std::chrono::high_resolution_clock::now();
	//duration_saving_singleframe = ( std::clock() - start_saving_singleframe ) / (double) CLOCKS_PER_SEC;
	//std::cout << "\nDuration for saving 1 frame: " << duration_saving_singleframe << " seconds"<< std::endl ;
	// Stop acquiring frames
	CHECK_IPS(IPS_StopGrabbing(handle_ips));
	//duration_saving = ( std::clock() - start_saving ) / (double) CLOCKS_PER_SEC;
	//std::cout << "\nDuration for saving "<< fpg << " frames: " << duration_saving << " seconds"<< std::endl ;

	/*std::cout << "   "<<std::endl;
	std::cout << "acquring: " << std::chrono::duration_cast<std::chrono::duration<double>>(t5 - t4).count()<<std::endl;
	std::cout << "saving:    " << std::chrono::duration_cast<std::chrono::duration<double>>(t6 - t5).count()<<std::endl;
	std::cout << "total time: " << std::chrono::duration_cast<std::chrono::duration<double>>(t6 - t4).count();
	std::cout << "   "<<std::endl;*/

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

void comp_imaging(int wn, int QCL_index, int QCL_MaxCur, HANDLE_IPS_ACQ handle,
				   int frames, std::string dest_sub_path, uint32_t threshold, int laserpower_low){

					  uint32_t ret;												//return value used by MIRcat laser control
					  bool IsOn = false;
					  bool santurate = false;
					  int laserpower_high = 100;
					  int mean_frame = 100;
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



					  while (!santurate){

					  	  int p = (laserpower_low + laserpower_high) / 2;
					  	  float fCurrentInMilliAmps = QCL_MaxCur * p / 100;
					  	  if(!(MIRcatSDK_SetQCLParams( QCL_index, 100000, 500, fCurrentInMilliAmps))){
								  std::cout << "Set Laser Current to " << p << '%' << std::endl;					//set laser current 
							  }

							  if(!(MIRcatSDK_IsEmissionOn(&IsOn))){

								  if(!(MIRcatSDK_TurnEmissionOn())){
									  std::cout << "Laser Emission on." << std::endl;
								  }
							  }
						  uint32_t mean = CalculateMean(handle, mean_frame);
						  tuning_count++;
						  std::cout << "mean = " << mean << std::endl;
						  int diff = mean - threshold;

						  if(abs(diff) <= 150 || tuning_count >= 7){
								santurate = true;
								CreateDisplayImageExample(handle, frames, dest_sub_path);											//capture images		 
						}
						  else if (mean > threshold)
						  	  laserpower_high = p - 1;
						  else if (mean < threshold)
						  	  laserpower_low = p + 1;
						}
}

int main(int argc, char* argv[]) {

	stim::arglist args;
	args.add("help", "prints usage information");
	args.add("frames", "total number of images to collect", "50", "integer (currently between 1 and 500)");
	args.add("WN", "wavenumber to do imaging at", "1250", "integer (currently between 910 and 1900)");
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

	ipsPrintDiagnostics(hcam);

	//
	//imaging
	//



		//tuning laser
		
		
		bool * IsOn;										//get the sub folder for saving different wn images


		printf( "========================================================\n");
		std::cout << "Tuning to WN :" << wn << std::endl;

			std::stringstream sub_dir;												//create an empty string stream
			sub_dir << dest_path << wn << "\\";												//append to the parent dir string
			std::string dest_sub_path = sub_dir.str();
			int mkdirFlag = mkdir(dest_sub_path.c_str());
			if (mkdirFlag != 0){
				printf ("Error : %s\n", strerror(errno));
			}
		
			if ( wn >= 910 && wn <= 1170){

			comp_imaging(wn, 4, 1400, hcam, frames, dest_sub_path, 9300, 60);

			}

			if ( wn >= 1172 && wn <= 1402){

			comp_imaging(wn, 3, 1000, hcam, frames, dest_sub_path, 9500, 40);

			}

			if ( wn >= 1404 && wn <= 1700){

			comp_imaging(wn, 2, 800, hcam, frames, dest_sub_path, 9300, 60);

			}

			if ( wn >= 1702 && wn <= 1910){

			comp_imaging(wn, 1, 550, hcam, frames, dest_sub_path, 9300, 60);

			}
		

	
		if(!(MIRcatSDK_TurnEmissionOff())){
			std::cout << "Laser Emission off." << std::endl;
		}
		//if(!(MIRcatSDK_DisarmLaser())){
			//std::cout << "Laser Disarmed." << std::endl;
		//}
	}