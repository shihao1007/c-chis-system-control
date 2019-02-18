#define _CRT_SECURE_NO_WARNINGS

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
// Main include file


// See the pre-build event to see the System Library dlls being copied to the output directory

#define NUM_LINES 10
#define LINE_SIZE 128

// This function will print whatever the latest error was
void PrintError();


using namespace std;
using namespace tsi::ips;

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

string GetModuleDirectory()
{
  char szModuleFileName[MAX_PATH+1], szDrive[_MAX_DRIVE+1], szDir[_MAX_DIR+1], szFile[_MAX_FNAME+1], szExt[_MAX_EXT+1];
  GetModuleFileName(NULL, szModuleFileName,MAX_PATH+1);
  _splitpath(szModuleFileName,szDrive,szDir,szFile,szExt);
  char szDirectoryName[MAX_PATH+1];
  _makepath(szDirectoryName, szDrive, szDir, NULL, NULL);
  return string(szDirectoryName);
}

string GetPGMFileName(const string &  parent_directory,
                      const string & base_file_name,
					  int grab_index,
                      int frame_index)
{
  std::stringstream ss;
  ss << parent_directory << base_file_name << "_" <<grab_index<<"_"<< frame_index << ".pgm";
  return ss.str();
}

///
/// <summary>
/// Saves a 16 bpp grayscale image to PGM encoded file.  
/// </summary>
/// <remarks>
///  IrfanView can be used to display PGM images. http://www.irfanview.com/
/// </remarks>
void SaveGrayScalePGM(uint16_t * p_pixel_data, int width, int height, const std::string pgm_path) 
{ 
  std::ofstream ofile(pgm_path, ios::out|ios::ate);
  ofile << "P2\n" << width << " " << height << "\n16383\n";
  int num_pixels = width*height;
  for (int i = 0; i < num_pixels; i++)
  {
    ofile << p_pixel_data[i] << " ";
  }
  ofile.close();
} 

void Initialize_SBF161_Decommute_Table( int grab_num_cols,
                                        int grab_num_rows,
                                        int display_num_cols,
                                        int display_num_rows,
                                        std::vector<int> & decommute_table)
{
  int num_pixels = grab_num_cols * grab_num_rows;

  for (int i = 0; i < num_pixels; i++)
  {
    int display_row = (i % 4) + 4 * (i / grab_num_cols);
    int display_col = (i / 4) % display_num_cols;
    int display_pixel = display_row * display_num_cols + display_col;
    if (display_pixel < decommute_table.size())
    {
      decommute_table[display_pixel] = i;
    }
  }
}

//enumerate capture sources and display them on the console
void ListCaptureSourcesExample(){
  uint32_t num_capture_sources = 0;										//stores the number of camera devices
  int32_t result = IPS_GetCaptureSourceCount( IPS_DEVICE_TYPE_CAMERALINK,
                                              &num_capture_sources);	//get the number of camera devices
 
  if (IPS_SUCCEEDED(result)){											//if the API call is successful
    for (uint32_t i = 0; i < num_capture_sources; i++){					//for each capture device
      char capture_source[IPS_MAX_CAPTURE_SOURCE_BYTES];				//name of the capture source
      char capture_source_descr[IPS_MAX_CAPTURE_SOURCE_BYTES];			//description of the camera
      //get the name and description for the capture device
      result = IPS_GetCaptureSource(IPS_DEVICE_TYPE_CAMERALINK,
                                    i,
                                    capture_source, sizeof(capture_source),
                                    capture_source_descr, sizeof(capture_source_descr));
      if (IPS_SUCCEEDED(result))										//if the API call is successful
        cout << capture_source << "\nDescription: " << capture_source_descr << endl;
      else																//otherwise output an error
        cout << "Failed to get capture source information.  result :  " <<  result << endl;
    }
  }
  else																	//if the API call (# devices) fails
    cout << "No capture source found!" << endl;							//output an error message
}


void InitDeinitExample()
{
  uint32_t num_capture_sources = 0;
  int32_t result = IPS_GetCaptureSourceCount( IPS_DEVICE_TYPE_CAMERALINK,
                                              &num_capture_sources);
  if (IPS_FAILED(result))
  {
    cout << "Failed to get capture source due to an error.  Error code = " 
          << result << endl;
    return;
  }

  if (num_capture_sources == 0)
  {
    cout << "No capture sources were found." << result << endl;
    return;
  }

  // Get the first capture source
  char capture_source[IPS_MAX_CAPTURE_SOURCE_BYTES];
  char capture_source_descr[IPS_MAX_CAPTURE_SOURCE_BYTES];
  result = IPS_GetCaptureSource(
                      IPS_DEVICE_TYPE_CAMERALINK,
                      0,
                      capture_source, sizeof(capture_source),
                      capture_source_descr, sizeof(capture_source_descr));
  if (IPS_FAILED(result))
  {
    cout << "Failed to get capture source.  Error code = " << result << endl;
    return;
  }

  // Initialize the library with the capture source
  HANDLE_IPS_ACQ handle_ips = NULL;
  result = IPS_InitAcq( CAM_ID_SBF161,  // Configure the SBF 161 camera
                        capture_source,
                        DEFAULT_CONFIG,
                        "$(IPS_SDK_DATA_DIR)\\license.lcx",
                        &handle_ips);

  if (IPS_SUCCEEDED(result))
  {
    // IPS frame acquisition is initialized
    // <<< Do useful things here >>>
    cout << "IPS frame acquisition is initialized.  Capture source is " 
         << capture_source << endl;
  }

  if (handle_ips)
  {
    // Always clean up.
    IPS_DeinitAcq(handle_ips);
  }
}

int32_t AcquireFramesExample(HANDLE_IPS_ACQ handle_ips)
{
  uint32_t frame_width = 128;
  uint32_t frame_height = 128;
  int bytes_per_pixel = 2;
  int frame_data_size = frame_width * frame_height * bytes_per_pixel;

  // Configure the frame acquisition window to acquire 128x128 frames
  int32_t result =  IPS_SetFrameWindow(handle_ips, 
                                        0,
                                        0,
                                        frame_width,
                                        frame_height);
  if (IPS_FAILED(result)) return result;

  // Display the camera and frame grabber diagnostic data
  vector<char> diag_buffer(IPS_MAX_DIAGNOSTIC_STRING_BYTES);
  uint32_t diag_buffer_size(0);
  result = IPS_GetCameraDiagnostics( handle_ips,
                                        diag_buffer.data(),
                                        (uint32_t) diag_buffer.size(),
                                        &diag_buffer_size);
  if (IPS_FAILED(result)) return result;
  if (strlen(diag_buffer.data()) == diag_buffer_size-1)
    cout << "Camera diagnostics : " << string(diag_buffer.data()) << endl;
  result = IPS_GetFrameGrabberDiagnostics( handle_ips,
                                              diag_buffer.data(),
                                              (uint32_t) diag_buffer.size(),
                                              &diag_buffer_size);
  if (IPS_FAILED(result)) return result;
  if (strlen(diag_buffer.data()) == diag_buffer_size-1)
    cout << "Frame grabber diagnostics : " << string(diag_buffer.data()) << endl;

  // Start capturing a block of 1000 frames
  VMemory<uint8_t> buffer(frame_data_size*1000);
  result = IPS_StartGrabbing( handle_ips,         
                              1000,          // Capture 1000 frames then stop capturing
                              buffer.data(), // User allocated buffer
                              buffer.size(), // size of user allocated buffer
                              false);        // No wrap
  if (IPS_FAILED(result)) return result;

  // Wait for all 1000 frames to be acquired and obtain a pointer to the 1000th frame
  uint64_t frame_number;
  uint8_t * p_frame = NULL;
  result = IPS_WaitFrame(handle_ips,
           1000,                        // Wait until the number of frame has been captured
           IPS_TIMEOUT_WAIT_FOREVER,    // Don't time out, wait for ever
           false,                       // No pause after WaitFrame returns
           &p_frame,                    // Return a pointer to the frame
           &frame_number);              // Return the frame number of the returned frame
  if (IPS_FAILED(result)) return result;
  cout << "Successfully acquired 1000 frames" << endl;

  // Stop acquiring frames
  result = IPS_StopGrabbing(handle_ips);
  return result;
}

int32_t ContinuousFrameAcquisitionExample(HANDLE_IPS_ACQ handle_ips)
{
  uint32_t frame_width = 128;
  uint32_t frame_height = 128;
  int bytes_per_pixel = 2;
  int frame_data_size = frame_width * frame_height * bytes_per_pixel;

  // Configure the frame acquisition window to acquire 128x128 frames
  int32_t result =  IPS_SetFrameWindow(handle_ips, 
                                  0,
                                  0,
                                  frame_width,
                                  frame_height);
  if (IPS_FAILED(result)) return result;

  // Start a continuous capture into a ring buffer capable of 
  // storing 1000 frames
  VMemory<uint8_t> buffer(frame_data_size*1000);
  result = IPS_StartContinuousGrabbing(handle_ips,         
                                          buffer.data(), // User allocated buffer
                                          buffer.size()); // size of user allocated buffer
  if (IPS_FAILED(result)) return result;

  // Sample 20 frames
  for (int i = 0; i < 20; i++)
  {
    // Wait for the next frame to be acquired and automatically pause
    // frame acquisition to give time to process the frame(s).
    uint64_t frame_number;
    uint8_t * p_frame = NULL;
    result = IPS_WaitFrame(handle_ips,
             IPS_GET_NEXT_FRAME,      // Wait until the number of frame has been captured
             IPS_TIMEOUT_WAIT_FOREVER,// Don't time out, wait for ever
             true,                    // Pause after WaitFrame returns
             &p_frame,                // Return a pointer to the frame
             &frame_number);          // Return the frame number of the returned frame
    if (IPS_FAILED(result)) return result;

    cout << "Acquired frame number : " << frame_number << endl;

    // Resume frame acquistion (it was paused in call to IPS_WaitFrame)
    result = IPS_ResumeGrabbing(handle_ips);
    if (IPS_FAILED(result)) return result;

    // Sleep for a bit to mimick a GUI sampling frames
    Sleep(300);
  }

  // Stop acquiring frames
  result = IPS_StopGrabbing(handle_ips);
  return result;
}

int32_t CreateDisplayImageExample(HANDLE_IPS_ACQ handle_ips, int grab_index, int Fra_Number)
{
  uint32_t frame_width = 128;
  uint32_t frame_height = 128;
  int bytes_per_pixel = 2;
  int frame_data_size = frame_width * frame_height * bytes_per_pixel;
  int32_t result = 0;
  

  // Read the current test mode so it can be restored later
  //int test_mode = 0;
  //result = IPS_GetInt32Param(handle_ips, IPS_READ_COUNTER_TEST_MODE, &test_mode);
  //if (IPS_FAILED(result)) return result;

  // Enable test mode - to verify frame acquisition and decommute is functioning properly
  //result = IPS_SetInt32Param(handle_ips, IPS_WRITE_COUNTER_TEST_MODE, 1);
  //if (IPS_FAILED(result)) return result;

  // Configure the frame acquisition window size
  result =  IPS_SetFrameWindow( handle_ips, 
                                0,
                                0,
                                frame_width,
                                frame_height);
  if (IPS_FAILED(result)) return result;

   

  // Start capturing a block of Fra_Number frames
  VMemory<uint8_t> buffer(frame_data_size*Fra_Number);
  result = IPS_StartGrabbing( handle_ips,         
                              Fra_Number,            // Capture Fra_Number frames then stop capturing
                              buffer.data(), // User allocated buffer
                              buffer.size(), // size of user allocated buffer
                              false);        // No wrap
  if (IPS_FAILED(result)) return result;

  // Wait for all frames to be acquired
  uint64_t frame_number;
  uint8_t * p_frame = NULL;
  result = IPS_WaitFrame(handle_ips,
           Fra_Number,                          // Wait until the number of frame has been captured
           IPS_TIMEOUT_WAIT_FOREVER,    // Don't time out, wait for ever
           false,                       // Pause after WaitFrame returns
           &p_frame,                    // Return a pointer to the frame
           &frame_number);              // Return the frame number of the returned frame
  if (IPS_FAILED(result)) return result;


  // *********** Decommute ********************
  //
  // The data coming from the SBF 161 must be de-commuted before it can be displayed.   
  // The first four pixels go in the first columns of the first four rows, the next 4 pixels 
  // in the second column and so on to the last column.  Then it continues to rows 5-8 and so on.

  // Create a decommute table for recording the pixels for display as an image
  vector<int> decommute_table(frame_width*frame_height);
  Initialize_SBF161_Decommute_Table(frame_width * 4,
                                    frame_height/4,
                                    frame_width,
                                    frame_height,
                                    decommute_table);

  // Decommute the images.
  vector<uint16_t> display_image(frame_width * frame_height);
  string module_dir = GetModuleDirectory();
  string image_dir = module_dir + "\Frames1800\\" ;

  for (int frame_index = 0; frame_index < frame_number; frame_index++)
  {
    // Get a pointer to the image
    uint16_t * p_image = (uint16_t* ) (buffer.data() + frame_index*frame_data_size);

    // Decommute the image
    for (int i = 0; i < display_image.size(); i++)
    {
      display_image[i] = p_image[decommute_table[i]];

      // Invert the image pixel 
	  // uncomment the following line to invert the image
      display_image[i] = display_image[i] ^ 0x3FFF;
    }

    uint16_t * p_display_image = display_image.data();

    // Save as PGM image
    SaveGrayScalePGM( p_display_image,
                      frame_width,
                      frame_height,
                      GetPGMFileName(image_dir, "sbf161_img", grab_index, frame_index+1));
  }

  // Restore the previous test mode
  //result = IPS_SetInt32Param(handle_ips, IPS_WRITE_COUNTER_TEST_MODE, test_mode);
  //if (IPS_FAILED(result)) return result;

  // Stop acquiring frames
  result = IPS_StopGrabbing(handle_ips);

  //cout << "Successfully Grabbed" << endl;

  return result;
}

int32_t JamSyncModeExample(HANDLE_IPS_ACQ handle_ips)
{
  // Set Jam Sync to IPS_JAM_SYNC_RS422_IN and verify
  // by reading it back
  int32_t result = IPS_SetInt32Param( handle_ips,
                                      IPS_WRITE_JAM_SYNC,
                                      IPS_JAM_SYNC_RS422_IN);
  if (IPS_FAILED(result)) return result;

  int jam_sync_mode(IPS_JAM_SYNC_NONE);
  result = IPS_GetInt32Param( handle_ips,
                              IPS_READ_JAM_SYNC,
                              &jam_sync_mode);
  if (IPS_FAILED(result)) return result;
  if (jam_sync_mode == IPS_JAM_SYNC_RS422_IN)
    cout << "Successfully set JAM Synch mode to RS422_IN" << endl;

  // Reset Jam Sync to IPS_JAM_SYNC_NONE and verify by reading
  // it back
  result = IPS_SetInt32Param( handle_ips,
                              IPS_WRITE_JAM_SYNC,
                              IPS_JAM_SYNC_NONE);
  if (IPS_FAILED(result)) return result;

  result = IPS_GetInt32Param( handle_ips,
                              IPS_READ_JAM_SYNC,
                              &jam_sync_mode);
  if (IPS_FAILED(result)) return result;
  if (jam_sync_mode == IPS_JAM_SYNC_NONE)
    cout << "Successfully reset JAM Synch mode to NONE" << endl;

  return result;
}

int32_t CounterTestModeExample(HANDLE_IPS_ACQ handle_ips, bool enable)
{
  // Set counter test mode enable/disable and verify
  // by reading it back
  int32_t result = IPS_SetInt32Param( handle_ips,
                                      IPS_WRITE_COUNTER_TEST_MODE,
                                      (int32_t) enable);
  if (IPS_FAILED(result)) return result;

  int counter_test_mode_enable(0);
  result = IPS_GetInt32Param( handle_ips,
                              IPS_READ_COUNTER_TEST_MODE,
                              &counter_test_mode_enable);
  if (IPS_FAILED(result)) return result;
  if (counter_test_mode_enable == (int32_t) enable)
    cout << "Successfully set counter test mode to :" 
         << counter_test_mode_enable << endl;
  return result;
}

int32_t GetRowColTicks( HANDLE_IPS_ACQ handle_ips,
                        int & row_ticks,
                        int & col_ticks)
{
  int32_t result = IPS_GetInt32Param(handle_ips,
                                        IPS_GET_COL_TICKS,
                                        &col_ticks);
  if (IPS_FAILED(result)) return result;
  result = IPS_GetInt32Param( handle_ips,
                              IPS_GET_ROW_TICKS,
                              &row_ticks);
  if (IPS_FAILED(result)) return result;
  return result;
}

int32_t IntegrationTimeWriteExample(HANDLE_IPS_ACQ handle_ips,
                                    int row_ticks,
                                    int col_ticks)
{
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

int32_t IntegrationTimeCalculateExample(HANDLE_IPS_ACQ handle_ips)
{
  // Verify the integration time matches the the calculated integration time
  // using integration row and col ticks
  double integration_time_readout(0);
  int32_t result = IPS_GetDoubleParam(handle_ips,
                                      IPS_READ_INTEGRATION_TIME,
                                      &integration_time_readout);
  if (IPS_FAILED(result)) return result;

  int integration_col_ticks(0);
  int integration_row_tick(0);
  result = IPS_GetInt32Param( handle_ips,
                              IPS_GET_INTEGRATION_COL_TICKS,
                              &integration_col_ticks);
  if (IPS_FAILED(result)) return result;
  result = IPS_GetInt32Param( handle_ips,
                              IPS_GET_INTEGRATION_ROW_TICKS,
                              &integration_row_tick);

  double tm_seconds(0);
  result = IPS_CalculateIntegrationTime(handle_ips,
                                        integration_row_tick,
                                        integration_col_ticks,
                                        &tm_seconds);
  if (IPS_FAILED(result)) return result;
  if (integration_time_readout == tm_seconds)
    cout << "Integration time from camera matches calculated integration time" 
         << endl;
  return result;
}

//get the string indicating the first available capture source (source 0)
uint32_t GetFirstAVailableCaptureSource(string & first_capture_source){
	first_capture_source.clear();												//empty the string

	uint32_t num_capture_sources = 0;											//store the number of sources
	int32_t result = IPS_GetCaptureSourceCount( IPS_DEVICE_TYPE_CAMERALINK,
	                                          &num_capture_sources);		//get the number of sources
	if (IPS_FAILED(result)){													//if the API failed
	cout << "Failed to get capture source due to an error.  Error code = " 
	      << result << endl;												//display an error
	return result;															//return
	}

	if (num_capture_sources == 0){											//if there are no capture sources
	cout << "No capture sources were found." << result << endl;				//display an error
	return result;															//return the error
	}

	char capture_source[IPS_MAX_CAPTURE_SOURCE_BYTES];						//allocate space for strings
	char capture_source_descr[IPS_MAX_CAPTURE_SOURCE_BYTES];

	//get the name and description of the first capture source 0
	result = IPS_GetCaptureSource(
	                  IPS_DEVICE_TYPE_CAMERALINK,
	                  0,
	                  capture_source, sizeof(capture_source),
	                  capture_source_descr, sizeof(capture_source_descr));
	if (IPS_FAILED(result)){												//if the API call failed
		cout << "Failed to get capture source.  Error code = " << result << endl;	//display an error
		return result;														//return the result
	}

	first_capture_source = capture_source;							//save the character array as a C++ string
	return IPS_SUCCESS;												//the function returns successfully
}

int main(int argc, char* argv[]){
	ListCaptureSourcesExample();				//list available capture sources
	InitDeinitExample();						//turns the first source on and then off again

	A3200Handle handle = NULL;					//create a handle for the stage API
	FILE * file = NULL;							//create a file handle
	int grabs = 500;								//number of frames grabbed
	int Fra_Number = 1;						//number of images grabbed for a single fram
	DOUBLE result_stage;
	char tmp[LINE_SIZE], command[LINE_SIZE * NUM_LINES];
	int i;

	//get the name of the first avaiable capture source
	string capture_source;
	int32_t result = GetFirstAVailableCaptureSource(capture_source);
	if (IPS_FAILED(result)){
		cout << "Failed to get capture source.  Error code = " << result << endl;
		return result;
	}

	//initialize the first capture source (from the name above)
	HANDLE_IPS_ACQ handle_ips = NULL;			//declare a handle to the camera
	result = IPS_InitAcq(CAM_ID_SBF161,  		//Configure the SBF 161 camera
	                    capture_source.c_str(),	//name collected from the above function
	                    DEFAULT_CONFIG,
	                    "$(IPS_SDK_DATA_DIR)\\license.lcx",
	                    &handle_ips);

	// Display the camera and frame grabber diagnostic data
	vector<char> diag_buffer(IPS_MAX_DIAGNOSTIC_STRING_BYTES);		//allocate a character array
	uint32_t diag_buffer_size(0);									//initialize the buffer size to zero (0)
	//get the camera diagnostics - fill diag_buffer with a string describing the camera status								
	result = IPS_GetCameraDiagnostics(handle_ips,
	                                diag_buffer.data(),
	                                (uint32_t) diag_buffer.size(),
	                                &diag_buffer_size);
	if (IPS_FAILED(result)) return result;							//if the API call fails, return
	if (strlen(diag_buffer.data()) == diag_buffer_size-1)			//if the diagnostic string exists
		cout << "Camera diagnostics : " << string(diag_buffer.data()) << endl;	//print it

	//get the frame grabber diagnostics
	result = IPS_GetFrameGrabberDiagnostics( handle_ips,
	                                          diag_buffer.data(),
	                                          (uint32_t) diag_buffer.size(),
	                                          &diag_buffer_size);
	if (IPS_FAILED(result)) return result;							//if the API call fails, return
	if (strlen(diag_buffer.data()) == diag_buffer_size-1)			//if the diagnostic string exists
		cout << "Frame grabber diagnostics : " << string(diag_buffer.data()) << endl;	//print it

	//connect to the A3200 Aerotech stage controller
	printf("Connecting to A3200. Initializing if necessary.\n");
	if(!A3200Connect(&handle)){										//attempt to connect to the controller
		PrintError();
		goto cleanup;
	}																//if there is an error, return

	printf("Enabling Axis.\n");										//enable the axis
	if(!A3200MotionEnable(handle, TASKID_01, AXISMASK_00)){
		PrintError(); 
		goto cleanup;
	}

	//perform an imaging pass across the sample
	for (int i = 0; i < grabs; i++){
		A3200CommandExecute(handle, TASKID_01, "LINEAR Z0.005", &result_stage);	//move the stage
		//A3200CommandExecute(handle, TASKID_01, "MOVEDELAY Z, 1000", &result_stage);	//wait
		result = CreateDisplayImageExample(handle_ips, i, Fra_Number);				//capture images
		if (IPS_FAILED(result)){													//if the capture didn't work
			cout << "CreateDisplayImageExample failed.  Error code = " << result << endl;	//display an error
			 return result;															//return
		 }

		cout << (float)(i + 1) / (float)grabs * 100 <<" %." << endl;		//display the number of images

		//A3200CommandExecute(handle, TASKID_01, "MOVEDELAY Z, 1000", &result_stage);	//wait again
	}
	A3200CommandExecute(handle, TASKID_01, "LINEAR Z-2.5", &result_stage);  //move stage back to origin

	//clean up camera handle
	if (handle_ips)
		result = IPS_DeinitAcq(handle_ips);

	return result;

	cleanup:
	if(NULL != file){
		fclose(file);
	}
	if(NULL != handle) {
		printf("Done.\n");
		if(!A3200ProgramStop(handle, TASKID_01)) { PrintError(); }
		if(!A3200MotionDisable(handle, TASKID_01, AXISMASK_00)) { PrintError(); }
		if(!A3200Disconnect(handle)) { PrintError(); }
	}

	#ifdef _DEBUG
	printf("Press ENTER to exit...\n");
	getchar();
	#endif
	return 0;
}

void PrintError() {
	CHAR data[1024];
	A3200GetLastErrorString(data, 1024);
	printf("Error : %s\n", data);
}
