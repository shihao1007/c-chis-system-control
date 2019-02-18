#ifndef IPS_H
#define IPS_H
#include <stdint.h>

#ifdef ips_EXPORTS
  #define IPS_API __declspec(dllexport)
#else
  #if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
    #define IPS_API __declspec(dllimport)
  #else
    #undef IPS_API
  #endif
#endif

#ifdef __cplusplus
extern "C"
{
#endif

  #define IPS_DEVICE_TYPE_CAMERALINK  1
  #define IPS_DEVICE_TYPE_GIGE        2
  
  #define IPS_MAX_CAPTURE_SOURCE_BYTES 1024
  #define IPS_MAX_DIAGNOSTIC_STRING_BYTES 5120
  #define IPS_TIMEOUT_WAIT_FOREVER -1
  #define IPS_GET_NEXT_FRAME 0

  #define IPS_SUCCEEDED(result) (result >= 0)
  #define IPS_FAILED(result) (result < 0)

  #define IPS_SUCCESS 0

  // Warnings
  #define IPS_WARNING_TIMEOUT 1
  #define IPS_WARNING_FRAME_GRABBING_IN_PROGRESS 2
  #define IPS_WARNING_FRAME_GRABBING_ALREADY_PAUSED 3
  #define IPS_WARNING_FRAME_GRABBING_FINISHED 4
  #define IPS_WARNING_FRAME_GRABBING_NOT_STARTED 5
  #define IPS_WARNING_FRAME_GRABBING_FRAME_OVERWRITTEN 6

  // Errors
  #define IPS_ERROR_LIBRARY_NOT_LOADED -1
  #define IPS_ERROR_SOURCE_NOT_FOUND -2 
  #define IPS_ERROR_SOURCE_NOT_SUPPORTED -3 
  #define IPS_ERROR_SOURCE_INIT_FAILED -4 
  #define IPS_ERROR_SOURCE_UNKNOWN_DATA_FORMAT -5
  #define IPS_ERROR_NOT_INTIALIZED -6
  #define IPS_ERROR_FRAME_GRABBING_FAILED -7 
  #define IPS_ERROR_FRAME_GRABBING_USER_BUFFER_INVALID -8
  #define IPS_ERROR_BUFFER_INSUFFICIENT -9
  #define IPS_ERROR_FRAME_GRABBING_NUM_FRAMES_INVALID -10
  #define IPS_ERROR_FRAME_GRABBING_DESIRED_FRAME_NUM_INVALID -11
  #define IPS_ERROR_FRAME_GRABBING_NOT_STARTED -12
  #define IPS_ERROR_FRAME_GRABBING_FRAME_OVERWRITTEN -13
  #define IPS_ERROR_PARAM_SET_FAILED -14
  #define IPS_ERROR_PARAM_GET_FAILED -15
  #define IPS_ERROR_PARAM_UNKNOWN -16
  #define IPS_ERROR_REQUIRED_PARAM_NOT_SET -17
  #define IPS_ERROR_CAMERA_INIT_FAILED -18
  #define IPS_ERROR_CAMERA_NOT_INITIALIZED -19
  #define IPS_ERROR_INVALID_POINTER -20
  #define IPS_ERROR_XML_PARSE_FAILED -21
  #define IPS_ERROR_XML_CONFIG_FILE_NOT_FOUND -22
  #define IPS_ERROR_FRAME_GRABBING_IN_PROGRESS -23
  #define IPS_ERROR_FRAME_ACQ_OFFLINE -24
  #define IPS_ERROR_FRAME_ACQ_RESET_REQUIRED -25

  #define IPS_ERROR_LIC_ENF_INTERNAL_ERROR  -120
  #define IPS_ERROR_LIC_ENF_HOSTID_MISMATCH -121
  #define IPS_ERROR_LIC_ENF_EXPIRED -122
  #define IPS_ERROR_LIC_ENF_CLOCK_TAMPERED -123
  #define IPS_ERROR_LIC_ENF_FEATURE_MISMATCH -124
  #define IPS_ERROR_LIC_ENF_WRONG_VERSION -125
  #define IPS_ERROR_LIC_ENF_ACCESS_DENIED -126
  #define IPS_ERROR_LIC_ENF_INVALID -127
  #define IPS_ERROR_LIC_MISSING -128
  #define IPS_ERROR_LIC_NOT_LICENSED -129
  
  #define IPS_ERROR_EXCEPTION -1000

  #define IPS_ERROR_INVALID_HANDLE -50
  
  // Configuration and control parameter identifiers
  #define IPS_GET_FRAME_RATE 100
  #define IPS_GET_INTEGRATION_TIME 101
  #define IPS_WRITE_INTEGRATION_ROW_TICKS 102
  #define IPS_WRITE_INTEGRATION_COL_TICKS 103
  #define IPS_WRITE_INTEGRATION_TIME 104
  #define IPS_READ_INTEGRATION_TIME 105
  #define IPS_WRITE_JAM_SYNC 106
  #define IPS_GET_COL_START 107
  #define IPS_GET_COL_END 108
  #define IPS_GET_ROW_START 109
  #define IPS_GET_ROW_END 110
  #define IPS_GET_COL_TICKS 111
  #define IPS_GET_ROW_TICKS 112
  #define IPS_GET_FPA_TYPE 113
  #define IPS_GET_SERIAL_NUM 114
  #define IPS_WRITE_FRAME_COUNTER_ENABLED 115
  #define IPS_READ_TEMP_FPA 116
  #define IPS_WRITE_COUNTER_TEST_MODE 117
  #define IPS_GET_NUM_OUTPUTS 118
  #define IPS_GET_FPA_DETECTOR_BIAS 119
  #define IPS_WRITE_FPA_DETECTOR_BIAS 120
  #define IPS_READ_FRAME_COUNTER_ENABLED 121
  #define IPS_READ_JAM_SYNC 122
  #define IPS_READ_COUNTER_TEST_MODE 123
  #define IPS_GET_INTEGRATION_ROW_TICKS 124
  #define IPS_GET_INTEGRATION_COL_TICKS 125
  #define IPS_SET_FRAME_RATE 126
  #define IPS_GET_GRAB_RATE 127
  #define IPS_SET_BIT_DEPTH 128
  #define IPS_GET_BIT_DEPTH 129
  #define IPS_SET_FRAME_NUM_ENABLED 130
  #define IPS_SET_FRAME_NUM_DISABLED 131
  #define IPS_WRITE_FAST_XFER_MODE 132
  #define IPS_GET_IMAGE_FORMAT 133
  #define IPS_SET_IMAGE_FORMAT 134
  #define IPS_SET_GRAB_WIDTH_MULTIPLIER 135
  #define IPS_SET_FRAME_PERIOD 136
  #define IPS_GET_FRAME_PERIOD 137
  #define IPS_WRITE_EXPOSURE 138
  #define IPS_GET_FRAME_ACQ_STATE 139
  #define IPS_GET_HACTIVE 140
  #define IPS_SET_HACTIVE 141
  #define IPS_GET_VACTIVE 142
  #define IPS_SET_VACTIVE 143
  #define IPS_GET_TAPS 144

  #define SAPERA_FRAME_GRABBER_TYPE "SAPERA"

  // Camera Identifiers
  #define  CAM_ID_NONE 0
  #define  CAM_ID_GENERIC 1
  #define  CAM_ID_SBF161 2
  #define  CAM_ID_DALSA_XRAY_DETECTOR 3

  typedef void * HANDLE_IPS_ACQ;
  typedef int BOOL_IPS;

  #define INFINITE_FRAMES UINT64_MAX

  // Jam Sync Modes
  #define IPS_JAM_SYNC_NONE 0 
  #define IPS_JAM_SYNC_TTL_OUT 1
  #define IPS_JAM_SYNC_TTL_IN 2
  #define IPS_JAM_SYNC_RS422_IN 3
 
  // Supported FPA types
  #define IPS_FPA_SBF161 8

  // Supported Image formats
  #define IPS_IMG_FORMAT_UNKNOWN -1 
  #define IPS_IMG_FORMAT_MONO8 0   
  #define IPS_IMG_FORMAT_MONO16 1 
  #define IPS_IMG_FORMAT_MONO24 2 
  #define IPS_IMG_FORMAT_MONO32 3 
  #define IPS_IMG_FORMAT_MONO64 4
  #define IPS_IMG_FORMAT_RGB5551 5
  #define IPS_IMG_FORMAT_RGB565 6
  #define IPS_IMG_FORMAT_RGB888 7
  #define IPS_IMG_FORMAT_RGBR888 8 
  #define IPS_IMG_FORMAT_RGB8888 9
  #define IPS_IMG_FORMAT_MONO10 10

  // Frame acquistion state flags
  // These are combined and returned when
  // calling GetIn32Param(IPS_GET_FRAME_ACQ_STATE,&state)
  #define IPS_ACQ_STATE_CAPTURING 1
  #define IPS_ACQ_STATE_OFFLINE 2
  #define IPS_ACQ_STATE_RESET_REQUIRED 3

  #define IPS_IS_CAPTURING(acq_state) (acq_state & IPS_ACQ_STATE_CAPTURING)
  #define IPS_IS_OFFLINE(acq_state) (acq_state & IPS_ACQ_STATE_OFFLINE)

  // Capture Source Discovery Functions
  IPS_API int32_t IPS_GetCaptureSourceCount(int32_t device_type, 
                                            uint32_t * num_capture_sources_found);
  IPS_API int32_t IPS_GetCaptureSource( int32_t device_type,
                                        uint32_t capture_source_index,
                                        char * p_capture_source,
                                        uint32_t capture_source_size_bytes,
                                        char * p_capture_source_descr,
                                        uint32_t capture_source_descr_bytes);

  // Capture Source Initialization and Cleanup Functions
  IPS_API int32_t IPS_InitAcq(uint32_t camera_id,
                              const char * p_capture_source,
                              const char * p_xml_config,
                              const char * p_license_file_path,
                              HANDLE_IPS_ACQ * p_handle);
  IPS_API int32_t IPS_DeinitAcq(const HANDLE_IPS_ACQ handle);

  // Camera Configuration Functions
  IPS_API int32_t IPS_SetInt32Param(const HANDLE_IPS_ACQ handle,
                                    uint32_t param_id,
                                    int32_t value);
  IPS_API int32_t IPS_SetDoubleParam( const HANDLE_IPS_ACQ handle,
                                      uint32_t param_id,
                                      double value);
  IPS_API int32_t IPS_GetInt32Param ( const HANDLE_IPS_ACQ handle,
                                      uint32_t param_id,
                                      int32_t * p_value);
  IPS_API int32_t IPS_GetDoubleParam( const HANDLE_IPS_ACQ handle,
                                      uint32_t param_id,
                                      double *  p_value);

  // Frame Acquisition Functions
  IPS_API int32_t IPS_StartContinuousGrabbing(const HANDLE_IPS_ACQ handle,
                                              uint8_t * p_ring_buffer,
                                              uint64_t  buffer_size_bytes);
  IPS_API int32_t IPS_StartGrabbing(const HANDLE_IPS_ACQ handle,
                                    uint64_t num_frames,
                                    uint16_t * p_buffer,
                                    uint64_t  buffer_size_bytes,
                                    BOOL_IPS  allow_wrap);
  IPS_API int32_t IPS_WaitFrame(const HANDLE_IPS_ACQ handle,
                                uint64_t desired_frame_number,
                                int32_t timeout_ms,
                                BOOL_IPS pause,
                                uint8_t ** pp_frame,
                                uint64_t * p_frame_number);
  IPS_API int32_t IPS_WaitFrame2( const HANDLE_IPS_ACQ handle,
                                  uint64_t desired_frame_number,
                                  int32_t timeout_ms,
                                  BOOL_IPS pause,
                                  uint8_t ** pp_desired_frame,
                                  uint64_t * p_desired_frame_number,
                                  uint8_t ** pp_most_recent_frame,
                                  uint64_t * p_most_recent_frame_number);
  IPS_API int32_t IPS_ResumeGrabbing(const HANDLE_IPS_ACQ handle);
  IPS_API int32_t IPS_StopGrabbing(const HANDLE_IPS_ACQ handle);
  IPS_API int32_t IPS_GetFrameSize( const HANDLE_IPS_ACQ handle,
                                    uint32_t * p_x_size,
                                    uint32_t * p_y_size);
  IPS_API int32_t IPS_GetFrameWindowOffsets(const HANDLE_IPS_ACQ handle,
                                            uint32_t * p_x_offset,
                                            uint32_t * p_y_offset);
  IPS_API int32_t IPS_SetFrameWindow( const HANDLE_IPS_ACQ handle,
                                      uint32_t x_offset,
                                      uint32_t y_offset,
                                      uint32_t x_size,
                                      uint32_t y_size);

  // Utility Functions
  IPS_API int32_t IPS_GetCameraDiagnostics( const HANDLE_IPS_ACQ handle,
                                            char * diagnostics_str_buffer,
                                            uint32_t buffer_size_in_bytes,
                                            uint32_t * p_buffer_size_out_bytes);
  IPS_API int32_t IPS_GetFrameGrabberDiagnostics( const HANDLE_IPS_ACQ handle,
                                                  char * diagnostics_str_buffer,
                                                  uint32_t buffer_size_in_bytes,
                                                  uint32_t * p_buffer_size_out_bytes);
  IPS_API int32_t IPS_CalculateIntegrationTime( const HANDLE_IPS_ACQ handle,
                                                int row_ticks,
                                                int col_ticks,
                                                double * p_tm_seconds);

#ifdef __cplusplus
}
#endif

#endif