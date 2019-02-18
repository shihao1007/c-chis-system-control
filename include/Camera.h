#ifndef SBFCAMERA_H
#define SBFCAMERA_H

#include <string>
#include "ips.h"

#ifdef sbfem_EXPORTS
#define IPS_API __declspec(dllexport)
#else
#define IPS_API __declspec(dllimport)
#endif

namespace SBF {

/** 
 * Emumulates the SBF::CCamera class.
 *
 * This class internally uses the TS&I IPS SDK.
 *
 * The SBF::CCamera class contains functions that can be used to
 * configure the camera.  They can control the frame rate, integration
 * time, Fpa window size, and jam sync state.  The read and write
 * functions write to and read from the camera over the serial
 * communications port.  The get functions read data from RAM that were
 * previously read from the camera.
 */
class IPS_API CCamera 
{
public:
  enum JamSyncType
  {
    NONE, /// The camera will be free-running.
    TTL_OUT, /// The camera will be free-running, but it will output a TTL pulse at the start of every frame.*/
    TTL_IN,  /// The camera is in jam sync mode.  It will grab a frame each time it receives a TTL pulse and will not grab any frames otherwise.
    RS422_IN /// The camera is in jam sync mode. It will grab a frame each time it receives a signal through the RS422 port and will not grab any frames otherwise.
  };

  enum FpaType
  {
    NOT_SPECIFIED = 0, /// FPA not specified
    FPA_SBFJ108 = 1,  /// SBFJ108 64x64 LWIR HgCdTe
    FPA_SBF119 = 2, /// SBF119 640x512 InSb Windowing
    FPA_SBF125 = 3, /// SBF125 320x256 InSb Windowing
    FPA_SBF134 = 4, /// SBF134 256x256 InSb SharpIntegration, 3Windows
    FPA_SBF135 = 5, /// SBF135 256x256 InSb RollingModeIntegration, 1Output
    FPA_SBF136 = 6, /// SBF136 128x128 InSb 32 Output HiSpeed Windowing
    FPA_SBF141 = 7, /// SBF141 640x512 InSb Windowing Quiet
    FPA_SBF161 = 8, /// SBF161 128x128 LWIR HgCdTe Windowing
    FPA_EGL2333  = 9, /// EGL2333 256x256 LWIR HgCdTe
    FPA_SBF162 = 10, /// SBF162 640x512 InSb VoltageMode LowPower Windowing
    FPA_SBF151 = 11, /// SBF151 320x256 InSb VoltageMode Windowing
    FPA_SBF167 = 12, /// SBF167 320x256 InSb VoltageMode Windowing Quiet
    FPA_SBF168 = 13, /// SBF168 320x256 InSb Digital FPA
    FPA_DRS640 = 14, /// DRS640 640x480 HgCdTe LWIR FPA
    FPA_SBF177 = 15, /// SBF177 640x512 InSb Windowing
    FPA_SBF180 = 16, /// SBF180 320x256 30µm VoltageMode LowPower Windowing
    FPA_SBF178 = 17, /// SBF178 640x512 20µm WindowingLowPower
    FPA_SBF184 = 18, /// SBF184 1024x1024 19.5µm Windowing
    FPA_SBF113 = 19, /// SBF113 128x128 InSb
    FPA_SBF191 = 20, /// SBF191 640x512 14-bit Digital, 20µm Windowing
    FPA_SBF193 = 21, /// SBF193 640x512 24µm Windowing
    FPA_SBF194 = 22, /// SBF194 320x256 30µm Windowing
    FPA_SBF196 = 23, /// SBF196 1024x1024 14-bit Digital, 25µm Windowing
    FPA_SBF191DASH = 24, /// SBF191 640x512 14-bit Digital, 20µm Windowing
    FPA_SBF200 = 25, /// SBF200 320x256 14-bit Digital, 30 µm Windowing
    FPA_SBF199 = 26, /// SBF199 640x512 14-bit Digital, 24 µm Windowing
    FPA_SBF204 = 27, /// SBF204 1280x1024 14-bit Digital, 12 µm CTIA, Windowing
    FPA_SBF207  = 28, /// SBF207 1280x1024 14-bit Digital, 12 µm Windowing
    FPA_SBF208 = 29, /// SBF208 1024x1024 14-bit Digital, 25 µm Windowing
    FPA_SBF209 = 30 /// SBF209 640x512 14-bit Digital, 20 µm Windowing
  };

  /**
   * Gets the unique instance of the CCamera class.
   * 
   * CCamera is a singleton class, so that only one instance of
   * it can exist at one time.
   *
   * You should not use the delete operator on the pointer returned
   * by Instance(), but instead call Destroy() when you are
   * completely finished using the SBF SDK functions.
   * 
   * @return A pointer to the unique instance of the CCamera class.
  *  @throws CException if no communication between the camera  head and computer/frame grabber serial port is detected.
  */
  static CCamera* Instance();

  /**
   * Deletes the unique instance of the CCamera class.
   * 
   * Deletes the singleton object returned by the Instance()
   * function and performs additional cleanup.  Destroy()
   * should only be called once, after you are completely done
   * using the SBF SDK classes, because the Instance() function
   * must perform extensive initialization if it is called again.
   */
  static void Destroy();

  /** 
   * Gets the current frame rate.
   *
   * This function does not communicate with the camera head.
   *
   * @return The current frame rate in Hertz.
   */
  float FrameRateGet() const;

  /**
   * Gets the current integration time.
   *
   * Uses the number of integration and row ticks previously read
   * from the camera to calculate the integration time in
   * seconds.  This function does not communicate with the camera
   * head.
   *
   * @return The current integration time in seconds. 
   * @see IntegrationTimeWrite()
   */
  float IntegrationTimeGet() const;

  /**
   * Write an integration time to the camera head.
   *
   * The integration time is determined by the number of
   * integration row and column ticks stored on the camera head.
   * This function allows you to set the integration time by
   * choosing the number of integration row and column ticks
   * directly.
   *
   * @param iRowTicks the desired integration row ticks. 
   * @param iColTicks the desired integration column ticks.
   * @param iSuperframeIndex this value is ignored
   * @return true if the write succeeded, otherwise false.
   * @see IntegrationTimeGet()
   */
  bool IntegrationTimeWrite(  int iRowTicks, 
                              int iColTicks, 
                              int iSuperframeIndex = 0);

  /**
   * Calculates the integration time corresponding to
   * given values of the integration row and column ticks.
   *
   * This function does not change the camera settings, although
   * it may read information from the camera head.
   *
   * @param iRowTicks integration row ticks
   * @param iColTicks integration column ticks
   * @return the integration time in seconds.
   */
  float IntegrationTimeCalculate(int iRowTicks, int iColTicks);

  /**
   * Writes the camera head integration mode to Integrate While Read
   * 
   * The integration time mode for most SBF cameras should be in Integrate
   * While Read (Fast Frame Transfer) mode, rather than in Integrate Then
   * Read mode. This function has no effect on FPAs which don't support
   * Integrate While Read mode (such as SBF135 & DRS640 FPAs).
   *
   * @return true if the write succeeded, otherwise false.
   */
  bool IntegrateWhileReadModeWrite();

  /**
   * Sets the jam sync state of the camera head.
   *
   * The jam sync state determines how the camera is synchronized
   * with external signals.
   *
   * @param jam_sync_type indicates the desired synchronization state of the camera.
   * @return true if the write succeeded, otherwise false.
   */
  bool JamSyncWrite(JamSyncType jam_sync_type);

  /**
   * Gets the zero-based index of the first column on the
   * focalplane which will transmit data.
   *
   * This function does not communicate with the camera head.
   *
   * @return The first column of the focalplane window.
   * @see FpaColumnEndGet(), FpaRowStartGet(), FpaRowEndGet(), 
   */
  int FpaColumnStartGet() const;

  /**
   * Gets the zero-base index of the last column on the
   * focalplane which will transmit data.
   *
   * This function does not communicate with the camera head.
   *
   * @return The last column of the focalplane window.
   * @see FpaColumnStartGet(), FpaRowStartGet(), FpaRowEndGet(), 
   */
  int FpaColumnEndGet() const;

  /**
   * Gets the zero-base index of the first row on the
   * focalplane which will transmit data.
   *
   * This function does not communicate with the camera head.
   *
   * @return The first column of the focalplane window.
   * @see FpaRowEndGet(), FpaColStartGet(), FpaColEndGet(), 
   */
  int FpaRowStartGet() const;

  /**
   * Gets the zero-base index of the last row on the
   * focalplane which will transmit data.
   *
   * This function does not communicate with the camera head.
   *
   * @return The first column of the focalplane window.
   * @see FpaRowStartGet(), FpaColStartGet(), FpaColEndGet(), 
   */
  int FpaRowEndGet() const;

  /**
   * Gets the number of timing column ticks needed to read one row of pixel data
   *
   * This number is related to the number of FPA columns and can be useful for diagnostics.
   *
   * This function does not communicate with the camera head.
   *
   * @return The number of timing ticks to read one row of pixel data
   * @see NumRowTicksGet()
   */
  int NumColTicksGet() const;

  /**
   * Gets the number of timing row ticks needed to read one frame of pixel data
   *
   * This number is related to the number of FPA rows and can be useful for diagnostics.
   *
   * This function does not communicate with the camera head.
   *
   * @return The number of timing ticks to read one frame of pixel data
   * @see NumRowTicksGet()
   */
  int NumRowTicksGet() const;

  /**
   * Returns current camera configuration and state formatted as an ASCII string.
   *
   * @return a pointer to a null terminated ASCII containing diagnostic information
   */
  const char * Diagnostics();

  /**
   * Returns the FPA type of the current camera
   *
   * This function does not communicate with the camera head
   *
   * @return the FpaType of the current camera
   */
  FpaType FpaTypeGet() const;

  /**
   * Gets the serial number stored in the camera head.
   *
   * The serial number returned should agree with the 6-digit
   * number marked on the dewar (e.g. 00-4398 would return 4398,
   * while 99-0345 would return 990345).
   *
   * This function does not communicate with the camera head
   *
   * @return the serial number stored in the camera head.
   */
  int SerialNumGet() const;

  /**
   * Sets the FrameCounterEnabled bit on the camera head.
   *
   * When the frame counter is enabled, the first pixel of each
   * frame will not correspond to an intensity, but to a counter
   * that is incremented each frame (modulo 16384).  This can be
   * used to make sure no frames are being dropped.
   *
   * @param bFrameCounter Set to true to enable the frame counter, false to disable it.
   * @return true if the write succeeded, otherwise false.
   */
  bool FrameCounterEnabledWrite(bool bFrameCounter);

  /**
   * Reads the FPA temperature from the camera head.
   *
   * @return The temperature of the FPA in Kelvin.
   */
  float TemperatureFpaRead() const;

 /**
  * Turns the electronics test mode on or off.
  *
  * The test mode outputs a counter that ramps repeatedly from 0
  * to 16383.  This is done without the help of the focalplane,
  * so it can be used to test the operation of the camera
  * electronics in isolation from the focalplane.
  *
  * @param bTestMode Set to true to turn test mode on, false to turn it off.
  */
  bool CounterTestModeWrite(bool bTestMode);

  /**
   * Gets the number of A/D converters used by the current focalplane.
   *
   * Usually 1, 2, 4, or 16.
   *
   * This function does not communicate with the camera head.
   *
   * @return number of A/D converters used by the current focalplane.
   */
  int NumOutputsGet() const;

  /**
   * Gets the FPA detector bias value
   *
   * Some FPAs or electronics may support only 8, 16, or 32 different values.
   * The FPA detector bias is pre-set to give optimum performance and it should not 
   * generally need to be changed for InSb detectors.
   *
   * This function does not communicate with the camera head.
   *
   * @return FPA detector bias value from 0 to 255 or -1 if bias not available.
   */
  int FpaDetectorBiasGet() const;

  /**
   * Writes the FPS detector bias to the camera head
   *
   * This function does not communicate with the camera head.
   *
   * @param iValue the desired value of the FPA detector bias. Must be between 0 and 255, inclusive. Otherwise, it is truncated to fit within this range.
   * @return FPA detector bias value
   * @note Some FPAs or electronics may support only 8, 16, or 32 different values.
   * @attention We recommend that you not  use this function.  The FPA detector bias is pre-set to give optimum performance and 
   * it should not generally need to be changed for InSb detectors.
   */
  int FpaDetectorBiasWrite(int iValue);


  /**
   * Returns the underlying IPS SDK handle
   * @return void *  the IPS SDK handle
   */
  void * handle_ips() {return handle_ips_;}

private:
  CCamera();
  virtual ~CCamera();
  void * handle_ips_;
  static CCamera * p_instance_;
  char diagnostics_str_buffer_[IPS_MAX_DIAGNOSTIC_STRING_BYTES];
};
}

#endif