#ifndef SBFGRAB_H
#define SBFGRAB_H

#include <string>
#include "ips.h"

#ifdef sbfem_EXPORTS
#define IPS_API __declspec(dllexport)
#else
#define IPS_API __declspec(dllimport)
#endif

namespace SBF {

/**
 * Wrapper Class for the SBF::CGrab class
 *
 * This class internally uses the TS&I IPS API.
 *
 * The SBF::CGrab class contains functions that can be used to grab image data to 
 * a user supplied buffer
 */
class IPS_API CGrab 
{
public:

  /**
   * Gets the unique instance of the CGrab class.
   * 
   * CGrab is a singleton class, so that only one instance of
   * it can exist at one time.
   *
   * You should not use the delete operator on the pointer returned
   * by Instance(), but instead call Destroy() when you are
   * completely finished using the SBF SDK functions.
   * 
   * @return A pointer to the unique instance of the CGrab class.
  *  @throws CException if no communication between the camera  head and computer/frame grabber serial port is detected.
  */
  static CGrab* Instance();

  /**
   * Deletes the unique instance of the CGrab class.
   * 
   * Deletes the singleton object returned by the Instance()
   * function and performs additional cleanup.  Destroy()
   * should only be called once, after you are completely done
   * using the SBF SDK classes, because the Instance() function
   * must perform extensive initialization if it is called again.
   */
  static void Destroy();

  /**
   * Starts a grab into RAM.
   *
   * This function returns immediately after starting the frame acquistion thread
   * Use Wait() to wait for a given frame.  Call Stop() to stop a grab and free up
   * resources.
   *
   * The buffer pointed to by p_img_buffer can be smaller than is necessary to hold
   * iFramesToGrab frames of data.  In this case, the p_img_buffer buffer will be filled
   * in ring fashion.  p_img_buffer should be large enough that you can process the data 
   * before it is overwritten by fresh data.
   *
   * Every call to Start() <B>must</B> be paired with a call to Stop().
   * @param p_img_buffer a pointer to a buffer that has been allocated by the caller
   * @param buffer_length the number of unsigned short elements allocated in p_img_buffer
   * infinite grab. In this case, RAM is filled in ring-buffer fashion.
   * @param iFramesToGrab the number of frames to grab.  Set iFramesToGrab to 0 for an 
   * infinite grab. In this case, RAM is filled in ring-buffer fashion.
   * @return true if successful, otherwise false.
   * @throws CException
   */
  bool Start(unsigned short * p_img_buffer, unsigned long buffer_length, unsigned int iFramesToGrab = 0);

  /**
   * Wait for the next image and return a pointer to it
   *
   * This function is to be used during a grab that was started using Start().  It returns 
   * when the frame is grabbed or when the timeout period has been exceeded.
   *
   * @param ppUShort The address of a pointer.  It may be 0.  If it is nonzero, then when 
   * the function returns it will hold a pointer to a frame.  This may not be the requested frame,
   * but it will correspond to the return value.  This pointer will be point somewhere 
   * within the buffer passed to Start()
   * @param nFrameToWaitFor The number of the frame to wait for.  Set nFrameToWaitFor to -1 to wait 
   * for the next available frame.
   * @param nMilliSecWait the time in milliseconds to wait before returning if the frame hasn't been 
   * grabbed.  Set nMilliSecWait to -1 for an infinite wait.
   * @return a frame number.  If the requested frame is already in memory when this function is called, 
   * then the returned frame number will be that of the most recently grabbed frame, which may not be 
   * the requested frame.  If the requested frame is not in memory, then it will wait for the  requested 
   * frame and return its frame number.  If it cannot wait for the requested frame, either because it 
   * times out or because the frame has been overwritten, it returns -1.
   * @throws :  CException
   */
  int Wait(unsigned short** ppUShort, int nFrameToWaitFor = -1, int nMilliSecWait = -1);

  /**
   * Stop grabbing images and return a pointer to the newest image
   *
   * This function is to be used to end a grab that was started using Start().  It works the same
   * as Wait() except that it stops the grab and frees up resources after it waits for the given frame.
   * It returns when the frame is grabbed or when the timeout period has been exceeded. 
   *
   * @param  ppUShort The address of a pointer.  It may be 0.  If it is nonzero, then when the function 
   * returns it will hold a pointer to a frame.  This may not be the requested frame, but it will correspond
   * to the return value.   This pointer will be point somewhere within the buffer passed to Start()
   * @param nFrameToWaitFor  The number of the frame to wait for.  Set nFrameToWaitFor to -1 to 
   * wait for the next available frame.
   * @param nMilliSecWait  the time in milliseconds to wait before returning if the frame hasn't been grabbed.
   * Set nMilliSecWait to -1 for an infinite wait.
   * @return  a frame number.  If the requested frame is already in memory when this function is called,
   * then the returned frame number will be that of the most recently grabbed frame, which may not be the 
   * requested frame.  If the requested frame is not in memory, then it will wait for the  requested frame 
   * and return its frame number.  If it cannot wait for the requested frame, either because it times out or 
   * because the frame has been overwritten, it returns -1.
   * @throw :  CException
   */
  int Stop(unsigned short** ppUShort, int nFrameToWaitFor = -1, int nMilliSecWait = -1);

  /**
   * Returns the frame width and height currently being grabbed
   *
   * @return pair<width,height> currently being grabbed
   */
  std::pair<int, int> GrabSizeGet() const;

  /**
   * Returns current frame grabber configuration formatted as an ASCII string.
   *
   * @return a pointer to a null terminated ASCII containing diagnostic information
   */
  const char * Diagnostics();

  /**
   * Returns the underlying IPS SDK handle
   * @return void *  the IPS SDK handle
   */
  void * handle_ips() {return handle_ips_;}

private:
  CGrab();
  virtual ~CGrab();
  static void ThrowErrorCodeException(const std::string & error_descr,
                                      int error_code);
  void * handle_ips_;
  static CGrab * p_instance_;
  char diagnostics_str_buffer_[IPS_MAX_DIAGNOSTIC_STRING_BYTES];
};

}

#endif
