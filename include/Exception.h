#ifndef SBFEXCEPTION_H
#define SBFEXCEPTION_H

#ifdef sbfem_EXPORTS
#define IPSSDK_API __declspec(dllexport)
#else
#define IPSSDK_API __declspec(dllimport)
#endif

#include <string>
#include <stdint.h>

#if _MSC_VER
#pragma warning(disable: 4251)
#endif


/* Wrapper class for the SBF::CException class.
 * the CGrab and CCamera wrapper classes will throw exceptions
 * of this type when errors are returned by the TS&I IPS SDK
 */
namespace SBF {

class IPSSDK_API CException 
{
public:
	CException();
	CException(const char* szMsg, int32_t status_code = 0);

	//!	A destructor.
	virtual ~CException();

	const char* MsgGet();
  int32_t StatusCode(){return m_status_code;}

private:
  char msg_[1024];
  int32_t m_status_code;
};

}

#endif
