/// \file A3200Plc.h
/// \brief Contains the functions to access PLC functionality.
///
/// Copyright (c) Aerotech, Inc. 2010-2016.
///

#ifndef __A3200_PLC_H__
#define __A3200_PLC_H__

#include "A3200CommonTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/// \brief The enumeration containing possible PLC shared tag types.
typedef enum PLC_SHARED_TAG_TYPE
{	
	/// \brief Represents a bool
	PLC_SHARED_TAG_TYPE_Bool = 17,
	/// \brief Represents a byte
	PLC_SHARED_TAG_TYPE_Byte = 15,
	/// \brief Represents a word
	PLC_SHARED_TAG_TYPE_Word = 10,
	/// \brief Represents a dword
	PLC_SHARED_TAG_TYPE_Dword = 5,
	/// \brief Represents a sint
	PLC_SHARED_TAG_TYPE_Sint = 12,
	/// \brief Represents a int
	PLC_SHARED_TAG_TYPE_Int = 7,
	/// \brief Represents a dint
	PLC_SHARED_TAG_TYPE_Dint = 2,
	/// \brief Represents a usint
	PLC_SHARED_TAG_TYPE_Usint = 14,
	/// \brief Represents a uint
	PLC_SHARED_TAG_TYPE_Uint = 9,
	/// \brief Represents a udint
	PLC_SHARED_TAG_TYPE_Udint = 4,
	/// \brief Represents a real
	PLC_SHARED_TAG_TYPE_Real = 21,
	/// \brief Represents a lreal
	PLC_SHARED_TAG_TYPE_Lreal = 19
} PLC_SHARED_TAG_TYPE;

/// \defgroup plc PLC Functions
/// @{

/// \brief Gets the value of a PLC shared tag
///
/// This function will get the value of the PLC shared tag with the specified name.
///
/// \param[in] handle The handle to the A3200
/// \param[in] sharedTagName The name of the PLC shared tag
/// \param[out] valueOut The value of the specified PLC shared tag
/// \return TRUE on success, FALSE if an error occurred. Call A3200GetLastError() for more information.
BOOL DLLENTRYDECLARATION A3200PlcGetSharedTag(A3200Handle handle, LPCSTR sharedTagName, DOUBLE* valueOut);

/// \brief Sets the value of a PLC shared tag
///
/// This function will set the value of the PLC shared tag with the specified name.
///
/// \param[in] handle The handle to the A3200
/// \param[in] sharedTagName The name of the PLC shared tag
/// \param[in] value The new value to set the specified PLC shared tag
/// \return TRUE on success, FALSE if an error occurred. Call A3200GetLastError() for more information.
BOOL DLLENTRYDECLARATION A3200PlcSetSharedTag(A3200Handle handle, LPCSTR sharedTagName, DOUBLE value);

/// \brief Gets the value of a PLC shared tag by byte offset
///
/// This function will get the value of a PLC shared tag with the specified byte offset and type.
///
/// \param[in] handle The handle to the A3200
/// \param[in] byteOffset The byte offset of the PLC shared tag
/// \param[in] type The type of the PLC shared tag
/// \param[out] valueOut The value of the specified PLC shared tag
/// \return TRUE on success, FALSE if an error occurred. Call A3200GetLastError() for more information.
BOOL DLLENTRYDECLARATION A3200PlcGetMemory(A3200Handle handle, DWORD byteOffset, PLC_SHARED_TAG_TYPE type, DOUBLE* valueOut);

/// \brief Sets the value of a PLC shared tag by byte offset
///
/// This function will set the value of a PLC shared tag with the specified byte offset and type.
///
/// \param[in] handle The handle to the A3200
/// \param[in] byteOffset The byte offset of the PLC shared tag
/// \param[in] type The type of the PLC shared tag
/// \param[in] value The new value to set the specified PLC shared tag
/// \return TRUE on success, FALSE if an error occurred. Call A3200GetLastError() for more information.
BOOL DLLENTRYDECLARATION A3200PlcSetMemory(A3200Handle handle, DWORD byteOffset, PLC_SHARED_TAG_TYPE type, DOUBLE value);

/// @}

#ifdef __cplusplus
}
#endif

#endif __A3200_PLC_H__