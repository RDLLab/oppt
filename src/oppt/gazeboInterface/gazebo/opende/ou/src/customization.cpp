/*************************************************************************
 *                                                                       *
 * ODER's Utilities Library. Copyright (C) 2008 Oleh Derevenko.          *
 * All rights reserved.  e-mail: odar@eleks.com (change all "a" to "e")  *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 3 of the License, or (at    *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE-LESSER.TXT. Since LGPL is the extension of GPL     *
 *       the text of GNU General Public License is also provided for     *
 *       your information in file LICENSE.TXT.                           *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *   (3) The zlib/libpng license that is included with this library in   *
 *       the file LICENSE-ZLIB.TXT                                       *
 *                                                                       *
 * This library is distributed WITHOUT ANY WARRANTY, including implied   *
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.      *
 * See the files LICENSE.TXT and LICENSE-LESSER.TXT or LICENSE-BSD.TXT   *
 * or LICENSE-ZLIB.TXT for more details.                                 *
 *                                                                       *
 *************************************************************************/

#include <ou/customization.h>


BEGIN_NAMESPACE_OU()


#if !defined(__FILE__)

// Definition of __FILE__ constant for the case if compiler does not support the macro
/*extern*/ const char *const __FILE__ = "<filename unavailable>";


#endif // #if !defined(__FILE__)


#if !defined(__LINE__)

// Definition of __LINE__ constant for the case if compiler does not support the macro
extern const unsigned int __LINE__ = 0;


#endif // #if !defined(__LINE__)


//////////////////////////////////////////////////////////////////////////

/*extern*/ CAssertionFailedProcedure CAssertionCheckCustomization::g_fnAssertFailureHandler = NULL;


//////////////////////////////////////////////////////////////////////////

/*extern*/ CMemoryAllocationProcedure CMemoryManagerCustomization::g_fnMemoryAllocationProcedure = NULL;
/*extern*/ CMemoryReallocationProcedure CMemoryManagerCustomization::g_fnMemoryReallocationProcedure = NULL;
/*extern*/ CMemoryDeallocationProcedure CMemoryManagerCustomization::g_fnMemoryDeallocationProcedure = NULL;


END_NAMESPACE_OU()

