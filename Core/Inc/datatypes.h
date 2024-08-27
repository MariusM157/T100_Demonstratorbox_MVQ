/** @file datatypes.h
 *
 *  @brief standard data types
 *
 *  Standardization of data types and simple global macros. Defines types with
 *  exact length for 8-bit controllers.
 *
 *  $Rev: 8437 $
 *
 *  $Author: dekestjo $
 *
 *  $Date: 2017-03-01 17:25:08 +0100 (Mi, 01. Mrz 2017) $
 *
 *  Copyright (C) 2016 ifm efector gmbh
 *
 *  @review Status: development\n
 *          Date: (not done yet)\n
 *          Reference to documentation: <a href="">Filename</a>\n
 */
/* $Id: datatypes.h 8437 2017-03-01 16:25:08Z dekestjo $ */

/* include guard *************************************************************/
#ifndef DATATYPES_H
#define DATATYPES_H


/* compiler switches *********************************************************/

/* includes ******************************************************************/
#include <stdint.h>
//#include "my_stdint.h"    /*local copy of <stdint.h> */
/* defines *******************************************************************/
    
/** @name General constants
 *  @{ */
#define TRUE    1
#define FALSE   0
#define HIGH    1
#define LOW     0
#define ON      1
#define OFF     0
/** @} */   
   
/** @name Zero values
    @{
*/
#define cNULL   ((char_t) 0 )
#define chNULL  ((int8_t) 0 )
#define byNULL  ((uint8_t) 0 )
#define iNULL   ((int16_t) 0 )
#define wNULL   ((uint16_t) 0 )
#define diNULL  ((int32_t) 0 )
#define dwNULL  ((uint32_t) 0 )
#define fNULL   ((float32_t) 0 )
#define dfNULL  ((float64_t) 0 )
#define vNULL   (void*) 0
/** @} */
#define chASCII_NULL    ((char_t)'\0')  /* ASCII for zero termination"\NUL"*/

/** @} */

/** @name Limits
 *  @verbatim
    - bool_t                          0 ... 1
    - char_t                          0 ... 255
    - int8_t                       -128 ... 127
    - uint8_t                         0 ... 255
    - int16_t                    -32768 ... 32767
    - uint16_t                        0 ... 65535
    - int32_t            -2.147.483.648 ... 2.147.483.647
    - uint32_t                        0 ... 4.294.967.295
    - float32_t            1.17549e-038 ... 3.40282e+038
    - float64_t            2.22507E-308 ... 1.79769E+308
    @endverbatim
 *  @{ */
#define BOOL_MAX 1
#define BOOL_MIN 0

#define CHAR_MAX 255
#define CHAR_MIN 0

/*is defined in <stdint.h> */
//#define INT8_MAX 0x7F
//#define INT8_MIN (-INT8_MAX - 1)

#ifdef __cplusplus
#define UINT8_MAX 0xFF
#define UINT8_MIN 0x00

#else  /*__cplusplus is not defined*/

/*is defined in <stdint.h> */
//#define UINT8_MAX 0xFF
#define UINT8_MIN 0x00
#endif


/*is defined in <stdint.h> */
//#define INT16_MAX 0x7FFF
//#define INT16_MIN (-INT16_MAX - 1)

//#define UINT16_MAX 0xFFFF
#define UINT16_MIN 0x0000

/*is defined in <stdint.h> */
//#define INT32_MAX 0x7FFFFFFFL
//#define INT32_MIN (-INT32_MAX - 1)


#ifdef __cplusplus

#define UINT32_MAX 0xFFFFFFFFUL
#define UINT32_MIN 0x00000000UL

#else  /*__cplusplus is not defined*/
/*is defined in <stdint.h> */
//#define UINT32_MAX 0xFFFFFFFFUL
#define UINT32_MIN 0x00000000UL
/** @} */
#endif

/* macros ********************************************************************/

/** @name Byte and word macros
    @{
*/
#define LO_UINT8(w)    ((uint8_t)(w))
#define HI_UINT8(w)    ((uint8_t)(((uint16_t)(w) >> 8) & 0xFF))

#define LO_UINT16(dw)  ((uint16_t)(uint32_t)(dw))
#define HI_UINT16(dw)  ((uint16_t)((((uint32_t)(dw)) >> 16) & 0xFFFF))

#define SWAP_UINT8(w)  ((uint16_t)((LO_UINT8(w) << 8) & 0xFF00) | HI_UINT8(w))
#define SWAP_UINT16(dw)((uint32_t)((LO_UINT16(dw) << 16) & 0xFFFF0000) | (uint16_t)HI_UINT16(dw))
/** @} */


/* types *********************************************************************/

/** @name Data types
 *  @{ */
/**     For unspecified types:  Name       No prefix for non-standard types     */
/**     For strings:            szXXName   XX type string terminated with zero  */
/**     For globals:            g_XXName   Global defined variable of type XX   */
/**     For function pointers:  pName                                           */
/**     For structures:         sName                                           */
/**     For unions:             uName                                           */
/**     For enums:              eName                                           */

/**                                        XX   == type of variable             */
/**                                        Name == name of variable             */

typedef void            *pvoid_t;       /**< Prefix: pvName */


typedef signed char     bool_t;         /**< Prefix: bName */

typedef unsigned char   char_t;         /**< Prefix: cName */
typedef unsigned char   *pchar_t;       /**< Prefix: pcName */

/*is defined in <stdint.h> */
//typedef signed char     int8_t;         /**< Prefix: chName */
typedef signed char     *pint8_t;       /**< Prefix: pchName */


//typedef unsigned char   uint8_t;        /**< Prefix: byName */
typedef unsigned char   *puint8_t;      /**< Prefix: pbyName */

/*is defined in <stdint.h> */
//typedef signed int      int16_t;        /**< Prefix: iName */ 
typedef signed int      *pint16_t;      /**< Prefix: piName */

/*is defined in <stdint.h> */
//typedef unsigned int    uint16_t;       /**< Prefix: wName */
typedef unsigned int    *puint16_t;     /**< Prefix: pwName */

/*is defined in <stdint.h> */
//typedef signed long     int32_t;        /**< Prefix: diName */
typedef signed long     *pint32_t;      /**< Prefix: pdiName */

/*is defined in <stdint.h> */
//typedef unsigned long   uint32_t;       /**< Prefix: dwName */
typedef unsigned long   *puint32_t;     /**< Prefix: pdwName */

typedef float           float32_t;      /**< Prefix: fName */
typedef float           *pfloat32_t;    /**< Prefix: pfName */

typedef double          float64_t;      /**< Prefix: dfName */
typedef double          *pfloat64_t;    /**< Prefix: pdfName */


/* variables *****************************************************************/


/* function prototypes *******************************************************/


/* include guard *************************************************************/
#endif /* DATATYPES_H */

/*************************** End of file *************************************/
