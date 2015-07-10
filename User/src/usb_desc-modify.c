/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : usb_desc.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Descriptors for Custom HID Demo
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"
#include "global_data.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



//const uint8_t CustomHID_ReportDescriptor[CUSTOMHID_SIZ_REPORT_DESC] = 
const uint8_t CustomHID_ReportDescriptor[] = 
{ 
	0x06, 0x00,0xff,                    // USAGE_PAGE
	0x09, 0x00,                   			 // USAGE
	0xa1, 0x01,                    			// COLLECTION (Application)
	0x85, 0xfd,                		 		//   REPORT_ID
	0x06, 0x00,0xff,                    // USAGE_PAGE
	0x09, 0x01,                    			// USAGE
	0x09, 0x02,                    			// USAGE
	0x09, 0x03,                    			// USAGE
	0x09, 0x04,                    			// USAGE
	0x09, 0x05,                    			// USAGE
	0x09, 0x06,                    			// USAGE
	0x15, 0x00, 							// 		LOGICAL_MINIMUM (0)
	0x26, 0xff,0x00, 					// 		LOGICAL_MAXIMUM (255)
	0x75, 0x08, 							// 		REPORT_SIZE (8)
//	0x95,  0x06, 							// 		REPORT_COUNT (6)
	0x95, 0x02, 							//		REPORT_COUNT (2)

	0x81, 0x02, 							 // 		INPUT (Data,Var,Abs)
	
	0x85, 0xfe,                		 		//   REPORT_ID
	0x06, 0x00,0xff,                   // USAGE_PAGE
	0x09, 0x01,                    			// USAGE
	0x09, 0x02,                    			// USAGE
	0x09, 0x03,                    			// USAGE
	0x09, 0x04,                    			// USAGE
	0x15, 0x00, 							// 		LOGICAL_MINIMUM (0)
	0x26, 0xff,0x00, 					// 		LOGICAL_MAXIMUM (255)
	0x75, 0x08, 							// 		REPORT_SIZE (8)
	0x95, 0x04, 							 // 		REPORT_COUNT (4)
	0xb1, 0x02,                         	//    FEATURE (Data,Var,Abs)
	0xc0, 										// 	END_COLLECTION	
	/*============mouse report ====================*/
	0x05, 0x01,                   					 // USAGE_PAGE (Generic Desktop)
	0x09, 0x02,                    					// USAGE (Mouse)
	0xa1, 0x01,                   					 // COLLECTION (Application)
	0x09, 0x01,                    					//   USAGE (Pointer)
	0xa1, 0x00, 									// 	COLLECTION (Physical)
	0x85, 0x01,                		 				//   REPORT_ID (mouse -> single touch)
	0x05, 0x09, 									 // 		USAGE_PAGE (Button)
	0x19, 0x01, 									 // 		USAGE_MINIMUM (Button 1)
	0x29, 0x03, 									 // 		USAGE_MAXIMUM (Button 3)
	0x15, 0x00, 									 // 		LOGICAL_MINIMUM (0)
	0x25, 0x01, 									 // 		LOGICAL_MAXIMUM (1)
	0x95, 0x03, 									 // 		REPORT_COUNT (3)
	0x75, 0x01, 									 // 		REPORT_SIZE (1)
	0x81, 0x02, 									 // 		INPUT (Data,Var,Abs)
	0x95, 0x01, 									 // 		REPORT_COUNT (1)
	0x75, 0x05, 									 // 		REPORT_SIZE (5)
	0x81, 0x03, 									 // 		INPUT (Cnst,Var,Abs)
	0x05, 0x01, 									 // 		USAGE_PAGE (Generic Desktop)
	0x09, 0x30, 									 // 		USAGE (X)
	0x09, 0x31, 									 // 		USAGE (Y)	
	0x15, 0x00, 									// 		LOGICAL_MINIMUM (-256)
	0x26, 0xff, 0x7f,            				 //       LOGICAL_MAXIMUM (32767)
	0x35, 0x00,                         			//       PHYSICAL_MINIMUM (0)
	0x46, 0xff, 0x7f,             			//       PHYSICAL_MAXIMUM (32767)
	0x75, 0x10, 									 // 		REPORT_SIZE (16)
	0x95, 0x02, 									 // 		REPORT_COUNT (2)
	0x81, 0x02, 									 // 		INPUT (Data,Var,Rel)	
	0x05, 0x0d,                         			// 		USAGE_PAGE (Digitizers)
	0x09, 0x33,                         			// 		USAGE (Touch)
	0x15, 0x0, 										 // 		LOGICAL_MINIMUM (0)
	0x25, 0xff, 									 // 		LOGICAL_MAXIMUM (255)
	0x35, 0x00,                         			//       PHYSICAL_MINIMUM (0)
	0x45, 0xff,            						 	//       PHYSICAL_MAXIMUM (255)
	0x75, 0x08, 									 // 		REPORT_SIZE (8)
	0x95, 0x01, 									 // 		REPORT_COUNT (12)
	0x81, 0x02, 									 // 		INPUT (Data,Var,Rel)
	0x05, 0x01,                    					// 		USAGE_PAGE (Generic Desktop)
	0x09, 0x38, 									 // 		USAGE (Wheel)	
	0x15, 0x81, 									 // 		LOGICAL_MINIMUM (-127)
	0x25, 0x7f, 									 // 		LOGICAL_MAXIMUM (127)
	0x95, 0x01, 									 // 		REPORT_COUNT (1)
	0x81, 0x06, 									 // 		INPUT (Data,Var,Rel)
	0xc0, 												 // 	END_COLLECTION
	0xc0, 												 // 	END_COLLECTION
	/*============ report ====================*/
	0x06,0x00,0xff, 									 // USAGE_PAGE
	0x09, 0x00, 									 // USAGE
	0xa1, 0x01, 									 // COLLECTION (Application)
	0x85, 0xfc, 									 // 	REPORT_ID
	0x15, 0x00, 										 // 		LOGICAL_MINIMUM (0)
	0x25, 0xff, 									 // 		LOGICAL_MAXIMUM (255)
	0x19, 0x01, 									 // USAGE
	0x29, 0x3f, 									 // USAGE	
	0x75, 0x08, 									 // 		REPORT_SIZE (8)
	0x95, 0x3f, 									 // 		REPORT_COUNT (63)
	0x81, 0x02, 									 // 		INPUT (Data,Var,Abs)
	0x19, 0x01, 									 // USAGE
	0x29, 0x3f, 									 // USAGE
	0x91, 0x02,                    					//   OUTPUT (Data,Var,Abs)
	0xc0, 												 // 	END_COLLECTION

	0x06,0x00,0xff, 							// USAGE_PAGE
	0x09, 0x00, 									 // USAGE
	0xa1, 0x01, 									 // COLLECTION (Application)
	0x85, 0xfb, 									 // 	REPORT_ID
	0x15, 0x00, 									 // 		LOGICAL_MINIMUM (0)
	0x25, 0xff, 									 // 		LOGICAL_MAXIMUM (255)
	0x19, 0x01, 									 // USAGE
	0x29, 0x3f, 									 // USAGE
	0x75, 0x08, 									 // 		REPORT_SIZE (8)
	0x95, 0x3f, 									 // 		REPORT_COUNT (63)
	0x81, 0x02, 									 // 		INPUT (Data,Var,Abs)	
	0x19, 0x01, 									 // USAGE
	0x29, 0x3f, 									 // USAGE
	0x91, 0x02,                    					//   OUTPUT (Data,Var,Abs)
	0xc0, 												 // 	END_COLLECTION
/*============Digitizer report ====================*/
	0x05, 0x0d,                         // USAGE_PAGE (Digitizers)
	0x09, 0x04,                         // USAGE (Touch Screen)
	0xa1, 0x01,                         // COLLECTION (Application)
	0x85, 0x02,                		 //   REPORT_ID (multi Touch)
	0x09, 0x22,                         //   USAGE (Finger)
	//=====================================1
	0xa1, 0x02,                         //     COLLECTION (Logical)
	0x09, 0x42,                         //       USAGE (Tip Switch)
	0x15, 0x00,                         //       LOGICAL_MINIMUM (0)
	0x25, 0x01,                         //       LOGICAL_MAXIMUM (1)
	0x75, 0x01,                         //       REPORT_SIZE (1)
	0x95, 0x01,                         //       REPORT_COUNT (1)
	0x81, 0x02,                         //       INPUT (Data,Var,Abs)
	0x09, 0x32,                         //       USAGE (In Range)
	0x81, 0x02,                         //       INPUT (Data,Var,Abs)
	0x95, 0x06,                         //       REPORT_COUNT (6)
	0x81, 0x03,                         //       INPUT (Data,Var,Abs)
	0x75, 0x08,                         //       REPORT_SIZE (8)
	0x09, 0x51,                         //       USAGE (Contact Identifier)
	0x95, 0x01,                         //       REPORT_COUNT (1)
	0x81, 0x02,                         //       INPUT (Data,Var,Abs)	
	0x05, 0x01,                         //       USAGE_PAGE (Generic Desk..
	0x15, 0x00,                         //       LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x7f,             //       LOGICAL_MAXIMUM (32767)
	0x75, 0x10,                         //       REPORT_SIZE (16)
	0x55, 0x00,                         //       UNIT_EXPONENT (0)
	0x65, 0x00,                         //       UNIT (None)
	0x09, 0x30,                         //       USAGE (X)
	0x35, 0x00,                         //       PHYSICAL_MINIMUM (0)
	0x46, 0x00, 0x00,             //       PHYSICAL_MAXIMUM (0)
	0x81, 0x02,                         //       INPUT (Data,Var,Abs)
	0x09, 0x31,                         //       USAGE (Y)
	0x81, 0x02,                         //       INPUT (Data,Var,Abs)
	0xc0,                               	//    END_COLLECTION
	//=====================================2
	0xa1, 0x02, 			 			//      COLLECTION (Logical)
	0x05, 0x0d, 						// 	    USAGE_PAGE (Digitizers)
	0x09, 0x42,                         //       USAGE (Tip Switch)
	0x15, 0x00,                         //       LOGICAL_MINIMUM (0)
	0x25, 0x01,                         //       LOGICAL_MAXIMUM (1)
	0x75, 0x01,                         //       REPORT_SIZE (1)
	0x95, 0x01,                         //       REPORT_COUNT (1)
	0x81, 0x02,                         //       INPUT (Data,Var,Abs)
	0x09, 0x32,                         //       USAGE (In Range)
	0x81, 0x02,                         //       INPUT (Data,Var,Abs)
	0x95, 0x06,                         //       REPORT_COUNT (6)
	0x81, 0x03,                         //       INPUT (Cnst,Ary,Abs)
	0x75, 0x08,                         //       REPORT_SIZE (8)
	0x09, 0x51,                         //       USAGE (Contact Identifier)
	0x95, 0x01,                         //       REPORT_COUNT (1)
	0x81, 0x02,                         //       INPUT (Data,Var,Abs)
	0x05, 0x01,                         //       USAGE_PAGE (Generic Desk..
	0x15, 0x00,                         //       LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x7f,              //       LOGICAL_MAXIMUM (32767)
	0x75, 0x10,                         //       REPORT_SIZE (16)
	0x55, 0x00,                         //       UNIT_EXPONENT (0)
	0x65, 0x00,                         //       UNIT (None)
	0x09, 0x30,                         //       USAGE (X)
	0x35, 0x00,                         //       PHYSICAL_MINIMUM (0)
	0x46, 0x00, 0x00,             //       PHYSICAL_MAXIMUM (0)
	0x81, 0x02,                         //       INPUT (Data,Var,Abs)
	0x09, 0x31,                         //       USAGE (Y)
	0x81, 0x02,                         //       INPUT (Data,Var,Abs)
	0xc0,                               		//    END_COLLECTION


#if 0
//=====================================3
	0xa1, 0x02, 			 			//      COLLECTION (Logical)
	0x05, 0x0d, 						// 	    USAGE_PAGE (Digitizers)
	0x09, 0x42,                         //       USAGE (Tip Switch)
	0x15, 0x00,                         //       LOGICAL_MINIMUM (0)
	0x25, 0x01,                         //       LOGICAL_MAXIMUM (1)
	0x75, 0x01,                         //       REPORT_SIZE (1)
	0x95, 0x01,                         //       REPORT_COUNT (1)
	0x81, 0x02,                         //       INPUT (Data,Var,Abs)
	0x09, 0x32,                         //       USAGE (In Range)
	0x81, 0x02,                         //       INPUT (Data,Var,Abs)
	0x95, 0x06,                         //       REPORT_COUNT (6)
	0x81, 0x03,                         //       INPUT (Cnst,Ary,Abs)
	0x75, 0x08,                         //       REPORT_SIZE (8)
	0x09, 0x51,                         //       USAGE (Contact Identifier)
	0x95, 0x01,                         //       REPORT_COUNT (1)
	0x81, 0x02,                         //       INPUT (Data,Var,Abs)
	0x05, 0x01,                         //       USAGE_PAGE (Generic Desk..
	0x15, 0x00,                         //       LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x7f,              //       LOGICAL_MAXIMUM (32767)
	0x75, 0x10,                         //       REPORT_SIZE (16)
	0x55, 0x00,                        					 //       UNIT_EXPONENT (0)
	0x65, 0x00,                         				//       UNIT (None)
	0x09, 0x30,                         					//       USAGE (X)
	0x35, 0x00,                         					//       PHYSICAL_MINIMUM (0)
	0x46, 0x00, 0x00,             					//       PHYSICAL_MAXIMUM (0)
	0x81, 0x02,                         					//       INPUT (Data,Var,Abs)
	0x09, 0x31,                        						 //       USAGE (Y)
	0x81, 0x02,                         					//       INPUT (Data,Var,Abs)
	0xc0,                               							//    END_COLLECTION
	//=====================================4
	0xa1, 0x02, 			 									// 		 COLLECTION (Logical)
	 0x05, 0x0d,												//			USAGE_PAGE (Digitizers)
	0x09, 0x42, 												//			 USAGE (Tip Switch)
	0x15, 0x00, 												//			 LOGICAL_MINIMUM (0)
	0x25, 0x01, 												//			 LOGICAL_MAXIMUM (1)
	0x75, 0x01, 												//			 REPORT_SIZE (1)
	0x95, 0x01, 												//			 REPORT_COUNT (1)
	0x81, 0x02, 												//			 INPUT (Data,Var,Abs)
	0x09, 0x32, 												//			 USAGE (In Range)
	0x81, 0x02, 												//			 INPUT (Data,Var,Abs)
	0x95, 0x06, 												//			 REPORT_COUNT (6)
	0x81, 0x03, 												//			 INPUT (Cnst,Ary,Abs)
	0x75, 0x08, 												//			 REPORT_SIZE (8)
	0x09, 0x51, 												//			 USAGE (Contact Identifier)
	0x95, 0x01, 												//			 REPORT_COUNT (1)
	0x81, 0x02, 												//			 INPUT (Data,Var,Abs)
	0x05, 0x01, 												//			 USAGE_PAGE (Generic Desk..
	0x15, 0x00, 												//			 LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x7f, 						 			// 			LOGICAL_MAXIMUM (32767)
	0x75, 0x10, 												//			 REPORT_SIZE (16)
	0x55, 0x00, 												//			 UNIT_EXPONENT (0)
	0x65, 0x00, 												//			 UNIT (None)
	0x09, 0x30, 												//			 USAGE (X)
	0x35, 0x00, 												//			 PHYSICAL_MINIMUM (0)
	0x46, 0x00, 0x00, 									//			 PHYSICAL_MAXIMUM (0)
	0x81, 0x02, 												//			 INPUT (Data,Var,Abs)
	0x09, 0x31, 												//			 USAGE (Y)
	0x81, 0x02, 												//			 INPUT (Data,Var,Abs)
	0xc0, 																//		END_COLLECTION
//=====================================5
	0xa1, 0x02, 			 									// 		 COLLECTION (Logical)
	0x05, 0x0d,												//			USAGE_PAGE (Digitizers)
	0x09, 0x42, 												//			 USAGE (Tip Switch)
	0x15, 0x00, 												//			 LOGICAL_MINIMUM (0)
	0x25, 0x01, 												//			 LOGICAL_MAXIMUM (1)
	0x75, 0x01, 												//			 REPORT_SIZE (1)
	0x95, 0x01, 												//			 REPORT_COUNT (1)
	0x81, 0x02, 												//			 INPUT (Data,Var,Abs)
	0x09, 0x32, 												//			 USAGE (In Range)
	0x81, 0x02, 												//			 INPUT (Data,Var,Abs)
	0x95, 0x06, 												//			 REPORT_COUNT (6)
	0x81, 0x03, 												//			 INPUT (Cnst,Ary,Abs)
	0x75, 0x08, 												//			 REPORT_SIZE (8)
	0x09, 0x51, 												//			 USAGE (Contact Identifier)
	0x95, 0x01, 												//			 REPORT_COUNT (1)
	0x81, 0x02, 												//			 INPUT (Data,Var,Abs)
	0x05, 0x01, 												//			 USAGE_PAGE (Generic Desk..
	0x15, 0x00, 												//			 LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x7f, 									 // 			LOGICAL_MAXIMUM (32767)
	0x75, 0x10, 												//			 REPORT_SIZE (16)
	0x55, 0x00, 												//			 UNIT_EXPONENT (0)
	0x65, 0x00, 												//			 UNIT (None)
	0x09, 0x30, 												//			 USAGE (X)
	0x35, 0x00, 												//			 PHYSICAL_MINIMUM (0)
	0x46, 0x00, 0x00, 									//			 PHYSICAL_MAXIMUM (0)
	0x81, 0x02, 												//			 INPUT (Data,Var,Abs)
	0x09, 0x31, 												//			 USAGE (Y)
	0x81, 0x02, 												//			 INPUT (Data,Var,Abs)
	0xc0, 																//		END_COLLECTION
	//=====================================6
	0xa1, 0x02, 			 									// 		 COLLECTION (Logical)
	0x05, 0x0d,												//			USAGE_PAGE (Digitizers)
	0x09, 0x42, 												//			 USAGE (Tip Switch)
	0x15, 0x00, 												//			 LOGICAL_MINIMUM (0)
	0x25, 0x01, 												//			 LOGICAL_MAXIMUM (1)
	0x75, 0x01, 												//			 REPORT_SIZE (1)
	0x95, 0x01, 												//			 REPORT_COUNT (1)
	0x81, 0x02, 												//			 INPUT (Data,Var,Abs)
	0x09, 0x32, 												//			 USAGE (In Range)
	0x81, 0x02, 												//			 INPUT (Data,Var,Abs)
	0x95, 0x06, 												//			 REPORT_COUNT (6)
	0x81, 0x03, 												//			 INPUT (Cnst,Ary,Abs)
	0x75, 0x08, 												//			 REPORT_SIZE (8)
	0x09, 0x51, 												//			 USAGE (Contact Identifier)
	0x95, 0x01, 												//			 REPORT_COUNT (1)
	0x81, 0x02, 												//			 INPUT (Data,Var,Abs)
	0x05, 0x01, 												//			 USAGE_PAGE (Generic Desk..
	0x15, 0x00, 												//			 LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x7f, 						 			// 			LOGICAL_MAXIMUM (32767)
	0x75, 0x10, 												//			 REPORT_SIZE (16)
	0x55, 0x00, 												//			 UNIT_EXPONENT (0)
	0x65, 0x00, 												//			 UNIT (None)
	0x09, 0x30, 												//			 USAGE (X)
	0x35, 0x00, 												//			 PHYSICAL_MINIMUM (0)
	0x46, 0x00, 0x00, 									//			 PHYSICAL_MAXIMUM (0)
	0x81, 0x02, 												//			 INPUT (Data,Var,Abs)
	0x09, 0x31, 												//			 USAGE (Y)
	0x81, 0x02, 												//			 INPUT (Data,Var,Abs)
	0xc0, 																//		END_COLLECTION
#endif	
	//==================================touch count
	#if 0  /*zhaorui 130814 +*/ 
	0x05, 0x0d,                         //    USAGE_PAGE (Digitizers)
	0x09, 0x54,                         //    USAGE (Contact Count)
	0x95, 0x01,                         //    REPORT_COUNT (1)
	0x75, 0x08,                         //    REPORT_SIZE (8)
	//0x81, 0x02,                         //    INPUT (Data,Var,Abs)
	0x81, MAX_POINT_CNT,                         //    INPUT (Data,Var,Abs)
	0x85, 0x03,                			 //   REPORT_ID (Touch)
	0x09, 0x55,                         //    USAGE(Contact Count Maximum)
	//0x25, 0x02,                        			 //    LOGICAL_MAXIMUM (2)
	0x25, MAX_POINT_CNT,                //    LOGICAL_MAXIMUM 
	//0xb1, 0x02,                         //    FEATURE (Data,Var,Abs)
	0xb1, MAX_POINT_CNT,                         //    FEATURE (Data,Var,Abs)
	0xc0,                              	 	// 		END_COLLECTION
	#endif
	
	0x05, 0x0d,                         //    USAGE_PAGE (Digitizers)
	0x09, 0x54,                         //    USAGE (Contact Count)
	0x95, 0x01,                         //    REPORT_COUNT (1)
	0x75, 0x08,                         //    REPORT_SIZE (8)
	0x25, MAX_POINT_CNT,				//	LOGICAL_MAXIMUM(MAX touch POINT) 
	0x81, MAX_POINT_CNT,				//    INPUT (Data,Var,Abs)
	0x09, 0x55, 						//	USAGE(Contact Count Maximum)
	0xb1, MAX_POINT_CNT,				//    FEATURE (Data,Var,Abs)
	//0x85, 0x03,                			 //   REPORT_ID (Touch)
	0xc0,    
	//=================================ferture
	0x09, 0x0E,                         // USAGE (Device Configuration)
	0xa1, 0x01,                         // COLLECTION (Application)
	0x85, 0x04,            				 //   REPORT_ID (Configuration)              
	0x09, 0x23,                         //   USAGE (Device Settings)              
	0xa1, 0x02,                         //   COLLECTION (logical)    
	0x09, 0x52,                         //    USAGE (Device Mode)         
	0x09, 0x53,                         //    USAGE (Device Identifier)
	0x15, 0x00,                         //    LOGICAL_MINIMUM (0)      
	0x25, 0x0a,                         //    LOGICAL_MAXIMUM (10)
	0x75, 0x08,                         //    REPORT_SIZE (8)         
	0x95, 0x02,                         //    REPORT_COUNT (2)         
	0xb1, 0x02,                         //   FEATURE (Data,Var,Abs    
	0xc0,                              		 //   END_COLLECTION
	0xc0,                              		 //   END_COLLECTION

}; /* CustomHID_ReportDescriptor */ 


/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
//const uint8_t CustomHID_ConfigDescriptor[CUSTOMHID_SIZ_CONFIG_DESC] =
const uint8_t CustomHID_ConfigDescriptor[] =
{
    0x09, /* bLength: Configuration Descriptor size */
    USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
    CUSTOMHID_SIZ_CONFIG_DESC,
    /* wTotalLength: Bytes returned */
    0x00,
    0x01,         /*bNumInterfaces: 1 interface*/   
    0x01,         /*bConfigurationValue: Configuration value*/
    0x00,         /*iConfiguration: Index of string descriptor describing
                                     the configuration*/
    0xa0,//0xc0,         /* bmAttributes: Bus powered */
                  /*Bus powered: 7th bit, Self Powered: 6th bit, Remote wakeup: 5th bit, reserved: 4..0 bits */
    //0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
    0x96,         /* MaxPower 300 mA: this current is used for detecting Vbus */
	
  /************** Descriptor of Custom HID interface ****************/
    0x09,         /*bLength: Interface Descriptor size*/
    USB_INTERFACE_DESCRIPTOR_TYPE,/*bDescriptorType: Interface descriptor type*/
    0x00,         /*bInterfaceNumber: Number of Interface*/
    0x00,         /*bAlternateSetting: Alternate setting*/
    0x02,         /* bNumEndpoints */
    0x03,         /*bInterfaceClass: HID*/
    0x00,         /* bInterfaceSubClass : 1=BOOT, 0=no boot */
    0x00,         /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
    0x00,            /*iInterface: Index of string descriptor*/
    
    /******************** Descriptor of Custom HID HID ********************/  
    0x09,         /*bLength: HID Descriptor size*/
    HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
    0x10,         /* bcdHID: HID Class Spec release number */
    0x01,
    0x00,         /*bCountryCode: Hardware target country*/
    0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
    0x22,         /*bDescriptorType*/
    
    //0x67,//CUSTOMHID_SIZ_REPORT_DESC,/* wItemLength: Total length of Report descriptor */
    //0x02,
    sizeof(CustomHID_ReportDescriptor)&0xff,
    (sizeof(CustomHID_ReportDescriptor)>>8)&0xff,
     /******************** Descriptor of Custom HID endpoints ******************/
    0x07,          /*bLength: Endpoint Descriptor size*/
    USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/

    0x81,          /* bEndpointAddress: Endpoint Address (IN) */               
                   // bit 3...0 : the endpoint number
                   // bit 6...4 : reserved
                    // bit 7     : 0(OUT), 1(IN)
    0x03,          /* bmAttributes: Interrupt endpoint */
    EP1_TXBUF,//0x02,          /* wMaxPacketSize: 20 Bytes max */
    0x00,
    0x01,          /*bInterval: Polling Interval (32 ms)*/
    //0x20,
    
    //------	
    0x07,	/* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,	/* bDescriptorType: */
			/*	Endpoint descriptor type */
    0x01,	/* bEndpointAddress: */
			/*	Endpoint Address (OUT) */
    0x03,	/* bmAttributes: Interrupt endpoint */
    EP1_RXBUF,//0x02,	/* wMaxPacketSize: 20 Bytes max  */
    0x00,
    0x01,	/* bInterval: Polling Interval (32 ms) */
    //0x20,

 } ; /* CustomHID_ConfigDescriptor */

/* USB Standard Device Descriptor */
//const uint8_t CustomHID_DeviceDescriptor[CUSTOMHID_SIZ_DEVICE_DESC] =
const uint8_t CustomHID_DeviceDescriptor[] =
{
	0x12,                       /*bLength */
	USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
	0x00,                       /*bcdUSB */
	0x02,
	0x00,                       /*bDeviceClass*/
	0x00,                       /*bDeviceSubClass*/
	0x00,                       /*bDeviceProtocol*/
	0x40,                       /*bMaxPacketSize 64*/
	0xf7,                       /*idVendor (0x0483)*/
	0x1f,
	0x17,                       /*idProduct = 0x5750*/
	0x00,
	0x00,                       /*bcdDevice rel. 2.00*/
	0x02,
	1,                          /*Index of string descriptor describing
	                                              manufacturer */
	2,                          /*Index of string descriptor describing
	                                             product*/
	3,                          /*Index of string descriptor describing the
	                                             device serial number */
	0x01                        /*bNumConfigurations*/
};
/* CustomHID_DeviceDescriptor */
  

/* USB String Descriptors (optional) */
const uint8_t CustomHID_StringLangID[CUSTOMHID_SIZ_STRING_LANGID] =
  {
    CUSTOMHID_SIZ_STRING_LANGID,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09,
    0x04
  }
  ; /* LangID = 0x0409: U.S. English */

const uint8_t CustomHID_StringVendor[CUSTOMHID_SIZ_STRING_VENDOR] =
  {
    CUSTOMHID_SIZ_STRING_VENDOR, /* Size of Vendor string */
    USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
    // Manufacturer: "STMicroelectronics" 
    'Z', 0, 'O', 0, 'W', 0,'E', 0,'E', 0, '_', 0, 'H', 0,'I',0,'D',0
  };

const uint8_t CustomHID_StringProduct[CUSTOMHID_SIZ_STRING_PRODUCT] =
  {
    CUSTOMHID_SIZ_STRING_PRODUCT,          /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'B', 0, 'y', 0, ' ', 0, 'H', 0, 'A', 0, 'L', 0,'E',0,'1',0
  };
uint8_t CustomHID_StringSerial[CUSTOMHID_SIZ_STRING_SERIAL] =
  {
    CUSTOMHID_SIZ_STRING_SERIAL,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'x', 0, 'x', 0, 'x', 0,'x', 0,'x', 0, 'x', 0, 'x', 0
  };

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

