// -----------------------------------------------------------------------------
//  Copyright (c) 2006, Yang Wu
//  All rights reserved.
//  Redistribute or use following BSD License.
// -----------------------------------------------------------------------------
//
// project via rhine nic driver
//   mii.h
//   This file contains MII register constants
//   Constants value copy from linux mii.h header file
//   I'm not sure MacOSX has same support or not. 
//	 If there are supports, please tell me.
//   
// Yang Wu 2006-4 @ Shanghai, China
// Email  : pinxue@hotmail.com
// Web   : http://www.pinxue.net
// -----------------------------------------------------------------------------

// MII
#define MII_BMCR            0x00        /* Basic mode control register */
#define MII_BMSR            0x01        /* Basic mode status register  */

// Basic mode control registers
#define BMC_FULLDUPLEX         0x0100  /* Full duplex                 */
#define BMC_ANRESTART          0x0200  /* Auto negotiation restart    */
#define BMC_ANENABLE           0x1000  /* Enable auto negotiation     */
#define BMC_SPEED100           0x2000  /* Select 100Mbps              */

// Basic mode status registers
#define BMS_LSTATUS            0x0004  /* Link status                 */
#define BMS_ANEGCAPABLE        0x0008  /* Enabled auto negotiation    */
#define BMS_ANEGCOMPLETE       0x0020  /* Auto negotiation completed  */
#define BMS_10HALF             0x0800  /*  10 BaseTX, half duplex     */
#define BMS_10FULL             0x1000  /*  10 BaseTx, full duplex     */
#define BMS_100HALF            0x2000  /* 100 BaseTx, half duplex     */
#define BMS_100FULL            0x4000  /* 100 BaseTx, full duplex     */
#define BMS_100BASE4           0x8000  /* 100 Base4                   */

