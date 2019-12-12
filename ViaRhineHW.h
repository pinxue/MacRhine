// -----------------------------------------------------------------------------
//  Copyright (c) 2006, Yang Wu
//  All rights reserved.
//  Redistribute or use following BSD License.
// -----------------------------------------------------------------------------
//
// project via rhine nic driver
//   ViaRhine.c
//   This file is via rhine ethernet interface definition.
//   Chip information is analyzed from linux driver (version 2.6.14) 
//    and http://CESDIS.gsfc.nasa.gov/linux/misc/VT86C100A.pdf
//   Special thanks to http://www.scyld.com/100mbps.html#via
//   
// Yang Wu 2006 @ Shanghai, China
// Email  : pinxue@hotmail.com
// Web    : http://www.pinxue.net
// -----------------------------------------------------------------------------

#ifndef _ViaRhineHW_H
#define _ViaRhineHW_H

enum pci_revs {
	VT86C100A	= 0x00,
	VTUnknown0	= 0x20,
	VT6102		= 0x40,
	VT8231		= 0x50,
	VT8233		= 0x60,
	VT8235		= 0x74,
	VT6103		= 0x78, // VT8237
	VTUnknown1	= 0x7C,
	VT6105		= 0x80,
	VT6105B0	= 0x83,
	VT6105L		= 0x8A,
	VT6107		= 0x8C,
	VT6107A		= 0x8D,
	VTUnknown2	= 0x8E,
	VT6105M		= 0x90,
};

enum register_index {
	StationAddr=0x00, // ethernet address, 6 bytes
	RxConfig=0x06, 
	TxConfig=0x07, 
	ChipCmd=0x08,
	ChipCmd1=0x09,
	IntrStatus=0x0C, 
	IntrEnable=0x0E,
	MulticastFilter0=0x10, 
	MulticastFilter1=0x14,
	RxRingPtr=0x18,  // bus address of rx descriptor ring
	TxRingPtr=0x1C,  // bus address of tx descriptor ring
	GFIFOTest=0x54,
	MIIPhyAddr=0x6C, 
	MIIStatus=0x6D, 
	PCIBusConfig=0x6E,
	MIICmd=0x70, 
	MIIRegAddr=0x71, 
	MIIData=0x72, 
	MACRegEEcsr=0x74,
	ConfigA=0x78, 
	ConfigB=0x79, 
	ConfigC=0x7A, 
	ConfigD=0x7B,
	RxMissed=0x7C, 
	RxCRCErrs=0x7E, 
	MiscCmd=0x81,
	StickyHW=0x83, 
	IntrStatus2=0x84,
	WOLcrSet=0xA0, 
	PwcfgSet=0xA1, 
	WOLcgSet=0xA3, 
	WOLcrClr=0xA4,
	WOLcrClr1=0xA6, 
	WOLcgClr=0xA7,
	PwrcsrSet=0xA8, 
	PwrcsrSet1=0xA9, 
	PwrcsrClr=0xAC, 
	PwrcsrClr1=0xAD,
};

enum configd_bits {
	BackOptional=0x01, 
	BackModify=0x02,
	BackCaptureEffect=0x04, 
	BackRandom=0x08
};

enum intr_status_bits {
	IntrRxDone=0x0001, 
	IntrRxErr=0x0004, 
	IntrRxEmpty=0x0020,
		
	IntrTxDone=0x0002, 
	IntrTxError=0x0008, 
	IntrTxUnderrun=0x0210,
	
	IntrPCIErr=0x0040,
	IntrStatsMax=0x0080, 
	
	IntrRxEarly=0x0100,
	IntrRxOverflow=0x0400, 
	IntrRxDropped=0x0800, 
	IntrRxNoBuf=0x1000,
	
	IntrTxAborted=0x2000, 
	IntrLinkChange=0x4000,
	IntrRxWakeUp=0x8000,
	
	IntrNormalSummary=0x0003, 
	IntrAbnormalSummary=0xC260,
	
	IntrTxDescRace=0x080000,	/* mapped from IntrStatus2 */
	IntrTxErrSummary=0x082218,
};

enum rx_cfg_reg_bits{
	AllowError		= 1,      // accept error packet
	AllowSmall		= 1 << 1, // accept small packet
	AllowMulticast	= 1 << 2, // accept multicast packet
	AllowBroadcast	= 1 << 3, // accept broadcast packet
	AllowPhysical	= 1 << 4, // 0-phy add must match node addr in PAR0-5, 1-any with phy addr
	RxThresh        = 7 << 5 // default recv fifo threshold is 0 (64k)
};

/* The Rx and Tx buffer descriptors. */
struct rx_desc {
	UInt32 rx_status;
	UInt32 desc_length; /* Chain flag Bit15, Buffer/frame length Bit0~10 */
	UInt32 addr;
	UInt32 next_desc;
};
struct tx_desc {
	UInt32 tx_status;
	UInt32 desc_length; /* Chain flag Bit15, Tx Config Bit16~23, Frame length Bit0~10 */
	UInt32 addr;
	UInt32 next_desc;
};

/* Initial value for tx_desc.desc_length, Buffer size goes to bits 0-10 */
#define TXDESC		0x00e08000

enum rx_status_bits {
	RxOK=0x8000, 
	RxWholePkt=0x0300, // STP:EDP is 1:1
	RxErr=0x008F
};

enum tx_status_bits {
	TxOwn	= 1 << 31, // 0-free, 1-own by chip
	TxOk	= 1 << 15, // 1-tx error occured
	TxJab	= 1 << 14, // 1=jabber condition, read only
	TxSerr	= 1 << 13, // system error
	TxCsr	= 1 << 10, // carrier sense lost
	TxOwc	= 1 << 9,  // late collisions
	TxAbt	= 1 << 8,  // transmit abort
	TxCdh	= 1 << 7,  // heart beat collision check failure
	TxNcr	= 0xf << 3,// collision count
	TxUdf	= 1 << 1,  // FIFO underflow
	TxDfr	= 1        // defer
};

/* Bits in *_desc.*_status */
enum desc_status_bits {
	DescOwn=0x80000000  // 0-free, 1-owned by nic controller
};

enum tx_cfg_bits {
    TxDescIC  = 1<<23,
	TxDescEDP = 1<<22, // end of packet
	TxDescSTP = 1<<21, // start of packet, STP:EDP = 1:1 means packet in single descriptor 
	TxDescNCRC= 1<<16
};

/* Bits in ChipCmd. */
enum chip_cmd_bits {
	CmdInit=0x01, 
	CmdStart=0x02, 
	CmdStop=0x04, 
	CmdRxOn=0x08,
	CmdTxOn=0x10, 
	Cmd1TxDemand=0x20, 
	CmdRxDemand=0x40,
	Cmd1EarlyRx=0x01, 
	Cmd1EarlyTx=0x02, 
	Cmd1FDuplex=0x04,
	Cmd1NoTxPoll=0x08, 
	Cmd1Reset=0x80,
};

enum mii_status_bits {
	MiiSpeed              =1<<0,  // 0 100MHz, 1 10MHz
	MiiLinkFail           =1<<1,
	MiiMgmtPortReadErr    =1<<2,
	MiiPhyDevRecErr       =1<<3,
	MiiFixedPhyAddr       =1<<4,
	MiiFastMDCAutopolling =1<<5, // 0 normal, 1 4x
	MiiGPIO1OutPolarity   =1<<7  // 0 low, 1 high
};

typedef UInt32 MediumIndex;

enum {
    MEDIUM_INDEX_10_HD = 0,
    MEDIUM_INDEX_10_FD = 1,
    MEDIUM_INDEX_TX_HD = 2,
    MEDIUM_INDEX_TX_FD = 3,
    MEDIUM_INDEX_T4    = 4,
    MEDIUM_INDEX_AUTO  = 5,
    MEDIUM_INDEX_NONE  = 6,
    MEDIUM_INDEX_COUNT = 6
};

#endif /* !_ViaRhineHW_H */
