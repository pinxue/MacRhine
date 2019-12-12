// -----------------------------------------------------------------------------
//  Copyright (c) 2006, Yang Wu
//  All rights reserved.
//  Redistribute or use following BSD License.
// -----------------------------------------------------------------------------
//
// project via rhine nic driver
//   ViaRhine.h
//   This file is Enthernet Controller class interface for via rhine driver
//   
// Yang Wu 2006-3 @ Shanghai, China
// Email : pinxue@hotmail.com
// Web   : http://www.pinxue.net
// -----------------------------------------------------------------------------

#ifndef _ViaRhine_H
#define _ViaRhine_H

#include <IOKit/assert.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IOGatedOutputQueue.h>
#include <IOKit/network/IOMbufMemoryCursor.h>

extern "C" {
    #include <sys/kpi_mbuf.h>
}

#include "ViaRhineHW.h"

#ifdef  DEBUG
#define DEBUG_LOG(args...)  IOLog(args)
#else
#define DEBUG_LOG(args...)
#endif

#define BUMP_NET_COUNTER(x) \
        netStats->x += 1;

#define BUMP_ETHER_COUNTER(x) \
        etherStats->dot3StatsEntry.x += 1;

#define BUMP_ETHER_RX_COUNTER(x) \
        etherStats->dot3RxExtraEntry.x += 1;

#define BUMP_ETHER_TX_COUNTER(x) \
        etherStats->dot3TxExtraEntry.x += 1;

#define kWatchdogTimerPeriod    10000  // milliseconds

#define kTransmitQueueCapacity  384
#define MINPACK                 60    // minimum output packet length 
#define MAXPACK                 1514  // maximum output packet length
#define RX_RING_SIZE            128	  // number of rx descriptors to compsite the ring
#define TX_RING_SIZE            128	  // number of tx descriptors to compsite the ring
#define TX_QUEUE_LEN			TX_RING_SIZE-4   // actually number of descriptors allowed to be using
#define PKT_BUF_SZ              1536

#define ViaRhine net_pinxue_driver_ViaRhine

class ViaRhine : public IOEthernetController
{
    OSDeclareDefaultStructors( net_pinxue_driver_ViaRhine )

protected:
    IOEthernetInterface *      netif;
    IOPCIDevice *              pciNub;
    IOWorkLoop *               workLoop;
    IOInterruptEventSource *   interruptSrc;
    IOOutputQueue *            transmitQueue;
    IOTimerEventSource *       timerSrc;
    //IOKernelDebugger *         debugger;
    IONetworkStats *           netStats;
    IOEthernetStats *          etherStats;
    IOMemoryMap *              csrMap;
    volatile void *            csrBase;
	bool					   promiscuousMode;
	
	IOBufferMemoryDescriptor * tx_md;
	struct tx_desc *           tx_ring;
	IOPhysicalAddress          tx_ring_dma;
	mbuf_t  				   tx_mbuf[TX_RING_SIZE];
	
	IOBufferMemoryDescriptor * tx_buf_md[TX_RING_SIZE];
	unsigned char *            tx_buf[TX_RING_SIZE];
	IOPhysicalAddress		   tx_buf_dma[TX_RING_SIZE];
	
	UInt32                     tx_cur;
	UInt32					   tx_dirty;
	
	UInt16					   rx_copybreak;
	
	IOBufferMemoryDescriptor * rx_md;
	struct rx_desc *           rx_ring;
	IOPhysicalAddress          rx_ring_dma;
	
	UInt32					   rx_buf_sz; // based on MTU+slack
	IOBufferMemoryDescriptor * rx_buf_md[RX_RING_SIZE];
	unsigned char *			   rx_buf[RX_RING_SIZE];
	IOPhysicalAddress		   rx_buf_dma[RX_RING_SIZE];
	
	UInt32					   rx_cur;
	UInt32					   rx_dirty;
	struct rx_desc *		   rx_head_desc;
	
	UInt8					   pciRev;
	const char *			   model;
	UInt16					   phyId;
	bool                       isRhineI;
	bool					   statusWBRace;
	bool					   sixPatterns;
 
    UInt8                      reg_config1;
    UInt32                     currentLevel;
    bool                       enabledByBSD;
    bool                       enabledByKDP;
    bool                       interruptEnabled;
	OSDictionary *             mediumDict;
	const IONetworkMedium *    mediumTable[MEDIUM_INDEX_COUNT];
	MediumIndex				   currentMediumIndex;
    UInt16                     phyStatusLast;
    UInt16                     reg_bms;
    UInt32                     reg_mar0;
    UInt32                     reg_mar1;
	UInt8					   tx_thresh;
	UInt8					   rx_thresh;
	UInt32					   multicast_filter_limit;
	
	void dumpRegisters( void );

    void restartReceiver( void );

    void transmitterInterrupt( bool * reclaimed );

    void receiverInterrupt( bool * queued );
	
	void errorInterrupt( UInt32 intrStatus );

    void interruptOccurred( OSObject * client, IOInterruptEventSource * src, int count );

    bool initEventSources( IOService * provider );

    bool initPCIConfigSpace( IOPCIDevice * provider );

    bool enableAdapter( UInt32 level );

    bool disableAdapter( UInt32 level );

    enum {
        kActivationLevel0 = 0,
        kActivationLevel1,
        kActivationLevel2
    };

    bool setActivationLevel( UInt32 newLevel );

    void timeoutOccurred( OSObject * owner, IOTimerEventSource * timer );

    bool allocateDescriptorMemory( void );

    enum {
        kFullInitialization = 0,
        kResetChip          = 1
    };

    bool initAdapter( IOOptionBits options );
	void reloadEEPRom( void );

    void disableHardwareInterrupts( void );

    void enableHardwareInterrupts( void );

    void reclaimTransmitBuffer( void );

    // PHY functions

	void enableLinkMon( void );
	void disableLinkMon( void );
	
	UInt16 mdioRead( UInt16 reg );
	void mdioWrite( UInt16 reg, UInt16 value );
	
	bool phyAddMediumType( IOMediumType type, UInt32 bps, MediumIndex  index);
    void phyProbeMediaCapability( void );

    bool phyReset( void );

    bool phyWaitForAutoNegotiation( void );

	bool phySetMedium( MediumIndex mediumIndex );

    bool phySetMedium( const IONetworkMedium * medium );

    void phyReportLinkStatus( bool forceStatusReport = false );

	const IONetworkMedium * phyGetMediumWithIndex( MediumIndex index ) const;

    // Access hardware registers

    inline void csrWrite32( UInt16 offset, UInt32 value )
    { pciNub->ioWrite32(offset, value, csrMap); }

    inline void csrWrite16( UInt16 offset, UInt16 value )
    { pciNub->ioWrite16(offset, value, csrMap); }

    inline void csrWrite8(  UInt16 offset, UInt8 value )
    { pciNub->ioWrite8(offset, value, csrMap); }

    inline void csrWrite32Slow( UInt16 offset, UInt32 value )
    { pciNub->ioWrite32(offset, value, csrMap); pciNub->ioRead32(offset, csrMap); }

    inline void csrWrite16Slow( UInt16 offset, UInt16 value )
    { pciNub->ioWrite16(offset, value, csrMap); pciNub->ioRead16(offset, csrMap); }

    inline void csrWrite8Slow(  UInt16 offset, UInt8 value )
    { pciNub->ioWrite8(offset, value, csrMap); pciNub->ioRead8(offset, csrMap); }

    inline UInt32 csrRead32( UInt16 offset )
    { return pciNub->ioRead32(offset, csrMap); }

    inline UInt16 csrRead16( UInt16 offset )
    { return pciNub->ioRead16(offset, csrMap); }

    inline UInt8  csrRead8(  UInt16 offset )
    { return pciNub->ioRead8(offset, csrMap); }
	
public:
    virtual bool init( OSDictionary * properties );

    virtual bool start( IOService * provider );

    virtual void stop( IOService * provider );

    virtual void free( void );

    virtual IOReturn enable( IONetworkInterface * netif );

    virtual IOReturn disable( IONetworkInterface * netif );

    virtual IOReturn enable( IOKernelDebugger * netif );

    virtual IOReturn disable( IOKernelDebugger * netif );

    virtual UInt32 outputPacket( mbuf_t m, void * param );

    virtual void getPacketBufferConstraints(
                 IOPacketBufferConstraints * constraints ) const;

    virtual IOOutputQueue * createOutputQueue( void );

    virtual const OSString * newVendorString( void ) const;

    virtual const OSString * newModelString( void ) const;

    virtual IOReturn selectMedium( const IONetworkMedium * medium );

    virtual bool configureInterface( IONetworkInterface * interface );

    virtual bool createWorkLoop( void );

    virtual IOWorkLoop * getWorkLoop( void ) const;

    virtual IOReturn getHardwareAddress( IOEthernetAddress * addr );

    virtual IOReturn setPromiscuousMode( bool enabled );

    virtual IOReturn setMulticastMode( bool enabled );

    virtual IOReturn setMulticastList( IOEthernetAddress * addrs,
                                       UInt32              count );

    virtual void sendPacket( void * pkt, UInt32 pkt_len );

    virtual void receivePacket( void * pkt, UInt32 * pkt_len, UInt32 timeout );

    virtual IOReturn registerWithPolicyMaker( IOService * policyMaker );

    virtual IOReturn setPowerState( unsigned long powerStateOrdinal,
                                    IOService *   policyMaker );
};

#endif /* !_ViaRhine_H */

