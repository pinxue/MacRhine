// -----------------------------------------------------------------------------
//  Copyright (c) 2006, Yang Wu
//  All rights reserved.
//  Redistribute or use following BSD License.
// -----------------------------------------------------------------------------
//
// project via rhine nic driver
//   ViaRhine.cpp
//   This file is Enthernet Controller class skeleton for via rhine driver
//   
// Yang Wu 2006-3 @ Shanghai, China
// Email : pinxue@hotmail.com
// Web   : http://www.pinxue.net
// -----------------------------------------------------------------------------

#include "ViaRhine.h"

#define super IOEthernetController

OSDefineMetaClassAndStructors( net_pinxue_driver_ViaRhine,
                               IOEthernetController )

bool ViaRhine::init( OSDictionary * properties )
{
	DEBUG_LOG("driver init\n");
    if ( false == super::init(properties) ) return false;

    currentLevel       = kActivationLevel0;
	currentMediumIndex = MEDIUM_INDEX_NONE;
	
	promiscuousMode = false;
	tx_thresh = 0x20;
	rx_thresh = 0x60;
	multicast_filter_limit = 32;
	reg_mar0 = reg_mar1 = 0;
	
	tx_cur = tx_dirty = 0;
	rx_cur = rx_dirty = 0;
	
	phyId = 0;
	isRhineI = false;
	statusWBRace = false;
	sixPatterns = false;
	
    return true;
}

bool ViaRhine::start( IOService * provider )
{
    bool success = false;
    
    DEBUG_LOG("start() ..begin..\n");

    do {
        if ( false == super::start(provider) )
            break;

        pciNub = OSDynamicCast( IOPCIDevice, provider );
        if ( 0 == pciNub )
            break;

        pciNub->retain();

        if ( false == pciNub->open(this) )
            break;

		// Active Probe : feature of nic
		pciRev = pciNub->configRead8( 0x08 );
		
		if ( pciRev < VTUnknown0 )
		{
			isRhineI = true;
			model = "Rhine I";
			// ioSize = 128;
		}else if ( pciRev >= VT6102 ){
			if ( pciRev < VT6105 )
			{
				// this is "Rhine II", support StatusWBRace
				statusWBRace = true;
				model = "Rhine II";
			}else{
				// this is intergarted PHY, phyId is fixed to "1"
				phyId = 1;
				if (pciRev >= VT6105B0)
					sixPatterns = true; // support 6patterns
					
				if (pciRev < VT6105M)
				{
					// this is "Rhine III"
					model = "Rhine III";
				}else{
					// this is "Rhine III (Management Adapter)"
					model = "Rhine III (Management Adapter)";
				}
			}
		}else{
			model = "Rhine Unknown revision";
		}
		IOLog( " %s: found %s.\n", getName(), model );

        if ( false == initPCIConfigSpace( pciNub ) )
            break;

        if ( false == initEventSources(provider) )
            break;

        // prepair memory mapped register accessing
		DEBUG_LOG( " %s: config base address is %p.\n", getName(), 
			pciNub->configRead32( kIOPCIConfigBaseAddress0 ) );
        csrMap = pciNub->mapDeviceMemoryWithRegister(  kIOPCIConfigBaseAddress0 );
        if ( 0 == csrMap )
		{
			IOLog( " %s: failed to setup memory mapped i/o.\n", getName() );
            break;
		}
		DEBUG_LOG( " %s: memory mapped io init. addr:%p,\t%p.\n", getName(),
				csrMap->getVirtualAddress(), csrMap->getPhysicalAddress()  );

        csrBase = (volatile void *) csrMap->getVirtualAddress();

		// patch PHY id
		if ( ! phyId )
		{
			phyId = csrRead8( 0x6c ); // PhyAddr, Rhine-I/II, phyId is loaded from EEPROM
		}
		IOLog( " %s: PCI_REV 0x%x, PHY ID 0x%x.\n", getName(), pciRev, phyId );

        if ( false == allocateDescriptorMemory() )
            break;

        // Reset chip to bring it to a known state.
		// includes set rx/tx descriptor ring address.
        if ( initAdapter( kResetChip ) == false )
        {
            IOLog("%s: initAdapter() failed\n", getName());
            break;
        }


        // Publish our media capabilities.
        phyProbeMediaCapability();
        if (false == publishMediumDictionary(mediumDict))
		{
			IOLog("%s: failed to publish medium dict!\n", getName());
			break;
		}

        success = true;
    }
    while ( false );

    if (pciNub) pciNub->close(this);
    
    do {
        if (false == success)
            break;

        success = false;

        if (false == attachInterface((IONetworkInterface **) &netif, false))
            break;

        // Optional: kernel debugging supports.
        //attachDebuggerClient( &debugger );

        netif->registerService();

        success = true;
    }
    while ( false );

    DEBUG_LOG("start() %x ..end..\n", success);
    
    return success;
}

//---------------------------------------------------------------------------

void ViaRhine::stop( IOService * provider )
{
    DEBUG_LOG("stop() ..begin..\n");

	detachInterface( netif );

    super::stop( provider );

    DEBUG_LOG("stop() ..end..\n");
}

//---------------------------------------------------------------------------

bool ViaRhine::initEventSources( IOService * provider )
{
    DEBUG_LOG("initEventSources() ..begin..\n");

	IOWorkLoop * wl = getWorkLoop();
	if ( 0 == wl )
        return false;

	transmitQueue = getOutputQueue();
	if ( 0 == transmitQueue )
        return false;

	interruptSrc = IOInterruptEventSource::interruptEventSource(this,
                   OSMemberFunctionCast(IOInterruptEventSource::Action, this, &ViaRhine::interruptOccurred),
                   provider);

	if ( !interruptSrc ||
		 (wl->addEventSource(interruptSrc) != kIOReturnSuccess) )
		return false;

    interruptSrc->enable();

	// Register watchdog timer.
	timerSrc = IOTimerEventSource::timerEventSource( this,
               OSMemberFunctionCast(IOTimerEventSource::Action, this, &ViaRhine::timeoutOccurred) );

	if ( !timerSrc || (wl->addEventSource(timerSrc) != kIOReturnSuccess) )
	{
		IOLog( " %s: failed to add watchdog timer event source!\n", getName() );
		return false;
	}

	// IONetworkMedium objects dictionary.
	mediumDict = OSDictionary::withCapacity(5);
	if ( 0 == mediumDict )
		return false;

	DEBUG_LOG("initEventSources() ..end..\n");

	return true;
}

// enable the I/O mapped PCI memory range,
// and bus-master interface.
bool ViaRhine::initPCIConfigSpace( IOPCIDevice * provider )
{
	UInt8  byte;
    UInt16 reg;
	
    DEBUG_LOG( "pciConfigInit() ..begin..\n" );
	
	byte = provider->configRead8( 0x52 ); // MODE2
	
	if ( pciRev < VT6102 )
		provider->configWrite8( 0x52, byte | 0x02 ); // MODE2_MODE10T
	else {
		provider->configWrite8( 0x52, byte | 0x80 ); // MODE2_PCEROPT
		provider->configWrite8( 0x53, provider->configRead8( 0x53 ) | 0x04 ); // MODE3_MIION
		if ( pciRev > VT6105L && pciRev < VT6105M )
			provider->configWrite8( 0x52, byte | 0x02 ); // MODE2_MODE10T
		if ( pciRev > VT6107A && pciRev < VT6105M )
			provider->configWrite8( 0x52, byte | 0x08 ); // MODE2_MRDPL
	}

    reg = provider->configRead16( kIOPCIConfigCommand );

    reg |= ( kIOPCICommandBusMaster |
             kIOPCICommandIOSpace   |
             kIOPCICommandMemWrInvalidate );

    reg &= ~kIOPCICommandMemorySpace;

    provider->configWrite16( kIOPCIConfigCommand, reg );

    DEBUG_LOG( "pciConfigInit() ..end..\n" );

    return true;
}

bool ViaRhine::createWorkLoop( void )
{
    DEBUG_LOG( "createWorkLoop() ..begin..\n" );
    workLoop = IOWorkLoop::workLoop();
    DEBUG_LOG( "createWorkLoop() ..end..\n" );
    return ( workLoop != 0 );
}

IOWorkLoop * ViaRhine::getWorkLoop( void ) const
{
	return workLoop;
}

bool ViaRhine::configureInterface( IONetworkInterface * netif )
{
    IONetworkData * data;

    DEBUG_LOG("configureInterface() ..begin..\n");

    if ( false == super::configureInterface(netif) )
        return false;
	
    // Get the generic network statistics structure.
    data = netif->getParameter( kIONetworkStatsKey );
    if ( !data || !(netStats = (IONetworkStats *) data->getBuffer()) ) 
    {
        return false;
    }

    // Get the Ethernet statistics structure.
    data = netif->getParameter( kIOEthernetStatsKey );
    if ( !data || !(etherStats = (IOEthernetStats *) data->getBuffer()) ) 
    {
        return false;
    }

    DEBUG_LOG("configureInterface() ..end..\n");
    return true;
}

void ViaRhine::free( void )
{
#define RELEASE(x) do { if(x) { (x)->release(); (x) = 0; } } while(0)

    DEBUG_LOG("free() ..begin..\n");

    if ( interruptSrc && workLoop )
    {
        workLoop->removeEventSource( interruptSrc );
    }
	
    RELEASE( netif        );
    //RELEASE( debugger     );
    RELEASE( interruptSrc );
    RELEASE( timerSrc     );
    RELEASE( csrMap       );
    RELEASE( pciNub       );
    RELEASE( workLoop     );

    if ( rx_md )
    {
        rx_md->complete();
        rx_md->release();
        rx_md = 0;
    }
	
	if ( tx_md )
	{
		tx_md->complete();
		tx_md->release();
		tx_md = 0;
	}

    for ( int i = 0; i < TX_RING_SIZE; i++ )
    {
        if ( tx_buf_md[i] )
        {
            tx_buf_md[i]->complete();
            tx_buf_md[i]->release();
            tx_buf_md[i] = 0;
        }
    }

    for ( int i = 0; i < RX_RING_SIZE; i++ )
    {
        if ( rx_buf_md[i] )
        {
            rx_buf_md[i]->complete();
            rx_buf_md[i]->release();
            rx_buf_md[i] = 0;
        }
    }

    super::free();

    DEBUG_LOG("free() ..end..\n");
}

// Enables the adapter & driver to the given level of support.
bool ViaRhine::enableAdapter( UInt32 level )
{
    bool success = false;

    DEBUG_LOG( "enableAdapter() ..begin..\n" );
    DEBUG_LOG( "enable level %ld\n", level );

    switch ( level ) 
    {
        case kActivationLevel1:

            if ( (0 == pciNub) || (false == pciNub->open(this)) )
                break;

            // Perform a full initialization sequence.
            if ( initAdapter( kFullInitialization ) != true )
			{
				DEBUG_LOG( "init adapter failed !\n");
                break;
			}
				
            // PHY medium selection.
			const IONetworkMedium * m = getSelectedMedium();
			if ( m == 0 )
				DEBUG_LOG("selected medium is NULL !\n");
			else
				DEBUG_LOG("selected medium : speed %0x, type %0x, flags %0x.\n",
				    m->getSpeed(), m->getType(), m->getFlags()  );
            if ( selectMedium( m ) != kIOReturnSuccess )
			{
				IOLog( " %s: select medium failed !\n", getName() );
                break;
			}

            // Start the watch-dog timer.
            timerSrc->setTimeoutMS( kWatchdogTimerPeriod );

            // to be ensure: does this real needed?
			// Unless we wait and ack PUN/LinkChg interrupts, the receiver will not work. 
/*
            for ( int i = 0; i < 100; i++ )
            {
                UInt16 isr = csrRead16( IntrStatus );
                if ( isr & IntrTxUnderrun )
                {
                    csrWrite16( IntrStatus, IntrTxUnderrun );
                    DEBUG_LOG("cleared PUN interrupt %x in %d\n", isr, i);
                    break;
                }
                IOSleep(10);
            }
*/			
			DEBUG_LOG( "init to KActivationLevel1 ok. " );

            success = true;
            break;
		
        case kActivationLevel2:

            transmitQueue->setCapacity( kTransmitQueueCapacity );
            transmitQueue->start();

            enableHardwareInterrupts();
            
			DEBUG_LOG( "init to KActivationLevel2 ok. " );

            success = true;
            break;
    }

    if ( false == success )
        IOLog("enable level %ld failed\n", level);

    DEBUG_LOG("enableAdapter() ..end..\n");
    return success;
}

bool ViaRhine::disableAdapter( UInt32 level )
{
    bool success = false;

    DEBUG_LOG("disableAdapter() ..begin..\n");
    DEBUG_LOG("disable level %ld\n", level);

    switch ( level )
    {
        case kActivationLevel1:

            timerSrc->cancelTimeout();

            initAdapter( kResetChip );

            phySetMedium( MEDIUM_INDEX_NONE );
            setLinkStatus( 0 );

            transmitQueue->setCapacity( 0 );
            transmitQueue->flush();

			// reset descriptor ring
			for ( int i = 0; i < TX_RING_SIZE; ++i )
			{
				(tx_ring+i)->tx_status = 0;
				(tx_ring+i)->desc_length = TXDESC | PKT_BUF_SZ;
			}
			
			for ( int i = 0; i < RX_RING_SIZE; ++i )
			{
				(rx_ring+i)->rx_status = DescOwn;
			}

            if ( pciNub ) pciNub->close(this);

            success = true;
            break;

        case kActivationLevel2:
			transmitQueue->flush();
			
            disableHardwareInterrupts();

            transmitQueue->stop();
			
            success = true;
            break;
    }

    if ( false == success )
        IOLog("disable level %ld failed\n", level);

    DEBUG_LOG("disableAdapter() ..end..\n");

    return success;
}

// kActivationLevel0 : Adapter disabled.
// kActivationLevel1 : Adapter partially enabled to support KDP.
// kActivationLevel2 : Adapter completely enabled for KDP and BSD.
bool ViaRhine::setActivationLevel( UInt32 level )
{
    bool    success = false;
    UInt32  nextLevel;

    DEBUG_LOG("setActivationLevel() ..begin..\n");
    DEBUG_LOG("---> Current Level: %ld, Target Level: %ld\n",
              currentLevel, level);

    if (currentLevel == level) 
        return true;

    for ( ; currentLevel > level; currentLevel--) 
    {
        if ( (success = disableAdapter(currentLevel)) == false )
            break;
    }

    for ( nextLevel = currentLevel + 1; currentLevel < level;
          currentLevel++, nextLevel++ ) 
    {
        if ( (success = enableAdapter(nextLevel)) == false )
            break;
    }

    DEBUG_LOG("---> New Current Level: %ld\n\n", currentLevel);
    DEBUG_LOG("setActivationLevel() ..end..\n");
    return success;
}

IOReturn ViaRhine::enable( IONetworkInterface * netif )
{
    DEBUG_LOG("enable(netif) ..begin..\n");
    
    if ( true == enabledByBSD )
    {
		//initPCIConfigSpace( pciNub );
		setActivationLevel( kActivationLevel2 );
		//restartReceiver();
        DEBUG_LOG("enable() ..end.. (enabled by bsd)\n");
        return kIOReturnSuccess;
    }

    enabledByBSD = setActivationLevel( kActivationLevel2 );

    DEBUG_LOG("enable(netif) ..end..\n");
	
	dumpRegisters();

    return enabledByBSD ? kIOReturnSuccess : kIOReturnIOError;
}

IOReturn ViaRhine::disable( IONetworkInterface * /*netif*/ )
{
    DEBUG_LOG("disable(netif) ..begin..\n");

    enabledByBSD = false;

    setActivationLevel( enabledByKDP ?
                        kActivationLevel1 : kActivationLevel0 );

    DEBUG_LOG("disable(netif) ..end..\n");
	
	dumpRegisters();

	return kIOReturnSuccess;
}

/*
IOReturn ViaRhine::enable( IOKernelDebugger * )
{
	DEBUG_LOG( "enabling with debugger\n" );
	if ( enabledByKDP || enabledByBSD )
    {
		enabledByKDP = true;
		return kIOReturnSuccess;
	}

	enabledByKDP = setActivationLevel( kActivationLevel1 );

	return enabledByKDP ? kIOReturnSuccess : kIOReturnIOError;
}
*/
/*
IOReturn ViaRhine::disable( IOKernelDebugger *  )
{
	enabledByKDP = false;

	if ( enabledByBSD == false )
		setActivationLevel( kActivationLevel0 );

	return kIOReturnSuccess;
}
*/

void ViaRhine::timeoutOccurred(OSObject *owner, IOTimerEventSource * timer )
{
    phyReportLinkStatus();
    timerSrc->setTimeoutMS( kWatchdogTimerPeriod );
}

IOReturn ViaRhine::setPromiscuousMode( bool enabled )
{
    DEBUG_LOG("setPromiscuousMode() ..begin..\n");
	
	DEBUG_LOG("current state: MulticastFilter0=0x%x, MulticastFilter1=0x%x, RxConfig=0x%x, reg_mar:0x%x,0x%x\n",
		csrRead32(MulticastFilter0), 
		csrRead32(MulticastFilter1), 
		csrRead32(RxConfig), 
		reg_mar0, reg_mar1
	);
    
	UInt8 rxMode=0x0c; // 0x02 = accept runt, 0x01=accept errs
    if ( enabled )
    {
        // Accept all multicast.
		rxMode = 0x1c;
        csrWrite32( MulticastFilter0, 0xffffffff );
        csrWrite32( MulticastFilter1, 0xffffffff );
    }
    else
    {
        // Restore multicast hash filter.
        csrWrite32( MulticastFilter0, reg_mar0 );
        csrWrite32( MulticastFilter1, reg_mar1 );
		rxMode = 0x0c;
    }
    
    csrWrite8( RxConfig , rx_thresh | rxMode );

    DEBUG_LOG("setPromiscuousMode rxMode = 0x%lx\n", rxMode );

	promiscuousMode = enabled;
    DEBUG_LOG("setPromiscuousMode() ..end..\n");

    return kIOReturnSuccess;
}

IOReturn ViaRhine::setMulticastMode( bool enabled )
{
    DEBUG_LOG("setMulticastMode() ..begin..\n");

	if ( enabled ) {
		DEBUG_LOG(" enabling multicast mode... \n ");
		csrWrite8( RxConfig, rx_thresh | 0x0c );
	}else{
		// what to do ?
	}

    DEBUG_LOG("setMulticastMode() ..end..\n");

    return kIOReturnSuccess;
}

static inline UInt32 ether_crc( int length, const unsigned char * data )
{
    static unsigned const ethernet_polynomial = 0x04c11db7U;
    int crc = -1;

	while (--length >= 0) {
		unsigned char current_octet = *data++;
		for (int bit = 0; bit < 8; bit++, current_octet >>= 1)
			crc = (crc << 1) ^
				((crc < 0) ^ (current_octet & 1) ? ethernet_polynomial : 0);
	}
	return crc;
}

IOReturn ViaRhine::setMulticastList( IOEthernetAddress * addrs, UInt32 count )
{
    DEBUG_LOG("setMulticastList() ..begin..\n");
	DEBUG_LOG("ethernet address : ");
	for ( int i = 0; i < kIOEthernetAddressSize; ++i )
	{
		DEBUG_LOG("%x:", addrs->bytes[i]);
	}
	DEBUG_LOG(", count = %d. \n", count );
	
	if ( count > multicast_filter_limit )
	{
		DEBUG_LOG("Break multicast filter limit, accept all!");
        csrWrite32( MulticastFilter0, 0xffffffff );
        csrWrite32( MulticastFilter1, 0xffffffff );
	}else {
		for ( UInt32 i = 0; i < count; i++, addrs++ )
		{
			int bit = ether_crc(6, (const UInt8 *) addrs) >> 26;
			DEBUG_LOG("multicast bits after crc calc: 0x%8.8x.\n", bit);
			if (bit < 32)
				reg_mar0 |= (1 << bit);
			else
				reg_mar1 |= (1 << (bit - 32));
		}
		
		csrWrite32( MulticastFilter0, reg_mar0 );
		csrWrite32( MulticastFilter1, reg_mar1 );
	}

	csrWrite8( RxConfig, rx_thresh | 0x0c );

    DEBUG_LOG("setMulticastList() ..end..\n");

    return kIOReturnSuccess;
}

void ViaRhine::getPacketBufferConstraints(
                       IOPacketBufferConstraints * constraints ) const
{
    DEBUG_LOG("getPacketBufferConstraints() ..begin..\n");

    constraints->alignStart  = kIOPacketBufferAlign4; // 4 bytes align
    constraints->alignLength = kIOPacketBufferAlign4; // 4 bytes align

    DEBUG_LOG("getPacketBufferConstraints() ..end..\n");
}

IOReturn ViaRhine::getHardwareAddress( IOEthernetAddress * address )
{
    UInt8  bytes[6];

    DEBUG_LOG("getHardwareAddress() ..begin..\n");

    // Fetch the hardware address bootstrapped from EEPROM.
	// Ethernet hardware address is 6 bytes
	for ( int i = 0; i < 6; ++i )
	{
		bytes[i] = csrRead8( StationAddr + i );
	}
	
//#define SWAP_BYTE
#ifdef SWAP_BYTE
    address->bytes[0] = bytes[1];
    address->bytes[1] = bytes[0];
    address->bytes[2] = bytes[3];
    address->bytes[3] = bytes[2];
    address->bytes[4] = bytes[5];
    address->bytes[5] = bytes[4];
#else
    address->bytes[0] = bytes[0];
    address->bytes[1] = bytes[1];
    address->bytes[2] = bytes[2];
    address->bytes[3] = bytes[3];
    address->bytes[4] = bytes[4];
    address->bytes[5] = bytes[5];
#endif

    DEBUG_LOG("getHardwareAddress() ..end..\n");
	return kIOReturnSuccess;
}

IOOutputQueue * ViaRhine::createOutputQueue( void )
{
    DEBUG_LOG("createOutputQueue() ..begin..\n");
    DEBUG_LOG("createOutputQueue() ..end..\n");

	// todo : change to basic queue to get full performance
    return IOGatedOutputQueue::withTarget( this, getWorkLoop() );
}

IOReturn ViaRhine::selectMedium( const IONetworkMedium * medium )
{
    bool success=false;
	MediumIndex mIndex;

    if ( medium == 0 )
	{
		mIndex = MEDIUM_INDEX_AUTO;
        medium = phyGetMediumWithIndex( MEDIUM_INDEX_AUTO );
		DEBUG_LOG( "MEDIUM is 0, need auto negotiation...\n" );
	} else {
		mIndex = medium->getIndex();
		DEBUG_LOG( "Selecting MEDIUM Index is %d\n", mIndex );
	}
	
    if ( medium == 0 )
	{
		DEBUG_LOG( "medium is still NULL, panic!\n" );
        return kIOReturnUnsupported;
	}
	
	DEBUG_LOG( "finally, we get a medium : type=0x%x, speed=%d, flags=0x%x, index=%d\n", 
			medium->getType(), medium->getSpeed(), medium->getFlags(), medium->getIndex() );

	success = phySetMedium( mIndex );
    if (success)
    {
		DEBUG_LOG( "phy set medium ok.\n" );
        
		setCurrentMedium( medium );
		DEBUG_LOG( "current medium setted.\n" );
        
		setLinkStatus( kIONetworkLinkValid );
		DEBUG_LOG( "link status is valid.");
		
        phyReportLinkStatus( true );
		DEBUG_LOG( "reported phy link status!" );
    }

    return success ? kIOReturnSuccess : kIOReturnIOError;
}

const OSString * ViaRhine::newVendorString( void ) const
{
    return OSString::withCString("VIA");
}

const OSString * ViaRhine::newModelString( void ) const
{
    return OSString::withCString(model);
}

// power management supports
IOReturn ViaRhine::registerWithPolicyMaker( IOService * policyMaker )
{
    enum {
        kPowerStateOff = 0,
        kPowerStateOn,
        kPowerStateCount
    };

    static IOPMPowerState powerStateArray[ kPowerStateCount ] =
    {
        { 1,0,0,0,0,0,0,0,0,0,0,0 },
        { 1,IOPMDeviceUsable,IOPMPowerOn,IOPMPowerOn,0,0,0,0,0,0,0,0 }
    };

    IOReturn ret;

    ret = policyMaker->registerPowerDriver( this, powerStateArray,
                                            kPowerStateCount );
    
    return ret;
}

IOReturn ViaRhine::setPowerState( unsigned long powerStateOrdinal,
                                 IOService *   policyMaker )
{
	// todo : power saving
    // todo : add support for wake on LAN (magic packet)

    DEBUG_LOG("setPowerState state %d\n", powerStateOrdinal);

    return IOPMAckImplied;
}
