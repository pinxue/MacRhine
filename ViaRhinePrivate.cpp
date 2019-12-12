// -----------------------------------------------------------------------------
//  Copyright (c) 2006, Yang Wu
//  All rights reserved.
//  Redistribute or use following BSD License.
// -----------------------------------------------------------------------------
//
// project via rhine nic driver
//   ViaRhine.h
//   This file is i/o part of Enthernet Controller class for via rhine driver
//   
// Yang Wu 2006-3 @ Shanghai, China
// Email : pinxue@hotmail.com
// Web   : http://www.pinxue.net
// -----------------------------------------------------------------------------

#include "ViaRhine.h"
#include "mii.h"

void ViaRhine::dumpRegisters( void )
{
	DEBUG_LOG( " -- register dumping ------------------ \n " );
	for ( int i = 0; i < 0x7F; ++i )
	{
		DEBUG_LOG( " %02x", csrRead8( i ) );
	}
	DEBUG_LOG( "\n" );
	DEBUG_LOG( " -------------------------------------- \n " );
}

// this is a part of interrupt handler
// handle transmit logics
void ViaRhine::transmitterInterrupt( bool * reclaimed )
{
    UInt32  tx_status;
	
	UInt8	entry = tx_dirty % TX_RING_SIZE;
	
	while ( tx_dirty != tx_cur )
	{
		tx_status = (tx_ring+entry)->tx_status;
		DEBUG_LOG( "Tx scavenge %d status %8.8x, cur=%d, dirty=%d.\n", entry, tx_status, tx_cur, tx_dirty );
		if ( tx_status & DescOwn )
			break;
		if ( tx_status & TxOk ) // TxOk==1 means tx error occured
		{
			DEBUG_LOG("%s: Transmit error, Tx status %8.8x.\n",
				getName(), tx_status );
			BUMP_NET_COUNTER( outputErrors );
			if ( tx_status & TxCsr ) BUMP_ETHER_COUNTER( carrierSenseErrors );
			if ( tx_status & TxOwc ) BUMP_ETHER_COUNTER( lateCollisions );
			if ( tx_status & TxAbt ) BUMP_ETHER_COUNTER( missedFrames );
			if ( tx_status & TxDfr ) BUMP_ETHER_COUNTER( deferredTransmissions );
			if ( ( isRhineI && tx_status & TxUdf ) ||
					( tx_status & 0x0800 ) || ( tx_status & 0x1000 ) )
			{
				// todo : bump fifo error counter
				(tx_ring + entry)->tx_status = DescOwn;
				DEBUG_LOG("packet bounced");
				break; // keep data, will try again.
			}
			// transmitter restarted in 'abnormal' handler
		}else{
			if ( isRhineI )
				netStats->collisions += ( tx_status & TxNcr ) >> 3;
			else
				netStats->collisions += tx_status & 0x0F;
			DEBUG_LOG( "Packet Sent, collisions: %1.1x:%1.1x\n", (tx_status >> 3) & 0xF, tx_status & 0xF );
			
			if ( tx_status & DescOwn )
			{
				DEBUG_LOG("warning: tx_status of entry %d is owned by nic yet!\n", entry);
			}
			
			BUMP_NET_COUNTER( outputPackets );
			// todo : bump tx bytes counter
		}
		
		// free the original pkt
		freePacket(  tx_mbuf[entry] );
		tx_mbuf[entry] = 0;
		entry = ( ++ tx_dirty ) % TX_RING_SIZE;
		if ( tx_dirty > tx_cur )
		{
			DEBUG_LOG( "tx_dirty over tx_cur! tx_dirty is %d\n", tx_dirty );
			break;
		}
	}
	
	if ( tx_cur - tx_dirty < TX_QUEUE_LEN -4 )
	{
	  // todo : wake queue
	  transmitQueue->start();
	}
    
    BUMP_ETHER_TX_COUNTER( interrupts );

}


// this is a part of interrupt handler
// handle recieved logics
void ViaRhine::receiverInterrupt( bool * queued )
{
    mbuf_t        pkt;
	UInt16        pkt_len;
	
	UInt8		  entry = rx_cur % RX_RING_SIZE;
	
	DEBUG_LOG("reciever interrupt occured. entry %d, status 0x%8.8x.(rx_cur %d, rx_dirty %d)\n",
			entry, rx_head_desc->rx_status, rx_cur, rx_dirty );
	
	struct rx_desc * desc; UInt32 desc_status; SInt16 data_size;
	while ( ! ( rx_head_desc->rx_status & DescOwn ) )
	{
		desc = rx_head_desc;
		desc_status = desc->rx_status;
		data_size = desc_status >> 16;
		DEBUG_LOG("rx desc status is 0x%8.8x.\n", desc_status);
		
		if ( (desc_status & (RxWholePkt | RxErr)) != RxWholePkt )
		{
			if ( (desc_status & RxWholePkt ) != RxWholePkt )
			{
				DEBUG_LOG("Oversized Ethernet frame spanned multiple buffers, entry %#x length %d, status 0x%8.8x!\n",
					entry, data_size, desc_status );
				// bump rx length error stats
				BUMP_ETHER_COUNTER( frameTooLongs );
			} else if ( desc_status & RxErr ) {
				DEBUG_LOG( "Rx error was 0x%8.8x.\n", desc_status);
				BUMP_NET_COUNTER( inputErrors );
				// todo : ensure error count name is right.
				if ( desc_status & 0x0030 ) BUMP_ETHER_COUNTER( frameTooLongs ); // bump rx length error
				if ( desc_status & 0x0048 ) BUMP_ETHER_RX_COUNTER( overruns ); // bump rx fifo error
				if ( desc_status & 0x0004 ) BUMP_ETHER_RX_COUNTER( frameTooShorts ); // bump rx frame error
				if ( desc_status & 0x0002 ) BUMP_ETHER_RX_COUNTER( phyErrors ); // bump rx crc error with irq lock
			}
			rx_head_desc->rx_status = DescOwn;
		}else{
			// single buffer packet
		
			pkt_len = data_size - 4 ; // skip crc bytes
			DEBUG_LOG( "Recieving single buffer packet, length=%d\n", pkt_len);
						
			pkt = allocatePacket( pkt_len );

			if ( 0 == pkt )
			{
				BUMP_ETHER_RX_COUNTER( resourceErrors );
				DEBUG_LOG("descriptor lost mbuf!\n");
			}else{
				mbuf_setlen( pkt, pkt_len );
				// bcopy( rx_buf[entry], mbuf_data(pkt), pkt_len );
				bcopy( rx_buf[entry], mbuf_data(pkt), pkt_len );

				// DEBUG DUMPING BEGIN
				/*
				{
				char buf[1024*3+1];
				char str[] = "0123456789ABCDEF";
				int i;
				for ( i = 0; i < ( pkt_len < 1024 ? pkt_len : 1024 ); ++i )
				{
					unsigned char c = *(rx_buf[entry]+i);
					buf[i*3]   = str[ c >> 4 ]; 
					buf[i*3+1] = str[ c & 0xF];
					buf[i*3+2] = ' ';
				}
				buf[i] = 0;
				DEBUG_LOG( "recv packet:\n----\n" );
				DEBUG_LOG( buf );
				DEBUG_LOG( "\n----\n");
				}
				*/
				// DEBUG DUMPING END
				
                netif->inputPacket( pkt, pkt_len, 
						IONetworkInterface::kInputOptionQueuePacket );
				BUMP_NET_COUNTER( inputPackets );
				*queued = true;
			}
			
			rx_head_desc->rx_status = DescOwn;
		} // else of if ! RxWholePkt
		
		entry = ( ++ rx_cur ) % RX_RING_SIZE;
		rx_head_desc = rx_ring+entry;
	} // while !DescOwn
	
	BUMP_ETHER_RX_COUNTER( interrupts );
}

//---------------------------------------------------------------------------

void ViaRhine::restartReceiver( void )
{

	DEBUG_LOG( " restart receiver ..begin..\n" );
    
	//csrWrite16( ChipCmd, cmd | CmdStart | CmdRxOn | CmdTxOn | (Cmd1NoTxPoll << 9) );

    UInt16 cmd = csrRead16( ChipCmd );
	csrWrite16( ChipCmd, cmd & (~CmdRxOn) );
    IOSleep(10);

    csrWrite32( RxRingPtr, rx_ring_dma );
	csrWrite16( ChipCmd, cmd | CmdRxOn );
	
	// to be ensured:
    csrWrite8( RxConfig, rx_thresh | AllowSmall | AllowMulticast | AllowBroadcast | AllowPhysical );
	
	DEBUG_LOG( " restart receiver ..end..\n" );

}

//---------------------------------------------------------------------------

void ViaRhine::enableHardwareInterrupts( void )
{
	DEBUG_LOG("enabling hardware interrupts!\n");
    csrWrite16( IntrEnable, IntrRxDone | IntrRxErr     | IntrRxEmpty  | IntrRxOverflow
	       | IntrRxDropped | IntrRxNoBuf  | IntrTxAborted 
	       | IntrTxDone    | IntrTxError  | IntrTxUnderrun
	       | IntrPCIErr    | IntrStatsMax | IntrLinkChange
	 );

    interruptEnabled = true;
}

void ViaRhine::disableHardwareInterrupts( void )
{
    csrWrite16( IntrEnable, 0); 
    interruptEnabled = false;
}

//---------------------------------------------------------------------------

bool ViaRhine::allocateDescriptorMemory( void )
{
    IOByteCount len;
	int i;
	IOPhysicalAddress next;

    // Allocate receiver memory.
	UInt32 maxSize; 
	getMaxPacketSize( & maxSize );
	rx_buf_sz = (maxSize<= 1500 ? PKT_BUF_SZ : maxSize + 32);
    rx_md = IOBufferMemoryDescriptor::withOptions( 
                      /* options   */ kIOMemoryPhysicallyContiguous,
                      /* capacity  */ RX_RING_SIZE * sizeof( struct rx_desc ),
                      /* alignment */ PAGE_SIZE );

    if ( 0 == rx_md || rx_md->prepare() != kIOReturnSuccess )
    {
        IOLog("%s: Can't allocate %d contiguous bytes\n",
              getName(), RX_RING_SIZE * sizeof( struct rx_desc )  );
        return false;
    }

    rx_ring = (struct rx_desc *) rx_md->getBytesNoCopy();
    rx_ring_dma  = rx_md->getPhysicalSegment( 0, &len );

    // DEBUG_LOG("Rx descriptor ring buffer: len = %d virt = %p phys = 0x%lx\n",
    //          rx_md->getCapacity(), rx_ring, rx_ring_dma);
	
	// init rx descriptors and link them as a ring
	rx_cur = rx_dirty = 0;
	rx_desc * rd = rx_ring;
	next = rx_ring_dma;
	for ( i = 0; i < RX_RING_SIZE; ++i, ++rd )
	{
		rd->rx_status = 0;
		rd->desc_length = rx_buf_sz;
		next += sizeof(struct rx_desc);
		rd->next_desc = next;
	}
	(rx_ring + i - 1)->next_desc = rx_ring_dma;	
	DEBUG_LOG( "last rx descriptor next_desc pointing to %p, rx_ring_dma is %p\n",
				( rx_ring + RX_RING_SIZE - 1 )->next_desc, rx_ring_dma );
	
	// create rx buffers
	for ( i = 0; i < RX_RING_SIZE; ++i )
	{
			rx_buf_md[i] = IOBufferMemoryDescriptor::withOptions(
							kIOMemoryPhysicallyContiguous, // options
							rx_buf_sz, // capacity
							PAGE_SIZE // alignment 
						);
			if ( 0 == rx_buf_md[i] || rx_buf_md[i]->prepare() != kIOReturnSuccess )
			{
				IOLog("%s: Can't allocate Rx descriptor buffer\n", getName());
				// todo : rx_md->complete(); // release here or at clean-up routine?
				return false;
			}
			rx_buf[i] = (UInt8 *) rx_buf_md[i]->getBytesNoCopy();
			rx_buf_dma[i] = rx_buf_md[i]->getPhysicalSegment( 0, &len );
		
		(rx_ring+i)->addr = rx_buf_dma[i];
		(rx_ring+i)->rx_status = DescOwn;
	}
	rx_dirty = (unsigned int) (i - RX_RING_SIZE);
	rx_head_desc = rx_ring;

    // Allocate transmitter memory.
	tx_md = IOBufferMemoryDescriptor::withOptions(
				0, //options
				TX_RING_SIZE * sizeof(struct tx_desc), // capacity
				PAGE_SIZE // alignment
			);
	
	if ( 0 == tx_md || tx_md->prepare() != kIOReturnSuccess )
	{
		IOLog("%s: Can't allocate Tx descriptor buffer\n", getName());
		return false;
	}
	
	tx_ring = (struct tx_desc *) tx_md->getBytesNoCopy();
	tx_ring_dma = tx_md->getPhysicalSegment( 0, &len );
	
    //DEBUG_LOG("Tx descriptor ring buffer: len = %d virt = %p phys = 0x%lx\n",
    //          tx_md->getCapacity(), tx_ring, tx_ring_dma);
	
	// todo : in linux, via-rhine driver use physical address of sk_buff.data directly for RhineII and later
	//        it seems we cannot use m_buf this way?
	// if ( isRhineI )
	{
		for ( i = 0; i < TX_RING_SIZE; ++i )
		{
			tx_buf_md[i] = IOBufferMemoryDescriptor::withOptions(
							0, // options
							PKT_BUF_SZ, // capacity
							PAGE_SIZE // alignment 
						);
			if ( 0 == tx_buf_md[i] || tx_buf_md[i]->prepare() != kIOReturnSuccess )
			{
				IOLog("%s: Can't allocate Tx buffer for RhineI\n", getName());
				// todo : tx_md->complete(); // release here or at clean-up routine?
				return false;
			}
			tx_buf[i] = (UInt8 *) tx_buf_md[i]->getBytesNoCopy();
			tx_buf_dma[i] = tx_buf_md[i]->getPhysicalSegment( 0, &len );

			//DEBUG_LOG("Tx buffer: len = %d virt = %p phys = 0x%lx\n",
			//		tx_buf_md[i]->getCapacity(), tx_buf[i], tx_buf_dma[i]);
		}
	}

	// init transmit descriptors and chain them as a ring
	next = tx_ring_dma;
	tx_desc * desc = tx_ring;
	for ( i = 0; i < TX_RING_SIZE; ++i, ++desc )
	{
		desc->tx_status = 0;
		desc->desc_length = TXDESC | PKT_BUF_SZ;
		desc->addr = tx_buf_dma[i];
		next += sizeof( struct tx_desc );
		desc->next_desc = next;
		tx_mbuf[i] = 0;
	}
	(--desc)->next_desc = tx_ring_dma;
	
	tx_dirty = tx_cur = 0;

    return true;
}

//---------------------------------------------------------------------------
void ViaRhine::reloadEEPRom( void )
{
	csrWrite8( MACRegEEcsr, 0x20 );
	while( 0 == csrRead8( MACRegEEcsr ) & 0x20 )
	  IOSleep(5);
	// todo : re-enable mmio
	// todo : turn off EEPROM controlled wake-up 
	// if ( rqWOL )
	// csrWrite8( ConfigA, csrRead8( ConfigA ) & 0xFC );
}
 
bool ViaRhine::initAdapter( IOOptionBits options )
{
    DEBUG_LOG("initAdapter() ..begin..\n");

    disableHardwareInterrupts();

    // Issue a software reset.
	csrWrite8( ChipCmd1, Cmd1Reset);
	// todo : sync
	IOSleep(5);
	
	UInt8 resetState = csrRead8( ChipCmd1 );
	if (! (resetState & Cmd1Reset) )
	{
		DEBUG_LOG("force chip reset!\n");
		// force reset
		// todo : chip identify, only via rhine ii or later support this feature
		csrWrite8(MiscCmd, 0x40);
		IOSleep(5);
	}


    // If all thats needed is a chip reset, then we're done.

    if ( options & kResetChip )
	{
		DEBUG_LOG( " chip reset ok.\n" );
		DEBUG_LOG( "initAdapter() ..end..\n" );
		return true;
	}

	// todo : power init 

    // Restore EEPROM Defaults
    reloadEEPRom();
	
	UInt8 n=0;
	if (isRhineI) {
		/* More recent docs say that this bit is reserved ... */
		n = csrRead8( ConfigA ) | 0x20;
		csrWrite8( ConfigA, n );
	} else {
		n = csrRead8( ConfigD ) | 0x80;	// GPIOEN
		csrWrite8( ConfigD, n);
	}
    csrWrite8( ConfigA, csrRead8( ConfigA ) & 0xFC );

	csrWrite16( PCIBusConfig, 0x0006 );

    // Clear the multicast hash.

    //reg_mar0 = reg_mar1 = 0;
    csrWrite32( MulticastFilter0, reg_mar0 );
    csrWrite32( MulticastFilter1, reg_mar1 );
	
    // Update the physical address of the receiver buffer.
    csrWrite32( RxRingPtr, rx_ring_dma + (rx_cur % RX_RING_SIZE) * sizeof( struct rx_desc ) );

    // Update the physical address of the transmit buffers.
    csrWrite32( TxRingPtr, tx_ring_dma + (tx_cur % TX_RING_SIZE) * sizeof( struct tx_desc ) );

    // transmit configuration register.
    // all defaults with threshold is ok.
	csrWrite8( TxConfig, tx_thresh );

    // receive configuration register.
    csrWrite8( RxConfig, rx_thresh | AllowSmall | AllowMulticast | AllowBroadcast | AllowPhysical );
	
    // Enable transmitter and receiver.   
	DEBUG_LOG( "default ChipCmd is 0x%x, 0x%x.\n", csrRead8( ChipCmd ), csrRead8( ChipCmd1 ) ); 
    csrWrite16( ChipCmd, csrRead16( ChipCmd ) | CmdStart | CmdRxOn | CmdTxOn | (Cmd1NoTxPoll << 9) );
	
    DEBUG_LOG("initAdapter() ..end..\n");

    return true;
}


// Interrupt handler
void ViaRhine::interruptOccurred(OSObject * client, IOInterruptEventSource * src, int count )
{
    bool    flushInputQ    = false;
    bool	serviceOutputQ = false;
	UInt32	intrStatus;
    //IODebuggerLockState lockState;
	int     boguscnt = 20;
	
	DEBUG_LOG("interruptOccured() ..bein..\n");

    // PCI drivers must be prepared to handle spurious interrupts when the
    // interrupt line is shared with other devices. The interruptEnabled
    // flag prevents the driver from touching hardware before it is ready. 

    if ( false == interruptEnabled ) return;

    //lockState = IODebuggerLock( this );

    do{
		// get intr status
		intrStatus = csrRead16( IntrStatus );
		if ( statusWBRace )
			intrStatus |= csrRead8( IntrStatus2 ) << 16;
		
        if ( ( intrStatus & 0x08FFFF ) == 0 )
        {
			//DEBUG_LOG("not my intr? status is 0x%x.\n", intrStatus);
            // May want to consider a filter interrupt source when
            // sharing interrupts to avoid scheduling the work loop.
            break;  // exit interrupt loop...
        }

		// ack all current intr source ASAP
		if ( intrStatus & IntrTxDescRace )
		{
			IOLog( "Tx Desc Race Intr occured!\n" );
			csrWrite8( IntrStatus2, 0x08 );
		}
		csrWrite16( IntrStatus, intrStatus & 0xffff );

		csrRead8( StationAddr ); // just IOSync : beware of pci posted writes
		
		//DEBUG_LOG( "Interrupt, status %8.8x.\n", intrStatus );

        // Transmitter OK, or error due to excessive collisions.

        if ( intrStatus & ( IntrTxDone | IntrTxErrSummary ) )
        {
			if ( intrStatus & IntrTxErrSummary )
			{
				// todo : wait for ! (ChipCmd & CmdTxOn)
				int i = 1024;
				while( (csrRead8( ChipCmd ) & CmdTxOn) && (--i) ) { };
				IOLog( "interupt() Tx engine still on.\n" );
			}
            transmitterInterrupt( &serviceOutputQ );
        }
		

        if ( intrStatus & ( IntrRxDone | IntrRxErr | IntrRxDropped 
							| IntrRxWakeUp | IntrRxEmpty | IntrRxNoBuf ) ) 
        {
            receiverInterrupt( &flushInputQ );
        }

		if ( intrStatus & (IntrPCIErr | IntrLinkChange 
				| IntrStatsMax    | IntrTxError | IntrTxAborted 
				| IntrTxUnderrun  | IntrTxDescRace  ) 		)
		{
			errorInterrupt( intrStatus );
		}

		if ( -- boguscnt < 0 )
		{
			DEBUG_LOG( "too much work at interrupt! status = %8.8x.\n", intrStatus );
		}
    }while( intrStatus );

	// todo : debugger reclaimed buffer support
	
    //IODebuggerUnlock( lockState );

    // Flush all inbound packets and pass them to the network stack.
    // Interrupts are not enabled until the network interface is enabled
    // by BSD, so netif must be valid.

    assert( netif );
    if ( flushInputQ )
	{
		DEBUG_LOG("flush input queue!\n");
		netif->flushInputQueue();
	}

    // Call service() without holding the debugger lock to prevent a
    // deadlock when service() calls our outputPacket() function.

    if ( serviceOutputQ )
	{
		DEBUG_LOG("service output queue!\n");
		transmitQueue->service();
	}

	DEBUG_LOG("interruptOccured() ..end..\n");
}

// this is a part of interrupt handler
// handle any error or notification
void ViaRhine::errorInterrupt( UInt32 intrStatus )
{
	DEBUG_LOG("interrupting to report error, status is 0x%x.\n", intrStatus);
	if ( intrStatus & IntrLinkChange )
	{
		DEBUG_LOG("Link media changed!\n");
		// check media via MII interface
		
		UInt16 bmcr = mdioRead( MII_BMCR );
		if ( bmcr & BMC_FULLDUPLEX )
		{
			DEBUG_LOG("enabling full duplex!\n");
			csrWrite8( ChipCmd1, csrRead8( ChipCmd1 ) | Cmd1FDuplex );
		}else{
			DEBUG_LOG("diabling full duplex!\n");
			csrWrite8( ChipCmd1, csrRead8( ChipCmd1 ) | ~Cmd1FDuplex );
		}
		
		
	}
	
	if ( intrStatus & IntrStatsMax )
	{
		DEBUG_LOG("IntrStatsMax : need clean.\n");
		// bump crc errors with read16(RxCRCErrs)
		// bump missed error with read16(RxMissed)

		// clear tally counters
/* to be ensured: is is real needed?
 * Clears the "tally counters" for CRC errors and missed frames(?).
 * It has been reported that some chips need a write of 0 to clear
 * these, for others the counters are set to 1 when written to and
 * instead cleared when read. So we clear them both ways ...
 */
		csrWrite32( RxMissed, 0 );
		csrRead16( RxCRCErrs );
		csrRead16( RxMissed );
	}
	
	if ( intrStatus & IntrTxAborted )
	{
		DEBUG_LOG("Abort %8.8x, frame dropped.\n", intrStatus);
	}
	
	if ( intrStatus & IntrTxUnderrun )
	{
		if ( tx_thresh < 0xE0 )
			csrWrite8( TxConfig, tx_thresh += 0x20 );
		DEBUG_LOG("Transmitter underrun, tx threshold now %2.2x.\n", tx_thresh);
	}
	
	if ( intrStatus & IntrTxDescRace)
	{
		DEBUG_LOG("Tx descriptor write-back race.\n");
	}
	
	if ( ( intrStatus & IntrTxError )
		&& 0==( intrStatus & ( IntrTxAborted | IntrTxUnderrun | IntrTxDescRace ) ) )
	{
		if ( tx_thresh < 0xE0 )
			csrWrite8( TxConfig, tx_thresh += 0x20 );
		DEBUG_LOG("Unspecified error, tx threshold now %2.2x.\n", tx_thresh);
	}
	
	if ( intrStatus & ( IntrTxAborted | IntrTxUnderrun | IntrTxDescRace | IntrTxError ) )
	{
		// restart tx
		if ( (intrStatus & IntrTxErrSummary) == 0 )
		{
			UInt8 entry = tx_dirty % TX_RING_SIZE;
			csrWrite32( TxRingPtr, tx_ring_dma + entry * sizeof( struct tx_desc ) );
			csrWrite8( ChipCmd, csrRead8( ChipCmd ) | CmdTxOn );
			csrWrite8( ChipCmd1, csrRead8( ChipCmd1 ) | Cmd1TxDemand );
			// todo : IOSYNC
		}else{
			DEBUG_LOG("restarting tx, Another error occured %8.8x.\n", intrStatus);
		}
	}
	
	if ( intrStatus & ~(IntrLinkChange | IntrStatsMax | IntrTxUnderrun |
			    IntrTxError | IntrTxAborted | IntrNormalSummary |
			    IntrTxDescRace) )
	{
		DEBUG_LOG("Something strange happend!\n");
	}
}

// init transmit for a packet
UInt32 ViaRhine::outputPacket( mbuf_t pkt, void * param )
{
    UInt8				entry;
	UInt8 *             des_ptr_ub;
    long		        pkt_len;
    mbuf_t              mn;
    //IODebuggerLockState lockState;

    DEBUG_LOG("outputPacket() ===>\n");

#warning "debug only!"
	DEBUG_LOG("tx registers: Cmd=0x%x, Cmd1=0x%x, TxConf=0x%x, TxRingPtr=0x%x\n",
		csrRead8( ChipCmd ), csrRead8( ChipCmd1 ),
		csrRead8( TxConfig ), csrRead32( TxRingPtr )
	);
	

    //lockState = IODebuggerLock( this );
	
	entry = tx_cur % TX_RING_SIZE;

	DEBUG_LOG("enter : tx_cur is %d, tx_dirty is %d, entry is %d, entry.tx_status is 0x%x, own is 0x%x\n",
		tx_cur, tx_dirty, entry, (tx_ring+entry)->tx_status, (tx_ring+entry)->tx_status&DescOwn );

    if ( (tx_ring+entry)->tx_status & DescOwn )
    {
        // Stall the output queue until the ack by the interrupt handler.
        IOLog("%s : stall output queue until intr ack.\noutputPacket() <===\n", getName());
        //IODebuggerUnlock( lockState );
        return kIOReturnOutputStall;
    }

    des_ptr_ub = tx_buf[entry];
	tx_mbuf[entry] = pkt;
	
	if ( ( tx_ring + entry )->addr != tx_buf_dma[entry] )
	{
		IOLog("fatal error: descriptor lost buffer addr\n" );
		return kIOReturnVMError;
	}else{
		DEBUG_LOG(" entry %d, descriptor %x, buffer addr %x\n",
			entry, tx_ring+entry, (tx_ring+entry)->addr        );
	}

    pkt_len = mbuf_pkthdr_len(pkt);
	
	// todo : ensure chip understand mbuf or not
	//        in linux driver, the sk_buff is direct passed to chip except RhineI
	if ( pkt_len > PKT_BUF_SZ )
	{
		DEBUG_LOG("drop a large pkt, size is %d bytes\n", pkt_len);
		freePacket(pkt);
		BUMP_NET_COUNTER(outputErrors);
		BUMP_ETHER_COUNTER(frameTooLongs);
		return kIOReturnOutputDropped;
	}

    for ( mn = pkt; mn; mn = mbuf_next(mn) )
    {
        bcopy( mbuf_data(mn), des_ptr_ub, mbuf_len(mn) );
        des_ptr_ub += mbuf_len(mn);
    }

    // Software padding of small frames. Hardware doesn't do this.

    if ( pkt_len < MINPACK )
    {
        memset( des_ptr_ub, 0x55, MINPACK - pkt_len );
        pkt_len = MINPACK;
    }

				// DEBUG DUMPING BEGIN
				/*
				{
				char buf[1024*3+1];
				char str[] = "0123456789ABCDEF";
				int i;
				for ( i = 0; i < ( pkt_len < 1024 ? pkt_len : 1024 ); ++i )
				{
					unsigned char c = *( des_ptr_ub+i);
					buf[i*3]   = str[ c >> 4 ]; 
					buf[i*3+1] = str[ c & 0xF];
					buf[i*3+2] = ' ';
				}
				buf[i] = 0;
				DEBUG_LOG( "send packet:\n-----\n" );
				DEBUG_LOG( buf );
				DEBUG_LOG( "\n-------\n" );
				}
				*/
				// DEBUG DUMPING END

    // it seems via rhine loop the descriptor ring automatically 

	(tx_ring+entry)->tx_status = DescOwn;
	(tx_ring+entry)->desc_length = TxDescIC | (pkt_len & 0x7FF); // bit[0~10] is length

	++tx_cur;

	DEBUG_LOG("ready : tx_cur is %d, tx_dirty is %d, entry is %d, entry.tx_status is 0x%x, own is 0x%x\n",
		tx_cur, tx_dirty, entry, (tx_ring+entry)->tx_status, (tx_ring+entry)->tx_status&DescOwn );

	// ask chip to transmit
	csrWrite8( ChipCmd1, csrRead8( ChipCmd1 ) | Cmd1TxDemand );
	
	if ( tx_cur == tx_dirty + TX_QUEUE_LEN )
	{
		IOLog( "exceed tx queue limit! tx_cur is %d, dirty_tx is %d, queue len is %d\n",
			tx_cur, tx_dirty, TX_QUEUE_LEN );
		// todo : stop the queue
	}
	
	DEBUG_LOG("%s: Transmit frame #%d queued in slot %d.\n",
		getName(), tx_cur-1, entry  );

	//IODebuggerUnlock( lockState );

    // freePacket( pkt ); // will free at intr handler

    BUMP_NET_COUNTER( outputPackets );

	DEBUG_LOG("outputPacket() <===\n");

    return kIOReturnOutputSuccess;
}

/*
// Send KDP packets in polled mode.
void ViaRhine::sendPacket( void * pkt, UInt32 pkt_len )
{
	// not support poll yet
	DEBUG_LOG("ViaRhine: NOT support poll mode yet!\n");
}

void ViaRhine::reclaimTransmitBuffer( void )
{
	// not support poll yet
	DEBUG_LOG("ViaRhine: NOT support poll mode yet!\n");
}

// Receive (KDP) packets in polled mode. 
void ViaRhine::receivePacket( void * pkt, UInt32 * pkt_len, UInt32 timeout )
{
	// not support poll yet
	DEBUG_LOG("ViaRhine: NOT support poll mode yet!\n");	
}
*/