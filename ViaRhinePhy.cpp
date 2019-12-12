// -----------------------------------------------------------------------------
//  Copyright (c) 2006, Yang Wu
//  All rights reserved.
//  Redistribute or use following BSD License.
// -----------------------------------------------------------------------------
//
// project via rhine nic driver
//   ViaRhine.h
//   This file is PHY part of Enthernet Controller class for via rhine driver
//   
// Yang Wu 2006-3 @ Shanghai, China
// Email  : pinxue@hotmail.com
// Web   : http://www.pinxue.net
// -----------------------------------------------------------------------------

#include "ViaRhine.h"
#include "mii.h"

//   Create IONetworkMedium dictionary and quick lookup table (array)
bool ViaRhine::phyAddMediumType( IOMediumType type, UInt32 bps, MediumIndex  index )
{	
	IONetworkMedium	* medium;
	bool              ret = false;
	
	medium = IONetworkMedium::medium( type, bps, 0, index );
	if ( medium )
    {
		ret = IONetworkMedium::addMedium( mediumDict, medium );
		if (ret) 
		{
			mediumTable[index] = medium;
/*			
			DEBUG_LOG("registering medium: index=%d, type=0x%x, speed=%d\n",
				index, type, bps  );
			DEBUG_LOG("generated   medium: index=%d, type=0x%x, speed=%d\n",
				medium->getIndex(), medium->getType(), medium->getSpeed()  );
*/
		}else{
			DEBUG_LOG("failed to add medium: index=%d, type=0x%x, speed=%d\n",
				index, type, bps  );
		}
		medium->release();
	}else{
		DEBUG_LOG("failed to create IONetworkMedium!\n");
	}

	return ret;
}

void ViaRhine::enableLinkMon(void)
{

//	DEBUG_LOG( "enabling link mon ...\n" );
	csrWrite8( MIICmd, 0 );
	csrWrite8( MIIRegAddr, MII_BMSR );
	csrWrite8( MIICmd, 0x80 );
	
	IOSleep(50);
	//while( ! ( csrRead8( MIIRegAddr ) & 0x20 ) );
	
	csrWrite8( MIIRegAddr, MII_BMSR | 0x40 );
}

void ViaRhine::disableLinkMon(void)
{
//	DEBUG_LOG( "disabling link mon ...\n" );
	csrWrite8( MIICmd, 0 );
	
	if ( isRhineI )
	{
		csrWrite8( MIIRegAddr, 0x01 ); // MII_BMSR
		csrWrite8( MIICmd, 0x80 ); // 0x80 must be set immediatedly before turning it off
		
		// while( ! ( csrRead8( MIIRegAddr ) & 0x20 ) ); // wait
		IOSleep(50);
		
		csrWrite8( MIICmd, 0 ); // clean again
	} else {
		// while( ! ( csrRead8( MIIRegAddr ) & 0x80 ) ); // wait
		IOSleep(50);
	}
}

UInt16 ViaRhine::mdioRead( UInt16 reg )
{
	//DEBUG_LOG( "mdioRead starting ...\n" );
	
	//disableLinkMon();
	
	csrWrite8( MIIPhyAddr, phyId );
	csrWrite8( MIIRegAddr, reg );
	csrWrite8( MIICmd, 0x40 ); // Trigger read
	
	// while( ! ( csrRead8( MIICmd ) & 0x40 ) ); // wait
	IOSleep(50);
	
	UInt16 result = csrRead16( MIIData );
	
//	DEBUG_LOG( "mdioReaded: reg=0x%x, result=0x%x\n", reg, result );
	
	//enableLinkMon();
	
	return result;
}

void ViaRhine::mdioWrite( UInt16 reg, UInt16 value )
{
	//DEBUG_LOG( "mdioWrite starting ...\n" );
	
	//disableLinkMon();
	
	csrWrite8( MIIPhyAddr, phyId );
	csrWrite8( MIIRegAddr, reg );
	csrWrite16( MIIData, value );
	csrWrite8( MIICmd, 0x20 ); // Trigger write
	
	// while( ! ( csrRead8( MIICmd ) & 0x40 ) ); // wait
	IOSleep(50);
	
	//enableLinkMon();
	
//	DEBUG_LOG("mdioWrited to reg=0x%x, value=0x%x!\n", reg, value);
}

//   Check PHY capabilities and register all supported media.
#define kMbScale 1000000 
void ViaRhine::phyProbeMediaCapability( void )
{
	reg_bms = mdioRead( MII_BMSR );
	
	DEBUG_LOG( "register auto negotiation medium.\n" );
    phyAddMediumType( kIOMediumEthernetAuto,
                      0, MEDIUM_INDEX_AUTO );

	if ( reg_bms & BMS_10HALF )
    {
		DEBUG_LOG( "supporting 10BaseT Half Duplex medium.\n" );
        phyAddMediumType( kIOMediumEthernet10BaseT |
                          kIOMediumOptionHalfDuplex,
                          10 * kMbScale, MEDIUM_INDEX_10_HD );
    }

    if ( reg_bms & BMS_10FULL )
    {
		DEBUG_LOG( "supporting 10BaseT Full Duplex medium.\n" );
        phyAddMediumType( kIOMediumEthernet10BaseT |
                          kIOMediumOptionFullDuplex,
                          10 * kMbScale, MEDIUM_INDEX_10_FD );
    }

    if ( reg_bms & BMS_100HALF )
    {
		DEBUG_LOG( "supporting 100BaseT Half Duplex medium.\n" );
        phyAddMediumType( kIOMediumEthernet100BaseTX |
                          kIOMediumOptionHalfDuplex,
                          100 * kMbScale, MEDIUM_INDEX_TX_HD);
    }

    if ( reg_bms & BMS_100FULL )
    {
		DEBUG_LOG( "supporting 100BaseT Full Duplex medium.\n" );
        phyAddMediumType( kIOMediumEthernet100BaseTX |
                          kIOMediumOptionFullDuplex,
                          100 * kMbScale, MEDIUM_INDEX_TX_FD );
    }

}

bool ViaRhine::phyReset( void )
{
	return true;
}

bool ViaRhine::phyWaitForAutoNegotiation( void )
{
    for ( SInt32 timeout = 1500; timeout > 0; --timeout)
    {
		if ( mdioRead( MII_BMSR ) & BMS_ANEGCOMPLETE )
		{
			return true;
		}
        IOSleep( 20 );
    }

    return false;
}

bool ViaRhine::phySetMedium( MediumIndex mediumIndex )
{
    UInt16  control;
    bool    success = false;
	
	DEBUG_LOG("setting medium, index=%d\n", mediumIndex);

    if ( mediumIndex == currentMediumIndex )
    {
		DEBUG_LOG("medium never changed!\n");
        return true;  // no change
    }

    switch ( mediumIndex )
    {
        case MEDIUM_INDEX_AUTO:

            // todo : is it real needed? => FIXME: set advertisement register.
			
			DEBUG_LOG("starting auto negotiation ...\n");

            // Turn on and restart auto-negotiation.
			mdioWrite( MII_BMCR, BMC_ANENABLE | BMC_ANRESTART );

            if ( phyWaitForAutoNegotiation() )
			{
				DEBUG_LOG("auto negotiation finished!\n");
			}else{
				DEBUG_LOG("auto negotiation failed!\n");
				DEBUG_LOG("MII_BMCR is 0x%x, MII_BMSR is 0x%x.\n",
						mdioRead( MII_BMCR ), mdioRead( MII_BMSR )  );
			}

            success = true;
            break;
        
        case MEDIUM_INDEX_10_HD:
        case MEDIUM_INDEX_10_FD:
        case MEDIUM_INDEX_TX_HD:
        case MEDIUM_INDEX_TX_FD:
        case MEDIUM_INDEX_T4:
            
            // todo : Check if the selection is supported.

            control = 0; // new PHY control register value

			if (( mediumIndex == MEDIUM_INDEX_TX_HD ) ||
                ( mediumIndex == MEDIUM_INDEX_TX_FD ) ||
                ( mediumIndex == MEDIUM_INDEX_T4    ))
            {
				control |= BMC_SPEED100;
            }

			if (( mediumIndex == MEDIUM_INDEX_10_FD ) ||
                ( mediumIndex == MEDIUM_INDEX_TX_FD ))
            {
				control |= BMC_FULLDUPLEX;
            }

			mdioWrite( MII_BMCR, control );
            success = true;
            break;

        case MEDIUM_INDEX_NONE:
            phyReset();
            success = true;
            break;

        default:
            break;
    }

    if ( success ) currentMediumIndex = mediumIndex;

	return success;
}

bool ViaRhine::phySetMedium( const IONetworkMedium * medium )
{
    return phySetMedium( medium->getIndex() );
}

// check and report link status
void ViaRhine::phyReportLinkStatus( bool forceStatusReport )
{
    UInt16       phyStatus;
    UInt16       linkChanged;
    MediumIndex  activeMediumIndex;
	
//	DEBUG_LOG("reporting link status ...\n");

    phyStatus = mdioRead( MII_BMSR );

    linkChanged = ( phyStatus ^ phyStatusLast ) &
                  ( BMS_LSTATUS | BMS_ANEGCOMPLETE ) ;

    if ( linkChanged || forceStatusReport )
    {
		DEBUG_LOG( "Link changed or force status report!\n" );

        if ( phyStatus & BMS_LSTATUS )
        {
            UInt8  mediaStatus = csrRead8( MIIStatus );
            UInt16 modeControl = mdioRead( MII_BMCR );

            if ( mediaStatus & MiiSpeed ) // 1 means 10M
            {
                if ( modeControl & BMC_FULLDUPLEX )
                    activeMediumIndex = MEDIUM_INDEX_10_FD;    
                else
                    activeMediumIndex = MEDIUM_INDEX_10_HD;
            }
            else
            {
                if ( modeControl & BMC_FULLDUPLEX )
                    activeMediumIndex = MEDIUM_INDEX_TX_FD;  
                else
                    activeMediumIndex = MEDIUM_INDEX_TX_HD;
            }

            setLinkStatus( kIONetworkLinkValid | kIONetworkLinkActive,
                           phyGetMediumWithIndex( activeMediumIndex ) );
        }
        else
        {
            DEBUG_LOG( "link down?\n" );
            setLinkStatus( kIONetworkLinkValid );
        }

        phyStatusLast = phyStatus;
    }
}

const IONetworkMedium * ViaRhine::phyGetMediumWithIndex( MediumIndex index ) const
{
	DEBUG_LOG("Retrieve medium, Medium Index is %d\n", index);
    if ( index < MEDIUM_INDEX_COUNT )
	{
		DEBUG_LOG("mediumTable[%d]: index=%d, type=0x%x, speed=%d\n",
			index, 
			mediumTable[index]->getIndex(), 
			mediumTable[index]->getType(), 
			mediumTable[index]->getSpeed()    );
        return mediumTable[index];
    }else{
		DEBUG_LOG("Medium Index out of range!\n");
        return 0;
	}
}
