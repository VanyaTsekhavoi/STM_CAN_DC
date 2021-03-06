// clang-format off
/*******************************************************************************
    CANopen Object Dictionary definition for CANopenNode v1 to v2

    This file was automatically generated with
    libedssharp Object Dictionary Editor v0.8-122-g6c02323

    https://github.com/CANopenNode/CANopenNode
    https://github.com/robincornelius/libedssharp

    DON'T EDIT THIS FILE MANUALLY !!!!
*******************************************************************************/
// For CANopenNode V2 users, C macro `CO_VERSION_MAJOR=2` has to be added to project options
#ifndef CO_VERSION_MAJOR
 #include "301/CO_driver.h"
 #include "CO_OD.h"
 #include "301/CO_SDOserver.h"
#elif CO_VERSION_MAJOR < 4
 #include "301/CO_driver.h"
 #include "CO_OD.h"
 #include "301/CO_SDOserver.h"
#else
 #error This Object dictionary is not compatible with CANopenNode v4.0 and up!
#endif

/*******************************************************************************
   DEFINITION AND INITIALIZATION OF OBJECT DICTIONARY VARIABLES
*******************************************************************************/


/***** Definition for RAM variables *******************************************/
struct sCO_OD_RAM CO_OD_RAM = {
           CO_OD_FIRST_LAST_WORD,

/*1001*/ 0x0L,
/*1002*/ 0x0000L,
/*1003*/ {0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
/*1010*/ {0x0003L},
/*1011*/ {0x0001L},
/*1280*/ {{0x3L, 0x0000L, 0x0000L, 0x0L}},
/*2100*/ {0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L},
/*2103*/ 0x00,
/*2104*/ 0x00,
/*2107*/ {0x3E8, 0x00, 0x00, 0x00, 0x00},
/*2108*/ {0x00},
/*2109*/ {0x00},
/*2110*/ {0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
/*2120*/ {0x5L, 0x1234567890ABCDEFL, 0x234567890ABCDEF1L, 12.345, 456.789, 0},
/*2130*/ {0x3L, {'-', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '}, 0x00000000L, 0x0000L},
/*6000*/ {0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L},
/*6200*/ {0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L},
/*6401*/ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
/*6411*/ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

           CO_OD_FIRST_LAST_WORD,
};


/***** Definition for ROM variables *******************************************/
struct sCO_OD_ROM CO_OD_ROM = {
           CO_OD_FIRST_LAST_WORD,

/*1000*/ 0x0000L,
/*1005*/ 0x0080L,
/*1006*/ 0x0000L,
/*1007*/ 0x0000L,
/*1008*/ {'C', 'A', 'N', 'o', 'p', 'e', 'n', 'N', 'o', 'd', 'e'},
/*1009*/ {'3', '.', '0', '0'},
/*100A*/ {'3', '.', '0', '0'},
/*1012*/ 0x80000100L,
/*1014*/ 0x0080L,
/*1015*/ 0x64,
/*1016*/ {0x0000L, 0x0000L, 0x0000L, 0x0000L},
/*1017*/ 0x3E8,
/*1018*/ {0x4L, 0x0000L, 0x0001L, 0x0000L, 0x0000L},
/*1019*/ 0x0L,
/*1029*/ {0x0L, 0x0L, 0x1L, 0x0L, 0x0L, 0x0L},
/*1200*/ {{0x2L, 0x0600L, 0x0580L}},
/*1400*/ {{0x2L, 0x0200L, 0xFFL},
/*1401*/ {0x2L, 0x0300L, 0x1L},
/*1402*/ {0x2L, 0x0400L, 0xFEL},
/*1403*/ {0x2L, 0x0500L, 0xFEL}},
/*1600*/ {{0x2L, 0x62000108L, 0x62000208L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
/*1601*/ {0x4L, 0x64110110L, 0x64110210L, 0x64110310L, 0x64110410L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
/*1602*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
/*1603*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L}},
/*1800*/ {{0x6L, 0x0180L, 0xFFL, 0x64, 0x0L, 0x00, 0x0L},
/*1801*/ {0x6L, 0x0280L, 0xFEL, 0x00, 0x0L, 0x00, 0x0L},
/*1802*/ {0x6L, 0x0380L, 0xFEL, 0x00, 0x0L, 0x00, 0x0L},
/*1803*/ {0x6L, 0x0480L, 0xFEL, 0x00, 0x0L, 0x00, 0x0L}},
/*1A00*/ {{0x2L, 0x60000108L, 0x60000208L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
/*1A01*/ {0x1L, 0x64010110L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
/*1A02*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
/*1A03*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L}},
/*1F80*/ 0x0008L,
/*2101*/ 0x30L,
/*2102*/ 0xFA,
/*2111*/ {0x0001L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},

           CO_OD_FIRST_LAST_WORD,
};


/***** Definition for EEPROM variables *******************************************/
struct sCO_OD_EEPROM CO_OD_EEPROM = {
           CO_OD_FIRST_LAST_WORD,

/*2106*/ 0x0000L,
/*2112*/ {0x0001L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},

           CO_OD_FIRST_LAST_WORD,
};




/*******************************************************************************
   STRUCTURES FOR RECORD TYPE OBJECTS
*******************************************************************************/


/*0x1018*/ const CO_OD_entryRecord_t OD_record1018[5] = {
           {(void*)&CO_OD_ROM.identity.maxSubIndex, 0x05, 0x1 },
           {(void*)&CO_OD_ROM.identity.vendorID, 0x85, 0x4 },
           {(void*)&CO_OD_ROM.identity.productCode, 0x85, 0x4 },
           {(void*)&CO_OD_ROM.identity.revisionNumber, 0x85, 0x4 },
           {(void*)&CO_OD_ROM.identity.serialNumber, 0x85, 0x4 },
};

/*0x1200*/ const CO_OD_entryRecord_t OD_record1200[3] = {
           {(void*)&CO_OD_ROM.SDOServerParameter[0].maxSubIndex, 0x05, 0x1 },
           {(void*)&CO_OD_ROM.SDOServerParameter[0].COB_IDClientToServer, 0x85, 0x4 },
           {(void*)&CO_OD_ROM.SDOServerParameter[0].COB_IDServerToClient, 0x85, 0x4 },
};

/*0x1280*/ const CO_OD_entryRecord_t OD_record1280[4] = {
           {(void*)&CO_OD_RAM.SDOClientParameter[0].maxSubIndex, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.SDOClientParameter[0].COB_IDClientToServer, 0xBE, 0x4 },
           {(void*)&CO_OD_RAM.SDOClientParameter[0].COB_IDServerToClient, 0xBE, 0x4 },
           {(void*)&CO_OD_RAM.SDOClientParameter[0].nodeIDOfTheSDOServer, 0x0E, 0x1 },
};

/*0x1400*/ const CO_OD_entryRecord_t OD_record1400[3] = {
           {(void*)&CO_OD_ROM.RPDOCommunicationParameter[0].maxSubIndex, 0x05, 0x1 },
           {(void*)&CO_OD_ROM.RPDOCommunicationParameter[0].COB_IDUsedByRPDO, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOCommunicationParameter[0].transmissionType, 0x0D, 0x1 },
};

/*0x1401*/ const CO_OD_entryRecord_t OD_record1401[3] = {
           {(void*)&CO_OD_ROM.RPDOCommunicationParameter[1].maxSubIndex, 0x05, 0x1 },
           {(void*)&CO_OD_ROM.RPDOCommunicationParameter[1].COB_IDUsedByRPDO, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOCommunicationParameter[1].transmissionType, 0x0D, 0x1 },
};

/*0x1402*/ const CO_OD_entryRecord_t OD_record1402[3] = {
           {(void*)&CO_OD_ROM.RPDOCommunicationParameter[2].maxSubIndex, 0x05, 0x1 },
           {(void*)&CO_OD_ROM.RPDOCommunicationParameter[2].COB_IDUsedByRPDO, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOCommunicationParameter[2].transmissionType, 0x0D, 0x1 },
};

/*0x1403*/ const CO_OD_entryRecord_t OD_record1403[3] = {
           {(void*)&CO_OD_ROM.RPDOCommunicationParameter[3].maxSubIndex, 0x05, 0x1 },
           {(void*)&CO_OD_ROM.RPDOCommunicationParameter[3].COB_IDUsedByRPDO, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOCommunicationParameter[3].transmissionType, 0x0D, 0x1 },
};

/*0x1600*/ const CO_OD_entryRecord_t OD_record1600[9] = {
           {(void*)&CO_OD_ROM.RPDOMappingParameter[0].numberOfMappedObjects, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[0].mappedObject1, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[0].mappedObject2, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[0].mappedObject3, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[0].mappedObject4, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[0].mappedObject5, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[0].mappedObject6, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[0].mappedObject7, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[0].mappedObject8, 0x8D, 0x4 },
};

/*0x1601*/ const CO_OD_entryRecord_t OD_record1601[9] = {
           {(void*)&CO_OD_ROM.RPDOMappingParameter[1].numberOfMappedObjects, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[1].mappedObject1, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[1].mappedObject2, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[1].mappedObject3, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[1].mappedObject4, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[1].mappedObject5, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[1].mappedObject6, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[1].mappedObject7, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[1].mappedObject8, 0x8D, 0x4 },
};

/*0x1602*/ const CO_OD_entryRecord_t OD_record1602[9] = {
           {(void*)&CO_OD_ROM.RPDOMappingParameter[2].numberOfMappedObjects, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[2].mappedObject1, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[2].mappedObject2, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[2].mappedObject3, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[2].mappedObject4, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[2].mappedObject5, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[2].mappedObject6, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[2].mappedObject7, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[2].mappedObject8, 0x8D, 0x4 },
};

/*0x1603*/ const CO_OD_entryRecord_t OD_record1603[9] = {
           {(void*)&CO_OD_ROM.RPDOMappingParameter[3].numberOfMappedObjects, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[3].mappedObject1, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[3].mappedObject2, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[3].mappedObject3, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[3].mappedObject4, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[3].mappedObject5, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[3].mappedObject6, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[3].mappedObject7, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.RPDOMappingParameter[3].mappedObject8, 0x8D, 0x4 },
};

/*0x1800*/ const CO_OD_entryRecord_t OD_record1800[7] = {
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[0].maxSubIndex, 0x05, 0x1 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[0].COB_IDUsedByTPDO, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[0].transmissionType, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[0].inhibitTime, 0x8D, 0x2 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[0].compatibilityEntry, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[0].eventTimer, 0x8D, 0x2 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[0].SYNCStartValue, 0x0D, 0x1 },
};

/*0x1801*/ const CO_OD_entryRecord_t OD_record1801[7] = {
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[1].maxSubIndex, 0x05, 0x1 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[1].COB_IDUsedByTPDO, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[1].transmissionType, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[1].inhibitTime, 0x8D, 0x2 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[1].compatibilityEntry, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[1].eventTimer, 0x8D, 0x2 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[1].SYNCStartValue, 0x0D, 0x1 },
};

/*0x1802*/ const CO_OD_entryRecord_t OD_record1802[7] = {
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[2].maxSubIndex, 0x05, 0x1 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[2].COB_IDUsedByTPDO, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[2].transmissionType, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[2].inhibitTime, 0x8D, 0x2 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[2].compatibilityEntry, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[2].eventTimer, 0x8D, 0x2 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[2].SYNCStartValue, 0x0D, 0x1 },
};

/*0x1803*/ const CO_OD_entryRecord_t OD_record1803[7] = {
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[3].maxSubIndex, 0x05, 0x1 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[3].COB_IDUsedByTPDO, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[3].transmissionType, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[3].inhibitTime, 0x8D, 0x2 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[3].compatibilityEntry, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[3].eventTimer, 0x8D, 0x2 },
           {(void*)&CO_OD_ROM.TPDOCommunicationParameter[3].SYNCStartValue, 0x0D, 0x1 },
};

/*0x1A00*/ const CO_OD_entryRecord_t OD_record1A00[9] = {
           {(void*)&CO_OD_ROM.TPDOMappingParameter[0].numberOfMappedObjects, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[0].mappedObject1, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[0].mappedObject2, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[0].mappedObject3, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[0].mappedObject4, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[0].mappedObject5, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[0].mappedObject6, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[0].mappedObject7, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[0].mappedObject8, 0x8D, 0x4 },
};

/*0x1A01*/ const CO_OD_entryRecord_t OD_record1A01[9] = {
           {(void*)&CO_OD_ROM.TPDOMappingParameter[1].numberOfMappedObjects, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[1].mappedObject1, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[1].mappedObject2, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[1].mappedObject3, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[1].mappedObject4, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[1].mappedObject5, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[1].mappedObject6, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[1].mappedObject7, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[1].mappedObject8, 0x8D, 0x4 },
};

/*0x1A02*/ const CO_OD_entryRecord_t OD_record1A02[9] = {
           {(void*)&CO_OD_ROM.TPDOMappingParameter[2].numberOfMappedObjects, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[2].mappedObject1, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[2].mappedObject2, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[2].mappedObject3, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[2].mappedObject4, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[2].mappedObject5, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[2].mappedObject6, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[2].mappedObject7, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[2].mappedObject8, 0x8D, 0x4 },
};

/*0x1A03*/ const CO_OD_entryRecord_t OD_record1A03[9] = {
           {(void*)&CO_OD_ROM.TPDOMappingParameter[3].numberOfMappedObjects, 0x0D, 0x1 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[3].mappedObject1, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[3].mappedObject2, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[3].mappedObject3, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[3].mappedObject4, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[3].mappedObject5, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[3].mappedObject6, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[3].mappedObject7, 0x8D, 0x4 },
           {(void*)&CO_OD_ROM.TPDOMappingParameter[3].mappedObject8, 0x8D, 0x4 },
};

/*0x2120*/ const CO_OD_entryRecord_t OD_record2120[6] = {
           {(void*)&CO_OD_RAM.testVar.maxSubIndex, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.testVar.I64, 0xBE, 0x8 },
           {(void*)&CO_OD_RAM.testVar.U64, 0xBE, 0x8 },
           {(void*)&CO_OD_RAM.testVar.R32, 0xBE, 0x4 },
           {(void*)&CO_OD_RAM.testVar.R64, 0xBE, 0x8 },
           {(void*)0, 0x0E, 0x0 },
};

/*0x2130*/ const CO_OD_entryRecord_t OD_record2130[4] = {
           {(void*)&CO_OD_RAM.time.maxSubIndex, 0x06, 0x1 },
           {(void*)&CO_OD_RAM.time.string, 0x06, 0x1E },
           {(void*)&CO_OD_RAM.time.epochTimeBaseMs, 0x8E, 0x8 },
           {(void*)&CO_OD_RAM.time.epochTimeOffsetMs, 0xBE, 0x4 },
};

/*******************************************************************************
   OBJECT DICTIONARY
*******************************************************************************/
const CO_OD_entry_t CO_OD[CO_OD_NoOfElements] = {
{0x1000, 0x00, 0x85,  4, (void*)&CO_OD_ROM.deviceType},
{0x1001, 0x00, 0x36,  1, (void*)&CO_OD_RAM.errorRegister},
{0x1002, 0x00, 0xB6,  4, (void*)&CO_OD_RAM.manufacturerStatusRegister},
{0x1003, 0x08, 0x8E,  4, (void*)&CO_OD_RAM.preDefinedErrorField[0]},
{0x1005, 0x00, 0x8D,  4, (void*)&CO_OD_ROM.COB_ID_SYNCMessage},
{0x1006, 0x00, 0x8D,  4, (void*)&CO_OD_ROM.communicationCyclePeriod},
{0x1007, 0x00, 0x8D,  4, (void*)&CO_OD_ROM.synchronousWindowLength},
{0x1008, 0x00, 0x05, 11, (void*)&CO_OD_ROM.manufacturerDeviceName},
{0x1009, 0x00, 0x05,  4, (void*)&CO_OD_ROM.manufacturerHardwareVersion},
{0x100A, 0x00, 0x05,  4, (void*)&CO_OD_ROM.manufacturerSoftwareVersion},
{0x1010, 0x01, 0x8E,  4, (void*)&CO_OD_RAM.storeParameters[0]},
{0x1011, 0x01, 0x8E,  4, (void*)&CO_OD_RAM.restoreDefaultParameters[0]},
{0x1012, 0x00, 0x8D,  4, (void*)&CO_OD_ROM.COB_ID_TIME},
{0x1014, 0x00, 0x85,  4, (void*)&CO_OD_ROM.COB_ID_EMCY},
{0x1015, 0x00, 0x8D,  2, (void*)&CO_OD_ROM.inhibitTimeEMCY},
{0x1016, 0x04, 0x8D,  4, (void*)&CO_OD_ROM.consumerHeartbeatTime[0]},
{0x1017, 0x00, 0x8D,  2, (void*)&CO_OD_ROM.producerHeartbeatTime},
{0x1018, 0x04, 0x00,  0, (void*)&OD_record1018},
{0x1019, 0x00, 0x0D,  1, (void*)&CO_OD_ROM.synchronousCounterOverflowValue},
{0x1029, 0x06, 0x0D,  1, (void*)&CO_OD_ROM.errorBehavior[0]},
{0x1200, 0x02, 0x00,  0, (void*)&OD_record1200},
{0x1280, 0x03, 0x00,  0, (void*)&OD_record1280},
{0x1400, 0x02, 0x00,  0, (void*)&OD_record1400},
{0x1401, 0x02, 0x00,  0, (void*)&OD_record1401},
{0x1402, 0x02, 0x00,  0, (void*)&OD_record1402},
{0x1403, 0x02, 0x00,  0, (void*)&OD_record1403},
{0x1600, 0x08, 0x00,  0, (void*)&OD_record1600},
{0x1601, 0x08, 0x00,  0, (void*)&OD_record1601},
{0x1602, 0x08, 0x00,  0, (void*)&OD_record1602},
{0x1603, 0x08, 0x00,  0, (void*)&OD_record1603},
{0x1800, 0x06, 0x00,  0, (void*)&OD_record1800},
{0x1801, 0x06, 0x00,  0, (void*)&OD_record1801},
{0x1802, 0x06, 0x00,  0, (void*)&OD_record1802},
{0x1803, 0x06, 0x00,  0, (void*)&OD_record1803},
{0x1A00, 0x08, 0x00,  0, (void*)&OD_record1A00},
{0x1A01, 0x08, 0x00,  0, (void*)&OD_record1A01},
{0x1A02, 0x08, 0x00,  0, (void*)&OD_record1A02},
{0x1A03, 0x08, 0x00,  0, (void*)&OD_record1A03},
{0x1F80, 0x00, 0x8D,  4, (void*)&CO_OD_ROM.NMTStartup},
{0x2100, 0x00, 0x36, 10, (void*)&CO_OD_RAM.errorStatusBits},
{0x2101, 0x00, 0x0D,  1, (void*)&CO_OD_ROM.CANNodeID},
{0x2102, 0x00, 0x8D,  2, (void*)&CO_OD_ROM.CANBitRate},
{0x2103, 0x00, 0x8E,  2, (void*)&CO_OD_RAM.SYNCCounter},
{0x2104, 0x00, 0x86,  2, (void*)&CO_OD_RAM.SYNCTime},
{0x2106, 0x00, 0x87,  4, (void*)&CO_OD_EEPROM.powerOnCounter},
{0x2107, 0x05, 0xBE,  2, (void*)&CO_OD_RAM.performance[0]},
{0x2108, 0x01, 0xB6,  2, (void*)&CO_OD_RAM.temperature[0]},
{0x2109, 0x01, 0xB6,  2, (void*)&CO_OD_RAM.voltage[0]},
{0x2110, 0x10, 0xFE,  4, (void*)&CO_OD_RAM.variableInt32[0]},
{0x2111, 0x10, 0xFD,  4, (void*)&CO_OD_ROM.variableROM_Int32[0]},
{0x2112, 0x10, 0xFF,  4, (void*)&CO_OD_EEPROM.variableNV_Int32[0]},
{0x2120, 0x05, 0x00,  0, (void*)&OD_record2120},
{0x2130, 0x03, 0x00,  0, (void*)&OD_record2130},
{0x6000, 0x08, 0x76,  1, (void*)&CO_OD_RAM.readInput8Bit[0]},
{0x6200, 0x08, 0x3E,  1, (void*)&CO_OD_RAM.writeOutput8Bit[0]},
{0x6401, 0x0C, 0xB6,  2, (void*)&CO_OD_RAM.readAnalogueInput16Bit[0]},
{0x6411, 0x08, 0xBE,  2, (void*)&CO_OD_RAM.writeAnalogueOutput16Bit[0]},
};
// clang-format on
