#ifndef config_H
#define config_H

#include "ethercat.h"


//------------------------------------------------------------------------------

#define LAB_1             1
#define LAB_2             2

#define SLAVE_NUM           2


//------------------------------------------------------------------------------

#define IO_MAP_SIZE 256


//------------------------------------------------------------------------------

typedef struct __attribute__((__packed__))
{
    uint8_t     Segments;
}out_LAB_1t;

typedef struct __attribute__((__packed__))
{
    uint16_t    Potentiometer;              // in this example we don't use the    
    uint8_t     Buttons;                    // potentiometer but we have to 
}in_LAB_1t;                                 // declare it in the data structure


//------------------------------------------------------------------------------

typedef struct __attribute__((__packed__))
{
    uint8_t     Segments;
}out_LAB_2t;

typedef struct __attribute__((__packed__))
{
    uint16_t    Potentiometer;              // in this example we don't use the 
    uint8_t     Buttons;                    // potentiometer but we have to 
}in_LAB_2t;                                 // declare it in the data structure


//------------------------------------------------------------------------------

out_LAB_1t    *out_LAB_1;
in_LAB_1t     *in_LAB_1;

out_LAB_2t    *out_LAB_2;
in_LAB_2t     *in_LAB_2;


//------------------------------------------------------------------------------

char IOmap[IO_MAP_SIZE];
char IOmapSafe[IO_MAP_SIZE];   


void MapLocalStructures (void)
{
    out_LAB_1 = (out_LAB_1t*)((char *)ec_slave[LAB_1].outputs - &IOmap[0] + &IOmapSafe[0]);
    in_LAB_1 =  (in_LAB_1t*)((char *)ec_slave[LAB_1].inputs - &IOmap[0] + &IOmapSafe[0]);                
    
    out_LAB_2 = (out_LAB_2t*)((char *)ec_slave[LAB_2].outputs - &IOmap[0] + &IOmapSafe[0]);
    in_LAB_2 =  (in_LAB_2t*)((char *)ec_slave[LAB_2].inputs - &IOmap[0] + &IOmapSafe[0]);                
}



//------------------------------------------------------------------------------

uint32_t network_configuration(void)
{

   if (ec_slavecount != SLAVE_NUM)                          // check if the number of slaves matches what we expect
      return 0;                                             
                                                            
   if (strcmp(ec_slave[LAB_1].name,"LAB_1"))              // verify slave by slave that the slave names are correct  
      return 0;
   
   else if (strcmp(ec_slave[LAB_2].name,"LAB_2"))
      return 0;
      
  return 1;
}


#endif