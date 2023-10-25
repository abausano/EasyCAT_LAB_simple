//********************************************************************************************    
//                                                                                           *
// This software is distributed as an example, "AS IS", in the hope that it could            *
// be useful, WITHOUT ANY WARRANTY of any kind, express or implied, included, but            *
// not limited,  to the warranties of merchantability, fitness for a particular              *
// purpose, and non infringiment. In no event shall the authors be liable for any            *    
// claim, damages or other liability, arising from, or in connection with this software.     *
//                                                                                           *
//******************************************************************************************** 


// The AB&T EasyCAT LAB is a complete experimental EtherCAT® system, composed by
// one master and two slaves.
// The EasyCAT LAB software is provided free of charge and its pourpose is to allow
// makers and educational institutes to experiment with the EtherCAT® protocol.
//
// The EasyCAT LAB is developed by "AB&T Tecnologie Informatiche" Via dell'About 2A Ivrea Italy.
// www.bausano.net
// www.easycatshield.com
//
// The EasyCAT LAB uses the SOEM library by rt:labs
// https://rt-labs.com/products/soem-ethercat-master-stack/
// 
// EtherCAT® is a registered trademark and patented technology, licensed by Beckhoff Automation GmbH.
// www.beckhoff.com
// www.ethercat.org     


//******************************************************************************

                        // IMPORTANT!!! 

#define ADA_TFT       // If your EasyCAT LAB uses the Adafruit TFT
                        // you must uncomment this define
                        
//#define SEEED_TFT     // If your EasyCAT LAB uses the Seeed Studio TFT
                        // you must uncomment this define     
                        
//#define PARA_TFT        // If your EasyCAT LAB uses the parallel interface TFT
                        // you must uncomment this define                         
                                                                     
//******************************************************************************

#define ETH_TXBUFNB 16
#define ETH_RXBUFNB 16

#include "mbed.h"    

#ifndef __align
#define __align MBED_ALIGN
#endif

#include "config.h"  
#include "SPI_TFT_ILI9341.h"
#include "Arial12x12.h"
#include "Arial24x23.h"
#include "Arial28x28.h"
#include "font_big.h"
#include "soem_start.h"

#define CYCLE_TIME 1000                 // master cycle time in uS                    
                                        // 1000 = 1mS 

                                  
#define SysMilliS() (uint32_t)Kernel::get_ms_count()                                      

UnbufferedSerial pc(USBTX,USBRX,115200);          // set the debug serial line speed to 115200


//---- TFT with resistive touchscreen pins -------------------------------------

// the display used is the SeeedStudio 2.8 inch TFT v2.0
// http://wiki.seeedstudio.com/2.8inch_TFT_Touch_Shield_v2.0/ 
//
// or the Adafruit 2.8" with resistive touchscreen
// https://www.adafruit.com/product/1651
//
// or the parallel interface ARD SHD 2,8TD 

// the touchscreen is not used in this example

#ifdef ADA_TFT                                      // pins for the Adafruit TFT                      
    #define PIN_CS_TFT  D10                         //               
    #define PIN_DC_TFT  D9                          //                     
    #define PIN_CS_TSC  D8                          // 

    #define PIN_MOSI    D11                         // SPI
    #define PIN_MISO    D12                         //
    #define PIN_SCLK    D13                         // 
#endif
    
#ifdef SEEED_TFT                                    // pins for the SeeedStudio TFT
    #define PIN_CS_TFT  D5                          //    
    #define PIN_DC_TFT  D6                          //

    #define PIN_MOSI    D11                         // SPI
    #define PIN_MISO    D12                         //
    #define PIN_SCLK    D13                         //    
#endif                                              //

#ifdef PARA_TFT                                     // pins for the parallel interface TFT
    #define PIN_D0_TFT  D8                          //
    #define PIN_D1_TFT  D9                          //
    #define PIN_D2_TFT  D2                          //
    #define PIN_D3_TFT  D3                          //
    #define PIN_D4_TFT  D4                          //
    #define PIN_D5_TFT  D5                          //
    #define PIN_D6_TFT  D6                          //
    #define PIN_D7_TFT  D7                          //
                                                       
    #define PIN_RD_TFT  A0                          //                
    #define PIN_WR_TFT  A1                          //
    #define PIN_DC_TFT  A2                          //
    #define PIN_CS_TFT  A3                          //
    #define PIN_RES_TFT A4                          //
#endif

 #ifdef SEEED_TFT
    #define PIN_YP      A3                          // resistive touchscreen
    #define PIN_YM      A1                          //
    #define PIN_XM      A2                          //
    #define PIN_XP      A0                          //
#else                                               //
    #define PIN_XP      A3                          //
    #define PIN_YP      A2                          //
    #define PIN_XM      D9                          //
    #define PIN_YM      D8                          //  
#endif
                                           //



//---- visualization parameters ------------------------------------------------

#define BUTTONS_1_X 80
#define BUTTONS_1_Y 60

#define BUTTONS_2_X 80
#define BUTTONS_2_Y 180 
 
 
#define BUTTONS_1_WIDTH 26
#define BUTTONS_1_R 3
#define BUTTONS_1_STEP 60 

#define BUTTONS_2_WIDTH     BUTTONS_1_WIDTH 
#define BUTTONS_2_R         BUTTONS_1_R
#define BUTTONS_2_STEP      BUTTONS_1_STEP 


//---- local functions ---------------------------------------------------------

void DrawBanner();
void DrawSlaveFixedParts();
void DrawButtons_1_Value(uint8_t Value);
void DrawButtons_2_Value(uint8_t Value);

void Application();   


//---- global variables --------------------------------------------------------


bool FirstRound;

//------------------------------------------------------------------------------

#ifdef PARA_TFT
    SPI_TFT_ILI9341 TFT(PIN_D0_TFT, PIN_D1_TFT, PIN_D2_TFT, PIN_D3_TFT, PIN_D4_TFT, PIN_D5_TFT,
    PIN_D6_TFT, PIN_D7_TFT, PIN_RD_TFT, PIN_WR_TFT, PIN_CS_TFT, PIN_DC_TFT, PIN_RES_TFT, "PARA");
#endif

#ifdef ADA_TFT
    SPI_TFT_ILI9341 TFT(PIN_MOSI, PIN_MISO, PIN_SCLK, PIN_CS_TFT, NC, PIN_DC_TFT, 27000000, "ADA");
#endif

#ifdef SEEED_TFT
    SPI_TFT_ILI9341 TFT(PIN_MOSI, PIN_MISO, PIN_SCLK, PIN_CS_TFT, NC, PIN_DC_TFT, 27000000, "SEEED");
#endif

Ticker SampleTicker;
Thread thread;

DigitalOut Test_1(D1);          // debug test points
DigitalOut Test_2(D2);          //
DigitalOut Test_3(D3);          //
DigitalOut Test_4(D4);          //


//------------------------------------------------------------------------------

uint8_t Buttons_1; 
uint8_t PrevButtons_1;
uint8_t Segments_1;
uint8_t PrevSegments_1;

uint8_t Buttons_2; 
uint8_t PrevButtons_2;
uint8_t Segments_2;
uint8_t PrevSegments_2;
   
   
//------------------------------------------------------------------------------   

int ExpectWorkCounter;
int WorkCounter; 
int WorkCounterSafe; 
bool NetworkError;  
bool NetworkErrorSafe; 

#define DATA_EXCHANGE_FLAG      (1UL << 0)
#define APPLICATION_FLAG        (1UL << 1)

EventFlags event_flags;

Mutex IO_data;


//---- data exchange thread ----------------------------------------------------

void ExchangeMaster()
{
    while (true)
    {
        event_flags.wait_any(DATA_EXCHANGE_FLAG);   // the thread waits for the synchronization flag
    
        //Test_1 = 1;
    
        IO_data.lock();                             // Ethercat data exchange
        ec_send_processdata();                      // 
        WorkCounter = ec_receive_processdata(EC_TIMEOUTRET);  
        
        if (WorkCounter != ExpectWorkCounter)
            NetworkError = true;
        else
            NetworkError = false;    
                            
        IO_data.unlock();                           //
        event_flags.set(APPLICATION_FLAG);          // synchronize the application    
        
        //Test_1 = 0;                    
    }
}


//----- thicker generated sample time ------------------------------------------

void SampleIsr()                                    // set the event that starts
{                                                   // the data exchange
    event_flags.set(DATA_EXCHANGE_FLAG);            //
}                                                   //
    
    
//****** initialization ********************************************************

int main()
{      
    int i;
    
    printf("Start \n");
    
    Test_1 = 0; 
    Test_2 = 0;
    Test_3 = 0;
    Test_4 = 0;      
  
    TFT.background(Black);                                          // init TFT  
    TFT.cls();                                                      //
    
    #ifdef ADA_TFT
        TFT.set_orientation(1);     
    #endif
    #ifdef SEEED_TFT
        TFT.set_orientation(3);                                        
    #endif  
    #ifdef PARA_TFT
        TFT.set_orientation(1);                                        
    #endif 

    DrawBanner();     
        
    NetworkError = false;  
      
    if (ec_init(NULL))                                              // init SOEM
    {
        printf("ec_init succeeded.\n");     
        printf("Scanning the network\n");

        TFT.cls();  
                                                            
        TFT.set_font((unsigned char*) Arial12x12);
        TFT.foreground(Green);
        TFT.locate(0, 0);
        
        TFT.printf("Scanning the network\n");           
          
        if (network_scanning())
        {   
            if (network_configuration())                            // check network configuration
            {
                ec_config_map(&IOmap);                              // map the I/O
                MapLocalStructures();                 

                printf("\nSlaves mapped, state to SAFE_OP.\n");     
                                                                    // wait for all slaves to reach SAFE_OP state         
                ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
    
                printf("Request operational state for all slaves\n");
                ec_slave[0].state = EC_STATE_OPERATIONAL;           
           
                ec_send_processdata();                              // send one valid process data to make outputs in slaves happy
                ExpectWorkCounter = ec_receive_processdata(EC_TIMEOUTRET);          
               
                ec_writestate(0);                                   // request OP state for all slaves 
                
                                                                    // wait for all slaves to reach OP state 
                ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE);
                if (ec_slave[0].state == EC_STATE_OPERATIONAL )
                {
                    printf("Operational state reached for all slaves.\n");
                }
                else
                {
                    printf("Not all slaves reached operational state.\n");
                    ec_readstate();
                    for(i = 1; i<=ec_slavecount ; i++)
                    {
                        if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                        {
                            printf("Slave %d State=0x%04x StatusCode=0x%04x\n",
                                    i, ec_slave[i].state, ec_slave[i].ALstatuscode);
                        }
                    }
                    
                    TFT.foreground(Red);                                                     
                    TFT.locate(0, 0);      
                    TFT.printf("Not all slaves reached operational state!");                           
                    while(1){}  
                }   
                
                DrawSlaveFixedParts(); 
                                
                thread.start(ExchangeMaster);
                thread.set_priority(osPriorityRealtime); 

                SampleTicker.attach_us(&SampleIsr, CYCLE_TIME);  
              
                Application();
            }
            
            else
            {
                printf("Mismatch of network units!\n");            
                TFT.foreground(Red);                                                     
                TFT.locate(0, 0);      
                TFT.printf("Mismatch of network units!");
    
                while(1){}  
            }                       
        }       
        
        else
        {
            printf("No slaves found!\n");
            TFT.foreground(Red);                                                          
            TFT.printf("No slaves found!");             
        
            while(1){}  
        }           
    }
    else
    {
        printf("Ethernet interface init failed!");
        TFT.foreground(Red);                                                     
        TFT.locate(0, 0);      
        TFT.printf("Ethernet interface init failed!");                
        while(1){}
    }    
}             
                                 
              
//****** user master application  **********************************************
      
void Application()      
{        
     
    while(1)
    {          
        event_flags.wait_any(APPLICATION_FLAG);                     // the application waits for the synchronization flag
        
        //Test_2 = 1;     
     
        IO_data.lock();                                             // copy the Ethercat data to a safe buffer
        memcpy(&IOmapSafe[0], &IOmap[0], IO_MAP_SIZE);              //
                                                                    //                
        if (NetworkError)                                           //    
        {                                                           //
            NetworkErrorSafe = NetworkError;                        //    
            WorkCounterSafe = WorkCounter;                          //
        }                                                           //
        IO_data.unlock();                                           // 
              
        if (NetworkErrorSafe)
        {           
            TFT.rect(35,50, 285, 182, Red);    
            TFT.fillrect(36,51, 284, 181, Black);
            TFT.foreground(Red);
            TFT.set_font((unsigned char*) Arial28x28);        
            TFT.locate(58, 65); 
            TFT.printf("Network error!");
            printf("Network error!\n");                  
            TFT.foreground(Magenta);
            TFT.set_font((unsigned char*) Arial12x12);        
            TFT.locate(58, 106);
            
            if(WorkCounterSafe >= 0)
            {
                TFT.printf("Expected working counter %d", ExpectWorkCounter);                                         
                TFT.locate(58, 118);                                                   
                TFT.printf("Actual working counter %d", WorkCounterSafe); 
                printf("Expected working counter %d\n", ExpectWorkCounter);                                                    
                printf("Actual working counter %d\n", WorkCounterSafe); 
            } 
            else
            {
                TFT.printf("Timeout");                 
                printf("Timeout\n");   
            }                        
                
            TFT.foreground(Green);    
            TFT.locate(58, 142);                                                   
            TFT.printf("Please fix the error and");
            TFT.locate(58, 154);                      
            TFT.printf("press the reset button");                              
            printf("Please fix the error and press the reset button \n"); 
            
            SampleTicker.detach();                                  // stop the sample interrupt 
            while(1){}                                              // and loop for ever                           
        }     
        
        
                                                                    //----- slave LAB_2_1 data management ------            
                                                                    //    
        Buttons_1 = in_LAB_1->Buttons;                              // read the buttons status from the slave  
                                                                    //            
        if (Buttons_1 != PrevButtons_1)                             // check if the buttons value has changed
        {                                                           //
            PrevButtons_1 = Buttons_1;                              // remember the current buttons value  
            DrawButtons_1_Value(Buttons_1);                         // draw the current buttons value on the TFT                                                                                              
        }                                                           //
                                                                    //
        out_LAB_1->Segments = Buttons_2;                            // send to the slave the buttons status
                                                                    // from the slave LAB_2_2                  

                                                                                                                    
                                                                    
                                                                    //----- slave 2_2 data management ----------                                                                      
                                                                    //
        Buttons_2 = in_LAB_2->Buttons;                              // read the buttons status from the slave  
                                                                    //    
        if (Buttons_2 != PrevButtons_2)                             // check if the buttons value has changed
        {                                                           //
            PrevButtons_2 = Buttons_2;                              // remember the current buttons value  
            DrawButtons_2_Value(Buttons_2);                         // draw the current buttons value on the TFT                                                                                             
        }                                                           //
                                                                    //               
        out_LAB_2->Segments = Buttons_1;                            // send to the slave the buttons status                                                                       
                                                                    // from the slave LAB_2_1                                                                        
                                                                                                                                          
                                           
                                           
        IO_data.lock();                                             // copy the IO data from the safe area
        memcpy(&IOmap[0], &IOmapSafe[0], IO_MAP_SIZE);              // to the EtherCAT buffer
        IO_data.unlock();                                           //    
        
        //Test_2 = 0;                             
    }       
}      
  

//******************************************************************************



//******* general functions ****************************************************


 
//----- draw the fixed part of the slaves visualization ------------------------

void DrawSlaveFixedParts()
{
    int i;
    
    TFT.cls();                                              // clear screen    
    TFT.line(0, 120, 340, 120, Green);                      // draw the separation line    
                                  
    for (i=0; i<3; i++)                                     // draw the buttons fixed parts        
    {                                                       // for the LAB_2_1 slave   
        TFT.circle(BUTTONS_1_X+(i*BUTTONS_1_STEP), BUTTONS_1_Y, BUTTONS_1_R, Red);                
        TFT.circle(BUTTONS_1_X+BUTTONS_1_WIDTH+(i*BUTTONS_1_STEP), BUTTONS_1_Y, BUTTONS_1_R, Red); 
                                                            //
        TFT.line(BUTTONS_1_X+(i*BUTTONS_1_STEP)-BUTTONS_1_R, BUTTONS_1_Y, BUTTONS_1_X+(i*BUTTONS_1_STEP)-BUTTONS_1_R-4, BUTTONS_1_Y, Red);  
        TFT.line(BUTTONS_1_X+BUTTONS_1_WIDTH+(i*BUTTONS_1_STEP)+BUTTONS_1_R, BUTTONS_1_Y, BUTTONS_1_X+BUTTONS_1_WIDTH+(i*BUTTONS_1_STEP)+BUTTONS_1_R+4 , BUTTONS_1_Y, Red);                          
    }                                                       //
                                                            //   
    TFT.set_font((unsigned char*) Arial12x12);              // 
    TFT.locate(BUTTONS_1_X+12, BUTTONS_1_Y+20);             // 
    TFT.printf("LAB_2_1 Buttons");                          //      
    
    for (i=0; i<3; i++)                                     // draw the buttons fixed parts        
    {                                                       // for the LAB_2_2 slave   
        TFT.circle(BUTTONS_2_X+(i*BUTTONS_2_STEP), BUTTONS_2_Y, BUTTONS_2_R, Red);                
        TFT.circle(BUTTONS_2_X+BUTTONS_2_WIDTH+(i*BUTTONS_2_STEP), BUTTONS_2_Y, BUTTONS_2_R, Red); 
                                                            //
        TFT.line(BUTTONS_2_X+(i*BUTTONS_2_STEP)-BUTTONS_2_R, BUTTONS_2_Y, BUTTONS_2_X+(i*BUTTONS_2_STEP)-BUTTONS_2_R-4, BUTTONS_2_Y, Red);  
        TFT.line(BUTTONS_2_X+BUTTONS_2_WIDTH+(i*BUTTONS_2_STEP)+BUTTONS_2_R, BUTTONS_2_Y, BUTTONS_2_X+BUTTONS_2_WIDTH+(i*BUTTONS_2_STEP)+BUTTONS_2_R+4 , BUTTONS_2_Y, Red);                          
    }                                                       //
                                                            //   
    TFT.set_font((unsigned char*) Arial12x12);              // 
    TFT.locate(BUTTONS_2_X+12, BUTTONS_2_Y+20);             // 
    TFT.printf("LAB_2_2 Buttons");                          //               
                     
    DrawButtons_1_Value (Buttons_1);                        // draw the buttons status  
    DrawButtons_2_Value (Buttons_2);                        //            
    
    FirstRound = true;                      
}    


//---- draw the starting banner ------------------------------------------------

void DrawBanner()
{
    TFT.set_font((unsigned char*) Arial24x23);
    TFT.foreground(Red);
    TFT.locate(30, 30);
    TFT.printf("EasyCAT");
    TFT.locate(30, 60);
    TFT.printf("SOEM MASTER");    

    TFT.foreground(Magenta);
    TFT.locate(30, 100);
    TFT.printf("simple example");    
    
    TFT.set_font((unsigned char*) Arial12x12);
    TFT.foreground(Green);
    TFT.locate(30, 150);
    TFT.printf("www.bausano.net");

    TFT.foreground(Green);
    TFT.locate(30, 170);
    TFT.printf("www.easycatshield.com"); 
    
    TFT.locate(30, 190);
    TFT.printf("https://openethercatsociety.github.io/");
    
    TFT.foreground(Red);    
    TFT.locate(30, 220);    
    #ifdef ADA_TFT                                 
        TFT.printf("Adafruit TFT");   
    #endif
    #ifdef SEEED_TFT  
        TFT.printf("Seeed Studio TFT");
    #endif  
    #ifdef PARA_TFT  
        TFT.printf("Parallel TFT");
    #endif  
}  


//---- slaves data visualization functions -------------------------------------

void DrawButtons_1_Value (uint8_t Value)            // visualize on the TFT the
{                                                   // slave LAB_2_1 buttons status    
    uint8_t Slope;
    int i;
  
    for (i=0; i<3; i++)
    {
        if ((Value & 0x04) == 0x04)
            Slope = BUTTONS_1_R;
        else
            Slope = 16;          
    
        TFT.fillrect(BUTTONS_1_X+(i*BUTTONS_1_STEP), BUTTONS_1_Y-16-1, BUTTONS_1_X+BUTTONS_1_WIDTH+(i*BUTTONS_1_STEP), BUTTONS_1_Y-BUTTONS_1_R-1, Black);
    
        TFT.line(BUTTONS_1_X+(i*BUTTONS_1_STEP), BUTTONS_1_Y-BUTTONS_1_R-1, BUTTONS_1_X+BUTTONS_1_WIDTH+(i*BUTTONS_1_STEP), BUTTONS_1_Y-Slope-1, Red); 

        Value = Value << 1;    
    } 
}    


void DrawButtons_2_Value (uint8_t Value)            // visualize on the TFT the
{                                                   // slave LAB_2_2 buttons status 
    uint8_t Slope;
    int i;
  
    for (i=0; i<3; i++)
    {
        if ((Value & 0x04) == 0x04)
            Slope = BUTTONS_1_R;
        else
            Slope = 16;          
    
        TFT.fillrect(BUTTONS_2_X+(i*BUTTONS_2_STEP), BUTTONS_2_Y-16-1, BUTTONS_2_X+BUTTONS_2_WIDTH+(i*BUTTONS_2_STEP), BUTTONS_2_Y-BUTTONS_2_R-1, Black);
    
        TFT.line(BUTTONS_2_X+(i*BUTTONS_2_STEP), BUTTONS_2_Y-BUTTONS_2_R-1, BUTTONS_2_X+BUTTONS_2_WIDTH+(i*BUTTONS_2_STEP), BUTTONS_2_Y-Slope-1, Red); 

        Value = Value << 1;    
    } 
}  

