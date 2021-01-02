#include "scrap_placard.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_task_wdt.h>

#include "lut.h"
#include "placard_image.h"

static const char* LOG = "scrap_placard";

static spi_device_handle_t spiDevice;

//IO settings
gpio_num_t BUSY_Pin = GPIO_NUM_27; 
//gpio_num_t RES_Pin = A15; 
gpio_num_t DC_Pin = GPIO_NUM_0; 
//gpio_num_t CS_Pin = GPIO_NUM_32; 
gpio_num_t SCK_Pin = GPIO_NUM_14; 
gpio_num_t SDI_Pin = GPIO_NUM_13; 

#define EPD_W21_MOSI_0  gpio_set_level(SDI_Pin,0)
#define EPD_W21_MOSI_1  gpio_set_level(SDI_Pin,1) 

#define EPD_W21_CLK_0 gpio_set_level(SCK_Pin,0)
#define EPD_W21_CLK_1 gpio_set_level(SCK_Pin,1)

//#define EPD_W21_CS_0 gpio_set_level(CS_Pin,0)
#define EPD_W21_CS_0
//#define EPD_W21_CS_1 gpio_set_level(CS_Pin,1)
#define EPD_W21_CS_1

#define EPD_W21_DC_0  gpio_set_level(DC_Pin,0)
#define EPD_W21_DC_1  gpio_set_level(DC_Pin,1)
//#define EPD_W21_RST_0 gpio_set_level(RES_Pin,0)
#define EPD_W21_RST_0
//#define EPD_W21_RST_1 gpio_set_level(RES_Pin,1)
#define EPD_W21_RST_1
#define isEPD_W21_BUSY gpio_get_level(BUSY_Pin)

////////FUNCTION//////
void driver_delay_us(const uint32_t xus);
void driver_delay_xms(const uint64_t xms);
void DELAY_S(const uint32_t delaytime);     
void SPI_Delay(const uint8_t xrate);
void SPI_Write(const uint8_t value);
void EPD_W21_WriteDATA(const uint8_t command);
void EPD_W21_WriteCMD(uint8_t command);
//EPD
void EPD_W21_Init();
void EPD_init();
void EPD_sleep();
void EPD_refresh();
void lcd_chkstatus();
void EPD_display_Clean();

//4 Gray

void EPD_init_4Gray();//EPD init 4 Gray
void pic_display_4bit();
void lut();

uint8_t HRES,VRES_byte1,VRES_byte2;

void setup() {
   gpio_pad_select_gpio(BUSY_Pin);
   gpio_set_direction(BUSY_Pin, GPIO_MODE_INPUT); 
   gpio_pad_select_gpio(DC_Pin);
   gpio_set_direction(DC_Pin, GPIO_MODE_OUTPUT);    

    EPD_init_4Gray(); //EPD init 4 Gray
    lut(); //Power settings
}
//Tips//
/*When the electronic paper is refreshed in full screen, the picture flicker is a normal phenomenon, and the main function is to clear the display afterimage in the previous picture.
  When the local refresh is performed, the screen does not flash.*/
/*When you need to transplant the driver, you only need to change the corresponding IO. The BUSY pin is the input mode and the others are the output mode. */

void display() {
    //4 gray picture
    pic_display_4bit(); //picture
    EPD_refresh();
    EPD_sleep();//EPD_sleep,Sleep instruction is necessary, please do not delete!!!
}
/////////////////////delay//////////////////////////////////////
void driver_delay_us(uint32_t xus)  //1us
{
    ets_delay_us(xus);
}
void driver_delay_xms(uint64_t xms) //1ms
{  
    vTaskDelay(xms / portTICK_PERIOD_MS);
}
void DELAY_S(uint32_t delaytime)     
{
    vTaskDelay(delaytime * 1000 / portTICK_PERIOD_MS);
}
//////////////////////SPI///////////////////////////////////
void SPI_Delay(uint8_t xrate)
{
    ets_delay_us(2 * xrate);
}


void SPI_Write(uint8_t value)                                    
{
    spi_transaction_t tx;
    tx.flags = SPI_TRANS_USE_TXDATA;
    tx.cmd = 0;
    tx.addr = 0;
    tx.length = 8;
    tx.rxlength = 0;
    tx.user = NULL;
    tx.tx_buffer = NULL;
    tx.tx_data[0] = tx.tx_data[1] = tx.tx_data[2] = tx.tx_data[3] = 0;
    tx.tx_data[0] = value;
    tx.rx_buffer = NULL;
    tx.rx_data[0] = tx.rx_data[1] = tx.rx_data[2] = tx.rx_data[3] = 0;

    spi_device_transmit(spiDevice, &tx);
}

void EPD_W21_WriteCMD(uint8_t command)
{
  EPD_W21_CS_0;                   
  EPD_W21_DC_0;   // command write
  SPI_Write(command);
  EPD_W21_CS_1;
}
void EPD_W21_WriteDATA(uint8_t data)
{
  EPD_W21_CS_0;                   
  EPD_W21_DC_1;   // data write
  SPI_Write(data);
  EPD_W21_CS_1;
}



/////////////////EPD settings Functions/////////////////////
void EPD_W21_Init()
{
  EPD_W21_RST_0;    // Module reset
  vTaskDelay(100 / portTICK_PERIOD_MS); //At least 10ms
  EPD_W21_RST_1;
  vTaskDelay(100 / portTICK_PERIOD_MS);
}
void EPD_init()
{
    HRES=0x98;            //152
    VRES_byte1=0x00;      //152
    VRES_byte2=0x98;
  
    EPD_W21_Init();//Electronic paper IC reset
  
    EPD_W21_WriteCMD(0x06);         //boost soft start
    EPD_W21_WriteDATA (0x17);   //A
    EPD_W21_WriteDATA (0x17);   //B
    EPD_W21_WriteDATA (0x17);   //C       

    EPD_W21_WriteCMD(0x04);  //Power on
    lcd_chkstatus();        //waiting for the electronic paper IC to release the idle signal

    EPD_W21_WriteCMD(0x00);     //panel setting
    EPD_W21_WriteDATA(0x1f);    //LUT from OTP£¬128x296
    EPD_W21_WriteDATA(0x0d);  //VCOM to 0V fast,This data is necessary, please do not delete!!!

    EPD_W21_WriteCMD(0x61);     //resolution setting
    EPD_W21_WriteDATA (HRES);          
    EPD_W21_WriteDATA (VRES_byte1);   
    EPD_W21_WriteDATA (VRES_byte2);

    EPD_W21_WriteCMD(0X50);     //VCOM AND DATA INTERVAL SETTING      
    EPD_W21_WriteDATA(0x97);    //WBmode:VBDF 17|D7 VBDW 97 VBDB 57   WBRmode:VBDF F7 VBDW 77 VBDB 37  VBDR B7
}
void EPD_init_4Gray()
{
    HRES=0x98;              //152
    VRES_byte1=0x00;        //152
    VRES_byte2=0x98;
    EPD_W21_Init(); //Electronic paper IC reset
    EPD_W21_WriteCMD(0x01);     //POWER SETTING
    EPD_W21_WriteDATA (0x03);
    EPD_W21_WriteDATA (0x00);      
    EPD_W21_WriteDATA (0x2b);                                  
    EPD_W21_WriteDATA (0x2b);   
    EPD_W21_WriteDATA (0x13);

    EPD_W21_WriteCMD(0x06);         //booster soft start
    EPD_W21_WriteDATA (0x17);   //A
    EPD_W21_WriteDATA (0x17);   //B
    EPD_W21_WriteDATA (0x17);   //C 
    
    EPD_W21_WriteCMD(0x04);  //Power on
    lcd_chkstatus();        //waiting for the electronic paper IC to release the idle signal
    
    EPD_W21_WriteCMD(0x00);     //panel setting
    EPD_W21_WriteDATA(0x3f);    //KW-3f   KWR-2F  BWROTP 0f BWOTP 1f
    
    EPD_W21_WriteCMD(0x30);     //PLL setting
    EPD_W21_WriteDATA (0x3c);       //100hz 
    
    EPD_W21_WriteCMD(0x61);     //resolution setting
    EPD_W21_WriteDATA (HRES);          
    EPD_W21_WriteDATA (VRES_byte1);   
    EPD_W21_WriteDATA (VRES_byte2);

    EPD_W21_WriteCMD(0x82);     //vcom_DC setting
    EPD_W21_WriteDATA (0x12);

    EPD_W21_WriteCMD(0X50);     //VCOM AND DATA INTERVAL SETTING      
    EPD_W21_WriteDATA(0x97);
}

uint8_t extractGrayBit(const char graySymbol, bool msb) {
    // extract the gray value MSB from the character representation
    switch (graySymbol) {
    case '.':
        return 0x00; // black
    case '+':
        return 0x01; // white
    case '@':
        return (msb ? 0x01 : 0x00); // light gray
    case '#':
    default:
        return (msb ? 0x00 : 0x01); // dark gray
    }
}

uint8_t getPlacardImagePixel(uint8_t x, uint8_t y, uint8_t rotation) {
    uint8_t translatedX;
    uint8_t translatedY;

    switch (rotation) {
    case 1:
        // rotate 90
        translatedX = y;
        translatedY = 151 - x;
        break;
    case 2:
        // rotate 180
        translatedX = 151 - x;
        translatedY = 151 - y;
        break;
    case 3:
        // rotate 270
        translatedX = 151 - y;
        translatedY = x;
        break;
    default:
        // no rotation
        translatedX = x;
        translatedY = y;
    }

    // the first 6 elements of placard_image_xpm are metadata; skip them
    const char* imageRow = placard_image_xpm[translatedY + 6];

    return imageRow[translatedX];
}

//4 grayscale demo function
/********Color display description
      white  gray1  gray2  black
0x10|  01     01     00     00
0x13|  01     00     01     00
                                   ****************/
void pic_display_4bit() {
    const uint8_t rotation = 2;

    EPD_W21_WriteCMD(0x10);        
    for (int i = 0; i < 152; i++) {
        for (int j = 0; j < (152 / 8); j++) {
            // assemble 8 image bits into a byte and write it
            uint8_t imageByte = 0;
            for (int k = 0; k < 8; k++) {
                uint8_t imageBit = extractGrayBit(getPlacardImagePixel(j * 8 + k, i, rotation),
                        true);
                imageByte |= (imageBit << (7 - k));
            }
            EPD_W21_WriteDATA(imageByte);     
        }
    }

    EPD_W21_WriteCMD(0x13);        

    for (int i = 0; i < 152; i++) {
        for (int j = 0; j < (152 / 8); j++) {
            // assemble 8 image bits into a byte and write it
            uint8_t imageByte = 0;
            for (int k = 0; k < 8; k++) {
                uint8_t imageBit = extractGrayBit(getPlacardImagePixel(j * 8 + k, i, rotation),
                        false);
                imageByte |= (imageBit << (7 - k));
            }
            EPD_W21_WriteDATA(imageByte);     
        }
    }
}
void EPD_refresh()
{
    EPD_W21_WriteCMD(0x12);     //DISPLAY REFRESH   
    driver_delay_xms(1);          //!!!The delay here is necessary, 200uS at least!!!     
    lcd_chkstatus();
} 
void EPD_sleep()
{
    EPD_W21_WriteCMD(0X50);  //VCOM AND DATA INTERVAL SETTING  
    EPD_W21_WriteDATA(0xf7);
       
    EPD_W21_WriteCMD(0X02);   //power off
    lcd_chkstatus();
    EPD_W21_WriteCMD(0X07);   //deep sleep
    EPD_W21_WriteDATA(0xA5);
}


void EPD_display_Clean()
{
    uint32_t i;
    EPD_W21_WriteCMD(0x10);        //Transfer old data
    for(i=0;i<2888;i++)       
  {
    EPD_W21_WriteDATA(0xff);
  }

    EPD_W21_WriteCMD(0x13);        //Transfer new data
    for(i=0;i<2888;i++)       
  {
    EPD_W21_WriteDATA(0xff);
  }
}

void lcd_chkstatus()
{
  uint8_t idle;
  do
  {
    esp_task_wdt_reset();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    EPD_W21_WriteCMD(0x71);
    idle = isEPD_W21_BUSY;
  }
  while(!idle);   
  driver_delay_xms(200);                       
}

//LUT download
void lut()
{
  uint32_t count;  
  {
    EPD_W21_WriteCMD(0x20);             //vcom
    for(count=0;count<42;count++)
      {EPD_W21_WriteDATA((lut_vcom[count]));}
    EPD_W21_WriteDATA(0x00);
    EPD_W21_WriteDATA(0x00);
    
  EPD_W21_WriteCMD(0x21);             //red not use
  for(count=0;count<42;count++)
    {EPD_W21_WriteDATA((lut_ww[count]));}

    EPD_W21_WriteCMD(0x22);             //bw r
    for(count=0;count<42;count++)
      {EPD_W21_WriteDATA((lut_bw[count]));}

    EPD_W21_WriteCMD(0x23);             //wb w
    for(count=0;count<42;count++)
      {EPD_W21_WriteDATA((lut_wb[count]));}

    EPD_W21_WriteCMD(0x24);             //bb b
    for(count=0;count<42;count++)
      {EPD_W21_WriteDATA((lut_bb[count]));}

    EPD_W21_WriteCMD(0x25);             //vcom
    for(count=0;count<42;count++)
      {EPD_W21_WriteDATA((lut_ww[count]));}
  }          
}

ScrapPlacard::ScrapPlacard(spi_host_device_t spiHost, gpio_num_t sclk, gpio_num_t mosi,
        gpio_num_t dc, gpio_num_t busy) : dc(dc), busy(busy) {
    spi_bus_config_t spiConfig;
    spiConfig.mosi_io_num = mosi;
    spiConfig.miso_io_num = -1;
    spiConfig.sclk_io_num = sclk;
    spiConfig.quadwp_io_num = -1;
    spiConfig.quadhd_io_num = -1;
    spiConfig.max_transfer_sz = 0;
    spiConfig.flags = SPICOMMON_BUSFLAG_MASTER;
    spiConfig.intr_flags = 0;
    spi_bus_initialize(spiHost, &spiConfig, 0);

    spi_device_interface_config_t spiInterfaceConfig;
    spiInterfaceConfig.command_bits = 0;
    spiInterfaceConfig.address_bits = 0;
    spiInterfaceConfig.dummy_bits = 0;
    spiInterfaceConfig.mode = 0;
    spiInterfaceConfig.duty_cycle_pos = 128; // 50% duty cycle
    spiInterfaceConfig.cs_ena_pretrans = 0;
    spiInterfaceConfig.cs_ena_posttrans = 0;
    spiInterfaceConfig.clock_speed_hz = 20 * 1000 * 1000; // UC8151C should be good up to 20 MHz
    spiInterfaceConfig.input_delay_ns = 0;
    spiInterfaceConfig.spics_io_num = GPIO_NUM_32;
    spiInterfaceConfig.flags = 0;
    spiInterfaceConfig.queue_size = 1;
    spiInterfaceConfig.pre_cb = NULL;
    spiInterfaceConfig.post_cb = NULL;

    spi_bus_add_device(spiHost, &spiInterfaceConfig, &spiDevice);
    ::spiDevice = spiDevice;
}

void ScrapPlacard::test() {
    setup();
    display();
}
