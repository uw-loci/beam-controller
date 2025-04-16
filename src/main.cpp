#include <Arduino.h>
#include <ModbusSerial.h>
#include <cmath>
#include "waveform.h"

#define DEVICE_ID 10
#define SIZE 10

#define M_PI 3.14159265358979323846

#include "hardware/spi.h"

enum {
COMMAND,
X,
Y,
PULSER_1,
PULSER_2,
PULSER_3,
BERTAN_1_OC,
BERTAN_2_OC,
MATSUSADA_1_OC,
MATSUSADA_2_OC
} REG_TYPE;

unsigned int InputDataReg [SIZE];
int cmdreg = 0;
int xreg = 0;

uint8_t tx_buf[2];
uint8_t rx_buf[2];

ModbusSerial ModbusObj (Serial2, DEVICE_ID, -1);

const uint32_t sampling_rate = 44100;  // 120 kHz sampling rate
const uint16_t frequency = 60;         // 60 Hz sine wave
const uint16_t SAMPLES = 32768;

bool core1_separate_stack = true;
int localcmd = 0;

void ModBuspacketHandler(){
  //for(int i = 0; i < SIZE; i++){
  //  InputDataReg[i] = ModbusObj.Hreg(0x00 + (i<<2));
  //}
  localcmd = ModbusObj.Hreg(0x00 + (0<<2));
  xreg = ModbusObj.Hreg(0x00 + 1);
}

void setup1(){
  //SPI.setCS(5);
  //SPI.setSCK(2);
  //SPI.setMOSI(3);
  //SPI.begin(true);
  spi_init(spi_default, 30000 * 1000);
  spi_set_format(spi_default, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(-1, GPIO_FUNC_SPI);
    gpio_set_function(2, GPIO_FUNC_SPI);
    gpio_set_function(3, GPIO_FUNC_SPI);
    gpio_set_function(5, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    //bi_decl(bi_4pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI));

}

void setup() {
  // put your setup code here, to run once:
  Serial2.setPinout(8, 9);
  Serial2.begin(115200, SERIAL_8E1);
  Serial.begin(115200);
  Serial2.println("Serial2");
  ModbusObj.config(115200);
  ModbusObj.setAdditionalServerData ("TEST");

  pinMode(14, OUTPUT);
  //pinMode(5, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(14, LOW);
  //digitalWrite(5, HIGH);
  //for(int i = 0; i < SIZE; i++){
    ModbusObj.addHreg(0x00 + (0<<2), cmdreg);
    ModbusObj.addCoil(12);
    ModbusObj.addCoil(13);
    ModbusObj.addCoil(14);
    ModbusObj.addCoil(15);
    ModbusObj.addHreg(0x00 + 1, xreg);
    ModbusObj.addHreg(0x00 + 2, 0);
  //}
  
}

int cnt = 0;
uint16_t recv[16];
uint16_t out_buf[16];
uint16_t sine_wave[65535];
uint32_t command = 0;
uint16_t val;
volatile uint16_t phase = 0;
void loop1(){
 // if(rp2040.fifo.available()) command = rp2040.fifo.pop();
    if(localcmd == 1){
      for(int i = 0; i< 65535; i = i + 2){
        out_buf[0] = i;
        spi_write16_blocking(spi_default, out_buf, 1);
      }
    }else if(localcmd == 2){
      for(uint16_t i = 0; i < 8192; i++){
        out_buf[0] = 32767 * sin(2*3.1415926535*i/8192) + 32767;//lut2[i];
        spi_write16_blocking(spi_default, out_buf, 1);
        
      }
    }else if(localcmd == 3){
      for(uint16_t i = 0; i< 32768; i = i + 1){
        out_buf[0] = lut[i];
        spi_write16_blocking(spi_default, out_buf, 1);
      }
    }else if(localcmd == 4){
      for(uint16_t i = 0; i< 16384; i = i + 1){
        out_buf[0] = hammingwave[i];
        spi_write16_blocking(spi_default, out_buf, 1);
      }
    }
}

double theta;
double amplitude;
void loop() {
  // put your main code here, to run repeatedly:
  ModbusObj.task();
  digitalWrite(LED_BUILTIN, ModbusObj.Coil(12));
  analogWrite(14, ModbusObj.Hreg(0x00 + 2));
  ModBuspacketHandler();
  //localcmd = ModbusObj.Coil(13);
}