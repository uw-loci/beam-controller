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
  X_DIRECT_WRITE,
  Y_DIRECT_WRITE,
  PULSER_1_DUTY,
  PULSER_2_DUTY,
  PULSER_3_DUTY,
  SAMPLES_RATE,
  AMPLITUDE,
  PHASE,
  OFFSET,
  BERTAN_1_OC,
  BERTAN_2_OC,
  MATSUSADA_1_OC,
  MATSUSADA_2_OC
} REG_TYPE;

unsigned int InputDataReg [SIZE];
int cmdreg = 0;
uint16_t xreg = 0;
uint16_t yreg = 0;
uint8_t pwm1_reg = 0;
uint8_t pwm2_reg = 0;
uint8_t pwm3_reg = 0;

uint8_t tx_buf[2];
uint8_t rx_buf[2];

ModbusSerial ModbusObj (Serial2, DEVICE_ID, -1);

bool core1_separate_stack = true;
int localcmd = 0;

uint16_t samples = 8192;
uint16_t amp = 32767;
int16_t phase = 0;
int16_t offset = 0;

void ModBuspacketHandler(){
  xreg = ModbusObj.Hreg(X_DIRECT_WRITE);
  yreg = ModbusObj.Hreg(Y_DIRECT_WRITE);
  pwm1_reg = ModbusObj.Hreg(PULSER_1_DUTY);
  pwm2_reg = ModbusObj.Hreg(PULSER_2_DUTY);
  pwm3_reg = ModbusObj.Hreg(PULSER_3_DUTY);
  samples = ModbusObj.Hreg(SAMPLES_RATE);
  amp = ModbusObj.Hreg(AMPLITUDE);
  phase = ModbusObj.Hreg(PHASE);
  offset = ModbusObj.Hreg(OFFSET);

  localcmd = ModbusObj.Hreg(COMMAND); //This should be update in the end
}

void setup1(){
  //SPI.setCS(5);
  //SPI.setSCK(2);
  //SPI.setMOSI(3);
  //SPI.begin(true);
  spi_init(spi_default, 36000 * 1000);
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
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(14, LOW);
    ModbusObj.addHreg(COMMAND, cmdreg);
    ModbusObj.addCoil(12);
    ModbusObj.addCoil(13);
    ModbusObj.addCoil(14);
    ModbusObj.addCoil(15);
    ModbusObj.addHreg(X_DIRECT_WRITE, xreg);
    ModbusObj.addHreg(Y_DIRECT_WRITE, yreg);
    ModbusObj.addHreg(PULSER_1_DUTY, pwm1_reg);
    ModbusObj.addHreg(PULSER_2_DUTY, pwm2_reg);
    ModbusObj.addHreg(PULSER_3_DUTY, pwm3_reg);
    ModbusObj.addHreg(SAMPLES_RATE, samples);
    ModbusObj.addHreg(AMPLITUDE, amp);
    ModbusObj.addHreg(PHASE, phase);
    ModbusObj.addHreg(OFFSET, offset);
}

int cnt = 0;
uint16_t recv[16];
uint16_t out_buf[16];
uint16_t sine_wave[65535];
uint32_t command = 0;
uint16_t val;
void loop1(){
    if(localcmd == 0){
      out_buf[0] = xreg;
      spi_write16_blocking(spi_default, out_buf, 1);
    }else if(localcmd == 1){
      out_buf[0] = yreg;
      spi_write16_blocking(spi_default, out_buf, 1);
    }else if(localcmd == 2){
      for(int i = 0; i< samples; i = i + (65536 / samples)){ //Slope
        out_buf[0] = (65536 / samples) * (i + phase) + offset;
        spi_write16_blocking(spi_default, out_buf, 1);
      }
    }else if(localcmd == 3){
      for(uint16_t i = 0; i < samples; i++){ //Sine wave
        out_buf[0] = amp * sin(2*M_PI*(i + phase)/samples) + amp + offset;//lut2[i];
        spi_write16_blocking(spi_default, out_buf, 1);
        
      }
    }else if(localcmd == 4){ //Triangle Wave
      for(int i = 0; i< samples; i = i + (65536 / samples)){ //Slope
        out_buf[0] = (65536 / samples) * (i + phase) + offset;
        spi_write16_blocking(spi_default, out_buf, 1);
      }

      for(int i = samples - 1; i > 0; i = i - (65536 / samples)){ //Slope
        out_buf[0] = (65536 / samples) * (i + phase) + offset;
        spi_write16_blocking(spi_default, out_buf, 1);
      }
    }else if(localcmd == 5){ //Hamming Wave
      for(uint16_t i = 0; i< samples; i++){
        out_buf[0] = amp * (0.5-0.4*cos((2*M_PI*(i + phase)/(samples-1)))) + amp + offset;
        spi_write16_blocking(spi_default, out_buf, 1);
      }
    }
}

void loop() {
  // put your main code here, to run repeatedly:
  ModbusObj.task();
  digitalWrite(LED_BUILTIN, ModbusObj.Coil(12));
  analogWrite(14, ModbusObj.Hreg(PULSER_1_DUTY));
  ModBuspacketHandler();
}