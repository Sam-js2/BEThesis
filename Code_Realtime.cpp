/*
Code for running MAG256 from Teensy 4.1 Breakout Board, gets 6 kSPS
@samsh, UNSW
2024-25

24 MHz is the highest that can be reliably achieved without OC
At these speed, 16x oversampling requires EXTREMELY MINOR waits, barely slowing down sampling
At these speeds, 8140 us of wait gives 120.3 SPS.
Setting cycleWait to 50 uS achives 4 kHz without bany issue
The 'Transaction Window' is about 200 uS per 'sweep'
All of this was verified with an oscillosocpe

16x and 32x oversampling are misbehaving (wierd sequencing), so use 8x instead
I suspect this might be because an SPI transaction with no output to memory is much shorter;
It wouldn't use CPU clock cycles to move from buffer into memory
I need to invesitage futher... but pump up the waits and it still works fine

Interestingly, noise is bimodally distributed between X0 and X1 channels on 16 and 32x oversampling

NOTE: Write anywhere, but compile in Teensyduino. 600 MHz, FAST (no LTO) works best
*/

// Including standard libraries for SPI
#include <SPI.h>

// Things that you can adjust
constexpr int overSampleNum  = 8        ;   // Oversampling, can be 1 (i.e., none), 2, 4, 8, 16, 32
constexpr uint32_t spiClock  = 24e6     ;   // 24 MHz - highest the Teensy can do reliably, 48 MHz w/OC
constexpr uint32_t cycleWait = 150      ;    // Delay to limit cycle speed - in us - set to 15 ms for ~60 Hz - CRITICAL WHEN LARGE OVERSAMPLING!
#define SERIALDIAGNOSTICS 0                  // Gives some diagnostics over the serial port
#define CAPTURETIMINGS    0                  // Records crude timing information for packets

// Define the digital I/O Pins
// ADC A
#define CSa 10
#define A0a 9
#define A1a 8
#define A2a 7
#define A3a 6

// ADC B
#define CSb 38
#define A0b 25
#define A1b 24
#define A2b 40
#define A3b 41

// Non-default SPI Parameters
#define MISO1 39
#define MOSI1 26
#define SCK1  27

// Experimental multiplier -- SPI Bus efficiency, found using Scope
// Time taken = SPIefficiency*idealTime
#define SPIefficiency 1

// Defining Times
// Defining various wait times - in ns
constexpr uint32_t t_quiet    = 50     ;  // quiet time -- minimum is actually 20 but 50 ns is teensy minimum
constexpr uint32_t t_settle   = 1500    ;  // Acceptable minimum filter settling time
constexpr uint32_t t_isettle  = 500    ;  // Internal MUX settling time
constexpr uint32_t t_hardrst  = 800    ;  // Time to wait after hard reset before measurements
constexpr uint32_t t_softrst  = 250    ;  // Time to wait after soft reset before measurements
constexpr uint32_t t_powerup  = 10000  ;  // 10 ms

// Conversion Time
constexpr uint32_t calc_t_convert(int overSampleNum) {
  return 10 + 250*(overSampleNum-1);
}
constexpr uint32_t t_convert = calc_t_convert(overSampleNum);

// More Complex Wait Times
// Wait time between Internal ADC Mux'ed Inputs
constexpr uint32_t calc_wait1() {
    // 48 bits to transmit, SPI clock in Hz → convert to nanoseconds
    int32_t transmission_time = static_cast<int32_t>((48.0 / spiClock) * 1e9 * SPIefficiency);
    
    int32_t temp = (t_convert > t_isettle) ? 
        (static_cast<int32_t>(t_convert) - transmission_time) : 
        (static_cast<int32_t>(t_isettle) - transmission_time);
    
    return (temp > 0) ? static_cast<uint32_t>(temp) : 0;
}
constexpr uint32_t wait1 = calc_wait1();

constexpr uint32_t ioDelay = 500;

// Ensure there are only allowed oversampling values
static_assert(
  overSampleNum == 1 || overSampleNum == 2 || overSampleNum == 4 ||
  overSampleNum == 8 || overSampleNum == 16 || overSampleNum == 32,
  "Invalid oversampling value. Must be 1, 2, 4, 8, 16, or 32."
);

/**   Configuration Register 1
15  14    11  10  9   8     5   4   3   2   1   0
[1] [001] [0] [1] [0] [000] [0] [0] [0] [0] [1] [0]
R/W  adr  CH  SE  OS    #     CRC    AL  RES REF PMODE
*     no ALERT, no resolution boost, external reference, normal power mode             */
constexpr uint16_t setconfig1(int overSampleNum){
  switch(overSampleNum){
    case 1  : return 0x9402 ; // 1x  Sampling
    case 2  : return 0x9442 ; // 2x  Sampling
    case 4  : return 0x9482 ; // 4x  Sampling
    case 8  : return 0x94C2 ; // 8x  Sampling
    case 16 : return 0x9502 ; // 16x Sampling
    case 32 : return 0x9542 ; // 32x Sampling
    default : return 0xFFFF ; // fallback to make compiler happy
  }
}
constexpr uint16_t config1       = setconfig1(overSampleNum) ;
constexpr uint16_t config1_read  = 0x1000 ;

/**   Configuration Register 2
[1] [010] [00] [11] [00111100]
R/W  adr  res  SDP  RST 
*     1-wire SPI             */ 
constexpr uint16_t config2           = 0xA300  ;
constexpr uint16_t config2SoftReset  = 0xA33C  ;
constexpr uint16_t config2HardReset  = 0xA3FF  ;
constexpr uint16_t config2_read      = 0x2000  ;

// Lookup table Gray codes + Baud Rate
constexpr uint8_t gray[17]  = {0, 1, 3, 2, 6, 7, 5, 4, 12, 13, 15, 14, 10, 11, 9, 8, 0} ;
constexpr uint16_t zero     = 0xFFFF    ;   // What will be sent in 'empty' SPI transactions

// ADC Class
class ADC {
  private:
    uint8_t csPin;
    uint8_t a0Pin;
    uint8_t a1Pin;
    uint8_t a2Pin;
    uint8_t a3Pin;
    SPIClass* spi;
    
  public:
    // Constructor
    ADC(uint8_t cs, uint8_t a0, uint8_t a1, uint8_t a2, uint8_t a3, SPIClass* spiInterface) {
      csPin = cs;
      a0Pin = a0;
      a1Pin = a1;
      a2Pin = a2;
      a3Pin = a3;
      spi = spiInterface;
      
      // Initialize pins
      pinMode(csPin, OUTPUT);
      pinMode(a0Pin, OUTPUT);
      pinMode(a1Pin, OUTPUT);
      pinMode(a2Pin, OUTPUT);
      pinMode(a3Pin, OUTPUT);
      
      digitalWriteFast(csPin, HIGH);
      digitalWriteFast(a0Pin, LOW);
      digitalWriteFast(a1Pin, LOW);
      digitalWriteFast(a2Pin, LOW);
      digitalWriteFast(a3Pin, LOW);
    }
    
    // Set MUX
    void setMUX(uint8_t grayCode) {
      digitalWriteFast(a0Pin, grayCode & 0x01);
      digitalWriteFast(a1Pin, (grayCode >> 1) & 0x01);
      digitalWriteFast(a2Pin, (grayCode >> 2) & 0x01);
      digitalWriteFast(a3Pin, (grayCode >> 3) & 0x01);
    }

    void writeRegister(uint16_t reg) {
      digitalWriteFast(csPin, LOW);
      spi->transfer16(zero);
      digitalWriteFast(csPin, HIGH);

      delayNanoseconds(ioDelay);

      digitalWriteFast(csPin, LOW);
      spi->transfer16(reg);
      digitalWriteFast(csPin, HIGH);

      delayNanoseconds(ioDelay);
    }

    uint16_t readRegister(uint16_t reg) {
      digitalWriteFast(csPin, LOW);
      spi->transfer16(zero);
      digitalWriteFast(csPin, HIGH);

      delayNanoseconds(ioDelay);

      digitalWriteFast(csPin, LOW);
      spi->transfer16(reg);
      digitalWriteFast(csPin, HIGH);

      delayNanoseconds(ioDelay);

      digitalWriteFast(csPin, LOW);
      uint16_t result = spi->transfer16(zero);
      digitalWriteFast(csPin, HIGH);

      delayNanoseconds(ioDelay);

      digitalWriteFast(csPin, LOW);
      spi->transfer16(zero);
      digitalWriteFast(csPin, HIGH);

      delayNanoseconds(ioDelay);

      return result ;
    }

    void hardReset() {
      writeRegister(config2HardReset);
      delayNanoseconds(t_hardrst);
    }

    void softReset() {
      writeRegister(config2SoftReset);
      delayNanoseconds(t_softrst);
    }

    // Configure ADC
    void configADC() {
      const int MAX_RETRIES = 500; // maximum times to attempt rewriting

      // Validate config1
      bool config1Status = false;
      uint16_t config1Out;
      int retries = 0;
      while (!config1Status && retries < MAX_RETRIES) {
          writeRegister(config1);
          config1Out = readRegister(config1_read);
          // Masking sets address/rw to 0 (not expected back from ADC)
          if (config1Out == (config1 & 0x0FFF)) {
              config1Status = true;
          }
          retries++;
      }
      
      // Validate config2
      bool config2Status = false;
      uint16_t config2Out;
      retries = 0;  // Reset retry counter
      while (!config2Status && retries < MAX_RETRIES) { 
          writeRegister(config2);
          config2Out = readRegister(config2_read);
          // Masking sets address/rw to 0 (not expected back from ADC)
          if (config2Out == (config2 & 0x0FFF)) {
              config2Status = true;
          }
          retries++;
      }

      #if SERIALDIAGNOSTICS 
        Serial.println("ADC Diagnostics:");
        if (!config1Status) {
          Serial.print("ERROR: Config1 validation failed, last value: ");
          Serial.println(config1Out, HEX);
        }
        if (!config2Status) {
          Serial.print("ERROR: Config2 validation failed, last valule: ");
          Serial.println(config2Out, HEX);
        }
        if (config1Status && config2Status) {
          Serial.println("ADC configuration successful");
        }
      #endif
    }
    
    // Read Results
    void spiRead(uint16_t* dest) {
      digitalWriteFast(csPin, LOW);
      *dest       = spi->transfer16(zero);
      *(dest + 1) = spi->transfer16(zero);
      *(dest + 2) = spi->transfer16(zero);
      digitalWriteFast(csPin, HIGH);
    }
    void spiRead() {
      digitalWriteFast(csPin, LOW);
      spi->transfer16(zero);
      spi->transfer16(zero);
      spi->transfer16(zero);
      digitalWriteFast(csPin, HIGH);
    }
};

// Serial Packet Structure
struct serialPacket {
  // Sync Bytes
  uint8_t sync1 = 0xAA ;
  uint8_t sync2 = 0xBB ;
  uint8_t sync3 = 0xCC ;
  uint8_t sync4 = 0xDD ;

  // Counter - detect packet loss
  uint32_t count     = 0 ;
  uint32_t timeStamp = 0 ; // optionally used

  // Packet Information
  uint16_t dataSize =  384 ; // data size in uint16_t's
  uint8_t  version  = 0x01 ; // version 1
  uint8_t  flags    = 0x00 ; // unused for now 

  // Actual Data
  uint16_t data[192] = {0};

  // Spare space to pad to size -- can be used in future
  uint8_t spareData[104] = {0};

  // Space for CRC, unused for now
  uint32_t CRC = 0; 

  //End Sync Bytes
  uint8_t sync5 = 0xEE ;
  uint8_t sync6 = 0xFF ;
  uint8_t sync7 = 0x11 ;
  uint8_t sync8 = 0x22 ;

} __attribute__((packed));

static_assert(sizeof(serialPacket) == 512, "SerialPacket size must be exactly 512 bytes");

static_assert(offsetof(serialPacket, data) % alignof(uint16_t) == 0, 
              "data array must be properly aligned for uint16_t access");

serialPacket txPacket;

void transmit() {
  txPacket.count += 1;
  #if CAPTURETIMINGS 
    txPacket.timeStamp = micros();
  #endif
  Serial.write((uint8_t*)&txPacket, sizeof(serialPacket));
}

// Function to pulse both CS at once - slightly faster sometimes
void pulseBothCS() {
  digitalWriteFast(CSa, LOW)    ;
  digitalWriteFast(CSb, LOW)    ;
  delayNanoseconds(t_quiet)     ;
  digitalWriteFast(CSa, HIGH)   ;
  digitalWriteFast(CSb, HIGH)   ;
}

// Declaring 'adc's
ADC adcA(CSa, A0a, A1a, A2a, A3a, &SPI);
ADC adcB(CSb, A0b, A1b, A2b, A3b, &SPI1);

// Setup Function
void setup() {
  
  // Initialize Serial monitor for debugging
  Serial.begin(115200);
  // This baude rate is for compatibility
  // It does not do anything and is ignored when compiled for Teensy

  // Initialize SPI
  SPI.begin();
  SPI.beginTransaction(SPISettings(spiClock, MSBFIRST, SPI_MODE2));

  SPI1.setMISO(MISO1);
  SPI1.setMOSI(MOSI1);
  SPI1.setSCK(SCK1);
  SPI1.begin();
  SPI1.beginTransaction(SPISettings(spiClock, MSBFIRST, SPI_MODE2));

  // Wait for powerup
  delayMicroseconds(t_powerup) ;

  // Hard Resets
  adcA.hardReset();
  adcB.hardReset();

  #if SERIALDIAGNOSTICS
  // Wait for serial connection to be established
  while (!Serial)           ;

  Serial.print("wait1 = ")  ;
  Serial.println(wait1)     ;
  #endif

  // ADC CONFIG
  adcA.configADC();
  adcB.configADC();

  delayMicroseconds(cycleWait);
}

// Main Loop
void loop() {

  // Junk data out, these trigger measurements
  adcA.spiRead()                                ;
  adcB.spiRead()                                ;
  // Not the most efficient way,
  // but preferable due to always same timing
  for (int i = 0; i <= 15; i++) {

    if constexpr (wait1 > 0){
    delayNanoseconds(wait1)                     ;
    }

    // These all give warnings, but packet structure gaurantees alignment
    // see that static assert!
    #pragma GCC diagnostic ignored "-Waddress-of-packed-member"

    // Take Second Results + Read First Results
    //  + THEN change MUX
    adcA.spiRead(&txPacket.data[6*i + 0 ])      ;
    adcA.setMUX(gray[i+1])                      ;

    adcB.spiRead(&txPacket.data[6*i + 96])      ;
    adcB.setMUX(gray[i+1])                      ;

    if constexpr (wait1 > 0){
    delayNanoseconds(wait1)                     ;
    }

    // Taking later results
    adcA.spiRead(&txPacket.data[6*i + 3 ])      ;
    adcB.spiRead(&txPacket.data[6*i + 99])      ;
    // These number provide positioning in the packet
    // All of ADCa, THEN ADCb
  }

  // Transmit Data
  transmit()                                    ;
  
  // Resets Sequencer
  pulseBothCS()                                 ;

  delayMicroseconds(cycleWait)                   ;
}