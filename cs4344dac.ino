#include "BluetoothA2DPSink.h"
#include "driver/i2s.h"
#include "audio_data.h"  // Include the header file with audio data


//install all pin!!!
#define mutepin 32
#define mosfetpin 33
#define powerbtn 27
#define vbat 14
#define btled 5
#define lowbatled 22
#define clipled 23
#define ppnext 15 // install pullup 1K and 1K divider, 2 button in 1 pin
                  // ppnext + 1K - 3.3V
                  //        + 1K - BTN - GND
                  //        + BTN - GND
#define opprev 4  // copy ppnext




//handle task
TaskHandle_t BufferProcess;

BluetoothA2DPSink a2dp_sink;
long timer;
// Custom I2S pin configuration
i2s_pin_config_t pin_config = {
  .mck_io_num = 3,      // MCLK pin
  .bck_io_num = 19,    // BCK pin
  .ws_io_num = 21,     // LRC pin
  .data_out_num = 18,  // DATA out pin
  .data_in_num = I2S_PIN_NO_CHANGE // Not used
  
};

// Custom I2S configuration
i2s_config_t i2s_config_stereo = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), // TX only
  .sample_rate = 44100,   // Sample rate
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // Bits per sample
  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // Channel format
  .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_STAND_I2S),  // Communication format
  .intr_alloc_flags = 0,     // Interrupt allocation
  .dma_buf_count = 8,                           // DMA buffer count
  .dma_buf_len = 1024,                            // DMA buffer length
  .use_apll = true,                            // Use APLL
  .tx_desc_auto_clear = true,                   // Auto clear tx descriptor on underflow
  .fixed_mclk = 0                // Konfigurasi MCLK ke 11.2896 MHz
                                        
};
i2s_config_t i2s_config_mono = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), // TX only
  .sample_rate = 44100,   // Sample rate
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // Bits per sample
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_STAND_I2S),  // Communication format
  .intr_alloc_flags = 0,     // Interrupt allocation
  .dma_buf_count = 8,                           // DMA buffer count
  .dma_buf_len = 1024,                            // DMA buffer length
  .use_apll = true,                            // Use APLL
  .tx_desc_auto_clear = true,                   // Auto clear tx descriptor on underflow
  .fixed_mclk = 0                // Konfigurasi MCLK ke 11.2896 MHz
                                        
};






#define FILTER_TAP_NUM 29
#define LOG_SAMPLES 10
#define LOG_VOLT 30
#define BUFFER_SIZE 4000

const float filter_taps[FILTER_TAP_NUM] = {
  0.0012, 0.0027, 0.0063, 0.0121, 0.0203, 0.0304, 0.0415, 0.0526, 0.0625,
  0.0702, 0.0748, 0.0757, 0.0727, 0.0656, 0.0547, 0.0412, 0.0261, 0.0105,
  -0.0036, -0.0155, -0.0245, -0.0303, -0.0324, -0.0310, -0.0261, -0.0183,
  -0.0085, 0.0022, 0.0129
};


float amplitudeLog[LOG_SAMPLES];
float logvolt[LOG_VOLT];
int16_t audiobuff[BUFFER_SIZE];
size_t audiolen = 0;
float sampleBuffer[BUFFER_SIZE/2];
float filteredSamples[BUFFER_SIZE/2];

float maxVoltage(float volt) {
  // Geser semua elemen ke kiri
  for (int i = 0; i < LOG_VOLT - 1; i++) {
    logvolt[i] = logvolt[i + 1];
  }

  // Tambahkan amplitude baru di posisi terakhir
  logvolt[LOG_VOLT - 1] = volt;
  float maxVal = logvolt[0];
  // Iterasi melalui semua elemen untuk menemukan nilai tertinggi
  for (int i = 1; i < LOG_VOLT; i++) {
    if (logvolt[i] > maxVal) {
      maxVal = logvolt[i];
    }
  }
  return maxVal;
}


void logAmplitude(float amplitude) {
  // Geser semua elemen ke kiri
  for (int i = 0; i < LOG_SAMPLES - 1; i++) {
    amplitudeLog[i] = amplitudeLog[i + 1];
  }

  // Tambahkan amplitude baru di posisi terakhir
  amplitudeLog[LOG_SAMPLES - 1] = amplitude;
}

float findMinAmplitude() {
  float minVal = amplitudeLog[0];  // Asumsikan nilai pertama adalah minimum

  // Iterasi melalui semua elemen untuk menemukan nilai terendah
  for (int i = 1; i < LOG_SAMPLES; i++) {
    if (amplitudeLog[i] < minVal) {
      minVal = amplitudeLog[i];
    }
  }

  return minVal;
}


float batvolt = 0;
bool buffgo = 0;

void bufferprocess(void *parameter){
  Serial.print("BUFFER LOOP using core ");  Serial.print(xPortGetCoreID());
  Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));
  while(true){
    if (buffgo){
      size_t injectlen = audiolen;
      for (size_t i = 0; i < injectlen/2; i++){
        sampleBuffer[i] = (float)audiobuff[i*2] / 32768.0;  // Normalisasi data 16-bit
      }
      // Aplikasi filter FIR
      for (size_t i = 0; i < injectlen/2; i++) {
        filteredSamples[i] = 0;
        for (int j = 0; j < FILTER_TAP_NUM; j++) {
          if (i - j >= 0) {
            filteredSamples[i] += sampleBuffer[i - j] * filter_taps[j];
          }
        }
      }
      //amplitude
      float amplitude=0;
      for (size_t i = 0; i < injectlen/2; i++) {
        amplitude+=abs(filteredSamples[i]);
      }
      logAmplitude(amplitude);

      int ledbrightness = 10*(amplitude-findMinAmplitude());
      // Serial.print(findMinAmplitude()*5,6);
      // Serial.print(",");
      // Serial.println(amplitude*5,6);
      analogWrite(clipled, ledbrightness);
      //Serial.print(analogRead(vbat));
      
      batvolt = (float(analogRead(vbat)) - 1850) * (4.15 - 3.0) / (2800 - 1850) + 3.0;
      //Serial.print(batvolt);
      //Serial.print(",");
      //Serial.println(maxVoltage(batvolt));
      if (maxVoltage(batvolt)<3.40){
        digitalWrite(lowbatled,HIGH);
      }
      else{
        digitalWrite(lowbatled,LOW);
      }
    }
    
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}



// Callback function to process received audio data

void audio_data_callback(const uint8_t *data, uint32_t len) {
  // len is in bytes, divide by 4 to get the number of 16-bit stereo samples
  size_t num_samples = len / 4;
  audiolen = num_samples * 2;
  // Temporary buffer to store converted int16_t data
  int16_t i2s_data[num_samples * 2];

  for (size_t i = 0; i < num_samples; i++) {
    // Convert each stereo sample from uint8_t to int16_t
    int16_t left = (data[i * 4 + 1] << 8) | data[i * 4];        // Left channel
    int16_t right = (data[i * 4 + 3] << 8) | data[i * 4 + 2];   // Right channel

    // Store the converted data in the i2s_data buffer
    i2s_data[i * 2] = left;
    i2s_data[i * 2 + 1] = right;
  }
  memcpy(audiobuff, i2s_data, sizeof(i2s_data));
  size_t i2s_bytes_written;

  i2s_write(I2S_NUM_0, i2s_data, sizeof(i2s_data), &i2s_bytes_written, portMAX_DELAY);
}

void startup_sound(){
    i2s_driver_install(I2S_NUM_0, &i2s_config_mono, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    delay (1000);
    digitalWrite(mutepin, HIGH);

    const size_t chunk_size = 1024; // Size of each chunk
    size_t bytes_written;

    for (size_t i = 0; i < startup_len; i += chunk_size) {
        // Calculate the number of bytes to write
        size_t bytes_to_write = (startup_len - i) < chunk_size ? (startup_len - i) * sizeof(int16_t) : chunk_size * sizeof(int16_t);
        
        // Write the chunk to I2S
        i2s_write(I2S_NUM_0, (const char*)&startup_data[i], bytes_to_write, &bytes_written, portMAX_DELAY);

        // Check if all bytes were written
        if (bytes_written != bytes_to_write) {
            // Handle error: not all bytes were written
            Serial.println("Error: Not all bytes were written to I2S");
        }
    }
    // Stop I2S driver
    digitalWrite(mutepin, LOW);
    i2s_driver_uninstall(I2S_NUM_0);
}






void setup() {
  Serial.begin(115200);

  //pinmode selector
  pinMode(mosfetpin, OUTPUT);
  pinMode(mutepin, OUTPUT);
  pinMode(btled, OUTPUT);
  pinMode(clipled, OUTPUT);
  pinMode(lowbatled, OUTPUT);

  pinMode(vbat, INPUT);
  pinMode(powerbtn, INPUT);
  pinMode(ppnext, INPUT);
  pinMode(opprev, INPUT);

  //output pin default state
  digitalWrite(btled, LOW);
  digitalWrite(lowbatled, LOW);
  digitalWrite(mosfetpin, HIGH);
  digitalWrite(mutepin, LOW);
  analogWrite(clipled, 0);

  
  startup_sound();
  //delay(1000);
  
  // Configure and initialize I2S driver
  i2s_driver_install(I2S_NUM_0, &i2s_config_stereo, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);


  // Set audio data callback function
  a2dp_sink.set_stream_reader(audio_data_callback, false);
  
  a2dp_sink.set_auto_reconnect(true);

  // Initialize Bluetooth A2DP sink with device name
  a2dp_sink.start("Harman/Kardon Genie");
  uint8_t battery_level = 50;

  //RUNNING ON CORE 0
  xTaskCreatePinnedToCore(
    bufferprocess,     // Fungsi untuk task1
    "bufferprocess",   // Nama task1
    16000,     // Ukuran stack
    NULL,      // Parameter task1
    5,         // Prioritas task1
    &BufferProcess,    // Handle task1
    0          // Jalankan pada core 1
  );
}

long presstime=0;
long presstimeprev=0;
bool pausestate=0;
bool nextpressed=0;
bool ignorenext=0;
bool ignoreprev=0;
bool prevpressed=0;
int volume=0;
bool pppressed=0;
long pptime = 0;
bool ignorepp = 0;
bool isplay=1;
long optime=0;
bool oppress=0;
bool ignoreop = 0;
bool isconnected = 0;
bool turnoff = 0;





void loop() {
  
  

  if (a2dp_sink.is_connected()){
    isconnected = 1;
    volume = a2dp_sink.get_volume();
    digitalWrite(btled, LOW);

  }
  else{
    isconnected = 0;
    digitalWrite(btled, HIGH);

  }
  
  //MUTE
  if (a2dp_sink.get_audio_state()==2){
    
    if (volume==0 || isconnected == 0){
      digitalWrite(mutepin, LOW);
      buffgo=0;
    }
    else{
      digitalWrite(mutepin, HIGH);
      buffgo=1;
    }
    
  }
  else{
    buffgo = 0;
    digitalWrite(mutepin, LOW);
  }
  //play pause
  if (analogRead(ppnext)>1000 && analogRead(ppnext)<3000){
    pppressed = 1;
    pptime = millis();

    while (analogRead(ppnext)>1000 && analogRead(ppnext)<3000){
      if(millis()-pptime>10 && ignorepp == 0){
        ignorepp=1;
        if (isplay == 1){
          isplay=0;
          Serial.println("PAUSED");
          a2dp_sink.pause();
        }
        else{
          isplay=1;
          Serial.println("PLAYED");
          a2dp_sink.play();
        }
      }
    }
    ignorepp=0;
    pppressed = 0;
  }

  //operator button
  if (analogRead(opprev)>1000 && analogRead(opprev)<3000){
    oppress = 1;
    optime = millis();
    while(analogRead(opprev)>1000 && analogRead(opprev)<3000){
      if(millis()-optime>700 && ignoreop == 0){
        ignoreop=1;
        Serial.println("OP PRESS LONG, DISCONNECTING!");
        //add more things like disconnect bt
        a2dp_sink.disconnect();
        
      }
    }
  }


  //next + volup
  if (analogRead(ppnext)==0){
    
    nextpressed = 1;
    presstime = millis();
    while(analogRead(ppnext)==0){
      if(millis()-presstime>700 && ignorenext == 0){
        Serial.println("NEXT SONG");
        a2dp_sink.next();
        ignorenext=1;
      }
    }

  }
  if(analogRead(ppnext)>4000){
    if(millis()-presstime > 10 && ignorenext==0 && nextpressed == 1){
      Serial.println("VOL UP");
      if (volume<130){
        volume=volume+10;
        a2dp_sink.set_volume(volume);
        
      }
      if (volume>130){
        volume=130;
      }
    }
    ignorenext = 0;
    nextpressed = 0;
    
  }

  //prev+ vol
  if (analogRead(opprev)==0){
    
    prevpressed = 1;
    presstimeprev = millis();
    while(analogRead(opprev)==0){
      if(millis()-presstimeprev>700 && ignoreprev == 0){
        Serial.println("PREV SONG");
        a2dp_sink.previous();
        ignoreprev=1;
      }
    }

  }
  if(analogRead(opprev)>4000){
    if(millis()-presstimeprev > 10 && ignoreprev==0 && prevpressed == 1){
      Serial.println("VOL DOWN");
      if (volume>0){
        volume=volume-10;
        if (volume<=0){
        volume=0;
        }
        a2dp_sink.set_volume(volume);
        
      }
    }
    //op button bounce
    if(millis()-optime > 10 && ignoreop==0 && oppress == 1){
      Serial.println("OP PRESS QUICK");

    }
    ignoreprev = 0;
    prevpressed = 0;
    ignoreop = 0;
    oppress = 0;
  }

  
  Serial.println(analogRead(powerbtn));
  //SHUTDOWN
  if (analogRead(powerbtn)<1000){
    analogWrite(mutepin,LOW);
    delay(800);
    analogWrite(mosfetpin,LOW);
  } 
  
  
}
