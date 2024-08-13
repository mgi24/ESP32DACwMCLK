#include "BluetoothA2DPSink.h"
#include "driver/i2s.h"
#include "audio_data.h"  // Include the header file with audio data


const int numtaps = 101; // Jumlah koefisien filter
float fir_coefficients[numtaps] = {
  0.0013314656590260256, 0.0013534178607180472, 0.0014058624152166591, 0.0014890566690132728, 0.001603117599609812, 0.001748020132548862, 0.0019235961389301357, 0.002129534122510832, 0.002365379602000791, 0.002630536190631662, 0.002924267371521364,
   0.0032456989637897594, 0.003593822270827496, 0.00396749789859631, 0.004365460228364659, 0.004786322524875693, 0.005228582657623999, 0.005690629409700414, 0.006170749345568882, 0.006667134206181491, 0.007177888797034355, 0.007701039332132867,
   0.008234542194384301, 0.008776293070682575, 0.009324136417906402, 0.009875875214229389, 0.010429280948549216, 0.010982103799491669, 0.011532082954341695, 0.01207695701740428, 0.012614474456707771, 0.01314240403763484, 0.013658545192004169,
   0.014160738271329456, 0.014646874633451518, 0.015114906512471481, 0.015562856622905546, 0.015988827450228827, 0.01639101018147253, 0.01676769323127602, 0.017117270320765824, 0.01743824806882636, 0.017729253057731056, 0.017989038337705557,
   0.01821648933778265, 0.018410629153267737, 0.018570623183247918, 0.018695783094831667, 0.018785570094181636, 0.018839597487883674, 0.018857632521761904, 0.018839597487883674, 0.018785570094181636, 0.018695783094831667,
   0.018570623183247915, 0.018410629153267737, 0.018216489337782648, 0.017989038337705553, 0.017729253057731053, 0.017438248068826358, 0.017117270320765824, 0.016767693231276012, 0.016391010181472526, 0.015988827450228827, 0.015562856622905545,
   0.015114906512471481, 0.014646874633451512, 0.014160738271329455, 0.013658545192004164, 0.013142404037634838, 0.01261447445670777, 0.012076957017404277, 0.011532082954341693, 0.010982103799491662, 0.01042928094854921, 0.009875875214229387,
   0.009324136417906399, 0.008776293070682574, 0.008234542194384294, 0.0077010393321328635, 0.007177888797034355, 0.006667134206181487, 0.006170749345568882, 0.005690629409700409, 0.005228582657623997, 0.004786322524875687, 0.004365460228364653,
   0.00396749789859631, 0.0035938222708274906, 0.003245698963789757, 0.00292426737152136, 0.0026305361906316604, 0.002365379602000791, 0.002129534122510828, 0.0019235961389301346, 0.00174802013254886, 0.0016031175996098108, 
  0.0014890566690132728, 0.001405862415216658, 0.0013534178607180472, 0.0013314656590260256
};
#define mutepin 23
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

float vReal[1000];
float bufferLeft[numtaps] = {0};
uint32_t bufferlen;
int bufferIndex = 0;
bool buf_go = 0;
bool i2s_go = 1;
void bufferprocess(void *parameter){
  Serial.print("BUFFER LOOP using core ");  Serial.print(xPortGetCoreID());
  Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));
  while(true){

    vTaskDelay(1/portTICK_PERIOD_MS);
  }
}
// Callback function to process received audio data

void audio_data_callback(const uint8_t *data, uint32_t len) {

  

  size_t i2s_bytes_written;
  // Serial.print("LEN ");
  // Serial.print(len);
  // Serial.print(" TIME ");
  // Serial.println(millis()-timer);
  // timer = millis();
  i2s_write(I2S_NUM_0, data, len, &i2s_bytes_written, portMAX_DELAY);
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
  
  pinMode(mutepin, OUTPUT);
  digitalWrite(mutepin, LOW);
  
  startup_sound();
  
  // Configure and initialize I2S driver
  i2s_driver_install(I2S_NUM_0, &i2s_config_stereo, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);


  // Set audio data callback function
  a2dp_sink.set_stream_reader(audio_data_callback, false);
  
  a2dp_sink.set_auto_reconnect(true);

  // Initialize Bluetooth A2DP sink with device name
  a2dp_sink.start("ESP32_A2DP_SINK");
  //RUNNING ON CORE 0
  xTaskCreatePinnedToCore(
    bufferprocess,     // Fungsi untuk task1
    "bufferprocess",   // Nama task1
    16000,     // Ukuran stack
    NULL,      // Parameter task1
    5,         // Prioritas task1
    &BufferProcess,    // Handle task1
    0          // Jalankan pada core 0
  );
}

void loop() {
  
  //MUTE
  if (a2dp_sink.get_audio_state()==2){
    digitalWrite(mutepin, HIGH);
  }
  else{
    digitalWrite(mutepin, LOW);
  }
}
