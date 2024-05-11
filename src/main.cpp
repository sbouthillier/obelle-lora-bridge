#include <Arduino.h>
#include <LoRaWan_APP.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <WiFi.h>
#include <HT_SSD1306Wire.h>
#include <Firebase_ESP_Client.h>
#include "lora_payload.pb.h"

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

//----------------------------------------------------------------
// LoRa
//----------------------------------------------------------------
#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             5        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here

static RadioEvents_t RadioEvents;

static uint8_t rxpacket[BUFFER_SIZE];

//----------------------------------------------------------------
// WiFi
//----------------------------------------------------------------
#define WIFI_SSID "BELL508"
#define WIFI_PASSWORD "3D33C9A35AA4"

//----------------------------------------------------------------
// Firebase
//----------------------------------------------------------------
// Firebase project API Key
#define API_KEY "AIzaSyAq9ehul4e8gp_lkTB6gSzZMIQicYbNEbI"

// RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://obelle-control-default-rtdb.firebaseio.com/"

#define USER_EMAIL "obelle-lora-bridge@gmail.com"
#define USER_PASSWORD "a1b2c3d4e5!"


// Define Firebase Data object
FirebaseData stream;
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variable to save USER UID
String uid;

// Variables to save database paths
String databasePath;

// Database child nodes
String tempPath = "/temperature";
String timePath = "/timestamp/.sv";

// Sensor path (where we'll save our readings)
String sensorPath = "/sensors/air/";

// JSON object to save sensor readings and timestamp
FirebaseJson json;

bool signupOK = false;

//----------------------------------------------------------------
// Display
//----------------------------------------------------------------
SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

//----------------------------------------------------------------
// Private data
//----------------------------------------------------------------
static uint32_t prevPacketId = 0;
static uint32_t errorCount   = 0;

static bool lora_idle = true;

//----------------------------------------------------------------
// Private functions
//----------------------------------------------------------------
static void OnRxDone(uint8_t *, uint16_t, int16_t, int8_t);
void initWiFi(void);
static void showStatus(uint32_t packetId, float temp, int16_t rssi, uint32_t errCnt);


void setup() {
    Serial.begin(115200);
    Mcu.begin();

    // Initialising the UI will init the display too.
    display.init();

    RadioEvents.RxDone = OnRxDone;

    Radio.Init( &RadioEvents );
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetRxConfig(MODEM_LORA,                       // modem
                      LORA_BANDWIDTH,                   // bandwidth
                      LORA_SPREADING_FACTOR,            // datarate
                      LORA_CODINGRATE,                  // coderate
                      0,                                // bandwidthAfc
                      LORA_PREAMBLE_LENGTH,             // preambleLen
                      LORA_SYMBOL_TIMEOUT,              // symbTimeout
                      LORA_FIX_LENGTH_PAYLOAD_ON,       // fixLen
                      0,                                // payloadLen
                      true,                             // crcOn
                      0,                                // freqHopOn
                      0,                                // hopPeriod
                      LORA_IQ_INVERSION_ON,             // iqInverted
                      true);                            // rxContinuous

    initWiFi();

    Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

    /* Assign the api key (required) */
    config.api_key = API_KEY;

    // Assign the user sign in credentials
    auth.user.email = USER_EMAIL; 
    auth.user.password = USER_PASSWORD;

    /* Assign the RTDB URL (required) */
    config.database_url = DATABASE_URL;

    /* Assign the callback function for the long running token generation task */
    config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

    // Assign the maximum retry of token generation
    config.max_token_generation_retry = 5;

    // Comment or pass false value when WiFi reconnection will control by your code or third party library e.g. WiFiManager
    Firebase.reconnectWiFi(true);

    // Since v4.4.x, BearSSL engine was used, the SSL buffer need to be set.
    // Large data transmission may require larger RX buffer, otherwise connection issue or data read time out can be occurred.
    fbdo.setBSSLBufferSize(4096 /* Rx buffer size in bytes from 512 - 16384 */, 1024 /* Tx buffer size in bytes from 512 - 16384 */);

    // Limit the size of response payload to be collected in FirebaseData
    fbdo.setResponseSize(2048);
    
    Firebase.begin(&config, &auth);

    //Server response read timeout in ms (1 sec - 1 min).
    config.timeout.serverResponse = 10 * 1000;

    showStatus(0, 0, -255, 0);

    Serial.println();
}

void loop() {
    if (lora_idle) {
        lora_idle = false;
        Radio.Rx(0);
    }

    Radio.IrqProcess();
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    memcpy(rxpacket, payload, size);

    /* Allocate space for the decoded message. */
    LoraPayload loraPayload = LoraPayload_init_default;
        
    /* Create a stream that reads from the buffer. */
    pb_istream_t stream = pb_istream_from_buffer(rxpacket, size);
        
    /* Now we are ready to decode the message. */
    if (pb_decode(&stream, LoraPayload_fields, &loraPayload)) {
        if (loraPayload.id != prevPacketId) {
            if (loraPayload.id > (prevPacketId + 1)) {
                ++errorCount;
            }

            prevPacketId = loraPayload.id;
            showStatus(loraPayload.id, loraPayload.temperature, rssi, errorCount);

            if (Firebase.ready()) {
                json.set(tempPath.c_str(), String(loraPayload.temperature)); 
                json.set(timePath, "timestamp");
                bool result = Firebase.RTDB.pushJSON(&fbdo, sensorPath.c_str(), &json);
                Serial.printf("Set json... %s\n", result ? "ok" : fbdo.errorReason().c_str());
            }
        }
    }
    else {
        Serial.printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));
    }

    digitalWrite(LED, HIGH);
    Radio.Sleep( );
    lora_idle = true;
}

void initWiFi(void)
{
    // Connect to Wi-Fi
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);

    String text = String("Connecting to Wi-Fi");
    display.drawString(0, 10, text);
    display.display();

    while (WiFi.status() != WL_CONNECTED)
    {
        text += ".";
        display.drawString(0, 10, text);
        display.display();
        delay(1000);
    }

    display.clear();
    display.drawString(0, 10,  "Connected!");

    text = "IP: " + WiFi.localIP().toString();
    display.drawString(0, 20, text);
    display.display();
    delay(3000);
}

static void showStatus(uint32_t packetId, float temp, int16_t rssi, uint32_t errCnt)
{
    // clear the display
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0,  "LoRa Bridge");
    display.drawString(0, 10, "Received packet ID " + (String)(packetId));
    display.drawString(0, 20, "Temperature: " + (String)(temp) + "°C");
    display.drawString(0, 30, "RSSI: " + String(rssi));
    display.drawString(0, 40, "Errors: " + String(errCnt));
    display.display();
}