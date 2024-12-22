// Copyright notice
// =========================================================
// Copyright (c) 2024 EmbedGenius
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// File description
// =========================================================
// main.cpp - Implementation of the main application for the LoRa Bridge application.
//
// This file contains the implementation of the main application which is used to
// receive LoRa packets and send their content to the RTDB.
//
// The application is event-driven, meaning that it waits for events from the
// LoRa module and the OLED display, and then reacts accordingly.

#include <Arduino.h>
#include <LoRaWan_APP.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <WiFiProv.h>
#include <WiFi.h>
#include <Preferences.h>
#include <HT_SSD1306Wire.h>
#include <Firebase_ESP_Client.h>
#include <OneButton.h>
#include <qrcode.h>
#include "lora_payload.pb.h"    // NOLINT
#include "gui.h"                // NOLINT

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

//----------------------------------------------------------------
// LoRa
//----------------------------------------------------------------
constexpr auto RF_FREQUENCY               {915000000};  // Hz

constexpr auto TX_OUTPUT_POWER            {5};          // dBm

constexpr auto LORA_BANDWIDTH             {0};          // [0: 125 kHz,
                                                        //  1: 250 kHz,
                                                        //  2: 500 kHz,
                                                        //  3: Reserved]
constexpr auto LORA_SPREADING_FACTOR      {7};          // [SF7..SF12]
constexpr auto LORA_CODINGRATE            {1};          // [1: 4/5,
                                                        //  2: 4/6,
                                                        //  3: 4/7,
                                                        //  4: 4/8]
constexpr auto LORA_PREAMBLE_LENGTH       {8};          // Same for Tx and Rx
constexpr auto LORA_SYMBOL_TIMEOUT        {0};          // Symbols
constexpr auto LORA_FIX_LENGTH_PAYLOAD_ON {false};
constexpr auto LORA_IQ_INVERSION_ON       {false};

constexpr auto RX_TIMEOUT_VALUE {1000};
constexpr auto BUFFER_SIZE      {30};                   // Define the payload size here

static RadioEvents_t RadioEvents;

//----------------------------------------------------------------
// Button
//----------------------------------------------------------------
constexpr auto BUTTON_IO {0};
OneButton button;

//----------------------------------------------------------------
// WiFi
//----------------------------------------------------------------
// cppcheck-suppress warning:whitespace/line_length
// uint8_t uuid[16] = {0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf, 0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02};
std::array<uint8_t, 16> uuid = {0xb4, 0xdf, 0x5a, 0x1c,
                                0x3f, 0x6b, 0xf4, 0xbf,
                                0xea, 0x4a, 0x82, 0x03,
                                0x04, 0x90, 0x1a, 0x02};

const String pop {"abcd1234"};                      // Proof of possession
const String service_name {"EG_LORA_BRIDGE"};       // Name of your device
// const String service_key  = nullptr;             // Password used for SofAP method (NULL = no password needed)

//----------------------------------------------------------------
// Definitions
//----------------------------------------------------------------
enum class AppEventId {
    APPRICATION_EVENT_NONE = 0,
    APPRICATION_EVENT_WIFI_CONNECTED,
    APPRICATION_EVENT_WIFI_DISCONNECTED,
    APPRICATION_EVENT_WIFI_CONNECTION_ERROR,
    APPRICATION_EVENT_WIFI_PROVISIONING_STARTED,
    APPRICATION_EVENT_BUTTON_CLICKED,
    APPRICATION_EVENT_BUTTON_PRESSED,
};

struct AppEventInfo_t {
    const void * data;
};

struct AppEvent_t {
    AppEventId      id;
    AppEventInfo_t  info;
};

using AppEvent = AppEvent_t;

//----------------------------------------------------------------
// Firebase
//----------------------------------------------------------------
// Firebase project API Key
const String API_KEY {"AIzaSyAq9ehul4e8gp_lkTB6gSzZMIQicYbNEbI"};

// RTDB URLefine the RTDB URL */
const String DATABASE_URL {"https://obelle-control-default-rtdb.firebaseio.com/"};

const String USER_EMAIL {"obelle-lora-bridge@gmail.com"};
const String USER_PASSWORD {"a1b2c3d4e5!"};

//----------------------------------------------------------------
// Private data
//----------------------------------------------------------------
// Define Firebase Data object
// FirebaseData stream;
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variable to save USER UID
const String uid;

// Variables to save database paths
const String databasePath {"water_tank"};
const String sensorPath {databasePath + "/sensors"};

// Database child nodes
// Water level sensor path
const String sensorWaterPath {sensorPath + "/water_level/"};
const String levelPath {"/level"};
const String distancePath {"/distance"};
const String errorPath {"/error"};
const String timePath {"/timestamp/.sv"};

// Temperature sensor path
const String sensorAirPath {sensorPath + "/air/"};
const String tempPath {"/temperature"};

// Time stamp path

// JSON object to save sensor readings and timestamp
static FirebaseJson json;

static bool signupOK = false;

static uint32_t prevPacketId = 0;
static uint32_t errorCount = 0;

static bool lora_idle = true;

static QueueHandle_t eventQueue;

//----------------------------------------------------------------
// Display
//----------------------------------------------------------------
//                   addr , freq , i2c group ,         resolution ,     rst
SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
OledGuiData guiData = oled_gui_data_init_default;
OledGui     gui(&display, &guiData);

//----------------------------------------------------------------
// Private functions
//----------------------------------------------------------------
static void initFirebase(void);
static void SysProvEvent(arduino_event_t *sys_event);
static void OnRxDone(uint8_t *, uint16_t, int16_t, int8_t);
void initWiFi(void);
static void showStatus(uint32_t packetId, uint8_t level, int16_t rssi, uint32_t errCnt);
static void buttonClick(void);
static void buttonPress(void);

/**
 * @brief Initializes the system components and configurations.
 * 
 * @note This function should be called once in the setup phase of the 
 *       application to ensure all components are properly initialized.
 */
void setup() {
    Serial.begin(115200);
    Mcu.begin();

    // setup OneButton
    button.setup(BUTTON_IO, INPUT, true);

    // link the doubleclick function to be called on a doubleclick event.
    button.attachClick(buttonClick);
    button.attachLongPressStart(buttonPress);

    Serial.println("LoRa Bridge");

    RadioEvents.RxDone = OnRxDone;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetRxConfig(MODEM_LORA,                   // modem
                      LORA_BANDWIDTH,               // bandwidth
                      LORA_SPREADING_FACTOR,        // datarate
                      LORA_CODINGRATE,              // coderate
                      0,                            // bandwidthAfc
                      LORA_PREAMBLE_LENGTH,         // preambleLen
                      LORA_SYMBOL_TIMEOUT,          // symbTimeout
                      LORA_FIX_LENGTH_PAYLOAD_ON,   // fixLen
                      0,                            // payloadLen
                      true,                         // crcOn
                      false,                        // freqHopOn
                      0,                            // hopPeriod
                      LORA_IQ_INVERSION_ON,         // iqInverted
                      true);                        // rxContinuous

    eventQueue = xQueueCreate(10, sizeof(AppEvent));

    if (eventQueue == nullptr) {
        Serial.println("Error creating the queue");
    }

    gui.init();
    gui.splashScreen();

    WiFi.onEvent(SysProvEvent);
    WiFi.setAutoReconnect(true);
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE,
                            WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
                            WIFI_PROV_SECURITY_1,
                            pop.c_str(),
                            service_name.c_str(),
                            nullptr,
                            uuid.data(),
                            false);

    Serial.println();
}

/**
 * @brief The main loop of the program.
 *
 * This function is called repeatedly after the setup() function. It is
 * responsible for processing any pending LoRa radio events and receiving any
 * incoming LoRa packets.
 *
 * When the radio is idle, this function starts the radio in receive mode.
 *
 * @note This function is called by the Arduino framework automatically.
 *       It should not be called directly.
 */
void loop() {
    AppEvent event;

    if (xQueueReceive(eventQueue, &event, 0) != 0) {
        switch (event.id) {
        case AppEventId::APPRICATION_EVENT_WIFI_CONNECTED:
            initFirebase();
            gui.refresh();
        break;

        case AppEventId::APPRICATION_EVENT_WIFI_DISCONNECTED:
            gui.showWifiDisconnectedScreen((const char *)event.info.data);
        break;

        case AppEventId::APPRICATION_EVENT_WIFI_PROVISIONING_STARTED:
            gui.showWifiProvScreen();
        break;

        case AppEventId::APPRICATION_EVENT_WIFI_CONNECTION_ERROR:
            wifi_prov_mgr_reset_sm_state_on_failure();
            gui.showWifiErrorScreen();
        break;

        case AppEventId::APPRICATION_EVENT_BUTTON_CLICKED:
            gui.nextScreen();
        break;

        case AppEventId::APPRICATION_EVENT_BUTTON_PRESSED:
            wifi_prov_mgr_deinit();
            wifi_prov_mgr_wait();
            wifi_prov_mgr_reset_provisioning();
            ESP.restart();
        break;

        default:
        break;
        }
    }

    button.tick();

    if (lora_idle) {
        lora_idle = false;
        Radio.Rx(0);
    }

    Radio.IrqProcess();
}

/**
 * @brief Initialize Firebase configuration and connection settings.
 *
 * This function sets up the Firebase client with the necessary configuration,
 * including API key, user credentials, database URL, and token generation settings.
 * It also initializes the Firebase connection and configures SSL buffer sizes and
 * response payload limits.
 */
static void initFirebase(void) {
    // Print the Firebase client version
    Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

    // Assign the API key for Firebase
    config.api_key = API_KEY;

    // Set user email and password for authentication
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;

    // Set the Firebase Realtime Database URL
    config.database_url = DATABASE_URL;

    // Assign the callback function for token status updates
    config.token_status_callback = tokenStatusCallback;     // see addons/TokenHelper.h

    // Set the maximum number of retries for token generation
    config.max_token_generation_retry = 5;

    // Enable automatic WiFi reconnection
    Firebase.reconnectWiFi(true);

    // Configure SSL buffer sizes for BearSSL engine
    // RX buffer size (bytes): 512 - 16384
    // TX buffer size (bytes): 512 - 16384
    fbdo.setBSSLBufferSize(4096, 1024);

    // Limit the size of response payload to be collected in FirebaseData
    fbdo.setResponseSize(2048);

    // Initialize Firebase with the provided configuration and authentication
    Firebase.begin(&config, &auth);

    // Set the server response read timeout in milliseconds (1 sec - 1 min)
    config.timeout.serverResponse = 10 * 1000;
}

/**
 * @brief The callback function for the provisioning event.
 *
 * This function is called when a provisioning event occurs. It is responsible
 * for handling the event and printing the corresponding message to the serial
 * console.
 *
 * @param sys_event The provisioning event.
 */
static void SysProvEvent(arduino_event_t *sys_event) {
    AppEvent event;

    switch (sys_event->event_id) {
    case ARDUINO_EVENT_WIFI_READY:
        Serial.print("\nWiFi interface ready\n");
    break;

    case ARDUINO_EVENT_WIFI_STA_START:
        Serial.print("\nWiFi station interface started\n");
    break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        Serial.print("\nConnected to the AP\n");
    break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        // Print the connected IP address
        Serial.print("\nConnected IP address : ");
        Serial.println(IPAddress(sys_event->event_info.got_ip.ip_info.ip.addr));

        event.id = AppEventId::APPRICATION_EVENT_WIFI_CONNECTED;
        xQueueSend(eventQueue, &event, 0);
        break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        // Print a message when the device is disconnected
        Serial.println("\nDisconnected. Connecting to the AP again... ");

        event.id        = AppEventId::APPRICATION_EVENT_WIFI_DISCONNECTED;
        event.info.data = WiFi.disconnectReasonName(
                              (wifi_err_reason_t)sys_event->event_info.wifi_sta_disconnected.reason);
        xQueueSend(eventQueue, &event, 0);
        break;

    case ARDUINO_EVENT_PROV_START:
        // Print a message when the provisioning starts
        Serial.println("\nProvisioning started\nGive Credentials of your access point using smartphone app");
        event.id = AppEventId::APPRICATION_EVENT_WIFI_PROVISIONING_STARTED;
        xQueueSend(eventQueue, &event, 0);
        break;

    case ARDUINO_EVENT_PROV_CRED_RECV:
        // Print a message when the provisioning credentials are received
        Serial.println("\nReceived Wi-Fi credentials");
        Serial.print("\tSSID : ");
        Serial.println((const char *)sys_event->event_info.prov_cred_recv.ssid);
        Serial.print("\tPassword : ");
        Serial.println((char const *)sys_event->event_info.prov_cred_recv.password);
        break;

    case ARDUINO_EVENT_PROV_CRED_FAIL:
        // Print a message when the provisioning fails
        Serial.println("\nProvisioning failed!\nPlease reset to factory and retry provisioning\n");

        if (sys_event->event_info.prov_fail_reason == WIFI_PROV_STA_AUTH_ERROR) {
            // Print a message when the provisioning fails due to an authentication error
            Serial.println("\nWi-Fi AP password incorrect");
        } else {
            // Print a message when the provisioning fails due to an unknown reason
            Serial.println("\nWi-Fi AP not found....Add API \" nvs_flash_erase() \" before beginProvision()");
        }

        event.id = AppEventId::APPRICATION_EVENT_WIFI_CONNECTION_ERROR;
        xQueueSend(eventQueue, &event, 0);
        break;

    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
        // Print a message when the provisioning is successful
        Serial.println("\nProvisioning Successful");
        break;

    case ARDUINO_EVENT_PROV_END:
        // Print a message when the provisioning ends
        Serial.println("\nProvisioning Ends");
        break;

    default:
        // Print a message when an unknown event occurs
        Serial.print("\nUnknown WiFi event!: ");
        Serial.println(sys_event->event_id);
        break;
    }
}

/**
 * @brief Called when a LoRa packet has been received.
 *
 * @param payload The received LoRa packet.
 * @param size The size of the received packet.
 * @param rssi The RSSI value of the received packet.
 * @param snr The SNR value of the received packet.
 *
 * Decodes the received packet as a LoraPayload message and displays the
 * received packet ID, water level and RSSI value on the OLED display.
 * If the Firebase Realtime Database is ready, the water level value is
 * written to the water level node in the Realtime Database.
 */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t /* snr */) {
    std::array<uint8_t, BUFFER_SIZE> rxpacket;
    rxpacket.fill(0);

    std::copy_n(payload, size, rxpacket.begin());

    /* Allocate space for the decoded message. */
    LoraPayload loraPayload = LoraPayload_init_default;

    /* Create a stream that reads from the buffer. */
    pb_istream_t stream = pb_istream_from_buffer(rxpacket.data(), size);

    /* Now we are ready to decode the message. */
    if (pb_decode(&stream, LoraPayload_fields, &loraPayload)) {
        if (loraPayload.id != prevPacketId) {
            if (loraPayload.id > (prevPacketId + 1)) {
                ++errorCount;
            }

            prevPacketId = loraPayload.id;

            guiData.info.water_level = loraPayload.level;
            guiData.stats.received_packet_id  = loraPayload.id;
            guiData.stats.receive_error_count = errorCount;
            guiData.stats.sensor_error_count  = loraPayload.err_sensor;
            guiData.stats.rssi = rssi;
            gui.refresh();

            if (Firebase.ready()) {
                json.set(levelPath.c_str(), String(loraPayload.level));
                json.set(distancePath.c_str(), String(loraPayload.distance));
                json.set(errorPath.c_str(), String(loraPayload.err_sensor));
                json.set(timePath, "timestamp");
                bool result = Firebase.RTDB.pushJSON(&fbdo, sensorWaterPath.c_str(), &json);
                Serial.printf("Set json... %s\n", result ? "ok" : fbdo.errorReason().c_str());
            }
        }
    } else {
        Serial.printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));
    }

    digitalWrite(LED, HIGH);
    Radio.Sleep();
    lora_idle = true;
}

/**
 * @brief Update the OLED display with the given packet ID, water level, RSSI and error count.
 *
 * @param packetId The ID of the received packet.
 * @param level The water level value received in the packet.
 * @param rssi The RSSI value of the received packet.
 * @param errCnt The number of errors since the last reset.
 */
static void showStatus(uint32_t packetId, uint8_t level, int16_t rssi, uint32_t errCnt) {
    // clear the display
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_10);
    display.drawString(DISPLAY_WIDTH / 2, 0, "LoRa Bridge");

    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawHorizontalLine(0, 12, DISPLAY_WIDTH);
    display.drawString(0, 20, "Water level: " + String(level) + " %");

    display.drawProgressBar(4, 40, 120, 10, level);
    display.display();
}

/**
 * @brief Send the APPRICATION_EVENT_BUTTON_CLICKED event when the button is clicked.
 *
 * This function is called by the OneButton library when the button is clicked.
 * It sends the APPRICATION_EVENT_BUTTON_CLICKED event to the event queue.
 */
static void buttonClick(void) {
    Serial.println("Button clicked");
    AppEventId event = AppEventId::APPRICATION_EVENT_BUTTON_CLICKED;
    xQueueSend(eventQueue, &event, 0);
}

/**
 * @brief Send the APPRICATION_EVENT_BUTTON_PRESSED event when the button is pressed.
 *
 * This function is called by the OneButton library when the button is pressed.
 * It sends the APPRICATION_EVENT_BUTTON_PRESSED event to the event queue.
 */
static void buttonPress(void) {
    Serial.println("Button pressed");
    AppEventId event = AppEventId::APPRICATION_EVENT_BUTTON_PRESSED;
    xQueueSend(eventQueue, &event, 0);
}
