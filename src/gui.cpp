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
// gui.cpp - Implementation of the GUI library for the LoRa Bridge application.
//
// This file contains the implementation of the GUI library which is used to
// display information on the built-in OLED display of the Heltec WiFi LoRa board.
//
// The library provides a simple interface to display different screens and
// refresh the display. The library also provides a simple interface to update
// the displayed information.

#include "gui.h"        // NOLINT
#include "logo.h"       // NOLINT


/**
 * @brief Constructs an OledGui object.
 *
 * @param guiDisplay Pointer to the SSD1306Wire object to use for the display.
 * @param guiData Pointer to the OledGuiData object to use to store the display data.
 */
OledGui::OledGui(SSD1306Wire *guiDisplay, OledGuiData *guiData) {
    display = guiDisplay;
    data    = guiData;
}

/**
 * @brief Initializes the OLED display.
 *
 * This function sets up the OLED display by calling its init method,
 * preparing it for subsequent operations such as drawing text or images.
 */
void OledGui::init() {
    display->init();
}

/**
 * @brief Draws a header on the OLED display.
 *
 * @param header The header text to display.
 */
void OledGui::screenHeader(String header) {
    display->clear();
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->setFont(ArialMT_Plain_10);
    display->drawString(DISPLAY_WIDTH / 2, 0, header);

    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->drawHorizontalLine(0, 12, DISPLAY_WIDTH);
}

/**
 * @brief Show the info screen, displaying the water level as a string and a progress bar.
 */
void OledGui::showInfoScreen() {
    screenHeader("Info");

    String waterLevelStr = "Water level: " + String(data->info.water_level) + " %";
    display->drawString(0, 20, waterLevelStr);

    display->drawProgressBar(4, 40, 120, 10, data->info.water_level);
    display->display();
}


/**
 * @brief Show the stats screen, displaying the received packet ID, RSSI, and error count.
 */
void OledGui::showStatsScreen() {
    screenHeader("Stats");

    String packetIdStr = "Received Packet ID: " + String(data->stats.received_packet_id);
    String rssiStr = "RSSI: " + String(data->stats.rssi);
    String rxErrorsStr = "Rx Errors: " + String(data->stats.receive_error_count);
    String sensorErrorsStr = "Sensor Errors: " + String(data->stats.sensor_error_count);

    display->drawString(0, 20, packetIdStr);
    display->drawString(0, 30, rssiStr);
    display->drawString(0, 40, rxErrorsStr);
    display->drawString(0, 50, sensorErrorsStr);
    display->display();
}

/**
 * @brief Display a screen prompting the user to configure WiFi.
 *
 * This function displays a message on the OLED screen indicating that 
 * the WiFi setup is required and instructs the user to use the mobile 
 * application for configuration.
 */
void OledGui::showWifiProvScreen() {
    screenHeader("WiFi Setup");

    display->drawStringMaxWidth(0, 20, DISPLAY_WIDTH,
        "Le WiFi a besoin d'être configuré. "
        "Veuillez utilisez l'application mobile.");
    display->display();
}

/**
 * @brief Display a screen indicating that there was an error during the WiFi connection.
 *
 * This function displays a message on the OLED screen indicating that an error occurred
 * during the WiFi connection and instructs the user to reset the configuration.
 */
void OledGui::showWifiErrorScreen() {
    screenHeader("WiFi Error");

    display->drawStringMaxWidth(0, 20, DISPLAY_WIDTH,
        "Erreur lors de la connection WiFi. "
        "Veuillez réinitialiser la configuration.");
    display->display();
}

/**
 * @brief Display a screen indicating that the WiFi connection has been disconnected.
 *
 * @param[in] reason The reason for the disconnection.
 */
void OledGui::showWifiDisconnectedScreen(const char *reason) {
    screenHeader("WiFi Déconnecté");

    display->drawStringMaxWidth(0, 20, DISPLAY_WIDTH, String(reason));
    display->display();
}

/**
 * @brief Display a splash screen on the OLED display.
 *
 * This function clears the display, draw the logo in binary format and displays it.
 */
void OledGui::splashScreen() {
    display->clear();
    display->drawXbm(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, logo);
    display->display();
    delay(2000);

    display->clear();
    int16_t x = DISPLAY_WIDTH  / 2;

    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->setFont(ArialMT_Plain_24);
    display->drawString(x, 2, "LoRa Bridge");

    display->setFont(ArialMT_Plain_10);
    display->drawString(x, 30, "by");

    display->setFont(ArialMT_Plain_10);
    display->drawString(x, 45, "Embed Genius");

    display->display();
    delay(2000);
}

/**
 * @brief Refresh the current screen.
 *
 * This function is a wrapper around the two show screens functions. It
 * refreshes the current screen by calling the relevant show screen
 * function.
 */
void OledGui::refresh() {
    if (current_screen == 0) {
        showInfoScreen();
    } else if (current_screen == 1) {
        showStatsScreen();
    }
}

/**
 * @brief Switch to the next screen.
 *
 * This function switches the current screen from the Info screen to the Stats
 * screen and vice versa. It calls the refresh() function to update the
 * display.
 */
void OledGui::nextScreen() {
    if (current_screen == 0) {
        current_screen = 1;
    } else if (current_screen == 1) {
        current_screen = 0;
    }

    refresh();
}
