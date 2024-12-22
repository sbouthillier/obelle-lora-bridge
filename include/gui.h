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
// gui.h - Header file containing the interface for the GUI library.

#ifndef INCLUDE_GUI_H_
#define INCLUDE_GUI_H_

#include <HT_SSD1306Wire.h>


typedef struct {
    uint32_t received_packet_id;
    uint32_t receive_error_count;
    uint32_t sensor_error_count;
    int16_t  rssi;
} oled_gui_stats_t;

typedef struct {
    uint8_t  water_level;
} oled_gui_info_t;

typedef struct {
    oled_gui_info_t  info;
    oled_gui_stats_t stats;
} oled_gui_data_t;

using OledGuiData = oled_gui_data_t;

#define oled_gui_data_init_default       {0, 0, 0, 0}

/**
 * @brief The OledGui class provides a simple interface to display information on the built-in
 *        OLED display.
 */
class OledGui {
 private:
    SSD1306Wire *display;
    OledGuiData *data;

    uint8_t current_screen {0};

    void showInfoScreen();
    void showStatsScreen();
    void screenHeader(String header);

 public:
    OledGui(SSD1306Wire *guiDisplay, OledGuiData *guiData);

    void init();
    void showWifiProvScreen();
    void showWifiErrorScreen();
    void showWifiDisconnectedScreen(const char *reason);
    void splashScreen();
    void refresh();
    void nextScreen();
};

#endif  // INCLUDE_GUI_H_
