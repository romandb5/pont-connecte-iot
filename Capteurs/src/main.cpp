#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <lmic.h>
#include <hal/hal.h>

// --- Configuration Matérielle ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define TDS_PIN 34           // Pin analogique pour le TDS
#define VREF 3.3             // Tension de référence ESP32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_MMA8451 mma = Adafruit_MMA8451();

// ====================================================================
// 1. TES CLÉS LORAWAN (Générées depuis ChirpStack)
// ====================================================================
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// DEVEUI en format LSB (Inversé)
static const u1_t PROGMEM DEVEUI[8] = { 0x0F, 0x76, 0xC8, 0xBE, 0x49, 0xFF, 0xB5, 0xD7 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// APPKEY en format MSB (Normal)
static const u1_t PROGMEM APPKEY[16] = { 0xBA, 0xC0, 0x41, 0x8B, 0x89, 0xF2, 0xBD, 0xA0, 0x9E, 0x10, 0x80, 0xAE, 0x0D, 0xF4, 0x1A, 0x31 };
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

// ====================================================================
// 2. VARIABLES ET MAPPING PINOUT
// ====================================================================
static uint8_t mydata[8]; // 6 octets (Accel) + 2 octets (TDS)
static osjob_t sendjob;
const unsigned TX_INTERVAL = 30;

const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN, 
    .rst = 23, 
    .dio = {26, 33, 32},
};

// Fonction d'affichage simplifiée
void displayMsg(String title, String details) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println(title);
    display.println("---------------------");
    display.println("");
    display.println(details);
    display.display();
}

// ====================================================================
// 3. LOGIQUE D'ACQUISITION ET ENVOI
// ====================================================================

void do_send(osjob_t* j){
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("TX en cours, attente..."));
    } else {
        // --- LECTURE ACCÉLÉROMÈTRE ---
        sensors_event_t event; 
        mma.getEvent(&event);
        int16_t x = event.acceleration.x * 100;
        int16_t y = event.acceleration.y * 100;
        int16_t z = event.acceleration.z * 100;

        // --- LECTURE TDS ---
        int rawADC = analogRead(TDS_PIN);
        float voltage = rawADC * VREF / 4095.0;
        // Formule de conversion tension -> ppm (simplifiée pour l'exemple)
        int16_t tdsValue = (int16_t)(voltage * 500); 

        // --- PRÉPARATION DU PAQUET (8 octets) ---
        mydata[0] = x >> 8; mydata[1] = x & 0xFF;
        mydata[2] = y >> 8; mydata[3] = y & 0xFF;
        mydata[4] = z >> 8; mydata[5] = z & 0xFF;
        mydata[6] = tdsValue >> 8; mydata[7] = tdsValue & 0xFF;

        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        
        displayMsg("LORA : ENVOI", "Capteurs : OK\nPaquet : 8 octets");
        Serial.println(F("Données envoyées !"));
    }
}

void onEvent (ev_t ev) {
    switch(ev) {
        case EV_JOINING: displayMsg("RESEAU", "Tentative de\nconnexion..."); break;
        case EV_JOINED:  displayMsg("RESEAU", "CONNECTE !\nPrêt pour l'envoi"); break;
        case EV_TXCOMPLETE:
            displayMsg("RESEAU", "Transmission OK\nProchain : 30s");
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        default: break;
    }
}

void setup() {
    // ... tes autres init
    analogSetAttenuation(ADC_11db); // Permet de lire jusqu'à 3.1V environ
    analogReadResolution(12);
    // ...
    Serial.begin(115200);
    Wire.begin(21, 22); // I2C pour MMA8451 et OLED

    // Init OLED
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { for(;;); }
    displayMsg("BOOT", "Demarrage...");

    // Init MMA8451
    if (!mma.begin()) {
        displayMsg("ERREUR", "MMA8451 absent !\nVerifiez I2C.");
        while (1);
    }
    
    // Init TDS
    analogReadResolution(12); // Resolution 4096

    // Init LoRaWAN
    os_init();
    LMIC_reset();
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}