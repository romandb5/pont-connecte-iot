#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <lmic.h>
#include <hal/hal.h>

// --- NOUVEAU : Bibliothèques pour le capteur de température ---
#include <OneWire.h>
#include <DallasTemperature.h>

// --- Configuration Matérielle ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define TDS_PIN 34      // Pin analogique pour le TDS
#define LEVEL_PIN 35           // Pin analogique pour le SEN0257
#define VREF 3.3             // Tension de référence ESP32

// --- NOUVEAU : Configuration DS18B20 ---
#define ONE_WIRE_BUS 13 
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_MMA8451 mma = Adafruit_MMA8451();

// ====================================================================
// 1. TES CLÉS LORAWAN 
// ====================================================================
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

static const u1_t PROGMEM DEVEUI[8] = { 0x0F, 0x76, 0xC8, 0xBE, 0x49, 0xFF, 0xB5, 0xD7 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

static const u1_t PROGMEM APPKEY[16] = { 0xBA, 0xC0, 0x41, 0x8B, 0x89, 0xF2, 0xBD, 0xA0, 0x9E, 0x10, 0x80, 0xAE, 0x0D, 0xF4, 0x1A, 0x31 };
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// ====================================================================
// 2. VARIABLES ET MAPPING PINOUT
// ====================================================================
static uint8_t mydata[8]; // On a largement la place (on va utiliser 6 octets)
static osjob_t sendjob;
const unsigned TX_INTERVAL = 30;

const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN, 
    .rst = 23, 
    .dio = {26, 33, 32},
};

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
        // --- 1. LECTURE ACCÉLÉROMÈTRE ---
        float max_G = 0.0;
        float min_G = 10000.0; 

        for(int i = 0; i < 50; i++) {
            sensors_event_t event; 
            mma.getEvent(&event);
            float magnitude = sqrt(event.acceleration.x * event.acceleration.x + 
                                   event.acceleration.y * event.acceleration.y + 
                                   event.acceleration.z * event.acceleration.z);
            if(magnitude > max_G) max_G = magnitude;
            if(magnitude < min_G) min_G = magnitude;
            delay(2);
        }
        float delta_vibration = max_G - min_G; 
        uint16_t vibrationLevel = (uint16_t)(delta_vibration * 100);

        // --- 2. LECTURE TDS ---
        int rawADC = analogRead(TDS_PIN);
        float voltage = rawADC * VREF / 4095.0;
        uint16_t tdsValue = (uint16_t)(voltage * 500); 

        // --- 3. LECTURE TEMPÉRATURE DS18B20 ---
        sensors.requestTemperatures(); 
        float tempC = sensors.getTempCByIndex(0);
        
        // On multiplie par 100 pour garder 2 décimales. 
        // On utilise int16_t (signé) au cas où la température descend en dessous de zéro !
        int16_t tempPayload = (int16_t)(tempC * 100);

        // --- 4. NOUVEAU : LECTURE NIVEAU D'EAU (SEN0257) ---
        int rawLevelADC = analogRead(LEVEL_PIN);
        float levelVoltage = rawLevelADC * VREF / 4095.0;
        
        // On multiplie par 1000 pour envoyer des millivolts (ex: 2.15V devient 2150 mV)
        uint16_t levelPayload = (uint16_t)(levelVoltage * 1000);

        // --- 5. PRÉPARATION DU PAQUET (Maintenant 8 octets !) ---
        mydata[0] = vibrationLevel >> 8; 
        mydata[1] = vibrationLevel & 0xFF;
        
        mydata[2] = tdsValue >> 8; 
        mydata[3] = tdsValue & 0xFF;

        mydata[4] = tempPayload >> 8;
        mydata[5] = tempPayload & 0xFF;

        // NOUVEAU : Ajout du niveau d'eau
        mydata[6] = levelPayload >> 8;
        mydata[7] = levelPayload & 0xFF;

        // On passe la taille du payload à 8 !
        LMIC_setTxData2(1, mydata, 8, 0);
        
        displayMsg("LORA : ENVOI", "Capteurs OK\nPaquet : 8 octets");
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
    analogSetAttenuation(ADC_11db); 
    analogReadResolution(12);
    Serial.begin(115200);
    
    Wire.begin(21, 22); 
    Wire.setClock(100000); 
    delay(200);            

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { for(;;); }
    displayMsg("BOOT", "Demarrage...");

    if (!mma.begin()) {
        displayMsg("ERREUR", "MMA8451 absent !\nVerifiez I2C.");
        while (1);
    }
    
    // NOUVEAU : Démarrage du capteur de température
    sensors.begin();

    os_init();
    LMIC_reset();
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
