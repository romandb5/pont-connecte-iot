#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <lmic.h>
#include <hal/hal.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --- Configuration Matérielle ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define TDS_PIN 34           
#define LEVEL_PIN 35         
#define ONE_WIRE_BUS 13 
#define VREF 3.3             
#define IR_PIN 14

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// DÉCLARATION DES 2 ACCÉLÉROMÈTRES
Adafruit_MMA8451 mma1 = Adafruit_MMA8451();
Adafruit_MMA8451 mma2 = Adafruit_MMA8451();

// ====================================================================
// 1. CLÉS LORAWAN 
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
// Le tableau passe à 11 octets (2 octets supplémentaires pour MMA2)
static uint8_t mydata[11]; 
static osjob_t sendjob;
const unsigned TX_INTERVAL = 10; 

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
        // --- 1A. LECTURE MMA8451 N°1 ---
        sensors_event_t event1; 
        mma1.getEvent(&event1);
        float mag1 = sqrt(event1.acceleration.x * event1.acceleration.x + 
                          event1.acceleration.y * event1.acceleration.y + 
                          event1.acceleration.z * event1.acceleration.z);
        float vib1_inst = abs(mag1 - 9.81);
        uint16_t vibration1_Level = (uint16_t)(vib1_inst * 100);

        // --- 1B. LECTURE MMA8451 N°2 ---
        sensors_event_t event2; 
        mma2.getEvent(&event2);
        float mag2 = sqrt(event2.acceleration.x * event2.acceleration.x + 
                          event2.acceleration.y * event2.acceleration.y + 
                          event2.acceleration.z * event2.acceleration.z);
        float vib2_inst = abs(mag2 - 9.81);
        uint16_t vibration2_Level = (uint16_t)(vib2_inst * 100);

        // --- 2. LECTURE TDS ---
        int rawADC = analogRead(TDS_PIN);
        float voltage = rawADC * VREF / 4095.0;
        uint16_t tdsValue = (uint16_t)(voltage * 500);

        // --- 3. LECTURE TEMPÉRATURE ---
        sensors.requestTemperatures();
        float tempC = sensors.getTempCByIndex(0);
        int16_t tempPayload = (int16_t)(tempC * 100);

        // --- 4. LECTURE NIVEAU D'EAU ---
        int rawLevelADC = analogRead(LEVEL_PIN);
        float levelVoltage = rawLevelADC * VREF / 4095.0;
        uint16_t levelPayload = (uint16_t)(levelVoltage * 1000);

        // --- 5. LECTURE CAPTEUR INFRAROUGE ---
        bool obstaclePresence = !digitalRead(IR_PIN);

        // --- 6. PRÉPARATION DU PAQUET (11 octets) ---
        mydata[0] = vibration1_Level >> 8; 
        mydata[1] = vibration1_Level & 0xFF;
        
        // Ajout du 2eme capteur
        mydata[2] = vibration2_Level >> 8; 
        mydata[3] = vibration2_Level & 0xFF;

        mydata[4] = tdsValue >> 8; 
        mydata[5] = tdsValue & 0xFF;

        mydata[6] = tempPayload >> 8;
        mydata[7] = tempPayload & 0xFF;

        mydata[8] = levelPayload >> 8;
        mydata[9] = levelPayload & 0xFF;

        mydata[10] = obstaclePresence ? 1 : 0;

        LMIC_setTxData2(1, mydata, 11, 0);
        
        displayMsg("LORA : ENVOI", "Capteurs OK\nPaquet : 11 octets");
        Serial.println(F("Données envoyées !"));
    }
}

void onEvent (ev_t ev) {
    switch(ev) {
        case EV_JOINING: displayMsg("RESEAU", "Tentative de\nconnexion..."); break;
        case EV_JOINED:  displayMsg("RESEAU", "CONNECTE !\nPrêt pour l'envoi"); break;
        case EV_TXCOMPLETE:
            displayMsg("RESEAU", "Transmission OK\nProchain : 10s");
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

    pinMode(IR_PIN, INPUT_PULLUP);

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { for(;;); }
    displayMsg("BOOT", "Demarrage...");

    // DÉMARRAGE DES DEUX CAPTEURS AVEC LEURS ADRESSES RESPECTIVES
    bool erreur_mma = false;
    
    if (!mma1.begin(0x1D)) { // Adresse par défaut
        Serial.println("MMA1 (0x1D) introuvable");
        erreur_mma = true;
    }
    
    if (!mma2.begin(0x1C)) { // Adresse modifiée
        Serial.println("MMA2 (0x1C) introuvable");
        erreur_mma = true;
    }

    if (erreur_mma) {
        displayMsg("ERREUR", "Un MMA8451 (ou plus)\nest absent !");
    }
    
    sensors.begin();
    os_init();
    LMIC_reset();
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
