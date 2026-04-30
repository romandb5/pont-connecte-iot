#include <Arduino.h>           // LIGNE OBLIGATOIRE SOUS PLATFORMIO
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "soc/soc.h"           
#include "soc/rtc_cntl_reg.h"  

// --- VOS IDENTIFIANTS WIFI ---
const char* ssid = "labo_snir";
const char* password = "snbaggio123";

WebServer serveur(80);

// --- CONFIGURATION DES BROCHES (AI-Thinker) ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// --- DÉCLARATION DES FONCTIONS (OBLIGATOIRE SOUS PLATFORMIO) ---
void gestionRacine();
void prendrePhoto();

void setup() {
  // 0. Désactivation de la sécurité de baisse de tension
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  
  Serial.begin(115200);
  delay(1000);

  // --- 1. INITIALISATION DE LA CAMÉRA ---
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  Serial.println("Démarrage de la caméra...");
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Erreur de la caméra avec le code 0x%x", err);
    return;
  }
  Serial.println("Caméra initialisée avec succès !");

  delay(2000); 

  // --- 2. INITIALISATION DU WIFI ---
  Serial.println("Connexion au WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnecté au WiFi !");
  Serial.print("Copiez ce lien dans votre navigateur : http://");
  Serial.println(WiFi.localIP());

  // --- 3. DÉMARRAGE DU SERVEUR WEB ---
  serveur.on("/", HTTP_GET, gestionRacine);
  serveur.on("/capture", HTTP_GET, prendrePhoto);
  serveur.begin();
}

void loop() {
  serveur.handleClient();
}

void gestionRacine() {
  String html = "<html><body style='text-align:center; background-color:#333; color:white;'>";
  html += "<h2>Ma Camera ESP32</h2>";
  html += "<img src='/capture' style='max-width:100%; border:2px solid white;'>";
  html += "</body></html>";
  serveur.send(200, "text/html", html);
}

void prendrePhoto() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Erreur de capture");
    serveur.send(500, "text/plain", "Erreur camera");
    return;
  }
  serveur.setContentLength(fb->len);
  serveur.send(200, "image/jpeg", "");
  serveur.client().write(fb->buf, fb->len);
  esp_camera_fb_return(fb); 
}