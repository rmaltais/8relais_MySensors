#include <Arduino.h>
#define MY_DEBUG
#define MY_RADIO_RF24
#define MY_REPEATER_FEATURE
#define MY_NODE_ID 12

#include <SPI.h>
#include <MySensors.h>
#include <LCD_I2C.h>
#include <TimeLib.h>

// -----------------------------------------------------------------------------
// Infos node
// -----------------------------------------------------------------------------
#define SN "Relais Remote"
#define SV "1.7"

// -----------------------------------------------------------------------------
// Matériel
// -----------------------------------------------------------------------------
#define NB_OUTPUTS 8
#define OUTPUT_SER 7
#define OUTPUT_RCLK 8
#define OUTPUT_SRCLK 4
#define OUTPUT_EN 6
#define HEARTBEAT_DELAY 3600000UL
#define LED_PIN 5
#define BUTTON_PIN 3

LCD_I2C lcd(0x27, 16, 2);

// -----------------------------------------------------------------------------
// États & timers
// -----------------------------------------------------------------------------
byte outputStates = 0;
bool initialValuesSent = false;
unsigned long lastHeartbeat = 0;

// Durées et timers (en secondes)
uint32_t relayDuration[NB_OUTPUTS] = {0};    // durée programmée
uint32_t remainingTime[NB_OUTPUTS] = {0};    // temps restant
uint32_t relayStartMillis[NB_OUTPUTS] = {0}; // millis() au démarrage
bool relayActive[NB_OUTPUTS] = {0};          // relais en timing ou non
unsigned long lastSendRemaining[NB_OUTPUTS] = {0};

// Messages MySensors
MyMessage msgStatus(0, V_STATUS);  // ON/OFF
MyMessage msgDuration(0, V_VAR1);  // durée configurée
MyMessage msgText(100, V_TEXT);    // ConfigText (R0=120, etc.)
MyMessage msgRemaining(0, V_VAR2); // temps restant
MyMessage msgSync(200, V_STATUS);

// -----------------------------------------------------------------------------
// 74HC595 — écriture centralisée
// -----------------------------------------------------------------------------
void writeShiftRegister(byte value)
{
  digitalWrite(OUTPUT_RCLK, LOW);
  shiftOut(OUTPUT_SER, OUTPUT_SRCLK, MSBFIRST, value);
  digitalWrite(OUTPUT_RCLK, HIGH);
}

// -----------------------------------------------------------------------------
// Change l’état d’un relais (via 74HC595)
// -----------------------------------------------------------------------------
void changeOutputState(uint8_t relay, bool state)
{
  bitWrite(outputStates, relay, state);
  writeShiftRegister(outputStates);
}

void initValves()
{
  if (!initialValuesSent && isTransportReady())

  {

    for (int i = 0; i < NB_OUTPUTS; i++)
    {

      msgStatus.setSensor(i);
      send(msgStatus.set(bitRead(outputStates, i)));

      // Durée initiale (V_VAR1)
      msgDuration.setSensor(i);
      send(msgDuration.set((uint32_t)5), true);
      wait(50);

      // État ON/OFF initial
      msgStatus.setSensor(i);
      send(msgStatus.set(false));
      wait(50);

      msgRemaining.setSensor(i);
      send(msgRemaining.set((uint32_t)0));
      wait(50);

      // Demander la valeur à HA
      request(i, V_VAR1);
      wait(2000, C_SET, V_VAR1);

      request(i, V_VAR2);
      wait(2000, C_SET, V_VAR2);
    }

    request(100, V_TEXT);
    wait(2000, C_SET, V_TEXT);

    initialValuesSent = true;
  }
}

void lightLcd()
{
  // -----------------------------------------------------------------------------
  // LCD — Séquence 2 pages (0–3 puis 4–7) parfaitement alignée
  // -----------------------------------------------------------------------------
  static uint8_t page = 0; // 0 = relais 0-3, 1 = relais 4-7
  static uint32_t lastSwitch = 0;
  uint32_t now = millis();

  // Changement automatique toutes les 2 secondes
  if (now - lastSwitch > 2000)
  {
    lastSwitch = now;
    page = !page;
  }

  // PAGE 1 : relais 1 à 4
  if (page == 0)
  {

    lcd.setCursor(0, 0);
    for (uint8_t i = 0; i < 4; i++)
    {
      uint8_t minutes = relayDuration[i] / 60;
      lcd.print(i + 1);
      lcd.print(":");
      if (minutes < 10)
        lcd.print("0");
      lcd.print(minutes);
    }

    for (uint8_t i = 0; i < 4; i++)
    {
      uint8_t col = i * 4;
      lcd.setCursor(col, 1);
      lcd.write(bitRead(outputStates, i) ? byte(0) : byte(1));
    }
  }

  else
  {

    lcd.setCursor(0, 0);
    for (uint8_t i = 4; i < 8; i++)
    {
      uint8_t minutes = relayDuration[i] / 60;
      lcd.print(i + 1);
      lcd.print(":");
      if (minutes < 10)
        lcd.print("0");
      lcd.print(minutes);
    }

    for (uint8_t i = 4; i < 8; i++)
    {
      uint8_t col = (i - 4) * 4;
      lcd.setCursor(col, 1);
      lcd.write(bitRead(outputStates, i) ? byte(0) : byte(1));
    }
  }
}

void gestTimer()
{

  // Gestion des timers
  uint32_t now = millis();

  for (int i = 0; i < NB_OUTPUTS; i++)
  {
    if (relayActive[i])
    {
      uint32_t elapsed = (now - relayStartMillis[i]) / 1000;

      if (elapsed >= relayDuration[i])
      {
        remainingTime[i] = 0;
        relayActive[i] = false;
        changeOutputState(i, false);
        msgStatus.setSensor(i);
        send(msgStatus.set(false));

        Serial.print("Relais ");
        Serial.print(i);
        Serial.println(" terminé");
      }
      else
      {
        remainingTime[i] = relayDuration[i] - elapsed;
      }

      // ENVOI LIMITÉ À 1 FOIS / SECONDE
      if (millis() - lastSendRemaining[i] >= 1000)
      {
        lastSendRemaining[i] = millis();
        msgRemaining.setSensor(i);
        send(msgRemaining.set((uint32_t)remainingTime[i]));
      }
    }
  }
}

void flashingLed()
{
  // ---------------------------------------------------------------------------
  // LED clignotante si un relais est ON
  // ---------------------------------------------------------------------------
  static uint32_t lastBlink = 0;
  static bool ledState = false;

  bool anyRelayOn = (outputStates != 0);

  if (anyRelayOn)
  {
    if (millis() - lastBlink >= 500) // clignote toutes les 500 ms
    {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  }
  else
  {
    // LED éteinte si aucun relais actif
    digitalWrite(LED_PIN, LOW);
    ledState = false;
  }
}

void syncLocal()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sync en cours...");

  // Confirmation LED
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    wait(150);
    digitalWrite(LED_PIN, LOW);
    wait(150);
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SYNC OK");
  wait(1000);
  lcd.clear();
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);

  pinMode(OUTPUT_SER, OUTPUT);
  pinMode(OUTPUT_RCLK, OUTPUT);
  pinMode(OUTPUT_SRCLK, OUTPUT);
  pinMode(OUTPUT_EN, OUTPUT);
  digitalWrite(OUTPUT_EN, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  writeShiftRegister(0); // Tous relais OFF au boot
  send(msgSync.set(0));

  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.print("Relais Remote");
  lcd.setCursor(0, 1);
  lcd.print("Boot...");

  byte carreePlein[8] = {
      B10001,
      B01010,
      B00100,
      B00100,
      B10101,
      B01110,
      B00100,
      B11111};

  byte triangle[8] = {
      B11111,
      B10001,
      B10011,
      B10101,
      B11001,
      B10001,
      B10001,
      B11111};

  lcd.createChar(0, carreePlein); // icône ON
  lcd.createChar(1, triangle);    // icône OFF
  lcd.clear();
}

// -----------------------------------------------------------------------------
// Presentation MySensors
// -----------------------------------------------------------------------------
void presentation()
{
  sendSketchInfo(SN, SV);

  // Canal texte pour commandes type "R0=120"
  present(100, S_INFO, "ConfigText");
  present(200, S_BINARY, "SyncRequest");

  // Chaque relais = S_CUSTOM (durée, etc.)
  for (int i = 0; i < NB_OUTPUTS; i++)
  {
    present(i, S_CUSTOM, "Delaiseconds");

    wait(50);
  }
}

// -----------------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------------
void loop()
{

  initValves();
  static bool lastButton = HIGH;
  bool currentButton = digitalRead(BUTTON_PIN);

  if (lastButton == HIGH && currentButton == LOW)
  {
    send(msgSync.set(1)); // demande de sync à HA
    wait(200);
    send(msgSync.set(0)); // reset du flag de sync
    syncLocal();         // sync local (LCD + LED)
  }

  lastButton = currentButton;

  gestTimer();

  // Heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_DELAY)
  {
    sendHeartbeat();
    lastHeartbeat = millis();
  }

  lightLcd();
  flashingLed();
}

// PAGE 2 : relais 5 à 8

// -----------------------------------------------------------------------------
// Réception MySensors
// -----------------------------------------------------------------------------
void receive(const MyMessage &message)
{
  uint8_t relay = message.sensor;

  switch (message.type)
  {
  case V_STATUS:
  { // ON/OFF depuis HA
    bool state = message.getBool();
    changeOutputState(relay, state);

    // Si ON et durée > 0 → démarrer le timer
    if (state && relayDuration[relay] > 0)
    {
      relayStartMillis[relay] = millis();
      remainingTime[relay] = relayDuration[relay];
      relayActive[relay] = true;
    }
    else if (!state)
    {
      relayActive[relay] = false;
      remainingTime[relay] = 0;
    }
    break;
  }

  case V_TEXT:
  {
    String txt = message.getString();

    if (txt.startsWith("R"))
    {
      int i = txt.substring(1, txt.indexOf("=")).toInt();
      int valeur = txt.substring(txt.indexOf("=") + 1).toInt();

      relayDuration[i] = valeur;

      Serial.print("Durée reçue pour R");
      Serial.print(i);
      Serial.print(" = ");
      Serial.println(valeur);
    }
    static uint8_t syncCount = 0;

    syncCount++;

    if (syncCount >= 8)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("SYNC OK");
      wait(1000);
      lcd.clear();
      syncCount = 0;
    }

    break;
  }

  case V_VAR1:
  {
    relayDuration[relay] = message.getULong();

    // Renvoi vers HA
    msgDuration.setSensor(relay);
    send(msgDuration.set((uint32_t)relayDuration[relay]));

    Serial.print("Durée modifiée pour relais ");
    Serial.print(relay);
    Serial.print(" = ");
    Serial.println(relayDuration[relay]);
    break;
  }

  default:
    break;
  }

  // Retour d’état ON/OFF
  msgStatus.setSensor(relay);
  msgStatus.set(bitRead(outputStates, relay));
  send(msgStatus);
}
