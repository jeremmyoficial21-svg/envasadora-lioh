/*
  Envasadora LiOH (maqueta) - Pesaje y enlace HMI <-> PLC
  Placa: Arduino Uno o Nano
  Libreria: "HX711 Arduino Library" de Bogdan Necula (Gestor de librerias)

  Que hace:
   - Lee la celda de carga con el HX711 y calcula el peso en gramos.
   - Envia el peso y la etapa del PLC a la HMI (computador) por USB, 10 veces por segundo.
   - Recibe los botones de la HMI (PARTIR, MARCHA, PARO, RESET) y los pasa al PLC con reles.
   - Avisa al PLC con un rele cuando el peso llega al punto de corte (meta - compensacion).
   - Avisa al PLC con otro rele si el peso final queda fuera de tolerancia.
   - Hace la tara cuando el PLC activa la salida Y10.

  Conexiones (resumen):
   HX711:  DT -> A0, SCK -> A1, VCC -> 5V, GND -> GND
   Reles hacia el PLC (modulo de reles 5 V, contacto NA en serie con la entrada X):
     D2 -> X20 PARTIR   D3 -> X21 MARCHA   D4 -> X22 PARO
     D5 -> X23 RESET    D6 -> X24 PESO DE CORTE   D7 -> X25 FUERA DE TOLERANCIA
   Senales desde el PLC (modulo optoacoplador PC817, salida baja = activo):
     Y10 TARAR -> D8    Y20 -> D9   Y21 -> D10   Y22 -> D11   Y23 -> D12 (codigo de etapa)
     Y11 CICLO ACTIVO -> A2

  Comandos por el monitor serie (115200 baudios, fin de linea "Nueva linea"):
   P  = PARTIR      M = MARCHA      S = PARO      R = RESET
   SP 500 = meta en gramos         CO 50 = compensacion (corte anticipado) en gramos
   TL 15  = tolerancia +/- gramos  Z = poner en cero (sin saco)
   CAL0   = paso 1 de calibracion (celda vacia)
   CAL 500 = paso 2 de calibracion (con un peso conocido de 500 g encima)
*/

#include <HX711.h>
#include <EEPROM.h>

// ---------- Pines ----------
const uint8_t PIN_DT  = A0;
const uint8_t PIN_SCK = A1;

const uint8_t R_PARTIR = 2;
const uint8_t R_MARCHA = 3;
const uint8_t R_PARO   = 4;
const uint8_t R_RESET  = 5;
const uint8_t R_CORTE  = 6;
const uint8_t R_TOL    = 7;

const uint8_t IN_TARA = 8;
const uint8_t IN_B0   = 9;
const uint8_t IN_B1   = 10;
const uint8_t IN_B2   = 11;
const uint8_t IN_B3   = 12;
const uint8_t IN_RUN  = A2;

// La mayoria de los modulos de reles chinos se activan con nivel BAJO.
// Si tu modulo se activa con nivel ALTO, cambia esto a HIGH.
const uint8_t RELE_ON  = LOW;
const uint8_t RELE_OFF = (RELE_ON == LOW) ? HIGH : LOW;

const unsigned long PULSO_MS = 300;   // duracion del pulso de los botones de la HMI

// ---------- Parametros guardados en EEPROM ----------
struct Config {
  uint16_t marca;      // 0xA5A5 = configuracion valida
  float factor;        // factor de calibracion del HX711
  int16_t meta;        // gramos
  int16_t comp;        // gramos de compensacion
  int16_t tol;         // +/- gramos
};
Config cfg;

HX711 balanza;
float peso = 0;               // peso filtrado en gramos
bool pesoValido = false;

// pulsos no bloqueantes
unsigned long finPulso[8] = {0};

// lectura de etapa con filtro
uint8_t etapa = 0, etapaLeida = 0, etapaPrevia = 0;
uint8_t repeticiones = 0;
bool cicloActivo = false;
bool taraPrevia = false;

// registro del ultimo saco
float pesoEnCorte = 0;
float pesoFinal = 0;

unsigned long ultimoEnvio = 0;
String linea = "";

void guardarConfig() { EEPROM.put(0, cfg); }

void cargarConfig() {
  EEPROM.get(0, cfg);
  if (cfg.marca != 0xA5A5 || isnan(cfg.factor) || cfg.factor == 0) {
    cfg.marca = 0xA5A5;
    cfg.factor = 420.0;   // valor inicial aproximado; calibrar con CAL0 y CAL 500
    cfg.meta = 500;
    cfg.comp = 50;
    cfg.tol = 15;
    guardarConfig();
  }
}

void pulsar(uint8_t pin) {
  digitalWrite(pin, RELE_ON);
  finPulso[pin - 2] = millis() + PULSO_MS;
}

void atenderPulsos() {
  for (uint8_t pin = R_PARTIR; pin <= R_RESET; pin++) {
    if (finPulso[pin - 2] && (long)(millis() - finPulso[pin - 2]) >= 0) {
      digitalWrite(pin, RELE_OFF);
      finPulso[pin - 2] = 0;
    }
  }
}

bool activo(uint8_t pin) { return digitalRead(pin) == LOW; }  // optoacoplador tira a GND

uint8_t leerEtapa() {
  return (activo(IN_B0) ? 1 : 0) | (activo(IN_B1) ? 2 : 0) |
         (activo(IN_B2) ? 4 : 0) | (activo(IN_B3) ? 8 : 0);
}

void leerPeso() {
  if (!balanza.is_ready()) return;
  float g = balanza.get_units(1);
  if (!pesoValido) { peso = g; pesoValido = true; }
  else peso = peso * 0.6 + g * 0.4;   // filtro simple
}

void tarar() {
  balanza.tare(5);   // unos 0,5 s a 10 lecturas por segundo
  peso = 0;
}

void procesarComando(String c) {
  c.trim();
  c.toUpperCase();
  if (c == "P") pulsar(R_PARTIR);
  else if (c == "M") pulsar(R_MARCHA);
  else if (c == "S") pulsar(R_PARO);
  else if (c == "R") pulsar(R_RESET);
  else if (c == "Z") tarar();
  else if (c.startsWith("SP ")) { cfg.meta = constrain(c.substring(3).toInt(), 10, 2000); guardarConfig(); }
  else if (c.startsWith("CO ")) { cfg.comp = constrain(c.substring(3).toInt(), 0, 500); guardarConfig(); }
  else if (c.startsWith("TL ")) { cfg.tol  = constrain(c.substring(3).toInt(), 1, 200); guardarConfig(); }
  else if (c == "CAL0") {
    balanza.set_scale();      // factor 1
    balanza.tare(10);
    Serial.println(F("{\"msg\":\"Celda en cero. Ponga el peso conocido y envie CAL <gramos>\"}"));
  }
  else if (c.startsWith("CAL ")) {
    float conocido = c.substring(4).toFloat();
    if (conocido > 0) {
      float lectura = balanza.get_value(10);
      cfg.factor = lectura / conocido;
      balanza.set_scale(cfg.factor);
      guardarConfig();
      Serial.print(F("{\"msg\":\"Calibrado. Factor = "));
      Serial.print(cfg.factor, 3);
      Serial.println(F("\"}"));
    }
  }
}

void leerSerie() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (linea.length()) procesarComando(linea);
      linea = "";
    } else if (linea.length() < 32) {
      linea += ch;
    }
  }
}

void enviarEstado() {
  Serial.print(F("{\"w\":"));   Serial.print(peso, 1);
  Serial.print(F(",\"st\":"));  Serial.print(etapa);
  Serial.print(F(",\"run\":")); Serial.print(cicloActivo ? 1 : 0);
  Serial.print(F(",\"cut\":")); Serial.print(digitalRead(R_CORTE) == RELE_ON ? 1 : 0);
  Serial.print(F(",\"tol\":")); Serial.print(digitalRead(R_TOL) == RELE_ON ? 1 : 0);
  Serial.print(F(",\"sp\":"));  Serial.print(cfg.meta);
  Serial.print(F(",\"co\":"));  Serial.print(cfg.comp);
  Serial.print(F(",\"tl\":"));  Serial.print(cfg.tol);
  Serial.println(F("}"));
}

void setup() {
  Serial.begin(115200);
  for (uint8_t pin = R_PARTIR; pin <= R_TOL; pin++) {
    digitalWrite(pin, RELE_OFF);   // primero el nivel, despues el modo: el rele no parpadea al encender
    pinMode(pin, OUTPUT);
  }
  pinMode(IN_TARA, INPUT_PULLUP);
  pinMode(IN_B0, INPUT_PULLUP);
  pinMode(IN_B1, INPUT_PULLUP);
  pinMode(IN_B2, INPUT_PULLUP);
  pinMode(IN_B3, INPUT_PULLUP);
  pinMode(IN_RUN, INPUT_PULLUP);

  cargarConfig();
  balanza.begin(PIN_DT, PIN_SCK);
  balanza.set_scale(cfg.factor);
  tarar();
  Serial.println(F("{\"msg\":\"Pesaje listo\"}"));
}

void loop() {
  leerSerie();
  atenderPulsos();
  leerPeso();

  // Etapa del PLC: se acepta cuando se lee igual 3 veces seguidas
  uint8_t e = leerEtapa();
  if (e == etapaLeida) { if (repeticiones < 3) repeticiones++; }
  else { etapaLeida = e; repeticiones = 0; }
  if (repeticiones >= 3) etapa = etapaLeida;
  cicloActivo = activo(IN_RUN);

  // Tara: al activarse Y10 (flanco de subida)
  bool tara = activo(IN_TARA);
  if (tara && !taraPrevia) tarar();
  taraPrevia = tara;

  // Senal de corte: peso >= meta - compensacion
  bool corte = pesoValido && peso >= (cfg.meta - cfg.comp);
  digitalWrite(R_CORTE, corte ? RELE_ON : RELE_OFF);

  // Senal de fuera de tolerancia: |peso - meta| > tolerancia
  bool fuera = pesoValido && fabs(peso - cfg.meta) > cfg.tol;
  digitalWrite(R_TOL, fuera ? RELE_ON : RELE_OFF);

  // Registro del saco: peso al cortar y peso final estable
  if (etapa != etapaPrevia) {
    if (etapaPrevia == 5 && etapa == 6) pesoEnCorte = peso;
    if (etapaPrevia == 6 && (etapa == 7 || etapa == 15)) {
      pesoFinal = peso;
      Serial.print(F("{\"evt\":\"fin\",\"wcut\":")); Serial.print(pesoEnCorte, 1);
      Serial.print(F(",\"wfin\":")); Serial.print(pesoFinal, 1);
      Serial.println(F("}"));
    }
    etapaPrevia = etapa;
  }

  if (millis() - ultimoEnvio >= 100) {
    ultimoEnvio = millis();
    enviarEstado();
  }
}
