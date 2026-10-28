#include <DHT.h>
#include <PinChangeInterrupt.h>
#include <LCDWIKI_GUI.h>
#include <LCDWIKI_SPI.h>

// ======================================================================
// ÁREA DE AJUSTES RÁPIDOS
// ======================================================================
#define RAMPA_PESO 5

#define PIN_R_EN 8
#define PIN_L_EN 6

// --- CONFIGURAÇÃO DA TELA ---
#define TFT_MODEL ST7735S
#define TFT_CS    A5
#define TFT_CD    A3
#define TFT_RST   A4
#define TFT_LED   A0

LCDWIKI_SPI my_lcd(TFT_MODEL, TFT_CS, TFT_CD, TFT_RST, TFT_LED);

// --- PINOS DO SISTEMA ---
#define PIN_TACO_C1     5
#define PIN_PWM_COOLERS 3
#define PIN_TACO_C2     4
#define PIN_TACO_MOTOR  2
#define PIN_PWM_MOTOR   9
#define PIN_DHT         7
#define PIN_VIBRACAO    12
#define PIN_POT_MOTOR   A2
#define PIN_POT_COOLERS A1

DHT dht(PIN_DHT, DHT11);

unsigned long tAnterior = 0;
unsigned long tAnteriorDHT = 0;
unsigned long tAnteriorRampa = 0;

volatile unsigned long pC1 = 0, pC2 = 0;

float uVal = 0, tVal = 0;   // só usado 1x a cada 3s, float aqui não pesa
uint8_t pwmAtualMotor = 0;  // 0-255, uint8_t é suficiente

int ultimoPotMBrutoValido = 0;

// --- Filtro EMA em ponto fixo (Q8), substitui o float suavizado ---
// alpha = 1/8 (>>3). Para deixar mais suave, use >>4; mais rápido, >>2.
static int32_t potAcc = 0;

// --- Tacômetro do motor: sensor Hall A3144, 2 ímãs, 2 pulsos/ímã ---
// = 4 pulsos por volta. Baseado em medição de intervalo entre pulsos
// com média móvel, validado separadamente antes de entrar no código principal.
const byte PULSOS_POR_VOLTA_MOTOR = 4;
const byte NUM_AMOSTRAS_MOTOR = 8;
const unsigned long TEMPO_MINIMO_PULSO_MOTOR = 1200UL;  // µs, filtro de ruído/bounce
const unsigned long TIMEOUT_PARADO_MOTOR = 500000UL;    // 500 ms sem pulso = motor parado

volatile unsigned long ultimoPulsoMotor = 0;
volatile unsigned long intervalosMotor[NUM_AMOSTRAS_MOTOR];
volatile byte indiceMotor = 0;
volatile bool bufferCheioMotor = false;

// --- Sensor de vibração 801S: classificação por ciclo de trabalho (% HIGH) ---
// Amostragem por polling (sem interrupção), janela própria de 2s,
// independente do ciclo de 1s do display/serial.
const unsigned long TEMPO_AMOSTRAGEM_VIBRA = 2000; // ms
const float LIMITE_OTIMO_VIBRA   = 5.0;  // % HIGH abaixo disto -> ÓTIMO
const float LIMITE_CRITICO_VIBRA = 90.0; // % HIGH acima disto -> CRÍTICO (entre os dois -> OK)

unsigned long contadorLOWVibra = 0;
unsigned long contadorHIGHVibra = 0;
unsigned long inicioVibra = 0;

enum EstadoVibracao { VIBRA_OTIMO, VIBRA_OK, VIBRA_CRITICO };
EstadoVibracao estadoVibracao = VIBRA_OTIMO;

void isrC1() { pC1++; }
void isrC2() { pC2++; }

void isrM() {
  unsigned long agora = micros();
  if (ultimoPulsoMotor == 0) {
    ultimoPulsoMotor = agora;
    return;
  }
  unsigned long dt = agora - ultimoPulsoMotor;
  if (dt < TEMPO_MINIMO_PULSO_MOTOR) return;

  intervalosMotor[indiceMotor] = dt;
  indiceMotor++;
  if (indiceMotor >= NUM_AMOSTRAS_MOTOR) {
    indiceMotor = 0;
    bufferCheioMotor = true;
  }
  ultimoPulsoMotor = agora;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_R_EN, OUTPUT);
  pinMode(PIN_L_EN, OUTPUT);
  digitalWrite(PIN_R_EN, LOW);
  digitalWrite(PIN_L_EN, LOW);

  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  delay(1000);

  my_lcd.Init_LCD();
  my_lcd.Fill_Screen(0x0000);
  my_lcd.Set_Rotation(3);

  my_lcd.Set_Text_Mode(1);
  my_lcd.Set_Text_Size(1);
  my_lcd.Set_Text_colour(0xFFFF);
  my_lcd.Print_String(F("PAINEL DE ROTOFIACAO"), CENTER, 5);
  my_lcd.Draw_Fast_HLine(0, 15, 160);

  my_lcd.Set_Text_colour(0x07FF);
  my_lcd.Print_String(F("[ MOTOR PRINCIPAL ]"), 5, 22);
  my_lcd.Set_Text_colour(0x07E0);
  my_lcd.Print_String(F("Forca:"), 5, 32);

  my_lcd.Set_Text_colour(0x07FF);
  my_lcd.Print_String(F("[ FLUXO DE AR ]"), 5, 52);
  my_lcd.Set_Text_colour(0xFDA0);
  my_lcd.Print_String(F("Vel. Media:"), 10, 62);

  my_lcd.Set_Text_colour(0x07FF);
  my_lcd.Print_String(F("[ AMBIENTE ]"), 5, 82);
  my_lcd.Set_Text_colour(0xFFE0);
  my_lcd.Print_String(F("Temp:"), 5, 90);
  my_lcd.Print_String(F("Umidade:"), 65, 90);

  my_lcd.Set_Text_colour(0x07FF);
  my_lcd.Print_String(F("[ STATUS ]"), 5, 105);

  dht.begin();
  pinMode(PIN_PWM_COOLERS, OUTPUT);
  pinMode(PIN_PWM_MOTOR, OUTPUT);

  // Sobe o PWM do Timer1 (pinos 9 e 10) para ~31kHz, fora do audível
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;

  pinMode(PIN_VIBRACAO, INPUT);
  pinMode(PIN_TACO_C1, INPUT_PULLUP);
  pinMode(PIN_TACO_C2, INPUT_PULLUP);
  pinMode(PIN_TACO_MOTOR, INPUT_PULLUP);

  attachPCINT(digitalPinToPCINT(PIN_TACO_C1), isrC1, FALLING);
  attachPCINT(digitalPinToPCINT(PIN_TACO_C2), isrC2, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_TACO_MOTOR), isrM, FALLING);

  ultimoPotMBrutoValido = analogRead(PIN_POT_MOTOR);
  potAcc = (int32_t)ultimoPotMBrutoValido << 8; // inicializa o filtro Q8

  inicioVibra = millis();

  delay(500);
  digitalWrite(PIN_R_EN, HIGH);
  digitalWrite(PIN_L_EN, HIGH);
}

void loop() {
  unsigned long agora = millis();

  // =======================================================
  // 1. TRAVA ABSOLUTA DE RUÍDO DO POTENCIÔMETRO
  // =======================================================
  int potM_bruto_atual = analogRead(PIN_POT_MOTOR);
  int potC = analogRead(PIN_POT_COOLERS);

  // map(potC, 0, 1023, 0, 255) equivale a dividir por 4 -> shift
  analogWrite(PIN_PWM_COOLERS, potC >> 2);

  if (abs(potM_bruto_atual - ultimoPotMBrutoValido) > 8) {
      ultimoPotMBrutoValido = potM_bruto_atual;
  }

  // EMA em ponto fixo (Q8), sem nenhuma multiplicação/divisão de float
  potAcc += (((int32_t)ultimoPotMBrutoValido << 8) - potAcc) >> 3;
  int potMotorSuavizado = potAcc >> 8; // valor 0-1023

  // map(potMotorSuavizado, 0, 1023, 0, 255) ~= >>2 (aprox., igual ao original)
  int pwmAlvo = potMotorSuavizado >> 2;
  if (pwmAlvo > 255) pwmAlvo = 255; // guarda contra arredondamento no limite

  if (agora - tAnteriorRampa >= RAMPA_PESO) {
    tAnteriorRampa = agora;

    if (abs(pwmAlvo - (int)pwmAtualMotor) > 1) {
      if (pwmAtualMotor < pwmAlvo) pwmAtualMotor++;
      else if (pwmAtualMotor > pwmAlvo) pwmAtualMotor--;
      analogWrite(PIN_PWM_MOTOR, pwmAtualMotor);
    }
  }
  // =======================================================

  // =======================================================
  // SENSOR DE VIBRAÇÃO (801S) - amostragem por ciclo de trabalho
  // =======================================================
  if (digitalRead(PIN_VIBRACAO) == LOW) contadorLOWVibra++;
  else contadorHIGHVibra++;

  if (agora - inicioVibra >= TEMPO_AMOSTRAGEM_VIBRA) {
    unsigned long totalVibra = contadorLOWVibra + contadorHIGHVibra;
    if (totalVibra > 0) {
      float percHIGH = (100.0f * contadorHIGHVibra) / totalVibra;
      if (percHIGH < LIMITE_OTIMO_VIBRA) estadoVibracao = VIBRA_OTIMO;
      else if (percHIGH <= LIMITE_CRITICO_VIBRA) estadoVibracao = VIBRA_OK;
      else estadoVibracao = VIBRA_CRITICO;
    }
    contadorLOWVibra = 0;
    contadorHIGHVibra = 0;
    inicioVibra = agora;
  }
  // =======================================================

  if (agora - tAnteriorDHT >= 3000) {
    tAnteriorDHT = agora;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) { uVal = h; tVal = t; }
  }

  if (agora - tAnterior >= 1000) {
    tAnterior = agora;

    unsigned long copiaIntervalosMotor[NUM_AMOSTRAS_MOTOR];
    unsigned long ultimoM;
    byte nM;

    noInterrupts();
    unsigned long cC1 = pC1;
    unsigned long cC2 = pC2;
    pC1 = 0;
    pC2 = 0;
    ultimoM = ultimoPulsoMotor;
    nM = bufferCheioMotor ? NUM_AMOSTRAS_MOTOR : indiceMotor;
    for (byte i = 0; i < nM; i++) copiaIntervalosMotor[i] = intervalosMotor[i];
    interrupts();

    int rpmCoolerMedio = ((cC1 * 60 / 2) + (cC2 * 60 / 2)) / 2;

    // RPM do motor via média móvel dos intervalos entre pulsos
    long rpmMotor = 0;
    if (nM > 0 && ultimoM != 0 && (micros() - ultimoM) <= TIMEOUT_PARADO_MOTOR) {
      unsigned long soma = 0;
      for (byte i = 0; i < nM; i++) soma += copiaIntervalosMotor[i];
      float periodoMedio = (float)soma / nM;
      float rpmF = 60000000.0f / (periodoMedio * PULSOS_POR_VOLTA_MOTOR);
      rpmMotor = (long)(rpmF + 0.5f);
    }

    int pM_perc = map(pwmAtualMotor, 0, 255, 0, 100); // roda 1x/s, custo irrelevante

    Serial.print(F("Forca: "));
    Serial.print(pM_perc);
    Serial.print(F("% | RPM: "));
    Serial.println(rpmMotor);

    my_lcd.Set_Text_Mode(0);
    my_lcd.Set_Text_Back_colour(0, 0, 0);
    char bufText[20];

    my_lcd.Set_Text_colour(0x07E0);
    sprintf(bufText, " %3d%%  ", pM_perc);
    my_lcd.Print_String(bufText, 40, 32);

    sprintf(bufText, " %5ld RPM  ", rpmMotor);
    my_lcd.Print_String(bufText, 85, 32);

    my_lcd.Set_Text_colour(0xFDA0);
    sprintf(bufText, " %5d RPM  ", rpmCoolerMedio);
    my_lcd.Print_String(bufText, 75, 62);

    my_lcd.Set_Text_colour(0xFFE0);
    sprintf(bufText, " %2dC  ", (int)tVal);
    my_lcd.Print_String(bufText, 35, 90);

    sprintf(bufText, " %2d%%  ", (int)uVal);
    my_lcd.Print_String(bufText, 115, 90);

    switch (estadoVibracao) {
      case VIBRA_OTIMO:
        my_lcd.Set_Text_colour(0x07E0); // verde
        my_lcd.Print_String(F(" SISTEMA: OTIMO          "), 5, 115);
        break;
      case VIBRA_OK:
        my_lcd.Set_Text_colour(0xFFE0); // amarelo
        my_lcd.Print_String(F(" SISTEMA: OK             "), 5, 115);
        break;
      case VIBRA_CRITICO:
        my_lcd.Set_Text_colour(0xF800); // vermelho
        my_lcd.Print_String(F(" ALERTA: VIBRACAO CRIT.  "), 5, 115);
        break;
    }
  }
}
