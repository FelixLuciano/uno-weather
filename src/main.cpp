#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_BMP280 bmp;
DHT dht(8, DHT22);

const char ccedil[8] PROGMEM = {0x00, 0x00, 0x0F, 0x10, 0x10, 0x10, 0x0F, 0x02}; // Caractere customizado 1 (ç)
const char atilde[8] PROGMEM = {0x0E, 0x00, 0x0E, 0x01, 0x0F, 0x11, 0x0F, 0x00}; // Caractere customizado 2 (ã)
const char oacute[8] PROGMEM = {0x02, 0x04, 0x0E, 0x11, 0x11, 0x11, 0x0E, 0x00}; // Caractere customizado 3 (ó)
const char degree[8] PROGMEM = {0x06, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00, 0x00}; // Caractere customizado 4 (°)

const uint16_t screenTime = 2000;

void intro ();

void setup()
{
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);

    analogReference(INTERNAL);

    Serial.begin(9600);

    lcd.init();
    lcd.setBacklight(HIGH);

    lcd.createChar(1, ccedil);
    lcd.createChar(2, atilde);
    lcd.createChar(3, oacute);
    lcd.createChar(4, degree);

    bmp.begin(0x76);
    dht.begin();

    intro();
}

void intro () {
    lcd.clear();
    lcd.print("Engenaria Turma");
    lcd.setCursor(0, 1);
    lcd.print("Insper    1B");
    Serial.println("Engenharia Insper");
    Serial.println("Turma 1B");
    delay(screenTime);

    lcd.clear();
    lcd.print("Esta");
    lcd.write(1);
    lcd.write(2);
    lcd.print("o");
    lcd.setCursor(0, 1);
    lcd.print("Meteorol");
    lcd.write(3);
    lcd.print("gica");
    Serial.println("Estação Meteorológica");
    delay(screenTime);

    lcd.clear();
    lcd.print("Henrry");
    lcd.setCursor(0, 1);
    lcd.print("Souza Miguel");
    Serial.println("Henrry Souza Miguel");
    delay(screenTime);

    lcd.clear();
    lcd.print("Luciano Felix");
    lcd.setCursor(0, 1);
    lcd.print("Dias");
    Serial.println("Luciano Felix Dias");
    delay(screenTime);

    lcd.clear();
    lcd.print("Pedro Cliquet");
    lcd.setCursor(0, 1);
    lcd.print("do Amaral");
    Serial.println("Pedro Cliquet do Amaral");
    delay(screenTime);
}

float getTemperature () {
    int B_LM33;
    float U_LM33, T_LM33;

    B_LM33 = analogRead(A1);            // Leitura digital do valor analógico do sensor
    U_LM33 = B_LM33 * 1.1 / (1024 - 1); // Conversão da leitura para a tensão no sensor (em V)
    T_LM33 = U_LM33 * 101.007 - 0.0001; // Conversão da tensão no sensor para temperatura (em ºC)

    return T_LM33;
}

void printTemperature (float temperature = getTemperature()) {
    lcd.clear();
    lcd.print("Temperatura");
    lcd.setCursor(0, 1);
    lcd.print(temperature);
    lcd.write(4);
    lcd.print("C");

    Serial.print("Temperatura: ");
    Serial.println(temperature);
}

float getHumidity () {
    float Ur_DHT22, Ur_calib;

    Ur_DHT22 = dht.readHumidity();     // Leitura da umidade relativa no sensor (em %)
    Ur_calib = Ur_DHT22 * 0.939 - 0.4; // Leitura calibrada da umidade relativa no sensor (em %)

    return Ur_calib;
}

void printHumidity (float humidity = getHumidity()) {
    lcd.clear();
    lcd.print("Umidade");
    lcd.setCursor(0, 1);
    lcd.print(humidity);
    lcd.print("%");

    Serial.print("Umidade: ");
    Serial.println(humidity);
}

float getPressure () {
    float P_BMP280 = bmp.readPressure() / 100; // Leitura da pressão no sensor (em hPA)

    return P_BMP280;
}

void printPressure (float pressure = getPressure()) {
    lcd.clear();
    lcd.print("Press");
    lcd.write(2);
    lcd.print("o");
    lcd.setCursor(0, 1);
    lcd.print(pressure);
    lcd.print(" hPa");

    Serial.print("Pressão: ");
    Serial.println(pressure);
}

float getLuminosity () {
    int B_LDR;
    float U_LDR, R_LDR, L_LDR;

    B_LDR = analogRead(A0);                     // Leitura digital do valor analógico do sensor
    U_LDR = B_LDR * 1.1 / (1024 - 1);           // Conversão da leitura para a tensão no sensor (em V)
    R_LDR = 100000 * U_LDR / (5 - U_LDR);       // Conversão da tensão no sensor para a resistência do sensor (em Ohms)
    L_LDR = pow(10, 6.5 - 1.25 * log10(R_LDR)); // COnversão da resistência do sensor para a luminosidade no sensor (em Lux)

    return L_LDR;
}

void printLuminosity (float luminosity = getLuminosity()) {
    lcd.clear();
    lcd.print("Luminosidade");
    lcd.setCursor(0, 1);
    lcd.print(luminosity);
    lcd.print(" lx");

    Serial.print("Luminosidade: ");
    Serial.println(luminosity);
}

void loop() {
    printTemperature();
    delay(screenTime);
    
    printHumidity();
    delay(screenTime);
    
    printPressure();
    delay(screenTime);
    
    printLuminosity();
    delay(screenTime);
}
