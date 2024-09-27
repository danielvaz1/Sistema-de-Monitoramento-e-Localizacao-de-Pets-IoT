#include <DHT.h>
#include <Wire.h>

// Definições para o DHT22
#define dhtpin 7      // Pino onde o DHT22 está conectado
#define dhttype DHT22 // Tipo de sensor DHT
DHT dht(dhtpin, dhttype); // Instancia o sensor 

// Definições para o MPU6050
const int MPU6050_ADDR = 0x68;  // Endereço I2C do MPU6050
int16_t ax, ay, az;  // Variáveis para aceleração
int16_t gx, gy, gz;  // Variáveis para giroscópio

// Variáveis de localização  
float latitude = -3.6867;  // Inicializa a latitude (Itapajé)
float longitude = -39.5850;  // Inicializa a longitude (Itapajé)

// Variáveis simulando o estado de conexão
bool connectedToServer = true;  // Simula conexão com servidor

void setup() {
  Serial.begin(9600);
  
  // Inicializa o DHT22
  dht.begin();

  // Inicializa o MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // Registrador de energia
  Wire.write(0);     // Acorda o MPU6050
  Wire.endTransmission(true);
}

void loop() {
  // Lê a umidade e a temperatura do DHT22
  float humidity = dht.readHumidity(); 
  float temperature = dht.readTemperature(); 

  // Lê os dados do MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // Endereço do primeiro registrador de aceleração
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);  // Solicita os 14 bytes de dados

  // Combina os bytes de dados em variáveis de 16 bits
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  // Cálculo da mudança em latitude e longitude baseada na aceleração e giroscópio
  float latChange = (ax * 0.00001) + (gy * 0.00001); // Fator de escala para latitude
  float longChange = (ay * 0.00001) - (gx * 0.00001); // Fator de escala para longitude

  // Atualiza a posição com a mudança baseada na aceleração e giroscópio
  latitude += latChange;
  longitude += longChange;
  
  // Simula envio dos dados para um servidor
  if (connectedToServer) {
    Serial.println("Conectado ao servidor... Enviando dados:");
    Serial.print("Latitude: "); Serial.println(latitude);
    Serial.print("Longitude: "); Serial.println(longitude);
    Serial.print("Umidade: "); Serial.println(humidity);
    Serial.print("Temperatura: "); Serial.println(temperature);
    Serial.print("Aceleracao X: "); Serial.println(ax);
    Serial.print("Aceleracao Y: "); Serial.println(ay);
    Serial.print("Aceleracao Z: "); Serial.println(az);
    Serial.print("Giroscopio X: "); Serial.println(gx);
    Serial.print("Giroscopio Y: "); Serial.println(gy);
    Serial.print("Giroscopio Z: "); Serial.println(gz);

  } else {
    Serial.println("Erro de conexão com o servidor!");
  }

  delay(3000); // Aguarda 3 segundos antes da próxima leitura
}