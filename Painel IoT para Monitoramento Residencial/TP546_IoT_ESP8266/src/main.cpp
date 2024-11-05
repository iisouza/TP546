
// Código desenvolvido para o projeto de dispositivos IoT da disciplina TP546 - IoT e Redes Veiculares, do curso de 
// Mestrado do Instituto Nacional de Telecomunicações | INATEL.
// Igor Gonçalves de Souza 931 | Samuel Baraldi Mafra

// Simulação de dispositivo para controle residencial, com monitoramento de Temperatura e nível de Gás, além de ativação
// de lâmpadas e janelas.
// Materiais utilizados: ESP8266, LEDs 5mm, Módulo Buzzer, ServoMotor SG90, sensores de Gás MQ2 e de Temperatura DHT11. 
// Resistores de diversos valores.

// ****************************************************************************************************************** //
//                                 Pinagem para os sensores e componentes do projeto.                                 //
//                                     Pinos virtuais do Blynk para o dashboard.                                      //
// ****************************************************************************************************************** //
#define ESP__MQ2_PIN      A0  // Sensor de Gás MQ2 (analógico, min = 0, max = 1024).
#define ESP__buzzerPIN    D4  // Buzzer (PWM com DC = 50%).
#define ESP__servoPIN     D5  // Servo Motor (analógico, 180°).
#define ESP__lampEsquerda D6  // LED externo 1 (digital, 5V).
#define ESP__lampDireita  D7  // LED externo 2 (digital, 5V).
#define ESP__DHT11PIN     D8  // Sensor de Temperatura DHT11 (serial).

#define Blynk__Temperatura        V0  // Pino virtual para Temperatura (Rótulo, double, min 0, max 50).
#define Blynk__lampEsquerda       V1  // Pino virtual para lamp. esquerda (interruptor toggle).
#define Blynk__lampDireita        V2  // Pino virtual para lamp. direita (interruptor toggle).
#define Blynk__interruptGeral     V3  // Pino virtual para iluminação geral (interruptor toggle).
#define Blynk__lamp_alertaGas     V4  // Pino virtual para alerta de gás (verde, amarelo ou vermelho).
#define Blynk__botaoJanela        V5  // Pino virtual para botão janela (interruptor de estado).
#define Blynk__sensorGas          V6  // Pino virtual para sensor inativo (imagem indicadora de falha).
#define Blynk__nivelGas           V7  // Pino virtual para nível de gás (rótulo, inteiro, min 0, max 1024).
#define Blynk__sensorTemperatura  V8  // Pino virtual para sensor inativo (imagem indicadora de falha).
#define Blynk__buzzerOFF          V9  // Pino virtual para controle do Buzzer (interruptor de estado).
#define Blynk__currentTime        V10 // Pino virtual para hora atual (String).
#define Blynk__currentDate        V11 // Pino virtual para data atual (String).

// ****************************************************************************************************************** //
//                                       Autenticação do Dispositivo no Blynk.                                        //
//                                         Bibliotecas utilizadas no projeto.                                         //
// ****************************************************************************************************************** //
#define BLYNK_PRINT Serial
#define BLYNK_AUTH_TOKEN    "********************************"
#define BLYNK_TEMPLATE_ID   "*************"
#define BLYNK_TEMPLATE_NAME "TP546 IoT ESP8266"

#include <Servo.h>              // Movimento Servo Motor.        
#include <DHTesp.h>             // Leitura Sensor de Temperatura. 
#include <TimeLib.h>            // Leitura des informações de Data e Hora.
#include <WiFiUdp.h>            // Conexão de Rede para Tempo Real.
#include <NTPClient.h>          // Sincronismo de Data e Hora.
#include <BlynkSimpleEsp8266.h> // Comunicação Dispositivo Blynk.

// ****************************************************************************************************************** //
//                                  Credenciais de conexão às redes WiFi utilizadas.                                  //
//                      Botões de Controle dos periféricos e Tempo de Publicação das Mensagens.                       //
// ****************************************************************************************************************** //
struct WiFiCredentials {
  struct redeInatel{      // Rede Inatel (testes de laboratório com possíveis instabilidades).
    const char* ssid      = "WLL-Inatel";
    const char* password  = "************";
  }; redeInatel inatel;
  
  struct redeIgor {       // Rede Celular (testes de laboratório com ponto de acesso próximo).
    const char* ssid      = "Igor's Galaxy A54 5G";
    const char* password  = "********";
  };  redeIgor local;

  struct redeDomestica {  // Rede Doméstica (testes em casa).
    const char* ssid      = "Igor";
    const char* password  = "********";
  }; redeDomestica domestica;
};  WiFiCredentials credenciais;

struct botoesComando {
  bool buttonL1     = false;  // Botão lamp. esquerda.
  bool buttonL2     = false;  // Botão lamp. direita.   
  bool buzzerState  = true;   // Controle alarme buzzer.
}; botoesComando comando;

struct timePublish{
  unsigned long temperature = 60*1000;  // 60 segundos.
  unsigned long realTime    = 1*1000;   // 1 segundo.
  unsigned long gas         = 5*1000;   // 5 segundos.            
}; timePublish publishMessage;

// ****************************************************************************************************************** //
//                                Paramêtros e limiares para leitura do sensor de gás.                                //
//                             Cores para alerta de vazamento de gás ou leitura inválida.                             //
// ****************************************************************************************************************** //
struct gasAlert {
  int gasValue      = 0;    // Valor do sensor de gás.
  int limiarMinimum = 100;  // Limiar para ambiente limpo.
  int gasThreshold  = 300;  // Limiar para ambiente seguro.
  int limiarMaximum = 500;  // Limiar para ambiente perigoso.
}; gasAlert alerta;

struct colorsGas {
  const char* blue    = "#87CEEB";  // Sinal para falha no sensor.
  const char* green   = "#00FF00";  // Sinal ambiente seguro.
  const char* yellow  = "#FFFF00";  // Sinal ambiente alerta.
  const char* red     = "#FF0000";  // Sinal ambiente perigoso. 
}; colorsGas colors;

struct positionWindow {
  int fechada       = 0;    // Posição do servo motor (graus) para janela fechada.
  int aberta        = 180;  // Posição do servo motor (graus) para janela aberta.
  int delayed       = 500;  // Atraso entre movimentos consecutivos do motor.
  int timeMoviment  = 5;    // Intervalo de movimentação da Janela nos dois sentidos. 
}; positionWindow posicaoJanela;

Servo servoMotor; BlynkTimer blynkTimer; DHTesp sensorDHT11;
WiFiUDP ntpUDP; NTPClient timeClient(ntpUDP, "pool.ntp.org", -3*60*60, 60000);

void buzzerAlert(int delayTime) { 
  // Função para controlar o acionamento do Buzzer como sinal sonoro para vazamento de gás.
  // Acionamento com DC = 50%, com variação de período, sendo:
  //  - 500ms para nível de gás em alerta, 300 < nivelGas < 500 [ppm];
  //  - 250ms para nível de gás crítico, acima de 500 [ppm].
  analogWrite(ESP__buzzerPIN, 128); delay(delayTime);
  analogWrite(ESP__buzzerPIN, 0); delay(delayTime);
}

void movimentWindow(int degrees, int interval) {
  // Função para controlar o movimento da Janela, podendo ser abertura ou fechamento.
  // Movimento dado em degress (graus) dentro de um interval segundos.
  // Serial.println("Movendo a janela...");
  int posicaoAtual = servoMotor.read(); // Serial.println("Posição atual: " + String(posicaoAtual));  
  int degAdjusted = posicaoJanela.aberta/interval; // Serial.println("Deg Adjusted: " + String(degAdjusted)); 

  if (posicaoAtual != degrees) {  // Janela já está na posição desejada.
    if (posicaoAtual < degrees) { // Movimento de abertura.
      Serial.println("Abrindo a janela...");
      for (int pos = posicaoAtual; pos <= degrees; pos += degAdjusted) {
        // Serial.println("Posição: " + String(pos));
        servoMotor.write(pos); delay(posicaoJanela.delayed);                     
      } servoMotor.write(posicaoJanela.aberta);
    } else {                      // Movimento de fechamento.
      Serial.println("Fechando a janela...");
      for (int pos = posicaoAtual; pos >= degrees; pos -= degAdjusted) {
        // Serial.println("Posição: " + String(pos));
        servoMotor.write(pos); delay(posicaoJanela.delayed);                        
      } servoMotor.write(posicaoJanela.fechada);
    }
  }
}

void publishTemperature(double temperature) {
  // Função para publicar dados de Temperatura (Rótulo, V0) no Blynk.
  // Chamada periódica a cada 60 segundos (variável lenta).
  //  - retorno do sensor será 'nan' se a leitura for inválida.
  //  - valores válidos entre 0 e 50 [°C] (0 a 1024 analogInput)
  if (!isnan(temperature) && (temperature >= 0 && temperature <= 50)) {
    Serial.println("Temperatura atual: " + String(temperature));
    Blynk.virtualWrite(Blynk__Temperatura, temperature);
    Blynk.virtualWrite(Blynk__sensorTemperatura, 1);  // Apaga o aviso de falha no sensor.
  } else {
    Serial.println("Sensor DHT11 desconectado.");
    Blynk.virtualWrite(Blynk__Temperatura, 0);        // Valor padrão para leitura inválida no sensor.
    Blynk.virtualWrite(Blynk__sensorTemperatura, 0);  // Mostra o aviso de falha no sensor.
  }
}

void publishGasState(int gasValue) {
  // Função para publicar dados de nível de Gás (Rótulo, V6) no Blynk.
  // Chamada periódica a cada 5 segundos (variável lenta, porém segurança necessária).
  // Intervalos por limiares de sepração com abertura de janela.
  if (gasValue < alerta.limiarMinimum) {    // Indicativo de falha na leitira do sensor.
    Blynk.setProperty(Blynk__lamp_alertaGas, "color", colors.blue); 
    Blynk.setProperty(Blynk__buzzerOFF, "enabled", true); 
    Blynk.virtualWrite(Blynk__sensorGas, 0); 
    Blynk.virtualWrite(Blynk__nivelGas, 0);
  } else {                                  // Indicativo de leitura válida do sensor.
    Serial.println("\nValor do sensor de gás: " + String(gasValue));
    Blynk.setProperty(Blynk__lamp_alertaGas, "color", colors.green); 
    Blynk.setProperty(Blynk__buzzerOFF, "enabled", true); 
    Blynk.virtualWrite(Blynk__nivelGas, gasValue); 
    Blynk.virtualWrite(Blynk__sensorGas, 1); 
    
    if (gasValue > alerta.gasThreshold) {   // Indicativo de vazamento de gás.
      publishMessage.gas = 3*1000;
      Blynk.setProperty(Blynk__buzzerOFF, "enabled", false); 
      if (servoMotor.read() == posicaoJanela.fechada) {         
        movimentWindow(posicaoJanela.aberta, posicaoJanela.timeMoviment); 
      } Blynk.setProperty(Blynk__lamp_alertaGas, "color", gasValue < alerta.limiarMaximum ? colors.yellow:colors.red);
    } else {
      comando.buzzerState = true;
    }
  }
}

void publishCurrentTime() {
  // Função para publicar a hora atual.
  // Chamada periódica a cada segundo.
  int currentDay = day(timeClient.getEpochTime()); 
  int currentMonth = month(timeClient.getEpochTime()); 
  int currentYear = year(timeClient.getEpochTime());

  Blynk.virtualWrite(Blynk__currentTime, timeClient.getFormattedTime());
  Blynk.virtualWrite(Blynk__currentDate, (currentDay < 10 ? "0":"") + String(currentDay) + "/" + 
    (currentMonth < 10 ? "0":"") + String(currentMonth) + "/" + String(currentYear).substring(2));
}

BLYNK_WRITE(Blynk__lampEsquerda) {  
  // Função de interpretação do estado do interruptor da lâmpada esquerda.
  // Interruptor funciona como um toggle, invertendo o estado da lâmpada.
  Serial.println("\nAlterando lâmpada esquerda...");
  comando.buttonL1 = !comando.buttonL1; digitalWrite(ESP__lampEsquerda, comando.buttonL1);
}

BLYNK_WRITE(Blynk__lampDireita) {
  // Função de interpretação do estado do interruptor da lâmpada direita.
  // Interruptor funciona como um toggle, invertendo o estado da lâmpada.
  Serial.println("\nAlterando lâmpada direita...");
  comando.buttonL2 = !comando.buttonL2;
  digitalWrite(ESP__lampDireita, comando.buttonL2);
}

BLYNK_WRITE(Blynk__interruptGeral) {
  // Função de interpretação do estado do interruptor das lâmpadas.
  // Interruptor funciona como um toggle, invertendo o estado das lâmpadas.
  Serial.println("\nAlterando estado das lâmpadas...");
  comando.buttonL1 = !comando.buttonL1; digitalWrite(ESP__lampEsquerda, comando.buttonL1);
  comando.buttonL2 = !comando.buttonL2; digitalWrite(ESP__lampDireita, comando.buttonL2);
}

BLYNK_WRITE(Blynk__botaoJanela) {
  // Função para interpretação do estado do interruptor da Janela.
  // A modificação do estado do interruptor não tem efeito na situação de vazamento de gás.
  Serial.println("\nAlterando posição da janela...");
  if (alerta.gasValue < alerta.gasThreshold){
    movimentWindow(param.asInt() == 0 ? posicaoJanela.fechada:posicaoJanela.aberta, posicaoJanela.timeMoviment);
  }
}

BLYNK_WRITE(Blynk__buzzerOFF) {
  // Função para interpretação do estado do interruptor do Buzzer.
  // O interruptor pode ser utilizado para desativar o alarme de vazamento de gás.
  if (alerta.gasValue > alerta.gasThreshold){
    Serial.println("\nDesativando alarme de vazamento...");
    Blynk.setProperty(Blynk__buzzerOFF, "enabled", true); 
    comando.buzzerState = false;
    analogWrite(ESP__buzzerPIN, 0);
  }
}

void setup() {
  // Inicialização e configurações do sistema. Definições dos pinos de saída e estado inicial.
  // Tempo de publicação das variáveis e parametrização dos dispositivos e sensores.
  Serial.begin(115200); 
  Serial.println(""); Serial.println("");

  servoMotor.attach(ESP__servoPIN); servoMotor.write(0);
  pinMode(ESP__buzzerPIN, OUTPUT); analogWrite(ESP__buzzerPIN, 0);
  pinMode(ESP__lampDireita, OUTPUT); digitalWrite(ESP__lampEsquerda, 0); 
  pinMode(ESP__lampEsquerda, OUTPUT); digitalWrite(ESP__lampDireita, 0);

  blynkTimer.setInterval(publishMessage.realTime, publishCurrentTime);
  blynkTimer.setInterval(publishMessage.gas, []() {publishGasState(alerta.gasValue);}); 
  blynkTimer.setInterval(publishMessage.temperature, []() {publishTemperature(sensorDHT11.getTemperature());});

  Blynk.begin(BLYNK_AUTH_TOKEN, credenciais.domestica.ssid, credenciais.domestica.password);
  sensorDHT11.setup(ESP__DHT11PIN, DHTesp::DHT11); 
  timeClient.begin();
  delay(500); 
}

void loop() {
  // Comandos para serem executados durante a operação do Sistema.
  Blynk.run(); blynkTimer.run(); timeClient.update();
  alerta.gasValue = analogRead(ESP__MQ2_PIN);

  if (alerta.gasValue < alerta.gasThreshold) {
    analogWrite(ESP__buzzerPIN, 0);
  } else if (comando.buzzerState){
    buzzerAlert(alerta.gasValue > alerta.limiarMaximum ? 250:500);
  } delay(100);
}