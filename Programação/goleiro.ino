#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <string.h>

// =========================
// BNO055
// =========================
Adafruit_BNO055 bno = Adafruit_BNO055(55);

bool  bnoCalibrado     = false;
float anguloRobo       = 0;
float offsetOrientacao = 0.0;  // Offset do zero virtual de orientação
bool  calibrar         = 0;    // Se 1, aguarda calibração completa no setup

void bno_calibrar() {
  Serial.println("Calibrando BNO055... aguarde.");
  uint8_t sys, gyro, accel, mag;
  while (true) {
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print("SYS:"); Serial.print(sys);
    Serial.print(" G:");  Serial.print(gyro);
    Serial.print(" A:");  Serial.print(accel);
    Serial.print(" M:");  Serial.println(mag);
    if (mag >= 2 && gyro >= 2) {
      bnoCalibrado = true;
      Serial.println(">>> BNO055 CALIBRADO - SISTEMA PRONTO <<<");
      break;
    }
    delay(200);
  }
}

// Zera a orientação: o yaw atual passa a ser considerado como 0 graus (frente)
void zerarOrientacao() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  offsetOrientacao = euler.x();
  Serial.print("Orientacao zerada. Novo offset: ");
  Serial.println(offsetOrientacao);
}

// Retorna yaw relativo ao zero virtual: -180 a +180 graus
// 0 = frente definida pelo último "zerar", positivo = direita, negativo = esquerda
float lerOrientacao() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float yaw = euler.x() - offsetOrientacao;
  while (yaw >  180.0) yaw -= 360.0;
  while (yaw < -180.0) yaw += 360.0;
  anguloRobo = yaw;
  return yaw;
}

// =======================================================================
// SISTEMA DE RECOVERY DE POSIÇÃO
// =======================================================================
// Distâncias de referência capturadas no startup (posição central ideal)
float distanciaReferenciaEsq = 0.0f;
float distanciaReferenciaDir = 0.0f;

// =======================================================================
// VARIÁVEIS DOS SENSORES IR FRONTAIS (BOLA)
// =======================================================================
const uint8_t NUM_IR = 5;
const uint8_t PIN_IR[5] = { 40, 42, 44, 46, 32 };
// [0] Extrema Esquerda, [1] Centro-Esq, [2] Centro, [3] Centro-Dir, [4] Extrema Dir
//
// ATENÇÃO: PIN_IR[2] = pino 44, mesmo pino que KICKER_PIN.
// Conflito de hardware pré-existente — não pode ser corrigido via software sem
// alterar o mapeamento físico. Preservado como estava no código original.
const int IR_DETECTADO = LOW;

// =========================
// PINOS MOTORES
// =========================
#define M1_IN1 38
#define M1_IN2 36
#define M1_PWM 8

#define M2_IN1 22
#define M2_IN2 24
#define M2_PWM 10

#define M3_IN1 45
#define M3_IN2 47
#define M3_PWM 9

#define M4_IN1 28
#define M4_IN2 26
#define M4_PWM 11

#define MOTOR1_DIR 1
#define MOTOR2_DIR 1
#define MOTOR3_DIR 1
#define MOTOR4_DIR 1

#define SWITCH_PIN  48   // Declarado, não utilizado no loop — preservado
#define KICKER_PIN  44   // ATENÇÃO: conflito com PIN_IR[2] (pino 44)

// =========================
// ULTRASSÔNICOS HC-SR04
// =========================
#define ECHO_ESQ  37
#define TRIG_ESQ  39
#define ECHO_TRAS 41
#define TRIG_TRAS 43
#define ECHO_DIR  33
#define TRIG_DIR  35

// =========================
// LDR / SENSORES DE LINHA
// =========================
// Pinos preservados e inicializados, mas NÃO utilizados como trava de movimento.
// Removidos da lógica de fronteira lateral conforme requisito do projeto.
#define luzEsq    52
#define luzFrente 53
#define luzDir    49
#define luzTras   30

#define botao 31


// PID de distância por ultrassônico
#define DIST_KP           8.0

// Tolerância em cm
#define DIST_TOLERANCIA   2.0

// Limites de velocidade
#define DIST_VEL_MIN      40
#define DIST_VEL_MAX      150

#define BALL_VEL_MAX        255
#define BALL_VEL_MIN         80

// direções do IR seeker (1..9), CENTRO=5
// 1,2,3 = esquerda | 4,5,6 = frente | 7,8,9 = direita
// Aproximação de "atrás": quando intensidade muito baixa OU não detecta (0)
#define ROT_KP             2.1
#define ROT_VELOCIDADE_MIN  56
#define ROT_VELOCIDADE_MAX 80
#define ROT_TOLERANCIA     4.0
// =========================
// TRACKBALL XY — movimentos discretos + PID de orientação para manter 0°
// "Ir atrás da bola": quando a bola está muito perto (intensidade alta) e alinhada,
// o robô recua/desacelera em vez de continuar avançando sobre ela
// =========================
#define TBXY_VEL_FRENTE     200
#define TBXY_VEL_LATERAL    220
#define TBXY_VEL_DIAGONAL   200
#define TBXY_VEL_RECUO      120
#define TBXY_INTENS_MUITO_PERTO  140   // acima disso, considera "na bola" → recua

// =======================================================================
// PARÂMETROS DE FRONTEIRA ULTRASSÔNICA (ajuste antes da competição)
// =======================================================================

// Distância mínima ao lado esquerdo (cm). Abaixo deste valor, bloqueia movimento à esquerda.
// Cálculo de referência: área de pênalti = 70 cm, robô = 20 cm → ~25 cm de folga.
// ASSUNÇÃO: valor padrão 25 cm. Medir fisicamente antes da competição.
#define LIMITE_ESQ_CM   25.0f

// Distância mínima ao lado direito (cm). Abaixo deste valor, bloqueia movimento à direita.
// Simétrico ao limite esquerdo.
// ASSUNÇÃO: valor padrão 25 cm. Medir fisicamente antes da competição.
#define LIMITE_DIR_CM   25.0f

// Distância mínima à parede traseira (cm). Abaixo deste valor, bloqueia movimento para trás.
// ASSUNÇÃO: valor padrão 20 cm. Distância traseira NÃO fornecida — MEDIR antes da competição.
#define LIMITE_TRAS_CM  20.0f

// Distância máxima válida para leitura ultrassônica (cm).
// Leituras acima deste valor são consideradas erro de reflexão ou fora de alcance.
#define ULTRASSONICO_MAX_CM  200.0f

// =======================================================================
// PARÂMETROS DE CONTROLE DE ORIENTAÇÃO (ajuste se necessário)
// =======================================================================

// Ganho proporcional para correção de yaw.
// Erro de 10° → correção de ~15 unidades PWM com Kp=1.5.
// ASSUNÇÃO: valor inicial 1.5. Ajustar em testes se o robô oscilar.
#define KP_ORIENTACAO   1.5f

// Zona morta de orientação (graus). Erros menores que este valor são ignorados.
// Evita micro-correções por ruído do IMU.
// ASSUNÇÃO: 2.0 graus.
#define ZONA_MORTA_GRAUS  2.0f

// Correção máxima de rotação aplicada ao moveRobot() (unidades PWM).
// Limita o quanto a correção de orientação pode interferir no movimento lateral.
// ASSUNÇÃO: 60 unidades PWM.
#define MAX_CORRECAO_ROT  60

// =========================
// VELOCIDADE DE RASTREIO
// =========================
#define VELOCIDADE_LATERAL 255

// Velocidade de retorno à posição de referência.
// Mais lento que VELOCIDADE_LATERAL para evitar overshooting.
#define VELOCIDADE_RECOVERY  80

// Tolerância de deslocamento lateral (cm).
// Erros menores que este valor não acionam o recovery.
// Histerese interna: para quando erro < TOLERANCIA_RECOVERY / 2.
#define TOLERANCIA_RECOVERY  3.0f

// =========================
// ULTRASSÔNICO - LEITURA
// =========================
float lerDistancia(int trigPin, int echoPin) {
  // Pulso de disparo
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Leitura com timeout de 30 ms (equivale a ~515 cm máximo de alcance)
  long duracao = pulseIn(echoPin, HIGH, 30000);
  // Timeout: pulseIn retorna 0 se não receber eco dentro do tempo limite
  if (duracao == 0) return -1.0f;
  return duracao * 0.0343f / 2.0f;
}

float lerEsq()  { return lerDistancia(TRIG_ESQ,  ECHO_ESQ);  }
float lerTras() { return lerDistancia(TRIG_TRAS, ECHO_TRAS); }
float lerDir()  { return lerDistancia(TRIG_DIR,  ECHO_DIR);  }

// Verifica se uma leitura ultrassônica é válida (não é timeout nem reflexão espúria)
bool leituraValida(float dist) {
  return (dist > 0.0f && dist < ULTRASSONICO_MAX_CM);
}

// =========================
// LDR - LEITURA
// =========================
// Funções preservadas para diagnóstico futuro. NÃO utilizadas em lógica de movimento.
int lerLuzEsq()    { return digitalRead(luzEsq);    }
int lerLuzFrente() { return digitalRead(luzFrente); }
int lerLuzDir()    { return digitalRead(luzDir);    }
int lerLuzTras()   { return digitalRead(luzTras);   }

void mostrarLDRs() {
  Serial.print("Esq: ");       Serial.print(lerLuzEsq());
  Serial.print(" | Frente: "); Serial.print(lerLuzFrente());
  Serial.print(" | Dir: ");    Serial.print(lerLuzDir());
  Serial.print(" | Tras: ");   Serial.println(lerLuzTras());
}

// =========================
// CONTROLE DE MOTORES
// =========================
void motorWrite(int in1, int in2, int pwmPin, int speedValue) {
  speedValue = constrain(speedValue, -255, 255);
  if (speedValue > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    analogWrite(pwmPin, speedValue);
  } else if (speedValue < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speedValue);
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void setMotor(int motor, int speedValue) {
  switch (motor) {
    case 1: motorWrite(M1_IN1, M1_IN2, M1_PWM, speedValue * MOTOR1_DIR); break;
    case 2: motorWrite(M2_IN1, M2_IN2, M2_PWM, speedValue * MOTOR2_DIR); break;
    case 3: motorWrite(M3_IN1, M3_IN2, M3_PWM, speedValue * MOTOR3_DIR); break;
    case 4: motorWrite(M4_IN1, M4_IN2, M4_PWM, speedValue * MOTOR4_DIR); break;
  }
}

void stopRobot() {
  setMotor(1, 0); setMotor(2, 0); setMotor(3, 0); setMotor(4, 0);
}

void front(int s) { setMotor(1,-s); setMotor(2, s); setMotor(3,-s); setMotor(4, s); }
void back(int s)  { setMotor(1, s); setMotor(2,-s); setMotor(3, s); setMotor(4,-s); }
void left(int s)  { setMotor(1, s); setMotor(2, s); setMotor(3,-s); setMotor(4,-s); }
void right(int s) { setMotor(1,-s); setMotor(2,-s); setMotor(3, s); setMotor(4, s); }

void rotate(int s, int dir) {
  setMotor(1,-s*dir); setMotor(2,-s*dir);
  setMotor(3,-s*dir); setMotor(4,-s*dir);
}
void diagonalLeft(int s, int dir) {
  setMotor(1,0); setMotor(2, s*dir); setMotor(3,-s*dir); setMotor(4,0);
}
void diagonalRight(int s, int dir) {
  setMotor(1,-s*dir); setMotor(2,0); setMotor(3,0); setMotor(4, s*dir);
}

// Movimento omnidirecional com normalização de velocidade
// x > 0 = direita, x < 0 = esquerda
// y > 0 = frente,  y < 0 = trás
// rot > 0 = horário, rot < 0 = anti-horário
void moveRobot(int x, int y, int rot) {
  int m1 = -y - x - rot;
  int m2 =  y - x - rot;
  int m3 = -y + x - rot;
  int m4 =  y + x - rot;
  int maxValue = max(max(abs(m1), abs(m2)), max(abs(m3), abs(m4)));
  if (maxValue > 255) {
    m1 = m1 * 255 / maxValue;
    m2 = m2 * 255 / maxValue;
    m3 = m3 * 255 / maxValue;
    m4 = m4 * 255 / maxValue;
  }
  setMotor(1, m1); setMotor(2, m2); setMotor(3, m3); setMotor(4, m4);
}

// =======================================================================
// CAPTURA A POSIÇÃO DE REFERÊNCIA (chama no setup, com robô centralizado)
// Tenta até 5 vezes por sensor para evitar falha por leitura espúria inicial.
// =======================================================================
void capturarPosicaoReferencia() {
  const int MAX_TENTATIVAS = 5;

  // Captura referência esquerda
  for (int i = 0; i < MAX_TENTATIVAS; i++) {
    float d = lerEsq();
    if (leituraValida(d)) {
      distanciaReferenciaEsq = d;
      break;
    }
    delay(30);
  }

  // Captura referência direita
  for (int i = 0; i < MAX_TENTATIVAS; i++) {
    float d = lerDir();
    if (leituraValida(d)) {
      distanciaReferenciaDir = d;
      break;
    }
    delay(30);
  }

  Serial.print("Referencia capturada — ESQ: ");
  Serial.print(distanciaReferenciaEsq);
  Serial.print(" cm | DIR: ");
  Serial.print(distanciaReferenciaDir);
  Serial.println(" cm");

  // Alerta se referência não foi capturada (sensor com problema no startup)
  if (distanciaReferenciaEsq == 0.0f) {
    Serial.println("AVISO: Referencia ESQ nao capturada — recovery desativado para esse lado.");
  }
  if (distanciaReferenciaDir == 0.0f) {
    Serial.println("AVISO: Referencia DIR nao capturada — recovery desativado para esse lado.");
  }
}

// =========================
// SETUP
// =========================
void setup() {
  // Inicializa pinos de direção dos motores como saída em LOW
  int inPins[] = {
    M1_IN1, M1_IN2,   // 38, 36
    M2_IN1, M2_IN2,   // 22, 24
    M3_IN1, M3_IN2,   // 45, 47
    M4_IN1, M4_IN2    // 28, 26
  };
  for (int i = 0; i < 8; i++) {
    pinMode(inPins[i], OUTPUT);
    digitalWrite(inPins[i], LOW);
  }

  // Força PWM = 0 explicitamente antes de qualquer comando
  int pwmPins[] = { M1_PWM, M2_PWM, M3_PWM, M4_PWM }; // 8, 10, 9, 11
  for (int i = 0; i < 4; i++) {
    pinMode(pwmPins[i], OUTPUT);
    digitalWrite(pwmPins[i], LOW);
    analogWrite(pwmPins[i], 0);
  }

  // Inicializa pinos do array IR como entrada
  for (uint8_t i = 0; i < NUM_IR; i++) {
    pinMode(PIN_IR[i], INPUT);
  }

  // Inicializa pinos de linha/LDR como entrada (mantidos para diagnóstico)
  pinMode(luzEsq,    INPUT);
  pinMode(luzDir,    INPUT);
  pinMode(luzFrente, INPUT);
  pinMode(luzTras,   INPUT);

  Serial.begin(115200);
  Wire.begin();

  stopRobot();

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  // Pino 44 configurado como saída para o kicker.
  // ATENÇÃO: este pino também é PIN_IR[2]. Conflito de hardware pré-existente.
  pinMode(KICKER_PIN, OUTPUT);

  // Inicializa pinos dos sensores ultrassônicos
  pinMode(TRIG_ESQ,  OUTPUT); pinMode(ECHO_ESQ,  INPUT);
  pinMode(TRIG_TRAS, OUTPUT); pinMode(ECHO_TRAS, INPUT);
  pinMode(TRIG_DIR,  OUTPUT); pinMode(ECHO_DIR,  INPUT);

  // Botão de zerar orientação
  pinMode(botao, INPUT_PULLUP);

  // Inicializa BNO055
  if (!bno.begin()) {
    Serial.println("ERRO: BNO055 nao encontrado! Verifique a ligacao.");
    while (1);
  }
  delay(1000);
  bno.setMode(OPERATION_MODE_NDOF);

  // Calibração manual opcional (calibrar = 0 por padrão, modo NDOF auto-calibra)
  if (calibrar) { bno_calibrar(); }

  // Define a orientação atual como frente (zero de referência)
  zerarOrientacao();

  // Captura posição de referência (robô deve estar centralizado ao ligar)
  capturarPosicaoReferencia();
  Serial.println("Sistema iniciado. Goleiro pronto.");
}

// Calcula erro angular mínimo considerando wrap-around de -180 a +180


// =========================
// DETECÇÃO DE BORDA DO BOTÃO
// =========================
bool botaoAnterior = HIGH;  // Pull-up → repouso = HIGH

void verificarBotaoZero() {
  bool botaoAtual = digitalRead(botao);
  // Borda de descida (HIGH → LOW) = botão recém pressionado → zera orientação
  if (botaoAnterior == HIGH && botaoAtual == LOW) {
    zerarOrientacao();
  }
  botaoAnterior = botaoAtual;
}

// =======================================================================
// CALCULA CORREÇÃO PROPORCIONAL DE ORIENTAÇÃO
// Retorna valor de rotação a aplicar em moveRobot().
// Positivo = corrige para direita (sentido horário)
// Negativo = corrige para esquerda (sentido anti-horário)
// =======================================================================
int calcularCorrecaoOrientacao() {
  float yawAtual = lerOrientacao();  // Lê yaw relativo ao zero definido

  // Alvo sempre 0 graus (frente definida no setup ou pelo botão)
  float erro = calcularErro(yawAtual, 0.0f);

  // Zona morta: ignora ruído pequeno do IMU
  if (abs(erro) < ZONA_MORTA_GRAUS) {
    return 0;
  }

  // Correção proporcional limitada
  int correcao = (int)(KP_ORIENTACAO * erro);
  correcao = constrain(correcao, -MAX_CORRECAO_ROT, MAX_CORRECAO_ROT);
  return correcao;
}

void leftPID(int velocidade) {
  int correcaoRot = calcularCorrecaoOrientacao();
  moveRobot(-velocidade, 0, correcaoRot);
}

void rightPID(int velocidade) {
  int correcaoRot = calcularCorrecaoOrientacao();
  moveRobot(velocidade, 0, correcaoRot);
}

// =======================================================================
// FUNÇÃO DE RASTREIO LATERAL COM FRONTEIRAS ULTRASSÔNICAS
//
// Substitui as travas baseadas em sensores de linha por limites baseados
// nos sensores ultrassônicos laterais. Integra correção de orientação do BNO055.
//
// Comportamento:
//   Bola à esquerda  → move à esquerda, a menos que fronteira esquerda ativa
//   Bola à direita   → move à direita, a menos que fronteira direita ativa
//   Bola no centro   → para (aplica só correção de orientação)
//   Sem bola         → para (aplica só correção de orientação)
//   Fronteira traseira → bloqueia movimento, aplica apenas correção de orientação
// =======================================================================
void rastrearLateral() {
  // --- Leitura dos 5 sensores de bola ---
  bool irEsqExtrema = (digitalRead(PIN_IR[0]) == IR_DETECTADO);
  bool irEsqCentro  = (digitalRead(PIN_IR[1]) == IR_DETECTADO);
  bool irCentro     = (digitalRead(PIN_IR[2]) == IR_DETECTADO);
  bool irDirCentro  = (digitalRead(PIN_IR[3]) == IR_DETECTADO);
  bool irDirExtrema = (digitalRead(PIN_IR[4]) == IR_DETECTADO);

  // --- Correção de orientação (BNO055) ---
  int correcaoRot = calcularCorrecaoOrientacao();

  // --- Leitura traseira (sempre verificada para evitar recuo involuntário) ---
  float distTras = lerTras();
  bool fronteiraTrasAtiva = leituraValida(distTras) && (distTras < LIMITE_TRAS_CM);

 

  // --- Lógica de rastreio lateral com fronteiras ultrassônicas ---
  if (irEsqExtrema || irEsqCentro) {
    // Bola à esquerda → verificar fronteira esquerda antes de mover
    right(180);
    Serial.println("direita");
    return;
  } else if (irDirExtrema || irDirCentro) {
    // Bola à direita → verificar fronteira direita antes de mover
    left(130);
    Serial.println("esq");
  } else {
    // Nenhum sensor IR detecta a bola → tenta retornar à posição de referência
    // (correcaoRot já é aplicado internamente pelo recoveryPosition)
    Serial.println("nada");
    stopRobot();
  }
}

// =======================================================================
// RECOVERY DE POSIÇÃO LATERAL
//
// Chamada apenas quando nenhum sensor IR detecta a bola.
// Compara distâncias atuais com referência e move o robô de volta ao centro.
//
// Histerese: aciona com erro > TOLERANCIA_RECOVERY,
//            para com erro < TOLERANCIA_RECOVERY / 2.
//            Evita "hunting" (ligar/desligar na borda da tolerância).
//
// Respeita fronteiras ultrassônicas existentes.
// Aplica correção de orientação BNO055 simultaneamente.
// =======================================================================

// Estado interno de histerese (preservado entre chamadas)
bool emRecovery = false;

void recoveryPosition() {
  // Se referências não foram capturadas, não há como fazer recovery
  if (distanciaReferenciaEsq == 0.0f && distanciaReferenciaDir == 0.0f) {
    moveRobot(0, 0, calcularCorrecaoOrientacao());
    return;
  }

  // Leitura atual dos sensores laterais
  float distEsq = lerEsq();
  float distDir = lerDir();

  bool esqValida = leituraValida(distEsq) && (distanciaReferenciaEsq > 0.0f);
  bool dirValida = leituraValida(distDir) && (distanciaReferenciaDir > 0.0f);

  // Se ambos os sensores falharam: para com segurança, tenta no próximo ciclo
  if (!esqValida && !dirValida) {
    moveRobot(0, 0, calcularCorrecaoOrientacao());
    return;
  }

  // Calcula erro de deslocamento:
  // Erro positivo → robô está à direita da referência (esq > refEsq)  → mover à esquerda
  // Erro negativo → robô está à esquerda da referência (esq < refEsq) → mover à direita
  float erro = 0.0f;
  int   fontes = 0;

  if (esqValida) {
    erro += (distEsq - distanciaReferenciaEsq);
    fontes++;
  }
  if (dirValida) {
    // Dir aumenta quando robô vai para esquerda → sinal inverso ao do esq
    erro -= (distDir - distanciaReferenciaDir);
    fontes++;
  }

  // Média dos erros disponíveis
  erro /= (float)fontes;

  // --- Histerese ---
  float limiteEntrada = TOLERANCIA_RECOVERY;
  float limiteSaida   = TOLERANCIA_RECOVERY / 2.0f;

  if (!emRecovery && abs(erro) > limiteEntrada) {
    emRecovery = true;   // Ativa recovery
  }
  if (emRecovery && abs(erro) < limiteSaida) {
    emRecovery = false;  // Desativa recovery — posição restaurada
  }

  // Correção de orientação aplicada em todos os casos
  int correcaoRot = calcularCorrecaoOrientacao();

  if (!emRecovery) {
    // Dentro da tolerância: apenas mantém orientação
    moveRobot(0, 0, correcaoRot);
    return;
  }

  // --- Fora da tolerância: move em direção à referência ---
  if (erro > 0.0f) {
    // Robô deslocado para a direita → mover à esquerda
    // Verifica fronteira esquerda antes de mover
    bool fronteiraEsqAtiva = esqValida && (distEsq < LIMITE_ESQ_CM);
    if (fronteiraEsqAtiva) {
      Serial.println("RECOVERY: Fronteira ESQ ativa, abortando movimento.");
      moveRobot(0, 0, correcaoRot);
    } else {
      Serial.print("RECOVERY: Corrigindo para ESQ, erro=");
      Serial.print(erro); Serial.println(" cm");
      moveRobot(-VELOCIDADE_RECOVERY, 0, correcaoRot);
    }
  } else {
    // Robô deslocado para a esquerda → mover à direita
    // Verifica fronteira direita antes de mover
    bool fronteiraDirAtiva = dirValida && (distDir < LIMITE_DIR_CM);
    if (fronteiraDirAtiva) {
      Serial.println("RECOVERY: Fronteira DIR ativa, abortando movimento.");
      moveRobot(0, 0, correcaoRot);
    } else {
      Serial.print("RECOVERY: Corrigindo para DIR, erro=");
      Serial.print(erro); Serial.println(" cm");
      moveRobot(VELOCIDADE_RECOVERY, 0, correcaoRot);
    }
  }
}
float calcularErro(float anguloAtual, float anguloAlvo) {
  float erro = anguloAlvo - anguloAtual;
  while (erro >  180) erro -= 360;
  while (erro < -180) erro += 360;
  return erro;
}

bool rotacionarParaAngulo(float anguloAlvo) {
  float atual = lerOrientacao();   // já usa o offset automaticamente
  float erro  = calcularErro(atual, anguloAlvo);

  Serial.print("Yaw: "); Serial.print(atual);
  Serial.print(" | Erro: "); Serial.println(erro);

  if (abs(erro) <= ROT_TOLERANCIA) {
    stopRobot();
    return true;
  }

  int velocidade = (int)(abs(erro) * ROT_KP);
  velocidade = constrain(velocidade, ROT_VELOCIDADE_MIN, ROT_VELOCIDADE_MAX);
  int direcao = (erro > 0) ? 1 : -1;
  rotate(velocidade, direcao);
  return false;
}

// =========================
// LOOP PRINCIPAL
// =========================
bool moverAteDistanciaEsq(float distanciaAlvo) {
  float atual = lerEsq();

  // Sensor sem leitura válida → não move, evita decisão errada
  if (atual < 0) {
    stopRobot();
    return false;
  }

  float erro = atual - distanciaAlvo;  // positivo = está mais longe que o alvo → precisa se aproximar (ir para a esquerda)

  Serial.print("[ESQ] Dist: "); Serial.print(atual);
  Serial.print(" | Alvo: "); Serial.print(distanciaAlvo);
  Serial.print(" | Erro: "); Serial.println(erro);

  if (abs(erro) <= DIST_TOLERANCIA) {
    stopRobot();
    if (abs(calcularErro(lerOrientacao(), 0)) > 10.0) {
  rotacionarParaAngulo(0);
  }
    return true;
  }

  int velocidade = (int)(abs(erro) * DIST_KP);
  velocidade = constrain(velocidade, DIST_VEL_MIN, DIST_VEL_MAX);

  // erro > 0 → muito longe do obstáculo esquerdo → mover para a esquerda (aproximar)
  // erro < 0 → muito perto do obstáculo esquerdo → mover para a direita (afastar)
  if (erro > 0) left(velocidade);
  else           right(velocidade);

  return false;
}

// ── Move usando o ultrassônico DIREITO como referência ──
bool moverAteDistanciaDir(float distanciaAlvo) {
  float atual = lerDir();

  if (atual < 0) {
    stopRobot();
    return false;
  }

  float erro = atual - distanciaAlvo;

  Serial.print("[DIR] Dist: "); Serial.print(atual);
  Serial.print(" | Alvo: "); Serial.print(distanciaAlvo);
  Serial.print(" | Erro: "); Serial.println(erro);

  if (abs(erro) <= DIST_TOLERANCIA) {
    stopRobot();
    if (abs(calcularErro(lerOrientacao(), 0)) > 10.0) {
  rotacionarParaAngulo(0);
  }
    return true;
  }

  int velocidade = (int)(abs(erro) * DIST_KP);
  velocidade = constrain(velocidade, DIST_VEL_MIN, DIST_VEL_MAX);

  // erro > 0 → muito longe do obstáculo direito → mover para a direita (aproximar)
  // erro < 0 → muito perto do obstáculo direito → mover para a esquerda (afastar)
  if (erro > 0) right(velocidade);
  else           left(velocidade);

  return false;
}

// ── Move usando o ultrassônico TRASEIRO como referência ──
bool moverAteDistanciaTras(float distanciaAlvo) {
  float atual = lerTras();

  if (atual < 0) {
    stopRobot();
    return false;
  }

  float erro = atual - distanciaAlvo;

  Serial.print("[TRAS] Dist: "); Serial.print(atual);
  Serial.print(" | Alvo: "); Serial.print(distanciaAlvo);
  Serial.print(" | Erro: "); Serial.println(erro);

  if (abs(erro) <= DIST_TOLERANCIA) {
    stopRobot();
    if (abs(calcularErro(lerOrientacao(), 0)) > 10.0) {
  rotacionarParaAngulo(0);
  }
    return true;
  }

  int velocidade = (int)(abs(erro) * DIST_KP);
  velocidade = constrain(velocidade, DIST_VEL_MIN, DIST_VEL_MAX);

  // erro > 0 → muito longe do obstáculo traseiro → mover para trás (aproximar)
  // erro < 0 → muito perto do obstáculo traseiro → mover para frente (afastar)
  if (erro > 0) back(velocidade);
  else           front(velocidade);

  return false;
}
void loop() {
  // Verifica se o botão foi pressionado para zerar orientação de referência
  //verificarBotaoZero();

  // Rastreio lateral da bola com fronteiras ultrassônicas e correção de orientação
 //if (abs(calcularErro(lerOrientacao(), 0)) > 10.0) {
  //rotacionarParaAngulo(0);
 // }
  // Mantém 30cm do sensor traseiro (não bloqueante)
  
  rastrearLateral();
 // if (abs(calcularErro(lerOrientacao(), 0)) > 10.0) {
  //rotacionarParaAngulo(0);
  //}
  // Lógica do kicker: botão D53 LOW → dispara kicker
  // ATENÇÃO: KICKER_PIN (44) compartilha pino com PIN_IR[2]. Conflito pré-existente.
 // Serial.println(digitalRead(botao));
  if (digitalRead(botao)) {
    digitalWrite(KICKER_PIN, LOW);
  } else {
    digitalWrite(KICKER_PIN, HIGH);
    Serial.println(lerLuzEsq()); // Diagnóstico de LDR esquerdo (não interfere em movimento)
  }

  delay(20);
}