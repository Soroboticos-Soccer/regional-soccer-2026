#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <HTInfraredSeeker.h>
#include <string.h>
// =========================
// BNO055
// =========================
Adafruit_BNO055 bno = Adafruit_BNO055(55);

bool  bnoCalibrado    = false;
float anguloRobo      = 0;
float offsetOrientacao = 0.0;   // ← offset do zero virtual
bool  calibrar        = 1;

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

// Zera a orientação: o yaw actual passa a ser 0
void zerarOrientacao() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  offsetOrientacao = euler.x();
  Serial.print("Orientacao zerada. Novo offset: ");
  Serial.println(offsetOrientacao);
}

// Retorna yaw relativo ao zero virtual: -180 a +180
// 0 = frente definida pelo ultimo "zerar", positivo = direita, negativo = esquerda
float lerOrientacao() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float yaw = euler.x() - offsetOrientacao;
  while (yaw >  180.0) yaw -= 360.0;
  while (yaw < -180.0) yaw += 360.0;
  anguloRobo = yaw;
  return yaw;
}

// =========================
// VARIAVEIS IR
// =========================
int ballDirecao;
int ballIntens;
// variável global — coloque junto das outras globais no topo
int ballIntensAnterior = 0;
// =========================
// PINOS MOTORES
// =========================
#define M1_IN1 36
#define M1_IN2 38
#define M1_PWM 8

#define M2_IN1 22
#define M2_IN2 24
#define M2_PWM 3

#define M3_IN1 45
#define M3_IN2 47
#define M3_PWM 12

#define M4_IN1 28
#define M4_IN2 26
#define M4_PWM 4

#define MOTOR1_DIR 1
#define MOTOR2_DIR 1
#define MOTOR3_DIR 1
#define MOTOR4_DIR 1

#define DRIBBLER_PIN 34
#define SWITCH_PIN   48
#define KICKER_PIN   44

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
// LDR
// =========================
#define luzEsq    53
#define luzFrente 52
#define luzDir    30
#define luzTras   49

#define botao 31

// =========================
// ULTRASSÔNICO - LEITURA
// =========================
float lerDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duracao = pulseIn(echoPin, HIGH, 30000);
  if (duracao == 0) return -1;
  return duracao * 0.0343 / 2.0;
}

float lerEsq()  { return lerDistancia(TRIG_ESQ,  ECHO_ESQ);  }
float lerTras() { return lerDistancia(TRIG_TRAS, ECHO_TRAS); }
float lerDir()  { return lerDistancia(TRIG_DIR,  ECHO_DIR);  }

// =========================
// LDR - LEITURA
// =========================
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
// SETUP
// =========================

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
    case 2: motorWrite(M2_IN1, M2_IN2, M2_PWM, speedValue * MOTOR2_DIR ); break;
    case 3: motorWrite(M3_IN1, M3_IN2, M3_PWM, speedValue * MOTOR3_DIR); break;
    case 4: motorWrite(M4_IN1, M4_IN2, M4_PWM, speedValue * MOTOR4_DIR); break;
  }
}

void stopRobot() { setMotor(1,0); setMotor(2,0); setMotor(3,0); setMotor(4,0); }

void front(int s) { setMotor(1,-s); setMotor(2, s); setMotor(3,-s); setMotor(4, s *1.4); }
void back(int s)  { setMotor(1, s); setMotor(2,-s); setMotor(3, s); setMotor(4,-s); }
void left(int s)  { setMotor(1, s); setMotor(2, s); setMotor(3,-s); setMotor(4,-s *1.4); }
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
void moveRobot(int x, int y, int rot) {
  int m1 = -y - x - rot;
  int m2 =  y - x - rot;
  int m3 = -y + x - rot;
  int m4 =  y + x - rot;

  int maxValue = max(max(abs(m1), abs(m2)), max(abs(m3), abs(m4)));
  if (maxValue > 255) {
    float escala = 255.0 / maxValue;  // ← float, não divisão inteira
    m1 = (int)(m1 * escala);
    m2 = (int)(m2 * escala);
    m3 = (int)(m3 * escala);
    m4 = (int)(m4 * escala);
  }

  setMotor(1, m1);
  setMotor(2, m2);
  setMotor(3, m3);
  setMotor(4, m4);
}
void setup() {

  int inPins[] = {
    M1_IN1, M1_IN2,   // 38, 36
    M2_IN1, M2_IN2,   // 22, 24
    M3_IN1, M3_IN2,   // 45, 47
    M4_IN1, M4_IN2    // 26, 28
  };
  for (int i = 0; i < 8; i++) {
    pinMode(inPins[i], OUTPUT);
    digitalWrite(inPins[i], LOW);
  }

  // ── Força PWM = 0 explicitamente ──
  int pwmPins[] = { M1_PWM, M2_PWM, M3_PWM, M4_PWM }; // 8, 10, 9, 11
  for (int i = 0; i < 4; i++) {
    pinMode(pwmPins[i], OUTPUT);
    digitalWrite(pwmPins[i], LOW); // antes do analogWrite, garante LOW
    analogWrite(pwmPins[i], 0);
  }

  Serial.begin(115200);
  Wire.begin();


  stopRobot();
  pinMode(DRIBBLER_PIN, OUTPUT);
  pinMode(SWITCH_PIN,   INPUT_PULLUP);
  pinMode(KICKER_PIN, OUTPUT);
  pinMode(TRIG_ESQ,  OUTPUT); pinMode(ECHO_ESQ,  INPUT);
  pinMode(TRIG_TRAS, OUTPUT); pinMode(ECHO_TRAS, INPUT);
  pinMode(TRIG_DIR,  OUTPUT); pinMode(ECHO_DIR,  INPUT);
  pinMode(botao, INPUT_PULLUP);

  if (!bno.begin()) {
    Serial.println("ERRO: BNO055 nao encontrado! Verifique a ligacao.");
    while (1);
  }
  delay(1000);
  bno.setMode(OPERATION_MODE_NDOF);
  digitalWrite(DRIBBLER_PIN,HIGH);
  digitalWrite(KICKER_PIN, HIGH);
  if (calibrar) { bno_calibrar(); }

  // Zera a orientação logo após calibrar (define a posição inicial como frente)
  zerarOrientacao();

  Serial.println("Sistema iniciado");
}

// =========================
// IR SEEKER
// =========================
// =========================
// SEGUIR BOLA (IR Seeker)
// =========================
// =========================
// SEGUIR BOLA (IR Seeker) — avanço contínuo com correção de orientação
// =========================
#define BALL_KP_ROT             45   // ganho proporcional da correção angular
#define BALL_VEL_FRENTE_MAX    255    // velocidade quando a bola está longe
#define BALL_VEL_FRENTE_MIN     145   // velocidade mínima quando está bem perto
#define BALL_VEL_ROT_MAX        185    // limite da correção de rotação
#define BALL_CENTRO              5
#define BALL_INTENS_MAX        150    // intensidade máxima do sensor
#define BALL_INTENS_PERTO       135    // intensidade a partir da qual já reduz bastante

void trackBall() {
  InfraredResult InfraredBall = InfraredSeeker::ReadAC();
  ballDirecao = InfraredBall.Direction;
  ballIntens  = InfraredBall.Strength;

  if (ballDirecao == 0) {
    stopRobot();
    return;
  }

  int erro = ballDirecao - BALL_CENTRO;

  // ── Correção de rotação proporcional ao erro ──
  int velRot = erro * BALL_KP_ROT;
  velRot = constrain(velRot, -BALL_VEL_ROT_MAX, BALL_VEL_ROT_MAX);

  // ── Velocidade de avanço: diminui conforme a bola se aproxima ──
  // Mapeia ballIntens (0..BALL_INTENS_MAX) → velFrente (MAX..MIN)
  int intensClamped = constrain(ballIntens, 0, BALL_INTENS_PERTO);
  int velFrente = map(intensClamped, 0, BALL_INTENS_PERTO, BALL_VEL_FRENTE_MAX, BALL_VEL_FRENTE_MIN);

  // ── Quando muito desalinhado, reduz ainda mais o avanço para não "fugir" da bola ──
  if (abs(erro) >= 2) {
    velFrente = velFrente * 0.7;
  }
  // ── Avança e corrige orientação simultaneamente ──
  moveRobot(0, velFrente, velRot);

  Serial.print("Dir:"); Serial.print(ballDirecao);
  Serial.print(" Erro:"); Serial.print(erro);
  Serial.print(" Intens:"); Serial.print(ballIntens);
  Serial.print(" VelF:"); Serial.print(velFrente);
  Serial.print(" VelR:"); Serial.println(velRot);
}


// =========================
// EVITAR LINHA — não-bloqueante
// =========================
// =========================
// EVITAR LINHA — não-bloqueante com filtro anti-ruído
// =========================
#define LINHA_VEL           255
#define LINHA_TEMPO         90   // ms de recuo
#define LINHA_CONFIRMACOES   3   // leituras consecutivas para confirmar linha

enum EstadoRobo { SEGUIR_BOLA, EVITAR_LINHA };
EstadoRobo estadoAtual = SEGUIR_BOLA;
unsigned long tempoInicioLinha = 0;

enum MovimentoLinha { MOV_BACK, MOV_FRONT, MOV_LEFT, MOV_RIGHT,
                      MOV_DIAG_LD, MOV_DIAG_LE, MOV_DIAG_TD, MOV_DIAG_TE };
MovimentoLinha movimentoLinha;

// Contadores de confirmação por sensor
int contEsq = 0, contFrente = 0, contDir = 0, contTras = 0;

bool gerenciarLinha() {
  bool rawEsq    = (lerLuzEsq()    == 0);
  bool rawFrente = (lerLuzFrente() == 0);
  bool rawDir    = (lerLuzDir()    == 0);
  bool rawTras   = (lerLuzTras()   == 0);

  // ── Atualiza contadores de confirmação ──
  contEsq    = rawEsq    ? contEsq    + 1 : 0;
  contFrente = rawFrente ? contFrente + 1 : 0;
  contDir    = rawDir    ? contDir    + 1 : 0;
  contTras   = rawTras   ? contTras   + 1 : 0;

  // Só considera "linha detectada" após N leituras consecutivas
  bool esq    = (contEsq    >= LINHA_CONFIRMACOES);
  bool frente = (contFrente >= LINHA_CONFIRMACOES);
  bool dir    = (contDir    >= LINHA_CONFIRMACOES);
  bool tras   = (contTras   >= LINHA_CONFIRMACOES);
  bool algumConfirmado = esq || frente || dir || tras;

  if (estadoAtual == SEGUIR_BOLA) {
    if (!algumConfirmado) return false;

    // Linha confirmada → define recuo e entra no estado
    tempoInicioLinha = millis();
    estadoAtual = EVITAR_LINHA;

    // Zera contadores para não reativar imediatamente
    contEsq = contFrente = contDir = contTras = 0;

    if      (frente && esq)  movimentoLinha = MOV_DIAG_LD;
    else if (frente && dir)  movimentoLinha = MOV_DIAG_LE;
    else if (tras   && esq)  movimentoLinha = MOV_DIAG_TD;
    else if (tras   && dir)  movimentoLinha = MOV_DIAG_TE;
    else if (frente)         movimentoLinha = MOV_BACK;
    else if (tras)           movimentoLinha = MOV_FRONT;
    else if (esq)            movimentoLinha = MOV_RIGHT;
    else                     movimentoLinha = MOV_LEFT;

    Serial.print("[LINHA CONFIRMADA] esq:"); Serial.print(esq);
    Serial.print(" frente:"); Serial.print(frente);
    Serial.print(" dir:"); Serial.print(dir);
    Serial.print(" tras:"); Serial.println(tras);
  }

  if (estadoAtual == EVITAR_LINHA) {
    switch (movimentoLinha) {
      case MOV_BACK:     back(LINHA_VEL);                 break;
      case MOV_FRONT:    front(LINHA_VEL);                break;
      case MOV_LEFT:     left(LINHA_VEL);                 break;
      case MOV_RIGHT:    right(LINHA_VEL);                break;
      case MOV_DIAG_LD:  diagonalRight(LINHA_VEL,  1);   break;
      case MOV_DIAG_LE:  diagonalLeft(LINHA_VEL,   1);   break;
      case MOV_DIAG_TD:  diagonalRight(LINHA_VEL, -1);   break;
      case MOV_DIAG_TE:  diagonalLeft(LINHA_VEL,  -1);   break;
    }

    // Sai só quando o tempo expirou — ignora se a linha sumiu antes
    if (millis() - tempoInicioLinha > LINHA_TEMPO) {
      estadoAtual = SEGUIR_BOLA;
    }
    return true;
  }
  return false;
}
// =========================
// SEGUIR BOLA XY — sem rotação proporcional ao IR
// Robô sempre mantém-se olhando para o ângulo 0 (definido pelo botão)
// Direção (x,y) é definida pela posição da bola
// =========================
#define BALL_VEL_MAX        255
#define BALL_VEL_MIN         80

// direções do IR seeker (1..9), CENTRO=5
// 1,2,3 = esquerda | 4,5,6 = frente | 7,8,9 = direita
// Aproximação de "atrás": quando intensidade muito baixa OU não detecta (0)
#define ROT_KP             2.0
#define ROT_VELOCIDADE_MIN  56
#define ROT_VELOCIDADE_MAX 80
#define ROT_TOLERANCIA     4.0
// =========================
// TRACKBALL XY — movimentos discretos + PID de orientação para manter 0°
// "Ir atrás da bola": quando a bola está muito perto (intensidade alta) e alinhada,
// o robô recua/desacelera em vez de continuar avançando sobre ela
// =========================
// ROTAÇÃO PARA ÂNGULO ALVO
// =========================

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
// DETECÇÃO DE BORDA DO BOTÃO
// =========================
bool botaoAnterior = HIGH;   // pull-up → repouso = HIGH
bool setado = false;
void verificarBotaoZero() {
  bool botaoAtual = digitalRead(botao);
  // Borda HIGH → LOW = botão acabou de ser pressionado
  if (botaoAnterior == HIGH && botaoAtual == LOW && setado == false) {
    //executarChute(1, 50);
    setado = true;
    zerarOrientacao();
  }
  botaoAnterior = botaoAtual;
}

// =========================
// LOOP
// =========================
// =========================
// MOVER ATÉ DISTÂNCIA (ULTRASSÔNICO)
// =========================
#define DIST_KP            4.0
#define DIST_TOLERANCIA    1.5   // cm — considera "chegou" dentro dessa margem
#define DIST_VEL_MIN       70
#define DIST_VEL_MAX       180

// ── Move usando o ultrassônico ESQUERDO como referência ──
// distanciaAlvo: distância desejada em cm entre o robô e o obstáculo à esquerda
// Retorna true quando atingir a distância (dentro da tolerância)
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

// Calcula a correção de rotação (velRot) para manter anguloAlvo


// ── FRENTE com PID ──
// forca: velocidade de avanço (0-255)
// anguloAlvo: ângulo a manter (graus, referência do zero do botão)

// ── TRÁS com PID ──


// ── ESQUERDA com PID ──

// ── DIREITA com PID ──

// ── DIAGONAL com PID ──
// direcao: "fe"=frente-esquerda, "fd"=frente-direita, "te"=trás-esquerda, "td"=trás-direita

void mostrarLinhas() {
  Serial.print("Esq:");    Serial.print(lerLuzEsq());
  Serial.print(" Frente:"); Serial.print(lerLuzFrente());
  Serial.print(" Dir:");    Serial.print(lerLuzDir());
  Serial.print(" Tras:");   Serial.println(lerLuzTras());
}
// =========================
// FUGIR DA LINHA — movimento oposto ao sensor ativado
// =========================
#define FUGA_VEL  200

void fugirDaLinha() {
  bool esq    = (lerLuzEsq()    == 0);
  bool frente = (lerLuzFrente() == 0);
  bool dir    = (lerLuzDir()    == 0);
  bool tras   = (lerLuzTras()   == 0);

  if (!esq && !frente && !dir && !tras) return;

  if      (esq)  diagonalLeft(FUGA_VEL, -1);   // foge para trás-direita
  else if (dir)  diagonalRight(FUGA_VEL, -1);  // foge para trás-esquerda
  else if (tras) front(FUGA_VEL);
}
// =========================
// SEGUIR BOLA — movimentos discretos + PID de orientação (mantém 0°)
// =========================
int TBOLA_VEL = 130;   // velocidade base de todos os movimentos
#define TBOLA_INTENS_PERTO 155   // dir 5: recua se passar disso

void trackBallOrientado() {
  InfraredResult InfraredBall = InfraredSeeker::ReadAC();
  ballDirecao = InfraredBall.Direction;
  ballIntens  = InfraredBall.Strength;
  bool temLinha = (lerLuzEsq() == 0 || lerLuzFrente() == 0 ||
                   lerLuzDir() == 0 || lerLuzTras()   == 0);
  //if (temLinha) {
   // fugirDaLinha();
  //  return;  // não executa trackball neste ciclo
 // }

  if(ballIntens > 120) {TBOLA_VEL = 100;}
  else {TBOLA_VEL = 130;}
  switch (ballDirecao) {
    case 0:
      stopRobot();
      //back(80);
      return;

    case 5:
      if (ballIntens >= TBOLA_INTENS_PERTO){
        float distTras = lerTras();
    if (distTras > 0 && distTras > 160.0) {
      back(200);
      delay(10);
      return;
    }
     chutar();
    // stopRobot();
      return;
      }else{
        front(TBOLA_VEL);
      }
      break;
      
    case 4:
      if (ballIntens > 120)
        diagonalRight(TBOLA_VEL, -1);   // trás-esquerda
      else
        left(TBOLA_VEL);
      break;

    case 3:
    if (ballIntens < 70)
      left(TBOLA_VEL - 10);
    else
      diagonalRight(TBOLA_VEL - 10, -1);     // trás-esquerda
      break;

    case 2:
      if (ballIntens > 50)
        back(100);
      else
        back(100);
      break;

    case 1:
      if (ballIntens > 30)
        back(100);
      else
        back(100);
      break;

    case 6:
      if (ballIntens < 160)
        right(TBOLA_VEL);
      else
        diagonalLeft(TBOLA_VEL, -1);  // trás-direita
      break;

    case 7:
      if (ballIntensAnterior > 0 && ballIntens < ballIntensAnterior - 10) {
    // estava se distanciando → vai para direita
    right(TBOLA_VEL);
  } else {
    // aproximando ou parado → diagonal trás-direita
    right(TBOLA_VEL);
  }
  break;

    case 8:
    case 9:
      back(100);
      break;

    default:
      back(100);
      return;
  }

  // Corrige orientação após cada movimento
  if (abs(calcularErro(lerOrientacao(), 0)) > 10.0) {
  rotacionarParaAngulo(0);
  }
  ballIntensAnterior = ballIntens;
  Serial.print("Dir:"); Serial.print(ballDirecao);
  Serial.print(" Int:"); Serial.print(ballIntens);
  Serial.print(" Yaw:"); Serial.println(anguloRobo);
}
// =========================
// CHUTE COM CURVA — avanço reto empurra bola, curva direciona pro gol
// =========================
// Avança reto por 'tempoReto' ms, depois curva por CHUTE_CURVA_TEMPO ms
#define CHUTE_VEL_AVANCO   255
#define CHUTE_VEL_CURVA    200  // aumentado — quanto maior, mais curva
#define CHUTE_CURVA_TEMPO  150  // ms da fase de curva

int definirLadoChute() {
  float distEsq = lerEsq();
  float distDir = lerDir();

  if (distEsq < 0 && distDir < 0) return 1;
  if (distEsq < 0) return -1;
  if (distDir < 0) return  1;

  Serial.print("[CHUTE] DistEsq:"); Serial.print(distEsq);
  Serial.print(" DistDir:"); Serial.println(distDir);

  return (distEsq < distDir) ? 1 : -1;
}


void executarChute(int lado, int tempoReto) {
  // ── Fase 1: avanço reto ──
  front(CHUTE_VEL_AVANCO);
  delay(tempoReto);

  // ── Fase 2: front + rotate combinados corretamente ──
  // front:  m1=-s, m2=+s, m3=-s, m4=+s
  // rotate: m1=-s*dir, m2=-s*dir, m3=-s*dir, m4=-s*dir
  // soma:   m1=-(av+curva*lado), m2=+(av-curva*lado), m3=-(av+curva*lado), m4=+(av-curva*lado)
  int av = CHUTE_VEL_AVANCO;
  int cv = CHUTE_VEL_CURVA * lado;
  setMotor(1, constrain(-(av + cv), -255, 255));
  setMotor(2, constrain( (av - cv), -255, 255));
  setMotor(3, constrain(-(av + cv), -255, 255));
  setMotor(4, constrain( (av - cv), -255, 255));
  delay(CHUTE_CURVA_TEMPO);
  stopRobot();
  delay(2000);  

  stopRobot();
}
void chutar() {
  float distTras = lerTras();
  int   lado     = definirLadoChute();
  left(150);
  delay(100);
  stopRobot();
  Serial.print("[CHUTE] DistTras:"); Serial.print(distTras);
  Serial.print(" Lado:"); Serial.println(lado == 1 ? "direita" : "esquerda");

  if (distTras < 0) {
    executarChute(lado, 100);  // sem leitura → avanço médio
    return;
  }

  if      (distTras > 70) executarChute(lado,  100);  // perto do gol → pouco avanço
  else if (distTras > 50) executarChute(lado, 100);  // médio
  else                    executarChute(lado, 100);  // longe → bastante avanço
}
void loop() {
  // Verifica se o botão foi pressionado para zerar orientação
  verificarBotaoZero();
  InfraredResult InfraredBall = InfraredSeeker::ReadAC();
  ballDirecao = InfraredBall.Direction;
  ballIntens  = InfraredBall.Strength;
  
  //back(100);
 // delay(2000);
  //stopRobot();
 // delay(2000);
 // back(100);
  //delay(2000);
 // stopRobot();
  //delay(2000);
  Serial.println(digitalRead(botao));
  if (digitalRead(botao)) {
    digitalWrite(DRIBBLER_PIN, HIGH);
  digitalWrite(KICKER_PIN, HIGH);
    
    stopRobot();
    //Serial.println(digitalRead(botao));
    //resetPidOrientacao();
    //Serial.print("Esq: ");
    //Serial.println(lerOrientacao());
    //rotacionarParaAngulo(90);
  } else {
    digitalWrite(DRIBBLER_PIN,HIGH);
  digitalWrite(KICKER_PIN, HIGH);
  //fugirDaLinha();

   //float distEsq  = lerEsq();
   // float distDir  = lerDir();
    //float distTras = lerTras();
    //Serial.println(distTras);
    /*
    if (distEsq > 0 && distEsq < 2.5) {
      right(200);
      delay(10);
      return;
    }
    if (distDir > 0 && distDir < 2.5) {
      left(200);
      delay(10);
      return;
    }
    */
 //   if (abs(calcularErro(lerOrientacao(), 0)) > 10.0) {
 // rotacionarParaAngulo(0);
 // }
 // Serial.println(lerOrientacao());
  /*
    if (distTras > 0 && distTras < 2.5) {
      front(100);
      delay(10);
      return;
    }
    */
   

  trackBallOrientado();
  float distTras = lerTras();
    if (distTras > 0 && distTras > 160.0) {
      back(200);
      delay(10);
      return;
    }
    else if (distTras <35 && distTras >0){
      front(200);
      delay(10);
      return;  

    }

  float distEsq = lerEsq();
  if (distEsq > 0 && distEsq < 15.0) {
    right(200);
    delay(10);
    return;
  }

  float distDir = lerDir();
  if (distDir > 0 && distDir < 15.0) {
    left(200);
    delay(10);
    return;
  }
 // moverAteDistanciaEsq(15);
 //fugirDaLinha();
 //Serial.println(lerDir());
 // Serial.println(lerDir());
 // rotacionarParaAngulo(0);
  /*
  diagonalRight(150, 1);
  delay(2000);
  stopRobot();
  delay(2000);
  diagonalRight(150, -1);
  delay(2000);
  stopRobot();
  delay(2000);
  diagonalLeft(150, 1);
  delay(2000);
  stopRobot();
  delay(2000);
  diagonalLeft(150, -1);
  delay(2000);
  stopRobot();
  delay(2000);
  */
  //mostrarLinhas();
    //digitalWrite(DRIBBLER_PIN, HIGH);
    //digitalWrite(KICKER_PIN, HIGH);
    //Serial.println(digitalRead(botao));
    //moverAteDistanciaEsq(15);
    //frontPID(90,0);
    //front(90);
    //stopRobot();
    //digitalWrite(DRIBBLER_PIN, HIGH);
   // Serial.println(digitalRead(SWITCH_PIN));
    //Serial.println(lerOrientacao());
   // Serial.println(digitalRead(SWITCH_PIN));
  // Serial.println(lerLuzDir());
    
   // moveRobot(0,100,0);
   //rotacionarParaAngulo(90);
   // if (!gerenciarLinha()) {
    //  trackBall();   // ou trackBallXY()
   // }
   //trackBall();
 
  }
  Serial.println(ballIntens);
  Serial.println(ballDirecao);
  delay(10);
}