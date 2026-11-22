// autodrive_skku 차량 펌웨어 (실차 검증본 run_test_fixed.ino 기반)
//
// 방향 극성(전진/후진, 좌/우 조향, RIGHT 모터 마운트 보정)은 2026-07 실차
// 주행으로 확정 완료 — 아래 RIGHT_IN1/IN2, setDrive(), readSerialCommand()의
// L/R 분기가 그 결과다. 추가로 손댈 필요 없음.
//
// 시리얼 프로토콜 (9600bps) — README '시리얼 프로토콜' 절과 동일하게 유지할 것
//   PC -> Arduino:
//     G / 1        주행 허용 (V 미수신 상태면 기본 속도로 전진)
//     2            후진 (레거시 — 수동 테스트용)
//     S / 3        정지 (주행 게이트 닫힘)
//     V<int>\n     부호 있는 속도 -255..255 (음수 = 후진). 최초 수신 시
//                  시리얼 속도 제어로 전환되고 워치독이 활성화된다
//     L / R        스티어링 모터 한 펄스 (120ms) — 바퀴 각도가 그만큼 이동
//     F            스티어링 모터 즉시 정지
//   Arduino -> PC:
//     0 / 1 / 2    상태 (정지/전진/후진)
//     P <adc>      조향 POT 원시값(A6, 0~1023), 50ms마다 — 항상 보냄(미장착이면
//                  플로팅값이라 의미 없는 숫자, 값 자체로 장착 여부 판단 불가).
//                  전용 보드 아니고 이 메가에 직결 — 모터 제어와 같은 시리얼
//                  왕복이라 캘리브레이션/향후 폐루프 조향에 지연 없이 바로 쓸 수 있음.
//     E <count> <ms>  뒷바퀴 로터리 엔코더(KY-040, CLK=D33 · DT=D32) 누적 카운트와
//                  그 값을 읽은 순간의 millis(), 50ms마다 — 항상 보냄(미장착이면 0 고정).
//                  4체배(A·B 양쪽 엣지) → KY-040(EC11 20클릭) 1회전 = 80 카운트.
//                  전진 = + (2026-11-21 실차에서 바퀴 앞으로 돌려 확인, ENC_DIR로 맞춤).
//                  카운트는 부팅(포트 열 때 리셋) 시 0, 32bit 부호 있는 정수.

const int LEFT_PWM = 4;
const int LEFT_IN1 = 26;
const int LEFT_IN2 = 27;

const int RIGHT_PWM = 3;
// 우측 모터는 좌측과 마주보게(대칭) 장착돼 있어, LEFT와 같은 극성을 주면 실제
// 회전방향이 반대로 나온다 — IN1/IN2를 뒤바꿔 보정(배선은 그대로).
const int RIGHT_IN1 = 25;
const int RIGHT_IN2 = 24;

const int STEER_PWM = 2;
const int STEER_IN1 = 22;
const int STEER_IN2 = 23;
// LEFT/STEER는 핀 배선이 원래부터 맞았음 — 방향 보정은 setDrive()/
// readSerialCommand()의 L·R 분기(명령 해석 단계)에서 처리한다.

const int POT_PIN = A6;  // 조향 POT (2026-07 실장착: A6) — 미장착이어도 analogRead는 안전(플로팅값만 나감)

// 뒷바퀴 엔코더 KY-040. 실배선은 CLK→D33, DT→D32 (2026-11-21 확인). 32·33은 외부/핀체인지
// 인터럽트가 없는 핀이라 폴링한다: Timer0 COMPA 인터럽트(약 1 kHz, millis와 같은 타이머 —
// OCR0A는 D13 PWM용인데 D13은 안 쓰고, D4 PWM은 OCR0B라 영향 없음) + loop()에서도 매번 샘플.
// KY-040 보드에 풀업이 있어도 내부 풀업을 같이 켠다 — 선이 빠져도 HIGH로 고정된다.
const int ENC_A_PIN = 33;   // KY-040 CLK
const int ENC_B_PIN = 32;   // KY-040 DT
const int8_t ENC_DIR = -1;  // 이 차 장착 방향에선 원시 디코딩이 전진 때 감소 → 뒤집어서 전진 = +

const int DEFAULT_SPEED = 100;
const int STEER_SPEED = 160;
const unsigned long STEER_PULSE_MS = 120;
const unsigned long WATCHDOG_MS = 500;   // V 수신 후 시리얼 두절 시 정지
const unsigned long STATUS_PRINT_MS = 100;
const unsigned long POT_PRINT_MS = 50;
const unsigned long ENC_PRINT_MS = 50;   // 9600bps 대역: P·상태·E 합쳐 약 550 B/s (한도 960 B/s)

bool canGo = false;
int commandedSpeed = 0;      // 부호 있는 속도. V 명령 또는 레거시 G/2로 설정
bool useSerialSpeed = false; // 첫 V 수신 후 true → 워치독 활성화

unsigned long steerStartTime = 0;
unsigned long lastSerialTime = 0;
int lastState = -1;
unsigned long lastStatusPrintTime = 0;
unsigned long lastPotPrintTime = 0;
unsigned long lastEncPrintTime = 0;

volatile long encCount = 0;   // ISR에서만 증감
volatile uint8_t encPrev = 0; // 직전 (A<<1)|B

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(20);  // parseInt가 오래 블록되지 않게

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(STEER_PWM, OUTPUT);
  pinMode(STEER_IN1, OUTPUT);
  pinMode(STEER_IN2, OUTPUT);

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  encPrev = (digitalRead(ENC_A_PIN) << 1) | digitalRead(ENC_B_PIN);
  OCR0A = 0x80;              // Timer0 카운트 중간에서 COMPA → 약 976 Hz 샘플
  TIMSK0 |= _BV(OCIE0A);

  stopCar();
  steerStop();
  printState(0);
}

void loop() {
  noInterrupts();            // 타이머 ISR과 겹치지 않게
  encoderSample();
  interrupts();

  readSerialCommand();

  // 조향 펄스 자동 종료
  if (steerStartTime > 0 && millis() - steerStartTime >= STEER_PULSE_MS) {
    steerStop();
  }

  bool watchdogTripped = useSerialSpeed && (millis() - lastSerialTime > WATCHDOG_MS);

  int speed = (!canGo || watchdogTripped) ? 0 : commandedSpeed;

  setDrive(speed);
  printState(speed > 0 ? 1 : (speed < 0 ? 2 : 0));
  printPot();
  printEncoder();
}

void readSerialCommand() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    lastSerialTime = millis();

    if (cmd == 'G' || cmd == '1') {
      canGo = true;
      if (!useSerialSpeed) {
        commandedSpeed = DEFAULT_SPEED;
      }
    }
    else if (cmd == '2') {
      canGo = true;
      if (!useSerialSpeed) {
        commandedSpeed = -DEFAULT_SPEED;
      }
    }
    else if (cmd == 'S' || cmd == '3') {
      canGo = false;
    }
    else if (cmd == 'V') {
      long v = Serial.parseInt();
      commandedSpeed = constrain(v, -255, 255);
      useSerialSpeed = true;
    }
    else if (cmd == 'F') {
      steerStop();
    }
    else if (cmd == 'L') {
      // PC가 보내는 'L'(좌회전 요청)을 물리적으로 steerRight()가 처리 — 실차
      // 방향 확정 결과(파일 상단 참고).
      steerRight();
    }
    else if (cmd == 'R') {
      steerLeft();
    }
  }
}

void setDrive(int speed) {
  // "전진 명령(양수)"의 물리적 의미를 여기서 반전 — G/1, '2', V<int> 전부 이
  // 함수를 거치므로 한 줄로 충분. 실차 방향 확정 결과(파일 상단 참고).
  speed = -speed;
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(LEFT_PWM, speed);
    analogWrite(RIGHT_PWM, speed);
  }
  else if (speed < 0) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    analogWrite(LEFT_PWM, -speed);
    analogWrite(RIGHT_PWM, -speed);
  }
  else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(LEFT_PWM, 0);
    analogWrite(RIGHT_PWM, 0);
  }
}

void stopCar() {
  setDrive(0);
}

void steerLeft() {
  digitalWrite(STEER_IN1, LOW);
  digitalWrite(STEER_IN2, HIGH);
  analogWrite(STEER_PWM, STEER_SPEED);
  steerStartTime = millis();
}

void steerRight() {
  digitalWrite(STEER_IN1, HIGH);
  digitalWrite(STEER_IN2, LOW);
  analogWrite(STEER_PWM, STEER_SPEED);
  steerStartTime = millis();
}

void steerStop() {
  digitalWrite(STEER_IN1, LOW);
  digitalWrite(STEER_IN2, LOW);
  analogWrite(STEER_PWM, 0);
  steerStartTime = 0;
}

void printState(int state) {
  unsigned long now = millis();

  if (state != lastState || now - lastStatusPrintTime >= STATUS_PRINT_MS) {
    Serial.println(state);
    lastState = state;
    lastStatusPrintTime = now;
  }
}

void printPot() {
  // POT 미장착이면 A6가 플로팅이라 값이 의미 없음 — 그래도 라인 자체는 계속
  // 보낸다. "진짜 POT이 있는지"는 파이썬 쪽에서 풀락 스윕 중 실제로 값이
  // 유의미하게 움직이는지로 판단한다(이 펌웨어는 하드웨어 유무를 모름).
  unsigned long now = millis();
  if (now - lastPotPrintTime >= POT_PRINT_MS) {
    Serial.print("P ");
    Serial.println(analogRead(POT_PIN));
    lastPotPrintTime = now;
  }
}

ISR(TIMER0_COMPA_vect) {
  encoderSample();
}

void encoderSample() {
  // 4체배 직교 디코딩: (이전 AB, 현재 AB) 16가지 → -1/0/+1.
  // 접점 채터링은 한 채널만 왔다 갔다 하므로 +1/-1이 상쇄된다. 두 비트가 한꺼번에 바뀐
  // 경우(엣지를 놓침)는 방향을 모르므로 0으로 버린다.
  static const int8_t TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  uint8_t cur = (digitalRead(ENC_A_PIN) << 1) | digitalRead(ENC_B_PIN);
  encCount += ENC_DIR * TABLE[(encPrev << 2) | cur];
  encPrev = cur;
}

void printEncoder() {
  unsigned long now = millis();
  if (now - lastEncPrintTime >= ENC_PRINT_MS) {
    noInterrupts();          // long(4바이트) 복사 중 ISR이 끼어들면 값이 찢어진다
    long c = encCount;
    interrupts();
    Serial.print("E ");
    Serial.print(c);
    Serial.print(' ');
    Serial.println(now);
    lastEncPrintTime = now;
  }
}
