# DYP-A21 Python Driver

DYP-A21 초음파 거리 센서를 제어하기 위한 Python 라이브러리입니다. **Modbus RTU** 통신을 통해 센서 설정값을 읽고 쓸 수 있으며, **UART Controlled 트리거 모드**로 실시간 거리 측정이 가능합니다.

## 주요 기능

- ✅ **거리 측정**
  - 트리거(0xFF) 기반 실시간 측정
  - 알고리즘 처리 거리 / 실시간 거리 Modbus 읽기
  - 에코 시간 기반 거리 계산

- ✅ **센서 설정** (전원 인가 후 500ms 이내)
  - 지향각 레벨 (1~4): 감도와 각도 조절
  - 측정 거리 범위 (1~5): 50cm ~ 500cm
  - 전원 노이즈 감소 (1~5): 배터리 ~ 복잡한 간섭 환경
  - 출력 단위: mm 또는 μs
  - 스위치 임계값 및 극성 설정

- ✅ **환경 모니터링**
  - 온도 센서 (0.5°C 분해능)
  - 에코 시간 측정

- ✅ **통신 설정**
  - Slave 주소 변경 (0x01~0xFE)
  - Baud rate 설정 (9600, 19200, 38400, 57600, 115200)

## 설치

### 필수 라이브러리

```bash
pip install pyserial
```

### 프로젝트 포함

```python
import sys
sys.path.insert(0, '/path/to/scripts')
from dyp_a21 import DYPA21, AngleLevel, ScaleGrade
```

## 빠른 시작

### 트리거 측정 (가장 간단한 방식)

```python
from dyp_a21 import DYPA21

with DYPA21('/dev/ttyUSB0') as sensor:
    distance_mm = sensor.trigger()  # 트리거 전송, mm 단위 거리 반환
    print(f"거리: {distance_mm}mm")
```

### Modbus 레지스터 읽기

```python
with DYPA21('/dev/ttyUSB0') as sensor:
    # 센서 전체 설정값 읽기
    config = sensor.info()
    
    print(f"Slave 주소: {config['slave_address']}")
    print(f"Baud rate: {config['baud_rate']}")
    print(f"지향각 레벨: {config['detection_angle_level']}")
    print(f"온도: {config['temperature_C']}°C")
    
    # 개별 값 읽기
    realtime_dist = sensor.get_realtime_distance()  # mm
    processing_dist = sensor.get_processing_distance()  # mm
    temperature = sensor.get_temperature()  # °C
```

### 센서 설정 (전원 인가 후 500ms 이내)

```python
from dyp_a21 import DYPA21, AngleLevel, ScaleGrade, PowerNoiseLevel

with DYPA21('/dev/ttyUSB0') as sensor:
    # 지향각 설정: 좁은 각도
    sensor.set_detection_angle(AngleLevel.NARROW)
    
    # 측정 범위: 250cm
    sensor.set_scale_grade(ScaleGrade.CM_250)
    
    # 노이즈 감소: USB 환경
    sensor.set_power_noise_reduction(PowerNoiseLevel.USB)
    
    # 출력 단위: 밀리미터
    sensor.set_output_unit(0)  # 0=mm, 1=us
    
    # 스위치 임계값: 1000mm
    sensor.set_switch_threshold(1000)
```

### USB 연결 대기 및 자동 읽기

```bash
python scripts/read_config_on_connect.py
```

이 스크립트는:
1. USB 포트 감지 (기존 연결 또는 새 장치)
2. Permission 해소 대기
3. 외부 전원 인가 후 500ms 윈도우에서 자동 읽기
4. 설정값 테이블 형식 출력

## API 레퍼런스

### DYPA21 클래스

#### 연결 관리

```python
sensor = DYPA21('/dev/ttyUSB0', baud=115200, slave_address=0x01, timeout=1.0)
sensor.connect()        # 포트 열기
sensor.disconnect()     # 포트 닫기

# Context manager 사용 (권장)
with DYPA21('/dev/ttyUSB0') as sensor:
    # 사용
    pass
```

#### 거리 측정

| 메서드 | 반환값 | 설명 |
|--------|--------|------|
| `trigger()` | `int` (mm) | 트리거 전송, 거리 반환 (체크섬 오류 시 None) |
| `get_realtime_distance()` | `int` (mm) | 실시간 거리 (응답: 15~140ms) |
| `get_processing_distance()` | `int` (mm) | 알고리즘 처리 거리 (응답: 190~750ms) |
| `get_echo_time_mm()` | `float` (mm) | 에코 시간 기반 거리 계산 |
| `get_temperature()` | `float` (°C) | 온도 (분해능 0.5°C) |

#### 설정 (전원 인가 후 500ms 이내 유효)

| 메서드 | 파라미터 | 범위 | 설명 |
|--------|---------|------|------|
| `set_detection_angle(level)` | `AngleLevel` | 1~4 | 지향각 조절 |
| `set_scale_grade(level)` | `ScaleGrade` | 1~5 | 측정 거리 범위 |
| `set_power_noise_reduction(level)` | `PowerNoiseLevel` | 1~5 | 노이즈 억제 레벨 |
| `set_output_unit(unit)` | 0 또는 1 | mm, us | 거리 출력 단위 |
| `set_switch_threshold(mm)` | `int` | 30~5000 | 스위치 임계값 |
| `set_switch_polarity(positive)` | `bool` | - | 스위치 극성 (HIGH/LOW) |
| `set_slave_address(addr)` | `int` | 0x01~0xFE | Modbus Slave 주소 |
| `set_baud_rate(baud)` | `int` | 9600, 19200, ... | 통신 속도 변경 |

#### 상태 조회

```python
config = sensor.info()  # 전체 설정값 딕셔너리 반환
```

### Enum 클래스

#### AngleLevel
```python
AngleLevel.NARROW       # 1: ~50°
AngleLevel.MEDIUM_LOW   # 2: ~55°
AngleLevel.MEDIUM_HIGH  # 3: ~65°
AngleLevel.WIDE         # 4: ~70° (기본값)
```

#### ScaleGrade
```python
ScaleGrade.CM_50        # 1: ~50cm
ScaleGrade.CM_150       # 2: ~150cm
ScaleGrade.CM_250       # 3: ~250cm
ScaleGrade.CM_350       # 4: ~350cm
ScaleGrade.CM_500       # 5: ~500cm (기본값)
```

#### PowerNoiseLevel
```python
PowerNoiseLevel.BATTERY     # 1: 배터리 (기본값)
PowerNoiseLevel.USB         # 2: USB 전원
PowerNoiseLevel.USB_LONG    # 3: 장거리 USB
PowerNoiseLevel.SWITCHING   # 4: 스위칭 전원
PowerNoiseLevel.COMPLEX     # 5: 복잡한 간섭 (비권장)
```

#### OutputUnit
```python
OutputUnit.MM  # 0: 밀리미터 (기본값)
OutputUnit.US  # 1: 마이크로초 (÷5.75 → mm)
```

## 하드웨어 요구사항

- **DYP-A21 센서**
- **FT232 USB-UART 컨버터** (또는 동등 장치)
- **Linux/macOS/Windows** 환경

## 통신 프로토콜

### UART Controlled 트리거 모드

```
전송: [0xFF]
응답: [0xFF][Data_H][Data_L][SUM]

거리(mm) = (Data_H << 8) + Data_L
체크섬: SUM = (0xFF + Data_H + Data_L) & 0xFF
트리거 주기: 최소 150ms
```

### Modbus RTU

**Function Code 0x03 (Read):**
```
요청:  [Slave(1)][0x03(1)][Reg(2)][Count(2)][CRC(2)]
응답:  [Slave(1)][0x03(1)][ByteCnt(1)][Data(2)][CRC(2)] = 7바이트
```

**Function Code 0x06 (Write):**
```
요청:  [Slave(1)][0x06(1)][Reg(2)][Value(2)][CRC(2)]
응답:  [Slave(1)][0x06(1)][Reg(2)][Value(2)][CRC(2)] = 8바이트 (에코)
```

## 주의사항

⚠️ **UART Controlled 모드에서 Modbus 설정 명령은 전원 인가 후 500ms 이내에만 유효합니다.**

```python
# 올바른 사용:
# 1. 외부 전원 OFF
# 2. 외부 전원 ON
# 3. 즉시 sensor.set_detection_angle(...) 호출 (500ms 이내)

# 설정이 안 되면 500ms 윈도우 확인
sensor.connect()
time.sleep(0.05)  # 충분히 빠르게
sensor.set_detection_angle(AngleLevel.NARROW)  # 성공
```

## 문서

- **DYP-A21 데이터시트**: `docs/DYP-A21-Output-Interfaces-Keila.pdf`
- **E09 데이터시트**: `docs/E09 Datasheet-240222.pdf`

## 라이선스

MIT License

## 제작사

Watt Robotics
