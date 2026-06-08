# DWM3001CDK UWB 지게차 안전 시스템

UWB(Ultra-Wideband) 기반 TDOA 측위를 이용한 지게차-작업자 충돌 예측 시스템입니다.

---

## 시스템 개요

```
작업자 (Sender/Tag)
  └─ UWB 신호 발사
        ↓
지게차 위 Receiver 4대 (DWM3001CDK, LISTENER 모드)
  └─ 각각 TS4ns 타임스탬프로 수신
        ↓
Raspberry Pi 4
  └─ 로그 수집 → TDOA 계산 → 작업자 위치 파악 → 충돌 예측/회피
```

- **Sender**: 작업자 착용 UWB 모듈 (INITF 역할)
- **Receiver**: 지게차 탑재 DWM3001CDK 4대 (LISTENER 모드)
- **호스트**: Raspberry Pi 4 (빌드 + 플래시 + 데이터 수집)

---

## 하드웨어 구성

| 장치 | 역할 | 수량 |
|---|---|---|
| DWM3001CDK | UWB 모듈 (nRF52833 + QM33) | 4대 (Receiver) + 1대 (Sender) |
| Raspberry Pi 4 | 빌드 머신 + 데이터 수집 서버 | 1대 |
| MacBook (개발) | 원격 접속 및 코드 수정 | 1대 |

### DWM3001CDK 포트 구분

| 포트 | 역할 |
|---|---|
| **J9** | J-Link (플래시) + UART 통신 |
| **J20** | USB CDC 통신 (USB_ENABLE 옵션 시) |

---

## 개발 환경 구축

### 1. Raspberry Pi OS 설치
Raspberry Pi Imager로 64-bit OS를 microSD에 굽습니다.

### 2. Raspberry Pi SSH 접속

### 3. Raspberry Pi 패키지 설치

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y cmake make python3 python3-pip python3-venv git screen
```

### 4. ARM 툴체인 설치 (aarch64용)

```bash
cd ~
wget "https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-aarch64-linux.tar.bz2"
sudo mkdir -p /opt/gcc-arm
sudo tar -xvf gcc-arm-none-eabi-10.3-2021.10-aarch64-linux.tar.bz2 -C /opt/gcc-arm
echo 'export PATH="/opt/gcc-arm/gcc-arm-none-eabi-10.3-2021.10/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# 확인
arm-none-eabi-gcc --version
```

### 5. J-Link 설치

1. PC에서 https://www.segger.com/downloads/jlink/ 접속합니다. 
2. Linux ARM 64-bit `.deb` 다운로드 후 Raspberry로 전송합니다.
3. .deb파일을 dpkg로 설치합니다.
```bash
# Raspberry Pi에서
sudo dpkg -i ~/JLink_Linux_V946_arm64.deb
```

### 6. SDK 다운로드 및 Raspberry Pi로 전송

1. Qorvo 공식 사이트 https://www.qorvo.com/products/p/DWM3001CDK 에서 하단 Documents/Software **DW3xxx & QM3xxx SDK v1.1.1.zip**를 다운로드합니다.
2. 압축을 풀면 나오는 폴더인 DW3_QM33_SDK를 Raspberry Pi로 전송합니다.


### 7. 이 repo clone 및 수정 파일 적용

Raspberry Pi에서 이 repo를 clone한 후 수정 파일들을 SDK에 덮어써주세요:

```bash
# Raspberry Pi에서
cp uwb-forklift-safety/firmware/task_listener.c ~/DW3_QM33_SDK/SDK/Firmware/DW3_QM33_SDK_1.1.1/Src/Apps/Src/listener/
cp uwb-forklift-safety/firmware/project_CLI.cmake ~/DW3_QM33_SDK/SDK/Firmware/DW3_QM33_SDK_1.1.1/Projects/FreeRTOS/CLI/DWM3001CDK/
cp uwb-forklift-safety/scripts/uwb_logger.py ~/
cp uwb-forklift-safety/scripts/flash_all.sh ~/
chmod +x ~/flash_all.sh
```

### 9. Python 가상환경 세팅

```bash
cd ~/DW3_QM33_SDK/SDK/Firmware/DW3_QM33_SDK_1.1.1
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install pyserial
```

---

## 소스코드 수정 내용

### 1. LISTENER 전체 패킷 출력 고정 (`task_listener.c`)

LSTN 배열이 기본 6바이트로 잘리는 문제를 해결하기 위해 mode를 1로 고정했습니다. Sender MAC 주소 파싱을 위해 전체 패킷이 필요합니다.

```c
error_e send_to_pc_listener_info(...)
{
    mode = 1;  // 전체 패킷 출력 고정
    ...
}
```

### 2. J9 UART 통신 설정 (`project_CLI.cmake`)

`USB_ENABLE`을 제거하여 J20 없이 J9 하나로 플래시 + 통신이 모두 가능하도록 설정합니다.

```cmake
set(CMAKE_CUSTOM_C_FLAGS
    "-Werror \
    -DBOARD_CUSTOM \
    -DCONFIG_GPIO_AS_PINRESET"
)
```

---

## 펌웨어 빌드 및 플래시

### flash_all.sh 시리얼 넘버 설정

보드별 J-Link 시리얼 넘버를 확인 후 `~/flash_all.sh`에 등록하세요:

```bash
ls /dev/serial/by-id/
```

| 보드 | J-Link 시리얼 |
|---|---|
| 보드1 | 760144486 |
| 보드2 | 760197326 |

### 빌드 + 플래시 실행

```bash
cd ~/DW3_QM33_SDK/SDK/Firmware/DW3_QM33_SDK_1.1.1
source .venv/bin/activate
rm -rf BuildOutput/CLI
python3 Projects/FreeRTOS/CLI/DWM3001CDK/CreateTarget.py -build Debug
cd BuildOutput/CLI/FreeRTOS/DWM3001CDK/Debug
make -j4 #수정 코드 빌드 
~/flash_all.sh #코드를 J-Link로 자동 플래시해주는 sh 실행
```

---

## 보드 초기 설정

보드를 처음 플래시한 후 아래 명령어를 실행해 부팅 시 자동으로 LISTENER 모드로 시작하도록 설정합니다.

```bash
TERM=vt100 screen /dev/ttyACM0 115200
```

접속 후:
```
SETAPP LISTENER
SAVE
```

---

## 로그 수집

두 보드의 LISTENER 로그를 동시에 수집하고 파일로 저장합니다.

```bash
source ~/DW3_QM33_SDK/SDK/Firmware/DW3_QM33_SDK_1.1.1/.venv/bin/activate
python3 ~/uwb_logger.py
```

로그는 `~/uwb_logs/` 폴더에 저장됩니다.
## 로그 포맷 분석

### LISTENER 출력 형식

```json
{"LSTN":[49,2B,01,00,26,13,00,FF,18,5A,...],"TS4ns":"0xDAB2CEA0","O":1123,"rsl":-50.96,"fsl":-51.73}
```

| 필드 | 의미 |
|---|---|
| **LSTN** | Raw IEEE 802.15.4 MAC 프레임 (hex) |
| **TS4ns** | UWB 수신 타임스탬프 (4ns 단위, TDOA 계산 핵심값) |
| **O** | Clock offset (ppm) |
| **rsl** | 수신 신호 강도 (dBm) |
| **fsl** | 첫 번째 경로 신호 강도 (dBm) |

### LSTN 배열 구조 (IEEE 802.15.4 프레임)

| 바이트 | 의미 |
|---|---|
| 0-1 | Frame Control |
| 2 | Sequence Number |
| 3-4 | PAN ID |
| 5-6 | Destination Address |
| **7-8** | **Source Address (Sender MAC = 작업자 ID)** |
| 나머지 | STS + Payload |

---


## 디렉토리 구조

```
~/DW3_QM33_SDK/
└── SDK/Firmware/DW3_QM33_SDK_1.1.1/
    ├── Projects/FreeRTOS/CLI/DWM3001CDK/
    │   └── project_CLI.cmake          # 빌드 설정 (J9 UART...)
    └── Src/Apps/Src/listener/
        └── task_listener.c            # LISTENER mode=1 고정

~/uwb_logger.py    # 로그 수집 스크립트
~/flash_all.sh     # 순차 플래시 스크립트
~/uwb_logs/        # 수집된 로그 파일들
```
