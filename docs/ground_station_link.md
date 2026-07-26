# 지상국 링크 요구사항과 RC 타임아웃 진단

비행 중 갑작스러운 모터 정지 대부분은 펌웨어가 아니라 **지상국 쪽 무선
링크**에서 온다. 이 문서는 그 판별법과, 2026-07-27에 실제로 겪은 사례의
근본 원인을 남긴다.

## 1. RC 타임아웃은 고장이 아니라 안전장치다

`dual_imu_cascade_pwm`는 `rc` 패킷이 `RC_TIMEOUT_MS`(500ms) 넘게 끊기면
`fault_rc`를 세우고 즉시 시동을 해제한다. 조종 입력이 끊긴 채로 모터가
마지막 명령대로 계속 도는 것을 막는 장치다.

**따라서 `Fault_RC=1`을 봤을 때 고쳐야 할 것은 워치독이 아니라 링크다.**
`RC_TIMEOUT_MS`를 키우면 증상만 가려지고, 실제 링크 두절 시 대응만 늦어진다.

## 2. 지상국 하드웨어 요구사항

### ⚠️ 게임패드는 USB로 연결한다 (블루투스 금지)

노트북의 WiFi와 블루투스가 **같은 콤보 칩**이면 2.4GHz 무선부와 안테나를
공유한다. 이 프로젝트의 실제 장비가 그렇다.

```text
WiFi : 00:14.3  Intel Meteor Lake CNVi → Wi-Fi 6E AX211
BT   : 8087:0033  Intel Corp. AX211 Bluetooth      ← 같은 AX211
```

이 경우 블루투스 게임패드를 쓰면 **전파 간섭이 아니라 하드웨어 점유 경합**이
발생한다. 칩 내부 coexistence 중재기가 시간을 쪼개 WiFi와 BT에 번갈아
라디오를 주기 때문이다. 그래서:

- **드론 AP 채널을 바꿔도 소용없다.** BT가 다른 주파수로 호핑해도 안테나는
  여전히 BT가 잡고 있다. 스펙트럼 겹침이 문제가 아니라 하드웨어 점유가
  문제다. (`WIFI_CHANNEL = 11`은 인근 AP 혼잡 회피 목적이며 그것대로 유효하다.)
- DualSense는 특히 나쁘다. 블루투스 연결 시 커널 `hid_playstation` 드라이버가
  확장 리포트 모드를 켜서 게임패드 입력 외에 **모션센서와 터치패드**까지
  올라온다. BT가 airtime을 지속적으로 크게 문다.

**무선 패드를 꼭 써야 한다면** 드론 링크용 **USB WiFi 동글**을 따로 꽂는다.
동글은 자체 라디오·안테나를 가지므로 내장 칩의 BT와 경합하지 않는다.

### 그 밖의 준수 사항

- 비행 세션 중에는 **노트북 절전(서스펜드)을 끈다.** 서스펜드는 링크를 끊고,
  복귀 시 NetworkManager가 저장된 다른 네트워크로 자동 접속해버린다.
- `Drone_Tuning` 연결 프로파일의 `802-11-wireless.powersave`는 `2 (disable)`로
  둔다. 확인: `nmcli -f 802-11-wireless.powersave connection show Drone_Tuning`
- 무장 전에 실제로 `Drone_Tuning`에 붙어 있는지 확인한다:
  `iw dev wlan0 link`

## 3. 왜 업링크만 죽는가 (비대칭의 이유)

증상이 "텔레메트리는 멀쩡한데 조종만 끊긴다"로 나타나는 이유다. **드론 쪽
라디오는 아무 영향도 받지 않는다**는 점이 비대칭을 만든다.

| | 다운링크 (드론 → PC) | 업링크 (PC → 드론) |
|---|---|---|
| 막히는 것 | 수신 기회 | **송신 기회** |
| 복구 주체 | 드론(무영향 라디오)의 802.11 재전송 | 없음 — 드라이버 큐에서 대기 |
| 결과 | ms 단위 손실, 거의 안 보임 | **수백 ms 정체 후 버스트** |

드론은 자기 라디오를 100% 자유롭게 쓴다. PC가 BT 슬롯이라 못 받으면 ACK가
안 가고 드론이 MAC 레벨에서 즉시 재전송하므로 수 ms 안에 메꿔진다. 반대로
PC가 보낼 때는 중재기가 라디오를 내줘야 하는데, BT가 잡고 있으면 프레임이
큐에서 대기만 한다. 첫 송신 기회 자체가 없으니 802.11 재전송 로직이 개입할
여지도 없다.

> **한계 명시**: 슬롯 단위 중재만으로는 보통 수~수십 ms 지터가 예상된다.
> 실측된 250~450ms는 그보다 한 자릿수 이상 크며, 큐 적체·재시도 백오프·
> 전송률 폴백이 겹쳐 증폭된 것으로 추정하지만 **이 증폭 부분은 측정으로
> 확인하지 않았다.** 상관관계와 비대칭 방향만 데이터로 확정된 사실이다.

## 4. 로그로 판별하는 법

`Fault_RC`가 떴을 때, 원인이 **드론 측인지 PC 측인지**는 CSV만으로 가려진다.

| 관찰 | 해석 |
|---|---|
| rc 정지 중에도 텔레메트리가 50ms 간격 유지 | `sendTelemetry()`는 rc를 읽는 `udp_task`와 **같은 태스크**다. 이게 돌았다면 드론 수신 루프는 안 멈춘 것 → **PC/링크 측 문제** |
| rc 정지 중 텔레메트리도 함께 끊김 | 드론 `udp_task`가 막혔거나 양방향 링크 두절 |
| `RC_Dropped_Pkts`가 안 늘어남 | 공중 유실이 아니다. 시퀀스 점프가 없으므로 **안 보냈거나 늦게 순서대로 도착**한 것 |
| 복구 시 `RC_Total_Pkts`가 한꺼번에 크게 증가 | 큐에 밀려 있다가 버스트로 배달됨 = 지연 |
| `PID_Loop_Hz`가 1000 유지 | 드론 제어 루프는 정상 |

### 무장 구간별 rc 레이트 측정

가장 결정적인 단일 지표다. 정상이면 **정확히 20.0Hz**(`CTRL_LOOP_HZ = 20`)가
나온다. `RC_Total_Pkts`는 부팅 이후 누적이고 재시동해도 0으로 돌아가지 않으므로,
반드시 **무장 구간별 증분**으로 계산해야 한다.

```python
import pandas as pd, datetime, glob
def ts(s): return datetime.datetime.strptime(s, "%H:%M:%S.%f")

for f in sorted(glob.glob('logs/flight_log_YYYY-MM-DD_*.csv')):
    d = pd.read_csv(f)
    if len(d) < 5 or 'Armed' not in d: continue
    d['t'] = d.Timestamp.map(ts)
    d['seg'] = (d.Armed.diff() != 0).cumsum()
    for _, g in d[d.Armed == 1].groupby('seg'):
        dur = (g.t.max() - g.t.min()).total_seconds()
        if dur < 0.5: continue
        n = int(g.RC_Total_Pkts.max() - g.RC_Total_Pkts.min())
        dr = int(g.RC_Dropped_Pkts.max() - g.RC_Dropped_Pkts.min())
        print(f"{g.Timestamp.min()} {dur:6.1f}s {n:5d}pkt {n/dur:6.1f}Hz drop+{dr}")
```

### 링크 자체를 재는 법

```bash
ping -D -i 0.02 -c 500 192.168.4.1          # 패드 USB / BT 각각 비교
iw dev wlan0 station dump | grep -E "tx retries|tx failed|signal"
```

`tx retries`·`tx failed`가 크게 뛰면 송신측 봉쇄가 수치로 확정된다.

## 5. 사례: 2026-07-27 RC 타임아웃 연발

### 증상

03:51 이후 모든 비행이 무장 후 1.8~9.9초 만에 `Fault_RC`로 정지.

### 근본 원인

**02:48:16에 DualSense를 USB에서 뽑고, 03:50:42에 블루투스로 재연결한 것.**

```text
03:50:42  playstation 0005:054C:0CE6.0006:
          BLUETOOTH HID v1.00 Gamepad [DualSense Wireless Controller]
03:51:06  ← 첫 번째 불량 로그
```

### 상관관계 (예외 0건)

| 시간대 | 패드 연결 | 무장 구간 rc 레이트 | 결과 |
|---|---|---|---|
| 01:20~02:25 | **USB** (Bus=0003) | 20.0~20.1Hz | 정상 (15~43초 유지) |
| 02:37~02:48 | **USB** | 20.0~20.1Hz | 정상 |
| **03:50:42~** | **블루투스** (Bus=0005) | **13.3~19.6Hz** | **전 구간 RC 타임아웃** |

USB 8개 구간 전부 정상, 블루투스 6개 구간 전부 실패.

### 배제된 원인

- **펌웨어**: 정상 세션(02:38, 02:43)과 불량 세션이 **같은 빌드**(둘 다 34필드
  텔레메트리, commit `1497d74`). 그 사이 코드 변경 없음.
- **BMM350 mag 융합**: 01:45부터 켜져 있었고 그 세션들은 20.0Hz 무결점.
  `mag 1`은 무죄다.
- **공중 패킷 유실**: 정지 구간 내내 `RC_Dropped_Pkts`가 34에서 불변.
- **드론 측 정지**: rc가 450ms간 0건인 동안 텔레메트리는 47/50/50/51/49/50/53ms로
  완벽히 규칙적. 복구 시 `RC_Total`이 313 → 314 → 320으로 버스트 도착.

### 부수 요인

같은 시간대에 노트북이 1시간에 4번 서스펜드했다(02:31, 03:44, 03:44:17, 03:50).
03:50 복귀 직후 NetworkManager가 `Member@UOS`로 자동 접속했고, `Drone_Tuning`으로
되돌릴 때 첫 association이 실패했다(`disconnected during association`, BSSID
10초 차단). 비행 세션 중 절전은 꺼둘 것.

## 6. 비행 전 링크 체크리스트

- [ ] 게임패드가 **USB로** 연결돼 있다 (`/proc/bus/input/devices`에서 `Bus=0003`)
- [ ] 노트북 절전/서스펜드 비활성
- [ ] `iw dev wlan0 link`가 `Drone_Tuning`을 가리킨다
- [ ] 무장 후 첫 5초 로그에서 rc 레이트가 20.0Hz다
- [ ] `RC_Dropped_Pkts`가 늘지 않는다

## 관련

- UDP 명령·텔레메트리 필드: [udp_protocol.md](udp_protocol.md)
- 전원 인가 벤치 절차: [power_on_bench_procedure.md](power_on_bench_procedure.md)
- 지상국 도구: [`scripts/README.md`](../scripts/README.md)
