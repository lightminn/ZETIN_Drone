// README에서 링크하는 유지 문서 원본. PDF는 이 파일에서 재생성한다.
#set document(
  title: "단일 PID 대비 캐스케이드(듀얼) PID의 이점",
  author: "ZETIN Drone",
)
#set page(paper: "a4", margin: (x: 2.2cm, y: 2.4cm), numbering: "1")
#set text(lang: "ko", font: ("Libertinus Serif", "NanumMyeongjo"), size: 10.5pt)
#set par(
  justify: true,
  leading: 0.72em,
  justification-limits: (
    tracking: (min: -0.012em, max: 0.012em),
    spacing: (min: 75%, max: 120%),
  ),
)
#set heading(numbering: "1.")
#show heading: set text(font: ("Libertinus Serif", "NanumBarunGothic"))
#show heading.where(level: 1): it => block(above: 1.6em, below: 0.9em)[
  #text(size: 1.15em, fill: rgb("#1a3a5c"))[#it]
]
#show raw: set text(font: ("D2Coding", "DejaVu Sans Mono"), size: 0.88em)
#set table(stroke: (x, y) => (
  top: if y <= 1 { 0.7pt } else { 0pt },
  bottom: 0.7pt,
))
#show table.cell.where(y: 0): strong

#let accent = rgb("#1a3a5c")
#let src(body) = text(size: 0.82em, fill: rgb("#6b6b6b"))[#raw(body)]

#let note(title: none, body) = block(
  width: 100%,
  breakable: false,
  fill: rgb("#f2f5f8"),
  stroke: (left: 2.5pt + accent),
  inset: (x: 12pt, y: 10pt),
  radius: (right: 3pt),
)[
  #if title != none [#text(weight: "bold", fill: accent)[#title] \ ]
  #body
]

#let warn(title: none, body) = block(
  width: 100%,
  breakable: false,
  fill: rgb("#fdf4ec"),
  stroke: (left: 2.5pt + rgb("#b5651d")),
  inset: (x: 12pt, y: 10pt),
  radius: (right: 3pt),
)[
  #if title != none [#text(weight: "bold", fill: rgb("#8a4a12"))[#title] \ ]
  #body
]

#let bx(body, fill: white) = rect(
  inset: (x: 8pt, y: 7pt),
  radius: 3pt,
  stroke: 0.6pt + rgb("#556"),
  fill: fill,
  align(center, text(size: 0.85em)[#body]),
)
#let ar = box(inset: (x: 4pt), baseline: -0.15em)[#text(
  fill: accent,
  size: 1.1em,
)[→]]

#title[단일 PID 대비 캐스케이드(듀얼) PID의 이점]

#align(center)[
  #text(size: 0.9em, fill: rgb("#666"))[
    `firmware/flight/dual_imu_cascade_pwm` (현재) vs
    `firmware/archive/legacy_flight/dual_imu_pid_pwm` (레거시) · 2026-08-04
  ]
]

#v(0.5em)

#note(title: "한 줄 요약")[
  캐스케이드의 이점은 *"각속도(deg/s)가 명시적인 신호가 된다"* 는 한 가지에서
  전부 파생된다. 그 신호를 *자를 수 있고*, 거기에 *적분과 감쇠를 붙일 수 있고*,
  *명령으로 줄 수 있고*, *따로 튜닝하고 시험할 수 있다.* 단일 PID에서는 이
  신호가 식 안에 숨어 있어 손댈 수 있는 손잡이가 없다.
]

= 두 구조

*단일 PID* --- 각도 오차가 곧바로 모터 출력이 된다. 루프가 하나뿐이다.

#align(center)[
  #stack(
    dir: ltr,
    spacing: 0pt,
    bx[$theta_"cmd"$],
    ar,
    bx(fill: rgb("#fdf3e7"))[각도 PID \ #text(0.9em)[2.5 / 0.005 / 1.2]],
    ar,
    bx[믹서],
    ar,
    bx[모터],
  )
]

*캐스케이드(듀얼) PID* --- 각도 루프가 "목표 각속도"를 만들고, 각속도 루프가
그것을 쫓는다. 두 루프 사이에 손댈 수 있는 지점이 생긴다.

#align(center)[
  #stack(
    dir: ltr,
    spacing: 0pt,
    bx[$theta_"cmd"$],
    ar,
    bx(
      fill: rgb("#eef4ea"),
    )[각도 P (외부) \ #text(0.9em)[250 Hz · $K_(p a)$ = 6.0]],
    ar,
    bx(fill: rgb("#fce9e9"))[제한 \ #text(0.9em)[±300 dps]],
    ar,
    bx(
      fill: rgb("#eaf0f7"),
    )[각속도 PID (내부) \ #text(0.9em)[1 kHz · 0.5 / 0.05 / 0.015]],
    ar,
    bx[믹서 \ #text(0.9em)[+desat]],
    ar,
    bx[모터],
  )
]

두 펌웨어 모두 PID 태스크는 Core 1에서 1 kHz로 동일하게 돈다. 루프 주기가 달라서
생기는 차이는 없다.

= 캐스케이드가 주는 것

== 명령을 물리적 단위로 자를 수 있다

#src("dual_imu_cascade_pwm.ino:1549-1552, MAX_TARGET_RATE_RP = 300")

외부 루프의 출력은 "목표 각속도(dps)"라는 실제 물리량이고, 내부 루프에 넘기기
전에 ±300 dps로 자른다. $K_(p a)$ = 6.0은 "5° 오차 → 30 dps 명령"이라는 뜻이며,
50° 오차가 들어와도 명령은 300 dps에서 멈춘다.

*왜 중요한가.* 단일 PID에서 각도 게인을 같은 비율로 올리면, 큰 각도 오차가
곧바로 믹서를 포화시킨다. 포화하면 자세 명령의 비율이 깨지고(roll이 pitch보다
많이 잘리는 식) 적분기는 계속 쌓인다 --- 자세 authority 상실과 windup이 동시에
온다. 캐스케이드는 그 앞에서 명령 크기를 물리적으로 의미 있는 값으로 제한하므로,
*각도 게인을 공격적으로 쓰면서도 내부 루프를 선형 영역에 남겨둘 수 있다.*

== 외란을 각속도 레벨에서 제거한다

#src("dual_imu_cascade_pwm.ino:1596-1606, Ki_Rate = 0.05, I_TERM_MAX_US = 50")

모터 추력 불균형이나 CG 치우침 같은 *일정 토크 외란*은 먼저 각속도로 나타나고,
그것이 적분된 뒤에야 각도로 나타난다. 캐스케이드의 적분기는 각속도 오차에
붙으므로, 기체가 실제로 기울어져 각도 오차가 쌓이기를 기다리지 않는다. *적분기
하나만큼 빠르게* 외란을 지운다.

레거시에서는 이 자리가 사실상 비어 있었다. $K_i$ = 0.005에 `errorSum` 클램프가
±15이므로 최대 기여가 *0.075 µs* --- PWM 1 µs도 만들지 못한다
#src("dual_imu_pid_pwm.ino:336-338"). 현재는 적분값을 모터 출력 기여(µs) 단위로
누적하고 ±50 µs로 제한한다. 호스트 SIL에서 0.005는 10초 뒤에도 클램프의 1.4%만
쓴다는 것이 확인되어 0.05로 올렸다 #src("cascade :144-148, commit 735f77d").

실측 근거도 있다. yaw는 P 단독으로 모터 토크 불균형에서 정상상태 각속도 오차가
최대 +16.6 dps 남아 자동 heading 잠금이 영영 걸리지 않았다
#src("cascade :1600-1604 주석").

== anti-windup이 실제로 의미를 갖는다

#src("dual_imu_cascade_pwm.ino:1578-1608, mixAndDesaturate")

적분값의 단위가 *모터 출력 기여(µs)* 이므로 `I_TERM_MAX_US` 클램프가 "적분기가
모터를 최대 몇 µs까지 밀 수 있는가"라는 직접적인 뜻을 갖는다. 각도·초 단위로
쌓던 레거시에서는 클램프 값을 봐도 그것이 출력에 얼마를 의미하는지 알 수 없었다.

게다가 멈추는 조건이 정확하다. *믹서가 자세 명령을 실제로 잘랐을 때*
(`mix.scaled`)만 적분을 멈추고, collective만 이동한 경우에는 자세 authority가
보존되므로 계속 적분한다.

== 두 루프를 따로 튜닝하고 따로 시험할 수 있다

내부 루프의 임무는 "목표 각속도 추종"이다. 이것은 *자세 추정과 무관하게* 시험할
수 있다 --- 자이로만 있으면 되고, 벤치에서 rate 스텝 응답만 보면 된다. ESC 지연,
모터 스핀업 시상수, 프레임 진동 같은 액추에이터 쪽 문제는 전부 여기서 끝난다.

외부 루프는 그다음이고, 손잡이가 스칼라 하나($K_(p a)$)뿐이며 "각도 오차 1°당
몇 dps를 요구할 것인가"라는 해석이 붙는다.

이 프로젝트가 실제로 그렇게 했다. `Ki_Rate`는 호스트 SIL에서 내부 루프만 놓고
결정했고 #src("commit 735f77d"), 최종 게인은 테더 벤치에서 상향 튜닝하는 것으로
남겨두었다 #src("cascade :147-148"). 단일 PID에서는 $K_p$와 $K_d$가 자세 추정
품질·액추에이터 지연·기체 관성에 동시에 얽혀 있어 이런 단계적 분리가 안 된다.

== 자세 추정이 흔들려도 안쪽 루프는 살아 있다

#src("dual_imu_cascade_pwm.ino:1572-1574")

내부 루프는 자이로만 본다. 자세 추정(상보필터)이 비행 중 가속도나 진동에 잠시
끌려가도, 각속도 감쇠는 그대로 동작한다. 코드에도 이 성질이 명시적으로 쓰인다 ---
*yaw 각도 유지를 꺼도 rate 감쇠(target 0)는 항상 돈다.* 모터 토크 불균형과
롤·피치 보정의 반작용으로 생기는 자유 회전이 그렇게 억제된다.

단일 PID에서는 모든 항이 각도 추정을 거치므로, 추정이 나빠지면 감쇠까지 함께
나빠진다.

== rate 명령 인터페이스가 공짜로 생긴다

#src("dual_imu_cascade_pwm.ino:656, :1541-1554, yaw_command.h")

내부 루프의 입력이 "목표 각속도"이므로, 조종 명령을 각속도로 주는 모드(acro
계열)가 구조상 이미 존재한다. 이 펌웨어의 yaw가 정확히 그렇다 --- 스틱은
`targetYawRate`(dps)를 주고, 스틱을 놓으면 그 시점 heading이 자동으로 잠긴다.
각도 명령과 각속도 명령을 같은 파이프라인에서 섞을 수 있다.

== D가 setpoint에 오염되지 않는다

#src("dual_imu_cascade_pwm.ino:1564-1567")

미분을 자이로에만 걸기 때문에 각도 명령을 스텝으로 넣어도 미분항이 튀지 않는다
(derivative kick 없음). 이것도 외부 게인을 공격적으로 쓸 수 있게 해주는 조건이다.
추가로 미분 입력이 각가속도이므로, ESC·모터의 1차 지연이나 프레임 flex 같은
비강체 요소에 대해 위상 앞섬을 만들어 *각속도 루프 자체의 공진*을 억제한다.
(이상적인 강체 모델만 보면 이 항은 유효 관성을 늘리는 쪽이라 --- $(J + K_d)
dot.double(theta) + dots$ --- 이득은 비강체 요소가 있을 때 나타난다. 값도
0.015로 보수적이다.)

= 이점이 실제 게인에 어떻게 반영됐나

#block(breakable: false, table(
  columns: (auto, 1fr, 1fr),
  align: (left, center, center),
  table.header([], [단일 PID (레거시)], [캐스케이드 (현재)]),

  [각도 루프],
  [$K_p$ 2.5 / $K_i$ 0.005 / $K_d$ 1.2 \ (한 덩어리, 1 kHz)],
  [$K_(p a)$ = 6.0 \ (P만, 250 Hz)],
  [각속도 루프], [없음], [0.50 / 0.05 / 0.015 \ (1 kHz)],
  table.hline(stroke: 0.4pt),
  [등가 각도 강성 $K_theta$], [2.5 µs/deg], [$6.0 times 0.5 =$ *3.0* µs/deg],
  [등가 감쇠 $K_omega$], [1.2 µs/(deg/s)], [*0.50* µs/(deg/s)],
  table.hline(stroke: 0.4pt),
  [적분기 최대 기여],
  [$15 times 0.005 =$ *0.075 µs*],
  [*50 µs* (`I_TERM_MAX_US`)],
  [내부 setpoint 제한], [없음], [±300 dps],
  [D의 입력], [자이로 (1차)], [자이로 미분 (각가속도), 40 Hz LPF],
))

#v(-0.3em)
#align(right)[#src(
  "legacy :18-20, :336-343  /  cascade :139-151, :391, :1550, :1565",
)]

포화·windup 걱정이 사라졌으므로 *각도 강성을 올리고 감쇠를 줄이는* 방향으로
갈 수 있었다. 같은 기체(관성 $J$ 동일)로 2차 모델을 세우면

$
  J dot.double(theta) + K_omega dot(theta) + K_theta theta = K_theta theta_"cmd",
  quad omega_n = sqrt(K_theta \/ J), quad zeta = K_omega / (2 sqrt(K_theta J))
$

$J$ 를 몰라도 비율은 나온다: 고유진동수 $omega_n$ 은
$sqrt(3.0\/2.5) = 1.10$배, 감쇠비 $zeta$ 는 $(0.5\/1.2) sqrt(2.5\/3.0) = 0.38$배.
*과감쇠에 가깝던 응답이 훨씬 덜 감쇠된 응답으로 바뀌었다* --- 반응이 빠릿해진
체감은 거의 전부 여기서 나온다.

= 오해하기 쉬운 두 가지

#warn(title: "① \"캐스케이드라서 구조적으로 빠르다\"는 절반만 맞다")[
  순수 PD 단일루프는 P--P 캐스케이드와 *수학적으로 동치*다.
  단일루프 #src("dual_imu_pid_pwm.ino:341") 는
  $ u = K_p (theta_"cmd" - theta) - K_d omega $
  이고, 캐스케이드에서 I·D를 빼고 P 항만 보면 #src("cascade :1549,1570")
  $
    u = K_(p r) (K_(p a) (theta_"cmd" - theta) - omega)
    = underbrace(K_(p a) K_(p r), K_theta) (theta_"cmd" - theta)
    - underbrace(K_(p r), K_omega) omega
  $
  --- 같은 식이다. 따라서 응답 속도 자체는 구조가 아니라 *값*이 만든다.
  캐스케이드가 준 것은 속도가 아니라 *그 값을 안전하게 쓸 자격*(2.1\~2.3)과
  *그 값에 도달하는 방법*(2.4)이다.
]

#v(0.4em)

#warn(title: "② 250 Hz / 1 kHz 분리는 속도를 위한 장치가 아니다")[
  `OUTER_DIV = 4` #src("cascade :387, :1548-1557") 는 오히려 각도 setpoint를
  최대 4 ms까지 stale하게 만든다. 목적은 상보필터 출력의 노이즈가 rate
  setpoint로 새는 것을 줄이는 것이다. 자세 추정은 $alpha$ = 0.999--0.9998
  (시상수 1--5초)로 강하게 필터링돼 있고 #src("cascade :1115"), 4 ms는 내부 루프
  시상수 대비 무시할 수준이라 손해가 드러나지 않을 뿐이다.
]

= 정리

#note[
  #set enum(numbering: "1.", spacing: 0.8em)
  + *캐스케이드의 본질은 각속도가 손댈 수 있는 신호가 되는 것.* 자르고, 적분
    붙이고, 명령으로 주고, 따로 튜닝한다.
  + *직접적 이점*: 포화·windup 관리(2.1\~2.3), 단계적 튜닝·시험(2.4), 자세 추정
    고장에 대한 내성(2.5), rate 명령 인터페이스(2.6), derivative kick 제거(2.7).
  + *간접적 결과*: 그래서 강성 2.5→3.0, 감쇠 1.2→0.5라는 공격적 튜닝이
    가능해졌고, 반응성은 거기서 나온다($zeta$ 0.38배).
  + *대가는 안정 여유*다. 감쇠비를 판 것이므로 오버슈트·진동 여유는 줄었다.
    최종 게인은 테더 벤치에서 확정한다 #src("cascade :147-148").
]

#v(0.8em)
#align(center)[
  #text(size: 0.85em, fill: rgb("#888"))[
    수치의 정답은 항상 레포다. 이 문서의 모든 상수는 위 파일·라인에서 읽은 값이며,
    게인이 바뀌면 계산도 다시 해야 한다.
  ]
]
