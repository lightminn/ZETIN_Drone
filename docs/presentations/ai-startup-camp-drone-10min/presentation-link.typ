#let deck-url = "https://lightminn.github.io/zetin-drone/10min/"
#let ink = rgb("#172033")
#let muted = rgb("#657087")
#let accent = rgb("#2357D9")
#let accent-dark = rgb("#163A99")
#let paper = rgb("#F5F7FB")

#set document(
  title: "자작 드론 비행 제어 개발 결과 - 링크",
)
#set page(
  width: 13.333in,
  height: 7.5in,
  margin: 0pt,
  fill: paper,
)
#set text(
  font: ("Pretendard", "Noto Sans CJK KR"),
  fill: ink,
  lang: "ko",
)
#set par(leading: 0.75em)

#place(top + left, dx: 0pt, dy: 0pt)[
  #rect(width: 100%, height: 10pt, fill: accent)
]

#place(top + right, dx: -62pt, dy: 62pt)[
  #circle(radius: 76pt, fill: accent.transparentize(92%))
]

#place(bottom + left, dx: 50pt, dy: -44pt)[
  #circle(radius: 44pt, fill: accent.transparentize(94%))
]

#align(center + horizon)[
  #block(width: 820pt)[
    #align(center)[
      #text(size: 15pt, weight: 650, tracking: 0.08em, fill: accent)[
        AI 창업 캠프
      ]

      #v(18pt)

      #text(size: 38pt, weight: 750)[자작 드론 비행 제어 개발 결과]

      #v(8pt)

      #text(size: 22pt, weight: 500, fill: muted)[10분 요약본 링크]

      #v(34pt)

      #link(deck-url)[
        #box(
          width: 440pt,
          inset: (x: 30pt, y: 19pt),
          fill: accent,
          radius: 12pt,
          stroke: (bottom: 3pt + accent-dark),
        )[
          #align(center)[
            #text(size: 20pt, weight: 700, fill: white)[요약본 열기 ↗]
          ]
        ]
      ]

      #v(20pt)

      #text(
        size: 12.5pt,
        fill: muted,
      )[버튼이 열리지 않으면 아래 링크를 눌러 주세요.]

      #v(6pt)

      #link(deck-url)[
        #underline(
          offset: 3pt,
          stroke: 0.7pt + accent,
          text(size: 13.5pt, weight: 600, fill: accent)[웹 발표자료 바로가기],
        )
      ]

      #v(34pt)

      #grid(
        columns: (1fr, 1fr, 1fr),
        gutter: 12pt,
        column-gutter: 12pt,
        ..(
          [Chrome 권장],
          [← → 슬라이드 이동],
          [영상 자동 재생],
        ).map(item => box(
          width: 100%,
          inset: (x: 16pt, y: 11pt),
          fill: white,
          stroke: 0.8pt + rgb("#DCE2EE"),
          radius: 8pt,
          align(center, text(size: 12.5pt, weight: 600, fill: muted, item)),
        )),
      )
    ]
  ]
]
