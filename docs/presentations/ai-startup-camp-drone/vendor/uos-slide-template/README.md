# UOS Slide Design System — how to build with it

This is a **presentation-slide** system, ported coordinate-for-coordinate from the University of
Seoul's official PowerPoint template (`UOS_PTformat_B`, 16:9). Everything you build with it is a
deck: a sequence of fixed 1280×720 slides, not a scrolling web page.

## The one hard rule: every screen is a `Slide`

There is **no provider and no theme context**. Tokens live in `:root` in the stylesheet, so
components are styled the moment `styles.css` is loaded. What you must not skip is the frame:

- **Always render a `Slide` (or a layout component, which contains one) as the root of each screen.**
  It establishes the 1280×720 canvas and auto-scales it to the parent's width via `ResizeObserver`.
- **Inside a slide, position with absolute pixel coordinates on the 1280×720 canvas** — not with
  page-flow layout. `left: 41.12` means 41.12px on the canvas at any rendered size.
- Do **not** put a `Slide` inside a scrolling container that constrains its height; give it a
  parent whose width you control and let the 16:9 aspect ratio drive the height.
- For a multi-slide deck, stack slides in a plain column with a gap. Do not nest slides.

The four layout components already contain their `Slide`, so use them directly:

| Component | Use for |
|---|---|
| `TitleSlide` | cover — diagonal blue panel, title, date badge, white knockout logo |
| `TocSlide` | table of contents / agenda |
| `ChapterSlide` | section divider |
| `ContentSlide` | every ordinary body slide — blue section title + content slot |

Reach for bare `Slide` only when none of those fit; then compose `AccentTab`, `DiagonalPanel`, and
`UosLogo` yourself at canvas coordinates.

## Styling idiom: CSS custom properties, not utility classes

There is **no utility-class vocabulary** (no Tailwind, no `p-4`, no `text-lg`) and the `uos-*`
classes are component internals — **never write them yourself**. For your own layout glue inside a
slide, use inline styles or your own classes, and pull every value from these tokens:

**Color** — `--uos-blue` (#004094, the brand color), `--uos-blue-deep`, `--uos-surface` (#ECEAE6,
the neutral content/figure fill), `--uos-ink`, `--uos-ink-muted`, `--uos-on-blue`, `--uos-rule`

**Type** — `--uos-font` (Noto Sans CJK KR, bundled; handles Korean and Latin),
`--uos-fs-title` (58.667px = 44pt), `--uos-fs-chapter` (32px = 24pt),
`--uos-fs-body` (20px = 15pt), `--uos-fs-caption` (18.667px = 14pt)

**Canvas & spacing** — `--uos-slide-w` / `--uos-slide-h` (1280 / 720px), `--uos-gutter` (41.12px,
the body left/right margin), `--uos-space-1` … `--uos-space-4`, `--uos-radius`

**Slot coordinates** — the template's own measurements are exposed as tokens so custom slides can
line up with the stock layouts: `--uos-content-left` / `--uos-content-top` / `--uos-content-width` /
`--uos-content-height` (the body content box), `--uos-content-title-left` / `--uos-content-title-top`
(the blue section title), `--uos-toc-body-left`, `--uos-title-text-left` / `--uos-title-text-top`,
`--uos-title-logo-left` / `--uos-title-logo-top` / `--uos-title-logo-size`, `--uos-tab-lg-width` /
`--uos-tab-lg-height` / `--uos-tab-sm-width` / `--uos-tab-sm-height`.

Sizes derive from PowerPoint points at 96dpi: **px = pt × 4/3**, and **1 canvas px = 9525 EMU**.
Keep new type sizes on that scale rather than inventing arbitrary values.

## Content blocks

Put these **inside `ContentSlide`'s children**, never directly on a bare page:
`BulletList` (markers `dot` | `dash` | `number`, one nested level), `DataTable` (blue header,
alternating `--uos-surface` rows, per-column `align`), `StatCard` (`tone` `blue` | `neutral`),
`FigureFrame` (`ratio` defaults to `16/9`; renders the `--uos-surface` placeholder when empty).

## Brand rules that are easy to get wrong

- **On blue, the logo must be `<UosLogo variant="white" />`.** The default `color` variant is UOS
  blue ink and disappears against `--uos-blue`. `TitleSlide` already does this for you.
- `UosLogo` is aspect-locked 1:1 — set `size`, never a separate width and height.
- `AccentTab` and `DiagonalPanel` carry the template's exact clipped geometry
  (`--uos-clip-tab-lg`, `--uos-clip-tab-sm`, `--uos-clip-title`, `--uos-clip-date`). Do not
  re-cut those shapes with your own `clip-path` — the angles are part of the brand.
- Multi-line titles use `\n` inside the `title` string; the components honor the line break.

## Where the truth lives

Read `_ds/<folder>/styles.css` (and the `_ds_bundle.css` it imports) for the full token list and
component CSS, and `components/<group>/<Name>/<Name>.d.ts` + `<Name>.prompt.md` for each API.

## An idiomatic slide

```jsx
<ContentSlide title="실험 결과 요약">
  <div style={{ display: 'flex', gap: 'var(--uos-space-3)' }}>
    <StatCard label="분류 정확도" value="94.7%" caption="baseline 대비 +15.0%p" />
    <StatCard label="추론 지연" value="-42%" tone="neutral" />
  </div>
  <BulletList
    marker="dash"
    items={[
      '공간 어텐션 추가 시 MAE가 4.81 → 4.11로 감소',
      { text: '한계', children: ['야간 시간대 표본 부족', '수종별 편차 미반영'] },
    ]}
  />
</ContentSlide>
```

# UOSSlideDS (uos-slide-ds@0.1.0)

This design system is the published uos-slide-ds React library, bundled as a single
browser global. All 12 components are the real upstream code.

## Where things are

- `_ds_bundle.js` — the whole-DS bundle at the project root; loads every component to `window.UOSSlideDS`. First line is a `/* @ds-bundle: … */` metadata header.
- `styles.css` — the single stylesheet entry: it `@import`s the tokens, fonts, and component styles (`_ds_bundle.css`). Link this one file.
- `components/<group>/<Name>/<Name>.prompt.md` (example JSX + variants), `<Name>.d.ts` (types), `<Name>.html` (variant grid).
- `tokens/*.css` — CSS custom properties, names verbatim from upstream.
- `fonts/` — `@font-face` files + `fonts.css` (when the package ships fonts).

For a specific component, `read_file("components/<group>/<Name>/<Name>.prompt.md")`.

## Loading

Add these two lines to your page once (React must be on the page first):

```html
<link rel="stylesheet" href="styles.css">
<script src="_ds_bundle.js"></script>
```

Components are then available at `window.UOSSlideDS.*`. Mount into a dedicated child node (e.g. `<div id="ds-root">`), not the host page's own React root, so the two trees don't collide:

```jsx
const { AccentTab } = window.UOSSlideDS;
ReactDOM.createRoot(document.getElementById('ds-root')).render(<AccentTab />);
```

## Tokens

68 CSS custom properties from uos-slide-ds. Names are
preserved verbatim from upstream. They are declared inside `_ds_bundle.css` (this DS ships one compiled stylesheet rather than separate token files).

- **color** (7): `--uos-surface`, `--uos-title-text-left`, `--uos-title-text-top`, …
- **spacing** (4): `--uos-space-1`, `--uos-space-2`, `--uos-space-3`, …
- **typography** (1): `--uos-font`
- **radius** (1): `--uos-radius`
- **other** (55): `--uos-blue`, `--uos-blue-deep`, `--uos-ink`, …

## Components

### general
- `AccentTab` — Renders the original UOS clipped blue accent tab in large or small size.
- `BulletList` — Renders a UOS-styled list with dot, dash, or numbered markers and one nested level.
- `ChapterSlide` — Renders the UOS chapter-divider layout with the original large accent tab.
- `ContentSlide` — Renders the UOS content layout with a transparent or placeholder content slot.
- `DataTable` — Renders a UOS data table with a blue header and alternating surface rows.
- `DiagonalPanel` — Renders the title slide's coordinate-faithful diagonal blue panel.
- `FigureFrame` — Renders an aspect-ratio figure slot with an optional child and caption.
- `Slide` — Renders a responsive 1280720 slide frame that scales to its parent width.
- `StatCard` — Renders a compact statistic card in UOS blue or neutral styling.
- `TitleSlide` — Renders the UOS cover layout with diagonal panel, title, date badge, and logo.
- `TocSlide` — Renders the UOS table-of-contents layout with up to page-labelled entries.
- `UosLogo` — Renders the bundled University of Seoul logo at a fixed square aspect ratio.
