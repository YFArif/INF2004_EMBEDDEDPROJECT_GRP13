# S1 dashboard design tokens

The dashboard follows Apple's Human Interface Guidelines (HIG). Every
colour, type size, radius and timing is a named token in
`testing/subsystem1/dashboard_ui/tokens.css`. No other page file holds a
colour. This document lists each token, where its value comes from and
where it differs from Apple.

## 1. Sources

Retrieved on 2026-11-14, as the Markdown form of each page.

| Page | URL | Latest change log entry on that day |
|---|---|---|
| Color | `https://developer.apple.com/design/human-interface-guidelines/color` | 16 December 2025 (Liquid Glass guidance); system colour values of 9 June 2025 |
| Dark Mode | `https://developer.apple.com/design/human-interface-guidelines/dark-mode` | 6 August 2024 |
| Materials | `https://developer.apple.com/design/human-interface-guidelines/materials` | 9 September 2025 |
| Typography | `https://developer.apple.com/design/human-interface-guidelines/typography` | 16 December 2025 |

The neutral structure (backgrounds, labels, separators, fills) is from
the UIKit "UI element colors" documentation. Its values were given by
the owner in the task. They were **not** fetched and checked against
that documentation on 2026-11-14. The system colours and the type tables
below were read from the HIG pages and match the task's values.

The HIG says not to hard-code system colour values in a native app. A
web page has no system-colour API, so the values are kept as tokens in
one file with this record of their source and date.

Nothing from Apple is copied: no screens, no logos, no SF Symbols glyphs
and no SF font files.

## 2. How the four contexts are written

- `light-dark(A, B)`: A in the light theme, B in the dark theme.
- `@media (prefers-contrast: more)`: the increased-contrast values.
- The theme follows the system (`color-scheme: light dark`) unless the
  page sets `data-theme="light"` or `"dark"` on `<html>`.

`light-dark()` needs Chrome or Edge 123, Firefox 120 or Safari 17.5 (all
from 2024). An older browser shows the page without its colours.

## 3. Apple system colours

Columns: default light, default dark, increased contrast (IC) light, IC
dark. Source: HIG Color, "Specifications".

| Token prefix | Apple name | Light | Dark | IC light | IC dark |
|---|---|---|---|---|---|
| `--sys-blue-` | Blue | #0088FF | #0091FF | #1E6EF4 | #5CB8FF |
| `--sys-red-` | Red | #FF383C | #FF4245 | #E9152D | #FF6165 |
| `--sys-green-` | Green | #34C759 | #30D158 | #008932 | #4AD968 |
| `--sys-orange-` | Orange | #FF8D28 | #FF9230 | #C55300 | #FFA056 |
| `--sys-yellow-` | Yellow | #FFCC00 | #FFD600 | #A16A00 | #FEDF43 |
| `--sys-gray-` | Gray | #8E8E93 | #8E8E93 | #6C6C70 | #AEAEB2 |
| `--sys-gray2-` | Gray (2) | #AEAEB2 | #636366 | not used | not used |
| `--sys-gray3-` | Gray (3) | #C7C7CC | #48484A | not used | not used |
| `--sys-gray4-` | Gray (4) | #D1D1D6 | #3A3A3C | not used | not used |
| `--sys-gray5-` | Gray (5) | #E5E5EA | #2C2C2E | not used | not used |
| `--sys-gray6-` | Gray (6) | #F2F2F7 | #1C1C1E | not used | not used |

The token names end `-l`, `-d`, `-lc` and `-dc`.

## 4. Role tokens

| Token | Apple name or role | Light | Dark | With `prefers-contrast: more` | Deviation |
|---|---|---|---|---|---|
| `--bg-page` | systemGroupedBackground | #F2F2F7 | #000000 | same | none |
| `--bg-card` | secondarySystemGroupedBackground | #FFFFFF | #1C1C1E | same | none |
| `--bg-inset` | tertiarySystemGroupedBackground | #F2F2F7 | #2C2C2E | same | none |
| `--label` | label | #000000 | #FFFFFF | same | none |
| `--label-2` | secondaryLabel | rgba(60,60,67,.76) | rgba(235,235,245,.6) | .9 / .78 | 3: Apple's light alpha is .6 |
| `--label-3` | tertiaryLabel | rgba(60,60,67,.58) | rgba(235,235,245,.4) | .76 / .6 | 3: Apple's alpha is .3 |
| `--label-4` | quaternaryLabel | rgba(60,60,67,.18) | rgba(235,235,245,.16) | same | none; never used for text |
| `--separator` | separator | rgba(60,60,67,.29) | rgba(84,84,88,.6) | rgba(60,60,67,.6) / rgba(235,235,245,.5) | none by default |
| `--separator-opaque` | opaqueSeparator | #C6C6C8 | #38383A | #8E8E93 / #7C7C80 | none by default |
| `--fill-1` to `--fill-4` | system, secondary, tertiary, quaternary fill | rgba(120,120,128, .2 / .16 / .12 / .08) | rgba(120,120,128, .36 / .32 / .24 / .18) | same | none |
| `--accent-text` | Blue, as text and glyphs | IC light #1E6EF4 | default dark #0091FF | #1E6EF4 / #5CB8FF | light theme uses the IC value (section 5) |
| `--red-text` | Red, as text | #E9152D | #FF4245 | #E9152D / #FF6165 | as above |
| `--green-text` | Green, as text | #008932 | #30D158 | #008932 / #4AD968 | as above |
| `--orange-text` | Orange, as text | #C55300 | #FF9230 | #C55300 / #FFA056 | as above |
| `--yellow-text` | Yellow, as text (not used yet) | #A16A00 | #FFD600 | #A16A00 / #FEDF43 | as above |
| `--accent-deco`, `--red-deco`, `--green-deco`, `--orange-deco`, `--gray-deco` | the same colours as dots, bars, plot lines and borders | default light | default dark | IC light / IC dark | none |
| `--accent-fill` | Blue, as a button fill | #1E6EF4 | #0091FF | #1E6EF4 / #5CB8FF | none |
| `--on-accent` | the text on that fill | #FFFFFF | #000000 | same | 7 |
| `--stop-fill`, `--on-stop` | the STOP tile | #E9152D, #FFFFFF | the same | the same | 4 |
| `--focus-ring` | keyboard focus | #1E6EF4 | #0091FF | #1E6EF4 / #5CB8FF | none |
| `--glass-fill` | Liquid Glass approximation | rgba(255,255,255,.6) | rgba(44,44,46,.55) | solid #FFFFFF / #2C2C2E | 5 |
| `--glass-solid` | the same control without transparency | #FFFFFF | #2C2C2E | same | 5 |
| `--glass-border`, `--glass-highlight`, `--glass-shadow`, `--card-shadow`, `--scrim`, `--stop-shadow` | hairline, inner highlight and shadows | see `tokens.css` | see `tokens.css` | border stronger | 5 |

Roles, as the HIG asks:

- Blue is the one accent and the only interactive tint. There is no
  accent picker.
- Red is STOP, faults and refusals. Green is online and acknowledged.
  Orange is a warning such as an obstacle ahead. Yellow is defined and
  not used.
- Colour sits on the primary action and on status, not on several
  controls at once.
- Colour is never the only signal: every coloured state also has words
  or a glyph.

## 5. Contrast

Requirement: at least 4.5:1 for text (3:1 for text of 24 px or more and
for tertiary text, which carries only information nobody needs).

- In the light theme the text tokens use Apple's increased-contrast
  values. The default light values are too weak as text on white: Blue
  3.52:1, Red 3.57, Green 2.22, Orange 2.31. The default light values
  are kept for decoration only.
- In the dark theme the default dark values pass on the card surface.
- Coloured text is placed on the card surface only. On the inset
  surface the dark Blue is 4.31:1 and the dark Red 4.06:1, so
  `app.css` keeps coloured text off it. A status in the header pill is
  shown in the label colour for the same reason.
- With `prefers-contrast: more` both themes use the increased-contrast
  set, and the secondary text, separators and glass get stronger.

The audit is `python3 -I -B testing/subsystem1/s1_dashboard.py
--token-audit`. It reads the `@pair` lines in `tokens.css`, resolves
each token in the four contexts, lays translucent colours over the
surface under them and computes the WCAG 2 ratio. It fails when:

1. a pair is below its minimum in any context;
2. `app.css`, `app.js` or `index.html` holds a colour literal;
3. a `color:` in `app.css` is not a token that is the text side of a
   pair.

Result on 2026-11-14: 28 pairs in 4 contexts, 0 below their minimum.
The lowest ratios are the coloured text on white in the light theme
(4.54 to 4.59), the STOP tile (4.56) and the secondary text on a
neutral fill over the page (4.60).

Not covered by the audit: which surface a coloured text really sits on
(rule 3 checks the token, not the place), and text drawn over the 3D
scene, which does not exist yet.

The audit is not part of `--check` yet. It joins `--check` as a step of
S1-U26 part A in stage 4.

## 6. Type

Font stack: `-apple-system, BlinkMacSystemFont, "Inter", system-ui,
sans-serif`. The first two keywords give SF Pro on Apple devices. SF
Pro's licence does not allow bundling it or linking it from a web page.
Inter 5.2.8 (SIL OFL 1.1, variable, Latin subset) is self-hosted as the
fallback for Windows, which is the demo laptop. See
`testing/subsystem1/dashboard_ui/THIRD_PARTY_NOTICES.md`.

Weights used: 400, 500, 600 and 700 only. Numbers use tabular figures.
`font-optical-sizing: auto`.

Scale: the iOS text styles at the default Large size (HIG Typography),
in px.

| Token | Apple style | Size / line | Used for |
|---|---|---|---|
| `--fs-display` | none (own) | 48 / 52 | the STOP word, bold |
| `--fs-large-title` | Large Title | 34 / 41 | the state word in the Live view |
| `--fs-title1` | Title 1 | 28 / 34 | numbers, semibold |
| `--fs-title2` | Title 2 | 22 / 28 | page title, map letters |
| `--fs-title3` | Title 3 | 20 / 25 | state words in cards |
| `--fs-headline` | Headline | 17 / 22 | card titles, START and RESET |
| `--fs-body` | Body | 17 / 22 | defined, not used yet |
| `--fs-callout` | Callout | 16 / 21 | Set map |
| `--fs-subhead` | Subhead | 15 / 20 | body text of the page, rows |
| `--fs-footnote` | Footnote | 13 / 18 | labels and notes |
| `--fs-caption1` | Caption 1 | 12 / 16 | ages, table headers; nothing is smaller |

Tracking: SF adjusts its own on Apple platforms, so the `--tr-*` tokens
are `normal` there. When Inter is in use they take Apple's SF Pro
tracking values (HIG Typography, "Tracking values"), in thousandths of
an em: 12 px 0, 13 px -6, 15 px -16, 16 px -20, 17 px -26, 20 px -23,
22 px -12, 28 px +14, 34 px +12, 48 px +8.

How the page knows which font is in use (`fontPath()` in `app.js`): at
load it measures a test string in `monospace`, then in `-apple-system,
BlinkMacSystemFont, monospace`. The same width means the keywords did
not resolve, so the page is on Inter and sets `font-inter` on `<html>`.
Otherwise it sets `font-system`. If the Inter file then fails to load,
the page falls to `system-ui` with Inter's tracking; that case is not
detected.

## 7. Materials

HIG Materials: Liquid Glass is for the controls and navigation layer
that floats above content, used sparingly, and "Don't use Liquid Glass
in the content layer."

- The content cards and the Developer drawer are solid grouped
  surfaces: no blur and no gradient on the page.
- Only the floating controls use `.glass`: the header status pill, the
  Developer button and the appearance switch. The chips over the 3D
  view will join them in stage 2.
- `.glass` is `backdrop-filter: blur(24px) saturate(1.8)`, a
  translucent neutral fill, a hairline border, a faint inner top
  highlight and a soft shadow.
- CSS cannot do the real lensing of Liquid Glass. Nothing fakes
  refraction.
- A browser without `backdrop-filter`, `prefers-reduced-transparency:
  reduce` and `prefers-contrast: more` all give a solid fill.

## 8. Shape, layout and motion

- 8 px spacing grid; page margins 16 to 24 px; minimum target 44 px.
- Cards: 24 px radius. Controls of 44 px or taller are capsules. Small
  elements: 12 px.
- Concentric corners: an element near a card corner gets the card
  radius minus the padding, but never less than 12 px. With 20 px of
  padding that is 12 px, which is the STOP tile's radius.
- `corner-shape: squircle` is applied inside `@supports` to cards, the
  STOP tile and the group mark. `border-radius` is the fallback. It was
  not checked in a browser that supports it.
- Hairline separators, not boxes.
- Icons are our own stroke drawings (round caps and joins, 1.8 to 2 px
  stroke). No SF Symbols path is copied.
- Motion: 300 ms, `cubic-bezier(0.2, 0.8, 0.2, 1)`. Nothing animates
  under `prefers-reduced-motion: reduce`.

## 9. Deviations from the HIG

| # | Deviation | Reason |
|---|---|---|
| 1 | A manual Auto, Light and Dark switch, with Auto as the default. The HIG advises against an app-specific appearance setting. | The page is projected in a bright room from a Windows laptop whose system setting may not suit the room. |
| 2 | Inter as the Windows fallback. | SF Pro cannot be bundled or linked. |
| 3 | Secondary and tertiary text darker than Apple's labels (alpha .76 and .58 in the light theme against .6 and .3; .4 against .3 for dark tertiary). | Apple's secondary label is 3.44:1 on white and its tertiary 1.74:1. The page needs 4.5:1 and 3:1. |
| 4 | The STOP tile is #E9152D with white text in both themes. | Apple's default reds with white text are 3.6:1 and 3.4:1. This one is 4.56:1. |
| 5 | Liquid Glass is approximated. | CSS has no lensing. |
| 6 | The 3D scene's own materials and colours. | Not built yet (stage 2). Its floor and chips will use the tokens. |
| 7 | Black text on the accent fill in the dark theme. Apple uses white on its blue buttons. | White on #0091FF is 3.23:1. Black is 6.49:1. |
| 8 | The light theme uses the increased-contrast colour values for text and for the accent fill even without `prefers-contrast: more`. | The default light values fail 4.5:1 as text on white (section 5). |

Deviations 7 and 8 were not in the task's list. They follow from its
contrast rule.
