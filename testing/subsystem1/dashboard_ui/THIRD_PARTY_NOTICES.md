# Third-party files in the S1 dashboard

Every file here is served by `s1_dashboard.py` from its fixed list
(`STATIC_FILES`). Nothing is loaded from the internet.

| File | What | Version | Source | SHA-256 | Licence |
|---|---|---|---|---|---|
| `fonts/inter-latin-opsz-normal.woff2` | Inter, variable font (weight and optical size axes), Latin subset, 72920 bytes | Fontsource `@fontsource-variable/inter` 5.2.8 (published 2025-09-17) | `https://cdn.jsdelivr.net/npm/@fontsource-variable/inter@5.2.8/files/inter-latin-opsz-normal.woff2` | `2c295d99e26dcf357d4d01bcf270fd6924b600c9a13dd8c363ef114f4c6976fa` | SIL Open Font License 1.1 |
| `fonts/OFL.txt` | The licence text of Inter, copied unchanged from the same package (`LICENSE`), 4477 bytes | as above | `https://cdn.jsdelivr.net/npm/@fontsource-variable/inter@5.2.8/LICENSE` | `3b0a5fca3d17942cde889069889dedbbbd075e9b599968c82a95f4d944e9b345` | SIL Open Font License 1.1 |

Inter is copyright 2016 The Inter Project Authors
(`https://github.com/rsms/inter`). Downloaded on 2026-11-14.

## How to update a file

1. Pick one pinned release that is at least two weeks old.
2. Download the file and the licence from that release only.
3. Replace the file, then update its row here: version, source URL and
   SHA-256 (`shasum -a 256 <file>`).
4. If the file name changes, change its entry in `STATIC_FILES` in
   `testing/subsystem1/s1_dashboard.py` and the `@font-face` rule in
   `tokens.css`.
5. Run `python3 -I -B testing/subsystem1/s1_dashboard.py --check`.
