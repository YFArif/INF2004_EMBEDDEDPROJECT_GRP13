# Third-party files in the S1 dashboard

Every file here is served by `s1_dashboard.py` from its fixed list
(`STATIC_FILES`). Nothing is loaded from the internet.

| File | What | Version | Source | SHA-256 | Licence |
|---|---|---|---|---|---|
| `fonts/inter-latin-opsz-normal.woff2` | Inter, variable font (weight and optical size axes), Latin subset, 72920 bytes | Fontsource `@fontsource-variable/inter` 5.2.8 (published 2025-09-17) | `https://cdn.jsdelivr.net/npm/@fontsource-variable/inter@5.2.8/files/inter-latin-opsz-normal.woff2` | `2c295d99e26dcf357d4d01bcf270fd6924b600c9a13dd8c363ef114f4c6976fa` | SIL Open Font License 1.1 |
| `fonts/OFL.txt` | The licence text of Inter, copied unchanged from the same package (`LICENSE`), 4477 bytes | as above | `https://cdn.jsdelivr.net/npm/@fontsource-variable/inter@5.2.8/LICENSE` | `3b0a5fca3d17942cde889069889dedbbbd075e9b599968c82a95f4d944e9b345` | SIL Open Font License 1.1 |
| `vendor/three/three.module.js` | three.js, the ES module build (WebGL renderer), 662772 bytes | three 0.186.1 (published 2026-09-24) | `https://cdn.jsdelivr.net/npm/three@0.186.1/build/three.module.js` | `9052042d676cb0fdc1ddfefe193053f34b7ac0513a616fdac4535d49987812ea` | MIT |
| `vendor/three/three.core.js` | three.js, the core file that the module build imports, 1458113 bytes | as above | `https://cdn.jsdelivr.net/npm/three@0.186.1/build/three.core.js` | `9edde002b066a9a05676a6127f67735b62baf399bdea529f2f7e31657da769e6` | MIT |
| `vendor/three/LICENSE.txt` | The licence text of three.js, copied unchanged from the same package (`LICENSE`), 1081 bytes | as above | `https://cdn.jsdelivr.net/npm/three@0.186.1/LICENSE` | `8b378ebe60e2fe500158cb0ac71cb5e8b7d92953c2abcc63a0eb90499653b5bc` | MIT |

Inter is copyright 2016 The Inter Project Authors
(`https://github.com/rsms/inter`). Downloaded on 2026-11-14.

three.js is copyright 2010-2026 three.js authors
(`https://github.com/mrdoob/three.js`). Downloaded on 2026-11-15. The
three files are the npm package's own: their SHA-256 values match the
package's file listing. The package for 0.186.1 holds no minified build
(`build/` has `three.module.js`, `three.core.js`, `three.cjs` and the
WebGPU and TSL files only), and the `.min.js` files that jsDelivr makes
on request are not the project's own, so the unminified build is used.

## How to update a file

1. Pick one pinned release that is at least two weeks old.
2. Download the file and the licence from that release only.
3. Replace the file, then update its row here: version, source URL and
   SHA-256 (`shasum -a 256 <file>`).
4. If the file name changes, change its entry in `STATIC_FILES` in
   `testing/subsystem1/s1_dashboard.py` and the `@font-face` rule in
   `tokens.css`.
5. Run `python3 -I -B testing/subsystem1/s1_dashboard.py --check`.
