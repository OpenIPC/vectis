# xterm.js for the vectis web terminal

The `-w` page draws its terminal with [xterm.js](https://github.com/xtermjs/xterm.js)
(MIT, see `LICENSE`). The files are built into `vectis` (`webterm_assets.h`), so the
browser needs no internet; `make WEBTERM=0` leaves them out.

| File           | From the npm package                | Change                          |
|----------------|-------------------------------------|---------------------------------|
| `xterm.js`     | `@xterm/xterm` 5.5.0, `lib/xterm.js` | last line (`sourceMappingURL`) removed |
| `xterm.css`    | `@xterm/xterm` 5.5.0, `css/xterm.css` | none                           |
| `addon-fit.js` | `@xterm/addon-fit` 0.10.0, `lib/addon-fit.js` | last line (`sourceMappingURL`) removed |

Package tarballs (registry.npmjs.org), SHA-256:

```
bd954fa721872170188cc5d7e83e88db3c83c9a18a4e8d24c2783d26491f59d2  xterm-5.5.0.tgz
917ac44972453d5eed52edc1e50260c76398ce48cf2290c2e60671102bba0b33  addon-fit-0.10.0.tgz
```

After changing a file here run `sh web/gen-assets.sh` and commit `webterm_assets.h`
with it; CI checks that the two agree. A new version also changes the asset paths in
`vectis.c` (`WT_ASSET_JS`, `WT_ASSET_CSS`).
