// Runs the YuvRing output shader (exact GLSL source from YuvRing.kt) in headless Chromium's
// WebGL2 on a synthetic rolling-shutter frame and checks that it restores the ideal
// (global shutter) image. Usage: NODE_PATH=$(npm root -g) node tools/shader_check/rolling_shutter.mjs
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
const require = createRequire(import.meta.url);
const { chromium } = require("playwright");

const kt = readFileSync(new URL("../../app/src/main/java/com/opentrack/camera/gl/YuvRing.kt", import.meta.url), "utf8");
const shader = (name) => kt.split(`const val ${name} = """`)[1].split('"""')[0];
const VERTEX = shader("VERTEX");
const FRAGMENT = shader("FRAGMENT");

async function page_fn({ VERTEX, FRAGMENT }) {
  // ---- scene & motion (source px, as in RollingShutterTest) ----
  const W = 3840, H = 2160, cx = W / 2, cy = H / 2;
  const skew = 16e-3, t = 2.0;
  const pose = (s) => ({
    ox: 1120 * s + 40 * Math.sin(2 * Math.PI * 9 * s), oy: -300 * s + 25 * Math.cos(2 * Math.PI * 7 * s),
    r: 0.3 * s + 0.004 * Math.sin(2 * Math.PI * 5 * s),
  });
  const toWorld = (P, x, y) => {
    const dx = x - cx - P.ox, dy = y - cy - P.oy, c = Math.cos(-P.r), s = Math.sin(-P.r);
    return [cx + c * dx - s * dy, cy + s * dx + c * dy];
  };
  // Smooth pattern with ~60 px features, so a 10 px error is obvious and bilinear error is not.
  const scene = (wx, wy) => 0.5 + 0.22 * Math.sin(wx * 0.105) * Math.cos(wy * 0.095) + 0.18 * Math.sin((wx + 2 * wy) * 0.033);
  const P0 = pose(t);
  const rawAt = (x, y) => scene(...toWorld(pose(t + skew * (y / H - 0.5)), x, y)); // row y read at its own time
  const idealAt = (x, y) => scene(...toWorld(P0, x, y));

  // ---- corrections (RollingShutter.corrections) ----
  const N = 9, rs = new Float32Array(N * 3);
  for (let k = 0; k < N; k++) {
    const f = -0.5 + k / (N - 1), P = pose(t + skew * f), dr = P.r - P0.r, c = Math.cos(dr), s = Math.sin(dr);
    rs[k * 3] = P.ox - (c * P0.ox - s * P0.oy);
    rs[k * 3 + 1] = P.oy - (s * P0.ox + c * P0.oy);
    rs[k * 3 + 2] = dr;
  }

  // ---- ring slot: capture window stored at reduced resolution (raw frame) ----
  const cap = { left: 700, top: 400, width: 2400, height: 1350 };
  const SW = 1200, SH = 676;
  const y8 = new Uint8Array(SW * SH), uv = new Uint8Array((SW / 2) * (SH / 2) * 2).fill(128);
  for (let j = 0; j < SH; j++) for (let i = 0; i < SW; i++) {
    const x = cap.left + (i + 0.5) / SW * cap.width, y = cap.top + (j + 0.5) / SH * cap.height;
    y8[j * SW + i] = Math.round(255 * rawAt(x, y));
  }
  // ---- crop: rotated, zoomed ----
  const crop = { cx: 1950, cy: 1060, w: 1600, h: 900, rot: 0.03 };
  const OW = 800, OH = 450;

  const canvas = document.createElement("canvas");
  canvas.width = OW; canvas.height = OH;
  const gl = canvas.getContext("webgl2");
  if (!gl) return { error: "no webgl2" };
  const sh = (type, src) => {
    const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s);
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
    return s;
  };
  const prog = gl.createProgram();
  gl.attachShader(prog, sh(gl.VERTEX_SHADER, VERTEX));
  gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, FRAGMENT));
  gl.linkProgram(prog);
  if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) return { error: gl.getProgramInfoLog(prog) };
  gl.useProgram(prog);
  const tex = (unit, w, h, internal, format, data) => {
    const t = gl.createTexture(); gl.activeTexture(gl.TEXTURE0 + unit); gl.bindTexture(gl.TEXTURE_2D, t);
    gl.pixelStorei(gl.UNPACK_ALIGNMENT, 1);
    gl.texImage2D(gl.TEXTURE_2D, 0, internal, w, h, 0, format, gl.UNSIGNED_BYTE, data);
    for (const [p, v] of [[gl.TEXTURE_MIN_FILTER, gl.LINEAR], [gl.TEXTURE_MAG_FILTER, gl.LINEAR], [gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE], [gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE]]) gl.texParameteri(gl.TEXTURE_2D, p, v);
  };
  tex(0, SW, SH, gl.R8, gl.RED, y8);
  tex(1, SW / 2, SH / 2, gl.RG8, gl.RG, uv);
  const U = (n) => gl.getUniformLocation(prog, n);
  gl.uniform1i(U("sY"), 0); gl.uniform1i(U("sUV"), 1);
  // YuvRing.draw: homography (row-major) from output-local source px to source px, uploaded
  // column-major. Flat crop, and the same with a perspective (keystone) term as PerspectiveView
  // produces for an off-axis virtual camera.
  const c = Math.cos(crop.rot), s = Math.sin(crop.rot);
  const flat = [c, -s, crop.cx, s, c, crop.cy, 0, 0, 1];
  const ka = 6e-5, kb = -3e-5; // flat · [[1,0,0],[0,1,0],[ka,kb,1]]
  const persp = [flat[0] + flat[2] * ka, flat[1] + flat[2] * kb, flat[2], flat[3] + flat[5] * ka, flat[4] + flat[5] * kb, flat[5], ka, kb, 1];
  const setH = (H) => gl.uniformMatrix3fv(U("uH"), false, new Float32Array([H[0], H[3], H[6], H[1], H[4], H[7], H[2], H[5], H[8]]));
  const mapH = (H, ux, uy) => { const w = H[6] * ux + H[7] * uy + H[8]; return [(H[0] * ux + H[1] * uy + H[2]) / w, (H[3] * ux + H[4] * uy + H[5]) / w]; };
  gl.uniform2f(U("uHalf"), crop.w / 2, crop.h / 2);
  gl.uniform3fv(U("uRs"), rs);
  gl.uniform2f(U("uCapOrigin"), cap.left, cap.top);
  gl.uniform2f(U("uCapSize"), cap.width, cap.height);
  gl.uniform2f(U("uCenter"), cx, cy);
  gl.uniform3f(U("uReadout"), 0, 1, H);
  const buf = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, buf);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 1, -1, -1, 1, 1, 1]), gl.STATIC_DRAW);
  const aPos = gl.getAttribLocation(prog, "aPos");
  gl.enableVertexAttribArray(aPos); gl.vertexAttribPointer(aPos, 2, gl.FLOAT, false, 0, 0);
  gl.viewport(0, 0, OW, OH);

  const render = (on, H) => {
    setH(H);
    gl.uniform1i(U("uRsOn"), on);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    const px = new Uint8Array(OW * OH * 4);
    gl.readPixels(0, 0, OW, OH, gl.RGBA, gl.UNSIGNED_BYTE, px);
    return px;
  };
  // Expected: the ideal frame through the same crop. Output pixel (i, j) from the top-left;
  // GL rows start at the bottom.
  const score = (px, H) => {
    let sum = 0, n = 0;
    for (let j = 8; j < OH - 8; j++) for (let i = 8; i < OW - 8; i++) {
      const ux = (i + 0.5) / OW - 0.5, uy = (j + 0.5) / OH - 0.5;
      const [x, y] = mapH(H, crop.w * ux, crop.h * uy);
      const got = px[((OH - 1 - j) * OW + i) * 4] / 255;
      sum += Math.abs(got - idealAt(x, y)); n++;
    }
    return sum / n;
  };
  const corrected = score(render(1, flat), flat), uncorrected = score(render(0, flat), flat);
  // Perspective view: still exact (and the projective divide really happens per pixel: an
  // affine interpolation of the corners would be off by tens of px in the middle).
  const perspective = score(render(1, persp), persp), perspectiveAsFlat = score(render(1, persp), flat);
  return { corrected, uncorrected, perspective, perspectiveAsFlat, renderer: gl.getParameter(gl.RENDERER) };
}

const browser = await chromium.launch({ executablePath: "/opt/pw-browsers/chromium", args: ["--use-gl=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"] }).catch(() => chromium.launch({ args: ["--use-angle=swiftshader", "--enable-unsafe-swiftshader"] }));
const page = await browser.newPage();
const r = await page.evaluate(page_fn, { VERTEX, FRAGMENT });
await browser.close();
console.log(JSON.stringify(r));
if (r.error) process.exit(2);
// Mean absolute luma error (0..1): corrected must be close to the resampling floor, and far
// better than leaving the rolling shutter distortion in.
const ok = r.corrected < 0.01 && r.corrected < r.uncorrected / 5 && r.perspective < 0.01 && r.perspective < r.perspectiveAsFlat / 5;
console.log(ok ? "PASS" : "FAIL");
process.exit(ok ? 0 : 1);
