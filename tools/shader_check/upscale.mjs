// Measures the output resampling (ring store + output draw, the exact GLSL from FrameRenderer.kt
// and YuvRing.kt) in headless Chromium's WebGL2 on real 4K frames, against the old bilinear path.
//
// Enlarging: the "sensor" is a 4K frame shrunk by u (1.5, 2, 3); the crop is rendered back at
// the 4K frame's own resolution and compared with it (the detail a sensor u times finer had).
// Shrinking: a 2560x1440 crop of the 4K frame rendered at 1600x900, compared with an area
// (box) filtered version of it (aliasing shows up as error).
//
//   python3 tools/shader_check/upscale_frames.py DIR      # test frames (needs OpenCV)
//   NODE_PATH=$(npm root -g) node tools/shader_check/upscale.mjs DIR [k1,k2,...]
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
const require = createRequire(import.meta.url);
const { chromium } = require("playwright");

const dir = process.argv[2];
const sharpens = (process.argv[3] || "0,0.25,0.5,0.75,1").split(",").map(Number);
const src = (f) => readFileSync(new URL(`../../app/src/main/java/com/opentrack/camera/gl/${f}`, import.meta.url), "utf8");
const block = (kt, name) => kt.split(`const val ${name} = `)[1].split('"""')[1];
const fr = src("FrameRenderer.kt");
const ring = src("YuvRing.kt");
// External OES texture → plain 2D texture for WebGL.
const oes = (s) => s.replace("#extension GL_OES_EGL_image_external : require", "").replace("samplerExternalOES", "sampler2D");
const STORE_VERT = block(fr, "VERTEX");
const STORE_NEW = oes(block(fr, "COMMON") + block(fr, "PLANE_Y_FRAGMENT"));
// The old store: one bilinear sample per stored pixel.
const STORE_OLD = oes(block(fr, "COMMON") + `
  void main() { gl_FragColor = vec4(dot(texture2D(sTex, imgToTex(vImg)).rgb, vec3(0.299, 0.587, 0.114)), 0.0, 0.0, 1.0); }`);
const DRAW_VERT = block(ring, "VERTEX");
const DRAW_NEW = block(ring, "FRAGMENT");
const DRAW_OLD = DRAW_NEW.replace("float y = lumaBicubic(at);", "float y = texture2D(sY, at).r;");
if (DRAW_OLD === DRAW_NEW || !STORE_NEW.includes("bicubic")) throw new Error("shader sources changed: update the harness");

const meta = JSON.parse(readFileSync(`${dir}/meta.json`, "utf8"));
const b64 = (f) => readFileSync(`${dir}/${f}`).toString("base64");

async function run(page, test) {
  return page.evaluate(async ({ test, shaders, sharpens }) => {
    const dec = (s) => Uint8Array.from(atob(s), (c) => c.charCodeAt(0));
    const S = dec(test.src), truth = dec(test.truth);
    const OW = 1600, OH = 900, SW = 2000, SH = 1124; // ring slot: output x 1.25 (even)
    const canvas = document.createElement("canvas");
    const gl = canvas.getContext("webgl2");
    const prog = (v, f) => {
      const p = gl.createProgram();
      for (const [t, s] of [[gl.VERTEX_SHADER, v], [gl.FRAGMENT_SHADER, f]]) {
        const sh = gl.createShader(t); gl.shaderSource(sh, s); gl.compileShader(sh);
        if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(sh));
        gl.attachShader(p, sh);
      }
      gl.bindAttribLocation(p, 0, "aPos"); gl.linkProgram(p); return p;
    };
    const tex = (w, h, internal, format, data) => {
      const t = gl.createTexture(); gl.bindTexture(gl.TEXTURE_2D, t);
      gl.pixelStorei(gl.UNPACK_ALIGNMENT, 1);
      gl.texImage2D(gl.TEXTURE_2D, 0, internal, w, h, 0, format, gl.UNSIGNED_BYTE, data);
      for (const [k, v] of [[gl.TEXTURE_MIN_FILTER, gl.LINEAR], [gl.TEXTURE_MAG_FILTER, gl.LINEAR], [gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE], [gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE]]) gl.texParameteri(gl.TEXTURE_2D, k, v);
      return t;
    };
    // The "camera" texture (RGB = luma), row 0 = image top.
    const rgb = new Uint8Array(test.sw * test.sh * 4);
    for (let i = 0; i < S.length; i++) { rgb[4 * i] = rgb[4 * i + 1] = rgb[4 * i + 2] = S[i]; rgb[4 * i + 3] = 255; }
    const camTex = tex(test.sw, test.sh, gl.RGBA, gl.RGBA, rgb);
    const quad = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, quad);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 1, -1, -1, 1, 1, 1]), gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0); gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0);
    const fbo = (t) => { const f = gl.createFramebuffer(); gl.bindFramebuffer(gl.FRAMEBUFFER, f); gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, t, 0); return f; };
    const ringY = tex(SW, SH, gl.R8, gl.RED, null), ringFbo = fbo(ringY);
    const uv = tex(SW / 2, SH / 2, gl.RG8, gl.RG, new Uint8Array(SW * SH / 2).fill(128));
    const outT = tex(OW, OH, gl.RGBA, gl.RGBA, null), outFbo = fbo(outT);
    // Capture window: the crop x 1.25 (source px).
    const cap = { l: test.cx - test.cw * 0.625, t: test.cy - test.ch * 0.625, w: test.cw * 1.25, h: test.ch * 1.25 };

    const store = (src) => {
      const p = prog(shaders.STORE_VERT, src); gl.useProgram(p);
      const U = (n) => gl.getUniformLocation(p, n);
      gl.bindFramebuffer(gl.FRAMEBUFFER, ringFbo); gl.viewport(0, 0, SW, SH);
      gl.activeTexture(gl.TEXTURE0); gl.bindTexture(gl.TEXTURE_2D, camTex); gl.uniform1i(U("sTex"), 0);
      gl.uniform4f(U("uCrop"), cap.l / test.sw, cap.t / test.sh, cap.w / test.sw, cap.h / test.sh);
      gl.uniform1f(U("uFlipY"), -1);
      // imgToTex: identity apart from the y flip (row 0 = image top).
      gl.uniformMatrix4fv(U("uTexMatrix"), false, new Float32Array([1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1]));
      gl.uniform1i(U("uRotation"), 0);
      gl.uniform2f(U("uPx"), cap.w / test.sw / SW, cap.h / test.sh / SH);
      gl.uniform2f(U("uSrc"), 1 / test.sw, 1 / test.sh);
      gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    };
    const draw = (src, k) => {
      const p = prog(shaders.DRAW_VERT, src); gl.useProgram(p);
      const U = (n) => gl.getUniformLocation(p, n);
      gl.bindFramebuffer(gl.FRAMEBUFFER, outFbo); gl.viewport(0, 0, OW, OH);
      gl.activeTexture(gl.TEXTURE0); gl.bindTexture(gl.TEXTURE_2D, ringY); gl.uniform1i(U("sY"), 0);
      gl.activeTexture(gl.TEXTURE1); gl.bindTexture(gl.TEXTURE_2D, uv); gl.uniform1i(U("sUV"), 1);
      gl.uniformMatrix3fv(U("uH"), false, new Float32Array([1, 0, 0, 0, 1, 0, test.cx, test.cy, 1]));
      gl.uniform2f(U("uHalf"), test.cw / 2, test.ch / 2);
      gl.uniform2f(U("uCapOrigin"), cap.l, cap.t); gl.uniform2f(U("uCapSize"), cap.w, cap.h);
      gl.uniform1i(U("uRsOn"), 0);
      gl.uniform2f(U("uTexel"), 1 / SW, 1 / SH); gl.uniform1f(U("uSharp"), k);
      gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
      const px = new Uint8Array(OW * OH * 4); gl.readPixels(0, 0, OW, OH, gl.RGBA, gl.UNSIGNED_BYTE, px);
      const y = new Float64Array(OW * OH); // flip rows: framebuffer row 0 is the bottom
      for (let j = 0; j < OH; j++) for (let i = 0; i < OW; i++) y[j * OW + i] = px[((OH - 1 - j) * OW + i) * 4];
      return y;
    };
    const metrics = (y) => {
      let se = 0, n = 0, go = 0, gt = 0;
      for (let j = 8; j < OH - 8; j++) for (let i = 8; i < OW - 8; i++) {
        const k = j * OW + i, d = y[k] - truth[k];
        se += d * d; n++;
        go += Math.abs(y[k + 1] - y[k]) + Math.abs(y[k + OW] - y[k]);
        gt += Math.abs(truth[k + 1] - truth[k]) + Math.abs(truth[k + OW] - truth[k]);
      }
      return { psnr: +(10 * Math.log10(255 * 255 / (se / n))).toFixed(2), sharp: +(go / gt).toFixed(3) };
    };
    const out = {};
    store(shaders.STORE_OLD); out.bilinear = metrics(draw(shaders.DRAW_OLD, 0));
    store(shaders.STORE_NEW);
    for (const k of sharpens) out[`new k=${k}`] = metrics(draw(shaders.DRAW_NEW, k));
    return out;
  }, { test, shaders: { STORE_VERT, STORE_OLD, STORE_NEW, DRAW_VERT, DRAW_OLD, DRAW_NEW }, sharpens });
}

const browser = await chromium.launch();
const page = await browser.newPage();
const rows = [];
for (const m of meta) {
  const truthUp = b64(`${m.name}_TC.raw`);
  for (const u of [1.5, 2, 3]) {
    const sw = Math.round(3840 / u), sh = Math.round(2160 / u);
    const r = await run(page, { src: b64(`${m.name}_S${u}.raw`), sw, sh, truth: truthUp, cx: m.cx / u, cy: m.cy / u, cw: 1600 / u, ch: 900 / u });
    rows.push({ frame: m.name, case: `enlarge x${u}`, ...Object.fromEntries(Object.entries(r).map(([k, v]) => [k, `${v.psnr} dB / ${v.sharp}`])) });
  }
  const r = await run(page, { src: b64(`${m.name}_T.raw`), sw: 3840, sh: 2160, truth: b64(`${m.name}_D.raw`), cx: m.dcx, cy: m.dcy, cw: 2560, ch: 1440 });
  rows.push({ frame: m.name, case: "shrink x0.625", ...Object.fromEntries(Object.entries(r).map(([k, v]) => [k, `${v.psnr} dB / ${v.sharp}`])) });
}
await browser.close();
console.log("PSNR vs truth / sharpness (mean gradient vs truth, 1 = as sharp as the truth)");
console.table(rows);
