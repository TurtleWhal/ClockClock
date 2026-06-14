/**
 * ClockClock CV — detect hand angles from an 8×3 clock matrix image.
 *
 * Usage:
 *   <script src="clockclock-cv.js"></script>
 *   const result = detectClockAngles(imageElement);
 *
 *   // result.clocks — 3×8 array of clock objects:
 *   //   { angles: [angle1, angle2], center: {x, y}, cellBounds: {x, y, w, h} }
 *   //   angles are in degrees: 0° = 12 o'clock, 90° = 3 o'clock (clockwise)
 *   //
 *   // result.canvas — canvas with the warped image and detected hands drawn
 *   // result.corners — the 4 detected corner points [TL, TR, BR, BL]
 *   // result.threshold — the auto-computed brightness threshold
 *   // result.handWidth — the measured hand width in pixels
 *
 *   Options:
 *     detectClockAngles(img, { corners: [[x,y],[x,y],[x,y],[x,y]], threshold: 160 })
 */

// eslint-disable-next-line no-unused-vars
function detectClockAngles(img, options) {
  'use strict';

  const COLS = 8, ROWS = 3;
  const ANGLE_STEPS = 360;
  const opts = options || {};

  // ── Helpers ──────────────────────────────────────────────────

  function dist(a, b) { return Math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2); }

  function morphErode(bin, w, h) {
    const out = new Uint8Array(w * h);
    for (let y = 1; y < h - 1; y++)
      for (let x = 1; x < w - 1; x++)
        if (bin[(y-1)*w+x-1] && bin[(y-1)*w+x] && bin[(y-1)*w+x+1] &&
            bin[y*w+x-1] && bin[y*w+x] && bin[y*w+x+1] &&
            bin[(y+1)*w+x-1] && bin[(y+1)*w+x] && bin[(y+1)*w+x+1])
          out[y*w+x] = 1;
    return out;
  }

  function morphDilate(bin, w, h) {
    const out = new Uint8Array(w * h);
    for (let y = 1; y < h - 1; y++)
      for (let x = 1; x < w - 1; x++)
        if (bin[(y-1)*w+x-1] || bin[(y-1)*w+x] || bin[(y-1)*w+x+1] ||
            bin[y*w+x-1] || bin[y*w+x] || bin[y*w+x+1] ||
            bin[(y+1)*w+x-1] || bin[(y+1)*w+x] || bin[(y+1)*w+x+1])
          out[y*w+x] = 1;
    return out;
  }

  function connectedComponents(bin, w, h) {
    const labels = new Int32Array(w * h);
    const parent = [0];
    let next = 1;
    function find(x) { while (parent[x] !== x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
    function union(a, b) { a = find(a); b = find(b); if (a !== b) parent[Math.max(a, b)] = Math.min(a, b); }
    for (let y = 0; y < h; y++)
      for (let x = 0; x < w; x++) {
        if (!bin[y * w + x]) continue;
        const up = y > 0 ? labels[(y-1)*w+x] : 0;
        const left = x > 0 ? labels[y*w+x-1] : 0;
        if (up && left) { labels[y*w+x] = Math.min(up, left); union(up, left); }
        else if (up) labels[y*w+x] = up;
        else if (left) labels[y*w+x] = left;
        else { labels[y*w+x] = next; parent.push(next); next++; }
      }
    for (let i = 0; i < labels.length; i++) if (labels[i]) labels[i] = find(labels[i]);
    return { labels };
  }

  function fitLineYofX(pts) {
    const n = pts.length; if (n < 4) return null;
    let sx=0,sy=0,sxx=0,sxy=0;
    for (const [x,y] of pts){sx+=x;sy+=y;sxx+=x*x;sxy+=x*y;}
    const d=n*sxx-sx*sx; if(Math.abs(d)<1e-6)return null;
    return{m:(n*sxy-sx*sy)/d, b:(sy*sxx-sx*sxy)/d};
  }
  function fitLineXofY(pts) {
    const n = pts.length; if (n < 4) return null;
    let sx=0,sy=0,syy=0,sxy=0;
    for (const [x,y] of pts){sx+=x;sy+=y;syy+=y*y;sxy+=x*y;}
    const d=n*syy-sy*sy; if(Math.abs(d)<1e-6)return null;
    return{m:(n*sxy-sx*sy)/d, b:(sx*syy-sy*sxy)/d};
  }
  function robustFitLineYofX(pts) {
    if (pts.length < 4) return null;
    let line = fitLineYofX(pts); if (!line) return null;
    for (let iter = 0; iter < 2; iter++) {
      const res = pts.map(([x,y]) => Math.abs(y-(line.m*x+line.b)));
      const mn = res.reduce((a,b)=>a+b,0)/res.length;
      const sd = Math.sqrt(res.reduce((a,r)=>a+(r-mn)**2,0)/res.length)||1;
      const f = pts.filter((_,i) => res[i] < mn+2*sd);
      if (f.length < 4) break; line = fitLineYofX(f); if (!line) break; pts = f;
    }
    return line;
  }
  function robustFitLineXofY(pts) {
    if (pts.length < 4) return null;
    let line = fitLineXofY(pts); if (!line) return null;
    for (let iter = 0; iter < 2; iter++) {
      const res = pts.map(([x,y]) => Math.abs(x-(line.m*y+line.b)));
      const mn = res.reduce((a,b)=>a+b,0)/res.length;
      const sd = Math.sqrt(res.reduce((a,r)=>a+(r-mn)**2,0)/res.length)||1;
      const f = pts.filter((_,i) => res[i] < mn+2*sd);
      if (f.length < 4) break; line = fitLineXofY(f); if (!line) break; pts = f;
    }
    return line;
  }
  function intersectYX(yLine, xLine) {
    const d = 1 - yLine.m * xLine.m;
    const x = (xLine.m * yLine.b + xLine.b) / d;
    return [x, yLine.m * x + yLine.b];
  }

  // ── Adaptive thresholding helpers ──────────────────────────

  function otsuThreshold(gray, length) {
    const hist = new Int32Array(256);
    for (let i = 0; i < length; i++) hist[gray[i]]++;
    let sumAll = 0;
    for (let i = 0; i < 256; i++) sumAll += i * hist[i];
    let sumB = 0, wB = 0, maxVariance = 0, bestThresh = 45;
    for (let t = 0; t < 256; t++) {
      wB += hist[t];
      if (wB === 0) continue;
      const wF = length - wB;
      if (wF === 0) break;
      sumB += t * hist[t];
      const mB = sumB / wB;
      const mF = (sumAll - sumB) / wF;
      const variance = wB * wF * (mB - mF) * (mB - mF);
      if (variance > maxVariance) { maxVariance = variance; bestThresh = t; }
    }
    return bestThresh;
  }

  function buildIntegralImage(gray, w, h) {
    const integral = new Float64Array(w * h);
    for (let y = 0; y < h; y++) {
      let rowSum = 0;
      for (let x = 0; x < w; x++) {
        rowSum += gray[y * w + x];
        integral[y * w + x] = rowSum + (y > 0 ? integral[(y - 1) * w + x] : 0);
      }
    }
    return integral;
  }

  function adaptiveThresholdBinary(gray, w, h, blockSize, C) {
    const integral = buildIntegralImage(gray, w, h);
    const binary = new Uint8Array(w * h);
    const half = Math.floor(blockSize / 2);
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        // Skip obviously bright pixels — not part of clock body
        if (gray[y * w + x] > 150) continue;
        const x0 = Math.max(0, x - half), y0 = Math.max(0, y - half);
        const x1 = Math.min(w - 1, x + half), y1 = Math.min(h - 1, y + half);
        const area = (x1 - x0 + 1) * (y1 - y0 + 1);
        let sum = integral[y1 * w + x1];
        if (x0 > 0) sum -= integral[y1 * w + (x0 - 1)];
        if (y0 > 0) sum -= integral[(y0 - 1) * w + x1];
        if (x0 > 0 && y0 > 0) sum += integral[(y0 - 1) * w + (x0 - 1)];
        const mean = sum / area;
        if (gray[y * w + x] < mean - C) binary[y * w + x] = 1;
      }
    }
    return binary;
  }

  function computePCA(pts) {
    const n = pts.length;
    let mx = 0, my = 0;
    for (const [x, y] of pts) { mx += x; my += y; }
    mx /= n; my /= n;
    let cxx = 0, cyy = 0, cxy = 0;
    for (const [x, y] of pts) {
      const dx = x - mx, dy = y - my;
      cxx += dx * dx; cyy += dy * dy; cxy += dx * dy;
    }
    cxx /= n; cyy /= n; cxy /= n;
    const trace = cxx + cyy;
    const det = cxx * cyy - cxy * cxy;
    const disc = Math.sqrt(Math.max(0, trace * trace / 4 - det));
    const lambda1 = trace / 2 + disc;
    const lambda2 = Math.max(0.001, trace / 2 - disc);
    let v1x, v1y;
    if (Math.abs(cxy) > 1e-6) { v1x = lambda1 - cyy; v1y = cxy; }
    else { v1x = cxx >= cyy ? 1 : 0; v1y = cxx >= cyy ? 0 : 1; }
    const mag = Math.sqrt(v1x * v1x + v1y * v1y);
    v1x /= mag; v1y /= mag;
    return { center: [mx, my], lambda1, lambda2, v1: [v1x, v1y], v2: [-v1y, v1x] };
  }

  // ── Perspective transform ───────────────────────────────────

  function computeHomography(src, dst) {
    const A = [], b = [];
    for (let i = 0; i < 4; i++) {
      const [x,y]=src[i],[X,Y]=dst[i];
      A.push([x,y,1,0,0,0,-X*x,-X*y]); b.push(X);
      A.push([0,0,0,x,y,1,-Y*x,-Y*y]); b.push(Y);
    }
    const h = solveLinear8(A, b);
    return [[h[0],h[1],h[2]],[h[3],h[4],h[5]],[h[6],h[7],1]];
  }
  function solveLinear8(A, b) {
    const n=8, M=A.map((r,i)=>[...r,b[i]]);
    for (let col=0;col<n;col++){
      let mv=Math.abs(M[col][col]),mr=col;
      for (let r=col+1;r<n;r++) if(Math.abs(M[r][col])>mv){mv=Math.abs(M[r][col]);mr=r;}
      [M[col],M[mr]]=[M[mr],M[col]];
      const p=M[col][col]; for(let j=col;j<=n;j++)M[col][j]/=p;
      for(let r=0;r<n;r++){if(r===col)continue;const f=M[r][col];for(let j=col;j<=n;j++)M[r][j]-=f*M[col][j];}
    }
    return M.map(r=>r[n]);
  }
  function perspectiveWarp(imgEl, corners, outW, outH) {
    const dst=[[0,0],[outW,0],[outW,outH],[0,outH]];
    const H = computeHomography(dst, corners);
    const sc=document.createElement('canvas'); sc.width=imgEl.width; sc.height=imgEl.height;
    const sctx=sc.getContext('2d'); sctx.drawImage(imgEl,0,0);
    const srcD=sctx.getImageData(0,0,imgEl.width,imgEl.height).data;
    const oc=document.createElement('canvas'); oc.width=outW; oc.height=outH;
    const octx=oc.getContext('2d');
    const oi=octx.createImageData(outW,outH); const out=oi.data;
    for(let oy=0;oy<outH;oy++) for(let ox=0;ox<outW;ox++){
      const w=H[2][0]*ox+H[2][1]*oy+H[2][2];
      const sx=(H[0][0]*ox+H[0][1]*oy+H[0][2])/w;
      const sy=(H[1][0]*ox+H[1][1]*oy+H[1][2])/w;
      const x0=Math.floor(sx),y0=Math.floor(sy);
      if(x0<0||y0<0||x0+1>=imgEl.width||y0+1>=imgEl.height)continue;
      const fx=sx-x0,fy=sy-y0;
      const idx=(oy*outW+ox)*4;
      for(let ch=0;ch<4;ch++){
        const v00=srcD[(y0*imgEl.width+x0)*4+ch], v10=srcD[(y0*imgEl.width+x0+1)*4+ch];
        const v01=srcD[((y0+1)*imgEl.width+x0)*4+ch], v11=srcD[((y0+1)*imgEl.width+x0+1)*4+ch];
        out[idx+ch]=Math.round(v00*(1-fx)*(1-fy)+v10*fx*(1-fy)+v01*(1-fx)*fy+v11*fx*fy);
      }
    }
    octx.putImageData(oi,0,0); return oc;
  }

  // ── Auto-detect rectangle ──────────────────────────────────

  function autoDetectRectangle(imgEl) {
    const scale = 0.5;
    const sw = Math.round(imgEl.width * scale), sh = Math.round(imgEl.height * scale);
    const c = document.createElement('canvas');
    c.width = sw; c.height = sh;
    const ctx = c.getContext('2d');
    ctx.drawImage(imgEl, 0, 0, sw, sh);
    const imgData = ctx.getImageData(0, 0, sw, sh);

    const gray = new Uint8Array(sw * sh);
    for (let i = 0; i < gray.length; i++) {
      const o = i * 4;
      gray[i] = Math.round(imgData.data[o] * 0.299 + imgData.data[o+1] * 0.587 + imgData.data[o+2] * 0.114);
    }

    // Adaptive threshold: Otsu finds the optimal global split between
    // dark (clock body) and bright (background / clock faces).
    // We use a fraction of Otsu's value so glare-lifted grays are still "dark".
    const otsu = otsuThreshold(gray, sw * sh);
    const DARK = Math.max(45, Math.min(Math.round(otsu * 0.65), 120));

    // Global binary from adaptive threshold
    const globalBin = new Uint8Array(sw * sh);
    for (let i = 0; i < gray.length; i++) globalBin[i] = gray[i] < DARK ? 1 : 0;

    // Local adaptive binary — catches glare gradients where one side of the
    // rectangle is darker than the other.  Block size ~15% of image dimension.
    const blockSize = (Math.round(Math.min(sw, sh) * 0.15) | 1) || 1;
    const localBin = adaptiveThresholdBinary(gray, sw, sh, blockSize, 15);

    // Union: pixel is dark if either method flags it
    const binary = new Uint8Array(sw * sh);
    for (let i = 0; i < binary.length; i++) binary[i] = (globalBin[i] || localBin[i]) ? 1 : 0;

    // More aggressive morphological opening to remove thin structures like cables
    let processed = binary;
    for (let i = 0; i < 5; i++) processed = morphErode(processed, sw, sh);
    for (let i = 0; i < 5; i++) processed = morphDilate(processed, sw, sh);

    const { labels } = connectedComponents(processed, sw, sh);

    const bboxes = {};
    for (let y = 0; y < sh; y++)
      for (let x = 0; x < sw; x++) {
        const lab = labels[y * sw + x];
        if (!lab) continue;
        if (!bboxes[lab]) bboxes[lab] = { x0: x, y0: y, x1: x, y1: y, count: 0 };
        const b = bboxes[lab];
        if (x < b.x0) b.x0 = x; if (x > b.x1) b.x1 = x;
        if (y < b.y0) b.y0 = y; if (y > b.y1) b.y1 = y;
        b.count++;
      }

    let bestScore = 0, bestBox = null, bestLabel = 0;
    for (const key in bboxes) {
      const b = bboxes[key];
      const bw = b.x1 - b.x0 + 1, bh = b.y1 - b.y0 + 1;
      const area = bw * bh, fill = b.count / area, aspect = bw / bh;
      if (aspect < 1.8 || aspect > 4.5) continue;
      if (area < sw * sh * 0.04) continue;
      if (fill < 0.25) continue;
      let brightCount = 0;
      for (let y = b.y0; y <= b.y1; y++)
        for (let x = b.x0; x <= b.x1; x++)
          if (gray[y * sw + x] > 150) brightCount++;
      const brightRatio = brightCount / area;
      const brightScr = Math.min(brightRatio / 0.03, 1);
      const aspectScore = 1 / (1 + 2 * Math.abs(aspect - 2.67));
      const score = area * fill * fill * aspectScore * (1 + brightScr * 10);
      if (score > bestScore) { bestScore = score; bestBox = b; bestLabel = +key; }
    }

    if (!bestBox) return null;

    // ── Trim bounding box by removing thin protrusions (cables, etc.) ──
    {
      const bbW = bestBox.x1 - bestBox.x0 + 1;
      const bbH = bestBox.y1 - bestBox.y0 + 1;
      const colCounts = new Int32Array(bbW);
      const rowCounts = new Int32Array(bbH);
      for (let y = bestBox.y0; y <= bestBox.y1; y++)
        for (let x = bestBox.x0; x <= bestBox.x1; x++)
          if (labels[y * sw + x] === bestLabel) {
            colCounts[x - bestBox.x0]++;
            rowCounts[y - bestBox.y0]++;
          }
      let peakCol = 0, peakRow = 0;
      for (let i = 0; i < bbW; i++) if (colCounts[i] > peakCol) peakCol = colCounts[i];
      for (let i = 0; i < bbH; i++) if (rowCounts[i] > peakRow) peakRow = rowCounts[i];
      const colThresh = peakCol * 0.25, rowThresh = peakRow * 0.25;
      let newX0 = bestBox.x0, newX1 = bestBox.x1;
      let newY0 = bestBox.y0, newY1 = bestBox.y1;
      for (let i = 0; i < bbW; i++)
        if (colCounts[i] >= colThresh) { newX0 = bestBox.x0 + i; break; }
      for (let i = bbW - 1; i >= 0; i--)
        if (colCounts[i] >= colThresh) { newX1 = bestBox.x0 + i; break; }
      for (let i = 0; i < bbH; i++)
        if (rowCounts[i] >= rowThresh) { newY0 = bestBox.y0 + i; break; }
      for (let i = bbH - 1; i >= 0; i--)
        if (rowCounts[i] >= rowThresh) { newY1 = bestBox.y0 + i; break; }
      bestBox.x0 = newX0; bestBox.x1 = newX1;
      bestBox.y0 = newY0; bestBox.y1 = newY1;
    }

    // ── Edge detection: find sharpest brightness transition ──────
    // Instead of "first dark pixel" (which catches shadows), we find the
    // pixel with the steepest gradient — real clock edges are razor-sharp
    // while shadows and cables produce gradual transitions.
    const EDGE_DARK = Math.max(55, Math.min(DARK + 15, 130));
    const bw = bestBox.x1 - bestBox.x0, bh = bestBox.y1 - bestBox.y0;
    const margin = Math.round(Math.max(bw, bh) * 0.08);
    const MIN_THICKNESS = Math.round(Math.min(bw, bh) * 0.08);
    const GRAD_RADIUS = 2; // pixels each side for gradient computation

    function hasThickness(x, y, dirX, dirY) {
      let count = 0;
      for (let d = 0; d < MIN_THICKNESS; d++) {
        const px = Math.round(x + dirX * d);
        const py = Math.round(y + dirY * d);
        if (px < 0 || px >= sw || py < 0 || py >= sh) break;
        if (gray[py * sw + px] < EDGE_DARK) count++;
        else break;
      }
      return count >= MIN_THICKNESS * 0.7;
    }

    // Compute gradient magnitude at (x,y) along a given axis.
    // Returns the average brightness of pixels on the + side minus - side.
    function edgeGradient(x, y, axisX, axisY) {
      let sumPlus = 0, nPlus = 0, sumMinus = 0, nMinus = 0;
      for (let d = 1; d <= GRAD_RADIUS; d++) {
        const px1 = x + axisX * d, py1 = y + axisY * d;
        const px2 = x - axisX * d, py2 = y - axisY * d;
        if (px1 >= 0 && px1 < sw && py1 >= 0 && py1 < sh) {
          sumPlus += gray[py1 * sw + px1]; nPlus++;
        }
        if (px2 >= 0 && px2 < sw && py2 >= 0 && py2 < sh) {
          sumMinus += gray[py2 * sw + px2]; nMinus++;
        }
      }
      if (!nPlus || !nMinus) return 0;
      return Math.abs(sumPlus / nPlus - sumMinus / nMinus);
    }

    // For each scan line, find the point with the strongest gradient
    // that also passes the thickness check (confirms it's a real dark region).
    const topPts = [], botPts = [], leftPts = [], rightPts = [];
    const MIN_GRAD = 25; // minimum gradient to count as a real edge

    for (let x = bestBox.x0; x <= bestBox.x1; x += 2) {
      // Top edge: scan downward, find steepest vertical gradient
      let bestY = -1, bestGrad = MIN_GRAD;
      const yStart = Math.max(0, bestBox.y0 - margin);
      const yEnd = bestBox.y0 + Math.round(bh * 0.25);
      for (let y = yStart; y <= yEnd; y++) {
        const g = edgeGradient(x, y, 0, 1);
        if (g > bestGrad && gray[y * sw + x] < EDGE_DARK + 30) {
          if (hasThickness(x, y, 0, 1)) { bestGrad = g; bestY = y; }
        }
      }
      if (bestY >= 0) topPts.push([x, bestY]);

      // Bottom edge: scan upward, find steepest vertical gradient
      bestY = -1; bestGrad = MIN_GRAD;
      const yStart2 = Math.min(sh - 1, bestBox.y1 + margin);
      const yEnd2 = bestBox.y1 - Math.round(bh * 0.25);
      for (let y = yStart2; y >= yEnd2; y--) {
        const g = edgeGradient(x, y, 0, 1);
        if (g > bestGrad && gray[y * sw + x] < EDGE_DARK + 30) {
          if (hasThickness(x, y, 0, -1)) { bestGrad = g; bestY = y; }
        }
      }
      if (bestY >= 0) botPts.push([x, bestY]);
    }

    for (let y = bestBox.y0; y <= bestBox.y1; y += 2) {
      // Left edge: scan rightward, find steepest horizontal gradient
      let bestX = -1, bestGrad = MIN_GRAD;
      const xStart = Math.max(0, bestBox.x0 - margin);
      const xEnd = bestBox.x0 + Math.round(bw * 0.25);
      for (let x = xStart; x <= xEnd; x++) {
        const g = edgeGradient(x, y, 1, 0);
        if (g > bestGrad && gray[y * sw + x] < EDGE_DARK + 30) {
          if (hasThickness(x, y, 1, 0)) { bestGrad = g; bestX = x; }
        }
      }
      if (bestX >= 0) leftPts.push([bestX, y]);

      // Right edge: scan leftward, find steepest horizontal gradient
      bestX = -1; bestGrad = MIN_GRAD;
      const xStart2 = Math.min(sw - 1, bestBox.x1 + margin);
      const xEnd2 = bestBox.x1 - Math.round(bw * 0.25);
      for (let x = xStart2; x >= xEnd2; x--) {
        const g = edgeGradient(x, y, 1, 0);
        if (g > bestGrad && gray[y * sw + x] < EDGE_DARK + 30) {
          if (hasThickness(x, y, -1, 0)) { bestGrad = g; bestX = x; }
        }
      }
      if (bestX >= 0) rightPts.push([bestX, y]);
    }

    const topLine = robustFitLineYofX(topPts);
    const botLine = robustFitLineYofX(botPts);
    const leftLine = robustFitLineXofY(leftPts);
    const rightLine = robustFitLineXofY(rightPts);

    const boxCorners = [
      [bestBox.x0 / scale, bestBox.y0 / scale], [bestBox.x1 / scale, bestBox.y0 / scale],
      [bestBox.x1 / scale, bestBox.y1 / scale], [bestBox.x0 / scale, bestBox.y1 / scale]
    ];

    if (!topLine || !botLine || !leftLine || !rightLine) return boxCorners;

    const tl = intersectYX(topLine, leftLine);
    const tr = intersectYX(topLine, rightLine);
    const br = intersectYX(botLine, rightLine);
    const bl = intersectYX(botLine, leftLine);

    const corners = [
      [tl[0] / scale, tl[1] / scale], [tr[0] / scale, tr[1] / scale],
      [br[0] / scale, br[1] / scale], [bl[0] / scale, bl[1] / scale]
    ];

    // ── Rectangularity validation ──────────────────────────────
    // Reject shapes that are too far from rectangular.  Perspective causes
    // some skew, but each corner angle should still be close to 90°.
    function cornerAngle(a, b, c) {
      const v1x = a[0] - b[0], v1y = a[1] - b[1];
      const v2x = c[0] - b[0], v2y = c[1] - b[1];
      const dot = v1x * v2x + v1y * v2y;
      const m1 = Math.sqrt(v1x * v1x + v1y * v1y);
      const m2 = Math.sqrt(v2x * v2x + v2y * v2y);
      if (m1 < 1 || m2 < 1) return 0;
      return Math.acos(Math.max(-1, Math.min(1, dot / (m1 * m2)))) * 180 / Math.PI;
    }

    const angles = [
      cornerAngle(corners[3], corners[0], corners[1]),
      cornerAngle(corners[0], corners[1], corners[2]),
      cornerAngle(corners[1], corners[2], corners[3]),
      cornerAngle(corners[2], corners[3], corners[0])
    ];

    // Each corner should be within 25° of 90° (allows perspective distortion)
    const maxDeviation = Math.max(...angles.map(a => Math.abs(a - 90)));
    if (maxDeviation > 25) return boxCorners;

    // Opposite sides should have similar lengths (ratio within 2:1)
    const topLen = dist(corners[0], corners[1]), botLen = dist(corners[3], corners[2]);
    const leftLen = dist(corners[0], corners[3]), rightLen = dist(corners[1], corners[2]);
    const hRatio = Math.max(topLen, botLen) / Math.min(topLen, botLen);
    const vRatio = Math.max(leftLen, rightLen) / Math.min(leftLen, rightLen);
    if (hRatio > 2 || vRatio > 2) return boxCorners;

    return corners;
  }

  // ── Hand width estimation ──────────────────────────────────

  function estimateHandWidth(gray, imgW, imgH, cellW, cellH, thresh) {
    const widths = [];
    for (let row = 0; row < ROWS; row++) {
      for (let col = 0; col < COLS; col++) {
        const cx = Math.round(col * cellW);
        const cy = Math.round(row * cellH);
        const cw = Math.round((col+1) * cellW) - cx;
        const ch = Math.round((row+1) * cellH) - cy;

        const bin = new Uint8Array(cw * ch);
        for (let y = 0; y < ch; y++)
          for (let x = 0; x < cw; x++)
            if (gray[(cy + y) * imgW + (cx + x)] >= thresh) bin[y * cw + x] = 1;

        const { labels } = connectedComponents(bin, cw, ch);
        const groups = {};
        for (let y = 0; y < ch; y++)
          for (let x = 0; x < cw; x++) {
            const lab = labels[y * cw + x];
            if (lab) { if (!groups[lab]) groups[lab] = []; groups[lab].push([x, y]); }
          }

        for (const key in groups) {
          const g = groups[key];
          if (g.length < 30) continue;
          const pca = computePCA(g);
          if (pca.lambda1 / pca.lambda2 < 5) continue;
          const nv = pca.v2;
          let minProj = Infinity, maxProj = -Infinity;
          for (const [x, y] of g) {
            const proj = (x - pca.center[0]) * nv[0] + (y - pca.center[1]) * nv[1];
            if (proj < minProj) minProj = proj;
            if (proj > maxProj) maxProj = proj;
          }
          widths.push(maxProj - minProj);
        }
      }
    }
    if (widths.length === 0) return Math.min(cellW, cellH) * 0.1;
    widths.sort((a, b) => a - b);
    return widths[Math.floor(widths.length / 2)];
  }

  // ── Auto threshold (hand-width sweep) ──────────────────────

  function computeAutoThreshold(gray, imgW, imgH, cellW, cellH) {
    const testThresholds = [];
    for (let t = 60; t <= 230; t += 5) testThresholds.push(t);

    const widths = [];
    for (const t of testThresholds)
      widths.push(estimateHandWidth(gray, imgW, imgH, cellW, cellH, t));

    let peakWidth = 0;
    for (let i = 0; i < widths.length; i++)
      if (widths[i] > peakWidth) peakWidth = widths[i];

    if (peakWidth < 1) return 160;

    const goodMin = peakWidth * 0.8;
    let lo = 0, hi = widths.length - 1;
    for (let i = 0; i < widths.length; i++)
      if (widths[i] >= goodMin) { lo = i; break; }
    for (let i = widths.length - 1; i >= 0; i--)
      if (widths[i] >= goodMin) { hi = i; break; }

    return testThresholds[Math.round((lo + hi) / 2)];
  }

  // ── Band sweep + peak extraction ──────────────────────────

  function bandSweep(brightPts, hx, hy, halfW, minR, maxR) {
    const accum = new Float32Array(ANGLE_STEPS);
    for (let ai = 0; ai < ANGLE_STEPS; ai++) {
      const rad = ai * Math.PI / 180;
      const dirX = Math.cos(rad), dirY = Math.sin(rad);
      const perpX = -Math.sin(rad), perpY = Math.cos(rad);
      let count = 0;
      for (let i = 0; i < brightPts.length; i++) {
        const ox = brightPts[i][0] - hx, oy = brightPts[i][1] - hy;
        const along = ox * dirX + oy * dirY;
        if (along < minR || along > maxR) continue;
        const perp = ox * perpX + oy * perpY;
        if (perp > -halfW && perp < halfW) count++;
      }
      accum[ai] = count;
    }
    const smoothed = new Float32Array(ANGLE_STEPS);
    for (let i = 0; i < ANGLE_STEPS; i++) {
      let s = 0, w = 0;
      for (let k = -8; k <= 8; k++) {
        const wt = Math.exp(-(k * k) / 18);
        s += accum[(i + k + ANGLE_STEPS) % ANGLE_STEPS] * wt;
        w += wt;
      }
      smoothed[i] = s / w;
    }
    return smoothed;
  }

  function extractPeaks(smoothed) {
    const maxVal = Math.max(...smoothed);
    if (maxVal < 1) return { angles: [0, 0], duplicated: true };

    const peakThresh = maxVal * 0.30;
    let startIdx = 0;
    for (let i = 0; i < ANGLE_STEPS; i++) if (smoothed[i] < peakThresh) { startIdx = i; break; }

    const regions = [];
    let inRegion = false, rStart = 0;
    for (let k = 0; k < ANGLE_STEPS; k++) {
      const i = (startIdx + k) % ANGLE_STEPS;
      if (smoothed[i] >= peakThresh) { if (!inRegion) { rStart = i; inRegion = true; } }
      else if (inRegion) { regions.push({ start: rStart, end: (i - 1 + ANGLE_STEPS) % ANGLE_STEPS }); inRegion = false; }
    }
    if (inRegion) regions.push({ start: rStart, end: (startIdx - 1 + ANGLE_STEPS) % ANGLE_STEPS });

    const peaks = [];
    for (const r of regions) {
      const len = r.end >= r.start ? r.end - r.start + 1 : (ANGLE_STEPS - r.start) + r.end + 1;
      let sumA = 0, sumW = 0;
      for (let k = 0; k < len; k++) {
        const idx = (r.start + k) % ANGLE_STEPS;
        sumA += k * smoothed[idx];
        sumW += smoothed[idx];
      }
      peaks.push({ angle: Math.round((r.start + sumA / sumW) % ANGLE_STEPS) % 360, strength: sumW });
    }
    peaks.sort((a, b) => b.strength - a.strength);

    const chosen = [];
    for (const p of peaks) {
      if (chosen.length >= 2) break;
      if (chosen.some(c => { let d = Math.abs(c.angle - p.angle); if (d > 180) d = 360 - d; return d < 15; })) continue;
      chosen.push(p);
    }

    let duplicated = false;
    if (chosen.length === 1) { chosen.push({ angle: chosen[0].angle }); duplicated = true; }
    else if (chosen.length === 0) { chosen.push({ angle: 0 }, { angle: 0 }); duplicated = true; }
    return { angles: chosen.map(c => c.angle), duplicated };
  }

  // ── Per-cell hand angle detection ─────────────────────────

  function detectHandAngles(gray, imgW, cellX, cellY, cw, ch, thresh, handWidth) {
    const midX = Math.round(cw / 2), midY = Math.round(ch / 2);
    const maxR = Math.min(cw, ch) * 0.40;
    const halfW = handWidth / 2;

    // ── Build binary mask & connected components of bright pixels ──
    const cellBin = new Uint8Array(cw * ch);
    for (let y = 0; y < ch; y++)
      for (let x = 0; x < cw; x++)
        if (gray[(cellY + y) * imgW + (cellX + x)] >= thresh) cellBin[y * cw + x] = 1;

    const { labels } = connectedComponents(cellBin, cw, ch);
    const groups = {};
    for (let y = 0; y < ch; y++)
      for (let x = 0; x < cw; x++) {
        const lab = labels[y * cw + x];
        if (lab) { if (!groups[lab]) groups[lab] = []; groups[lab].push([x, y]); }
      }

    // ── Classify components: keep only elongated hand-like structures ──
    const minHandPixels = Math.max(15, cw * ch * 0.004);
    const handComponents = [];
    const filteredPts = [];

    for (const key in groups) {
      const g = groups[key];
      if (g.length < minHandPixels) continue;

      const pca = computePCA(g);
      const elongation = pca.lambda1 / pca.lambda2;

      // Must be elongated to be a hand (not a screw, tick mark, or reflection)
      if (elongation < 2.5) continue;

      // Must be in the general vicinity of the cell center
      const dx = pca.center[0] - midX, dy = pca.center[1] - midY;
      if (Math.sqrt(dx * dx + dy * dy) > maxR * 2) continue;

      handComponents.push({ pts: g, pca, elongation });
      for (const p of g) filteredPts.push(p);
    }

    // Fall back to all bright points if filtering removed everything
    const allBrightPts = [];
    for (let y = 0; y < ch; y++)
      for (let x = 0; x < cw; x++)
        if (cellBin[y * cw + x]) allBrightPts.push([x, y]);

    if (allBrightPts.length < 10)
      return { angles: [0, 0], centerX: cellX + midX, centerY: cellY + midY, duplicated: true };

    const usePts = filteredPts.length >= 10 ? filteredPts : allBrightPts;

    // ── Fast path: 2+ clear hand components → direct PCA angles ──
    if (handComponents.length >= 2) {
      // Pick the best two by size × elongation
      handComponents.sort((a, b) =>
        (b.pts.length * b.elongation) - (a.pts.length * a.elongation));

      // Select top 2 with sufficiently different angles
      const withAngles = handComponents.map(h => {
        let a = Math.atan2(h.pca.v1[1], h.pca.v1[0]) * 180 / Math.PI;
        if (a < 0) a += 360;
        return { ...h, rawAngle: a };
      });

      const chosen = [withAngles[0]];
      for (let i = 1; i < withAngles.length && chosen.length < 2; i++) {
        const c = withAngles[i];
        let dominated = false;
        for (const prev of chosen) {
          // PCA gives an axis, so angles 180° apart are the same axis
          let d = Math.abs(prev.rawAngle - c.rawAngle);
          if (d > 180) d = 360 - d;
          if (d < 25 || Math.abs(d - 180) < 25) { dominated = true; break; }
        }
        if (!dominated) chosen.push(c);
      }

      if (chosen.length === 2) {
        const h1 = chosen[0], h2 = chosen[1];

        // Find hinge as intersection of PCA centerlines
        const cross = h1.pca.v1[0] * h2.pca.v1[1] - h1.pca.v1[1] * h2.pca.v1[0];
        let hingeX = midX, hingeY = midY;

        if (Math.abs(cross) > 0.05) {
          const dpx = h2.pca.center[0] - h1.pca.center[0];
          const dpy = h2.pca.center[1] - h1.pca.center[1];
          const t = (dpx * h2.pca.v1[1] - dpy * h2.pca.v1[0]) / cross;
          const ix = h1.pca.center[0] + t * h1.pca.v1[0];
          const iy = h1.pca.center[1] + t * h1.pca.v1[1];
          const dFromMid = Math.sqrt((ix - midX) ** 2 + (iy - midY) ** 2);
          if (dFromMid < Math.min(cw, ch) * 0.25 && ix >= 0 && ix < cw && iy >= 0 && iy < ch) {
            hingeX = Math.round(ix);
            hingeY = Math.round(iy);
          }
        }

        // Orient each hand's PCA direction away from hinge
        const angles = chosen.map(h => {
          const dx = h.pca.center[0] - hingeX;
          const dy = h.pca.center[1] - hingeY;
          const dot = dx * h.pca.v1[0] + dy * h.pca.v1[1];
          const dirX = dot >= 0 ? h.pca.v1[0] : -h.pca.v1[0];
          const dirY = dot >= 0 ? h.pca.v1[1] : -h.pca.v1[1];
          let angle = Math.atan2(dirY, dirX) * 180 / Math.PI;
          if (angle < 0) angle += 360;
          return Math.round(angle) % 360;
        });

        return {
          angles,
          centerX: cellX + hingeX,
          centerY: cellY + hingeY,
          duplicated: false
        };
      }
    }

    // ── Slow path: band sweep on filtered points ──

    // Stage 0: Arc/circle detection for hinge
    let arcHingeX = -1, arcHingeY = -1, arcFound = false;
    {
      const edgePts = [];
      for (let y = 1; y < ch - 1; y++)
        for (let x = 1; x < cw - 1; x++) {
          if (!cellBin[y * cw + x]) continue;
          if (!cellBin[(y-1)*cw+x] || !cellBin[(y+1)*cw+x] ||
              !cellBin[y*cw+x-1] || !cellBin[y*cw+x+1])
            edgePts.push([x, y]);
        }

      const expectedR = handWidth / 2;
      const rTol = Math.max(2, expectedR * 0.4);
      const searchRadius = Math.round(Math.min(cw, ch) * 0.10);
      const arcStep = Math.max(1, Math.round(searchRadius / 8));
      let bestArcScore = 0;

      for (let ty = midY - searchRadius; ty <= midY + searchRadius; ty += arcStep) {
        for (let tx = midX - searchRadius; tx <= midX + searchRadius; tx += arcStep) {
          if (tx < 0 || tx >= cw || ty < 0 || ty >= ch) continue;
          let arcCount = 0;
          for (let i = 0; i < edgePts.length; i++) {
            const dx = edgePts[i][0] - tx, dy = edgePts[i][1] - ty;
            const d = Math.sqrt(dx * dx + dy * dy);
            if (Math.abs(d - expectedR) <= rTol) arcCount++;
          }
          const minArcPixels = Math.max(5, expectedR * Math.PI * 0.3);
          if (arcCount >= minArcPixels && arcCount > bestArcScore) {
            bestArcScore = arcCount;
            arcHingeX = tx;
            arcHingeY = ty;
          }
        }
      }

      if (bestArcScore > 0) {
        const fineR = arcStep + 1;
        const coarseX = arcHingeX, coarseY = arcHingeY;
        for (let ty = coarseY - fineR; ty <= coarseY + fineR; ty++) {
          for (let tx = coarseX - fineR; tx <= coarseX + fineR; tx++) {
            if (tx < 0 || tx >= cw || ty < 0 || ty >= ch) continue;
            let arcCount = 0;
            for (let i = 0; i < edgePts.length; i++) {
              const dx = edgePts[i][0] - tx, dy = edgePts[i][1] - ty;
              const d = Math.sqrt(dx * dx + dy * dy);
              if (Math.abs(d - expectedR) <= rTol) arcCount++;
            }
            if (arcCount > bestArcScore) {
              bestArcScore = arcCount;
              arcHingeX = tx;
              arcHingeY = ty;
            }
          }
        }
        arcFound = true;
      }
    }

    // Stage 1: Approximate hinge via angular-peak scoring (fallback)
    const HINGE_BINS = 72;
    const hingeMinR = handWidth * 0.5;
    const searchR = Math.round(Math.min(cw, ch) * 0.2);
    const step = Math.max(2, Math.round(searchR / 4));

    let bestHx = midX, bestHy = midY, bestHingeScore = -1;

    if (arcFound) {
      bestHx = arcHingeX;
      bestHy = arcHingeY;
      bestHingeScore = 1;
    } else {
      for (let ty = midY - searchR; ty <= midY + searchR; ty += step) {
        for (let tx = midX - searchR; tx <= midX + searchR; tx += step) {
          if (tx < 0 || tx >= cw || ty < 0 || ty >= ch) continue;
          const hist = new Float32Array(HINGE_BINS);
          let inRange = 0;
          for (let i = 0; i < usePts.length; i++) {
            const ox = usePts[i][0] - tx, oy = usePts[i][1] - ty;
            const d2 = ox * ox + oy * oy;
            if (d2 < hingeMinR * hingeMinR || d2 > maxR * maxR) continue;
            inRange++;
            let angle = Math.atan2(oy, ox);
            if (angle < 0) angle += 2 * Math.PI;
            let bin = Math.floor(angle * HINGE_BINS / (2 * Math.PI));
            if (bin >= HINGE_BINS) bin = HINGE_BINS - 1;
            hist[bin]++;
          }
          if (inRange < 10) continue;
          const sm = new Float32Array(HINGE_BINS);
          for (let i = 0; i < HINGE_BINS; i++)
            sm[i] = hist[i] * 0.5
                  + hist[(i - 1 + HINGE_BINS) % HINGE_BINS] * 0.25
                  + hist[(i + 1) % HINGE_BINS] * 0.25;
          const sorted = Array.from(sm).map((v, i) => ({ v, i })).sort((a, b) => b.v - a.v);
          let p1 = sorted[0].v, p2 = 0;
          for (let j = 1; j < sorted.length; j++) {
            let gap = Math.abs(sorted[j].i - sorted[0].i);
            if (gap > HINGE_BINS / 2) gap = HINGE_BINS - gap;
            if (gap >= 3) { p2 = sorted[j].v; break; }
          }
          const score = p1 * p2;
          if (score > bestHingeScore) { bestHingeScore = score; bestHx = tx; bestHy = ty; }
        }
      }
    }

    // Stage 2: Preliminary angle sweep (on filtered points)
    const sweepMinR = handWidth * 1.0;
    const prelim = bandSweep(usePts, bestHx, bestHy, halfW, sweepMinR, maxR);
    const prelimResult = extractPeaks(prelim);

    // Stage 3: Refine hinge via PCA centerline intersection
    if (!prelimResult.duplicated && prelimResult.angles.length === 2) {
      const lines = [];
      for (const ang of prelimResult.angles) {
        const rad = ang * Math.PI / 180;
        const dirX = Math.cos(rad), dirY = Math.sin(rad);
        const perpX = -Math.sin(rad), perpY = Math.cos(rad);
        const handPts = [];
        for (const [px, py] of usePts) {
          const ox = px - bestHx, oy = py - bestHy;
          const along = ox * dirX + oy * dirY;
          if (along < 0 || along > maxR) continue;
          const perp = ox * perpX + oy * perpY;
          if (perp > -halfW * 1.3 && perp < halfW * 1.3) handPts.push([px, py]);
        }
        if (handPts.length < 15) continue;
        const pca = computePCA(handPts);
        lines.push({ cx: pca.center[0], cy: pca.center[1], dx: pca.v1[0], dy: pca.v1[1] });
      }

      if (lines.length === 2) {
        const cross = lines[0].dx * lines[1].dy - lines[0].dy * lines[1].dx;
        if (Math.abs(cross) > 0.05) {
          const dpx = lines[1].cx - lines[0].cx;
          const dpy = lines[1].cy - lines[0].cy;
          const t = (dpx * lines[1].dy - dpy * lines[1].dx) / cross;
          const ix = lines[0].cx + t * lines[0].dx;
          const iy = lines[0].cy + t * lines[0].dy;
          const refDist = Math.sqrt((ix - bestHx) ** 2 + (iy - bestHy) ** 2);
          if (refDist < Math.min(cw, ch) * 0.2 && ix >= 0 && ix < cw && iy >= 0 && iy < ch) {
            bestHx = Math.round(ix);
            bestHy = Math.round(iy);
          }
        }
      }
    }

    // Stage 4: Final angle sweep from refined hinge (on filtered points)
    const finalHist = bandSweep(usePts, bestHx, bestHy, halfW, sweepMinR, maxR);
    const finalResult = extractPeaks(finalHist);

    return {
      angles: finalResult.angles,
      centerX: cellX + bestHx,
      centerY: cellY + bestHy,
      duplicated: finalResult.duplicated
    };
  }

  // ── Render hands on canvas ──────────────────────────────────

  function renderClocks(canvas, clocks, handWidth) {
    const ctx = canvas.getContext('2d');
    const handLen = Math.min(canvas.width / COLS, canvas.height / ROWS) * 0.38;
    const lineWidth = Math.max(2, handWidth * 0.4);

    ctx.lineCap = 'round';
    ctx.lineWidth = lineWidth;

    for (let row = 0; row < ROWS; row++) {
      for (let col = 0; col < COLS; col++) {
        const clock = clocks[row][col];
        const cx = clock.center.x;
        const cy = clock.center.y;

        // Draw each hand
        for (let i = 0; i < 2; i++) {
          const clockAngle = clock.angles[i];
          // Convert clock angle (0°=12 o'clock, clockwise) to radians
          // Clock: 0°=up, 90°=right, 180°=down, 270°=left
          // Math/screen: need -90° rotation, and negate for clockwise
          const rad = (clockAngle - 90) * Math.PI / 180;
          const endX = cx + Math.cos(rad) * handLen;
          const endY = cy + Math.sin(rad) * handLen;

          // Draw hand with outline for visibility
          ctx.strokeStyle = '#000';
          ctx.lineWidth = lineWidth + 2;
          ctx.beginPath();
          ctx.moveTo(cx, cy);
          ctx.lineTo(endX, endY);
          ctx.stroke();

          ctx.strokeStyle = i === 0 ? '#ff3333' : '#33ff33';
          ctx.lineWidth = lineWidth;
          ctx.beginPath();
          ctx.moveTo(cx, cy);
          ctx.lineTo(endX, endY);
          ctx.stroke();
        }

        // Draw center dot
        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.arc(cx, cy, lineWidth * 0.8, 0, 2 * Math.PI);
        ctx.fill();
        ctx.strokeStyle = '#000';
        ctx.lineWidth = 1;
        ctx.stroke();
      }
    }
  }

  // ── Convert math angle to clock angle ─────────────────────

  function mathToClockAngle(mathAngle) {
    // Math convention: 0°=right, 90°=down (screen coords)
    // Clock convention: 0°=12 o'clock (up), 90°=3 o'clock (right), clockwise
    // Conversion: clock = (math + 90) % 360, but we want 0=up
    // Actually: clock = (90 - math + 360) % 360 to flip direction
    // Let's verify: math 0 (right) → clock 90 (3 o'clock) ✓
    //              math 90 (down) → clock 180 (6 o'clock) ✓
    //              math 180 (left) → clock 270 (9 o'clock) ✓
    //              math 270 (up) → clock 0 (12 o'clock) ✓
    return (mathAngle + 90) % 360;
  }

  // ── Main pipeline ─────────────────────────────────────────

  // Detect or use provided corners
  const srcCorners = opts.corners || autoDetectRectangle(img);
  if (!srcCorners) {
    return { clocks: [], canvas: null, corners: null, threshold: 0, handWidth: 0, error: 'Could not detect clock rectangle' };
  }

  // Compute output dimensions
  const topLen = dist(srcCorners[0], srcCorners[1]), botLen = dist(srcCorners[3], srcCorners[2]);
  const leftLen = dist(srcCorners[0], srcCorners[3]), rightLen = dist(srcCorners[1], srcCorners[2]);
  const outW = Math.round((topLen + botLen) / 2);
  const outH = Math.round((leftLen + rightLen) / 2);

  // Perspective warp
  const warped = perspectiveWarp(img, srcCorners, outW, outH);

  // Grayscale
  const wctx = warped.getContext('2d');
  const warpData = wctx.getImageData(0, 0, outW, outH);
  const gray = new Uint8Array(outW * outH);
  for (let i = 0; i < gray.length; i++) {
    const o = i * 4;
    gray[i] = Math.round(warpData.data[o] * 0.299 + warpData.data[o+1] * 0.587 + warpData.data[o+2] * 0.114);
  }

  const cellW = outW / COLS, cellH = outH / ROWS;

  // Threshold
  const THRESH = opts.threshold || computeAutoThreshold(gray, outW, outH, cellW, cellH);

  // Hand width
  const handWidth = estimateHandWidth(gray, outW, outH, cellW, cellH, THRESH);

  // Detect all cells and build clock data structure
  const clocks = [];
  for (let row = 0; row < ROWS; row++) {
    const rowClocks = [];
    for (let col = 0; col < COLS; col++) {
      const cx = Math.round(col * cellW);
      const cy = Math.round(row * cellH);
      const cw = Math.round((col + 1) * cellW) - cx;
      const ch = Math.round((row + 1) * cellH) - cy;
      const result = detectHandAngles(gray, outW, cx, cy, cw, ch, THRESH, handWidth);

      // Convert to clock angles (0°=12 o'clock, clockwise)
      const clockAngles = result.angles.map(mathToClockAngle);

      rowClocks.push({
        angles: clockAngles,
        center: { x: result.centerX, y: result.centerY },
        cellBounds: { x: cx, y: cy, w: cw, h: ch },
        duplicated: result.duplicated
      });
    }
    clocks.push(rowClocks);
  }

  // Create visualization canvas
  const canvas = document.createElement('canvas');
  canvas.width = outW;
  canvas.height = outH;
  const ctx = canvas.getContext('2d');
  ctx.drawImage(warped, 0, 0);
  renderClocks(canvas, clocks, handWidth);

  // Helper: extract just the angles array (backwards compatibility)
  const angles = clocks.map(row => row.map(c => c.angles));

  return {
    clocks,       // 3×8 array of clock objects with angles, center, cellBounds
    angles,       // 3×8 array of [angle1, angle2] pairs (backwards compat)
    canvas,       // Canvas with warped image and hands overlay
    corners: srcCorners,
    threshold: THRESH,
    handWidth
  };
}

/**
 * ClockClockCV — helper utilities for video stream detection
 */
// eslint-disable-next-line no-unused-vars
const ClockClockCV = {
  /**
   * Create a video stream detector for real-time ClockClock detection.
   * @param {HTMLVideoElement} video - The video element to process
   * @param {Object} options - Configuration options
   * @param {number} options.frameRate - Target frames per second (default: 10)
   * @param {function} options.onDetection - Callback with detection result
   * @param {function} options.onError - Error callback
   * @returns {Object} Controller with start(), stop(), capture() methods
   */
  createVideoDetector: function(video, options) {
    const opts = options || {};
    let frameRate = opts.frameRate || 10;
    const onDetection = opts.onDetection || function() {};
    const onError = opts.onError || function() {};

    let intervalId = null;
    let lastResult = null;
    let lastSnapshot = null;
    let frameCount = 0;
    let lastFpsTime = Date.now();
    let currentFps = 0;
    let isProcessing = false;

    // ── Temporal smoothing state ──
    // Stabilize corners across frames to prevent jitter in the live preview.
    let smoothedCorners = null;  // EMA-smoothed corners
    let stableCount = 0;        // consecutive frames with consistent corners
    const LOCK_AFTER = 3;       // frames of consistency before locking
    const EMA_ALPHA = 0.3;      // blend factor (0 = full history, 1 = full new)
    let jumpThreshold = 50;     // max avg corner movement (px), computed dynamically

    function cornerDistance(a, b) {
      if (!a || !b || a.length !== 4 || b.length !== 4) return Infinity;
      let total = 0;
      for (let i = 0; i < 4; i++) {
        const dx = a[i][0] - b[i][0], dy = a[i][1] - b[i][1];
        total += Math.sqrt(dx * dx + dy * dy);
      }
      return total / 4;  // average per-corner displacement
    }

    function blendCorners(old, cur, alpha) {
      return old.map(function(o, i) {
        return [
          o[0] * (1 - alpha) + cur[i][0] * alpha,
          o[1] * (1 - alpha) + cur[i][1] * alpha
        ];
      });
    }

    const tempCanvas = document.createElement('canvas');
    const tempCtx = tempCanvas.getContext('2d');

    function processFrame() {
      if (isProcessing || video.videoWidth === 0 || video.videoHeight === 0) return;
      isProcessing = true;

      try {
        tempCanvas.width = video.videoWidth;
        tempCanvas.height = video.videoHeight;
        tempCtx.drawImage(video, 0, 0);

        // Snapshot the frame before processing so capture returns the exact frame used
        const snap = document.createElement('canvas');
        snap.width = tempCanvas.width;
        snap.height = tempCanvas.height;
        snap.getContext('2d').drawImage(tempCanvas, 0, 0);

        // Dynamic jump threshold: ~3% of image diagonal
        jumpThreshold = Math.sqrt(video.videoWidth * video.videoWidth +
                                  video.videoHeight * video.videoHeight) * 0.03;

        // Run detection fresh to get this frame's raw corners
        var rawResult = detectClockAngles(tempCanvas);
        var rawCorners = rawResult.corners;

        if (rawCorners) {
          if (!smoothedCorners) {
            // First detection — accept immediately
            smoothedCorners = rawCorners.map(function(c) { return [c[0], c[1]]; });
            stableCount = 1;
          } else {
            var jump = cornerDistance(rawCorners, smoothedCorners);
            if (jump < jumpThreshold) {
              // Consistent — blend into smoothed corners
              smoothedCorners = blendCorners(smoothedCorners, rawCorners, EMA_ALPHA);
              stableCount = Math.min(stableCount + 1, LOCK_AFTER + 10);
            } else if (jump < jumpThreshold * 4) {
              // Moderate jump — might be real movement, blend slowly
              smoothedCorners = blendCorners(smoothedCorners, rawCorners, EMA_ALPHA * 0.5);
              stableCount = Math.max(1, stableCount - 1);
            } else {
              // Large jump — likely a detection error, or camera moved a lot.
              // Reset and start tracking the new position.
              smoothedCorners = rawCorners.map(function(c) { return [c[0], c[1]]; });
              stableCount = 1;
            }
          }

          // Once stable, re-run detection with smoothed corners for consistent results
          if (stableCount >= LOCK_AFTER) {
            lastResult = detectClockAngles(tempCanvas, { corners: smoothedCorners });
          } else {
            lastResult = rawResult;
          }
        } else {
          // No detection this frame — keep previous result but mark not detected
          if (smoothedCorners) {
            // Try with last known corners — the clock is probably still there
            lastResult = detectClockAngles(tempCanvas, { corners: smoothedCorners });
          } else {
            lastResult = rawResult;
          }
        }

        lastSnapshot = snap;

        // Update FPS
        frameCount++;
        var now = Date.now();
        if (now - lastFpsTime >= 1000) {
          currentFps = frameCount;
          frameCount = 0;
          lastFpsTime = now;
        }

        onDetection({
          result: lastResult,
          fps: currentFps,
          detected: !!lastResult.corners
        });
      } catch (err) {
        onError(err);
      }

      isProcessing = false;
    }

    return {
      start: function() {
        if (intervalId) return;
        intervalId = setInterval(processFrame, 1000 / frameRate);
      },
      stop: function() {
        if (intervalId) {
          clearInterval(intervalId);
          intervalId = null;
        }
      },
      setFrameRate: function(fps) {
        frameRate = fps;
        if (intervalId) {
          clearInterval(intervalId);
          intervalId = setInterval(processFrame, 1000 / frameRate);
        }
      },
      capture: function() {
        if (!lastResult) return null;
        // For capture, re-run with smoothed corners for best quality
        if (smoothedCorners && lastSnapshot) {
          var captureResult = detectClockAngles(lastSnapshot, { corners: smoothedCorners });
          captureResult.snapshot = lastSnapshot;
          return captureResult;
        }
        lastResult.snapshot = lastSnapshot;
        return lastResult;
      },
      resetTracking: function() {
        smoothedCorners = null;
        stableCount = 0;
      },
      isRunning: function() {
        return !!intervalId;
      }
    };
  },

  /**
   * Draw detection overlay on a canvas.
   * @param {CanvasRenderingContext2D} ctx - Canvas context to draw on
   * @param {Object} result - Detection result from detectClockAngles
   * @param {number} scaleX - X scale factor (overlay width / video width)
   * @param {number} scaleY - Y scale factor (overlay height / video height)
   * @param {Object} options - Drawing options
   */
  drawOverlay: function(ctx, result, scaleX, scaleY, options) {
    const opts = options || {};
    ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);

    if (!result || !result.corners) return;

    // Draw detection rectangle
    ctx.strokeStyle = opts.strokeColor || '#00ff00';
    ctx.lineWidth = opts.lineWidth || 2;
    ctx.beginPath();
    ctx.moveTo(result.corners[0][0] * scaleX, result.corners[0][1] * scaleY);
    for (let i = 1; i < result.corners.length; i++) {
      ctx.lineTo(result.corners[i][0] * scaleX, result.corners[i][1] * scaleY);
    }
    ctx.closePath();
    ctx.stroke();

    // Semi-transparent fill
    if (opts.fill !== false) {
      ctx.fillStyle = opts.fillColor || 'rgba(0, 255, 0, 0.1)';
      ctx.fill();
    }
  },

  /**
   * Initialize camera with fallback options.
   * @param {HTMLVideoElement} video - Video element to attach stream to
   * @param {Object} options - Camera options
   * @param {string} options.facingMode - 'environment' (back) or 'user' (front)
   * @returns {Promise<MediaStream>} The media stream
   */
  initCamera: async function(video, options) {
    const opts = options || {};
    const facingMode = opts.facingMode || 'environment';

    if (!window.isSecureContext) {
      throw new Error('Camera requires HTTPS or localhost');
    }
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      throw new Error('getUserMedia not supported');
    }

    const constraintOptions = [
      { video: { facingMode: { ideal: facingMode }, width: { ideal: 1920 }, height: { ideal: 1080 } } },
      { video: { facingMode: { ideal: facingMode } } },
      { video: true }
    ];

    let stream = null;
    let lastError = null;

    for (const constraints of constraintOptions) {
      try {
        stream = await navigator.mediaDevices.getUserMedia(constraints);
        break;
      } catch (err) {
        lastError = err;
        stream = null;
      }
    }

    if (!stream) {
      throw lastError || new Error('Could not access camera');
    }

    video.srcObject = stream;
    await new Promise((resolve, reject) => {
      video.onloadedmetadata = () => video.play().then(resolve).catch(reject);
      video.onerror = reject;
      setTimeout(() => reject(new Error('Video load timeout')), 10000);
    });

    return stream;
  }
};
