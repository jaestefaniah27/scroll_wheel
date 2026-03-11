/**
 * OLED Bitmap Converter
 * Converts an SVG icon path to a 72×40 monochrome (1-bit) bitmap
 * suitable for transmission to the Pico OLED displays.
 * 
 * Output: base64-encoded byte array where each bit = 1 pixel (1=white, 0=black)
 * Total size: ceil(72*40/8) = 360 bytes
 */
export async function convertIconToOledBitmap(svgPath, width = 72, height = 40) {
  return new Promise((resolve, reject) => {
    const canvas = document.createElement('canvas');
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext('2d');

    const img = new Image();
    img.onload = () => {
      // Black background
      ctx.fillStyle = '#000000';
      ctx.fillRect(0, 0, width, height);
      // Draw icon — SVGs use stroke="white" so they appear white on black
      ctx.drawImage(img, 0, 0, width, height);

      const imageData = ctx.getImageData(0, 0, width, height);
      const pixels = imageData.data;
      const totalPixels = width * height;
      const bytes = new Uint8Array(Math.ceil(totalPixels / 8));

      for (let i = 0; i < totalPixels; i++) {
        const r = pixels[i * 4];
        const g = pixels[i * 4 + 1];
        const b = pixels[i * 4 + 2];
        const luminance = (r * 0.299 + g * 0.587 + b * 0.114);
        if (luminance > 64) { // threshold
          bytes[Math.floor(i / 8)] |= (1 << (7 - (i % 8)));
        }
      }

      resolve(btoa(String.fromCharCode(...bytes)));
    };

    img.onerror = (e) => {
      console.warn('[OLED] Could not load icon for conversion:', svgPath);
      // Return blank bitmap on error
      resolve(btoa(String.fromCharCode(...new Uint8Array(Math.ceil(width * height / 8)))));
    };

    img.src = svgPath;
  });
}

/**
 * Render a preview of what the OLED will actually show onto a <canvas> element.
 */
export function renderOledPreview(ctx, svgPath, width = 72, height = 40) {
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, width, height);
  if (!svgPath) return;

  const img = new Image();
  img.onload = () => {
    ctx.drawImage(img, 0, 0, width, height);
    // Quantize to 1-bit (optional visual effect in preview)
    const d = ctx.getImageData(0, 0, width, height);
    for (let i = 0; i < d.data.length; i += 4) {
      const v = (d.data[i] + d.data[i+1] + d.data[i+2]) / 3 > 64 ? 255 : 0;
      d.data[i] = d.data[i+1] = d.data[i+2] = v;
    }
    ctx.putImageData(d, 0, 0);
  };
  img.src = svgPath;
}
