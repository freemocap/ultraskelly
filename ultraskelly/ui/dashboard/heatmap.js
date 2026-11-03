// The depth heat map drawn on the GPU: the millimetres go up as a 16-bit integer texture and a fragment
// shader colours each pixel with the same turbo polynomial as the legend (colormap.js), red near and blue
// far. The 3D scene's depth texture lives in another WebGL context, which cannot draw into this tile, so the
// tile has its own small one; uploading the same 126 KB again costs far less than colouring on the CPU.
const VERTEX = `#version 300 es
out vec2 uv;
void main() {  // One triangle that covers the canvas
  vec2 corner = vec2(gl_VertexID == 1 ? 2.0 : 0.0, gl_VertexID == 2 ? 2.0 : 0.0);
  uv = corner;
  gl_Position = vec4(corner * 2.0 - 1.0, 0.0, 1.0);
}`;
const FRAGMENT = `#version 300 es
precision highp float;
precision highp usampler2D;
uniform usampler2D depth;
uniform vec2 range;  // Near and far, millimetres
in vec2 uv;
out vec4 colour;
vec3 turbo(float t) {
  vec3 c0 = vec3(0.13572138, 0.09140261, 0.10667330), c1 = vec3(4.61539260, 2.19418839, 12.64194608);
  vec3 c2 = vec3(-42.66032258, 4.84296658, -60.58204836), c3 = vec3(132.13108234, -14.18503333, 110.36276771);
  vec3 c4 = vec3(-152.94239396, 4.27729857, -89.90310912), c5 = vec3(59.28637943, 2.82956604, 27.34824973);
  return clamp(c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * c5)))), 0.0, 1.0);
}
void main() {
  ivec2 size = textureSize(depth, 0);
  uint d = texelFetch(depth, clamp(ivec2(uv.x * float(size.x), (1.0 - uv.y) * float(size.y)), ivec2(0), size - 1), 0).r;
  if (d == 0u) { colour = vec4(0.0, 0.0, 0.0, 1.0); return; }  // Unknown stays black
  float t = clamp((float(d) - range.x) / max(1.0, range.y - range.x), 0.0, 1.0);
  colour = vec4(turbo(1.0 - t), 1.0);
}`;

export class HeatMap {
  // canvas: the heat tile's canvas, used only through this WebGL2 context.
  constructor(canvas) {
    this.canvas = canvas;
    // The drawing buffer is kept so the enlarged view can copy it between draws; it starts transparent, so the
    // tile looks like the other empty tiles until depth comes, and every pixel drawn is opaque.
    const gl = this.gl = canvas.getContext('webgl2', {antialias: false, depth: false, preserveDrawingBuffer: true});
    if (!gl) throw new Error('The heat map needs WebGL2');
    const shader = (type, source) => {
      const made = gl.createShader(type);
      gl.shaderSource(made, source); gl.compileShader(made);
      if (!gl.getShaderParameter(made, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(made));
      return made;
    };
    const program = this.program = gl.createProgram();
    gl.attachShader(program, shader(gl.VERTEX_SHADER, VERTEX));
    gl.attachShader(program, shader(gl.FRAGMENT_SHADER, FRAGMENT));
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program));
    this.range = gl.getUniformLocation(program, 'range');
    this.texture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, this.texture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.pixelStorei(gl.UNPACK_ALIGNMENT, 2);
    this.size = '';
  }

  // mm: the depth in millimetres, row-major; near and far: the colour range in millimetres.
  draw(mm, width, height, near, far) {
    const gl = this.gl;
    if (this.canvas.width !== width || this.canvas.height !== height) { this.canvas.width = width; this.canvas.height = height; }
    gl.bindTexture(gl.TEXTURE_2D, this.texture);
    if (this.size !== `${width}x${height}`) {
      gl.texImage2D(gl.TEXTURE_2D, 0, gl.R16UI, width, height, 0, gl.RED_INTEGER, gl.UNSIGNED_SHORT, mm);
      this.size = `${width}x${height}`;
    } else {
      gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, width, height, gl.RED_INTEGER, gl.UNSIGNED_SHORT, mm);
    }
    gl.viewport(0, 0, width, height);
    gl.useProgram(this.program);
    gl.uniform2f(this.range, near, far);
    gl.drawArrays(gl.TRIANGLES, 0, 3);
  }
}
