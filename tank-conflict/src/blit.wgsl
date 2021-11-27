[[block]]
struct Data {
    color: vec3<f32>;
    transform: mat4x4<f32>;
};

[[block]]
struct Window {
    projection: mat4x4<f32>;
};

[[group(0), binding(0)]]
var<uniform> u_data: Data;
[[group(1), binding(0)]]
var<uniform> u_proj: Window;

struct VertexOut {
    [[builtin(position)]] position: vec4<f32>;
    [[location(0)]] tex_coord: vec2<f32>;
};

[[stage(vertex)]]
fn main([[location(0)]] pos: vec2<f32>, [[location(1)]] tex_coord: vec2<f32>) -> VertexOut {
    var out: VertexOut;
    out.position = u_proj.projection * u_data.transform * vec4<f32>(pos, 0.0, 1.0);
    out.tex_coord = tex_coord;
    return out;
}

[[group(2), binding(0)]]
var t_texture: texture_2d<f32>;
[[group(2), binding(1)]]
var s_texture: sampler;

[[stage(fragment)]]
fn main(in: VertexOut) -> [[location(0)]] vec4<f32> {
    return textureSample(t_texture, s_texture, in.tex_coord) * vec4<f32>(u_data.color, 1.0);
}
