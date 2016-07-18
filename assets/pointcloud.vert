#version 410

layout(location = 0) in vec4 position;

layout(location = 1) in vec4 color;

out vec4 vColor;

uniform mat4 ciModelViewProjection;

void main() {
    gl_Position = ciModelViewProjection * position;
    vColor = vec4(color[2], color[1], color[0], color[3]) / 255.0;
}
