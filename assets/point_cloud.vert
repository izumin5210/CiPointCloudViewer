#version 410

layout(location = 0) in vec4 position;
layout(location = 1) in vec4 color;

out vec4 calibedPosition;
out vec4 vColor;

uniform mat4 ciModelViewProjection;

void main() {
    calibedPosition = position;
    gl_Position = ciModelViewProjection * calibedPosition;
    vColor = vec4(color[2], color[1], color[0], color[3]) / 255.0;
}
