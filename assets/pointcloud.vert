#version 410

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;

out vec4 calibedPosition;
out vec4 vColor;

uniform mat4 ciModelViewProjection;
uniform mat4 calibMatrix;
uniform float fx;
uniform float fy;
uniform float cx;
uniform float cy;

void main() {
    float zw = position[2] / 1000.0;
    float xw = (position[0] - cx) / fx * zw;
    float yw = (position[1] - cy) / fy * zw;
    calibedPosition = calibMatrix * vec4(xw, yw, zw, 1.0);
    gl_Position = ciModelViewProjection * calibedPosition;
    vColor = vec4(color / 255.0, 1.0);
}
