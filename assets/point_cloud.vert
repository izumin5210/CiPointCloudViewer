#version 410

layout(location = 0) in vec4 position;
layout(location = 1) in vec4 color;

out vec4 calibedPosition;
out vec4 vColor;
flat out int userId;

uniform bool calibrated = true;
uniform mat4 ciModelViewProjection;
uniform mat4 calibMatrix;
uniform float fx;
uniform float fy;
uniform float cx;
uniform float cy;

void main() {
    // FIXME
    userId = 0;
    if (calibrated) {
        calibedPosition = position;
    } else {
//        float zw = position[2] / 1000.0;
//        float xw = (position[0] - cx) / fx * zw;
//        float yw = (position[1] - cy) / fy * zw;
//        calibedPosition = calibMatrix * vec4(xw, yw, zw, 1.0);
        calibedPosition = calibMatrix * position;
    }
    gl_Position = ciModelViewProjection * calibedPosition;
    vColor = vec4(color[2], color[1], color[0], color[3]) / 255.0;
}
