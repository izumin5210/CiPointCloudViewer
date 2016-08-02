#version 410

in vec4 calibedPosition;
in vec4 vColor;
out vec4 fColor;

struct PassThroughFilterParams {
    bool enable;
    float min;
    float max;
};

uniform PassThroughFilterParams xPassThroughParams;
uniform PassThroughFilterParams yPassThroughParams;
uniform PassThroughFilterParams zPassThroughParams;

bool isFiltered(PassThroughFilterParams params, float pos) {
    return params.enable && (params.min > pos || params.max < pos);
}

void main() {
    if (
        isFiltered(xPassThroughParams, calibedPosition[0])
        || isFiltered(yPassThroughParams, calibedPosition[1])
        || isFiltered(zPassThroughParams, calibedPosition[2])
    ) {
        discard;
    }
    fColor = vColor;
}
