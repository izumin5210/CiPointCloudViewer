#version 410

in vec4 calibedPosition;
in vec4 vColor;
flat in int userId;

out vec4 fColor;

struct PassThroughFilterParams {
    bool enable;
    float min;
    float max;
};

uniform PassThroughFilterParams xPassThroughParams;
uniform PassThroughFilterParams yPassThroughParams;
uniform PassThroughFilterParams zPassThroughParams;
uniform bool enableUsersThrough;

bool isFiltered(PassThroughFilterParams params, float pos) {
    return params.enable && (params.min > pos || params.max < pos);
}

bool isFiltered(bool enableUsersThrough, int userId) {
    return enableUsersThrough && (userId == 0);
}

void main() {
    if (
        isFiltered(xPassThroughParams, calibedPosition[0])
        || isFiltered(yPassThroughParams, calibedPosition[1])
        || isFiltered(zPassThroughParams, calibedPosition[2])
        || isFiltered(enableUsersThrough, userId)
    ) {
        discard;
    }
    fColor = vColor;
}
